//! Where a host fault in the GPU clients' page window lies among the
//! clients' mappings, for its refault and for the line that names it.
//!
//! The window holds more than clients' mappings: the host's stacks between
//! guard pages and Linux's own kernel mappings share it. Only a page inside
//! a client's mapping is Linux's to refault, as Linux's fault handler serves
//! only an address inside a VMA. Any other page is no client's: an access
//! there is the host code's own error, which no refault can answer.
use alloc::collections::BTreeMap;
use core::fmt;

const PAGE: u64 = 4096;

/// A client's mapping of a file in the window.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Span {
    pub start: u64,
    pub pages: u64,
    /// The client's descriptor and the file offset it mapped.
    pub fd: i32,
    pub offset: u64,
}

impl Span {
    pub fn end(&self) -> u64 {
        self.start.saturating_add(self.pages.saturating_mul(PAGE))
    }
}

/// Where a fault lies.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Place {
    /// In a client's mapping, at byte `at` of it: Linux's to refault.
    Inside { span: Span, at: u64 },
    /// In no client's mapping, between the nearest ones.
    Outside {
        below: Option<Span>,
        above: Option<Span>,
    },
}

/// Where `address` lies, given the mapping that starts at or below it and
/// the one that starts above it.
pub fn place(below: Option<Span>, above: Option<Span>, address: u64) -> Place {
    match below {
        Some(span) if address >= span.start && address < span.end() => Place::Inside {
            span,
            at: address - span.start,
        },
        _ => Place::Outside { below, above },
    }
}

/// The line that names a fault Linux did not serve, after `GPU `.
pub struct Unserved {
    pub address: u64,
    pub write: bool,
    pub place: Place,
}

impl fmt::Display for Unserved {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let access = if self.write { "write" } else { "read" };
        match self.place {
            Place::Inside { span, at } => write!(
                f,
                "CLIENT MAPPING REFUSED a {access} at {:#x}: Linux did not back byte {at:#x} of \
                 fd {}'s mapping of {} KiB from file offset {:#x} at {:#x}",
                self.address,
                span.fd,
                (span.pages * PAGE) >> 10,
                span.offset,
                span.start,
            ),
            Place::Outside { below, above } => {
                write!(
                    f,
                    "FAULT OUTSIDE CLIENT MAPPINGS a {access} at {:#x} is in no client's \
                     mapping, so it is the host's own access to a page of the window that \
                     maps nothing",
                    self.address
                )?;
                if let Some(span) = below {
                    write!(
                        f,
                        "; below it fd {}'s mapping {:#x}..{:#x}",
                        span.fd,
                        span.start,
                        span.end()
                    )?;
                }
                if let Some(span) = above {
                    write!(
                        f,
                        "; above it fd {}'s mapping {:#x}..{:#x}",
                        span.fd,
                        span.start,
                        span.end()
                    )?;
                }
                Ok(())
            }
        }
    }
}

/// The clients' mappings, by start, each with the faults running in it.
///
/// Linux serves a fault while its mapping is held (`mmap_lock`), and a fault
/// may sleep there: TTM waits for a buffer's move or eviction when memory is
/// short. Other calls run meanwhile, a new mapping or an unmapping among
/// them, so no borrow of the table lasts across the fault. A mapping a client
/// unmaps while faults run in it leaves the table's lookups at once and is
/// released by the last of them ([`Self::end_fault`]), so Linux never frees
/// a mapping a fault still uses, and its addresses are not given out again
/// before then.
#[derive(Debug)]
pub struct Mappings<M> {
    entries: BTreeMap<u64, Entry<M>>,
}

#[derive(Debug)]
struct Entry<M> {
    map: M,
    span: Span,
    faults: u32,
    unmapped: bool,
}

/// A fault's lookup: where it lies, and the mapping Linux refaults it in,
/// which the fault holds until [`Mappings::end_fault`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Lookup<M> {
    pub place: Place,
    pub held: Option<M>,
}

/// An unmapping of no mapping at that address.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NotMapped;

impl<M> Default for Mappings<M> {
    fn default() -> Self {
        Self {
            entries: BTreeMap::new(),
        }
    }
}

impl<M: Copy> Mappings<M> {
    /// A new mapping, `map`, at `span`.
    pub fn insert(&mut self, span: Span, map: M) {
        self.entries.insert(
            span.start,
            Entry {
                map,
                span,
                faults: 0,
                unmapped: false,
            },
        );
    }

    /// Where a fault at `address` lies among the mappings a client still
    /// holds. Inside one, it is held for the fault until
    /// [`Self::end_fault`] with the place's span start.
    pub fn begin_fault(&mut self, address: u64) -> Lookup<M> {
        let live = |entry: &&Entry<M>| !entry.unmapped;
        let below = self
            .entries
            .range(..=address)
            .map(|(_, e)| e)
            .rev()
            .find(live);
        let above = self
            .entries
            .range(address.saturating_add(1)..)
            .map(|(_, e)| e)
            .find(live);
        let place = place(below.map(|e| e.span), above.map(|e| e.span), address);
        let held = match place {
            Place::Inside { span, .. } => self.entries.get_mut(&span.start).map(|entry| {
                entry.faults += 1;
                entry.map
            }),
            Place::Outside { .. } => None,
        };
        Lookup { place, held }
    }

    /// Ends a fault held in the mapping at `start`: the mapping and its span
    /// when it was unmapped while the fault ran and this was its last, for
    /// the caller to release now.
    pub fn end_fault(&mut self, start: u64) -> Option<(M, Span)> {
        let entry = self.entries.get_mut(&start)?;
        entry.faults = entry.faults.saturating_sub(1);
        if entry.unmapped && entry.faults == 0 {
            return self
                .entries
                .remove(&start)
                .map(|entry| (entry.map, entry.span));
        }
        None
    }

    /// Unmaps the mapping at `start`: it and its span to release now, or
    /// none when faults still run in it, the last of which releases it.
    pub fn unmap(&mut self, start: u64) -> Result<Option<(M, Span)>, NotMapped> {
        let entry = self
            .entries
            .get_mut(&start)
            .filter(|entry| !entry.unmapped)
            .ok_or(NotMapped)?;
        if entry.faults != 0 {
            entry.unmapped = true;
            return Ok(None);
        }
        Ok(self
            .entries
            .remove(&start)
            .map(|entry| (entry.map, entry.span)))
    }

    /// The mappings not yet released, unmapped ones whose faults run among them.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    const BELOW: Span = Span {
        start: 0xffff_8010_0c00_0000,
        pages: 16,
        fd: 7,
        offset: 0x1_0000_0000,
    };
    const ABOVE: Span = Span {
        start: 0xffff_8010_0e00_0000,
        pages: 1,
        fd: 9,
        offset: 0,
    };

    #[test]
    fn a_page_inside_a_mapping_is_the_clients() {
        let address = BELOW.start + 3 * PAGE + 0x10;
        assert_eq!(
            place(Some(BELOW), Some(ABOVE), address),
            Place::Inside {
                span: BELOW,
                at: 3 * PAGE + 0x10
            }
        );
        assert_eq!(
            place(Some(BELOW), None, BELOW.end() - 1),
            Place::Inside {
                span: BELOW,
                at: 16 * PAGE - 1
            }
        );
    }

    #[test]
    fn a_page_past_every_mapping_is_no_clients() {
        // The boot that stopped: a read of the guard page above a host
        // stack, which lies in the window past a client's mapping, was sent
        // to Linux as a client's fault and stopped as one Linux refused.
        let address = 0xffff_8010_0d34_3000;
        assert_eq!(
            place(Some(BELOW), Some(ABOVE), address),
            Place::Outside {
                below: Some(BELOW),
                above: Some(ABOVE)
            }
        );
        assert_eq!(
            place(None, Some(ABOVE), address),
            Place::Outside {
                below: None,
                above: Some(ABOVE)
            }
        );
        assert_eq!(
            place(Some(BELOW), None, BELOW.end()),
            Place::Outside {
                below: Some(BELOW),
                above: None
            }
        );
    }

    #[test]
    fn the_lines_name_the_mapping_or_its_neighbours() {
        let refused = Unserved {
            address: BELOW.start + PAGE,
            write: true,
            place: place(Some(BELOW), None, BELOW.start + PAGE),
        }
        .to_string();
        assert_eq!(
            refused,
            "CLIENT MAPPING REFUSED a write at 0xffff80100c001000: Linux did not back byte \
             0x1000 of fd 7's mapping of 64 KiB from file offset 0x100000000 at \
             0xffff80100c000000"
        );
        let outside = Unserved {
            address: 0xffff_8010_0d34_3000,
            write: false,
            place: place(Some(BELOW), Some(ABOVE), 0xffff_8010_0d34_3000),
        }
        .to_string();
        assert_eq!(
            outside,
            "FAULT OUTSIDE CLIENT MAPPINGS a read at 0xffff80100d343000 is in no client's \
             mapping, so it is the host's own access to a page of the window that maps \
             nothing; below it fd 7's mapping 0xffff80100c000000..0xffff80100c010000; above \
             it fd 9's mapping 0xffff80100e000000..0xffff80100e001000"
        );
    }

    #[test]
    fn a_mapping_unmapped_during_its_fault_is_released_by_the_fault() {
        let mut maps = Mappings::default();
        maps.insert(BELOW, 1u32);
        maps.insert(ABOVE, 2u32);
        let address = BELOW.start + PAGE;
        let lookup = maps.begin_fault(address);
        assert_eq!(lookup.held, Some(1));
        // The fault sleeps; another call maps and the client unmaps.
        let other = Span {
            start: ABOVE.end() + PAGE,
            ..ABOVE
        };
        maps.insert(other, 3);
        assert_eq!(maps.unmap(BELOW.start), Ok(None));
        assert_eq!(maps.unmap(BELOW.start), Err(NotMapped));
        // Its addresses stay taken, but no new fault finds it.
        assert_eq!(maps.len(), 3);
        let late = maps.begin_fault(address);
        assert_eq!(late.held, None);
        assert!(matches!(late.place, Place::Outside { below: None, .. }));
        assert_eq!(maps.end_fault(BELOW.start), Some((1, BELOW)));
        assert_eq!(maps.len(), 2);
        assert_eq!(maps.end_fault(BELOW.start), None);
    }

    #[test]
    fn faults_in_a_mapping_end_before_its_release() {
        let mut maps = Mappings::default();
        maps.insert(BELOW, 1u32);
        assert_eq!(maps.begin_fault(BELOW.start).held, Some(1));
        assert_eq!(maps.begin_fault(BELOW.start + 2 * PAGE).held, Some(1));
        assert_eq!(maps.unmap(BELOW.start), Ok(None));
        assert_eq!(maps.end_fault(BELOW.start), None);
        assert_eq!(maps.end_fault(BELOW.start), Some((1, BELOW)));
        assert!(maps.is_empty());
        // Without a fault running, an unmapping releases at once.
        maps.insert(ABOVE, 2);
        assert_eq!(maps.begin_fault(ABOVE.start).held, Some(2));
        assert_eq!(maps.end_fault(ABOVE.start), None);
        assert_eq!(maps.unmap(ABOVE.start), Ok(Some((2, ABOVE))));
        assert_eq!(maps.begin_fault(ABOVE.start).held, None);
    }
}

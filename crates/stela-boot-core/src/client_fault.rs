//! Where a host fault in the GPU clients' page window lies among the
//! clients' mappings, for its refault and for the line that names it.
//!
//! The window holds more than clients' mappings: the host's stacks between
//! guard pages and Linux's own kernel mappings share it. Only a page inside
//! a client's mapping is Linux's to refault, as Linux's fault handler serves
//! only an address inside a VMA. Any other page is no client's: an access
//! there is the host code's own error, which no refault can answer.
use alloc::collections::BTreeMap;
use alloc::vec::Vec;
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
    /// In a page of a client's mapping that the client already unmapped,
    /// at byte `at` of the mapping: as Linux's split mapping, no longer it.
    Released { span: Span, at: u64 },
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
            Place::Released { span, at } => write!(
                f,
                "FAULT IN UNMAPPED CLIENT PAGES a {access} at {:#x} is byte {at:#x} of fd {}'s \
                 mapping of {} KiB at {:#x}, in pages the client unmapped, so it is no longer \
                 the client's",
                self.address,
                span.fd,
                (span.pages * PAGE) >> 10,
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
///
/// As Linux's `munmap`, a client may unmap any page-aligned run of a mapping:
/// ANV maps a buffer from the kernel's start of it and unmaps the pages
/// before its slab entry. Those pages are no longer the mapping's, and the
/// mapping ends when every page of it is.
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
    /// The page runs the client unmapped, sorted and merged.
    released: Vec<(u64, u64)>,
}

impl<M> Entry<M> {
    fn released(&self, page: u64) -> bool {
        self.released
            .iter()
            .any(|&(first, end)| page >= first && page < end)
    }

    /// Releases the pages of `address..address + bytes` in the mapping:
    /// whether every page of it now is.
    fn release(&mut self, address: u64, bytes: u64) -> bool {
        let first = (address - self.span.start) / PAGE;
        let end = first
            .saturating_add(bytes.div_ceil(PAGE))
            .min(self.span.pages);
        self.released.push((first, end));
        self.released.sort_unstable();
        let mut merged: Vec<(u64, u64)> = Vec::with_capacity(self.released.len());
        for &(first, end) in &self.released {
            match merged.last_mut() {
                Some(previous) if first <= previous.1 => previous.1 = previous.1.max(end),
                _ => merged.push((first, end)),
            }
        }
        self.released = merged;
        self.released == [(0, self.span.pages)]
    }
}

/// A fault's lookup: where it lies, and the mapping Linux refaults it in,
/// which the fault holds until [`Mappings::end_fault`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Lookup<M> {
    pub place: Place,
    pub held: Option<M>,
}

/// An unmapping refused, as Linux's `munmap` refuses it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Refused {
    /// No mapping holds the address.
    NotMapped,
    /// No bytes, or an address not on a page.
    Invalid,
}

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
                released: Vec::new(),
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
        let place = match place(below.map(|e| e.span), above.map(|e| e.span), address) {
            Place::Inside { span, at } if below.is_some_and(|e| e.released(at / PAGE)) => {
                Place::Released { span, at }
            }
            place => place,
        };
        let held = match place {
            Place::Inside { span, .. } => self.entries.get_mut(&span.start).map(|entry| {
                entry.faults += 1;
                entry.map
            }),
            Place::Released { .. } | Place::Outside { .. } => None,
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

    /// Unmaps the `bytes` at `address` of the mapping that holds it: the
    /// mapping and its span to release now when that was its last page, or
    /// none when pages of it remain or faults still run in it, the last of
    /// which releases it.
    pub fn unmap(&mut self, address: u64, bytes: u64) -> Result<Option<(M, Span)>, Refused> {
        if bytes == 0 || !address.is_multiple_of(PAGE) {
            return Err(Refused::Invalid);
        }
        let (&start, entry) = self
            .entries
            .range_mut(..=address)
            .next_back()
            .filter(|(_, entry)| !entry.unmapped && address < entry.span.end())
            .ok_or(Refused::NotMapped)?;
        if !entry.release(address, bytes) {
            return Ok(None);
        }
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
        assert_eq!(maps.unmap(BELOW.start, BELOW.pages * PAGE), Ok(None));
        assert_eq!(
            maps.unmap(BELOW.start, BELOW.pages * PAGE),
            Err(Refused::NotMapped)
        );
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
        assert_eq!(maps.unmap(BELOW.start, u64::MAX), Ok(None));
        assert_eq!(maps.end_fault(BELOW.start), None);
        assert_eq!(maps.end_fault(BELOW.start), Some((1, BELOW)));
        assert!(maps.is_empty());
        // Without a fault running, an unmapping releases at once.
        maps.insert(ABOVE, 2);
        assert_eq!(maps.begin_fault(ABOVE.start).held, Some(2));
        assert_eq!(maps.end_fault(ABOVE.start), None);
        assert_eq!(maps.unmap(ABOVE.start, PAGE), Ok(Some((2, ABOVE))));
        assert_eq!(maps.begin_fault(ABOVE.start).held, None);
    }

    #[test]
    fn a_mapping_goes_once_every_page_of_it_is_released() {
        // ANV's slab entry: the kernel maps a buffer from its start, so the
        // entry at 12 KiB of a 16-page slab is mapped with the three pages
        // before it, which are unmapped at once; the rest goes later.
        let mut maps = Mappings::default();
        maps.insert(BELOW, 1u32);
        assert_eq!(maps.unmap(BELOW.start, 3 * PAGE), Ok(None));
        assert_eq!(maps.begin_fault(BELOW.start + 3 * PAGE).held, Some(1));
        assert_eq!(maps.end_fault(BELOW.start), None);
        // A run inside, then the rest in two parts, out of order.
        assert_eq!(maps.unmap(BELOW.start + 8 * PAGE, 2 * PAGE), Ok(None));
        assert_eq!(maps.unmap(BELOW.start + 10 * PAGE, 100 * PAGE), Ok(None));
        // A page released twice counts once.
        assert_eq!(maps.unmap(BELOW.start + 9 * PAGE, PAGE), Ok(None));
        assert_eq!(
            maps.unmap(BELOW.start + 3 * PAGE, 5 * PAGE),
            Ok(Some((1, BELOW)))
        );
        assert!(maps.is_empty());
    }

    #[test]
    fn an_empty_unaligned_or_unmapped_unmap_is_refused_and_releases_nothing() {
        let mut maps = Mappings::default();
        maps.insert(BELOW, 1u32);
        assert_eq!(maps.unmap(BELOW.start, 0), Err(Refused::Invalid));
        assert_eq!(maps.unmap(BELOW.start + 1, PAGE), Err(Refused::Invalid));
        assert_eq!(maps.unmap(BELOW.end(), PAGE), Err(Refused::NotMapped));
        assert_eq!(
            maps.unmap(BELOW.start - PAGE, PAGE),
            Err(Refused::NotMapped)
        );
        assert_eq!(
            maps.unmap(BELOW.start, BELOW.pages * PAGE),
            Ok(Some((1, BELOW)))
        );
    }

    #[test]
    fn a_page_unmapped_during_a_fault_leaves_the_mapping_held_by_it() {
        let mut maps = Mappings::default();
        maps.insert(BELOW, 1u32);
        let lookup = maps.begin_fault(BELOW.start + 5 * PAGE);
        assert_eq!(lookup.held, Some(1));
        // While the fault sleeps the client unmaps the first pages, then all
        // but the faulting one's: the fault's page stays the mapping's.
        assert_eq!(maps.unmap(BELOW.start, 5 * PAGE), Ok(None));
        assert_eq!(maps.unmap(BELOW.start + 6 * PAGE, 10 * PAGE), Ok(None));
        // A fault in a released page is not the mapping's.
        let late = maps.begin_fault(BELOW.start + PAGE);
        assert_eq!(late.held, None);
        assert_eq!(
            late.place,
            Place::Released {
                span: BELOW,
                at: PAGE
            }
        );
        // The last page goes while the fault runs: the fault releases it.
        assert_eq!(maps.unmap(BELOW.start + 5 * PAGE, PAGE), Ok(None));
        assert_eq!(maps.len(), 1);
        assert_eq!(maps.begin_fault(BELOW.start + 5 * PAGE).held, None);
        assert_eq!(maps.end_fault(BELOW.start), Some((1, BELOW)));
        assert!(maps.is_empty());
    }

    #[test]
    fn a_fault_in_an_unmapped_page_is_named_so() {
        let line = Unserved {
            address: BELOW.start + PAGE,
            write: false,
            place: Place::Released {
                span: BELOW,
                at: PAGE,
            },
        }
        .to_string();
        assert_eq!(
            line,
            "FAULT IN UNMAPPED CLIENT PAGES a read at 0xffff80100c001000 is byte 0x1000 of fd \
             7's mapping of 64 KiB at 0xffff80100c000000, in pages the client unmapped, so it \
             is no longer the client's"
        );
    }
}

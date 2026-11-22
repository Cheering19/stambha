//! HLE's engine glue ([`crate::hle`]): a machine's configuration and
//! counts, the match of a function at its entry as the engine compiles it,
//! before any stored, shared or rebased block is looked for
//! ([`Machine::hle_compile`], called from `compile_window`), and the
//! install of a composite block in the translated block's place.
//!
//! Only an EL0 compile of an ordinary variant matches: not single-step,
//! translated, DIT-timed or streaming, with FP enabled and no alignment
//! checks. A match whose function crosses its fetch page, starts at a
//! device site, holds a host breakpoint or trace point, or holds a word the
//! chip runs as another (`Machine::admit`) is refused, and so is one whose
//! first instruction does not compile alone or whose composite cannot be
//! laid out: each is translated as before.
//!
//! A composite (`stela_compiler::x86_64::hle::compose`) is the routine's
//! fast path, then the function's first instruction compiled alone, which
//! the fast path bails to. It is installed as an ordinary block of one
//! instruction at the entry's PC and fetch page, so an IC IVAU of any byte
//! of the function retires it, and it is never stored, offered or rebased.
//! Its block holds a [`Mark`]: where its fast path ends.
use super::*;
use crate::hle::{Config, Id, PREFIX_BYTES, Refusal, Signature, Statistics};
use core::num::NonZeroU8;
use stela_compiler::x86_64::hle::{self as composite, Request};

/// A machine's HLE: its configuration, what it did, the bytes of a
/// candidate function longer than the compile's window, and the mark of
/// the composite it is installing, for its block ([`Self::installing`]).
#[derive(Default)]
pub(super) struct State {
    config: Config,
    statistics: Statistics,
    scratch: Vec<u8>,
    /// While a composite installs: the mark its block takes, as
    /// `installing_stub` marks a device stub's.
    pub(super) installing: Option<Mark>,
}

/// What a composite block holds of its composite: the length of its fast
/// path, in [`composite::PREFIX_ALIGN`] units, which a fault's offset is
/// compared with. One byte, with a niche for `None`, so a [`Block`] stays
/// within its bound.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Mark {
    prefix_units: NonZeroU8,
}

impl Mark {
    fn new(prefix_end: u32) -> Self {
        let units = prefix_end / composite::PREFIX_ALIGN;
        assert!(
            prefix_end.is_multiple_of(composite::PREFIX_ALIGN),
            "a fast path ends at a multiple of its alignment"
        );
        Self {
            prefix_units: u8::try_from(units)
                .ok()
                .and_then(NonZeroU8::new)
                .expect("a fast path of 1 to 255 units"),
        }
    }

    /// Where the fast path ends and the original's first instruction
    /// starts, in the block's code.
    #[expect(dead_code, reason = "the fault bypass reads it next")]
    pub(super) fn prefix_end(self) -> usize {
        usize::from(self.prefix_units.get()) * composite::PREFIX_ALIGN as usize
    }
}

const _: () = assert!(size_of::<Option<Mark>>() == 1);
const _: () = assert!(composite::MAX_PREFIX_BYTES / composite::PREFIX_ALIGN <= u8::MAX as u32);

/// A function [`Machine::hle_compile`] accepted: the routine that replaces
/// it and the offset of its exit `ret`.
struct Site {
    routine: Id,
    exit: u64,
}

/// Whether a block of `variant`, compiled at `el`, may be replaced.
fn replaceable(variant: Variant, el: u8) -> bool {
    el == 0
        && !variant.single
        && !variant.translated
        && !variant.timing
        && variant.scalable != Some(true)
        && variant.fp
        && !variant.checks.alignment
}

impl<H: Host, D: Devices> Machine<H, D> {
    /// Match the functions `config` enables in the blocks compiled from now
    /// on. A boot sets it once, before its PEs run.
    pub fn set_hle(&mut self, config: Config) {
        self.hle.config = config;
    }

    pub fn hle_statistics(&self) -> Statistics {
        self.hle.statistics
    }

    /// At the compile of the block at `pc` (`physical`) from the raw bytes
    /// `source`, its window's first: whether an enabled signature's
    /// function starts there and may be replaced, and its composite
    /// installed if so. `Ok(None)` leaves the compile to translate it.
    ///
    /// A compile reads a first window ([`FIRST_WINDOW_BYTES`]) and, when its
    /// block may run past it, the whole one: the second read looks no more,
    /// so a function is matched (and refused) once per compile.
    pub(super) fn hle_compile(
        &mut self,
        pc: u64,
        physical: u64,
        variant: Variant,
        source: &[u8],
    ) -> Result<Option<Fetched>, Stop> {
        if self.hle.config.enabled == 0 || source.len() > FIRST_WINDOW_BYTES {
            return Ok(None);
        }
        let Some(head) = source.first_chunk::<PREFIX_BYTES>() else {
            return Ok(None);
        };
        if !replaceable(variant, self.cpu.pstate.el) {
            return Ok(None);
        }
        let offset = pc & (FETCH_PAGE - 1);
        let table = self.hle.config.table.clone();
        for (index, signature) in crate::hle::find(&table, self.hle.config.enabled, head) {
            let length = signature.length() as u64;
            if offset + length > FETCH_PAGE {
                self.hle.statistics.refuse(Refusal::CrossesPage);
                continue;
            }
            if !self.hle_matches(physical, source, signature) {
                continue;
            }
            self.hle.statistics.matched += 1;
            self.hle.statistics.matched_signatures |= 1 << index;
            let refusal = if self.device_site(physical) {
                Some(Refusal::DeviceSite)
            } else if self
                .breakpoints
                .iter()
                .chain(&self.traces)
                .any(|stop| (pc..pc + length).contains(stop))
            {
                Some(Refusal::Breakpoint)
            } else if !self.hle_admitted() {
                Some(Refusal::Admitted)
            } else {
                None
            };
            if let Some(refusal) = refusal {
                self.hle.statistics.refuse(refusal);
                return Ok(None);
            }
            let site = Site {
                routine: signature.id(),
                exit: signature.exit(),
            };
            return self.install_hle(pc, physical, variant, source, site);
        }
        Ok(None)
    }

    /// Whether `signature`'s function starts at `physical`: from `source`
    /// when it holds the whole function, else from RAM. The function's
    /// bytes are left in the scratch buffer either way.
    fn hle_matches(&mut self, physical: u64, source: &[u8], signature: &Signature) -> bool {
        let mut function = core::mem::take(&mut self.hle.scratch);
        function.clear();
        let read = match source.get(..signature.length()) {
            Some(bytes) => {
                function.extend_from_slice(bytes);
                true
            }
            None => {
                function.resize(signature.length(), 0);
                self.host.read_ram(physical, &mut function)
            }
        };
        let matches = read && signature.matches(&function);
        self.hle.scratch = function;
        matches
    }

    /// Whether the chip runs every word of the function in the scratch
    /// buffer as itself: one it lacks is UDF or a NOP there
    /// (`Chip::admit`), where the bytes the signature checked would trap or
    /// run.
    fn hle_admitted(&self) -> bool {
        let (words, _) = self.hle.scratch.as_chunks::<4>();
        words.iter().all(|word| {
            let word = u32::from_le_bytes(*word);
            self.admit(word) == word
        })
    }

    /// Install the composite of `site`'s routine at the function entry
    /// `pc` (`physical`), as [`Self::install_device_stub`] installs a stub:
    /// the entry's first word, admitted, compiled alone in `variant` as the
    /// bail, the composite laid out in front of it, and the block of that
    /// one instruction installed with its [`Mark`]. `source` is the
    /// compile's raw window, which `compile_window` admits only after this.
    /// `Ok(None)` when the first word does not compile alone (counted as
    /// [`Refusal::Bail`]) or the composite cannot be laid out
    /// ([`Refusal::Compose`]).
    fn install_hle(
        &mut self,
        pc: u64,
        physical: u64,
        variant: Variant,
        source: &[u8],
        site: Site,
    ) -> Result<Option<Fetched>, Stop> {
        let preparation = self.preparation(variant);
        let mut first: [u8; INSTRUCTION as usize] = source[..INSTRUCTION as usize]
            .try_into()
            .expect("a window holds its first word");
        preparation.admit(&mut first);
        let bail = match preparation.prepare(&first, pc, first.len()) {
            Ok(Prepared::Block {
                compiled, input: 4, ..
            }) => compiled,
            _ => {
                self.hle.statistics.refuse(Refusal::Bail);
                return Ok(None);
            }
        };
        let request = Request {
            routine: site.routine,
            pc: GuestAddress(pc),
            exit: GuestAddress(pc + site.exit),
            max_bytes: self.hle.config.max_bytes,
        };
        let Ok(composite) = composite::compose(request, bail) else {
            self.hle.statistics.refuse(Refusal::Compose);
            return Ok(None);
        };
        let mark = Mark::new(composite.prefix_end);
        let emission = Arc::new(super::stored::Emission::new(
            first.to_vec(),
            composite.block,
            preparation,
        ));
        self.hle.installing = Some(mark);
        let installed = self.install(
            pc,
            physical,
            variant,
            &first,
            recovery::Template::compiled(emission),
        );
        self.hle.installing = None;
        let fetched = installed?;
        self.hle.statistics.installed += 1;
        Ok(Some(fetched))
    }
}

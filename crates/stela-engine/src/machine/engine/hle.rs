//! HLE's engine glue ([`crate::hle`]): a machine's configuration and
//! counts, and the match of a function at its entry as the engine compiles
//! it, before any stored, shared or rebased block is looked for
//! ([`Machine::hle_compile`], called from `compile_window`).
//!
//! Only an EL0 compile of an ordinary variant matches: not single-step,
//! translated, DIT-timed or streaming, with FP enabled and no alignment
//! checks. A match whose function crosses its fetch page, starts at a
//! device site, holds a host breakpoint or trace point, or holds a word the
//! chip runs as another (`Machine::admit`) is refused. A match is
//! translated as before: nothing is installed in its place yet.
use super::*;
use crate::hle::{Config, PREFIX_BYTES, Refusal, Signature, Statistics};

/// A machine's HLE: its configuration, what it did, and the bytes of a
/// candidate function longer than the compile's window.
#[derive(Default)]
pub(super) struct State {
    config: Config,
    statistics: Statistics,
    scratch: Vec<u8>,
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
    /// function starts there, and may be replaced. It never is yet, so this
    /// is always `Ok(None)`.
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
            }
            return Ok(None);
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
}

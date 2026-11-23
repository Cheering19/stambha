//! `--tier2-check`: sampled tier-2 entries compared against tier 1.
//!
//! One entry in [`CHECK_EVERY`] into a tier-2 block whose words only read
//! memory ([`reads_only`]) runs that block alone (no transfer out of it),
//! and then, from the same start, the same number of instructions again as
//! single-instruction blocks: tier 1's lowest form, which breakpoints and
//! software step run. No interrupt or device step comes between them. The
//! two ends are compared: registers, NZCV, vectors and FP status, the PC,
//! the instructions retired and the BTYPE.
//!
//! The block writes no memory, so running it twice writes nothing twice;
//! its loads may read what another PE wrote between the two runs, which can
//! only make a mismatch where none is (the check then fails safe). A run
//! that leaves through an exception or a fault stands, unchecked. On a
//! mismatch tier 1's end stands (it is the state tier 1 reaches from the
//! start), the block's page is invalidated as an IC IVAU of it would be,
//! and the block is promoted no more ([`super::tier2`]); the count and the
//! entry PC go to `PES TIER2`.
use super::Machine;
use crate::machine::{Devices, Host};
use stela_compiler::GuestState;

/// Entries into tier-2 blocks between two checks, by default.
pub const CHECK_EVERY: u32 = 64;
/// The instructions a check replays at most: a block's.
const CHECK_INSTRUCTIONS: u64 = stela_compiler::block::MAX_BLOCK_INSTRUCTIONS as u64;

/// Whether `word` only reads memory, if it touches memory at all: data
/// processing, FP and SIMD data processing, direct and register branches,
/// NOP and plain loads. Stores, atomics, exclusives, acquires, structure
/// loads, tag accesses and every other System instruction do not.
pub(super) fn reads_only(word: u32) -> bool {
    match (word >> 25) & 0xf {
        // Data processing (immediate).
        0b1000 | 0b1001 => true,
        // Data processing (register).
        0b0101 | 0b1101 => true,
        // SIMD and FP data processing.
        0b0111 | 0b1111 => true,
        // Branches, exception generation and System.
        0b1010 | 0b1011 => {
            word & 0x7c00_0000 == 0x1400_0000 // B, BL
                || word & 0xff00_0010 == 0x5400_0000 // B.cond
                || word & 0x7e00_0000 == 0x3400_0000 // CBZ, CBNZ
                || word & 0x7e00_0000 == 0x3600_0000 // TBZ, TBNZ
                || word & 0xff9f_fc1f == 0xd61f_0000 // BR, BLR, RET
                || word == 0xd503_201f // NOP
        }
        // Loads and stores.
        _ => match (word >> 27) & 7 {
            // Single registers: the unsigned offset forms, and the register
            // offset and 9-bit forms. Atomics and PAC loads share the space
            // with bit 21 set and bits 11:10 other than 0b10.
            0b111 => {
                let unsigned = (word >> 24) & 1 == 1;
                let other = !unsigned && (word >> 21) & 1 == 1 && (word >> 10) & 3 != 0b10;
                let opc = (word >> 22) & 3;
                let simd = (word >> 26) & 1 == 1;
                !other && if simd { opc & 1 == 1 } else { opc != 0 }
            }
            // Literal loads (and PRFM literal).
            0b011 => (word >> 24) & 0x3b == 0x18,
            // Pairs: LDP, LDNP, LDPSW.
            0b101 => (word >> 22) & 1 == 1,
            _ => false,
        },
    }
}

/// A state to compare: registers, NZCV, vectors and FP status, the PC, the
/// instructions retired and the BTYPE.
type Compared = (GuestState, u64, u64, u8);

impl<H: Host, D: Devices> Machine<H, D> {
    fn compared(&self) -> Compared {
        (
            self.cpu.state.clone(),
            self.cpu.pc,
            self.cpu.retired,
            self.cpu.pstate.btype,
        )
    }

    /// Whether the run about to start at block `index` is checked: a tier-2
    /// block that only reads memory, at one of [`CHECK_EVERY`] entries.
    pub(super) fn tier2_check_due(&mut self, index: usize, single: bool) -> bool {
        if single || self.tier2_checking {
            return false;
        }
        let tier2_block = self.code.blocks.get(index).is_some_and(|block| block.tier2);
        let Some(tier2) = self.tier2.as_deref_mut() else {
            return false;
        };
        if !tier2.check || !tier2_block {
            return false;
        }
        tier2.since_check += 1;
        if tier2.since_check < tier2.check_every {
            return false;
        }
        tier2.since_check = 0;
        let block = &self.code.blocks[index];
        let (physical, hash) = (block.physical, block.source_hash);
        let mut words = alloc::vec![0; block.source_bytes()];
        // Its words as installed: else the guest rewrote them, and its next
        // IC IVAU takes the block away.
        let pure = self.host.read_ram(physical, &mut words)
            && super::source_hash(&words) == hash
            && words
                .as_chunks::<4>()
                .0
                .iter()
                .all(|word| reads_only(u32::from_le_bytes(*word)));
        if !pure {
            self.statistics.tier2.check_skipped += 1;
        }
        pure
    }

    /// Run tier-2 block `index` alone, then the same instructions from the
    /// same start as single-instruction blocks, and compare their ends.
    pub(super) fn tier2_checked(&mut self, index: usize) -> super::Step {
        let start = self.compared();
        let (el, exceptions) = (self.cpu.pstate.el, self.statistics.exceptions);
        self.tier2_checking = true;
        let step = self.execute(index, 0);
        let clean = step.is_ok()
            && self.native_failure.is_none()
            && self.statistics.exceptions == exceptions
            && self.cpu.pstate.el == el;
        let ran = self.cpu.retired.wrapping_sub(start.2);
        if !clean || ran == 0 || ran > CHECK_INSTRUCTIONS {
            self.tier2_checking = false;
            self.statistics.tier2.check_skipped += 1;
            return step;
        }
        let tier2 = self.compared();
        // Tier 1 from the same start, one instruction at a time.
        (
            self.cpu.state,
            self.cpu.pc,
            self.cpu.retired,
            self.cpu.pstate.btype,
        ) = start.clone();
        let mut replayed = true;
        let mut result = Ok(());
        while self.cpu.retired.wrapping_sub(start.2) < ran {
            let single = match self.fetch_block(self.cpu.pc, true) {
                Ok(super::Fetched::Block(single)) => single,
                _ => {
                    replayed = false;
                    break;
                }
            };
            result = self.execute(single, 0);
            if result.is_err()
                || self.native_failure.is_some()
                || self.statistics.exceptions != exceptions
            {
                replayed = false;
                break;
            }
        }
        self.tier2_checking = false;
        // Tier 1 ended elsewhere: where tier 2 retired no exception, it took
        // one, or failed; what it reached stands.
        if !replayed || self.compared() != tier2 {
            self.statistics.tier2.check_mismatches += 1;
            self.statistics.tier2.mismatch_pc = start.1;
            self.tier2_deopt(index);
        } else {
            self.statistics.tier2.checks += 1;
        }
        // A stop the replay met is tier 1's, and the machine's.
        result
    }
}

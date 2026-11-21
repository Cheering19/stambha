//! `--profile`'s host samples of native code named by guest block
//! ([`step_timing::pc_samples`], the `PES PC` lines): after each native run
//! the PE takes the samples the host took in it ([`Host::profile_blocks`])
//! and names each block while it is still in its slot (a block another PE
//! retired stays there until this PE's grace): its PC, its level and its
//! code page's key.
//!
//! A native run can change level itself (SVC, ERET, GENTER and GEXIT
//! complete natively, `transitions`), so the PE's state after it does not
//! say where a sample was. A sample's level is its block's half instead:
//! EL0 code runs in the lower half and the kernel and the guarded levels in
//! the upper, which the PE's state after the run then tells apart.
//!
//! The page key is the first 8 bytes of the SHA-256 of the block's 4 KiB
//! fetch page, read at once: the key the board's PEs knew for those bytes
//! (`known_keys`), else one the PE named for them before, else their hash.
use super::*;
use crate::translations::{Digest, PAGE_BYTES};
use step_timing::{BlockSample, Level, NO_BLOCK, PcHit, PcSamples};

/// Pages a PE keeps the keys it named for, by frame.
const PAGE_KEYS: usize = 256;
/// Samples taken from the host at a time.
const TAKEN: usize = 8;
/// Named samples a PE keeps one by one; more wait counted by block, for a
/// PE whose samples are taken only at the beat (a serialized board's).
const HITS: usize = 4096;

/// One PE's samples not yet taken ([`Machine::take_pc_samples`]), and the
/// keys it named for pages.
#[derive(Default)]
pub(super) struct PcSampling {
    hits: Vec<PcHit>,
    /// Samples past [`HITS`], by block.
    counted: PcSamples,
    /// Samples, and their cycles, that named no block.
    gone: [u64; 2],
    /// Frame, check and key of the pages named, by frame; a frame of
    /// `u64::MAX` holds none.
    keys: Vec<(u64, u64, u64)>,
    page: Option<Box<[u8; PAGE_BYTES]>>,
}

/// A block's sampled PC and level, from its `pc`: EL0's half or the
/// kernel's ([`Level::Guarded`] when `guarded`), and a kernel or guarded PC
/// less the vector base `vbar`, as `PES HOT` keeps them.
pub(super) fn sampled_pc(pc: u64, guarded: bool, vbar: u64) -> (u64, Level) {
    match (pc >> 63 != 0, guarded) {
        (false, _) => (pc, Level::User),
        (true, false) => (pc.wrapping_sub(vbar), Level::Kernel),
        (true, true) => (pc.wrapping_sub(vbar), Level::Guarded),
    }
}

/// A page key's first 8 bytes, in their order.
pub(super) fn key_prefix(key: &Digest) -> u64 {
    let mut first = [0; 8];
    first.copy_from_slice(&key.0[..8]);
    u64::from_be_bytes(first)
}

impl<H: Host, D: Devices> Machine<H, D> {
    /// Under step timing, after a native run: name the blocks the host's
    /// samples of the run found running.
    pub(super) fn take_native_samples(&mut self) {
        let mut taken = [BlockSample::default(); TAKEN];
        loop {
            let count = self.host.profile_blocks(&mut taken).min(TAKEN);
            for &sample in &taken[..count] {
                self.name_sample(sample);
            }
            if count < TAKEN {
                return;
            }
        }
    }

    /// Name the block `sample` found running, or count it gone.
    fn name_sample(&mut self, sample: BlockSample) {
        let index = (sample.block & u64::from(u32::MAX)) as usize;
        let block = (sample.block != NO_BLOCK)
            .then(|| self.code.blocks.get(index))
            .flatten()
            .map(|block| (block.pc, block.physical));
        let Some((pc, physical)) = block else {
            self.pc_sampling.gone[0] += 1;
            self.pc_sampling.gone[1] += sample.cycles;
            return;
        };
        // The kernel's vector base at its EL: EL2's under VHE.
        let vbar = if pc >> 63 == 0 {
            0
        } else if self.cpu.pstate.el == 2 || self.cpu.pstate.el == 0 && self.cpu.in_host() {
            self.cpu.read(SystemRegister::VBAR_EL2) & !0x7ff
        } else {
            self.cpu.read(SystemRegister::VBAR_EL1) & !0x7ff
        };
        let (sampled, level) = sampled_pc(pc, self.cpu.pstate.guarded, vbar);
        let page = self.sampled_page_key(physical);
        let sampling = &mut self.pc_sampling;
        sampling.hits.push(PcHit {
            pc: sampled,
            level,
            page,
            offset: (pc & (FETCH_PAGE - 1)) as u16,
            cycles: sample.cycles,
        });
        if sampling.hits.len() >= HITS {
            for hit in sampling.hits.drain(..) {
                sampling.counted.add(hit);
            }
        }
    }

    /// The key prefix of the fetch page holding `physical`, whose bytes
    /// are read now; `None` where they are not RAM.
    fn sampled_page_key(&mut self, physical: u64) -> Option<u64> {
        let frame = physical & !(FETCH_PAGE - 1);
        let sampling = &mut self.pc_sampling;
        let page = sampling
            .page
            .get_or_insert_with(|| Box::new([0; PAGE_BYTES]));
        if !self.host.read_ram(frame, &mut page[..]) {
            return None;
        }
        let check = stored::page_check(page);
        if sampling.keys.is_empty() {
            sampling.keys.resize(PAGE_KEYS, (u64::MAX, 0, 0));
        }
        let slot = (frame >> 12) as usize % PAGE_KEYS;
        let (held, held_check, key) = sampling.keys[slot];
        if held == frame && held_check == check {
            return Some(key);
        }
        let key = match self.code.known_keys.peek(frame, check) {
            Some(known) => known.key,
            None => Digest::of(page.as_slice()),
        };
        let key = key_prefix(&key);
        sampling.keys[slot] = (frame, check, key);
        Some(key)
    }

    /// Whether named samples wait to be taken ([`Self::take_pc_samples`]).
    pub fn has_pc_samples(&self) -> bool {
        let sampling = &self.pc_sampling;
        !sampling.hits.is_empty() || !sampling.counted.is_empty() || sampling.gone[0] != 0
    }

    /// Add the named samples since the last call to `samples`.
    pub fn take_pc_samples(&mut self, samples: &mut PcSamples) {
        let sampling = &mut self.pc_sampling;
        if !sampling.counted.is_empty() {
            samples.merge(&core::mem::take(&mut sampling.counted));
        }
        for hit in sampling.hits.drain(..) {
            samples.add(hit);
        }
        let [count, cycles] = core::mem::take(&mut sampling.gone);
        samples.add_gone(count, cycles);
    }
}

#[cfg(test)]
mod tests {
    use super::super::tests::{FakeHost, machine};
    use super::*;
    use crate::machine::NoDevices;

    #[test]
    fn sampled_pcs_take_their_half_and_the_kernels_vector_base() {
        assert_eq!(
            sampled_pc(0x1_8a2b_3c40, false, 0x800),
            (0x1_8a2b_3c40, Level::User)
        );
        // The PE's guarded state does not move a lower-half block.
        assert_eq!(sampled_pc(0x1000, true, 0x800), (0x1000, Level::User));
        let kernel = 0xffff_fe00_0712_3c40;
        let vbar = 0xffff_fe00_0700_0000;
        assert_eq!(sampled_pc(kernel, false, vbar), (0x12_3c40, Level::Kernel));
        assert_eq!(sampled_pc(kernel, true, vbar), (0x12_3c40, Level::Guarded));
        // Below the vector base, wrapping, as `PES HOT` prints `v-`.
        assert_eq!(
            sampled_pc(vbar - 0x40, false, vbar),
            (0u64.wrapping_sub(0x40), Level::Kernel)
        );
    }

    #[test]
    fn a_key_prefix_is_the_digests_first_bytes_in_order() {
        let mut key = Digest([0; 32]);
        key.0[..8].copy_from_slice(&[0x1f, 0x2e, 0x3d, 0x4c, 0x5b, 0x6a, 0x79, 0x80]);
        key.0[8] = 0xff;
        assert_eq!(key_prefix(&key), 0x1f2e_3d4c_5b6a_7980);
        assert_eq!(&format!("{:016x}", key_prefix(&key)), &key.hex()[..16]);
    }

    /// Two compiled blocks; each step runs one, the host's samples queued
    /// for the run.
    fn compiled() -> (Machine<FakeHost, NoDevices>, u64) {
        const ADD: u32 = 0x9100_0400; // add x0, x0, #1
        let mut machine = machine(&[ADD, 0x1400_0002, 0, ADD, 0x17ff_fffc]);
        let base = machine.cpu.pc;
        machine.slice_end = u64::MAX;
        machine.block_limit = u64::MAX;
        machine.host.exits = alloc::vec![direct(base + 12), direct(base)];
        machine.step().unwrap();
        machine.step().unwrap();
        (machine, base)
    }

    fn direct(next_pc: u64) -> NativeExit {
        NativeExit::Exit {
            next_pc,
            kind: stela_compiler::block::ExitKind::Direct as u64,
        }
    }

    /// The index of the block at `pc`.
    fn index_of(machine: &Machine<FakeHost, NoDevices>, pc: u64) -> u64 {
        (0..64)
            .find(|&index| {
                machine
                    .code
                    .blocks
                    .get(index)
                    .is_some_and(|block| block.pc == pc)
            })
            .expect("a block at the PC") as u64
    }

    #[test]
    fn samples_name_their_blocks_pages_only_under_step_timing() {
        let (mut machine, base) = compiled();
        let index = index_of(&machine, base);
        let queued = alloc::vec![
            BlockSample {
                block: index | 1 << 32,
                cycles: 1 << 24
            },
            BlockSample {
                block: NO_BLOCK,
                cycles: 7
            },
            BlockSample {
                block: 60_000,
                cycles: 3
            },
        ];
        // Without step timing the host is not asked and nothing is named.
        machine.host.block_samples = queued.clone();
        machine.host.exits = alloc::vec![direct(machine.cpu.pc ^ base ^ (base + 12))];
        machine.step().unwrap();
        assert_eq!(machine.host.block_samples, queued);
        assert!(!machine.has_pc_samples());
        machine.set_step_timing(true);
        machine.host.exits = alloc::vec![direct(machine.cpu.pc ^ base ^ (base + 12))];
        machine.step().unwrap();
        assert!(machine.host.block_samples.is_empty(), "all taken");
        assert!(machine.has_pc_samples());
        let mut samples = PcSamples::default();
        machine.take_pc_samples(&mut samples);
        assert!(!machine.has_pc_samples());
        let frame = machine.code.blocks.get(index as usize).unwrap().physical & !0xfff;
        let mut page = [0u8; PAGE_BYTES];
        assert!(machine.host.read_ram(frame, &mut page));
        let key = key_prefix(&Digest::of(&page));
        let top = samples.top(Level::User, 4);
        assert_eq!(
            top,
            [step_timing::PcBlock {
                pc: base,
                level: Level::User,
                page: Some(key),
                offset: (base & 0xfff) as u16,
                samples: 1,
                cycles: 1 << 24,
            }]
        );
        assert_eq!(samples.gone, [2, 10], "no block, and an empty slot");
        // The page's key is kept: the next sample of it reads the page and
        // its check alone, and names the same key.
        machine.host.block_samples = alloc::vec![BlockSample {
            block: index,
            cycles: 5
        }];
        machine.host.exits = alloc::vec![direct(machine.cpu.pc ^ base ^ (base + 12))];
        machine.step().unwrap();
        machine.take_pc_samples(&mut samples);
        assert_eq!(samples.top(Level::User, 4)[0].samples, 2);
        assert_eq!(samples.top(Level::User, 4)[0].cycles, (1 << 24) + 5);
    }

    /// The samples the host gives for `index` at once, named and taken.
    fn named(machine: &mut Machine<FakeHost, NoDevices>, index: u64) -> PcSamples {
        machine.host.block_samples = alloc::vec![BlockSample {
            block: index,
            cycles: 1
        }];
        machine.take_native_samples();
        let mut samples = PcSamples::default();
        machine.take_pc_samples(&mut samples);
        samples
    }

    /// Change the block in `index`'s slot as `change` does.
    fn rewrite(
        machine: &mut Machine<FakeHost, NoDevices>,
        index: u64,
        change: impl FnOnce(&mut Block),
    ) {
        let slot = index as usize;
        machine.code.blocks.retire(slot);
        let mut block = machine.code.blocks.vacate(slot).expect("a block");
        change(&mut block);
        machine.code.blocks.fill(slot, block);
    }

    /// A block another PE retired stays in its slot until this PE's grace:
    /// a sample of the run that was in it still names it.
    #[test]
    fn a_retired_block_still_in_its_slot_is_named() {
        let (mut machine, base) = compiled();
        let index = index_of(&machine, base + 12);
        machine.code.blocks.retire(index as usize);
        assert!(machine.code.blocks.installed(index as usize).is_none());
        let samples = named(&mut machine, index);
        assert_eq!(samples.gone, [0, 0]);
        assert_eq!(samples.top(Level::User, 4)[0].pc, base + 12);
    }

    /// A kernel block's PC is kept less its EL's vector base, EL2's at EL2
    /// and under VHE at EL0; the PE's guarded state makes it a guarded one;
    /// its offset is that of its own PC and its page is still read.
    #[test]
    fn kernel_and_guarded_samples_are_kept_from_their_vector_base() {
        use crate::machine::system::Register;
        const KERNEL: u64 = 0xffff_fe00_0712_3c40;
        const VBAR_EL1: u64 = 0xffff_fe00_0700_0000;
        const VBAR_EL2: u64 = 0xffff_fe00_0710_0000;
        let (mut machine, base) = compiled();
        let index = index_of(&machine, base + 12);
        rewrite(&mut machine, index, |block| block.pc = KERNEL);
        // Their low bits are no part of the base.
        machine
            .cpu
            .registers
            .set(Register::VBAR_EL1, 0, VBAR_EL1 | 0x7ff);
        machine.cpu.registers.set(Register::VBAR_EL2, 0, VBAR_EL2);
        let kept = |machine: &mut Machine<FakeHost, NoDevices>, el, guarded| {
            machine.cpu.pstate.el = el;
            machine.cpu.pstate.guarded = guarded;
            let samples = named(machine, index);
            let level = if guarded {
                Level::Guarded
            } else {
                Level::Kernel
            };
            let top = samples.top(level, 4);
            assert_eq!(top.len(), 1, "EL{el} guarded={guarded}");
            assert_eq!(top[0].offset, 0xc40);
            assert!(top[0].page.is_some());
            top[0].pc
        };
        assert_eq!(kept(&mut machine, 1, false), KERNEL - VBAR_EL1);
        assert_eq!(kept(&mut machine, 1, true), KERNEL - VBAR_EL1);
        assert_eq!(kept(&mut machine, 2, false), KERNEL - VBAR_EL2);
        const HCR_TGE: u64 = 1 << 27;
        const HCR_E2H: u64 = 1 << 34;
        machine
            .cpu
            .registers
            .set(Register::HCR_EL2, 0, HCR_TGE | HCR_E2H);
        assert!(machine.cpu.in_host());
        assert_eq!(kept(&mut machine, 0, false), KERNEL - VBAR_EL2);
    }

    /// A block whose page is not RAM is named with no page (`?`).
    #[test]
    fn a_page_that_cannot_be_read_names_no_key() {
        let (mut machine, base) = compiled();
        let index = index_of(&machine, base + 12);
        rewrite(&mut machine, index, |block| {
            block.physical = 0xdead_0000_0000 | (base + 12) & 0xfff
        });
        let samples = named(&mut machine, index);
        let top = samples.top(Level::User, 4);
        assert_eq!(
            (top[0].pc, top[0].page, top[0].offset),
            (base + 12, None, ((base + 12) & 0xfff) as u16)
        );
        assert!(samples.lines("PES PC", 4)[1].contains("@?+"));
    }

    /// A page whose key the board's PEs know for its bytes takes that key
    /// (`known_keys`) and is not hashed again.
    #[test]
    fn a_page_the_board_knows_takes_its_known_key() {
        let (mut machine, base) = compiled();
        let index = index_of(&machine, base);
        let frame = machine.code.blocks.get(index as usize).unwrap().physical & !0xfff;
        let mut page = [0u8; PAGE_BYTES];
        assert!(machine.host.read_ram(frame, &mut page));
        let known = super::super::known_keys::Known {
            key: Digest([7; 32]),
            absent: false,
        };
        let slots = machine.known_slots;
        machine
            .code
            .known_keys
            .put(slots, frame, stored::page_check(&page), known);
        let samples = named(&mut machine, index);
        assert_eq!(
            samples.top(Level::User, 1)[0].page,
            Some(0x0707_0707_0707_0707)
        );
    }

    /// More samples than one take holds are all taken.
    #[test]
    fn every_sample_of_a_run_is_taken() {
        let (mut machine, base) = compiled();
        let index = index_of(&machine, base + 12);
        machine.set_step_timing(true);
        machine.host.block_samples = (0..3 * TAKEN as u64 + 1)
            .map(|_| BlockSample {
                block: index,
                cycles: 2,
            })
            .collect();
        machine.host.exits = alloc::vec![direct(machine.cpu.pc ^ base ^ (base + 12))];
        machine.step().unwrap();
        let mut samples = PcSamples::default();
        machine.take_pc_samples(&mut samples);
        let top = samples.top(Level::User, 4);
        assert_eq!(top.len(), 1);
        assert_eq!(top[0].pc, base + 12);
        assert_eq!(top[0].samples, 3 * TAKEN as u64 + 1);
    }
}

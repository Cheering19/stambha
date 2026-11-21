//! Where native code spends its host cycles, by guest block (`PES PC`,
//! under `--profile`). A host that samples where native code runs
//! (`Host::profile_blocks`) gives each of its samples in native code as the
//! word `ChainControl::block` held then, which names the block running, and
//! the cycles the sample stands for. The PE that ran the block names it
//! right after the run, while the block is still in its slot
//! (`Machine::take_native_samples`): its guest PC, its [`Level`] and the
//! identity of its code, the first 8 bytes of the SHA-256 of its 4 KiB
//! fetch page (the key the translation store names pages by) and its
//! offset there. An offline tool maps that identity to a dyld shared
//! cache's or a kernel collection's code without knowing the slide
//! (`cargo xtask symbolize`).
//!
//! Unlike `PES HOT`, which weighs blocks by transfer, a sample here weighs
//! the cycles its period counted, so a self-loop that runs a whole memcmp
//! counts for its time.
use super::Level;
use alloc::{collections::BTreeMap, format, string::String, vec::Vec};

/// One sample the host took in native code ([`crate::machine::Host::profile_blocks`]):
/// what `ChainControl::block` held, the block's index in its low 32 bits,
/// or [`NO_BLOCK`], and the host cycles the sample stands for.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BlockSample {
    pub block: u64,
    pub cycles: u64,
}

/// A sample no block is named for: taken while native code ran with no
/// chain control the host knew, or one the host had no room to keep.
pub const NO_BLOCK: u64 = u64::MAX;

/// A sample named by its block ([`PcSamples::add`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PcHit {
    /// As [`super::HotBlock::pc`]: an EL0 block's PC; a kernel or guarded
    /// block's PC less the sampling PE's vector base (wrapping).
    pub pc: u64,
    pub level: Level,
    /// The first 8 bytes of the SHA-256 of the block's 4 KiB fetch page, in
    /// their order (big-endian), or `None` where its bytes could not be
    /// read.
    pub page: Option<u64>,
    /// The block's offset in that page.
    pub offset: u16,
    pub cycles: u64,
}

/// A block's samples ([`PcSamples::top`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PcBlock {
    pub pc: u64,
    pub level: Level,
    pub page: Option<u64>,
    pub offset: u16,
    pub samples: u64,
    pub cycles: u64,
}

/// The blocks a trimmed [`PcSamples`] holds apart at most
/// ([`PcSamples::trim`]); samples of others are in [`PcSamples::other`].
pub const PC_BLOCKS: usize = 1 << 16;
/// The most blocks a table holds between trims; samples of new blocks
/// past them are in [`PcSamples::other`].
pub const PC_BLOCKS_MOST: usize = 2 * PC_BLOCKS;

/// A table key: level, page, offset, PC.
type Key = (u8, Option<u64>, u16, u64);

/// Samples by block, and by [`Level`].
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PcSamples {
    blocks: BTreeMap<Key, (u64, u64)>,
    /// Samples and cycles by level, the table's and the others'.
    pub samples: [u64; Level::COUNT],
    pub cycles: [u64; Level::COUNT],
    /// Samples and cycles of blocks a trim folded ([`Self::trim`]) or the
    /// table had no room for.
    pub other: [u64; 2],
    /// Samples and cycles whose block could not be named: its slot was
    /// empty, or the host kept no block for it ([`NO_BLOCK`]).
    pub gone: [u64; 2],
}

impl PcSamples {
    /// Count one sample.
    pub fn add(&mut self, hit: PcHit) {
        self.add_block(
            (hit.level as u8, hit.page, hit.offset, hit.pc),
            1,
            hit.cycles,
        );
    }

    /// Count `samples` samples of `cycles` of the block `key`.
    fn add_block(&mut self, key: Key, samples: u64, cycles: u64) {
        let level = usize::from(key.0).min(Level::COUNT - 1);
        self.samples[level] += samples;
        self.cycles[level] += cycles;
        let room = self.blocks.len() < PC_BLOCKS_MOST;
        match self.blocks.get_mut(&key) {
            Some(counts) => {
                counts.0 += samples;
                counts.1 += cycles;
            }
            None if room => {
                self.blocks.insert(key, (samples, cycles));
            }
            None => {
                self.other[0] += samples;
                self.other[1] += cycles;
            }
        }
    }

    /// When the table holds more than [`PC_BLOCKS`] blocks, keep the three
    /// quarters of that many with the most cycles and fold the others'
    /// samples and cycles into [`Self::other`]: a long boot's total keeps
    /// the blocks that matter, a later workload's hot ones among them,
    /// rather than the first it saw. It takes time in the blocks held, so
    /// the report calls it once a beat, and a trim leaves room for a
    /// quarter more before the next.
    pub fn trim(&mut self) {
        if self.blocks.len() <= PC_BLOCKS {
            return;
        }
        let keep = PC_BLOCKS * 3 / 4;
        let mut blocks: Vec<(Key, (u64, u64))> =
            core::mem::take(&mut self.blocks).into_iter().collect();
        // Most cycles first, then most samples, then by key: a total order.
        blocks.select_nth_unstable_by(keep, |a, b| {
            (b.1)
                .1
                .cmp(&(a.1).1)
                .then((b.1).0.cmp(&(a.1).0))
                .then(a.0.cmp(&b.0))
        });
        for (_, (samples, cycles)) in blocks.drain(keep..) {
            self.other[0] += samples;
            self.other[1] += cycles;
        }
        self.blocks = blocks.into_iter().collect();
    }

    /// Count `samples` samples of `cycles` that named no block.
    pub fn add_gone(&mut self, samples: u64, cycles: u64) {
        self.gone[0] += samples;
        self.gone[1] += cycles;
    }

    /// Add every sample `other` holds.
    pub fn merge(&mut self, other: &Self) {
        for (&key, &(samples, cycles)) in &other.blocks {
            self.add_block(key, samples, cycles);
        }
        // `add_block` counted the table's samples by level; the others'
        // only here.
        for level in 0..Level::COUNT {
            let (samples, cycles) = other
                .blocks
                .iter()
                .filter(|(key, _)| usize::from(key.0) == level)
                .fold((0, 0), |(s, c), (_, &(samples, cycles))| {
                    (s + samples, c + cycles)
                });
            self.samples[level] += other.samples[level] - samples;
            self.cycles[level] += other.cycles[level] - cycles;
        }
        for (mine, theirs) in self.other.iter_mut().zip(other.other) {
            *mine += theirs;
        }
        self.add_gone(other.gone[0], other.gone[1]);
    }

    pub fn is_empty(&self) -> bool {
        self.samples.iter().all(|&samples| samples == 0) && self.gone[0] == 0
    }

    /// The blocks with samples.
    pub fn len(&self) -> usize {
        self.blocks.len()
    }

    /// The `shown` blocks of `level` with the most cycles, most first, then
    /// those with more samples, then by page, offset and PC.
    pub fn top(&self, level: Level, shown: usize) -> Vec<PcBlock> {
        let mut blocks: Vec<PcBlock> = self
            .blocks
            .iter()
            .filter(|(key, _)| key.0 == level as u8)
            .map(|(&(_, page, offset, pc), &(samples, cycles))| PcBlock {
                pc,
                level,
                page,
                offset,
                samples,
                cycles,
            })
            .collect();
        // A total order, so that the blocks a total of tens of thousands
        // shows are selected first and only they are sorted.
        let order = |a: &PcBlock, b: &PcBlock| {
            b.cycles
                .cmp(&a.cycles)
                .then(b.samples.cmp(&a.samples))
                .then((a.page, a.offset, a.pc).cmp(&(b.page, b.offset, b.pc)))
        };
        if shown < blocks.len() {
            blocks.select_nth_unstable_by(shown, order);
            blocks.truncate(shown);
        }
        blocks.sort_unstable_by(order);
        blocks
    }

    /// The lines of a report under `prefix` (`PES PC`, `PES PC TOTAL`):
    ///
    /// - `{prefix} samples=N cycles=C gone=G/GC other=O/OC`: the samples
    ///   and their host cycles, then those that named no block and those
    ///   of blocks the table had no room for;
    /// - per level with samples, `{prefix} {level} samples=N cycles=C
    ///   blocks=B`, then its `shown` blocks with the most cycles, most
    ///   first, as ` {pc}:{samples}:{cycles}@{page}+{offset}`. The PC is
    ///   [`super::HotBlock::pc`]'s, an EL1 or guarded one as `v+0x...` or
    ///   `v-0x...` from the vector base; the page 16 hex digits, or `?`
    ///   where its bytes could not be read; the offset hex.
    ///
    /// Cycles are whole, so that sums of lines stay exact. No lines
    /// without samples.
    pub fn lines(&self, prefix: &str, shown: usize) -> Vec<String> {
        if self.is_empty() {
            return Vec::new();
        }
        let mut lines = alloc::vec![format!(
            "{prefix} samples={} cycles={} gone={}/{} other={}/{}",
            self.samples.iter().sum::<u64>(),
            self.cycles.iter().sum::<u64>(),
            self.gone[0],
            self.gone[1],
            self.other[0],
            self.other[1],
        )];
        for level in [Level::User, Level::Kernel, Level::Guarded] {
            let samples = self.samples[level as usize];
            if samples == 0 {
                continue;
            }
            let blocks = self
                .blocks
                .keys()
                .filter(|key| key.0 == level as u8)
                .count();
            let mut line = format!(
                "{prefix} {} samples={samples} cycles={} blocks={blocks}",
                level_name(level),
                self.cycles[level as usize],
            );
            for block in self.top(level, shown) {
                let page = match block.page {
                    Some(page) => format!("{page:016x}"),
                    None => String::from("?"),
                };
                line.push_str(&format!(
                    " {}:{}:{}@{page}+{:x}",
                    pc_name(level, block.pc),
                    block.samples,
                    block.cycles,
                    block.offset
                ));
            }
            lines.push(line);
        }
        lines
    }
}

/// A level's name in the lines.
pub fn level_name(level: Level) -> &'static str {
    match level {
        Level::User => "user",
        Level::Kernel => "kernel",
        Level::Guarded => "guarded",
    }
}

/// A block's PC as the lines print it ([`PcSamples::lines`]).
pub fn pc_name(level: Level, pc: u64) -> String {
    if level == Level::User {
        format!("{pc:#x}")
    } else if (pc as i64) >= 0 {
        format!("v+{pc:#x}")
    } else {
        format!("v-{:#x}", pc.wrapping_neg())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hit(pc: u64, level: Level, page: Option<u64>, cycles: u64) -> PcHit {
        PcHit {
            pc,
            level,
            page,
            offset: (pc & 0xfff) as u16,
            cycles,
        }
    }

    #[test]
    fn samples_rank_by_cycles_per_level_and_merge() {
        let mut samples = PcSamples::default();
        assert!(samples.is_empty());
        samples.add(hit(0x1000, Level::User, Some(7), 30));
        samples.add(hit(0x2000, Level::User, Some(8), 10));
        samples.add(hit(0x2000, Level::User, Some(8), 10));
        samples.add(hit(0x2000, Level::User, Some(8), 10));
        // The same PC in another page is other code: another process's.
        samples.add(hit(0x1000, Level::User, Some(9), 5));
        // And at another level, another block.
        samples.add(hit(0x1000, Level::Kernel, Some(7), 4));
        samples.add_gone(2, 50);
        assert!(!samples.is_empty());
        assert_eq!(samples.len(), 4);
        let top: Vec<(u64, Option<u64>, u64, u64)> = samples
            .top(Level::User, 8)
            .iter()
            .map(|b| (b.pc, b.page, b.samples, b.cycles))
            .collect();
        assert_eq!(
            top,
            [
                (0x2000, Some(8), 3, 30),
                (0x1000, Some(7), 1, 30),
                (0x1000, Some(9), 1, 5)
            ],
            "most cycles first, then most samples, then by key"
        );
        assert_eq!(samples.top(Level::User, 1).len(), 1);
        assert_eq!(samples.samples, [5, 1, 0]);
        assert_eq!(samples.cycles, [65, 4, 0]);
        assert_eq!(samples.gone, [2, 50]);
        let mut merged = PcSamples::default();
        merged.add(hit(0x2000, Level::User, Some(8), 1));
        merged.add_gone(1, 1);
        merged.merge(&samples);
        assert_eq!(merged.top(Level::User, 1)[0].cycles, 31);
        assert_eq!(merged.top(Level::User, 1)[0].samples, 4);
        assert_eq!(merged.samples, [6, 1, 0]);
        assert_eq!(merged.cycles, [66, 4, 0]);
        assert_eq!(merged.gone, [3, 51]);
        assert_eq!(merged.len(), 4);
    }

    /// Between trims a table holds at most PC_BLOCKS_MOST blocks: new
    /// blocks past it count as other.
    #[test]
    fn a_full_table_counts_new_blocks_as_other() {
        let mut samples = PcSamples::default();
        for index in 0..PC_BLOCKS_MOST as u64 + 3 {
            samples.add(hit(index * 4, Level::User, None, 2));
        }
        assert_eq!(samples.len(), PC_BLOCKS_MOST);
        assert_eq!(samples.other, [3, 6]);
        // A held block still counts.
        samples.add(hit(0, Level::User, None, 2));
        assert_eq!(samples.top(Level::User, 1)[0].samples, 2);
        assert_eq!(
            samples.samples[Level::User as usize],
            PC_BLOCKS_MOST as u64 + 4
        );
        let mut merged = PcSamples::default();
        merged.merge(&samples);
        assert_eq!(merged.other, [3, 6]);
        assert_eq!(merged.samples, samples.samples);
    }

    /// A trim keeps the blocks with the most cycles and folds the rest
    /// into other: a workload's hot blocks that came after the table filled
    /// with a boot's cold ones are kept, not lost to other.
    #[test]
    fn a_trim_keeps_the_blocks_with_the_most_cycles() {
        let mut samples = PcSamples::default();
        for index in 0..PC_BLOCKS as u64 {
            samples.add(hit(index * 4, Level::User, None, 1));
        }
        for index in 0..100u64 {
            samples.add(hit(0x100_0000 + index * 4, Level::User, Some(1), 1000));
        }
        samples.add(hit(0xffff_fe00_0000_0000, Level::Kernel, None, 500));
        assert_eq!(samples.other, [0, 0], "held until a trim");
        samples.trim();
        assert_eq!(samples.len(), PC_BLOCKS * 3 / 4);
        let hot = samples.top(Level::User, 100);
        assert!(hot.iter().all(|block| block.cycles == 1000), "{hot:?}");
        assert_eq!(samples.top(Level::Kernel, 4).len(), 1);
        // The folded blocks' samples and cycles are other's; the levels'
        // counts stand.
        let folded = (PC_BLOCKS + 101 - PC_BLOCKS * 3 / 4) as u64;
        assert_eq!(samples.other, [folded, folded]);
        assert_eq!(samples.samples, [PC_BLOCKS as u64 + 100, 1, 0]);
        assert_eq!(samples.cycles, [PC_BLOCKS as u64 + 100_000, 500, 0]);
        // A table with room is left as it is.
        let before = samples.clone();
        samples.trim();
        assert_eq!(samples, before);
    }

    #[test]
    fn pcs_print_as_hot_blocks_do() {
        assert_eq!(pc_name(Level::User, 0x1_8a2b_3c40), "0x18a2b3c40");
        assert_eq!(pc_name(Level::Kernel, 0x2c40), "v+0x2c40");
        assert_eq!(pc_name(Level::Guarded, 0u64.wrapping_sub(0x40)), "v-0x40");
    }

    #[test]
    fn lines_name_each_levels_blocks_with_their_pages() {
        assert!(PcSamples::default().lines("PES PC", 8).is_empty());
        let mut samples = PcSamples::default();
        samples.add(PcHit {
            pc: 0x1_8a2b_3c40,
            level: Level::User,
            page: Some(0x1f2e_3d4c_5b6a_7980),
            offset: 0xc40,
            cycles: 33_554_432,
        });
        samples.add(PcHit {
            pc: 0x1_8a2b_3c40,
            level: Level::User,
            page: Some(0x1f2e_3d4c_5b6a_7980),
            offset: 0xc40,
            cycles: 16_777_216,
        });
        samples.add(PcHit {
            pc: 0x2c4d0,
            level: Level::Kernel,
            page: None,
            offset: 0x4d0,
            cycles: 16_777_216,
        });
        samples.add(PcHit {
            pc: 0x100,
            level: Level::User,
            page: Some(0xab),
            offset: 0x100,
            cycles: 1_000_000,
        });
        samples.add_gone(1, 16_777_216);
        // Cycles are printed whole: a block of one sample of a 2^24
        // period reads 16777216, not 16M, and sums of lines stay exact.
        assert_eq!(
            samples.lines("PES PC", 8),
            [
                "PES PC samples=4 cycles=68108864 gone=1/16777216 other=0/0",
                "PES PC user samples=3 cycles=51331648 blocks=2 \
                 0x18a2b3c40:2:50331648@1f2e3d4c5b6a7980+c40 \
                 0x100:1:1000000@00000000000000ab+100",
                "PES PC kernel samples=1 cycles=16777216 blocks=1 v+0x2c4d0:1:16777216@?+4d0",
            ]
        );
        // `shown` bounds each level's blocks, not the counts.
        assert_eq!(
            samples.lines("PES PC TOTAL", 1)[1],
            "PES PC TOTAL user samples=3 cycles=51331648 blocks=2 \
             0x18a2b3c40:2:50331648@1f2e3d4c5b6a7980+c40"
        );
    }
}

//! The block samples `--profile`'s NMI takes in native code (`PES PC`):
//! each the word `ChainControl::block` held, which names the block running,
//! and the cycles the sample stands for. The NMI keeps them in its
//! processor's ring ([`BlockRing::push`]) and the engine on that processor
//! takes them after each native run (`Host::profile_blocks`), while the
//! blocks they name are still in their slots.
//!
//! One producer, the NMI, and one consumer, the code it interrupts: the NMI
//! writes only the slot past `head` and then moves `head`, and the consumer
//! reads the slots below `head` and then moves `tail`, so neither waits and
//! neither allocates, as the NMI gate requires. A sample that finds the
//! ring full keeps its cycles in `overflow`, which the next take gives as
//! one sample of [`NO_BLOCK`].
use core::sync::atomic::{AtomicU64, Ordering};
use stela_engine::machine::engine::step_timing::{BlockSample, NO_BLOCK};

/// Samples a ring holds: native runs last tens of microseconds and each
/// is drained at its end, while an NMI comes every few milliseconds.
pub const BLOCK_RING: usize = 64;

pub struct BlockRing {
    blocks: [AtomicU64; BLOCK_RING],
    cycles: [AtomicU64; BLOCK_RING],
    /// Samples pushed and taken so far.
    head: AtomicU64,
    tail: AtomicU64,
    /// The cycles of samples that found the ring full, not yet taken.
    overflow: AtomicU64,
}

impl Default for BlockRing {
    fn default() -> Self {
        Self::new()
    }
}

impl BlockRing {
    pub const fn new() -> Self {
        Self {
            blocks: [const { AtomicU64::new(0) }; BLOCK_RING],
            cycles: [const { AtomicU64::new(0) }; BLOCK_RING],
            head: AtomicU64::new(0),
            tail: AtomicU64::new(0),
            overflow: AtomicU64::new(0),
        }
    }

    /// The producer's: keep a sample of `block` standing for `cycles`.
    pub fn push(&self, block: u64, cycles: u64) {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);
        if head.wrapping_sub(tail) >= BLOCK_RING as u64 {
            self.overflow.fetch_add(cycles, Ordering::Relaxed);
            return;
        }
        let slot = head as usize % BLOCK_RING;
        self.blocks[slot].store(block, Ordering::Relaxed);
        self.cycles[slot].store(cycles, Ordering::Relaxed);
        self.head.store(head.wrapping_add(1), Ordering::Release);
    }

    /// The consumer's: move the oldest samples into `into`, as many as fit,
    /// the overflow's cycles last as one sample of [`NO_BLOCK`]; how many.
    pub fn take(&self, into: &mut [BlockSample]) -> usize {
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Relaxed);
        let count = (head.wrapping_sub(tail) as usize).min(into.len());
        for (index, sample) in into[..count].iter_mut().enumerate() {
            let slot = tail.wrapping_add(index as u64) as usize % BLOCK_RING;
            *sample = BlockSample {
                block: self.blocks[slot].load(Ordering::Relaxed),
                cycles: self.cycles[slot].load(Ordering::Relaxed),
            };
        }
        self.tail
            .store(tail.wrapping_add(count as u64), Ordering::Release);
        if count < into.len() {
            let cycles = self.overflow.swap(0, Ordering::Relaxed);
            if cycles != 0 {
                into[count] = BlockSample {
                    block: NO_BLOCK,
                    cycles,
                };
                return count + 1;
            }
        }
        count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn taken(ring: &BlockRing, room: usize) -> Vec<(u64, u64)> {
        let mut into = vec![BlockSample::default(); room];
        let count = ring.take(&mut into);
        into[..count]
            .iter()
            .map(|sample| (sample.block, sample.cycles))
            .collect()
    }

    #[test]
    fn samples_come_out_in_order_and_once() {
        let ring = BlockRing::new();
        assert!(taken(&ring, 4).is_empty());
        ring.push(3, 100);
        ring.push(1 << 40 | 7, 200);
        assert_eq!(taken(&ring, 4), [(3, 100), (1 << 40 | 7, 200)]);
        assert!(taken(&ring, 4).is_empty());
        // A take with less room leaves the rest for the next.
        for index in 0..5 {
            ring.push(index, 1);
        }
        assert_eq!(taken(&ring, 2), [(0, 1), (1, 1)]);
        assert_eq!(taken(&ring, 8), [(2, 1), (3, 1), (4, 1)]);
    }

    #[test]
    fn a_full_ring_keeps_the_cycles_as_no_block() {
        let ring = BlockRing::new();
        for index in 0..BLOCK_RING as u64 + 3 {
            ring.push(index, 10);
        }
        let all = taken(&ring, 2 * BLOCK_RING);
        assert_eq!(all.len(), BLOCK_RING + 1);
        assert_eq!(all[0], (0, 10));
        assert_eq!(all[BLOCK_RING - 1], (BLOCK_RING as u64 - 1, 10));
        assert_eq!(all[BLOCK_RING], (NO_BLOCK, 30), "three samples overflowed");
        assert!(taken(&ring, 4).is_empty());
        // The overflow waits for room in a take.
        for index in 0..BLOCK_RING as u64 + 1 {
            ring.push(index, 1);
        }
        assert_eq!(taken(&ring, BLOCK_RING).len(), BLOCK_RING);
        assert_eq!(taken(&ring, 4), [(NO_BLOCK, 1)]);
        // The ring wraps.
        for round in 0..3 * BLOCK_RING as u64 {
            ring.push(round, round);
            assert_eq!(taken(&ring, 1), [(round, round)]);
        }
    }

    /// A push between a take's reads and its move of `tail`, as an NMI's
    /// would be, neither overwrites what the take read nor is lost.
    #[test]
    fn a_push_during_a_take_is_kept() {
        let ring = BlockRing::new();
        for index in 0..BLOCK_RING as u64 - 1 {
            ring.push(index, 1);
        }
        // One slot is free: the interrupting push takes it.
        let head = ring.head.load(Ordering::Acquire);
        ring.push(99, 5);
        // The take saw `head` before the push: it moves `tail` to it.
        ring.tail.store(head, Ordering::Release);
        assert_eq!(taken(&ring, 4), [(99, 5)]);
    }
}

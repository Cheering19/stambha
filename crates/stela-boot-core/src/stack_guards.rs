//! The live stacks that lie between absent guard pages in a window clients
//! map into too, so a fault on a guard page is known for the host's own
//! stack running off its end, never a client mapping to refault.
//!
//! The #PF handler reads it on any processor, without a lock or an
//! allocation; stacks are added and removed rarely.
use core::ops::Range;
use core::sync::atomic::{AtomicU64, Ordering};

const PAGE: u64 = 4096;

/// A fault on a guard page of a registered stack.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GuardHit {
    /// The stack's usable bytes.
    pub usable: Range<u64>,
    /// The guard page above the stack's top, where a read up the stack past
    /// its first frame goes; otherwise the one below, where it overflows.
    pub above: bool,
}

/// Up to `N` stacks, each by its usable range.
pub struct StackGuards<const N: usize> {
    /// The usable start of each stack; zero for a free slot.
    starts: [AtomicU64; N],
    /// Its end, stored after the start and cleared before it.
    ends: [AtomicU64; N],
}

impl<const N: usize> Default for StackGuards<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> StackGuards<N> {
    pub const fn new() -> Self {
        Self {
            starts: [const { AtomicU64::new(0) }; N],
            ends: [const { AtomicU64::new(0) }; N],
        }
    }

    /// Records the stack whose usable pages are `usable`, with an absent
    /// page on each side. False when every slot is taken: its guards then
    /// go unnamed.
    pub fn add(&self, usable: Range<u64>) -> bool {
        if usable.start == 0 || usable.is_empty() {
            return false;
        }
        for (start, end) in self.starts.iter().zip(&self.ends) {
            if start
                .compare_exchange(0, usable.start, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                end.store(usable.end, Ordering::Release);
                return true;
            }
        }
        false
    }

    /// Forgets the stack whose usable pages start at `usable_start`.
    pub fn remove(&self, usable_start: u64) {
        for (start, end) in self.starts.iter().zip(&self.ends) {
            if start.load(Ordering::Acquire) == usable_start {
                end.store(0, Ordering::Release);
                start.store(0, Ordering::Release);
                return;
            }
        }
    }

    /// The stack `address` lies in a guard page of, if any.
    pub fn hit(&self, address: u64) -> Option<GuardHit> {
        self.starts.iter().zip(&self.ends).find_map(|(start, end)| {
            let start = start.load(Ordering::Acquire);
            let end = end.load(Ordering::Acquire);
            if start == 0 || end <= start {
                return None;
            }
            let below = start.checked_sub(PAGE)?..start;
            let above = end..end.checked_add(PAGE)?;
            (below.contains(&address) || above.contains(&address)).then(|| GuardHit {
                usable: start..end,
                above: above.contains(&address),
            })
        })
    }
}

/// The stack the boot's first stop does its work on: storing the logs runs
/// the USB's drivers, whose frames pass a Linux task's 16 KiB stack, and a
/// stop is often raised on one (a Linux driver's failed probe). Kept once,
/// for the boot.
pub struct StopStack {
    start: AtomicU64,
    end: AtomicU64,
}

impl Default for StopStack {
    fn default() -> Self {
        Self::new()
    }
}

impl StopStack {
    pub const fn new() -> Self {
        Self {
            start: AtomicU64::new(0),
            end: AtomicU64::new(0),
        }
    }

    /// Keeps the stack whose usable bytes are `usable`. False for an empty
    /// range or when one is kept already.
    pub fn keep(&self, usable: Range<u64>) -> bool {
        if usable.start == 0 || usable.is_empty() {
            return false;
        }
        if self
            .start
            .compare_exchange(0, usable.start, Ordering::AcqRel, Ordering::Relaxed)
            .is_err()
        {
            return false;
        }
        self.end.store(usable.end, Ordering::Release);
        true
    }

    /// Whether a stack is kept.
    pub fn kept(&self) -> bool {
        self.start.load(Ordering::Acquire) != 0
    }

    /// The top a stop running at stack pointer `pointer` moves to: none
    /// while no stack is kept, or when `pointer` lies on it already, as a
    /// stop within the stop does, whose caller's frames are below it.
    pub fn top_for(&self, pointer: u64) -> Option<u64> {
        let end = self.end.load(Ordering::Acquire);
        let start = self.start.load(Ordering::Acquire);
        if start == 0 || end <= start {
            return None;
        }
        // At the top itself nothing is pushed yet: a pointer there is on
        // the stack too.
        (!(start..=end).contains(&pointer)).then_some(end)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOP: u64 = 0xffff_8010_0d34_3000;
    const BYTES: u64 = 16 << 20;

    #[test]
    fn the_page_above_a_stack_top_is_its_guard() {
        // The fault that ended a boot: a scan up a job's stack read the
        // first word past its top.
        let guards = StackGuards::<4>::new();
        assert!(guards.add(TOP - BYTES..TOP));
        assert_eq!(
            guards.hit(TOP),
            Some(GuardHit {
                usable: TOP - BYTES..TOP,
                above: true
            })
        );
        assert_eq!(guards.hit(TOP + PAGE - 8).map(|hit| hit.above), Some(true));
        assert_eq!(
            guards.hit(TOP - BYTES - 8).map(|hit| hit.above),
            Some(false)
        );
    }

    #[test]
    fn the_stack_itself_and_pages_past_its_guards_are_not_guards() {
        let guards = StackGuards::<4>::new();
        assert!(guards.add(TOP - BYTES..TOP));
        assert_eq!(guards.hit(TOP - 8), None);
        assert_eq!(guards.hit(TOP - BYTES), None);
        assert_eq!(guards.hit(TOP + PAGE), None);
        assert_eq!(guards.hit(TOP - BYTES - PAGE - 8), None);
    }

    #[test]
    fn a_removed_stack_frees_its_slot_and_its_guards() {
        let guards = StackGuards::<1>::new();
        assert!(guards.add(TOP - BYTES..TOP));
        assert!(!guards.add(0x1000..0x2000), "only one slot");
        guards.remove(TOP - BYTES);
        assert_eq!(guards.hit(TOP), None);
        assert!(guards.add(0x1000..0x2000));
        assert_eq!(
            guards.hit(0x2000).map(|hit| hit.usable),
            Some(0x1000..0x2000)
        );
    }

    #[test]
    fn a_stop_on_a_linux_task_stack_moves_to_the_stop_stack() {
        // igpu3: amdgpu's failed probe stopped the boot on its 16 KiB task
        // stack at 0xffff_8008_0075_8000, and the log's store through the
        // USB's drivers ran off its bottom.
        let task = 0xffff_8008_0075_8000..0xffff_8008_0075_c000_u64;
        let stop = StopStack::new();
        assert!(!stop.kept());
        assert_eq!(stop.top_for(task.end - 0x200), None, "none kept yet");
        const STOP_BYTES: u64 = 1 << 20;
        assert!(stop.keep(TOP - STOP_BYTES..TOP));
        assert!(stop.kept());
        assert!(!stop.keep(0x1000..0x2000), "kept once");
        assert_eq!(stop.top_for(task.start + 0x100), Some(TOP));
        // A stop within the stop, already on it, stays where it is.
        assert_eq!(stop.top_for(TOP - 0x800), None);
        assert_eq!(stop.top_for(TOP), None);
        assert_eq!(stop.top_for(TOP - STOP_BYTES), None);
        // Just past either end is another stack.
        assert_eq!(stop.top_for(TOP + 8), Some(TOP));
        assert_eq!(stop.top_for(TOP - STOP_BYTES - 8), Some(TOP));
    }

    #[test]
    fn an_empty_stop_stack_is_not_kept() {
        let stop = StopStack::new();
        assert!(!stop.keep(0x1000..0x1000));
        assert!(!stop.keep(0..0x1000));
        assert_eq!(stop.top_for(0x800), None);
    }

    #[test]
    fn an_empty_range_is_not_recorded() {
        let guards = StackGuards::<1>::new();
        assert!(!guards.add(0x1000..0x1000));
        assert!(!guards.add(0..0x1000));
        assert_eq!(guards.hit(0x1000), None);
    }
}

//! The kernel's lines of a GPU fault, for the Metal device's fault dump
//! (`stela_driver_gpu::metal::fault_dump`) to follow with: nouveau's
//! `Xid` lines (GSP-RM's error log, whose Xid 13 text names the exception
//! and the ESR words, such as `Graphics Exception: ESR 0x405840=...`), its
//! `rc` line (the channel GSP-RM reset, by chid and exception type) and the
//! channel it disabled. A WindowServer that faults at each start prints
//! hundreds, so the first of the boot are kept, with a count of all.
use alloc::{format, string::String, vec::Vec};
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, AtomicUsize, Ordering};

/// What a kernel line of a GPU fault contains.
const MARKERS: [&str; 3] = ["Xid:", " rc engn:", "errored - disabling channel"];

/// Whether `line` is a kernel line of a GPU fault.
pub fn is_fault_line(line: &str) -> bool {
    MARKERS.iter().any(|marker| line.contains(marker))
}

/// The first `SLOTS` kernel lines of GPU faults of the boot, each cut to
/// `BYTES`, and how many there were. Each slot is written once; read
/// without a lock.
pub struct FaultLines<const SLOTS: usize, const BYTES: usize> {
    lines: [Line<BYTES>; SLOTS],
    claimed: AtomicUsize,
    seen: AtomicU64,
}

struct Line<const BYTES: usize> {
    ready: AtomicBool,
    length: AtomicUsize,
    bytes: [AtomicU8; BYTES],
}

impl<const SLOTS: usize, const BYTES: usize> Default for FaultLines<SLOTS, BYTES> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const SLOTS: usize, const BYTES: usize> FaultLines<SLOTS, BYTES> {
    pub const fn new() -> Self {
        Self {
            lines: [const {
                Line {
                    ready: AtomicBool::new(false),
                    length: AtomicUsize::new(0),
                    bytes: [const { AtomicU8::new(0) }; BYTES],
                }
            }; SLOTS],
            claimed: AtomicUsize::new(0),
            seen: AtomicU64::new(0),
        }
    }

    /// Counts `line` if it is a GPU fault's and keeps it while a slot is
    /// free, cut to the slot's size at a character boundary; whether it is
    /// one.
    pub fn observe(&self, line: &str) -> bool {
        if !is_fault_line(line) {
            return false;
        }
        self.seen.fetch_add(1, Ordering::Relaxed);
        let Ok(slot) = self
            .claimed
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |claimed| {
                (claimed < SLOTS).then_some(claimed + 1)
            })
        else {
            return true;
        };
        let mut end = line.len().min(BYTES);
        while !line.is_char_boundary(end) {
            end -= 1;
        }
        let slot = &self.lines[slot];
        for (byte, value) in slot.bytes.iter().zip(line[..end].bytes()) {
            byte.store(value, Ordering::Relaxed);
        }
        slot.length.store(end, Ordering::Relaxed);
        slot.ready.store(true, Ordering::Release);
        true
    }

    /// The kept lines, in the order they were printed.
    pub fn kept(&self) -> Vec<String> {
        self.lines
            .iter()
            .take_while(|slot| slot.ready.load(Ordering::Acquire))
            .filter_map(|slot| {
                let length = slot.length.load(Ordering::Relaxed);
                let bytes = slot.bytes[..length]
                    .iter()
                    .map(|byte| byte.load(Ordering::Relaxed))
                    .collect();
                String::from_utf8(bytes).ok()
            })
            .collect()
    }

    /// The GPU fault lines the kernel printed this boot.
    pub fn seen(&self) -> u64 {
        self.seen.load(Ordering::Relaxed)
    }
}

/// The lines that follow fault `fault`'s opening in the log, each after
/// the log's `GPU FAULT `: the first fault gives the kept lines, and every
/// fault the count so far, which says whether each start faults alike.
pub fn kernel_lines(fault: u32, kept: &[String], seen: u64) -> Vec<String> {
    let mut lines = Vec::new();
    if fault == 1 {
        lines.extend(kept.iter().map(|line| format!("{fault} kernel: {line}")));
    }
    let shown = match (fault, kept.len()) {
        (_, 0) => String::new(),
        (1, count) => format!(", the first {count} above"),
        (_, count) => format!(", the first {count} after fault 1"),
    };
    lines.push(format!(
        "{fault} kernel: {seen} Xid, rc and disabled-channel lines so far this boot{shown}"
    ));
    lines
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    const XID: &str = "nouveau 0000:01:00.0: gsp: Xid:13 Graphics Exception: Shader Program \
                       Header 11 Error";
    const ESR: &str = "nouveau 0000:01:00.0: gsp: Xid:13 Graphics Exception: ESR \
                       0x405840=0xa2040800 0x405844=0x0 0x405848=0x0 0x40584c=0x0";
    const RC: &str = "nouveau 0000:01:00.0: gsp: rc engn:00000001 chid:22 gfid:0 level:2 \
                      type:13 scope:1 part:233 fault_addr:0000000000000000 fault_type:ffffffff";
    const DISABLED: &str = "nouveau 0000:01:00.0: fifo:000000:0016:[stela-managed[22]] errored \
                            - disabling channel";

    #[test]
    fn the_fault_lines_are_nouveaus_xid_rc_and_disabled_channel() {
        for line in [XID, ESR, RC, DISABLED] {
            assert!(is_fault_line(line), "{line}");
        }
        for line in [
            "nouveau 0000:01:00.0: gsp: mmu fault queued",
            "nouveau 0000:01:00.0: stopped answering: BOOT_0 read ffffffff for 1004 ms",
            "GPU REFUSED DeviceLost in operation 12",
        ] {
            assert!(!is_fault_line(line), "{line}");
        }
    }

    /// The first lines of the boot are kept in order, each cut at a
    /// character boundary, and every one is counted.
    #[test]
    fn the_first_lines_are_kept_and_every_one_counted() {
        let lines: FaultLines<2, 64> = FaultLines::new();
        assert!(!lines.observe("nouveau 0000:01:00.0: gsp: mmu fault queued"));
        assert_eq!(lines.seen(), 0);
        for line in [XID, ESR, RC, DISABLED] {
            assert!(lines.observe(line));
        }
        assert_eq!(lines.seen(), 4);
        assert_eq!(lines.kept(), [&XID[..64], &ESR[..64]]);
        let narrow: FaultLines<1, 5> = FaultLines::new();
        narrow.observe("Xid:é13");
        assert_eq!(
            narrow.kept(),
            ["Xid:"],
            "cut before a character the slot splits"
        );
    }

    #[test]
    fn the_first_fault_gives_the_kept_lines_and_each_fault_the_count() {
        let kept = vec![String::from(XID), String::from(ESR)];
        assert_eq!(
            kernel_lines(1, &kept, 2),
            [
                format!("1 kernel: {XID}"),
                format!("1 kernel: {ESR}"),
                String::from(
                    "1 kernel: 2 Xid, rc and disabled-channel lines so far this boot, the first \
                     2 above"
                ),
            ]
        );
        assert_eq!(
            kernel_lines(3, &kept, 240),
            [
                "3 kernel: 240 Xid, rc and disabled-channel lines so far this boot, the first 2 \
              after fault 1"
            ]
        );
        assert_eq!(
            kernel_lines(1, &[], 0),
            ["1 kernel: 0 Xid, rc and disabled-channel lines so far this boot"]
        );
    }
}

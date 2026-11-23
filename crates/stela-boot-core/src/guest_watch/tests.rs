use super::*;
use crate::console_lines::Lines;
use alloc::{string::String, vec::Vec};

/// The console of the 2026-11-23 boot that panicked at 42.7 s and then spun
/// at one PC for twenty minutes, around its panic, as XNU printed it: a
/// line that only mentions panics first, the nested panic's log, then the
/// original header and its log.
const PANICKED: &str = "AppleInterruptControllerV2::start: Num Shared Timestamps == 16\n\
Reconfig Watchdog: ICC = 360000000\n\
panic flow Config Engine Trigger NOT found\n\
IOReturn AppleSEPManager::_setPowerState(unsigned long *): power state 0 -> 2\n\
Nested panic detected - entry count: 2 panic_caller: 0xfffffe003acdbef4\n\
Kernel text exec slide: 0x0000000033340000\n\
Panicked thread: 0xfffffe2be43b0900, backtrace: 0xfffffe83e7f66b80, tid: 497\n\
\t\t  lr: 0xfffffe003a397f3c  fp: 0xfffffe83e7f66c00\n\
\t  pc:  0xfffffe0039e19768 cpsr: 0x604013c8         esr: 0x0000000096000005  far: 0x0000000000000124\r\n\
Nested panic string:\n\
\n\
<end nested panic string>\n\
panic(cpu 0 caller 0xfffffe003acd0158): Ticket spinlock[0xfffffe003b18d000] timeout after 4243396 ticks; cticket: 0x9, nticket: 0xa, waiting for 0x9, current owner: 0 (on CPU -1), start time: 313898759, now: 318142155, timeout: 3000000 @lock_ticket.c:399\n\
Debugger message: panic\n\
Memory ID: 0x0\n\
Kernel version: Darwin Kernel Version 27.2.0: Wed Oct 14 23:48:16 PDT 2026; root:xnu-13432.41.4~25/RELEASE_ARM64_T8112\n\
Paniclog version: 16\n\
CORE 0 is the one that panicked. Check the full backtrace for details.\n\
CORE 1: PC=0x0000000000000000, LR=0x0000000000000000, FP=0x0000000000000000\n\
Panicked task 0xfffffe2be4034d80: 0 pages, 163 threads: pid 0: kernel_task\n\
Panicked thread: 0xfffffe2be43b0900, backtrace: 0xfffffe83e7f666c0, tid: 497\n\
\t\t  lr: 0xfffffe003a3984e0  fp: 0xfffffe83e7f66760\n";

/// The 42.7 s boot's header, as the watch keeps it.
const HEADER: &str = "panic(cpu 0 caller 0xfffffe003acd0158): Ticket spinlock[0xfffffe003b18d000] \
     timeout after 4243396 ticks; cticket: 0x9, nticket: 0xa, waiting for 0x9, current owner: 0 \
     (on CPU -1), start time: 313898759, now: 318142155, timeout: 3000000 @lock_ticket.c:399";

/// Feed `console` to `watch` through the UART's line assembly in chunks of
/// `chunk` bytes, one millisecond a chunk from `start`, as the home PE
/// writes the lines the UART ended; the time after the last chunk.
fn feed(watch: &mut Watch, console: &[u8], chunk: usize, start: u64) -> u64 {
    let mut lines = Lines::default();
    let mut now = start;
    for piece in console.chunks(chunk) {
        for &byte in piece {
            if let Some(line) = lines.push(byte) {
                watch.line(now, &line.text, line.beginning);
            }
        }
        now += 1;
    }
    now
}

#[test]
fn a_panic_header_split_across_console_chunks_is_seen_once_whole() {
    for chunk in [1, 3, 7, 64, 4096] {
        let mut watch = Watch::new();
        let end = feed(&mut watch, PANICKED.as_bytes(), chunk, 42_000);
        let panic = watch.panic().expect("the header");
        assert!(
            panic
                .line
                .starts_with("panic(cpu 0 caller 0xfffffe003acd0158): Ticket spinlock")
        );
        assert!(panic.line.len() <= KEPT_BYTES + 4, "{}", panic.line);
        assert!(HEADER.starts_with(panic.line.trim_end_matches(" ...")));
        // Its log after it: every line with text, not the header itself.
        assert_eq!(panic.lines, 9, "chunk {chunk}");
        assert!(panic.at >= 42_000 && panic.at < end);
    }
}

#[test]
fn the_run_stops_after_the_hold_when_no_reset_came() {
    let mut watch = Watch::new();
    assert_eq!(watch.poll(39_553, false), None);
    feed(&mut watch, PANICKED.as_bytes(), 61, 42_700);
    let at = watch.panic().unwrap().at;
    match watch.poll(at + 1, false) {
        Some(Event::Panicked { line, keep_running }) => {
            assert!(line.starts_with("panic(cpu 0 "));
            assert!(!keep_running);
        }
        other => panic!("{other:?}"),
    }
    // The panic's log and a reset that may still come get the hold.
    assert_eq!(watch.poll(at + PANIC_HOLD_MILLIS - 1, false), None);
    match watch.poll(at + PANIC_HOLD_MILLIS, false) {
        Some(Event::Stop(panic)) => {
            assert_eq!(panic.at, at);
            assert_eq!(panic.lines, 9);
        }
        other => panic!("{other:?}"),
    }
    // Once.
    assert_eq!(watch.poll(at + PANIC_HOLD_MILLIS + 1, true), None);
}

#[test]
fn keep_running_names_the_panic_and_never_stops() {
    let mut watch = Watch::new();
    watch.keep_running(true);
    feed(&mut watch, PANICKED.as_bytes(), 4096, 42_700);
    assert!(matches!(
        watch.poll(42_701, false),
        Some(Event::Panicked {
            keep_running: true,
            ..
        })
    ));
    assert_eq!(watch.poll(42_700 + 20 * PANIC_HOLD_MILLIS, true), None);
}

#[test]
fn lines_that_only_mention_a_panic_are_no_panic() {
    let mut watch = Watch::new();
    let console = "kernel panic medic: nothing to report\n\
        panic flow Config Engine Trigger NOT found\n\
        AppleImage4: panic-on-failure policy=0 image4_panic(cpu 0) not taken\n\
        Nested panic detected - entry count: 2\n\
        driver: panic(cpu 1 caller 0x1000): ordinary data\n\
        panic(cpu x caller 0x1000): not XNU's header\n\
        Panicked task 0xfffffe2be4034d80: 0 pages\n\
        PANIC(cpu 0 caller 0x1): another case\n";
    feed(&mut watch, console.as_bytes(), 5, 1_000);
    assert_eq!(watch.panic(), None);
    assert_eq!(watch.poll(1_000 + 10 * PANIC_HOLD_MILLIS, true), None);
    for line in console.lines() {
        assert!(!begins_panic(line.trim_start()), "{line}");
    }
}

#[test]
fn a_header_inside_a_long_line_s_continuation_is_no_panic() {
    let mut watch = Watch::new();
    let mut console: Vec<u8> = core::iter::repeat_n(b'x', 1024).collect();
    console.extend_from_slice(b"panic(cpu 0 caller 0x1000): inside another line\n");
    feed(&mut watch, &console, 100, 0);
    assert_eq!(watch.panic(), None);
}

#[test]
fn a_quiet_console_before_the_first_frame_is_named_once_a_stretch() {
    let mut watch = Watch::new();
    assert_eq!(watch.poll(40_000, false), None);
    watch.line(41_000, "AppleSEPManager::_setPowerState 0 -> 2", true);
    assert_eq!(watch.poll(41_000 + STALL_MILLIS - 1, false), None);
    assert_eq!(
        watch.poll(41_000 + STALL_MILLIS, false),
        Some(Event::Stalled {
            seconds: STALL_MILLIS / 1000,
            last: Some(String::from("AppleSEPManager::_setPowerState 0 -> 2")),
        })
    );
    // Not again in the same stretch.
    assert_eq!(watch.poll(41_000 + 5 * STALL_MILLIS, false), None);
    // A new line starts a new stretch; blank lines do not.
    watch.line(800_000, "IOConsoleUsers: gIOScreenLockState 3", true);
    watch.line(900_000, "   ", true);
    assert_eq!(watch.poll(800_000 + STALL_MILLIS - 1, false), None);
    assert!(matches!(
        watch.poll(800_000 + STALL_MILLIS, false),
        Some(Event::Stalled { seconds: 120, .. })
    ));
}

#[test]
fn a_console_that_never_printed_is_named_from_when_watching_began() {
    let mut watch = Watch::new();
    assert_eq!(watch.poll(10_000, false), None);
    assert_eq!(
        watch.poll(10_000 + STALL_MILLIS, false),
        Some(Event::Stalled {
            seconds: STALL_MILLIS / 1000,
            last: None
        })
    );
}

#[test]
fn a_quiet_console_after_the_guest_s_first_frame_is_not_named() {
    let mut watch = Watch::new();
    watch.line(0, "loginwindow: started", true);
    assert_eq!(watch.poll(10 * STALL_MILLIS, true), None);
}

#[test]
fn a_kept_line_is_cut_at_a_character_boundary() {
    let long = "é".repeat(KEPT_BYTES);
    let line = kept(&long);
    assert!(line.ends_with(" ..."));
    assert!(line.len() <= KEPT_BYTES + 4);
    assert_eq!(kept("short  "), "short");
}

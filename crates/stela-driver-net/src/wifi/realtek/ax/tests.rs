extern crate std;
use super::*;
use crate::Clock;
use alloc::{sync::Arc, vec::Vec};
use core::{ptr::NonNull, sync::atomic::AtomicU32};
use model::{Op, RegisterModel};

pub(crate) mod model;

#[test]
fn field32_shifts_value_into_mask() {
    let mut m = RegisterModel::new();
    field32(&mut m, 0x0200, 0xE000, 7).unwrap();
    assert_eq!(m.get32(0x0200), 0xE000);
    // Neighbouring bits survive and the old field is replaced.
    m.set32(0x0200, 0xFFFF_FFFF);
    field32(&mut m, 0x0200, 0xE000, 2).unwrap();
    assert_eq!(m.get32(0x0200), 0xFFFF_5FFF);
    // An over-wide value never leaks outside the mask.
    field32(&mut m, 0x0200, 0xE000, 0xFF).unwrap();
    assert_eq!(m.get32(0x0200), 0xFFFF_FFFF);
}

#[test]
fn set_and_clear_are_read_modify_write_at_each_width() {
    let mut m = RegisterModel::new();
    m.set32(0x10, 0x1234_5678);
    set32(&mut m, 0x10, 0x8000_0001).unwrap();
    clear32(&mut m, 0x10, 0x0000_00F0).unwrap();
    assert_eq!(m.get32(0x10), 0x9234_5609);
    assert_eq!(
        m.trace,
        [
            Op::R32(0x10),
            Op::W32(0x10, 0x9234_5679),
            Op::R32(0x10),
            Op::W32(0x10, 0x9234_5609)
        ]
    );
    m.set32(0x20, 0xAABB_CCDD);
    set16(&mut m, 0x20, 0x0F00).unwrap();
    clear16(&mut m, 0x22, 0x00FF).unwrap();
    field16(&mut m, 0x20, 0x00F0, 0x3).unwrap();
    assert_eq!(m.get32(0x20), 0xAA00_CF3D);
    set8(&mut m, 0x20, 0x80).unwrap();
    clear8(&mut m, 0x23, 0xF0).unwrap();
    field8(&mut m, 0x23, 0x0C, 1).unwrap();
    assert_eq!(m.get32(0x20), 0x0600_CFBD);
}

#[test]
fn writes_helper_drops_reads_and_keeps_delays() {
    let mut m = RegisterModel::new();
    set32(&mut m, 4, 1).unwrap();
    m.delay_us(10);
    m.read8(0).unwrap();
    m.write16(2, 7).unwrap();
    assert_eq!(
        m.writes(),
        [Op::W32(4, 1), Op::Delay(10), Op::W16(2, 7)].to_vec()
    );
    assert_eq!(m.trace.len(), 5);
}

#[test]
fn poll_succeeds_after_hook_flips_ready_bit() {
    let mut m = RegisterModel::new();
    let mut reads = 0;
    m.on_read(
        0x30,
        alloc::boxed::Box::new(move |_m: &mut RegisterModel| {
            reads += 1;
            if reads >= 4 { 0x10 } else { 0 }
        }),
    );
    poll32(&mut m, 0x30, 0x10, 0x10, 5, 1_000).unwrap();
    let r = m.trace.iter().filter(|o| **o == Op::R32(0x30)).count();
    assert_eq!(r, 4);
    assert_eq!(m.writes(), [Op::Delay(5); 3].to_vec());
}

#[test]
fn poll_times_out_on_virtual_clock() {
    let mut m = RegisterModel::new();
    let start = m.now_us().unwrap();
    assert_eq!(
        poll32(&mut m, 0x40, 0x1, 0x1, 10, 500),
        Err(AxError::Timeout {
            register: 0x40,
            mask: 0x1
        })
    );
    assert!(m.now_us().unwrap() - start >= 500);
}

#[test]
fn poll_with_stalled_clock_reports_clock_error_not_hang() {
    let mut m = RegisterModel::new();
    m.freeze_clock = true;
    assert_eq!(poll32(&mut m, 0x40, 1, 1, 10, 500), Err(AxError::Clock));
}

#[test]
fn poll_rejects_all_ones() {
    let mut m = RegisterModel::new();
    m.set32(0x50, 0xFFFF_FFFF);
    assert_eq!(
        poll32(&mut m, 0x50, 0x1, 0x0, 10, 500),
        Err(AxError::AllOnes { register: 0x50 })
    );
    assert_eq!(m.trace, [Op::R32(0x50)].to_vec());
}

#[test]
fn error_display_names_the_register() {
    use alloc::string::ToString;
    let t = AxError::Timeout {
        register: 0x1234,
        mask: 0x10,
    }
    .to_string();
    assert!(t.contains("0x1234") && t.contains("0x10"));
    assert!(AxError::Firmware("x").to_string().contains('x'));
}

const OK_BYTES: usize = 0x40004;
const CLOCK: Clock = Clock {
    ticks_per_microsecond: 1,
};

fn window() -> Arc<Vec<AtomicU32>> {
    Arc::new((0..OK_BYTES / 4).map(|_| AtomicU32::new(0)).collect())
}

fn native(words: &Arc<Vec<AtomicU32>>, bytes: usize, clock: Clock) -> Result<NativeAxIo, AxError> {
    let ptr = NonNull::new(words[0].as_ptr() as *mut u8).unwrap();
    // SAFETY: the test-owned window is plain memory used by this test alone;
    // `words` keeps it alive and nothing else accesses it.
    unsafe { NativeAxIo::new(ptr, bytes, words.clone(), clock) }
}

#[test]
fn native_refuses_small_window() {
    let words = window();
    assert!(matches!(
        native(&words, 0x1000, CLOCK),
        Err(AxError::OutOfWindow { .. })
    ));
    assert!(matches!(
        native(&words, OK_BYTES - 4, CLOCK),
        Err(AxError::OutOfWindow { .. })
    ));
    assert!(matches!(
        native(
            &words,
            OK_BYTES,
            Clock {
                ticks_per_microsecond: 0
            }
        ),
        Err(AxError::Clock)
    ));
}

#[test]
fn native_checks_bounds_alignment_and_quarantine() {
    let words = window();
    let weak = Arc::downgrade(&words);
    let _time = crate::test_clock::virtualize();
    let mut io = native(&words, OK_BYTES, CLOCK).unwrap();
    drop(words);
    let oob = |register| AxError::OutOfWindow { register };
    io.write32(0x40000, 0x1234_abcd).unwrap();
    assert_eq!(io.read32(0x40000), Ok(0x1234_abcd));
    assert_eq!(io.read8(0x40001), Ok(0xab));
    assert_eq!(io.read16(0x40002), Ok(0x1234));
    io.write16(0x40002, 0x5555).unwrap();
    io.write8(0x40000, 0x11).unwrap();
    assert_eq!(io.read32(0x40000), Ok(0x5555_ab11));
    assert_eq!(io.read32(OK_BYTES as u32), Err(oob(OK_BYTES as u32)));
    assert_eq!(io.read32(u32::MAX), Err(oob(u32::MAX)));
    assert_eq!(io.read16(0x41), Err(oob(0x41)));
    assert_eq!(io.write32(2, 0), Err(oob(2)));
    let t = io.now_us().unwrap();
    assert!(io.now_us().unwrap() >= t);
    io.quarantine();
    assert_eq!(io.read32(0), Err(oob(0)));
    assert_eq!(io.now_us(), Err(AxError::Clock));
    drop(io);
    assert!(
        weak.upgrade().is_some(),
        "quarantined owner must be retained"
    );
}

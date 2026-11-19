extern crate std;
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::{boxed::Box, format, vec::Vec};

fn without_delays(ops: Vec<Op>) -> Vec<Op> {
    ops.into_iter()
        .filter(|op| !matches!(op, Op::Delay(_)))
        .collect()
}

/// The XTAL SI command bit (0x0270 bit 31) self-clears, as the hardware's does.
fn settled() -> RegisterModel {
    let mut m = RegisterModel::new();
    m.on_read(
        0x0270,
        Box::new(|m: &mut RegisterModel| m.get32(0x0270) & 0x7FFF_FFFF),
    );
    m
}

#[test]
fn enable_bb_rf_matches_linux_sequence() {
    let mut m = settled();
    m.set32(0x0002, 0);
    m.set32(0x02F0, 0x0002_0000);
    m.set32(0x0444, 0xFFFF_FFFF);
    enable_bb_rf(&mut m).unwrap();
    assert_eq!(
        without_delays(m.writes()),
        vec![
            Op::W8(0x0002, 0x03),
            Op::W32(0x02F0, 0x0002_0000),
            Op::W32(0x02F0, 0),
            Op::W32(0x02F0, 0x0002_0000),
            Op::W32(0x0444, 0xFFBF_FFFF),
            Op::W32(0x0444, 0xFDBF_FFFF),
            Op::W32(0x0270, 0x80FF_0700),
            Op::W32(0x0270, 0x80FF_6C90),
            Op::W32(0x0270, 0x80FF_C780),
            Op::W32(0x0270, 0x80FF_C781),
            Op::W32(0x0270, 0x80FF_0D03),
        ]
    );
}

#[test]
fn reset_bb_rf_disables_then_enables() {
    // rtw89_chip_reset_bb_rf (mac.h:1377-1391): rtw8852c_mac_disable_bb_rf
    // (rtw8852c.c:3036-3043), then the M2a enable sequence unchanged.
    let mut m = settled();
    m.set32(0x02F0, 0x0002_0000);
    m.set32(0x0002, 0x03);
    m.set32(0x0444, 0xFFFF_FFFF);
    reset_bb_rf(&mut m).unwrap();
    assert_eq!(
        without_delays(m.writes()),
        vec![
            Op::W32(0x02F0, 0),
            Op::W8(0x0002, 0),
            Op::W8(0x0002, 0x03),
            Op::W32(0x02F0, 0x0002_0000),
            Op::W32(0x02F0, 0),
            Op::W32(0x02F0, 0x0002_0000),
            Op::W32(0x0444, 0xFFBF_FFFF),
            Op::W32(0x0444, 0xFDBF_FFFF),
            Op::W32(0x0270, 0x80FF_0700),
            Op::W32(0x0270, 0x80FF_6C90),
            Op::W32(0x0270, 0x80FF_C780),
            Op::W32(0x0270, 0x80FF_C781),
            Op::W32(0x0270, 0x80FF_0D03),
        ]
    );
    // Both disable steps are read-modify-writes that keep the other bits.
    assert_eq!(
        &m.trace[..4],
        [
            Op::R32(0x02F0),
            Op::W32(0x02F0, 0),
            Op::R8(0x0002),
            Op::W8(0x0002, 0),
        ]
    );
}

#[test]
fn disable_bb_rf_keeps_the_other_bits() {
    let mut m = RegisterModel::new();
    m.set32(0x02F0, 0xFFFF_FFFF);
    m.set32(0x0000, 0xFFFF_FFFF);
    disable_bb_rf(&mut m).unwrap();
    assert_eq!(
        m.writes(),
        vec![Op::W32(0x02F0, 0xFFFD_FFFF), Op::W8(0x0002, 0xFC)]
    );
}

#[test]
fn reset_bb_rf_xtal_timeout_fails_after_the_disable() {
    let mut m = RegisterModel::new(); // 0x0270 bit 31 never clears
    assert_eq!(
        reset_bb_rf(&mut m),
        Err(AxError::Timeout {
            register: 0x270,
            mask: 0x8000_0000
        })
    );
    assert_eq!(m.writes()[..2], [Op::W32(0x02F0, 0), Op::W8(0x0002, 0)]);
}

#[test]
fn sys_init_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    sys_init(&mut m).unwrap();
    assert_eq!(
        m.writes(),
        vec![
            Op::W32(0x8400, 0xFB7D_4000),
            Op::W32(0x8404, 0x0B1F_0000),
            Op::W32(0xC004, 0x4000_003F),
            Op::W32(0xC000, 0xF000_003F),
        ]
    );
}

#[test]
fn enable_bb_rf_xtal_timeout_fails() {
    let mut m = RegisterModel::new(); // 0x0270 bit 31 never clears
    assert_eq!(
        enable_bb_rf(&mut m),
        Err(AxError::Timeout {
            register: 0x270,
            mask: 0x8000_0000
        })
    );
}

#[test]
fn new_errors_display() {
    assert_eq!(format!("{}", AxError::Mac("mac says")), "mac says");
    assert_eq!(format!("{}", AxError::Rx("rx says")), "rx says");
    assert_eq!(
        format!(
            "{}",
            AxError::NoAck {
                class: 1,
                func: 0x10,
                seq: 7,
                other_c2h: 2,
                unmatched_ack: 3,
                skipped: 4,
                rxq_hw: 5,
            }
        ),
        "firmware sent no acknowledgement of H2C class 1 func 0x10 seq 7 within 100 ms (RXQ \
         index 5, other C2H 2, unmatched acks 3, skipped 4)"
    );
    assert_eq!(
        format!(
            "{}",
            AxError::H2cRejected {
                class: 1,
                func: 0x10,
                seq: 7,
                ret: 3
            }
        ),
        "firmware refused H2C class 1 func 0x10 seq 7 (raw return 0x03)"
    );
}

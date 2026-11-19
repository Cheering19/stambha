// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::{vec, vec::Vec};

const F: u32 = 0xFFFFF;

/// A model answering SWSI, with the CMAC enabled and the scoreboard's
/// firmware byte as milestone 1 leaves it.
fn model() -> RegisterModel {
    let mut m = RegisterModel::new();
    m.emulate_rf();
    m.set32(0xC000, 0x4000_0000);
    m.set32(0x00AC, 0x8100_0000);
    m
}

/// One TRX mask LUT entry as four RF writes.
fn lut(path: u8, group: u32, val: u32) -> Vec<Op> {
    vec![
        Op::RfW(path, 0xEF, F, 0x20000),
        Op::RfW(path, 0x33, F, group),
        Op::RfW(path, 0x3F, F, val),
        Op::RfW(path, 0xEF, F, 0),
    ]
}

/// The writes up to the first TRX mask entry (B1-B10).
fn head() -> Vec<Op> {
    vec![
        Op::W32(0xDA00, 0x8001_0000),
        Op::W32(0xDA10, 1),
        Op::W16(0xC340, 0x20),
        Op::W16(0xC340, 0x20),
        Op::W32(0xDA00, 0x8001_0000),
        Op::W32(0xDA04, 5),
        Op::W32(0xDA24, 0x400),
        Op::W32(0xDA24, 0x0040_0400),
        Op::RfW(0, 0x02, F, 0),
        Op::RfW(1, 0x02, F, 0),
    ]
}

/// B23, B24, R1, R3, N1-N14 and the scoreboard write (W1).
fn tail() -> Vec<Op> {
    vec![
        Op::W32(0xDA44, 0xF0FF_FFFF),
        Op::W32(0xDA10, 3),
        Op::W32(0xDA48, 0x0005_7DDF),
        Op::W16(0xC67C, 0x0111),
        Op::W32(0x14C00, 0),
        Op::W32(0x14CC4, 0),
        Op::W32(0x14AD4, 0x60),
        Op::W32(0x14AE0, 0x60),
        Op::W32(0x14688, 0x1A00_0000),
        Op::W32(0x1476C, 0x1A00_0000),
        Op::W32(0x14694, 0x2A),
        Op::W32(0x14694, 0x2A2A),
        Op::W32(0x14778, 0x2A),
        Op::W32(0x14778, 0x2A2A),
        Op::W32(0x14AE4, 0x79_E99E),
        Op::W32(0x14AEC, 0x79_E99E),
        Op::W32(0x1469C, 0x9800_0000),
        Op::W32(0x149F0, 0x9800_0000),
        Op::W32(0x00AC, 0x8100_4003),
        Op::Delay(1000),
    ]
}

#[test]
fn coex_init_shared_matches_linux_sequence() {
    let mut m = model();
    init(&mut m, 1).unwrap();
    let mut want = head();
    want.extend(lut(0, 0, 0x5FF));
    want.extend(lut(1, 0, 0x5FF));
    want.extend(lut(0, 2, 0x5FF));
    want.extend(tail());
    assert_eq!(m.rf_ops(), want);
}

#[test]
fn rfe_zero_and_odd_are_shared_even_is_dedicated() {
    for (rfe, shared) in [(0, true), (1, true), (2, false), (3, true), (4, false)] {
        let mut m = model();
        init(&mut m, rfe).unwrap();
        let luts = m
            .rf_ops()
            .iter()
            .filter(|op| matches!(op, Op::RfW(_, 0x33, _, _)))
            .count();
        assert_eq!(luts, if shared { 3 } else { 2 }, "rfe {rfe}");
    }
}

#[test]
fn coex_init_dedicated_writes_two_5df_masks() {
    let mut m = model();
    init(&mut m, 2).unwrap();
    let mut want = head();
    want.extend(lut(0, 0, 0x5DF));
    want.extend(lut(1, 0, 0x5DF));
    want.extend(tail());
    assert_eq!(m.rf_ops(), want);
}

#[test]
fn coex_init_refuses_cmac_off() {
    let mut m = model();
    m.set32(0xC000, 0);
    assert_eq!(
        init(&mut m, 1),
        Err(AxError::Mac("CMAC function is not enabled"))
    );
    assert!(m.trace.iter().any(|op| matches!(op, Op::W32(0xDA48, _))));
    assert!(!m.trace.iter().any(|op| matches!(op, Op::W16(0xC67C, _))));
    // Nothing of the BB or the scoreboard ran after it.
    assert!(!m.trace.iter().any(|op| matches!(op, Op::W32(0x14C00, _))));
    assert!(!m.trace.iter().any(|op| matches!(op, Op::W32(0x00AC, _))));
}

#[test]
fn coex_init_keeps_bits_outside_its_fields() {
    let mut m = model();
    m.set32(0xDA00, 0x0300_FFFF);
    m.set32(0xDA04, 0xAABB_CC00);
    m.set32(0x14AD4, 0xFFFF_FFFF);
    m.set32(0x1469C, 0x03FF_FFFF);
    init(&mut m, 1).unwrap();
    // BTC_EN | LNA1 set, MODE [25:24] cleared, the rest kept.
    assert_eq!(m.get32(0xDA00), 0x8001_FFFF);
    assert_eq!(m.get32(0xDA04), 0xAABB_CC05);
    assert_eq!(m.get32(0x14AD4), 0x60);
    assert_eq!(m.get32(0x1469C), 0x9BFF_FFFF);
}

#[test]
fn scoreboard_keeps_firmware_bits() {
    let mut m = model();
    m.set32(0x00AC, 0x8500_0000);
    scoreboard(&mut m, SCBD_WL_ON).unwrap();
    assert_eq!(m.writes(), [Op::W32(0x00AC, 0x8500_4003), Op::Delay(1000)]);
}

#[test]
fn scoreboard_clears_btgs1_notify_and_sets_tp_major() {
    let mut m = model();
    m.set32(0x00AC, 0xFE00_FFFF);
    scoreboard(&mut m, SCBD_WL_ON).unwrap();
    // fw = (0x7E & !1) | 0x81 = 0xFF; the driver's bits are replaced.
    assert_eq!(m.get32(0x00AC), 0xFF00_4003);
}

#[test]
fn rfk_notify_start_and_stop() {
    let mut m = model();
    rfk_notify(&mut m, true).unwrap();
    assert_eq!(
        m.trace,
        [
            Op::W32(0xDA48, 0x5_7DDF),
            Op::R32(0xC000),
            Op::W16(0xC67C, 0x111),
            Op::R32(0x00AC),
            Op::W32(0x00AC, 0x8100_4803),
            Op::Delay(1000),
        ]
    );
    let mut m = model();
    rfk_notify(&mut m, false).unwrap();
    assert_eq!(
        m.trace,
        [
            Op::R32(0x00AC),
            Op::W32(0x00AC, 0x8100_4003),
            Op::Delay(1000),
        ]
    );
}

#[test]
fn rfk_start_refuses_cmac_off_before_the_scoreboard() {
    let mut m = model();
    m.set32(0xC000, 0);
    assert_eq!(
        rfk_notify(&mut m, true),
        Err(AxError::Mac("CMAC function is not enabled"))
    );
    assert!(!m.trace.iter().any(|op| matches!(op, Op::W32(0x00AC, _))));
}

#[test]
fn gnt_reads_the_grant_value_register() {
    let mut m = model();
    m.set32(0xDA4C, 0x0000_0006);
    assert_eq!(gnt(&mut m), Ok(6));
    assert_eq!(m.trace, [Op::R32(0xDA4C)]);
}

#[test]
fn grant_constant_is_the_gnt_v1_bit_arithmetic() {
    // WL VAL: RFC S0 b3, S1 b7, TX b11, RX b13, BB b1.
    let wl_val = 1 << 3 | 1 << 7 | 1 << 11 | 1 << 13 | 1 << 1;
    // WL SWCTRL: RFC S0 b2, S1 b6, TX b10, RX b12, BB b0.
    let wl_sw = 1 << 2 | 1 << 6 | 1 << 10 | 1 << 12 | 1;
    // BT SWCTRL (VAL 0): RFC S0 b4, S1 b8, TX b14, RX b16, WL_ACT b18.
    let bt_sw = 1 << 4 | 1 << 8 | 1 << 14 | 1 << 16 | 1 << 18;
    assert_eq!(wl_val | wl_sw | bt_sw, GNT_WL_ONLY);
    assert_eq!(PLT_GNT_WL, 1 | 1 << 4 | 1 << 8);
    assert_eq!(SCBD_WL_ON, 1 | 1 << 1 | 1 << 14);
    assert_eq!(SCBD_WLRFK, 0x800);
}

#[test]
fn radio_off_hands_the_antenna_to_bt_then_clears_the_driver_scoreboard() {
    let mut m = model();
    init(&mut m, 1).unwrap();
    m.trace.clear();
    radio_off(&mut m).unwrap();
    assert_eq!(
        m.trace,
        [
            Op::W32(0xDA48, 0x0007_D775),
            Op::R32(0xC000),
            Op::W16(0xC67C, 0x0111),
            Op::R32(0x00AC),
            // Toggle, the firmware's TP_MAJOR byte kept, every driver bit
            // (ACTIVE, ON, BTLOG) cleared.
            Op::W32(0x00AC, 0x8100_0000),
            Op::Delay(1000),
        ]
    );
}

#[test]
fn radio_off_still_clears_the_scoreboard_when_the_cmac_reads_off() {
    let mut m = model();
    m.set32(0x00AC, 0x8100_4003);
    m.set32(0xC000, 0);
    assert_eq!(
        radio_off(&mut m),
        Err(AxError::Mac("CMAC function is not enabled"))
    );
    assert!(!m.trace.iter().any(|o| matches!(o, Op::W16(0xC67C, _))));
    assert_eq!(m.get32(0x00AC), 0x8100_0000);
}

#[test]
fn wl_off_grant_is_the_gnt_v1_bit_arithmetic() {
    // BT VAL: RFC S0 b5, S1 b9, TX b15, RX b17.
    let bt_val = 1 << 5 | 1 << 9 | 1 << 15 | 1 << 17;
    // BT SWCTRL: RFC S0 b4, S1 b8, TX b14, RX b16, and WL_ACT_SWCTRL b18.
    let bt_sw = 1 << 4 | 1 << 8 | 1 << 14 | 1 << 16 | 1 << 18;
    // WL SWCTRL with VAL 0: BB b0, RFC S0 b2, S1 b6, TX b10, RX b12.
    let wl_sw = 1 | 1 << 2 | 1 << 6 | 1 << 10 | 1 << 12;
    assert_eq!(GNT_WL_OFF, bt_val | bt_sw | wl_sw);
}

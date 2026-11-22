// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
use super::*;
use crate::wifi::realtek::ax::chan::{Bw, Subband};
use crate::wifi::realtek::ax::regd::Regd;
use crate::wifi::realtek::ax::tests::model::{Op, READ_FAILED, RegisterModel};
use alloc::{boxed::Box, vec, vec::Vec};

/// Zeroed TX power tables at the world regd.
fn power() -> TxPower<'static> {
    TxPower {
        tables: Box::leak(Box::new(TxPowerTables::default())),
        regds: Regds {
            g2: Regd::Ww,
            g5: Regd::Ww,
        },
    }
}

/// RF 0x18 as M2b's bring-up is expected to leave it (2G ch 1, 20 MHz
/// bits clear), on both paths, behind SWSI and in the direct window.
const RF18_RESET: u32 = 0x01001;
/// MMIO of RF 0x18 in the direct window: BB `rf_base_addr[path] + 0x18 * 4`
/// (0xE060, 0xF060) plus `CR_BASE`.
const DIRECT_18: [u32; 2] = [0x1E060, 0x1F060];
/// `enum rtw89_cv` (core.h:201-207): `CHIP_CAV` and `CHIP_CBV` (this chip).
const CAV: u8 = 0;
const CBV: u8 = 1;

/// CMAC enabled and RF 0x18 at [`RF18_RESET`] everywhere.
fn model() -> RegisterModel {
    let mut m = RegisterModel::new();
    m.emulate_rf();
    m.set32(0xC000, 0x4000_0000);
    for p in 0..2 {
        m.set_rf(p, 0x18, RF18_RESET);
        m.set32(DIRECT_18[usize::from(p)], RF18_RESET);
    }
    m
}

fn g2(ch: u8) -> Chan {
    Chan::new20(HwBand::G2, ch).unwrap()
}

fn g5(ch: u8) -> Chan {
    Chan::new20(HwBand::G5, ch).unwrap()
}

/// A read-modify-write of `reg` that leaves `v`.
fn rmw(reg: u32, v: u32) -> [Op; 2] {
    [Op::R32(reg), Op::W32(reg, v)]
}

#[test]
fn help_enter_matches_linux_sequence() {
    let mut m = model();
    m.set32(0xC398, 0x3FFFF);
    // As `rtw89_mac_cfg_ppdu_status_bands(true)` and dm init leave them.
    m.set32(0xCE40, 0x2B);
    m.set32(0x10000, 0x8000_0000);
    assert_eq!(help_enter(&mut m, &g2(1)), Ok(0x3FFFF));

    let mut want = vec![
        // rtw89_mac_stop_sch_tx_v1 (mac.c:3444-3495): tx_en, then
        // rtw89_set_hw_sch_tx_en_v1 (mac.c:3385-3401).
        Op::R32(0xC398),
        Op::R32(0xC000),
        Op::R32(0xC398),
        Op::W32(0xC398, 0),
        // cfg_ppdu_status(false).
        Op::R32(0xC000),
        Op::R32(0xCE40),
        Op::W32(0xCE40, 0x2A),
    ];
    // dfs_en(false), tssi_cont_en_phyidx(false), adc_en(false).
    want.extend(rmw(0x10000, 0));
    want.extend(rmw(0x15818, 0x4000_0000));
    want.extend(rmw(0x15820, 0x8000_0000));
    want.extend(rmw(0x17818, 0x4000_0000));
    want.extend(rmw(0x17820, 0x8000_0000));
    want.extend(rmw(0x120FC, 0x0F00_0000));
    want.push(Op::Delay(40));
    // bb_reset_en(false) (rtw8852c.c:1686-1695).
    want.extend(rmw(0x12320, 1));
    want.extend(rmw(0x10C3C, 0x200));
    want.extend(rmw(0x11200, 0x7000_0000));
    want.extend(rmw(0x13200, 0x7000_0000));
    want.push(Op::Delay(1));
    want.extend(rmw(0x10704, 0));
    assert_eq!(m.trace, want);
}

#[test]
fn help_enter_writes_from_zero_in_brief_order() {
    let mut m = model();
    m.set32(0xC398, 0x3FFFF);
    help_enter(&mut m, &g2(1)).unwrap();
    assert_eq!(
        m.writes(),
        [
            Op::W32(0xC398, 0),
            Op::W32(0xCE40, 0),
            Op::W32(0x10000, 0),
            Op::W32(0x15818, 0x4000_0000),
            Op::W32(0x15820, 0x8000_0000),
            Op::W32(0x17818, 0x4000_0000),
            Op::W32(0x17820, 0x8000_0000),
            Op::W32(0x120FC, 0x0F00_0000),
            Op::Delay(40),
            Op::W32(0x12320, 1),
            Op::W32(0x10C3C, 0x200),
            Op::W32(0x11200, 0x7000_0000),
            Op::W32(0x13200, 0x7000_0000),
            Op::Delay(1),
            Op::W32(0x10704, 0),
        ]
    );
}

#[test]
fn help_enter_returns_the_whole_register_and_keeps_its_high_bits() {
    let mut m = model();
    m.set32(0xC398, 0xA005_5555);
    assert_eq!(help_enter(&mut m, &g5(36)), Ok(0xA005_5555));
    // Only TXEN_ALL [17:0] is cleared.
    assert_eq!(m.get32(0xC398), 0xA004_0000);
    // The 5 GHz enter disables CCK RX CCA too: Linux's disable is band-blind.
    assert!(m.trace.contains(&Op::W32(0x12320, 1)));
}

#[test]
fn help_enter_refuses_cmac_off() {
    let mut m = model();
    m.set32(0xC000, 0);
    m.set32(0xC398, 0x3FFFF);
    assert_eq!(
        help_enter(&mut m, &g2(1)),
        Err(AxError::Mac("CMAC function is not enabled"))
    );
    assert!(m.writes().is_empty());
    assert_eq!(m.trace, [Op::R32(0xC398), Op::R32(0xC000)]);
}

/// The registers as `help_enter` leaves them, from the brief's zero start.
fn entered() -> RegisterModel {
    let mut m = model();
    m.set32(0xC398, 0x3FFFF);
    m.set32(0xCE40, 0x2B);
    m.set32(0x10000, 0x8000_0000);
    help_enter(&mut m, &g2(1)).unwrap();
    m.trace.clear();
    m
}

/// `help_exit`'s trace up to `bb_reset_en(true)`'s RSTB write, which both
/// bands share.
fn exit_common() -> Vec<Op> {
    let mut want = vec![
        // cfg_ppdu_status(true).
        Op::R32(0xC000),
        Op::W32(0xCE40, 0x2B),
        Op::R32(0x9C18),
        Op::W32(0x9C18, 1),
    ];
    // adc_en(true), dfs_en(true), tssi_cont_en_phyidx(true) without the DE.
    want.extend(rmw(0x120FC, 0));
    want.extend(rmw(0x10000, 0x8000_0000));
    want.extend(rmw(0x15818, 0));
    want.extend(rmw(0x15820, 0));
    want.extend(rmw(0x17818, 0));
    want.extend(rmw(0x17820, 0));
    // bb_reset_en(true) (rtw8852c.c:1677-1685).
    want.extend(rmw(0x11200, 0));
    want.extend(rmw(0x13200, 0));
    want.extend(rmw(0x10704, 2));
    want
}

/// rtw89_mac_resume_sch_tx_v1 (mac.c:3497-3507).
const RESUME: [Op; 3] = [Op::R32(0xC000), Op::R32(0xC398), Op::W32(0xC398, 0x3FFFF)];

#[test]
fn help_exit_matches_linux_sequence() {
    let mut m = entered();
    assert_eq!(help_exit(&mut m, &g2(1), 0x3FFFF), Ok(()));
    let mut want = exit_common();
    want.extend(rmw(0x12320, 0));
    want.extend(rmw(0x10C3C, 0));
    want.extend(RESUME);
    assert_eq!(m.trace, want);

    // 5 GHz leaves CCK RX CCA disabled, as Linux's 2G-only re-enable does.
    let mut m = entered();
    assert_eq!(help_exit(&mut m, &g5(36), 0x3FFFF), Ok(()));
    let mut want = exit_common();
    want.extend(rmw(0x10C3C, 0));
    want.extend(RESUME);
    assert_eq!(m.trace, want);
    assert_eq!(m.get32(0x12320), 1);
}

#[test]
fn help_exit_resumes_only_the_txen_field() {
    let mut m = entered();
    m.set32(0xC398, 0xA004_0000);
    // tx_en's [17:0] is 0x3_5555; 0xC398's bit 18 and above stay.
    help_exit(&mut m, &g2(1), 0xFFFF_5555).unwrap();
    assert_eq!(m.get32(0xC398), 0xA007_5555);
}

#[test]
fn help_enter_then_exit_restores_the_2g_state() {
    let mut m = model();
    m.set32(0xC398, 0x3FFFF);
    m.set32(0xCE40, 0x2B);
    m.set32(0x10000, 0x8000_0000);
    m.set32(0x10704, 2);
    let before: Vec<u32> = [
        0xC398, 0xCE40, 0x10000, 0x15818, 0x15820, 0x17818, 0x17820, 0x120FC, 0x12320, 0x10C3C,
        0x11200, 0x13200, 0x10704,
    ]
    .iter()
    .map(|&r| m.get32(r))
    .collect();
    let tx_en = help_enter(&mut m, &g2(1)).unwrap();
    help_exit(&mut m, &g2(1), tx_en).unwrap();
    let after: Vec<u32> = [
        0xC398, 0xCE40, 0x10000, 0x15818, 0x15820, 0x17818, 0x17820, 0x120FC, 0x12320, 0x10C3C,
        0x11200, 0x13200, 0x10704,
    ]
    .iter()
    .map(|&r| m.get32(r))
    .collect();
    assert_eq!(before, after);
}

#[test]
fn help_exit_refuses_cmac_off() {
    let mut m = entered();
    m.set32(0xC000, 0);
    assert_eq!(
        help_exit(&mut m, &g2(1), 0x3FFFF),
        Err(AxError::Mac("CMAC function is not enabled"))
    );
    assert!(m.writes().is_empty());
}

#[test]
fn set_channel_mac_by_band() {
    let mut m = model();
    assert_eq!(set_channel_mac(&mut m, &g2(1)), Ok(()));
    assert_eq!(
        m.trace,
        [
            Op::R8(0xC010),
            Op::W8(0xC010, 0),
            Op::W32(0xC088, 0),
            Op::R8(0xC628),
            Op::W8(0xC628, 0),
            Op::R8(0xC628),
            Op::W8(0xC628, 0x10),
        ]
    );

    let mut m = model();
    assert_eq!(set_channel_mac(&mut m, &g5(36)), Ok(()));
    assert_eq!(
        m.trace,
        [
            Op::R8(0xC010),
            Op::W8(0xC010, 0),
            Op::W32(0xC088, 0),
            Op::R8(0xC628),
            Op::W8(0xC628, 0),
            Op::R8(0xC628),
            Op::W8(0xC628, 0x03),
        ]
    );
}

#[test]
fn set_channel_mac_keeps_unrelated_bits() {
    // RFMOD [1:0] and the three rate-check bits are replaced; the rest stays.
    let mut m = model();
    m.set32(0xC010, 0xFF);
    m.set32(0xC088, 0x0ABC);
    m.set32(0xC628, 0xFF);
    set_channel_mac(&mut m, &g5(149)).unwrap();
    assert_eq!(m.get32(0xC010), 0xFC);
    assert_eq!(m.get32(0xC088), 0);
    assert_eq!(m.trace[4], Op::W8(0xC628, 0xEC));
    assert_eq!(m.get32(0xC628), 0xEF);

    let mut m = model();
    m.set32(0xC628, 0xFF);
    set_channel_mac(&mut m, &g2(6)).unwrap();
    assert_eq!(m.get32(0xC628), 0xFC);
}

/// `_ch_setting(path, DAV)` then `(path, DDV)` at the RF level
/// (rtw8852c_rfk.c:3899-3933): the SWSI read with its 2 us settle, the SWSI
/// write, 100 us; then the direct window's write with its 1 us settle
/// (its two reads are not RF ops), 100 us.
fn ch_ops(p: u8, v: u32) -> Vec<Op> {
    vec![
        Op::RfR(p, 0x18),
        Op::Delay(2),
        Op::RfW(p, 0x18, RFREG_MASK, v),
        Op::Delay(100),
        Op::W32(DIRECT_18[usize::from(p)], v),
        Op::Delay(1),
        Op::Delay(100),
    ]
}

/// `_bw_setting(path, 20 MHz, DAV)` then `(path, DDV)` (:3822-3865): the
/// read, CFCH_BW0 [27:26] = 3 and CFCH_BW1 [8:5] = 0xF on the path's BB
/// registers, the write.
fn bw_ops(p: u8, v: u32) -> Vec<Op> {
    let bw0 = 0x1C0D4 | (u32::from(p) << 8);
    let bw1 = 0x1C0D8 | (u32::from(p) << 8);
    vec![
        Op::RfR(p, 0x18),
        Op::Delay(2),
        Op::W32(bw0, 0x0C00_0000),
        Op::W32(bw1, 0x1E0),
        Op::RfW(p, 0x18, RFREG_MASK, v),
        Op::W32(bw0, 0x0C00_0000),
        Op::W32(bw1, 0x1E0),
        Op::W32(DIRECT_18[usize::from(p)], v),
        Op::Delay(1),
    ]
}

/// `_set_rxbb_bw(path, 20 MHz)` and `_set_tia_bw` (:3961-3996).
fn rxbb_ops(p: u8) -> Vec<Op> {
    vec![
        Op::RfW(p, 0xEE, 0x4, 0x4),
        Op::RfW(p, 0x33, 0x1F, 0xA),
        Op::RfW(p, 0x3F, 0x3F, 0x1B),
        Op::RfW(p, 0xEE, 0x4, 0),
        Op::RfW(p, 0x8F, 0x60, 0x40),
    ]
}

fn rf_want(ch_val: u32, bw_val: u32) -> Vec<Op> {
    let mut want = Vec::new();
    for p in 0..2 {
        want.extend(ch_ops(p, ch_val));
    }
    for p in 0..2 {
        want.extend(bw_ops(p, bw_val));
    }
    for p in 0..2 {
        want.extend(rxbb_ops(p));
    }
    want
}

fn assert_rf18_everywhere(m: &RegisterModel, v: u32) {
    for p in 0..2 {
        assert_eq!(m.get_rf(p, 0x18), v, "SWSI RF 0x18 path {p}");
        assert_eq!(
            m.get32(DIRECT_18[usize::from(p)]),
            v,
            "direct RF 0x18 path {p}"
        );
    }
}

#[test]
fn set_channel_rf_2g_ch1_matches_linux_sequence() {
    let mut m = model();
    assert_eq!(set_channel_rf(&mut m, &g2(1), CBV), Ok(()));
    assert_eq!(m.rf_ops(), rf_want(0x01001, 0x01C01));
    assert_rf18_everywhere(&m, 0x01C01);
    // Each direct-window access reads first: two in `_ch_setting`, two in
    // `_bw_setting` (the read, then the write's read-modify-write).
    for reg in DIRECT_18 {
        assert_eq!(m.trace.iter().filter(|op| **op == Op::R32(reg)).count(), 4);
    }
}

#[test]
fn set_channel_rf_5g_ch36_sets_band_bits() {
    let mut m = model();
    assert_eq!(set_channel_rf(&mut m, &g5(36), CBV), Ok(()));
    // BAND1 [17:16] = 1, BAND0 [9:8] = 1, CH [7:0] = 36, then BW [11:10] = 3.
    assert_eq!(m.rf_ops(), rf_want(0x11124, 0x11D24));
    assert_rf18_everywhere(&m, 0x11D24);
}

#[test]
fn set_channel_rf_reads_each_window_on_its_own() {
    // The DAV (SWSI) and DDV (direct) registers are read and rewritten
    // separately, so a path whose two windows disagree keeps each one's own
    // untouched bits [19:18] and [15:12]. A channel field of 0 against 1 (the
    // ambiguous M2b path B readback) does not survive: CH [7:0] is replaced.
    let mut m = model();
    m.set_rf(1, 0x18, 0x01000);
    m.set32(DIRECT_18[1], 0xFFF0_0000 | 0x8_3001);
    set_channel_rf(&mut m, &g2(11), CBV).unwrap();
    assert_eq!(m.get_rf(0, 0x18), 0x01C0B);
    assert_eq!(m.get_rf(1, 0x18), 0x01C0B);
    assert_eq!(m.get32(DIRECT_18[0]), 0x01C0B);
    // The direct window keeps its bits above RFREG_MASK too.
    assert_eq!(m.get32(DIRECT_18[1]), 0xFFF8_3C0B);
}

#[test]
fn set_channel_rf_moves_5g_back_to_2g() {
    let mut m = model();
    set_channel_rf(&mut m, &g5(165), CBV).unwrap();
    assert_rf18_everywhere(&m, 0x11DA5);
    set_channel_rf(&mut m, &g2(1), CBV).unwrap();
    assert_rf18_everywhere(&m, 0x01C01);
}

#[test]
fn set_channel_rf_skips_the_channel_outside_ctrl_ch_ranges() {
    // `_ctrl_ch` returns for a channel it does not program
    // (rtw8852c_rfk.c:3941-3944); `_ctrl_bw` and `_rxbb_bw` still run.
    let mut m = model();
    let chan = Chan {
        channel: 20,
        band: HwBand::G2,
        freq: 0,
        subband: Subband::G2,
        primary: 20,
        width: Bw::Mhz20,
        pri_ch_idx: 0,
        txsc: [0; 3],
    };
    set_channel_rf(&mut m, &chan, CBV).unwrap();
    let mut want = Vec::new();
    for p in 0..2 {
        want.extend(bw_ops(p, 0x01C01));
    }
    for p in 0..2 {
        want.extend(rxbb_ops(p));
    }
    assert_eq!(m.rf_ops(), want);
}

#[test]
fn set_channel_rf_refuses_6g_untouched() {
    let mut m = model();
    let chan = Chan {
        channel: 1,
        band: HwBand::G6,
        freq: 5955,
        subband: Subband::G2,
        primary: 1,
        width: Bw::Mhz20,
        pri_ch_idx: 0,
        txsc: [0; 3],
    };
    assert!(matches!(
        set_channel_rf(&mut m, &chan, CBV),
        Err(AxError::Phy(_))
    ));
    assert!(m.trace.is_empty());
}

#[test]
fn set_channel_rf_cav_copies_path_a_into_path_b() {
    // Path B's SWSI RF 0x18 has bit 13 set, so its own result differs from
    // path A's; on CAV `_ctrl_bw` then overwrites it with path A's
    // (rtw8852c_rfk.c:3888-3895).
    let mut m = model();
    m.set_rf(1, 0x18, 0x03001);
    m.set_rf(1, 0x05, 0xFFFFF);
    m.set_rf(1, 0x19, 0);
    assert_eq!(set_channel_rf(&mut m, &g2(1), CAV), Ok(()));

    let mut want = ch_ops(0, 0x01001);
    // Path B: SWSI from 0x03001, the direct window from 0x01001.
    want.extend([
        Op::RfR(1, 0x18),
        Op::Delay(2),
        Op::RfW(1, 0x18, RFREG_MASK, 0x03001),
        Op::Delay(100),
        Op::W32(DIRECT_18[1], 0x01001),
        Op::Delay(1),
        Op::Delay(100),
    ]);
    want.extend(bw_ops(0, 0x01C01));
    let mut b_bw = bw_ops(1, 0x01C01);
    b_bw[4] = Op::RfW(1, 0x18, RFREG_MASK, 0x03C01);
    want.extend(b_bw);
    want.extend([
        // RR_RSV1 RR_RSV1_RST = 0.
        Op::RfW(1, 0x05, 0x1, 0),
        // tmp = path A's RF 0x18 over SWSI.
        Op::RfR(0, 0x18),
        Op::Delay(2),
        // RR_APK RR_APK_MOD [5:4] = 3.
        Op::RfW(1, 0x19, 0x30, 0x30),
        // Path B's RF 0x18 over SWSI only = tmp.
        Op::RfW(1, 0x18, RFREG_MASK, 0x01C01),
        Op::Delay(100),
        // RR_RSV1 RR_RSV1_RST = 1.
        Op::RfW(1, 0x05, 0x1, 1),
    ]);
    for p in 0..2 {
        want.extend(rxbb_ops(p));
    }
    assert_eq!(m.rf_ops(), want);

    assert_eq!(m.get_rf(1, 0x18), 0x01C01);
    assert_eq!(m.get_rf(1, 0x05), 0xFFFFF);
    assert_eq!(m.get_rf(1, 0x19), 0x30);
    // The direct window is not part of the copy.
    assert_eq!(m.get32(DIRECT_18[1]), 0x01C01);
    // Path A is only read.
    assert!(
        !m.rf_ops()
            .iter()
            .any(|op| matches!(op, Op::RfW(0, 0x05 | 0x19, ..)))
    );
}

#[test]
fn set_channel_rf_later_cuts_skip_the_cav_copy() {
    for cut in [CBV, 2, 3, 4] {
        let mut m = model();
        m.set_rf(1, 0x18, 0x03001);
        set_channel_rf(&mut m, &g2(1), cut).unwrap();
        // Path B keeps its own bit 13; no RF 0x05 or 0x19 access at all.
        assert_eq!(m.get_rf(1, 0x18), 0x03C01, "cut {cut}");
        assert!(
            !m.rf_ops()
                .iter()
                .any(|op| matches!(op, Op::RfW(_, 0x05 | 0x19, ..) | Op::RfR(_, 0x05 | 0x19))),
            "cut {cut}"
        );
        // Path A's RF 0x18 is read exactly twice over SWSI (ch, bw).
        let a_reads = m
            .rf_ops()
            .iter()
            .filter(|op| **op == Op::RfR(0, 0x18))
            .count();
        assert_eq!(a_reads, 2, "cut {cut}");
    }
    // And cut 1 still gives the brief's exact sequence.
    let mut m = model();
    set_channel_rf(&mut m, &g2(1), CBV).unwrap();
    assert_eq!(m.rf_ops(), rf_want(0x01001, 0x01C01));
}

// ---- set_channel_bb (rtw8852c.c:1834-1930) and the whole set_channel ----

/// One step of an expected sequence, by MMIO address: a
/// `rtw89_phy_write32_mask`/`rtw89_write32_mask` of `(reg, mask, value)`
/// (an RMW even for a full mask), a plain write, or a delay.
#[derive(Clone, Copy)]
enum S {
    F(u32, u32, u32),
    W(u32, u32),
    D(u64),
}

/// The trace `steps` give from all-zero registers.
fn from_zero(steps: &[S]) -> Vec<Op> {
    let mut regs = alloc::collections::BTreeMap::<u32, u32>::new();
    let mut ops = Vec::new();
    for &step in steps {
        match step {
            S::F(reg, mask, v) => {
                let old = regs.get(&reg).copied().unwrap_or(0);
                let new = (old & !mask) | ((v << mask.trailing_zeros()) & mask);
                regs.insert(reg, new);
                ops.extend(rmw(reg, new));
            }
            S::W(reg, v) => {
                regs.insert(reg, v);
                ops.push(Op::W32(reg, v));
            }
            S::D(us) => ops.push(Op::Delay(us)),
        }
    }
    ops
}

/// An s8 as `rtw89_phy_write32_mask` takes it: sign-extended to u32.
fn s8(v: i8) -> u32 {
    i32::from(v) as u32
}

/// `bb_gain_lna` (rtw8852c.c:932-947): the 2G and 5G register per path and
/// the mask, i = 0..6, as MMIO.
const LNA: [([u32; 2], [u32; 2], u32); 7] = [
    ([0x14678, 0x1475C], [0x145DC, 0x14740], 0x00FF_0000),
    ([0x14678, 0x1475C], [0x145DC, 0x14740], 0xFF00_0000),
    ([0x1467C, 0x14760], [0x14660, 0x14744], 0x0000_00FF),
    ([0x1467C, 0x14760], [0x14660, 0x14744], 0x0000_FF00),
    ([0x1467C, 0x14760], [0x14660, 0x14744], 0x00FF_0000),
    ([0x1467C, 0x14760], [0x14660, 0x14744], 0xFF00_0000),
    ([0x14680, 0x14764], [0x14664, 0x14748], 0x0000_00FF),
];
/// `bb_gain_tia` (:949-954).
const TIA: [([u32; 2], [u32; 2], u32); 2] = [
    ([0x14680, 0x14764], [0x14664, 0x14748], 0x00FF_0000),
    ([0x14680, 0x14764], [0x14664, 0x14748], 0xFF00_0000),
];
/// `bb_gain_bypass_lna` (:963-979): 2G registers and mask, 5G registers
/// and mask.
const BYPASS: [([u32; 2], u32, [u32; 2], u32); 7] = [
    (
        [0x14BB8, 0x14C7C],
        0xFF00_0000,
        [0x14BB4, 0x14C78],
        0x0000_00FF,
    ),
    (
        [0x14BBC, 0x14C80],
        0x0000_00FF,
        [0x14BB4, 0x14C78],
        0x0000_FF00,
    ),
    (
        [0x14BBC, 0x14C80],
        0x0000_FF00,
        [0x14BB4, 0x14C78],
        0x00FF_0000,
    ),
    (
        [0x14BBC, 0x14C80],
        0x00FF_0000,
        [0x14BB4, 0x14C78],
        0xFF00_0000,
    ),
    (
        [0x14BBC, 0x14C80],
        0xFF00_0000,
        [0x14BB8, 0x14C7C],
        0x0000_00FF,
    ),
    (
        [0x14BC0, 0x14C84],
        0x0000_00FF,
        [0x14BB8, 0x14C7C],
        0x0000_FF00,
    ),
    (
        [0x14BC0, 0x14C84],
        0x0000_FF00,
        [0x14BB8, 0x14C7C],
        0x00FF_0000,
    ),
];
/// `bb_gain_op1db_a` (:991-1010): the lna and tia_lna registers per path
/// and the mask.
const OP1DB: [([u32; 2], [u32; 2], u32); 7] = [
    ([0x14668, 0x1474C], [0x14670, 0x14754], 0x0000_00FF),
    ([0x14668, 0x1474C], [0x14670, 0x14754], 0x0000_FF00),
    ([0x14668, 0x1474C], [0x14670, 0x14754], 0x00FF_0000),
    ([0x14668, 0x1474C], [0x14670, 0x14754], 0xFF00_0000),
    ([0x1466C, 0x14750], [0x14674, 0x14758], 0x0000_00FF),
    ([0x1466C, 0x14750], [0x14674, 0x14758], 0x0000_FF00),
    ([0x1466C, 0x14750], [0x14674, 0x14758], 0x00FF_0000),
];

/// `rtw8852c_set_gain_error`'s steps for `chan` and path `p`, read from
/// Linux's tables above.
fn gain_error_steps(g: &BbGain, chan: &Chan, p: usize) -> Vec<S> {
    let gb = chan.bb_gain_band();
    let is_2g = chan.band == HwBand::G2;
    let mut s = Vec::new();
    for i in 0..7 {
        let (lg, la, lm) = LNA[i];
        s.push(S::F(
            if is_2g { lg[p] } else { la[p] },
            lm,
            s8(g.lna_gain[gb][p][i]),
        ));
        let (bg, bmg, ba, bma) = BYPASS[i];
        let (reg, mask) = if is_2g { (bg[p], bmg) } else { (ba[p], bma) };
        s.push(S::F(reg, mask, s8(g.lna_gain_bypass[gb][p][i])));
        if !is_2g {
            let (lna, tia_lna, mask) = OP1DB[i];
            s.push(S::F(lna[p], mask, s8(g.lna_op1db[gb][p][i])));
            s.push(S::F(tia_lna[p], mask, s8(g.tia_lna_op1db[gb][p][i])));
        }
    }
    if !is_2g {
        s.push(S::F(
            [0x14674, 0x14758][p],
            0xFF00_0000,
            s8(g.tia_lna_op1db[gb][p][7]),
        ));
    }
    for (i, &(tg, ta, tm)) in TIA.iter().enumerate() {
        s.push(S::F(
            if is_2g { tg[p] } else { ta[p] },
            tm,
            s8(g.tia_gain[gb][p][i]),
        ));
    }
    s
}

fn rx_gain(valid: bool) -> RxGain {
    RxGain {
        valid,
        offset: [[0; 13]; 2],
    }
}

fn inputs<'a>(gain: &'a BbGain, rx: &'a RxGain, offset_base: i8) -> ChannelInputs<'a> {
    ChannelInputs {
        gain,
        rx_gain: rx,
        offset_base,
        cut: CBV,
    }
}

/// Index of the first op `want` matches.
fn first(m: &RegisterModel, want: impl Fn(&Op) -> bool) -> usize {
    m.trace.iter().position(want).expect("no matching op")
}

/// Index of the first write (any width) to `reg`.
fn first_w(m: &RegisterModel, reg: u32) -> usize {
    first(
        m,
        |op| matches!(*op, Op::W8(r, _) | Op::W16(r, _) | Op::W32(r, _) if r == reg),
    )
}

#[test]
fn gain_error_2g_writes_lna_bypass_then_tia() {
    let mut g = BbGain::default();
    g.lna_gain[0][0] = [1, 2, 3, 4, 5, 6, 7];
    g.lna_gain_bypass[0][0] = [11, 12, 13, 14, 15, 16, 17];
    g.tia_gain[0][0] = [21, 22];
    let mut m = RegisterModel::new();
    set_gain_error(&mut m, &g, &g2(1), RfPath::A).unwrap();
    let want = from_zero(&[
        S::F(0x14678, 0x00FF_0000, 1),
        S::F(0x14BB8, 0xFF00_0000, 11),
        S::F(0x14678, 0xFF00_0000, 2),
        S::F(0x14BBC, 0x0000_00FF, 12),
        S::F(0x1467C, 0x0000_00FF, 3),
        S::F(0x14BBC, 0x0000_FF00, 13),
        S::F(0x1467C, 0x0000_FF00, 4),
        S::F(0x14BBC, 0x00FF_0000, 14),
        S::F(0x1467C, 0x00FF_0000, 5),
        S::F(0x14BBC, 0xFF00_0000, 15),
        S::F(0x1467C, 0xFF00_0000, 6),
        S::F(0x14BC0, 0x0000_00FF, 16),
        S::F(0x14680, 0x0000_00FF, 7),
        S::F(0x14BC0, 0x0000_FF00, 17),
        S::F(0x14680, 0x00FF_0000, 21),
        S::F(0x14680, 0xFF00_0000, 22),
    ]);
    assert_eq!(want.len(), 32);
    assert_eq!(m.trace, want);
    // The same through the Linux tables.
    assert_eq!(m.trace, from_zero(&gain_error_steps(&g, &g2(1), 0)));
}

#[test]
fn gain_error_5g_adds_op1db() {
    let mut g = BbGain::default();
    g.lna_gain[1][1] = [1, 2, 3, 4, 5, 6, 7];
    g.lna_gain_bypass[1][1] = [11, 12, 13, 14, 15, 16, 17];
    g.lna_op1db[1][1] = [31, 32, 33, 34, 35, 36, 37];
    g.tia_lna_op1db[1][1] = [41, 42, 43, 44, 45, 46, 47, -90];
    g.tia_gain[1][1] = [21, 22];
    // Path A of the same band and path B of another band stay unread.
    g.lna_gain[1][0] = [99; 7];
    g.lna_gain[2][1] = [98; 7];
    let mut m = RegisterModel::new();
    set_gain_error(&mut m, &g, &g5(36), RfPath::B).unwrap();
    assert_eq!(m.writes().len(), 31);
    assert_eq!(m.trace[0], Op::R32(0x14740));
    assert_eq!(m.trace[1], Op::W32(0x14740, 1 << 16));
    let want = from_zero(&[
        S::F(0x14740, 0x00FF_0000, 1),
        S::F(0x14C78, 0x0000_00FF, 11),
        S::F(0x1474C, 0x0000_00FF, 31),
        S::F(0x14754, 0x0000_00FF, 41),
        S::F(0x14740, 0xFF00_0000, 2),
        S::F(0x14C78, 0x0000_FF00, 12),
        S::F(0x1474C, 0x0000_FF00, 32),
        S::F(0x14754, 0x0000_FF00, 42),
        S::F(0x14744, 0x0000_00FF, 3),
        S::F(0x14C78, 0x00FF_0000, 13),
        S::F(0x1474C, 0x00FF_0000, 33),
        S::F(0x14754, 0x00FF_0000, 43),
        S::F(0x14744, 0x0000_FF00, 4),
        S::F(0x14C78, 0xFF00_0000, 14),
        S::F(0x1474C, 0xFF00_0000, 34),
        S::F(0x14754, 0xFF00_0000, 44),
        S::F(0x14744, 0x00FF_0000, 5),
        S::F(0x14C7C, 0x0000_00FF, 15),
        S::F(0x14750, 0x0000_00FF, 35),
        S::F(0x14758, 0x0000_00FF, 45),
        S::F(0x14744, 0xFF00_0000, 6),
        S::F(0x14C7C, 0x0000_FF00, 16),
        S::F(0x14750, 0x0000_FF00, 36),
        S::F(0x14758, 0x0000_FF00, 46),
        S::F(0x14748, 0x0000_00FF, 7),
        S::F(0x14C7C, 0x00FF_0000, 17),
        S::F(0x14750, 0x00FF_0000, 37),
        S::F(0x14758, 0x00FF_0000, 47),
        // TIA1_LNA6 of tia_lna_op1db, -90.
        S::F(0x14758, 0xFF00_0000, s8(-90)),
        S::F(0x14748, 0x00FF_0000, 21),
        S::F(0x14748, 0xFF00_0000, 22),
    ]);
    assert_eq!(m.trace, want);
    assert_eq!(m.trace, from_zero(&gain_error_steps(&g, &g5(36), 1)));
    // A stored -90 lands as field 0xA6, the rest of the word kept.
    assert_eq!(m.get32(0x14758), 0xA62F_2E2D);
}

#[test]
fn gain_error_uses_the_subbands_gain_band() {
    let mut g = BbGain::default();
    for (band, v) in [(1usize, 1i8), (2, 2), (3, 3)] {
        g.lna_gain[band][0][0] = v;
    }
    for (ch, v) in [(36u8, 1u32), (100, 2), (149, 3), (165, 3)] {
        let mut m = RegisterModel::new();
        set_gain_error(&mut m, &g, &g5(ch), RfPath::A).unwrap();
        assert_eq!(m.get32(0x145DC), v << 16, "ch {ch}");
    }
}

#[test]
fn gain_offset_skipped_when_invalid() {
    let mut rx = rx_gain(false);
    rx.offset[0][1] = 5;
    let mut m = RegisterModel::new();
    set_gain_offset(&mut m, &rx, -16, &g2(1), RfPath::A).unwrap();
    assert!(m.trace.is_empty());
}

#[test]
fn gain_offset_2g_matches_linux_arithmetic() {
    let mut rx = rx_gain(true);
    rx.offset[0][0] = 2;
    rx.offset[0][1] = -3;
    let mut m = RegisterModel::new();
    set_gain_offset(&mut m, &rx, -16, &g2(1), RfPath::A).unwrap();
    // CCK: clamp(-2 * 8 + (-16 >> 1)) = -24 -> 0x68 in [14:8].
    // OFDM: q0 = 3, -(3 * 4 + (-16 >> 2)) = -8 -> 0xF8; 3 * 16 = 48 -> 0x30.
    assert_eq!(
        m.trace,
        from_zero(&[
            S::F(0x12340, 0x7F00, 0x68),
            S::F(0x14694, 0x00FF_0000, 0xF8),
            S::F(0x14E0C, 0x0000_FF00, 0x30),
            S::F(0x14E2C, 0x0000_00FF, 0x30),
        ])
    );
}

#[test]
fn gain_offset_path_b_5g_uses_its_own_fields_and_no_cck() {
    let mut rx = rx_gain(true);
    rx.offset[1][0] = 7;
    rx.offset[1][2] = 1;
    let mut m = RegisterModel::new();
    set_gain_offset(&mut m, &rx, 5, &g5(36), RfPath::B).unwrap();
    // q0 = -1: -(-4 + (5 >> 2)) = 3; -1 * 16 = -16 -> 0xF0.
    assert_eq!(
        m.trace,
        from_zero(&[
            S::F(0x14778, 0x00FF_0000, 3),
            S::F(0x14E0C, 0x00FF_0000, 0xF0),
            S::F(0x14E2C, 0x0000_FF00, 0xF0),
        ])
    );
}

#[test]
fn gain_offset_clamps() {
    let mut rx = rx_gain(true);
    rx.offset[1][4] = -8;
    let mut m = RegisterModel::new();
    set_gain_offset(&mut m, &rx, 0, &g5(149), RfPath::B).unwrap();
    // q0 = 8: -(32) = -32 -> 0xE0; 8 * 16 = 128 clamps to 127.
    assert_eq!(m.get32(0x14778), 0x00E0_0000);
    assert_eq!(m.get32(0x14E0C), 0x007F_0000);
    assert_eq!(m.get32(0x14E2C), 0x0000_7F00);

    // CCK clamps at -64 and 63, OFDM at -128 and 127.
    let mut rx = rx_gain(true);
    rx.offset[0][0] = 7;
    rx.offset[0][1] = 7;
    let mut m = RegisterModel::new();
    set_gain_offset(&mut m, &rx, -128, &g2(6), RfPath::A).unwrap();
    // CCK: -56 - 64 = -120 -> -64 = 0x40; OFDM: -(-28 - 32) = 60;
    // -7 * 16 = -112 -> 0x90.
    assert_eq!(m.get32(0x12340), 0x40 << 8);
    assert_eq!(m.get32(0x14694), 60 << 16);
    assert_eq!(m.get32(0x14E0C), 0x90 << 8);
    let mut rx = rx_gain(true);
    rx.offset[0][0] = -8;
    rx.offset[0][1] = -8;
    let mut m = RegisterModel::new();
    set_gain_offset(&mut m, &rx, 127, &g2(6), RfPath::A).unwrap();
    // CCK: 64 + 63 = 127 -> 63; OFDM: -(32 + 31) = -63 = 0xC1;
    // 8 * 16 = 128 -> 127.
    assert_eq!(m.get32(0x12340), 63 << 8);
    assert_eq!(m.get32(0x14694), 0xC1 << 16);
    assert_eq!(m.get32(0x14E0C), 0x7F << 8);
}

/// `rtw8852c_ctrl_bw(pri 0, 20 MHz)` with `bw_setting` and the EDCCA
/// per-20 bitmap (rtw8852c.c:1280-1386, 1241-1278).
const CTRL_BW_20: [S; 15] = [
    S::F(0x14974, 0xC000_0000, 0),
    S::F(0x14978, 0x3000, 0),
    S::F(0x14978, 0x0F00, 0),
    S::F(0x1C0D4, 0x0C00_0000, 3),
    S::F(0x1C1D4, 0x0C00_0000, 3),
    S::F(0x1C0D8, 0x01E0, 0xF),
    S::F(0x1C1D8, 0x01E0, 0xF),
    S::F(0x14E30, 1 << 26, 0),
    S::F(0x14424, 1 << 2, 0),
    S::F(0x1C0EC, 0x6000, 0),
    S::F(0x1C0E4, 0x30, 2),
    S::F(0x1C1EC, 0x6000, 0),
    S::F(0x1C1E4, 0x30, 2),
    S::F(0x10C9C, 0x000F_F000, 0xFF),
    S::F(0x10CA0, 0x000F_F000, 0),
];

/// `rtw8852c_ctrl_btg_bt_rx(true)` (rtw8852c.c:2640-2659).
const BTG_ON: [S; 11] = [
    S::F(0x14738, 1 << 19, 1),
    S::F(0x14738, 1 << 22, 0),
    S::F(0x1476C, 0xFF00_0000, 0x20),
    S::F(0x14778, 0xFF, 0x30),
    S::F(0x14AA4, 1 << 19, 1),
    S::F(0x14AA4, 1 << 22, 1),
    S::F(0x10980, 0x001E_0000, 0),
    S::F(0x14978, 1 << 14, 1),
    S::F(0x14974, 0x03C0_0000, 2),
    S::F(0x1441C, 1 << 31, 1),
    S::F(0x10C6C, 1 << 21, 1),
];

/// `rtw8852c_ctrl_btg_bt_rx(false)` (:2660-2679).
const BTG_OFF: [S; 12] = [
    S::F(0x14738, 1 << 19, 0),
    S::F(0x14738, 1 << 22, 0),
    S::F(0x1476C, 0xFF00_0000, 0x1A),
    S::F(0x14778, 0xFF, 0x2A),
    S::F(0x14AA4, 1 << 19, 0),
    S::F(0x14AA4, 1 << 22, 0),
    S::F(0x10980, 0x001E_0000, 0xF),
    S::F(0x10980, 0x3C00_0000, 4),
    S::F(0x14978, 1 << 14, 0),
    S::F(0x14974, 0x03C0_0000, 0),
    S::F(0x1441C, 1 << 31, 0),
    S::F(0x10C6C, 1 << 21, 0),
];

/// From `rtw8852c_5m_mask` to the end of `set_channel_bb` (:1595-1646,
/// 1866-1929, 2430-2490, 1648-1671) for 20 MHz on 2.4 or 5 GHz.
fn bb_tail() -> Vec<S> {
    let mut s = vec![
        S::F(0x14C4C, 1 << 12, 0),
        S::F(0x14D10, 1 << 12, 0),
        S::F(0x14450, 1 << 24, 0),
        S::F(0x146C0, 1, 1),
        S::F(0x158DC, 0xC000_0000, 1),
        S::F(0x158DC, 0xC000_0000, 3),
        S::F(0x178DC, 0xC000_0000, 1),
        S::F(0x178DC, 0xC000_0000, 3),
        S::F(0x140F8, 1, 0),
        S::F(0x109A4, 0x1C, 0),
        S::F(0x109A4, 0x1C, 0),
    ];
    for reg in (0xDC00..=0xDDFC).step_by(4) {
        s.push(S::W(reg, 0));
    }
    for (reg, v) in [
        (0xD800, 0x1999_9980),
        (0xD804, 0x3333_3333),
        (0xD808, 0x0120_9333),
        (0xD80C, 0x4924_9249),
        (0xD810, 0x1C9C_9C49),
        (0xD814, 0x3939_3939),
        (0xD818, 0x3939_3939),
        (0xD81C, 0x3939_3939),
        (0xD820, 0x0000_3939),
        (0xD824, 0x0000_07C0),
        (0xD828, 0xE000_0000),
        (0xD82C, 0),
    ] {
        s.push(S::W(reg, v));
    }
    s.extend([
        S::F(0x11200, 0x7000_0000, 7),
        S::F(0x13200, 0x7000_0000, 7),
        S::D(1),
        S::F(0x10704, 2, 1),
        S::F(0x10704, 2, 0),
        S::F(0x11200, 0x7000_0000, 0),
        S::F(0x13200, 0x7000_0000, 0),
        S::F(0x10704, 2, 1),
    ]);
    s
}

/// `set_gain_offset`'s steps from a valid map of zero offsets and base 0.
fn zero_offset_steps(chan: &Chan, p: usize) -> Vec<S> {
    let mut s = Vec::new();
    if chan.band == HwBand::G2 {
        s.push(S::F(0x12340, 0x7F00, 0));
    }
    s.push(S::F([0x14694, 0x14778][p], 0x00FF_0000, 0));
    s.push(S::F(0x14E0C, [0x0000_FF00, 0x00FF_0000][p], 0));
    s.push(S::F(0x14E2C, [0x0000_00FF, 0x0000_FF00][p], 0));
    s
}

/// spur_elimination without a spur at 20 MHz (rtw8852c.c:1549-1593).
const NO_SPUR: [S; 6] = [
    S::F(0x142C4, 1 << 23, 0),
    S::F(0x14C14, 1 << 12, 0),
    S::F(0x14C14, 1 << 12, 0),
    S::F(0x14CD8, 1 << 12, 0),
    S::F(0x14CD8, 1 << 12, 0),
    S::F(0x149E8, 1 << 7, 1),
];

#[test]
fn set_channel_bb_2g_ch1_matches_linux_sequence() {
    let mut g = BbGain::default();
    g.lna_gain[0] = [[-90, -54, -18, 14, 44, 75, 108], [1, 2, 3, 4, 5, 6, 7]];
    g.tia_gain[0] = [[54, 86], [8, 9]];
    let rx = rx_gain(true);
    let chan = g2(1);
    let mut m = RegisterModel::new();
    set_channel_bb(&mut m, &chan, &inputs(&g, &rx, 0)).unwrap();

    // ctrl_sco_cck, ch 1.
    let mut s = vec![
        S::F(0x14A1C, 0x7_FFFF, 0x1FE4F),
        S::F(0x14A20, 0x7_FFFF, 0x2BDAC),
    ];
    // ctrl_ch.
    s.extend(gain_error_steps(&g, &chan, 0));
    s.extend(zero_offset_steps(&chan, 0));
    s.push(S::F(0x14738, 1 << 17, 1));
    s.extend(gain_error_steps(&g, &chan, 1));
    s.extend(zero_offset_steps(&chan, 1));
    s.push(S::F(0x14AA4, 1 << 17, 1));
    s.push(S::F(0x14970, 1 << 1, 0));
    s.push(S::F(0x14E30, 0x1FFF, 2412));
    s.push(S::F(0x14974, 0x7F, 109));
    for (reg, v) in [
        (0x145BC, 0x3D23FF),
        (0x145CC, 0x29B354),
        (0x145D0, 0x0FC1C8),
        (0x145D4, 0xFDB053),
        (0x145D8, 0xF86F9A),
        (0x145C0, 0xFAEF92),
        (0x145C4, 0xFE5FCC),
        (0x145C8, 0xFFDFF5),
    ] {
        s.push(S::F(reg, 0xFF_FFFF, v));
    }
    s.push(S::F(0x10734, 0x00FF_0000, 1));
    s.extend(CTRL_BW_20);
    // CCK on.
    s.extend([
        S::F(0x10700, 1 << 5, 1),
        S::F(0x12320, 1, 0),
        S::F(0x10C80, 1 << 31, 0),
    ]);
    s.extend(NO_SPUR);
    s.extend(BTG_ON);
    s.extend(bb_tail());
    assert_eq!(m.trace, from_zero(&s));
}

#[test]
fn set_channel_bb_5g_36_matches_linux_sequence() {
    let mut g = BbGain::default();
    g.lna_gain[1] = [[-60, -40, -20, 0, 20, 40, 60], [1, 2, 3, 4, 5, 6, 7]];
    g.lna_op1db[1][0] = [-1, -2, -3, -4, -5, -6, -7];
    g.tia_lna_op1db[1][1] = [9, 8, 7, 6, 5, 4, 3, -2];
    let rx = rx_gain(true);
    let chan = g5(36);
    let mut m = RegisterModel::new();
    set_channel_bb(&mut m, &chan, &inputs(&g, &rx, 0)).unwrap();

    let mut s = gain_error_steps(&g, &chan, 0);
    s.extend(zero_offset_steps(&chan, 0));
    s.push(S::F(0x14738, 1 << 17, 0));
    s.extend(gain_error_steps(&g, &chan, 1));
    s.extend(zero_offset_steps(&chan, 1));
    s.push(S::F(0x14AA4, 1 << 17, 0));
    s.push(S::F(0x14970, 1 << 1, 0));
    s.push(S::F(0x14E30, 0x1FFF, 5180));
    // DIV_ROUND_CLOSEST(1 << 18, 5180) = 51.
    s.push(S::F(0x14974, 0x7F, 51));
    // rtw89_encode_chan_idx(36, 5G): base index 2, offset 0.
    s.push(S::F(0x10734, 0x00FF_0000, 0x20));
    s.extend(CTRL_BW_20);
    // CCK off.
    s.extend([
        S::F(0x10700, 1 << 5, 0),
        S::F(0x12320, 1, 1),
        S::F(0x10C80, 1 << 31, 1),
    ]);
    s.extend(NO_SPUR);
    s.extend(BTG_OFF);
    s.extend(bb_tail());
    assert_eq!(m.trace, from_zero(&s));
    // No 2G-only register is touched (0x4BB8 is not one: the 5G bypass LNA
    // gains 4-6 of path A live there).
    for reg in [0x14A1C, 0x14A20, 0x12340, 0x145BC, 0x14678, 0x14BC0] {
        assert!(!m.trace.contains(&Op::R32(reg)), "{reg:#x}");
    }
}

#[test]
fn set_channel_bb_2g_ch1_order() {
    let g = BbGain::default();
    let rx = rx_gain(true);
    let mut m = RegisterModel::new();
    set_channel_bb(&mut m, &g2(1), &inputs(&g, &rx, 0)).unwrap();
    let order = [
        first_w(&m, 0x14A1C),
        first_w(&m, 0x14678),
        first_w(&m, 0x14738),
        first_w(&m, 0x14AA4),
        first_w(&m, 0x14970),
        first(&m, |op| *op == Op::W32(0x14E30, 0x96C)),
        first_w(&m, 0x14974),
        first_w(&m, 0x145BC),
        first(&m, |op| *op == Op::W32(0x10734, 0x0001_0000)),
        first_w(&m, 0x1C0D4),
        first_w(&m, 0x10700),
        first_w(&m, 0x142C4),
        first_w(&m, 0x14C14),
        first_w(&m, 0x149E8),
        first_w(&m, 0x14C4C),
        first_w(&m, 0x146C0),
        first_w(&m, 0x158DC),
        first_w(&m, 0x140F8),
        first_w(&m, 0x109A4),
        first(&m, |op| *op == Op::W32(0xDC00, 0)),
        first(&m, |op| *op == Op::W32(0xD800, 0x1999_9980)),
        first(&m, |op| *op == Op::W32(0x11200, 0x7000_0000)),
    ];
    assert!(order.windows(2).all(|w| w[0] < w[1]), "{order:?}");
    // The first 0x14974 write is the SCO, [6:0] = 0x6D.
    assert_eq!(m.trace[order[6]], Op::W32(0x14974, 0x6D));
}

#[test]
fn nbi_without_spur_writes_notch1_twice() {
    let g = BbGain::default();
    let rx = rx_gain(false);
    let mut m = RegisterModel::new();
    m.set32(0x14C14, 0xFFFF_FFFF);
    m.set32(0x14CD8, 0xFFFF_FFFF);
    set_channel_bb(&mut m, &g2(1), &inputs(&g, &rx, 0)).unwrap();
    let notch: Vec<Op> = m
        .trace
        .iter()
        .copied()
        .filter(|op| {
            matches!(
                op,
                Op::R32(0x14C14 | 0x14CD8 | 0x14C20 | 0x14CE4)
                    | Op::W32(0x14C14 | 0x14CD8 | 0x14C20 | 0x14CE4, _)
            )
        })
        .collect();
    let mut want = Vec::new();
    for reg in [0x14C14, 0x14C14, 0x14CD8, 0x14CD8] {
        want.extend(rmw(reg, 0xFFFF_EFFF));
    }
    assert_eq!(notch, want);
}

#[test]
fn set_channel_bb_5g_153_programs_the_spur() {
    let g = BbGain::default();
    let rx = rx_gain(false);
    let mut m = RegisterModel::new();
    set_channel_bb(&mut m, &g5(153), &inputs(&g, &rx, 0)).unwrap();
    // csi: round_closest(-5e6 / 78125) = -64, mod 2048 = 0x7C0.
    let csi: Vec<Op> = m
        .trace
        .iter()
        .copied()
        .filter(|op| matches!(op, Op::W32(0x142AC | 0x142C4, _)))
        .collect();
    assert_eq!(csi, [Op::W32(0x142AC, 0x7C0), Op::W32(0x142C4, 1 << 23)]);
    // nbi: -5e6 / 312500 = -16 rem 0; tone (-16 + 32) mod 64 = 16; frac 0.
    for (n1, n2) in [(0x14C14, 0x14C20), (0x14CD8, 0x14CE4)] {
        let notch: Vec<Op> = m
            .trace
            .iter()
            .copied()
            .filter(|op| matches!(*op, Op::W32(r, _) if r == n1 || r == n2))
            .collect();
        assert_eq!(
            notch,
            [
                Op::W32(n1, 0x10),
                Op::W32(n1, 0x10),
                Op::W32(n1, 0x10),
                Op::W32(n1, 0x1010),
                Op::W32(n2, 0),
            ],
            "{n1:#x}"
        );
    }
    // Path A's notch is programmed before path B's, and both before PD boost.
    assert!(first_w(&m, 0x14C20) < first_w(&m, 0x14CD8));
    assert!(first_w(&m, 0x14CE4) < first_w(&m, 0x149E8));
}

#[test]
fn spur_notch_keeps_other_bits_and_frac_rounds() {
    // 2G ch 5: 2440 - 2432 = 8 MHz: nbi 25 rem 187500, tone (25 + 32) mod 64
    // = 57, frac round_closest(187500 / 78125) = 2; csi round_closest(8e6 /
    // 78125) = 102.
    let g = BbGain::default();
    let rx = rx_gain(false);
    let mut m = RegisterModel::new();
    m.set32(0x14C14, 0xFFFF_E000);
    m.set32(0x142AC, 0xFFFF_F800);
    set_channel_bb(&mut m, &g2(5), &inputs(&g, &rx, 0)).unwrap();
    assert_eq!(m.get32(0x142AC), 0xFFFF_F800 | 102);
    assert_eq!(m.get32(0x14C14), 0xFFFF_E000 | 1 << 12 | 2 << 10 | 57);
    // 2G ch 13: 2480 - 2472 = 8 MHz, the same indices.
    let mut m = RegisterModel::new();
    set_channel_bb(&mut m, &g2(13), &inputs(&g, &rx, 0)).unwrap();
    assert_eq!(m.get32(0x14CD8), 1 << 12 | 2 << 10 | 57);
    // 5G 163: 5760 - 5815 = -55 MHz: csi round_closest(-704) = -704, mod
    // 2048 = 1344; nbi -176 rem 0, tone (-144) mod 64 = 48.
    let mut m = RegisterModel::new();
    set_channel_bb(&mut m, &g5(163), &inputs(&g, &rx, 0)).unwrap();
    assert_eq!(m.get32(0x142AC), 1344);
    assert_eq!(m.get32(0x14C14), 1 << 12 | 48);
    // Their neighbours have no spur.
    for chan in [g2(4), g2(9), g2(14), g5(149), g5(157), g5(165)] {
        let mut m = RegisterModel::new();
        set_channel_bb(&mut m, &chan, &inputs(&g, &rx, 0)).unwrap();
        assert!(
            !m.trace.iter().any(|op| matches!(op, Op::W32(0x142AC, _))),
            "ch {}",
            chan.channel
        );
        assert_eq!(m.get32(0x142C4), 0, "ch {}", chan.channel);
    }
}

#[test]
fn set_channel_bb_ch14_uses_its_cck_filter() {
    let g = BbGain::default();
    let rx = rx_gain(false);
    let mut m = RegisterModel::new();
    set_channel_bb(&mut m, &g2(14), &inputs(&g, &rx, 0)).unwrap();
    for (reg, v) in [
        (0x145BC, 0x3B13FF),
        (0x145CC, 0x1C42DE),
        (0x145D0, 0xFDB0AD),
        (0x145D4, 0xF60F6E),
        (0x145D8, 0xFD8F92),
        (0x145C0, 0x02D011),
        (0x145C4, 0x01C02C),
        (0x145C8, 0xFFF00A),
    ] {
        assert_eq!(m.get32(reg), v, "{reg:#x}");
    }
    assert_eq!(m.get32(0x14A1C), 0x20D8A);
    assert_eq!(m.get32(0x14A20), 0x2D29E);
    assert_eq!(m.get32(0x10734), 14 << 16);
}

#[test]
fn set_channel_bb_refuses_a_channel_new20_would_not_build() {
    let g = BbGain::default();
    let rx = rx_gain(true);
    for chan in [
        Chan {
            channel: 1,
            band: HwBand::G6,
            freq: 5955,
            subband: Subband::G2,
            primary: 1,
            width: Bw::Mhz20,
            pri_ch_idx: 0,
            txsc: [0; 3],
        },
        Chan {
            channel: 0,
            band: HwBand::G2,
            freq: 0,
            subband: Subband::G2,
            primary: 0,
            width: Bw::Mhz20,
            pri_ch_idx: 0,
            txsc: [0; 3],
        },
        Chan {
            channel: 6,
            band: HwBand::G2,
            freq: 2412,
            subband: Subband::G2,
            primary: 6,
            width: Bw::Mhz20,
            pri_ch_idx: 0,
            txsc: [0; 3],
        },
    ] {
        let mut m = model();
        assert!(matches!(
            set_channel_bb(&mut m, &chan, &inputs(&g, &rx, 0)),
            Err(AxError::Phy(_))
        ));
        assert!(m.trace.is_empty());
        let mut m = model();
        assert!(matches!(
            set_channel(&mut m, &chan, &inputs(&g, &rx, 0), power()),
            Err(ChannelError {
                step: "set channel",
                error: AxError::Phy(_)
            })
        ));
        assert!(m.trace.is_empty());
    }
}

#[test]
fn set_channel_runs_help_mac_bb_rf_txpwr_help_in_order() {
    let g = BbGain::default();
    let rx = rx_gain(true);
    let mut m = model();
    m.set32(0xC398, 0x3FFFF);
    assert_eq!(
        set_channel(&mut m, &g2(1), &inputs(&g, &rx, 0), power()),
        Ok(())
    );
    // Ruling R1 (core.c:584-588): set_txpwr's by rate after the RF part,
    // and help_exit's PPDU status report after set_txpwr.
    let order = [
        first(&m, |op| matches!(op, Op::W32(0xC398, _))),
        first(&m, |op| matches!(op, Op::W8(0xC628, _))),
        first(&m, |op| matches!(op, Op::W32(0x14A1C, _))),
        first(&m, |op| matches!(op, Op::RfW(0, 0x18, ..))),
        first(&m, |op| matches!(op, Op::W32(0xD2C0, _))),
        first(&m, |op| *op == Op::W32(0x176CC, 0x7E)),
        first(&m, |op| *op == Op::W32(0xCE40, 0x2B)),
    ];
    assert!(order.windows(2).all(|w| w[0] < w[1]), "{order:?}");
    assert_eq!(m.trace.last(), Some(&Op::W32(0xC398, 0x3FFFF)));
    // The BB part ends with the TX path table and bb_reset_all before the RF
    // part starts (help_exit writes 0x13200 = 0 again later).
    let tmac = first(&m, |op| *op == Op::W32(0xD82C, 0));
    let reset = tmac
        + m.trace[tmac..]
            .iter()
            .position(|op| *op == Op::W32(0x13200, 0))
            .unwrap();
    assert!(reset < order[3]);
    assert_rf18_everywhere(&m, 0x01C01);
}

#[test]
fn set_channel_matches_its_parts_in_order() {
    let mut g = BbGain::default();
    g.lna_gain[3][0] = [5; 7];
    let mut rx = rx_gain(true);
    rx.offset[0][4] = -2;
    let chan = g5(153);
    let mut tables = TxPowerTables::default();
    tables.byr[1][0].ofdm[4] = 0x28;
    let power = TxPower {
        tables: &tables,
        regds: Regds {
            g2: Regd::Fcc,
            g5: Regd::Etsi,
        },
    };
    for cut in [CAV, CBV] {
        let ins = ChannelInputs {
            cut,
            ..inputs(&g, &rx, -12)
        };
        let mut whole = model();
        whole.set32(0xC398, 0x2_5555);
        set_channel(&mut whole, &chan, &ins, power).unwrap();

        let mut parts = model();
        parts.set32(0xC398, 0x2_5555);
        let tx_en = help_enter(&mut parts, &chan).unwrap();
        set_channel_mac(&mut parts, &chan).unwrap();
        set_channel_bb(&mut parts, &chan, &ins).unwrap();
        set_channel_rf(&mut parts, &chan, cut).unwrap();
        txpwr::set_txpwr(&mut parts, &tables, power.regds, &chan).unwrap();
        help_exit(&mut parts, &chan, tx_en).unwrap();
        assert_eq!(whole.trace, parts.trace, "cut {cut}");
    }
    // The cut reaches set_channel_rf: CAV copies path A into path B.
    let ins = ChannelInputs {
        cut: CAV,
        ..inputs(&g, &rx, 0)
    };
    let mut m = model();
    set_channel(&mut m, &g2(1), &ins, power).unwrap();
    assert!(m.trace.iter().any(|op| matches!(op, Op::RfW(1, 0x05, ..))));
    let mut m = model();
    set_channel(&mut m, &g2(1), &inputs(&g, &rx, 0), power).unwrap();
    assert!(!m.trace.iter().any(|op| matches!(op, Op::RfW(1, 0x05, ..))));
}

#[test]
fn set_channel_stops_at_a_failed_cmac_check() {
    let g = BbGain::default();
    let rx = rx_gain(true);
    let mut m = model();
    m.set32(0xC000, 0);
    assert_eq!(
        set_channel(&mut m, &g2(1), &inputs(&g, &rx, 0), power()),
        Err(ChannelError {
            step: "set channel",
            error: AxError::Mac("CMAC function is not enabled")
        })
    );
    assert!(m.writes().is_empty());
}

#[test]
fn a_tx_power_failure_is_named_and_leaves_the_help_undone() {
    // set_txpwr's SAR ceiling of path A cannot be read: the switch stops
    // there, with the TX scheduler stopped and the BB held in reset.
    let g = BbGain::default();
    let rx = rx_gain(true);
    let mut m = model();
    m.set32(0xC398, 0x3FFFF);
    m.fail_reads_of(0x156CC);
    assert_eq!(
        set_channel(&mut m, &g5(36), &inputs(&g, &rx, 0), power()),
        Err(ChannelError {
            step: "tx power",
            error: READ_FAILED
        })
    );
    assert_eq!(m.trace.last(), Some(&Op::R32(0x156CC)));
    assert!(!m.trace.contains(&Op::W32(0xCE40, 0x2B)));
    assert_eq!(m.get32(0xC398) & 0x3FFFF, 0);
}

#[test]
#[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES"]
fn real_8852c_fw_2_gain_error_2g_uses_the_firmware_table() {
    extern crate std;
    use crate::wifi::realtek::ax::elements::phy_tables;
    let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
    let b = std::fs::read(root.join("rtw89/rtw8852c_fw-2.bin")).unwrap();
    let t = phy_tables(&b, 0).unwrap();
    let mut m = RegisterModel::new();
    let init = bb::init_bb_reg(&mut m, &t.bb.unwrap(), &t.bb_gain.unwrap(), 1, 1).unwrap();

    let mut m = RegisterModel::new();
    set_gain_error(&mut m, &init.gain, &g2(1), RfPath::A).unwrap();
    assert_eq!(m.writes().len(), 16);
    // The LNA then TIA writes, in order, each carry the firmware's value in
    // its field.
    let fields: Vec<u32> = m
        .trace
        .iter()
        .filter_map(|op| match *op {
            Op::W32(r, v) if (0x14678..=0x14680).contains(&r) => Some((r, v)),
            _ => None,
        })
        .zip(LNA.iter().chain(TIA.iter()))
        .map(|((r, v), &(g, _, mask))| {
            assert_eq!(r, g[0]);
            (v & mask) >> mask.trailing_zeros()
        })
        .collect();
    assert_eq!(
        fields,
        [0xA6, 0xCA, 0xEE, 0x0E, 0x2C, 0x4B, 0x6C, 0x36, 0x56]
    );
    assert_eq!(m.get32(0x14678), 0xCAA6_0000);
    assert_eq!(m.get32(0x1467C), 0x4B2C_0EEE);
    assert_eq!(m.get32(0x14680), 0x5636_006C);
}

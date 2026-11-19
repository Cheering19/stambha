// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
extern crate std;
use super::*;
use crate::wifi::realtek::ax::elements::{
    ID_BB_GAIN, ID_BB_REG, ID_RF_NCTL, phy_tables,
    tests::{container, element, pairs},
};
use crate::wifi::realtek::ax::tests::model::{Op, READ_FAILED, RegisterModel};
use alloc::{boxed::Box, format, string::String, vec, vec::Vec};

#[test]
fn bb_helpers_add_the_cr_base() {
    let mut m = RegisterModel::new();
    field32(&mut m, 0x0704, 0x2, 1).unwrap();
    assert_eq!(m.trace, [Op::R32(0x10704), Op::W32(0x10704, 0x2)]);

    let mut m = RegisterModel::new();
    write32(&mut m, 0x4AD4, 0x40).unwrap();
    assert_eq!(m.trace, [Op::W32(0x14AD4, 0x40)]);
}

#[test]
fn bb_set_clear_and_reads_add_the_cr_base() {
    let mut m = RegisterModel::new();
    m.set32(0x10A10, 0x1234_5678);
    set32(&mut m, 0x0A10, 0x8000_0000).unwrap();
    clear32(&mut m, 0x0A10, 0x0000_0078).unwrap();
    assert_eq!(read32(&mut m, 0x0A10), Ok(0x9234_5600));
    // (0x9234_5600 & 0x00FF_0000) >> 16
    assert_eq!(read32_mask(&mut m, 0x0A10, 0x00FF_0000), Ok(0x34));
    assert_eq!(
        m.trace,
        [
            Op::R32(0x10A10),
            Op::W32(0x10A10, 0x9234_5678),
            Op::R32(0x10A10),
            Op::W32(0x10A10, 0x9234_5600),
            Op::R32(0x10A10),
            Op::R32(0x10A10),
        ]
    );
}

#[test]
fn bb_field32_is_a_read_modify_write_even_for_a_full_mask() {
    // core.h:8224-8235: rtw89_write32_mask always reads first.
    let mut m = RegisterModel::new();
    m.set32(0x10370, 0xFFFF_FFFF);
    field32(&mut m, 0x0370, u32::MAX, 0x1050_0000).unwrap();
    assert_eq!(m.trace, [Op::R32(0x10370), Op::W32(0x10370, 0x1050_0000)]);
}

#[test]
fn bb_read32_mask_of_an_empty_mask_is_zero() {
    let mut m = RegisterModel::new();
    m.set32(0x10000, 0xFFFF_FFFF);
    assert_eq!(read32_mask(&mut m, 0, 0), Ok(0));
}

#[test]
fn bb_offset_past_the_address_space_is_refused_untouched() {
    let mut m = RegisterModel::new();
    let past = u32::MAX - 0xFFFF;
    assert_eq!(
        read32(&mut m, past),
        Err(AxError::OutOfWindow { register: past })
    );
    assert_eq!(
        write32(&mut m, past, 0),
        Err(AxError::OutOfWindow { register: past })
    );
    assert_eq!(m.trace, Vec::new());
}

#[test]
fn config_bb_reg_handles_delays_bypass_and_plain_writes() {
    // phy.c:1347-1376: the six delay addresses, BYPASS_CR_DATA, else a
    // plain BB write with no read.
    let mut m = RegisterModel::new();
    for (addr, data) in [
        (0xFE, 0),
        (0xFD, 0),
        (0xFC, 0),
        (0xFB, 0),
        (0xFA, 0),
        (0xF9, 0),
        (0x4AD4, 0xBABE_CAFE),
        (0x4AD4, 0x40),
    ] {
        config_bb_reg(&mut m, addr, data).unwrap();
    }
    assert_eq!(
        m.trace,
        [
            Op::Delay(50_000),
            Op::Delay(5_000),
            Op::Delay(1_000),
            Op::Delay(50),
            Op::Delay(5),
            Op::Delay(1),
            Op::W32(0x14AD4, 0x40),
        ]
    );

    // The address is tested before the data: a delay entry carrying
    // BYPASS_CR_DATA still delays, and 0xF8 is an ordinary address.
    let mut m = RegisterModel::new();
    config_bb_reg(&mut m, 0xFE, 0xBABE_CAFE).unwrap();
    config_bb_reg(&mut m, 0xF8, 0x1).unwrap();
    assert_eq!(m.trace, [Op::Delay(50_000), Op::W32(0x100F8, 0x1)]);
}

/// The 59 writes `rtw8852c_init_txpwr_unit(PHY0)` makes from all-zero
/// registers (spec S8.3).
fn txpwr_unit_writes() -> Vec<Op> {
    let mut want = vec![
        Op::W32(0xD248, 0x0776_3333),
        Op::W32(0xD220, 0x01EB_F000),
        Op::W32(0xD240, 0x0002_F8FF),
    ];
    // set_txpwr_ul_tb_offset(0): val_1t 0, val_2t -12 (0xF4) per byte.
    let mut d290 = 0;
    for i in 0..4 {
        want.push(Op::W32(0xD28C, 0));
        d290 |= 0xF4 << (8 * i);
        want.push(Op::W32(0xD290, d290));
    }
    // init_tssi_ctrl: R_AX_TSSI_CTRL_HEAD..=TAIL, then ctrl_ini.
    want.extend((0xD908..=0xD95C).step_by(4).map(|a| Op::W32(a, 0)));
    want.extend([
        Op::W32(0xD938, 0x0001_0100),
        Op::W32(0xD93C, 0x0500_D500),
        Op::W32(0xD940, 0x0000_0500),
        Op::W32(0xD944, 0x0000_0005),
        Op::W32(0xD94C, 0x0022_0000),
        Op::W32(0xD950, 0x0003_0000),
    ]);
    // tssi_ctrl_set_bandedge_cfg(FLAT): 15 fields of 0, all zero already.
    want.extend(
        [
            0xD91C, 0xD920, 0xD920, 0xD920, 0xD920, 0xD924, 0xD924, 0xD914, 0xD918, 0xD918, 0xD918,
            0xD918, 0xD91C, 0xD91C, 0xD91C,
        ]
        .map(|a| Op::W32(a, 0)),
    );
    // BANDEDGE_CFG [31:30] = FLAT (0), then the flat fast-mode fields.
    want.extend([
        Op::W32(0xD94C, 0x0022_0000),
        Op::W32(0xD934, 0),
        Op::W32(0xD934, 0),
        Op::W32(0xD938, 0x0001_0100),
        Op::W32(0xD934, 0),
    ]);
    want
}

#[test]
fn init_txpwr_unit_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    init_txpwr_unit(&mut m).unwrap();
    let want = txpwr_unit_writes();
    assert_eq!(want.len(), 59);
    assert_eq!(m.writes(), want);
    // The 8 UL TB, 15 bandedge, 1 BANDEDGE_CFG and 4 fast-mode writes are
    // read-modify-writes of the register they write; the rest are plain.
    let reads: Vec<(usize, u32)> = m
        .trace
        .iter()
        .enumerate()
        .filter_map(|(i, op)| match op {
            Op::R32(a) => Some((i, *a)),
            _ => None,
        })
        .collect();
    assert_eq!(reads.len(), 28);
    for (i, a) in reads {
        assert!(matches!(m.trace[i + 1], Op::W32(w, _) if w == a), "{i}");
    }
}

#[test]
fn init_txpwr_unit_keeps_bits_outside_its_fields() {
    // The UL TB fields are bytes of 0xD28C/0xD290, outside the TSSI range.
    let mut m = RegisterModel::new();
    m.set32(0xD28C, 0xFFFF_FFFF);
    m.set32(0xD290, 0x1234_5678);
    init_txpwr_unit(&mut m).unwrap();
    assert_eq!(m.get32(0xD28C), 0);
    assert_eq!(m.get32(0xD290), 0xF4F4_F4F4);
}

#[test]
fn bb_reset_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    bb_reset(&mut m).unwrap();
    assert_eq!(
        m.writes(),
        [
            Op::W32(0x11200, 0x7000_0000),
            Op::W32(0x13200, 0x7000_0000),
            Op::Delay(1),
            Op::W32(0x10704, 0x2),
            Op::W32(0x10704, 0),
            Op::W32(0x11200, 0),
            Op::W32(0x13200, 0),
            Op::W32(0x10704, 0x2),
        ]
    );
    // Every write is a read-modify-write (rtw89_phy_write32_mask).
    let reads = m.trace.iter().filter(|op| matches!(op, Op::R32(_))).count();
    assert_eq!(reads, 7);
}

#[test]
fn bb_reset_keeps_bits_outside_its_fields() {
    let mut m = RegisterModel::new();
    m.set32(0x11200, 0x8FFF_FFFF);
    m.set32(0x13200, 0x0000_0001);
    m.set32(0x10704, 0xFFFF_FFFD);
    bb_reset(&mut m).unwrap();
    assert_eq!(m.get32(0x11200), 0x8FFF_FFFF);
    assert_eq!(m.get32(0x13200), 0x0000_0001);
    assert_eq!(m.get32(0x10704), 0xFFFF_FFFF);
}

#[test]
fn bb_gain_decodes_each_cfg_type() {
    let mut g = BbGain::default();
    g.decode(0x0000_0000, 0x0EEE_CAA6, 1);
    assert_eq!(g.lna_gain[0][0][0..4], [-90, -54, -18, 14]);
    g.decode(0x0000_0001, 0x006C_4B2C, 1);
    assert_eq!(g.lna_gain[0][0][4..7], [44, 75, 108]);
    g.decode(0x0000_0002, 0x5636, 1);
    assert_eq!(g.tia_gain[0][0], [54, 86]);

    // cfg 1, band 1, path 0, bw 1, rxsc 1: two bytes from index 1, so a
    // preset index 2 is overwritten with the data's second byte, 0.
    g.rpl_ofst_40[1][0][2] = 7;
    g.decode(0x0101_0011, 0x05, 1);
    assert_eq!(g.rpl_ofst_40[1][0][1..3], [5, 0]);

    // cfg 3, band 0, path 1, type 3: tia_lna_op1db [4..8].
    g.decode(0x0300_0103, 0x0403_0201, 1);
    assert_eq!(g.tia_lna_op1db[0][1][4..8], [1, 2, 3, 4]);
}

#[test]
fn bb_gain_decodes_every_rpl_ofst_window_bypass_and_op1db() {
    let mut g = BbGain::default();
    let mut want = BbGain::default();
    let d = 0x8483_8281; // bytes -127, -126, -125, -124
    let four = [-127, -126, -125, -124];

    // cfg 1 bw 0 takes the low byte whatever rxsc says.
    g.decode(0x0107_0105, d, 1);
    want.rpl_ofst_20[7][1] = -127;
    // bw 1: rxsc 0 is one byte at [0].
    g.decode(0x0100_0010, d, 1);
    want.rpl_ofst_40[0][0][0] = -127;
    // bw 2: rxsc 0 / 1 / 9.
    g.decode(0x0100_0020, d, 1);
    want.rpl_ofst_80[0][0][0] = -127;
    g.decode(0x0100_0021, d, 1);
    want.rpl_ofst_80[0][0][1..5].copy_from_slice(&four);
    g.decode(0x0100_0029, d, 1);
    want.rpl_ofst_80[0][0][9..11].copy_from_slice(&four[..2]);
    // bw 3: rxsc 0 / 1 / 5 / 9 / 13.
    g.decode(0x0102_0130, d, 1);
    want.rpl_ofst_160[2][1][0] = -127;
    g.decode(0x0102_0131, d, 1);
    want.rpl_ofst_160[2][1][1..5].copy_from_slice(&four);
    g.decode(0x0102_0135, d, 1);
    want.rpl_ofst_160[2][1][5..9].copy_from_slice(&four);
    g.decode(0x0102_0139, d, 1);
    want.rpl_ofst_160[2][1][9..13].copy_from_slice(&four);
    g.decode(0x0102_013D, d, 1);
    want.rpl_ofst_160[2][1][13..15].copy_from_slice(&four[..2]);
    // cfg 2: lna_gain_bypass [0..4] / [4..7].
    g.decode(0x0203_0000, d, 1);
    want.lna_gain_bypass[3][0][0..4].copy_from_slice(&four);
    g.decode(0x0203_0001, d, 1);
    want.lna_gain_bypass[3][0][4..7].copy_from_slice(&four[..3]);
    // cfg 3: lna_op1db [0..4] / [4..7], tia_lna_op1db [0..4].
    g.decode(0x0304_0100, d, 1);
    want.lna_op1db[4][1][0..4].copy_from_slice(&four);
    g.decode(0x0304_0101, d, 1);
    want.lna_op1db[4][1][4..7].copy_from_slice(&four[..3]);
    g.decode(0x0304_0102, d, 1);
    want.tia_lna_op1db[4][1][0..4].copy_from_slice(&four);
    assert_eq!(g, want);

    // Unknown sub-keys are ignored, as Linux warns and skips them.
    for addr in [
        0x0000_0003, // cfg 0 type 3
        0x0100_0012, // bw 1 rxsc 2
        0x0100_0025, // bw 2 rxsc 5
        0x0100_0032, // bw 3 rxsc 2
        0x0100_0040, // bw 4
        0x0200_0002, // cfg 2 type 2
        0x0300_0004, // cfg 3 type 4
        0x0400_0000, // cfg 4 at RFE 50: unknown
        0x0500_0000, // cfg 5
    ] {
        g.decode(addr, 0x0102_0304, 50);
        assert_eq!(g, want, "{addr:#x}");
    }
}

#[test]
fn bb_gain_ignores_type_4_below_rfe_50_and_bad_band_path_or_delays() {
    for (addr, data, rfe) in [
        (0x0400_0000, 1, 1),
        (0x0008_0000, 1, 1),
        (0x0000_0200, 1, 1),
        (0xFE, 1, 1),
    ] {
        let mut g = BbGain::default();
        g.decode(addr, data, rfe);
        assert_eq!(g, BbGain::default(), "{addr:#x}");
    }
}

/// A file with one BB_REG and one BB_GAIN element; a TXPWR element follows
/// so the last table is still read (fw.c:1584).
fn bb_file(bb: &[(u32, u32)], gain: &[(u32, u32)]) -> Vec<u8> {
    let mut b = container(32, 16);
    element(&mut b, ID_BB_REG, [0; 4], 0, 0, &pairs(bb));
    element(&mut b, ID_BB_GAIN, [0; 4], 0, 0, &pairs(gain));
    element(&mut b, 9, [0; 4], 0, 0, &[0; 16]);
    b
}

fn first(m: &RegisterModel, want: impl Fn(&Op) -> bool) -> usize {
    m.trace.iter().position(want).unwrap()
}

#[test]
fn init_bb_reg_runs_table_txpwr_gain_then_reset() {
    let b = bb_file(&[(0x4AD4, 0x40)], &[(0, 0x0EEE_CAA6)]);
    let t = phy_tables(&b, 0).unwrap();
    let mut m = RegisterModel::new();
    let init = init_bb_reg(&mut m, &t.bb.unwrap(), &t.bb_gain.unwrap(), 1, 1).unwrap();

    let table = first(&m, |op| *op == Op::W32(0x14AD4, 0x40));
    let txpwr = first(&m, |op| matches!(op, Op::W32(0xD248, _)));
    let reset = first(&m, |op| *op == Op::W32(0x11200, 0x7000_0000));
    assert!(table < txpwr && txpwr < reset, "{table} {txpwr} {reset}");
    assert_eq!((init.bb, init.bb_gain), (1, 1));
    assert_eq!(init.gain.lna_gain[0][0][0..4], [-90, -54, -18, 14]);
    // The gain table writes nothing: the table's write, txpwr's 59, then
    // the reset's 8 operations.
    let writes = m.writes();
    assert_eq!(writes.len(), 1 + 59 + 8);
    assert_eq!(writes[1..60], txpwr_unit_writes());
    assert_eq!(writes[60], Op::W32(0x11200, 0x7000_0000));
}

#[test]
fn init_bb_reg_refuses_an_unselected_table_before_any_write() {
    // A headline for RFE 2 only: Linux logs "invalid PHY package".
    let b = bb_file(&[(0xF002_0001, 0), (0x4AD4, 0x40)], &[]);
    let t = phy_tables(&b, 0).unwrap();
    let mut m = RegisterModel::new();
    assert_eq!(
        init_bb_reg(&mut m, &t.bb.unwrap(), &t.bb_gain.unwrap(), 1, 1),
        Err(AxError::PhyPackage {
            table: "BB_REG",
            rfe: 1,
            cv: 1
        })
    );
    assert_eq!(m.trace, []);
}

#[test]
fn init_bb_reg_stops_at_a_failed_access() {
    let b = bb_file(&[(0x4AD4, 0x40)], &[]);
    let t = phy_tables(&b, 0).unwrap();
    let mut m = RegisterModel::new();
    m.fail_reads_of(0xD28C);
    assert_eq!(
        init_bb_reg(&mut m, &t.bb.unwrap(), &t.bb_gain.unwrap(), 1, 1),
        Err(READ_FAILED)
    );
    assert!(!m.trace.iter().any(|op| matches!(op, Op::W32(0x11200, _))));
}

#[test]
#[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES"]
fn real_8852c_fw_2_bb_init_writes_the_selected_table() {
    use sha2::{Digest, Sha256};
    let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
    let b = std::fs::read(root.join("rtw89/rtw8852c_fw-2.bin")).unwrap();
    let t = phy_tables(&b, 0).unwrap();
    let mut m = RegisterModel::new();
    let init = init_bb_reg(&mut m, &t.bb.unwrap(), &t.bb_gain.unwrap(), 1, 1).unwrap();

    let txpwr = first(&m, |op| matches!(op, Op::W32(0xD248, _)));
    let table = &m.trace[..txpwr];
    // The table is plain BB writes: nothing is read before init_txpwr_unit.
    assert!(
        !table
            .iter()
            .any(|op| matches!(op, Op::R8(_) | Op::R16(_) | Op::R32(_)))
    );
    let mut hash = Sha256::new();
    let mut writes = 0;
    for op in table {
        if let Op::W32(a, d) = *op {
            hash.update((a - CR_BASE).to_le_bytes());
            hash.update(d.to_le_bytes());
            writes += 1;
        }
    }
    let hex: String = hash.finalize().iter().map(|b| format!("{b:02x}")).collect();
    assert_eq!(writes, 1426);
    assert_eq!(
        hex,
        "8b8d51888011a74317d9bac62e6af0304593cb8d07c89c7ca303cc1ffd843589"
    );
    assert_eq!((init.bb, init.bb_gain), (1426, 336));
    assert_eq!(init.gain.lna_gain[0][0][0..4], [-90, -54, -18, 14]);
    assert_eq!(init.gain.lna_gain[0][0][4..7], [44, 75, 108]);
    assert_eq!(init.gain.tia_gain[0][0], [54, 86]);
    // bb_reset ends it, setting B_RSTB_ASYNC_ALL in what the table wrote
    // to 0x0704.
    assert_eq!(m.trace.last(), Some(&Op::W32(0x10704, 0x601C_0502)));
    assert_eq!(m.get32(0x10704) & 0x2, 0x2);
}

/// A file with one RF_NCTL element; a TXPWR element follows so it is read.
fn nctl_file(nctl: &[(u32, u32)]) -> Vec<u8> {
    let mut b = container(32, 16);
    element(&mut b, ID_RF_NCTL, [0; 4], 0, 0, &pairs(nctl));
    element(&mut b, 9, [0; 4], 0, 0, &[0; 16]);
    b
}

#[test]
fn nctl_init_matches_linux_sequence() {
    // phy.c:2041-2078: the IQK/DPK clock and reset bits are set, 0x8000 is
    // written 8, 0x8080 is polled (write 4, 1 us, read), then the table.
    let b = nctl_file(&[(0x8008, 0), (0x8000, 8), (0xFB, 0), (0x81C0, 0xBABE_CAFE)]);
    let t = phy_tables(&b, 0).unwrap();
    let mut m = RegisterModel::new();
    m.on_read(0x18080, Box::new(|_| 4));
    assert_eq!(init_rf_nctl(&mut m, &t.nctl.unwrap(), 1, 1), Ok(false));
    assert_eq!(
        m.writes(),
        [
            Op::W32(0x10C60, 3),
            Op::W32(0x10C6C, 1),
            Op::W32(0x158AC, 0x0800_0000),
            Op::W32(0x178AC, 0x0800_0000),
            Op::W32(0x18000, 8),
            Op::W32(0x18080, 4),
            Op::Delay(1),
            Op::W32(0x18008, 0),
            Op::W32(0x18000, 8),
            Op::Delay(50),
        ]
    );
}

#[test]
fn nctl_poll_timeout_is_recorded_and_the_table_still_loads() {
    let b = nctl_file(&[(0x8008, 0), (0x8000, 8)]);
    let t = phy_tables(&b, 0).unwrap();
    let mut m = RegisterModel::new();
    // The model stores the 4 the poll writes; hardware that never comes up
    // reads something else.
    m.on_read(0x18080, Box::new(|_| 0));
    assert_eq!(init_rf_nctl(&mut m, &t.nctl.unwrap(), 1, 1), Ok(true));
    let polls = m
        .trace
        .iter()
        .filter(|op| **op == Op::W32(0x18080, 4))
        .count();
    // One sample every 11 us (the 1 us settle and the 10 us interval) until
    // the clock passes 1000 us: samples at 0, 11, ..., 1001.
    assert_eq!(polls, 92);
    let waits = m.trace.iter().filter(|op| **op == Op::Delay(10)).count();
    assert_eq!(waits, 91);
    let tail = &m.trace[m.trace.len() - 2..];
    assert_eq!(tail, [Op::W32(0x18008, 0), Op::W32(0x18000, 8)]);
}

#[test]
fn nctl_init_refuses_an_unselected_table_after_the_poll() {
    let b = nctl_file(&[(0xF002_0001, 0), (0x8008, 0)]);
    let t = phy_tables(&b, 0).unwrap();
    let mut m = RegisterModel::new();
    m.on_read(0x18080, Box::new(|_| 4));
    assert_eq!(
        init_rf_nctl(&mut m, &t.nctl.unwrap(), 1, 1),
        Err(AxError::PhyPackage {
            table: "RF_NCTL",
            rfe: 1,
            cv: 1
        })
    );
}

#[test]
#[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES"]
fn real_8852c_fw_2_nctl_writes_1774_entries() {
    use sha2::{Digest, Sha256};
    let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
    let b = std::fs::read(root.join("rtw89/rtw8852c_fw-2.bin")).unwrap();
    let t = phy_tables(&b, 0).unwrap();
    let mut m = RegisterModel::new();
    m.on_read(0x18080, Box::new(|_| 4));
    assert_eq!(init_rf_nctl(&mut m, &t.nctl.unwrap(), 1, 1), Ok(false));

    // After the poll's Delay(1) the rest is the table: plain writes (and
    // table delays, which are not writes).
    let poll = first(&m, |op| *op == Op::Delay(1));
    let mut hash = Sha256::new();
    let mut writes = Vec::new();
    for op in &m.trace[poll + 1..] {
        if let Op::W32(a, d) = *op {
            hash.update((a - CR_BASE).to_le_bytes());
            hash.update(d.to_le_bytes());
            writes.push((a - CR_BASE, d));
        }
    }
    let hex: String = hash.finalize().iter().map(|b| format!("{b:02x}")).collect();
    assert_eq!(writes.len(), 1774);
    assert_eq!(writes[..2], [(0x8008, 0), (0x8000, 8)]);
    assert_eq!(
        hex,
        "fec1c1dcc061316be04e931aecde57a42eea8f4af0112147502c34c53cbf2df5"
    );
}

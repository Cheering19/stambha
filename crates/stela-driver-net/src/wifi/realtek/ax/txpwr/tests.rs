// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
extern crate std;
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use crate::wifi::realtek::ax::txpwr_tables::{RS_MCS, RS_OFDM};
use alloc::{vec, vec::Vec};

const FCC: Regds = Regds {
    g2: Regd::Fcc,
    g5: Regd::Fcc,
};
const FCC_U8: u8 = Regd::Fcc as u8;
const WW_U8: u8 = Regd::Ww as u8;
const ETSI_U8: u8 = Regd::Etsi as u8;

fn chan(band: HwBand, ch: u8) -> Chan {
    Chan::new20(band, ch).unwrap()
}

/// A read-modify-write of `reg` that leaves `v`.
fn rmw(reg: u32, v: u32) -> [Op; 2] {
    [Op::R32(reg), Op::W32(reg, v)]
}

/// The plain 32-bit writes of the MAC words from `first` to `last`.
fn words(m: &RegisterModel, first: u32, last: u32) -> Vec<(u32, u32)> {
    m.trace
        .iter()
        .filter_map(|op| match *op {
            Op::W32(reg, v) if (first..=last).contains(&reg) => Some((reg, v)),
            _ => None,
        })
        .collect()
}

/// Four signed bytes as one little-endian word, `v[0]` in [7:0].
fn le(v: [i8; 4]) -> u32 {
    u32::from_le_bytes(v.map(|x| x as u8))
}

#[test]
fn ch_idx_matches_linux() {
    // rtw89_channel_to_idx (phy.c:2535-2553).
    for (band, ch, idx) in [
        (HwBand::G2, 1, 0),
        (HwBand::G2, 14, 13),
        (HwBand::G5, 36, 0),
        (HwBand::G5, 64, 14),
        (HwBand::G5, 100, 15),
        (HwBand::G5, 144, 37),
        (HwBand::G5, 149, 38),
        (HwBand::G5, 165, 46),
        (HwBand::G5, 177, 52),
    ] {
        assert_eq!(ch_idx(&chan(band, ch)), idx, "{band:?} {ch}");
    }
}

#[test]
fn zero_limit_falls_back_to_world() {
    let mut t = TxPowerTables::default();
    // [bw 0][nt 0][OFDM][bf 0][regd][ch_idx 3]
    t.lmt_2ghz[0][0][1][0][usize::from(WW_U8)][3] = 60;
    assert_eq!(
        read_limit(&t, Regd::Fcc, HwBand::G2, 0, 0, RS_OFDM, 0, 3),
        30
    );
    // A regd value of its own wins over WW, and halves arithmetically.
    t.lmt_2ghz[0][0][1][0][usize::from(FCC_U8)][3] = -9;
    assert_eq!(
        read_limit(&t, Regd::Fcc, HwBand::G2, 0, 0, RS_OFDM, 0, 3),
        -5
    );
    t.lmt_5ghz[3][1][2][1][usize::from(WW_U8)][52] = 44;
    assert_eq!(
        read_limit(&t, Regd::Mkk, HwBand::G5, 3, 1, RS_MCS, 1, 52),
        22
    );
    // Nothing anywhere reads 0, and so does a bandwidth past the band's.
    assert_eq!(
        read_limit(&t, Regd::Fcc, HwBand::G2, 0, 1, RS_OFDM, 0, 3),
        0
    );
    assert_eq!(
        read_limit(&t, Regd::Fcc, HwBand::G2, 2, 0, RS_OFDM, 0, 3),
        0
    );
    // The RU limit falls back the same way.
    t.lmt_ru_5ghz[2][1][usize::from(WW_U8)][15] = 30;
    assert_eq!(read_limit_ru(&t, Regd::Etsi, HwBand::G5, 2, 1, 15), 15);
    t.lmt_ru_5ghz[2][1][usize::from(ETSI_U8)][15] = 26;
    assert_eq!(read_limit_ru(&t, Regd::Etsi, HwBand::G5, 2, 1, 15), 13);
}

#[test]
fn limit_is_capped_at_63() {
    let mut t = TxPowerTables::default();
    t.lmt_2ghz[1][1][0][1][usize::from(FCC_U8)][0] = 127;
    assert_eq!(
        read_limit(&t, Regd::Fcc, HwBand::G2, 1, 1, RS_CCK, 1, 0),
        63
    );
    t.lmt_ru_2ghz[0][0][usize::from(FCC_U8)][0] = 127;
    assert_eq!(read_limit_ru(&t, Regd::Fcc, HwBand::G2, 0, 0, 0), 63);
}

/// By-rate tables whose every slot is distinct: 2.4 GHz CCK 2, 4, 6, 8 and
/// 5 GHz CCK 100.., so a 5 GHz channel that read its own band's CCK shows.
fn distinct_byrate() -> TxPowerTables {
    let mut t = TxPowerTables::default();
    for (band, base) in [(0usize, 0i8), (1, 40)] {
        let b = &mut t.byr[band][0];
        for i in 0..4 {
            b.cck[i] = base * 2 + 2 * (i as i8 + 1) + if band == 1 { 20 } else { 0 };
        }
        for i in 0..8 {
            b.ofdm[i] = base + 10 + 2 * i as i8;
        }
        for nss in 0..2 {
            for i in 0..12 {
                b.mcs[0][nss][i] = base / 4 - 30 + 24 * nss as i8 + 2 * i as i8;
                // OFDMA rows and higher NSS are never read.
                b.mcs[1][nss][i] = 99;
            }
            for i in 0..4 {
                b.hedcm[0][nss][i] = base + 1 + 2 * (4 * nss as i8 + i as i8);
                b.hedcm[1][nss][i] = 99;
            }
        }
        b.mcs[0][2] = [99; 16];
        b.mcs[0][0][12..].copy_from_slice(&[99; 4]);
        b.offset = [2, -2, 4, 6, 8, 99, 99, 99];
    }
    t
}

#[test]
fn byrate_writes_eleven_dwords_from_2g_cck() {
    let t = distinct_byrate();
    let c = chan(HwBand::G5, 36);
    let mut m = RegisterModel::new();
    set_txpwr_byrate(&mut m, &t, &c).unwrap();
    // Plain writes only (rtw89_mac_txpwr_write32, mac.h:1588-1600).
    assert!(m.trace.iter().all(|op| matches!(op, Op::W32(..))));
    let b5 = &t.byr[1][0];
    let half = |v: &[i8]| -> u32 { le([v[0] >> 1, v[1] >> 1, v[2] >> 1, v[3] >> 1]) };
    let want = vec![
        // CCK always from 2.4 GHz (phy.c:2501-2502): 2, 4, 6, 8 halved.
        (0xD2C0, le([1, 2, 3, 4])),
        (0xD2C4, half(&b5.ofdm[0..4])),
        (0xD2C8, half(&b5.ofdm[4..8])),
        (0xD2CC, half(&b5.mcs[0][0][0..4])),
        (0xD2D0, half(&b5.mcs[0][0][4..8])),
        (0xD2D4, half(&b5.mcs[0][0][8..12])),
        (0xD2D8, half(&b5.hedcm[0][0])),
        (0xD2DC, half(&b5.mcs[0][1][0..4])),
        (0xD2E0, half(&b5.mcs[0][1][4..8])),
        (0xD2E4, half(&b5.mcs[0][1][8..12])),
        (0xD2E8, half(&b5.hedcm[0][1])),
    ];
    assert_eq!(words(&m, 0, u32::MAX), want);
    // The MCS rows hold negative values: their bytes are the s8's.
    assert_eq!(b5.mcs[0][0][0], -20);
    assert_eq!(want[3].1 & 0xFF, 0xF6);
}

#[test]
fn offset_writes_five_nibbles_from_the_band() {
    let t = distinct_byrate();
    let mut m = RegisterModel::new();
    m.set32(0xD204, 0xFFFF_FFFF);
    set_txpwr_offset(&mut m, &t, &chan(HwBand::G2, 6)).unwrap();
    // offset 2, -2, 4, 6, 8 halved: 1, -1, 2, 3, 4 as nibbles [3:0]..[19:16]
    // (phy.c:3126-3152); [31:20] kept by the read-modify-write.
    let mut want = Vec::new();
    want.extend(rmw(0xD204, 0xFFF4_32F1));
    assert_eq!(m.trace, want);
}

/// Limit tables with every 2.4 GHz FCC entry set by rate section, bw and
/// bf, and every other regd or channel at a value that must not be read.
fn limits() -> TxPowerTables {
    let mut t = TxPowerTables::default();
    for bw in 0..2 {
        for nt in 0..2 {
            for rs in 0..3 {
                for bf in 0..2 {
                    for regd in 0..16 {
                        for ch in 0..14 {
                            let v = if regd == usize::from(FCC_U8) && ch == 5 {
                                (2 * (30 * nt + 12 * bw + 4 * rs + 2 * bf + 1)) as i8
                            } else {
                                120
                            };
                            t.lmt_2ghz[bw][nt][rs][bf][regd][ch] = v;
                        }
                    }
                }
            }
        }
    }
    for ru in 0..8 {
        for nt in 0..2 {
            for regd in 0..16 {
                for ch in 0..14 {
                    let v = if regd == usize::from(FCC_U8) && ch == 5 {
                        (2 * (20 * nt + 3 * ru + 1)) as i8
                    } else {
                        120
                    };
                    t.lmt_ru_2ghz[ru][nt][regd][ch] = v;
                }
            }
        }
    }
    t
}

#[test]
fn limit_writes_twenty_dwords_with_only_20mhz_fields() {
    let t = limits();
    let mut m = RegisterModel::new();
    set_txpwr_limit(&mut m, &t, FCC, &chan(HwBand::G2, 6)).unwrap();
    assert!(m.trace.iter().all(|op| matches!(op, Op::W32(..))));
    // v(nt, bw, rs, bf) = 30nt + 12bw + 4rs + 2bf + 1 after the halving.
    let v = |nt: i8, bw: i8, rs: i8, bf: i8| 30 * nt + 12 * bw + 4 * rs + 2 * bf + 1;
    let mut want = Vec::new();
    let mut addr = 0xD2EC;
    for nt in 0..2 {
        // cck_20m[bf], cck_40m[bf] (bw 40), ofdm[bf], mcs_20m[0][bf]; the
        // rest of the 40-byte page (phy.h:504-516) stays zero.
        let page: [u32; 10] = [
            le([
                v(nt, 0, 0, 0),
                v(nt, 0, 0, 1),
                v(nt, 1, 0, 0),
                v(nt, 1, 0, 1),
            ]),
            le([
                v(nt, 0, 1, 0),
                v(nt, 0, 1, 1),
                v(nt, 0, 2, 0),
                v(nt, 0, 2, 1),
            ]),
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];
        for w in page {
            want.push((addr, w));
            addr += 4;
        }
    }
    assert_eq!(addr, 0xD33C);
    assert_eq!(words(&m, 0, u32::MAX), want);
}

#[test]
fn limit_reads_uk_as_etsi() {
    let mut t = TxPowerTables::default();
    t.lmt_5ghz[0][0][1][0][usize::from(ETSI_U8)][0] = 40;
    t.lmt_5ghz[0][0][1][0][Regd::Uk as usize][0] = 10;
    let uk = Regds {
        g2: Regd::Uk,
        g5: Regd::Uk,
    };
    let mut m = RegisterModel::new();
    set_txpwr_limit(&mut m, &t, uk, &chan(HwBand::G5, 36)).unwrap();
    // ofdm[0] of ntx 0 is byte 4: the second word's [7:0].
    assert_eq!(words(&m, 0xD2F0, 0xD2F0), vec![(0xD2F0, 20)]);
}

#[test]
fn limit_ru_writes_twelve_dwords() {
    let t = limits();
    let mut m = RegisterModel::new();
    set_txpwr_limit_ru(&mut m, &t, FCC, &chan(HwBand::G2, 6)).unwrap();
    assert!(m.trace.iter().all(|op| matches!(op, Op::W32(..))));
    // v(nt, ru) = 20nt + 3ru + 1 at ru26[0], ru52[0] and ru106[0] of each
    // 24-byte page (phy.h:539-543).
    let v = |nt: u32, ru: u32| 20 * nt + 3 * ru + 1;
    let mut want = Vec::new();
    let mut addr = 0xD33C;
    for nt in 0..2 {
        for w in [v(nt, 0), 0, v(nt, 1), 0, v(nt, 2), 0] {
            want.push((addr, w));
            addr += 4;
        }
    }
    assert_eq!(addr, 0xD36C);
    assert_eq!(words(&m, 0, u32::MAX), want);
}

/// `rtw8852c_bb_set_tx_shape_dfir`'s sharp parameters and filter addresses
/// (rtw8852c.c:2150-2163), as MMIO.
const SHARP: [u32; 8] = [
    0x003D83FF, 0x002C636A, 0x0013F204, 0x00008090, 0x00F87FB0, 0x00F99F83, 0x00FDBFBA, 0x00003FF5,
];
const SHARP_14: [u32; 8] = [
    0x003B13FF, 0x001C42DE, 0x00FDB0AD, 0x00F60F6E, 0x00FD8F92, 0x0002D011, 0x0001C02C, 0x00FFF00A,
];
const FLAT: [u32; 8] = [
    0x003D23FF, 0x0029B354, 0x000FC1C8, 0x00FDB053, 0x00F86F9A, 0x00FAEF92, 0x00FE5FCC, 0x00FFDFF5,
];
const FILTER: [u32; 8] = [
    0x145BC, 0x145CC, 0x145D0, 0x145D4, 0x145D8, 0x145C0, 0x145C4, 0x145C8,
];

/// `rtw89_phy_tssi_ctrl_set_bandedge_cfg` (phy.c:8536-8566) from zeroed
/// registers for a level config: `rtw89_tssi_bandedge_regs_level` with the
/// `rtw89_8852c_tssi_dbw_table` row (rtw8852c_table.c:57132-57140), the
/// config in 0xD94C [31:30], then `rtw89_tssi_fastmode_regs_level` with
/// the row's SBW20 value.
fn bandedge_level(cfg: u32, row: [u32; 15]) -> Vec<Op> {
    let regs: [(u32, u32); 15] = [
        (0xD910, 0xFF),
        (0xD910, 0xFF00),
        (0xD910, 0xFF_0000),
        (0xD910, 0xFF00_0000),
        (0xD914, 0xFF),
        (0xD914, 0xFF00),
        (0xD914, 0xFF_0000),
        (0xD908, 0xFF),
        (0xD908, 0xFF00),
        (0xD908, 0xFF_0000),
        (0xD908, 0xFF00_0000),
        (0xD90C, 0xFF),
        (0xD90C, 0xFF00),
        (0xD90C, 0xFF_0000),
        (0xD90C, 0xFF00_0000),
    ];
    // Each masked write reads back what the one before it left.
    fn put(m: &mut RegisterModel, ops: &mut Vec<Op>, reg: u32, mask: u32, v: u32) {
        let old = m.get32(reg);
        let new = (old & !mask) | ((v << mask.trailing_zeros()) & mask);
        m.set32(reg, new);
        ops.extend(rmw(reg, new));
    }
    let mut m = RegisterModel::new();
    let mut ops = Vec::new();
    for ((reg, mask), v) in regs.into_iter().zip(row) {
        put(&mut m, &mut ops, reg, mask, v);
    }
    put(&mut m, &mut ops, 0xD94C, 0xC000_0000, cfg);
    for (reg, mask) in [
        (0xD930, 0xFF_0000),
        (0xD930, 0xFF00_0000),
        (0xD934, 0xFF),
        (0xD930, 0xFF00),
    ] {
        put(&mut m, &mut ops, reg, mask, row[0]);
    }
    ops
}

const HIGH: [u32; 15] = [
    0x2a, 0x2a, 0x2a, 0x46, 0x17, 0x17, 0x46, 0x53, 0x3b, 0x24, 0xc, 0xc, 0x24, 0x3b, 0x53,
];

fn shapes(cck: u8, ofdm: u8) -> TxPowerTables {
    let mut t = TxPowerTables::default();
    for band in 0..2 {
        t.tx_shape_lmt[band][0][usize::from(FCC_U8)][0] = cck;
        t.tx_shape_lmt[band][1][usize::from(FCC_U8)][0] = ofdm;
    }
    t
}

fn dfir(param: [u32; 8]) -> Vec<Op> {
    FILTER
        .into_iter()
        .zip(param)
        .flat_map(|(reg, v)| rmw(reg, v))
        .collect()
}

fn dpd_en() -> Vec<Op> {
    let mut ops = Vec::new();
    ops.extend(rmw(0x158F8, 0x8000_0000));
    ops.extend(rmw(0x178F8, 0x8000_0000));
    ops
}

#[test]
fn tx_shape_2g_sharp_writes_the_dfir_then_bandedge() {
    let t = shapes(1, 3);
    let mut m = RegisterModel::new();
    set_tx_shape(&mut m, &t, FCC, &chan(HwBand::G2, 6)).unwrap();
    let mut want = dfir(SHARP);
    want.extend(bandedge_level(3, HIGH));
    want.extend(dpd_en());
    assert_eq!(m.trace, want);

    // Channel 14 takes sharp_14 whatever the index; index 0 is flat.
    let mut m = RegisterModel::new();
    set_tx_shape(&mut m, &t, FCC, &chan(HwBand::G2, 14)).unwrap();
    assert_eq!(m.trace[..16], dfir(SHARP_14)[..]);
    let mut m = RegisterModel::new();
    set_tx_shape(&mut m, &shapes(0, 3), FCC, &chan(HwBand::G2, 1)).unwrap();
    assert_eq!(m.trace[..16], dfir(FLAT)[..]);
}

#[test]
fn tx_shape_5g_skips_the_dfir_and_an_unknown_bandedge() {
    // 5 GHz has no CCK filter; OFDM index 2 is the MID row.
    let mid = [
        0x24, 0x24, 0x24, 0x3b, 0x13, 0x13, 0x3b, 0x46, 0x32, 0x1e, 0xa, 0xa, 0x1e, 0x32, 0x46,
    ];
    let mut m = RegisterModel::new();
    set_tx_shape(&mut m, &shapes(1, 2), FCC, &chan(HwBand::G5, 36)).unwrap();
    let mut want = bandedge_level(2, mid);
    want.extend(dpd_en());
    assert_eq!(m.trace, want);

    // An index past RTW89_TSSI_CFG_NUM writes no bandedge (phy.c:8545-8546).
    let mut m = RegisterModel::new();
    set_tx_shape(&mut m, &shapes(1, 4), FCC, &chan(HwBand::G5, 36)).unwrap();
    assert_eq!(m.trace, dpd_en());
}

#[test]
fn sar_diff_writes_0x7e_on_both_paths() {
    let mut m = RegisterModel::new();
    m.set32(0x156CC, 0xFFFF_FFFF);
    set_txpwr_sar_diff(&mut m).unwrap();
    // mac_to_rf(RTW89_SAR_TXPWR_MAC_MAX) = 63 << 1 into [8:0].
    let mut want = Vec::new();
    want.extend(rmw(0x156CC, 0xFFFF_FE7E));
    want.extend(rmw(0x176CC, 0x7E));
    assert_eq!(m.trace, want);
}

#[test]
fn diff_is_set_txpwr_ref_at_zero() {
    let mut a = RegisterModel::new();
    set_txpwr_diff(&mut a).unwrap();
    let mut b = RegisterModel::new();
    crate::wifi::realtek::ax::dm::set_txpwr_ctrl(&mut b).unwrap();
    assert_eq!(a.trace, b.trace);
    assert_eq!(a.trace[0], Op::R32(0xD200));
}

#[test]
fn set_txpwr_runs_in_linux_order() {
    let mut t = limits();
    t.byr = distinct_byrate().byr;
    t.tx_shape_lmt = shapes(1, 3).tx_shape_lmt;
    let mut m = RegisterModel::new();
    set_txpwr(&mut m, &t, FCC, &chan(HwBand::G2, 6)).unwrap();
    let first = |want: Op| {
        m.trace
            .iter()
            .position(|&op| op == want)
            .unwrap_or_else(|| panic!("{want:?} missing"))
    };
    let order = [
        first(Op::W32(0xD2C0, le([1, 2, 3, 4]))),
        first(Op::R32(0xD204)),
        first(Op::R32(0x145BC)),
        first(Op::R32(0xD910)),
        first(Op::R32(0x158F8)),
        first(Op::W32(0xD2EC, le([1, 3, 13, 15]))),
        first(Op::W32(0xD33C, 1)),
        first(Op::R32(0xD200)),
        first(Op::R32(0x156CC)),
    ];
    assert!(order.windows(2).all(|w| w[0] < w[1]), "{order:?}");
    // byrate 11 + offset 2 + dfir 16 + bandedge 40 + dpd 4 + limit 20 +
    // limit_ru 12 + ref 10 + sar 4.
    assert_eq!(m.trace.len(), 11 + 2 + 16 + 40 + 4 + 20 + 12 + 10 + 4);
    assert_eq!(m.trace.last(), Some(&Op::W32(0x176CC, 0x7E)));
}

#[test]
fn a_channel_new20_would_not_build_is_refused_before_io() {
    let mut bad = chan(HwBand::G2, 6);
    bad.channel = 15;
    let mut m = RegisterModel::new();
    let err = set_txpwr(&mut m, &TxPowerTables::default(), FCC, &bad);
    assert_eq!(err, Err(AxError::Phy("not a 20 MHz 2.4 or 5 GHz channel")));
    assert!(m.trace.is_empty());
}

#[test]
fn an_io_error_stops_the_sequence() {
    let mut m = RegisterModel::new();
    m.fail_reads_of(0xD204);
    let err = set_txpwr(&mut m, &TxPowerTables::default(), FCC, &chan(HwBand::G2, 1));
    assert!(err.is_err());
    assert_eq!(m.trace.last(), Some(&Op::R32(0xD204)));
    assert!(!m.trace.iter().any(|op| matches!(op, Op::R32(0x145BC))));
}

/// The MAC words `rtw8852c_set_txpwr` writes on 2.4 GHz channel 1 under FCC
/// from the pinned file's rfe 0 elements (rfe 1 takes them), recomputed
/// from the raw element bytes by a script independent of this crate
/// (fw.c entry rules, phy.c:2493-2646 and :3069-3222): by-rate
/// 0xD2C0-0xD2E8, limit 0xD2EC-0xD338, RU limit 0xD33C-0xD368.
const REAL_2G_CH1_FCC: [(u32, u32); 43] = [
    (0xD2C0, 0x28282828),
    (0xD2C4, 0x28282828),
    (0xD2C8, 0x24262828),
    (0xD2CC, 0x28282828),
    (0xD2D0, 0x22242628),
    (0xD2D4, 0x1A1C1E20),
    (0xD2D8, 0x28282828),
    (0xD2DC, 0x28282828),
    (0xD2E0, 0x22242628),
    (0xD2E4, 0x1A1C1E20),
    (0xD2E8, 0x28282828),
    (0xD2EC, 0x003F0026),
    (0xD2F0, 0x00210021),
    (0xD2F4, 0),
    (0xD2F8, 0),
    (0xD2FC, 0),
    (0xD300, 0),
    (0xD304, 0),
    (0xD308, 0),
    (0xD30C, 0),
    (0xD310, 0),
    (0xD314, 0x003F0026),
    (0xD318, 0x20200021),
    (0xD31C, 0),
    (0xD320, 0),
    (0xD324, 0),
    (0xD328, 0),
    (0xD32C, 0),
    (0xD330, 0),
    (0xD334, 0),
    (0xD338, 0),
    (0xD33C, 0x1E),
    (0xD340, 0),
    (0xD344, 0x21),
    (0xD348, 0),
    (0xD34C, 0x23),
    (0xD350, 0),
    (0xD354, 0x1C),
    (0xD358, 0),
    (0xD35C, 0x1D),
    (0xD360, 0),
    (0xD364, 0x20),
    (0xD368, 0),
];

#[test]
#[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES"]
fn real_8852c_fw_2_txpwr_2g_ch1_us() {
    use crate::wifi::realtek::ax::{regd::select, txpwr_tables::tx_power_tables};
    use sha2::{Digest, Sha256};
    let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
    let b = std::fs::read(root.join("rtw89/rtw8852c_fw-2.bin")).unwrap();
    assert_eq!(b.len(), 2_644_326);
    let t = tx_power_tables(&b, 0, 1).unwrap();
    let (regds, alpha2) = select(t.regd_map(), Some(*b"US"), None);
    assert_eq!((regds, alpha2), (FCC, *b"US"));
    let mut m = RegisterModel::new();
    set_txpwr(&mut m, &t, regds, &chan(HwBand::G2, 1)).unwrap();

    assert_eq!(words(&m, 0xD2C0, 0xD368), REAL_2G_CH1_FCC);
    // The by-rate offsets are all 0, so [19:0] of 0xD204 is cleared.
    assert_eq!(m.trace[11..13], rmw(0xD204, 0));
    // TX shape: CCK index 1 (sharp DFIR) and OFDM index 3 (the HIGH
    // bandedge row).
    let mut shape = dfir(SHARP);
    shape.extend(bandedge_level(3, HIGH));
    shape.extend(dpd_en());
    assert_eq!(m.trace[13..13 + shape.len()], shape[..]);

    // The whole trace, one line per access ("R32 0x0000D204" or
    // "W32 0x0000D204 0x00000000"), pinned by its SHA-256.
    let text: std::string::String = m
        .trace
        .iter()
        .map(|op| match *op {
            Op::R32(reg) => std::format!("R32 0x{reg:08X}\n"),
            Op::W32(reg, v) => std::format!("W32 0x{reg:08X} 0x{v:08X}\n"),
            other => panic!("unexpected access {other:?}"),
        })
        .collect();
    assert_eq!(m.trace.len(), 119);
    let hex: std::string::String = Sha256::digest(text.as_bytes())
        .iter()
        .map(|x| std::format!("{x:02x}"))
        .collect();
    assert_eq!(
        hex,
        "82f54cc1bc88773b6ef8a6da68b2f5bc6078c47614e5797109ee13d153ad9772"
    );
}

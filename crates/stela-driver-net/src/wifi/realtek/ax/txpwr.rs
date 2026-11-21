// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
//! An RTL8852C's TX power for a 20 MHz 2.4 or 5 GHz [`Chan`] on PHY 0, as
//! `rtw8852c_set_txpwr` (pinned rtw89/rtw8852c.c:2248-2259) programs it
//! after each channel switch (`__rtw89_set_channel`, core.c:584-588), from
//! the firmware's tables ([`super::txpwr_tables`]) at the regulatory index
//! of [`super::regd`]. [`set_txpwr`] runs, in Linux's order:
//! 1. by rate: `rtw89_phy_set_txpwr_byrate_ax` (phy.c:3069-3123), each
//!    value read by `rtw89_phy_read_txpwr_byrate` (phy.c:2495-2507) from
//!    the 20 MHz table, CCK always from 2.4 GHz;
//! 2. the rate offsets: `rtw89_phy_set_txpwr_offset_ax` (phy.c:3125-3152);
//! 3. the TX shape: `rtw8852c_set_tx_shape` (rtw8852c.c:2192-2211) with the
//!    CCK filter of `rtw8852c_bb_set_tx_shape_dfir` (:2137-2190) on
//!    2.4 GHz and `rtw89_phy_tssi_ctrl_set_bandedge_cfg` (phy.c:8536-8566,
//!    [`super::bb`]), at the indices `rtw89_get_tx_shape_idx` reads
//!    (core.h:8935-8957);
//! 4. the limit: `rtw89_phy_set_txpwr_limit_ax` (phy.c:3154-3187) with
//!    `rtw89_phy_fill_txpwr_limit_ax` (:2824-2853) at 20 MHz (:2660-2673),
//!    each value read by `rtw89_phy_read_txpwr_limit` (:2570-2646, as
//!    [`read_limit`]) at `rtw89_channel_to_idx` (:2535-2553, [`ch_idx`]);
//! 5. the RU limit: `rtw89_phy_set_txpwr_limit_ru_ax` (:3189-3222) with
//!    `rtw89_phy_fill_txpwr_limit_ru_ax` (:3037-3066) at 20 MHz
//!    (:2928-2942), read by `rtw89_phy_read_txpwr_limit_ru` (:2855-2926);
//! 6. the path difference: `rtw8852c_set_txpwr_diff` (rtw8852c.c:2213-2221),
//!    `rtw8852c_set_txpwr_ref` at offset 0 ([`super::dm::set_txpwr_ctrl`]);
//! 7. the SAR ceiling: `rtw8852c_set_txpwr_sar_diff` (:2223-2246).
//!
//! Units follow `rtw8852c_chip_info` (`txpwr_factor_rf = 2`, `_mac = 1`,
//! :3185-3187): tables hold 0.25 dB and MAC words 0.5 dB, so
//! `rtw89_phy_txpwr_rf_to_mac` (phy.h:1111-1116) is an arithmetic `>> 1`
//! with no clamp. `rtw89_query_sar` (sar.c:274-292) has no SAR source
//! without ACPI and returns `RTW89_SAR_TXPWR_MAC_MAX` (63); the 6 GHz TPE
//! constraint is `rtw89_phy_txpwr_dbm_to_mac(S8_MAX)`, also 63; and
//! `rtw89_can_apply_ant_gain` is false without ACPI, so no DA limit or
//! antenna-gain offset applies. A limit is therefore
//! `min(rf_to_mac(v), 63)`. The MAC TX power words are inside the window
//! `rtw89_mac_get_txpwr_cr_ax` (mac.c:6258-6287) accepts for PHY 0, so
//! `rtw89_mac_txpwr_write32` is a plain write and `_write32_mask` a
//! read-modify-write (mac.h:1588-1614); every BB write is
//! `rtw89_phy_write32_mask`, `_idx` on PHY 0 or `_set`, each a
//! read-modify-write.
//!
//! This module grants no 40, 80 or 160 MHz limit page, no 6 GHz, no PHY 1,
//! no SAR, TAS, DA or antenna-gain source, no TSSI or TX power tracking,
//! and no channel switch; it programs the TX power registers of the
//! channel it is given and transmits nothing.
use super::{
    bb,
    chan::{Chan, HwBand},
    dm,
    io::{self, AxError, AxIo},
    regd::{Regd, Regds},
    txpwr_tables::{
        BW_NUM_2G, BW_NUM_5G, ByRate, RATE_CCK_NUM, RATE_HEDCM_NUM, RATE_OFDM_NUM, RS_CCK,
        RS_HEDCM, RS_MCS, RS_OFDM, RS_OFFSET, TxPowerTables,
    },
};

/// `rtw8852c_chip_info.rf_path_num` (rtw8852c.c:3215): the NSS and NTX
/// counts the by-rate and limit writes loop over.
const RF_PATH_NUM: u8 = 2;

// reg.h (pinned rtw89): MAC TX power words.
const R_AX_PWR_RATE_OFST_CTRL: u32 = 0xD204;
/// GENMASK(19, 0): five nibbles, one per `rtw89_rate_offset_indexes` entry
/// up to CCK (core.h:720-732).
const PWR_RATE_OFST_MASK: u32 = 0x000F_FFFF;
const R_AX_PWR_BY_RATE: u32 = 0xD2C0;
const R_AX_PWR_LMT: u32 = 0xD2EC;
const R_AX_PWR_RU_LMT: u32 = 0xD33C;

// reg.h (pinned rtw89): BB offsets.
const R_P0_DAC_COMP_POST_DPD_EN: u32 = 0x58F8;
const R_P1_DAC_COMP_POST_DPD_EN: u32 = 0x78F8;
/// `B_P0_DAC_COMP_POST_DPD_EN` and `B_P1_..`: BIT(31).
const B_DAC_COMP_POST_DPD_EN: u32 = 1 << 31;
const R_TXPWRB: u32 = 0x56CC;
const R_P1_TXPWRB: u32 = 0x76CC;
/// `B_TXPWRB_MAX`: GENMASK(8, 0).
const B_TXPWRB_MAX: u32 = 0x1FF;

/// `RTW89_RATE_MCS_NUM_AX` and `RTW89_RATE_OFFSET_NUM_AX` (core.h:720-741).
const RATE_MCS_NUM_AX: usize = 12;
const RATE_OFFSET_NUM_AX: usize = 5;
/// `RTW89_NSS_NUM` and `RTW89_NSS_HEDCM_NUM`.
const NSS_NUM: u8 = 4;
const NSS_HEDCM_NUM: u8 = 2;

/// `RTW89_CHANNEL_WIDTH_20` and `_40` as limit table bandwidth indices.
const BW_20: u8 = 0;
const BW_40: u8 = 1;
/// `RTW89_BF_NUM`.
const BF_NUM: u8 = 2;
/// `RTW89_RU26`, `RTW89_RU52` and `RTW89_RU106` (core.h:998-1001).
const RU26: u8 = 0;
const RU52: u8 = 1;
const RU106: u8 = 2;

/// `RTW89_TXPWR_LMT_PAGE_SIZE_AX`: one `struct rtw89_txpwr_limit_ax`
/// (phy.h:504-516), whose 20 MHz fields are `cck_20m[2]` at 0,
/// `cck_40m[2]` at 2, `ofdm[2]` at 4 and `mcs_20m[0][2]` at 6.
const LMT_PAGE_SIZE: usize = 40;
const LMT_CCK_20M: usize = 0;
const LMT_CCK_40M: usize = 2;
const LMT_OFDM: usize = 4;
const LMT_MCS_20M_0: usize = 6;
/// `RTW89_TXPWR_LMT_RU_PAGE_SIZE_AX`: one `struct rtw89_txpwr_limit_ru_ax`
/// (phy.h:535-543), `ru26[8]`, `ru52[8]` and `ru106[8]`.
const LMT_RU_PAGE_SIZE: usize = 24;
const RU_SEC_NUM_AX: usize = 8;

/// `RTW89_SAR_TXPWR_MAC_MAX` (sar.h:10): `rtw89_query_sar` without a SAR
/// source (sar.c:285-286).
const SAR_TXPWR_MAC_MAX: i8 = 63;
/// `rtw89_phy_get_tpe_constraint` off 6 GHz: `dbm_to_mac(S8_MAX)` clamped
/// to 63 (phy.c:2483-2493, phy.h:1118-1123).
const TPE_CONSTRAINT: i8 = 63;

/// `rtw8852c_bb_set_tx_shape_dfir`'s parameters and filter addresses
/// (rtw8852c.c:2150-2163), each written to [23:0] (`__DFIR_CFG_MASK`).
const DFIR_FLAT: [u32; 8] = [
    0x003D_23FF,
    0x0029_B354,
    0x000F_C1C8,
    0x00FD_B053,
    0x00F8_6F9A,
    0x00FA_EF92,
    0x00FE_5FCC,
    0x00FF_DFF5,
];
const DFIR_SHARP: [u32; 8] = [
    0x003D_83FF,
    0x002C_636A,
    0x0013_F204,
    0x0000_8090,
    0x00F8_7FB0,
    0x00F9_9F83,
    0x00FD_BFBA,
    0x0000_3FF5,
];
const DFIR_SHARP_14: [u32; 8] = [
    0x003B_13FF,
    0x001C_42DE,
    0x00FD_B0AD,
    0x00F6_0F6E,
    0x00FD_8F92,
    0x0002_D011,
    0x0001_C02C,
    0x00FF_F00A,
];
const DFIR_FILTER: [u32; 8] = [
    0x45BC, 0x45CC, 0x45D0, 0x45D4, 0x45D8, 0x45C0, 0x45C4, 0x45C8,
];
const DFIR_CFG_MASK: u32 = 0x00FF_FFFF;

/// `rtw89_channel_to_idx` (phy.c:2535-2553) for a 2.4 or 5 GHz channel:
/// 2.4 GHz `ch - 1`; 5 GHz `(ch - 36) / 2` for 36-64, `(ch - 100) / 2 + 15`
/// for 100-144 and `(ch - 149) / 2 + 38` for 149-177. Any other channel is
/// 0, as Linux returns after its warning; 6 GHz is not supported and reads
/// 0 too.
pub fn ch_idx(chan: &Chan) -> usize {
    let ch = usize::from(chan.channel);
    match (chan.band, ch) {
        (HwBand::G6, _) => 0,
        (_, 1..=14) => ch - 1,
        (_, 36..=64) => (ch - 36) / 2,
        (_, 100..=144) => (ch - 100) / 2 + 15,
        (_, 149..=177) => (ch - 149) / 2 + 38,
        _ => 0,
    }
}

/// `rtw89_phy_txpwr_rf_to_mac` (phy.h:1111-1116): 0.25 dB to 0.5 dB.
const fn rf_to_mac(v: i8) -> i8 {
    v >> 1
}

/// `rtw89_phy_txpwr_mac_to_rf` (phy.h:1125-1130): 0.5 dB to 0.25 dB.
const fn mac_to_rf(v: i8) -> i16 {
    (v as i16) << 1
}

/// `rtw89_phy_read_txpwr_limit` (phy.c:2570-2646) for a 2.4 or 5 GHz
/// table at `ch_idx` (see [`ch_idx`]), with `regd` the index
/// `rtw89_regd_get` gives ([`Regds::txpwr`]): the regd's value, or WW's when
/// it is 0, in MAC units and capped at 63 by the SAR and TPE ceilings. A
/// bandwidth past the band's table reads 0, as Linux returns before the
/// table (phy.c:2593-2596).
#[allow(clippy::too_many_arguments)] // One argument per table index.
pub fn read_limit(
    t: &TxPowerTables,
    regd: Regd,
    band: HwBand,
    bw: u8,
    nt: u8,
    rs: u8,
    bf: u8,
    ch_idx: usize,
) -> i8 {
    let bw_ok = match band {
        HwBand::G2 => usize::from(bw) < BW_NUM_2G,
        HwBand::G5 => usize::from(bw) < BW_NUM_5G,
        HwBand::G6 => false,
    };
    if !bw_ok {
        return 0;
    }
    let lmt = match t.lmt(band, bw, nt, rs, bf, regd as u8, ch_idx) {
        0 => t.lmt(band, bw, nt, rs, bf, Regd::Ww as u8, ch_idx),
        v => v,
    };
    ceiling(lmt)
}

/// `rtw89_phy_read_txpwr_limit_ru` (phy.c:2855-2926), as [`read_limit`]
/// reads a limit.
pub fn read_limit_ru(
    t: &TxPowerTables,
    regd: Regd,
    band: HwBand,
    ru: u8,
    nt: u8,
    ch_idx: usize,
) -> i8 {
    let lmt_ru = match t.lmt_ru(band, ru, nt, regd as u8, ch_idx) {
        0 => t.lmt_ru(band, ru, nt, Regd::Ww as u8, ch_idx),
        v => v,
    };
    ceiling(lmt_ru)
}

/// `min3(rf_to_mac(v), sar, cstr)` with no SAR source and no TPE.
fn ceiling(v: i8) -> i8 {
    rf_to_mac(v).min(SAR_TXPWR_MAC_MAX).min(TPE_CONSTRAINT)
}

/// Four signed bytes as `FIELD_PREP(GENMASK(7, 0), v[0]) | ... |
/// FIELD_PREP(GENMASK(31, 24), v[3])`: each s8 cut to its byte.
fn pack(v: &[i8]) -> u32 {
    u32::from_le_bytes([v[0] as u8, v[1] as u8, v[2] as u8, v[3] as u8])
}

/// `rtw89_phy_read_txpwr_byrate(band, 0, desc)` (phy.c:2495-2507) with
/// `ofdma = 0`: CCK reads the 2.4 GHz table whatever the band, through
/// `rtw89_phy_raw_byr_seek` (phy.c:2425-2444).
fn read_byrate(t: &TxPowerTables, band: HwBand, rs: u8, nss: u8, idx: usize) -> i8 {
    let band = if rs == RS_CCK { HwBand::G2 } else { band };
    let head: &ByRate = t.byrate(band);
    let nss = usize::from(nss);
    let v = match rs {
        RS_CCK => head.cck[idx],
        RS_OFDM => head.ofdm[idx],
        RS_MCS => head.mcs[0][nss][idx],
        RS_HEDCM => head.hedcm[0][nss][idx],
        _ => head.offset[idx],
    };
    rf_to_mac(v)
}

/// `rtw89_rs_nss_num_ax` and `rtw89_rs_idx_num_ax` (phy.c:2409-2423) for
/// the rate sections the by-rate words cover.
const BYRATE_SECTIONS: [(u8, u8, usize); 4] = [
    (RS_CCK, 1, RATE_CCK_NUM),
    (RS_OFDM, 1, RATE_OFDM_NUM),
    (RS_MCS, NSS_NUM, RATE_MCS_NUM_AX),
    (RS_HEDCM, NSS_HEDCM_NUM, RATE_HEDCM_NUM),
];

/// `rtw89_phy_set_txpwr_byrate_ax` (phy.c:3069-3123): for NSS 0 and 1, the
/// CCK, OFDM, MCS 0-11 and HE DCM sections the NSS has, four values a
/// word, `v[0]` in [7:0], from 0xD2C0 up in 4-byte steps (11 words).
fn set_txpwr_byrate<I: AxIo + ?Sized>(
    io: &mut I,
    t: &TxPowerTables,
    chan: &Chan,
) -> Result<(), AxError> {
    let mut addr = R_AX_PWR_BY_RATE;
    for nss in 0..RF_PATH_NUM {
        for (rs, nss_num, idx_num) in BYRATE_SECTIONS {
            if nss >= nss_num {
                continue;
            }
            let mut v = [0i8; 4];
            for idx in 0..idx_num {
                v[idx % 4] = read_byrate(t, chan.band, rs, nss, idx);
                if (idx + 1) % 4 != 0 {
                    continue;
                }
                io.write32(addr, pack(&v))?;
                addr += 4;
            }
        }
    }
    Ok(())
}

/// `rtw89_phy_set_txpwr_offset_ax` (phy.c:3125-3152): the band's HE, VHT,
/// HT, OFDM and CCK offsets as nibbles [3:0] to [19:16] of 0xD204, a
/// read-modify-write of [19:0].
fn set_txpwr_offset<I: AxIo + ?Sized>(
    io: &mut I,
    t: &TxPowerTables,
    chan: &Chan,
) -> Result<(), AxError> {
    let mut val = 0;
    for idx in 0..RATE_OFFSET_NUM_AX {
        let v = read_byrate(t, chan.band, RS_OFFSET, 0, idx);
        val |= (u32::from(v as u8) & 0xF) << (4 * idx);
    }
    io::field32(io, R_AX_PWR_RATE_OFST_CTRL, PWR_RATE_OFST_MASK, val)
}

/// `rtw8852c_bb_set_tx_shape_dfir` (rtw8852c.c:2137-2190): the CCK filter
/// of a 2.4 GHz channel, sharp_14 on channel 14, else flat for index 0 and
/// sharp for any other. A channel above 14 writes nothing, as Linux warns
/// and returns.
fn set_tx_shape_dfir<I: AxIo + ?Sized>(
    io: &mut I,
    chan: &Chan,
    tx_shape_idx: u8,
) -> Result<(), AxError> {
    let param = match chan.channel {
        15.. => return Ok(()),
        14 => &DFIR_SHARP_14,
        _ if tx_shape_idx == 0 => &DFIR_FLAT,
        _ => &DFIR_SHARP,
    };
    for (&addr, &v) in DFIR_FILTER.iter().zip(param) {
        bb::field32(io, addr, DFIR_CFG_MASK, v)?;
    }
    Ok(())
}

/// `rtw8852c_set_tx_shape` (rtw8852c.c:2192-2211): on 2.4 GHz the CCK
/// filter for the CCK index, then the TSSI bandedge for the OFDM index,
/// then the post-DPD DAC compensation enabled on both paths.
fn set_tx_shape<I: AxIo + ?Sized>(
    io: &mut I,
    t: &TxPowerTables,
    regds: Regds,
    chan: &Chan,
) -> Result<(), AxError> {
    let regd = regds.txpwr(chan.band) as u8;
    let tx_shape_cck = t.tx_shape(chan.band, RS_CCK, regd);
    let tx_shape_ofdm = t.tx_shape(chan.band, RS_OFDM, regd);
    if chan.band == HwBand::G2 {
        set_tx_shape_dfir(io, chan, tx_shape_cck)?;
    }
    bb::tssi_ctrl_set_bandedge_cfg(io, tx_shape_ofdm)?;
    bb::set32(io, R_P0_DAC_COMP_POST_DPD_EN, B_DAC_COMP_POST_DPD_EN)?;
    bb::set32(io, R_P1_DAC_COMP_POST_DPD_EN, B_DAC_COMP_POST_DPD_EN)
}

/// `rtw89_phy_set_txpwr_limit_ax` (phy.c:3154-3187): for NTX 0 and 1, the
/// 40-byte limit page `rtw89_phy_fill_txpwr_limit_20m_ax` fills over zero
/// (phy.c:2660-2673, :2834), four bytes a word, from 0xD2EC up (20 words).
/// At 20 MHz only `cck_20m`, `cck_40m` (the 40 MHz CCK limit on the same
/// channel), `ofdm` and `mcs_20m[0]` are filled, each for non-BF and BF.
fn set_txpwr_limit<I: AxIo + ?Sized>(
    io: &mut I,
    t: &TxPowerTables,
    regds: Regds,
    chan: &Chan,
) -> Result<(), AxError> {
    let regd = regds.txpwr(chan.band);
    let ch = ch_idx(chan);
    let mut addr = R_AX_PWR_LMT;
    for ntx in 0..RF_PATH_NUM {
        let mut lmt = [0i8; LMT_PAGE_SIZE];
        for (at, bw, rs) in [
            (LMT_CCK_20M, BW_20, RS_CCK),
            (LMT_CCK_40M, BW_40, RS_CCK),
            (LMT_OFDM, BW_20, RS_OFDM),
            (LMT_MCS_20M_0, BW_20, RS_MCS),
        ] {
            for bf in 0..BF_NUM {
                lmt[at + usize::from(bf)] = read_limit(t, regd, chan.band, bw, ntx, rs, bf, ch);
            }
        }
        for word in lmt.chunks(4) {
            io.write32(addr, pack(word))?;
            addr += 4;
        }
    }
    Ok(())
}

/// `rtw89_phy_set_txpwr_limit_ru_ax` (phy.c:3189-3222): for NTX 0 and 1,
/// the 24-byte RU limit page `rtw89_phy_fill_txpwr_limit_ru_20m_ax` fills
/// over zero (phy.c:2928-2942, :3046): `ru26[0]`, `ru52[0]` and `ru106[0]`,
/// four bytes a word, from 0xD33C up (12 words).
fn set_txpwr_limit_ru<I: AxIo + ?Sized>(
    io: &mut I,
    t: &TxPowerTables,
    regds: Regds,
    chan: &Chan,
) -> Result<(), AxError> {
    let regd = regds.txpwr(chan.band);
    let ch = ch_idx(chan);
    let mut addr = R_AX_PWR_RU_LMT;
    for ntx in 0..RF_PATH_NUM {
        let mut lmt_ru = [0i8; LMT_RU_PAGE_SIZE];
        for (section, ru) in [RU26, RU52, RU106].into_iter().enumerate() {
            lmt_ru[section * RU_SEC_NUM_AX] = read_limit_ru(t, regd, chan.band, ru, ntx, ch);
        }
        for word in lmt_ru.chunks(4) {
            io.write32(addr, pack(word))?;
            addr += 4;
        }
    }
    Ok(())
}

/// `rtw8852c_set_txpwr_diff` (rtw8852c.c:2213-2221):
/// `rtw89_phy_ant_gain_pwr_offset` is 0 without ACPI (phy.c:2370-2386), so
/// this is `rtw8852c_set_txpwr_ref(PHY0, 0)`, the writes of
/// [`dm::set_txpwr_ctrl`].
fn set_txpwr_diff<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    dm::set_txpwr_ctrl(io)
}

/// `rtw8852c_set_txpwr_sar_diff` (rtw8852c.c:2223-2246) on PHY 0: each
/// path's TX power ceiling, `mac_to_rf(rtw89_query_sar)` with no SAR source
/// (0x7E), into BB 0x56CC and 0x76CC [8:0].
fn set_txpwr_sar_diff<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    for reg in [R_TXPWRB, R_P1_TXPWRB] {
        let sar_rf = mac_to_rf(SAR_TXPWR_MAC_MAX);
        bb::field32(io, reg, B_TXPWRB_MAX, sar_rf as u32)?;
    }
    Ok(())
}

/// `rtw8852c_set_txpwr` (rtw8852c.c:2248-2259) on PHY 0 for `chan`, from
/// `t` at the regulatory indices `regds`: by rate, offsets, TX shape,
/// limit, RU limit, path difference and SAR ceiling, in that order.
///
/// A channel [`Chan::new20`] would not build is refused before any I/O.
/// Linux ignores the TX power writes' returns, which fail only for a word
/// outside the PHY 0 window; here an I/O error stops the sequence where it
/// happened.
pub fn set_txpwr<I: AxIo + ?Sized>(
    io: &mut I,
    t: &TxPowerTables,
    regds: Regds,
    chan: &Chan,
) -> Result<(), AxError> {
    if Chan::new20(chan.band, chan.channel).ok() != Some(*chan) {
        return Err(AxError::Phy("not a 20 MHz 2.4 or 5 GHz channel"));
    }
    set_txpwr_byrate(io, t, chan)?;
    set_txpwr_offset(io, t, chan)?;
    set_tx_shape(io, t, regds, chan)?;
    set_txpwr_limit(io, t, regds, chan)?;
    set_txpwr_limit_ru(io, t, regds, chan)?;
    set_txpwr_diff(io)?;
    set_txpwr_sar_diff(io)
}

#[cfg(test)]
mod tests;

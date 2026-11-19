// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! Baseband (BB) register access for an AX-generation rtw89 chip, as the
//! `rtw89_phy_read32*`/`rtw89_phy_write32*` accessors (pinned rtw89/phy.h:
//! 712-790) reach it: every BB offset is MMIO `offset + CR_BASE`, with
//! `rtw89_phy_gen_ax.cr_base = 0x10000` (phy.c:9042). The masked helpers
//! are `rtw89_write32_mask`/`_set`/`_clr` and `rtw89_read32_mask`
//! (core.h:8185-8235) over that address; the PHY write goes straight to
//! MMIO, as `rtw89_raw_phy_write32` does (fw.c:12690).
//!
//! On top of the accessors, [`init_bb_reg`] is `rtw89_phy_init_bb_reg`
//! (phy.c:1885-1911) for an RTL8852C's PHY 0 without DBCC: the firmware's
//! BB table through [`config_bb_reg`] (`rtw89_phy_config_bb_reg`,
//! phy.c:1347-1376), `rtw8852c_init_txpwr_unit` (rtw8852c.c:2293-2316),
//! the BB gain table decoded into [`BbGain`] (`rtw89_phy_config_bb_gain_ax`,
//! phy.c:1577-1621, and its helpers from phy.c:1394) and
//! `rtw8852c_bb_reset_all` (rtw8852c.c:1648-1671). `io_pack`/`unpack` do
//! nothing on PCIe (fw.c:12682-12692) and are not ported.
//!
//! This module grants no PHY 1, no RF table, no channel, TX power by rate
//! or limit, no gain-error write (the gain table is memory only on AX) and
//! no calibration.
use super::elements::Reg2;
use super::io::{self, AxError, AxIo};
use super::phy_table::{self, BB_GAIN, BB_REG};

/// `rtw89_phy_gen_ax.cr_base` (phy.c:9042).
pub const CR_BASE: u32 = 0x10000;

/// The MMIO address of BB `offset`, refused if it would wrap.
fn at(offset: u32) -> Result<u32, AxError> {
    offset
        .checked_add(CR_BASE)
        .ok_or(AxError::OutOfWindow { register: offset })
}

/// `rtw89_phy_read32`.
pub fn read32<I: AxIo + ?Sized>(io: &mut I, offset: u32) -> Result<u32, AxError> {
    io.read32(at(offset)?)
}

/// `rtw89_phy_write32`: a plain write, with no read first.
pub fn write32<I: AxIo + ?Sized>(io: &mut I, offset: u32, v: u32) -> Result<(), AxError> {
    io.write32(at(offset)?, v)
}

/// `rtw89_phy_write32_mask`: a read-modify-write of the `mask` field, even
/// when the mask is all ones, with `v` shifted to the mask's lowest bit.
pub fn field32<I: AxIo + ?Sized>(
    io: &mut I,
    offset: u32,
    mask: u32,
    v: u32,
) -> Result<(), AxError> {
    io::field32(io, at(offset)?, mask, v)
}

/// `rtw89_phy_write32_set`.
pub fn set32<I: AxIo + ?Sized>(io: &mut I, offset: u32, bits: u32) -> Result<(), AxError> {
    io::set32(io, at(offset)?, bits)
}

/// `rtw89_phy_write32_clr`.
pub fn clear32<I: AxIo + ?Sized>(io: &mut I, offset: u32, bits: u32) -> Result<(), AxError> {
    io::clear32(io, at(offset)?, bits)
}

/// `rtw89_phy_read32_mask`: `(v & mask) >> ffs(mask)`. An empty mask, for
/// which Linux's `__ffs` is undefined, reads 0.
pub fn read32_mask<I: AxIo + ?Sized>(io: &mut I, offset: u32, mask: u32) -> Result<u32, AxError> {
    let v = read32(io, offset)?;
    Ok((v & mask).checked_shr(mask.trailing_zeros()).unwrap_or(0))
}

/// `BYPASS_CR_DATA` (core.h:50): a table entry that writes nothing.
const BYPASS_CR_DATA: u32 = 0xBABE_CAFE;

/// `rtw89_phy_config_bb_reg` (phy.c:1347-1376) for PHY 0: addresses 0xFE
/// to 0xF9 are delays of 50, 5 and 1 ms and 50, 5 and 1 µs
/// (`rtw89_io_mdelay`/`udelay`, which are plain `mdelay`/`udelay` on PCIe);
/// otherwise an entry whose data is `BYPASS_CR_DATA` is skipped, and any
/// other is a plain BB write of `data`, with no read.
pub fn config_bb_reg<I: AxIo + ?Sized>(io: &mut I, addr: u32, data: u32) -> Result<(), AxError> {
    match addr {
        0xFE => io.delay_us(50_000),
        0xFD => io.delay_us(5_000),
        0xFC => io.delay_us(1_000),
        0xFB => io.delay_us(50),
        0xFA => io.delay_us(5),
        0xF9 => io.delay_us(1),
        _ if data == BYPASS_CR_DATA => {}
        _ => write32(io, addr, data)?,
    }
    Ok(())
}

// reg.h (pinned rtw89): the TX power unit registers are MAC addresses,
// which `rtw89_mac_txpwr_write32` (mac.h:1588-1600) accepts for PHY 0 from
// R_AX_PWR_RATE_CTRL (0xD200) up and writes plainly.
const R_AX_PWR_COEXT_CTRL: u32 = 0xD220;
const R_AX_PWR_UL_CTRL0: u32 = 0xD240;
const R_AX_PWR_UL_CTRL2: u32 = 0xD248;
const R_AX_PWR_UL_TB_1T: u32 = 0xD28C;
const R_AX_PWR_UL_TB_2T: u32 = 0xD290;
/// `B_AX_PWR_UL_TB_1T_V1_MASK` and `_2T_V1_MASK`: GENMASK(7, 0).
const B_AX_PWR_UL_TB_V1_MASK: u32 = 0xFF;
const R_AX_TSSI_CTRL_HEAD: u32 = 0xD908;
const R_AX_TSSI_CTRL_TAIL: u32 = 0xD95C;
const R_AX_BANDEDGE_CFG: u32 = 0xD94C;
const B_AX_BANDEDGE_CFG_IDX_MASK: u32 = 0xC000_0000; // GENMASK(31, 30)

/// `rtw8852c_init_tssi_ctrl`'s `ctrl_ini` (rtw8852c.c:2270-2277).
const TSSI_CTRL_INI: [(u32, u32); 6] = [
    (0xD938, 0x0001_0100),
    (0xD93C, 0x0500_D500),
    (0xD940, 0x0000_0500),
    (0xD944, 0x0000_0005),
    (0xD94C, 0x0022_0000),
    (0xD950, 0x0003_0000),
];
/// `rtw89_tssi_bandedge_regs_flat` (phy.c:8500-8516), in order.
const TSSI_BANDEDGE_FLAT: [(u32, u32); 15] = [
    (0xD91C, 0xFF00_0000),
    (0xD920, 0x0000_00FF),
    (0xD920, 0x0000_FF00),
    (0xD920, 0x00FF_0000),
    (0xD920, 0xFF00_0000),
    (0xD924, 0x0000_00FF),
    (0xD924, 0x0000_FF00),
    (0xD914, 0xFF00_0000),
    (0xD918, 0x0000_00FF),
    (0xD918, 0x0000_FF00),
    (0xD918, 0x00FF_0000),
    (0xD918, 0xFF00_0000),
    (0xD91C, 0x0000_00FF),
    (0xD91C, 0x0000_FF00),
    (0xD91C, 0x00FF_0000),
];
/// `rtw89_tssi_fastmode_regs_flat` (phy.c:8465-8470), in order.
const TSSI_FAST_MODE_FLAT: [(u32, u32); 4] = [
    (0xD934, 0x00FF_0000),
    (0xD934, 0xFF00_0000),
    (0xD938, 0x0000_00FF),
    (0xD934, 0x0000_FF00),
];
/// `RTW89_TSSI_BANDEDGE_FLAT` (phy.h:319), whose `rtw89_8852c_tssi_dbw_table`
/// row is all zeros (rtw8852c_table.c:57133).
const TSSI_BANDEDGE_FLAT_CFG: u32 = 0;
const TSSI_DBW_FLAT: u32 = 0;
/// The `pw_ofst` `rtw8852c_init_txpwr_unit` passes (rtw8852c.c:2310).
const UL_TB_PW_OFST: i8 = 0;

/// `rtw8852c_set_txpwr_ul_tb_offset`'s 1TX and 2TX values for `pw_ofst`
/// in -32..=31 (rtw8852c.c:2077-2083): `pw_ofst << 2` and
/// `max(pw_ofst - 3, -32) << 2`.
fn ul_tb_values(pw_ofst: i8) -> (i8, i8) {
    (pw_ofst << 2, (pw_ofst - 3).max(-32) << 2)
}

/// `rtw8852c_init_txpwr_unit(PHY0)` (rtw8852c.c:2293-2316): the UL and
/// coex TX power words, `rtw8852c_set_txpwr_ul_tb_offset(0, MAC_0)`
/// (:2068-2101), then `rtw8852c_init_tssi_ctrl` (:2267-2291), which clears
/// the TSSI control block, writes `ctrl_ini` and sets the flat bandedge
/// through `rtw89_phy_tssi_ctrl_set_bandedge_cfg` (phy.c:8536-8566). Linux
/// ignores its return (core.h:8784-8798); here only an I/O error stops it.
pub(super) fn init_txpwr_unit<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    io.write32(R_AX_PWR_UL_CTRL2, 0x0776_3333)?;
    io.write32(R_AX_PWR_COEXT_CTRL, 0x01EB_F000)?;
    io.write32(R_AX_PWR_UL_CTRL0, 0x0002_F8FF)?;

    // pw_ofst 0: val_2t is -12, an s8 that `rtw89_write32_mask` takes as
    // the u32 0xFFFF_FFF4 and cuts to the byte field.
    let (val_1t, val_2t) = ul_tb_values(UL_TB_PW_OFST);
    for i in 0..4 {
        let mask = B_AX_PWR_UL_TB_V1_MASK << (8 * i);
        io::field32(io, R_AX_PWR_UL_TB_1T, mask, i32::from(val_1t) as u32)?;
        io::field32(io, R_AX_PWR_UL_TB_2T, mask, i32::from(val_2t) as u32)?;
    }

    for addr in (R_AX_TSSI_CTRL_HEAD..=R_AX_TSSI_CTRL_TAIL).step_by(4) {
        io.write32(addr, 0)?;
    }
    for (addr, data) in TSSI_CTRL_INI {
        io.write32(addr, data)?;
    }
    for (addr, mask) in TSSI_BANDEDGE_FLAT {
        io::field32(io, addr, mask, TSSI_DBW_FLAT)?;
    }
    io::field32(
        io,
        R_AX_BANDEDGE_CFG,
        B_AX_BANDEDGE_CFG_IDX_MASK,
        TSSI_BANDEDGE_FLAT_CFG,
    )?;
    // `rtw89_phy_tssi_ctrl_set_fast_mode_cfg` with data[RTW89_TSSI_SBW20].
    for (addr, mask) in TSSI_FAST_MODE_FLAT {
        io::field32(io, addr, mask, TSSI_DBW_FLAT)?;
    }
    Ok(())
}

// reg.h (pinned rtw89), BB offsets.
const R_S0_HW_SI_DIS: u32 = 0x1200;
const R_S1_HW_SI_DIS: u32 = 0x3200;
/// `B_S0_HW_SI_DIS_W_R_TRIG` and `B_S1_HW_SI_DIS_W_R_TRIG`: GENMASK(30, 28).
const B_HW_SI_DIS_W_R_TRIG: u32 = 0x7000_0000;
const R_RSTB_ASYNC: u32 = 0x0704;
const B_RSTB_ASYNC_ALL: u32 = 1 << 1;

/// `rtw8852c_bb_reset_all(PHY0)` (rtw8852c.c:1648-1671), which is the
/// chip's `bb_reset` (:1699-1703): hold both hardware SI write/read
/// triggers, pulse the asynchronous reset, release the triggers and leave
/// the reset released. `rtw89_phy_write32_idx` on PHY 0 is
/// `rtw89_phy_write32_mask` (phy.c:2111-2117).
pub(super) fn bb_reset<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    field32(io, R_S0_HW_SI_DIS, B_HW_SI_DIS_W_R_TRIG, 0x7)?;
    field32(io, R_S1_HW_SI_DIS, B_HW_SI_DIS_W_R_TRIG, 0x7)?;
    io.delay_us(1);
    field32(io, R_RSTB_ASYNC, B_RSTB_ASYNC_ALL, 1)?;
    field32(io, R_RSTB_ASYNC, B_RSTB_ASYNC_ALL, 0)?;
    field32(io, R_S0_HW_SI_DIS, B_HW_SI_DIS_W_R_TRIG, 0x0)?;
    field32(io, R_S1_HW_SI_DIS, B_HW_SI_DIS_W_R_TRIG, 0x0)?;
    field32(io, R_RSTB_ASYNC, B_RSTB_ASYNC_ALL, 1)
}

/// `RTW89_BB_GAIN_BAND_NR` (core.h:7207).
const GAIN_BANDS: usize = 8;
/// The RTL8852C's `rf_path_num`; Linux sizes the arrays by `RF_PATH_MAX`
/// but filters paths by this (phy.c:1590-1591).
const GAIN_PATHS: usize = 2;
/// `LNA_GAIN_NUM` and `TIA_GAIN_NUM` (core.h:6829-6830).
const LNA_GAIN_NUM: usize = 7;
const TIA_GAIN_NUM: usize = 2;

/// `struct rtw89_phy_bb_gain_info` (core.h:7258-7272) for two paths, each
/// field indexed `[gain_band][path]…`. It is filled from the BB gain table
/// and consumed by the gain-error setup when a channel is set.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BbGain {
    pub lna_gain: [[[i8; LNA_GAIN_NUM]; GAIN_PATHS]; GAIN_BANDS],
    pub tia_gain: [[[i8; TIA_GAIN_NUM]; GAIN_PATHS]; GAIN_BANDS],
    pub lna_gain_bypass: [[[i8; LNA_GAIN_NUM]; GAIN_PATHS]; GAIN_BANDS],
    pub lna_op1db: [[[i8; LNA_GAIN_NUM]; GAIN_PATHS]; GAIN_BANDS],
    /// TIA0_LNA0..6 and TIA1_LNA6.
    pub tia_lna_op1db: [[[i8; LNA_GAIN_NUM + 1]; GAIN_PATHS]; GAIN_BANDS],
    pub rpl_ofst_20: [[i8; GAIN_PATHS]; GAIN_BANDS],
    /// `RTW89_BB_RXSC_NUM_40` (core.h:7251-7255): sub-channel 0, 1..=8.
    pub rpl_ofst_40: [[[i8; 9]; GAIN_PATHS]; GAIN_BANDS],
    /// `RTW89_BB_RXSC_NUM_80`: 0, 1..=8, 9..=12.
    pub rpl_ofst_80: [[[i8; 13]; GAIN_PATHS]; GAIN_BANDS],
    /// `RTW89_BB_RXSC_NUM_160`: 0, 1..=8, 9..=12, 13..=14.
    pub rpl_ofst_160: [[[i8; 15]; GAIN_PATHS]; GAIN_BANDS],
}

/// `RTW89_BB_RXSC_START_IDX_*` (phy.c:1425-1431); only these starts
/// are stored, any other is ignored.
const RXSC_FULL: u8 = 0;
const RXSC_20: u8 = 1;
const RXSC_20_1: u8 = 5;
const RXSC_40: u8 = 9;
const RXSC_80: u8 = 13;

/// Store the first `n` bytes of `data`, LSB first, as i8 from `dst[at]`.
fn store(dst: &mut [i8], at: usize, n: usize, data: u32) {
    let bytes = data.to_le_bytes().map(|b| b as i8);
    dst[at..at + n].copy_from_slice(&bytes[..n]);
}

impl BbGain {
    /// `rtw89_phy_config_bb_gain_ax` (phy.c:1577-1621) for one BB gain
    /// table entry at RFE type `rfe`. `addr` is the `rtw89_phy_bb_gain_arg`
    /// union (phy.c:1378-1392): `type` [7:0] (for cfg 1, `rxsc_start`
    /// [3:0] and `bw` [7:4]), `path` [15:8], `gain_band` [23:16] and
    /// `cfg_type` [31:24]. A band past 8, a path past the chip's two and
    /// the delay addresses 0xF9..=0xFE are ignored, in that order, and so
    /// is every key Linux warns of as unknown. Nothing is written to the
    /// chip.
    pub fn decode(&mut self, addr: u32, data: u32, rfe: u8) {
        let [kind, path, band, cfg] = addr.to_le_bytes();
        let (band, path) = (usize::from(band), usize::from(path));
        if band >= GAIN_BANDS || path >= GAIN_PATHS || (0xF9..=0xFE).contains(&addr) {
            return;
        }
        match cfg {
            // rtw89_phy_cfg_bb_gain_error (phy.c:1394-1423).
            0 => match kind {
                0 => store(&mut self.lna_gain[band][path], 0, 4, data),
                1 => store(&mut self.lna_gain[band][path], 4, 3, data),
                2 => store(&mut self.tia_gain[band][path], 0, 2, data),
                _ => {}
            },
            1 => self.rpl_ofst(band, path, kind & 0xF, kind >> 4, data),
            // rtw89_phy_cfg_bb_gain_bypass (phy.c:1515-1540).
            2 => match kind {
                0 => store(&mut self.lna_gain_bypass[band][path], 0, 4, data),
                1 => store(&mut self.lna_gain_bypass[band][path], 4, 3, data),
                _ => {}
            },
            // rtw89_phy_cfg_bb_gain_op1db (phy.c:1542-1575).
            3 => match kind {
                0 => store(&mut self.lna_op1db[band][path], 0, 4, data),
                1 => store(&mut self.lna_op1db[band][path], 4, 3, data),
                2 => store(&mut self.tia_lna_op1db[band][path], 0, 4, data),
                3 => store(&mut self.tia_lna_op1db[band][path], 4, 4, data),
                _ => {}
            },
            // phy.c:1609-1613: only RFE types from 50 (with an eFEM) use
            // cfg 4. Below 50 it is skipped quietly; from 50 Linux warns of
            // an unknown cfg type. Neither stores anything.
            4 if rfe < 50 => {}
            _ => {}
        }
    }

    /// `rtw89_phy_cfg_bb_rpl_ofst` (phy.c:1433-1511): by `bw` (20, 40, 80,
    /// 160 MHz) and the first sub-channel `rxsc`.
    fn rpl_ofst(&mut self, band: usize, path: usize, rxsc: u8, bw: u8, data: u32) {
        let (dst, n): (&mut [i8], usize) = match (bw, rxsc) {
            (0, _) => (core::slice::from_mut(&mut self.rpl_ofst_20[band][path]), 1),
            (1, RXSC_FULL) => (&mut self.rpl_ofst_40[band][path][..], 1),
            (1, RXSC_20) => (&mut self.rpl_ofst_40[band][path][..], 2),
            (2, RXSC_FULL) => (&mut self.rpl_ofst_80[band][path][..], 1),
            (2, RXSC_20) => (&mut self.rpl_ofst_80[band][path][..], 4),
            (2, RXSC_40) => (&mut self.rpl_ofst_80[band][path][..], 2),
            (3, RXSC_FULL) => (&mut self.rpl_ofst_160[band][path][..], 1),
            (3, RXSC_20 | RXSC_20_1 | RXSC_40) => (&mut self.rpl_ofst_160[band][path][..], 4),
            (3, RXSC_80) => (&mut self.rpl_ofst_160[band][path][..], 2),
            _ => return,
        };
        let at = if bw == 0 { 0 } else { usize::from(rxsc) };
        store(dst, at, n, data);
    }
}

/// What [`init_bb_reg`] applied: how many entries of each table its branch
/// handed on, and the decoded BB gain table.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BbInit {
    pub bb: u16,
    pub bb_gain: u16,
    pub gain: BbGain,
}

/// `rtw89_phy_init_bb_reg` (phy.c:1885-1911) for PHY 0 without DBCC, at
/// RFE type `rfe` and cut `cv`, in Linux's order: the `bb` table through
/// [`config_bb_reg`], `init_txpwr_unit`, the `bb_gain` table into a
/// [`BbGain`], then `bb_reset`. A table whose headline selects nothing
/// (or that reaches an unselected ELSE) fails as [`phy_table::apply`]
/// says; an I/O error stops the sequence where it happened.
pub fn init_bb_reg<I: AxIo + ?Sized>(
    io: &mut I,
    bb: &Reg2<'_>,
    bb_gain: &Reg2<'_>,
    rfe: u8,
    cv: u8,
) -> Result<BbInit, AxError> {
    let bb_count = phy_table::apply(bb, BB_REG, rfe, cv, &mut |addr, data| {
        config_bb_reg(io, addr, data)
    })?;
    init_txpwr_unit(io)?;
    let mut gain = BbGain::default();
    let gain_count = phy_table::apply(bb_gain, BB_GAIN, rfe, cv, &mut |addr, data| {
        gain.decode(addr, data, rfe);
        Ok(())
    })?;
    bb_reset(io)?;
    Ok(BbInit {
        bb: bb_count,
        bb_gain: gain_count,
        gain,
    })
}

#[cfg(test)]
mod tests;

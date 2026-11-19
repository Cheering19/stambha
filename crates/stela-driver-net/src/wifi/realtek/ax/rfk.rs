// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
//! The analog front-end and receiver calibrations an RTL8852C runs in
//! `rtw8852c_rfk_init` (pinned rtw89/rtw8852c.c:1985-1997), from
//! rtw8852c_rfk.c, with the DRCK and DACK tables from rtw8852c_rfk_table.c:
//! - [`rck`] is `rtw8852c_rck` (rtw8852c_rfk.c:4133-4139), `_rck`
//!   (:1455-1489) on path A and then B: the RF RC filter calibration;
//! - [`drck`] is `_drck` (:568-586) with `rtw8852c_drck_defs`
//!   (rtw8852c_rfk_table.c:50-55): the DAC reference clock;
//! - [`addck`] is `_addck` (:369-405): the ADC DC offset of S0 and S1;
//! - [`addck_reload`] is `_addck_backup` (:218-233) then `_addck_reload`
//!   (:235-249);
//! - [`dack`] is `rtw8852c_dack` (:4141-4148): the coex RFK notify
//!   ([`coex::rfk_notify`]) around `_dac_cal` (:588-626), which runs the
//!   three calls above and then `_dack` (:562-566), that is `_dack_s0`
//!   (:508-533) and `_dack_s1` (:535-560) with `rtw8852c_txck_force`
//!   (:443-452), `_dack_reset` (:407-412), `_check_dack_done` (:489-506),
//!   `_dack_backup_s0`/`_s1` (:251-301) and `_dack_reload_by_path`
//!   (:303-359), and the `dack_reload_defs`, `dack_reset_defs_a`/`_b` and
//!   `dack_defs_s0`/`_s1` tables (rtw8852c_rfk_table.c:7-48);
//! - [`rx_dck`] is `rtw8852c_rx_dck(PHY0, false)` (:4223-4226), `_rx_dck`
//!   (:4169-4221) with `retry_limit` 1 on path A, then B: `_set_rx_dck`
//!   (:1738-1754) with `_rx_dck_toggle` (:1719-1736), then
//!   `_rx_dck_recover` (:1670-1717) with `_rx_dck_fix_if_need`
//!   (:1645-1668) and `_rx_dck_value_rewrite` (:1570-1595): the receivers'
//!   DC offsets.
//!
//! BB registers go through [`bb`] (`rtw89_phy_write32_mask` and
//! `rtw89_phy_read32_mask`, every write a read-modify-write) and RF
//! registers through [`rf`]. `rtw89_rfk_parser`'s delay entries are
//! `udelay` (phy.c:8435-8439), and `_addck`'s `fsleep(1)` and `udelay(1)`
//! are both a 1 us busy wait (`fsleep` takes `udelay` up to 10 us,
//! include/linux/delay.h).
//!
//! Every poll is Linux's `read_poll_timeout_atomic`, with its iteration
//! budget ([`poll_sample_atomic`]). Linux only logs a timeout and carries
//! on, so these do too and return it to the caller to record (spec ruling
//! S1.3), each on the path it happened on: Linux records S1's ADDCK and DACK
//! timeouts in `addck_timeout[0]` (:402) and `msbk_timeout[0]` (:551), which
//! only a debug dump reads. An RF access SWSI never serves still fails, as
//! [`rf`] says. The register reads Linux makes only for its debug prints
//! are kept (spec ruling S1.4); `_dack_dump` (:170-216) prints only what
//! was already read and reads nothing.
//!
//! `_dac_cal`'s `BTC_WRFK_ONESHOT_START`/`_STOP` notifies around `_addck`
//! and `_dack` do no I/O (coex.c:9429-9433, spec ruling S1.11) and are not
//! ported, nor are `rtw89_btc_phymap` and the `dack_done`/`dack_cnt`
//! bookkeeping, which are memory only.
//!
//! This module grants no IQK, DPK, TSSI, LCK or RX DCK tracking, no PHY 1
//! and no channel: it leaves the calibration results in the registers and
//! returns them.
use super::bb;
use super::coex;
use super::elements::RfPath;
use super::io::{AxError, AxIo, Polled, poll_sample_atomic};
use super::rf::{self, RFREG_MASK};

/// `RR_MOD` (reg.h:8535) and its `RR_MOD_MASK` [19:16], set to
/// `RR_MOD_V_RX` (reg.h:8539, 8547).
const RR_MOD: u32 = 0x00;
const RR_MOD_MASK: u32 = 0xF_0000;
const RR_MOD_V_RX: u32 = 0x3;
/// `RR_RSV1` and `RR_RSV1_RST` b0 (reg.h:8565-8566).
const RR_RSV1: u32 = 0x05;
const RR_RSV1_RST: u32 = 1 << 0;
/// `RR_RCKC` with its `RR_RCKC_CA` [14:10], and `RR_RCKS` (reg.h:8612-8614).
const RR_RCKC: u32 = 0x1B;
const RR_RCKC_CA: u32 = 0x7C00;
const RR_RCKS: u32 = 0x1C;
/// The RCK trigger `_rck` writes to `RR_RCKC` (rtw8852c_rfk.c:1472).
const RCK_TRIGGER: u32 = 0x00240;
/// The bit `_rck` polls in RF 0x1C (rtw8852c_rfk.c:1474-1475).
const RCK_DONE: u32 = 1 << 3;

/// `R_DRCK_FH` 0xC094 with `B_DRCK_LAT` b9; `R_DRCK` 0xC0C4 with
/// `B_DRCK_IDLE` b9, `B_DRCK_EN` b6 and `B_DRCK_VAL` [4:0]; `R_DRCK_RES`
/// 0xC0C8 with `B_DRCK_RES` [19:15] and `B_DRCK_POL` b3 (reg.h:10367-10376).
const R_DRCK_FH: u32 = 0xC094;
const B_DRCK_LAT: u32 = 1 << 9;
const R_DRCK: u32 = 0xC0C4;
const B_DRCK_IDLE: u32 = 1 << 9;
const B_DRCK_EN: u32 = 1 << 6;
const B_DRCK_VAL: u32 = 0x1F;
const R_DRCK_RES: u32 = 0xC0C8;
const B_DRCK_RES: u32 = 0xF_8000;
const B_DRCK_POL: u32 = 1 << 3;

/// `R_ADDCK0`/`R_ADDCK1` with `B_ADDCK0`/`B_ADDCK1` [9:8], `_EN` b4 and
/// `_RST` b2; `R_ADDCK0_RL`/`R_ADDCK1_RL` with `_RLS` [29:28], `_RL1`
/// [27:18] and `_RL0` [17:8]; `R_ADDCKR0`/`R_ADDCKR1` with `_A0` [19:10] and
/// `_A1` [9:0] (reg.h:10405-10420, 10464-10476). Index 0 is S0, 1 is S1.
const R_ADDCK: [u32; 2] = [0xC0F4, 0xC1F4];
const B_ADDCK: u32 = 0x300;
const B_ADDCK_EN: u32 = 1 << 4;
const B_ADDCK_RST: u32 = 1 << 2;
const R_ADDCK_RL: [u32; 2] = [0xC0F8, 0xC1F8];
const B_ADDCK_RLS: u32 = 0x3000_0000;
const B_ADDCK_RL1: u32 = 0x0FFC_0000;
const B_ADDCK_RL0: u32 = 0x0003_FF00;
const R_ADDCKR: [u32; 2] = [0xC0FC, 0xC1FC];
const B_ADDCKR_A0: u32 = 0xF_FC00;
const B_ADDCKR_A1: u32 = 0x3FF;
/// The done bit `_addck` polls in each result register (:382-383, :398-399).
const ADDCK_DONE: u32 = 1 << 0;

/// `RR_MODOPT` (reg.h:8556), cleared on both paths before DACK.
const RR_MODOPT: u32 = 0x01;
/// The `RR_MOD` value `_dac_cal` holds both paths in for ADDCK and DACK
/// (rtw8852c_rfk.c:604-605).
const DACK_RF_MODE: u32 = 0x337E1;

/// `R_P0_RXCK` 0x12A0's `B_P0_TXCK_ON` b15 and `B_P0_TXCK_VAL` [14:12]
/// (reg.h:9083-9090), which `rtw8852c_txck_force` reaches as `R_P0_RXCK |
/// path << 13`, and `DAC_160M` (rtw8852c_rfk.c:425).
const B_TXCK_ON: u32 = 1 << 15;
const B_TXCK_VAL: u32 = 0x7000;
const DAC_160M: u32 = 3;
/// `B_P0_NRBW_DBG` / `B_P1_DBGMOD_ON` b30 (reg.h:9099, 9440): DACK's debug
/// mode, which `dack_defs_s0`/`_s1` and the backups set.
const B_DACK_DBG: u32 = 1 << 30;
/// The two entries `dack_defs_s0` and `_s1` share
/// (rtw8852c_rfk_table.c:32-33, 42-43): 0x030C b28 set, 0x032C b31 clear.
const R_DACK_DEFS_030C: u32 = 0x030C;
const B_DACK_DEFS_030C: u32 = 1 << 28;
const R_DACK_DEFS_032C: u32 = 0x032C;
const B_DACK_DEFS_032C: u32 = 1 << 31;
/// The [31:20] field both `dack_defs` tables set to 0x30 in each side's two
/// control registers (rtw8852c_rfk_table.c:34-35, 44-45).
const B_DACK_CTRL: u32 = 0xFFF0_0000;
const DACK_CTRL: u32 = 0x30;
/// b17 of the selector register, which `dack_reset_defs_a`/`_b` clear and
/// set (rtw8852c_rfk_table.c:16-26).
const B_DACK_RST: u32 = 1 << 17;
/// `B_DCOF0_V`/`B_DACK10` [4:1], the MSBK selector (reg.h:10339, 10423).
const B_DACK_SEL: u32 = 0x1E;
/// `B_DCOF1_S`/`B_DACK1_EN` b0, the DACK trigger (reg.h:10343, 10427).
const B_DACK_EN: u32 = 1 << 0;
/// The MSBK result [31:24], the bias [11:2] and the DADCK [31:24]
/// (`B_DACK_S0M0` and the rest, reg.h:10350-10453).
const B_DACK_MSBK: u32 = 0xFF00_0000;
const B_DACK_BIAS: u32 = 0xFFC;
const B_DACK_DADCK: u32 = 0xFF00_0000;
/// `RTW89_DACK_MSBK_NR` (core.h:6619).
const DACK_MSBK_NR: usize = 16;
/// `dack_reload_defs` (rtw8852c_rfk_table.c:7-12): b17 of these four set
/// before every reload.
const DACK_RELOAD_DEFS: [u32; 4] = [0xC004, 0xC024, 0xC104, 0xC124];
const B_DACK_RELOAD: u32 = 1 << 17;
/// The first register `_dack_reload_by_path` writes, before its index
/// (0x14) and path B (0x28) offsets (rtw8852c_rfk.c:311-313, 319).
const R_DACK_RELOAD: u32 = 0xC200;
const DACK_RELOAD_INDEX_OFFSET: u32 = 0x14;
const DACK_RELOAD_PATH_OFFSET: u32 = 0x28;

/// One side of DACK: S0 (path A) or S1 (path B). Each pair is Linux's
/// index 0 (`ic`) and 1 (`qc`).
struct DackSide {
    /// `R_P0_RXCK | path << 13`: 0x12A0 or 0x32A0.
    rxck: u32,
    /// `R_P0_NRBW` 0x12B8 or `R_P1_DBGMOD` 0x32B8.
    dbg: u32,
    /// The control registers `dack_defs` sets [31:20] of, the first also
    /// the trigger (`R_DCOF1` 0xC004 or `R_DACK1_K` 0xC104).
    ctrl: [u32; 2],
    /// The MSBK selectors (`R_DCOF0`/`R_DCOF8` or `R_DACK10`/`R_DACK11`),
    /// the first also the reset register.
    sel: [u32; 2],
    /// `_check_dack_done`'s chain of registers and bits, in order.
    done: [(u32, u32); 4],
    /// The MSBK results each selector picks.
    msbk: [u32; 2],
    bias: [u32; 2],
    dadck: [u32; 2],
}

/// S0 (`_dack_s0`, `dack_defs_s0`, `_dack_backup_s0`) and S1 (`_dack_s1`,
/// `dack_defs_s1`, `_dack_backup_s1`), with reg.h:9083-9099, 9439 and
/// 10337-10453.
const DACK_SIDES: [DackSide; 2] = [
    DackSide {
        rxck: 0x12A0,
        dbg: 0x12B8,
        ctrl: [0xC004, 0xC024],
        sel: [0xC000, 0xC020],
        done: [
            (0xC040, 1 << 31),
            (0xC064, 1 << 31),
            (0xC05C, 1 << 2),
            (0xC080, 1 << 2),
        ],
        msbk: [0xC05C, 0xC080],
        bias: [0xC048, 0xC06C],
        dadck: [0xC060, 0xC084],
    },
    DackSide {
        rxck: 0x32A0,
        dbg: 0x32B8,
        ctrl: [0xC104, 0xC124],
        sel: [0xC100, 0xC120],
        done: [
            (0xC140, 1 << 31),
            (0xC164, 1 << 31),
            (0xC15C, 1 << 2),
            (0xC180, 1 << 2),
        ],
        msbk: [0xC15C, 0xC180],
        bias: [0xC148, 0xC16C],
        dadck: [0xC160, 0xC184],
    },
];

/// `RR_MOD_DCK` [14:10], RX DCK's offset index, and `RR_MOD_LO_SEL` b1
/// (reg.h:8540, 8555).
const RR_MOD_DCK: u32 = 0x7C00;
const RR_MOD_LO_SEL: u32 = 1 << 1;
/// `RR_DCK` 0x92 with `RR_DCK_TIA` [15:9], `RR_DCK_DONE` [7:5],
/// `RR_DCK_FINE` b1 and `RR_DCK_LV` b0; `RR_DCK1` 0x93 with `RR_DCK1_TIA`
/// [15:9], `RR_DCK1_DONE` b5 and `RR_DCK1_CLR` [3:0]; `RR_DCKC` 0x95 with
/// `RR_DCKC_CHK` b3 (reg.h:8734-8749).
const RR_DCK: u32 = 0x92;
const RR_DCK_TIA: u32 = 0xFE00;
const RR_DCK_DONE: u32 = 0xE0;
const RR_DCK_FINE: u32 = 1 << 1;
const RR_DCK_LV: u32 = 1 << 0;
const RR_DCK1: u32 = 0x93;
const RR_DCK1_TIA: u32 = 0xFE00;
const RR_DCK1_DONE: u32 = 1 << 5;
const RR_DCK1_CLR: u32 = 0xF;
const RR_DCKC: u32 = 0x95;
const RR_DCKC_CHK: u32 = 1 << 3;
/// `RR_RXBB2` 0x8F with `RR_RXBB2_IDAC` [11:9] (reg.h:8725, 8729).
const RR_RXBB2: u32 = 0x8F;
const RR_RXBB2_IDAC: u32 = 0xE00;
/// The LUT write path: `RR_LUTPLL` 0xEC `RR_CAL_RW` b19, `RR_RFC` 0xF0
/// `RR_WCAL` b16, `RR_LUTWA` 0x33 and `RR_LUTWD0` 0x3F with
/// `RR_LUTWD0_MB` [11:6] and `_LB` [5:0] (reg.h:8627-8634, 8803-8811), and
/// `MASKBYTE0` (core.h:40).
const RR_LUTPLL: u32 = 0xEC;
const RR_CAL_RW: u32 = 1 << 19;
const RR_RFC: u32 = 0xF0;
const RR_WCAL: u32 = 1 << 16;
const RR_LUTWA: u32 = 0x33;
const RR_LUTWD0: u32 = 0x3F;
const RR_LUTWD0_MB: u32 = 0xFC0;
const RR_LUTWD0_LB: u32 = 0x3F;
const MASKBYTE0: u32 = 0xFF;
/// `_dck_addr_bs` and `_dck_addr` (rtw8852c_rfk.c:73-74): each group's
/// base and target offset index, each the even one of a pair.
const DCK_ADDR_BS: [u8; 2] = [0x00, 0x10];
const DCK_ADDR: [u8; 2] = [0x0C, 0x1C];
/// `_rx_dck_fix_if_need`'s `th` (rtw8852c_rfk.c:1648).
const RX_DCK_TH: i32 = 10;

/// The `read_poll_timeout_atomic(.., 1, 10000, false, ..)` of DRCK, ADDCK
/// and DACK: at most 9992 samples, 1 us apart.
const DACK_POLL_DELAY_US: u64 = 1;
const DACK_POLL_TIMEOUT_US: u64 = 10_000;

/// `rtw8852c_rck`: `_rck` on path A, then B. Returns each path's RCK code,
/// `RR_RCKC_CA` as read back after the poll, and whether its poll timed out.
pub fn rck<I: AxIo + ?Sized>(io: &mut I) -> Result<([u8; 2], [bool; 2]), AxError> {
    let mut code = [0; 2];
    let mut timed_out = [false; 2];
    for (i, path) in [RfPath::A, RfPath::B].into_iter().enumerate() {
        (code[i], timed_out[i]) = rck_path(io, path)?;
    }
    Ok((code, timed_out))
}

/// `_rck` (rtw8852c_rfk.c:1455-1489) on one path.
fn rck_path<I: AxIo + ?Sized>(io: &mut I, path: RfPath) -> Result<(u8, bool), AxError> {
    let reg5 = rf::read_rf(io, path, RR_RSV1, RFREG_MASK)?;
    rf::write_rf(io, path, RR_RSV1, RR_RSV1_RST, 0)?;
    rf::write_rf(io, path, RR_MOD, RR_MOD_MASK, RR_MOD_V_RX)?;
    // The "[RCK] RF0x00" debug print's read.
    rf::read_rf(io, path, RR_MOD, RFREG_MASK)?;
    rf::write_rf(io, path, RR_RCKC, RFREG_MASK, RCK_TRIGGER)?;
    // read_poll_timeout_atomic(rtw89_read_rf, val, val, 2, 20, false, ..,
    // path, 0x1c, BIT(3)): at most 11 samples.
    let timed_out = matches!(
        poll_sample_atomic(
            io,
            2,
            20,
            false,
            |io| rf::read_rf(io, path, RR_RCKS, RCK_DONE),
            |v| v != 0,
        )?,
        Polled::Expired(_)
    );
    let code = rf::read_rf(io, path, RR_RCKC, RR_RCKC_CA)?;
    // Written back unshifted over the whole register, as Linux does.
    rf::write_rf(io, path, RR_RCKC, RFREG_MASK, code)?;
    rf::write_rf(io, path, RR_RSV1, RFREG_MASK, reg5)?;
    // The "[RCK] RF 0x1b / 0x1c" debug print's two reads. C leaves their
    // order unspecified; this takes them as written.
    rf::read_rf(io, path, RR_RCKC, RFREG_MASK)?;
    rf::read_rf(io, path, RR_RCKS, RFREG_MASK)?;
    // A 5-bit field.
    Ok((code as u8, timed_out))
}

/// A DACK-side `read_poll_timeout_atomic(rtw89_phy_read32_mask, val, val, 1,
/// 10000, false, .., reg, bit)`: whether it timed out.
fn poll_bit<I: AxIo + ?Sized>(io: &mut I, reg: u32, bit: u32) -> Result<bool, AxError> {
    Ok(matches!(
        poll_sample_atomic(
            io,
            DACK_POLL_DELAY_US,
            DACK_POLL_TIMEOUT_US,
            false,
            |io| bb::read32_mask(io, reg, bit),
            |v| v != 0,
        )?,
        Polled::Expired(_)
    ))
}

/// `_drck` (rtw8852c_rfk.c:568-586). Returns the DRCK code `B_DRCK_RES`, as
/// written to `B_DRCK_VAL`, and whether the poll timed out (Linux only
/// prints "[DACK]DRCK timeout").
pub(super) fn drck<I: AxIo + ?Sized>(io: &mut I) -> Result<(u8, bool), AxError> {
    bb::field32(io, R_DRCK, B_DRCK_EN, 1)?;
    let timed_out = poll_bit(io, R_DRCK_RES, B_DRCK_POL)?;
    // rtw8852c_drck_defs (rtw8852c_rfk_table.c:50-55).
    bb::field32(io, R_DRCK, B_DRCK_EN, 0)?;
    bb::field32(io, R_DRCK_FH, B_DRCK_LAT, 1)?;
    io.delay_us(1);
    bb::field32(io, R_DRCK_FH, B_DRCK_LAT, 0)?;
    let code = bb::read32_mask(io, R_DRCK_RES, B_DRCK_RES)?;
    bb::field32(io, R_DRCK, B_DRCK_IDLE, 0)?;
    bb::field32(io, R_DRCK, B_DRCK_VAL, code)?;
    // The "[DACK]0xc0c4" debug print's read.
    bb::read32(io, R_DRCK)?;
    // A 5-bit field.
    Ok((code as u8, timed_out))
}

/// `_addck` (rtw8852c_rfk.c:369-405): S0, then S1. Returns whether each
/// side's poll timed out, S1's in index 1 where Linux writes index 0.
pub(super) fn addck<I: AxIo + ?Sized>(io: &mut I) -> Result<[bool; 2], AxError> {
    let mut timed_out = [false; 2];
    for (s, out) in timed_out.iter_mut().enumerate() {
        let reg = R_ADDCK[s];
        bb::field32(io, reg, B_ADDCK_RST, 1)?;
        bb::field32(io, reg, B_ADDCK_EN, 1)?;
        bb::field32(io, reg, B_ADDCK_EN, 0)?;
        // S0's fsleep(1) and S1's udelay(1): 1 us either way.
        io.delay_us(1);
        bb::field32(io, reg, B_ADDCK, 1)?;
        *out = poll_bit(io, R_ADDCKR[s], ADDCK_DONE)?;
        bb::field32(io, reg, B_ADDCK_RST, 0)?;
    }
    Ok(timed_out)
}

/// `_addck_backup` (rtw8852c_rfk.c:218-233), then `_addck_reload`
/// (:235-249). Returns each side's `(A0 [19:10], A1 [9:0])` result, Linux's
/// `addck_d[s][0]` and `[1]`, as reloaded into `_RL1` and `_RL0`.
pub(super) fn addck_reload<I: AxIo + ?Sized>(io: &mut I) -> Result<[(u16, u16); 2], AxError> {
    let mut addck = [(0, 0); 2];
    for (s, d) in addck.iter_mut().enumerate() {
        bb::field32(io, R_ADDCK[s], B_ADDCK, 0)?;
        // 10-bit fields.
        let a0 = bb::read32_mask(io, R_ADDCKR[s], B_ADDCKR_A0)? as u16;
        let a1 = bb::read32_mask(io, R_ADDCKR[s], B_ADDCKR_A1)? as u16;
        *d = (a0, a1);
    }
    for (s, &(a0, a1)) in addck.iter().enumerate() {
        bb::field32(io, R_ADDCK_RL[s], B_ADDCK_RL1, a0.into())?;
        bb::field32(io, R_ADDCK_RL[s], B_ADDCK_RL0, a1.into())?;
        bb::field32(io, R_ADDCK_RL[s], B_ADDCK_RLS, 0x3)?;
    }
    Ok(addck)
}

/// What [`dack`] leaves in the registers and the polls it saw time out,
/// each on the path it happened on (index 0 is path A/S0, 1 is B/S1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Dack {
    /// The DRCK code, as [`drck`] returns it.
    pub drck: u8,
    pub drck_timed_out: bool,
    /// Each side's ADDCK `(A0, A1)` result, as [`addck_reload`] returns it.
    pub addck: [(u16, u16); 2],
    pub addck_timed_out: [bool; 2],
    /// Whether each side's `_check_dack_done` poll timed out. Linux records
    /// S1's in `msbk_timeout[0]` (rtw8852c_rfk.c:551); spec ruling S1.3.
    pub dack_timed_out: [bool; 2],
}

/// `rtw8852c_dack` (rtw8852c_rfk.c:4141-4148): the coex RFK START, then
/// `_dac_cal` (:588-626), then the coex RFK STOP. `_dac_cal` saves both
/// paths' `RR_MOD`, runs [`drck`], clears `RR_RSV1_RST` and sets
/// `RR_MOD` to 0x337E1 on both paths, runs [`addck`] and [`addck_reload`],
/// clears `RR_MODOPT` on both paths, runs DACK on S0 and then S1, and
/// restores `RR_MOD` and sets `RR_RSV1_RST` on both paths. A timed-out poll
/// is recorded and the sequence carries on, as in Linux. An I/O error (an
/// RF access SWSI never serves, a failed CMAC check in the coex START)
/// returns at once with no STOP, for the caller to tear down: the
/// bring-up's teardown ([`coex::radio_off`]) writes the scoreboard with
/// every driver bit clear, the RFK bit with them.
pub fn dack<I: AxIo + ?Sized>(io: &mut I) -> Result<Dack, AxError> {
    coex::rfk_notify(io, true)?;
    let rf0_0 = rf::read_rf(io, RfPath::A, RR_MOD, RFREG_MASK)?;
    let rf1_0 = rf::read_rf(io, RfPath::B, RR_MOD, RFREG_MASK)?;
    let (drck_code, drck_timed_out) = drck(io)?;
    for path in [RfPath::A, RfPath::B] {
        rf::write_rf(io, path, RR_RSV1, RR_RSV1_RST, 0)?;
    }
    for path in [RfPath::A, RfPath::B] {
        rf::write_rf(io, path, RR_MOD, RFREG_MASK, DACK_RF_MODE)?;
    }
    let addck_timed_out = addck(io)?;
    let addck_results = addck_reload(io)?;
    for path in [RfPath::A, RfPath::B] {
        rf::write_rf(io, path, RR_MODOPT, RFREG_MASK, 0)?;
    }
    // _dack: _dack_s0, then _dack_s1.
    let mut dack_timed_out = [false; 2];
    for (s, (side, out)) in DACK_SIDES.iter().zip(&mut dack_timed_out).enumerate() {
        *out = dack_side(io, s, side)?;
    }
    rf::write_rf(io, RfPath::A, RR_MOD, RFREG_MASK, rf0_0)?;
    rf::write_rf(io, RfPath::B, RR_MOD, RFREG_MASK, rf1_0)?;
    for path in [RfPath::A, RfPath::B] {
        rf::write_rf(io, path, RR_RSV1, RR_RSV1_RST, 1)?;
    }
    coex::rfk_notify(io, false)?;
    Ok(Dack {
        drck: drck_code,
        drck_timed_out,
        addck: addck_results,
        addck_timed_out,
        dack_timed_out,
    })
}

/// `_dack_s0` (rtw8852c_rfk.c:508-533) for `s` 0, `_dack_s1` (:535-560)
/// for 1. Returns whether the done poll timed out; the backup and reload
/// run either way.
fn dack_side<I: AxIo + ?Sized>(io: &mut I, s: usize, side: &DackSide) -> Result<bool, AxError> {
    // rtw8852c_txck_force(path, true, DAC_160M).
    bb::field32(io, side.rxck, B_TXCK_ON, 0)?;
    bb::field32(io, side.rxck, B_TXCK_VAL, DAC_160M)?;
    bb::field32(io, side.rxck, B_TXCK_ON, 1)?;
    // dack_defs_s0 / dack_defs_s1.
    bb::field32(io, side.dbg, B_DACK_DBG, 1)?;
    bb::field32(io, R_DACK_DEFS_030C, B_DACK_DEFS_030C, 1)?;
    bb::field32(io, R_DACK_DEFS_032C, B_DACK_DEFS_032C, 0)?;
    for reg in side.ctrl {
        bb::field32(io, reg, B_DACK_CTRL, DACK_CTRL)?;
    }
    // _dack_reset: dack_reset_defs_a / _b.
    bb::field32(io, side.sel[0], B_DACK_RST, 0)?;
    bb::field32(io, side.sel[0], B_DACK_RST, 1)?;
    bb::field32(io, side.ctrl[0], B_DACK_EN, 1)?;
    // read_poll_timeout_atomic(_check_dack_done, done, done, 1, 10000,
    // false, .., s0).
    let timed_out = matches!(
        poll_sample_atomic(
            io,
            DACK_POLL_DELAY_US,
            DACK_POLL_TIMEOUT_US,
            false,
            |io| dack_done(io, side),
            |done| done,
        )?,
        Polled::Expired(_)
    );
    bb::field32(io, side.ctrl[0], B_DACK_EN, 0)?;
    // rtw8852c_txck_force(path, false, DAC_960M): the force bit only.
    bb::field32(io, side.rxck, B_TXCK_ON, 0)?;
    // _dack_backup_s0 / _s1.
    bb::field32(io, side.dbg, B_DACK_DBG, 1)?;
    let mut msbk = [[0u8; DACK_MSBK_NR]; 2];
    for i in 0..DACK_MSBK_NR {
        for (d, m) in msbk.iter_mut().enumerate() {
            // At most 15, into a 4-bit field.
            bb::field32(io, side.sel[d], B_DACK_SEL, i as u32)?;
            // 8-bit fields.
            m[i] = bb::read32_mask(io, side.msbk[d], B_DACK_MSBK)? as u8;
        }
    }
    let mut biask = [0u16; 2];
    for (b, &reg) in biask.iter_mut().zip(&side.bias) {
        // 10-bit fields.
        *b = bb::read32_mask(io, reg, B_DACK_BIAS)? as u16;
    }
    let mut dadck = [0u8; 2];
    for (d, &reg) in dadck.iter_mut().zip(&side.dadck) {
        // 8-bit fields.
        *d = bb::read32_mask(io, reg, B_DACK_DADCK)? as u8;
    }
    // _dack_reload(path): index 0, then 1.
    for index in 0..2 {
        dack_reload(io, s, index, &msbk[index], biask[index], dadck[index])?;
    }
    bb::field32(io, side.dbg, B_DACK_DBG, 0)?;
    Ok(timed_out)
}

/// `_check_dack_done` (rtw8852c_rfk.c:489-506): the four done bits, read in
/// order and stopping at the first clear one, as C's `||` does.
fn dack_done<I: AxIo + ?Sized>(io: &mut I, side: &DackSide) -> Result<bool, AxError> {
    for &(reg, bit) in &side.done {
        if bb::read32_mask(io, reg, bit)? == 0 {
            return Ok(false);
        }
    }
    Ok(true)
}

/// `_dack_reload_by_path` (rtw8852c_rfk.c:303-359) for side `s` (path A or
/// B) and `index`: `dack_reload_defs`, then the MSBK words 15..12, 11..8,
/// 7..4 and 3..0 (lowest index in the lowest byte), each written whole and
/// read back for the debug print, then the bias and DADCK word, written
/// whole and then its b0 set.
fn dack_reload<I: AxIo + ?Sized>(
    io: &mut I,
    s: usize,
    index: usize,
    msbk: &[u8; DACK_MSBK_NR],
    biask: u16,
    dadck: u8,
) -> Result<(), AxError> {
    // `s` and `index` are 0 or 1.
    let base = R_DACK_RELOAD
        + index as u32 * DACK_RELOAD_INDEX_OFFSET
        + s as u32 * DACK_RELOAD_PATH_OFFSET;
    for reg in DACK_RELOAD_DEFS {
        bb::field32(io, reg, B_DACK_RELOAD, 1)?;
    }
    let mut addr = base;
    for word in msbk.as_chunks::<4>().0.iter().rev() {
        let v = word
            .iter()
            .rev()
            .fold(0u32, |v, &b| (v << 8) | u32::from(b));
        bb::write32(io, addr, v)?;
        // The "[DACK]0x%x=0x%x" debug print's read.
        bb::read32(io, addr)?;
        addr += 4;
    }
    // A 10-bit bias in [31:22] and an 8-bit DADCK in [21:14].
    let v = (u32::from(biask) << 22) | (u32::from(dadck) << 14);
    bb::write32(io, addr, v)?;
    bb::set32(io, addr, 1 << 0)
}

/// `rtw8852c_rx_dck(PHY0, is_afe = false)` (rtw8852c_rfk.c:4223-4226), as
/// `rtw8852c_rfk_init` calls it: `_rx_dck` (:4169-4221) with `retry_limit`
/// 1 on path A, then B (`_kpath` is `RF_AB` without DBCC, :80-92). Returns,
/// per path, whether a `_rx_dck_toggle` poll timed out (Linux only warns
/// "RXDCK timeout") and how many offsets `_rx_dck_recover` rewrote.
///
/// There is no coex notify here: only `rtw8852c_rx_dck_track` (:4265,
/// :4279) notifies `BTC_WRFKT_RXDCK`. `is_afe` is unused by 8852C's
/// `_set_rx_dck`, `is_tssi_mode` is false at init so `R_P0_TSSI_TRK` is
/// not touched, and `rx_dck->thermal` is memory only. RF 0x00 is left in
/// RX mode at the last offset index, as Linux leaves it.
pub fn rx_dck<I: AxIo + ?Sized>(io: &mut I) -> Result<([bool; 2], [u8; 2]), AxError> {
    let mut timed_out = [false; 2];
    let mut rewrites = [0; 2];
    for (i, path) in [RfPath::A, RfPath::B].into_iter().enumerate() {
        (timed_out[i], rewrites[i]) = rx_dck_path(io, path)?;
    }
    Ok((timed_out, rewrites))
}

/// One path of `_rx_dck`. With `retry_limit` 1 the first try is the last,
/// so `_rx_dck_rek_check` never runs and `_rx_dck_recover` always does.
fn rx_dck_path<I: AxIo + ?Sized>(io: &mut I, path: RfPath) -> Result<(bool, u8), AxError> {
    let reg5 = rf::read_rf(io, path, RR_RSV1, RFREG_MASK)?;
    rf::write_rf(io, path, RR_RSV1, RR_RSV1_RST, 0)?;
    rf::write_rf(io, path, RR_MOD, RR_MOD_MASK, RR_MOD_V_RX)?;
    // rtwdev->dbcc_en, false.
    rf::write_rf(io, path, RR_MOD, RR_MOD_LO_SEL, 0)?;
    let timed_out = set_rx_dck(io, path)?;
    let rewrites = rx_dck_recover(io, path)?;
    rf::write_rf(io, path, RR_RSV1, RFREG_MASK, reg5)?;
    Ok((timed_out, rewrites))
}

/// `_set_rx_dck` (rtw8852c_rfk.c:1738-1754): clear `RR_DCK1_CLR`, toggle,
/// and if `RR_DCKC_CHK` is set and `RR_DCK_DONE` is over 1, toggle again
/// with that value in `RR_RXBB2_IDAC`, then set it back to 1. Returns
/// whether either toggle's poll timed out.
fn set_rx_dck<I: AxIo + ?Sized>(io: &mut I, path: RfPath) -> Result<bool, AxError> {
    rf::write_rf(io, path, RR_DCK1, RR_DCK1_CLR, 0)?;
    let mut timed_out = rx_dck_toggle(io, path)?;
    if rf::read_rf(io, path, RR_DCKC, RR_DCKC_CHK)? == 0 {
        return Ok(timed_out);
    }
    let res = rf::read_rf(io, path, RR_DCK, RR_DCK_DONE)?;
    if res > 1 {
        rf::write_rf(io, path, RR_RXBB2, RR_RXBB2_IDAC, res)?;
        timed_out |= rx_dck_toggle(io, path)?;
        rf::write_rf(io, path, RR_RXBB2, RR_RXBB2_IDAC, 1)?;
    }
    Ok(timed_out)
}

/// `_rx_dck_toggle` (rtw8852c_rfk.c:1719-1736): pulse `RR_DCK_LV` and wait
/// for `RR_DCK1_DONE` with `read_poll_timeout_atomic(.., 2, 2000, false,
/// ..)`, at most 1001 samples. Returns whether the poll timed out.
fn rx_dck_toggle<I: AxIo + ?Sized>(io: &mut I, path: RfPath) -> Result<bool, AxError> {
    rf::write_rf(io, path, RR_DCK, RR_DCK_LV, 0)?;
    rf::write_rf(io, path, RR_DCK, RR_DCK_LV, 1)?;
    let timed_out = matches!(
        poll_sample_atomic(
            io,
            2,
            2000,
            false,
            |io| rf::read_rf(io, path, RR_DCK1, RR_DCK1_DONE),
            |v| v != 0,
        )?,
        Polled::Expired(_)
    );
    rf::write_rf(io, path, RR_DCK, RR_DCK_LV, 0)?;
    Ok(timed_out)
}

/// `_rx_dck_recover` (rtw8852c_rfk.c:1670-1717): for each of the two
/// groups, read the base offsets at `_dck_addr_bs` and `+1`, then the
/// target's at `_dck_addr` and `+1`, each target checked against its base
/// right after it is read. Returns how many targets were rewritten.
fn rx_dck_recover<I: AxIo + ?Sized>(io: &mut I, path: RfPath) -> Result<u8, AxError> {
    let mut rewrites = 0;
    for (bs, tg) in DCK_ADDR_BS.into_iter().zip(DCK_ADDR) {
        let even_bs = dck_offsets(io, path, bs)?;
        let odd_bs = dck_offsets(io, path, bs + 1)?;
        let even = dck_offsets(io, path, tg)?;
        rewrites += u8::from(rx_dck_fix_if_need(io, path, tg, even_bs, even)?);
        let odd = dck_offsets(io, path, tg + 1)?;
        rewrites += u8::from(rx_dck_fix_if_need(io, path, tg + 1, odd_bs, odd)?);
    }
    Ok(rewrites)
}

/// Select offset index `addr` in `RR_MOD_DCK` and read its `(i, q)`:
/// `RR_DCK_TIA` and `RR_DCK1_TIA`.
fn dck_offsets<I: AxIo + ?Sized>(io: &mut I, path: RfPath, addr: u8) -> Result<(u8, u8), AxError> {
    rf::write_rf(io, path, RR_MOD, RR_MOD_DCK, addr.into())?;
    // 7-bit fields.
    let i = rf::read_rf(io, path, RR_DCK, RR_DCK_TIA)? as u8;
    let q = rf::read_rf(io, path, RR_DCK1, RR_DCK1_TIA)? as u8;
    Ok((i, q))
}

/// `_rx_dck_fix_if_need` (rtw8852c_rfk.c:1645-1668): pass when the i delta
/// is under 10 and the q delta at most 10 (Linux's asymmetric test).
/// Otherwise a value more than 10 off takes its base's, and the pair is
/// rewritten even if neither is (a delta of exactly 10 on i). Returns
/// whether it rewrote.
fn rx_dck_fix_if_need<I: AxIo + ?Sized>(
    io: &mut I,
    path: RfPath,
    addr: u8,
    (i_bs, q_bs): (u8, u8),
    (i, q): (u8, u8),
) -> Result<bool, AxError> {
    // C's abs() of the u8s promoted to int.
    let di = (i32::from(i_bs) - i32::from(i)).abs();
    let dq = (i32::from(q_bs) - i32::from(q)).abs();
    if di < RX_DCK_TH && dq <= RX_DCK_TH {
        return Ok(false);
    }
    let i = if di > RX_DCK_TH { i_bs } else { i };
    let q = if dq > RX_DCK_TH { q_bs } else { q };
    rx_dck_value_rewrite(io, path, addr, i, q)?;
    Ok(true)
}

/// `_rx_dck_value_rewrite` (rtw8852c_rfk.c:1570-1595): write the 7-bit
/// `(i, q)` halved into the 6-bit `RR_LUTWD0_LB`/`_MB` of LUT entry `addr`,
/// the data word twice, between the LUT write enables. Linux reads nothing
/// back: its "Final" debug print decodes the value it wrote.
fn rx_dck_value_rewrite<I: AxIo + ?Sized>(
    io: &mut I,
    path: RfPath,
    addr: u8,
    i: u8,
    q: u8,
) -> Result<(), AxError> {
    let v = ((u32::from(q >> 1) << 6) & RR_LUTWD0_MB) | (u32::from(i >> 1) & RR_LUTWD0_LB);
    rf::write_rf(io, path, RR_LUTPLL, RR_CAL_RW, 1)?;
    rf::write_rf(io, path, RR_RFC, RR_WCAL, 1)?;
    rf::write_rf(io, path, RR_DCK, RR_DCK_FINE, 1)?;
    rf::write_rf(io, path, RR_LUTWA, MASKBYTE0, addr.into())?;
    rf::write_rf(io, path, RR_LUTWD0, RFREG_MASK, v)?;
    rf::write_rf(io, path, RR_LUTWD0, RFREG_MASK, v)?;
    rf::write_rf(io, path, RR_DCK, RR_DCK_FINE, 0)?;
    rf::write_rf(io, path, RR_RFC, RR_WCAL, 0)?;
    rf::write_rf(io, path, RR_LUTPLL, RR_CAL_RW, 0)
}

#[cfg(test)]
mod tests;

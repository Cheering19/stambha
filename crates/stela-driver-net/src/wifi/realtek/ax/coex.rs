// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
//! The hardware end state of Linux's Bluetooth coexistence start for an
//! RTL8852C whose BT is off, as `rtw89_btc_ntfy_init(BTC_MODE_NORMAL)`
//! leaves it (pinned rtw89/coex.c:8723). Only registers are written; the
//! coex firmware commands (the H2Cs of `_fw_set_drv_info`, `btc_fw_set_monreg`
//! and the slot and policy sets) are not sent, and the driver's own coex
//! state is not kept. [`init`] runs, in this order:
//! 1. `rtw8852c_btc_init_cfg` (rtw8852c.c:2693-2736): `rtw89_mac_coex_init_v1`
//!    (mac.c:6461-6482) for the RTK mode, `rtw8852c_btc_set_wl_pri` for the
//!    TX response and beacon (rtw8852c.c:2738-2759), `RR_WLSEL` cleared on
//!    both paths, the TRX mask table of `rtw8852c_set_trx_mask`
//!    (rtw8852c.c:2685-2691), the PTA break table and the BT counter;
//! 2. `_action_wl_init` (coex.c:5721): `_set_ant(BTC_ANT_WINIT)` with BT
//!    seen as off, which is `_set_gnt(WL SW high, BT SW low)`
//!    (coex.c:5650-5655, 3616-3747) through `rtw89_mac_cfg_gnt_v1`
//!    (mac.c:6530-6580) into `R_AX_GNT_SW_CTRL`, then `_set_bt_plut`
//!    (coex.c:5572-5601) through `rtw89_mac_cfg_plt_ax` (mac.c:6583-6606);
//! 3. `_set_wl_preagc_ctrl` (coex.c:6257-6340), which with BT off reaches
//!    `rtw8852c_ctrl_nbtg_bt_tx(false)` (rtw8852c.c:2522-2550);
//! 4. the scoreboard write queued by `_write_scbd` and sent in
//!    `_action_common` (coex.c:6974-7009) as `rtw89_mac_cfg_sb`
//!    (mac.c:6608-6627).
//!
//! BT is off because `rtw89_btc_ntfy_init` sets `bt0.enable.now`, but
//! `_run_coex(BTC_RSN_NTFY_INIT)` first runs `_update_bt_scbd` (coex.c:8467),
//! which takes the BT scoreboard from `bt->scbd_c2h`; no BT C2H has arrived
//! at a first start, so `BTC_BSCB_ON` is clear. Every register here is
//! reached directly or through RF (SWSI); no BT-side indirect register is
//! written, so nothing here needs its own poll.
//!
//! This module grants no coex H2C, no BT-on grant, no TDMA or policy, no
//! second MAC and no RFK beyond [`rfk_notify`]'s register end state.
use super::bb;
use super::cmac::check_cmac_en;
use super::elements::RfPath;
use super::io::{AxError, AxIo, clear16, field32, set16, set32};
use super::rf::{self, RFREG_MASK};

/// `R_AX_BTC_CFG` (reg.h:3618): `B_AX_BTC_EN` b31, `B_AX_BTG_LNA1_GAIN_SEL`
/// b16, `B_AX_BTC_MODE_MASK` [25:24] (`MAC_AX_RTK_MODE` is 0).
const R_AX_BTC_CFG: u32 = 0xDA00;
const BTC_EN_BTG_LNA1: u32 = 0x8001_0000;
const B_AX_BTC_MODE_MASK: u32 = 0x0300_0000;
const MAC_AX_RTK_MODE: u32 = 0;
/// `R_AX_RTK_MODE_CFG_V1` (reg.h:3635), `B_AX_SAMPLE_CLK_MASK` [7:0] and
/// `MAC_AX_RTK_RATE`.
const R_AX_RTK_MODE_CFG_V1: u32 = 0xDA04;
const B_AX_SAMPLE_CLK_MASK: u32 = 0xFF;
const MAC_AX_RTK_RATE: u32 = 5;
/// `R_AX_BT_CNT_CFG` (reg.h:3648): `B_AX_BT_CNT_EN` b0, `B_AX_BT_CNT_RST_V1`
/// b1.
const R_AX_BT_CNT_CFG: u32 = 0xDA10;
const BT_CNT_EN: u32 = 1;
const BT_CNT_EN_RST: u32 = 3;
/// `R_AX_CCA_CFG_0` (reg.h:2279): `B_AX_BTCCA_EN` b5,
/// `B_AX_BTCCA_BRK_TXOP_EN` b9.
const R_AX_CCA_CFG_0: u32 = 0xC340;
const B_AX_BTCCA_EN: u16 = 1 << 5;
const B_AX_BTCCA_BRK_TXOP_EN: u16 = 1 << 9;
/// `R_BTC_COEX_WL_REQ` (reg.h:3661): `B_BTC_RSP_ACK_HI` b10,
/// `B_BTC_TX_BCN_HI` b22.
const R_BTC_COEX_WL_REQ: u32 = 0xDA24;
const B_BTC_RSP_ACK_HI: u32 = 1 << 10;
const B_BTC_TX_BCN_HI: u32 = 1 << 22;
/// `R_AX_BT_BREAK_TABLE` (reg.h:3698) and `BTC_BREAK_PARAM` (reg.h:3671).
const R_AX_BT_BREAK_TABLE: u32 = 0xDA44;
const BTC_BREAK_PARAM: u32 = 0xF0FF_FFFF;
/// `R_AX_GNT_SW_CTRL` (reg.h:3707) and `R_AX_GNT_VAL_V1` (reg.h:3737).
const R_AX_GNT_SW_CTRL: u32 = 0xDA48;
const R_AX_GNT_VAL_V1: u32 = 0xDA4C;
/// `R_AX_BT_PLT` (reg.h:2623).
const R_AX_BT_PLT: u32 = 0xC67C;
/// `R_AX_SCOREBOARD` (reg.h:156) and its fields (reg.h:157-162).
const R_AX_SCOREBOARD: u32 = 0x00AC;
const B_AX_TOGGLE: u32 = 1 << 31;
const SB_FW_SHIFT: u32 = 24;
const B_MAC_AX_SB_FW_MASK: u32 = 0x7F00_0000;
const B_MAC_AX_SB_DRV_MASK: u32 = 0x00FF_FFFF;
const B_MAC_AX_BTGS1_NOTIFY: u32 = 1;
const MAC_AX_NOTIFY_TP_MAJOR: u32 = 0x81;
/// `fsleep(1000)` after a scoreboard write (mac.c:6626).
const SB_DELAY_US: u64 = 1000;

/// `RR_WLSEL`, `RR_LUTWE`, `RR_LUTWA` and `RR_LUTWD0` (reg.h:8563, 8808,
/// 8627, 8632).
const RR_WLSEL: u32 = 0x02;
const RR_LUTWE: u32 = 0xEF;
const RR_LUTWA: u32 = 0x33;
const RR_LUTWD0: u32 = 0x3F;
const LUTWE_OPEN: u32 = 0x2_0000;
/// `BTC_BT_SS_GROUP` and `BTC_BT_TX_GROUP` (coex.h:96-97).
const BT_SS_GROUP: u32 = 0;
const BT_TX_GROUP: u32 = 2;

/// The `R_AX_GNT_SW_CTRL` value of `rtw89_mac_cfg_gnt_v1` for both bands
/// with WL at SW high and BT at SW low: WL RFC S0/S1, RX, TX and BB VAL and
/// SWCTRL, BT RFC S0/S1, RX and TX SWCTRL with VAL 0, and `WL_ACT_SWCTRL`
/// (reg.h:3707-3729; mac.c:6530-6580).
pub const GNT_WL_ONLY: u32 = 0x0005_7DDF;
/// `B_AX_TX_PLT_GNT_WL | B_AX_RX_PLT_GNT_WL | B_AX_PLT_EN` (reg.h:2627-2635;
/// mac.c:6583-6606).
pub const PLT_GNT_WL: u16 = 0x0111;
/// `BTC_WSCB_ACTIVE | BTC_WSCB_ON | BTC_WSCB_BTLOG` (coex.c:805-818).
pub const SCBD_WL_ON: u32 = 0x4003;
/// `BTC_WSCB_WLRFK` (coex.c:815).
pub const SCBD_WLRFK: u32 = 1 << 11;

/// `rtw8852c_set_trx_mask` (rtw8852c.c:2685-2691).
fn set_trx_mask<I: AxIo + ?Sized>(
    io: &mut I,
    path: RfPath,
    group: u32,
    val: u32,
) -> Result<(), AxError> {
    rf::write_rf(io, path, RR_LUTWE, RFREG_MASK, LUTWE_OPEN)?;
    rf::write_rf(io, path, RR_LUTWA, RFREG_MASK, group)?;
    rf::write_rf(io, path, RR_LUTWD0, RFREG_MASK, val)?;
    rf::write_rf(io, path, RR_LUTWE, RFREG_MASK, 0)
}

/// `rtw8852c_btc_set_rfe`'s antenna type (rtw8852c.c:2609-2635): shared when
/// the RFE type is 0 or odd, dedicated when it is even and above 0.
fn shared_antenna(rfe: u8) -> bool {
    rfe == 0 || rfe % 2 == 1
}

/// `rtw89_mac_cfg_gnt_v1` for WL SW high and BT SW low, then
/// `rtw89_mac_cfg_plt_ax` for band 0, which first checks the CMAC.
fn grant_wl<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    io.write32(R_AX_GNT_SW_CTRL, GNT_WL_ONLY)?;
    check_cmac_en(io)?;
    io.write16(R_AX_BT_PLT, PLT_GNT_WL)
}

/// `rtw8852c_ctrl_nbtg_bt_tx(false, PHY0)` (rtw8852c.c:2522-2550, the else
/// branch at :2536-2549).
fn nbtg_bt_tx_off<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    bb::field32(io, 0x4C00, 0x3, 0)?;
    bb::field32(io, 0x4CC4, 0x3, 0)?;
    bb::field32(io, 0x4AD4, u32::MAX, 0x60)?;
    bb::field32(io, 0x4AE0, u32::MAX, 0x60)?;
    bb::field32(io, 0x4688, 0xFF00_0000, 0x1A)?;
    bb::field32(io, 0x476C, 0xFF00_0000, 0x1A)?;
    bb::field32(io, 0x4694, 0x0000_00FF, 0x2A)?;
    bb::field32(io, 0x4694, 0x0000_FF00, 0x2A)?;
    bb::field32(io, 0x4778, 0x0000_00FF, 0x2A)?;
    bb::field32(io, 0x4778, 0x0000_FF00, 0x2A)?;
    bb::field32(io, 0x4AE4, 0x00FF_FFFF, 0x79_E99E)?;
    bb::field32(io, 0x4AEC, 0x00FF_FFFF, 0x79_E99E)?;
    bb::field32(io, 0x469C, 0xFC00_0000, 0x26)?;
    bb::field32(io, 0x49F0, 0xFC00_0000, 0x26)
}

/// `rtw89_mac_cfg_sb` (mac.c:6608-6627) with the chip powered on: the
/// firmware's bits (b24-30) are kept with `B_MAC_AX_BTGS1_NOTIFY` cleared
/// and `MAC_AX_NOTIFY_TP_MAJOR` set, the driver's 24 bits become `scbd`, and
/// the toggle bit is set, then 1 ms is waited so the BT firmware sees it.
fn scoreboard<I: AxIo + ?Sized>(io: &mut I, scbd: u32) -> Result<(), AxError> {
    let rd = io.read32(R_AX_SCOREBOARD)?;
    let fw = (((rd & B_MAC_AX_SB_FW_MASK) >> SB_FW_SHIFT) & !B_MAC_AX_BTGS1_NOTIFY)
        | MAC_AX_NOTIFY_TP_MAJOR;
    io.write32(
        R_AX_SCOREBOARD,
        B_AX_TOGGLE | ((fw << SB_FW_SHIFT) & B_MAC_AX_SB_FW_MASK) | (scbd & B_MAC_AX_SB_DRV_MASK),
    )?;
    io.delay_us(SB_DELAY_US);
    Ok(())
}

/// The hardware end state of `rtw89_btc_ntfy_init(BTC_MODE_NORMAL)` with BT
/// off (see the module doc), for the efuse RFE type `rfe`. The CMAC must be
/// enabled (checked after the grant register is written, as Linux's
/// `rtw89_mac_cfg_plt_ax` does).
pub fn init<I: AxIo + ?Sized>(io: &mut I, rfe: u8) -> Result<(), AxError> {
    // rtw89_mac_coex_init_v1, RTK mode.
    set32(io, R_AX_BTC_CFG, BTC_EN_BTG_LNA1)?;
    set32(io, R_AX_BT_CNT_CFG, BT_CNT_EN)?;
    set16(io, R_AX_CCA_CFG_0, B_AX_BTCCA_EN)?;
    clear16(io, R_AX_CCA_CFG_0, B_AX_BTCCA_BRK_TXOP_EN)?;
    field32(io, R_AX_BTC_CFG, B_AX_BTC_MODE_MASK, MAC_AX_RTK_MODE)?;
    field32(
        io,
        R_AX_RTK_MODE_CFG_V1,
        B_AX_SAMPLE_CLK_MASK,
        MAC_AX_RTK_RATE,
    )?;
    // rtw8852c_btc_set_wl_pri: TX response, then beacon, high priority.
    set32(io, R_BTC_COEX_WL_REQ, B_BTC_RSP_ACK_HI)?;
    set32(io, R_BTC_COEX_WL_REQ, B_BTC_TX_BCN_HI)?;
    // RF gnt debug off.
    rf::write_rf(io, RfPath::A, RR_WLSEL, RFREG_MASK, 0)?;
    rf::write_rf(io, RfPath::B, RR_WLSEL, RFREG_MASK, 0)?;
    // The TRX mask table.
    if shared_antenna(rfe) {
        set_trx_mask(io, RfPath::A, BT_SS_GROUP, 0x5FF)?;
        set_trx_mask(io, RfPath::B, BT_SS_GROUP, 0x5FF)?;
        set_trx_mask(io, RfPath::A, BT_TX_GROUP, 0x5FF)?;
    } else {
        set_trx_mask(io, RfPath::A, BT_SS_GROUP, 0x5DF)?;
        set_trx_mask(io, RfPath::B, BT_SS_GROUP, 0x5DF)?;
    }
    // PTA break table, then the BT counter enable and reset.
    io.write32(R_AX_BT_BREAK_TABLE, BTC_BREAK_PARAM)?;
    set32(io, R_AX_BT_CNT_CFG, BT_CNT_EN_RST)?;
    // The WL-init grant, the pre-AGC off state, the scoreboard.
    grant_wl(io)?;
    nbtg_bt_tx_off(io)?;
    scoreboard(io, SCBD_WL_ON)
}

/// The register end state of `_ntfy_wl_rfk` (coex.c:9400-9462): on start the
/// WL grant again and the scoreboard with `BTC_WSCB_WLRFK` set, on stop the
/// scoreboard without it.
pub fn rfk_notify<I: AxIo + ?Sized>(io: &mut I, start: bool) -> Result<(), AxError> {
    if start {
        grant_wl(io)?;
        scoreboard(io, SCBD_WL_ON | SCBD_WLRFK)
    } else {
        scoreboard(io, SCBD_WL_ON)
    }
}

/// `R_AX_GNT_VAL_V1` (reg.h:3737-3741): `GNT_WL_RFC_S0` b1, `S1` b2,
/// `GNT_BT_RFC_S0` b3, `S1` b4.
pub fn gnt<I: AxIo + ?Sized>(io: &mut I) -> Result<u32, AxError> {
    io.read32(R_AX_GNT_VAL_V1)
}

/// `R_AX_GNT_SW_CTRL` (reg.h:3707) read back: [`GNT_WL_ONLY`] once [`init`]
/// has run, unless the chip changed it.
pub fn gnt_sw<I: AxIo + ?Sized>(io: &mut I) -> Result<u32, AxError> {
    io.read32(R_AX_GNT_SW_CTRL)
}

#[cfg(test)]
mod tests;

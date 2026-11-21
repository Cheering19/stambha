// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! `rtw89_mac_vif_init` (`mac.c:5045-5085`) for the one unassociated station
//! interface of M2c (spec S7): `mac_id` 0, port 0, MAC 0, net type NO_LINK,
//! a fresh mac80211 vif (`beacon_int`, `dtim_period`, BSS color 0, not
//! nontransmitted). In Linux's order:
//!
//! 1. `rtw89_mac_port_update` (`mac.c:5104-5135`) with its `port_cfg_*`
//!    helpers (`mac.c:4557-4975`), port 0 on MAC 0 so no port or band offset
//!    applies ([`port_update`], [`PortConfig::NO_LINK`]). The association
//!    runs it again for the infrastructure BSS joined
//!    ([`PortConfig::infra`], M3 spec S10.7, from
//!    [`super::assoc::associate`]).
//! 2. `rtw89_mac_dmac_tbl_init` and `rtw89_mac_cmac_tbl_init`
//!    (`mac.c:4403-4435`) by indirect access ([`tbl_init`]). Both return
//!    early under secure boot; the bring-up refuses secure-boot firmware
//!    (`SecureBootUnsupported`), so they always run here.
//! 3. `rtw89_mac_set_macid_pause(0, false)` (`mac.c:4437-4457`): the CMAC
//!    check, then `rtw89_fw_h2c_macid_pause` (`fw.c:5180-5235`).
//! 4. `rtw89_fw_h2c_role_maintain(CREATE)` (`fw.c:4949-5003`),
//!    `rtw89_fw_h2c_join_info(dis_conn)` (`fw.c:5045-5143`), `rtw89_cam_init`
//!    (software only, `cam.c:741-761`: address CAM 0 and BSSID CAM 0, which
//!    the builder fixes), `rtw89_fw_h2c_cam(CREATE)` (`fw.c:2290-2335`) and
//!    `rtw89_fw_h2c_default_cmac_tbl` (`fw.c:3612-3661`, 8852C's
//!    `h2c_default_cmac_tbl`). 8852C's `h2c_default_dmac_tbl` is NULL
//!    (`rtw8852c.c:3118`), so nothing follows.
//!
//! Every H2C goes through the [`Session`] with no wait, as `rtw89_h2c_tx`
//! sends them; their acks are recorded, not waited on (spec S1.14). Linux
//! ignores the macid pause H2C's own result (`mac.c:4454-4456`); a failed CH12
//! submit fails here instead, as `core_tail` does, since every later H2C
//! would fail the same way.
//!
//! This module grants no register access beyond port 0's registers of spec
//! S7.1, 0x0C04, 0x40000-0x4001C and the CMAC check's read of 0xC000; no
//! H2C beyond these five; no wait beyond `func_sw`'s `beacon_int + 1` ms
//! and the 20 µs before the beacon early time; no AP, ad hoc or mesh path
//! (beacon drop, beacon TX, the high queue window), and no frame delivery:
//! frames drained during it are dropped. The associated station's port
//! (INFRA: TSF sync and BSSID match on) is [`port_update`]'s only other
//! branch.
use super::cmac;
use super::command::Session;
use super::io::{AxError, AxIo, clear32, field16, field32, set8, set32};
use super::receive::Event;

/// The vif's `mac_id`, the first free one `rtw89_acquire_mac_id`
/// (`core.c:6771`) gives the first vif (spec S1).
pub const MAC_ID: u8 = 0;
/// The vif's `port`, `RTW89_PORT_0` (`core.h:329`) for the first vif.
pub const PORT: u8 = 0;

/// `R_AX_PORT_CFG_P0` and its bits (`reg.h:2381-2401`).
const R_AX_PORT_CFG_P0: u32 = 0xC400;
const B_AX_BRK_SETUP: u32 = 1 << 16;
const B_AX_TBTT_PROHIB_EN: u32 = 1 << 13;
const B_AX_BCNTX_EN: u32 = 1 << 12;
const B_AX_NET_TYPE_MASK: u32 = 0x0C00; // GENMASK(11, 10)
const B_AX_TSFTR_RST: u32 = 1 << 5;
const B_AX_RX_BSSID_FIT_EN: u32 = 1 << 4;
const B_AX_TSF_UDT_EN: u32 = 1 << 3;
const B_AX_PORT_FUNC_EN: u32 = 1 << 2;
const B_AX_TXBCN_RPT_EN: u32 = 1 << 1;
const B_AX_RXBCN_RPT_EN: u32 = 1 << 0;
/// `R_AX_TBTT_PROHIB_P0`: `TBTT_HOLD` [27:16], `TBTT_SETUP` [7:0]
/// (`reg.h:2403-2409`).
const R_AX_TBTT_PROHIB_P0: u32 = 0xC404;
const B_AX_TBTT_HOLD_MASK: u32 = 0x0FFF_0000;
const B_AX_TBTT_SETUP_MASK: u32 = 0x0000_00FF;
/// `R_AX_BCN_AREA_P0`: `BCN_MSK_AREA` [27:16] (`reg.h:2411-2416`).
const R_AX_BCN_AREA_P0: u32 = 0xC408;
const B_AX_BCN_MSK_AREA_MASK: u32 = 0x0FFF_0000;
/// `R_AX_BCNERLYINT_CFG_P0`: `BCNERLY` [11:0] (`reg.h:2419-2424`).
const R_AX_BCNERLYINT_CFG_P0: u32 = 0xC40C;
const B_AX_BCNERLY_MASK: u32 = 0x0FFF;
/// `R_AX_TBTTERLYINT_CFG_P0`: `TBTTERLY` [11:0] (`reg.h:2426-2431`).
const R_AX_TBTTERLYINT_CFG_P0: u32 = 0xC40E;
const B_AX_TBTTERLY_MASK: u16 = 0x0FFF;
/// `R_AX_TBTT_AGG_P0`: `TBTT_AGG_NUM` [15:8] (`reg.h:2433-2438`).
const R_AX_TBTT_AGG_P0: u32 = 0xC412;
const B_AX_TBTT_AGG_NUM_MASK: u16 = 0xFF00;
/// `R_AX_BCN_SPACE_CFG_P0`: `BCN_SPACE` [15:0] (`reg.h:2440-2446`).
const R_AX_BCN_SPACE_CFG_P0: u32 = 0xC414;
const B_AX_BCN_SPACE_MASK: u32 = 0xFFFF;
/// `R_AX_DTIM_CTRL_P0`: `DTIM_NUM` [15:8] (`reg.h:2480-2485`).
const R_AX_DTIM_CTRL_P0: u32 = 0xC426;
const B_AX_DTIM_NUM_MASK: u16 = 0xFF00;
/// `R_AX_BCN_CNT_TMR_P0` (`reg.h:2497`).
const R_AX_BCN_CNT_TMR_P0: u32 = 0xC434;
/// `R_AX_MBSSID_CTRL`: `P0MB_ALL` [23:1] (`reg.h:2526-2528`).
const R_AX_MBSSID_CTRL: u32 = 0xC568;
const B_AX_P0MB_ALL_MASK: u32 = 0x00FF_FFFE;
/// `R_AX_P0MB_HGQ_WINDOW_CFG_0`, port 0's `hiq_win` (`reg.h:2546`).
const R_AX_P0MB_HGQ_WINDOW_CFG_0: u32 = 0xC590;
/// `R_AX_MBSSID_DROP_0`: `PORT_DROP_4_0` [20:16] (`reg.h:2604-2608`).
const R_AX_MBSSID_DROP_0: u32 = 0xC63C;
const B_AX_PORT_DROP_4_0_SHIFT: u32 = 16;
/// `R_AX_PTCL_BSS_COLOR_0`: port 0's color [5:0] (`reg.h:2637-2642`).
const R_AX_PTCL_BSS_COLOR_0: u32 = 0xC6A0;
const B_AX_BSS_COLOB_AX_PORT_0_MASK: u32 = 0x3F;
/// `R_AX_MD_TSFT_STMP_CTL`: `UPD_HGQMD` b1, `UPD_TIMIE` b0
/// (`reg.h:2995-3000`).
const R_AX_MD_TSFT_STMP_CTL: u32 = 0xCA08;
const B_AX_UPD_HGQMD: u8 = 1 << 1;
const B_AX_UPD_TIMIE: u8 = 1 << 0;
/// `R_AX_BCN_PSR_RPT_P0`: `BCAID_P0` [10:0] (`reg.h:3406-3408`).
const R_AX_BCN_PSR_RPT_P0: u32 = 0xCE84;
const B_AX_BCAID_P0_MASK: u32 = 0x07FF;

/// The defaults of `mac.c:4547-4555`.
const BCN_INTERVAL: u32 = 100;
const BCN_ERLY_DEF: u32 = 160;
const BCN_SETUP_DEF: u32 = 2;
const BCN_HOLD_DEF: u32 = 200;
const BCN_MASK_DEF: u32 = 0;
const TBTT_ERLY_DEF: u16 = 5;
const TBTT_AGG_DEF: u16 = 1;
/// `BCN_ERLY_SET_DLY` (`10 * 2`), in µs as `fsleep` takes it.
const BCN_ERLY_SET_DLY_US: u64 = 10 * 2;

/// The vif's `bss_conf` `he_bss_color` and, not being nontransmitted,
/// `bssid_index`: 0 for a fresh vif (spec S16.7), and for an HT or legacy
/// BSS (M3 spec S1.7) that is no nontransmitted BSSID profile, the only
/// BSSes a station joins here.
const BSS_COLOR: u32 = 0;
const BSSID_INDEX: u32 = 0;
/// `RTW89_NET_TYPE_NO_LINK` and `RTW89_NET_TYPE_INFRA` (`core.h:250-255`).
pub const NET_TYPE_NO_LINK: u8 = 0;
pub const NET_TYPE_INFRA: u8 = 2;
/// `RTW89_ROLE_CREATE`, the first `enum rtw89_upd_mode` (`core.h:272-273`).
const RTW89_ROLE_CREATE: u8 = 0;

/// What `rtw89_mac_port_update` reads of the vif: its `net_type` and its
/// `bss_conf` `beacon_int` and `dtim_period`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PortConfig {
    /// `RTW89_NET_TYPE_*`: [`NET_TYPE_NO_LINK`] or [`NET_TYPE_INFRA`] (the
    /// AD_HOC and AP branches are not ported).
    pub net_type: u8,
    /// `bss_conf->beacon_int`, in TU.
    pub beacon_int: u16,
    /// `bss_conf->dtim_period`.
    pub dtim: u8,
}

impl PortConfig {
    /// The fresh, unassociated station vif of M2c (spec S7.1): NO_LINK,
    /// `beacon_int` and `dtim_period` 0.
    pub const NO_LINK: Self = Self {
        net_type: NET_TYPE_NO_LINK,
        beacon_int: 0,
        dtim: 0,
    };

    /// An associated station (M3 spec S10.7): INFRA with the BSS's beacon
    /// interval and DTIM period, as mac80211 set them before
    /// BSS_CHANGED_ASSOC (`mlme.c:7091-7095`; `dtim_period` is at least 1
    /// there).
    pub const fn infra(beacon_int: u16, dtim: u8) -> Self {
        Self {
            net_type: NET_TYPE_INFRA,
            beacon_int,
            dtim,
        }
    }
}

/// `R_AX_FILTER_MODEL_ADDR` (`reg.h:357`) selects the table word that
/// `R_AX_INDIR_ACCESS_ENTRY` (`mac.h:302`) then reaches.
const R_AX_FILTER_MODEL_ADDR: u32 = 0x0C04;
const R_AX_INDIR_ACCESS_ENTRY: u32 = 0x40000;
/// `DMAC_TBL_BASE_ADDR`, `CMAC_TBL_BASE_ADDR` and `CCTL_INFO_SIZE`
/// (`mac.h:310-353`).
const DMAC_TBL_BASE_ADDR: u32 = 0x1880_0000;
const CMAC_TBL_BASE_ADDR: u32 = 0x1884_0000;
const CCTL_INFO_SIZE: u32 = 32;
/// `rtw89_mac_cmac_tbl_init`'s eight words (`mac.c:4426-4434`).
const CMAC_TBL_DEFAULT: [u32; 8] = [0x4, 0x400A_0004, 0, 0, 0, 0x0E43_000B, 0, 0x000B_8109];

/// `rtw89_mac_port_cfg_func_sw` (`mac.c:4557-4603`) for 8852C and a non-AP
/// vif: nothing unless the port function is on; then, after
/// `msleep(beacon_int + 1)`, the port function and BRK_SETUP off, the TSF
/// reset and the beacon counter timer cleared.
fn cfg_func_sw<I: AxIo + ?Sized>(io: &mut I, beacon_int: u16) -> Result<(), AxError> {
    if io.read32(R_AX_PORT_CFG_P0)? & B_AX_PORT_FUNC_EN == 0 {
        return Ok(());
    }
    io.delay_us((u64::from(beacon_int) + 1) * 1000);
    clear32(io, R_AX_PORT_CFG_P0, B_AX_PORT_FUNC_EN | B_AX_BRK_SETUP)?;
    set32(io, R_AX_PORT_CFG_P0, B_AX_TSFTR_RST)?;
    io.write32(R_AX_BCN_CNT_TMR_P0, 0)
}

/// `rtw89_mac_port_cfg_*`'s `rtw89_write32_port_set` or `_clr` of `bits` in
/// 0xC400, as `on` asks.
fn port_cfg_bits<I: AxIo + ?Sized>(io: &mut I, bits: u32, on: bool) -> Result<(), AxError> {
    if on {
        set32(io, R_AX_PORT_CFG_P0, bits)
    } else {
        clear32(io, R_AX_PORT_CFG_P0, bits)
    }
}

/// `rtw89_mac_port_update` for port 0 of a station vif as `port` describes
/// it, NO_LINK (spec S7.1) or INFRA (M3 spec S10.7), in Linux's order:
///
/// - `cfg_func_sw`, sleeping `beacon_int + 1` ms when the port function is
///   on; `cfg_tx_rpt(false)` and `cfg_rx_rpt(false)`; `cfg_net_type`; then
///   the net-type tests: `cfg_bcn_prct` sets TBTT_PROHIB_EN and BRK_SETUP for
///   any linked type, `cfg_rx_sw` RX_BSSID_FIT_EN and
///   `cfg_rx_sync_by_nettype` TSF_UDT_EN for INFRA, and
///   `cfg_tx_sw_by_nettype` clears BCNTX_EN (no AP or ad hoc), each clearing
///   its 0xC400 bits otherwise.
/// - `cfg_bcn_intv` (the beacon interval, `BCN_INTERVAL` 100 for 0),
///   `cfg_hiq_win` (0, not AP), `cfg_hiq_dtim` (the DTIM period),
///   `cfg_hiq_drop`.
/// - The beacon setup and hold times, the mask area, the TBTT early time
///   and aggregation, the BSS color, `cfg_mbssid` (port 0, not AP).
/// - The port function on; `tsf_resync_all` does nothing without an AP vif
///   (`mac.c:5017-5042`); 20 µs; the beacon early time; the PSR report's
///   BSSID index.
///
/// Every masked write is the `rtw89_write*_mask`, `_set` or `_clr`
/// read-modify-write at Linux's width. Any other net type is refused with
/// `Mac` before any access: AD_HOC and AP mode are not ported.
pub fn port_update<I: AxIo + ?Sized>(io: &mut I, port: &PortConfig) -> Result<(), AxError> {
    let infra = match port.net_type {
        NET_TYPE_NO_LINK => false,
        NET_TYPE_INFRA => true,
        _ => {
            return Err(AxError::Mac(
                "port update for a net type other than NO_LINK or INFRA",
            ));
        }
    };
    cfg_func_sw(io, port.beacon_int)?;
    clear32(io, R_AX_PORT_CFG_P0, B_AX_TXBCN_RPT_EN)?;
    clear32(io, R_AX_PORT_CFG_P0, B_AX_RXBCN_RPT_EN)?;
    field32(
        io,
        R_AX_PORT_CFG_P0,
        B_AX_NET_TYPE_MASK,
        u32::from(port.net_type),
    )?;
    port_cfg_bits(io, B_AX_TBTT_PROHIB_EN | B_AX_BRK_SETUP, infra)?;
    port_cfg_bits(io, B_AX_RX_BSSID_FIT_EN, infra)?;
    port_cfg_bits(io, B_AX_TSF_UDT_EN, infra)?;
    clear32(io, R_AX_PORT_CFG_P0, B_AX_BCNTX_EN)?;
    let bcn_int = if port.beacon_int != 0 {
        u32::from(port.beacon_int)
    } else {
        BCN_INTERVAL
    };
    field32(io, R_AX_BCN_SPACE_CFG_P0, B_AX_BCN_SPACE_MASK, bcn_int)?;
    io.write8(R_AX_P0MB_HGQ_WINDOW_CFG_0, 0)?;
    set8(io, R_AX_MD_TSFT_STMP_CTL, B_AX_UPD_HGQMD | B_AX_UPD_TIMIE)?;
    field16(
        io,
        R_AX_DTIM_CTRL_P0,
        B_AX_DTIM_NUM_MASK,
        u16::from(port.dtim),
    )?;
    // `cfg_hiq_drop`: port 0's PORT_DROP bit and bit 0, one read and one
    // plain write.
    let drop = io.read32(R_AX_MBSSID_DROP_0)?;
    let drop = drop & !(1 << (B_AX_PORT_DROP_4_0_SHIFT + u32::from(PORT))) & !1;
    io.write32(R_AX_MBSSID_DROP_0, drop)?;
    field32(io, R_AX_TBTT_PROHIB_P0, B_AX_TBTT_SETUP_MASK, BCN_SETUP_DEF)?;
    field32(io, R_AX_TBTT_PROHIB_P0, B_AX_TBTT_HOLD_MASK, BCN_HOLD_DEF)?;
    field32(io, R_AX_BCN_AREA_P0, B_AX_BCN_MSK_AREA_MASK, BCN_MASK_DEF)?;
    field16(
        io,
        R_AX_TBTTERLYINT_CFG_P0,
        B_AX_TBTTERLY_MASK,
        TBTT_ERLY_DEF,
    )?;
    field16(io, R_AX_TBTT_AGG_P0, B_AX_TBTT_AGG_NUM_MASK, TBTT_AGG_DEF)?;
    field32(
        io,
        R_AX_PTCL_BSS_COLOR_0,
        B_AX_BSS_COLOB_AX_PORT_0_MASK,
        BSS_COLOR,
    )?;
    clear32(io, R_AX_MBSSID_CTRL, B_AX_P0MB_ALL_MASK)?;
    set32(io, R_AX_PORT_CFG_P0, B_AX_PORT_FUNC_EN)?;
    io.delay_us(BCN_ERLY_SET_DLY_US);
    field32(io, R_AX_BCNERLYINT_CFG_P0, B_AX_BCNERLY_MASK, BCN_ERLY_DEF)?;
    field32(io, R_AX_BCN_PSR_RPT_P0, B_AX_BCAID_P0_MASK, BSSID_INDEX)
}

/// `rtw89_mac_dmac_tbl_init(macid)` then `rtw89_mac_cmac_tbl_init(macid)`
/// (spec S7.2): the macid's four DMAC table words cleared, then its CMAC
/// table entry set to Linux's eight defaults, each word selected through
/// 0x0C04 and written at 0x40000 onwards. Plain writes, no reads.
pub fn tbl_init<I: AxIo + ?Sized>(io: &mut I, macid: u8) -> Result<(), AxError> {
    let macid = u32::from(macid);
    for i in 0..4 {
        io.write32(
            R_AX_FILTER_MODEL_ADDR,
            DMAC_TBL_BASE_ADDR + (macid << 4) + (i << 2),
        )?;
        io.write32(R_AX_INDIR_ACCESS_ENTRY, 0)?;
    }
    io.write32(
        R_AX_FILTER_MODEL_ADDR,
        CMAC_TBL_BASE_ADDR + macid * CCTL_INFO_SIZE,
    )?;
    for (i, word) in (0u32..).zip(CMAC_TBL_DEFAULT) {
        io.write32(R_AX_INDIR_ACCESS_ENTRY + 4 * i, word)?;
    }
    Ok(())
}

/// `rtw89_mac_vif_init` for M2c's station vif with local address `mac`
/// (spec S7): [`port_update`], [`tbl_init`] for [`MAC_ID`], the CMAC check
/// of `rtw89_mac_set_macid_pause` (a disabled or dead CMAC fails with
/// `Mac` before any H2C, as Linux returns its `-EFAULT`), then through
/// `session`, each submitted and followed by one RXQ drain: the macid
/// unpause, ROLE_MAINTAIN (CREATE), JOININFO (disconnected), the address CAM
/// and the default CMAC table. With M2b and `core_tail` before it they carry
/// seq 12 to 16. The first failure is returned and nothing after it runs.
///
/// # Safety
/// The contract of `Ch12::submit` and `Rxq::poll` for `io` and `session`'s
/// rings: `io` is the exclusive register window of the same RTL8852C
/// requester that `pci::pre_init` programmed with them, and the `pre_init`
/// contract still holds.
pub unsafe fn init<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    mac: [u8; 6],
) -> Result<(), AxError> {
    port_update(io, &PortConfig::NO_LINK)?;
    tbl_init(io, MAC_ID)?;
    // `rtw89_mac_set_macid_pause`: the H2C only if CMAC 0 is on
    // (`mac.c:4450-4452`).
    cmac::check_cmac_en(io)?;
    // Before the scan no frame is wanted.
    let drop = &mut |_: Event<'_>| {};
    // Each H2C takes its seq only when it is built, right before its send,
    // as `rtw89_h2c_pkt_set_hdr` does: a failed send leaves the later
    // ones unnumbered.
    let packet = session.h2c.macid_pause(MAC_ID, false);
    // SAFETY: the caller's contract is `send`'s, word for word.
    unsafe { session.send(io, packet, drop) }?;
    let packet = session.h2c.role_maintain(MAC_ID, PORT, RTW89_ROLE_CREATE);
    // SAFETY: as above.
    unsafe { session.send(io, packet, drop) }?;
    let packet = session.h2c.join_info_disconnect(MAC_ID, PORT);
    // SAFETY: as above.
    unsafe { session.send(io, packet, drop) }?;
    let packet = session.h2c.addr_cam_unassociated(mac, MAC_ID, PORT);
    // SAFETY: as above.
    unsafe { session.send(io, packet, drop) }?;
    let packet = session.h2c.default_cmac_tbl(MAC_ID);
    // SAFETY: as above.
    unsafe { session.send(io, packet, drop) }?;
    Ok(())
}

#[cfg(test)]
mod tests;

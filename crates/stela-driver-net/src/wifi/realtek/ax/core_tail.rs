// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! The end of `rtw89_core_start` (`core.c:6699-6721`) for an RTL8852C on
//! MAC 0 without DBCC, after `rtw89_phy_dm_init` (M2c spec S3):
//!
//! 1. `rtw89_mac_set_edcca_mode_bands`: nothing, `set_edcca_mode` is NULL on
//!    AX (`mac.c:7499`).
//! 2. `rtw89_mac_cfg_ppdu_status_bands(true)` (`mac.h:1441-1453`):
//!    `rtw89_mac_cfg_ppdu_status_ax` (`mac.c:6289-6311`), the CMAC check,
//!    then PPDU status reports to the host (`cmac::cfg_ppdu_status`).
//! 3. `rtw89_mac_cfg_phy_rpt_bands`: nothing, `cfg_phy_rpt` is NULL on AX
//!    (`mac.c:7498`).
//! 4. `rtw89_mac_update_rts_threshold` (`mac.c:6313-6350`): the default
//!    thresholds, `rts_threshold` being `(u32)-1` ([`update_rts_threshold`]).
//! 5. `rtw89_chip_rfk_init_late` is NULL for 8852C (`rtw8852c.c:3079`).
//! 6. `rtw89_fw_h2c_fw_log(false)` (`fw.c:2827-2862`) and
//!    `rtw89_fw_h2c_init_ba_cam` (`fw.h:5678-5684`), which for 8852C's
//!    `RTW89_BACAM_V0_EXT` is `rtw89_fw_h2c_init_dynamic_ba_cam_v0_ext`
//!    (`fw.c:2694-2706`): entries `bacam_num` (8) on, uids from 0, eight
//!    of them (`bacam_dynamic_num`, `rtw8852c.c:3221-3222`).
//! 7. `rtw89_tas_fw_timer_enable`: nothing on AX (`sar.c:851-863`).
//!
//! Linux ignores the results of steps 2 and 6; a disabled CMAC or a failed
//! CH12 submit fails here instead, since the scan after it could not work.
//! Their acks are recorded by the [`Session`], not waited on (spec S1.14).
//!
//! Not ported: `rtw89_hci_start` (interrupts, spec S1.8), the track works,
//! `RTW89_FLAG_RUNNING` (Stela has no C2H gate) and
//! `rtw89_btc_ntfy_radio_state(WL_ON)` (spec S1.7). This module grants no
//! register access beyond 0xC000, 0xCE40, 0x9C18 and 0xC614, no H2C beyond
//! these nine, no wait and no frame delivery: frames drained before the
//! scan are dropped.
use super::cmac;
use super::command::Session;
use super::io::{AxError, AxIo, field16};
use super::receive::Event;

/// `R_AX_AGG_LEN_HT_0` (`reg.h:2569-2573`): `RTS_TXTIME_TH` [15:8] and
/// `RTS_LEN_TH` [7:0].
const R_AX_AGG_LEN_HT_0: u32 = 0xC614;
const B_AX_RTS_TXTIME_TH_MASK: u16 = 0xFF00;
const B_AX_RTS_LEN_TH_MASK: u16 = 0x00FF;
/// `MAC_AX_TIME_TH_DEF` 88 >> `MAC_AX_TIME_TH_SH` 5, and `MAC_AX_LEN_TH_DEF`
/// 4080 >> `MAC_AX_LEN_TH_SH` 4 (`mac.c:6316-6337`), both under their
/// maximum of 255.
const RTS_TIME_TH: u16 = 88 >> 5;
const RTS_LEN_TH: u16 = 4080 >> 4;
/// 8852C's `bacam_num` and `bacam_dynamic_num` (`rtw8852c.c:3221-3222`).
const BACAM_NUM: u8 = 8;
const BACAM_DYNAMIC_NUM: u8 = 8;

/// `__rtw89_mac_update_rts_threshold(RTW89_MAC_0)` with `rts_threshold`
/// `(u32)-1`: two 16-bit read-modify-writes of 0xC614, the TX time threshold
/// `[15:8]` = 2, then the length threshold `[7:0]` = 255.
pub fn update_rts_threshold<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    field16(io, R_AX_AGG_LEN_HT_0, B_AX_RTS_TXTIME_TH_MASK, RTS_TIME_TH)?;
    field16(io, R_AX_AGG_LEN_HT_0, B_AX_RTS_LEN_TH_MASK, RTS_LEN_TH)
}

/// Spec S3 in Linux order: PPDU status reports on, the RTS threshold, then
/// the firmware log H2C and the eight BA CAM H2Cs through `session`, each
/// submitted and followed by one RXQ drain. The first failure is returned
/// and nothing after it runs.
///
/// # Safety
/// The contract of `Ch12::submit` and `Rxq::poll` for `io` and `session`'s
/// rings: `io` is the exclusive register window of the same RTL8852C
/// requester that `pci::pre_init` programmed with them, and the `pre_init`
/// contract still holds.
pub unsafe fn run<I: AxIo + ?Sized>(io: &mut I, session: &mut Session<'_>) -> Result<(), AxError> {
    cmac::cfg_ppdu_status(io, true)?;
    update_rts_threshold(io)?;
    // Before the scan no frame is wanted.
    let drop = &mut |_: Event<'_>| {};
    let packet = session.h2c.fw_log();
    // SAFETY: the caller's contract is `send`'s, word for word.
    unsafe { session.send(io, packet, drop) }?;
    for i in 0..BACAM_DYNAMIC_NUM {
        let packet = session.h2c.ba_cam_init(BACAM_NUM + i, i);
        // SAFETY: as above.
        unsafe { session.send(io, packet, drop) }?;
    }
    Ok(())
}

#[cfg(test)]
mod tests;

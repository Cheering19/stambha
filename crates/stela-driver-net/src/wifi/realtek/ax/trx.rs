// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE TRX init, from `trx_init_ax` (`mac.c:4029`): `dmac::init`, then
//! `cmac::init`, then `enable_imr_ax` for the DMAC (`mac.c:3936`, with
//! `rtw89_wdrls_imr_enable` through `rtw89_bbrpt_imr_enable`, `:3739-3862`)
//! and for CMAC band 0 (`rtw89_scheduler_imr_enable` through
//! `rtw89_tmac_imr_enable`, `:3864-3934`), `err_imr_ctrl_ax(true)` (`:3974`),
//! `set_host_rpr_ax` (`:4009`, PCIe) and the RTL8852C's clear of
//! `B_AX_RSP_STATIC_RTS_CHK_SERV_BW_EN` (`:4075`). The masks are
//! `rtw8852c_imr_info` (`rtw8852c.c:168`) with the values from pinned rtw89
//! `reg.h`. Only the RTL8852C, PCIe, band 0 branches are ported: the
//! `dbcc_enable_ax` branch is skipped as the SCC layout is not DBCC, and
//! `err_imr_ctrl_ax` skips the C1 error IMR because `c1_rx_qta` is 0. This
//! module grants no band 1, no interrupt delivery (`R_AX_HIMR` and the PCI
//! interrupt masks stay untouched), no TX or RX path and no firmware command;
//! the bring-up runs it after `sys::sys_init`. No band 1 CMAC register is
//! touched. `R_AX_TXPKTCTL_B1_ERRFLAG_IMR` is a DMAC register, which
//! `rtw89_mac_txpktctl_imr_enable` (`mac.c:3811-3816`) writes unconditionally
//! as well, so it is written here as Linux does.
use super::cmac::{self, check_cmac_en};
use super::dmac::{self, check_dmac_en};
use super::io::{AxError, AxIo, clear32, field32, set32};

// DMAC IMR registers and masks (`reg.h`, `rtw8852c_imr_info`).
const R_AX_WDRLS_ERR_IMR: u32 = 0x9430;
const WDRLS_IMR: u32 = 0x3337; // B_AX_WDRLS_IMR_EN_CLR == B_AX_WDRLS_IMR_SET_V1
const R_AX_SEC_ERROR_FLAG_IMR: u32 = 0x9D2C;
const SEC_ERROR_IMR_SET: u32 = 0x3; // B_AX_TX_HANG_IMR | B_AX_RX_HANG_IMR
const R_AX_MPDU_TX_ERR_IMR: u32 = 0x9BF4;
/// `B_AX_TX_GET_ERRPKTID_INT_EN` .. `B_AX_TX_HDR3_SIZE_ERR_INT_EN` (1-5).
const MPDU_TX_IMR_CLR: u32 = 0x3E;
/// `B_AX_TX_ETH_TYPE_ERR_EN` .. `B_AX_TX_KSRCH_ERR_EN` (6-9), RTL8852C only.
const MPDU_TX_IMR_CLR_8852C: u32 = 0x3C0;
const MPDU_TX_IMR_SET: u32 = 0x36E; // B_AX_MPDU_TX_IMR_SET_V1
const R_AX_MPDU_RX_ERR_IMR: u32 = 0x9CF4;
/// `B_AX_GETPKTID_ERR_INT_EN | MHDRLEN_ERR_INT_EN | RPT_ERR_INT_EN`.
const MPDU_RX_IMR_CLR: u32 = 0xB;
const MPDU_RX_IMR_SET: u32 = 0x8; // B_AX_MPDU_RX_IMR_SET_V1
const R_AX_STA_SCHEDULER_ERR_IMR: u32 = 0x9EF0;
const STA_SCH_IMR: u32 = 0x7; // clear mask and B_AX_STA_SCHEDULER_IMR_SET
const R_AX_TXPKTCTL_B0_ERRFLAG_IMR: u32 = 0x9F78;
const TXPKTCTL_B0_CLR: u32 = 0x0035_0F0F;
const TXPKTCTL_B0_SET: u32 = 0x0035_0B03;
const R_AX_TXPKTCTL_B1_ERRFLAG_IMR: u32 = 0x9FB8;
const TXPKTCTL_B1_CLR: u32 = 0x0035_0F0F;
const TXPKTCTL_B1_SET: u32 = 0x0035_0F03;
const R_AX_WDE_ERR_IMR: u32 = 0x8C38;
const R_AX_PLE_ERR_IMR: u32 = 0x9038;
/// `B_AX_{WDE,PLE}_IMR_{CLR,SET}_V1` are all the same mask.
const WDE_PLE_IMR: u32 = 0x3F0F_F3FF;
const R_AX_PKTIN_ERR_IMR: u32 = 0x9A20;
const B_AX_PKTIN_GETPKTID_ERR_INT_EN: u32 = 1 << 0;
const R_AX_HOST_DISPATCHER_ERR_IMR: u32 = 0x8850;
const HOST_DISP_IMR_CLR: u32 = 0xFFD7_FFFF;
const HOST_DISP_IMR_SET: u32 = 0x0C10_0060;
const R_AX_CPU_DISPATCHER_ERR_IMR: u32 = 0x8854;
const CPU_DISP_IMR_CLR: u32 = 0x7F7B_FFFD;
const CPU_DISP_IMR_SET: u32 = 0x3600_0060;
const R_AX_OTHER_DISPATCHER_ERR_IMR: u32 = 0x8858;
const OTHER_DISP_IMR_CLR: u32 = 0xFFFF_DFDF;
const OTHER_DISP_IMR_SET: u32 = 0x303C_C000;
const R_AX_CPUIO_ERR_IMR: u32 = 0x9840;
const CPUIO_IMR: u32 = 0x1111; // B_AX_CPUIO_IMR_CLR == B_AX_CPUIO_IMR_SET
const R_AX_BBRPT_COM_ERR_IMR: u32 = 0x9608;
const B_AX_BBRPT_COM_NULL_PLPKTID_ERR_INT_EN: u32 = 1 << 0;
const R_AX_BBRPT_CHINFO_ERR_IMR: u32 = 0x9628;
const BBRPT_CHINFO_IMR: u32 = 0xFF; // B_AX_BBRPT_CHINFO_IMR_CLR == ..._SET_V1
const R_AX_BBRPT_DFS_ERR_IMR: u32 = 0x9638;
const B_AX_BBRPT_DFS_TO_ERR_INT_EN: u32 = 1 << 0;
const R_AX_LA_ERRFLAG: u32 = 0x966C;
const B_AX_LA_IMR_DATA_LOSS_ERR: u32 = 1 << 0;

// CMAC band 0 IMR registers and masks.
const R_AX_SCHEDULE_ERR_IMR: u32 = 0xC3E8;
const B_AX_SORT_NON_IDLE_ERR_INT_EN: u32 = 1 << 1;
const B_AX_FSM_TIMEOUT_ERR_INT_EN: u32 = 1 << 0;
const R_AX_PTCL_IMR0: u32 = 0xC6C0;
const PTCL_IMR: u32 = 0x3;
const R_AX_RX_ERR_FLAG_IMR: u32 = 0xC804;
const CDMA_RX_IMR: u32 = 0x7FFF_FF06;
const R_AX_TX_ERR_FLAG_IMR: u32 = 0xC870;
const CDMA_TX_IMR: u32 = 0xFF00_C000;
const R_AX_PHYINFO_ERR_IMR_V1: u32 = 0xCCF8;
const PHYINFO_IMR: u32 = 0x3F;
const R_AX_RX_ERR_IMR: u32 = 0xCEF8;
const RMAC_IMR: u32 = 0x3FF;
const R_AX_TRXPTCL_ERROR_INDICA_MASK: u32 = 0xCCBC;
const TMAC_IMR_CLR: u32 = 0x1FF;
const TMAC_IMR_SET: u32 = 0x17F;

// `err_imr_ctrl_ax(true)`: `DMAC_ERR_IMR_EN` and `CMAC0_ERR_IMR_EN` are both
// GENMASK(31, 0).
const R_AX_DMAC_ERR_IMR: u32 = 0x8520;
const R_AX_CMAC_ERR_IMR: u32 = 0xC160;
const ERR_IMR_EN: u32 = 0xFFFF_FFFF;

// `set_host_rpr_ax`.
const R_AX_WDRLS_CFG: u32 = 0x9408;
const B_AX_WDRLS_MODE_MASK: u32 = 0x3; // GENMASK(1, 0)
const RTW89_RPR_MODE_POH: u32 = 0; // core.h:5906
const R_AX_RLSRPT0_CFG0: u32 = 0x9410;
const B_AX_RLSRPT0_FLTR_MAP_MASK: u32 = 0x0F00_0000; // GENMASK(27, 24)
const R_AX_RLSRPT0_CFG1: u32 = 0x9414;
const B_AX_RLSRPT0_AGGNUM_MASK: u32 = 0xFF; // GENMASK(7, 0)
const B_AX_RLSRPT0_TO_MASK: u32 = 0x00FF_0000; // GENMASK(23, 16)
const RLSRPT0_AGGNUM: u32 = 30;
const RLSRPT0_TO: u32 = 255;

// The RTL8852C tail of `trx_init_ax`.
const R_AX_RSP_CHK_SIG: u32 = 0xCC00;
const B_AX_RSP_STATIC_RTS_CHK_SERV_BW_EN: u32 = 1 << 30;

/// Why `trx_init_ax` failed: the step that did, and the error it raised.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TrxError {
    /// `"dmac init"`, `"cmac init"` or `"imr"`.
    pub step: &'static str,
    pub error: AxError,
}

fn at(step: &'static str) -> impl Fn(AxError) -> TrxError {
    move |error| TrxError { step, error }
}

/// `rtw89_write32_clr(reg, clr)` then `rtw89_write32_set(reg, set)`.
fn clr_set<I: AxIo + ?Sized>(io: &mut I, reg: u32, clr: u32, set: u32) -> Result<(), AxError> {
    clear32(io, reg, clr)?;
    set32(io, reg, set)
}

/// `enable_imr_ax(rtwdev, RTW89_MAC_0, RTW89_DMAC_SEL)` (`mac.c:3936`),
/// RTL8852C.
fn enable_imr_dmac<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_dmac_en(io)?;
    // rtw89_wdrls_imr_enable
    clr_set(io, R_AX_WDRLS_ERR_IMR, WDRLS_IMR, WDRLS_IMR)?;
    // rtw89_wsec_imr_enable
    set32(io, R_AX_SEC_ERROR_FLAG_IMR, SEC_ERROR_IMR_SET)?;
    // rtw89_mpdu_trx_imr_enable, with its RTL8852C second clear
    clear32(io, R_AX_MPDU_TX_ERR_IMR, MPDU_TX_IMR_CLR)?;
    clear32(io, R_AX_MPDU_TX_ERR_IMR, MPDU_TX_IMR_CLR_8852C)?;
    set32(io, R_AX_MPDU_TX_ERR_IMR, MPDU_TX_IMR_SET)?;
    clr_set(io, R_AX_MPDU_RX_ERR_IMR, MPDU_RX_IMR_CLR, MPDU_RX_IMR_SET)?;
    // rtw89_sta_sch_imr_enable
    clr_set(io, R_AX_STA_SCHEDULER_ERR_IMR, STA_SCH_IMR, STA_SCH_IMR)?;
    // rtw89_txpktctl_imr_enable
    clr_set(
        io,
        R_AX_TXPKTCTL_B0_ERRFLAG_IMR,
        TXPKTCTL_B0_CLR,
        TXPKTCTL_B0_SET,
    )?;
    clr_set(
        io,
        R_AX_TXPKTCTL_B1_ERRFLAG_IMR,
        TXPKTCTL_B1_CLR,
        TXPKTCTL_B1_SET,
    )?;
    // rtw89_wde_imr_enable, rtw89_ple_imr_enable
    clr_set(io, R_AX_WDE_ERR_IMR, WDE_PLE_IMR, WDE_PLE_IMR)?;
    clr_set(io, R_AX_PLE_ERR_IMR, WDE_PLE_IMR, WDE_PLE_IMR)?;
    // rtw89_pktin_imr_enable
    set32(io, R_AX_PKTIN_ERR_IMR, B_AX_PKTIN_GETPKTID_ERR_INT_EN)?;
    // rtw89_dispatcher_imr_enable
    clr_set(
        io,
        R_AX_HOST_DISPATCHER_ERR_IMR,
        HOST_DISP_IMR_CLR,
        HOST_DISP_IMR_SET,
    )?;
    clr_set(
        io,
        R_AX_CPU_DISPATCHER_ERR_IMR,
        CPU_DISP_IMR_CLR,
        CPU_DISP_IMR_SET,
    )?;
    clr_set(
        io,
        R_AX_OTHER_DISPATCHER_ERR_IMR,
        OTHER_DISP_IMR_CLR,
        OTHER_DISP_IMR_SET,
    )?;
    // rtw89_cpuio_imr_enable
    clr_set(io, R_AX_CPUIO_ERR_IMR, CPUIO_IMR, CPUIO_IMR)?;
    // rtw89_bbrpt_imr_enable
    set32(
        io,
        R_AX_BBRPT_COM_ERR_IMR,
        B_AX_BBRPT_COM_NULL_PLPKTID_ERR_INT_EN,
    )?;
    clr_set(
        io,
        R_AX_BBRPT_CHINFO_ERR_IMR,
        BBRPT_CHINFO_IMR,
        BBRPT_CHINFO_IMR,
    )?;
    set32(io, R_AX_BBRPT_DFS_ERR_IMR, B_AX_BBRPT_DFS_TO_ERR_INT_EN)?;
    set32(io, R_AX_LA_ERRFLAG, B_AX_LA_IMR_DATA_LOSS_ERR)
}

/// `enable_imr_ax(rtwdev, RTW89_MAC_0, RTW89_CMAC_SEL)` (`mac.c:3936`),
/// RTL8852C.
fn enable_imr_cmac<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_cmac_en(io)?;
    // rtw89_scheduler_imr_enable: clear two bits, set one of them again.
    clr_set(
        io,
        R_AX_SCHEDULE_ERR_IMR,
        B_AX_SORT_NON_IDLE_ERR_INT_EN | B_AX_FSM_TIMEOUT_ERR_INT_EN,
        B_AX_FSM_TIMEOUT_ERR_INT_EN,
    )?;
    clr_set(io, R_AX_PTCL_IMR0, PTCL_IMR, PTCL_IMR)?;
    // rtw89_cdma_imr_enable, with its RTL8852C second register
    clr_set(io, R_AX_RX_ERR_FLAG_IMR, CDMA_RX_IMR, CDMA_RX_IMR)?;
    clr_set(io, R_AX_TX_ERR_FLAG_IMR, CDMA_TX_IMR, CDMA_TX_IMR)?;
    clr_set(io, R_AX_PHYINFO_ERR_IMR_V1, PHYINFO_IMR, PHYINFO_IMR)?;
    clr_set(io, R_AX_RX_ERR_IMR, RMAC_IMR, RMAC_IMR)?;
    clr_set(
        io,
        R_AX_TRXPTCL_ERROR_INDICA_MASK,
        TMAC_IMR_CLR,
        TMAC_IMR_SET,
    )
}

/// `err_imr_ctrl_ax(rtwdev, true)` (`mac.c:3974`). The C1 error IMR is
/// skipped: `dle_info.c1_rx_qta` is 0 in the SCC layout.
fn err_imr<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    io.write32(R_AX_DMAC_ERR_IMR, ERR_IMR_EN)?;
    io.write32(R_AX_CMAC_ERR_IMR, ERR_IMR_EN)
}

/// `set_host_rpr_ax` (`mac.c:4009`), PCIe branch.
fn host_rpr<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    field32(io, R_AX_WDRLS_CFG, B_AX_WDRLS_MODE_MASK, RTW89_RPR_MODE_POH)?;
    set32(io, R_AX_RLSRPT0_CFG0, B_AX_RLSRPT0_FLTR_MAP_MASK)?;
    field32(
        io,
        R_AX_RLSRPT0_CFG1,
        B_AX_RLSRPT0_AGGNUM_MASK,
        RLSRPT0_AGGNUM,
    )?;
    field32(io, R_AX_RLSRPT0_CFG1, B_AX_RLSRPT0_TO_MASK, RLSRPT0_TO)
}

/// Everything in `trx_init_ax` after `cmac_init_ax` (`mac.c:4055-4077`).
fn imr_tail<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    enable_imr_dmac(io)?;
    enable_imr_cmac(io)?;
    err_imr(io)?;
    host_rpr(io)?;
    clear32(io, R_AX_RSP_CHK_SIG, B_AX_RSP_STATIC_RTS_CHK_SERV_BW_EN)
}

/// `trx_init_ax` (`mac.c:4029`) for an RTL8852C on PCIe with the SCC layout.
/// The result is whether the CMAC's BA CAM reset timed out (`cmac::init`).
pub fn init<I: AxIo + ?Sized>(io: &mut I) -> Result<bool, TrxError> {
    dmac::init(io).map_err(at("dmac init"))?;
    let bacam_reset_timed_out = cmac::init(io).map_err(at("cmac init"))?;
    imr_tail(io).map_err(at("imr"))?;
    Ok(bacam_reset_timed_out)
}

#[cfg(test)]
mod tests;

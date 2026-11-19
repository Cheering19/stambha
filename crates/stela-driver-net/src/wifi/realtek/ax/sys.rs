// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE baseband/RF enable, disable and reset, and MAC function
//! enables, from `rtw8852c_mac_enable_bb_rf` (`rtw8852c.c:2999`, reached
//! through `rtw89_chip_enable_bb_rf`), `rtw8852c_mac_disable_bb_rf`
//! (`rtw8852c.c:3036`), `rtw89_chip_reset_bb_rf` (`mac.h:1377`) and
//! `sys_init_ax` (`mac.c:1701`) with `dmac_func_en_ax` (`:1656`),
//! `cmac_func_en_ax` (`:1610`, band 0 only) and `chip_func_en_ax`
//! (`:1690`, which does nothing for an RTL8852C). Register addresses and
//! bits are from pinned rtw89 `reg.h` and `mac.h`. This module grants no
//! band 1, no DLE, HFC or CMAC table setup, no BB or RF register tables and
//! no calibration; the bring-up runs its enable and system init after the
//! efuse step, and the reset after OFLD_CFG (`phy::init_radio`).
use super::io::{AxError, AxIo, clear8, clear32, field32, set8, set32};
use super::power::xtal_si_write;

// reg.h (pinned rtw89).
const R_AX_SYS_FUNC_EN: u32 = 0x0002;
const B_AX_FEN_BBRSTB: u8 = 1 << 0;
const B_AX_FEN_BB_GLB_RSTN: u8 = 1 << 1;
const R_AX_WLRF_CTRL: u32 = 0x02F0;
const B_AX_AFC_AFEDIG: u32 = 1 << 17;
const R_AX_AFE_OFF_CTRL1: u32 = 0x0444;
const B_AX_S1_LDO_VSEL_F_MASK: u32 = 0x0300_0000; // GENMASK(25, 24)
const B_AX_S0_LDO_VSEL_F_MASK: u32 = 0x0060_0000; // GENMASK(22, 21)

// mac.h `enum rtw89_xtal_si_offset` (`:1676-1711`); FULL_BIT_MASK is
// GENMASK(7, 0) (`:1722`).
const XTAL0: u8 = 0x00;
const XTAL3: u8 = 0x03;
const XTAL_SI_WL_RFC_S0: u8 = 0x80;
const XTAL_SI_WL_RFC_S1: u8 = 0x81;
const XTAL_SI_ANAPAR_WL: u8 = 0x90;
const FULL_BIT_MASK: u8 = 0xFF;

const R_AX_DMAC_FUNC_EN: u32 = 0x8400;
const R_AX_DMAC_CLK_EN: u32 = 0x8404;
const R_AX_CMAC_FUNC_EN: u32 = 0xC000;
const R_AX_CK_EN: u32 = 0xC004;

const fn bit(n: u32) -> u32 {
    1 << n
}

// `dmac_func_en_ax`, 8852C branch (`mac.c:1661-1666`).
const DMAC_FUNC_EN_8852C: u32 = bit(30) // B_AX_MAC_FUNC_EN
    | bit(29) // B_AX_DMAC_FUNC_EN
    | bit(16) // B_AX_MAC_SEC_EN
    | bit(18) // B_AX_DISPATCHER_EN
    | bit(19) // B_AX_DLE_CPUIO_EN
    | bit(20) // B_AX_PKT_IN_EN
    | bit(21) // B_AX_DMAC_TBL_EN
    | bit(22) // B_AX_PKT_BUF_EN
    | bit(24) // B_AX_STA_SCH_EN
    | bit(25) // B_AX_TXPKT_CTRL_EN
    | bit(27) // B_AX_WD_RLS_EN
    | bit(28) // B_AX_MPDU_PROC_EN
    | bit(31) // B_AX_DMAC_CRPRT
    | bit(14); // B_AX_H_AXIDMA_EN
// `dmac_func_en_ax` clock word (`mac.c:1676-1679`); AXIDMA_CLK_EN is 8852BT only.
const DMAC_CLK_EN_8852C: u32 = bit(16) // B_AX_MAC_SEC_CLK_EN
    | bit(17) // B_AX_BBRPT_CLK_EN
    | bit(18) // B_AX_DISPATCHER_CLK_EN
    | bit(19) // B_AX_DLE_CPUIO_CLK_EN
    | bit(20) // B_AX_PKT_IN_CLK_EN
    | bit(24) // B_AX_STA_SCH_CLK_EN
    | bit(25) // B_AX_TXPKT_CTRL_CLK_EN
    | bit(27); // B_AX_WD_RLS_CLK_EN
// `cmac_func_en_ax` (`mac.c:1618-1626`).
const CMAC_FUNC_EN: u32 = bit(30) // B_AX_CMAC_EN
    | bit(29) // B_AX_CMAC_TXEN
    | bit(28) // B_AX_CMAC_RXEN
    | bit(5) // B_AX_PHYINTF_EN
    | bit(4) // B_AX_CMAC_DMA_EN
    | bit(3) // B_AX_PTCLTOP_EN
    | bit(2) // B_AX_SCHEDULER_EN
    | bit(1) // B_AX_TMAC_EN
    | bit(0) // B_AX_RMAC_EN
    | bit(31); // B_AX_CMAC_CRPRT
const CMAC_CK_EN: u32 = bit(30) // B_AX_CMAC_CKEN
    | bit(5) // B_AX_PHYINTF_CKEN
    | bit(4) // B_AX_CMAC_DMA_CKEN
    | bit(3) // B_AX_PTCLTOP_CKEN
    | bit(2) // B_AX_SCHEDULER_CKEN
    | bit(1) // B_AX_TMAC_CKEN
    | bit(0); // B_AX_RMAC_CKEN

/// `rtw8852c_mac_enable_bb_rf` (`rtw8852c.c:2999-3034`): release the
/// baseband reset, cycle the AFE digital clock, set both LDO selects, and
/// write the five XTAL SI words. Any XTAL SI failure stops the sequence.
pub fn enable_bb_rf<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set8(io, R_AX_SYS_FUNC_EN, B_AX_FEN_BBRSTB | B_AX_FEN_BB_GLB_RSTN)?;

    set32(io, R_AX_WLRF_CTRL, B_AX_AFC_AFEDIG)?;
    clear32(io, R_AX_WLRF_CTRL, B_AX_AFC_AFEDIG)?;
    set32(io, R_AX_WLRF_CTRL, B_AX_AFC_AFEDIG)?;

    field32(io, R_AX_AFE_OFF_CTRL1, B_AX_S0_LDO_VSEL_F_MASK, 0x1)?;
    field32(io, R_AX_AFE_OFF_CTRL1, B_AX_S1_LDO_VSEL_F_MASK, 0x1)?;

    xtal_si_write(io, XTAL0, 0x7, FULL_BIT_MASK)?;
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, 0x6c, FULL_BIT_MASK)?;
    xtal_si_write(io, XTAL_SI_WL_RFC_S0, 0xc7, FULL_BIT_MASK)?;
    xtal_si_write(io, XTAL_SI_WL_RFC_S1, 0xc7, FULL_BIT_MASK)?;
    xtal_si_write(io, XTAL3, 0xd, FULL_BIT_MASK)
}

/// `rtw8852c_mac_disable_bb_rf` (`rtw8852c.c:3036-3043`): gate the AFE
/// digital clock, then hold the baseband in reset. Both are
/// read-modify-writes that keep the other bits.
pub fn disable_bb_rf<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    clear32(io, R_AX_WLRF_CTRL, B_AX_AFC_AFEDIG)?;
    clear8(io, R_AX_SYS_FUNC_EN, B_AX_FEN_BBRSTB | B_AX_FEN_BB_GLB_RSTN)
}

/// `rtw89_chip_reset_bb_rf` (`mac.h:1377-1391`) for an AX chip:
/// [`disable_bb_rf`], then [`enable_bb_rf`]. An XTAL SI failure in the
/// enable fails it, with the baseband left as far as the enable got.
pub fn reset_bb_rf<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    disable_bb_rf(io)?;
    enable_bb_rf(io)
}

/// `sys_init_ax` (`mac.c:1701`) for an RTL8852C: `dmac_func_en_ax` writes
/// the DMAC function and clock words outright, then `cmac_func_en_ax(0,
/// true)` sets band 0's clock and function bits. `chip_func_en_ax` does
/// nothing for this chip. The DMAC write clears `DLE_WDE_EN` and
/// `DLE_PLE_EN`, as Linux does; the DLE init sets them again.
pub fn sys_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    io.write32(R_AX_DMAC_FUNC_EN, DMAC_FUNC_EN_8852C)?;
    io.write32(R_AX_DMAC_CLK_EN, DMAC_CLK_EN_8852C)?;
    set32(io, R_AX_CK_EN, CMAC_CK_EN)?;
    set32(io, R_AX_CMAC_FUNC_EN, CMAC_FUNC_EN)
}

#[cfg(test)]
mod tests;

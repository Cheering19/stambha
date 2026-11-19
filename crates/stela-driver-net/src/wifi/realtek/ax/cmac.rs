// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE band 0 CMAC init, from `cmac_init_ax` (`mac.c:3093`) with its
//! helpers `scheduler_init_ax` (`:2591`), `addr_cam_init_ax` (`:2564`),
//! `rx_fltr_init_ax` (`:2692`), `cca_ctrl_init_ax` (`:2758`, with
//! `_patch_dis_resp_chk`'s default branch), `nav_ctrl_init_ax` (`:2794`),
//! `spatial_reuse_init_ax` (`:2808`), `tmac_init_ax` (`:2830`),
//! `trxptcl_init_ax` (`:2856`), `rmac_init_ax` (`:2919`, with `rst_bacam`
//! `:2904`), `cmac_com_init_ax` (`:2983`), `ptcl_init_ax` (`:3021`) and
//! `cmac_dma_init_ax` (`:3075`, which does nothing for an RTL8852C). The
//! RTL8852C's response-rate values are `rtw8852c_rrsr_cfgs`
//! (`rtw8852c.c:214`), and `rmac_init_ax`'s RX length limit uses the SCC
//! layout's `c0_rx_qta` and `ple_pg_size` from `get_dle_mem_cfg`
//! (`mac.c:1906`). Only the RTL8852C, PCIe, band 0 branches are ported;
//! register addresses and bits are from pinned rtw89 `reg.h`. This module grants no band 1, no BB or RF
//! setup, no MAC address or BSSID, no TX or RX path and no firmware command;
//! `trx::init` runs it inside the bring-up.
use super::io::{
    AxError, AxIo, Polled, clear8, clear32, field8, field32, poll_sample, poll32_atomic, set8,
    set32,
};

// reg.h (pinned rtw89).
const R_AX_CMAC_FUNC_EN: u32 = 0xC000;
const B_AX_CMAC_EN: u32 = 1 << 30;
const R_AX_SEC_ENG_CTRL: u32 = 0x9D00;
const B_AX_TX_PARTIAL_MODE: u32 = 1 << 11;

// scheduler_init_ax.
const R_AX_PREBKF_CFG_1: u32 = 0xC33C;
const B_AX_SIFS_MACTXEN_T1_MASK: u32 = 0x7F; // GENMASK(6, 0)
const SIFS_MACTXEN_T1_V1: u32 = 0x41;
const R_AX_CCA_CFG_0: u32 = 0xC340;
const B_AX_BTCCA_EN: u32 = 1 << 5;
const R_AX_PREBKF_CFG_0: u32 = 0xC338;
const B_AX_PREBKF_TIME_MASK: u32 = 0x1F; // GENMASK(4, 0)
const SCH_PREBKF_24US: u32 = 0x18;

// addr_cam_init_ax.
const R_AX_ADDR_CAM_CTRL: u32 = 0xCE34;
const B_AX_ADDR_CAM_RANGE_MASK: u32 = 0x00FF_0000; // GENMASK(23, 16)
const B_AX_ADDR_CAM_CLR: u32 = 1 << 8;
const B_AX_ADDR_CAM_EN: u32 = 1 << 0;
/// `TRXCFG_WAIT_CNT`, the `read_poll_timeout` budget in us.
const TRXCFG_WAIT_CNT: u64 = 2000;

// rx_fltr_init_ax: R_AX_{MGNT,CTRL,DATA}_FLTR, RX_FLTR_FRAME_TO_HOST
// (`reg.h:3355-3362`), `hal.rx_fltr` = DEFAULT_AX_RX_FLTR (`reg.h:3348`).
const R_AX_MGNT_FLTR: u32 = 0xCE28;
const R_AX_CTRL_FLTR: u32 = 0xCE24;
const R_AX_DATA_FLTR: u32 = 0xCE2C;
const RX_FLTR_FRAME_TO_HOST: u32 = 0x5555_5555;
const R_AX_RX_FLTR_OPT: u32 = 0xCE20;
/// `B_AX_A_A1_MATCH | A_BC | A_MC | A_UC_CAM_MATCH | A_BC_CAM_MATCH |
/// A_PWR_MGNT (10) | A_FTM_REQ (14) | UID_FILTER [31:24] = 3 | A_BCN_CHK_EN`.
const DEFAULT_AX_RX_FLTR: u32 = 0x0300_44BE;
const B_AX_RX_MPDU_MAX_LEN_MASK: u32 = 0x003F_0000; // GENMASK(21, 16)
const R_AX_PLCP_HDR_FLTR: u32 = 0xCE04;
/// `B_AX_CCK_CRC_CHK | CCK_SIG_CHK | LSIG_PARITY_CHK_EN | SIGA_CRC_CHK |
/// VHT_SU_SIGB_CRC_CHK | VHT_MU_SIGB_CRC_CHK | HE_SIGB_CRC_CHK` (bits 0-6).
const PLCP_FLTR: u16 = 0x7F;
const B_AX_VHT_SU_SIGB_CRC_CHK: u8 = 1 << 4;

// cca_ctrl_init_ax and _patch_dis_resp_chk (default branch).
const R_AX_CCA_CONTROL: u32 = 0xC390;
/// `B_AX_TB_CHK_BASIC_NAV` (30), `TB_CHK_BTCCA` (29), `TB_CHK_EDCCA` (28),
/// `TB_CHK_CCA_P20` (24), `SIFS_CHK_BTCCA` (21), `SIFS_CHK_CCA_P20` (16) and
/// `CTN_CHK_*` bits 0-7.
const CCA_CONTROL_SET: u32 = 0x7121_00FF;
/// `B_AX_TB_CHK_TX_NAV` (31), `TB_CHK_CCA_S80/S40/S20` (27-25),
/// `SIFS_CHK_CCA_S80/S40/S20` (19-17), `SIFS_CHK_EDCCA` (20), `CTN_CHK_TXNAV` (8).
const CCA_CONTROL_CLEAR: u32 = 0x8E1E_0100;
const R_AX_RSP_CHK_SIG: u32 = 0xCC00;
/// `B_AX_RSP_CHK_TXNAV | INTRA_NAV | BASIC_NAV` (bits 19-21).
const RSP_CHK_NAV: u32 = 0x0038_0000;
const R_AX_TRXPTCL_RESP_0: u32 = 0xCC04;
/// `B_AX_RSP_CHK_SEC_CCA_80 .. B_AX_RSP_CHK_CCA` (bits 23-28).
const RSP_CHK_CCA: u32 = 0x1F80_0000;

// nav_ctrl_init_ax.
const R_AX_WMAC_NAV_CTL: u32 = 0xCC80;
/// `B_AX_WMAC_PLCP_UP_NAV_EN` (17) | `TF_UP_NAV_EN` (16) | `NAV_UPPER_EN` (26).
const WMAC_NAV_EN: u32 = 0x0403_0000;
const B_AX_WMAC_NAV_UPPER_MASK: u32 = 0xFF00; // GENMASK(15, 8)
const NAV_25MS: u32 = 0xC4;

// spatial_reuse_init_ax.
const R_AX_RX_SR_CTRL: u32 = 0xCE4A;
const B_AX_SR_EN: u8 = 1 << 0;
const R_AX_BSSID_SRC_CTRL: u32 = 0xCE4B;
const B_AX_PLCP_SRC_EN: u8 = 1 << 0;

// tmac_init_ax.
const R_AX_MAC_LOOPBACK: u32 = 0xCC20;
const B_AX_MACLBK_EN: u32 = 1 << 0;
const R_AX_TCR0: u32 = 0xCA00;
const B_AX_TCR_UDF_THSD_MASK: u32 = 0x007F_0000; // GENMASK(22, 16)
const TCR_UDF_THSD: u32 = 0x6;
const R_AX_TXD_FIFO_CTRL: u32 = 0xCA1C;
const B_AX_TXDFIFO_HIGH_MCS_THRE_MASK: u32 = 0xF000; // GENMASK(15, 12)
const B_AX_TXDFIFO_LOW_MCS_THRE_MASK: u32 = 0x0F00; // GENMASK(11, 8)
const TXDFIFO_HIGH_MCS_THRE: u32 = 0x7;
const TXDFIFO_LOW_MCS_THRE: u32 = 0x7;

// trxptcl_init_ax. `WMAC_SPEC_SIFS_OFDM_52C` is the default branch's value.
const B_AX_WMAC_SPEC_SIFS_CCK_MASK: u32 = 0x00FF; // GENMASK(7, 0)
const B_AX_WMAC_SPEC_SIFS_OFDM_MASK: u32 = 0xFF00; // GENMASK(15, 8)
const WMAC_SPEC_SIFS_CCK: u32 = 0xA;
const WMAC_SPEC_SIFS_OFDM_52C: u32 = 0x11;
const R_AX_RXTRIG_TEST_USER_2: u32 = 0xCCB0;
const B_AX_RXTRIG_FCSCHK_EN: u32 = 1 << 20;
// `rtw8852c_rrsr_cfgs` (`rtw8852c.c:214`): `ref_rate` {R_AX_TRXPTCL_RRSR_CTL_0,
// B_AX_WMAC_RESP_REF_RATE_SEL (bit 9), 0}, `rsc` {R_AX_PTCL_RRSR1, B_AX_RSC_MASK
// GENMASK(7, 6), 2}.
const R_AX_TRXPTCL_RRSR_CTL_0: u32 = 0xCC08;
const B_AX_WMAC_RESP_REF_RATE_SEL: u32 = 1 << 9;
const R_AX_PTCL_RRSR1: u32 = 0xC090;
const B_AX_RSC_MASK: u32 = 0xC0;

// rmac_init_ax and rst_bacam.
const R_AX_RESPBA_CAM_CTRL: u32 = 0xCE3C;
const B_AX_BACAM_RST_MASK: u32 = 0x3; // GENMASK(1, 0)
const S_AX_BACAM_RST_ALL: u32 = 2;
const B_AX_SSN_SEL: u8 = 1 << 2;
const R_AX_DLK_PROTECT_CTL: u32 = 0xCE02;
const B_AX_RX_DLK_CCA_TIME_MASK: u16 = 0xFF00; // GENMASK(15, 8)
const B_AX_RX_DLK_DATA_TIME_MASK: u16 = 0x00F0; // GENMASK(7, 4)
const TRXCFG_RMAC_CCA_TO: u16 = 32;
const TRXCFG_RMAC_DATA_TO: u16 = 15;
const R_AX_RCR: u32 = 0xCE00;
const B_AX_CH_EN_MASK: u8 = 0xF; // GENMASK(3, 0)
/// `dle_info.c0_rx_qta` and `ple_pg_size` of the SCC layout (`get_dle_mem_cfg`,
/// `mac.c:1906`).
const C0_RX_QTA: u32 = 178;
const PLE_PG_SIZE: u32 = 128;
const PLD_RLS_MAX_PG: u32 = 127;
const RX_MAX_LEN_UNIT: u32 = 512;
const RX_SPEC_MAX_LEN: u32 = 11454 + RX_MAX_LEN_UNIT;

// cmac_com_init_ax.
const R_AX_TX_SUB_CARRIER_VALUE: u32 = 0xC088;
/// `B_AX_TXSC_20M_MASK | 40M | 80M`, GENMASK(11, 0).
const TXSC_MASK: u32 = 0x0FFF;

// ptcl_init_ax, PCIe.
const R_AX_SIFS_SETTING: u32 = 0xC624;
const B_AX_HW_CTS2SELF_PKT_LEN_TH_MASK: u32 = 0xFF00_0000; // GENMASK(31, 24)
const B_AX_HW_CTS2SELF_PKT_LEN_TH_TWW_MASK: u32 = 0x00FC_0000; // GENMASK(23, 18)
const B_AX_HW_CTS2SELF_EN: u32 = 1 << 16;
const S_AX_CTS2S_TH_1K: u32 = 4;
const S_AX_CTS2S_TH_SEC_256B: u32 = 1;
const R_AX_PTCL_FSM_MON: u32 = 0xC6E8;
const B_AX_PTCL_TX_ARB_TO_MODE: u32 = 1 << 6;
const B_AX_PTCL_TX_ARB_TO_THR_MASK: u32 = 0x3F; // GENMASK(5, 0)
const S_AX_PTCL_TO_2MS: u32 = 0x3F;
const R_AX_PTCL_COMMON_SETTING_0: u32 = 0xC600;
/// `B_AX_CMAC_TX_MODE_0 | B_AX_CMAC_TX_MODE_1`.
const CMAC_TX_MODE: u8 = 0x03;
/// `B_AX_PTCL_TRIGGER_SS_EN_0 | _1 | _UL` (bits 2, 3, 4).
const PTCL_TRIGGER_SS_EN: u8 = 0x1C;
const R_AX_PTCLRPT_FULL_HDL: u32 = 0xC660;
const B_AX_SPE_RPT_PATH_MASK: u8 = 0x30; // GENMASK(5, 4)
const FWD_TO_WLCPU: u8 = 1;

/// `rtw89_mac_check_mac_en_ax(RTW89_MAC_0, RTW89_CMAC_SEL)` (`mac.c:62`).
pub(super) fn check_cmac_en<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    let value = io.read32(R_AX_CMAC_FUNC_EN)?;
    // RTW89_R32_EA and RTW89_R32_DEAD (`core.h:51`).
    if value == 0xEAEA_EAEA || value == 0xDEAD_BEEF || value & B_AX_CMAC_EN != B_AX_CMAC_EN {
        return Err(AxError::Mac("CMAC function is not enabled"));
    }
    Ok(())
}

/// `scheduler_init_ax` (`mac.c:2591`), 8852C branches.
fn scheduler_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_cmac_en(io)?;
    field32(
        io,
        R_AX_PREBKF_CFG_1,
        B_AX_SIFS_MACTXEN_T1_MASK,
        SIFS_MACTXEN_T1_V1,
    )?;
    clear32(io, R_AX_CCA_CFG_0, B_AX_BTCCA_EN)?;
    // The 8852C sets the pre-backoff time only when TX_PARTIAL_MODE is clear.
    if io.read32(R_AX_SEC_ENG_CTRL)? & B_AX_TX_PARTIAL_MODE == 0 {
        field32(
            io,
            R_AX_PREBKF_CFG_0,
            B_AX_PREBKF_TIME_MASK,
            SCH_PREBKF_24US,
        )?;
    }
    Ok(())
}

/// `addr_cam_init_ax` (`mac.c:2564`): `read_poll_timeout(rtw89_read16, ...)`,
/// 1 us over `TRXCFG_WAIT_CNT`, for the clear bit to drop.
fn addr_cam_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_cmac_en(io)?;
    // `u32_encode_bits(0x7f, B_AX_ADDR_CAM_RANGE_MASK)` | CLR | EN.
    set32(
        io,
        R_AX_ADDR_CAM_CTRL,
        ((0x7F << 16) & B_AX_ADDR_CAM_RANGE_MASK) | B_AX_ADDR_CAM_CLR | B_AX_ADDR_CAM_EN,
    )?;
    let polled = poll_sample(
        io,
        1,
        TRXCFG_WAIT_CNT,
        |io| {
            let value = io.read16(R_AX_ADDR_CAM_CTRL)?;
            if value == u16::MAX {
                return Err(AxError::AllOnes {
                    register: R_AX_ADDR_CAM_CTRL,
                });
            }
            Ok(value)
        },
        |value| u32::from(value) & B_AX_ADDR_CAM_CLR == 0,
    )?;
    match polled {
        Polled::Met(_) => Ok(()),
        Polled::Expired(_) => Err(AxError::Timeout {
            register: R_AX_ADDR_CAM_CTRL,
            mask: B_AX_ADDR_CAM_CLR,
        }),
    }
}

/// `rx_fltr_init_ax` (`mac.c:2692`).
fn rx_fltr_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_cmac_en(io)?;
    for reg in [R_AX_MGNT_FLTR, R_AX_CTRL_FLTR, R_AX_DATA_FLTR] {
        io.write32(reg, RX_FLTR_FRAME_TO_HOST)?;
    }
    io.write32(R_AX_RX_FLTR_OPT, DEFAULT_AX_RX_FLTR)?;
    io.write16(R_AX_PLCP_HDR_FLTR, PLCP_FLTR)
}

/// `cca_ctrl_init_ax` (`mac.c:2758`) and `_patch_dis_resp_chk`'s default
/// branch (`mac.c:2747`).
fn cca_ctrl_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_cmac_en(io)?;
    let value = io.read32(R_AX_CCA_CONTROL)?;
    io.write32(
        R_AX_CCA_CONTROL,
        (value | CCA_CONTROL_SET) & !CCA_CONTROL_CLEAR,
    )?;
    set32(io, R_AX_RSP_CHK_SIG, RSP_CHK_NAV)?;
    set32(io, R_AX_TRXPTCL_RESP_0, RSP_CHK_CCA)
}

/// `nav_ctrl_init_ax` (`mac.c:2794`), which Linux does not guard.
fn nav_ctrl_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set32(io, R_AX_WMAC_NAV_CTL, WMAC_NAV_EN)?;
    field32(io, R_AX_WMAC_NAV_CTL, B_AX_WMAC_NAV_UPPER_MASK, NAV_25MS)
}

/// `spatial_reuse_init_ax` (`mac.c:2808`).
fn spatial_reuse_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_cmac_en(io)?;
    clear8(io, R_AX_RX_SR_CTRL, B_AX_SR_EN)?;
    set8(io, R_AX_BSSID_SRC_CTRL, B_AX_PLCP_SRC_EN)
}

/// `tmac_init_ax` (`mac.c:2830`).
fn tmac_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_cmac_en(io)?;
    clear32(io, R_AX_MAC_LOOPBACK, B_AX_MACLBK_EN)?;
    field32(io, R_AX_TCR0, B_AX_TCR_UDF_THSD_MASK, TCR_UDF_THSD)?;
    field32(
        io,
        R_AX_TXD_FIFO_CTRL,
        B_AX_TXDFIFO_HIGH_MCS_THRE_MASK,
        TXDFIFO_HIGH_MCS_THRE,
    )?;
    field32(
        io,
        R_AX_TXD_FIFO_CTRL,
        B_AX_TXDFIFO_LOW_MCS_THRE_MASK,
        TXDFIFO_LOW_MCS_THRE,
    )
}

/// `trxptcl_init_ax` (`mac.c:2856`), 8852C `rtw8852c_rrsr_cfgs`.
fn trxptcl_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_cmac_en(io)?;
    let mut value = io.read32(R_AX_TRXPTCL_RESP_0)?;
    value = (value & !B_AX_WMAC_SPEC_SIFS_CCK_MASK) | WMAC_SPEC_SIFS_CCK;
    value = (value & !B_AX_WMAC_SPEC_SIFS_OFDM_MASK) | (WMAC_SPEC_SIFS_OFDM_52C << 8);
    io.write32(R_AX_TRXPTCL_RESP_0, value)?;
    set32(io, R_AX_RXTRIG_TEST_USER_2, B_AX_RXTRIG_FCSCHK_EN)?;
    field32(io, R_AX_TRXPTCL_RRSR_CTL_0, B_AX_WMAC_RESP_REF_RATE_SEL, 0)?;
    field32(io, R_AX_PTCL_RRSR1, B_AX_RSC_MASK, 2)
}

/// `rst_bacam` (`mac.c:2904`): `read_poll_timeout_atomic`, 1 us over 1000 us.
/// Linux only warns ("failed to reset BA CAM") when the reset never
/// completes, so a timeout is not an error here; a dead read still is. The
/// result is whether it timed out, which the bring-up reports in its line.
fn rst_bacam<I: AxIo + ?Sized>(io: &mut I) -> Result<bool, AxError> {
    field32(
        io,
        R_AX_RESPBA_CAM_CTRL,
        B_AX_BACAM_RST_MASK,
        S_AX_BACAM_RST_ALL,
    )?;
    match poll32_atomic(io, R_AX_RESPBA_CAM_CTRL, B_AX_BACAM_RST_MASK, 0, 1, 1000) {
        Ok(_) => Ok(false),
        Err(AxError::Timeout { .. }) => Ok(true),
        Err(error) => Err(error),
    }
}

/// `rmac_init_ax` (`mac.c:2919`), band 0, 8852C. The result is whether
/// `rst_bacam` timed out.
fn rmac_init<I: AxIo + ?Sized>(io: &mut I) -> Result<bool, AxError> {
    check_cmac_en(io)?;
    let bacam_reset_timed_out = rst_bacam(io)?;
    set8(io, R_AX_RESPBA_CAM_CTRL, B_AX_SSN_SEL)?;
    // One read and one write: two `u16_replace_bits` on the same value.
    let mut value = io.read16(R_AX_DLK_PROTECT_CTL)?;
    value = (value & !B_AX_RX_DLK_DATA_TIME_MASK) | (TRXCFG_RMAC_DATA_TO << 4);
    value = (value & !B_AX_RX_DLK_CCA_TIME_MASK) | (TRXCFG_RMAC_CCA_TO << 8);
    io.write16(R_AX_DLK_PROTECT_CTL, value)?;
    field8(io, R_AX_RCR, B_AX_CH_EN_MASK, 0x1)?;
    let rx_qta = C0_RX_QTA.min(PLD_RLS_MAX_PG);
    let rx_max_len = (rx_qta * PLE_PG_SIZE).min(RX_SPEC_MAX_LEN) / RX_MAX_LEN_UNIT;
    field32(io, R_AX_RX_FLTR_OPT, B_AX_RX_MPDU_MAX_LEN_MASK, rx_max_len)?;
    clear8(io, R_AX_PLCP_HDR_FLTR, B_AX_VHT_SU_SIGB_CRC_CHK)?;
    Ok(bacam_reset_timed_out)
}

/// `cmac_com_init_ax` (`mac.c:2983`); the RRSR1 step is for 8852A and 885xB.
fn cmac_com_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_cmac_en(io)?;
    clear32(io, R_AX_TX_SUB_CARRIER_VALUE, TXSC_MASK)
}

/// `ptcl_init_ax` (`mac.c:3021`), PCIe, band 0, 8852C.
fn ptcl_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_cmac_en(io)?;
    let mut value = io.read32(R_AX_SIFS_SETTING)?;
    value = (value & !B_AX_HW_CTS2SELF_PKT_LEN_TH_MASK) | (S_AX_CTS2S_TH_1K << 24);
    value = (value & !B_AX_HW_CTS2SELF_PKT_LEN_TH_TWW_MASK) | (S_AX_CTS2S_TH_SEC_256B << 18);
    io.write32(R_AX_SIFS_SETTING, value | B_AX_HW_CTS2SELF_EN)?;

    let mut value = io.read32(R_AX_PTCL_FSM_MON)?;
    value = (value & !B_AX_PTCL_TX_ARB_TO_THR_MASK) | S_AX_PTCL_TO_2MS;
    io.write32(R_AX_PTCL_FSM_MON, value & !B_AX_PTCL_TX_ARB_TO_MODE)?;

    set8(io, R_AX_PTCL_COMMON_SETTING_0, CMAC_TX_MODE)?;
    clear8(io, R_AX_PTCL_COMMON_SETTING_0, PTCL_TRIGGER_SS_EN)?;
    field8(
        io,
        R_AX_PTCLRPT_FULL_HDL,
        B_AX_SPE_RPT_PATH_MASK,
        FWD_TO_WLCPU,
    )
}

/// `cmac_init_ax(rtwdev, RTW89_MAC_0)` (`mac.c:3093`) for an RTL8852C over
/// PCIe: each step checks that the CMAC is enabled first, except
/// `nav_ctrl_init_ax`, which has no check. `cmac_dma_init_ax` returns before
/// any I/O for a chip that is not an RTL885xB, so it has no step here. The
/// result is whether the BA CAM reset timed out (Linux only warns of it).
pub fn init<I: AxIo + ?Sized>(io: &mut I) -> Result<bool, AxError> {
    scheduler_init(io)?;
    addr_cam_init(io)?;
    rx_fltr_init(io)?;
    cca_ctrl_init(io)?;
    nav_ctrl_init(io)?;
    spatial_reuse_init(io)?;
    tmac_init(io)?;
    trxptcl_init(io)?;
    let bacam_reset_timed_out = rmac_init(io)?;
    cmac_com_init(io)?;
    ptcl_init(io)?;
    Ok(bacam_reset_timed_out)
}

#[cfg(test)]
mod tests;

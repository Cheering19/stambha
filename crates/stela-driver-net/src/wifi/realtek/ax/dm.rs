// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
//! `rtw89_phy_dm_init` (pinned rtw89/phy.c:8235-8261) for an RTL8852C on
//! PHY 0 without DBCC, each call a function, its first part in Linux's
//! order:
//! 1. [`stat_init`] is `rtw89_phy_stat_init` (phy.c:5895-5917), whose only
//!    I/O is `rtw89_phy_stat_thermal_update` (phy.c:5812-5831) reading
//!    `rtw8852c_get_thermal` (rtw8852c.c:2585-2607) on both paths;
//! 2. [`bb_sethw`] is `rtw8852c_bb_sethw` (rtw8852c.c:1815-1832) with
//!    `rtw8852c_bb_macid_ctrl_init` (:1805-1813), `rtw8852c_bb_gpio_init`
//!    (:1758-1803), `rtw8852c_bb_gpio_trsw` (:1705-1724) and
//!    `rtw8852c_bb_gpio_rfm` (:1732-1756);
//! 3. [`env_monitor_init`] is `rtw89_phy_env_monitor_init` (phy.c:8174-8191),
//!    `rtw89_phy_ccx_top_setting_init` (phy.c:6284-6304) then
//!    `rtw89_phy_ifs_clm_setting_init` (phy.c:6478-6503) with
//!    `rtw89_phy_ifs_clm_th_update_check` (phy.c:6363-6416) and
//!    `rtw89_phy_ifs_clm_set_th_reg` (phy.c:6418-6448), over
//!    `rtw89_ccx_regs_ax` (phy.c:8935);
//! 4. [`physts_parsing_init`] is `rtw89_physts_parsing_init`
//!    (phy.c:7157-7214) outside monitor mode;
//! 5. [`dig_init`] is `rtw89_phy_dig_init` (phy.c:7369-7384) through
//!    `rtw89_phy_dig_reset` (phy.c:7707-7717) with `support_igi = false`
//!    and `support_cckpd = true` (core.c:7170-7174), over
//!    `rtw8852c_dig_regs` (rtw8852c.c:228-250);
//! 6. [`cfo_init`] is `rtw89_phy_cfo_init` (phy.c:5073-5099), the crystal cap
//!    through `rtw89_phy_cfo_set_crystal_cap` (phy.c:4974-5004) and
//!    `rtw89_dcfo_comp_init` (phy.c:5052-5071) with `cfo_hw_comp = false`
//!    (rtw8852c.c:3274);
//! 7. [`edcca_init`] is `rtw89_phy_edcca_init` (phy.c:8201-8233).
//!
//! After `rtw89_phy_init_rf_nctl` ([`super::bb::init_rf_nctl`]) and
//! `rtw8852c_rfk_init` ([`super::rfk`]), its tail (phy.c:8258-8260):
//! 8. [`set_txpwr_ctrl`] is `rtw8852c_set_txpwr_ctrl` (rtw8852c.c:2261-2265),
//!    `rtw8852c_set_txpwr_ref(PHY0, 0)` (:2103-2135);
//! 9. [`power_trim`] is `rtw8852c_power_trim` (rtw8852c.c:816-820), the
//!    efuse thermal and PA bias trims;
//! 10. [`cfg_txrx_path`] is `rtw8852c_bb_cfg_txrx_path`
//!     (rtw8852c.c:2556-2583) for both RX paths, two streams and 2G.
//!
//! `rtw89_phy_diag_init`, `nhm_setting_init` (`support_noise = false`),
//! `bb_wrap_init` and `ch_info_init` (NULL on AX), `ul_tb_info_init`,
//! `antdiv_init`, `rfe_gpio` (NULL), `antdiv_set_ant` and `rfk_hw_init`
//! (NULL) only touch driver memory on this chip, so they are not ported
//! (spec S12.1). Linux's driver-side state these calls fill (the thermal
//! average, the IFS thresholds, the DIG and CFO tracking state) is not kept;
//! the values Stela needs later are returned instead.
//!
//! Where Linux ignores a failed XTAL SI command (phy.c:4990-4995), this
//! fails, as a timed-out XTAL SI is an I/O fault (spec S12.7); an RF
//! access that SWSI never serves fails as [`super::rf`] says. This module
//! grants no tracking loop (thermal, DIG, CFO, EDCCA or env monitor), no
//! PHY 1, no channel, no TX power by rate, limit or TSSI beyond the 0 dBm
//! reference, no single-path or 5G/6G RX configuration, no NCTL and no
//! calibration: it only leaves the registers as the first
//! `rtw89_phy_dm_init` after probe does.
use super::bb;
use super::efuse::PhyEfuse;
use super::elements::RfPath;
use super::io::{self, AxError, AxIo};
use super::power::{xtal_si_read, xtal_si_write};
use super::rf;

/// `RR_TM` (reg.h:8635), `RR_TM_TRI` b19 and `RR_TM_VAL` [6:1].
const RR_TM: u32 = 0x42;
const RR_TM_TRI: u32 = 1 << 19;
const RR_TM_VAL: u32 = 0x7E;
/// `fsleep(200)` between the thermal trigger and the read
/// (rtw8852c.c:2595).
const THERMAL_SETTLE_US: u64 = 200;
/// `__THM_MASK_SIGN` b0 and `__THM_MASK_VAL8` b4 (rtw8852c.c:729-731).
const THM_MASK_SIGN: u8 = 1 << 0;
const THM_MASK_VAL8: u8 = 1 << 4;

/// `rtw8852c_get_thermal` (rtw8852c.c:2585-2607) on `path`: pulse the
/// meter trigger, wait 200 us, read [6:1] of RF 0x42, then add 8 or take 8
/// off when the path's trim byte has VAL8. `trim` is `Some` exactly when
/// Linux's `pg_thermal_trim` holds (either byte != 0xFF), so a 0xFF byte on
/// one path still compensates that path by -8, as Linux does. The sum is
/// a u8, as Linux returns it, so it wraps.
fn thermal<I: AxIo + ?Sized>(io: &mut I, path: RfPath, trim: Option<u8>) -> Result<u8, AxError> {
    rf::write_rf(io, path, RR_TM, RR_TM_TRI, 1)?;
    rf::write_rf(io, path, RR_TM, RR_TM_TRI, 0)?;
    rf::write_rf(io, path, RR_TM, RR_TM_TRI, 1)?;
    io.delay_us(THERMAL_SETTLE_US);
    let val = rf::read_rf(io, path, RR_TM, RR_TM_VAL)? as u8;
    let comp: i8 = match trim {
        Some(t) if t & THM_MASK_VAL8 != 0 => {
            if t & THM_MASK_SIGN != 0 {
                -8
            } else {
                8
            }
        }
        _ => 0,
    };
    Ok(val.wrapping_add_signed(comp))
}

/// `rtw89_phy_stat_init` (phy.c:5895-5917): read the thermal meter of path
/// A then path B (`rtw89_phy_stat_thermal_update`, phy.c:5812-5831), with
/// `trim` the efuse thermal trims (`PhyEfuse::thermal_trim`, index 0 path
/// A). Returns `[A, B]`. Linux averages only non-zero readings, which
/// matters to tracking only; the readings are returned as they are.
pub fn stat_init<I: AxIo + ?Sized>(io: &mut I, trim: Option<[u8; 2]>) -> Result<[u8; 2], AxError> {
    let a = thermal(io, RfPath::A, trim.map(|t| t[0]))?;
    let b = thermal(io, RfPath::B, trim.map(|t| t[1]))?;
    Ok([a, b])
}

// reg.h (pinned rtw89), BB offsets of rtw8852c_bb_sethw.
/// `R_DBCC_80P80_SEL_EVM_RPT` and `_RPT2`, both enabled by b0
/// (reg.h:8960-8961, 9388-9389).
const R_DBCC_80P80_SEL_EVM_RPT: u32 = 0x0A10;
const R_DBCC_80P80_SEL_EVM_RPT2: u32 = 0x2A10;
const B_DBCC_80P80_SEL_EVM_RPT_EN: u32 = 1 << 0;
/// `R_AX_PWR_MACID_LMT_TABLE0` and `TABLE127` (reg.h:3562-3563), MAC
/// addresses inside the TX power window `rtw89_mac_get_txpwr_cr_ax`
/// accepts for PHY 0 (mac.c:6258-6287), so each is a plain write.
const R_AX_PWR_MACID_LMT_TABLE0: u32 = 0xD36C;
const R_AX_PWR_MACID_LMT_TABLE127: u32 = 0xD568;
/// `rtw8852c_bb_gpio_init`'s `cr_bases` (rtw8852c.c:1760).
const GPIO_CR_BASES: [u32; 2] = [0x5800, 0x7800];
/// `B_P0_TRSW_A` b1, `B_P0_TRSW_B` b0, `B_P0_TRSW_X` b2 and
/// `B_P0_TRSW_SO_A2` [7:5] (reg.h:9905-9910).
const B_P0_TRSW_A: u32 = 1 << 1;
const B_P0_TRSW_B: u32 = 1 << 0;
const B_P0_TRSW_X: u32 = 1 << 2;
const B_P0_TRSW_SO_A2: u32 = 0xE0;
/// `R_RFE_E_A2`, `R_RFE_O_SEL_A2`, `R_RFE_SEL0_A2`, `R_RFE_SEL32_A2`
/// (reg.h:8851-8855).
const R_RFE_E_A2: u32 = 0x0334;
const R_RFE_O_SEL_A2: u32 = 0x0338;
const R_RFE_SEL0_A2: u32 = 0x033C;
const R_RFE_SEL32_A2: u32 = 0x0340;
/// `rtw8852c_bb_gpio_trsw`'s `path_cr_bases` (rtw8852c.c:1710).
const TRSW_CR: [u32; 2] = [0x5868, 0x7868];
/// The `(tx_path_en, trsw_tx, trsw_rx, trsw, trsw_b)` of each
/// `rtw8852c_bb_gpio_trsw` call, per path (rtw8852c.c:1778-1794).
const TRSW_CALLS: [(u32, u32, u32, u32, u32); 8] = [
    (0, 0, 0, 0, 1),
    (0, 0, 1, 1, 0),
    (0, 1, 0, 1, 0),
    (0, 1, 1, 1, 0),
    (1, 0, 0, 0, 1),
    (1, 0, 1, 1, 0),
    (1, 1, 0, 1, 0),
    (1, 1, 1, 1, 0),
];
/// `rtw8852c_bb_gpio_rfm`'s `path_cr_bases` and per-source byte offsets
/// (rtw8852c.c:1737-1738).
const RFM_CR: [u32; 2] = [0x5894, 0x7894];
const RFM_MASK_OFST: [u32; 3] = [0, 8, 16];
/// `B_P0_RFM_OUT` [4:0] (reg.h:9931); `DIS_WL` b7, `TX_OPT` b6 and `BT_EN`
/// b5 are 0 in every call.
const B_P0_RFM_OUT: u32 = 0x1F;
/// The `rfm_output_val` of the PAPE, TRSW and LNAON calls
/// (rtw8852c.c:1796-1802), by source.
const RFM_OUTPUT: [u32; 3] = [0x0, 0x4, 0x8];
/// `R_RPL_BIAS_COMP` / `R_RPL_BIAS_COMP1`, [7:0] each (reg.h:9811-9812,
/// 10054-10055).
const R_RPL_BIAS_COMP: u32 = 0x4DF0;
const R_RPL_BIAS_COMP1: u32 = 0x6DF0;
const B_RPL_BIAS_COMP_MASK: u32 = 0xFF;

/// `rtw8852c_bb_gpio_trsw` (rtw8852c.c:1705-1724): the two-bit field at
/// `16 + 2 * (4 * en + 2 * tx + rx)` of `cr` becomes `trsw << 1 | trsw_b`.
fn gpio_trsw<I: AxIo + ?Sized>(
    io: &mut I,
    cr: u32,
    (en, tx, rx, trsw, trsw_b): (u32, u32, u32, u32, u32),
) -> Result<(), AxError> {
    let mask_ofst = 16 + (en * 4 + tx * 2 + rx) * 2;
    let val = ((trsw << 1) & B_P0_TRSW_A) | (trsw_b & B_P0_TRSW_B);
    bb::field32(io, cr, (B_P0_TRSW_A | B_P0_TRSW_B) << mask_ofst, val)
}

/// `rtw8852c_bb_gpio_rfm` (rtw8852c.c:1732-1756) with `dis_tx_gnt_wl`,
/// `active_tx_opt` and `act_bt_en` all 0: byte `src` of `cr` becomes
/// `rfm_output_val`.
fn gpio_rfm<I: AxIo + ?Sized>(io: &mut I, cr: u32, src: usize, out: u32) -> Result<(), AxError> {
    bb::field32(io, cr, 0xFF << RFM_MASK_OFST[src], out & B_P0_RFM_OUT)
}

/// `rtw8852c_bb_gpio_init` (rtw8852c.c:1758-1803).
fn gpio_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    for base in GPIO_CR_BASES {
        bb::set32(io, base | 0x68, B_P0_TRSW_A)?;
        bb::clear32(io, base | 0x68, B_P0_TRSW_X)?;
        bb::clear32(io, base | 0x68, B_P0_TRSW_SO_A2)?;
        bb::write32(io, base | 0x80, 0x7777_7777)?;
        bb::write32(io, base | 0x84, 0x7777_7777)?;
    }
    bb::write32(io, R_RFE_E_A2, 0xFFFF_FFFF)?;
    bb::write32(io, R_RFE_O_SEL_A2, 0)?;
    bb::write32(io, R_RFE_SEL0_A2, 0)?;
    bb::write32(io, R_RFE_SEL32_A2, 0)?;
    for cr in TRSW_CR {
        for call in TRSW_CALLS {
            gpio_trsw(io, cr, call)?;
        }
    }
    for cr in RFM_CR {
        for (src, out) in RFM_OUTPUT.into_iter().enumerate() {
            gpio_rfm(io, cr, src, out)?;
        }
    }
    Ok(())
}

/// `rtw8852c_bb_sethw` (rtw8852c.c:1815-1832): enable both EVM reports,
/// zero the 128 per-MACID TX power limits, route the RFE and TRSW pins,
/// then read the RPL bias of PHY 0 and PHY 1 that the BB table left
/// (`efuse_gain.offset_base`, an s8 each, core.h:7300). Returns them for
/// the channel setup.
pub fn bb_sethw<I: AxIo + ?Sized>(io: &mut I) -> Result<[i8; 2], AxError> {
    bb::set32(io, R_DBCC_80P80_SEL_EVM_RPT, B_DBCC_80P80_SEL_EVM_RPT_EN)?;
    bb::set32(io, R_DBCC_80P80_SEL_EVM_RPT2, B_DBCC_80P80_SEL_EVM_RPT_EN)?;
    for addr in (R_AX_PWR_MACID_LMT_TABLE0..=R_AX_PWR_MACID_LMT_TABLE127).step_by(4) {
        io.write32(addr, 0)?;
    }
    gpio_init(io)?;
    let phy0 = bb::read32_mask(io, R_RPL_BIAS_COMP, B_RPL_BIAS_COMP_MASK)? as u8 as i8;
    let phy1 = bb::read32_mask(io, R_RPL_BIAS_COMP1, B_RPL_BIAS_COMP_MASK)? as u8 as i8;
    Ok([phy0, phy1])
}

// rtw89_ccx_regs_ax (phy.c:8935-8961) over reg.h:8962-9016, BB offsets.
/// `R_CCX` with `B_CCX_EN_MSK` b0, `B_CCX_TRIG_OPT_MSK` b1,
/// `B_MEASUREMENT_TRIG_MSK` b2 and `B_CCX_EDCCA_OPT_MSK` [6:4].
const R_CCX: u32 = 0x0C00;
const B_CCX_EN_MSK: u32 = 1 << 0;
const B_CCX_TRIG_OPT_MSK: u32 = 1 << 1;
const B_MEASUREMENT_TRIG_MSK: u32 = 1 << 2;
const B_CCX_EDCCA_OPT_MSK: u32 = 0x70;
/// `RTW89_CCX_EDCCA_BW20_0` (core.h:7055).
const RTW89_CCX_EDCCA_BW20_0: u32 = 0;
/// `R_IFS_COUNTER` and `B_IFS_COLLECT_EN` b12.
const R_IFS_COUNTER: u32 = 0x0C28;
const B_IFS_COLLECT_EN: u32 = 1 << 12;
/// `R_IFS_T1`..`R_IFS_T4`, each with `TH_HIGH` [31:16], `EN` b15 and
/// `TH_LOW` [14:0].
const R_IFS_T: [u32; 4] = [0x0C2C, 0x0C30, 0x0C34, 0x0C38];
const B_IFS_TH_HIGH_MSK: u32 = 0xFFFF_0000;
const B_IFS_EN_MSK: u32 = 1 << 15;
const B_IFS_TH_LOW_MSK: u32 = 0x7FFF;
/// `RTW89_IFS_CLM_NUM` (core.h:7073).
const IFS_CLM_NUM: usize = 4;
/// `IFS_CLM_TH0_UPPER` and `IFS_CLM_TH_MUL` (phy.h:113-114), the
/// `RTW89_IFS_CLM_INIT` thresholds.
const IFS_CLM_TH0_UPPER: u32 = 64;
const IFS_CLM_TH_MUL: u32 = 4;
/// `ilog2(CCX_US_BASE_RATIO) + RTW89_CCX_32_US` (phy.h:227-232): the shift
/// `rtw89_phy_ccx_us_to_idx` (phy.c:6268-6274) applies at the unit
/// `rtw89_phy_ccx_top_setting_init` sets.
const CCX_US_SHIFT: u32 = 4u32.ilog2() + 3;

/// `rtw89_phy_ifs_clm_th_update_check` (phy.c:6363-6416) for
/// `RTW89_IFS_CLM_INIT` after the setting init made the app
/// `BACKGROUND`, so the thresholds always update: `(low, high)` per
/// region, in CCX units, `low[i] = high[i-1] + 1` and the microsecond
/// bound multiplied by 4 each region.
fn ifs_clm_thresholds() -> ([u16; IFS_CLM_NUM], [u16; IFS_CLM_NUM]) {
    let mut low = [0u16; IFS_CLM_NUM];
    let mut high = [0u16; IFS_CLM_NUM];
    let mut high_us = IFS_CLM_TH0_UPPER;
    high[0] = (high_us >> CCX_US_SHIFT) as u16;
    for i in 1..IFS_CLM_NUM {
        low[i] = high[i - 1] + 1;
        high_us *= IFS_CLM_TH_MUL;
        high[i] = (high_us >> CCX_US_SHIFT) as u16;
    }
    (low, high)
}

/// `rtw89_phy_env_monitor_init` (phy.c:8174-8191) for PHY 0: the CCX top
/// setting (enable, trigger option, measurement trigger, EDCCA option
/// BW20_0), then the IFS CLM thresholds (all lows, then all highs), the IFS
/// collector and each region enabled. Every write is a
/// `rtw89_phy_write32_idx`, so a read-modify-write.
pub fn env_monitor_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    bb::field32(io, R_CCX, B_CCX_EN_MSK, 1)?;
    bb::field32(io, R_CCX, B_CCX_TRIG_OPT_MSK, 1)?;
    bb::field32(io, R_CCX, B_MEASUREMENT_TRIG_MSK, 1)?;
    bb::field32(io, R_CCX, B_CCX_EDCCA_OPT_MSK, RTW89_CCX_EDCCA_BW20_0)?;

    let (low, high) = ifs_clm_thresholds();
    for (reg, th) in R_IFS_T.into_iter().zip(low) {
        bb::field32(io, reg, B_IFS_TH_LOW_MSK, u32::from(th))?;
    }
    for (reg, th) in R_IFS_T.into_iter().zip(high) {
        bb::field32(io, reg, B_IFS_TH_HIGH_MSK, u32::from(th))?;
    }
    bb::field32(io, R_IFS_COUNTER, B_IFS_COLLECT_EN, 1)?;
    for reg in R_IFS_T {
        bb::field32(io, reg, B_IFS_EN_MSK, 1)?;
    }
    Ok(())
}

/// `R_PLCP_HISTOGRAM` with `B_STS_DIS_TRIG_BY_FAIL` b3 and
/// `B_STS_DIS_TRIG_BY_BRK` b2 (reg.h:8906-8910), via
/// `rtw89_physts_regs_ax` (phy.c:9024-9027).
const R_PLCP_HISTOGRAM: u32 = 0x0738;
const B_STS_DIS_TRIG_BY_FAIL: u32 = 1 << 3;
const B_STS_DIS_TRIG_BY_BRK: u32 = 1 << 2;
/// `R_PHY_STS_BITMAP_ADDR_START` (reg.h:8911-8914), `physt_bmp_start` of
/// `rtw89_phy_gen_ax` (phy.c:9043); pages are 4 bytes apart.
const R_PHY_STS_BITMAP_ADDR_START: u32 = 0x073C;
/// `enum rtw89_phy_status_bitmap` (phy.h:274-294).
const RTW89_HE_MU: u8 = 6;
const RTW89_VHT_MU: u8 = 7;
const RTW89_RSVD_9: u8 = 9;
const RTW89_TRIG_BASE_PPDU: u8 = 10;
const RTW89_CCK_PKT: u8 = 11;
const RTW89_HT_PKT: u8 = 13;
const RTW89_EHT_PKT: u8 = 16;
const RTW89_PHYSTS_BITMAP_NUM: u8 = 17;
/// The IE bits of `enum rtw89_phy_status_ie_type` (phy.h:237-256) that
/// the parsing init sets or clears.
const IE01_CMN_OFDM: u32 = 1 << 1;
/// IE04_CMN_EXT_PATH_A..IE07_CMN_EXT_PATH_D: GENMASK(7, 4).
const IE04_TO_IE07_CMN_EXT_PATH: u32 = 0xF0;
const IE13_DL_MU_DEF: u32 = 1 << 13;
const IE20_DBG_OFDM_FD_USER_SEG_0: u32 = 1 << 20;

/// The register page of bitmap `i`, as `rtw89_physts_ie_page_valid`
/// (phy.c:7066-7081) and `rtw89_phy_get_ie_bitmap_addr` (phy.c:7083-7093)
/// map it on AX: 9 is reserved, 10..=15 sit one page lower, and EHT does
/// not exist.
fn physts_page(i: u8) -> Option<u32> {
    match i {
        RTW89_RSVD_9 | RTW89_EHT_PKT.. => None,
        0..RTW89_RSVD_9 => Some(u32::from(i)),
        _ => Some(u32::from(i - 1)),
    }
}

/// `rtw89_physts_parsing_init` (phy.c:7157-7214) for PHY 0, not in monitor
/// mode (so the monitor-mode IEs add nothing): disable the fail and break
/// triggers (`rtw89_physts_enable_fail_report(false)`), then rewrite each
/// bitmap page. Each page is read (`rtw89_physts_get_ie_bitmap`, MASKDWORD)
/// and written back with `rtw89_phy_write32_idx(MASKDWORD)`, which reads it
/// again first.
pub fn physts_parsing_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    bb::set32(io, R_PLCP_HISTOGRAM, B_STS_DIS_TRIG_BY_FAIL)?;
    bb::set32(io, R_PLCP_HISTOGRAM, B_STS_DIS_TRIG_BY_BRK)?;
    // `rtw89_physts_enable_hdr_2` returns at once on AX (phy.c:7146-7155).
    for i in 0..RTW89_PHYSTS_BITMAP_NUM {
        let Some(page) = physts_page(i) else {
            continue;
        };
        let addr = R_PHY_STS_BITMAP_ADDR_START + (page << 2);
        let mut val = bb::read32_mask(io, addr, u32::MAX)?;
        match i {
            RTW89_HE_MU | RTW89_VHT_MU => val |= IE13_DL_MU_DEF,
            RTW89_TRIG_BASE_PPDU => val |= IE13_DL_MU_DEF | IE01_CMN_OFDM,
            RTW89_CCK_PKT.. => {
                val &= !IE04_TO_IE07_CMN_EXT_PATH;
                if i == RTW89_CCK_PKT {
                    val |= IE01_CMN_OFDM;
                } else if i >= RTW89_HT_PKT {
                    val |= IE20_DBG_OFDM_FD_USER_SEG_0;
                }
            }
            _ => {}
        }
        // `B_PHY_STS_BITMAP_MSK_52A` applies to the RTL8852A only.
        bb::field32(io, addr, u32::MAX, val)?;
    }
    Ok(())
}

// rtw8852c_dig_regs (rtw8852c.c:228-250) over reg.h, BB offsets.
/// `R_SEG0R_PD` with `B_SEG0R_PD_LOWER_BOUND_MSK` [10:6] and
/// `B_SEG0R_PD_SPATIAL_REUSE_EN_MSK` b29 (reg.h:9685-9696).
const R_SEG0R_PD: u32 = 0x481C;
const B_SEG0R_PD_LOWER_BOUND_MSK: u32 = 0x7C0;
const B_SEG0R_PD_SPATIAL_REUSE_EN_MSK: u32 = 1 << 29;
/// `R_BMODE_PDTH_EN_V1` with `B_BMODE_PDTH_LIMIT_EN_MSK_V1` b30, and
/// `R_BMODE_PDTH_V1` with `B_BMODE_PDTH_LOWER_BOUND_MSK_V1` [31:24]
/// (reg.h:9825-9832).
const R_BMODE_PDTH_EN_V1: u32 = 0x4B74;
const B_BMODE_PDTH_LIMIT_EN_MSK_V1: u32 = 1 << 30;
const R_BMODE_PDTH_V1: u32 = 0x4B64;
const B_BMODE_PDTH_LOWER_BOUND_MSK_V1: u32 = 0xFF00_0000;
/// The P20/S20 "follow PAGC UGC" enables of both paths, each b5
/// (reg.h:9605-9668).
const PAGCUGC_EN: [u32; 4] = [0x4C24, 0x4C28, 0x4CE8, 0x4CEC];
const B_PAGCUGC_EN_MSK: u32 = 1 << 5;
/// `rssi_nolink` (phy.c:7291), the RSSI `rtw89_phy_dig_reset` passes.
const RSSI_NOLINK: u8 = 22;
/// `dig->igi_rssi` at the first start: the driver state is zeroed and
/// nothing has set it yet (only DIG tracking does, phy.c:7297-7310).
const IGI_RSSI_FIRST_START: u8 = 0;
/// `pd_low_th_offset` (phy.c:7345), `PD_TH_SB_FLTR_CMP_VAL` and
/// `PD_TH_BW20_CMP_VAL` (phy.h:139-141): the AX under-region at the zeroed
/// channel's 20 MHz (spec S1.7).
const PD_LOW_TH_OFST: u8 = 16;
const PD_TH_SB_FLTR_CMP_VAL: u8 = 7;
const PD_TH_BW20_CMP_VAL: u8 = 0;
/// `CCKPD_TH_MIN_RSSI` and `IGI_RSSI_MAX` (phy.h:132-135).
const CCKPD_TH_MIN_RSSI: i8 = -18;
const IGI_RSSI_MAX: i32 = 110;

/// The CCK PD lower bound `rtw89_phy_dig_dyn_pd_th` computes for `rssi`
/// and `igi_rssi` (phy.c:7685-7701): `max(min(rssi, igi_rssi) -
/// under_region, CCKPD_TH_MIN_RSSI) - IGI_RSSI_MAX`, the difference cut to
/// an s8 and the result taken as a u32. At the first start (`rssi_nolink`,
/// `igi_rssi` 0, an under-region of 23) that is -128, so the field gets
/// 0x80.
fn cck_pd_lower_bound(rssi: u8, igi_rssi: u8) -> u32 {
    let final_rssi = rssi.min(igi_rssi);
    let under_region = PD_LOW_TH_OFST + PD_TH_SB_FLTR_CMP_VAL + PD_TH_BW20_CMP_VAL;
    let cck_cca_th =
        ((i32::from(final_rssi) - i32::from(under_region)) as i8).max(CCKPD_TH_MIN_RSSI);
    (i32::from(cck_cca_th) - IGI_RSSI_MAX) as u32
}

/// `rtw89_phy_dig_init` (phy.c:7369-7384) for PHY 0. `support_igi` is
/// false, so `rtw89_phy_dig_update_gain_para` and `dig_set_igi_cr` do
/// nothing; `rtw89_phy_dig_reset` (phy.c:7707-7717) then sets the PD
/// thresholds with dynamic PD off (`dig_dyn_pd_th(rssi_nolink, false)`:
/// lower bound 0, spatial reuse off, the CCK limit off and the CCK lower
/// bound of [`cck_pd_lower_bound`]; `disabled_dm_bitmap` is only set from
/// debugfs) and turns `sdagc_follow_pagc` off on both paths.
pub fn dig_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    let enable = 0;
    bb::field32(io, R_SEG0R_PD, B_SEG0R_PD_LOWER_BOUND_MSK, 0)?;
    bb::field32(io, R_SEG0R_PD, B_SEG0R_PD_SPATIAL_REUSE_EN_MSK, enable)?;
    // `support_cckpd` is true for the RTL8852C.
    bb::field32(io, R_BMODE_PDTH_EN_V1, B_BMODE_PDTH_LIMIT_EN_MSK_V1, enable)?;
    bb::field32(
        io,
        R_BMODE_PDTH_V1,
        B_BMODE_PDTH_LOWER_BOUND_MSK_V1,
        cck_pd_lower_bound(RSSI_NOLINK, IGI_RSSI_FIRST_START),
    )?;
    for reg in PAGCUGC_EN {
        bb::field32(io, reg, B_PAGCUGC_EN_MSK, 0)?;
    }
    Ok(())
}

/// DIG's pause for PHY 0, `bb->dig.pause_dig` with `rtw89_phy_dig_ctrl`
/// (phy.c:7850-7901). It starts unpaused: the driver state is zeroed at
/// probe and nothing in `rtw89_phy_dig_init` sets it. A suspend or resume
/// that finds the pause already as asked does nothing (phy.c:7859), so a
/// scan begun after a join that never associated, with DIG still suspended
/// by the join, leaves the register alone.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Dig {
    paused: bool,
}

impl Dig {
    /// Whether DIG is suspended.
    pub fn paused(&self) -> bool {
        self.paused
    }

    /// `rtw89_phy_dig_suspend`: unless already paused, the PD lower bound 0
    /// (`disabled_dm_bitmap` is only set from debugfs) and spatial reuse off,
    /// each a masked write, then paused.
    pub fn suspend<I: AxIo + ?Sized>(&mut self, io: &mut I) -> Result<(), AxError> {
        if self.paused {
            return Ok(());
        }
        bb::field32(io, R_SEG0R_PD, B_SEG0R_PD_LOWER_BOUND_MSK, 0)?;
        bb::field32(io, R_SEG0R_PD, B_SEG0R_PD_SPATIAL_REUSE_EN_MSK, 0)?;
        self.paused = true;
        Ok(())
    }

    /// `rtw89_phy_dig_resume(restore)`: when paused, the PD lower bound
    /// `restore ? bak_dig : 0`, which is 0 either way as only DIG tracking
    /// moves `bak_dig` off the 0 `rtw89_phy_dig_reset` left (phy.c:7651-7676,
    /// :7707-7714) and no tracking runs, then spatial reuse on when
    /// `total_sta_assoc > 0`, which `linked` says; then unpaused.
    pub fn resume<I: AxIo + ?Sized>(&mut self, io: &mut I, linked: bool) -> Result<(), AxError> {
        if !self.paused {
            return Ok(());
        }
        bb::field32(io, R_SEG0R_PD, B_SEG0R_PD_LOWER_BOUND_MSK, 0)?;
        bb::field32(
            io,
            R_SEG0R_PD,
            B_SEG0R_PD_SPATIAL_REUSE_EN_MSK,
            u32::from(linked),
        )?;
        self.paused = false;
        Ok(())
    }
}

/// `B_AX_XTAL_SC_MASK` (reg.h:289): the crystal cap's 7 bits, applied to
/// the raw efuse byte with no default for an unprogrammed 0xFF
/// (phy.c:5078).
const B_AX_XTAL_SC_MASK: u8 = 0x7F;
/// `XTAL_SI_XTAL_SC_XO` 0x05 and `XTAL_SI_XTAL_SC_XI` 0x04, both under
/// GENMASK(7, 0) (mac.h:1678-1681).
const XTAL_SI_XTAL_SC_XO: u8 = 0x05;
const XTAL_SI_XTAL_SC_XI: u8 = 0x04;
const XTAL_SC_MASK: u8 = 0xFF;
// rtw89_cfo_regs_ax (phy.c:9034-9039) and R_DCFO over reg.h, BB offsets.
/// `R_DCFO_OPT` with `B_DCFO_OPT_EN` b29, `R_DCFO_WEIGHT` with
/// `B_DCFO_WEIGHT_MSK` [27:24], and `R_DCFO` with `B_DCFO` [7:0]
/// (reg.h:9482-9514).
const R_DCFO_OPT: u32 = 0x4494;
const B_DCFO_OPT_EN: u32 = 1 << 29;
const R_DCFO_WEIGHT: u32 = 0x4490;
const B_DCFO_WEIGHT_MSK: u32 = 0x0F00_0000;
const R_DCFO: u32 = 0x4264;
const B_DCFO: u32 = 0xFF;
/// `R_AX_PWR_UL_CTRL2` (MAC) and `B_AX_PWR_UL_CFO_MASK` [2:0]
/// (reg.h:3509-3510).
const R_AX_PWR_UL_CTRL2: u32 = 0xD248;
const B_AX_PWR_UL_CFO_MASK: u32 = 0x7;

/// `rtw89_dcfo_comp_init` (phy.c:5052-5071) on AX without `cfo_hw_comp`.
fn dcfo_comp_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    bb::field32(io, R_DCFO_OPT, B_DCFO_OPT_EN, 1)?;
    bb::field32(io, R_DCFO_WEIGHT, B_DCFO_WEIGHT_MSK, 8)?;
    bb::field32(io, R_DCFO, B_DCFO, 1)?;
    io::clear32(io, R_AX_PWR_UL_CTRL2, B_AX_PWR_UL_CFO_MASK)
}

/// `rtw89_phy_cfo_init` (phy.c:5073-5099) with `xtal_cap` the raw efuse
/// crystal cap (`PhyEfuse::xtal_cap`): write the cap, masked to 7 bits, to
/// XTAL SI XO then XI, read XO then XI back (as
/// `rtw89_phy_cfo_set_crystal_cap(.., true)` does on a non-8852A/8851B
/// chip, phy.c:4988-4995), then the DCFO compensation init. Returns
/// `(XO, XI)` as read back. A timed-out XTAL SI command fails here, where
/// Linux carries on with what it did not get.
pub fn cfo_init<I: AxIo + ?Sized>(io: &mut I, xtal_cap: u8) -> Result<(u8, u8), AxError> {
    let cap = xtal_cap & B_AX_XTAL_SC_MASK;
    xtal_si_write(io, XTAL_SI_XTAL_SC_XO, cap, XTAL_SC_MASK)?;
    xtal_si_write(io, XTAL_SI_XTAL_SC_XI, cap, XTAL_SC_MASK)?;
    let xo = xtal_si_read(io, XTAL_SI_XTAL_SC_XO)?;
    let xi = xtal_si_read(io, XTAL_SI_XTAL_SC_XI)?;
    dcfo_comp_init(io)?;
    Ok((xo, xi))
}

/// `R_TX_COLLISION_T2R_ST` and `B_TX_COLLISION_T2R_ST_M` [25:20]
/// (reg.h:9030-9031), `rtw8852c_edcca_regs.tx_collision_t2r_st`
/// (rtw8852c.c:269-270).
const R_TX_COLLISION_T2R_ST: u32 = 0x0C70;
const B_TX_COLLISION_T2R_ST_M: u32 = 0x03F0_0000;

/// `rtw89_phy_edcca_init` (phy.c:8201-8233) for PHY 0: the TX-to-RX
/// collision start becomes 0x29. The RTL8922A CAV writes before it do not
/// apply.
pub fn edcca_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    bb::field32(io, R_TX_COLLISION_T2R_ST, B_TX_COLLISION_T2R_ST_M, 0x29)
}

/// `R_AX_PWR_RATE_CTRL` (reg.h:3493), a MAC word inside the TX power window
/// `rtw89_mac_get_txpwr_cr_ax` accepts for PHY 0 (mac.c:6258-6287), and
/// the [27:10] field `rtw8852c_set_txpwr_ref` clears.
const R_AX_PWR_RATE_CTRL: u32 = 0xD200;
const PWR_RATE_CTRL_REF_MASK: u32 = 0x0FFF_FC00;
/// `rtw8852c_set_txpwr_ref`'s per-path BB bases, OFDM and CCK offsets and
/// mask (rtw8852c.c:2106-2110).
const TXPWR_REF_BASE: [u32; 2] = [0x5800, 0x7800];
const TXPWR_REF_OFST_OFDM: u32 = 0x4;
const TXPWR_REF_OFST_CCK: u32 = 0x8;
const TXPWR_REF_MASK: u32 = 0x07FF_FFFF;
/// `rtw8852c_bb_cal_txpwr_ref(ref = 0, pwr_ofst_decrease = 0)`
/// (rtw8852c.c:2041-2066), the word every path gets for OFDM and CCK when
/// `rtw8852c_set_txpwr_ctrl` passes `pwr_ofst = 0` (so both paths'
/// `ofst_dec` are 0, :2121-2122) and both references are 0:
/// - `pwr_s10_3 = (0 << 1) + (0x27 << 3) - 0 = 0x138`;
/// - `bb_pwr_cw = pwr_s10_3[2:0] = 0`, `rf_pwr_cw = pwr_s10_3[8:3] = 0x27`
///   (inside the 15..=63 clamp), so `pwr_cw = 0x27 << 3 | 0 = 0x138`;
/// - `tssi_ofst_cw = 0x12C + 0 - (16 << 3) - 0 = 0xAC`;
/// - the word is `0xAC << 18 | 0x138 << 9 | (0 & 0x1FF)`.
const TXPWR_REF_0DBM: u32 = (0xAC << 18) | (0x138 << 9);

/// `rtw8852c_set_txpwr_ctrl(PHY0)` (rtw8852c.c:2261-2265), which is
/// `rtw8852c_set_txpwr_ref(PHY0, 0)` (:2103-2135): clear [27:10] of the MAC
/// rate control word (`rtw89_mac_txpwr_write32_mask`, a read-modify-write,
/// mac.h:1602-1614), then write the 0 dBm reference to [26:0] of the OFDM
/// word of path A then B, then of the CCK word of path A then B. Each BB
/// write is `rtw89_phy_write32_idx` on PHY 0, so a read-modify-write
/// (phy.c:2111-2117).
pub fn set_txpwr_ctrl<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    io::field32(io, R_AX_PWR_RATE_CTRL, PWR_RATE_CTRL_REF_MASK, 0)?;
    for ofst in [TXPWR_REF_OFST_OFDM, TXPWR_REF_OFST_CCK] {
        for base in TXPWR_REF_BASE {
            bb::field32(io, base + ofst, TXPWR_REF_MASK, TXPWR_REF_0DBM)?;
        }
    }
    Ok(())
}

/// `RR_TM2` with `RR_TM2_OFF` [19:16] (reg.h:8640-8641).
const RR_TM2: u32 = 0x43;
const RR_TM2_OFF: u32 = 0xF_0000;
/// `RR_BIASA` with `RR_BIASA_TXG` [15:12] and `RR_BIASA_TXA` [19:16]
/// (reg.h:8671-8673).
const RR_BIASA: u32 = 0x60;
const RR_BIASA_TXG: u32 = 0xF000;
const RR_BIASA_TXA: u32 = 0xF_0000;
/// `__THM_MASK_3BITS` [3:1] (rtw8852c.c:730).
const THM_MASK_3BITS: u8 = 0xE;

/// `__thm_setting(raw)` (rtw8852c.c:735-739): the sign bit moves to b3 and
/// [3:1] down to [2:0]; VAL8 and the high bits are dropped. 0xFF gives 0xF.
fn thm_setting(raw: u8) -> u8 {
    ((raw & THM_MASK_SIGN) << 3) | ((raw & THM_MASK_3BITS) >> 1)
}

/// `rtw8852c_power_trim` (rtw8852c.c:816-820) with the efuse trims of
/// `efuse`:
/// - `rtw8852c_thermal_trim` (:733-759): when `thermal_trim` is `Some`
///   (Linux's `pg_thermal_trim`: either byte != 0xFF), RF 0x43 [19:16] of
///   path A then B becomes `__thm_setting` of the path's byte;
/// - `rtw8852c_pa_bias_trim` (:781-805): when `pa_bias_trim` is `Some`, for
///   path A then B, RF 0x60 [15:12] becomes the byte's low nibble (2G),
///   then [19:16] its high nibble (5G).
///
/// Each trim's flag covers both paths, so a path whose byte is 0xFF inside
/// a programmed pair is written too (0xF for thermal, 0xF and 0xF for PA
/// bias), as Linux does. `None` writes nothing.
pub fn power_trim<I: AxIo + ?Sized>(io: &mut I, efuse: &PhyEfuse) -> Result<(), AxError> {
    if let Some(trim) = efuse.thermal_trim {
        for (path, raw) in [RfPath::A, RfPath::B].into_iter().zip(trim) {
            rf::write_rf(io, path, RR_TM2, RR_TM2_OFF, u32::from(thm_setting(raw)))?;
        }
    }
    if let Some(trim) = efuse.pa_bias_trim {
        for (path, raw) in [RfPath::A, RfPath::B].into_iter().zip(trim) {
            rf::write_rf(io, path, RR_BIASA, RR_BIASA_TXG, u32::from(raw & 0xF))?;
            rf::write_rf(io, path, RR_BIASA, RR_BIASA_TXA, u32::from(raw >> 4))?;
        }
    }
    Ok(())
}

// rtw8852c_bb_cfg_rx_path and rtw8852c_ctrl_btg_bt_rx over reg.h, BB
// offsets.
/// `R_CHBW_MOD` with `B_BT_SHARE` b14 and `B_ANT_RX_SEG0` [3:0]
/// (reg.h:9711-9716).
const R_CHBW_MOD: u32 = 0x4978;
const B_BT_SHARE: u32 = 1 << 14;
const B_ANT_RX_SEG0: u32 = 0xF;
/// `R_FC0_BW` with `B_ANT_RX_BT_SEG0` [25:22], `B_ANT_RX_1RCCA_SEG1`
/// [21:18] and `B_ANT_RX_1RCCA_SEG0` [17:14] (reg.h:9701-9706).
const R_FC0_BW: u32 = 0x4974;
const B_ANT_RX_BT_SEG0: u32 = 0x03C0_0000;
const B_ANT_RX_1RCCA_SEG1: u32 = 0x003C_0000;
const B_ANT_RX_1RCCA_SEG0: u32 = 0x0003_C000;
/// `R_RXHT_MCS_LIMIT` [9:8] and `R_RXVHT_MCS_LIMIT` [22:21], one word
/// (reg.h:9051-9054).
const R_RXHT_MCS_LIMIT: u32 = 0x0D18;
const B_RXHT_MCS_LIMIT: u32 = 0x300;
const R_RXVHT_MCS_LIMIT: u32 = 0x0D18;
const B_RXVHT_MCS_LIMIT: u32 = 0x0060_0000;
/// `R_RXHE` with `B_RXHETB_MAX_NSS` [25:23], `B_RXHE_MAX_NSS` [16:14] and
/// `B_RXHE_USER_MAX` [13:6] (reg.h:9059-9062).
const R_RXHE: u32 = 0x0D80;
const B_RXHETB_MAX_NSS: u32 = 0x0380_0000;
const B_RXHE_MAX_NSS: u32 = 0x0001_C000;
const B_RXHE_USER_MAX: u32 = 0x3FC0;
/// `R_P0_TXPW_RSTB` and `R_P1_TXPW_RSTB`, each `rst_mask` the `_MANON` b30
/// and `_TSSI` b31 bits (reg.h:9936-9938, 10097-10099; rtw8852c.c:
/// 2322-2323).
const R_P0_TXPW_RSTB: u32 = 0x58DC;
const R_P1_TXPW_RSTB: u32 = 0x78DC;
const B_TXPW_RSTB_MANON_TSSI: u32 = 0xC000_0000;
/// `R_PATH0_BT_SHARE_V1` b19 and `R_PATH0_BTG_PATH_V1` b22, one word
/// (reg.h:9630-9633).
const R_PATH0_BT_SHARE_V1: u32 = 0x4738;
const B_PATH0_BT_SHARE_V1: u32 = 1 << 19;
const R_PATH0_BTG_PATH_V1: u32 = 0x4738;
const B_PATH0_BTG_PATH_V1: u32 = 1 << 22;
/// `R_PATH1_G_LNA6_OP1DB_V1` [31:24] and `R_PATH1_G_TIA0_LNA6_OP1DB_V1`
/// [7:0] (reg.h:9655-9670).
const R_PATH1_G_LNA6_OP1DB_V1: u32 = 0x476C;
const B_PATH1_G_LNA6_OP1DB_V1: u32 = 0xFF00_0000;
const R_PATH1_G_TIA0_LNA6_OP1DB_V1: u32 = 0x4778;
const B_PATH1_G_TIA0_LNA6_OP1DB_V1: u32 = 0xFF;
/// `R_PATH1_BT_SHARE_V1` b19 and `R_PATH1_BTG_PATH_V1` b22, one word
/// (reg.h:9676-9679).
const R_PATH1_BT_SHARE_V1: u32 = 0x4AA4;
const B_PATH1_BT_SHARE_V1: u32 = 1 << 19;
const R_PATH1_BTG_PATH_V1: u32 = 0x4AA4;
const B_PATH1_BTG_PATH_V1: u32 = 1 << 22;
/// `R_PMAC_GNT` with `B_PMAC_GNT_P1` [20:17] and `B_PMAC_GNT_P2` [29:26]
/// (reg.h:8935-8939).
const R_PMAC_GNT: u32 = 0x0980;
const B_PMAC_GNT_P1: u32 = 0x001E_0000;
const B_PMAC_GNT_P2: u32 = 0x3C00_0000;
/// `R_BT_DYN_DC_EST_EN` b31 (reg.h:9501, 9503) and `R_GNT_BT_WGT_EN` b21
/// (reg.h:9024-9025).
const R_BT_DYN_DC_EST_EN: u32 = 0x441C;
const B_BT_DYN_DC_EST_EN_MSK: u32 = 1 << 31;
const R_GNT_BT_WGT_EN: u32 = 0x0C6C;
const B_GNT_BT_WGT_EN: u32 = 1 << 21;
/// `chip->rx_nss` (rtw8852c.c:3217), which `hal.rx_nss` keeps when the
/// PHY_CAP reply is not read (spec S1.6).
const RX_NSS: u32 = 2;

/// `rtw8852c_ctrl_btg_bt_rx(en, PHY0)` (rtw8852c.c:2637-2680). With `en`
/// (2.4 GHz) BT shares path 0's and path 1's 2G front ends; without it
/// (5 GHz, `rtw8852c_set_channel_bb`) the sharing is undone, path 1's G-band
/// LNA6/TIA0 OP1dB go back to 0x1A/0x2A and the PMAC grant of P1 and P2 to
/// 0xF and 4. Every write is a read-modify-write.
pub(super) fn ctrl_btg_bt_rx<I: AxIo + ?Sized>(io: &mut I, en: bool) -> Result<(), AxError> {
    let on = u32::from(en);
    bb::field32(io, R_PATH0_BT_SHARE_V1, B_PATH0_BT_SHARE_V1, on)?;
    bb::field32(io, R_PATH0_BTG_PATH_V1, B_PATH0_BTG_PATH_V1, 0)?;
    let (lna6, tia0) = if en { (0x20, 0x30) } else { (0x1A, 0x2A) };
    bb::field32(io, R_PATH1_G_LNA6_OP1DB_V1, B_PATH1_G_LNA6_OP1DB_V1, lna6)?;
    bb::field32(
        io,
        R_PATH1_G_TIA0_LNA6_OP1DB_V1,
        B_PATH1_G_TIA0_LNA6_OP1DB_V1,
        tia0,
    )?;
    bb::field32(io, R_PATH1_BT_SHARE_V1, B_PATH1_BT_SHARE_V1, on)?;
    bb::field32(io, R_PATH1_BTG_PATH_V1, B_PATH1_BTG_PATH_V1, on)?;
    if en {
        bb::field32(io, R_PMAC_GNT, B_PMAC_GNT_P1, 0)?;
    } else {
        bb::field32(io, R_PMAC_GNT, B_PMAC_GNT_P1, 0xF)?;
        bb::field32(io, R_PMAC_GNT, B_PMAC_GNT_P2, 0x4)?;
    }
    bb::field32(io, R_CHBW_MOD, B_BT_SHARE, on)?;
    bb::field32(io, R_FC0_BW, B_ANT_RX_BT_SEG0, if en { 2 } else { 0 })?;
    bb::field32(io, R_BT_DYN_DC_EST_EN, B_BT_DYN_DC_EST_EN_MSK, on)?;
    bb::field32(io, R_GNT_BT_WGT_EN, B_GNT_BT_WGT_EN, on)
}

/// The HT, VHT, HE and HE TB stream limits for `rx_nss` streams, as
/// `rtw8852c_bb_cfg_txrx_path` and `bb_cfg_rx_path` write them: each field
/// holds `rx_nss - 1`.
fn rx_nss_limits<I: AxIo + ?Sized>(io: &mut I, rx_nss: u32) -> Result<(), AxError> {
    let v = rx_nss - 1;
    bb::field32(io, R_RXHT_MCS_LIMIT, B_RXHT_MCS_LIMIT, v)?;
    bb::field32(io, R_RXVHT_MCS_LIMIT, B_RXVHT_MCS_LIMIT, v)?;
    bb::field32(io, R_RXHE, B_RXHE_MAX_NSS, v)?;
    bb::field32(io, R_RXHE, B_RXHETB_MAX_NSS, v)
}

/// `rtw8852c_bb_cfg_txrx_path` (rtw8852c.c:2556-2583) with Linux's
/// no-reply defaults, `antenna_rx = 0` and `rx_nss = 2` (spec S1.6), so
/// `rtw8852c_bb_cfg_rx_path(RF_PATH_AB)` (:2318-2427) without DBCC at the
/// zeroed channel's 2G band (spec S1.7):
/// 1. both antennas on segment 0 and the 1RCCA segments, the stream limits
///    for two streams, `ctrl_btg_bt_rx(true)`, then the TX power reset of
///    path A and of path B (MANON|TSSI to 1, then 3);
/// 2. `B_RXHE_USER_MAX` = 8;
/// 3. the stream limits for `rx_nss` 2 again.
///
/// Every write is `rtw89_phy_write32_mask`, a read-modify-write. The
/// single-path (A or B, `rx_nss` 1), DBCC and 5G/6G branches are not
/// ported.
pub fn cfg_txrx_path<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    bb::field32(io, R_CHBW_MOD, B_ANT_RX_SEG0, 3)?;
    bb::field32(io, R_FC0_BW, B_ANT_RX_1RCCA_SEG0, 3)?;
    bb::field32(io, R_FC0_BW, B_ANT_RX_1RCCA_SEG1, 3)?;
    // The RF_PATH_AB branch writes two-stream limits whatever `rx_nss` is.
    rx_nss_limits(io, 2)?;
    ctrl_btg_bt_rx(io, true)?;
    for reg in [R_P0_TXPW_RSTB, R_P1_TXPW_RSTB] {
        bb::field32(io, reg, B_TXPW_RSTB_MANON_TSSI, 1)?;
        bb::field32(io, reg, B_TXPW_RSTB_MANON_TSSI, 3)?;
    }
    bb::field32(io, R_RXHE, B_RXHE_USER_MAX, 8)?;
    rx_nss_limits(io, RX_NSS)
}

#[cfg(test)]
mod tests;

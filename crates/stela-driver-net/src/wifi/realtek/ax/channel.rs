// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! An RTL8852C's channel switch, `__rtw89_set_channel` (`core.c:568-598`)
//! on MAC 0 and PHY 0 without DBCC, for a 20 MHz 2.4 or 5 GHz [`Chan`].
//! [`set_channel`] runs its parts in `rtw8852c_set_channel`'s order
//! (`rtw8852c.c:1932-1940`), then `chip->ops->set_txpwr` (`core.c:586`),
//! all between the two halves of the help:
//! - [`help_enter`] is `rtw8852c_set_channel_help(enter)`
//!   (`rtw8852c.c:1960-1974`): `rtw89_mac_stop_sch_tx_v1` (`mac.c:3444-3495`
//!   with `rtw89_set_hw_sch_tx_en_v1`, `:3385-3401`), PPDU status off
//!   (`cmac::cfg_ppdu_status`), `rtw8852c_dfs_en` (`:1942-1948`),
//!   `rtw8852c_tssi_cont_en_phyidx` (`rtw8852c_rfk.c:3789-3820`),
//!   `rtw8852c_adc_en` (`:1950-1958`) and `rtw8852c_bb_reset_en`
//!   (`:1673-1695`);
//! - [`set_channel_mac`] is `rtw8852c_set_channel_mac` (`rtw8852c.c:822-890`);
//! - [`set_channel_bb`] is `rtw8852c_set_channel_bb` (`rtw8852c.c:1834-1930`):
//!   `rtw8852c_ctrl_sco_cck` (`:902-930`), `rtw8852c_ctrl_ch`
//!   (`:1119-1240`) with [`set_gain_error`] (`rtw8852c_set_gain_error`,
//!   `:1012-1074`) and [`set_gain_offset`] (`rtw8852c_set_gain_offset`,
//!   `:1076-1117`), `rtw8852c_ctrl_bw` (`:1280-1386`) with
//!   `rtw8852c_bw_setting` and `rtw8852c_edcca_per20_bitmap_sifs`
//!   (`:1241-1278`), `rtw8852c_spur_elimination` (`:1549-1593`) with
//!   `rtw8852c_spur_freq`, `rtw8852c_set_csi_tone_idx` and
//!   `rtw8852c_set_nbi_tone_idx` (`:1388-1525`) and util.h's
//!   `s32_div_u32_round_down`/`_round_closest` (`util.h:48-70`),
//!   `rtw8852c_ctrl_btg_bt_rx` (`super::dm`), `rtw8852c_5m_mask`
//!   (`:1595-1646`), `rtw8852c_ctrl_tx_path_tmac` (`:2430-2490`) and
//!   `rtw8852c_bb_reset_all` (`:1648-1671`, `bb::bb_reset`);
//! - [`set_channel_rf`] is `rtw8852c_set_channel_rf` (`rtw8852c_rfk.c:
//!   4078-4094`): `_ctrl_ch` (`:3935-3959`) with `_ch_setting`
//!   (`:3899-3933`), `_ctrl_bw` (`:3867-3897`) with `_bw_setting`
//!   (`:3822-3865`), and `_rxbb_bw` (`:3998-4012`) with `_set_rxbb_bw` and
//!   `_set_tia_bw` (`:3961-3996`);
//! - then `chip->ops->set_txpwr`, `rtw8852c_set_txpwr`
//!   ([`super::txpwr::set_txpwr`]), with the TX scheduler still stopped and
//!   the BB still held in reset;
//! - [`help_exit`] is `rtw8852c_set_channel_help(exit)` (`:1975-1982`), with
//!   `rtw89_mac_resume_sch_tx_v1` (`mac.c:3497-3507`).
//!
//! The BB part takes the firmware's BB gain table, the efuse RX gain
//! offsets, the RPL bias and the chip cut as [`ChannelInputs`]; the cut
//! goes to [`set_channel_rf`]. The TX power takes the firmware's TX power
//! tables at the regulatory indices as [`TxPower`]. RF 0x18 (`RR_CFGCH`) is
//! reached twice per path: as 0x18, which `rtw89_phy_write_rf_v1` sends
//! over SWSI (the "DAV" register), and as 0x10018 (`RR_CFGCH_V1`, address
//! bit 16 set), which goes to the path's direct BB window at
//! `rf_base_addr[path] + 0x18 * 4` (MMIO 0x1E060 for A, 0x1F060 for B; the
//! "DDV" register). Each is read and rewritten from its own value
//! ([`super::rf`]).
//!
//! Where Linux ignores a failed CMAC check (`rtw8852c.c:1967-1969`,
//! `:1976`, `:1981`), this fails, as a disabled CMAC is a broken MAC (spec
//! S6.1). The TSSI DE write of `tssi_cont_en(true)` (`_tssi_set_efuse_to_de`)
//! only serves TSSI, which is not ported (M3 spec S1.8). `_ctrl_bw`'s path-B
//! workaround for the CAV cut (`:3888-3895`, `hal.cv == CHIP_CAV`) is
//! ported and runs only when [`set_channel_rf`] is given cut 0; this
//! laptop's chip is CBV (cut 1). A [`Chan`] that [`Chan::new20`] would not
//! build is refused before any I/O. This module grants no DBCC or PHY 1, no
//! 6 GHz and no 40, 80 or 160 MHz channel, and transmits nothing.
use super::bb::{self, BbGain};
use super::chan::{Chan, HwBand, Subband};
use super::cmac::{cfg_ppdu_status, check_cmac_en};
use super::dm::ctrl_btg_bt_rx;
use super::efuse::RxGain;
use super::elements::RfPath;
use super::io::{AxError, AxIo};
use super::regd::Regds;
use super::rf::{RFREG_MASK, read_rf, write_rf};
use super::txpwr;
use super::txpwr_tables::TxPowerTables;

// reg.h (pinned rtw89): MAC registers of band 0.
const R_AX_CTN_DRV_TXEN: u32 = 0xC398;
const B_AX_CTN_TXEN_ALL_MASK_V1: u32 = 0x3_FFFF; // GENMASK(17, 0)
const R_AX_WMAC_RFMOD: u32 = 0xC010;
const B_AX_WMAC_RFMOD_MASK: u8 = 0x3; // GENMASK(1, 0)
const AX_WMAC_RFMOD_20M: u8 = 0;
const R_AX_TX_SUB_CARRIER_VALUE: u32 = 0xC088;
const R_AX_TXRATE_CHK: u32 = 0xC628;
const B_AX_BAND_MODE: u8 = 1 << 4;
const B_AX_RTS_LIMIT_IN_OFDM6: u8 = 1 << 1;
const B_AX_CHECK_CCK_EN: u8 = 1 << 0;

// reg.h: BB registers (offsets from `bb::CR_BASE`).
const R_UPD_P0: u32 = 0x0000;
const B_UPD_P0_EN: u32 = 1 << 31;
const R_ADC_FIFO: u32 = 0x20FC;
const B_ADC_FIFO_RST: u32 = 0xFF00_0000; // GENMASK(31, 24)
/// CCK RX CCA disable: `R_RXCCA_V1` / `B_RXCCA_DIS_V1` as
/// `rtw8852c_bb_reset_en` names it, the same bit as `R_11B_RX_V1` /
/// `B_11B_RXCCA_DIS_V1` (reg.h:9340-9347).
const R_RXCCA_V1: u32 = 0x2320;
const B_RXCCA_DIS_V1: u32 = 1 << 0;
const R_PD_CTRL: u32 = 0x0C3C;
const B_PD_HIT_DIS: u32 = 1 << 9;
const R_S0_HW_SI_DIS: u32 = 0x1200;
const R_S1_HW_SI_DIS: u32 = 0x3200;
/// `B_S0_HW_SI_DIS_W_R_TRIG` and `B_S1_...`: GENMASK(30, 28).
const B_HW_SI_DIS_W_R_TRIG: u32 = 0x7000_0000;
const R_RSTB_ASYNC: u32 = 0x0704;
const B_RSTB_ASYNC_ALL: u32 = 1 << 1;
/// `rtw8852c_tssi_cont_en`'s `tssi_trk` (BIT(30)) and `tssi_en` (BIT(31))
/// registers, path A then B.
const TSSI_TRK: [u32; 2] = [0x5818, 0x7818];
const TSSI_EN: [u32; 2] = [0x5820, 0x7820];
const B_TSSI_TRK: u32 = 1 << 30;
const B_TSSI_EN: u32 = 1 << 31;
/// `R_P0_CFCH_BW0` / `B_P0_CFCH_BW0` [27:26] and `R_P0_CFCH_BW1` /
/// `B_P0_CFCH_BW1` [8:5]; path B is `| 1 << 8`.
const R_P0_CFCH_BW0: u32 = 0xC0D4;
const B_P0_CFCH_BW0: u32 = 0x0C00_0000;
const R_P0_CFCH_BW1: u32 = 0xC0D8;
const B_P0_CFCH_BW1: u32 = 0x01E0;

// reg.h: RF registers.
/// `RR_CFGCH` (DAV, over SWSI) and `RR_CFGCH_V1` (DDV, the direct window).
const RR_CFGCH: u32 = 0x18;
const RR_CFGCH_V1: u32 = 0x10018;
const RR_CFGCH_BAND1: u32 = 0x3_0000; // GENMASK(17, 16)
const RR_CFGCH_BW: u32 = 0x0C00; // GENMASK(11, 10)
const RR_CFGCH_BAND0: u32 = 0x0300; // GENMASK(9, 8)
const RR_CFGCH_CH: u32 = 0x00FF; // GENMASK(7, 0)
/// `CFGCH_BAND1_5G` and `CFGCH_BAND0_5G` (1) in their fields; 2G is 0.
const CFGCH_BAND_5G: u32 = (1 << 16) | (1 << 8);
/// `CFGCH_BW_20M` (3) in `RR_CFGCH_BW`.
const CFGCH_BW_20M: u32 = 3 << 10;
const RR_LUTWE2: u32 = 0xEE;
const RR_LUTWE2_RTXBW: u32 = 1 << 2;
const RR_LUTWA: u32 = 0x33;
const RR_LUTWA_M2: u32 = 0x1F; // GENMASK(4, 0)
const RR_LUTWD0: u32 = 0x3F;
const RR_LUTWD0_LB: u32 = 0x3F; // GENMASK(5, 0)
const RR_RXBB2: u32 = 0x8F;
const RR_RXBB2_EBW: u32 = 0x60; // GENMASK(6, 5)
const RR_RSV1: u32 = 0x05;
const RR_RSV1_RST: u32 = 1 << 0;
const RR_APK: u32 = 0x19;
const RR_APK_MOD: u32 = 0x30; // GENMASK(5, 4)

/// `CHIP_CAV` of `enum rtw89_cv` (`core.h:201-202`).
const CHIP_CAV: u8 = 0;

/// `_kpath` without DBCC is `RF_AB` (`rtw8852c_rfk.c:80-92`).
const KPATH: [RfPath; 2] = [RfPath::A, RfPath::B];

/// `rtw89_set_hw_sch_tx_en_v1(MAC_0, tx_en, B_AX_CTN_TXEN_ALL_MASK_V1)`
/// (`mac.c:3385-3401`): the CMAC check, then the TXEN field of 0xC398
/// replaced by `tx_en`'s.
fn set_sch_tx_en<I: AxIo + ?Sized>(io: &mut I, tx_en: u32) -> Result<(), AxError> {
    check_cmac_en(io)?;
    let value = io.read32(R_AX_CTN_DRV_TXEN)?;
    io.write32(
        R_AX_CTN_DRV_TXEN,
        (value & !B_AX_CTN_TXEN_ALL_MASK_V1) | (tx_en & B_AX_CTN_TXEN_ALL_MASK_V1),
    )
}

/// `rtw8852c_tssi_cont_en_phyidx(en, PHY0)` without DBCC: path A then B,
/// `tssi_trk` BIT(30) and `tssi_en` BIT(31) set to `!en`.
fn tssi_cont_en<I: AxIo + ?Sized>(io: &mut I, en: bool) -> Result<(), AxError> {
    let v = u32::from(!en);
    for path in 0..2 {
        bb::field32(io, TSSI_TRK[path], B_TSSI_TRK, v)?;
        bb::field32(io, TSSI_EN[path], B_TSSI_EN, v)?;
        // `_tssi_set_efuse_to_de` would follow here when `en`: TSSI only,
        // not ported (M3 spec S1.8).
    }
    Ok(())
}

/// `rtw8852c_set_channel_help(true, ..)` (`rtw8852c.c:1960-1974`): stop the
/// TX scheduler, turn the PPDU status report, DFS, TSSI tracking and the
/// ADC FIFO off, then hold the BB in reset. The result is `tx_en`, the
/// whole of 0xC398 as `rtw89_mac_stop_sch_tx_v1` reads it before the CMAC
/// check, for [`help_exit`]. Linux passes the channel, but nothing in the
/// enter half reads it: `bb_reset_en(false)` disables CCK RX CCA (BB 0x2320
/// b0, `R_RXCCA_V1`, also named `R_11B_RX_V1` / `B_11B_RXCCA_DIS_V1`) on
/// every band.
pub fn help_enter<I: AxIo + ?Sized>(io: &mut I, _chan: &Chan) -> Result<u32, AxError> {
    // rtw89_mac_stop_sch_tx_v1(MAC_0, SEL_ALL).
    let tx_en = io.read32(R_AX_CTN_DRV_TXEN)?;
    set_sch_tx_en(io, 0)?;
    cfg_ppdu_status(io, false)?;
    // rtw8852c_dfs_en(false).
    bb::field32(io, R_UPD_P0, B_UPD_P0_EN, 0)?;
    tssi_cont_en(io, false)?;
    // rtw8852c_adc_en(false).
    bb::field32(io, R_ADC_FIFO, B_ADC_FIFO_RST, 0xF)?;
    io.delay_us(40);
    // rtw8852c_bb_reset_en(band, PHY0, false) (`:1686-1695`).
    bb::field32(io, R_RXCCA_V1, B_RXCCA_DIS_V1, 1)?;
    bb::field32(io, R_PD_CTRL, B_PD_HIT_DIS, 1)?;
    bb::field32(io, R_S0_HW_SI_DIS, B_HW_SI_DIS_W_R_TRIG, 0x7)?;
    bb::field32(io, R_S1_HW_SI_DIS, B_HW_SI_DIS_W_R_TRIG, 0x7)?;
    io.delay_us(1);
    bb::field32(io, R_RSTB_ASYNC, B_RSTB_ASYNC_ALL, 0)?;
    Ok(tx_en)
}

/// `rtw8852c_set_channel_help(false, ..)` (`rtw8852c.c:1975-1982`): the
/// PPDU status report, ADC FIFO, DFS and TSSI tracking back on, the BB out
/// of reset (CCK RX CCA re-enabled on 2.4 GHz only, as Linux does), then the TX
/// scheduler resumed with `tx_en`'s TXEN field.
pub fn help_exit<I: AxIo + ?Sized>(io: &mut I, chan: &Chan, tx_en: u32) -> Result<(), AxError> {
    cfg_ppdu_status(io, true)?;
    // rtw8852c_adc_en(true).
    bb::field32(io, R_ADC_FIFO, B_ADC_FIFO_RST, 0)?;
    // rtw8852c_dfs_en(true).
    bb::field32(io, R_UPD_P0, B_UPD_P0_EN, 1)?;
    tssi_cont_en(io, true)?;
    // rtw8852c_bb_reset_en(band, PHY0, true) (`:1676-1685`).
    bb::field32(io, R_S0_HW_SI_DIS, B_HW_SI_DIS_W_R_TRIG, 0)?;
    bb::field32(io, R_S1_HW_SI_DIS, B_HW_SI_DIS_W_R_TRIG, 0)?;
    bb::field32(io, R_RSTB_ASYNC, B_RSTB_ASYNC_ALL, 1)?;
    if chan.band == HwBand::G2 {
        bb::field32(io, R_RXCCA_V1, B_RXCCA_DIS_V1, 0)?;
    }
    bb::field32(io, R_PD_CTRL, B_PD_HIT_DIS, 0)?;
    // rtw89_mac_resume_sch_tx_v1(MAC_0, tx_en).
    set_sch_tx_en(io, tx_en)
}

/// `rtw8852c_set_channel_mac(chan, MAC_0)` (`rtw8852c.c:822-890`) at 20 MHz:
/// RFMOD 20M and no TX sub-carrier, then the TX rate checks of the band
/// (2.4 GHz `BAND_MODE`; 5 and 6 GHz `CHECK_CCK_EN | RTS_LIMIT_IN_OFDM6`).
pub fn set_channel_mac<I: AxIo + ?Sized>(io: &mut I, chan: &Chan) -> Result<(), AxError> {
    let chk_rate_mask = match chan.band {
        HwBand::G2 => B_AX_BAND_MODE,
        HwBand::G5 | HwBand::G6 => B_AX_CHECK_CCK_EN | B_AX_RTS_LIMIT_IN_OFDM6,
    };
    super::io::field8(io, R_AX_WMAC_RFMOD, B_AX_WMAC_RFMOD_MASK, AX_WMAC_RFMOD_20M)?;
    io.write32(R_AX_TX_SUB_CARRIER_VALUE, 0)?;
    super::io::clear8(
        io,
        R_AX_TXRATE_CHK,
        B_AX_BAND_MODE | B_AX_CHECK_CCK_EN | B_AX_RTS_LIMIT_IN_OFDM6,
    )?;
    super::io::set8(io, R_AX_TXRATE_CHK, chk_rate_mask)
}

/// Whether `_ctrl_ch` programs `ch` on 2.4 or 5 GHz (`rtw8852c_rfk.c:
/// 3941-3944`); it returns without I/O for any other.
fn ctrl_ch_programs(ch: u8) -> bool {
    !((ch > 14 && ch < 36) || (ch > 64 && ch < 100) || (ch > 144 && ch < 149) || ch > 177)
}

/// `_ch_setting(path, ch, band, is_dav)` (`rtw8852c_rfk.c:3899-3933`): the
/// band and channel fields of RF 0x18 (DAV) or 0x10018 (DDV), read, changed
/// and written whole, then 100 us.
fn ch_setting<I: AxIo + ?Sized>(
    io: &mut I,
    path: RfPath,
    chan: &Chan,
    addr: u32,
) -> Result<(), AxError> {
    let mut rf18 = read_rf(io, path, addr, RFREG_MASK)?;
    rf18 &= !(RR_CFGCH_BAND1 | RR_CFGCH_BAND0 | RR_CFGCH_CH);
    rf18 |= u32::from(chan.channel) & RR_CFGCH_CH;
    if chan.band == HwBand::G5 {
        rf18 |= CFGCH_BAND_5G;
    }
    write_rf(io, path, addr, RFREG_MASK, rf18)?;
    io.delay_us(100);
    Ok(())
}

/// `_bw_setting(path, 20 MHz, is_dav)` (`rtw8852c_rfk.c:3822-3865`): RF 0x18
/// or 0x10018 read, the path's BB CFCH_BW0 = 3 and CFCH_BW1 = 0xF, then the
/// register written back with BW 20M.
fn bw_setting<I: AxIo + ?Sized>(io: &mut I, path: RfPath, addr: u32) -> Result<(), AxError> {
    let mut rf18 = read_rf(io, path, addr, RFREG_MASK)?;
    rf18 = (rf18 & !RR_CFGCH_BW) | CFGCH_BW_20M;
    let shift = (path as u32) << 8;
    bb::field32(io, R_P0_CFCH_BW0 | shift, B_P0_CFCH_BW0, 0x3)?;
    bb::field32(io, R_P0_CFCH_BW1 | shift, B_P0_CFCH_BW1, 0xF)?;
    write_rf(io, path, addr, RFREG_MASK, rf18)
}

/// `_ctrl_bw`'s path-B workaround for the CAV cut (`rtw8852c_rfk.c:
/// 3888-3895`), after path B's two `_bw_setting`s: path B's RF reset bit
/// (`RR_RSV1` b0) cleared, path A's RF 0x18 read over SWSI, path B's
/// `RR_APK` [5:4] = 3, path A's value written to path B's RF 0x18 over
/// SWSI (the direct window is not touched), 100 us, then the reset bit set
/// again.
fn cav_copy_path_b<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    write_rf(io, RfPath::B, RR_RSV1, RR_RSV1_RST, 0x0)?;
    let tmp = read_rf(io, RfPath::A, RR_CFGCH, RFREG_MASK)?;
    write_rf(io, RfPath::B, RR_APK, RR_APK_MOD, 0x3)?;
    write_rf(io, RfPath::B, RR_CFGCH, RFREG_MASK, tmp)?;
    io.delay_us(100);
    write_rf(io, RfPath::B, RR_RSV1, RR_RSV1_RST, 0x1)
}

/// `rtw8852c_set_channel_rf(chan, PHY0)` (`rtw8852c_rfk.c:4078-4094`), that
/// is `rtw8852c_ctrl_bw_ch` at 20 MHz with `kpath = RF_AB`, on chip cut
/// `cut` (`hal.cv`, `enum rtw89_cv`: 0 is CAV):
/// 1. `_ctrl_ch`: for path A then B, `_ch_setting` on DAV (0x18, SWSI) then
///    DDV (0x10018, the direct window);
/// 2. `_ctrl_bw`: for path A then B, `_bw_setting` on DAV then DDV, and on
///    CAV only, after path B's, the copy of path A's RF 0x18 into path B's
///    (`cav_copy_path_b`);
/// 3. `_rxbb_bw`: for path A then B, the RX baseband LUT (0xEE, 0x33, 0x3F)
///    and the TIA bandwidth (RF 0x8F bits 6:5 = 2).
///
/// From RF 0x18 = 0x01001, 2.4 GHz channel 1 leaves 0x01C01 and 5 GHz
/// channel 36 leaves 0x11D24 on both paths and both windows. A 6 GHz
/// channel is refused before any I/O; its band bits and channel ranges are
/// not ported.
pub fn set_channel_rf<I: AxIo + ?Sized>(io: &mut I, chan: &Chan, cut: u8) -> Result<(), AxError> {
    if chan.band == HwBand::G6 {
        return Err(AxError::Phy("6 GHz channels are not ported"));
    }
    if ctrl_ch_programs(chan.channel) {
        for path in KPATH {
            ch_setting(io, path, chan, RR_CFGCH)?;
            ch_setting(io, path, chan, RR_CFGCH_V1)?;
        }
    }
    for path in KPATH {
        bw_setting(io, path, RR_CFGCH)?;
        bw_setting(io, path, RR_CFGCH_V1)?;
        // Without DBCC (`:3885-3886` does not `continue`).
        if path == RfPath::B && cut == CHIP_CAV {
            cav_copy_path_b(io)?;
        }
    }
    for path in KPATH {
        // _set_rxbb_bw(path, 20 MHz).
        write_rf(io, path, RR_LUTWE2, RR_LUTWE2_RTXBW, 0x1)?;
        write_rf(io, path, RR_LUTWA, RR_LUTWA_M2, 0xA)?;
        write_rf(io, path, RR_LUTWD0, RR_LUTWD0_LB, 0x1B)?;
        write_rf(io, path, RR_LUTWE2, RR_LUTWE2_RTXBW, 0x0)?;
        // _set_tia_bw(path, 20 MHz).
        write_rf(io, path, RR_RXBB2, RR_RXBB2_EBW, 0x2)?;
    }
    Ok(())
}

// rtw8852c_set_channel_bb over reg.h: BB registers (offsets from
// `bb::CR_BASE`).
/// `R_BK_FC0_INV_V1` and `R_CCK_FC0_INV_V1`, `B_*_FC0_INV_MSK_V1` [18:0]
/// each (reg.h:9746-9749).
const R_BK_FC0_INV_V1: u32 = 0x4A1C;
const R_CCK_FC0_INV_V1: u32 = 0x4A20;
const B_FC0_INV_MSK_V1: u32 = 0x7_FFFF;
/// `R_RPL_OFST` / `B_RPL_OFST_MASK` [14:8] (reg.h:9342-9343).
const R_RPL_OFST: u32 = 0x2340;
const B_RPL_OFST_MASK: u32 = 0x7F00;
/// `R_PATH0_G_TIA0_LNA6_OP1DB_V1` and `R_PATH1_G_TIA0_LNA6_OP1DB_V1`
/// (`rssi_ofst_addr`), with `B_PATH0_R_G_OFST_MASK` [23:16]
/// (reg.h:9620-9623, 9669).
const RSSI_OFST_ADDR: [u32; 2] = [0x4694, 0x4778];
const B_PATH0_R_G_OFST_MASK: u32 = 0x00FF_0000;
/// `R_RPL_PATHAB` with `B_RPL_PATHA_MASK` [15:8] and `B_RPL_PATHB_MASK`
/// [23:16]; `R_RSSI_M_PATHAB` with `B_RSSI_M_PATHA_MASK` [7:0] and
/// `B_RSSI_M_PATHB_MASK` [15:8] (reg.h:9813-9818).
const R_RPL_PATHAB: u32 = 0x4E0C;
const RPL_MASK: [u32; 2] = [0x0000_FF00, 0x00FF_0000];
const R_RSSI_M_PATHAB: u32 = 0x4E2C;
const RPL_TB_MASK: [u32; 2] = [0x0000_00FF, 0x0000_FF00];
/// `R_PATH0_BAND_SEL_V1` and `R_PATH1_BAND_SEL_V1`, each
/// `B_PATH*_BAND_SEL_MSK_V1` b17 (reg.h:9627-9628, 9673-9674).
const BAND_SEL: [u32; 2] = [0x4738, 0x4AA4];
const B_BAND_SEL_MSK_V1: u32 = 1 << 17;
/// `R_2P4G_BAND` / `B_2P4G_BAND_SEL` (reg.h:9699-9700).
const R_2P4G_BAND: u32 = 0x4970;
const B_2P4G_BAND_SEL: u32 = 1 << 1;
/// `R_FC0_V1` with `B_FC0_MSK_V1` [12:0], the same word as
/// `R_RX_BW40_2XFFT_EN_V1` / `B_RX_BW40_2XFFT_EN_MSK_V1` b26
/// (reg.h:9819-9822).
const R_FC0_V1: u32 = 0x4E30;
const B_FC0_MSK_V1: u32 = 0x1FFF;
const B_RX_BW40_2XFFT_EN_MSK_V1: u32 = 1 << 26;
/// `R_FC0_BW` with `B_FC0_BW_SET` [31:30] and `B_FC0_BW_INV` [6:0]
/// (reg.h:9701-9707).
const R_FC0_BW: u32 = 0x4974;
const B_FC0_BW_SET: u32 = 0xC000_0000;
const B_FC0_BW_INV: u32 = 0x7F;
/// `R_PCOEFF0_V1`..`R_PCOEFFE_V1` in `rtw8852c_ctrl_ch`'s order, each
/// `B_PCOEFF*_MSK_V1` [23:0] (reg.h:9549-9564), with the values for channel
/// 14 and for every other 2.4 GHz channel (rtw8852c.c:1199-1236).
const PCOEFF: [u32; 8] = [
    0x45BC, 0x45CC, 0x45D0, 0x45D4, 0x45D8, 0x45C0, 0x45C4, 0x45C8,
];
const B_PCOEFF_MSK_V1: u32 = 0xFF_FFFF;
const PCOEFF_CH14: [u32; 8] = [
    0x3B_13FF, 0x1C_42DE, 0xFD_B0AD, 0xF6_0F6E, 0xFD_8F92, 0x02_D011, 0x01_C02C, 0xFF_F00A,
];
const PCOEFF_OTHER: [u32; 8] = [
    0x3D_23FF, 0x29_B354, 0x0F_C1C8, 0xFD_B053, 0xF8_6F9A, 0xFA_EF92, 0xFE_5FCC, 0xFF_DFF5,
];
/// `R_MAC_PIN_SEL` / `B_CH_IDX_SEG0` [23:16] (reg.h:8903-8905).
const R_MAC_PIN_SEL: u32 = 0x0734;
const B_CH_IDX_SEG0: u32 = 0x00FF_0000;
/// `R_CHBW_MOD` with `B_CHBW_MOD_SBW` [13:12] and `B_CHBW_MOD_PRICH`
/// [11:8] (reg.h:9711-9715).
const R_CHBW_MOD: u32 = 0x4978;
const B_CHBW_MOD_SBW: u32 = 0x3000;
const B_CHBW_MOD_PRICH: u32 = 0x0F00;
/// `R_PATH0_SAMPL_DLY_T_V1` and `R_PATH1_..`, [27:26]; `R_PATH0_BW_SEL_V1`
/// and `R_PATH1_..`, [8:5] (reg.h:10384-10385, 10454-10460). They are the
/// registers `_bw_setting` names `R_P0_CFCH_BW0`/`BW1`.
const SAMPL_DLY_T: [u32; 2] = [0xC0D4, 0xC1D4];
const B_SAMPL_DLY_T_MSK_V1: u32 = 0x0C00_0000;
const BW_SEL: [u32; 2] = [0xC0D8, 0xC1D8];
const B_BW_SEL_MSK_V1: u32 = 0x01E0;
/// `R_T2F_GI_COMB` / `B_T2F_GI_COMB_EN` (reg.h:9499-9500).
const R_T2F_GI_COMB: u32 = 0x4424;
const B_T2F_GI_COMB_EN: u32 = 1 << 2;
/// `rtw8852c_bw_setting`'s `adc_sel` [14:13] and `wbadc_sel` [5:4]
/// (rtw8852c.c:1241-1265).
const ADC_SEL: [u32; 2] = [0xC0EC, 0xC1EC];
const B_ADC_SEL: u32 = 0x6000;
const WBADC_SEL: [u32; 2] = [0xC0E4, 0xC1E4];
const B_WBADC_SEL: u32 = 0x30;
/// `R_SNDCCA_A1` / `B_SNDCCA_A1_EN` and `R_SNDCCA_A2` / `B_SNDCCA_A2_VAL`,
/// [19:12] each (reg.h:9042-9045).
const R_SNDCCA_A1: u32 = 0x0C9C;
const R_SNDCCA_A2: u32 = 0x0CA0;
const B_SNDCCA: u32 = 0x000F_F000;
/// `R_UPD_CLK_ADC` / `B_ENABLE_CCK` (reg.h:8886-8890).
const R_UPD_CLK_ADC: u32 = 0x0700;
const B_ENABLE_CCK: u32 = 1 << 5;
/// `R_PD_ARBITER_OFF` / `B_PD_ARBITER_OFF` (reg.h:9040-9041).
const R_PD_ARBITER_OFF: u32 = 0x0C80;
const B_PD_ARBITER_OFF: u32 = 1 << 31;
/// `R_SEG0CSI` / `B_SEG0CSI_IDX` [10:0] and `R_SEG0CSI_EN` / `B_SEG0CSI_EN`
/// (reg.h:9484-9489).
const R_SEG0CSI: u32 = 0x42AC;
const B_SEG0CSI_IDX: u32 = 0x07FF;
const R_SEG0CSI_EN: u32 = 0x42C4;
const B_SEG0CSI_EN: u32 = 1 << 23;
/// `rtw8852c_nbi_reg_def` (rtw8852c.c:1448-1465): each path's notch 1 and
/// notch 2 register, both with the index [7:0], fraction [11:10] and
/// enable b12.
const NBI_NOTCH: [(u32, u32); 2] = [(0x4C14, 0x4C20), (0x4CD8, 0x4CE4)];
const NBI_IDX: u32 = 0x00FF;
const NBI_FRAC_IDX: u32 = 0x0C00;
const NBI_EN: u32 = 0x1000;
/// `R_PD_BOOST_EN` / `B_PD_BOOST_EN` (reg.h:9739-9740).
const R_PD_BOOST_EN: u32 = 0x49E8;
const B_PD_BOOST_EN: u32 = 1 << 7;
/// `R_PATH0_5MDET` and `R_PATH1_5MDET`, `B_PATH*_5MDET_EN` b12; and
/// `R_ASSIGN_SBD_OPT` / `B_ASSIGN_SBD_OPT_EN` (reg.h:9506-9507, 9782-9798).
const PATH_5MDET: [u32; 2] = [0x4C4C, 0x4D10];
const B_5MDET_EN: u32 = 1 << 12;
const R_ASSIGN_SBD_OPT: u32 = 0x4450;
const B_ASSIGN_SBD_OPT_EN: u32 = 1 << 24;
/// `R_CDD_EVM_CHK_EN` / `B_CDD_EVM_CHK_EN` (reg.h:9625-9626).
const R_CDD_EVM_CHK_EN: u32 = 0x46C0;
const B_CDD_EVM_CHK_EN: u32 = 1 << 0;
/// `R_P0_TXPW_RSTB` and `R_P1_TXPW_RSTB`, with `B_P*_TXPW_RSTB_TSSI` b31 |
/// `_MANON` b30 (reg.h:9936-9938, 10097-10099).
const TXPW_RSTB: [u32; 2] = [0x58DC, 0x78DC];
const B_TXPW_RSTB_TSSI_MANON: u32 = 0xC000_0000;
/// `R_MUIC` / `B_MUIC_EN` (reg.h:9472-9473).
const R_MUIC: u32 = 0x40F8;
const B_MUIC_EN: u32 = 1 << 0;
/// `R_MAC_SEL` / `B_MAC_SEL_MOD` [4:2] (reg.h:8944-8949).
const R_MAC_SEL: u32 = 0x09A4;
const B_MAC_SEL_MOD: u32 = 0x1C;

// reg.h: MAC registers of band 0 that `rtw8852c_ctrl_tx_path_tmac` writes.
/// `R_AX_MACID_ANT_TABLE` ..= `R_AX_MACID_ANT_TABLE_LAST` (reg.h:3766-3767).
const R_AX_MACID_ANT_TABLE: u32 = 0xDC00;
const R_AX_MACID_ANT_TABLE_LAST: u32 = 0xDDFC;
/// `R_AX_PATH_COM0`..`11` with their values for `RF_AB`: `AX_PATH_COM*_PATHAB`
/// for 0, 1, 2, 7 and 8, `AX_PATH_COM*_DFVAL` for the rest (reg.h:3565-3605).
const PATH_COM_AB: [(u32, u32); 12] = [
    (0xD800, 0x1999_9980),
    (0xD804, 0x3333_3333),
    (0xD808, 0x0120_9333),
    (0xD80C, 0x4924_9249),
    (0xD810, 0x1C9C_9C49),
    (0xD814, 0x3939_3939),
    (0xD818, 0x3939_3939),
    (0xD81C, 0x3939_3939),
    (0xD820, 0x0000_3939),
    (0xD824, 0x0000_07C0),
    (0xD828, 0xE000_0000),
    (0xD82C, 0x0000_0000),
];

/// `rtw8852c_sco_barker_threshold` and `rtw8852c_sco_cck_threshold`
/// (rtw8852c.c:892-900), by channel - 1.
const SCO_BARKER_THRESHOLD: [u32; 14] = [
    0x1FE4F, 0x1FF5E, 0x2006C, 0x2017B, 0x2028A, 0x20399, 0x204A8, 0x205B6, 0x206C5, 0x207D4,
    0x208E3, 0x209F2, 0x20B00, 0x20D8A,
];
const SCO_CCK_THRESHOLD: [u32; 14] = [
    0x2BDAC, 0x2BF21, 0x2C095, 0x2C209, 0x2C37E, 0x2C4F2, 0x2C666, 0x2C7DB, 0x2C94F, 0x2CAC3,
    0x2CC38, 0x2CDAC, 0x2CF21, 0x2D29E,
];

/// `LNA_GAIN_NUM` and `TIA_GAIN_NUM` (core.h:6829-6830).
const LNA_GAIN_NUM: usize = 7;
/// `bb_gain_lna` (rtw8852c.c:932-947): the 2G register by path, the 5G
/// register by path, the mask.
const BB_GAIN_LNA: [([u32; 2], [u32; 2], u32); LNA_GAIN_NUM] = [
    ([0x4678, 0x475C], [0x45DC, 0x4740], 0x00FF_0000),
    ([0x4678, 0x475C], [0x45DC, 0x4740], 0xFF00_0000),
    ([0x467C, 0x4760], [0x4660, 0x4744], 0x0000_00FF),
    ([0x467C, 0x4760], [0x4660, 0x4744], 0x0000_FF00),
    ([0x467C, 0x4760], [0x4660, 0x4744], 0x00FF_0000),
    ([0x467C, 0x4760], [0x4660, 0x4744], 0xFF00_0000),
    ([0x4680, 0x4764], [0x4664, 0x4748], 0x0000_00FF),
];
/// `bb_gain_tia` (rtw8852c.c:949-954), laid out as [`BB_GAIN_LNA`].
const BB_GAIN_TIA: [([u32; 2], [u32; 2], u32); 2] = [
    ([0x4680, 0x4764], [0x4664, 0x4748], 0x00FF_0000),
    ([0x4680, 0x4764], [0x4664, 0x4748], 0xFF00_0000),
];
/// `bb_gain_bypass_lna` (rtw8852c.c:963-979): the 2G registers by path,
/// the 5G registers by path, the 2G mask, the 5G mask.
const BB_GAIN_BYPASS_LNA: [([u32; 2], [u32; 2], u32, u32); LNA_GAIN_NUM] = [
    ([0x4BB8, 0x4C7C], [0x4BB4, 0x4C78], 0xFF00_0000, 0x0000_00FF),
    ([0x4BBC, 0x4C80], [0x4BB4, 0x4C78], 0x0000_00FF, 0x0000_FF00),
    ([0x4BBC, 0x4C80], [0x4BB4, 0x4C78], 0x0000_FF00, 0x00FF_0000),
    ([0x4BBC, 0x4C80], [0x4BB4, 0x4C78], 0x00FF_0000, 0xFF00_0000),
    ([0x4BBC, 0x4C80], [0x4BB8, 0x4C7C], 0xFF00_0000, 0x0000_00FF),
    ([0x4BC0, 0x4C84], [0x4BB8, 0x4C7C], 0x0000_00FF, 0x0000_FF00),
    ([0x4BC0, 0x4C84], [0x4BB8, 0x4C7C], 0x0000_FF00, 0x00FF_0000),
];
/// `bb_gain_op1db_a.reg` (rtw8852c.c:991-1010): the `lna` registers by
/// path, the `tia_lna` registers by path, the mask.
const BB_GAIN_OP1DB: [([u32; 2], [u32; 2], u32); LNA_GAIN_NUM] = [
    ([0x4668, 0x474C], [0x4670, 0x4754], 0x0000_00FF),
    ([0x4668, 0x474C], [0x4670, 0x4754], 0x0000_FF00),
    ([0x4668, 0x474C], [0x4670, 0x4754], 0x00FF_0000),
    ([0x4668, 0x474C], [0x4670, 0x4754], 0xFF00_0000),
    ([0x466C, 0x4750], [0x4674, 0x4758], 0x0000_00FF),
    ([0x466C, 0x4750], [0x4674, 0x4758], 0x0000_FF00),
    ([0x466C, 0x4750], [0x4674, 0x4758], 0x00FF_0000),
];
/// `bb_gain_op1db_a.reg_tia0_lna6` and `.mask_tia0_lna6` (:1008-1009).
const BB_GAIN_OP1DB_TIA0_LNA6: [u32; 2] = [0x4674, 0x4758];
const B_OP1DB_TIA0_LNA6: u32 = 0xFF00_0000;

/// `RTW89_GAIN_OFFSET_2G_CCK` (core.h:138-154), an index into
/// `RxGain::offset`.
const GAIN_OFFSET_2G_CCK: usize = 0;

/// `CARRIER_SPACING_312_5`, `CARRIER_SPACING_78_125` and `MAX_TONE_NUM`
/// (rtw8852c.c:1423-1425).
const CARRIER_SPACING_312_5: u32 = 312_500;
const CARRIER_SPACING_78_125: u32 = 78_125;
const MAX_TONE_NUM: u32 = 2048;

/// What the baseband part of a channel switch needs beyond the [`Chan`]:
/// the firmware's BB gain table and the RPL bias as the bring-up decoded
/// and read them ([`bb::init_bb_reg`], [`super::phy::calibrate`]), the
/// efuse RX gain offsets ([`super::efuse::rx_gain`]), and the chip cut
/// (`hal.cv`) the RF part branches on ([`set_channel_rf`]).
#[derive(Clone, Copy, Debug)]
pub struct ChannelInputs<'a> {
    /// `rtwdev->bb_gain.ax`.
    pub gain: &'a BbGain,
    /// `efuse_gain.offset` and `offset_valid`.
    pub rx_gain: &'a RxGain,
    /// `efuse_gain.offset_base[RTW89_PHY_0]`.
    pub offset_base: i8,
    /// `hal.cv` (`enum rtw89_cv`, 0 is CAV).
    pub cut: u8,
}

/// The index of `path` into the RTL8852C's two-path arrays.
fn path_index(path: RfPath) -> Result<usize, AxError> {
    match path {
        RfPath::A => Ok(0),
        RfPath::B => Ok(1),
        RfPath::C | RfPath::D => Err(AxError::Phy("the RTL8852C has RF paths A and B only")),
    }
}

/// An `s8` gain as `rtw89_phy_write32_mask` takes it: sign-extended to the
/// u32 that the mask then cuts.
fn s8(v: i8) -> u32 {
    i32::from(v) as u32
}

/// `rtw8852c_set_gain_error(subband, path)` (rtw8852c.c:1012-1074): the
/// firmware's gains for `chan`'s BB gain band into the path's gain
/// registers. For i = 0..6 the LNA gain, then the bypass LNA gain, then on
/// 5 GHz the LNA and TIA+LNA OP1dB; then on 5 GHz the TIA1_LNA6 OP1dB; then
/// the two TIA gains. Every write is a read-modify-write of the field.
pub fn set_gain_error<I: AxIo + ?Sized>(
    io: &mut I,
    gain: &BbGain,
    chan: &Chan,
    path: RfPath,
) -> Result<(), AxError> {
    let p = path_index(path)?;
    let gb = chan.bb_gain_band();
    let is_2g = chan.subband == Subband::G2;
    for i in 0..LNA_GAIN_NUM {
        let (reg_g, reg_a, mask) = BB_GAIN_LNA[i];
        let reg = if is_2g { reg_g[p] } else { reg_a[p] };
        bb::field32(io, reg, mask, s8(gain.lna_gain[gb][p][i]))?;

        let (reg_g, reg_a, mask_g, mask_a) = BB_GAIN_BYPASS_LNA[i];
        let (reg, mask) = if is_2g {
            (reg_g[p], mask_g)
        } else {
            (reg_a[p], mask_a)
        };
        bb::field32(io, reg, mask, s8(gain.lna_gain_bypass[gb][p][i]))?;

        if !is_2g {
            let (lna, tia_lna, mask) = BB_GAIN_OP1DB[i];
            bb::field32(io, lna[p], mask, s8(gain.lna_op1db[gb][p][i]))?;
            bb::field32(io, tia_lna[p], mask, s8(gain.tia_lna_op1db[gb][p][i]))?;
        }
    }
    if !is_2g {
        bb::field32(
            io,
            BB_GAIN_OP1DB_TIA0_LNA6[p],
            B_OP1DB_TIA0_LNA6,
            s8(gain.tia_lna_op1db[gb][p][LNA_GAIN_NUM]),
        )?;
    }
    for (i, &(reg_g, reg_a, mask)) in BB_GAIN_TIA.iter().enumerate() {
        let reg = if is_2g { reg_g[p] } else { reg_a[p] };
        bb::field32(io, reg, mask, s8(gain.tia_gain[gb][p][i]))?;
    }
    Ok(())
}

/// `rtw8852c_set_gain_offset(chan, PHY0, path)` (rtw8852c.c:1076-1117):
/// nothing unless the efuse offsets are valid. Then, with `q4` the RPL bias
/// `offset_base` and arithmetic shifts on i32:
/// 1. 2.4 GHz only: the CCK RPL offset, `clamp(-cck * 8 + (q4 >> 1), -64,
///    63)` into BB 0x2340 bits 14:8 (shared by both paths);
/// 2. with `q0 = -ofdm` (the subband's OFDM offset), the path's RSSI offset
///    `clamp(-(q0 * 4 + (q4 >> 2)), -128, 127)`;
/// 3. `clamp(q0 * 16, -128, 127)` into the path's RPL and RPL TB fields.
pub fn set_gain_offset<I: AxIo + ?Sized>(
    io: &mut I,
    rx_gain: &RxGain,
    offset_base: i8,
    chan: &Chan,
    path: RfPath,
) -> Result<(), AxError> {
    if !rx_gain.valid {
        return Ok(());
    }
    let p = path_index(path)?;
    let offset_base_q4 = i32::from(offset_base);
    if chan.band == HwBand::G2 {
        let offset_q0 = i32::from(rx_gain.offset[p][GAIN_OFFSET_2G_CCK]);
        let tmp = ((-offset_q0 << 3) + (offset_base_q4 >> 1)).clamp(-64, 63);
        bb::field32(io, R_RPL_OFST, B_RPL_OFST_MASK, (tmp & 0x7F) as u32)?;
    }
    let offset_q0 = -i32::from(rx_gain.offset[p][chan.ofdm_gain_offset()]);
    let tmp = (-((offset_q0 << 2) + (offset_base_q4 >> 2))).clamp(-128, 127);
    bb::field32(
        io,
        RSSI_OFST_ADDR[p],
        B_PATH0_R_G_OFST_MASK,
        (tmp & 0xFF) as u32,
    )?;
    let tmp = ((offset_q0 << 4).clamp(-128, 127) & 0xFF) as u32;
    bb::field32(io, R_RPL_PATHAB, RPL_MASK[p], tmp)?;
    bb::field32(io, R_RSSI_M_PATHAB, RPL_TB_MASK[p], tmp)
}

/// A [`Chan`] exactly as [`Chan::new20`] builds it, or an error: the BB
/// tables are indexed by channel and the SCO divides by the frequency, and
/// the fields are public.
pub(super) fn check_chan(chan: &Chan) -> Result<(), AxError> {
    match Chan::new20(chan.band, chan.channel) {
        Ok(built) if built == *chan => Ok(()),
        _ => Err(AxError::Phy("not a 20 MHz 2.4 or 5 GHz channel")),
    }
}

/// `rtw8852c_ctrl_sco_cck` (rtw8852c.c:902-930) at 20 MHz: the Barker and
/// CCK SCO thresholds of a 2.4 GHz channel 1-14.
fn ctrl_sco_cck<I: AxIo + ?Sized>(io: &mut I, chan: &Chan) -> Result<(), AxError> {
    let i = usize::from(chan.channel - 1);
    bb::field32(
        io,
        R_BK_FC0_INV_V1,
        B_FC0_INV_MSK_V1,
        SCO_BARKER_THRESHOLD[i],
    )?;
    bb::field32(io, R_CCK_FC0_INV_V1, B_FC0_INV_MSK_V1, SCO_CCK_THRESHOLD[i])
}

/// `rtw8852c_ctrl_ch(chan, PHY0)` without DBCC (rtw8852c.c:1119-1240):
/// each path's gain error and offset and band select, the 2.4 GHz band
/// select of PHY 0 cleared, the centre frequency and its SCO, the 2.4 GHz
/// CCK filter, then the channel index. Linux's zero-frequency return
/// cannot happen for a checked [`Chan`].
fn ctrl_ch<I: AxIo + ?Sized>(
    io: &mut I,
    chan: &Chan,
    inputs: &ChannelInputs<'_>,
) -> Result<(), AxError> {
    let is_2g = chan.band == HwBand::G2;
    for (path, band_sel) in [RfPath::A, RfPath::B].into_iter().zip(BAND_SEL) {
        set_gain_error(io, inputs.gain, chan, path)?;
        set_gain_offset(io, inputs.rx_gain, inputs.offset_base, chan, path)?;
        bb::field32(io, band_sel, B_BAND_SEL_MSK_V1, u32::from(is_2g))?;
    }
    bb::clear32(io, R_2P4G_BAND, B_2P4G_BAND_SEL)?;
    bb::field32(io, R_FC0_V1, B_FC0_MSK_V1, u32::from(chan.freq))?;
    bb::field32(io, R_FC0_BW, B_FC0_BW_INV, u32::from(chan.sco()))?;
    if is_2g {
        let coeff = if chan.channel == 14 {
            PCOEFF_CH14
        } else {
            PCOEFF_OTHER
        };
        for (reg, v) in PCOEFF.into_iter().zip(coeff) {
            bb::field32(io, reg, B_PCOEFF_MSK_V1, v)?;
        }
    }
    bb::field32(io, R_MAC_PIN_SEL, B_CH_IDX_SEG0, u32::from(chan.chan_idx()))
}

/// `rtw8852c_ctrl_bw(pri_ch_idx, 20 MHz, PHY0)` (rtw8852c.c:1280-1386)
/// without DBCC: no BW40 set or sub-band mode, the sample delay and BW
/// select of both paths, no 2x FFT or GI combining, `rtw8852c_bw_setting`
/// A then B (:1241-1265) and `rtw8852c_edcca_per20_bitmap_sifs` (:1267-1278).
fn ctrl_bw_20<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    bb::field32(io, R_FC0_BW, B_FC0_BW_SET, 0)?;
    bb::field32(io, R_CHBW_MOD, B_CHBW_MOD_SBW, 0)?;
    bb::field32(io, R_CHBW_MOD, B_CHBW_MOD_PRICH, 0)?;
    for reg in SAMPL_DLY_T {
        bb::field32(io, reg, B_SAMPL_DLY_T_MSK_V1, 0x3)?;
    }
    for reg in BW_SEL {
        bb::field32(io, reg, B_BW_SEL_MSK_V1, 0xF)?;
    }
    bb::field32(io, R_FC0_V1, B_RX_BW40_2XFFT_EN_MSK_V1, 0)?;
    bb::field32(io, R_T2F_GI_COMB, B_T2F_GI_COMB_EN, 0)?;
    for (adc, wbadc) in ADC_SEL.into_iter().zip(WBADC_SEL) {
        bb::field32(io, adc, B_ADC_SEL, 0x0)?;
        bb::field32(io, wbadc, B_WBADC_SEL, 0x2)?;
    }
    bb::field32(io, R_SNDCCA_A1, B_SNDCCA, 0xFF)?;
    bb::field32(io, R_SNDCCA_A2, B_SNDCCA, 0)
}

/// `s32_div_u32_round_down` (util.h:48-65): the floor quotient and the
/// non-negative remainder.
fn s32_div_u32_round_down(dividend: i32, divisor: u32) -> (i32, i32) {
    let divisor = divisor as i32;
    let (mut quotient, mut remainder) = (dividend / divisor, dividend % divisor);
    if remainder < 0 {
        quotient -= 1;
        remainder += divisor;
    }
    (quotient, remainder)
}

/// `s32_div_u32_round_closest` (util.h:67-70). C adds `divisor / 2` in u32
/// and passes the result back as an s32, which wraps the same way.
fn s32_div_u32_round_closest(dividend: i32, divisor: u32) -> i32 {
    s32_div_u32_round_down(dividend.wrapping_add((divisor / 2) as i32), divisor).0
}

/// `rtw8852c_spur_freq` (rtw8852c.c:1388-1421) at 20 MHz, in MHz: 2.4 GHz
/// channels 5-8 and 13, and 5 GHz 151, 153, 155 and 163, have a spur; any
/// other channel has none (0).
fn spur_freq(chan: &Chan) -> u32 {
    match (chan.band, chan.channel) {
        (HwBand::G2, 5..=8) => 2440,
        (HwBand::G2, 13) => 2480,
        (HwBand::G5, 151 | 153 | 155 | 163) => 5760,
        _ => 0,
    }
}

/// `(spur_freq - fc) * 1000000` in Hz as Linux computes it: u32 arithmetic
/// that wraps, taken as an s32.
fn freq_diff(spur_freq: u32, fc: u16) -> i32 {
    spur_freq
        .wrapping_sub(u32::from(fc))
        .wrapping_mul(1_000_000) as i32
}

/// `rtw8852c_set_csi_tone_idx(chan, PHY0)` (rtw8852c.c:1427-1446).
fn set_csi_tone_idx<I: AxIo + ?Sized>(io: &mut I, chan: &Chan) -> Result<(), AxError> {
    let spur = spur_freq(chan);
    if spur == 0 {
        return bb::field32(io, R_SEG0CSI_EN, B_SEG0CSI_EN, 0);
    }
    let csi_idx = s32_div_u32_round_closest(freq_diff(spur, chan.freq), CARRIER_SPACING_78_125);
    let (_, csi_tone_idx) = s32_div_u32_round_down(csi_idx, MAX_TONE_NUM);
    bb::field32(io, R_SEG0CSI, B_SEG0CSI_IDX, csi_tone_idx as u32)?;
    bb::field32(io, R_SEG0CSI_EN, B_SEG0CSI_EN, 1)
}

/// `rtw8852c_set_nbi_tone_idx(chan, path)` (rtw8852c.c:1467-1525) at
/// 20 MHz, on the path's notch registers. Without a spur, notch 1 is
/// disabled twice, as Linux writes it. With one, notch 1 gets the tone
/// `(nbi + 32) mod 64` and the fraction `round_closest(r / 78125)`, where
/// `nbi` and `r` are the floor quotient and remainder of the offset by
/// 312.5 kHz; it is enabled through a 0 then 1, and notch 2 is disabled.
fn set_nbi_tone_idx<I: AxIo + ?Sized>(
    io: &mut I,
    chan: &Chan,
    (notch1, notch2): (u32, u32),
) -> Result<(), AxError> {
    let spur = spur_freq(chan);
    if spur == 0 {
        bb::field32(io, notch1, NBI_EN, 0)?;
        return bb::field32(io, notch1, NBI_EN, 0);
    }
    let (nbi_idx, nbi_frac_idx) =
        s32_div_u32_round_down(freq_diff(spur, chan.freq), CARRIER_SPACING_312_5);
    let (_, nbi_tone_idx) = s32_div_u32_round_down(nbi_idx + 32, 64);
    let nbi_frac_tone_idx = s32_div_u32_round_closest(nbi_frac_idx, CARRIER_SPACING_78_125);
    bb::field32(io, notch1, NBI_IDX, nbi_tone_idx as u32)?;
    bb::field32(io, notch1, NBI_FRAC_IDX, nbi_frac_tone_idx as u32)?;
    bb::field32(io, notch1, NBI_EN, 0)?;
    bb::field32(io, notch1, NBI_EN, 1)?;
    bb::field32(io, notch2, NBI_EN, 0)
}

/// `rtw8852c_spur_elimination(chan, RTW89_SC_DONT_CARE, PHY0)`
/// (rtw8852c.c:1549-1593) at 20 MHz without DBCC: the CSI tone, the NBI
/// notch of path A then B, then PD boost on (the primary index is neither
/// UP3X nor LOW3X).
fn spur_elimination<I: AxIo + ?Sized>(io: &mut I, chan: &Chan) -> Result<(), AxError> {
    set_csi_tone_idx(io, chan)?;
    for notch in NBI_NOTCH {
        set_nbi_tone_idx(io, chan, notch)?;
    }
    bb::field32(io, R_PD_BOOST_EN, B_PD_BOOST_EN, 1)
}

/// `rtw8852c_ctrl_tx_path_tmac(RF_AB, MAC_0)` (rtw8852c.c:2430-2490): the
/// MAC select of PHY 0 and PHY 1 (one register without DBCC), the 128 MACID
/// antenna words cleared, then the path common table for two TX paths.
fn ctrl_tx_path_tmac<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    bb::field32(io, R_MAC_SEL, B_MAC_SEL_MOD, 0)?;
    bb::field32(io, R_MAC_SEL, B_MAC_SEL_MOD, 0)?;
    for addr in (R_AX_MACID_ANT_TABLE..=R_AX_MACID_ANT_TABLE_LAST).step_by(4) {
        io.write32(addr, 0)?;
    }
    for (addr, data) in PATH_COM_AB {
        io.write32(addr, data)?;
    }
    Ok(())
}

/// `rtw8852c_set_channel_bb(chan, PHY0)` (rtw8852c.c:1834-1930) for a
/// 20 MHz 2.4 or 5 GHz [`Chan`] without DBCC, in Linux's order:
/// 1. 2.4 GHz only, the CCK SCO thresholds (`ctrl_sco_cck`);
/// 2. `ctrl_ch`: [`set_gain_error`] and [`set_gain_offset`] with each path's
///    band select, the frequency and SCO, the CCK filter, the channel index;
/// 3. `ctrl_bw` at 20 MHz;
/// 4. CCK on for 2.4 GHz, off for 5 GHz (`B_ENABLE_CCK`, CCK RX CCA,
///    the PD arbiter);
/// 5. `spur_elimination`: the CSI tone and both NBI notches, PD boost;
/// 6. `ctrl_btg_bt_rx(is_2g)` ([`super::dm`]);
/// 7. `5m_mask` off, CDD EVM check on, the TX power reset of both paths,
///    MU-MIMO IC off (`R_MUIC`, not 6 GHz);
/// 8. `ctrl_tx_path_tmac(RF_AB)` (`hal.antenna_tx` is 0, so both paths);
/// 9. `bb_reset_all` (`bb::bb_reset`).
///
/// A channel [`Chan::new20`] would not build is refused before any I/O.
/// Every masked write is a read-modify-write; an I/O error stops the
/// sequence where it happened.
pub fn set_channel_bb<I: AxIo + ?Sized>(
    io: &mut I,
    chan: &Chan,
    inputs: &ChannelInputs<'_>,
) -> Result<(), AxError> {
    check_chan(chan)?;
    let is_2g = chan.band == HwBand::G2;
    if is_2g {
        ctrl_sco_cck(io, chan)?;
    }
    ctrl_ch(io, chan, inputs)?;
    ctrl_bw_20(io)?;
    let cck_en = u32::from(is_2g);
    bb::field32(io, R_UPD_CLK_ADC, B_ENABLE_CCK, cck_en)?;
    bb::field32(io, R_RXCCA_V1, B_RXCCA_DIS_V1, 1 - cck_en)?;
    bb::field32(io, R_PD_ARBITER_OFF, B_PD_ARBITER_OFF, 1 - cck_en)?;
    spur_elimination(io, chan)?;
    ctrl_btg_bt_rx(io, is_2g)?;
    // rtw8852c_5m_mask (:1595-1646): no 5 MHz mask at 20 MHz.
    for reg in PATH_5MDET {
        bb::field32(io, reg, B_5MDET_EN, 0)?;
    }
    bb::field32(io, R_ASSIGN_SBD_OPT, B_ASSIGN_SBD_OPT_EN, 0)?;
    // The 160 MHz block (:1870-1888) does not apply.
    bb::field32(io, R_CDD_EVM_CHK_EN, B_CDD_EVM_CHK_EN, 1)?;
    for reg in TXPW_RSTB {
        bb::field32(io, reg, B_TXPW_RSTB_TSSI_MANON, 0x1)?;
        bb::field32(io, reg, B_TXPW_RSTB_TSSI_MANON, 0x3)?;
    }
    bb::clear32(io, R_MUIC, B_MUIC_EN)?;
    ctrl_tx_path_tmac(io)?;
    bb::bb_reset(io)
}

/// What `chip->ops->set_txpwr` reads in `__rtw89_set_channel`: the
/// firmware's TX power tables for the efuse RFE type
/// ([`super::txpwr_tables`]) and the regulatory indices
/// ([`super::regd::select`]).
#[derive(Clone, Copy)]
pub struct TxPower<'a> {
    pub tables: &'a TxPowerTables,
    pub regds: Regds,
}

/// A failed channel switch: `step` is `"set channel"` for the help, MAC,
/// BB or RF part (or a channel refused before any I/O), `"tx power"` for
/// [`super::txpwr::set_txpwr`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChannelError {
    pub step: &'static str,
    pub error: AxError,
}

fn at(step: &'static str) -> impl Fn(AxError) -> ChannelError {
    move |error| ChannelError { step, error }
}

/// `__rtw89_set_channel` (`core.c:568-598`) on MAC 0 and PHY 0 for a 20 MHz
/// 2.4 or 5 GHz [`Chan`], as `rtw8852c_set_channel` (`rtw8852c.c:1932-1940`)
/// orders the chip parts: [`help_enter`], [`set_channel_mac`],
/// [`set_channel_bb`], [`set_channel_rf`] on `inputs.cut`; then
/// `chip->ops->set_txpwr` (`core.c:586`, [`super::txpwr::set_txpwr`] from
/// `power`), with the TX scheduler still stopped and the BB still held in
/// reset; then [`help_exit`] with the TX enable `help_enter` returned
/// (`core.c:588`). The band notify that follows does no I/O in Stela's state
/// (spec S6.6).
///
/// A channel [`Chan::new20`] would not build is refused before any I/O.
/// Any failure stops the switch where it happened, leaving the TX scheduler
/// stopped and the BB held in reset as far as [`help_enter`] got; the
/// caller powers the chip off (spec S1.16). The error names the part as its
/// step.
pub fn set_channel<I: AxIo + ?Sized>(
    io: &mut I,
    chan: &Chan,
    inputs: &ChannelInputs<'_>,
    power: TxPower<'_>,
) -> Result<(), ChannelError> {
    let switching = at("set channel");
    check_chan(chan).map_err(&switching)?;
    let tx_en = help_enter(io, chan).map_err(&switching)?;
    set_channel_mac(io, chan).map_err(&switching)?;
    set_channel_bb(io, chan, inputs).map_err(&switching)?;
    set_channel_rf(io, chan, inputs.cut).map_err(&switching)?;
    txpwr::set_txpwr(io, power.tables, power.regds, chan).map_err(at("tx power"))?;
    help_exit(io, chan, tx_en).map_err(switching)
}

#[cfg(test)]
mod tests;

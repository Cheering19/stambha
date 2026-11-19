// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE MAC pre-init for firmware download, from the part of
//! `rtw89_mac_partial_init` (`mac.c:4307`) that runs before the PCI pre-init
//! and the download: `rtw89_mac_ctrl_hci_dma_trx(true)` (`mac.h:1642`) and
//! `rtw89_mac_dmac_pre_init` (`mac.c:4258`) with its AX helpers
//! `rtw89_mac_hci_func_en_ax` (`:4208`), `rtw89_mac_dmac_func_pre_en_ax`
//! (`:4222`), `rtw89_mac_dle_init(RTW89_QTA_DLFW)` (`:2274`, with
//! `dle_func_en_ax` `:2069`, `dle_clk_en_ax` `:2079`, `dle_mix_cfg_ax`
//! `:2092`, `chk_dle_rdy_ax` `:2149`, `wde_quota_cfg_ax` `:2178`,
//! `ple_quota_cfg_ax` `:2193`) and `rtw89_mac_hfc_init(reset, !en, h2c_en)`
//! (`:1194`) down its H2C-only path (`hfc_func_en_ax` `:1178`,
//! `hfc_h2c_cfg_ax` `:1127`). Register addresses are from pinned rtw89
//! `reg.h`.
//!
//! Only the RTL8852C, PCIe branches are ported. `pre_init` lays out DLFW;
//! `init` is `dmac_init_ax` (`mac.c:2521`): `dle_init(&SCC, None)` (the
//! `RTW89_QTA_SCC` layout, `rtw8852c.c:97`), `preload_init`
//! (`preload_init_set_ax`, `mac.c:2345`), `hfc_init` (the full `en && h2c_en`
//! path with the SCC `hfc_param`, `mac.c:1194`), then `sta_sch_init_ax`
//! (`:2425`), `mpdu_proc_init_ax` (`:2458`) and `sec_eng_init_ax` (`:2479`).
//! This module does not grant the BB pre-init, the PCI pre-init, the CMAC
//! init, or firmware readiness.
use super::io::{AxError, AxIo, clear32, field32, poll32, set32};

// reg.h / mac.h (pinned rtw89).
/// `R_AX_HCI_FUNC_EN_V1` (`reg.h:519`), the 8852C `hci_func_en_addr`
/// (`rtw8852c.c:3261`); `B_AX_HCI_TXDMA_EN`/`B_AX_HCI_RXDMA_EN` (`reg.h:548`).
const R_AX_HCI_FUNC_EN_V1: u32 = 0x7880;
const B_AX_HCI_TXDMA_EN: u32 = 1 << 0;
const B_AX_HCI_RXDMA_EN: u32 = 1 << 1;

const R_AX_DMAC_FUNC_EN: u32 = 0x8400;
const B_AX_DLE_PLE_EN: u32 = 1 << 23;
const B_AX_DLE_WDE_EN: u32 = 1 << 26;
const B_AX_MAC_FUNC_EN: u32 = 1 << 30;
const B_AX_DMAC_FUNC_EN: u32 = 1 << 29;
const B_AX_PKT_BUF_EN: u32 = 1 << 22;
const B_AX_DISPATCHER_EN: u32 = 1 << 18;
const B_AX_H_AXIDMA_EN: u32 = 1 << 14;
const R_AX_DMAC_CLK_EN: u32 = 0x8404;
const B_AX_DLE_PLE_CLK_EN: u32 = 1 << 23;
const B_AX_DLE_WDE_CLK_EN: u32 = 1 << 26;
const B_AX_DISPATCHER_CLK_EN: u32 = 1 << 18;

const R_AX_HAXI_INIT_CFG1: u32 = 0x1000;
/// `B_AX_DMA_MODE_MASK` GENMASK(19, 18); `DMA_MOD_PCIE_1B` is 0 (`reg.h:362`).
const B_AX_DMA_MODE_MASK: u32 = 0x3 << 18;
const B_AX_STOP_AXI_MST: u32 = 1 << 17;
const B_AX_RXHCI_EN_V1: u32 = 1 << 15;
const B_AX_TXHCI_EN_V1: u32 = 1 << 7;
const DMA_MOD_PCIE_1B: u32 = 0;
const R_AX_HAXI_DMA_STOP1: u32 = 0x1010;
/// `B_AX_STOP_ACH0..ACH7` (bits 8-15), `B_AX_STOP_CH8` (16), `CH9` (17) and
/// `CH12` (18). `B_AX_STOP_WPDMA` (19) is not in Linux's list.
const STOP1_BITS: u32 = 0x0007_FF00;
const R_AX_HAXI_DMA_STOP2: u32 = 0x11C0;
/// `B_AX_STOP_CH10 | B_AX_STOP_CH11`.
const STOP2_BITS: u32 = 0x3;
const R_AX_PLATFORM_ENABLE: u32 = 0x0088;
const B_AX_AXIDMA_EN: u32 = 1 << 3;

const R_AX_WDE_PKTBUF_CFG: u32 = 0x8C08;
const R_AX_PLE_PKTBUF_CFG: u32 = 0x9008;
const B_PKTBUF_PAGE_SEL_MASK: u32 = 0x3;
const B_PKTBUF_START_BOUND_MASK: u32 = 0x3F00;
const B_PKTBUF_FREE_PAGE_NUM_MASK: u32 = 0x1FFF_0000;
/// `S_AX_WDE_PAGE_SEL_64` / `S_AX_PLE_PAGE_SEL_128` (`mac.h:605-614`).
const PAGE_SEL_64: u32 = 0;
const PAGE_SEL_128: u32 = 1;
/// `DLE_BOUND_UNIT` (`mac.h:597`).
const DLE_BOUND_UNIT: u32 = 8 * 1024;
const R_AX_WDE_QTA0_CFG: u32 = 0x8C40;
const R_AX_WDE_QTA1_CFG: u32 = 0x8C44;
const R_AX_WDE_QTA3_CFG: u32 = 0x8C4C;
const R_AX_WDE_QTA4_CFG: u32 = 0x8C50;
const R_AX_PLE_QTA0_CFG: u32 = 0x9040;
/// `B_AX_{WDE,PLE}_MIN_SIZE_MASK` GENMASK(11, 0), `..._MAX_SIZE_MASK`
/// GENMASK(27, 16) (`reg.h:1373`, `:1577`).
const QTA_MIN_MASK: u32 = 0x0FFF;
const QTA_MAX_MASK: u32 = 0x0FFF_0000;
const R_AX_WDE_INI_STATUS: u32 = 0x8D00;
const R_AX_PLE_INI_STATUS: u32 = 0x9100;
/// `WDE_MGN_INI_RDY` / `PLE_MGN_INI_RDY`: bits 1 and 0.
const MGN_INI_RDY: u32 = 0x3;

const R_AX_HCI_FC_CTRL_V1: u32 = 0x1700;
const R_AX_CH_PAGE_CTRL_V1: u32 = 0x1704;
const B_AX_HCI_FC_EN: u32 = 1 << 0;
const B_AX_HCI_FC_CH12_EN: u32 = 1 << 3;
/// `B_AX_HCI_FC_CH12_FULL_COND_MASK` GENMASK(11, 10).
const B_AX_HCI_FC_CH12_FULL_COND_MASK: u32 = 0x3 << 10;
/// `B_AX_PREC_PAGE_CH12_MASK` GENMASK(24, 16).
const B_AX_PREC_PAGE_CH12_MASK: u32 = 0x1FF << 16;

/// `struct rtw89_dle_size` (`core.h:5151`): `{pge_size, lnk_pge_num, unlnk_pge_num}`.
struct DleSize {
    page: u32,
    linked: u32,
    unlinked: u32,
}

impl DleSize {
    /// The bytes of the pages this size lays out (`dle_used_size`, `mac.c:2041`).
    const fn bytes(&self) -> u32 {
        self.page * (self.linked + self.unlinked)
    }
}

/// `struct rtw89_dle_mem` (`core.h`) for one `rtw89_qta_mode`: the WDE and PLE
/// sizes and the WDE `{hif, wcpu, pkt_in, cpu_io}` and PLE `{cma0_tx, cma1_tx,
/// c2h, h2c, wcpu, mpdu_proc, cma0_dma, cma1_dma, bb_rpt, wd_rel, cpu_io,
/// tx_rpt}` quotas (`core.h:5167`). `tx_rpt` (QTA11) is written because the
/// chip is an RTL8852C.
struct DleMem {
    wde: DleSize,
    ple: DleSize,
    wde_min: [u32; 4],
    wde_max: [u32; 4],
    ple_min: [u32; 12],
    ple_max: [u32; 12],
}

/// `rtw8852c.c:3150`: `.fifo_size`, which `dle_used_size` (`mac.c:2041`)
/// must equal (`dle_expected_used_size`, `mac.c:2058`; `dle_scc_rsvd_size` is
/// 0, `rtw8852c.c:3152`, and neither table has `rsvd0_size`/`rsvd1_size`).
const FIFO_BYTES: u32 = 458_752;

/// `rtw8852c_dle_mem_pcie[RTW89_QTA_DLFW]` (`rtw8852c.c:101-104`): `wde_size18
/// = {RTW89_WDE_PG_64, 0, 2048}` (`mac.c:1747`), `ple_size18 =
/// {RTW89_PLE_PG_128, 2544, 16}` (`:1778`), `wde_qt17 = {0, 0, 0, 0}`
/// (`:1811`) for both min and max, `ple_qt44` (`:1857`, min) and `ple_qt45`
/// (`:1860`, max).
const DLFW: DleMem = DleMem {
    wde: DleSize {
        page: 64,
        linked: 0,
        unlinked: 2048,
    },
    ple: DleSize {
        page: 128,
        linked: 2544,
        unlinked: 16,
    },
    wde_min: [0; 4],
    wde_max: [0; 4],
    ple_min: [0, 0, 16, 256, 0, 0, 0, 0, 0, 0, 0, 0],
    ple_max: [0, 0, 32, 256, 0, 0, 0, 0, 0, 0, 0, 0],
};
const _: () = assert!(DLFW.wde.bytes() + DLFW.ple.bytes() == FIFO_BYTES);

/// `rtw8852c_dle_mem_pcie[RTW89_QTA_SCC]` (`rtw8852c.c:97-100`): `wde_size19 =
/// {RTW89_WDE_PG_64, 3328, 0}` (`mac.c:1750`), `ple_size19 =
/// {RTW89_PLE_PG_128, 1904, 16}` (`:1780`), `wde_qt18 = {3228, 60, 0, 40}`
/// (`:1813`) for both min and max, `ple_qt46` (`:1863`, min) and `ple_qt47`
/// (`:1865`, max).
const SCC: DleMem = DleMem {
    wde: DleSize {
        page: 64,
        linked: 3328,
        unlinked: 0,
    },
    ple: DleSize {
        page: 128,
        linked: 1904,
        unlinked: 16,
    },
    wde_min: [3228, 60, 0, 40],
    wde_max: [3228, 60, 0, 40],
    ple_min: [525, 0, 16, 20, 13, 13, 178, 0, 32, 62, 8, 16],
    ple_max: [525, 0, 32, 20, 1034, 13, 1199, 0, 1053, 62, 160, 1037],
};
const _: () = assert!(SCC.wde.bytes() + SCC.ple.bytes() == FIFO_BYTES);

/// `rtw89_mac_size.hfc_preccfg_pcie = {2, 40, 0, 0, 1, 0, 0, 0}`
/// (`mac.c:1717`) as `{ch011_prec, h2c_prec, wp_ch07_prec, wp_ch811_prec,
/// ch011_full_cond, h2c_full_cond, wp_ch07_full_cond, wp_ch811_full_cond}`.
/// `hfc_reset_param` (`mac.c:895`) reads the parameter entry of
/// `dle_info.qta_mode`, which is SCC once `rtw89_mac_dle_init(SCC)` has run;
/// DLFW and SCC share this `prec_cfg` (`rtw8852c.c:46-54`), so
/// `pre_init`'s H2C-only path is unchanged.
const HFC_CH011_PREC: u32 = 2;
const HFC_H2C_PREC_PAGES: u32 = 40;
const HFC_CH011_FULL_COND: u32 = 1;
const HFC_H2C_FULL_COND: u32 = 0;
/// `RTW89_HCIFC_POH`, `rtw8852c_hfc_param_ini_pcie[SCC].mode` (`core.h:5887`).
const HFC_MODE_POH: u32 = 0;
/// `rtw8852c_hfc_pub_cfg_pcie` (`rtw8852c.c:39-44`): `{grp0, grp1, pub_max,
/// wp_thrd}`.
const HFC_PUB_GRP0: u32 = 1614;
const HFC_PUB_GRP1: u32 = 1614;
const HFC_PUB_MAX: u32 = 3228;
const HFC_WP_THRD: u32 = 0;
/// `rtw8852c_hfc_ch_cfg_pcie` (`rtw8852c.c:23-37`) for DMA channels 0 to 11
/// (`RTW89_DMA_ACH0` to `RTW89_DMA_B1HI`) as `(min, max, grp)`: ACH4 to ACH7
/// and B1MGQ, B1HIQ are in group 1. Channel 12 (FWCMDQ) is `hfc_h2c_cfg_ax`'s.
const HFC_CH_CFG: [(u32, u32, u32); 12] = [
    (13, 1614, 0),
    (13, 1614, 0),
    (13, 1614, 0),
    (13, 1614, 0),
    (13, 1614, 1),
    (13, 1614, 1),
    (13, 1614, 1),
    (13, 1614, 1),
    (13, 1614, 0),
    (13, 1614, 0),
    (13, 1614, 1),
    (13, 1614, 1),
];
// `hfc_ch_cfg_chk` (`mac.c:926`) for the constant table: a non-zero `min` is
// not below `ch011_prec`, `max` is within `pub_max` and `grp` is 0 or 1
// (`grp_num`).
const _: () = {
    let mut ch = 0;
    while ch < HFC_CH_CFG.len() {
        let (min, max, grp) = HFC_CH_CFG[ch];
        assert!(min == 0 || min >= HFC_CH011_PREC);
        assert!(max <= HFC_PUB_MAX);
        assert!(grp < 2);
        ch += 1;
    }
};
// `hfc_pub_cfg_chk` (`mac.c:961`): the configured groups add up to `pub_max`.
const _: () = assert!(HFC_PUB_GRP0 + HFC_PUB_GRP1 == HFC_PUB_MAX);
/// `RTW89_DMA_CH_NUM`-bounded channel count `hfc_init` walks (`mac.c:1219`).
const HFC_CH_COUNT: u32 = HFC_CH_CFG.len() as u32;
// `chip->dma_ch_mask` of the 8852C is 0 (`rtw8852c.c:3288`), so `hfc_init`'s
// `dma_ch_mask & BIT(ch)` skip (`mac.c:1220`, `:1237`) never fires and the
// loops visit every channel.

// `rtw8852c_page_regs` (`rtw8852c.c:149-162`) and `reg.h` fields.
const R_AX_ACH0_PAGE_CTRL_V1: u32 = 0x1710;
const R_AX_ACH0_PAGE_INFO_V1: u32 = 0x1750;
const R_AX_PUB_PAGE_INFO3_V1: u32 = 0x178C;
const R_AX_PUB_PAGE_CTRL1_V1: u32 = 0x1790;
const R_AX_PUB_PAGE_CTRL2_V1: u32 = 0x1794;
const R_AX_PUB_PAGE_INFO1_V1: u32 = 0x1798;
const R_AX_PUB_PAGE_INFO2_V1: u32 = 0x179C;
const R_AX_WP_PAGE_CTRL1_V1: u32 = 0x17A0;
const R_AX_WP_PAGE_CTRL2_V1: u32 = 0x17A4;
const R_AX_WP_PAGE_INFO1_V1: u32 = 0x17A8;
/// `B_AX_MIN_PG_MASK` GENMASK(12, 0), `B_AX_MAX_PG_MASK` GENMASK(28, 16),
/// `B_AX_GRP` BIT(31).
const B_AX_MIN_PG_SHIFT: u32 = 0;
const B_AX_MAX_PG_SHIFT: u32 = 16;
const B_AX_GRP: u32 = 1 << 31;
/// `B_AX_PUBPG_G0_MASK` GENMASK(12, 0), `B_AX_PUBPG_G1_MASK` GENMASK(28, 16).
const B_AX_PUBPG_G1_SHIFT: u32 = 16;
/// `B_AX_PUBPG_ALL_MASK` GENMASK(12, 0), `B_AX_WP_THRD_MASK` GENMASK(12, 0).
const B_AX_13BIT_MASK: u32 = 0x1FFF;
/// `B_AX_G0_USE_PG_MASK` GENMASK(12, 0), `B_AX_G1_USE_PG_MASK`
/// GENMASK(28, 16), `B_AX_PUB_AVAL_PG_MASK` GENMASK(12, 0).
const B_AX_G1_USE_PG_SHIFT: u32 = 16;
/// `B_AX_HCI_FC_MODE_MASK` GENMASK(2, 1), `WD_FULL_COND` GENMASK(5, 4),
/// `WP_CH07_FULL_COND` GENMASK(7, 6), `WP_CH811_FULL_COND` GENMASK(9, 8).
const B_AX_HCI_FC_MODE_MASK: u32 = 0x3 << 1;
const B_AX_HCI_FC_WD_FULL_COND_MASK: u32 = 0x3 << 4;
const B_AX_HCI_FC_WP_CH07_FULL_COND_MASK: u32 = 0x3 << 6;
const B_AX_HCI_FC_WP_CH811_FULL_COND_MASK: u32 = 0x3 << 8;
/// `B_AX_PREC_PAGE_CH011_MASK` GENMASK(8, 0).
const B_AX_PREC_PAGE_CH011_MASK: u32 = 0x1FF;

// TX packet preload (`reg.h:1981-1994`).
const R_AX_TXPKTCTL_B0_PRELD_CFG0: u32 = 0x9F48;
const B_AX_B0_PRELD_FEN: u32 = 1 << 31;
/// `B_AX_B0_PRELD_USEMAXSZ_MASK` GENMASK(25, 16).
const B_AX_B0_PRELD_USEMAXSZ_MASK: u32 = 0x3FF << 16;
const PRELD_B0_ENT_NUM: u32 = 10;
const PRELD_AMSDU_SIZE: u32 = 52;
const R_AX_TXPKTCTL_B0_PRELD_CFG1: u32 = 0x9F4C;
/// `B_AX_B0_PRELD_NXT_TXENDWIN_MASK` GENMASK(11, 8),
/// `B_AX_B0_PRELD_NXT_RSVMINSZ_MASK` GENMASK(7, 0).
const B_AX_B0_PRELD_NXT_TXENDWIN_MASK: u32 = 0xF << 8;
const B_AX_B0_PRELD_NXT_RSVMINSZ_MASK: u32 = 0xFF;
const PRELD_NEXT_WND: u32 = 1;

// STA scheduler, MPDU processor, security engine (reg.h:1820-1907).
const R_AX_SS_CTRL: u32 = 0x9E10;
const B_AX_SS_EN: u8 = 1 << 0;
const B_AX_SS_NONEMPTY_SS2FINFO_EN: u32 = 1 << 28;
const B_AX_SS_WARM_INIT_FLG: u32 = 1 << 29;
const B_AX_SS_INIT_DONE_1: u32 = 1 << 31;
const R_AX_SS2FINFO_PATH: u32 = 0x9E50;
const B_AX_SS_DEST_QUEUE_MASK: u32 = 0x3F << 8;
const SS2F_PATH_WLCPU: u32 = 0x0A;
/// `TRXCFG_WAIT_CNT` (`mac.h:599`), microseconds.
const TRXCFG_WAIT_CNT: u64 = 2000;
const R_AX_MPDU_PROC: u32 = 0x9C00;
const B_AX_APPEND_FCS: u32 = 1 << 0;
const B_AX_A_ICV_ERR: u32 = 1 << 1;
const R_AX_ACTION_FWD0: u32 = 0x9C04;
const TRXCFG_MPDU_PROC_ACT_FRWD: u32 = 0x02A9_5A95;
const R_AX_TF_FWD: u32 = 0x9C14;
const TRXCFG_MPDU_PROC_TF_FRWD: u32 = 0x0000_AA55;
const R_AX_CUT_AMSDU_CTRL: u32 = 0x9C40;
const TRXCFG_MPDU_PROC_CUT_CTRL: u32 = 0x010E_05F0;
const R_AX_SEC_ENG_CTRL: u32 = 0x9D00;
const B_AX_SEC_TX_ENC: u32 = 1 << 0;
const B_AX_SEC_RX_DEC: u32 = 1 << 1;
const B_AX_BC_DEC: u32 = 1 << 2;
const B_AX_MC_DEC: u32 = 1 << 3;
const B_AX_UC_MGNT_DEC: u32 = 1 << 4;
const B_AX_CLK_EN_WEP_TKIP: u32 = 1 << 8;
const B_AX_CLK_EN_WAPI: u32 = 1 << 9;
const B_AX_CLK_EN_CGCMP: u32 = 1 << 10;
const R_AX_SEC_MPDU_PROC: u32 = 0x9D04;
const B_AX_APPEND_MIC: u32 = 1 << 0;
const B_AX_APPEND_ICV: u32 = 1 << 1;
const R_AX_SEC_DEBUG1: u32 = 0x9D1C;
const B_AX_TX_TIMEOUT_SEL_MASK: u32 = 0x3 << 30;
const AX_TX_TO_VAL: u32 = 0x2;

/// `rtw89_mac_check_mac_en_ax(RTW89_MAC_0, RTW89_DMAC_SEL)` (`mac.c:62`).
pub(super) fn check_dmac_en<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    let want = B_AX_MAC_FUNC_EN | B_AX_DMAC_FUNC_EN;
    let value = io.read32(R_AX_DMAC_FUNC_EN)?;
    // RTW89_R32_EA and RTW89_R32_DEAD (`core.h:51`).
    if value == 0xEAEA_EAEA || value == 0xDEAD_BEEF || value & want != want {
        return Err(AxError::Dma("DMAC function is not enabled"));
    }
    Ok(())
}

/// `rtw89_mac_hci_func_en_ax`, 8852C branch (`mac.c:4208`).
fn hci_func_en<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    io.write32(
        R_AX_DMAC_FUNC_EN,
        B_AX_MAC_FUNC_EN
            | B_AX_DMAC_FUNC_EN
            | B_AX_DISPATCHER_EN
            | B_AX_PKT_BUF_EN
            | B_AX_H_AXIDMA_EN,
    )
}

/// `rtw89_mac_dmac_func_pre_en_ax`, 8852C PCIe branch (`mac.c:4222`).
fn dmac_func_pre_en<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    io.write32(R_AX_DMAC_CLK_EN, B_AX_DISPATCHER_CLK_EN)?;

    let mut val = io.read32(R_AX_HAXI_INIT_CFG1)?;
    val &= !(B_AX_DMA_MODE_MASK | B_AX_STOP_AXI_MST);
    val |= B_AX_TXHCI_EN_V1 | B_AX_RXHCI_EN_V1;
    val |= (DMA_MOD_PCIE_1B << B_AX_DMA_MODE_MASK.trailing_zeros()) & B_AX_DMA_MODE_MASK;
    io.write32(R_AX_HAXI_INIT_CFG1, val)?;

    clear32(io, R_AX_HAXI_DMA_STOP1, STOP1_BITS)?;
    clear32(io, R_AX_HAXI_DMA_STOP2, STOP2_BITS)?;
    set32(io, R_AX_PLATFORM_ENABLE, B_AX_AXIDMA_EN)
}

/// `dle_func_en_ax` (`mac.c:2069`).
fn dle_func_en<I: AxIo + ?Sized>(io: &mut I, enable: bool) -> Result<(), AxError> {
    let bits = B_AX_DLE_WDE_EN | B_AX_DLE_PLE_EN;
    if enable {
        set32(io, R_AX_DMAC_FUNC_EN, bits)
    } else {
        clear32(io, R_AX_DMAC_FUNC_EN, bits)
    }
}

/// `dle_clk_en_ax(enable = true)` (`mac.c:2079`); the RTL8851B-only
/// `B_AX_AXIDMA_CLK_EN` is not added.
fn dle_clk_en<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set32(
        io,
        R_AX_DMAC_CLK_EN,
        B_AX_DLE_WDE_CLK_EN | B_AX_DLE_PLE_CLK_EN,
    )
}

/// `dle_mix_cfg_ax` (`mac.c:2092`).
fn dle_mix_cfg<I: AxIo + ?Sized>(io: &mut I, mem: &DleMem) -> Result<(), AxError> {
    let mut val = io.read32(R_AX_WDE_PKTBUF_CFG)?;
    val &= !(B_PKTBUF_PAGE_SEL_MASK | B_PKTBUF_START_BOUND_MASK | B_PKTBUF_FREE_PAGE_NUM_MASK);
    // WDE: page select 64 B, start bound 0, free pages = linked pages.
    val |= PAGE_SEL_64;
    val |= (mem.wde.linked << B_PKTBUF_FREE_PAGE_NUM_MASK.trailing_zeros())
        & B_PKTBUF_FREE_PAGE_NUM_MASK;
    io.write32(R_AX_WDE_PKTBUF_CFG, val)?;

    // PLE starts where WDE's pages end, in 8 KiB units.
    let bound = mem.wde.bytes() / DLE_BOUND_UNIT;
    let mut val = io.read32(R_AX_PLE_PKTBUF_CFG)?;
    val &= !(B_PKTBUF_PAGE_SEL_MASK | B_PKTBUF_START_BOUND_MASK | B_PKTBUF_FREE_PAGE_NUM_MASK);
    val |= PAGE_SEL_128;
    val |= (bound << B_PKTBUF_START_BOUND_MASK.trailing_zeros()) & B_PKTBUF_START_BOUND_MASK;
    val |= (mem.ple.linked << B_PKTBUF_FREE_PAGE_NUM_MASK.trailing_zeros())
        & B_PKTBUF_FREE_PAGE_NUM_MASK;
    io.write32(R_AX_PLE_PKTBUF_CFG, val)
}

fn qta_value(min: u32, max: u32) -> u32 {
    (min & QTA_MIN_MASK) | ((max << QTA_MAX_MASK.trailing_zeros()) & QTA_MAX_MASK)
}

/// `wde_quota_cfg_ax` (`mac.c:2178`): QTA0 (hif), QTA1 (wcpu, with the
/// extension minimum when there is one), QTA3 (pkt_in), QTA4 (cpu_io). QTA2
/// is not written.
fn wde_quota_cfg<I: AxIo + ?Sized>(
    io: &mut I,
    mem: &DleMem,
    ext_wcpu_min: Option<u32>,
) -> Result<(), AxError> {
    let [hif, wcpu, pkt_in, cpu_io] = mem.wde_min;
    let [max_hif, max_wcpu, max_pkt_in, max_cpu_io] = mem.wde_max;
    io.write32(R_AX_WDE_QTA0_CFG, qta_value(hif, max_hif))?;
    io.write32(
        R_AX_WDE_QTA1_CFG,
        qta_value(ext_wcpu_min.unwrap_or(wcpu), max_wcpu),
    )?;
    io.write32(R_AX_WDE_QTA3_CFG, qta_value(pkt_in, max_pkt_in))?;
    io.write32(R_AX_WDE_QTA4_CFG, qta_value(cpu_io, max_cpu_io))
}

/// `ple_quota_cfg_ax` (`mac.c:2193`): QTA0 to QTA11 in order.
fn ple_quota_cfg<I: AxIo + ?Sized>(io: &mut I, mem: &DleMem) -> Result<(), AxError> {
    for (i, (min, max)) in mem.ple_min.iter().zip(mem.ple_max).enumerate() {
        io.write32(R_AX_PLE_QTA0_CFG + 4 * i as u32, qta_value(*min, max))?;
    }
    Ok(())
}

/// `rtw89_mac_dle_init(mode, ext_mode)` (`mac.c:2274`) for the table `mem`.
/// `ext_wcpu_min` is the extension mode's WDE wcpu minimum: `Some(60)` when
/// DLFW is laid out ahead of SCC (`ext_cfg->wde_min_qt->wcpu`), `None` for
/// SCC itself (`INVALID_QT_WCPU`). The size check of `mac.c:2318` is the
/// `const` assert on each table.
fn dle_init<I: AxIo + ?Sized>(
    io: &mut I,
    mem: &DleMem,
    ext_wcpu_min: Option<u32>,
) -> Result<(), AxError> {
    check_dmac_en(io)?;
    dle_func_en(io, false)?;
    dle_clk_en(io)?;
    dle_mix_cfg(io, mem)?;
    wde_quota_cfg(io, mem, ext_wcpu_min)?;
    ple_quota_cfg(io, mem)?;
    dle_func_en(io, true)?;
    // chk_dle_rdy_ax (`mac.c:2149`): read_poll_timeout(.., 1, 2000), WDE
    // first, then PLE.
    poll32(io, R_AX_WDE_INI_STATUS, MGN_INI_RDY, MGN_INI_RDY, 1, 2000)?;
    poll32(io, R_AX_PLE_INI_STATUS, MGN_INI_RDY, MGN_INI_RDY, 1, 2000)
}

/// `preload_init_set_ax(RTW89_MAC_0, RTW89_QTA_SCC)` (`mac.c:2345`), reached
/// through `rtw89_mac_preload_init` (`:2373`), which runs it for a PCIe
/// RTL8852C (`is_qta_poh`).
pub(super) fn preload_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    field32(
        io,
        R_AX_TXPKTCTL_B0_PRELD_CFG0,
        B_AX_B0_PRELD_USEMAXSZ_MASK,
        PRELD_B0_ENT_NUM * PRELD_AMSDU_SIZE,
    )?;
    set32(io, R_AX_TXPKTCTL_B0_PRELD_CFG0, B_AX_B0_PRELD_FEN)?;
    field32(
        io,
        R_AX_TXPKTCTL_B0_PRELD_CFG1,
        B_AX_B0_PRELD_NXT_TXENDWIN_MASK,
        PRELD_NEXT_WND,
    )?;
    field32(
        io,
        R_AX_TXPKTCTL_B0_PRELD_CFG1,
        B_AX_B0_PRELD_NXT_RSVMINSZ_MASK,
        PRELD_AMSDU_SIZE,
    )
}

/// `hfc_func_en_ax` (`mac.c:1178`): set or clear `HCI_FC_EN` by `en` and
/// `HCI_FC_CH12_EN` by `h2c_en`.
fn hfc_func_en<I: AxIo + ?Sized>(io: &mut I, en: bool, h2c_en: bool) -> Result<(), AxError> {
    let mut val = io.read32(R_AX_HCI_FC_CTRL_V1)?;
    if en {
        val |= B_AX_HCI_FC_EN;
    } else {
        val &= !B_AX_HCI_FC_EN;
    }
    if h2c_en {
        val |= B_AX_HCI_FC_CH12_EN;
    } else {
        val &= !B_AX_HCI_FC_CH12_EN;
    }
    io.write32(R_AX_HCI_FC_CTRL_V1, val)
}

/// `rtw89_mac_hfc_init(rtwdev, true, false, true)` (`mac.c:1194`): only the
/// `!en && h2c_en` early return, served by `hfc_h2c_cfg_ax` (`mac.c:1127`).
/// `hfc_reset_param` does no register I/O.
fn hfc_init_h2c_only<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_dmac_en(io)?;
    hfc_func_en(io, false, false)?;
    io.write32(
        R_AX_CH_PAGE_CTRL_V1,
        (HFC_H2C_PREC_PAGES << B_AX_PREC_PAGE_CH12_MASK.trailing_zeros())
            & B_AX_PREC_PAGE_CH12_MASK,
    )?;
    field32(
        io,
        R_AX_HCI_FC_CTRL_V1,
        B_AX_HCI_FC_CH12_FULL_COND_MASK,
        HFC_H2C_FULL_COND,
    )?;
    hfc_func_en(io, false, true)
}

/// `hfc_ch_ctrl` (`mac.c:972`) for `ch` in 0..12. Its `hfc_ch_cfg_chk`
/// (`mac.c:926`) is the compile-time check on `HFC_CH_CFG`.
fn hfc_ch_ctrl<I: AxIo + ?Sized>(io: &mut I, ch: u32) -> Result<(), AxError> {
    check_dmac_en(io)?;
    let (min, max, grp) = HFC_CH_CFG[ch as usize];
    let val = (min << B_AX_MIN_PG_SHIFT)
        | (max << B_AX_MAX_PG_SHIFT)
        | if grp != 0 { B_AX_GRP } else { 0 };
    io.write32(R_AX_ACH0_PAGE_CTRL_V1 + ch * 4, val)
}

/// `hfc_upd_ch_info` (`mac.c:1000`) for `ch` in 0..12: the page info read is
/// the only register access, and nothing is kept from it.
fn hfc_upd_ch_info<I: AxIo + ?Sized>(io: &mut I, ch: u32) -> Result<(), AxError> {
    check_dmac_en(io)?;
    io.read32(R_AX_ACH0_PAGE_INFO_V1 + ch * 4).map(|_| ())
}

/// `hfc_pub_ctrl` (`mac.c:1027`); its `hfc_pub_cfg_chk` (`mac.c:961`) is the
/// compile-time check on the group sizes.
fn hfc_pub_ctrl<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_dmac_en(io)?;
    io.write32(
        R_AX_PUB_PAGE_CTRL1_V1,
        HFC_PUB_GRP0 | (HFC_PUB_GRP1 << B_AX_PUBPG_G1_SHIFT),
    )?;
    io.write32(R_AX_WP_PAGE_CTRL2_V1, HFC_WP_THRD & B_AX_13BIT_MASK)
}

/// `hfc_mix_cfg_ax` (`mac.c:1143`) with the SCC `hfc_param`. The two WP
/// precisions are 0 and so are their full conditions.
fn hfc_mix_cfg<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    io.write32(
        R_AX_CH_PAGE_CTRL_V1,
        (HFC_CH011_PREC & B_AX_PREC_PAGE_CH011_MASK)
            | ((HFC_H2C_PREC_PAGES << B_AX_PREC_PAGE_CH12_MASK.trailing_zeros())
                & B_AX_PREC_PAGE_CH12_MASK),
    )?;
    io.write32(R_AX_PUB_PAGE_CTRL2_V1, HFC_PUB_MAX & B_AX_13BIT_MASK)?;
    io.write32(R_AX_WP_PAGE_CTRL1_V1, 0)?;

    let mut val = io.read32(R_AX_HCI_FC_CTRL_V1)?;
    val &= !(B_AX_HCI_FC_MODE_MASK
        | B_AX_HCI_FC_WD_FULL_COND_MASK
        | B_AX_HCI_FC_CH12_FULL_COND_MASK
        | B_AX_HCI_FC_WP_CH07_FULL_COND_MASK
        | B_AX_HCI_FC_WP_CH811_FULL_COND_MASK);
    val |= (HFC_MODE_POH << B_AX_HCI_FC_MODE_MASK.trailing_zeros()) & B_AX_HCI_FC_MODE_MASK;
    val |= (HFC_CH011_FULL_COND << B_AX_HCI_FC_WD_FULL_COND_MASK.trailing_zeros())
        & B_AX_HCI_FC_WD_FULL_COND_MASK;
    val |= (HFC_H2C_FULL_COND << B_AX_HCI_FC_CH12_FULL_COND_MASK.trailing_zeros())
        & B_AX_HCI_FC_CH12_FULL_COND_MASK;
    io.write32(R_AX_HCI_FC_CTRL_V1, val)
}

/// `hfc_upd_mix_info` (`mac.c:1108`) over `hfc_get_mix_info_ax`
/// (`mac.c:1053`) and `hfc_pub_info_chk` (`mac.c:945`). Linux reads all ten
/// registers into `hfc_param` in this order; only the public page counts and
/// the `HCI_FC_EN` and `pub_max` read-backs are consulted afterwards.
fn hfc_upd_mix_info<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_dmac_en(io)?;
    let info1 = io.read32(R_AX_PUB_PAGE_INFO1_V1)?;
    io.read32(R_AX_PUB_PAGE_INFO3_V1)?;
    let info2 = io.read32(R_AX_PUB_PAGE_INFO2_V1)?;
    io.read32(R_AX_WP_PAGE_INFO1_V1)?;
    let fc_ctrl = io.read32(R_AX_HCI_FC_CTRL_V1)?;
    io.read32(R_AX_CH_PAGE_CTRL_V1)?;
    let pub_ctrl2 = io.read32(R_AX_PUB_PAGE_CTRL2_V1)?;
    io.read32(R_AX_WP_PAGE_CTRL1_V1)?;
    io.read32(R_AX_WP_PAGE_CTRL2_V1)?;
    io.read32(R_AX_PUB_PAGE_CTRL1_V1)?;

    let g0_used = info1 & B_AX_13BIT_MASK;
    let g1_used = (info1 >> B_AX_G1_USE_PG_SHIFT) & B_AX_13BIT_MASK;
    let pub_aval = info2 & B_AX_13BIT_MASK;
    let pub_max = pub_ctrl2 & B_AX_13BIT_MASK;
    // `param->en` is what the register now reads; the mismatch fails when set
    // (only the RTL8852A is excused, `mac.c:952`).
    if fc_ctrl & B_AX_HCI_FC_EN != 0 && g0_used + g1_used + pub_aval != pub_max {
        return Err(AxError::Mac("HCI flow control public pages do not add up"));
    }
    Ok(())
}

/// `rtw89_mac_hfc_init(rtwdev, true, true, true)` (`mac.c:1194`), the full
/// path of `dmac_init_ax` (`mac.c:2521`) for the SCC `hfc_param`.
pub(super) fn hfc_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_dmac_en(io)?;
    hfc_func_en(io, false, false)?;
    for ch in 0..HFC_CH_COUNT {
        hfc_ch_ctrl(io, ch)?;
    }
    hfc_pub_ctrl(io)?;
    hfc_mix_cfg(io)?;
    hfc_func_en(io, true, true)?;
    io.delay_us(10);
    for ch in 0..HFC_CH_COUNT {
        hfc_upd_ch_info(io, ch)?;
    }
    hfc_upd_mix_info(io)
}

/// `sta_sch_init_ax` (`mac.c:2425`), with `_patch_ss2f_path` (`:2414`), which
/// the RTL8852C does not skip.
fn sta_sch_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_dmac_en(io)?;
    let ctrl = io.read8(R_AX_SS_CTRL)?;
    io.write8(R_AX_SS_CTRL, ctrl | B_AX_SS_EN)?;
    poll32(
        io,
        R_AX_SS_CTRL,
        B_AX_SS_INIT_DONE_1,
        B_AX_SS_INIT_DONE_1,
        1,
        TRXCFG_WAIT_CNT,
    )?;
    set32(io, R_AX_SS_CTRL, B_AX_SS_WARM_INIT_FLG)?;
    clear32(io, R_AX_SS_CTRL, B_AX_SS_NONEMPTY_SS2FINFO_EN)?;
    field32(
        io,
        R_AX_SS2FINFO_PATH,
        B_AX_SS_DEST_QUEUE_MASK,
        SS2F_PATH_WLCPU,
    )
}

/// `mpdu_proc_init_ax` (`mac.c:2458`).
fn mpdu_proc_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_dmac_en(io)?;
    io.write32(R_AX_ACTION_FWD0, TRXCFG_MPDU_PROC_ACT_FRWD)?;
    io.write32(R_AX_TF_FWD, TRXCFG_MPDU_PROC_TF_FRWD)?;
    set32(io, R_AX_MPDU_PROC, B_AX_APPEND_FCS | B_AX_A_ICV_ERR)?;
    io.write32(R_AX_CUT_AMSDU_CTRL, TRXCFG_MPDU_PROC_CUT_CTRL)
}

/// `sec_eng_init_ax` (`mac.c:2479`), RTL8852C on PCIe: `UC_MGNT_DEC` is set
/// and `TX_PARTIAL_MODE` is left as found (cleared only for USB, `:2499`).
fn sec_eng_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    check_dmac_en(io)?;
    let ctrl = io.read32(R_AX_SEC_ENG_CTRL)?
        | B_AX_CLK_EN_CGCMP
        | B_AX_CLK_EN_WAPI
        | B_AX_CLK_EN_WEP_TKIP
        | B_AX_SEC_TX_ENC
        | B_AX_SEC_RX_DEC
        | B_AX_MC_DEC
        | B_AX_BC_DEC
        | B_AX_UC_MGNT_DEC;
    io.write32(R_AX_SEC_ENG_CTRL, ctrl)?;
    let mpdu = io.read32(R_AX_SEC_MPDU_PROC)? | B_AX_APPEND_ICV | B_AX_APPEND_MIC;
    io.write32(R_AX_SEC_MPDU_PROC, mpdu)?;
    field32(io, R_AX_SEC_DEBUG1, B_AX_TX_TIMEOUT_SEL_MASK, AX_TX_TO_VAL)
}

/// `dmac_init_ax(rtwdev, 0)` (`mac.c:2521`), RTL8852C on PCIe, with the SCC
/// layout.
pub fn init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    dle_init(io, &SCC, None)?;
    preload_init(io)?;
    hfc_init(io)?;
    sta_sch_init(io)?;
    mpdu_proc_init(io)?;
    sec_eng_init(io)
}

/// `rtw89_mac_ctrl_hci_dma_trx(true)` (`mac.h:1642`) then
/// `rtw89_mac_dmac_pre_init` (`mac.c:4258`): the MAC side of
/// `rtw89_mac_partial_init`, before the PCI pre-init and firmware download.
pub fn pre_init<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set32(
        io,
        R_AX_HCI_FUNC_EN_V1,
        B_AX_HCI_TXDMA_EN | B_AX_HCI_RXDMA_EN,
    )?;
    hci_func_en(io)?;
    dmac_func_pre_en(io)?;
    dle_init(io, &DLFW, Some(SCC.wde_min[1]))?;
    hfc_init_h2c_only(io)
}

#[cfg(test)]
mod tests;

// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2020  Realtek Corporation
//! RTL8852CE PCIe pre-init, DMA rings and the CH12 (FWCMD) TX queue, from
//! `rtw89_pci_ops_mac_pre_init_ax` (`pci.c:3070-3144`) and the helpers it
//! calls that execute for an RTL8852C (`pci.c:47`, `:249-275`,
//! `:2091-2118`, `:2551`, `:2709-2858`, `:2867-2951`, `:2953`),
//! `rtw89_pci_ops_reset` / `rtw89_pci_reset_trx_rings` /
//! `rtw89_pci_init_wp_16sel` (`pci.c:1722`, `:1776-1896`) with the 8852C
//! tables `rtw89_pci_ch_dma_addr_set_v1` (`pci.c:1116`) and
//! `rtw89_bd_ram_table_dual` (`pci.c:1693`), the ring pools of
//! `rtw89_pci_alloc_tx_rings` / `rtw89_pci_alloc_rx_ring` /
//! `rtw89_pci_init_rx_bd` (`pci.c:3512-3800`), and the FWCMD path of
//! `rtw89_pci_fwcmd_submit` (`pci.c:1551`), `rtw89_pci_tx_bd_ring_update` /
//! `__rtw89_pci_tx_kick_off` (`pci.c:1332-1358`) and the reclaim read of
//! `rtw89_pci_txbd_recalc` (`pci.c:87`). The post-init is
//! `rtw89_pci_ops_mac_post_init_ax` (`pci.c:3238`) with
//! `rtw89_pci_ltr_set_v1` (`pci.c:3188`) and `rtw89_pci_ctrl_txdma_ch_ax`
//! (`pci.c:249`), and its teardown `rtw89_pci_ops_deinit` (`pci.c:3053`);
//! the RXQ consumer is the index arithmetic and buffer
//! handover of `rtw89_pci_poll_rxq_dma` (`pci.c:436`),
//! `rtw89_pci_rxbd_recalc` / `rtw89_pci_dma_recalc` (`pci.c:146`, `:61`)
//! and `rtw89_pci_rxbd_deliver` (`pci.c:413-434`). The data (ACH0-3) and
//! management (CH8) queues are `rtw89_pci_txbd_submit` /
//! `rtw89_pci_txwd_submit` (`pci.c:1494-1636`) with
//! `rtw89_pci_fill_txaddr_info_v1` (`pci.c:1457-1491`),
//! `rtw89_pci_get_avail_txbd_num` (`pci.c:1222-1231`) and the WD page
//! recycling of `rtw89_pci_reclaim_txbd` / `rtw89_pci_release_txwd_skb`
//! (`pci.c:501-570`); RPQ is polled as RXQ is, and its release reports are
//! read as `rtw89_pci_release_tx_skbs` / `rtw89_pci_parse_rpp` /
//! `rtw89_pci_release_rpp` (`pci.c:572-625`, `:644-718`) read them, with
//! the qsel map of `rtw89_core_get_ch_dma` (`core.c:819-854`). Register
//! addresses are from pinned rtw89 `pci.h` and `reg.h`; the descriptor
//! layouts are `pci.h:1466-1514`.
//!
//! Only the RTL8852C branches are ported; the DMI-quirk BER fix
//! (`rtw89_pci_ber`, Dell only) and the 8852A/885xB PHY steps are not. This
//! module does not grant interrupts, NAPI, RX parsing (RXQ and RPQ buffers
//! are handed over raw, as far as the device wrote them; only [`releases`]
//! reads an RPQ buffer), TX on ACH4-7, CH9-11, ASPM/L1SS setup, the WD body
//! and info (the caller hands them over whole, the info always included),
//! or firmware readiness:
//! a consumed TXBD proves only that the DMA engine read the WD page, and a
//! release report only that the MAC finished with the frame.
use super::io::{AxError, AxIo, clear16, clear32, field32, poll32, poll32_atomic, set32};
use crate::bus::dma::{
    mapping::{Direction, DmaDomain, DmaError},
    memory::{MemoryError, PageSource},
};
use crate::wifi::realtek::beacon::{self, Allocation};
use alloc::{sync::Arc, vec::Vec};
use core::{
    cell::Cell,
    sync::atomic::{Ordering, fence},
};

/// Entries in each data ring, ACH0-ACH3: `RTW89_PCI_TXBD_NUM_MAX`
/// (`pci.h:1122`), the length `rtw89_pci_alloc_tx_rings` gives every ring
/// (`pci.c:3647-3650`), so a TID's A-MPDU and the frames behind it fit (M4
/// spec S9.2).
pub const TX_ENTRIES_DATA: usize = 256;
/// Entries in CH8 (management), CH12 (FWCMD) and the rings no frame goes
/// on (ACH4-7, CH9-11): 16, where Linux gives them 256 too. A software
/// length written to NUM, a multiple of 16 as `rtw89_pci_alloc_tx_rings`
/// requires (`BUILD_BUG_ON(RTW89_PCI_TXBD_NUM_MAX % 16)`, `pci.c:3647`);
/// management frames go one at a time, and each CH12 entry has a 4 KiB slot.
pub const TX_ENTRIES_CTRL: usize = 16;
/// RXQ's buffers: `RTW89_PCI_RXBD_NUM_MAX` (`pci.h:1123`), so one poll's
/// lag holds a 64-MPDU aggregate and its PPDU statuses (M4 spec S9.2).
pub const RX_ENTRIES_RXQ: usize = 256;
/// RPQ's buffers: 32, where Linux has 256 (M4 spec S9.2, a named
/// deviation): each RPQ buffer carries up to 2 800 four-byte release
/// reports, and [`Rings::rpq_full_batches`] counts a poll that found it
/// full.
pub const RX_ENTRIES_RPQ: usize = 32;
/// `RTW89_PCI_RX_BUF_SIZE` (`pci.h:1128`).
pub const RX_BUFFER_BYTES: usize = 11454 + 40 + 4;
/// Bytes of one CH12 packet slot. The largest FWDL packet is a 16-byte TX
/// descriptor plus 2020 section bytes; the header packet is smaller.
pub const FWCMD_SLOT_BYTES: usize = 4096;

/// `struct rtw89_pci_tx_bd_32` and `struct rtw89_pci_rx_bd_32` (`pci.h`).
const BD_BYTES: usize = 8;
const TX_CHANNELS: usize = 13;
const RX_CHANNELS: usize = 2;
/// TX ring `ring`'s entries: [`TX_ENTRIES_DATA`] for ACH0-ACH3,
/// [`TX_ENTRIES_CTRL`] for the rest.
pub(crate) const fn tx_entries(ring: usize) -> usize {
    if ring < 4 {
        TX_ENTRIES_DATA
    } else {
        TX_ENTRIES_CTRL
    }
}
/// Where each TX ring starts in the TX BD pool: the rings in channel order,
/// each `tx_entries` BDs long, as `rtw89_pci_alloc_tx_rings` lays them out
/// (`pci.c:3663-3676`); the last entry is the pool's length.
pub(crate) const TX_RING_AT: [usize; TX_CHANNELS + 1] = {
    let mut at = [0; TX_CHANNELS + 1];
    let mut ring = 0;
    while ring < TX_CHANNELS {
        at[ring + 1] = at[ring] + tx_entries(ring) * BD_BYTES;
        ring += 1;
    }
    at
};
const TX_POOL_BYTES: usize = TX_RING_AT[TX_CHANNELS];
/// RX ring `ring`'s buffers.
const fn rx_entries(ring: usize) -> usize {
    if ring == RXQ {
        RX_ENTRIES_RXQ
    } else {
        RX_ENTRIES_RPQ
    }
}
/// RX buffer `entry` of RX ring `ring`: RXQ's first, then RPQ's.
const fn rx_buffer(ring: usize, entry: usize) -> usize {
    if ring == RXQ {
        entry
    } else {
        RX_ENTRIES_RXQ + entry
    }
}
const RX_BUFFERS: usize = RX_ENTRIES_RXQ + RX_ENTRIES_RPQ;
/// Each RX BD ring starts on its own page of the RX pool, so every DESA is
/// page aligned whatever alignment the HAXI engine needs. (Linux's RPQ sits
/// 256 BDs = 2048 bytes into its pool.)
const RX_RING_STRIDE: usize = 4096;
const _: () = assert!(RX_ENTRIES_RXQ * BD_BYTES <= RX_RING_STRIDE);
const _: () = assert!(RX_ENTRIES_RPQ * BD_BYTES <= RX_RING_STRIDE);
/// `RTW89_TXCH_CH12`, the FWCMD channel.
const CH12: usize = 12;

const RTW89_PCI_TXBD_OPT_LS: u16 = 1 << 14;
/// `RTW89_PCI_TXBD_OPT_DMA_HI` / `RTW89_PCI_RXBD_OPT_DMA_HI`, GENMASK(13, 6).
const BD_OPT_DMA_HI_SHIFT: u32 = 6;
/// `TXBD_HW_IDX_MASK` (`pci.h:558`): the hardware read index of a TX ring.
const TXBD_HW_IDX_MASK: u32 = 0x0FFF_0000;
/// `__pci_flush_txch`'s reads of one ring (`pci.c:1401`).
const FLUSH_TRIES: usize = 60;

/// `enum rtw89_cv` (`core.h:201`).
const CHIP_CAV: u8 = 0;

// reg.h / pci.h (pinned rtw89).
const R_AX_SYS_SDIO_CTRL: u32 = 0x0070;
const B_AX_PCIE_DIS_L2_CTRL_LDO_HCI: u32 = 1 << 15;
const B_AX_PCIE_FORCE_PWR_NGAT: u32 = 1 << 13;
const R_AX_HCI_OPT_CTRL: u32 = 0x0074;
const BIT_WAKE_CTRL: u32 = 1 << 5;
const R_AX_HCI_BG_CTRL: u32 = 0x0078;
const R_AX_PMC_DBG_CTRL2: u32 = 0x00CC;
const B_AX_SYSON_DIS_PMCR_AX_WRMSK: u32 = 1 << 2;
const R_AX_PCIE_PS_CTRL_V1: u32 = 0x3008;
const B_AX_SEL_REQ_ENTR_L1: u32 = 1 << 2;
const B_AX_DMAC0_EXIT_L1_EN: u32 = 1 << 6;
const R_AX_PCIE_BG_CLR: u32 = 0x303C;
const B_AX_BG_CLR_ASYNC_M3: u32 = 1 << 4;
const R_RAC_DIRECT_OFFSET_G1: u32 = 0x3800;
const R_RAC_DIRECT_OFFSET_G2: u32 = 0x3880;
const RAC_ANA24: u32 = 0x24;
const B_AX_DEGLITCH: u16 = 0x0F00; // GENMASK(11, 8)
const R_AX_PCIE_IO_RCY_M1: u32 = 0x3100;
const R_AX_PCIE_WDT_TIMER_M1: u32 = 0x3104;
const R_AX_PCIE_IO_RCY_M2: u32 = 0x310C;
const R_AX_PCIE_WDT_TIMER_M2: u32 = 0x3110;
const R_AX_PCIE_IO_RCY_E0: u32 = 0x3118;
const R_AX_PCIE_WDT_TIMER_E0: u32 = 0x311C;
const R_AX_PCIE_IO_RCY_S1: u32 = 0x3124;
/// `B_AX_PCIE_IO_RCY_WDT_MODE_{M1,M2,E0,S1}` are all BIT(3).
const B_AX_PCIE_IO_RCY_WDT_MODE: u32 = 1 << 3;
/// `MAC_AX_IO_RCY_ANA_TMR_6MS` (`pci.h:1303`), `rtw8852c_pci_info.io_rcy_tmr`.
const IO_RCY_ANA_TMR_6MS: u32 = 72000;

const R_AX_HAXI_INIT_CFG1: u32 = 0x1000;
const B_AX_WD_ITVL_IDLE_V1_MASK: u32 = 0xF000_0000; // GENMASK(31, 28)
const B_AX_WD_ITVL_ACT_V1_MASK: u32 = 0x0F00_0000; // GENMASK(27, 24)
const B_AX_STOP_AXI_MST: u32 = 1 << 17;
const B_AX_RXHCI_EN_V1: u32 = 1 << 15;
const B_AX_RXBD_MODE_V1: u32 = 1 << 14;
const B_AX_HAXI_MAX_RXDMA_MASK: u32 = 0x0300; // GENMASK(9, 8)
const B_AX_TXHCI_EN_V1: u32 = 1 << 7;
const B_AX_RST_BDRAM: u32 = 1 << 3;
const B_AX_HAXI_MAX_TXDMA_MASK: u32 = 0x0003; // GENMASK(1, 0)
const R_AX_HAXI_DMA_STOP1: u32 = 0x1010;
const B_AX_STOP_PCIEIO: u32 = 1 << 20;
const B_AX_STOP_WPDMA: u32 = 1 << 19;
const B_AX_STOP_CH12: u32 = 1 << 18;
/// `B_AX_TX_STOP1_MASK`: ACH0-7, CH8, CH9 and CH12 (bits 8-18).
const B_AX_TX_STOP1_MASK: u32 = 0x0007_FF00;
const R_AX_TXBD_RWPTR_CLR1: u32 = 0x1014;
/// ACH0-3, CH8, CH9, CH12 and (8852A/8852C) ACH4-7: bits 0-10.
const B_AX_CLR_TXBD_IDX1: u32 = 0x07FF;
const R_AX_HAXI_DMA_BUSY1: u32 = 0x101C;
/// `DMA_BUSY1_CHECK`: ACH0-7, CH8, CH9 and CH12 busy (bits 8-18).
const DMA_BUSY1_CHECK: u32 = 0x0007_FF00;
const R_AX_HAXI_DMA_STOP2: u32 = 0x11C0;
/// `B_AX_TX_STOP2_ALL`: CH10, CH11.
const B_AX_TX_STOP2_ALL: u32 = 0x3;
const R_AX_TXBD_RWPTR_CLR2_V1: u32 = 0x11C4;
/// `B_AX_CLR_CH10_IDX | B_AX_CLR_CH11_IDX`.
const B_AX_CLR_TXBD_IDX2: u32 = 0x3;
const R_AX_HAXI_DMA_BUSY2: u32 = 0x11C8;
/// `B_AX_CH10_BUSY | B_AX_CH11_BUSY`.
const DMA_BUSY2_CHECK: u32 = 0x3;
const R_AX_RXBD_RWPTR_CLR_V1: u32 = 0x1200;
/// `B_AX_CLR_RXQ_IDX | B_AX_CLR_RPQ_IDX`.
const B_AX_CLR_RXBD_IDX: u32 = 0x3;
const R_AX_HAXI_EXP_CTRL: u32 = 0x1204;
const B_AX_MAX_TAG_NUM_V1_MASK: u32 = 0x7; // GENMASK(2, 0)
const R_AX_HAXI_DMA_BUSY3: u32 = 0x1208;
/// `B_AX_RXQ_BUSY | B_AX_RPQ_BUSY`.
const DMA_BUSY3_CHECK: u32 = 0x3;
const R_AX_WP_ADDR_H_SEL0_3: u32 = 0x1334;
const R_AX_TX_ADDRESS_INFO_MODE_SETTING: u32 = 0x8810;
const B_AX_HOST_ADDR_INFO_8B_SEL: u32 = 1 << 0;
const R_AX_PKTIN_SETTING: u32 = 0x9A00;
const B_AX_WD_ADDR_INFO_LENGTH: u32 = 1 << 1;

// LTR (reg.h:437-456, :594-619), as rtw89_pci_ltr_set_v1 uses them.
const R_AX_LTR_DEC_CTRL: u32 = 0x1600;
const B_AX_LTR_REQ_DRV: u32 = 1 << 7;
const B_AX_LTR_IDX_DRV_MASK: u32 = 0x60; // GENMASK(6, 5)
/// `PCIE_LTR_IDX_IDLE`.
const PCIE_LTR_IDX_IDLE: u32 = 3;
const B_AX_LTR_DRV_DEC_EN: u32 = 1 << 4;
const B_AX_LTR_FW_DEC_EN: u32 = 1 << 3;
const B_AX_LTR_HW_DEC_EN: u32 = 1 << 2;
/// `LTR_EN_BITS`: the HW, FW and driver decision enables.
const LTR_EN_BITS: u32 = B_AX_LTR_HW_DEC_EN | B_AX_LTR_FW_DEC_EN | B_AX_LTR_DRV_DEC_EN;
const B_AX_LTR_SPACE_IDX_V1_MASK: u32 = 0x3; // GENMASK(1, 0)
const R_AX_LTR_LATENCY_IDX0: u32 = 0x1604;
const R_AX_LTR_LATENCY_IDX3: u32 = 0x1610;
const R_AX_LTR_CTRL_0: u32 = 0x8410;
const B_AX_LTR_IDLE_TIMER_IDX_MASK: u32 = 0x0700; // GENMASK(10, 8)
const B_AX_LTR_WD_NOEMP_CHK_V1: u32 = 1 << 1;
const B_AX_LTR_HW_EN: u32 = 1 << 0;
const R_AX_LTR_CTRL_1: u32 = 0x8414;
const B_AX_LTR_RX1_TH_MASK: u32 = 0x0FFF_0000; // GENMASK(27, 16)
const B_AX_LTR_RX0_TH_MASK: u32 = 0x0FFF; // GENMASK(11, 0)
/// `PCI_LTR_SPC_500US`.
const LTR_SPC_500US: u32 = 2;
/// `PCI_LTR_IDLE_TIMER_3_2MS`.
const LTR_IDLE_TIMER_3_2MS: u32 = 7;
/// The RX0 and RX1 thresholds `rtw89_pci_ltr_set_v1` writes.
const LTR_RX_TH: u32 = 0x28;
const LTR_IDX3_LATENCY: u32 = 0x9003_9003;
const LTR_IDX0_LATENCY: u32 = 0x880B_880B;

/// RXQ, ring 0 of `RX_TABLE`.
const RXQ: usize = 0;
/// RPQ, ring 1 of `RX_TABLE`.
const RPQ: usize = 1;

// rtw8852c_pci_info (rtw8852ce.c:21-79) mode values (pci.h enums).
/// `MAC_AX_TX_BURST_V1_256B`.
const TX_BURST_V1_256B: u32 = 2;
/// `MAC_AX_RX_BURST_V1_128B`.
const RX_BURST_V1_128B: u32 = 1;
/// `MAC_AX_TAG_NUM_8`.
const TAG_NUM_8: u32 = 7;
/// `MAC_AX_WD_DMA_INTVL_256NS`, both idle and active.
const WD_DMA_INTVL_256NS: u32 = 1;

/// `read_poll_timeout(..., 10, 100, ...)` in the DMA idle polls.
const IDLE_POLL_INTERVAL_US: u64 = 10;
const IDLE_POLL_TIMEOUT_US: u64 = 100;
/// `read_poll_timeout_atomic(..., 1, RTW89_PCI_POLL_BDRAM_RST_CNT, ...)`.
const BDRAM_POLL_TIMEOUT_US: u64 = 100;
/// The CH12 reclaim wait: 1 us samples for up to the 400 ms FWDL budget.
const RECLAIM_INTERVAL_US: u64 = 1;
const RECLAIM_TIMEOUT_US: u64 = 400_000;

/// One TX channel's registers (`rtw89_pci_ch_dma_addr_set_v1`) and BD RAM
/// split (`rtw89_bd_ram_table_dual`).
struct TxRing {
    num: u32,
    idx: u32,
    bdram: u32,
    desa_l: u32,
    start_idx: u32,
    max_num: u32,
    min_num: u32,
}

const fn tx(num: u32, idx: u32, bdram: u32, desa_l: u32, bd_ram: [u32; 3]) -> TxRing {
    TxRing {
        num,
        idx,
        bdram,
        desa_l,
        start_idx: bd_ram[0],
        max_num: bd_ram[1],
        min_num: bd_ram[2],
    }
}

/// `RTW89_TXCH_ACH0` .. `RTW89_TXCH_CH12`, in the donor's loop order.
/// `DEF_TXCHADDRS(info, X, _V1)` keeps the plain NUM/IDX and takes the `_V1`
/// BDRAM/DESA; `DEF_TXCHADDRS_TYPE1` (CH10, CH11) takes `_V1` for all.
const TX_TABLE: [TxRing; TX_CHANNELS] = [
    tx(0x1024, 0x1058, 0x1300, 0x1230, [0, 5, 2]),  // ACH0
    tx(0x1026, 0x105C, 0x1304, 0x1238, [5, 5, 2]),  // ACH1
    tx(0x1028, 0x1060, 0x1308, 0x1240, [10, 5, 2]), // ACH2
    tx(0x102A, 0x1064, 0x130C, 0x1248, [15, 5, 2]), // ACH3
    tx(0x102C, 0x1068, 0x1310, 0x1250, [20, 5, 2]), // ACH4
    tx(0x102E, 0x106C, 0x1314, 0x1258, [25, 5, 2]), // ACH5
    tx(0x1030, 0x1070, 0x1318, 0x1260, [30, 5, 2]), // ACH6
    tx(0x1032, 0x1074, 0x131C, 0x1268, [35, 5, 2]), // ACH7
    tx(0x1034, 0x1078, 0x1320, 0x1270, [40, 5, 1]), // CH8
    tx(0x1036, 0x107C, 0x1324, 0x1278, [45, 5, 1]), // CH9
    tx(0x1438, 0x11D0, 0x1420, 0x1458, [50, 5, 1]), // CH10
    tx(0x143A, 0x11D4, 0x1424, 0x1460, [55, 5, 1]), // CH11
    tx(0x1038, 0x1080, 0x1328, 0x1280, [60, 4, 1]), // CH12 (FWCMD)
];

/// `DEF_RXCHADDRS(AX, RXQ/RPQ, ..., _V1)`: NUM, IDX, DESA_L.
const RX_TABLE: [(u32, u32, u32); RX_CHANNELS] = [
    (0x1210, 0x1218, 0x1220), // RXQ
    (0x1212, 0x121C, 0x1228), // RPQ
];

/// The 8-byte `struct rtw89_pci_tx_bd_32` of `rtw89_pci_fwcmd_submit`:
/// le16 length, le16 opt = LS | dma`[39:32]` << 6, le32 dma`[31:0]`.
pub fn encode_txbd(len: u16, dma: u64) -> [u8; 8] {
    encode_bd(len, RTW89_PCI_TXBD_OPT_LS, dma)
}

/// `struct rtw89_pci_rx_bd_32` as `rtw89_pci_init_rx_bd` fills it.
fn encode_rxbd(buf_size: u16, dma: u64) -> [u8; 8] {
    encode_bd(buf_size, 0, dma)
}

fn encode_bd(len: u16, flags: u16, dma: u64) -> [u8; 8] {
    let high = ((dma >> 32) & 0xFF) as u16;
    let opt = flags | high << BD_OPT_DMA_HI_SHIFT;
    let mut bd = [0; BD_BYTES];
    bd[0..2].copy_from_slice(&len.to_le_bytes());
    bd[2..4].copy_from_slice(&opt.to_le_bytes());
    bd[4..8].copy_from_slice(&(dma as u32).to_le_bytes());
    bd
}

impl From<beacon::Error> for AxError {
    fn from(error: beacon::Error) -> Self {
        match error {
            beacon::Error::Memory(MemoryError::OutOfMemory) => Self::Dma("out of ring memory"),
            beacon::Error::Memory(_) => Self::Dma("ring memory unavailable"),
            beacon::Error::Dma(error) => error.into(),
            beacon::Error::Address => Self::Dma("ring address above 4 GiB"),
            _ => Self::Dma("ring allocation failed"),
        }
    }
}

impl From<DmaError> for AxError {
    fn from(error: DmaError) -> Self {
        Self::Dma(match error {
            DmaError::AddressLimit => "ring memory above the device's address limit",
            DmaError::VisibilityFailed => "ring memory visibility failed",
            DmaError::Unmapped => "ring memory is unmapped",
            _ => "ring memory mapping failed",
        })
    }
}

/// The DMA memory behind every rtw89 ring: one pool of 13 TX BD rings, one
/// pool of the RXQ and RPQ BD rings with their buffers, and the CH12 packet
/// slots. All of it lies below 4 GiB.
///
/// Once `pre_init` has written a ring address to the device, the memory is
/// never returned: dropping the rings then leaves every mapping in
/// `DmaMapping`'s quarantine, because a drop cannot prove the DMA stopped.
pub struct Rings {
    tx: Allocation,
    rx: Allocation,
    rx_buffers: Vec<Allocation>,
    fwcmd: Allocation,
    /// A ring address may have reached the device.
    programmed: Cell<bool>,
    /// `bd_ring->wp` of CH12.
    ch12_host: Cell<u16>,
    /// CH12 is started by `pre_init` and has no entry in flight.
    ch12_ready: Cell<bool>,
    /// The data and management rings were started by `pre_init` and not
    /// since cleared by `deinit` or a later `pre_init`.
    data_ready: Cell<bool>,
    /// `bd_ring->wp` of each [`TxChannel`]: the next entry the host fills.
    data_host: [Cell<u16>; DATA_CHANNELS],
    /// The host index each [`TxChannel`]'s last kick wrote: entries from it up
    /// to `data_host` are written and not yet handed over.
    data_kicked: [Cell<u16>; DATA_CHANNELS],
    /// `bd_ring->rp` of each [`TxChannel`]: the hardware index last read.
    /// Entries from it up to the host index are the device's.
    data_hw: [Cell<u16>; DATA_CHANNELS],
    /// `bd_ring->wp` of RXQ and RPQ: the next entry the host reads. Entries
    /// from it up to the hardware index are filled and the host's; the rest
    /// are the device's.
    rx_host: [Cell<u16>; RX_CHANNELS],
    /// RXQ polls that found the ring full, saturating; see
    /// [`Rings::rxq_full_batches`].
    rxq_full: Cell<u16>,
    /// RPQ polls that found the ring full, saturating; see
    /// [`Rings::rpq_full_batches`].
    rpq_full: Cell<u16>,
}

impl Rings {
    /// Allocate and fill every ring as `rtw89_pci_alloc_trx_rings` does:
    /// zeroed TX BDs, and each RX BD pointing at a zeroed buffer of
    /// `RX_BUFFER_BYTES`. Nothing reaches the device until `pre_init`.
    ///
    /// # Safety
    /// `domain` isolates exactly the RTL8852C PCI requester whose register
    /// window these rings will be given to by `pre_init`. `source` yields
    /// real coherent WB DMA RAM for that requester, with NoSnoop and ATS
    /// disabled (WB alone is not coherence proof). One worker owns the
    /// device and these rings; no other engine, IRQ or reset worker touches
    /// them.
    pub unsafe fn install(
        source: &dyn PageSource,
        domain: Arc<dyn DmaDomain>,
    ) -> Result<Self, AxError> {
        let tx = Allocation::new(
            source,
            domain.clone(),
            TX_POOL_BYTES,
            Direction::Bidirectional,
        )?;
        let rx = Allocation::new(
            source,
            domain.clone(),
            RX_CHANNELS * RX_RING_STRIDE,
            Direction::Bidirectional,
        )?;
        let mut rx_buffers = Vec::new();
        rx_buffers
            .try_reserve_exact(RX_BUFFERS)
            .map_err(|_| AxError::Dma("out of ring memory"))?;
        for _ in 0..RX_BUFFERS {
            rx_buffers.push(Allocation::new(
                source,
                domain.clone(),
                RX_BUFFER_BYTES,
                Direction::FromDevice,
            )?);
        }
        let fwcmd = Allocation::new(
            source,
            domain,
            TX_ENTRIES_CTRL * FWCMD_SLOT_BYTES,
            Direction::ToDevice,
        )?;
        let rings = Self {
            tx,
            rx,
            rx_buffers,
            fwcmd,
            programmed: Cell::new(false),
            ch12_host: Cell::new(0),
            ch12_ready: Cell::new(false),
            data_ready: Cell::new(false),
            data_host: Default::default(),
            data_kicked: Default::default(),
            data_hw: Default::default(),
            rx_host: Default::default(),
            rxq_full: Cell::new(0),
            rpq_full: Cell::new(0),
        };
        zero(&rings.tx, TX_POOL_BYTES)?;
        rings.tx.dma.sync_for_device(0..TX_POOL_BYTES)?;
        zero(&rings.rx, RX_CHANNELS * RX_RING_STRIDE)?;
        // RXQ's buffers come first, then RPQ's (`rx_buffer`); each one's BD is
        // its entry of the ring that starts at page r of the pool.
        for (i, buffer) in rings.rx_buffers.iter().enumerate() {
            // `memset(skb->data, 0, buf_sz)` before `dma_map_single`.
            zero(buffer, RX_BUFFER_BYTES)?;
            buffer.dma.sync_for_device(0..RX_BUFFER_BYTES)?;
            let bd = encode_rxbd(RX_BUFFER_BYTES as u16, u64::from(buffer.address()?));
            let (ring, entry) = if i < RX_ENTRIES_RXQ {
                (RXQ, i)
            } else {
                (RPQ, i - RX_ENTRIES_RXQ)
            };
            let at = ring * RX_RING_STRIDE + entry * BD_BYTES;
            rings.rx.write(at, &bd)?;
        }
        rings
            .rx
            .dma
            .sync_for_device(0..RX_CHANNELS * RX_RING_STRIDE)?;
        Ok(rings)
    }

    /// The firmware command queue. `pre_init` must have succeeded first.
    pub fn ch12(&mut self) -> Ch12<'_> {
        Ch12 { rings: self }
    }

    /// The RXQ ring, read by polling. `pre_init` must have succeeded first.
    pub fn rxq(&mut self) -> Rxq<'_> {
        Rxq {
            rings: self,
            copy: Vec::new(),
            hw: 0,
        }
    }

    /// The data or management queue `ch`, with its WD pages and payload
    /// slots from `pools`. It refuses frames until `pre_init` has succeeded.
    pub fn txq<'r>(&'r mut self, pools: &'r mut TxPools, ch: TxChannel) -> Txq<'r> {
        Txq {
            rings: self,
            pools,
            ch,
        }
    }

    /// `rtw89_pci_ops_flush_queues` (`pci.c:1432-1436`): for every TX ring
    /// but CH12, which `__rtw89_pci_ops_flush_txchs` skips (`pci.c:1414-1430`;
    /// the 8852CE masks no other, `tx_dma_ch_mask` 0, `rtw8852ce.c:63`), in
    /// ring order, `__pci_flush_txch` (`pci.c:1388-1410`): up to 60 reads of
    /// the ring's IDX, each not yet showing the hardware index (`[27:16]`) at
    /// the host's (`bd_ring->wp`: a data or management channel's, 0 for a
    /// ring nothing is queued on) followed by a 1 us delay. A ring still
    /// behind after them is passed over, as Linux only logs it. Nothing is
    /// written and no TXBD is taken back; only a failed read is an error.
    pub fn flush_tx<I: AxIo + ?Sized>(&self, io: &mut I) -> Result<(), AxError> {
        for (ring, entry) in TX_TABLE.iter().enumerate().take(CH12) {
            let wp = match ring {
                0..=3 => self.data_host[ring].get(),
                8 => self.data_host[TxChannel::Ch8.slot()].get(),
                _ => 0,
            };
            for _ in 0..FLUSH_TRIES {
                let value = io.read32(entry.idx)?;
                let rp = (value & TXBD_HW_IDX_MASK) >> TXBD_HW_IDX_MASK.trailing_zeros();
                if rp == u32::from(wp) {
                    break;
                }
                io.delay_us(1);
            }
        }
        Ok(())
    }

    /// The RPQ ring, read by polling as RXQ is. `pre_init` must have
    /// succeeded first.
    pub fn rpq(&mut self) -> Rpq<'_> {
        Rpq {
            rings: self,
            copy: Vec::new(),
        }
    }

    /// CH12 refuses packets: `pre_init` has not succeeded yet, or a submit
    /// failed after handing its packet over. Only another `pre_init` starts
    /// it again.
    pub fn ch12_stopped(&self) -> bool {
        !self.ch12_ready.get()
    }

    /// How many `Rxq::poll` batches found RXQ full: `RX_ENTRIES_RXQ - 1` entries
    /// filled, the most the ring holds with `rx_ring_eq_is_full` false
    /// (`rtw89_pci_dma_recalc`, `pci.c:61`). Each such batch means the device
    /// may have had no free buffer before the poll. Saturates at `u16::MAX`.
    pub fn rxq_full_batches(&self) -> u16 {
        self.rxq_full.get()
    }

    /// How many `Rpq::poll` batches found RPQ full, as
    /// [`Rings::rxq_full_batches`] counts RXQ's (M4 spec S9.2, S9.4): the
    /// device may have had no buffer for its release reports. Saturates at
    /// `u16::MAX`.
    pub fn rpq_full_batches(&self) -> u16 {
        self.rpq_full.get()
    }

    fn tx_ring(&self, channel: usize) -> Result<u32, AxError> {
        Ok(self.tx.address()? + TX_RING_AT[channel] as u32)
    }

    fn rx_ring(&self, channel: usize) -> Result<u32, AxError> {
        Ok(self.rx.address()? + (channel * RX_RING_STRIDE) as u32)
    }
}

impl Drop for Rings {
    fn drop(&mut self) {
        if self.programmed.get() {
            // The device may still reach any ring: keep every mapping.
            self.tx.published = true;
            self.rx.published = true;
            self.fwcmd.published = true;
            for buffer in &mut self.rx_buffers {
                buffer.published = true;
            }
        }
    }
}

fn zero(allocation: &Allocation, bytes: usize) -> Result<(), AxError> {
    const CHUNK: [u8; 512] = [0; 512];
    let mut at = 0;
    while at < bytes {
        let n = CHUNK.len().min(bytes - at);
        allocation.write(at, &CHUNK[..n])?;
        at += n;
    }
    Ok(())
}

/// `rtw89_pci_ops_mac_pre_init_ax` for an RTL8852C of chip cut `cut`
/// (`enum rtw89_cv`, 0 = CAV): PCIe PHY and power settings, stop and drain
/// all DMA, clear every ring index, set the DMA mode, program every ring of
/// `rings`, reset the BD RAM and start DMA with only CH12 enabled.
///
/// Runs after the MAC's DMAC pre-init, as `rtw89_mac_partial_init` orders
/// them; nothing here assumes it runs first. A failure leaves CH12 and the
/// data and management queues stopped. Every host index restarts as soon as
/// `clr_idx_all` has restarted the hardware's, so a failure after it leaves
/// RXQ and RPQ with nothing to deliver.
///
/// # Safety
/// `io` is the exclusive register window of exactly the RTL8852C requester
/// that `rings`' DMA domain isolates, and `rings` was installed on real
/// coherent WB RAM for it with NoSnoop and ATS disabled. This publishes
/// every ring address to that device and starts its DMA (RX included).
/// One worker owns the device and the queues: no other engine, IRQ or reset
/// worker touches them.
pub unsafe fn pre_init<I: AxIo + ?Sized>(
    io: &mut I,
    cut: u8,
    rings: &Rings,
) -> Result<(), AxError> {
    rings.ch12_ready.set(false);
    rings.data_ready.set(false);
    // rtw89_pci_ber: only on the Dell RTW89_QUIRK_PCI_BER DMI models.
    // rtw89_pci_rxdma_prefth, _l1off_pwroff, _l2_rxen_lat, _aphy_pwrcut,
    // _dphy_delay, _autok_x, _auto_refclk_cal: 8852A/885xB only.
    // rtw89_pci_deglitch_setting (pci.c:2551), RTL8852C branch.
    clear16(io, R_RAC_DIRECT_OFFSET_G1 + RAC_ANA24 * 2, B_AX_DEGLITCH)?;
    clear16(io, R_RAC_DIRECT_OFFSET_G2 + RAC_ANA24 * 2, B_AX_DEGLITCH)?;
    // rtw89_pci_hci_ldo (pci.c:2709), RTL8852C branch.
    clear32(io, R_AX_SYS_SDIO_CTRL, B_AX_PCIE_DIS_L2_CTRL_LDO_HCI)?;
    // rtw89_pci_power_wake_ax(true) (pci.c:2733).
    set32(io, R_AX_HCI_OPT_CTRL, BIT_WAKE_CTRL)?;
    // rtw89_pci_autoload_hang (pci.c:2741).
    set32(io, R_AX_PCIE_BG_CLR, B_AX_BG_CLR_ASYNC_M3)?;
    clear32(io, R_AX_PCIE_BG_CLR, B_AX_BG_CLR_ASYNC_M3)?;
    if cut == CHIP_CAV {
        // rtw89_pci_l12_vmain (pci.c:2750).
        set32(io, R_AX_SYS_SDIO_CTRL, B_AX_PCIE_FORCE_PWR_NGAT)?;
        // rtw89_pci_gen2_force_ib (pci.c:2758).
        set32(io, R_AX_PMC_DBG_CTRL2, B_AX_SYSON_DIS_PMCR_AX_WRMSK)?;
        set32(io, R_AX_HCI_BG_CTRL, B_AX_BG_CLR_ASYNC_M3)?;
        clear32(io, R_AX_PMC_DBG_CTRL2, B_AX_SYSON_DIS_PMCR_AX_WRMSK)?;
    }
    // rtw89_pci_l1_ent_lat (pci.c:2770) and rtw89_pci_wd_exit_l1 (:2778).
    clear32(io, R_AX_PCIE_PS_CTRL_V1, B_AX_SEL_REQ_ENTR_L1)?;
    set32(io, R_AX_PCIE_PS_CTRL_V1, B_AX_DMAC0_EXIT_L1_EN)?;
    // rtw89_pci_set_sic, _set_lbc, _set_dbg, _set_keep_reg skip 8852C.
    // rtw89_pci_set_io_rcy (pci.c:2814), io_rcy_en enabled.
    for timer in [
        R_AX_PCIE_WDT_TIMER_M1,
        R_AX_PCIE_WDT_TIMER_M2,
        R_AX_PCIE_WDT_TIMER_E0,
    ] {
        io.write32(timer, IO_RCY_ANA_TMR_6MS)?;
    }
    for mode in [
        R_AX_PCIE_IO_RCY_M1,
        R_AX_PCIE_IO_RCY_M2,
        R_AX_PCIE_IO_RCY_E0,
    ] {
        set32(io, mode, B_AX_PCIE_IO_RCY_WDT_MODE)?;
    }
    clear32(io, R_AX_PCIE_IO_RCY_S1, B_AX_PCIE_IO_RCY_WDT_MODE)?;

    // pci.c:3113-3123.
    stop_dma(io)?;
    poll_dma_all_idle(io)?;
    clr_idx_all(io)?;
    // The host indices restart with the hardware's, before any later step
    // can fail: an RX host index left where it was would read the zeroed
    // hardware index as a ring of filled (stale) entries.
    restart_host_indices(rings);
    mode_op(io)?;
    // rtw89_pci_ops_reset: the ring registers.
    reset_trx_rings(io, rings)?;
    // rtw89_pci_rst_bdram_ax (pci.c:47).
    rst_bdram(io)?;
    // rtw89_pci_ctrl_txdma_ch_ax(false), then _fw_ch_ax(true) (pci.c:249-275).
    set32(io, R_AX_HAXI_DMA_STOP1, B_AX_TX_STOP1_MASK)?;
    set32(io, R_AX_HAXI_DMA_STOP2, B_AX_TX_STOP2_ALL)?;
    clear32(io, R_AX_HAXI_DMA_STOP1, B_AX_STOP_CH12)?;
    ctrl_dma_all(io, true)?;
    rings.ch12_ready.set(true);
    rings.data_ready.set(true);
    Ok(())
}

/// `rtw89_pci_ops_mac_post_init_ax` (`pci.c:3238-3268`) for an RTL8852C:
/// `rtw89_pci_ltr_set_v1(true)` when `ltr`, then the TX DMA of every
/// channel enabled (`rtw89_pci_ctrl_txdma_ch_ax(true)`) and PCI IO released
/// (`B_AX_STOP_WPDMA`, which `pre_init` left set, and `B_AX_STOP_PCIEIO`).
///
/// `ltr` is the function's PCIe DEVCTL2 `LTR_EN`, which Linux reads at this
/// point (`rtw89_pci_dev_ltr_enabled`, `pci.c:3042`); when it is false no
/// LTR register is read or written. An LTR register that reads an error
/// value refuses the step before anything is written. The 8852A LTR trigger
/// and the 8852A/885xB address-info mode (`pci.c:3249-3258`) do not apply;
/// `pre_init`'s `mode_op` already set the 8852C's.
///
/// Every TX ring but CH12 holds only zeroed TXBDs with its host index at 0,
/// so releasing their DMA hands the device nothing to fetch.
pub fn post_init<I: AxIo + ?Sized>(io: &mut I, ltr: bool) -> Result<(), AxError> {
    if ltr {
        ltr_set_v1(io, true)?;
    }
    // rtw89_pci_ctrl_txdma_ch_ax(true) (pci.c:249).
    clear32(io, R_AX_HAXI_DMA_STOP1, B_AX_TX_STOP1_MASK)?;
    clear32(io, R_AX_HAXI_DMA_STOP2, B_AX_TX_STOP2_ALL)?;
    // Release PCI IO (pci.c:3263-3265).
    clear32(io, R_AX_HAXI_DMA_STOP1, B_AX_STOP_WPDMA | B_AX_STOP_PCIEIO)
}

/// `rtw89_pci_ops_deinit` (`pci.c:3053-3068`) for an RTL8852C, which
/// `rtw89_core_stop` runs between `rtw89_hci_stop` and `rtw89_mac_pwr_off`
/// (`core.c:6765-6767`): `rtw89_pci_power_wake_ax(false)` (`pci.c:2733`,
/// `BIT_WAKE_CTRL` cleared), `rtw89_pci_ltr_set_v1(false)` when `ltr`, the
/// AXI master stopped and the TX/RX HCI DMA disabled
/// (`rtw89_pci_ctrl_dma_all(false)`), then every ring index cleared
/// (`rtw89_pci_clr_idx_all_ax`). The 8852A LTR trigger does not apply.
///
/// Linux's writes cannot fail and it ignores `ltr_set`'s `-EINVAL`, so every
/// step runs whatever an earlier one returned; the first error is returned.
/// The hardware indices restart, so CH12 and the data and management queues
/// are marked stopped, and once every index clear succeeded the RXQ and RPQ
/// host indices restart at 0: only another `pre_init` starts the rings
/// again. A clear that fails stops the ones after it, the RX clear last, so
/// the RX host indices stay with the hardware's they still match. This
/// relies on a failed clear never reaching the device: on the real MMIO
/// window the range check runs before any access and every ring register is
/// in range, so `clr_idx_all` fails only in a test model, never after a
/// write has landed.
pub fn deinit<I: AxIo + ?Sized>(io: &mut I, rings: &Rings, ltr: bool) -> Result<(), AxError> {
    rings.ch12_ready.set(false);
    rings.data_ready.set(false);
    let wake = clear32(io, R_AX_HCI_OPT_CTRL, BIT_WAKE_CTRL);
    let ltr = if ltr { ltr_set_v1(io, false) } else { Ok(()) };
    let dma = ctrl_dma_all(io, false);
    let indices = clr_idx_all(io);
    if indices.is_ok() {
        for host in &rings.rx_host {
            host.set(0);
        }
    }
    wake.and(ltr).and(dma).and(indices)
}

/// `rtw89_pci_ltr_set_v1(rtwdev, en)` (`pci.c:3188-3235`) once
/// `rtw89_pci_dev_ltr_enabled` held: read and check all five LTR registers,
/// then program them. Disabling clears `LTR_EN_BITS` and selects the
/// driver's idle index with `LTR_REQ_DRV`, and leaves `LTR_CTRL_0`'s enables
/// as they are.
fn ltr_set_v1<I: AxIo + ?Sized>(io: &mut I, en: bool) -> Result<(), AxError> {
    // rtw89_pci_ltr_is_err_reg_val (pci.h:1731).
    let checked = |io: &mut I, reg: u32| -> Result<u32, AxError> {
        let value = io.read32(reg)?;
        if value == u32::MAX || value == 0xEAEA_EAEA {
            return Err(AxError::Mac("LTR register reads an error value"));
        }
        Ok(value)
    };
    checked(io, R_AX_LTR_CTRL_0)?;
    checked(io, R_AX_LTR_CTRL_1)?;
    let mut dec_ctrl = checked(io, R_AX_LTR_DEC_CTRL)?;
    checked(io, R_AX_LTR_LATENCY_IDX3)?;
    checked(io, R_AX_LTR_LATENCY_IDX0)?;
    if en {
        dec_ctrl |= B_AX_LTR_HW_DEC_EN;
    } else {
        dec_ctrl &= !(LTR_EN_BITS | B_AX_LTR_IDX_DRV_MASK | B_AX_LTR_HW_DEC_EN);
        dec_ctrl |= PCIE_LTR_IDX_IDLE << B_AX_LTR_IDX_DRV_MASK.trailing_zeros() | B_AX_LTR_REQ_DRV;
    }
    // B_AX_LTR_SPACE_IDX_V1_MASK starts at bit 0.
    dec_ctrl = (dec_ctrl & !B_AX_LTR_SPACE_IDX_V1_MASK) | LTR_SPC_500US;
    if en {
        set32(
            io,
            R_AX_LTR_CTRL_0,
            B_AX_LTR_WD_NOEMP_CHK_V1 | B_AX_LTR_HW_EN,
        )?;
    }
    field32(
        io,
        R_AX_LTR_CTRL_0,
        B_AX_LTR_IDLE_TIMER_IDX_MASK,
        LTR_IDLE_TIMER_3_2MS,
    )?;
    field32(io, R_AX_LTR_CTRL_1, B_AX_LTR_RX0_TH_MASK, LTR_RX_TH)?;
    field32(io, R_AX_LTR_CTRL_1, B_AX_LTR_RX1_TH_MASK, LTR_RX_TH)?;
    io.write32(R_AX_LTR_DEC_CTRL, dec_ctrl)?;
    io.write32(R_AX_LTR_LATENCY_IDX3, LTR_IDX3_LATENCY)?;
    io.write32(R_AX_LTR_LATENCY_IDX0, LTR_IDX0_LATENCY)
}

/// Stop the PCIe side's DMA as `rtw89_pci_ops_mac_pre_init_ax` does first
/// (`pci.c:3113-3116`): `B_AX_STOP_WPDMA` in `R_AX_HAXI_DMA_STOP1`, then
/// `rtw89_pci_ctrl_dma_all(false)`: AXI master stopped and the TX/RX HCI
/// DMA disabled. Nothing is waited for.
pub(crate) fn stop_dma<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set32(io, R_AX_HAXI_DMA_STOP1, B_AX_STOP_WPDMA)?;
    ctrl_dma_all(io, false)
}

/// `rtw89_pci_ctrl_dma_all` (`pci.c:2114`): `dma_io_stop`, then the
/// RX/TX HCI enables of `init_cfg_reg`.
fn ctrl_dma_all<I: AxIo + ?Sized>(io: &mut I, enable: bool) -> Result<(), AxError> {
    let hci = B_AX_RXHCI_EN_V1 | B_AX_TXHCI_EN_V1;
    if enable {
        clear32(io, R_AX_HAXI_INIT_CFG1, B_AX_STOP_AXI_MST)?;
        set32(io, R_AX_HAXI_INIT_CFG1, hci)
    } else {
        set32(io, R_AX_HAXI_INIT_CFG1, B_AX_STOP_AXI_MST)?;
        clear32(io, R_AX_HAXI_INIT_CFG1, hci)
    }
}

/// `rtw89_pci_poll_dma_all_idle` (`pci.c:2934`): TX busy1, TX busy2 (CH10,
/// CH11), then RX busy3.
fn poll_dma_all_idle<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    for (reg, mask) in [
        (R_AX_HAXI_DMA_BUSY1, DMA_BUSY1_CHECK),
        (R_AX_HAXI_DMA_BUSY2, DMA_BUSY2_CHECK),
        (R_AX_HAXI_DMA_BUSY3, DMA_BUSY3_CHECK),
    ] {
        poll32(
            io,
            reg,
            mask,
            0,
            IDLE_POLL_INTERVAL_US,
            IDLE_POLL_TIMEOUT_US,
        )?;
    }
    Ok(())
}

/// `rtw89_pci_rst_bdram_ax` (`pci.c:47-54`): set the reset bit, then
/// `read_poll_timeout_atomic(.., 1, 100, false)` for it to clear.
fn rst_bdram<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set32(io, R_AX_HAXI_INIT_CFG1, B_AX_RST_BDRAM)?;
    poll32_atomic(
        io,
        R_AX_HAXI_INIT_CFG1,
        B_AX_RST_BDRAM,
        0,
        1,
        BDRAM_POLL_TIMEOUT_US,
    )
}

/// `rtw89_pci_clr_idx_all_ax` (`pci.c:2867`), RTL8852C channel set.
fn clr_idx_all<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    set32(io, R_AX_TXBD_RWPTR_CLR1, B_AX_CLR_TXBD_IDX1)?;
    set32(io, R_AX_TXBD_RWPTR_CLR2_V1, B_AX_CLR_TXBD_IDX2)?;
    set32(io, R_AX_RXBD_RWPTR_CLR_V1, B_AX_CLR_RXBD_IDX)
}

/// `rtw89_pci_mode_op` (`pci.c:2953`) for `rtw8852c_pci_info`: BD trunc
/// mode (no 8852C register), RXBD packet mode, burst sizes, tag count, WD
/// DMA intervals, then the trunc-mode address-info settings.
fn mode_op<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    clear32(io, R_AX_HAXI_INIT_CFG1, B_AX_RXBD_MODE_V1)?;
    field32(
        io,
        R_AX_HAXI_INIT_CFG1,
        B_AX_HAXI_MAX_TXDMA_MASK,
        TX_BURST_V1_256B,
    )?;
    field32(
        io,
        R_AX_HAXI_INIT_CFG1,
        B_AX_HAXI_MAX_RXDMA_MASK,
        RX_BURST_V1_128B,
    )?;
    field32(io, R_AX_HAXI_EXP_CTRL, B_AX_MAX_TAG_NUM_V1_MASK, TAG_NUM_8)?;
    field32(
        io,
        R_AX_HAXI_INIT_CFG1,
        B_AX_WD_ITVL_IDLE_V1_MASK,
        WD_DMA_INTVL_256NS,
    )?;
    field32(
        io,
        R_AX_HAXI_INIT_CFG1,
        B_AX_WD_ITVL_ACT_V1_MASK,
        WD_DMA_INTVL_256NS,
    )?;
    set32(
        io,
        R_AX_TX_ADDRESS_INFO_MODE_SETTING,
        B_AX_HOST_ADDR_INFO_8B_SEL,
    )?;
    clear32(io, R_AX_PKTIN_SETTING, B_AX_WD_ADDR_INFO_LENGTH)
}

/// The software indices of `rtw89_pci_reset_trx_rings` (`pci.c:1776`),
/// which `pre_init` restarts as soon as `clr_idx_all` has restarted the
/// hardware's.
fn restart_host_indices(rings: &Rings) {
    rings.ch12_host.set(0);
    // Every TX `bd_ring->wp` and `rp` restarts at 0 (`pci.c:1804-1805`). A
    // WD page still in flight stays out of its pool: no BD names it now, so
    // it is never handed out again.
    for ((host, hw), kicked) in rings
        .data_host
        .iter()
        .zip(&rings.data_hw)
        .zip(&rings.data_kicked)
    {
        host.set(0);
        hw.set(0);
        kicked.set(0);
    }
    // RX `bd_ring->wp` restarts at 0, unwritten (`rx_ring_eq_is_full`
    // false), as `clr_idx_all` restarted the hardware's.
    for host in &rings.rx_host {
        host.set(0);
    }
}

/// `rtw89_pci_reset_trx_rings` (`pci.c:1776`) without `group_bd_addr`, with
/// `rx_ring_eq_is_full` false, then `rtw89_pci_init_wp_16sel`. Its software
/// indices are `restart_host_indices`.
fn reset_trx_rings<I: AxIo + ?Sized>(io: &mut I, rings: &Rings) -> Result<(), AxError> {
    let mut tx = [0; TX_CHANNELS];
    for (channel, address) in tx.iter_mut().enumerate() {
        *address = rings.tx_ring(channel)?;
    }
    let rx = [rings.rx_ring(0)?, rings.rx_ring(1)?];
    // The first ring address store publishes the rings.
    rings.programmed.set(true);
    for (ring, (channel, address)) in TX_TABLE.iter().zip(tx).enumerate() {
        io.write16(channel.num, tx_entries(ring) as u16)?;
        let bd_ram = channel.start_idx | channel.max_num << 8 | channel.min_num << 16;
        io.write32(channel.bdram, bd_ram)?;
        io.write32(channel.desa_l, address)?;
        io.write32(channel.desa_l + 4, 0)?;
    }
    for (ring, (&(num, _idx, desa_l), address)) in RX_TABLE.iter().zip(rx).enumerate() {
        io.write16(num, rx_entries(ring) as u16)?;
        io.write32(desa_l, address)?;
        io.write32(desa_l + 4, 0)?;
    }
    for i in (0..16u32).step_by(4) {
        let value = i | (i + 1) << 8 | (i + 2) << 16 | (i + 3) << 24;
        io.write32(R_AX_WP_ADDR_H_SEL0_3 + i, value)?;
    }
    Ok(())
}

/// The CH12 (FWCMD) TX queue: one packet in flight at a time.
pub struct Ch12<'r> {
    rings: &'r mut Rings,
}

impl Ch12<'_> {
    /// Send `packet` (its TX descriptor included) as `rtw89_pci_fwcmd_submit`
    /// and `__rtw89_pci_tx_kick_off` do, then wait until the hardware read
    /// index (`IDX` `[27:16]`) has passed the entry (1 us samples, 400 ms).
    ///
    /// Any failure after the length check (the slot or TXBD write, a sync,
    /// the kick or the reclaim wait) leaves the queue refusing further
    /// packets until the next successful `pre_init`: it is marked stopped
    /// before the slot is written.
    ///
    /// # Safety
    /// `io` is the exclusive register window of the same RTL8852C requester
    /// that `pre_init` programmed with these rings, and the `pre_init`
    /// contract still holds: its DMA domain isolates that requester, the
    /// rings are coherent WB RAM with NoSnoop and ATS disabled, and one
    /// worker owns the device and this queue.
    pub unsafe fn submit<I: AxIo + ?Sized>(
        &mut self,
        io: &mut I,
        packet: &[u8],
    ) -> Result<(), AxError> {
        let rings = &mut *self.rings;
        if !rings.ch12_ready.get() {
            return Err(AxError::Dma("CH12 is stopped"));
        }
        if packet.is_empty() || packet.len() > FWCMD_SLOT_BYTES {
            return Err(AxError::Dma("CH12 packet length out of range"));
        }
        let slot = usize::from(rings.ch12_host.get());
        let at = slot * FWCMD_SLOT_BYTES;
        let dma = u64::from(rings.fwcmd.address()?) + at as u64;
        let bd = TX_RING_AT[CH12] + slot * BD_BYTES;
        rings.ch12_ready.set(false);
        rings.fwcmd.write(at, packet)?;
        rings.fwcmd.dma.sync_for_device(at..at + packet.len())?;
        rings.tx.write(bd, &encode_txbd(packet.len() as u16, dma))?;
        rings.tx.dma.sync_for_device(bd..bd + BD_BYTES)?;
        fence(Ordering::Release);
        // rtw89_pci_tx_bd_ring_update: advance and wrap the host index.
        let host = ((slot + 1) % TX_ENTRIES_CTRL) as u16;
        rings.ch12_host.set(host);
        let idx = TX_TABLE[CH12].idx;
        io.write16(idx, host)?;
        poll32(
            io,
            idx,
            TXBD_HW_IDX_MASK,
            u32::from(host) << 16,
            RECLAIM_INTERVAL_US,
            RECLAIM_TIMEOUT_US,
        )?;
        fence(Ordering::Acquire);
        // SAFETY: the hardware read index has passed this entry, so the DMA
        // engine has finished reading the TXBD, and it cannot read the slot
        // again until the host index wraps back to it on a later submit.
        unsafe { rings.tx.dma.sync_for_cpu(bd..bd + BD_BYTES) }?;
        rings.ch12_ready.set(true);
        Ok(())
    }
}

/// WD pages, and payload slots, per data channel (ACH0-ACH3): one per TXBD,
/// as a frame holds one of each until both come back (M3 spec S5.1), so the
/// pages never run out before the BDs. Linux keeps `RTW89_PCI_TXWD_NUM_MAX`
/// (512, `pci.h:1124`; M4 spec S9.2, a named deviation). CH8 has one per
/// TXBD of its own ring, [`TX_ENTRIES_CTRL`].
pub const WD_PAGES: usize = TX_ENTRIES_DATA;
/// `RTW89_PCI_TXWD_PAGE_SIZE` (`pci.h:1125`).
pub const WD_PAGE_BYTES: usize = 128;
/// Bytes of one payload slot.
pub const PAYLOAD_BYTES: usize = 2048;
/// `TXADDR_INFO_LENTHG_V1_MAX` (`pci.h:1501`): the most one TX address
/// entry carries. Every frame here takes exactly one.
pub const MAX_TX_FRAME: usize = 2044;
/// The WD body v1 (32 bytes) and WD info (24 bytes) a frame's caller builds.
/// The info is required: wp_info always follows at byte 56, where Linux puts
/// it at 32 for a frame without the info (`pci.c:1524-1527`), so
/// [`Txq::submit`] refuses a head whose body dword 0 lacks WD_INFO_EN.
/// Linux sets it on every management and data frame (`core.c:925`, `:1149`).
pub const WD_HEAD_BYTES: usize = 56;
/// `RTW89_TX_DONE` (`core.h:4676`): the frame was acknowledged.
pub const TX_DONE: u8 = 0;

/// `struct rtw89_pci_tx_wp_info` (`pci.h:1476`).
const WP_INFO_BYTES: usize = 8;
/// `struct rtw89_pci_tx_addr_info_32_v1` (`pci.h:1496`).
const ADDR_INFO_BYTES: usize = 6;
/// `txwd->len`, the TXBD length: head, wp_info and one addr_info (70).
const WD_BYTES: usize = WD_HEAD_BYTES + WP_INFO_BYTES + ADDR_INFO_BYTES;
/// ACH0-3 and CH8.
const DATA_CHANNELS: usize = 5;
/// Pool slot `slot`'s pages: one per TXBD of its ring.
const fn pages_of(slot: usize) -> usize {
    if slot < 4 { WD_PAGES } else { TX_ENTRIES_CTRL }
}
/// Each pool slot's first page among all of them; the last entry is their
/// number.
const PAGE_AT: [usize; DATA_CHANNELS + 1] = {
    let mut at = [0; DATA_CHANNELS + 1];
    let mut slot = 0;
    while slot < DATA_CHANNELS {
        at[slot + 1] = at[slot] + pages_of(slot);
        slot += 1;
    }
    at
};
/// The pool holds every channel's WD pages, then every payload slot.
const WD_REGION: usize = PAGE_AT[DATA_CHANNELS] * WD_PAGE_BYTES;
const POOL_BYTES: usize = WD_REGION + PAGE_AT[DATA_CHANNELS] * PAYLOAD_BYTES;
// A page's seq is a u8 in its token, and a TXBD's page is too.
const _: () = assert!(WD_PAGES == TX_ENTRIES_DATA && WD_PAGES <= u8::MAX as usize + 1);
const _: () = assert!(WD_BYTES <= WD_PAGE_BYTES && MAX_TX_FRAME <= PAYLOAD_BYTES);
/// `RTW89_PCI_TXWP_VALID` (`pci.h:1474`).
const TXWP_VALID: u16 = 1 << 15;
/// `RTW89_TXWD_BODY0_WD_INFO_EN` (`txrx.h:72`), in body dword 0.
const WD_INFO_EN: u32 = 1 << 22;
/// `B_PCIADDR_LS_V1_MASK` (`pci.h:1500`).
const PCIADDR_LS_V1: u16 = 1 << 15;

// The RPQ buffer: `struct rtw89_pci_rxbd_info` (`pci.h:1536-1538`), the AX
// RX descriptor's first dword (`txrx.h:351-352`) and the RPP dword
// (`pci.h:1507-1509`).
const RXBD_FS: u32 = 1 << 15;
const RXBD_LS: u32 = 1 << 14;
const RXBD_WRITE_SIZE: u32 = 0x3FFF;
const RXBD_INFO_BYTES: usize = 4;
const RXD_LONG: u32 = 1 << 31;
const RXD_DRV_INFO_SHIFT: u32 = 28;
const RXD_DRV_INFO: u32 = 0x7;
const RXD_SHORT_BYTES: usize = 16;
const RXD_LONG_BYTES: usize = 32;
const RPP_BYTES: usize = 4;
const RPP_SEQ_SHIFT: u32 = 16;
const RPP_SEQ: u32 = 0x7FFF;
const RPP_TX_STATUS_SHIFT: u32 = 13;
const RPP_TX_STATUS: u32 = 0x7;
const RPP_QSEL_SHIFT: u32 = 8;
const RPP_QSEL: u32 = 0x1F;

/// The TX DMA channels that carry frames: data by access category, and
/// management on band 0 (`RTW89_TXCH_ACH0`-`ACH3`, `RTW89_TXCH_CH8`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxChannel {
    Ach0,
    Ach1,
    Ach2,
    Ach3,
    Ch8,
}

impl TxChannel {
    /// The channel `rtw89_core_get_ch_dma` (`core.c:819-854`) gives `qsel`:
    /// BE_0 (0) → ACH0, BK_0 (1) → ACH1, VI_0 (2) → ACH2, VO_0 (3) → ACH3,
    /// B0_MGMT (0x12) → CH8 (`txrx.h:798-817`). Every other qsel is `None`:
    /// no frame here is queued with one, and where Linux falls through to
    /// ACH0 for an unknown qsel, a report naming one returns no page.
    pub const fn of_qsel(qsel: u8) -> Option<Self> {
        match qsel {
            0x00 => Some(Self::Ach0),
            0x01 => Some(Self::Ach1),
            0x02 => Some(Self::Ach2),
            0x03 => Some(Self::Ach3),
            0x12 => Some(Self::Ch8),
            _ => None,
        }
    }

    /// `enum rtw89_tx_channel`: the TX ring's index (0-3, 8).
    pub const fn ring(self) -> usize {
        match self {
            Self::Ach0 => 0,
            Self::Ach1 => 1,
            Self::Ach2 => 2,
            Self::Ach3 => 3,
            Self::Ch8 => 8,
        }
    }

    /// The channel's place among the pools and the data ring indices, and in
    /// [`super::station::chip::TxCounts`]' arrays.
    pub const fn slot(self) -> usize {
        match self {
            Self::Ach0 => 0,
            Self::Ach1 => 1,
            Self::Ach2 => 2,
            Self::Ach3 => 3,
            Self::Ch8 => 4,
        }
    }
}

/// One WD page and its payload slot. Free when not `in_flight`; once
/// submitted it returns only when the TXBD hardware index has passed its
/// entry (`bd_done`) and its release report has arrived (`reported`), in
/// either order, as `rtw89_pci_reclaim_txbd` and
/// `rtw89_pci_release_txwd_skb` (`pci.c:501-570`) return a `txwd`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Page {
    in_flight: bool,
    bd_done: bool,
    reported: bool,
}

/// One channel's pages, and the page each TXBD entry names. CH8 uses the
/// first [`TX_ENTRIES_CTRL`] of each.
#[derive(Clone, Copy)]
struct Pool {
    /// Page `i` has seq `i` (`txwd->seq = i`, `pci.c:3581`).
    pages: [Page; WD_PAGES],
    /// `bd[e]` is the page TXBD entry `e` was last given.
    bd: [u8; TX_ENTRIES_DATA],
}

impl Pool {
    const EMPTY: Self = Self {
        pages: [Page {
            in_flight: false,
            bd_done: false,
            reported: false,
        }; WD_PAGES],
        bd: [0; TX_ENTRIES_DATA],
    };
}

/// The WD pages and payload slots of ACH0-3 and CH8: one page of 128 bytes
/// (`rtw89_pci_alloc_tx_wd_ring`, `pci.c:3539`) and one slot of 2048 bytes
/// per TXBD of the channel's ring, 256 each for ACH0-3 and 16 for CH8, in
/// one DMA allocation of 2 263 040 bytes (553 pages) below 4 GiB: every WD
/// page, then every payload slot.
///
/// Once a TXBD naming the pool has been kicked, the memory is never
/// returned: dropping the pools then leaves the mapping in `DmaMapping`'s
/// quarantine, because a drop cannot prove the DMA stopped.
pub struct TxPools {
    memory: Allocation,
    channels: [Pool; DATA_CHANNELS],
}

impl TxPools {
    /// Allocate the pools. Nothing reaches the device until a
    /// [`Txq::submit`]; every page is zeroed before each use.
    ///
    /// # Safety
    /// The [`Rings::install`] contract, for the same requester and domain
    /// as the rings these pools will be used with.
    pub unsafe fn install(
        source: &dyn PageSource,
        domain: Arc<dyn DmaDomain>,
    ) -> Result<Self, AxError> {
        let memory = Allocation::new(source, domain, POOL_BYTES, Direction::ToDevice)?;
        Ok(Self {
            memory,
            channels: [Pool::EMPTY; DATA_CHANNELS],
        })
    }

    /// Take the release report for page `seq` of channel `ch`, as
    /// `rtw89_pci_release_rpp` (`pci.c:596-625`) does. The page returns to
    /// its pool only if its TXBD entry has also been passed; otherwise it
    /// returns at the [`Txq::reclaim`] that sees it passed.
    ///
    /// Returns whether the report was taken: false for a seq past the pool,
    /// a page not in flight, or a page already reported (a repeated
    /// report), none of which changes anything.
    pub fn release(&mut self, ch: TxChannel, seq: u8) -> bool {
        let pages = &mut self.channels[ch.slot()].pages[..pages_of(ch.slot())];
        let Some(page) = pages.get_mut(usize::from(seq)) else {
            return false;
        };
        if !page.in_flight || page.reported {
            return false;
        }
        page.reported = true;
        if page.bd_done {
            *page = Page::default();
        }
        true
    }

    fn wd_offset(ch: TxChannel, page: usize) -> usize {
        (PAGE_AT[ch.slot()] + page) * WD_PAGE_BYTES
    }

    fn payload_offset(ch: TxChannel, page: usize) -> usize {
        WD_REGION + (PAGE_AT[ch.slot()] + page) * PAYLOAD_BYTES
    }
}

/// A data or management queue: its TX ring and its channel's pages.
pub struct Txq<'r> {
    rings: &'r mut Rings,
    pools: &'r mut TxPools,
    ch: TxChannel,
}

impl Txq<'_> {
    /// Frames that can be submitted now: the lesser of the free TXBDs
    /// (`rtw89_pci_get_avail_txbd_num`, `pci.c:1222-1231`, one entry kept
    /// empty) and the free WD pages. It is as of the last [`Txq::reclaim`].
    pub fn free(&self) -> usize {
        let slot = self.ch.slot();
        let wp = usize::from(self.rings.data_host[slot].get());
        let rp = usize::from(self.rings.data_hw[slot].get());
        let bds = if rp > wp {
            rp - wp - 1
        } else {
            tx_entries(self.ch.ring()) - (wp - rp) - 1
        };
        let pages = self.pools.channels[slot].pages[..pages_of(slot)]
            .iter()
            .filter(|page| !page.in_flight)
            .count();
        bds.min(pages)
    }

    /// Take back the TXBDs the DMA engine has read, as
    /// `rtw89_pci_reclaim_txbd` (`pci.c:501-520`) does: read the hardware
    /// index (`IDX` `[27:16]`, `rtw89_pci_txbd_recalc` `pci.c:87`), and for
    /// each entry from the last one read up to it, take the TXBD back for
    /// the CPU and mark its page's TXBD done; a page already reported
    /// returns to its pool. Nothing is written to the device.
    ///
    /// Before `pre_init`, after `deinit` and after a `pre_init` that failed,
    /// it is refused with `Tx("TX rings are stopped")` and nothing is read:
    /// `clr_idx_all` restarts the hardware index at 0 while the host's still
    /// stand where the last reclaim left them, so a 0 read then would pass
    /// for TXBDs read that nothing proves the device read. Their pages stay
    /// in flight; once the next `pre_init` restarts the ring no TXBD names
    /// them, so they never return (see `restart_host_indices`).
    ///
    /// The device reads only what a [`Txq::kick`] handed it, so the bound is
    /// the host index last kicked, not the one [`Txq::submit`] advanced: an
    /// entry written since stays the host's. With no kicked TXBD outstanding
    /// nothing is read. A hardware index past the ring, or past the kicked
    /// host index (one the device was never given, which Linux would take as
    /// a wrap), or all ones, is refused before anything changes. A failure
    /// to take a TXBD back leaves it and the entries after it outstanding
    /// for the next reclaim.
    ///
    /// # Safety
    /// [`Ch12::submit`]'s contract, for this queue. The hardware index
    /// having passed an entry is the proof that the device finished reading
    /// its TXBD and WD page.
    pub unsafe fn reclaim<I: AxIo + ?Sized>(&mut self, io: &mut I) -> Result<(), AxError> {
        let slot = self.ch.slot();
        let rings = &mut *self.rings;
        if !rings.data_ready.get() {
            return Err(AxError::Tx("TX rings are stopped"));
        }
        let wp = usize::from(rings.data_kicked[slot].get());
        let rp = usize::from(rings.data_hw[slot].get());
        if wp == rp {
            return Ok(());
        }
        let idx = TX_TABLE[self.ch.ring()].idx;
        let value = io.read32(idx)?;
        if value == u32::MAX {
            return Err(AxError::AllOnes { register: idx });
        }
        let hw = ((value & TXBD_HW_IDX_MASK) >> TXBD_HW_IDX_MASK.trailing_zeros()) as usize;
        let entries = tx_entries(self.ch.ring());
        if hw >= entries {
            return Err(AxError::Tx("TX hardware index out of range"));
        }
        let outstanding = (wp + entries - rp) % entries;
        let done = (hw + entries - rp) % entries;
        if done > outstanding {
            return Err(AxError::Tx("TX hardware index past the host index"));
        }
        // No TXBD read may move before the index read that proved it done.
        fence(Ordering::Acquire);
        let pool = &mut self.pools.channels[slot];
        for n in 0..done {
            let entry = (rp + n) % entries;
            let bd = TX_RING_AT[self.ch.ring()] + entry * BD_BYTES;
            // SAFETY: the hardware index has passed `entry`, so the DMA
            // engine has finished reading its TXBD, and it cannot read the
            // entry again until the host index passes it on a later submit.
            unsafe { rings.tx.dma.sync_for_cpu(bd..bd + BD_BYTES) }?;
            let page = &mut pool.pages[usize::from(pool.bd[entry])];
            page.bd_done = true;
            if page.reported {
                *page = Page::default();
            }
            rings.data_hw[slot].set(((entry + 1) % entries) as u16);
        }
        Ok(())
    }

    /// Whether entries were written since the last [`Txq::kick`]: the host
    /// index moved past what the device was last told.
    pub fn pending_kick(&self) -> bool {
        let slot = self.ch.slot();
        self.rings.data_host[slot].get() != self.rings.data_kicked[slot].get()
    }

    /// `__rtw89_pci_tx_kick_off` (`pci.c:1332-1345`): after a fence, write
    /// the host index (W16 `IDX`), handing over every entry [`Txq::submit`]
    /// wrote since the last kick. Nothing is written when none was (`Ok(false)`;
    /// Linux kicks a channel only after pushing frames to it,
    /// `rtw89_core_txq_schedule`, `core.c:4854-4858`). A failed write keeps
    /// the entries pending, and the next kick writes the same index again:
    /// the device may have seen it, so their pages return only by their
    /// TXBDs and reports, like any other.
    ///
    /// # Safety
    /// [`Txq::submit`]'s contract.
    pub unsafe fn kick<I: AxIo + ?Sized>(&mut self, io: &mut I) -> Result<bool, AxError> {
        if !self.pending_kick() {
            return Ok(false);
        }
        let slot = self.ch.slot();
        let host = self.rings.data_host[slot].get();
        // Every TXBD and page write is visible before the index that names
        // them.
        fence(Ordering::Release);
        io.write16(TX_TABLE[self.ch.ring()].idx, host)?;
        self.rings.data_kicked[slot].set(host);
        Ok(true)
    }

    /// Write one frame as `rtw89_pci_tx_write` (`pci.c:1637-1678`) with
    /// `rtw89_pci_txbd_submit` / `rtw89_pci_txwd_submit` (`pci.c:1494-1636`)
    /// does: take the lowest free page, zero it, write `head` (the WD body
    /// and info), wp_info (`seq0 = page | VALID`, `seq1..3 = 0`) and one
    /// addr_info v1 (`LEN | LS`, the slot's address below 4 GiB) into it and
    /// `payload` into its slot, and make both visible; then write the TXBD
    /// (length 70, the page's address), make it visible, and advance the
    /// host index (`rtw89_pci_tx_bd_ring_update`). Nothing is written to the
    /// device: [`Txq::kick`] hands the frames over. Returns the page's seq.
    ///
    /// A full queue ([`Txq::free`] 0) returns `Ok(None)` without touching
    /// the device. A frame that is empty or longer than [`MAX_TX_FRAME`] is
    /// refused, as is a head without WD_INFO_EN (b22 of body dword 0; see
    /// [`WD_HEAD_BYTES`]) and any frame before `pre_init` or after `deinit`.
    /// A failure before the host index advances hands nothing over: the
    /// page stays free and the host index unmoved.
    ///
    /// # Safety
    /// [`Ch12::submit`]'s contract, for this queue, and `pools` were
    /// installed under [`TxPools::install`]'s contract for the same
    /// requester.
    pub unsafe fn submit(
        &mut self,
        head: &[u8; WD_HEAD_BYTES],
        payload: &[u8],
    ) -> Result<Option<u8>, AxError> {
        if payload.is_empty() {
            return Err(AxError::Tx("empty frame"));
        }
        if payload.len() > MAX_TX_FRAME {
            return Err(AxError::Tx("frame longer than one TX address entry"));
        }
        if u32::from_le_bytes([head[0], head[1], head[2], head[3]]) & WD_INFO_EN == 0 {
            return Err(AxError::Tx("WD head without its WD info"));
        }
        if !self.rings.data_ready.get() {
            return Err(AxError::Tx("TX rings are stopped"));
        }
        if self.free() == 0 {
            return Ok(None);
        }
        let ch = self.ch;
        let slot = ch.slot();
        let rings = &mut *self.rings;
        let pools = &mut *self.pools;
        let Some(page) = pools.channels[slot].pages[..pages_of(slot)]
            .iter()
            .position(|page| !page.in_flight)
        else {
            return Ok(None);
        };
        let wd_at = TxPools::wd_offset(ch, page);
        let slot_at = TxPools::payload_offset(ch, page);
        let base = u64::from(pools.memory.address()?);
        let wd_dma = base + wd_at as u64;
        let slot_dma = (base + slot_at as u64) as u32;
        // rtw89_pci_enqueue_txwd zeroes a page as it returns (pci.h:1726).
        let mut wd = [0u8; WD_PAGE_BYTES];
        wd[..WD_HEAD_BYTES].copy_from_slice(head);
        let wp_info = WD_HEAD_BYTES;
        wd[wp_info..wp_info + 2].copy_from_slice(&(page as u16 | TXWP_VALID).to_le_bytes());
        // rtw89_pci_fill_txaddr_info_v1: one entry, HIGH_SEL 0, LS.
        let addr = wp_info + WP_INFO_BYTES;
        let length_opt = payload.len() as u16 | PCIADDR_LS_V1;
        wd[addr..addr + 2].copy_from_slice(&length_opt.to_le_bytes());
        wd[addr + 2..addr + 4].copy_from_slice(&(slot_dma as u16).to_le_bytes());
        wd[addr + 4..addr + 6].copy_from_slice(&((slot_dma >> 16) as u16).to_le_bytes());
        pools.memory.write(wd_at, &wd)?;
        pools.memory.write(slot_at, payload)?;
        pools
            .memory
            .dma
            .sync_for_device(wd_at..wd_at + WD_PAGE_BYTES)?;
        pools
            .memory
            .dma
            .sync_for_device(slot_at..slot_at + payload.len())?;
        let entry = usize::from(rings.data_host[slot].get());
        let bd = TX_RING_AT[ch.ring()] + entry * BD_BYTES;
        // From here the TXBD may name the pool: it is never unmapped.
        pools.memory.published = true;
        rings.tx.write(bd, &encode_txbd(WD_BYTES as u16, wd_dma))?;
        rings.tx.dma.sync_for_device(bd..bd + BD_BYTES)?;
        let pool = &mut pools.channels[slot];
        pool.pages[page] = Page {
            in_flight: true,
            bd_done: false,
            reported: false,
        };
        pool.bd[entry] = page as u8;
        // rtw89_pci_tx_bd_ring_update: advance and wrap the host index.
        let host = ((entry + 1) % tx_entries(ch.ring())) as u16;
        rings.data_host[slot].set(host);
        Ok(Some(page as u8))
    }
}

/// One RPP of an RPQ buffer (`struct rtw89_pci_rpp_info`, `pci.h:1506-1514`):
/// the page `seq` of the channel `TxChannel::of_qsel(qsel)` is released with
/// `status` ([`TX_DONE`] when acknowledged; `RTW89_TX_RETRY_LIMIT` 1,
/// `_LIFE_TIME` 2 and `_MACID_DROP` 3 when not, `core.h:4676-4679`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Release {
    pub seq: u16,
    pub status: u8,
    pub qsel: u8,
}

/// The release reports of one RPQ buffer ([`releases`]): the ones that name
/// a page, in buffer order, and how many were skipped (M3 spec S6's
/// `invalid`, saturating).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Releases {
    pub reports: Vec<Release>,
    pub invalid: u16,
}

/// The release reports of one whole RPQ buffer, as
/// `rtw89_pci_release_tx_skbs` (`pci.c:644-695`) reads them: the RX BD info
/// word, then the RX descriptor (16 bytes, or 32 when `LONG_RXD`) after
/// `DRV_INFO_SIZE` 8-byte units (`rtw89_core_query_rxdesc`,
/// `core.c:4140-4146`, the shift always 0), then one RPP dword every 4
/// bytes while `offset + 4 <= WRITE_SIZE`. A report whose seq is past the
/// pools (16) or whose qsel names no [`TxChannel`] is skipped and counted in
/// [`Releases::invalid`], where Linux warns and skips seqs from 512 and
/// channels it has no ring for (`pci.c:607-617`).
///
/// A buffer that is not both FS and LS (`pci.c:672-675`), one too short
/// for its descriptor, or one whose `WRITE_SIZE` runs past it is refused.
pub fn releases(buffer: &[u8]) -> Result<Releases, AxError> {
    let word = |at: usize| -> Option<u32> {
        Some(u32::from_le_bytes(buffer.get(at..at + 4)?.try_into().ok()?))
    };
    const SHORT: AxError = AxError::Tx("release report shorter than its descriptor");
    let info = word(0).ok_or(SHORT)?;
    if info & RXBD_FS == 0 || info & RXBD_LS == 0 {
        return Err(AxError::Tx("release report is not one whole buffer"));
    }
    let d0 = word(RXBD_INFO_BYTES).ok_or(SHORT)?;
    let write_size = (info & RXBD_WRITE_SIZE) as usize;
    if write_size > buffer.len() {
        return Err(AxError::Tx("release report longer than its buffer"));
    }
    let rxd = if d0 & RXD_LONG != 0 {
        RXD_LONG_BYTES
    } else {
        RXD_SHORT_BYTES
    };
    let drv_info = ((d0 >> RXD_DRV_INFO_SHIFT) & RXD_DRV_INFO) as usize * 8;
    let mut offset = RXBD_INFO_BYTES + drv_info + rxd;
    let mut out = Releases {
        reports: Vec::new(),
        invalid: 0,
    };
    while offset + RPP_BYTES <= write_size {
        let rpp = word(offset).ok_or(SHORT)?;
        offset += RPP_BYTES;
        let release = Release {
            seq: ((rpp >> RPP_SEQ_SHIFT) & RPP_SEQ) as u16,
            status: ((rpp >> RPP_TX_STATUS_SHIFT) & RPP_TX_STATUS) as u8,
            qsel: ((rpp >> RPP_QSEL_SHIFT) & RPP_QSEL) as u8,
        };
        if usize::from(release.seq) >= WD_PAGES || TxChannel::of_qsel(release.qsel).is_none() {
            out.invalid = out.invalid.saturating_add(1);
            continue;
        }
        out.reports
            .try_reserve(1)
            .map_err(|_| AxError::Tx("out of memory for release reports"))?;
        out.reports.push(release);
    }
    Ok(out)
}

/// The RXQ ring, read by polling as `rtw89_pci_poll_rxq_dma` reads it
/// under NAPI. Buffers are delivered raw, as far as the device wrote them; nothing is parsed.
pub struct Rxq<'r> {
    rings: &'r mut Rings,
    /// The CPU's copy of the buffer being delivered, reused for every entry.
    copy: Vec<u8>,
    /// The last in-range hardware index `poll` read.
    hw: u16,
}

impl Rxq<'_> {
    /// The last in-range RXQ hardware index (`IDX` `[27:16]`) `poll` read; 0
    /// before the first.
    pub fn hardware_index(&self) -> u16 {
        self.hw
    }

    /// Deliver every RXQ entry the device has filled, as
    /// `rtw89_pci_poll_rxq_dma` (`pci.c:436`) does with an unlimited budget:
    /// read the hardware index (`IDX` `[27:16]`, `rtw89_pci_rxbd_recalc`
    /// `pci.c:146`), count the entries from the host index to it
    /// (`rtw89_pci_dma_recalc` `pci.c:61`, `rx_ring_eq_is_full` false), and
    /// for each one take its buffer for the CPU, copy out what the device
    /// wrote (its BD info's `WRITE_SIZE`, at most `RX_BUFFER_BYTES`), call
    /// `each` with the copy, give the buffer back to the device and advance.
    /// The host index is written once after a batch that consumed anything
    /// (`pci.c:433`). Returns the number of entries consumed. A batch that
    /// finds `RX_ENTRIES_RXQ - 1` entries filled, the ring full, is counted in [`Rings::rxq_full_batches`] before any
    /// of them is delivered.
    ///
    /// A hardware index past the ring, or all ones, is refused before any
    /// buffer is touched. An error from `each` ends the batch: as Linux's
    /// `err_sync_device` path (`pci.c:401-403`), that buffer is still given
    /// back and consumed, the host index is written for every entry consumed,
    /// and the error is returned. A failure to take a buffer leaves that
    /// entry unconsumed (the host's, and redelivered by the next poll), with
    /// the host index still written for the entries before it. A failure to
    /// give a buffer back after `each` has run on it also ends the batch,
    /// but that entry is consumed, so nothing `each` has seen is delivered
    /// twice; `each`'s own error wins over it.
    ///
    /// # Safety
    /// `io` is the exclusive register window of the same RTL8852C requester
    /// that `pre_init` programmed with these rings, and the `pre_init`
    /// contract still holds: its DMA domain isolates that requester, the
    /// rings are coherent WB RAM with NoSnoop and ATS disabled, and one
    /// worker owns the device and this queue. The hardware index having
    /// passed an entry is the proof that the device finished writing its
    /// buffer.
    pub unsafe fn poll<I: AxIo + ?Sized>(
        &mut self,
        io: &mut I,
        each: &mut dyn FnMut(&[u8]) -> Result<(), AxError>,
    ) -> Result<usize, AxError> {
        // SAFETY: the caller's `poll` contract, forwarded unchanged.
        unsafe { poll_rx(io, self.rings, RXQ, &mut self.copy, &mut self.hw, each) }
    }
}

/// The RPQ ring, read by polling with RXQ's rules. Buffers are delivered
/// raw, as far as the device wrote them; [`releases`] reads the reports in one.
pub struct Rpq<'r> {
    rings: &'r mut Rings,
    /// The CPU's copy of the buffer being delivered, reused for every entry.
    copy: Vec<u8>,
}

impl Rpq<'_> {
    /// Deliver every RPQ entry the device has filled, exactly as
    /// [`Rxq::poll`] delivers RXQ's, with RPQ's own IDX (0x121C) and host
    /// index, as `rtw89_pci_poll_rpq_dma` (`pci.c:720`) reads them with
    /// `rtw89_pci_rxbd_recalc` and writes them back after
    /// `rtw89_pci_release_tx` (`pci.c:697-718`). A full RPQ is counted in
    /// [`Rings::rpq_full_batches`], not [`Rings::rxq_full_batches`]. A
    /// hardware index past the ring is `Tx("RPQ hardware index out of
    /// range")`, refused untouched.
    ///
    /// Where Linux skips every remaining entry after a buffer it cannot
    /// read (`pci.c:704-711`), an error from `each` here ends the batch
    /// after that one buffer, as on RXQ: the entries after it stay the
    /// host's and the next poll delivers them.
    ///
    /// # Safety
    /// The [`Rxq::poll`] contract, for RPQ.
    pub unsafe fn poll<I: AxIo + ?Sized>(
        &mut self,
        io: &mut I,
        each: &mut dyn FnMut(&[u8]) -> Result<(), AxError>,
    ) -> Result<usize, AxError> {
        let mut hw = 0;
        // SAFETY: the caller's `poll` contract, forwarded unchanged.
        unsafe { poll_rx(io, self.rings, RPQ, &mut self.copy, &mut hw, each) }
    }
}

/// One poll of RX ring `ring` (RXQ or RPQ), as [`Rxq::poll`] describes;
/// `last_hw` takes the hardware index once it is found in range.
///
/// # Safety
/// The [`Rxq::poll`] contract, for `ring`.
unsafe fn poll_rx<I: AxIo + ?Sized>(
    io: &mut I,
    rings: &mut Rings,
    ring: usize,
    copy: &mut Vec<u8>,
    last_hw: &mut u16,
    each: &mut dyn FnMut(&[u8]) -> Result<(), AxError>,
) -> Result<usize, AxError> {
    let idx = RX_TABLE[ring].1;
    let value = io.read32(idx)?;
    if value == u32::MAX {
        return Err(AxError::AllOnes { register: idx });
    }
    let hw = ((value & TXBD_HW_IDX_MASK) >> TXBD_HW_IDX_MASK.trailing_zeros()) as usize;
    let entries = rx_entries(ring);
    if hw >= entries {
        return Err(if ring == RXQ {
            AxError::Rx("RXQ hardware index out of range")
        } else {
            AxError::Tx("RPQ hardware index out of range")
        });
    }
    *last_hw = hw as u16;
    let host = &rings.rx_host[ring];
    let start = usize::from(host.get());
    let count = if hw >= start {
        hw - start
    } else {
        entries - (start - hw)
    };
    if count == 0 {
        return Ok(0);
    }
    if count == entries - 1 {
        // Every entry the device can fill is filled: it may have had no
        // free buffer since the last poll.
        let full = if ring == RXQ {
            &rings.rxq_full
        } else {
            &rings.rpq_full
        };
        full.set(full.get().saturating_add(1));
    }
    // No buffer read may move before the index read that proved it full.
    fence(Ordering::Acquire);
    if copy.len() != RX_BUFFER_BYTES {
        copy.try_reserve_exact(RX_BUFFER_BYTES)
            .map_err(|_| AxError::Rx("out of memory for the RX buffer copy"))?;
        copy.resize(RX_BUFFER_BYTES, 0);
    }
    let mut consumed = 0;
    let mut failed = None;
    while consumed < count {
        let entry = (start + consumed) % entries;
        let buffer = &rings.rx_buffers[rx_buffer(ring, entry)];
        // SAFETY: `entry` lies in [host index, hardware index), which the
        // hardware index has passed: the device finished writing its buffer
        // and cannot write it again until the host index is written past
        // it, which happens only below, once `deliver` has copied it out and
        // is done with it; the CPU reads it again only after another
        // `sync_for_cpu`. The `poll` contract holds.
        match unsafe { deliver(buffer, copy, each) } {
            Ok(delivered) => {
                consumed += 1;
                host.set(((entry + 1) % entries) as u16);
                if let Err(error) = delivered {
                    failed = Some(error);
                    break;
                }
            }
            Err(error) => {
                failed = Some(error);
                break;
            }
        }
    }
    if consumed > 0 {
        // Every CPU read of the buffers is done before the device may write
        // them again.
        fence(Ordering::Release);
        let written = io.write16(idx, host.get());
        // The first cause wins; a refused write alone is the error.
        if failed.is_none() {
            written?;
        }
    }
    match failed {
        Some(error) => Err(error),
        None => Ok(consumed),
    }
}

/// One filled RXQ entry: `sync_for_cpu` its buffer, copy what the device
/// wrote into `copy`, call `each` with the copy, then `sync_for_device` the
/// buffer. What it wrote is its BD info's `WRITE_SIZE` from the buffer's
/// start, the BD info included (`rx_info->len`, which `rtw89_skb_put_rx_data`
/// copies to, `pci.c:284`, `:301`), at least the BD info and at most the
/// buffer; the rest of the buffer is stale and not copied (M4 spec S9.3).
/// `Ok` means `each` has run, so the entry is consumed: it carries `each`'s
/// error, else the `sync_for_device` failure, else success. `Err` means the
/// buffer could not be taken and `each` did not run.
///
/// # Safety
/// The device has finished writing `buffer` and cannot write it until the
/// caller next writes the host index past it; the `Rxq::poll` contract
/// holds.
unsafe fn deliver(
    buffer: &Allocation,
    copy: &mut [u8],
    each: &mut dyn FnMut(&[u8]) -> Result<(), AxError>,
) -> Result<Result<(), AxError>, AxError> {
    let whole = 0..RX_BUFFER_BYTES;
    // SAFETY: the caller proves the device completed its writes and does not
    // own the buffer until the host index passes it.
    unsafe { buffer.dma.sync_for_cpu(whole.clone()) }?;
    let from = buffer
        .cpu
        .address(0, RX_BUFFER_BYTES)
        .map_err(beacon::Error::Memory)?;
    // SAFETY: the read mirror of `Allocation::write`: `from` is the bounded
    // CPU view of `RX_BUFFER_BYTES` of native WB RAM, now CPU-owned and
    // synced, which no device write and no Rust reference aliases until the
    // host index passes it; `copy` is a distinct heap buffer of at least
    // that length, and the BD info lies within both.
    unsafe {
        from.as_ptr()
            .copy_to_nonoverlapping(copy.as_mut_ptr(), RXBD_INFO_BYTES)
    };
    let info = u32::from_le_bytes([copy[0], copy[1], copy[2], copy[3]]);
    let written = ((info & RXBD_WRITE_SIZE) as usize).clamp(RXBD_INFO_BYTES, RX_BUFFER_BYTES);
    let copy = &mut copy[..written];
    // SAFETY: as above, for the first `written` bytes, which lie within both.
    unsafe {
        from.as_ptr()
            .copy_to_nonoverlapping(copy.as_mut_ptr(), written)
    };
    let delivered = each(copy);
    let given = buffer.dma.sync_for_device(whole);
    // Once `each` has seen the buffer it is consumed whatever the give-back
    // says: delivered again, an RPQ report could return a page since reused.
    // The first cause wins.
    Ok(delivered.and(given.map_err(AxError::from)))
}

#[cfg(test)]
impl Rings {
    /// RXQ's host index at `entry`, as if the entries before it had been
    /// consumed, for a test that starts the device's index there too.
    pub(crate) fn rxq_resume_at(&self, entry: u16) {
        self.rx_host[RXQ].set(entry);
    }
}

#[cfg(test)]
pub(super) mod tests;

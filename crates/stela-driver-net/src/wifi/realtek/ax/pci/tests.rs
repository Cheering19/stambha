extern crate std;
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, READ_FAILED, RegisterModel};
use crate::wifi::realtek::beacon::tests::{Domain, Model, Source};
use alloc::{boxed::Box, vec, vec::Vec};
use std::sync::Mutex;

// The beacon DMA model hands out IOVAs 0x1000_0000 + 0x10000 * n in mapping
// order, and `Rings::install` maps the TX BD pool, the RX BD pool, the 288 RX
// buffers (RXQ's 256, then RPQ's 32) and the CH12 slots in that order.
pub(crate) const TX_POOL: u32 = 0x1001_0000;
const RX_POOL: u32 = 0x1002_0000;
const RX_BUFFER0: u32 = 0x1003_0000;
pub(crate) const FWCMD: u32 = 0x1123_0000;
const MAPPINGS: usize = 291;

fn install(m: &Arc<Mutex<Model>>) -> Rings {
    // SAFETY: the model domain isolates only the modeled requester behind
    // the test's `RegisterModel`; its pages are plain kept test RAM with no
    // real device, and this test is the queue's only worker.
    unsafe { Rings::install(&Source, Arc::new(Domain(m.clone()))) }.unwrap()
}

fn start(io: &mut RegisterModel, cut: u8, rings: &Rings) -> Result<(), AxError> {
    // SAFETY: `io` is the modeled requester that `rings`' model domain
    // isolates; no real device can reach the published IOVAs, and this test
    // is the only worker.
    unsafe { pre_init(io, cut, rings) }
}

fn send(rings: &mut Rings, io: &mut RegisterModel, packet: &[u8]) -> Result<(), AxError> {
    // SAFETY: every test sends on the same `RegisterModel` that `start`
    // programmed with these rings, under the same model-only contract.
    unsafe { rings.ch12().submit(io, packet) }
}

/// The BD RAM reset (0x1000 bit3) self-clears, as the hardware's does.
fn settled() -> RegisterModel {
    let mut io = RegisterModel::new();
    io.on_read(
        0x1000,
        Box::new(|m: &mut RegisterModel| m.get32(0x1000) & !(1 << 3)),
    );
    io
}

/// The CH12 DMA engine consumes every kicked entry at once.
pub(crate) fn reclaim_at_once(io: &mut RegisterModel) {
    io.on_read(
        0x1080,
        Box::new(|m: &mut RegisterModel| {
            let host = m.get32(0x1080) & 0xFFF;
            host | host << 16
        }),
    );
}

/// What the device would read at `iova`, through the model's mapping.
pub(crate) fn device_read(m: &Model, iova: u64, len: usize) -> Vec<u8> {
    let (base, (pages, cpu)) = m.mappings.range(..=iova).next_back().unwrap();
    assert!(iova + len as u64 <= base + pages.bytes() as u64);
    let at = cpu + (iova - base) as usize;
    // SAFETY: the range lies inside a live pinned allocation this test's
    // model holds, and nothing writes it concurrently.
    unsafe { core::slice::from_raw_parts(at as *const u8, len) }.to_vec()
}

/// Put `bytes` at `iova` as the device would write them, through the
/// model's mapping: the mirror of `device_read`.
pub(crate) fn device_write(m: &Model, iova: u64, bytes: &[u8]) {
    let (base, (pages, cpu)) = m.mappings.range(..=iova).next_back().unwrap();
    assert!(iova + bytes.len() as u64 <= base + pages.bytes() as u64);
    let at = cpu + (iova - base) as usize;
    // SAFETY: the range lies inside a live pinned allocation this test's
    // model holds, the test buffer cannot overlap it, and nothing reads or
    // writes the range concurrently.
    unsafe { core::ptr::copy_nonoverlapping(bytes.as_ptr(), at as *mut u8, bytes.len()) };
}

pub(crate) fn running() -> (Arc<Mutex<Model>>, Rings, RegisterModel) {
    let m = Arc::new(Mutex::new(Model::default()));
    let rings = install(&m);
    let mut io = settled();
    start(&mut io, 2, &rings).unwrap();
    io.trace.clear();
    (m, rings, io)
}

#[test]
fn pre_init_matches_linux_sequence() {
    let m = Arc::new(Mutex::new(Model::default()));
    let rings = install(&m);
    let mut io = settled();
    // Bits the sequence clears start set, so every clear shows.
    io.set32(0x3848, 0xFFFF);
    io.set32(0x38C8, 0x0F01);
    io.set32(0x0070, 0x8001);
    io.set32(0x3008, 0x0004);
    io.set32(0x3124, 0x0008);
    io.set32(0x1000, 0xC080);
    io.set32(0x9A00, 0x0002);
    // CH8 is busy on the first busy1 sample: one 10 us wait.
    let mut busy = true;
    io.on_read(
        0x101C,
        Box::new(move |_m: &mut RegisterModel| {
            if core::mem::take(&mut busy) {
                1 << 16
            } else {
                0
            }
        }),
    );
    start(&mut io, 2, &rings).unwrap();
    let t = |channel: usize| TX_POOL + TX_RING_AT[channel] as u32;
    let expected = vec![
        // rtw89_pci_deglitch_setting, 8852C: 0x3800/0x3880 + RAC_ANA24 * 2 [11:8].
        Op::W16(0x3848, 0xF0FF),
        Op::W16(0x38C8, 0x0001),
        // rtw89_pci_hci_ldo, 8852C: clear B_AX_PCIE_DIS_L2_CTRL_LDO_HCI.
        Op::W32(0x0070, 0x0001),
        // rtw89_pci_power_wake_ax(true): BIT_WAKE_CTRL.
        Op::W32(0x0074, 0x0020),
        // rtw89_pci_autoload_hang: B_AX_BG_CLR_ASYNC_M3 pulse.
        Op::W32(0x303C, 0x0010),
        Op::W32(0x303C, 0x0000),
        // (cut 2: no l12_vmain / gen2_force_ib.)
        // rtw89_pci_l1_ent_lat, rtw89_pci_wd_exit_l1 (pci.c:2770-2784).
        Op::W32(0x3008, 0x0000),
        Op::W32(0x3008, 0x0040),
        // rtw89_pci_set_io_rcy (pci.c:2814): 6 ms timers, WDT modes, S1 off.
        Op::W32(0x3104, 72000),
        Op::W32(0x3110, 72000),
        Op::W32(0x311C, 72000),
        Op::W32(0x3100, 0x0008),
        Op::W32(0x310C, 0x0008),
        Op::W32(0x3118, 0x0008),
        Op::W32(0x3124, 0x0000),
        // pci.c:3114: B_AX_STOP_WPDMA.
        Op::W32(0x1010, 0x0008_0000),
        // rtw89_pci_ctrl_dma_all(false): STOP_AXI_MST, then clear RX/TXHCI_EN.
        Op::W32(0x1000, 0x0002_C080),
        Op::W32(0x1000, 0x0002_4000),
        // rtw89_pci_poll_dma_all_idle: one 10 us wait on busy1.
        Op::Delay(10),
        // rtw89_pci_clr_idx_all_ax, 8852C channel set.
        Op::W32(0x1014, 0x07FF),
        Op::W32(0x11C4, 0x0003),
        Op::W32(0x1200, 0x0003),
        // rtw89_pci_mode_op: RXBD packet mode, TX burst 256B, RX burst 128B,
        Op::W32(0x1000, 0x0002_0000),
        Op::W32(0x1000, 0x0002_0002),
        Op::W32(0x1000, 0x0002_0102),
        // tag num 8, WD intervals 256 ns, then the trunc-mode settings.
        Op::W32(0x1204, 0x0007),
        Op::W32(0x1000, 0x1002_0102),
        Op::W32(0x1000, 0x1102_0102),
        Op::W32(0x8810, 0x0001),
        Op::W32(0x9A00, 0x0000),
        // rtw89_pci_reset_trx_rings (pci.c:1776): per TX channel NUM, BDRAM
        // (rtw89_bd_ram_table_dual, pci.c:1693), DESA_L, DESA_L + 4; the
        // addresses are rtw89_pci_ch_dma_addr_set_v1 (pci.c:1116).
        Op::W16(0x1024, 256), // ACH0: RTW89_PCI_TXBD_NUM_MAX
        Op::W32(0x1300, 0x0002_0500),
        Op::W32(0x1230, t(0)),
        Op::W32(0x1234, 0),
        Op::W16(0x1026, 256), // ACH1
        Op::W32(0x1304, 0x0002_0505),
        Op::W32(0x1238, t(1)),
        Op::W32(0x123C, 0),
        Op::W16(0x1028, 256), // ACH2
        Op::W32(0x1308, 0x0002_050A),
        Op::W32(0x1240, t(2)),
        Op::W32(0x1244, 0),
        Op::W16(0x102A, 256), // ACH3
        Op::W32(0x130C, 0x0002_050F),
        Op::W32(0x1248, t(3)),
        Op::W32(0x124C, 0),
        Op::W16(0x102C, 16), // ACH4
        Op::W32(0x1310, 0x0002_0514),
        Op::W32(0x1250, t(4)),
        Op::W32(0x1254, 0),
        Op::W16(0x102E, 16), // ACH5
        Op::W32(0x1314, 0x0002_0519),
        Op::W32(0x1258, t(5)),
        Op::W32(0x125C, 0),
        Op::W16(0x1030, 16), // ACH6
        Op::W32(0x1318, 0x0002_051E),
        Op::W32(0x1260, t(6)),
        Op::W32(0x1264, 0),
        Op::W16(0x1032, 16), // ACH7
        Op::W32(0x131C, 0x0002_0523),
        Op::W32(0x1268, t(7)),
        Op::W32(0x126C, 0),
        Op::W16(0x1034, 16), // CH8
        Op::W32(0x1320, 0x0001_0528),
        Op::W32(0x1270, t(8)),
        Op::W32(0x1274, 0),
        Op::W16(0x1036, 16), // CH9
        Op::W32(0x1324, 0x0001_052D),
        Op::W32(0x1278, t(9)),
        Op::W32(0x127C, 0),
        Op::W16(0x1438, 16), // CH10 (TYPE1: _V1 NUM)
        Op::W32(0x1420, 0x0001_0532),
        Op::W32(0x1458, t(10)),
        Op::W32(0x145C, 0),
        Op::W16(0x143A, 16), // CH11 (TYPE1: _V1 NUM)
        Op::W32(0x1424, 0x0001_0537),
        Op::W32(0x1460, t(11)),
        Op::W32(0x1464, 0),
        Op::W16(0x1038, 16), // CH12: sidx 60, max 4, min 1
        Op::W32(0x1328, 0x0001_043C),
        Op::W32(0x1280, t(12)),
        Op::W32(0x1284, 0),
        // RX: NUM, DESA_L, DESA_L + 4; no IDX (rx_ring_eq_is_full false).
        Op::W16(0x1210, 256), // RXQ: RTW89_PCI_RXBD_NUM_MAX
        Op::W32(0x1220, RX_POOL),
        Op::W32(0x1224, 0),
        Op::W16(0x1212, 32), // RPQ: its own page of the RX pool.
        Op::W32(0x1228, RX_POOL + 4096),
        Op::W32(0x122C, 0),
        // rtw89_pci_init_wp_16sel (pci.c:1722).
        Op::W32(0x1334, 0x0302_0100),
        Op::W32(0x1338, 0x0706_0504),
        Op::W32(0x133C, 0x0B0A_0908),
        Op::W32(0x1340, 0x0F0E_0D0C),
        // rtw89_pci_rst_bdram_ax (pci.c:47).
        Op::W32(0x1000, 0x1102_010A),
        // rtw89_pci_ctrl_txdma_ch_ax(false): TX_STOP1_MASK, TX_STOP2_ALL.
        Op::W32(0x1010, 0x000F_FF00),
        Op::W32(0x11C0, 0x0003),
        // rtw89_pci_ctrl_txdma_fw_ch_ax(true): clear B_AX_STOP_CH12.
        Op::W32(0x1010, 0x000B_FF00),
        // rtw89_pci_ctrl_dma_all(true).
        Op::W32(0x1000, 0x1100_0102),
        Op::W32(0x1000, 0x1100_8182),
    ];
    assert_eq!(io.writes(), expected);
    // The idle polls sample busy1 twice, then busy2 and busy3 once, and the
    // BD RAM reset is sampled once.
    let polls: Vec<Op> = io
        .trace
        .iter()
        .copied()
        .filter(|op| matches!(op, Op::R32(0x101C | 0x11C8 | 0x1208)))
        .collect();
    assert_eq!(
        polls,
        [
            Op::R32(0x101C),
            Op::R32(0x101C),
            Op::R32(0x11C8),
            Op::R32(0x1208)
        ]
    );
}

fn pre_init_writes(cut: u8) -> Vec<Op> {
    let m = Arc::new(Mutex::new(Model::default()));
    let rings = install(&m);
    let mut io = settled();
    start(&mut io, cut, &rings).unwrap();
    io.writes()
}

#[test]
fn cav_only_step_runs_only_on_cut_0() {
    let cav = pre_init_writes(0);
    let cbv = pre_init_writes(2);
    // After autoload_hang, CAV alone runs rtw89_pci_l12_vmain and
    // rtw89_pci_gen2_force_ib.
    let at = cbv.iter().position(|op| *op == Op::W32(0x303C, 0)).unwrap() + 1;
    let mut with = cbv.clone();
    with.splice(
        at..at,
        [
            Op::W32(0x0070, 0x2000),
            Op::W32(0x00CC, 0x0004),
            Op::W32(0x0078, 0x0010),
            Op::W32(0x00CC, 0x0000),
        ],
    );
    assert_eq!(cav, with);
    assert!(
        !cbv.iter()
            .any(|op| matches!(op, Op::W32(0x00CC | 0x0078, _)))
    );
    assert_eq!(
        cbv.iter()
            .filter(|op| matches!(op, Op::W32(0x0070, _)))
            .count(),
        1
    );
}

#[test]
fn txbd_layout() {
    let bd = encode_txbd(2044, 0x1_2345_6780);
    assert_eq!(bd[0..2], 2044u16.to_le_bytes());
    assert_eq!(bd[2..4], ((1u16 << 14) | (0x01 << 6)).to_le_bytes());
    assert_eq!(bd[4..8], 0x2345_6780u32.to_le_bytes());
    // dma[39:32] fills opt[13:6]; nothing above bit 39 leaks into LS.
    let bd = encode_txbd(1, 0xFFAB_0000_0004);
    assert_eq!(bd[2..4], ((1u16 << 14) | (0xAB << 6)).to_le_bytes());
    assert_eq!(bd[4..8], 4u32.to_le_bytes());
}

#[test]
fn install_points_every_rx_bd_at_a_zeroed_buffer() {
    let m = Arc::new(Mutex::new(Model::default()));
    let rings = install(&m);
    let buffers: Vec<u32> = (0..288).map(|i| RX_BUFFER0 + i * 0x10000).collect();
    // Copy the device's view out before asserting: a failed assertion under
    // the lock would poison it for the rings' unmap on unwind.
    let (count, directions, tx_bds, rx_bds, contents) = {
        let state = m.lock().unwrap();
        let directions: Vec<Direction> = [TX_POOL, RX_POOL, FWCMD]
            .iter()
            .chain(&buffers)
            .map(|iova| state.directions[&u64::from(*iova)])
            .collect();
        let contents: Vec<Vec<u8>> = buffers
            .iter()
            .map(|iova| device_read(&state, u64::from(*iova), RX_BUFFER_BYTES))
            .collect();
        (
            state.mappings.len(),
            directions,
            device_read(&state, u64::from(TX_POOL), TX_POOL_BYTES),
            device_read(&state, u64::from(RX_POOL), 2 * 4096),
            contents,
        )
    };
    assert_eq!(count, MAPPINGS);
    let mut want = vec![
        Direction::Bidirectional,
        Direction::Bidirectional,
        Direction::ToDevice,
    ];
    want.extend([Direction::FromDevice; 288]);
    assert_eq!(directions, want);
    // ACH0-3's 256 BDs each, then the other nine rings' 16.
    assert_eq!(TX_POOL_BYTES, (4 * 256 + 9 * 16) * 8);
    assert_eq!(tx_bds, vec![0; TX_POOL_BYTES]);
    // RXQ's 256 BDs start the pool and RPQ's 32 start its second page; the
    // rest of each page stays zero. RXQ's BD e is buffer e, RPQ's buffer
    // 256 + e.
    let mut want = vec![0u8; 2 * 4096];
    for (i, buffer) in buffers.iter().enumerate() {
        // le16 buf_size 11498, le16 opt 0 (dma[39:32] = 0), le32 dma.
        let at = if i < 256 { i * 8 } else { 4096 + (i - 256) * 8 };
        want[at..at + 4].copy_from_slice(&[0xEA, 0x2C, 0, 0]);
        want[at + 4..at + 8].copy_from_slice(&buffer.to_le_bytes());
        assert_eq!(contents[i], vec![0; RX_BUFFER_BYTES]);
    }
    assert_eq!(rx_bds, want);
    // Never programmed: every mapping is returned.
    drop(rings);
    assert!(m.lock().unwrap().mappings.is_empty());
}

#[test]
fn submit_waits_for_reclaim() {
    let (m, mut rings, mut io) = running();
    let mut reads = 0;
    io.on_read(
        0x1080,
        Box::new(move |m: &mut RegisterModel| {
            reads += 1;
            let host = m.get32(0x1080) & 0xFFF;
            if reads >= 2 { host | host << 16 } else { host }
        }),
    );
    let (syncs, cpu_syncs) = {
        let s = m.lock().unwrap();
        (s.syncs, s.cpu_syncs)
    };
    let packet: Vec<u8> = (0..16 + 2020).map(|i| i as u8).collect();
    send(&mut rings, &mut io, &packet).unwrap();
    assert_eq!(
        io.trace,
        [
            Op::W16(0x1080, 1),
            Op::R32(0x1080),
            Op::Delay(1),
            Op::R32(0x1080)
        ]
    );
    // The device finds the TXBD through CH12's DESA and the packet through it.
    let state = m.lock().unwrap();
    let ring = u64::from(io.get32(0x1280));
    assert_eq!(ring, u64::from(TX_POOL) + TX_RING_AT[12] as u64);
    assert_eq!(
        device_read(&state, ring, 8),
        encode_txbd(2036, u64::from(FWCMD))
    );
    assert_eq!(device_read(&state, u64::from(FWCMD), 2036), packet);
    // Packet and TXBD made visible before the kick; TXBD returned after.
    assert_eq!(state.syncs - syncs, 2);
    assert_eq!(state.cpu_syncs - cpu_syncs, 1);
}

#[test]
fn submit_times_out_without_reclaim() {
    let (_m, mut rings, mut io) = running();
    let began = io.now;
    assert_eq!(
        send(&mut rings, &mut io, &[1, 2, 3]),
        Err(AxError::Timeout {
            register: 0x1080,
            mask: 0x0FFF_0000
        })
    );
    assert!(io.now - began >= 400_000);
    assert_eq!(io.trace[0], Op::W16(0x1080, 1));
    assert!(io.writes()[1..].iter().all(|op| *op == Op::Delay(1)));
    // The entry may still be in flight: nothing more is sent.
    assert!(rings.ch12_stopped());
    io.trace.clear();
    assert_eq!(
        send(&mut rings, &mut io, &[4]),
        Err(AxError::Dma("CH12 is stopped"))
    );
    assert!(io.trace.is_empty());
    // A new pre_init drains the DMA and restarts CH12 from index 0.
    start(&mut io, 2, &rings).unwrap();
    io.trace.clear();
    reclaim_at_once(&mut io);
    send(&mut rings, &mut io, &[5]).unwrap();
    assert_eq!(io.trace, [Op::W16(0x1080, 1), Op::R32(0x1080)]);
}

#[test]
fn submit_wraps_at_ring_size() {
    let (m, mut rings, mut io) = running();
    reclaim_at_once(&mut io);
    for i in 0..17u8 {
        send(&mut rings, &mut io, &[i; 40]).unwrap();
    }
    let kicks: Vec<u16> = io
        .trace
        .iter()
        .filter_map(|op| match op {
            Op::W16(0x1080, v) => Some(*v),
            _ => None,
        })
        .collect();
    let mut want: Vec<u16> = (1..16).collect();
    want.extend([0, 1]);
    assert_eq!(kicks, want);
    let state = m.lock().unwrap();
    let ring = u64::from(TX_POOL) + TX_RING_AT[12] as u64;
    // Slot 15 used the last packet slot; slot 0 was reused by packet 16.
    assert_eq!(
        device_read(&state, ring + 15 * 8, 8),
        encode_txbd(40, u64::from(FWCMD) + 15 * 4096)
    );
    assert_eq!(
        device_read(&state, ring, 8),
        encode_txbd(40, u64::from(FWCMD))
    );
    assert_eq!(device_read(&state, u64::from(FWCMD), 40), vec![16; 40]);
}

#[test]
fn submit_refuses_before_pre_init_and_bad_lengths() {
    let m = Arc::new(Mutex::new(Model::default()));
    let mut rings = install(&m);
    let mut io = settled();
    assert!(rings.ch12_stopped());
    assert_eq!(
        send(&mut rings, &mut io, &[1]),
        Err(AxError::Dma("CH12 is stopped"))
    );
    start(&mut io, 2, &rings).unwrap();
    assert!(!rings.ch12_stopped());
    io.trace.clear();
    let length = Err(AxError::Dma("CH12 packet length out of range"));
    assert_eq!(send(&mut rings, &mut io, &[]), length);
    assert_eq!(send(&mut rings, &mut io, &[0; 4097]), length);
    assert!(io.trace.is_empty());
    // A refused length never reached the queue, so CH12 is still running.
    assert!(!rings.ch12_stopped());
    reclaim_at_once(&mut io);
    send(&mut rings, &mut io, &[7; 4096]).unwrap();
}

#[test]
fn failed_idle_poll_keeps_rings_unpublished_and_ch12_stopped() {
    let m = Arc::new(Mutex::new(Model::default()));
    let mut rings = install(&m);
    let mut io = settled();
    io.set32(0x101C, 1 << 18); // CH12 stays busy.
    assert_eq!(
        start(&mut io, 2, &rings),
        Err(AxError::Timeout {
            register: 0x101C,
            mask: 0x0007_FF00
        })
    );
    assert!(
        !io.writes()
            .iter()
            .any(|op| matches!(op, Op::W16(0x1038, _) | Op::W32(0x1280, _)))
    );
    assert_eq!(
        send(&mut rings, &mut io, &[1]),
        Err(AxError::Dma("CH12 is stopped"))
    );
    drop(rings);
    assert!(m.lock().unwrap().mappings.is_empty());
}

#[test]
fn programmed_rings_stay_mapped_after_drop() {
    let (m, rings, _io) = running();
    drop(rings);
    // The device may still hold the ring addresses: quarantine, no unmap.
    assert_eq!(m.lock().unwrap().mappings.len(), MAPPINGS);
}

/// `read_poll_timeout_atomic(.., 1, RTW89_PCI_POLL_BDRAM_RST_CNT, false)`
/// (`pci.c:54`): left_ns = 100_000, cost 1001 per failure, so the expired
/// sample is k = 100_000 / 1001 + 1 = 100 and the poll makes 101 samples,
/// where wall time with a 1 us read ends near 50.
#[test]
fn bdram_reset_poll_spends_the_linux_iteration_budget() {
    let mut m = RegisterModel::new();
    m.read_cost_us = 1;
    m.set32(R_AX_HAXI_INIT_CFG1, B_AX_RST_BDRAM);
    assert!(rst_bdram(&mut m).is_err());
    let reads = m
        .trace
        .iter()
        .filter(|o| **o == Op::R32(R_AX_HAXI_INIT_CFG1))
        .count();
    // One read for the set32, then the poll samples.
    assert_eq!(reads, 1 + 101);
}

/// The registers `post_init` reads and writes, preset so every set, clear
/// and field shows: LTR off in hardware, every TX channel stopped, and
/// pre-init's `B_AX_STOP_WPDMA` plus `B_AX_STOP_PCIEIO` still set.
fn post_init_model() -> RegisterModel {
    let mut io = RegisterModel::new();
    io.set32(0x8410, 0);
    io.set32(0x8414, 0);
    io.set32(0x1600, 0x0000_0010);
    io.set32(0x1010, 0x001F_FF00);
    io.set32(0x11C0, 0x3);
    io
}

/// `rtw89_pci_ctrl_txdma_ch_ax(true)` (`pci.c:249`), then the PCI IO
/// release (`pci.c:3260-3265`).
const RELEASE_DMA: [Op; 3] = [
    Op::W32(0x1010, 0x0018_0000),
    Op::W32(0x11C0, 0),
    Op::W32(0x1010, 0),
];

#[test]
fn post_init_with_ltr_matches_linux_sequence() {
    let mut io = post_init_model();
    post_init(&mut io, true).unwrap();
    let mut want = vec![
        // rtw89_pci_ltr_set_v1(true) (pci.c:3188): set LTR_WD_NOEMP_CHK_V1 |
        // LTR_HW_EN, idle timer 3.2 ms, RX0/RX1 thresholds 0x28,
        Op::W32(0x8410, 0x3),
        Op::W32(0x8410, 0x703),
        Op::W32(0x8414, 0x28),
        Op::W32(0x8414, 0x0028_0028),
        // dec_ctrl | LTR_HW_DEC_EN with SPACE_IDX_V1 = 500 us, the latencies.
        Op::W32(0x1600, 0x16),
        Op::W32(0x1610, 0x9003_9003),
        Op::W32(0x1604, 0x880B_880B),
    ];
    want.extend(RELEASE_DMA);
    assert_eq!(io.writes(), want);
    // Every LTR register is read and checked before anything is written.
    assert_eq!(
        io.trace[..5],
        [
            Op::R32(0x8410),
            Op::R32(0x8414),
            Op::R32(0x1600),
            Op::R32(0x1610),
            Op::R32(0x1604)
        ]
    );
}

#[test]
fn post_init_without_ltr_only_releases_dma() {
    let mut io = post_init_model();
    post_init(&mut io, false).unwrap();
    assert!(
        !io.trace
            .iter()
            .any(|op| matches!(op, Op::R32(0x8410 | 0x8414 | 0x1600 | 0x1610 | 0x1604)))
    );
    assert_eq!(io.writes(), RELEASE_DMA);
}

#[test]
fn post_init_refuses_ltr_error_value() {
    let refused = Err(AxError::Mac("LTR register reads an error value"));
    let mut io = post_init_model();
    io.set32(0x1600, 0xEAEA_EAEA);
    assert_eq!(post_init(&mut io, true), refused);
    assert!(io.writes().is_empty());
    // rtw89_pci_ltr_set_v1 returns at the first error value.
    assert_eq!(
        io.trace,
        [Op::R32(0x8410), Op::R32(0x8414), Op::R32(0x1600)]
    );
    // All ones (pci.h:1731) is refused the same way, on the last register too.
    let mut io = post_init_model();
    io.set32(0x1604, 0xFFFF_FFFF);
    assert_eq!(post_init(&mut io, true), refused);
    assert!(io.writes().is_empty());
}

/// The registers `deinit` reads and writes, as `post_init` with LTR left
/// them: PCI wake set, LTR on (idle timer 3.2 ms, thresholds 0x28,
/// `dec_ctrl` = HW_DEC_EN | SPACE_IDX 500 us), the HCI DMA enabled.
fn deinit_model() -> RegisterModel {
    let mut io = RegisterModel::new();
    io.set32(0x0074, 0x0000_0021);
    io.set32(0x8410, 0x0000_0703);
    io.set32(0x8414, 0x0028_0028);
    io.set32(0x1600, 0x0000_0016);
    io.set32(0x1000, 0x0000_8080);
    io
}

/// `rtw89_pci_ctrl_dma_all(false)` (`pci.c:2114`) on 0x8080, then
/// `rtw89_pci_clr_idx_all_ax` (`pci.c:2867`) from zeroed index clears.
const STOP_DMA_AND_INDICES: [Op; 5] = [
    Op::W32(0x1000, 0x0002_8080),
    Op::W32(0x1000, 0x0002_0000),
    Op::W32(0x1014, 0x07FF),
    Op::W32(0x11C4, 0x3),
    Op::W32(0x1200, 0x3),
];

#[test]
fn deinit_with_ltr_matches_rtw89_pci_ops_deinit() {
    let (_m, rings, _) = running();
    let mut io = deinit_model();
    assert_eq!(deinit(&mut io, &rings, true), Ok(()));
    let mut want = vec![
        // rtw89_pci_power_wake_ax(false) (pci.c:2733): BIT_WAKE_CTRL cleared.
        Op::W32(0x0074, 0x0000_0001),
        // rtw89_pci_ltr_set_v1(false) (pci.c:3188): no LTR_CTRL_0 set; the
        // idle timer and thresholds rewritten as they are,
        Op::W32(0x8410, 0x0000_0703),
        Op::W32(0x8414, 0x0028_0028),
        Op::W32(0x8414, 0x0028_0028),
        // dec_ctrl without LTR_EN_BITS, with LTR_IDX_DRV = IDLE (3) and
        // LTR_REQ_DRV, SPACE_IDX_V1 still 500 us: 0x16 -> 0xE2.
        Op::W32(0x1600, 0x0000_00E2),
        Op::W32(0x1610, 0x9003_9003),
        Op::W32(0x1604, 0x880B_880B),
    ];
    want.extend(STOP_DMA_AND_INDICES);
    assert_eq!(io.writes(), want);
    // The hardware indices restart, so CH12 needs a fresh pre-init.
    assert!(rings.ch12_stopped());
}

#[test]
fn deinit_without_ltr_touches_no_ltr_register() {
    let (_m, rings, _) = running();
    let mut io = deinit_model();
    assert_eq!(deinit(&mut io, &rings, false), Ok(()));
    assert!(
        !io.trace
            .iter()
            .any(|op| matches!(op, Op::R32(0x8410 | 0x8414 | 0x1600 | 0x1610 | 0x1604)))
    );
    let mut want = vec![Op::W32(0x0074, 0x0000_0001)];
    want.extend(STOP_DMA_AND_INDICES);
    assert_eq!(io.writes(), want);
}

#[test]
fn deinit_still_stops_dma_after_an_ltr_error_value() {
    // rtw89_pci_ops_deinit ignores ltr_set's -EINVAL and goes on.
    let (_m, rings, _) = running();
    let mut io = deinit_model();
    io.set32(0x1600, 0xEAEA_EAEA);
    assert_eq!(
        deinit(&mut io, &rings, true),
        Err(AxError::Mac("LTR register reads an error value"))
    );
    let mut want = vec![Op::W32(0x0074, 0x0000_0001)];
    want.extend(STOP_DMA_AND_INDICES);
    assert_eq!(io.writes(), want);
    assert!(rings.ch12_stopped());
}

/// Every later read of RXQ's IDX (0x1218) reports hardware index `hw`.
fn hardware_index(io: &mut RegisterModel, hw: u32) {
    io.on_read(0x1218, Box::new(move |_m: &mut RegisterModel| hw << 16));
}

/// RXQ entry `entry`'s buffer, as the device sees it: RXQ is ring 0, so
/// its entries are the first `RX_ENTRIES_RXQ` buffers mapped.
pub(crate) fn rxq_buffer(entry: usize) -> u64 {
    assert!(entry < RX_ENTRIES_RXQ);
    u64::from(RX_BUFFER0) + 0x10000 * entry as u64
}

/// Poll RXQ once, keeping a copy of every buffer delivered.
fn drain(
    rings: &mut Rings,
    io: &mut RegisterModel,
    seen: &mut Vec<Vec<u8>>,
) -> Result<usize, AxError> {
    // SAFETY: every test polls the same `RegisterModel` that `start`
    // programmed with these rings, under the same model-only contract.
    unsafe {
        rings.rxq().poll(io, &mut |buffer| {
            seen.push(buffer.to_vec());
            Ok(())
        })
    }
}

fn syncs(m: &Arc<Mutex<Model>>) -> (usize, usize) {
    let s = m.lock().unwrap();
    (s.syncs, s.cpu_syncs)
}

#[test]
fn rxq_consumer_delivers_filled_entries() {
    let (m, mut rings, mut io) = running();
    let mut first: Vec<u8> = (0..RX_BUFFER_BYTES).map(|i| (i * 7) as u8).collect();
    let mut second: Vec<u8> = (0..RX_BUFFER_BYTES).map(|i| (i * 13 + 1) as u8).collect();
    // WRITE_SIZE (BD info [13:0]): past the buffer, so all of it; then 100.
    first[..4].copy_from_slice(&0x3FFFu32.to_le_bytes());
    second[..4].copy_from_slice(&100u32.to_le_bytes());
    {
        let state = m.lock().unwrap();
        device_write(&state, rxq_buffer(0), &first);
        device_write(&state, rxq_buffer(1), &second);
    }
    hardware_index(&mut io, 2);
    let (before, cpu_before) = syncs(&m);
    let mut seen = Vec::new();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(2));
    assert_eq!(seen.len(), 2);
    assert!(seen[0] == first && seen[1] == second[..100]);
    assert_eq!(io.trace, [Op::R32(0x1218), Op::W16(0x1218, 2)]);
    // Each buffer is taken for the CPU once and handed back once.
    let (after, cpu_after) = syncs(&m);
    assert_eq!((after - before, cpu_after - cpu_before), (2, 2));
}

#[test]
fn rxq_consumer_wraps_and_writes_back_host_index() {
    let (m, mut rings, mut io) = running();
    {
        let state = m.lock().unwrap();
        for entry in 0..RX_ENTRIES_RXQ {
            device_write(
                &state,
                rxq_buffer(entry),
                &[(entry as u8).wrapping_add(1); 16],
            );
        }
    }
    let mut counts = Vec::new();
    let mut seen = Vec::new();
    for hw in [254, 1, 1] {
        hardware_index(&mut io, hw);
        counts.push(drain(&mut rings, &mut io, &mut seen).unwrap());
    }
    assert_eq!(counts, [254, 3, 0]);
    // Entries 0-253, then 254, 255 and (wrapped) 0; markers are a byte.
    let order: Vec<u8> = seen.iter().map(|buffer| buffer[0]).collect();
    let mut want: Vec<u8> = (1..=255).collect();
    want.extend([0, 1]);
    assert_eq!(order, want);
    assert_eq!(io.writes(), [Op::W16(0x1218, 254), Op::W16(0x1218, 1)]);
    // The third poll only read the index.
    assert_eq!(io.trace[io.trace.len() - 1], Op::R32(0x1218));
}

#[test]
fn rxq_index_past_ring_is_refused() {
    let (m, mut rings, mut io) = running();
    // The largest index IDX [27:16] can carry.
    hardware_index(&mut io, 0xFFF);
    let before = syncs(&m);
    let mut seen = Vec::new();
    assert_eq!(
        drain(&mut rings, &mut io, &mut seen),
        Err(AxError::Rx("RXQ hardware index out of range"))
    );
    assert!(seen.is_empty());
    assert_eq!(io.trace, [Op::R32(0x1218)]);
    assert_eq!(syncs(&m), before);
    // A dead function's all-ones read is refused as such (spec S9 step 3).
    io.on_read(0x1218, Box::new(|_m: &mut RegisterModel| u32::MAX));
    assert_eq!(
        drain(&mut rings, &mut io, &mut seen),
        Err(AxError::AllOnes { register: 0x1218 })
    );
    assert!(io.writes().is_empty());
}

/// Put entry `entry`'s marker byte (`entry + 1`, a byte) at the start of
/// each RXQ buffer.
fn mark_every_rxq_entry(m: &Arc<Mutex<Model>>) {
    let state = m.lock().unwrap();
    for entry in 0..RX_ENTRIES_RXQ {
        device_write(
            &state,
            rxq_buffer(entry),
            &[(entry as u8).wrapping_add(1); 4],
        );
    }
}

#[test]
fn rxq_full_counts_at_255() {
    let (m, mut rings, mut io) = running();
    mark_every_rxq_entry(&m);
    assert_eq!(rings.rxq_full_batches(), 0);
    // Host index 0, hardware index 255: RX_ENTRIES_RXQ - 1 entries filled,
    // the ring full (`rx_ring_eq_is_full` false).
    hardware_index(&mut io, 255);
    let mut seen = Vec::new();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(255));
    let order: Vec<u8> = seen.iter().map(|buffer| buffer[0]).collect();
    assert_eq!(order, (1..=255).collect::<Vec<u8>>());
    assert_eq!(io.trace, [Op::R32(0x1218), Op::W16(0x1218, 255)]);
    assert_eq!(rings.rxq_full_batches(), 1);
    // Nothing new: nothing delivered, nothing counted.
    io.trace.clear();
    seen.clear();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(0));
    assert!(seen.is_empty());
    assert_eq!(io.trace, [Op::R32(0x1218)]);
    assert_eq!(rings.rxq_full_batches(), 1);
    // A batch of 254 (host 255, hardware 253) is not a full ring; 31, the
    // old ring's full mark, is not either.
    hardware_index(&mut io, 253);
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(254));
    assert_eq!(rings.rxq_full_batches(), 1);
    hardware_index(&mut io, (253 + 31) % 256);
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(31));
    assert_eq!(rings.rxq_full_batches(), 1);
    // The count saturates (host 28, hardware 27: full again).
    rings.rxq_full.set(u16::MAX);
    hardware_index(&mut io, 27);
    seen.clear();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(255));
    assert_eq!(rings.rxq_full_batches(), u16::MAX);
}

#[test]
fn rxq_wraps_past_entry_255() {
    let (m, mut rings, mut io) = running();
    mark_every_rxq_entry(&m);
    let mut seen = Vec::new();
    hardware_index(&mut io, 254);
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(254));
    // Host 254, hardware 2: entries 254, 255, 0, 1.
    io.trace.clear();
    seen.clear();
    hardware_index(&mut io, 2);
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(4));
    let order: Vec<u8> = seen.iter().map(|buffer| buffer[0]).collect();
    assert_eq!(order, [255, 0, 1, 2]);
    assert_eq!(io.trace, [Op::R32(0x1218), Op::W16(0x1218, 2)]);
    assert_eq!(rings.rxq_full_batches(), 0);
}

#[test]
fn hardware_index_256_is_refused() {
    let (m, mut rings, mut io) = running();
    mark_every_rxq_entry(&m);
    hardware_index(&mut io, 256);
    let before = syncs(&m);
    let mut seen = Vec::new();
    assert_eq!(
        drain(&mut rings, &mut io, &mut seen),
        Err(AxError::Rx("RXQ hardware index out of range"))
    );
    // No buffer taken, given back or delivered; no host index written.
    assert!(seen.is_empty());
    assert_eq!(syncs(&m), before);
    assert_eq!(io.trace, [Op::R32(0x1218)]);
    assert_eq!(rings.rxq_full_batches(), 0);
}

#[test]
fn rxq_consumer_error_ends_the_batch_after_writing_back() {
    let (m, mut rings, mut io) = running();
    {
        let state = m.lock().unwrap();
        for entry in 0..3 {
            device_write(
                &state,
                rxq_buffer(entry),
                &[(entry as u8).wrapping_add(1); 4],
            );
        }
    }
    hardware_index(&mut io, 3);
    let (before, _) = syncs(&m);
    let mut seen = Vec::new();
    // SAFETY: as `drain`.
    let polled = unsafe {
        rings.rxq().poll(&mut io, &mut |buffer| {
            seen.push(buffer[0]);
            if seen.len() == 2 {
                Err(AxError::Rx("malformed"))
            } else {
                Ok(())
            }
        })
    };
    assert_eq!(polled, Err(AxError::Rx("malformed")));
    assert_eq!(seen, [1, 2]);
    // As `rtw89_pci_rxbd_deliver_skbs`' err_sync_device path (pci.c:401-403),
    // the refused buffer is handed back and consumed with the ones before it.
    assert_eq!(io.trace, [Op::R32(0x1218), Op::W16(0x1218, 2)]);
    assert_eq!(syncs(&m).0 - before, 2);
    // The next poll starts at the entry after the refused one.
    let mut seen = Vec::new();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(1));
    assert_eq!(seen[0][0], 3);
    assert_eq!(io.writes().last(), Some(&Op::W16(0x1218, 3)));
}

#[test]
fn rxq_buffer_that_cannot_be_taken_stays_unconsumed() {
    let (m, mut rings, mut io) = running();
    {
        let state = m.lock().unwrap();
        for entry in 0..3 {
            device_write(
                &state,
                rxq_buffer(entry),
                &[(entry as u8).wrapping_add(1); 4],
            );
        }
    }
    hardware_index(&mut io, 3);
    {
        let mut state = m.lock().unwrap();
        state.fail_cpu_sync_at = Some(state.cpu_syncs + 2);
    }
    let (before, _) = syncs(&m);
    let mut seen = Vec::new();
    assert_eq!(
        drain(&mut rings, &mut io, &mut seen),
        Err(AxError::Dma("ring memory visibility failed"))
    );
    // Entry 1 was never read nor given back: only entry 0 is returned.
    assert_eq!(seen.len(), 1);
    assert_eq!(syncs(&m).0 - before, 1);
    assert_eq!(io.trace, [Op::R32(0x1218), Op::W16(0x1218, 1)]);
    // It is still the host's, so the next poll delivers it.
    m.lock().unwrap().fail_cpu_sync_at = None;
    seen.clear();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(2));
    let order: Vec<u8> = seen.iter().map(|buffer| buffer[0]).collect();
    assert_eq!(order, [2, 3]);
    assert_eq!(io.writes().last(), Some(&Op::W16(0x1218, 3)));
}

#[test]
fn pre_init_restarts_rxq_at_entry_0() {
    let (m, mut rings, mut io) = running();
    {
        let state = m.lock().unwrap();
        for entry in 0..RX_ENTRIES_RXQ {
            device_write(
                &state,
                rxq_buffer(entry),
                &[(entry as u8).wrapping_add(1); 4],
            );
        }
    }
    let mut seen = Vec::new();
    hardware_index(&mut io, 3);
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(3));
    // pre_init clears the hardware indices (rtw89_pci_clr_idx_all_ax), and
    // rtw89_pci_reset_trx_rings the host's.
    start(&mut io, 2, &rings).unwrap();
    io.trace.clear();
    seen.clear();
    hardware_index(&mut io, 2);
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(2));
    let order: Vec<u8> = seen.iter().map(|buffer| buffer[0]).collect();
    assert_eq!(order, [1, 2]);
    assert_eq!(io.writes(), [Op::W16(0x1218, 2)]);
}

// TX pools, the data and management queues, and RPQ (M3 spec S5.1, S5.2,
// S5.5, S6).

/// `TxPools::install` maps its one pool after the rings' 291 mappings: WD
/// pages first (ACH0-3 256 x 128 bytes each, then CH8's 16), then the
/// payload slots (as many, of 2048 bytes).
const POOLS: u32 = 0x1124_0000;
const POOL_PAGES: usize = 4 * 256 + 16;
const POOL_BYTES: usize = POOL_PAGES * 128 + POOL_PAGES * 2048;

fn pools(m: &Arc<Mutex<Model>>) -> TxPools {
    // SAFETY: as `install`: the model domain isolates only the modeled
    // requester, its pages are kept test RAM, and this test is the only
    // worker.
    unsafe { TxPools::install(&Source, Arc::new(Domain(m.clone()))) }.unwrap()
}

/// The device address of channel slot `slot`'s WD page `page`.
fn wd_page(slot: usize, page: usize) -> u64 {
    u64::from(POOLS) + ((slot * 256 + page) * 128) as u64
}

/// The device address of channel slot `slot`'s payload slot `page`.
fn payload_slot(slot: usize, page: usize) -> u64 {
    u64::from(POOLS) + (POOL_PAGES * 128 + (slot * 256 + page) * 2048) as u64
}

/// TX ring `ring`'s BD entry `entry`, as the device sees it.
fn tx_bd(ring: usize, entry: usize) -> u64 {
    u64::from(TX_POOL) + (TX_RING_AT[ring] + entry * 8) as u64
}

/// A WD body and info of bytes counting up from `fill`, with WD_INFO_EN
/// (body dword 0 b22) set, as on every frame Linux sends.
fn head(fill: u8) -> [u8; WD_HEAD_BYTES] {
    let mut head = core::array::from_fn(|i| fill.wrapping_add(i as u8));
    head[2] |= 0x40;
    head
}

/// [`submit`], then the kick: M3's submit, which kicked each frame.
fn transmit(
    rings: &mut Rings,
    pools: &mut TxPools,
    io: &mut RegisterModel,
    ch: TxChannel,
    head: &[u8; WD_HEAD_BYTES],
    payload: &[u8],
) -> Result<Option<u8>, AxError> {
    let page = submit(rings, pools, io, ch, head, payload)?;
    kick(rings, pools, io, ch)?;
    Ok(page)
}

fn submit(
    rings: &mut Rings,
    pools: &mut TxPools,
    io: &mut RegisterModel,
    ch: TxChannel,
    head: &[u8; WD_HEAD_BYTES],
    payload: &[u8],
) -> Result<Option<u8>, AxError> {
    // The submit writes no register: `io` is the window the kick will use.
    let _ = io;
    // SAFETY: every test submits on the same `RegisterModel` that `start`
    // programmed with these rings, under the same model-only contract.
    unsafe { rings.txq(pools, ch).submit(head, payload) }
}

fn kick(
    rings: &mut Rings,
    pools: &mut TxPools,
    io: &mut RegisterModel,
    ch: TxChannel,
) -> Result<bool, AxError> {
    // SAFETY: as `submit`.
    unsafe { rings.txq(pools, ch).kick(io) }
}

fn reclaim(
    rings: &mut Rings,
    pools: &mut TxPools,
    io: &mut RegisterModel,
    ch: TxChannel,
) -> Result<(), AxError> {
    // SAFETY: as `transmit`.
    unsafe { rings.txq(pools, ch).reclaim(io) }
}

fn free(rings: &mut Rings, pools: &mut TxPools, ch: TxChannel) -> usize {
    rings.txq(pools, ch).free()
}

/// Every later read of TX ring `ring`'s IDX reports hardware index `hw`
/// over the host index last written.
fn tx_hardware_index(io: &mut RegisterModel, ring: usize, hw: u32) {
    let idx = [0x1058, 0x105C, 0x1060, 0x1064, 0, 0, 0, 0, 0x1078][ring];
    io.on_read(
        idx,
        Box::new(move |m: &mut RegisterModel| hw << 16 | (m.get32(idx) & 0xFFF)),
    );
}

fn running_with_pools() -> (Arc<Mutex<Model>>, Rings, TxPools, RegisterModel) {
    let (m, rings, io) = running();
    let pools = pools(&m);
    (m, rings, pools, io)
}

fn page(pools: &TxPools, ch: TxChannel, seq: usize) -> Page {
    pools.channels[ch.slot()].pages[seq]
}

fn pages_free(pools: &TxPools, ch: TxChannel) -> usize {
    (0..pages_of(ch.slot()))
        .filter(|&seq| !page(pools, ch, seq).in_flight)
        .count()
}

#[test]
fn tx_error_prints_its_subsystem() {
    assert_eq!(alloc::format!("{}", AxError::Tx("x")), "tx: x");
}

#[test]
fn tx_channels_follow_qsel_and_ring() {
    let all = [
        (0x00, TxChannel::Ach0, 0),
        (0x01, TxChannel::Ach1, 1),
        (0x02, TxChannel::Ach2, 2),
        (0x03, TxChannel::Ach3, 3),
        (0x12, TxChannel::Ch8, 8),
    ];
    for (qsel, ch, ring) in all {
        assert_eq!(TxChannel::of_qsel(qsel), Some(ch));
        assert_eq!(ch.ring(), ring);
    }
    // BE_1, VO_3, B0_BCN, B0_HI, B0_NOPS, B0_MGMT_FAST and anything past
    // QSEL's 5 bits map to no pool here.
    for qsel in [0x04, 0x0F, 0x10, 0x11, 0x13, 0x14, 0x1F, 0x20, 0xFF] {
        assert_eq!(TxChannel::of_qsel(qsel), None);
    }
    assert_eq!(
        (
            WD_PAGES,
            WD_PAGE_BYTES,
            PAYLOAD_BYTES,
            MAX_TX_FRAME,
            WD_HEAD_BYTES
        ),
        (256, 128, 2048, 2044, 56)
    );
    assert_eq!(TX_DONE, 0);
}

#[test]
fn pools_take_one_to_device_allocation_of_553_pages() {
    let m = Arc::new(Mutex::new(Model::default()));
    let pools = pools(&m);
    let (count, bytes, direction) = {
        let state = m.lock().unwrap();
        let (iova, (pages, _)) = state.mappings.iter().next().unwrap();
        (state.mappings.len(), pages.bytes(), state.directions[iova])
    };
    assert_eq!(count, 1);
    // 1 040 pages of 128 bytes and slots of 2048: 2 263 040 bytes, 553 whole
    // pages.
    assert_eq!(POOL_BYTES, 2_263_040);
    assert_eq!(super::POOL_BYTES, POOL_BYTES);
    assert_eq!(bytes, 553 * 4096);
    assert_eq!(direction, Direction::ToDevice);
    // Never handed to the device: the mapping is returned.
    drop(pools);
    assert!(m.lock().unwrap().mappings.is_empty());
}

#[test]
fn submit_writes_wd_tail_bd() {
    let (m, mut rings, mut pools, mut io) = running_with_pools();
    let payload: Vec<u8> = (0..30).map(|i| 0xA0 ^ i as u8).collect();
    let (before, cpu_before) = syncs(&m);
    assert_eq!(
        submit(
            &mut rings,
            &mut pools,
            &mut io,
            TxChannel::Ach0,
            &head(1),
            &payload
        ),
        Ok(Some(0))
    );
    // Written, not kicked: no register access (`rtw89_pci_tx_write`).
    assert!(io.trace.is_empty());
    assert!(rings.txq(&mut pools, TxChannel::Ach0).pending_kick());
    let (wd, bd, slot) = {
        let state = m.lock().unwrap();
        (
            device_read(&state, wd_page(0, 0), 128),
            device_read(&state, tx_bd(0, 0), 8),
            device_read(&state, payload_slot(0, 0), 30),
        )
    };
    // Body and info as given, then wp_info: seq0 = page 0 | VALID b15.
    assert_eq!(wd[..56], head(1));
    assert_eq!(wd[56..64], [0x00, 0x80, 0, 0, 0, 0, 0, 0]);
    // addr_info v1: LEN [10:0] | LS b15, then the slot's dma[15:0], [31:16].
    let dma = payload_slot(0, 0) as u32;
    let mut addr = Vec::new();
    addr.extend((30u16 | 0x8000).to_le_bytes());
    addr.extend((dma as u16).to_le_bytes());
    addr.extend(((dma >> 16) as u16).to_le_bytes());
    assert_eq!(wd[64..70], addr[..]);
    assert_eq!(wd[70..], [0; 58]);
    assert_eq!(bd, encode_txbd(70, wd_page(0, 0)));
    assert_eq!(slot, payload);
    // WD page and payload slot, then the TXBD, made visible before the kick.
    assert_eq!(syncs(&m), (before + 3, cpu_before));
    assert!(page(&pools, TxChannel::Ach0, 0).in_flight);
    // The kick writes the host index.
    assert_eq!(
        kick(&mut rings, &mut pools, &mut io, TxChannel::Ach0),
        Ok(true)
    );
    assert_eq!(io.trace, [Op::W16(0x1058, 1)]);
    // The device may now read the pool: it is never unmapped.
    drop(pools);
    assert_eq!(m.lock().unwrap().mappings.len(), MAPPINGS + 1);
}

#[test]
fn ch8_kicks_0x1078() {
    let (m, mut rings, mut pools, mut io) = running_with_pools();
    assert_eq!(
        transmit(
            &mut rings,
            &mut pools,
            &mut io,
            TxChannel::Ch8,
            &head(9),
            &[0xB0; 24]
        ),
        Ok(Some(0))
    );
    assert_eq!(io.trace, [Op::W16(0x1078, 1)]);
    // CH8 is pool slot 4 and TX ring 8; ACH0's ring is untouched.
    let (bd, wd, slot, ach0) = {
        let state = m.lock().unwrap();
        (
            device_read(&state, tx_bd(8, 0), 8),
            device_read(&state, wd_page(4, 0), 56),
            device_read(&state, payload_slot(4, 0), 24),
            device_read(&state, tx_bd(0, 0), 8),
        )
    };
    assert_eq!(bd, encode_txbd(70, wd_page(4, 0)));
    assert_eq!(wd, head(9));
    assert_eq!(slot, [0xB0; 24]);
    assert_eq!(ach0, [0; 8]);
}

#[test]
fn submit_refuses_frames_past_2044() {
    let (m, mut rings, mut pools, mut io) = running_with_pools();
    let before = syncs(&m);
    for len in [2045, 4096] {
        assert_eq!(
            transmit(
                &mut rings,
                &mut pools,
                &mut io,
                TxChannel::Ach0,
                &head(0),
                &vec![1; len]
            ),
            Err(AxError::Tx("frame longer than one TX address entry"))
        );
    }
    assert_eq!(
        transmit(
            &mut rings,
            &mut pools,
            &mut io,
            TxChannel::Ach0,
            &head(0),
            &[]
        ),
        Err(AxError::Tx("empty frame"))
    );
    assert!(io.trace.is_empty());
    assert_eq!(syncs(&m), before);
    assert_eq!(pages_free(&pools, TxChannel::Ach0), 256);
    // 2044 bytes fit one address entry.
    assert_eq!(
        transmit(
            &mut rings,
            &mut pools,
            &mut io,
            TxChannel::Ach0,
            &head(0),
            &vec![2; 2044]
        ),
        Ok(Some(0))
    );
    let wd = device_read(&m.lock().unwrap(), wd_page(0, 0), 70);
    assert_eq!(wd[64..66], (2044u16 | 0x8000).to_le_bytes());
}

#[test]
fn submit_refuses_a_head_without_wd_info() {
    let (m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ach0;
    let refused = Err(AxError::Tx("WD head without its WD info"));
    // wp_info always follows at byte 56; without WD_INFO_EN Linux puts it
    // at 32 (pci.c:1524-1527), so such a head is refused untouched, however
    // many other dword-0 bits it sets.
    let mut bare = head(1);
    bare[2] &= !0x40;
    let mut all_but = [0xFF; WD_HEAD_BYTES];
    all_but[2] = !0x40;
    let before = syncs(&m);
    for head in [bare, all_but, [0; WD_HEAD_BYTES]] {
        assert_eq!(
            transmit(&mut rings, &mut pools, &mut io, ch, &head, &[1; 8]),
            refused
        );
    }
    assert!(io.trace.is_empty());
    assert_eq!(syncs(&m), before);
    assert_eq!(pages_free(&pools, ch), 256);
    assert_eq!(free(&mut rings, &mut pools, ch), 255);
    // WD_INFO_EN alone is enough.
    let mut only = [0; WD_HEAD_BYTES];
    only[2] = 0x40;
    assert_eq!(
        transmit(&mut rings, &mut pools, &mut io, ch, &only, &[1]),
        Ok(Some(0))
    );
    assert_eq!(io.trace, [Op::W16(0x1058, 1)]);
}

#[test]
fn submit_refuses_before_pre_init() {
    let m = Arc::new(Mutex::new(Model::default()));
    let mut rings = install(&m);
    let mut pools = pools(&m);
    let mut io = settled();
    assert_eq!(
        transmit(
            &mut rings,
            &mut pools,
            &mut io,
            TxChannel::Ach0,
            &head(0),
            &[1]
        ),
        Err(AxError::Tx("TX rings are stopped"))
    );
    assert!(io.trace.is_empty());
    // Never handed over: both are returned.
    drop(pools);
    drop(rings);
    assert!(m.lock().unwrap().mappings.is_empty());
}

#[test]
fn submit_refuses_when_pages_are_out() {
    let (m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ach1;
    assert_eq!(free(&mut rings, &mut pools, ch), 255);
    for seq in 0..=254u8 {
        assert_eq!(
            transmit(&mut rings, &mut pools, &mut io, ch, &head(seq), &[seq; 8]),
            Ok(Some(seq))
        );
    }
    // 255 in flight: the BD ring keeps one entry empty (pci.c:1222-1231).
    assert_eq!(free(&mut rings, &mut pools, ch), 0);
    io.trace.clear();
    let before = syncs(&m);
    assert_eq!(
        transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[1]),
        Ok(None)
    );
    assert!(io.trace.is_empty());
    assert_eq!(syncs(&m), before);
    // The DMA engine reads all 255, but RPQ stays silent: every BD is free
    // again and only the page never used remains.
    tx_hardware_index(&mut io, 1, 255);
    reclaim(&mut rings, &mut pools, &mut io, ch).unwrap();
    assert_eq!(free(&mut rings, &mut pools, ch), 1);
    assert_eq!(
        transmit(&mut rings, &mut pools, &mut io, ch, &head(255), &[255; 8]),
        Ok(Some(255))
    );
    io.trace.clear();
    let before = syncs(&m);
    assert_eq!(free(&mut rings, &mut pools, ch), 0);
    assert_eq!(
        transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[1]),
        Ok(None)
    );
    assert!(io.trace.is_empty());
    assert_eq!(syncs(&m), before);
    // No page was reused: each still holds the frame it was given.
    let state = m.lock().unwrap();
    for seq in 0..256 {
        assert_eq!(device_read(&state, wd_page(1, seq), 1), [seq as u8]);
        assert_eq!(device_read(&state, payload_slot(1, seq), 8), [seq as u8; 8]);
    }
}

#[test]
fn ch8_keeps_16_entries_and_pages() {
    // M4 spec S9.2: management frames go one at a time, so CH8 keeps M3's
    // 16 TXBDs and as many pages.
    let (_m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ch8;
    assert_eq!(free(&mut rings, &mut pools, ch), 15);
    assert_eq!(pages_free(&pools, ch), 16);
    for seq in 0..15u8 {
        assert_eq!(
            transmit(&mut rings, &mut pools, &mut io, ch, &head(seq), &[seq]),
            Ok(Some(seq))
        );
    }
    assert_eq!(free(&mut rings, &mut pools, ch), 0);
    // A report for a seq past CH8's 16 pages is refused.
    assert!(!pools.release(ch, 16));
    assert!(!pools.release(ch, 255));
}

#[test]
fn page_returns_only_after_both_bd_and_report() {
    let (_m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ach2;
    for seq in 0..2u8 {
        assert_eq!(
            transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[seq]),
            Ok(Some(seq))
        );
    }
    assert_eq!(free(&mut rings, &mut pools, ch), 253);
    // Page 0: the report first, then the hardware index past it.
    assert!(pools.release(ch, 0));
    assert!(page(&pools, ch, 0).reported && page(&pools, ch, 0).in_flight);
    assert_eq!(pages_free(&pools, ch), 254);
    tx_hardware_index(&mut io, 2, 1);
    reclaim(&mut rings, &mut pools, &mut io, ch).unwrap();
    assert_eq!(page(&pools, ch, 0), Page::default());
    assert_eq!(pages_free(&pools, ch), 255);
    // BDs: one still outstanding of 255.
    assert_eq!(free(&mut rings, &mut pools, ch), 254);
    // Page 1: the hardware index first, then the report.
    tx_hardware_index(&mut io, 2, 2);
    reclaim(&mut rings, &mut pools, &mut io, ch).unwrap();
    assert!(page(&pools, ch, 1).in_flight && page(&pools, ch, 1).bd_done);
    assert_eq!(pages_free(&pools, ch), 255);
    assert!(pools.release(ch, 1));
    assert_eq!(page(&pools, ch, 1), Page::default());
    assert_eq!(pages_free(&pools, ch), 256);
    assert_eq!(free(&mut rings, &mut pools, ch), 255);
    // A second report for a returned page, a page never sent, a seq past
    // CH8's pool and another channel's report change nothing.
    assert!(!pools.release(ch, 1));
    assert!(!pools.release(ch, 7));
    assert!(!pools.release(TxChannel::Ch8, 16));
    assert!(!pools.release(TxChannel::Ach0, 0));
    // The next frame takes the lowest returned page.
    assert_eq!(
        transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[9]),
        Ok(Some(0))
    );
}

#[test]
fn a_report_twice_does_not_return_a_page_in_flight() {
    let (_m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ach3;
    transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[1]).unwrap();
    assert!(pools.release(ch, 0));
    assert!(!pools.release(ch, 0));
    assert!(page(&pools, ch, 0).in_flight);
    assert_eq!(pages_free(&pools, ch), 255);
}

#[test]
fn reclaim_reads_nothing_with_no_bd_outstanding() {
    let (_m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ach0;
    reclaim(&mut rings, &mut pools, &mut io, ch).unwrap();
    assert!(io.trace.is_empty());
    transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[1]).unwrap();
    io.trace.clear();
    // Not consumed yet: one read, nothing written.
    reclaim(&mut rings, &mut pools, &mut io, ch).unwrap();
    assert_eq!(io.trace, [Op::R32(0x1058)]);
    assert!(!page(&pools, ch, 0).bd_done);
}

#[test]
fn reclaim_refuses_an_index_the_host_never_gave() {
    let (m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ach0;
    for _ in 0..3 {
        transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[1]).unwrap();
    }
    io.trace.clear();
    let before = syncs(&m);
    for (hw, error) in [
        (4, AxError::Tx("TX hardware index past the host index")),
        (16, AxError::Tx("TX hardware index past the host index")),
        (255, AxError::Tx("TX hardware index past the host index")),
        (256, AxError::Tx("TX hardware index out of range")),
        (0xFFF, AxError::Tx("TX hardware index out of range")),
    ] {
        tx_hardware_index(&mut io, 0, hw);
        assert_eq!(reclaim(&mut rings, &mut pools, &mut io, ch), Err(error));
    }
    io.on_read(0x1058, Box::new(|_m: &mut RegisterModel| u32::MAX));
    assert_eq!(
        reclaim(&mut rings, &mut pools, &mut io, ch),
        Err(AxError::AllOnes { register: 0x1058 })
    );
    // Nothing was taken back or written.
    assert_eq!(syncs(&m), before);
    assert!(io.writes().is_empty());
    assert!((0..3).all(|seq| !page(&pools, ch, seq).bd_done));
    // A sane index still reclaims, each BD taken back for the CPU.
    tx_hardware_index(&mut io, 0, 3);
    reclaim(&mut rings, &mut pools, &mut io, ch).unwrap();
    assert!((0..3).all(|seq| page(&pools, ch, seq).bd_done));
    assert_eq!(syncs(&m), (before.0, before.1 + 3));
}

#[test]
fn submit_wraps_the_bd_ring_and_reuses_entries_only_once_passed() {
    // CH8's 16-entry ring; ACH0's 256 wrap in `a_data_ring_wraps_at_256`.
    let (m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ch8;
    // 20 frames, each read and reported as soon as it is sent.
    for n in 0..20u32 {
        let seq = transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[n as u8])
            .unwrap()
            .unwrap();
        assert_eq!(seq, 0);
        tx_hardware_index(&mut io, 8, (n + 1) % 16);
        reclaim(&mut rings, &mut pools, &mut io, ch).unwrap();
        assert!(pools.release(ch, seq));
    }
    let kicks: Vec<u16> = io
        .trace
        .iter()
        .filter_map(|op| match op {
            Op::W16(0x1078, v) => Some(*v),
            _ => None,
        })
        .collect();
    let mut want: Vec<u16> = (1..16).collect();
    want.extend([0, 1, 2, 3, 4]);
    assert_eq!(kicks, want);
    // Entry 3 was rewritten by frame 19, pointing at page 0 again.
    let (bd, slot) = {
        let state = m.lock().unwrap();
        (
            device_read(&state, tx_bd(8, 3), 8),
            device_read(&state, payload_slot(4, 0), 1),
        )
    };
    assert_eq!(bd, encode_txbd(70, wd_page(4, 0)));
    assert_eq!(slot, [19]);
}

#[test]
fn pre_init_restarts_the_data_rings_and_keeps_pages_out() {
    let (_m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ach0;
    transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[1]).unwrap();
    start(&mut io, 2, &rings).unwrap();
    io.trace.clear();
    // The ring restarts at entry 0; page 0 may still be the device's, so it
    // is never handed out again.
    assert_eq!(free(&mut rings, &mut pools, ch), 255);
    assert_eq!(
        transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[2]),
        Ok(Some(1))
    );
    assert_eq!(io.trace, [Op::W16(0x1058, 1)]);
    assert!(page(&pools, ch, 0).in_flight);
}

#[test]
fn deinit_stops_the_data_rings() {
    let (_m, mut rings, mut pools, mut io) = running_with_pools();
    deinit(&mut io, &rings, false).unwrap();
    io.trace.clear();
    assert_eq!(
        transmit(
            &mut rings,
            &mut pools,
            &mut io,
            TxChannel::Ach0,
            &head(0),
            &[1]
        ),
        Err(AxError::Tx("TX rings are stopped"))
    );
    assert!(io.trace.is_empty());
}

/// CH8 (16 entries) with its host index wrapped and three TXBDs outstanding: 14 frames
/// sent, 13 of them read and reported (their pages back), then 2 more,
/// which take pages 0 and 1 at entries 14 and 15. Entries 13-15 (pages 13,
/// 0 and 1) are the device's; the host index is 0 and the last hardware
/// index read 13.
fn wrapped_with_three_outstanding() -> (Arc<Mutex<Model>>, Rings, TxPools, RegisterModel) {
    let (m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ch8;
    for seq in 0..14u8 {
        assert_eq!(
            transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[seq]),
            Ok(Some(seq))
        );
    }
    tx_hardware_index(&mut io, 8, 13);
    reclaim(&mut rings, &mut pools, &mut io, ch).unwrap();
    for seq in 0..13 {
        assert!(pools.release(ch, seq));
    }
    for seq in 0..2u8 {
        assert_eq!(
            transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[0x40 | seq]),
            Ok(Some(seq))
        );
    }
    assert_eq!(io.trace.last(), Some(&Op::W16(0x1078, 0)));
    (m, rings, pools, io)
}

/// Pages 13, 0 and 1 of CH8 are in flight with no TXBD read proven.
fn three_unproven(pools: &TxPools) -> bool {
    [13, 0, 1].iter().all(|&seq| {
        let page = page(pools, TxChannel::Ch8, seq);
        page.in_flight && !page.bd_done
    })
}

#[test]
fn reclaim_refuses_rings_deinit_stopped() {
    let (m, mut rings, mut pools, mut io) = wrapped_with_three_outstanding();
    let ch = TxChannel::Ch8;
    deinit(&mut io, &rings, false).unwrap();
    // rtw89_pci_clr_idx_all_ax restarted the hardware index at 0, which from
    // a last read of 13 and host index 0 would pass for three TXBDs read.
    tx_hardware_index(&mut io, 8, 0);
    io.trace.clear();
    let before = syncs(&m);
    assert_eq!(
        reclaim(&mut rings, &mut pools, &mut io, ch),
        Err(AxError::Tx("TX rings are stopped"))
    );
    assert!(io.trace.is_empty());
    assert_eq!(syncs(&m), before);
    assert!(three_unproven(&pools));
    // Their reports alone return none of them.
    for seq in [13, 0, 1] {
        assert!(pools.release(ch, seq));
    }
    assert!(three_unproven(&pools));
    assert_eq!(pages_free(&pools, ch), 13);
    // A fresh pre_init restarts the ring with nothing outstanding, and the
    // three pages stay out.
    start(&mut io, 2, &rings).unwrap();
    io.trace.clear();
    reclaim(&mut rings, &mut pools, &mut io, ch).unwrap();
    assert!(io.trace.is_empty());
    assert_eq!(pages_free(&pools, ch), 13);
    assert_eq!(free(&mut rings, &mut pools, ch), 13);
}

#[test]
fn reclaim_refuses_rings_a_failed_pre_init_stopped() {
    let (m, mut rings, mut pools, mut io) = wrapped_with_three_outstanding();
    let ch = TxChannel::Ch8;
    // pre_init fails in rtw89_pci_mode_op (its first R_AX_HAXI_EXP_CTRL
    // read), after rtw89_pci_clr_idx_all_ax zeroed every hardware index and
    // before rtw89_pci_reset_trx_rings programmed any ring.
    io.trace.clear();
    io.fail_reads_of(0x1204);
    assert_eq!(start(&mut io, 2, &rings), Err(READ_FAILED));
    let writes = io.writes();
    let cleared = writes
        .iter()
        .position(|op| *op == Op::W32(0x1200, 0x3))
        .unwrap();
    assert_eq!(
        writes[cleared - 2..=cleared],
        [
            Op::W32(0x1014, 0x07FF),
            Op::W32(0x11C4, 0x3),
            Op::W32(0x1200, 0x3)
        ]
    );
    // Only mode_op's three R_AX_HAXI_INIT_CFG1 steps follow.
    assert_eq!(writes[cleared + 1..], [Op::W32(0x1000, 0x1102_0102); 3]);
    tx_hardware_index(&mut io, 8, 0);
    io.trace.clear();
    let before = syncs(&m);
    assert_eq!(
        reclaim(&mut rings, &mut pools, &mut io, ch),
        Err(AxError::Tx("TX rings are stopped"))
    );
    assert!(io.trace.is_empty());
    assert_eq!(syncs(&m), before);
    assert!(three_unproven(&pools));
}

#[test]
fn a_pre_init_failing_after_the_index_clear_restarts_the_rx_host_indices() {
    let (_m, mut rings, mut io) = running();
    hardware_index(&mut io, 2);
    io.on_read(0x121C, Box::new(|_m: &mut RegisterModel| 3 << 16));
    let mut seen = Vec::new();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(2));
    assert_eq!(drain_rpq(&mut rings, &mut io, &mut seen), Ok(3));
    // As above: rtw89_pci_clr_idx_all_ax ran, then pre_init failed.
    io.fail_reads_of(0x1204);
    assert_eq!(start(&mut io, 2, &rings), Err(READ_FAILED));
    // Both hardware indices restarted at 0. Read against the old host
    // indices that would be 254 RXQ and 29 RPQ entries filled, every one
    // stale, and on RPQ every stale report could return a page since reused.
    hardware_index(&mut io, 0);
    io.on_read(0x121C, Box::new(|_m: &mut RegisterModel| 0));
    io.trace.clear();
    seen.clear();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(0));
    assert_eq!(drain_rpq(&mut rings, &mut io, &mut seen), Ok(0));
    assert!(seen.is_empty());
    assert_eq!(io.trace, [Op::R32(0x1218), Op::R32(0x121C)]);
}

#[test]
fn a_deinit_whose_index_clear_fails_keeps_the_rx_host_indices() {
    let (_m, mut rings, mut io) = running();
    hardware_index(&mut io, 2);
    io.on_read(0x121C, Box::new(|_m: &mut RegisterModel| 3 << 16));
    let mut seen = Vec::new();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(2));
    assert_eq!(drain_rpq(&mut rings, &mut io, &mut seen), Ok(3));
    // rtw89_pci_clr_idx_all_ax fails at its first TX clear, so the RX index
    // clear never runs and the hardware RX indices stay at 2 and 3.
    io.fail_reads_of(0x1014);
    assert_eq!(deinit(&mut io, &rings, false), Err(READ_FAILED));
    assert!(!io.writes().contains(&Op::W32(0x1200, 0x3)));
    assert!(rings.ch12_stopped());
    // Against host indices restarted at 0 that would be 2 RXQ and 3 RPQ
    // entries filled, each already consumed.
    io.trace.clear();
    seen.clear();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(0));
    assert_eq!(drain_rpq(&mut rings, &mut io, &mut seen), Ok(0));
    assert!(seen.is_empty());
    assert_eq!(io.trace, [Op::R32(0x1218), Op::R32(0x121C)]);
}

/// An RPQ buffer as the device writes it: the RX BD info word (FS | LS |
/// WRITE_SIZE), a long RX descriptor with `drv_info` 8-byte units of driver
/// info after it, then one RPP dword per entry of `rpps`.
pub(crate) fn rpq_buffer(drv_info: u32, rpps: &[u32]) -> Vec<u8> {
    let rpp_at = 4 + 32 + drv_info as usize * 8;
    let size = rpp_at + rpps.len() * 4;
    let mut b = vec![0u8; RX_BUFFER_BYTES];
    b[0..4].copy_from_slice(&(1u32 << 15 | 1 << 14 | size as u32).to_le_bytes());
    // d0: LONG_RXD b31, DRV_INFO_SIZE [30:28], RPKT_TYPE 3 [27:24], RPKT_LEN.
    let d0 = 1u32 << 31 | drv_info << 28 | 3 << 24 | (rpps.len() * 4) as u32;
    b[4..8].copy_from_slice(&d0.to_le_bytes());
    b[8..rpp_at].fill(0xEE);
    for (i, rpp) in rpps.iter().enumerate() {
        b[rpp_at + i * 4..rpp_at + i * 4 + 4].copy_from_slice(&rpp.to_le_bytes());
    }
    b
}

/// An RPP dword: SEQ [30:16], TX_STATUS [15:13], QSEL [12:8], MACID [7:0].
pub(crate) fn rpp(seq: u32, status: u32, qsel: u32) -> u32 {
    seq << 16 | status << 13 | qsel << 8 | 0x5A
}

#[test]
fn releases_parse_every_rpp_in_a_buffer() {
    let buffer = rpq_buffer(
        1,
        &[
            rpp(3, 0, 0x12),
            rpp(256, 0, 0x00),
            rpp(15, 1, 0x00),
            rpp(4, 0, 0x04),
            1 << 31 | rpp(0, 3, 0x03),
        ],
    );
    // Seq 256 is past the pools and qsel 4 names no channel: both skipped
    // and counted (spec S6).
    assert_eq!(
        releases(&buffer),
        Ok(Releases {
            reports: vec![
                Release {
                    seq: 3,
                    status: TX_DONE,
                    qsel: 0x12
                },
                Release {
                    seq: 15,
                    status: 1,
                    qsel: 0
                },
                Release {
                    seq: 0,
                    status: 3,
                    qsel: 3
                },
            ],
            invalid: 2,
        })
    );
    // A short descriptor and no driver info: the RPPs start at byte 20.
    let mut short = vec![0u8; 64];
    short[0..4].copy_from_slice(&(1u32 << 15 | 1 << 14 | 28).to_le_bytes());
    short[20..24].copy_from_slice(&rpp(2, 2, 0x01).to_le_bytes());
    short[24..28].copy_from_slice(&rpp(5, 0, 0x02).to_le_bytes());
    // Past WRITE_SIZE: not a report.
    short[28..32].copy_from_slice(&rpp(6, 0, 0x02).to_le_bytes());
    let found = releases(&short).unwrap();
    let seqs: Vec<u16> = found.reports.iter().map(|r| r.seq).collect();
    assert_eq!(seqs, [2, 5]);
    assert_eq!(found.invalid, 0);
    // A trailing partial dword is not read (pci.c:681).
    short[0..4].copy_from_slice(&(1u32 << 15 | 1 << 14 | 31).to_le_bytes());
    assert_eq!(releases(&short).unwrap().reports.len(), 2);
}

#[test]
fn releases_refuse_malformed_buffers() {
    let mut buffer = rpq_buffer(0, &[rpp(1, 0, 0)]);
    assert_eq!(
        releases(&buffer[..7]),
        Err(AxError::Tx("release report shorter than its descriptor"))
    );
    // Not FS and LS (pci.c:672-675).
    buffer[1] &= !0x80;
    assert_eq!(
        releases(&buffer),
        Err(AxError::Tx("release report is not one whole buffer"))
    );
    let whole = rpq_buffer(0, &[rpp(1, 0, 0)]);
    assert_eq!(
        releases(&whole[..39]),
        Err(AxError::Tx("release report longer than its buffer"))
    );
    assert_eq!(releases(&whole[..40]).unwrap().reports.len(), 1);
}

/// RPQ entry `entry`'s buffer: RPQ is ring 1, after RXQ's 256 buffers.
pub(crate) fn rpq_buffer_iova(entry: usize) -> u64 {
    u64::from(RX_BUFFER0) + 0x10000 * (RX_ENTRIES_RXQ + entry) as u64
}

fn drain_rpq(
    rings: &mut Rings,
    io: &mut RegisterModel,
    seen: &mut Vec<Vec<u8>>,
) -> Result<usize, AxError> {
    // SAFETY: as `drain`.
    unsafe {
        rings.rpq().poll(io, &mut |buffer| {
            seen.push(buffer.to_vec());
            Ok(())
        })
    }
}

#[test]
fn rpq_polls_its_own_index() {
    let (m, mut rings, mut io) = running();
    {
        let state = m.lock().unwrap();
        for entry in 0..3 {
            let buffer = rpq_buffer(0, &[rpp(entry as u32, 0, 0)]);
            device_write(&state, rpq_buffer_iova(entry), &buffer);
        }
        // RXQ's buffers are not RPQ's.
        device_write(&state, rxq_buffer(0), &[0xCC; 8]);
    }
    io.on_read(0x121C, Box::new(|_m: &mut RegisterModel| 3 << 16));
    let (before, cpu_before) = syncs(&m);
    let mut seen = Vec::new();
    assert_eq!(drain_rpq(&mut rings, &mut io, &mut seen), Ok(3));
    assert_eq!(io.trace, [Op::R32(0x121C), Op::W16(0x121C, 3)]);
    assert_eq!(syncs(&m), (before + 3, cpu_before + 3));
    let seqs: Vec<u16> = seen
        .iter()
        .flat_map(|buffer| releases(buffer).unwrap().reports)
        .map(|r| r.seq)
        .collect();
    assert_eq!(seqs, [0, 1, 2]);
    // RXQ's host index did not move.
    hardware_index(&mut io, 1);
    let mut rx = Vec::new();
    assert_eq!(drain(&mut rings, &mut io, &mut rx), Ok(1));
    assert_eq!(rx[0][..8], [0xCC; 8]);
    // RPQ wraps as RXQ does.
    io.trace.clear();
    io.on_read(0x121C, Box::new(|_m: &mut RegisterModel| 1 << 16));
    seen.clear();
    assert_eq!(drain_rpq(&mut rings, &mut io, &mut seen), Ok(30));
    assert_eq!(io.trace, [Op::R32(0x121C), Op::W16(0x121C, 1)]);
    // An index past the ring and an all-ones read are refused untouched.
    io.trace.clear();
    let before = syncs(&m);
    io.on_read(0x121C, Box::new(|_m: &mut RegisterModel| 32 << 16));
    assert_eq!(
        drain_rpq(&mut rings, &mut io, &mut seen),
        Err(AxError::Tx("RPQ hardware index out of range"))
    );
    io.on_read(0x121C, Box::new(|_m: &mut RegisterModel| u32::MAX));
    assert_eq!(
        drain_rpq(&mut rings, &mut io, &mut seen),
        Err(AxError::AllOnes { register: 0x121C })
    );
    assert!(io.writes().is_empty());
    assert_eq!(syncs(&m), before);
    // A full RPQ is not counted as a full RXQ.
    assert_eq!(rings.rxq_full_batches(), 0);
}

#[test]
fn rpq_buffer_read_but_not_given_back_is_still_consumed() {
    let (m, mut rings, mut io) = running();
    {
        let state = m.lock().unwrap();
        for entry in 0..4 {
            let buffer = rpq_buffer(0, &[rpp(entry as u32, 0, 0)]);
            device_write(&state, rpq_buffer_iova(entry), &buffer);
        }
    }
    io.on_read(0x121C, Box::new(|_m: &mut RegisterModel| 4 << 16));
    // Entry 1's give-back fails after `each` has read its reports.
    {
        let mut state = m.lock().unwrap();
        state.fail_sync_at = Some(state.syncs + 2);
    }
    let mut seen = Vec::new();
    assert_eq!(
        drain_rpq(&mut rings, &mut io, &mut seen),
        Err(AxError::Dma("ring memory visibility failed"))
    );
    assert_eq!(seen.len(), 2);
    // It is consumed with entry 0, so its reports are never replayed.
    assert_eq!(io.trace, [Op::R32(0x121C), Op::W16(0x121C, 2)]);
    // Entry 2: `each` fails and so does the give-back; the first cause
    // wins, and the entry is consumed all the same.
    {
        let mut state = m.lock().unwrap();
        state.fail_sync_at = Some(state.syncs + 1);
    }
    io.trace.clear();
    // SAFETY: as `drain`.
    let polled = unsafe {
        rings.rpq().poll(&mut io, &mut |buffer| {
            seen.push(buffer.to_vec());
            Err(AxError::Tx("refused"))
        })
    };
    assert_eq!(polled, Err(AxError::Tx("refused")));
    assert_eq!(io.trace, [Op::R32(0x121C), Op::W16(0x121C, 3)]);
    // The next poll delivers entry 3 alone.
    m.lock().unwrap().fail_sync_at = None;
    io.trace.clear();
    assert_eq!(drain_rpq(&mut rings, &mut io, &mut seen), Ok(1));
    assert_eq!(io.trace, [Op::R32(0x121C), Op::W16(0x121C, 4)]);
    let seqs: Vec<u16> = seen
        .iter()
        .flat_map(|buffer| releases(buffer).unwrap().reports)
        .map(|r| r.seq)
        .collect();
    assert_eq!(seqs, [0, 1, 2, 3]);
}

#[test]
fn pre_init_and_deinit_restart_rpq_at_entry_0() {
    let (_m, mut rings, mut io) = running();
    io.on_read(0x121C, Box::new(|_m: &mut RegisterModel| 3 << 16));
    let mut seen = Vec::new();
    assert_eq!(drain_rpq(&mut rings, &mut io, &mut seen), Ok(3));
    deinit(&mut io, &rings, false).unwrap();
    io.on_read(0x121C, Box::new(|_m: &mut RegisterModel| 2 << 16));
    assert_eq!(drain_rpq(&mut rings, &mut io, &mut seen), Ok(2));
    start(&mut io, 2, &rings).unwrap();
    io.trace.clear();
    io.on_read(0x121C, Box::new(|_m: &mut RegisterModel| 1 << 16));
    assert_eq!(drain_rpq(&mut rings, &mut io, &mut seen), Ok(1));
    assert_eq!(io.writes(), [Op::W16(0x121C, 1)]);
}

#[test]
fn kick_writes_the_index_once() {
    // `rtw89_pci_tx_write` advances `bd_ring->wp` per frame and
    // `__rtw89_pci_tx_kick_off` writes it once (pci.c:1332-1345,
    // :1637-1678).
    let (_m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ach0;
    assert!(!rings.txq(&mut pools, ch).pending_kick());
    for seq in 0..3u8 {
        assert_eq!(
            submit(&mut rings, &mut pools, &mut io, ch, &head(seq), &[seq; 4]),
            Ok(Some(seq))
        );
    }
    assert!(io.trace.is_empty());
    assert!(rings.txq(&mut pools, ch).pending_kick());
    assert_eq!(kick(&mut rings, &mut pools, &mut io, ch), Ok(true));
    assert_eq!(io.trace, [Op::W16(0x1058, 3)]);
    // Nothing moved since: no write.
    assert!(!rings.txq(&mut pools, ch).pending_kick());
    assert_eq!(kick(&mut rings, &mut pools, &mut io, ch), Ok(false));
    assert_eq!(io.trace, [Op::W16(0x1058, 3)]);
    // Another channel's kick writes only its own index.
    assert_eq!(
        kick(&mut rings, &mut pools, &mut io, TxChannel::Ach1),
        Ok(false)
    );
    submit(&mut rings, &mut pools, &mut io, ch, &head(3), &[3]).unwrap();
    assert_eq!(kick(&mut rings, &mut pools, &mut io, ch), Ok(true));
    assert_eq!(io.trace, [Op::W16(0x1058, 3), Op::W16(0x1058, 4)]);
}

#[test]
fn a_failed_kick_is_written_again_by_the_next() {
    // The host index advanced and the device may have seen the write, so
    // the pages stay in flight; the next kick writes the same index again.
    let (_m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ach2;
    submit(&mut rings, &mut pools, &mut io, ch, &head(0), &[1]).unwrap();
    io.fail_writes_of(0x1060);
    assert!(kick(&mut rings, &mut pools, &mut io, ch).is_err());
    assert!(page(&pools, ch, 0).in_flight);
    assert!(rings.txq(&mut pools, ch).pending_kick());
    io.allow_writes_of(0x1060);
    io.trace.clear();
    assert_eq!(kick(&mut rings, &mut pools, &mut io, ch), Ok(true));
    assert_eq!(io.trace, [Op::W16(0x1060, 1)]);
}

#[test]
fn pre_init_restarts_the_kick_with_the_ring() {
    let (_m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ach0;
    submit(&mut rings, &mut pools, &mut io, ch, &head(0), &[1]).unwrap();
    start(&mut io, 2, &rings).unwrap();
    // The ring and its last kick both restart at entry 0.
    assert!(!rings.txq(&mut pools, ch).pending_kick());
    io.trace.clear();
    assert_eq!(kick(&mut rings, &mut pools, &mut io, ch), Ok(false));
    assert!(io.trace.is_empty());
}

#[test]
fn pre_init_writes_linux_depths() {
    // M4 spec S9.2: rtw89_pci_reset_trx_rings writes each ring's length to
    // its NUM (pci.c:1776-1896). ACH0-3 and RXQ take Linux's 256
    // (pci.h:1122-1123); CH8, CH12 and the unused rings keep 16, and RPQ 32.
    let writes = pre_init_writes(2);
    let num = |reg: u32| -> Vec<u16> {
        writes
            .iter()
            .filter_map(|op| match op {
                Op::W16(r, v) if *r == reg => Some(*v),
                _ => None,
            })
            .collect()
    };
    for reg in [0x1024, 0x1026, 0x1028, 0x102A] {
        assert_eq!(num(reg), [256], "{reg:#x}");
    }
    for reg in [
        0x102C, 0x102E, 0x1030, 0x1032, 0x1034, 0x1036, 0x1438, 0x143A, 0x1038,
    ] {
        assert_eq!(num(reg), [16], "{reg:#x}");
    }
    assert_eq!(num(0x1210), [256]);
    assert_eq!(num(0x1212), [32]);
    assert_eq!(
        (
            TX_ENTRIES_DATA,
            TX_ENTRIES_CTRL,
            RX_ENTRIES_RXQ,
            RX_ENTRIES_RPQ,
            WD_PAGES
        ),
        (256, 16, 256, 32, 256)
    );
    // Each ring's DESA is its place in the pool: ACH0-3 2048 bytes apart,
    // then 128.
    assert_eq!(TX_RING_AT[..6], [0, 2048, 4096, 6144, 8192, 8320]);
    assert_eq!(TX_RING_AT[12], 8192 + 8 * 128);
}

#[test]
fn rx_copies_only_the_frame() {
    // M4 spec S9.3: a buffer is copied for what the device wrote, its BD
    // info's WRITE_SIZE (`rx_info->len`, pci.c:284, :301), not all 11 498
    // bytes.
    let (m, mut rings, mut io) = running();
    let frame: Vec<u8> = (0..100).map(|i| i as u8 ^ 0x5A).collect();
    // A short descriptor, then a long one with one 8-byte DRV_INFO unit.
    let short = 4 + 16 + 100;
    let long = 4 + 32 + 8 + 100;
    let mut a = vec![0u8; short];
    a[..4].copy_from_slice(&(0xC000 | short as u32).to_le_bytes());
    a[4..8].copy_from_slice(&100u32.to_le_bytes());
    a[20..].copy_from_slice(&frame);
    let mut b = vec![0u8; long];
    b[..4].copy_from_slice(&(0xC000 | long as u32).to_le_bytes());
    b[4..8].copy_from_slice(&(100u32 | 1 << 28 | 1 << 31).to_le_bytes());
    b[44..].copy_from_slice(&frame);
    {
        let state = m.lock().unwrap();
        // What lies past the frame is a stale earlier frame's.
        device_write(&state, rxq_buffer(0), &[0xEE; RX_BUFFER_BYTES]);
        device_write(&state, rxq_buffer(1), &[0xEE; RX_BUFFER_BYTES]);
        device_write(&state, rxq_buffer(0), &a);
        device_write(&state, rxq_buffer(1), &b);
    }
    hardware_index(&mut io, 2);
    let mut seen = Vec::new();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(2));
    assert_eq!(seen, [a, b]);
    // A WRITE_SIZE under the BD info's own 4 bytes still hands those over.
    {
        let state = m.lock().unwrap();
        device_write(&state, rxq_buffer(2), &[0x00, 0xC0, 0x00, 0x00]);
    }
    hardware_index(&mut io, 3);
    seen.clear();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(1));
    assert_eq!(seen, [vec![0x00, 0xC0, 0, 0]]);
}

#[test]
fn a_data_ring_wraps_at_256() {
    // ACH0's 256 entries (`RTW89_PCI_TXBD_NUM_MAX`): 260 frames, each read
    // and reported as it is sent, wrap the host index past 255 to 0.
    let (_m, mut rings, mut pools, mut io) = running_with_pools();
    let ch = TxChannel::Ach0;
    for n in 0..260u32 {
        let seq = transmit(&mut rings, &mut pools, &mut io, ch, &head(0), &[n as u8])
            .unwrap()
            .unwrap();
        assert_eq!(seq, 0);
        tx_hardware_index(&mut io, 0, (n + 1) % 256);
        reclaim(&mut rings, &mut pools, &mut io, ch).unwrap();
        assert!(pools.release(ch, seq));
    }
    let kicks: Vec<u16> = io
        .trace
        .iter()
        .filter_map(|op| match op {
            Op::W16(0x1058, v) => Some(*v),
            _ => None,
        })
        .collect();
    let mut want: Vec<u16> = (1..256).collect();
    want.extend([0, 1, 2, 3, 4]);
    assert_eq!(kicks, want);
}

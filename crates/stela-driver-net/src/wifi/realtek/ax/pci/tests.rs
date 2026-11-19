extern crate std;
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use crate::wifi::realtek::beacon::tests::{Domain, Model, Source};
use alloc::{boxed::Box, vec, vec::Vec};
use std::sync::Mutex;

// The beacon DMA model hands out IOVAs 0x1000_0000 + 0x10000 * n in mapping
// order, and `Rings::install` maps the TX BD pool, the RX BD pool, the 64 RX
// buffers (RXQ's 32, then RPQ's 32) and the CH12 slots in that order.
pub(crate) const TX_POOL: u32 = 0x1001_0000;
const RX_POOL: u32 = 0x1002_0000;
const RX_BUFFER0: u32 = 0x1003_0000;
pub(crate) const FWCMD: u32 = 0x1043_0000;
const MAPPINGS: usize = 67;

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
    let t = |channel: u32| TX_POOL + channel * 16 * 8;
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
        Op::W16(0x1024, 16), // ACH0
        Op::W32(0x1300, 0x0002_0500),
        Op::W32(0x1230, t(0)),
        Op::W32(0x1234, 0),
        Op::W16(0x1026, 16), // ACH1
        Op::W32(0x1304, 0x0002_0505),
        Op::W32(0x1238, t(1)),
        Op::W32(0x123C, 0),
        Op::W16(0x1028, 16), // ACH2
        Op::W32(0x1308, 0x0002_050A),
        Op::W32(0x1240, t(2)),
        Op::W32(0x1244, 0),
        Op::W16(0x102A, 16), // ACH3
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
        Op::W16(0x1210, 32), // RXQ
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
    let buffers: Vec<u32> = (0..64).map(|i| RX_BUFFER0 + i * 0x10000).collect();
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
            device_read(&state, u64::from(TX_POOL), 13 * 16 * 8),
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
    want.extend([Direction::FromDevice; 64]);
    assert_eq!(directions, want);
    assert_eq!(tx_bds, vec![0; 13 * 16 * 8]);
    // RXQ's 32 BDs start the pool and RPQ's start its second page; the rest
    // of each page stays zero. BD e of ring r is buffer r * 32 + e.
    let mut want = vec![0u8; 2 * 4096];
    for (i, buffer) in buffers.iter().enumerate() {
        // le16 buf_size 11498, le16 opt 0 (dma[39:32] = 0), le32 dma.
        let at = (i / 32) * 4096 + (i % 32) * 8;
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
    assert_eq!(ring, u64::from(TX_POOL + 12 * 16 * 8));
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
    let ring = u64::from(TX_POOL + 12 * 16 * 8);
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
/// its entries are the first `RX_ENTRIES` buffers mapped.
pub(crate) fn rxq_buffer(entry: usize) -> u64 {
    assert!(entry < RX_ENTRIES);
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
    let first: Vec<u8> = (0..RX_BUFFER_BYTES).map(|i| (i * 7) as u8).collect();
    let second: Vec<u8> = (0..RX_BUFFER_BYTES).map(|i| (i * 13 + 1) as u8).collect();
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
    assert!(seen[0] == first && seen[1] == second);
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
        for entry in 0..RX_ENTRIES {
            device_write(&state, rxq_buffer(entry), &[entry as u8 + 1; 16]);
        }
    }
    let mut counts = Vec::new();
    let mut seen = Vec::new();
    for hw in [30, 1, 1] {
        hardware_index(&mut io, hw);
        counts.push(drain(&mut rings, &mut io, &mut seen).unwrap());
    }
    assert_eq!(counts, [30, 3, 0]);
    // Entries 0-29, then 30, 31 and (wrapped) 0.
    let order: Vec<u8> = seen.iter().map(|buffer| buffer[0]).collect();
    let mut want: Vec<u8> = (1..=32).collect();
    want.push(1);
    assert_eq!(order, want);
    assert_eq!(io.writes(), [Op::W16(0x1218, 30), Op::W16(0x1218, 1)]);
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

/// Put entry `entry`'s marker byte (`entry + 1`) at the start of each RXQ
/// buffer.
fn mark_every_rxq_entry(m: &Arc<Mutex<Model>>) {
    let state = m.lock().unwrap();
    for entry in 0..RX_ENTRIES {
        device_write(&state, rxq_buffer(entry), &[entry as u8 + 1; 4]);
    }
}

#[test]
fn rxq_poll_counts_a_full_ring() {
    let (m, mut rings, mut io) = running();
    mark_every_rxq_entry(&m);
    assert_eq!(rings.rxq_full_batches(), 0);
    // Host index 0, hardware index 31: RX_ENTRIES - 1 entries filled, the
    // ring full (`rx_ring_eq_is_full` false).
    hardware_index(&mut io, 31);
    let mut seen = Vec::new();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(31));
    let order: Vec<u8> = seen.iter().map(|buffer| buffer[0]).collect();
    assert_eq!(order, (1..=31).collect::<Vec<u8>>());
    assert_eq!(io.trace, [Op::R32(0x1218), Op::W16(0x1218, 31)]);
    assert_eq!(rings.rxq_full_batches(), 1);
    // Nothing new: nothing delivered, nothing counted.
    io.trace.clear();
    seen.clear();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(0));
    assert!(seen.is_empty());
    assert_eq!(io.trace, [Op::R32(0x1218)]);
    assert_eq!(rings.rxq_full_batches(), 1);
    // A batch of 30 (host 31, hardware 29) is not a full ring.
    hardware_index(&mut io, 29);
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(30));
    assert_eq!(rings.rxq_full_batches(), 1);
    // The count saturates (host 29, hardware 28: full again).
    rings.rxq_full.set(u16::MAX);
    hardware_index(&mut io, 28);
    seen.clear();
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(31));
    assert_eq!(rings.rxq_full_batches(), u16::MAX);
}

#[test]
fn rxq_wraps_past_entry_31() {
    let (m, mut rings, mut io) = running();
    mark_every_rxq_entry(&m);
    let mut seen = Vec::new();
    hardware_index(&mut io, 30);
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(30));
    // Host 30, hardware 2: entries 30, 31, 0, 1.
    io.trace.clear();
    seen.clear();
    hardware_index(&mut io, 2);
    assert_eq!(drain(&mut rings, &mut io, &mut seen), Ok(4));
    let order: Vec<u8> = seen.iter().map(|buffer| buffer[0]).collect();
    assert_eq!(order, [31, 32, 1, 2]);
    assert_eq!(io.trace, [Op::R32(0x1218), Op::W16(0x1218, 2)]);
    assert_eq!(rings.rxq_full_batches(), 0);
}

#[test]
fn hardware_index_32_is_refused() {
    let (m, mut rings, mut io) = running();
    mark_every_rxq_entry(&m);
    hardware_index(&mut io, 32);
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
            device_write(&state, rxq_buffer(entry), &[entry as u8 + 1; 4]);
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
            device_write(&state, rxq_buffer(entry), &[entry as u8 + 1; 4]);
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
        for entry in 0..RX_ENTRIES {
            device_write(&state, rxq_buffer(entry), &[entry as u8 + 1; 4]);
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

extern crate std;
use super::*;
use crate::wifi::realtek::ax::elements::{
    ID_BB_GAIN, ID_BB_REG, ID_RADIO_A, ID_RADIO_B, ID_RF_NCTL, PhyTables, RfPath, phy_tables,
    tests::{container, element, pairs},
};
use crate::wifi::realtek::ax::fwdl::tests::image_bytes;
use crate::wifi::realtek::ax::pci::tests::{device_write, rxq_buffer};
use crate::wifi::realtek::ax::radio::RfAt;
use crate::wifi::realtek::ax::tests::model::{Op, READ_FAILED, RegisterModel};
use crate::wifi::realtek::beacon::tests::{Domain, Model, Source};
use crate::wifi::realtek::firmware::{Ce, Rtw89};
use alloc::{boxed::Box, string::ToString, sync::Arc, vec, vec::Vec};
use core::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

/// A MAC whose bytes need lower-case hex and a leading zero.
const MAC: [u8; 6] = [0x00, 0xE0, 0x4C, 0x88, 0x52, 0x0C];

/// Spec S10's crafted RXQ buffer: the REC_ACK of OFLD_CFG at seq 0 (RX BD
/// info FS | LS | WRITE_SIZE 32, a short RX descriptor with RPKT_LEN 12 and
/// type 10, C2H cat 1 class 0 func 0 of length 12, then w2 = cat 1, class
/// 9, func 0x14, seq 0).
const REC_ACK: [u8; 32] = [
    0x20, 0xC0, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x0A, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x01, 0x00,
    0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x25, 0x14, 0x00, 0x00,
];
/// The same as a DONE_ACK (func 1) with return 0.
const DONE_ACK: [u8; 32] = [
    0x20, 0xC0, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x0A, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x01, 0x01,
    0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x25, 0x14, 0x00, 0x00,
];
/// A DONE_ACK whose `H2C_RETURN` ([23:16] of w2) is 3.
const DONE_ACK_REFUSED: [u8; 32] = [
    0x20, 0xC0, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x0A, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x01, 0x01,
    0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x25, 0x14, 0x03, 0x00,
];
/// A plain firmware log C2H (class 0 func 2) of 13 bytes, whose text is
/// `fw up`: WRITE_SIZE 33, RPKT_LEN 13.
const LOG: [u8; 33] = [
    0x21, 0xC0, 0x00, 0x00, 0x0D, 0x00, 0x00, 0x0A, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x01, 0x02,
    0x00, 0x00, 0x0D, 0x00, 0x00, 0x00, b'f', b'w', b' ', b'u', b'p',
];
/// `REC_ACK` with byte `at` replaced by `value`.
const fn rec_ack_with(at: usize, value: u8) -> [u8; 32] {
    let mut b = REC_ACK;
    b[at] = value;
    b
}
/// A REC_ACK for seq 1 (w2 [23:16]): an ack, but not OFLD_CFG's.
const REC_ACK_SEQ1: [u8; 32] = rec_ack_with(30, 0x01);
/// A class 1 MAC C2H (w0 = cat 1 | class 1 << 2): not an ack at all.
const OTHER_C2H: [u8; 32] = rec_ack_with(20, 0x05);
/// An RX packet of type 0 (WIFI, RX descriptor byte 3), not a C2H.
const NOT_C2H: [u8; 32] = rec_ack_with(7, 0x00);
/// Both acks, as the firmware may send them.
const BOTH: &[&[u8]] = &[&REC_ACK, &DONE_ACK];

/// A 2048-byte physical efuse (0xFF = blank) whose logical map holds `MAC`
/// at 0x400, crystal cap 0x2F at 0x2B9 and RFE type 1 at 0x2CA.
fn efuse_with_mac() -> Vec<u8> {
    efuse_with_rfe(1)
}

/// As [`efuse_with_mac`], with RFE type `rfe`: block 0x80 (words 0-2) holds
/// the MAC, block 0x57 word 0 the crystal cap (0x2B8, 0x2B9) and block 0x59
/// word 1 the RFE type (0x2CA, 0x2CB). 0x5EC/0x5ED are left blank: secure
/// boot off; the phycap window 0x590..0x5F0 is blank too, so no trim.
fn efuse_with_rfe(rfe: u8) -> Vec<u8> {
    let mut phys = vec![0xFF; 2048];
    phys[4..6].copy_from_slice(&[0x08, 0x08]);
    phys[6..12].copy_from_slice(&MAC);
    phys[12..16].copy_from_slice(&[0x05, 0x7E, 0xFF, 0x2F]);
    phys[16..20].copy_from_slice(&[0x05, 0x9D, rfe, 0xFF]);
    phys
}

/// The firmware tables every bring-up here runs: BB `[(0x4AD4, 0x40),
/// (0x4AE0, 0x40)]`, BB gain `[(0, 1)]`, radio slot 0 path B and slot 1
/// path A both `[(0x05, 0), (0x10005, 1)]` (one SWSI write, one direct
/// window write, which alone is stored) behind a headline for RFE 1 at any
/// cut, and NCTL `[(0x8008, 0)]`.
fn tables_file() -> Vec<u8> {
    let radio = pairs(&[(0xF001_00FF, 0), (0x05, 0), (0x10005, 1)]);
    let mut b = container(32, 5);
    element(
        &mut b,
        ID_BB_REG,
        [0; 4],
        0,
        0,
        &pairs(&[(0x4AD4, 0x40), (0x4AE0, 0x40)]),
    );
    element(&mut b, ID_BB_GAIN, [0; 4], 0, 0, &pairs(&[(0, 1)]));
    element(&mut b, ID_RADIO_A, [0; 4], 0, 1, &radio);
    element(&mut b, ID_RADIO_B, [0; 4], 0, 0, &radio);
    element(&mut b, ID_RF_NCTL, [0; 4], 0, 0, &pairs(&[(0x8008, 0)]));
    b
}

/// What [`tables_file`] reports through a full bring-up of [`chip`].
const PHY: &str = ", PHY rfe 1 xtal 0x2f: BB 2, BB gain 1, RF B 2 A 2, RF pages 1+1, RF18 A \
                   0x01001 B 0x01001, GNT 0x00000006, GNT RFC 0x06, GNT SW 0x00057ddf, GNT \
                   before coex 0x00000018";

/// The chip as every bring-up poll wants it:
/// - power: RDY_SYSPWR (0x0004 bit17) reads set, APFN_ONMAC and APFM_OFFMAC
///   (bits 8, 9) self-clear, and XTAL SI commands (0x0270 bit31) complete;
/// - efuse: 0x0030 answers EF_RDY with the byte at the written address;
/// - DMAC: the WDE and PLE managers (0x8D00, 0x9100) report ready;
/// - PCI: the DMA busy registers read idle (zero) and the BD RAM reset
///   (0x1000 bit3) self-clears; the CH12 engine (0x1080) consumes every
///   kicked entry at once, unless `ch12_holds` says to hold it;
/// - WCPU: once the WCPU is enabled (0x0088 bit1), 0x01E0 reports both path
///   ready bits and download status `status` in [7:5];
/// - MAC init: the STA scheduler reports init done (0x9E10 bit31), the
///   ADDR CAM clear (0xCE34 bit8) and BA CAM reset (0xCE3C [1:0])
///   self-clear, and the public page count (0x179C) reads 0x0C9C, which
///   matches the SCC `pub_max`;
/// - RXQ: at its first index read, which only the OFLD_CFG wait makes, after
///   the kick, the device fills entries 0.. with `acks` and its index
///   (0x1218 [27:16]) reports them.
///
/// `pre_inits` counts PCI pre-inits by their first step's read, the G1
/// deglitch (0x3848); the deinit reads TXBD_RWPTR_CLR1 too, so that is no
/// longer pre-init's alone. `m`
/// is the DMA model the RXQ buffers live in.
struct Chip {
    io: RegisterModel,
    pre_inits: Arc<AtomicUsize>,
    m: Arc<Mutex<Model>>,
}

fn chip(
    phys: Vec<u8>,
    status: u32,
    ch12_holds: fn(usize) -> bool,
    acks: &'static [&'static [u8]],
) -> Chip {
    let mut io = RegisterModel::new();
    io.on_read(
        0x0004,
        Box::new(|m: &mut RegisterModel| (m.get32(0x0004) | 1 << 17) & !(1 << 8 | 1 << 9)),
    );
    io.on_read(
        0x0270,
        Box::new(|m: &mut RegisterModel| m.get32(0x0270) & 0x7FFF_FFFF),
    );
    io.on_read(
        0x0030,
        Box::new(move |m: &mut RegisterModel| {
            let ctl = m.get32(0x0030);
            let addr = ((ctl >> 16) & 0x7FF) as usize;
            (ctl & 0x07FF_0000) | 1 << 29 | u32::from(phys[addr])
        }),
    );
    io.set32(0x8D00, 0x3);
    io.set32(0x9100, 0x3);
    io.on_read(
        0x1000,
        Box::new(|m: &mut RegisterModel| m.get32(0x1000) & !(1 << 3)),
    );
    let pre_inits = Arc::new(AtomicUsize::new(0));
    let counted = pre_inits.clone();
    io.on_read(
        0x3848,
        Box::new(move |m: &mut RegisterModel| {
            counted.fetch_add(1, Ordering::Relaxed);
            m.get32(0x3848)
        }),
    );
    let seen = pre_inits.clone();
    io.on_read(
        0x1080,
        Box::new(move |m: &mut RegisterModel| {
            let host = m.get32(0x1080) & 0xFFF;
            if ch12_holds(seen.load(Ordering::Relaxed)) {
                host
            } else {
                host | host << 16
            }
        }),
    );
    io.on_read(
        0x01E0,
        Box::new(move |m: &mut RegisterModel| {
            let v = m.get32(0x01E0);
            if m.get32(0x0088) & 1 << 1 != 0 {
                (v & !0xE0) | status << 5 | 1 << 2 | 1 << 1
            } else {
                v
            }
        }),
    );
    io.on_read(
        0x9E10,
        Box::new(|m: &mut RegisterModel| m.get32(0x9E10) | 0x8000_0000),
    );
    io.on_read(
        0xCE34,
        Box::new(|m: &mut RegisterModel| m.get32(0xCE34) & !0x100),
    );
    io.on_read(
        0xCE3C,
        Box::new(|m: &mut RegisterModel| m.get32(0xCE3C) & !0x3),
    );
    io.set32(0x179C, 0x0C9C);
    // The radio: SWSI answers from the RF file, whose 0x18 reads 0x01001 on
    // both paths, and R_AX_GNT_VAL_V1 reads BT granted on both paths (bits
    // 3, 4) until the WL-only grant is written, then WL on both (bits 1, 2).
    io.emulate_rf();
    io.set_rf(0, 0x18, 0x01001);
    io.set_rf(1, 0x18, 0x01001);
    io.on_read(
        0xDA4C,
        Box::new(|m: &mut RegisterModel| {
            if m.trace.contains(&Op::W32(0xDA48, 0x0005_7DDF)) {
                0x6
            } else {
                0x18
            }
        }),
    );
    let m = Arc::new(Mutex::new(Model::default()));
    let model = m.clone();
    let mut sent = false;
    io.on_read(
        0x1218,
        Box::new(move |_: &mut RegisterModel| {
            if !sent {
                let state = model.lock().unwrap();
                for (entry, buffer) in acks.iter().enumerate() {
                    device_write(&state, rxq_buffer(entry), buffer);
                }
                sent = true;
            }
            (acks.len() as u32) << 16
        }),
    );
    Chip { io, pre_inits, m }
}

fn never(_: usize) -> bool {
    false
}

/// The host this bring-up runs on: LTR off unless a test says otherwise.
const NO_LTR: Host = Host { ltr: false };

fn run(io: &mut RegisterModel, m: &Arc<Mutex<Model>>) -> Result<(Running, Rings), BringUpError> {
    run_on(io, m, NO_LTR)
}

fn run_on(
    io: &mut RegisterModel,
    m: &Arc<Mutex<Model>>,
    host: Host,
) -> Result<(Running, Rings), BringUpError> {
    let file = tables_file();
    run_with(io, m, host, &phy_tables(&file, 0).unwrap())
}

fn run_with(
    io: &mut RegisterModel,
    m: &Arc<Mutex<Model>>,
    host: Host,
    tables: &PhyTables<'_>,
) -> Result<(Running, Rings), BringUpError> {
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    // SAFETY: the model domain isolates only the modeled requester behind
    // this test's `RegisterModel`; its pages are plain kept test RAM that no
    // real device can reach, and this test is the only worker.
    unsafe {
        bring_up(
            io,
            &Source,
            Arc::new(Domain(m.clone())),
            2,
            &fw.image,
            host,
            tables,
        )
    }
}

/// Bring up a chip as [`chip`] models it with `acks`, on `host`.
fn ran(
    acks: &'static [&'static [u8]],
    host: Host,
) -> (Chip, Result<(Running, Rings), BringUpError>) {
    let mut c = chip(efuse_with_mac(), 7, never, acks);
    let m = c.m.clone();
    let result = run_on(&mut c.io, &m, host);
    (c, result)
}

/// What a full bring-up reports when the firmware sent `rec` and `done`.
fn acked(rec: bool, done: bool) -> H2cAck {
    H2cAck {
        rec,
        done,
        other_c2h: 0,
        unmatched_ack: 0,
        skipped: 0,
        log: None,
    }
}

/// The error of a bring-up that must fail (`Rings` has no `Debug`).
fn failed(result: Result<(Running, Rings), BringUpError>) -> BringUpError {
    match result {
        Ok((running, _)) => panic!("bring-up succeeded: {running}"),
        Err(error) => error,
    }
}

/// `rtw89_mac_disable_cpu_ax` starts every download attempt by selecting
/// the WDT control word through the indirect window.
const ATTEMPT: Op = Op::W32(0x0C04, 0x1800_3040);

fn count(io: &RegisterModel, op: Op) -> usize {
    io.trace.iter().filter(|o| **o == op).count()
}

fn xtal(offset: u32, value: u32, mask: u32) -> Op {
    Op::W32(0x0270, 1 << 31 | mask << 16 | value << 8 | offset)
}

/// The writes the trace must end with: `rtw8852c_pwr_off_func`
/// (`rtw8852c.c:453-529`), then `rtw89_mac_update_scoreboard(
/// MAC_AX_NOTIFY_PWR_MAJOR)` (`mac.c:1580`). `None` values are
/// read-modify-writes whose value depends on earlier state.
fn power_off_tail() -> Vec<(Op, bool)> {
    let exact = |op| (op, true);
    let at = |reg| (Op::W32(reg, 0), false);
    vec![
        exact(xtal(0x90, 0x10, 0x10)),
        exact(xtal(0x90, 0, 0x08)),
        exact(xtal(0x90, 0, 0x04)),
        exact(xtal(0x80, 0, 0x01)),
        exact(xtal(0x81, 0, 0x01)),
        exact(xtal(0x90, 0x80, 0x80)),
        exact(xtal(0x90, 0, 0x02)),
        exact(xtal(0x90, 0, 0x01)),
        at(0x0004),
        at(0x02F0),
        (Op::W8(0x0002, 0), false),
        at(0x0080),
        at(0x0018),
        exact(xtal(0x90, 0, 0x20)),
        at(0x0018),
        exact(xtal(0x90, 0, 0x40)),
        at(0x0004),
        exact(Op::W32(0x0090, 0x0001_A0B2)),
        at(0x0004),
        at(0x0010),
        at(0x0200),
        at(0x0004),
        exact(Op::W8(0x00AF, 0x80)),
    ]
}

/// The three writes just before the power-off tail are `pci::stop_dma`'s:
/// STOP_WPDMA (0x1010 bit 19) set, then `rtw89_pci_ctrl_dma_all(false)`:
/// STOP_AXI_MST (0x1000 bit 17) set and TXHCI_EN/RXHCI_EN (bits 7, 15)
/// cleared.
fn assert_dma_stopped_before_power_off(io: &RegisterModel) {
    let writes = io.writes();
    let start = writes.len() - power_off_tail().len();
    let stop = &writes[start - 3..start];
    assert!(
        matches!(stop[0], Op::W32(0x1010, v) if v & 1 << 19 != 0)
            && matches!(stop[1], Op::W32(0x1000, v) if v & 1 << 17 != 0)
            && matches!(stop[2], Op::W32(0x1000, v) if v & (1 << 7 | 1 << 15) == 0 && v & 1 << 17 != 0),
        "{stop:x?}"
    );
}

/// No `pci::stop_dma` and no deinit: no write to 0x1010 or 0x1000 at all,
/// which only the MAC pre-init and later steps make.
fn assert_no_dma_writes(io: &RegisterModel) {
    assert!(
        !io.writes()
            .iter()
            .any(|o| matches!(o, Op::W32(0x1010 | 0x1000, _))),
        "{:x?}",
        io.writes()
    );
}

fn assert_ends_with_power_off(io: &RegisterModel) {
    let writes = io.writes();
    let tail = power_off_tail();
    assert!(writes.len() >= tail.len());
    let got = &writes[writes.len() - tail.len()..];
    for (i, (got, (want, exact))) in got.iter().zip(&tail).enumerate() {
        let same = match (got, want) {
            _ if *exact => got == want,
            (Op::W32(a, _), Op::W32(b, _)) => a == b,
            (Op::W8(a, _), Op::W8(b, _)) => a == b,
            _ => false,
        };
        assert!(same, "power-off write {i}: got {got:?}, want {want:?}");
    }
    // The last APFM_SWLPS write sets bit 10 (rtw8852c.c:520).
    assert!(matches!(got[tail.len() - 2], Op::W32(0x0004, v) if v & 1 << 10 != 0));
}

/// What a full bring-up of [`chip`] with [`tables_file`] applies and reads.
const REPORT: PhyReport = PhyReport {
    rfe: 1,
    xtal: 0x2F,
    bb: 2,
    bb_gain: 1,
    rf: [2, 2],
    rf_words: [1, 1],
    rf18: [0x01001, 0x01001],
    gnt: 0x6,
    gnt_sw: 0x0005_7DDF,
    gnt_before: 0x18,
    c2h_after: 0,
};

#[test]
fn happy_path_reports_running() {
    let (c, result) = ran(BOTH, NO_LTR);
    let (running, rings) = result.unwrap();
    assert_eq!(
        running,
        Running {
            cut: 2,
            firmware: (0, 27, 66, 1),
            mac: MAC,
            secure_boot: false,
            mac_init: true,
            ltr: false,
            bacam_reset_timed_out: false,
            h2c: acked(true, true),
            phy: Some(REPORT),
        }
    );
    assert_eq!(
        running.to_string(),
        alloc::format!(
            "RTL8852CE firmware running: cut 2, firmware 0.27.66.1, MAC 00:e0:4c:88:52:0c, \
             secure boot off, MAC init done, LTR off, H2C OFLD_CFG acknowledged (receive and \
             done){PHY}"
        )
    );
    // The brief's M2b1 text, then the grant reads appended in its style.
    assert!(running.to_string().contains(
        ", PHY rfe 1 xtal 0x2f: BB 2, BB gain 1, RF B 2 A 2, RF pages 1+1, RF18 A 0x01001 B \
         0x01001, GNT 0x00000006, "
    ));
    // The RF pages carried seq 1 and 2, after OFLD_CFG's seq 0: neither
    // asks for a REC_ACK, so nothing was left unread in RXQ.
    assert_eq!(running.phy.map(|phy| phy.c2h_after), Some(0));
    // One download attempt, one PCI pre-init.
    assert_eq!(count(&c.io, ATTEMPT), 1);
    assert_eq!(c.pre_inits.load(Ordering::Relaxed), 1);
    // RXQ's two entries were consumed and its host index written back once.
    assert_eq!(count(&c.io, Op::W16(0x1218, 2)), 1);
    // rtw89_core_stop (core.c:6765-6767): rtw89_pci_ops_deinit after the
    // OFLD_CFG and RXQ activity, then rtw89_mac_pwr_off.
    assert_eq!(deinit_writes(&c.io), DEINIT);
    let write_back = c.io.trace.iter().position(|o| *o == Op::W16(0x1218, 2));
    let wake_off =
        c.io.trace
            .iter()
            .rposition(|o| matches!(o, Op::W32(0x0074, _)));
    assert!(write_back < wake_off, "{write_back:?} {wake_off:?}");
    assert_ends_with_power_off(&c.io);
    // The rings were programmed: the caller keeps them.
    assert!(!c.m.lock().unwrap().mappings.is_empty());
    drop(rings);
}

#[test]
fn happy_path_runs_the_steps_in_linux_order() {
    let (c, result) = ran(BOTH, NO_LTR);
    result.unwrap();
    let t = &c.io.trace;
    let first = |op: Op| t.iter().position(|o| *o == op).unwrap();
    // The first trace entry after `from` that `want` accepts.
    let after = |from: usize, want: fn(&Op) -> bool| {
        from + 1
            + t[from + 1..]
                .iter()
                .position(want)
                .unwrap_or_else(|| panic!("nothing after trace entry {from}"))
    };
    // power on: the last write of rtw8852c_pwr_on_func.
    let power_on = first(Op::W32(0x02DC, 0x0100_0000));
    // secure boot: efuse 0x5EC, then the TP_MAJOR scoreboard (mac.c:1569).
    let secure = first(Op::W32(0x0030, 0x05EC_0000));
    let tp_major = first(Op::W8(0x00AF, 0x81));
    // mac pre-init: rtw89_mac_ctrl_hci_dma_trx(true) on HCI_FUNC_EN_V1.
    let mac_pre_init = first(Op::R32(0x7880));
    // pci pre-init: its first step, the G1 deglitch.
    let pci_pre_init = first(Op::R16(0x3848));
    let download = first(ATTEMPT);
    // efuse: the full physical dump starts at address 0.
    let efuse = first(Op::W32(0x0030, 0));
    // bb rf enable: rtw8852c_mac_enable_bb_rf's first write, the BB resets
    // in R_AX_SYS_FUNC_EN (only the power-off writes 0x0002 otherwise).
    let bb_rf = first_of(t, |o| matches!(o, Op::W8(0x0002, _)));
    // sys init: dmac_func_en_ax's word.
    let sys = first(Op::W32(0x8400, 0xFB7D_4000));
    // dmac init: the SCC DLE public quota of dle_mix_cfg.
    let dmac = first(Op::W32(0x8C40, 0x0C9C_0C9C));
    // cmac init: the first CMAC register only cmac_init_ax writes.
    let cmac = first_of(t, |o| matches!(o, Op::W32(0xC33C, _)));
    // imr: rtw89_wdrls_imr_enable, enable_imr_ax's first.
    let imr = first_of(t, |o| matches!(o, Op::W32(0x9430, _)));
    // pci post-init: rtw89_pci_ctrl_txdma_ch_ax(true) on 0x1010.
    let post_init = after(imr, |o| matches!(o, Op::W32(0x1010, _)));
    // h2c: the OFLD_CFG kick on CH12.
    let kick = after(post_init, |o| matches!(o, Op::W16(0x1080, _)));
    // c2h: the RXQ host index written back after the acks.
    let c2h = first_of(t, |o| matches!(o, Op::W16(0x1218, _)));
    // power off: rtw8852c_pwr_off_func's first XTAL SI command.
    let power_off = after(c2h, |o| *o == xtal(0x90, 0x10, 0x10));
    let order = [
        power_on,
        secure,
        tp_major,
        mac_pre_init,
        pci_pre_init,
        download,
        efuse,
        bb_rf,
        sys,
        dmac,
        cmac,
        imr,
        post_init,
        kick,
        c2h,
        power_off,
    ];
    assert!(order.windows(2).all(|w| w[0] < w[1]), "{order:?}");
    // Exactly one TP_MAJOR and one PWR_MAJOR scoreboard write.
    assert_eq!(count(&c.io, Op::W8(0x00AF, 0x81)), 1);
    assert_eq!(count(&c.io, Op::W8(0x00AF, 0x80)), 1);
    // The PCI IO release (STOP_WPDMA | STOP_PCIEIO, bits 19-20) is the
    // post-init's last write, before the kick.
    assert!(matches!(
        t[..kick]
            .iter()
            .rev()
            .find(|o| matches!(o, Op::W32(0x1010, _))),
        Some(Op::W32(0x1010, v)) if v & 0x0018_0000 == 0
    ));
}

/// The first trace entry `want` accepts.
fn first_of(t: &[Op], want: fn(&Op) -> bool) -> usize {
    t.iter().position(want).unwrap()
}

#[test]
fn running_reports_which_acks_arrived_and_the_first_firmware_log() {
    for (acks, h2c, line) in [
        (
            &[&REC_ACK as &[u8]] as &'static [&'static [u8]],
            acked(true, false),
            "receive only",
        ),
        (&[&DONE_ACK as &[u8]], acked(false, true), "done only"),
    ] {
        let (_, result) = ran(acks, NO_LTR);
        let (running, _rings) = result.unwrap();
        assert_eq!(running.h2c, h2c);
        assert_eq!(
            running.to_string(),
            alloc::format!(
                "RTL8852CE firmware running: cut 2, firmware 0.27.66.1, MAC 00:e0:4c:88:52:0c, \
                 secure boot off, MAC init done, LTR off, H2C OFLD_CFG acknowledged \
                 ({line}){PHY}"
            )
        );
    }
    // Counts are named only when nonzero, after the acks.
    let (_, result) = ran(
        &[&OTHER_C2H, &REC_ACK_SEQ1, &NOT_C2H, &REC_ACK, &DONE_ACK],
        NO_LTR,
    );
    let (running, _rings) = result.unwrap();
    assert!(
        running.to_string().ends_with(&alloc::format!(
            "H2C OFLD_CFG acknowledged (receive and done), other C2H 1, unmatched acks 1, \
             skipped 1{PHY}"
        )),
        "{running}"
    );
    let (_, result) = ran(&[&NOT_C2H, &DONE_ACK], NO_LTR);
    let (running, _rings) = result.unwrap();
    assert!(
        running
            .to_string()
            .ends_with(&alloc::format!("acknowledged (done only), skipped 1{PHY}")),
        "{running}"
    );
    let (_, result) = ran(&[&LOG, &REC_ACK, &DONE_ACK], NO_LTR);
    let (running, _rings) = result.unwrap();
    assert_eq!(
        running.h2c.log.map(|log| log.as_str() == "fw up"),
        Some(true)
    );
    assert_eq!(
        running.to_string(),
        alloc::format!(
            "RTL8852CE firmware running: cut 2, firmware 0.27.66.1, MAC 00:e0:4c:88:52:0c, \
             secure boot off, MAC init done, LTR off, H2C OFLD_CFG acknowledged (receive and \
             done), firmware log \"fw up\"{PHY}"
        )
    );
}

/// The registers of `rtw89_pci_ops_deinit` (`pci.c:3053`) without LTR:
/// `power_wake(false)`, `ctrl_dma_all(false)` (two writes), `clr_idx_all`.
const DEINIT: [u32; 6] = [0x0074, 0x1000, 0x1000, 0x1014, 0x11C4, 0x1200];
/// The same with `ltr_set_v1(false)` between the wake and the DMA stop.
const DEINIT_LTR: [u32; 12] = [
    0x0074, 0x8410, 0x8414, 0x8414, 0x1600, 0x1610, 0x1604, 0x1000, 0x1000, 0x1014, 0x11C4, 0x1200,
];

/// The registers of the 32-bit writes between the post-init's PCI IO
/// release (its last 0x1010 write) and the power-off's first XTAL SI
/// command: what the bring-up writes after post-init that is not H2C or
/// RXQ traffic (those are 16-bit index writes) and not the radio's. The
/// radio steps write the BB and RF windows (0x10000 up), XTAL SI (0x0270),
/// the BB/RF reset's 0x02F0 and 0x0444, the CMAC's TX power and coex
/// registers (0xC000-0xDFFF) and the coex scoreboard (0x00AC); the deinit
/// writes none of them.
fn deinit_writes(io: &RegisterModel) -> Vec<u32> {
    deinit_and_radio_writes(io)
        .into_iter()
        .filter(|reg| {
            !(*reg >= 0x10000
                || (0xC000..0xE000).contains(reg)
                || matches!(*reg, 0x0270 | 0x02F0 | 0x0444 | 0x00AC))
        })
        .collect()
}

/// Every 32-bit write between the post-init's PCI IO release and the
/// power-off's first XTAL SI command.
fn deinit_and_radio_writes(io: &RegisterModel) -> Vec<u32> {
    let t = &io.trace;
    let release = t
        .iter()
        .rposition(|o| matches!(o, Op::W32(0x1010, _)))
        .unwrap();
    let off = release
        + t[release..]
            .iter()
            .position(|o| *o == xtal(0x90, 0x10, 0x10))
            .unwrap();
    t[release + 1..off]
        .iter()
        .filter_map(|o| match o {
            Op::W32(reg, _) => Some(*reg),
            _ => None,
        })
        .collect()
}

/// Run a full bring-up of [`chip`] with `acks` on `host`, changed by
/// `fault`, which must fail: its error, after checking the chip ended
/// powered off, after `rtw89_pci_ops_deinit` exactly when the post-init
/// had run (the `"h2c"` and `"c2h"` failures and every radio step's).
fn mac_init_fails(
    acks: &'static [&'static [u8]],
    host: Host,
    fault: impl FnOnce(&mut RegisterModel),
) -> BringUpError {
    fails_on(efuse_with_mac(), acks, host, fault)
}

/// [`mac_init_fails`] on a chip whose physical efuse is `phys`.
fn fails_on(
    phys: Vec<u8>,
    acks: &'static [&'static [u8]],
    host: Host,
    fault: impl FnOnce(&mut RegisterModel),
) -> BringUpError {
    let mut c = chip(phys, 7, never, acks);
    fault(&mut c.io);
    let m = c.m.clone();
    let err = failed(run_on(&mut c.io, &m, host));
    // The efuse ran: every failure here is past milestone 1's steps.
    assert!(c.io.trace.contains(&Op::W32(0x0030, 0)), "{err}");
    let efuse =
        c.io.trace
            .iter()
            .position(|o| *o == Op::W32(0x0030, 0))
            .unwrap();
    if matches!(err.step, "h2c" | "c2h") || PHY_STEPS.contains(&err.step) {
        let want: &[u32] = if host.ltr { &DEINIT_LTR } else { &DEINIT };
        assert_eq!(deinit_writes(&c.io), want, "{err}");
    } else {
        // Linux's mac_init failure path is rtw89_mac_pwr_off alone
        // (mac.c:4397): no deinit write after the efuse dump.
        assert!(
            !c.io.trace[efuse..]
                .iter()
                .any(|o| matches!(o, Op::W32(0x0074 | 0x1014 | 0x11C4 | 0x1200, _))),
            "{err}"
        );
        // The DMA live since mac pre-init is stopped before the power-off.
        assert_dma_stopped_before_power_off(&c.io);
    }
    // Only a failure after the coex init runs its undo, and then before the
    // deinit; before it, no scoreboard word is written at all (the
    // power-off's PWR_MAJOR is a byte write).
    if err.step == "phy report" {
        assert_radio_off_before_deinit(&c.io);
    } else {
        assert!(
            !c.io
                .trace
                .iter()
                .any(|o| matches!(o, Op::W32(0x00AC, _)) || *o == Op::W32(0xDA48, GNT_WL_OFF)),
            "{err}"
        );
    }
    assert_ends_with_power_off(&c.io);
    err
}

/// `coex::GNT_WL_OFF`.
const GNT_WL_OFF: u32 = 0x0007_D775;

/// The coex half of `rtw89_core_stop` after the coex init: the WL-off grant,
/// the PLT, then the scoreboard with every driver bit clear (0x8100_0000
/// after the init's 0x8100_4003), all after the init's scoreboard and
/// before the deinit's first write (PCI wake, 0x0074).
fn assert_radio_off_before_deinit(io: &RegisterModel) {
    let t = &io.trace;
    let on = t
        .iter()
        .position(|o| *o == Op::W32(0x00AC, 0x8100_4003))
        .unwrap();
    let after = |from: usize, want: Op| {
        from + 1
            + t[from + 1..]
                .iter()
                .position(|o| *o == want)
                .unwrap_or_else(|| panic!("no {want:x?} after trace entry {from}"))
    };
    let grant = after(on, Op::W32(0xDA48, GNT_WL_OFF));
    let plt = after(grant, Op::W16(0xC67C, 0x0111));
    let clear = after(plt, Op::W32(0x00AC, 0x8100_0000));
    let delay = after(clear, Op::Delay(1000));
    let deinit = on
        + t[on..]
            .iter()
            .position(|o| matches!(o, Op::W32(0x0074, _)))
            .unwrap();
    assert!(delay < deinit, "{clear} {delay} {deinit}");
    assert_eq!(delay, clear + 1);
}

#[test]
fn sys_init_failure_names_step_and_powers_off() {
    // sys_init_ax has no check of its own: its read of the CMAC clock enable
    // (R_AX_CK_EN, 0xC004) fails.
    let err = mac_init_fails(BOTH, NO_LTR, |io| io.fail_reads_of(0xC004));
    assert_eq!(err, BringUpError::new("sys init", READ_FAILED));
}

#[test]
fn imr_failure_names_step_and_powers_off() {
    // dmac and cmac init finish, then ptcl_init's last read (0xC660) finds
    // the DMAC disabled, which is the first check of enable_imr_ax.
    let err = mac_init_fails(BOTH, NO_LTR, |io| {
        io.on_read(
            0xC660,
            Box::new(|m: &mut RegisterModel| {
                m.set32(0x8400, 0);
                m.get32(0xC660)
            }),
        );
    });
    assert_eq!(
        err,
        BringUpError::new("imr", AxError::Dma("DMAC function is not enabled"))
    );
}

#[test]
fn mac_init_failure_names_step_and_powers_off() {
    // bb rf enable: the XTAL SI command after the efuse dump never
    // completes; the power-off's own commands do.
    let err = mac_init_fails(BOTH, NO_LTR, |io| {
        io.on_read(
            0x0270,
            Box::new(|m: &mut RegisterModel| {
                let v = m.get32(0x0270);
                let last = m.trace.iter().rev().find_map(|o| match o {
                    Op::W32(0x0270, v) => Some(*v),
                    _ => None,
                });
                if m.trace.contains(&Op::W32(0x0030, 0)) && last == Some(0x80FF_0700) {
                    v
                } else {
                    v & 0x7FFF_FFFF
                }
            }),
        );
    });
    assert_eq!(err.step, "bb rf enable");
    assert!(
        matches!(
            err.error,
            AxError::Timeout {
                register: 0x0270,
                ..
            }
        ),
        "{err}"
    );

    // dmac init: the public pages read back one short of pub_max.
    let err = mac_init_fails(BOTH, NO_LTR, |io| io.set32(0x179C, 0x0C9B));
    assert_eq!(err.step, "dmac init");
    assert!(matches!(err.error, AxError::Mac(_)), "{err}");

    // cmac init: the security engine's last read finds the CMAC disabled.
    let err = mac_init_fails(BOTH, NO_LTR, |io| {
        io.on_read(
            0x9D1C,
            Box::new(|m: &mut RegisterModel| {
                m.set32(0xC000, 0);
                m.get32(0x9D1C)
            }),
        );
    });
    assert_eq!(
        err,
        BringUpError::new("cmac init", AxError::Mac("CMAC function is not enabled"))
    );

    // pci post-init: an LTR register reads rtw89_pci_ltr_is_err_reg_val's
    // 0xEAEAEAEA.
    let err = mac_init_fails(BOTH, Host { ltr: true }, |io| io.set32(0x1600, 0xEAEA_EAEA));
    assert_eq!(
        err,
        BringUpError::new(
            "pci post-init",
            AxError::Mac("LTR register reads an error value")
        )
    );

    // c2h: the firmware never answers.
    let err = mac_init_fails(&[], NO_LTR, |_| ());
    assert_eq!(
        err,
        BringUpError::new(
            "c2h",
            AxError::NoAck {
                class: 9,
                func: 0x14,
                seq: 0,
                other_c2h: 0,
                unmatched_ack: 0,
                skipped: 0,
                rxq_hw: 0,
            }
        )
    );
    // A dead RX path: nothing arrived, and RXQ's index never moved.
    assert_eq!(
        err.to_string(),
        "RTL8852CE bring-up: c2h: firmware sent no acknowledgement of H2C class 9 func 0x14 \
         seq 0 within 100 ms (RXQ index 0, other C2H 0, unmatched acks 0, skipped 0)"
    );

    // c2h with traffic: a C2H that is not an ack, an ack for another seq
    // and a non-C2H packet arrived, and RXQ's index reads 3.
    let err = mac_init_fails(&[&OTHER_C2H, &REC_ACK_SEQ1, &NOT_C2H], NO_LTR, |_| ());
    assert_eq!(
        err.error,
        AxError::NoAck {
            class: 9,
            func: 0x14,
            seq: 0,
            other_c2h: 1,
            unmatched_ack: 1,
            skipped: 1,
            rxq_hw: 3,
        }
    );
    assert!(
        err.to_string()
            .ends_with("(RXQ index 3, other C2H 1, unmatched acks 1, skipped 1)"),
        "{err}"
    );

    // c2h on an LTR host: the deinit's ltr_set_v1(false) runs too.
    let err = mac_init_fails(&[], Host { ltr: true }, |_| ());
    assert_eq!(err.step, "c2h");

    // h2c: the firmware refuses OFLD_CFG with return 3.
    let err = mac_init_fails(&[&DONE_ACK_REFUSED], NO_LTR, |_| ());
    assert_eq!(
        err,
        BringUpError::new(
            "h2c",
            AxError::H2cRejected {
                class: 9,
                func: 0x14,
                seq: 0,
                ret: 3
            }
        )
    );
}

#[test]
fn ltr_host_flag_controls_ltr_writes() {
    let ltr_writes = |io: &RegisterModel| {
        io.writes()
            .iter()
            .filter(|o| matches!(o, Op::W32(0x1600, _)))
            .count()
    };
    let (c, result) = ran(BOTH, Host { ltr: false });
    result.unwrap();
    assert_eq!(ltr_writes(&c.io), 0);
    assert!(!c.io.trace.contains(&Op::R32(0x1600)));
    assert_eq!(deinit_writes(&c.io), DEINIT);
    let (c, result) = ran(BOTH, Host { ltr: true });
    let (running, _rings) = result.unwrap();
    assert!(running.ltr);
    assert!(
        running
            .to_string()
            .contains(", MAC init done, LTR on, H2C OFLD_CFG")
    );
    // One write from post-init's ltr_set_v1(true), one from the deinit's
    // ltr_set_v1(false): HW_DEC_EN (bit 2) set, then cleared with
    // LTR_REQ_DRV (bit 7) and LTR_IDX_DRV = IDLE ([6:5] = 3).
    let values: Vec<u32> =
        c.io.writes()
            .iter()
            .filter_map(|o| match o {
                Op::W32(0x1600, v) => Some(*v),
                _ => None,
            })
            .collect();
    assert_eq!(values.len(), 2, "{values:x?}");
    assert_eq!(ltr_writes(&c.io), 2);
    assert!(values[0] & 1 << 2 != 0 && values[0] & 1 << 7 == 0);
    assert_eq!(values[1] & 0xFC, 0xE0);
    assert_eq!(deinit_writes(&c.io), DEINIT_LTR);
}

#[test]
fn secure_boot_on_refuses_and_powers_off() {
    // 0x5EC = 0xFF, 0x5ED = 0x7F: v0 key table row 0 (efuse.c:459-480).
    let mut phys = efuse_with_mac();
    phys[0x5EC] = 0xFF;
    phys[0x5ED] = 0x7F;
    let mut c = chip(phys, 7, never, BOTH);
    let m = c.m.clone();
    let err = failed(run(&mut c.io, &m));
    assert_eq!(
        err,
        BringUpError::new("secure boot", AxError::SecureBootUnsupported)
    );
    assert_eq!(
        err.to_string(),
        "RTL8852CE bring-up: secure boot: secure-boot firmware is unsupported"
    );
    assert_ends_with_power_off(&c.io);
    assert_no_dma_writes(&c.io);
    // Nothing past the secure-boot read: no rings, no MAC pre-init.
    assert!(m.lock().unwrap().mappings.is_empty());
    assert!(!c.io.trace.contains(&Op::R32(0x7880)));
}

#[test]
fn mac_pre_init_failure_stops_the_dma_it_enabled_before_powering_off() {
    // The DLFW WDE manager never reports ready, so dmac::pre_init fails
    // after rtw89_mac_dmac_func_pre_en_ax enabled the TX/RX HCI DMA (0x1000
    // bits 7 and 15): the DMA is stopped before the power-off.
    let mut c = chip(efuse_with_mac(), 7, never, BOTH);
    c.io.on_read(0x8D00, Box::new(|_: &mut RegisterModel| 1));
    let m = c.m.clone();
    let err = failed(run(&mut c.io, &m));
    assert_eq!(err.step, "mac pre-init");
    assert!(
        c.io.writes()
            .iter()
            .any(|o| matches!(o, Op::W32(0x1000, v) if v & (1 << 7 | 1 << 15) == 1 << 7 | 1 << 15))
    );
    assert_ends_with_power_off(&c.io);
    assert_dma_stopped_before_power_off(&c.io);
}

#[test]
fn a_ring_failure_powers_off_without_a_dma_stop() {
    // No ring memory can be mapped: "dma rings" fails before any HCI
    // register is written, so only the power-off follows.
    let mut c = chip(efuse_with_mac(), 7, never, BOTH);
    let m = c.m.clone();
    m.lock().unwrap().address_overflow = true;
    let err = failed(run(&mut c.io, &m));
    assert_eq!(err.step, "dma rings");
    assert_ends_with_power_off(&c.io);
    assert_no_dma_writes(&c.io);
}

#[test]
fn download_retries_five_times_then_fails() {
    // The firmware never reports WCPU_FW_INIT_RDY (status stays 0).
    let mut c = chip(efuse_with_mac(), 0, never, BOTH);
    let m = c.m.clone();
    let err = failed(run(&mut c.io, &m));
    // Every attempt failed the same way, so only the last is named, with
    // the whole 0x01E0 read after its wait: path bits 1 and 2, FWDL_EN, and
    // status 0 in [7:5].
    assert_eq!(
        err,
        BringUpError::new(
            "firmware download",
            AxError::FirmwareStatus {
                what: "not ready",
                fw_ctrl: 0x0000_0007,
            }
        )
    );
    assert_eq!(
        err.to_string(),
        "RTL8852CE bring-up: firmware download: not ready (0x01E0=0x00000007)"
    );
    // rtw89_fw_download (fw.c:2043): five attempts. CH12 never failed, so
    // no PCI pre-init re-ran between them.
    assert_eq!(count(&c.io, ATTEMPT), 5);
    assert_eq!(count(&c.io, Op::Delay(5000)), 5);
    assert_eq!(c.pre_inits.load(Ordering::Relaxed), 1);
    // No efuse dump; the chip is powered off after the last attempt.
    assert!(!c.io.trace.contains(&Op::W32(0x0030, 0)));
    let last_attempt = c.io.trace.iter().rposition(|o| *o == ATTEMPT).unwrap();
    let off =
        c.io.trace
            .iter()
            .rposition(|o| *o == xtal(0x90, 0x10, 0x10))
            .unwrap();
    assert!(off > last_attempt);
    assert_ends_with_power_off(&c.io);
    assert_dma_stopped_before_power_off(&c.io);
}

#[test]
fn ch12_failure_reruns_pci_pre_init_before_the_retry() {
    // CH12 never consumes the first attempt's header packet, so its submit
    // times out and stops the queue. The retry needs a fresh PCI pre-init,
    // after which the engine consumes and the download succeeds.
    fn first_pre_init(pre_inits: usize) -> bool {
        pre_inits < 2
    }
    let mut c = chip(efuse_with_mac(), 7, first_pre_init, BOTH);
    let m = c.m.clone();
    let (running, _rings) = run(&mut c.io, &m).unwrap();
    assert_eq!(running.mac, MAC);
    assert_eq!(count(&c.io, ATTEMPT), 2);
    assert_eq!(c.pre_inits.load(Ordering::Relaxed), 2);
    // The second pre-init ran between the two attempts.
    let attempts: Vec<usize> =
        c.io.trace
            .iter()
            .enumerate()
            .filter(|(_, o)| **o == ATTEMPT)
            .map(|(i, _)| i)
            .collect();
    let second_pre_init =
        c.io.trace
            .iter()
            .rposition(|o| *o == Op::R16(0x3848))
            .unwrap();
    assert!(attempts[0] < second_pre_init && second_pre_init < attempts[1]);
    assert_ends_with_power_off(&c.io);
}

#[test]
fn download_failure_names_the_first_attempt_when_it_differs() {
    // Attempt 1: CH12 never consumes the header packet, so its submit times
    // out and stops the queue. Attempts 2-5, after a fresh PCI pre-init:
    // the firmware never reports ready.
    fn first_pre_init(pre_inits: usize) -> bool {
        pre_inits < 2
    }
    let mut c = chip(efuse_with_mac(), 0, first_pre_init, BOTH);
    let m = c.m.clone();
    let err = failed(run(&mut c.io, &m));
    assert_eq!(
        err,
        BringUpError {
            step: "firmware download",
            error: AxError::FirmwareStatus {
                what: "not ready",
                fw_ctrl: 0x0000_0007,
            },
            earlier: Earlier::First {
                tries: 5,
                error: AxError::Timeout {
                    register: 0x1080,
                    mask: 0x0FFF_0000,
                },
            },
            rf: None,
        }
    );
    assert_eq!(
        err.to_string(),
        "RTL8852CE bring-up: firmware download: not ready (0x01E0=0x00000007) (first of 5 \
         attempts: timeout waiting on register 0x1080 mask 0xfff0000)"
    );
    assert_eq!(count(&c.io, ATTEMPT), 5);
    assert_eq!(c.pre_inits.load(Ordering::Relaxed), 2);
    assert_ends_with_power_off(&c.io);
}

#[test]
fn a_failed_pci_re_init_keeps_the_download_error_that_stopped_ch12() {
    // Attempt 1's header packet is never consumed, which stops CH12; the
    // PCI pre-init re-run for attempt 2 then finds the TX DMA busy (CH12,
    // R_AX_HAXI_DMA_BUSY1 bit 18) and times out.
    fn always(_: usize) -> bool {
        true
    }
    let mut c = chip(efuse_with_mac(), 7, always, BOTH);
    let m = c.m.clone();
    let mut busy_reads = 0;
    c.io.on_read(
        0x101C,
        Box::new(move |_: &mut RegisterModel| {
            busy_reads += 1;
            if busy_reads == 1 { 0 } else { 1 << 18 }
        }),
    );
    let err = failed(run(&mut c.io, &m));
    assert_eq!(
        err,
        BringUpError {
            step: "pci pre-init",
            error: AxError::Timeout {
                register: 0x101C,
                mask: 0x0007_FF00,
            },
            earlier: Earlier::Ch12Stopped {
                attempt: 1,
                stopped: AxError::Timeout {
                    register: 0x1080,
                    mask: 0x0FFF_0000,
                },
                first: None,
            },
            rf: None,
        }
    );
    assert_eq!(
        err.to_string(),
        "RTL8852CE bring-up: pci pre-init: timeout waiting on register 0x101c mask 0x7ff00 \
         (re-run after firmware download attempt 1 of 5 failed and stopped CH12: timeout \
         waiting on register 0x1080 mask 0xfff0000)"
    );
    // One download attempt; the second never started.
    assert_eq!(count(&c.io, ATTEMPT), 1);
    assert_ends_with_power_off(&c.io);
}

#[test]
fn a_ch12_stop_after_a_different_first_failure_names_both() {
    let err = BringUpError {
        step: "pci pre-init",
        error: AxError::AllOnes { register: 0x3848 },
        earlier: Earlier::Ch12Stopped {
            attempt: 3,
            stopped: AxError::Dma("ring memory visibility failed"),
            first: Some(AxError::FirmwareStatus {
                what: "checksum failed",
                fw_ctrl: 0x0000_0047,
            }),
        },
        rf: None,
    };
    assert_eq!(
        err.to_string(),
        "RTL8852CE bring-up: pci pre-init: register 0x3848 read all ones (re-run after firmware \
         download attempt 3 of 5 failed and stopped CH12: ring memory visibility failed; first \
         of 5 attempts: checksum failed (0x01E0=0x00000047))"
    );
}

#[test]
fn power_on_failure_keeps_both_errors_and_ends_powered_off() {
    // The MAC reads on at first, so rtw89_mac_pwr_on powers it off and
    // retries; then RDY_SYSPWR never comes up. Power-off itself works.
    let m = Arc::new(Mutex::new(Model::default()));
    let mut io = RegisterModel::new();
    io.on_read(
        0x0004,
        Box::new(|m: &mut RegisterModel| m.get32(0x0004) & !(1 << 17 | 1 << 8 | 1 << 9)),
    );
    io.on_read(
        0x0270,
        Box::new(|m: &mut RegisterModel| m.get32(0x0270) & 0x7FFF_FFFF),
    );
    let mut reads = 0;
    io.on_read(
        0x03F0,
        Box::new(move |_: &mut RegisterModel| {
            reads += 1;
            if reads == 1 { 0x100 } else { 0 }
        }),
    );
    let err = failed(run(&mut io, &m));
    assert_eq!(
        err,
        BringUpError {
            step: "power on",
            error: AxError::Timeout {
                register: 0x0004,
                mask: 1 << 17,
            },
            earlier: Earlier::First {
                tries: 2,
                error: AxError::AlreadyPowered,
            },
            rf: None,
        }
    );
    assert_eq!(
        err.to_string(),
        "RTL8852CE bring-up: power on: timeout waiting on register 0x4 mask 0x20000 (first of 2 \
         attempts: MAC has already powered on)"
    );
    // The retry's power-off, then the bring-up's own: the chip is left off
    // and coex is told, twice.
    assert_ends_with_power_off(&io);
    assert_eq!(count(&io, Op::W8(0x00AF, 0x80)), 2);
    assert!(m.lock().unwrap().mappings.is_empty());
}

#[test]
fn power_on_failure_still_tries_the_power_off() {
    // RDY_SYSPWR never comes up and XTAL SI never completes: both tries
    // fail the same way, and so do the retry's power-off and the bring-up's
    // own, each at its first XTAL SI command, so coex is never told.
    let m = Arc::new(Mutex::new(Model::default()));
    let mut io = RegisterModel::new();
    let err = failed(run(&mut io, &m));
    assert_eq!(
        err,
        BringUpError::new(
            "power on",
            AxError::Timeout {
                register: 0x0004,
                mask: 1 << 17,
            }
        )
    );
    assert_eq!(count(&io, xtal(0x90, 0x10, 0x10)), 2);
    let last_write = io
        .writes()
        .into_iter()
        .rfind(|op| !matches!(op, Op::Delay(_)));
    assert_eq!(last_write, Some(xtal(0x90, 0x10, 0x10)));
    assert!(!io.trace.iter().any(|o| matches!(o, Op::W8(0x00AF, _))));
    assert!(m.lock().unwrap().mappings.is_empty());
}

#[test]
fn efuse_failure_powers_off() {
    // Empty blocks up to the last header (physical 1210), which announces
    // a word past the physical limit: rtw89_dump_logical_efuse_map refuses
    // the map (efuse.c:269).
    let mut phys = vec![0xFF; 2048];
    for i in (4..1210).step_by(2) {
        phys[i] = 0x00;
        phys[i + 1] = 0x0F;
    }
    phys[1210] = 0x00;
    phys[1211] = 0x0E;
    let mut c = chip(phys, 7, never, BOTH);
    let m = c.m.clone();
    let err = failed(run(&mut c.io, &m));
    assert_eq!(err.step, "efuse");
    assert!(matches!(err.error, AxError::Efuse(_)));
    assert_ends_with_power_off(&c.io);
    assert_dma_stopped_before_power_off(&c.io);
}

#[test]
fn running_display_says_on_for_secure_boot() {
    let r = Running {
        cut: 0,
        firmware: (0, 27, 97, 0),
        mac: [0xFF, 0xAB, 0x01, 0x10, 0x00, 0x9C],
        secure_boot: true,
        mac_init: false,
        ltr: false,
        bacam_reset_timed_out: false,
        h2c: acked(false, false),
        phy: None,
    };
    // Without the MAC init no H2C was sent, so nothing is said of one.
    assert_eq!(
        r.to_string(),
        "RTL8852CE firmware running: cut 0, firmware 0.27.97.0, MAC ff:ab:01:10:00:9c, \
         secure boot on"
    );
    let r = Running {
        mac_init: true,
        ltr: true,
        ..r
    };
    assert_eq!(
        r.to_string(),
        "RTL8852CE firmware running: cut 0, firmware 0.27.97.0, MAC ff:ab:01:10:00:9c, \
         secure boot on, MAC init done, LTR on, H2C OFLD_CFG not acknowledged"
    );
    // A BA CAM reset that timed out is appended, and only then.
    let r = Running {
        bacam_reset_timed_out: true,
        ..r
    };
    assert!(
        r.to_string()
            .ends_with("H2C OFLD_CFG not acknowledged, BA CAM reset timed out"),
        "{r}"
    );
    // The radio's report comes last, and only when there is one.
    let r = Running {
        phy: Some(REPORT),
        ..r
    };
    assert!(
        r.to_string()
            .ends_with(&alloc::format!("BA CAM reset timed out{PHY}")),
        "{r}"
    );
}

#[test]
fn an_rf_failure_names_its_path_and_page_between_step_and_error() {
    let at = |path, page| BringUpError {
        rf: Some(RfAt { path, page }),
        ..BringUpError::new("rf h2c", AxError::Dma("CH12 is stopped"))
    };
    assert_eq!(
        at(RfPath::A, Some(1)).to_string(),
        "RTL8852CE bring-up: rf h2c: path A page 1: CH12 is stopped"
    );
    assert_eq!(
        at(RfPath::B, None).to_string(),
        "RTL8852CE bring-up: rf h2c: path B: CH12 is stopped"
    );
}

/// The radio steps after OFLD_CFG, each of which is torn down as M2a's
/// `"h2c"` and `"c2h"` failures are.
const PHY_STEPS: [&str; 6] = [
    "bb rf reset",
    "bb init",
    "rf init",
    "rf h2c",
    "coex init",
    "phy report",
];

/// The trace index of the OFLD_CFG kick: the first CH12 kick after the
/// efuse dump (the download's kicks all come before it).
fn ofld_cfg_kick(t: &[Op]) -> usize {
    let efuse = t.iter().position(|o| *o == Op::W32(0x0030, 0)).unwrap();
    efuse
        + t[efuse..]
            .iter()
            .position(|o| matches!(o, Op::W16(0x1080, _)))
            .unwrap()
}

#[test]
fn phy_steps_run_in_linux_order() {
    let (c, result) = ran(BOTH, NO_LTR);
    result.unwrap();
    let t = &c.io.trace;
    let first = |op: Op| {
        t.iter()
            .position(|o| *o == op)
            .unwrap_or_else(|| panic!("no {op:x?}"))
    };
    let first_reg = |reg: u32| first_of_dyn(t, &|o| matches!(o, Op::W32(r, _) if *r == reg));
    let after = |from: usize, want: &dyn Fn(&Op) -> bool| {
        from + 1
            + t[from + 1..]
                .iter()
                .position(want)
                .unwrap_or_else(|| panic!("nothing after trace entry {from}"))
    };
    let kick = |o: &Op| matches!(o, Op::W16(0x1080, _));
    let ofld_cfg = ofld_cfg_kick(t);
    // rtw89_chip_reset_bb_rf: the disable's clear of AFC_AFEDIG.
    let reset = after(ofld_cfg, &|o| matches!(o, Op::W32(0x02F0, _)));
    // rtw89_phy_init_bb_reg: the BB table, the TX power unit, the BB reset.
    let bb = first(Op::W32(0x14AD4, 0x40));
    let txpwr = first_reg(0xD248);
    let bb_reset = first(Op::W32(0x11200, 0x7000_0000));
    // rtw89_phy_init_rf_reg: path B's direct-window write and its page,
    // then path A's.
    let rf_b = first_reg(0x1F014);
    let page_b = after(rf_b, &kick);
    let rf_a = first_reg(0x1E014);
    let page_a = after(rf_a, &kick);
    // rtw89_btc_ntfy_init: R_AX_BTC_CFG first, the scoreboard last.
    let coex = first_reg(0xDA00);
    let scoreboard = first(Op::W32(0x00AC, 0x8100_4003));
    // Then rtw89_core_stop's deinit and the power-off.
    let deinit = after(scoreboard, &|o| matches!(o, Op::W32(0x0074, _)));
    let power_off = after(deinit, &|o| *o == xtal(0x90, 0x10, 0x10));
    let order = [
        ofld_cfg, reset, bb, txpwr, bb_reset, rf_b, page_b, rf_a, page_a, coex, scoreboard, deinit,
        power_off,
    ];
    assert!(order.windows(2).all(|w| w[0] < w[1]), "{order:?}");
    // Two H2Cs after OFLD_CFG, the RF pages, and no other.
    assert_eq!(
        t[ofld_cfg + 1..deinit].iter().filter(|o| kick(o)).count(),
        2
    );
    // Nothing reached SWSI before the RF init: its first busy check is path
    // B's first entry, an SWSI write, after the BB init.
    let swsi = first(Op::R32(0x1174C));
    assert!(bb_reset < swsi && swsi < rf_b, "{swsi}");
    // The GNT sample before coex, between the last page and coex's first write.
    let before = after(page_a, &|o| *o == Op::R32(0xDA4C));
    assert!(before < coex);
    // The report: RF18 A then B, GNT, the GNT SW readback, then one RXQ
    // index read, all after the scoreboard and before the deinit.
    let rf18_a = after(scoreboard, &|o| *o == Op::RfR(0, 0x18));
    let rf18_b = after(rf18_a, &|o| *o == Op::RfR(1, 0x18));
    let gnt = after(rf18_b, &|o| *o == Op::R32(0xDA4C));
    let gnt_sw = after(gnt, &|o| *o == Op::R32(0xDA48));
    let drain = after(gnt_sw, &|o| *o == Op::R32(0x1218));
    // Then rtw89_core_stop's coex half: the WL-off grant and the cleared
    // scoreboard, before the deinit.
    let wl_off = after(drain, &|o| *o == Op::W32(0xDA48, GNT_WL_OFF));
    let cleared = after(wl_off, &|o| *o == Op::W32(0x00AC, 0x8100_0000));
    assert!(cleared < deinit, "{drain} {wl_off} {cleared} {deinit}");
    assert_radio_off_before_deinit(&c.io);
    // The report reads write nothing but the RF read latches.
    assert!(
        !t[rf18_a..=drain]
            .iter()
            .any(|o| matches!(o, Op::W8(..) | Op::W16(..) | Op::RfW(..))
                || matches!(o, Op::W32(r, _) if *r != 0x10378)),
        "{:x?}",
        &t[rf18_a..=drain]
    );
}

/// The first trace entry `want` accepts.
fn first_of_dyn(t: &[Op], want: &dyn Fn(&Op) -> bool) -> usize {
    t.iter()
        .position(want)
        .unwrap_or_else(|| panic!("no such trace entry"))
}

/// CH12 slot `slot`'s first `len` bytes, as the device would read them.
fn ch12_packet(m: &Arc<Mutex<Model>>, slot: usize, len: usize) -> Vec<u8> {
    use crate::wifi::realtek::ax::pci::{
        FWCMD_SLOT_BYTES,
        tests::{FWCMD, device_read},
    };
    let state = m.lock().unwrap();
    device_read(
        &state,
        u64::from(FWCMD) + (slot * FWCMD_SLOT_BYTES) as u64,
        len,
    )
}

#[test]
fn rf_pages_follow_ofld_cfg_at_seq_1_and_2_without_an_ack_request() {
    let (c, result) = ran(BOTH, NO_LTR);
    result.unwrap();
    let t = &c.io.trace;
    let ofld_cfg = ofld_cfg_kick(t);
    let kicks: Vec<u16> = t[ofld_cfg..]
        .iter()
        .filter_map(|o| match o {
            Op::W16(0x1080, v) => Some(*v),
            _ => None,
        })
        .collect();
    assert_eq!(kicks.len(), 3, "{kicks:?}");
    // Each kick writes the host index past the slot it fills.
    let slot = |kick: u16| (usize::from(kick) + pci::TX_ENTRIES - 1) % pci::TX_ENTRIES;
    // OFLD_CFG: seq 0, which forces REC_ACK (bit 14), and DONE_ACK.
    let p = ch12_packet(&c.m, slot(kicks[0]), 24);
    assert_eq!(p[19], 0);
    assert_eq!(
        u32::from_le_bytes(p[20..24].try_into().unwrap()) & 0xC000,
        0xC000
    );
    // RF_REG_B then RF_REG_A (cat 2, class 9 then 8, page 0) at seq 1 and
    // 2, with neither ack asked for, each one stored word of RF 0x10005 = 1.
    for (kick, class, seq) in [(kicks[1], 9u8, 1u8), (kicks[2], 8, 2)] {
        let p = ch12_packet(&c.m, slot(kick), 28);
        assert_eq!(p[16], 2 | class << 2, "seq {seq}");
        assert_eq!(p[17], 0, "seq {seq}");
        assert_eq!(p[19], seq);
        let hdr1 = u32::from_le_bytes(p[20..24].try_into().unwrap());
        assert_eq!(hdr1 & 0xC000, 0, "seq {seq}");
        assert_eq!(hdr1 & 0x3FFF, 8 + 4, "seq {seq}");
        assert_eq!(
            u32::from_le_bytes(p[24..28].try_into().unwrap()),
            crate::wifi::realtek::ax::radio::rf_reg_word(0x10005, 1)
        );
    }
}

#[test]
fn phycap_is_dumped_after_the_efuse_map_and_before_the_mac_init() {
    let (c, result) = ran(BOTH, NO_LTR);
    result.unwrap();
    let t = &c.io.trace;
    // The 1216-byte map ends at 0x4BF; the phycap window is 0x590..0x5F0.
    let map_end = first_of_dyn(t, &|o| *o == Op::W32(0x0030, 0x04BF_0000));
    let phycap = first_of_dyn(t, &|o| *o == Op::W32(0x0030, 0x0590_0000));
    let phycap_end = first_of_dyn(t, &|o| *o == Op::W32(0x0030, 0x05EF_0000));
    let bb_rf = first_of_dyn(t, &|o| matches!(o, Op::W8(0x0002, _)));
    assert!(map_end < phycap && phycap < phycap_end && phycap_end < bb_rf);
    assert!(!t.contains(&Op::W32(0x0030, 0x05F0_0000)));
}

#[test]
fn phycap_failure_names_step_and_stops_the_dma() {
    // EF_RDY never comes for the phycap window once the whole map was read
    // (the secure-boot read of 0x5EC/0x5ED before it still completes).
    let err = fails_on(efuse_with_mac(), BOTH, NO_LTR, |io| {
        let phys = efuse_with_mac();
        // Set once the map's last byte, 0x4BF, has been polled.
        let mut map_read = false;
        io.on_read(
            0x0030,
            Box::new(move |m: &mut RegisterModel| {
                let ctl = m.get32(0x0030);
                let addr = ((ctl >> 16) & 0x7FF) as usize;
                map_read |= addr == 0x4BF;
                if addr >= 0x590 && map_read {
                    ctl & 0x07FF_0000
                } else {
                    (ctl & 0x07FF_0000) | 1 << 29 | u32::from(phys[addr])
                }
            }),
        );
    });
    assert_eq!(err.step, "phycap");
    assert!(
        matches!(
            err.error,
            AxError::Timeout {
                register: 0x0030,
                ..
            }
        ),
        "{err}"
    );
}

#[test]
fn phy_failure_names_step_and_powers_off() {
    // rf init: no radio headline is for RFE type 0xFF (BB, BB gain and NCTL
    // take any RFE). Path B, slot 0, is the first radio table walked.
    let err = fails_on(efuse_with_rfe(0xFF), BOTH, NO_LTR, |_| ());
    assert_eq!(
        err,
        BringUpError {
            step: "rf init",
            error: AxError::PhyPackage {
                table: "RADIO_B",
                rfe: 0xFF,
                cv: 2
            },
            earlier: Earlier::Nothing,
            rf: Some(RfAt {
                path: RfPath::B,
                page: None
            }),
        }
    );
    assert_eq!(
        err.to_string(),
        "RTL8852CE bring-up: rf init: path B: no RADIO_B branch for RFE type 255 at cut 2"
    );

    // rf init: SWSI stays write-busy. Nothing reads SWSI before the RF init
    // (see `phy_steps_run_in_linux_order`), so this holds only after
    // OFLD_CFG.
    let err = mac_init_fails(BOTH, NO_LTR, |io| io.swsi_status = 1 << 24);
    assert_eq!(err.step, "rf init");
    assert!(
        matches!(
            err.error,
            AxError::Timeout {
                register: 0x1174C,
                ..
            }
        ),
        "{err}"
    );
    assert_eq!(
        err.rf,
        Some(RfAt {
            path: RfPath::B,
            page: None
        })
    );

    // rf h2c: CH12 stops consuming once path B's table has been written, so
    // path B's page 0 is never taken.
    let err = mac_init_fails(BOTH, NO_LTR, |io| {
        io.on_read(
            0x1080,
            Box::new(|m: &mut RegisterModel| {
                let host = m.get32(0x1080) & 0xFFF;
                if m.get32(0x1F014) == 0 {
                    host | host << 16
                } else {
                    let hw = (host + pci::TX_ENTRIES as u32 - 1) % pci::TX_ENTRIES as u32;
                    host | hw << 16
                }
            }),
        );
    });
    assert_eq!(
        err,
        BringUpError {
            step: "rf h2c",
            error: AxError::Timeout {
                register: 0x1080,
                mask: 0x0FFF_0000
            },
            earlier: Earlier::Nothing,
            rf: Some(RfAt {
                path: RfPath::B,
                page: Some(0)
            }),
        }
    );
    assert!(
        err.to_string()
            .starts_with("RTL8852CE bring-up: rf h2c: path B page 0: "),
        "{err}"
    );

    // bb rf reset: the XTAL SI command that starts the reset's enable (the
    // second write of XTAL0 = 0x07) never completes; the power-off's own
    // commands do.
    let err = mac_init_fails(BOTH, NO_LTR, |io| {
        io.on_read(
            0x0270,
            Box::new(|m: &mut RegisterModel| {
                let v = m.get32(0x0270);
                let enables = m
                    .trace
                    .iter()
                    .filter(|o| **o == Op::W32(0x0270, 0x80FF_0700))
                    .count();
                if enables == 2 && v == 0x80FF_0700 {
                    v
                } else {
                    v & 0x7FFF_FFFF
                }
            }),
        );
    });
    assert_eq!(err.step, "bb rf reset");
    assert!(
        matches!(
            err.error,
            AxError::Timeout {
                register: 0x0270,
                ..
            }
        ),
        "{err}"
    );
    assert_eq!(err.rf, None);

    // coex init: the CMAC reads disabled once R_AX_BTC_CFG is set, so the
    // grant's PLT check fails.
    let err = mac_init_fails(BOTH, NO_LTR, |io| {
        io.on_read(
            0xC000,
            Box::new(|m: &mut RegisterModel| {
                if m.get32(0xDA00) != 0 {
                    0
                } else {
                    m.get32(0xC000)
                }
            }),
        );
    });
    assert_eq!(
        err,
        BringUpError::new("coex init", AxError::Mac("CMAC function is not enabled"))
    );

    // phy report: the GNT SW readback fails, after the coex init, so the
    // coex half of rtw89_core_stop still runs before the deinit
    // (`fails_on` checks it).
    let err = mac_init_fails(BOTH, NO_LTR, |io| io.fail_reads_of(0xDA48));
    assert_eq!(err, BringUpError::new("phy report", READ_FAILED));

    // coex init: the grant sample before it fails; coex never ran, so
    // nothing of it is undone.
    let err = mac_init_fails(BOTH, NO_LTR, |io| io.fail_reads_of(0xDA4C));
    assert_eq!(err, BringUpError::new("coex init", READ_FAILED));
}

#[test]
fn tables_missing_one_are_refused_before_power_on() {
    let mut c = chip(efuse_with_mac(), 7, never, BOTH);
    let m = c.m.clone();
    let file = tables_file();
    let mut tables = phy_tables(&file, 0).unwrap();
    tables.nctl = None;
    let err = failed(run_with(&mut c.io, &m, NO_LTR, &tables));
    assert_eq!(
        err,
        BringUpError::new("phy tables", AxError::Phy("firmware carries no NCTL table"))
    );
    assert!(c.io.trace.is_empty(), "{:x?}", c.io.trace);
}

/// Power hooks only: RDY_SYSPWR reads set, APFN_ONMAC and APFM_OFFMAC
/// self-clear, XTAL SI commands complete, and 0x03F0 reads `state`.
fn quiet(state: u32) -> RegisterModel {
    let mut io = RegisterModel::new();
    io.on_read(
        0x0004,
        Box::new(|m: &mut RegisterModel| (m.get32(0x0004) | 1 << 17) & !(1 << 8 | 1 << 9)),
    );
    io.on_read(
        0x0270,
        Box::new(|m: &mut RegisterModel| m.get32(0x0270) & 0x7FFF_FFFF),
    );
    io.set32(0x03F0, state);
    io
}

#[test]
fn quiesce_leaves_a_mac_that_reads_off_alone() {
    let mut io = quiet(0x000);
    assert_eq!(quiesce(&mut io), Ok(false));
    assert!(io.writes().is_empty());
}

#[test]
fn quiesce_refuses_a_mac_in_lps_or_an_undefined_state_and_writes_nothing() {
    for (state, field) in [(0x200, 2), (0x300, 3)] {
        let mut io = quiet(state);
        assert_eq!(quiesce(&mut io), Err(AxError::MacPowerState(field)));
        assert!(io.writes().is_empty(), "{state:#x}");
    }
    assert_eq!(
        AxError::MacPowerState(2).to_string(),
        "the MAC is in firmware-managed low power (LPS), so its firmware may still run; Bus \
         Master stays off; power the machine fully off (not restart) and boot again"
    );
    assert_eq!(
        AxError::MacPowerState(3).to_string(),
        "the MAC power state reads 3, which rtw89 does not define; Bus Master stays off"
    );
}

#[test]
fn quiesce_stops_the_pcie_dma_then_powers_off_a_mac_left_on_and_tells_coex() {
    let mut io = quiet(0x100);
    assert_eq!(quiesce(&mut io), Ok(true));
    let mut expected = quiet(0x100);
    pci::stop_dma(&mut expected).unwrap();
    power::power_off(&mut expected).unwrap();
    let mut expected = expected.writes();
    expected.push(Op::W8(0x00AF, 0x80));
    assert_eq!(io.writes(), expected);
    // Linux's exact bits: STOP_WPDMA (0x1010 bit19), then STOP_AXI_MST
    // (0x1000 bit17) set and TXHCI_EN/RXHCI_EN (bits 7, 15) cleared, all
    // before the power-off's first XTAL SI command.
    assert_eq!(
        io.writes()[..3],
        [
            Op::W32(0x1010, 1 << 19),
            Op::W32(0x1000, 1 << 17),
            Op::W32(0x1000, 1 << 17),
        ]
    );
}

#[test]
fn quiesce_refuses_a_device_that_reads_all_ones() {
    let mut io = quiet(u32::MAX);
    assert_eq!(quiesce(&mut io), Err(AxError::AllOnes { register: 0x03F0 }));
    assert!(io.writes().is_empty());
}

#[test]
fn quiesce_reports_a_power_off_that_never_completes_without_the_scoreboard() {
    let mut io = RegisterModel::new();
    io.on_read(
        0x0270,
        Box::new(|m: &mut RegisterModel| m.get32(0x0270) & 0x7FFF_FFFF),
    );
    io.set32(0x03F0, 0x100);
    assert_eq!(
        quiesce(&mut io),
        Err(AxError::Timeout {
            register: 0x0004,
            mask: 1 << 9
        })
    );
    assert!(!io.writes().contains(&Op::W8(0x00AF, 0x80)));
}

#[test]
fn a_firmware_file_error_is_its_message_alone() {
    assert_eq!(
        AxError::FirmwareFile("rtw89/rtw8852c_fw-2.bin is not on the USB stick").to_string(),
        "rtw89/rtw8852c_fw-2.bin is not on the USB stick"
    );
}

#[test]
fn every_bring_up_line_names_its_subsystem_once() {
    for (err, line) in [
        (
            BringUpError::new("efuse", AxError::Efuse("logical map out of range")),
            "RTL8852CE bring-up: efuse: logical map out of range",
        ),
        (
            BringUpError::new("dma rings", AxError::Dma("out of ring memory")),
            "RTL8852CE bring-up: dma rings: out of ring memory",
        ),
        (
            BringUpError::new(
                "firmware download",
                AxError::Firmware("WCPU already enabled"),
            ),
            "RTL8852CE bring-up: firmware download: WCPU already enabled",
        ),
        (
            BringUpError::new(
                "firmware download",
                AxError::FirmwareStatus {
                    what: "H2C path not ready",
                    fw_ctrl: 0x0000_0001,
                },
            ),
            "RTL8852CE bring-up: firmware download: H2C path not ready (0x01E0=0x00000001)",
        ),
        (
            BringUpError::new("quiesce", AxError::MacPowerState(2)),
            "RTL8852CE bring-up: quiesce: the MAC is in firmware-managed low power (LPS), so its \
             firmware may still run; Bus Master stays off; power the machine fully off (not \
             restart) and boot again",
        ),
    ] {
        assert_eq!(err.to_string(), line);
    }
}

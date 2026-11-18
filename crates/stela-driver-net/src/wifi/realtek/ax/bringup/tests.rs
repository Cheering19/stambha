extern crate std;
use super::*;
use crate::wifi::realtek::ax::fwdl::tests::image_bytes;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use crate::wifi::realtek::beacon::tests::{Domain, Model, Source};
use crate::wifi::realtek::firmware::{Ce, Rtw89};
use alloc::{boxed::Box, string::ToString, sync::Arc, vec, vec::Vec};
use core::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

/// A MAC whose bytes need lower-case hex and a leading zero.
const MAC: [u8; 6] = [0x00, 0xE0, 0x4C, 0x88, 0x52, 0x0C];

/// A 2048-byte physical efuse (0xFF = blank) whose logical map holds `MAC`
/// at 0x400: one block header at physical 4 (block 0x80, words 0-2), then
/// the six bytes. 0x5EC/0x5ED are left blank: secure boot off.
fn efuse_with_mac() -> Vec<u8> {
    let mut phys = vec![0xFF; 2048];
    phys[4..6].copy_from_slice(&[0x08, 0x08]);
    phys[6..12].copy_from_slice(&MAC);
    phys
}

/// The chip as every bring-up poll wants it:
/// - power: RDY_SYSPWR (0x0004 bit17) reads set, APFN_ONMAC and APFM_OFFMAC
///   (bits 8, 9) self-clear, and XTAL SI commands (0x0270 bit31) complete;
/// - efuse: 0x0030 answers EF_RDY with the byte at the written address;
/// - DMAC: the WDE and PLE managers (0x8D00, 0x9100) report ready;
/// - PCI: the DMA busy registers read idle (zero) and the BD RAM reset
///   (0x1000 bit3) self-clears; the CH12 engine (0x1080) consumes every
///   kicked entry at once, unless `ch12_holds` says to hold it;
/// - WCPU: once the WCPU is enabled (0x0088 bit1), 0x01E0 reports both path
///   ready bits and download status `status` in [7:5].
///
/// `pre_inits` counts PCI pre-inits by their read of TXBD_RWPTR_CLR1.
struct Chip {
    io: RegisterModel,
    pre_inits: Arc<AtomicUsize>,
}

fn chip(phys: Vec<u8>, status: u32, ch12_holds: fn(usize) -> bool) -> Chip {
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
        0x1014,
        Box::new(move |m: &mut RegisterModel| {
            counted.fetch_add(1, Ordering::Relaxed);
            m.get32(0x1014)
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
    Chip { io, pre_inits }
}

fn never(_: usize) -> bool {
    false
}

fn run(io: &mut RegisterModel, m: &Arc<Mutex<Model>>) -> Result<(Running, Rings), BringUpError> {
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    // SAFETY: the model domain isolates only the modeled requester behind
    // this test's `RegisterModel`; its pages are plain kept test RAM that no
    // real device can reach, and this test is the only worker.
    unsafe { bring_up(io, &Source, Arc::new(Domain(m.clone())), 2, &fw.image) }
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
/// (`rtw8852c.c:439-530`), then `rtw89_mac_update_scoreboard(
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
    // The last APFM_SWLPS write sets bit 10 (rtw8852c.c:502).
    assert!(matches!(got[tail.len() - 2], Op::W32(0x0004, v) if v & 1 << 10 != 0));
}

#[test]
fn happy_path_reports_running() {
    let m = Arc::new(Mutex::new(Model::default()));
    let mut c = chip(efuse_with_mac(), 7, never);
    let (running, rings) = run(&mut c.io, &m).unwrap();
    assert_eq!(
        running,
        Running {
            cut: 2,
            firmware: (0, 27, 66, 1),
            mac: MAC,
            secure_boot: false,
        }
    );
    assert_eq!(
        running.to_string(),
        "RTL8852CE firmware running: cut 2, firmware 0.27.66.1, MAC 00:e0:4c:88:52:0c, \
         secure boot off"
    );
    // One download attempt, one PCI pre-init.
    assert_eq!(count(&c.io, ATTEMPT), 1);
    assert_eq!(c.pre_inits.load(Ordering::Relaxed), 1);
    // rtw89_chip_info_setup ends in rtw89_mac_pwr_off on success too.
    assert_ends_with_power_off(&c.io);
    // The rings were programmed: the caller keeps them.
    assert!(!m.lock().unwrap().mappings.is_empty());
    drop(rings);
}

#[test]
fn happy_path_runs_the_steps_in_linux_order() {
    let m = Arc::new(Mutex::new(Model::default()));
    let mut c = chip(efuse_with_mac(), 7, never);
    run(&mut c.io, &m).unwrap();
    let t = &c.io.trace;
    let first = |op: Op| t.iter().position(|o| *o == op).unwrap();
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
    let order = [
        power_on,
        secure,
        tp_major,
        mac_pre_init,
        pci_pre_init,
        download,
        efuse,
    ];
    assert!(order.windows(2).all(|w| w[0] < w[1]), "{order:?}");
    // Exactly one TP_MAJOR and one PWR_MAJOR scoreboard write.
    assert_eq!(count(&c.io, Op::W8(0x00AF, 0x81)), 1);
    assert_eq!(count(&c.io, Op::W8(0x00AF, 0x80)), 1);
}

#[test]
fn secure_boot_on_refuses_and_powers_off() {
    // 0x5EC = 0xFF, 0x5ED = 0x7F: v0 key table row 0 (efuse.c:459-480).
    let mut phys = efuse_with_mac();
    phys[0x5EC] = 0xFF;
    phys[0x5ED] = 0x7F;
    let m = Arc::new(Mutex::new(Model::default()));
    let mut c = chip(phys, 7, never);
    let err = failed(run(&mut c.io, &m));
    assert_eq!(
        err,
        BringUpError {
            step: "secure boot",
            error: AxError::SecureBootUnsupported,
        }
    );
    assert_eq!(
        err.to_string(),
        "RTL8852CE bring-up: secure boot: secure-boot firmware is unsupported"
    );
    assert_ends_with_power_off(&c.io);
    // Nothing past the secure-boot read: no rings, no MAC pre-init.
    assert!(m.lock().unwrap().mappings.is_empty());
    assert!(!c.io.trace.contains(&Op::R32(0x7880)));
}

#[test]
fn download_retries_five_times_then_fails() {
    // The firmware never reports WCPU_FW_INIT_RDY (status stays 0).
    let m = Arc::new(Mutex::new(Model::default()));
    let mut c = chip(efuse_with_mac(), 0, never);
    let err = failed(run(&mut c.io, &m));
    assert_eq!(
        err,
        BringUpError {
            step: "firmware download",
            error: AxError::Firmware("not ready"),
        }
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
}

#[test]
fn ch12_failure_reruns_pci_pre_init_before_the_retry() {
    // CH12 never consumes the first attempt's header packet, so its submit
    // times out and stops the queue. The retry needs a fresh PCI pre-init,
    // after which the engine consumes and the download succeeds.
    fn first_pre_init(pre_inits: usize) -> bool {
        pre_inits < 2
    }
    let m = Arc::new(Mutex::new(Model::default()));
    let mut c = chip(efuse_with_mac(), 7, first_pre_init);
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
            .rposition(|o| *o == Op::R32(0x1014))
            .unwrap();
    assert!(attempts[0] < second_pre_init && second_pre_init < attempts[1]);
    assert_ends_with_power_off(&c.io);
}

#[test]
fn power_on_failure_does_not_power_off_again() {
    // RDY_SYSPWR never comes up and XTAL SI never completes:
    // rtw89_mac_pwr_on's own power-off and retry fail, and the bring-up adds
    // nothing after them.
    let m = Arc::new(Mutex::new(Model::default()));
    let mut io = RegisterModel::new();
    let err = failed(run(&mut io, &m));
    assert_eq!(err.step, "power on");
    assert!(matches!(err.error, AxError::Timeout { .. }));
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
    let m = Arc::new(Mutex::new(Model::default()));
    let mut c = chip(phys, 7, never);
    let err = failed(run(&mut c.io, &m));
    assert_eq!(err.step, "efuse");
    assert!(matches!(err.error, AxError::Efuse(_)));
    assert_ends_with_power_off(&c.io);
}

#[test]
fn running_display_says_on_for_secure_boot() {
    let r = Running {
        cut: 0,
        firmware: (0, 27, 97, 0),
        mac: [0xFF, 0xAB, 0x01, 0x10, 0x00, 0x9C],
        secure_boot: true,
    };
    assert_eq!(
        r.to_string(),
        "RTL8852CE firmware running: cut 0, firmware 0.27.97.0, MAC ff:ab:01:10:00:9c, \
         secure boot on"
    );
}

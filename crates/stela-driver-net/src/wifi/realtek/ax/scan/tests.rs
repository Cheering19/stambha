extern crate std;
use super::*;
use crate::wifi::realtek::ax::bb::BbGain;
use crate::wifi::realtek::ax::bringup::FirmwareFile;
use crate::wifi::realtek::ax::bringup::tests::{
    BOTH, NO_LTR, chip_hearing, depth_free, efuse_with_mac, never, same_access, tables_file,
    trace_sha256, txpwr_window,
};
use crate::wifi::realtek::ax::chan::Chan;
use crate::wifi::realtek::ax::channel::{ChannelInputs, TxPower};
use crate::wifi::realtek::ax::efuse::RxGain;
use crate::wifi::realtek::ax::fwdl::tests::image_bytes;
use crate::wifi::realtek::ax::h2c::H2c;
use crate::wifi::realtek::ax::live::{self, BOOT_CHANNEL, Live, StartError, leave_ips};
use crate::wifi::realtek::ax::pci::tests::{FWCMD, device_read, running};
use crate::wifi::realtek::ax::pci::{FWCMD_SLOT_BYTES, Rings, TX_ENTRIES_CTRL};
use crate::wifi::realtek::ax::receive::{Mode, Tag};
use crate::wifi::realtek::ax::regd::{Regd, Regds};
use crate::wifi::realtek::ax::tests::firmware::{
    Firmware, frame_with_status, mgnt_frame, ppdu_status, rx_buffer, scanofld,
};
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use crate::wifi::realtek::ax::txpwr::set_txpwr;
use crate::wifi::realtek::ax::txpwr_tables::TxPowerTables;
use crate::wifi::realtek::beacon::tests::{Domain, Model, Source};
use crate::wifi::realtek::firmware::{Ce, Rtw89};
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec;
use sha2::{Digest, Sha256};
use std::sync::Mutex;

const TEST_MAC: [u8; 6] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
const HOME: [u8; 6] = [0x02, 0xAA, 0, 0, 0, 1];
const OFFICE: [u8; 6] = [0x02, 0xAA, 0, 0, 0, 2];
const BROADCAST: [u8; 6] = [0xFF; 6];
const FIRMWARE: (u8, u8, u8, u8) = (0, 27, 129, 4);
/// Listed channel indices of 2G 5, 6, 7, 13, 5G 36 and 5G 165.
const CH5: usize = 4;
const CH6: usize = 5;
const CH7: usize = 6;
const CH13: usize = 12;
const CH36: usize = 13;
const CH165: usize = 37;
/// SCANOFLD_RSP reasons (`fw.h:228-239`).
const ENTER_CH: u8 = 3;

/// A running chip whose CMAC is on, whose RF 0x18 is M2b's end state on
/// both paths and whose RX filter is M2a's (`[21:16]` = 23 over the default
/// filter).
fn chip() -> (Arc<Mutex<Model>>, Rings, RegisterModel) {
    let (m, rings, mut io) = running();
    io.emulate_rf();
    for path in 0..2 {
        io.set_rf(path, 0x18, 0x01001);
    }
    io.set32(0xC000, 1 << 30);
    io.set32(0xCE20, 0x0317_44BE);
    (m, rings, io)
}

/// The world regd on both bands.
const WW: Regds = Regds {
    g2: Regd::Ww,
    g5: Regd::Ww,
};

/// M2c's `run` from M2b's H2C sequence (3) with firmware version
/// `firmware`, the test MAC, a default BB gain table, no valid efuse RX gain
/// offsets and zeroed TX power tables at the world regd: on one session,
/// the live chip's [`leave_ips`] steps on 2.4 GHz channel 1, then the
/// passive scan.
fn scan_on(
    rings: &mut Rings,
    io: &mut RegisterModel,
    firmware: (u8, u8, u8, u8),
) -> (Result<ScanReport, ScanError>, H2c) {
    let gain = BbGain::default();
    let rx = RxGain {
        valid: false,
        offset: [[-1; 13]; 2],
    };
    let channel = ChannelInputs {
        gain: &gain,
        rx_gain: &rx,
        offset_base: 0,
        cut: 1,
    };
    let tables = TxPowerTables::default();
    let power = TxPower {
        tables: &tables,
        regds: WW,
    };
    let inputs = ScanInputs {
        mac: TEST_MAC,
        firmware,
    };
    let boot = Chan::new20(BOOT_CHANNEL.band, BOOT_CHANNEL.channel).unwrap();
    let mut h2c = H2c::at(3);
    let mut session = Session::new(rings, &mut h2c, BOOT_CHANNEL);
    // SAFETY: `io` is the modeled requester that `running` programmed with
    // these rings through `pre_init`; the model domain lets no real device
    // reach the published IOVAs, and this test is the only worker.
    let got = unsafe { leave_ips(io, &mut session, TEST_MAC, &boot, &channel, power) }
        // SAFETY: as above.
        .and_then(|()| unsafe { passive(io, &mut session, &inputs) });
    drop(session);
    (got, h2c)
}

struct Scanned {
    got: Result<ScanReport, ScanError>,
    io: RegisterModel,
    m: Arc<Mutex<Model>>,
    h2c: H2c,
}

impl Scanned {
    fn report(&self) -> ScanReport {
        self.got.unwrap_or_else(|e| panic!("{e:?}"))
    }
}

/// `run` on [`chip`] with the firmware `script` sets up, at `firmware`.
fn scan_at(
    firmware: (u8, u8, u8, u8),
    script: impl FnOnce(&mut Firmware, &mut RegisterModel),
) -> Scanned {
    let (m, mut rings, mut io) = chip();
    let mut fw = Firmware::new(&m);
    script(&mut fw, &mut io);
    fw.install(&mut io);
    let (got, h2c) = scan_on(&mut rings, &mut io, firmware);
    Scanned { got, io, m, h2c }
}

fn scan(script: impl FnOnce(&mut Firmware)) -> Scanned {
    scan_at(FIRMWARE, |fw, _| script(fw))
}

/// Trace indices of the CH12 kicks: the kick of seq `3 + k` is entry `k`.
fn kicks(trace: &[Op]) -> Vec<usize> {
    trace
        .iter()
        .enumerate()
        .filter(|(_, o)| matches!(o, Op::W16(0x1080, _)))
        .map(|(i, _)| i)
        .collect()
}

/// The first index past `after` whose op `want` matches.
fn first_after(trace: &[Op], after: usize, want: impl Fn(&Op) -> bool) -> usize {
    after
        + 1
        + trace[after + 1..]
            .iter()
            .position(want)
            .unwrap_or_else(|| panic!("not in the trace after {after}"))
}

/// The virtual time at trace index `at`: reads cost nothing in [`chip`].
fn time_at(trace: &[Op], at: usize) -> u64 {
    trace[..at]
        .iter()
        .map(|o| match o {
            Op::Delay(us) => *us,
            _ => 0,
        })
        .sum()
}

/// The first `len` bytes of the FWCMD slot that carried `seq`.
fn slot(m: &Arc<Mutex<Model>>, seq: u8, len: usize) -> Vec<u8> {
    let n = usize::from(seq - 3) % TX_ENTRIES_CTRL;
    let state = m.lock().unwrap();
    device_read(
        &state,
        u64::from(FWCMD) + (n * FWCMD_SLOT_BYTES) as u64,
        len,
    )
}

fn listed(r: &ScanReport) -> Vec<String> {
    r.listed().map(|l| l.to_string()).collect()
}

fn network(bssid: [u8; 6], band: HwBand, channel: u8, signal: Option<i8>, ssid: &[u8]) -> Network {
    let mut bytes = [0; 32];
    bytes[..ssid.len()].copy_from_slice(ssid);
    Network {
        bssid,
        band,
        channel,
        signal,
        ssid: bytes,
        ssid_len: ssid.len() as u8,
        hidden: false,
        frames: 1,
    }
}

/// What a live chip started on `bringup::tests::chip_hearing` with BOTH acks
/// and `script` scanned with [`run_once`], with the receiver's mode and tag
/// once `then` ran on it, and the trace once stopped.
struct Ran {
    got: Result<ScanReport, ScanError>,
    h2c: H2c,
    mode: Mode,
    tag: Tag,
    io: RegisterModel,
}

fn run_once_hearing(script: &dyn Fn(&mut Firmware)) -> Ran {
    run_once_then(script, &|_| {})
}

fn run_once_then(script: &dyn Fn(&mut Firmware), then: &dyn Fn(&mut Live<RegisterModel>)) -> Ran {
    let c = chip_hearing(efuse_with_mac(), 7, never, BOTH, script);
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    let file = tables_file();
    let firmware = FirmwareFile {
        image: &fw.image,
        file: &file,
    };
    // SAFETY: the model domain isolates only the modeled requester behind
    // this test's `RegisterModel`; its pages are plain kept test RAM that no
    // real device can reach, and this test is the only worker.
    let started = unsafe {
        live::start(
            c.io,
            &Source,
            Arc::new(Domain(c.m.clone())),
            2,
            firmware,
            NO_LTR,
            None,
        )
    };
    let mut live: Live<RegisterModel> = match started {
        Ok(live) => live,
        Err(StartError { error, .. }) => panic!("{error}"),
    };
    // SAFETY: as above; the live chip is this test's alone.
    let got = unsafe { run_once(&mut live) };
    then(&mut live);
    let (h2c, mode, tag) = (live.h2c(), live.receiver().mode(), live.receiver().tag());
    // SAFETY: as above.
    let (io, _rings, _pools) = unsafe { live.stop() };
    Ran {
        got,
        h2c,
        mode,
        tag,
        io,
    }
}

#[test]
fn scan_runs_linux_order_and_reports() {
    // M2c's scan through the live chip: start, run_once, stop.
    let s = run_once_hearing(&|fw| {
        fw.beacon(CH6, HOME, b"Home", 0x70);
        fw.beacon(CH36, OFFICE, b"Office", 0x50);
    });
    let r = s.got.unwrap_or_else(|e| panic!("{e:?}"));
    assert_eq!((r.channels, r.entered, r.end), (38, 38, ScanEnd::Ok));
    assert_eq!((r.frames, r.without_rssi, r.dropped), (2, 0, 0));
    assert_eq!((r.rxq_full, r.refused, r.networks_total), (0, None, 2));
    assert_eq!(
        r.networks[0],
        Some(network(HOME, HwBand::G2, 6, Some(-54), b"Home"))
    );
    assert_eq!(
        r.networks[1],
        Some(network(OFFICE, HwBand::G5, 36, Some(-70), b"Office"))
    );
    assert!(r.networks[2..].iter().all(Option::is_none));
    assert_eq!(
        r.to_string(),
        format!(
            ", scan 38/38 channels in {} ms, end ok: 2 networks from 2 frames (0 without RSSI)",
            r.elapsed_ms
        )
    );
    assert_eq!(
        listed(&r),
        [
            "WIFI NETWORK 1/2 2G ch 6 -54 dBm 02:aa:00:00:00:01 \"Home\"",
            "WIFI NETWORK 2/2 5G ch 36 -70 dBm 02:aa:00:00:00:02 \"Office\"",
        ]
    );
    // Seq 3 to 19: core start, vif init, then the CAM, ADD_SCANOFLD_CH and
    // SCANOFLD of the scan start; no stop.
    assert_eq!(s.h2c, H2c::at(20));
    // Linux order after the calibration (dm tail's cfg_txrx_path): core
    // start, then set channel (RF 0x18 on both paths), then vif init (port
    // 0), then the scan start.
    let t = &s.io.trace;
    let dm_tail = t.iter().position(|o| *o == Op::W32(0x14978, 3)).unwrap();
    let k: Vec<usize> = kicks(t).into_iter().filter(|&i| i > dm_tail).collect();
    assert_eq!(k.len(), 17);
    let ppdu = first_after(t, dm_tail, |o| *o == Op::W32(0xCE40, 0x2B));
    let rf = first_after(t, dm_tail, |o| matches!(o, Op::RfW(_, 0x18, ..)));
    let port = first_after(t, dm_tail, |o| *o == Op::R32(0xC400));
    assert!(ppdu < k[0] && k[8] < rf && rf < port && port < k[9]);
    assert_eq!(s.io.get_rf(0, 0x18), 0x01C01);
    assert_eq!(s.io.get_rf(1, 0x18), 0x01C01);
}

/// The trace of M2c's `scan_runs_linux_order_and_reports` at `b4b9ab57e`
/// (its `run` from core start on this module's [`chip`]), as the SHA-256 of
/// one `{op:?}` line per access.
const M2C_SCAN_TRACE: &str = "caaf79f2cce9186e63ad11e992ba6aaeeed01d9577f8f418d1daa9ca794938fd";

#[test]
fn the_scan_steps_are_m2cs_with_set_txpwr_alone_inserted_before_help_exit() {
    let s = scan(|fw| {
        fw.beacon(CH6, HOME, b"Home", 0x70);
        fw.beacon(CH36, OFFICE, b"Office", 0x50);
    });
    s.report();
    let t = &s.io.trace;
    let mut alone = RegisterModel::new();
    let ch1 = Chan::new20(HwBand::G2, 1).unwrap();
    set_txpwr(&mut alone, &TxPowerTables::default(), WW, &ch1).unwrap();
    let window = txpwr_window(t, 0);
    assert_eq!(window.len(), 119);
    assert!(same_access(&alone.trace, &t[window.clone()]));
    assert_eq!(
        t[window.end..window.end + 2],
        [Op::R32(0xC000), Op::W32(0xCE40, 0x2B)]
    );
    // Its view across M4's ring depths is the same trace: the scan writes no
    // ring size or address, and RXQ's host index stays under 32.
    let rest = depth_free(t[..window.start].iter().chain(&t[window.end..]));
    assert_eq!(trace_sha256(&rest), M2C_SCAN_TRACE);
}

/// A data frame on channel 6 during the scan: M2c's scan receiver counts it
/// among its other frames, where a station receiver would deliver it and
/// the scanner refuse it as dropped.
fn home_and_data(fw: &mut Firmware) {
    fw.beacon(CH6, HOME, b"Home", 0x70);
    fw.buffers(
        CH6,
        vec![mgnt_frame(0x08, BROADCAST, HOME, b"data", None, 9)],
    );
}

#[test]
fn run_once_scans_with_the_scan_receiver_and_leaves_the_restore_to_end_scan() {
    let s = run_once_hearing(&home_and_data);
    let r = s.got.unwrap_or_else(|e| panic!("{e:?}"));
    assert_eq!((r.frames, r.dropped, r.networks_total), (1, 0, 1));
    // run_once runs none of rtw89_hw_scan_complete's restores: the scan's RX
    // filter is still on, so the scan's receiver stays, tagging with the
    // channel the firmware left the radio on, the last ENTER_CH (5 GHz
    // 165).
    assert_eq!(s.mode, Mode::Scan);
    assert_eq!(
        s.tag,
        Tag {
            band: HwBand::G5,
            channel: 165
        }
    );
}

#[test]
fn end_scan_after_run_once_restores_the_operating_channel_and_the_station_receiver() {
    let s = run_once_then(&home_and_data, &|live| {
        assert_eq!(live.scan_state(), Some(ScanEnd::Ok));
        assert_eq!(live.io().get32(0xCE20), 0x0317_4438);
        // SAFETY: the live chip is this test's alone, as in
        // `run_once_then`.
        unsafe { live.end_scan() }.unwrap();
        assert_eq!(live.scan_state(), None);
        // The station's RX filter again.
        assert_eq!(live.io().get32(0xCE20), 0x0317_44BE);
    });
    s.got.unwrap_or_else(|e| panic!("{e:?}"));
    // The address CAM of the restore took seq 20.
    assert_eq!(s.h2c, H2c::at(21));
    assert_eq!(s.mode, Mode::Station);
    assert_eq!(s.tag, BOOT_CHANNEL);
}

#[test]
fn scan_list_and_budget_follow_m2cs_rules() {
    // Spec S8.1: 2 * n * 105 ms + 1 s; M2c's 38 channels give its 8 980 ms,
    // the keeper's largest list (14 + 28) 9 820 ms.
    assert_eq!(scan_budget_us(SCAN_CHANNELS.len()), SCAN_WAIT_US);
    assert_eq!(SCAN_WAIT_US, 8_980_000);
    assert_eq!(scan_budget_us(3), 1_630_000);
    assert_eq!(scan_budget_us(42), 9_820_000);
    // Each channel passive, as M2c's list has it.
    let list = scan_list(&[(HwBand::G2, 1), (HwBand::G5, 36), (HwBand::G5, 165)]).unwrap();
    assert_eq!(
        list,
        [SCAN_CHANNELS[0], SCAN_CHANNELS[13], SCAN_CHANNELS[37]]
    );
    let all: Vec<(HwBand, u8)> = SCAN_CHANNELS.iter().map(|c| (c.band, c.channel)).collect();
    assert_eq!(scan_list(&all).unwrap(), SCAN_CHANNELS);
    // Nothing to scan, and channels no 20 MHz switch here could tune.
    assert_eq!(scan_list(&[]), Err(AxError::Scan("empty scan list")));
    for bad in [(HwBand::G6, 37), (HwBand::G2, 15), (HwBand::G5, 32)] {
        assert_eq!(
            scan_list(&[(HwBand::G2, 1), bad]),
            Err(AxError::Scan(
                "scan channel not a 20 MHz 2.4 or 5 GHz channel"
            )),
            "{bad:?}"
        );
    }
}

#[test]
fn scan_start_matches_linux_sequence() {
    // DIG's register starts all ones, so both of its clears show.
    let s = scan_at(FIRMWARE, |_, io| io.set32(0x1481C, 0xFFFF_FFFF));
    s.report();
    let t = &s.io.trace;
    let k = kicks(t);
    // Kick 13 is seq 16, vif init's last H2C.
    let vif_end = k[13];
    let rx_sync = first_after(t, vif_end, |o| matches!(o, Op::W32(0xC400, _)));
    let Op::W32(_, port_cfg) = t[rx_sync] else {
        unreachable!()
    };
    assert_eq!(port_cfg & 0x8, 0);
    assert_eq!(t[rx_sync - 1], Op::R32(0xC400));
    let mit = first_after(t, vif_end, |o| *o == Op::W32(0x1184, 0));
    let edcca_reads: Vec<usize> = t
        .iter()
        .enumerate()
        .skip(vif_end)
        .filter(|(_, o)| **o == Op::R32(0x14840))
        .map(|(i, _)| i)
        .take(3)
        .collect();
    let edcca = first_after(t, vif_end, |o| *o == Op::W32(0x14840, 0xF9));
    let fltr = first_after(t, vif_end, |o| *o == Op::W32(0xCE20, 0x0317_4438));
    let dig = first_after(t, vif_end, |o| matches!(o, Op::W32(0x1481C, _)));
    assert!(rx_sync < mit, "{rx_sync} {mit}");
    assert!(mit < edcca_reads[0] && edcca_reads[2] < edcca);
    assert!(edcca < k[14] && k[14] < fltr && fltr < dig && dig < k[15] && k[15] < k[16]);
    // EDCCA [7:0], [15:8] and [31:24] at EDCCA_MAX, DIG's PD lower bound and
    // spatial reuse off.
    assert_eq!(s.io.get32(0x14840), 0xF900_F9F9);
    let dig_writes: Vec<Op> = t[vif_end..]
        .iter()
        .copied()
        .filter(|o| matches!(o, Op::W32(0x1481C, _)))
        .collect();
    assert_eq!(
        dig_writes,
        [Op::W32(0x1481C, 0xFFFF_F83F), Op::W32(0x1481C, 0xDFFF_F83F)]
    );
    // Seq 17 is the address CAM again, byte for byte but the seq (spec S7.4).
    assert_eq!(slot(&s.m, 17, 16 + 68)[16..20], [0x19, 0x00, 0x00, 0x11]);
    assert_eq!(slot(&s.m, 17, 16 + 68)[20..], slot(&s.m, 15, 16 + 68)[20..]);
}

#[test]
fn scan_offload_h2cs_match_linux_bytes() {
    let s = scan(|_| {});
    s.report();
    // ADD_SCANOFLD_CH at seq 18 (spec S9.2).
    let add = slot(&s.m, 18, 1092);
    let mut head = vec![0x34, 0x04, 0x00, 0x0D];
    head.extend_from_slice(&[0; 12]);
    head.extend_from_slice(&[0x25, 0x16, 0x00, 0x12, 0x34, 0xC4, 0x00, 0x00]);
    assert_eq!(add[..24], head[..]);
    assert_eq!(add[24..28], [0x26, 0x07, 0x00, 0x00]);
    assert_eq!(
        add[28..36],
        [0x69, 0x14, 0x01, 0x01, 0xF8, 0x30, 0xFF, 0x01]
    );
    let ch36 = 28 + 13 * 28;
    assert_eq!(
        add[ch36..ch36 + 8],
        [0x69, 0x14, 0x24, 0x24, 0xF8, 0x70, 0xFF, 0x01]
    );
    assert_eq!(
        <[u8; 32]>::from(Sha256::digest(&add[24..])),
        [
            0x03, 0xfc, 0x97, 0xf3, 0xc0, 0x50, 0x54, 0x02, 0xe7, 0xe6, 0xc2, 0xde, 0x29, 0x10,
            0x51, 0x72, 0x6a, 0xc7, 0xcc, 0xa6, 0x85, 0x52, 0x53, 0x46, 0x7a, 0xfd, 0x32, 0x1b,
            0xf2, 0xd9, 0xef, 0x53
        ]
    );
    // SCANOFLD start at seq 19 (spec S9.3).
    let mut start = vec![0x24, 0x00, 0x00, 0x0D];
    start.extend_from_slice(&[0; 12]);
    start.extend_from_slice(&[0x25, 0x17, 0x00, 0x13, 0x24, 0xC0, 0x00, 0x00]);
    start.extend_from_slice(&[0x00, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00]);
    start.extend_from_slice(&[0; 20]);
    assert_eq!(slot(&s.m, 19, 52), start);
}

#[test]
fn scan_channels_are_the_passive_list() {
    assert_eq!(SCAN_CHANNELS.len(), 38);
    assert!(
        SCAN_CHANNELS
            .iter()
            .all(|c| { (c.period, c.dwell, c.dfs) == (105, 20, true) })
    );
    let g2: Vec<u8> = SCAN_CHANNELS[..13].iter().map(|c| c.channel).collect();
    assert_eq!(g2, (1..=13).collect::<Vec<u8>>());
    assert!(SCAN_CHANNELS[..13].iter().all(|c| c.band == HwBand::G2));
    let g5: Vec<u8> = SCAN_CHANNELS[13..].iter().map(|c| c.channel).collect();
    let want: Vec<u8> = (36..=64)
        .step_by(4)
        .chain((100..=144).step_by(4))
        .chain((149..=165).step_by(4))
        .collect();
    assert_eq!(g5, want);
    assert!(SCAN_CHANNELS[13..].iter().all(|c| c.band == HwBand::G5));
}

#[test]
fn add_ch_refused_fails_scan_start() {
    let s = scan(|fw| fw.refuse(9, 0x16, 1));
    assert_eq!(
        s.got,
        Err(ScanError {
            step: "scan start",
            error: AxError::H2cRejected {
                class: 9,
                func: 0x16,
                seq: 18,
                ret: 1
            }
        })
    );
    // SCANOFLD was never sent.
    assert_eq!(kicks(&s.io.trace).len(), 16);
}

#[test]
fn add_ch_return_above_the_mask_is_accepted() {
    // `RTW89_C2H_SCAN_DONE_ACK_RETURN` keeps bits 5:0 only.
    let s = scan(|fw| fw.refuse(9, 0x16, 0x40));
    assert_eq!(s.report().end, ScanEnd::Ok);
}

#[test]
fn silent_scanofld_fails_after_100ms() {
    let s = scan(|fw| fw.silent(9, 0x17));
    let Err(ScanError {
        step: "scan start",
        error:
            AxError::NoAck {
                class: 9,
                func: 0x17,
                seq: 19,
                ..
            },
    }) = s.got
    else {
        panic!("{:?}", s.got);
    };
    let t = &s.io.trace;
    let k = kicks(t);
    assert_eq!(k.len(), 17);
    assert!(time_at(t, t.len()) - time_at(t, k[16]) > 100_000);
}

#[test]
fn old_firmware_is_refused_before_scan_writes() {
    let s = scan_at((0, 27, 35, 0), |_, _| {});
    assert_eq!(
        s.got,
        Err(ScanError {
            step: "scan start",
            error: AxError::Scan("firmware predates SCAN_OFFLOAD (0.27.36.0)")
        })
    );
    let t = &s.io.trace;
    assert!(!t.iter().any(|o| matches!(o, Op::W32(0xCE20, _))));
    assert!(!t.iter().any(|o| matches!(o, Op::W32(0x1184, _))));
    // After vif init's last kick (seq 16), only its RXQ drain writes.
    let k = kicks(t);
    assert_eq!(k.len(), 14);
    let writes: Vec<&Op> = t[k[13] + 1..]
        .iter()
        .filter(|o| matches!(o, Op::W8(..) | Op::W16(..) | Op::W32(..)))
        .collect();
    assert!(
        writes.iter().all(|o| matches!(o, Op::W16(0x1218, _))),
        "{writes:x?}"
    );
    // The first firmware with SCAN_OFFLOAD scans.
    assert_eq!(scan_at((0, 27, 36, 0), |_, _| {}).report().end, ScanEnd::Ok);
    assert_eq!(scan_at((1, 0, 0, 0), |_, _| {}).report().end, ScanEnd::Ok);
}

#[test]
fn missing_end_scan_stops_and_still_reports() {
    let s = scan(|fw| {
        fw.no_end();
        fw.beacon(CH6, HOME, b"Home", 0x70);
    });
    let r = s.report();
    assert_eq!(r.end, ScanEnd::Stopped { ended: false });
    assert!(r.elapsed_ms >= 8_980, "{}", r.elapsed_ms);
    assert_eq!((r.entered, r.networks_total), (38, 1));
    assert_eq!(r.networks[0].map(|n| n.signal), Some(Some(-54)));
    assert!(
        r.to_string().contains(" ms, no end, stopped: 1 networks"),
        "{r}"
    );
    // The stop is seq 20, sent once the budget after SCANOFLD's kick ran
    // out: SCANOFLD with operation 0 (spec S9.3).
    assert_eq!(s.h2c, H2c::at(21));
    let t = &s.io.trace;
    let k = kicks(t);
    assert_eq!(k.len(), 18);
    assert!(time_at(t, k[17]) - time_at(t, k[16]) >= SCAN_WAIT_US);
    let mut stop = vec![0x24, 0x00, 0x00, 0x0D];
    stop.extend_from_slice(&[0; 12]);
    stop.extend_from_slice(&[0x25, 0x17, 0x00, 0x14, 0x24, 0xC0, 0x00, 0x00]);
    stop.extend_from_slice(&[0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00]);
    stop.extend_from_slice(&[0; 20]);
    assert_eq!(slot(&s.m, 20, 52), stop);
    // No END_SCAN came: the stop waited its whole 100 ms.
    assert!(time_at(t, t.len()) - time_at(t, k[17]) >= STOP_WAIT_US);
}

#[test]
fn end_scan_after_stop_is_stopped_and_ended() {
    let s = scan(|fw| {
        fw.no_end();
        fw.end_on_stop();
        fw.end_status(2);
    });
    let r = s.report();
    assert_eq!(r.end, ScanEnd::Stopped { ended: true });
    assert!(
        r.to_string()
            .contains(" ms, no end, stopped and ended: 0 networks"),
        "{r}"
    );
    // END_SCAN ended the stop's wait early.
    let t = &s.io.trace;
    let k = kicks(t);
    assert!(time_at(t, t.len()) - time_at(t, k[17]) < STOP_WAIT_US);
}

#[test]
fn end_scan_status_is_reported() {
    let r = scan(|fw| fw.end_status(2)).report();
    assert_eq!(r.end, ScanEnd::Status(2));
    assert!(
        r.to_string().contains(" ms, end status 2: 0 networks"),
        "{r}"
    );
}

#[test]
fn beacon_seen_on_two_channels_is_one_network() {
    let s = scan(|fw| {
        // Heard on channel 5, advertising channel 6: cfg80211 files it under
        // 6 without the off-channel RSSI.
        fw.buffers(
            CH5,
            vec![
                mgnt_frame(0x80, BROADCAST, HOME, b"Home", Some(6), 6),
                ppdu_status(6, 0x60, 0),
            ],
        );
        fw.beacon(CH6, HOME, b"Home", 0x70);
    });
    let r = s.report();
    assert_eq!((r.networks_total, r.frames, r.dropped), (1, 2, 0));
    let mut want = network(HOME, HwBand::G2, 6, Some(-54), b"Home");
    want.frames = 2;
    assert_eq!(r.networks[0], Some(want));
}

#[test]
fn signal_is_replaced_only_by_a_measured_one() {
    let s = scan(|fw| {
        fw.beacon(CH6, HOME, b"Home", 0x70);
        // A beacon whose PPDU status never comes, on channel 6.
        fw.buffers(
            CH6,
            vec![mgnt_frame(0x80, BROADCAST, HOME, b"Home", Some(6), 5)],
        );
        // Heard on channel 7: no RSSI for channel 6.
        fw.buffers(
            CH7,
            vec![
                mgnt_frame(0x80, BROADCAST, HOME, b"Home", Some(6), 3),
                ppdu_status(3, 0x40, 0),
            ],
        );
    });
    let r = s.report();
    assert_eq!((r.frames, r.without_rssi), (3, 1));
    let mut want = network(HOME, HwBand::G2, 6, Some(-54), b"Home");
    want.frames = 3;
    assert_eq!(r.networks[0], Some(want));
}

#[test]
fn hidden_ssid_takes_the_probe_response_name() {
    let s = scan(|fw| {
        fw.probe_response(CH6, HOME, b"Lab", 0x70);
        fw.beacon(CH6, HOME, b"", 0x70);
        fw.beacon(CH7, OFFICE, b"", 0x60);
    });
    let r = s.report();
    assert_eq!(r.networks_total, 2);
    let mut lab = network(HOME, HwBand::G2, 6, Some(-54), b"Lab");
    lab.frames = 2;
    assert_eq!(r.networks[0], Some(lab));
    let mut hidden = network(OFFICE, HwBand::G2, 7, Some(-62), b"");
    hidden.hidden = true;
    assert_eq!(r.networks[1], Some(hidden));
    assert_eq!(
        listed(&r)[1],
        "WIFI NETWORK 2/2 2G ch 7 -62 dBm 02:aa:00:00:00:02 <hidden>"
    );
}

#[test]
fn off_list_channel_is_dropped() {
    let s = scan(|fw| {
        // IE01's channel index 0x0E: 2.4 GHz channel 14, not scanned.
        fw.buffers(
            CH13,
            vec![
                mgnt_frame(0x80, BROADCAST, HOME, b"Far", None, 4),
                ppdu_status(4, 0x70, 0x0E),
            ],
        );
    });
    let r = s.report();
    assert_eq!((r.frames, r.dropped, r.networks_total), (0, 1, 0));
    assert!(
        r.to_string().ends_with("(0 without RSSI), dropped 1"),
        "{r}"
    );
}

#[test]
fn short_frames_count_as_dropped_from_the_scan_on() {
    // A beacon payload shorter than a header and an FCS.
    let short = |cnt| rx_buffer(0, true, 0, cnt, &[0x80; 20]);
    let s = scan(|fw| {
        // Before the scan: not counted.
        fw.after(5, 0x0A, vec![short(1)]);
        fw.buffers(CH6, vec![short(2)]);
    });
    let r = s.report();
    assert_eq!((r.frames, r.dropped), (0, 1));
}

#[test]
fn network_line_escapes_the_ssid() {
    let home = network(TEST_MAC, HwBand::G2, 6, Some(-54), b"Home");
    let line = |n: &Network| {
        Listed {
            index: 1,
            total: 2,
            network: n,
        }
        .to_string()
    };
    assert_eq!(
        line(&home),
        "WIFI NETWORK 1/2 2G ch 6 -54 dBm 02:11:22:33:44:55 \"Home\""
    );
    let odd = network(TEST_MAC, HwBand::G5, 149, Some(-48), b"a\"b\x01\\~ \x7F");
    assert_eq!(
        line(&odd),
        "WIFI NETWORK 1/2 5G ch 149 -48 dBm 02:11:22:33:44:55 \"a\\x22b\\x01\\x5c~ \\x7f\""
    );
    let mut hidden = home;
    hidden.hidden = true;
    assert!(line(&hidden).ends_with("02:11:22:33:44:55 <hidden>"));
    let mut quiet = home;
    quiet.signal = None;
    assert_eq!(
        line(&quiet),
        "WIFI NETWORK 1/2 2G ch 6 no RSSI 02:11:22:33:44:55 \"Home\""
    );
}

#[test]
fn report_keeps_the_32_strongest() {
    // Two APs per RSSI step, the larger BSSID first in the script.
    let bssid = |i: usize| [0x02, 0xBB, 0, 0, 0, 0x40 - i as u8];
    let raw = |i: usize| 0x40 + 4 * (i / 2) as u8;
    let s = scan(|fw| {
        for i in 0..40 {
            fw.beacon(CH36, bssid(i), b"ap", raw(i));
        }
    });
    let r = s.report();
    assert_eq!((r.networks_total, r.frames), (40, 40));
    let mut want: Vec<([u8; 6], i8)> = (0..40)
        .map(|i| (bssid(i), ((raw(i) >> 1) as i8) - 110))
        .collect();
    want.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    let got: Vec<([u8; 6], i8)> = r
        .listed()
        .map(|l| (l.network.bssid, l.network.signal.unwrap()))
        .collect();
    assert_eq!(got, want[..32]);
    assert_eq!(
        r.listed().last().map(|l| (l.index, l.total)),
        Some((32, 40))
    );
}

#[test]
fn a_network_without_rssi_is_listed_last() {
    let s = scan(|fw| {
        fw.beacon(CH36, OFFICE, b"Office", 0x20);
        fw.buffers(
            CH6,
            vec![mgnt_frame(0x80, BROADCAST, HOME, b"Home", Some(6), 5)],
        );
    });
    let r = s.report();
    assert_eq!((r.frames, r.without_rssi), (2, 1));
    assert_eq!(r.networks[0].map(|n| n.bssid), Some(OFFICE));
    assert_eq!(
        r.networks[1].map(|n| (n.bssid, n.signal)),
        Some((HOME, None))
    );
    assert!(listed(&r)[1].contains(" ch 6 no RSSI "));
}

#[test]
fn refused_vif_h2c_is_named_in_the_line() {
    let s = scan(|fw| fw.refuse(8, 4, 2));
    let r = s.report();
    assert_eq!(r.end, ScanEnd::Ok);
    assert_eq!(
        r.refused,
        Some(Refused {
            class: 8,
            func: 4,
            seq: 13,
            ret: 2
        })
    );
    assert!(
        r.to_string().ends_with(", H2C 8/4 seq 13 returned 2"),
        "{r}"
    );
}

#[test]
fn rxq_full_is_reported() {
    let r = scan(|fw| fw.burst(255)).report();
    assert_eq!((r.rxq_full, r.dropped, r.frames), (1, 0, 0));
    assert!(
        r.to_string().ends_with("(0 without RSSI), RXQ full 1"),
        "{r}"
    );
}

#[test]
fn every_suffix_prints_in_order() {
    let mut r = scan(|_| {}).report();
    r.dropped = 3;
    r.rxq_full = 2;
    r.tail_capped = true;
    r.refused = Some(Refused {
        class: 6,
        func: 0,
        seq: 15,
        ret: 9,
    });
    assert!(
        r.to_string().ends_with(
            "(0 without RSSI), dropped 3, RXQ full 2, RXQ not drained after 64 polls, H2C \
             6/0 seq 15 returned 9"
        ),
        "{r}"
    );
}

#[test]
fn steps_fail_with_their_names() {
    // CH12 never reclaims the firmware log H2C: no firmware answers.
    let (_m, mut rings, mut io) = chip();
    let (got, _) = scan_on(&mut rings, &mut io, FIRMWARE);
    assert_eq!(
        got,
        Err(ScanError {
            step: "core start",
            error: AxError::Timeout {
                register: 0x1080,
                mask: 0x0FFF_0000
            }
        })
    );
    // SWSI stays busy once core start is done.
    let s = scan_at(FIRMWARE, |_, io| io.swsi_status = 0x0300_0000);
    assert_eq!(s.got.map_err(|e| e.step), Err("set channel"));
    // CMAC turns off once the channel is set: vif init's port update is
    // the first to read 0xC400.
    let s = scan_at(FIRMWARE, |_, io| {
        io.on_read(
            0xC400,
            alloc::boxed::Box::new(|r: &mut RegisterModel| {
                r.set32(0xC000, 0);
                r.get32(0xC400)
            }),
        );
    });
    assert_eq!(
        s.got,
        Err(ScanError {
            step: "vif init",
            error: AxError::Mac("CMAC function is not enabled")
        })
    );
    // A malformed RXQ buffer during the scan.
    let s = scan(|fw| {
        let mut bad = rx_buffer(0, false, 0, 0, &[0x80; 30]);
        bad[..4].copy_from_slice(&0xC004u32.to_le_bytes());
        fw.buffers(CH6, vec![bad]);
    });
    assert_eq!(s.got.map_err(|e| e.step), Err("scan"));
}

#[test]
fn frames_queued_before_the_scan_are_flushed_away() {
    // A beacon that arrives after the default CMAC table H2C (vif init's
    // last, seq 16) is drained during the scan start's address CAM and
    // waits for a PPDU status that never comes. It is no scan result.
    let s = scan(|fw| {
        fw.after(
            5,
            0x0A,
            vec![mgnt_frame(0x80, BROADCAST, HOME, b"Old", Some(1), 7)],
        );
    });
    let r = s.report();
    assert_eq!((r.frames, r.dropped, r.networks_total), (0, 0, 0));
}

#[test]
fn frames_before_scanofld_is_acked_are_kept() {
    // A beacon with its PPDU status right after ADD_SCANOFLD_CH's acks: it
    // is drained during SCANOFLD's wait, before `Scanner::submitted`, on the
    // boot channel.
    let s = scan(|fw| {
        let [frame, status] = frame_with_status(0x80, BROADCAST, HOME, b"Early", Some(1), 6, 0x70);
        fw.after(9, 0x16, vec![frame, status]);
    });
    let r = s.report();
    assert_eq!((r.frames, r.dropped, r.networks_total), (1, 0, 1));
    assert_eq!(
        r.networks[0],
        Some(network(HOME, HwBand::G2, 1, Some(-54), b"Early"))
    );
}

#[test]
fn frames_past_the_early_bound_are_dropped() {
    // 66 beacons, each with its PPDU status, right after ADD_SCANOFLD_CH's
    // acks: drained during SCANOFLD's wait, before `Scanner::submitted`,
    // the first MAX_EARLY wait for it and the two past are dropped.
    let s = scan(|fw| {
        let mut batch = Vec::new();
        for n in 0..66u8 {
            let bssid = [0x02, 0xBB, 0, 0, 0, n];
            batch.extend(frame_with_status(
                0x80,
                BROADCAST,
                bssid,
                b"Early",
                Some(1),
                n % 8,
                0x70,
            ));
        }
        fw.after(9, 0x16, batch);
    });
    let r = s.report();
    assert_eq!(MAX_EARLY, 64);
    assert_eq!((r.frames, r.dropped, r.networks_total), (64, 2, 64));
}

#[test]
fn a_bssid_past_the_table_bound_is_dropped() {
    // 513 BSSIDs over the listed channels: the table keeps the first 512
    // heard, and the 513th's beacon is dropped.
    let s = scan(|fw| {
        for n in 0..513u16 {
            let [hi, lo] = n.to_be_bytes();
            fw.beacon(
                usize::from(n) % 38,
                [0x02, 0xBB, 0, 0, hi, lo],
                b"Net",
                0x70,
            );
        }
    });
    let r = s.report();
    assert_eq!(MAX_NETWORKS, 512);
    assert_eq!((r.frames, r.dropped, r.networks_total), (512, 1, 512));
}

#[test]
fn frames_after_end_scan_are_drained_then_flushed() {
    // After END_SCAN: a beacon, its PPDU status in the next batch, then a
    // beacon whose status never comes. Both are on channel 165, the last
    // ENTER_CH.
    let s = scan(|fw| {
        fw.after_end(vec![mgnt_frame(0x80, BROADCAST, HOME, b"Late", None, 6)]);
        fw.after_end(vec![ppdu_status(6, 0x70, 0)]);
        fw.after_end(vec![mgnt_frame(0x80, BROADCAST, OFFICE, b"Last", None, 5)]);
    });
    let r = s.report();
    assert_eq!(r.end, ScanEnd::Ok);
    assert_eq!((r.frames, r.without_rssi, r.networks_total), (2, 1, 2));
    assert_eq!(
        r.networks[0],
        Some(network(HOME, HwBand::G5, 165, Some(-54), b"Late"))
    );
    assert_eq!(
        r.networks[1],
        Some(network(OFFICE, HwBand::G5, 165, None, b"Last"))
    );
}

#[test]
fn frames_after_a_stop_are_drained_then_flushed() {
    let s = scan(|fw| {
        fw.no_end();
        fw.end_on_stop();
        fw.after_end(vec![mgnt_frame(0x80, BROADCAST, HOME, b"Late", None, 6)]);
        fw.after_end(vec![ppdu_status(6, 0x70, 0)]);
    });
    let r = s.report();
    assert_eq!(r.end, ScanEnd::Stopped { ended: true });
    assert_eq!(
        r.networks[0],
        Some(network(HOME, HwBand::G5, 165, Some(-54), b"Late"))
    );
}

#[test]
fn six_ghz_and_band_3_reports_are_handled() {
    let s = scan(|fw| {
        fw.beacon(CH165, OFFICE, b"Office", 0x50);
        fw.buffers(
            CH165,
            vec![
                // Band 2: a 6 GHz hop and a frame tagged 6 GHz channel 37.
                scanofld(ENTER_CH, 0, 2, 37, 105),
                mgnt_frame(0x80, BROADCAST, [0x02, 0xCC, 0, 0, 0, 1], b"six", None, 2),
                ppdu_status(2, 0x70, 0),
                // Band 3 on a channel 2.4 GHz does not have.
                scanofld(ENTER_CH, 0, 3, 200, 105),
                mgnt_frame(0x80, BROADCAST, [0x02, 0xCC, 0, 0, 0, 2], b"three", None, 3),
                ppdu_status(3, 0x70, 0),
                // Band 3 on 2.4 GHz channel 6: read as 2.4 GHz, as Linux's
                // band default does.
                scanofld(ENTER_CH, 0, 3, 6, 105),
                mgnt_frame(0x80, BROADCAST, HOME, b"Home", Some(6), 4),
                ppdu_status(4, 0x70, 0),
            ],
        );
    });
    let r = s.report();
    assert_eq!((r.end, r.entered), (ScanEnd::Ok, 41));
    assert_eq!((r.frames, r.dropped, r.networks_total), (2, 2, 2));
    assert_eq!(
        r.networks[0],
        Some(network(HOME, HwBand::G2, 6, Some(-54), b"Home"))
    );
    assert_eq!(
        r.networks[1],
        Some(network(OFFICE, HwBand::G5, 165, Some(-70), b"Office"))
    );
}

/// A LEAVE_CH report on 2.4 GHz channel 1: a C2H the scan only counts.
fn leave_ch() -> Vec<u8> {
    scanofld(4, 0, 0, 1, 105)
}

#[test]
fn the_tail_drain_stops_at_its_cap_and_says_so() {
    // 63 batches after END_SCAN: the 64th tail poll finds RXQ empty.
    let r = scan(|fw| {
        for _ in 0..MAX_TAIL_POLLS - 1 {
            fw.after_end(vec![leave_ch()]);
        }
    })
    .report();
    assert!(!r.tail_capped);
    assert!(!r.to_string().contains("RXQ not drained"), "{r}");
    // 64: every tail poll consumed a batch, so RXQ was never seen empty, and
    // the 65th batch is left in the ring.
    let s = scan(|fw| {
        for _ in 0..=MAX_TAIL_POLLS {
            fw.after_end(vec![leave_ch()]);
        }
        fw.after_end(vec![mgnt_frame(0x80, BROADCAST, HOME, b"Late", None, 6)]);
    });
    let r = s.report();
    assert!(r.tail_capped);
    assert_eq!((r.end, r.networks_total), (ScanEnd::Ok, 0));
    assert!(
        r.to_string()
            .ends_with("(0 without RSSI), RXQ not drained after 64 polls"),
        "{r}"
    );
}

#[test]
fn the_submission_is_validated_before_scanofld() {
    // The clock jumps past the scan budget while ADD_SCANOFLD_CH (the 16th
    // kick, seq 18) waits for its DONE_ACK: the scanner's recheck before
    // SCANOFLD cancels the request, and SCANOFLD is never sent.
    let (m, mut rings, mut io) = chip();
    let fw = Firmware::new(&m).answering();
    let ch12 = fw.clone();
    io.on_read(
        0x1080,
        alloc::boxed::Box::new(move |r: &mut RegisterModel| ch12.ch12(r)),
    );
    let mut jumped = false;
    io.on_read(
        0x1218,
        alloc::boxed::Box::new(move |r: &mut RegisterModel| {
            if !jumped && kicks(&r.trace).len() == 16 {
                r.now += SCAN_WAIT_US;
                jumped = true;
            }
            fw.rxq(r)
        }),
    );
    let (got, _) = scan_on(&mut rings, &mut io, FIRMWARE);
    assert_eq!(
        got,
        Err(ScanError {
            step: "scan start",
            error: AxError::Scan("scan timed out")
        })
    );
    assert_eq!(kicks(&io.trace).len(), 16);
}

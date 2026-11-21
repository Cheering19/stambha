extern crate std;
use super::*;
use crate::wifi::realtek::ax::assoc::{AddrCam, AssocInputs, RaPeer};
use crate::wifi::realtek::ax::bringup::tests::{
    BOTH, Chip, DEINIT, GNT_WL_OFF, HOME, MAC, NO_LTR, REPORT, RF18, RFK,
    assert_dma_stopped_before_power_off, assert_ends_with_power_off,
    assert_radio_off_before_deinit, chip, chip_hearing, cmac_off_after, count, deinit_writes,
    efuse_with_country, efuse_with_mac, first_of, never, same_access, tables_file,
    tables_file_without, txpwr_alone, txpwr_window,
};
use crate::wifi::realtek::ax::chan::{HwBand, Subband};
use crate::wifi::realtek::ax::command::H2cAck;
use crate::wifi::realtek::ax::fwdl::tests::image_bytes;
use crate::wifi::realtek::ax::h2c::ScanChannel;
use crate::wifi::realtek::ax::pci::tests::{FWCMD, device_read};
use crate::wifi::realtek::ax::pci::{FWCMD_SLOT_BYTES, TX_ENTRIES_CTRL};
use crate::wifi::realtek::ax::receive::{Event, Mode, Received, Tag};
use crate::wifi::realtek::ax::regd::Regd;
use crate::wifi::realtek::ax::scan::{STOP_WAIT_US, ScanEnd, run_once, scan_budget_us, scan_list};
use crate::wifi::realtek::ax::tests::firmware::{
    Firmware, frame_with_status, mgnt_frame, ppdu_status,
};
use crate::wifi::realtek::ax::tests::model::{Op, READ_FAILED, RegisterModel};
use crate::wifi::realtek::ax::txpwr_tables::ID_TXPWR_LMT_5G;
use crate::wifi::realtek::beacon::tests::{Domain, Model, Source};
use crate::wifi::realtek::firmware::{Ce, Rtw89};
use crate::wifi::station::phy::AccessCategory;
use alloc::{boxed::Box, sync::Arc, vec, vec::Vec};
use std::sync::Mutex;

mod ba;
mod beacon;
mod chip;
mod keys;

type Started = Result<Live<RegisterModel>, StartError<RegisterModel>>;

/// [`start`] on `c`'s chip, cut 2, without LTR, with the elements of `file`
/// and the network country `country`.
fn start_on(c: Chip, file: &[u8], country: Option<[u8; 2]>) -> (Started, Arc<Mutex<Model>>) {
    let Chip { io, m, .. } = c;
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    let firmware = FirmwareFile {
        image: &fw.image,
        file,
    };
    // SAFETY: the model domain isolates only the modeled requester behind
    // this test's `RegisterModel`; its pages are plain kept test RAM that no
    // real device can reach, and this test is the only worker.
    let started = unsafe {
        start(
            io,
            &Source,
            Arc::new(Domain(m.clone())),
            2,
            firmware,
            NO_LTR,
            country,
        )
    };
    (started, m)
}

/// A live chip from [`chip`] and [`tables_file`], with no country.
fn live() -> (Live<RegisterModel>, Arc<Mutex<Model>>) {
    let (started, m) = start_on(chip(efuse_with_mac(), 7, never, BOTH), &tables_file(), None);
    (started.unwrap_or_else(|e| panic!("{}", e.error)), m)
}

/// The failure of a start that must fail.
fn refused(started: Started) -> StartError<RegisterModel> {
    match started {
        Ok(live) => panic!("started: {}", live.running()),
        Err(failed) => failed,
    }
}

fn g2(channel: u8) -> Chan {
    Chan::new20(HwBand::G2, channel).unwrap()
}

fn g5(channel: u8) -> Chan {
    Chan::new20(HwBand::G5, channel).unwrap()
}

/// The world regd with no country anywhere: `00`.
const WW: (Regds, [u8; 2]) = (
    Regds {
        g2: Regd::Ww,
        g5: Regd::Ww,
    },
    *b"00",
);

/// The first trace entry after `from` that `want` accepts.
fn after(t: &[Op], from: usize, want: impl Fn(&Op) -> bool) -> usize {
    from + 1
        + t[from + 1..]
            .iter()
            .position(want)
            .unwrap_or_else(|| panic!("nothing after trace entry {from}"))
}

#[test]
fn start_runs_core_channel_txpwr_vif_in_order() {
    let (live, _m) = live();
    let t = &live.io().trace;
    // dm tail: cfg_txrx_path's first write, M2b2's last step.
    let dm_tail = first_of(t, |o| *o == Op::W32(0x14978, 3));
    // core start: cfg_ppdu_status_bands(true).
    let core = after(t, dm_tail, |o| *o == Op::W32(0xCE40, 0x2B));
    // set channel: help_enter's TX scheduler stop.
    let channel = after(t, core, |o| matches!(o, Op::W32(0xC398, _)));
    // tx power: by rate's first word, then path B's SAR ceiling last.
    let txpwr = after(t, channel, |o| matches!(o, Op::W32(0xD2C0, _)));
    let sar = after(t, txpwr, |o| *o == Op::W32(0x176CC, 0x7E));
    // vif init: port 0's beacon interval.
    let vif = after(t, sar, |o| *o == Op::W32(0xC414, 0x64));
    let order = [dm_tail, core, channel, txpwr, sar, vif];
    assert!(order.windows(2).all(|w| w[0] < w[1]), "{order:?}");
    // Ruling R1 (core.c:584-588): set_txpwr's last access comes right
    // before help_exit's first two, its CMAC check and the PPDU status.
    assert_eq!(
        t[sar + 1..sar + 3],
        [Op::R32(0xC000), Op::W32(0xCE40, 0x2B)]
    );
    // No teardown: no WL-off grant, no deinit, no power-off.
    assert!(!t.contains(&Op::W32(0xDA48, GNT_WL_OFF)));
    assert!(!t[dm_tail..].iter().any(|o| matches!(o, Op::W32(0x0074, _))));
    assert_eq!(count(live.io(), Op::W8(0x00AF, 0x80)), 0);
    // What M2b2 reported: no scan yet.
    assert_eq!(
        *live.running(),
        Running {
            cut: 2,
            firmware: (0, 27, 66, 1),
            mac: MAC,
            secure_boot: false,
            mac_init: true,
            ltr: false,
            bacam_reset_timed_out: false,
            h2c: H2cAck {
                rec: true,
                done: true,
                other_c2h: 0,
                unmatched_ack: 0,
                skipped: 0,
                log: None,
            },
            phy: Some(REPORT),
            rfk: Some(RFK),
            rf18: Some(RF18),
            scan: None,
        }
    );
    assert_eq!(live.mac(), MAC);
    assert_eq!(live.regds(), WW);
    assert_eq!(live.chan(), g2(1));
    // Spec S3.1: the BA CAM took seq 4-11 and vif init 12-16.
    assert_eq!(live.h2c(), H2c::at(17));
}

#[test]
fn start_leaves_a_station_receiver_on_its_channel_and_fresh_tx_pools() {
    let (live, m) = live();
    assert_eq!(live.receiver().mode(), Mode::Station);
    assert_eq!(
        live.receiver().tag(),
        Tag {
            band: HwBand::G2,
            channel: 1
        }
    );
    // "tx rings": the pools' own allocation of 2 263 040 bytes in 553 pages,
    // the last this start mapped, after the rings'.
    let sizes: Vec<usize> = {
        let model = m.lock().unwrap();
        model.mappings.values().map(|(p, _)| p.bytes()).collect()
    };
    assert_eq!(sizes.iter().filter(|&&b| b == 553 * 4096).count(), 1);
    assert_eq!(sizes.last(), Some(&(553 * 4096)), "{sizes:?}");
}

#[test]
fn start_takes_the_efuse_country_then_the_networks() {
    let fcc = Regds {
        g2: Regd::Fcc,
        g5: Regd::Fcc,
    };
    let etsi = Regds {
        g2: Regd::Etsi,
        g5: Regd::Etsi,
    };
    for (efuse, network, want) in [
        (efuse_with_mac(), None, WW),
        (efuse_with_mac(), Some(*b"US"), (fcc, *b"US")),
        // The efuse country wins when the firmware's map has it.
        (efuse_with_country(*b"DE"), Some(*b"US"), (etsi, *b"DE")),
        // One the map lacks falls to the network's.
        (efuse_with_country(*b"JP"), Some(*b"US"), (fcc, *b"US")),
        (efuse_with_country(*b"JP"), None, WW),
    ] {
        let (started, _m) = start_on(chip(efuse, 7, never, BOTH), &tables_file(), network);
        let live = started.unwrap_or_else(|e| panic!("{}", e.error));
        assert_eq!(live.regds(), want, "{network:?}");
    }
}

#[test]
fn start_failure_powers_off_and_hands_back_io() {
    // SWSI stays busy from help_enter's TX scheduler read on, so the RF
    // part of set channel times out.
    let mut c = chip(efuse_with_mac(), 7, never, BOTH);
    c.io.on_read(
        0xC398,
        Box::new(|m: &mut RegisterModel| {
            if m.trace.contains(&Op::W32(0x14978, 3)) {
                m.swsi_status = 0x0300_0000;
            }
            m.get32(0xC398)
        }),
    );
    let (started, _m) = start_on(c, &tables_file(), None);
    let StartError { error, io, rings } = refused(started);
    assert_eq!(error.step, "set channel", "{error}");
    assert!(matches!(error.error, AxError::Timeout { .. }), "{error}");
    // RF 0x18 was read before, so the failure keeps it.
    assert_eq!(error.rf18, Some(RF18));
    // The chip's own window comes back with the rings, torn down as
    // rtw89_core_stop does: the coex half, the deinit, the power-off.
    assert!(rings.is_some());
    assert_radio_off_before_deinit(&io);
    assert_eq!(deinit_writes(&io), DEINIT);
    assert_ends_with_power_off(&io);
    // set_txpwr never ran.
    let dm_tail = first_of(&io.trace, |o| *o == Op::W32(0x14978, 3));
    assert!(
        !io.trace[dm_tail..]
            .iter()
            .any(|o| matches!(o, Op::W32(0xD2C0, _)))
    );
}

#[test]
fn missing_tx_power_element_fails_the_firmware_elements() {
    let (started, _m) = start_on(
        chip(efuse_with_mac(), 7, never, BOTH),
        &tables_file_without(Some(ID_TXPWR_LMT_5G)),
        None,
    );
    let StartError { error, io, rings } = refused(started);
    assert_eq!(
        error,
        BringUpError::new(
            "firmware elements",
            AxError::TxPower("firmware lacks TXPWR_LMT_5GHZ")
        )
    );
    // The step and the error each name what they are, once.
    assert_eq!(
        error.to_string(),
        "RTL8852CE bring-up: firmware elements: tx power: firmware lacks TXPWR_LMT_5GHZ"
    );
    // As rtw89_chip_info_setup fails at rtw89_fw_recognize_elements
    // (core.c:7413): after the efuse, which the RFE type comes from, and
    // before the MAC init, so none of the file's PHY tables reached the
    // chip either; the DMA mac pre-init started is stopped, then the
    // power-off.
    assert!(rings.is_some());
    assert!(io.trace.contains(&Op::W32(0x0030, 0)));
    assert!(!io.trace.contains(&Op::W32(0x14AD4, 0x40)));
    // sys init's dmac_func_en_ax word, the MAC init's first after the efuse.
    assert!(!io.trace.contains(&Op::W32(0x8400, 0xFB7D_4000)));
    assert_dma_stopped_before_power_off(&io);
    assert_ends_with_power_off(&io);
}

#[test]
fn stop_runs_deinit_then_power_off() {
    let (live, m) = live();
    let started = live.io().trace.len();
    // SAFETY: as in `start_on`; the live chip is this test's alone.
    let (io, rings, pools) = unsafe { live.stop() };
    // rtw89_core_stop (core.c:6738-6767): the coex half first, then the
    // deinit, then rtw89_mac_pwr_off.
    assert_eq!(io.trace[started], Op::W32(0xDA48, GNT_WL_OFF));
    assert_radio_off_before_deinit(&io);
    assert_eq!(deinit_writes(&io), DEINIT);
    assert_ends_with_power_off(&io);
    // The rings and pools come back still mapped: nothing proves the DMA
    // stopped.
    assert!(m.lock().unwrap().mappings.len() > 1);
    drop((rings, pools));
}

#[test]
fn tune_sets_channel_then_tx_power_then_help_exit_and_retags() {
    let (mut live, _m) = live();
    let tuned = live.io().trace.len();
    // SAFETY: as in `start_on`.
    unsafe { live.tune(&g5(36)) }.unwrap();
    let t = &live.io().trace[tuned..];
    // __rtw89_set_channel (ruling R1): help_enter's TX scheduler stop, the
    // MAC, BB and RF parts, set_txpwr (103 accesses on 5 GHz, which has no
    // CCK filter), then help_exit, whose TX scheduler resume is the last.
    let enter = first_of(t, |o| matches!(o, Op::W32(0xC398, _)));
    let window = txpwr_window(t, enter);
    assert_eq!(window.len(), 103);
    assert!(same_access(&txpwr_alone(g5(36)), &t[window.clone()]));
    assert!(matches!(t[window.start - 1], Op::RfW(1, 0x8F, 0x60, _)));
    assert_eq!(
        t[window.end..window.end + 2],
        [Op::R32(0xC000), Op::W32(0xCE40, 0x2B)]
    );
    assert!(matches!(t.last(), Some(Op::W32(0xC398, _))));
    assert_eq!(live.chan(), g5(36));
    // The receiver tags what follows with the new channel.
    assert_eq!(
        live.receiver().tag(),
        Tag {
            band: HwBand::G5,
            channel: 36
        }
    );
    assert_eq!(live.receiver().mode(), Mode::Station);
}

#[test]
fn tune_refuses_an_unbuilt_channel_before_any_io() {
    let (mut live, _m) = live();
    let tuned = live.io().trace.len();
    let bad = Chan {
        channel: 6,
        band: HwBand::G2,
        freq: 2412,
        subband: Subband::G2,
    };
    // SAFETY: as in `start_on`.
    let refused = unsafe { live.tune(&bad) };
    assert_eq!(
        refused,
        Err(AxError::Phy("not a 20 MHz 2.4 or 5 GHz channel"))
    );
    assert_eq!(live.io().trace.len(), tuned);
    // The chip stays on: the next channel is set.
    // SAFETY: as above.
    unsafe { live.tune(&g2(6)) }.unwrap();
    assert_eq!(live.receiver().tag().channel, 6);
}

/// After a failed tune: the chip was torn down as a failed start is, and
/// nothing after it reaches the registers.
fn assert_powered_off_for_good(mut live: Live<RegisterModel>) {
    assert_radio_off_before_deinit(live.io());
    assert_eq!(deinit_writes(live.io()), DEINIT);
    assert_ends_with_power_off(live.io());
    let off = live.io().trace.len();
    // SAFETY: as in `start_on`.
    assert_eq!(unsafe { live.tune(&g2(1)) }, Err(POWERED_OFF));
    // No command session either: the scan is refused at its start.
    // SAFETY: as above.
    let scanned = unsafe { run_once(&mut live) };
    assert_eq!(
        scanned,
        Err(ScanError {
            step: "scan start",
            error: POWERED_OFF
        })
    );
    assert_eq!(live.io().trace.len(), off);
    // SAFETY: as above.
    let (io, _rings, _pools) = unsafe { live.stop() };
    assert_eq!(io.trace.len(), off, "a second teardown ran");
}

#[test]
fn tune_failure_in_tx_power_powers_off_and_refuses_later_calls() {
    // set_txpwr's SAR ceiling of path A (BB 0x56CC) cannot be read: the
    // switch stops with the TX scheduler stopped and the BB held in reset.
    let (mut live, _m) = live();
    live.io_mut().fail_reads_of(0x156CC);
    // SAFETY: as in `start_on`.
    assert_eq!(unsafe { live.tune(&g5(36)) }, Err(READ_FAILED));
    let t = &live.io().trace;
    let failed = t.iter().rposition(|o| *o == Op::R32(0x156CC)).unwrap();
    // help_exit never ran: no PPDU status report after the failed read.
    let off = after(t, failed, |o| *o == Op::W32(0xDA48, GNT_WL_OFF));
    assert!(!t[failed..off].contains(&Op::W32(0xCE40, 0x2B)));
    assert_powered_off_for_good(live);
}

#[test]
fn tune_failure_in_the_switch_powers_off_too() {
    // help_enter's CMAC check finds the CMAC off once.
    let (mut live, _m) = live();
    cmac_off_after(live.io_mut(), Op::W32(0x14978, 3));
    // SAFETY: as in `start_on`.
    let failed = unsafe { live.tune(&g2(11)) };
    assert_eq!(failed, Err(AxError::Mac("CMAC function is not enabled")));
    assert_powered_off_for_good(live);
}

const OFFICE: [u8; 6] = [0x02, 0xAA, 0, 0, 0, 2];
const BROADCAST: [u8; 6] = [0xFF; 6];

/// A live chip from [`chip_hearing`], whose firmware `script` sets up, and
/// [`tables_file`], with no country.
fn live_hearing(script: &dyn Fn(&mut Firmware)) -> (Live<RegisterModel>, Arc<Mutex<Model>>) {
    let c = chip_hearing(efuse_with_mac(), 7, never, BOTH, script);
    let (started, m) = start_on(c, &tables_file(), None);
    (started.unwrap_or_else(|e| panic!("{}", e.error)), m)
}

/// 2.4 GHz channels 1 and 6, then 5 GHz 36: listed channels 0, 1 and 2.
fn three() -> Vec<ScanChannel> {
    scan_list(&[(HwBand::G2, 1), (HwBand::G2, 6), (HwBand::G5, 36)]).unwrap()
}

/// Every frame [`Live::drain`] hands out, one drain at a time, until the
/// scan has ended.
fn drain_until_ended(live: &mut Live<RegisterModel>) -> Vec<Vec<Received>> {
    let mut drains = Vec::new();
    for _ in 0..100 {
        let mut frames = Vec::new();
        // SAFETY: as in `start_on`.
        unsafe {
            live.drain(&mut |e| {
                if let Event::Frame(f) = e {
                    frames.push(f);
                }
            })
        }
        .unwrap();
        drains.push(frames);
        if live.scan_state().is_some() {
            return drains;
        }
    }
    panic!("the scan never ended");
}

/// The CH12 packet, its descriptor included, whose FWCMD header names cat
/// 1, `class`, `func` and `seq`.
fn h2c_bytes(m: &Arc<Mutex<Model>>, class: u8, func: u8, seq: u8) -> Vec<u8> {
    // The guard goes before any panic, so a missing H2C fails this test
    // alone and leaves the model's mutex unpoisoned.
    let found = {
        let state = m.lock().unwrap();
        (0..TX_ENTRIES_CTRL).find_map(|n| {
            let at = u64::from(FWCMD) + (n * FWCMD_SLOT_BYTES) as u64;
            let head = device_read(&state, at, 24);
            let hdr0 = u32::from_le_bytes(head[16..20].try_into().unwrap());
            let hdr1 = u32::from_le_bytes(head[20..24].try_into().unwrap());
            let named = (
                hdr0 & 0x3,
                (hdr0 >> 2) & 0x3F,
                (hdr0 >> 8) & 0xFF,
                hdr0 >> 24,
            );
            (named == (1, u32::from(class), u32::from(func), u32::from(seq)))
                .then(|| device_read(&state, at, 16 + (hdr1 & 0x3FFF) as usize))
        })
    };
    found.unwrap_or_else(|| panic!("no H2C {class}/{func} seq {seq} in the CH12 slots"))
}

/// The CH12 kicks in a trace.
fn kicks(t: &[Op]) -> usize {
    t.iter().filter(|o| matches!(o, Op::W16(0x1080, _))).count()
}

/// The virtual time the delays of `t` took.
fn delays(t: &[Op]) -> u64 {
    t.iter()
        .map(|o| match o {
            Op::Delay(us) => *us,
            _ => 0,
        })
        .sum()
}

#[test]
fn begin_scan_offloads_the_given_list() {
    let (mut live, m) = live_hearing(&|_| {});
    let list = three();
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&list) }.unwrap();
    // Seq 17 is the scan start's address CAM, 18 ADD_SCANOFLD_CH, 19
    // SCANOFLD start.
    assert_eq!(live.h2c(), H2c::at(20));
    let add = h2c_bytes(&m, 9, 0x16, 18);
    let mut want = vec![0x03, 0x07, 0x00, 0x00];
    for c in &list {
        for w in c.words() {
            want.extend_from_slice(&w.to_le_bytes());
        }
    }
    assert_eq!(want.len(), 4 + 3 * 28);
    assert_eq!(add[24..], want[..]);
    // 5 GHz 36 as M2c's list carries it (M2c spec S9.2).
    assert_eq!(
        add[28 + 2 * 28..28 + 2 * 28 + 8],
        [0x69, 0x14, 0x24, 0x24, 0xF8, 0x70, 0xFF, 0x01]
    );
    // SCANOFLD start: operation 1, notify_end.
    let start = h2c_bytes(&m, 9, 0x17, 19);
    assert_eq!(
        start[24..32],
        [0x00, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00]
    );
    assert!(live.scanning());
    assert_eq!(live.scan_state(), None);
    // The scan's receiver while the scan's RX filter is on.
    assert_eq!(live.io().get32(0xCE20), 0x0317_4438);
    assert_eq!(live.receiver().mode(), Mode::Scan);
}

#[test]
fn seventy_channels_are_refused() {
    let seventy: Vec<(HwBand, u8)> = (0..70).map(|i| (HwBand::G2, 1 + i % 13)).collect();
    assert_eq!(
        scan_list(&seventy),
        Err(AxError::Scan("scan list past 69 channels"))
    );
    let sixty_nine = scan_list(&seventy[..69]).unwrap();
    assert_eq!(sixty_nine.len(), 69);
    let (mut live, _m) = live();
    let mut list = sixty_nine.clone();
    list.push(sixty_nine[0]);
    let before = live.io().trace.len();
    // SAFETY: as in `start_on`.
    let refused = unsafe { live.begin_scan(&list) };
    assert_eq!(refused, Err(AxError::Scan("scan list past 69 channels")));
    // Refused before any access, and nothing changed: the 69 are scanned.
    assert_eq!(live.io().trace.len(), before);
    assert!(!live.scanning());
    // SAFETY: as above.
    unsafe { live.begin_scan(&sixty_nine) }.unwrap();
}

#[test]
fn end_scan_restores_in_linux_order() {
    let (mut live, m) = live_hearing(&|_| {});
    // EDCCA's three levels, and DIG's register all ones so both its writes
    // show.
    live.io_mut().set32(0x14840, 0x1234_5678);
    live.io_mut().set32(0x1481C, 0xFFFF_FFFF);
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    // The scan start (M2c spec S8): EDCCA at its max, the scan's RX filter.
    assert_eq!(live.io().get32(0x14840), 0xF934_F9F9);
    assert_eq!(live.io().get32(0xCE20), 0x0317_4438);
    drain_until_ended(&mut live);
    assert_eq!(live.scan_state(), Some(ScanEnd::Ok));
    // The firmware left the radio on the last listed channel.
    assert_eq!(
        live.receiver().tag(),
        Tag {
            band: HwBand::G5,
            channel: 36
        }
    );
    let ended = live.io().trace.len();
    // SAFETY: as above.
    unsafe { live.end_scan() }.unwrap();
    let t = &live.io().trace[ended..];
    // rtw89_hw_scan_complete (spec S8.3): the switch back to the operating
    // channel, 2.4 GHz channel 1, with its TX power before the help's exit
    // (ruling R1), then the RX filter, the address CAM, EDCCA, TSF sync and
    // DIG.
    let switch = first_of(t, |o| matches!(o, Op::W32(0xC398, _)));
    let window = txpwr_window(t, switch);
    assert!(same_access(&txpwr_alone(g2(1)), &t[window.clone()]));
    let fltr = after(t, window.end, |o| *o == Op::W32(0xCE20, 0x0317_44BE));
    let cam = after(t, fltr, |o| matches!(o, Op::W16(0x1080, _)));
    let edcca = after(t, cam, |o| matches!(o, Op::W32(0x14840, _)));
    let rx_sync = after(
        t,
        edcca,
        |o| matches!(o, Op::W32(0xC400, v) if v & 0x8 != 0),
    );
    let dig = after(t, rx_sync, |o| matches!(o, Op::W32(0x1481C, _)));
    assert_eq!(t[fltr - 1], Op::R32(0xCE20));
    // EDCCA's [7:0], [15:8] and [31:24] back from their backups, in turn.
    let edcca_writes: Vec<Op> = t[edcca..rx_sync]
        .iter()
        .copied()
        .filter(|o| matches!(o, Op::W32(0x14840, _)))
        .collect();
    assert_eq!(
        edcca_writes,
        [
            Op::W32(0x14840, 0xF934_F978),
            Op::W32(0x14840, 0xF934_5678),
            Op::W32(0x14840, 0x1234_5678)
        ]
    );
    assert_eq!(t[rx_sync - 1], Op::R32(0xC400));
    // dig_resume: PD lower bound bak_dig (0), spatial reuse off as no
    // station is associated; nothing after it.
    assert_eq!(
        t[dig - 1..],
        [
            Op::R32(0x1481C),
            Op::W32(0x1481C, 0xDFFF_F83F),
            Op::R32(0x1481C),
            Op::W32(0x1481C, 0xDFFF_F83F)
        ]
    );
    // The address CAM at seq 20, vif init's (seq 15) byte for byte but its
    // seq and the REC_ACK every fourth seq asks for (M2c spec S7.4).
    let restored = h2c_bytes(&m, 6, 0, 20);
    let vif = h2c_bytes(&m, 6, 0, 15);
    assert_eq!(restored[16..20], [0x19, 0x00, 0x00, 20]);
    assert_eq!(restored[20..24], [0x44, 0xC0, 0x00, 0x00]);
    assert_eq!(vif[20..24], [0x44, 0x80, 0x00, 0x00]);
    assert_eq!(restored[24..], vif[24..]);
    assert_eq!(live.h2c(), H2c::at(21));
    // The station receiver again, tagging with the operating channel.
    assert_eq!(live.receiver().mode(), Mode::Station);
    assert_eq!(
        live.receiver().tag(),
        Tag {
            band: HwBand::G2,
            channel: 1
        }
    );
    assert_eq!(live.chan(), g2(1));
    assert!(!live.scanning());
    assert_eq!(live.scan_state(), None);
}

#[test]
fn end_scan_after_run_once_writes_back_the_edcca_run_once_read() {
    let (mut live, _m) = live_hearing(&|_| {});
    live.io_mut().set32(0x14840, 0x1234_5678);
    // SAFETY: as in `start_on`.
    let report = unsafe { run_once(&mut live) }.unwrap();
    assert_eq!(live.scan_state(), Some(report.end));
    assert_eq!(live.io().get32(0x14840), 0xF934_F9F9);
    // SAFETY: as above.
    unsafe { live.end_scan() }.unwrap();
    assert_eq!(live.io().get32(0x14840), 0x1234_5678);
}

#[test]
fn end_scan_returns_to_the_last_tuned_channel() {
    let (mut live, _m) = live_hearing(&|_| {});
    // SAFETY: as in `start_on`.
    unsafe { live.tune(&g5(149)) }.unwrap();
    // SAFETY: as above.
    unsafe { live.begin_scan(&three()) }.unwrap();
    drain_until_ended(&mut live);
    let ended = live.io().trace.len();
    // SAFETY: as above.
    unsafe { live.end_scan() }.unwrap();
    // The operating channel, with its 5 GHz TX power (103 accesses).
    let t = &live.io().trace[ended..];
    let window = txpwr_window(t, 0);
    assert!(same_access(&txpwr_alone(g5(149)), &t[window]));
    assert_eq!(live.chan(), g5(149));
    assert_eq!(
        live.receiver().tag(),
        Tag {
            band: HwBand::G5,
            channel: 149
        }
    );
}

#[test]
fn budget_expiry_stops_and_ends() {
    for end_on_stop in [true, false] {
        // A beacon after the stop's END_SCAN, whose PPDU status never comes.
        let (mut live, m) = live_hearing(&|fw| {
            fw.no_end();
            if end_on_stop {
                fw.end_on_stop();
            }
            fw.after_end(vec![mgnt_frame(0x80, BROADCAST, HOME, b"Late", None, 6)]);
        });
        // SAFETY: as in `start_on`.
        unsafe { live.begin_scan(&three()) }.unwrap();
        // SCANOFLD start's DONE_ACK came now.
        let begun = live.io().now;
        // Every hop arrives, END_SCAN never does.
        for _ in 0..20 {
            // SAFETY: as above.
            unsafe { live.drain(&mut |_| {}) }.unwrap();
        }
        assert_eq!(live.scan_state(), None);
        // 1 us short of 2 * 3 * 105 ms + 1 s: nothing is stopped.
        live.io_mut().now = begun + scan_budget_us(3) - 1;
        let k = kicks(&live.io().trace);
        // SAFETY: as above.
        unsafe { live.expire_scan(&mut |_| {}) }.unwrap();
        assert_eq!(kicks(&live.io().trace), k);
        assert_eq!(live.scan_state(), None);
        // At the budget: SCANOFLD stop at seq 20 (operation 0), then up to
        // 100 ms for END_SCAN.
        live.io_mut().now = begun + scan_budget_us(3);
        let stopped = live.io().trace.len();
        let mut late = Vec::new();
        // SAFETY: as above.
        unsafe {
            live.expire_scan(&mut |e| {
                if let Event::Frame(f) = e {
                    late.push((f.signal, f.tag));
                }
            })
        }
        .unwrap();
        // What followed END_SCAN was drained and flushed before the scan
        // ended: the beacon, without a signal, on 5 GHz 36.
        let ch36 = Tag {
            band: HwBand::G5,
            channel: 36,
        };
        let want = if end_on_stop {
            vec![(None, ch36)]
        } else {
            Vec::new()
        };
        assert_eq!(late, want);
        let stop = h2c_bytes(&m, 9, 0x17, 20);
        assert_eq!(stop[24..28], [0x00, 0x00, 0x00, 0x00]);
        assert_eq!(kicks(&live.io().trace[stopped..]), 1);
        assert_eq!(
            live.scan_state(),
            Some(ScanEnd::Stopped { ended: end_on_stop })
        );
        let waited = delays(&live.io().trace[stopped..]);
        if end_on_stop {
            assert!(waited < STOP_WAIT_US, "{waited}");
        } else {
            assert!(waited >= STOP_WAIT_US, "{waited}");
        }
        // Once ended, the budget stops nothing more.
        let k = kicks(&live.io().trace);
        live.io_mut().now += 10 * scan_budget_us(3);
        // SAFETY: as above.
        unsafe { live.expire_scan(&mut |_| {}) }.unwrap();
        assert_eq!(kicks(&live.io().trace), k);
        // SAFETY: as above.
        unsafe { live.end_scan() }.unwrap();
        assert_eq!(live.h2c(), H2c::at(22));
        assert_eq!(live.receiver().mode(), Mode::Station);
    }
}

#[test]
fn a_station_scan_keeps_beacons_and_probe_responses_until_end_scan() {
    // Spec S1.3 and M2c S1.10: the scan's RX filter passes other BSSes'
    // frames, and an unassociated station acts only on the beacons and
    // probe responses among them.
    let (mut live, _m) = live_hearing(&|fw| {
        fw.beacon(1, HOME, b"Home", 0x70);
        fw.buffers(1, vec![mgnt_frame(0x08, BROADCAST, HOME, b"data", None, 7)]);
    });
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    let others = live.receiver().counts.other_frames;
    let frames: Vec<Received> = drain_until_ended(&mut live).concat();
    // The beacon alone, on 2.4 GHz channel 6 with its signal; the data frame
    // was counted, not handed out.
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].mpdu[16..22], HOME);
    assert_eq!(
        (frames[0].tag, frames[0].signal),
        (
            Tag {
                band: HwBand::G2,
                channel: 6
            },
            Some(-54)
        )
    );
    assert_eq!(live.receiver().counts.other_frames, others + 1);
    // SAFETY: as above.
    unsafe { live.end_scan() }.unwrap();
    assert_eq!(live.receiver().mode(), Mode::Station);
}

#[test]
fn frames_drained_while_a_scan_starts_or_ends_are_handed_out_first() {
    let (mut live, _m) = live_hearing(&|fw| {
        // A beacon after vif init's last H2C, drained by the scan start's
        // address CAM with the station's filter and waiting for a status
        // that never comes: received before the scan, handed on without a
        // signal (Linux flushes it to mac80211 too).
        fw.after(
            5,
            0x0A,
            vec![mgnt_frame(0x80, BROADCAST, HOME, b"Before", Some(1), 7)],
        );
        // A beacon with its status after ADD_SCANOFLD_CH's acks, drained in
        // SCANOFLD's wait on the operating channel.
        let [frame, status] =
            frame_with_status(0x80, BROADCAST, OFFICE, b"Early", Some(1), 6, 0x70);
        fw.after(9, 0x16, vec![frame, status]);
    });
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    let drains = drain_until_ended(&mut live);
    let first: Vec<([u8; 6], Option<i8>, Tag)> = drains[0]
        .iter()
        .map(|f| (f.mpdu[16..22].try_into().unwrap(), f.signal, f.tag))
        .collect();
    let boot = Tag {
        band: HwBand::G2,
        channel: 1,
    };
    assert_eq!(first, [(HOME, None, boot), (OFFICE, Some(-54), boot)]);
    assert_eq!(live.held_dropped(), 0);
}

#[test]
fn the_drain_that_ends_a_scan_hands_out_its_tail_first() {
    // After END_SCAN: a beacon, its PPDU status in the next batch, then a
    // beacon whose status never comes, all on 5 GHz 36, the last ENTER_CH.
    let (mut live, _m) = live_hearing(&|fw| {
        fw.after_end(vec![mgnt_frame(0x80, BROADCAST, HOME, b"Late", None, 6)]);
        fw.after_end(vec![ppdu_status(6, 0x70, 0)]);
        fw.after_end(vec![mgnt_frame(0x80, BROADCAST, OFFICE, b"Last", None, 5)]);
    });
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    let drains = drain_until_ended(&mut live);
    let last: Vec<([u8; 6], Option<i8>, Tag)> = drains[drains.len() - 1]
        .iter()
        .map(|f| (f.mpdu[16..22].try_into().unwrap(), f.signal, f.tag))
        .collect();
    let ch36 = Tag {
        band: HwBand::G5,
        channel: 36,
    };
    assert_eq!(last, [(HOME, Some(-54), ch36), (OFFICE, None, ch36)]);
    assert!(drains[..drains.len() - 1].iter().all(Vec::is_empty));
}

#[test]
fn a_refused_offload_aborts_and_restores() {
    let (mut live, m) = live_hearing(&|fw| fw.refuse(9, 0x16, 1));
    let before = live.io().trace.len();
    // SAFETY: as in `start_on`.
    let refused = unsafe { live.begin_scan(&three()) };
    assert_eq!(
        refused,
        Err(AxError::H2cRejected {
            class: 9,
            func: 0x16,
            seq: 18,
            ret: 1
        })
    );
    // rtw89_ops_hw_scan's abort (mac80211.c:1313-1317): SCANOFLD stop at
    // seq 19, then rtw89_hw_scan_complete, its address CAM at seq 20.
    let stop = h2c_bytes(&m, 9, 0x17, 19);
    assert_eq!(stop[24..28], [0x00, 0x00, 0x00, 0x00]);
    assert_eq!(live.h2c(), H2c::at(21));
    let t = &live.io().trace[before..];
    // The switch back, then the station's RX filter.
    let switch = first_of(t, |o| matches!(o, Op::W32(0xC398, _)));
    after(t, switch, |o| *o == Op::W32(0xCE20, 0x0317_44BE));
    assert!(!live.scanning());
    assert_eq!(live.receiver().mode(), Mode::Station);
    assert_eq!(live.receiver().tag().channel, 1);
    // Still powered: no teardown, and the next switch runs.
    assert!(!t.contains(&Op::W32(0xDA48, GNT_WL_OFF)));
    // SAFETY: as above.
    unsafe { live.tune(&g2(6)) }.unwrap();
}

#[test]
fn a_failed_scan_start_step_powers_off() {
    // EDCCA's level cannot be read: the scan start stops with TSF sync off.
    let (mut live, _m) = live();
    live.io_mut().fail_reads_of(0x14840);
    // SAFETY: as in `start_on`.
    assert_eq!(unsafe { live.begin_scan(&three()) }, Err(READ_FAILED));
    assert!(!live.scanning());
    assert_powered_off_for_good(live);
}

#[test]
fn a_failed_restore_step_powers_off() {
    let (mut live, _m) = live_hearing(&|_| {});
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    drain_until_ended(&mut live);
    // EDCCA's restore cannot read its level, after the switch and the RX
    // filter went back.
    live.io_mut().fail_reads_of(0x14840);
    // SAFETY: as above.
    assert_eq!(unsafe { live.end_scan() }, Err(READ_FAILED));
    assert!(live.io().trace.contains(&Op::W32(0xCE20, 0x0317_44BE)));
    assert!(!live.scanning());
    assert_powered_off_for_good(live);
}

#[test]
fn scan_calls_out_of_turn_are_refused() {
    let (mut live, _m) = live_hearing(&|_| {});
    let idle = live.io().trace.len();
    // SAFETY: as in `start_on`.
    let nothing = unsafe { live.end_scan() };
    assert_eq!(nothing, Err(AxError::Scan("no scan to end")));
    // No scan: the budget stops nothing.
    // SAFETY: as above.
    unsafe { live.expire_scan(&mut |_| {}) }.unwrap();
    assert_eq!(live.io().trace.len(), idle);
    // SAFETY: as above.
    unsafe { live.begin_scan(&three()) }.unwrap();
    let running = live.io().trace.len();
    // SAFETY: as above.
    assert_eq!(unsafe { live.begin_scan(&three()) }, Err(SCAN_NOT_ENDED));
    // SAFETY: as above.
    assert_eq!(unsafe { live.tune(&g2(6)) }, Err(SCAN_NOT_ENDED));
    // SAFETY: as above.
    let scanned = unsafe { run_once(&mut live) };
    assert_eq!(
        scanned,
        Err(ScanError {
            step: "scan start",
            error: SCAN_NOT_ENDED
        })
    );
    // SAFETY: as above.
    let early = unsafe { live.end_scan() };
    assert_eq!(early, Err(AxError::Scan("the scan is still running")));
    assert_eq!(live.io().trace.len(), running);
    drain_until_ended(&mut live);
    // Ended but not restored: still refused.
    // SAFETY: as above.
    assert_eq!(unsafe { live.begin_scan(&three()) }, Err(SCAN_NOT_ENDED));
    // SAFETY: as above.
    unsafe { live.end_scan() }.unwrap();
    // SAFETY: as above.
    unsafe { live.begin_scan(&three()) }.unwrap();
}

/// The plan's test AP.
const AP: [u8; 6] = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07];

fn ac(aci: u8, aifsn: u8, ecw_min: u8, ecw_max: u8, txop_32us: u16) -> AccessCategory {
    AccessCategory {
        aci,
        admission_control: false,
        aifsn,
        ecw_min,
        ecw_max,
        txop_32us,
    }
}

/// An association with [`AP`] on the live chip's band: AID 1, beacon
/// interval 100, DTIM period 1, every legacy rate, no HT, no WMM.
fn association(live: &Live<RegisterModel>) -> AssocInputs {
    let band = live.chan().band;
    AssocInputs {
        mac: live.mac(),
        bssid: AP,
        aid: 1,
        band,
        beacon_int: 100,
        dtim: 1,
        peer: RaPeer {
            legacy: if band == HwBand::G2 { 0xFFF } else { 0xFF },
            ht: None,
            rssi: 0,
        },
        edca: None,
        short_slot: true,
    }
}

#[test]
fn a_scan_is_refused_while_a_bssid_is_set() {
    let (mut live, _m) = live_hearing(&|_| {});
    // SAFETY: as in `start_on`.
    unsafe { live.set_bssid(AP) }.unwrap();
    assert_eq!(live.bssid(), Some(AP));
    let before = live.io().trace.len();
    // rtw89_is_any_vif_connected_or_connecting (fw.c:10514-10529): a BSSID
    // set means connected or connecting, whose scan this chip does not
    // port; the restore's address CAM would also clear the BSSID entry.
    // SAFETY: as above.
    assert_eq!(unsafe { live.begin_scan(&three()) }, Err(CONNECTED));
    // SAFETY: as above.
    let scanned = unsafe { run_once(&mut live) };
    assert_eq!(
        scanned,
        Err(ScanError {
            step: "scan start",
            error: CONNECTED
        })
    );
    assert_eq!(live.io().trace.len(), before);
    assert!(!live.scanning());
    let a = association(&live);
    // SAFETY: as above.
    unsafe { live.associate(&a) }.unwrap();
    // SAFETY: as above.
    assert_eq!(unsafe { live.begin_scan(&three()) }, Err(CONNECTED));
    // Once the station has left, scans run again.
    // SAFETY: as above.
    unsafe { live.leave() }.unwrap();
    assert_eq!(live.bssid(), None);
    // SAFETY: as above.
    unsafe { live.begin_scan(&three()) }.unwrap();
}

#[test]
fn association_steps_are_refused_while_a_scan_runs() {
    let (mut live, _m) = live_hearing(&|_| {});
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    let before = live.io().trace.len();
    let h2c = live.h2c();
    // Each would drain RXQ outside the scan's tracking, losing END_SCAN.
    // SAFETY: as above.
    assert_eq!(unsafe { live.set_bssid(AP) }, Err(SCAN_NOT_ENDED));
    let a = association(&live);
    // SAFETY: as above.
    assert_eq!(unsafe { live.associate(&a) }, Err(SCAN_NOT_ENDED));
    // SAFETY: as above.
    assert_eq!(unsafe { live.leave() }, Err(SCAN_NOT_ENDED));
    assert_eq!(live.io().trace.len(), before);
    assert_eq!(live.h2c(), h2c);
    assert_eq!(live.bssid(), None);
    // The scan still ends at its END_SCAN.
    drain_until_ended(&mut live);
    assert_eq!(live.scan_state(), Some(ScanEnd::Ok));
    // SAFETY: as above.
    unsafe { live.end_scan() }.unwrap();
    // SAFETY: as above.
    unsafe { live.set_bssid(AP) }.unwrap();
}

#[test]
fn associate_needs_its_bssid_band_and_address() {
    let (mut live, _m) = live_hearing(&|_| {});
    let a = association(&live);
    let before = live.io().trace.len();
    let refused =
        |got: Result<(), AxError>| assert!(matches!(got, Err(AxError::Mac(_))), "{got:?}");
    // No BSSID set yet, and a zero BSSID is no BSSID.
    // SAFETY: as in `start_on`.
    refused(unsafe { live.associate(&a) });
    // SAFETY: as above.
    refused(unsafe { live.set_bssid([0; 6]) });
    assert_eq!(live.io().trace.len(), before);
    // SAFETY: as above.
    unsafe { live.set_bssid([0x02, 0x03, 0x04, 0x05, 0x06, 0x08]) }.unwrap();
    let joined = live.io().trace.len();
    // Another BSSID, another band than the one tuned, another address.
    // SAFETY: as above.
    refused(unsafe { live.associate(&a) });
    // SAFETY: as above.
    unsafe { live.set_bssid(AP) }.unwrap();
    let joined_ap = live.io().trace.len();
    assert!(joined_ap > joined);
    let other_band = AssocInputs {
        band: HwBand::G5,
        ..a
    };
    // SAFETY: as above.
    refused(unsafe { live.associate(&other_band) });
    let other_mac = AssocInputs {
        mac: [0x02, 0, 0, 0, 0, 1],
        ..a
    };
    // SAFETY: as above.
    refused(unsafe { live.associate(&other_mac) });
    assert_eq!(live.io().trace.len(), joined_ap);
    assert!(!live.associated());
    // SAFETY: as above.
    unsafe { live.associate(&a) }.unwrap();
    assert!(live.associated());
    let associated = live.io().trace.len();
    // Associated: no second association, and no new BSSID until it leaves.
    // SAFETY: as above.
    refused(unsafe { live.associate(&a) });
    // SAFETY: as above.
    refused(unsafe { live.set_bssid(AP) });
    assert_eq!(live.io().trace.len(), associated);
}

#[test]
fn a_live_chip_joins_associates_and_leaves() {
    let (mut live, m) = live_hearing(&|fw| {
        // A frame from the AP right after the beacon filter's acks.
        fw.after(9, 0x1E, vec![mgnt_frame(0x08, MAC, AP, b"m1", None, 3)]);
    });
    // SAFETY: as in `start_on`.
    unsafe { live.tune(&g5(36)) }.unwrap();
    // Seq 17: the BSSID's address CAM.
    // SAFETY: as above.
    unsafe { live.set_bssid(AP) }.unwrap();
    let pre_auth = H2c::at(17)
        .addr_cam_with(&AddrCam::with_bssid(MAC, AP), 0, 0)
        .1;
    assert_eq!(h2c_bytes(&m, 6, 0, 17)[24..], pre_auth[24..]);
    let wmm = [
        ac(0, 3, 4, 10, 0),
        ac(1, 7, 4, 10, 0),
        ac(2, 2, 3, 4, 94),
        ac(3, 2, 2, 3, 47),
    ];
    let a = AssocInputs {
        edca: Some(wmm),
        ..association(&live)
    };
    // SAFETY: as above.
    unsafe { live.associate(&a) }.unwrap();
    assert!(live.associated());
    // Seq 18-30: four EDCA, the CMAC table, join, CAM, RA, the beacon filter
    // and four EDCA again (the 5 GHz short slot).
    assert_eq!(live.h2c(), H2c::at(31));
    let connected = H2c::at(24)
        .addr_cam_with(&AddrCam::connected(MAC, AP, 1), 0, 0)
        .1;
    assert_eq!(h2c_bytes(&m, 6, 0, 24)[24..], connected[24..]);
    // The test image is 0.27.66.1: BEACON_FILTER but not
    // BEACON_LOSS_COUNT_V1, so the loss count is capped at 15.
    assert_eq!(live.running().firmware, (0, 27, 66, 1));
    assert_eq!(
        h2c_bytes(&m, 9, 0x1E, 26)[24..28],
        0x0028_4F07u32.to_le_bytes()
    );
    // The frame drained meanwhile comes first in the next drain.
    let mut frames = Vec::new();
    // SAFETY: as above.
    unsafe {
        live.drain(&mut |e| {
            if let Event::Frame(f) = e {
                frames.push(f);
            }
        })
    }
    .unwrap();
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].mpdu[16..22], AP);
    // SAFETY: as above.
    unsafe { live.leave() }.unwrap();
    assert!(!live.associated());
    assert_eq!(live.bssid(), None);
    // Seq 31-43: the beacon filter off, the CMAC table, join, the CAM
    // without the link, the CAM without the BSSID, eight EDCA.
    assert_eq!(live.h2c(), H2c::at(44));
    assert_eq!(
        h2c_bytes(&m, 9, 0x1E, 31)[24..28],
        0x0028_4F00u32.to_le_bytes()
    );
    let cleared = H2c::at(35).addr_cam_unassociated(MAC, 0, 0).1;
    assert_eq!(h2c_bytes(&m, 6, 0, 35)[24..], cleared[24..]);
    // Leaving with nothing set does nothing.
    let left = live.io().trace.len();
    // SAFETY: as above.
    unsafe { live.leave() }.unwrap();
    assert_eq!(live.io().trace.len(), left);
}

#[test]
fn a_scan_after_a_failed_join_finds_dig_paused() {
    let (mut live, _m) = live_hearing(&|_| {});
    let dig_writes = |t: &[Op]| {
        t.iter()
            .filter(|o| matches!(o, Op::W32(0x1481C, _)))
            .count()
    };
    let before = live.io().trace.len();
    // SAFETY: as in `start_on`.
    unsafe { live.set_bssid(AP) }.unwrap();
    assert_eq!(dig_writes(&live.io().trace[before..]), 2);
    // The join fails: the BSSID is cleared, and DIG stays suspended, as
    // Linux resumes it only at association (core.c:5974) or after a scan.
    // SAFETY: as above.
    unsafe { live.leave() }.unwrap();
    let scan = live.io().trace.len();
    // SAFETY: as above.
    unsafe { live.begin_scan(&three()) }.unwrap();
    // The scan's suspend finds DIG paused (phy.c:7859).
    assert_eq!(dig_writes(&live.io().trace[scan..]), 0);
    drain_until_ended(&mut live);
    let ended = live.io().trace.len();
    // SAFETY: as above.
    unsafe { live.end_scan() }.unwrap();
    assert_eq!(dig_writes(&live.io().trace[ended..]), 2);
    // Resumed: the next join suspends it again.
    let join = live.io().trace.len();
    // SAFETY: as above.
    unsafe { live.set_bssid(AP) }.unwrap();
    assert_eq!(dig_writes(&live.io().trace[join..]), 2);
}

#[test]
fn a_failed_association_step_powers_off() {
    let (mut live, _m) = live_hearing(&|_| {});
    // SAFETY: as in `start_on`.
    unsafe { live.set_bssid(AP) }.unwrap();
    // DIG's register cannot be read when association resumes it.
    live.io_mut().fail_reads_of(0x1481C);
    let a = association(&live);
    // SAFETY: as above.
    assert_eq!(unsafe { live.associate(&a) }, Err(READ_FAILED));
    assert!(!live.associated());
    assert_powered_off_for_good(live);
}

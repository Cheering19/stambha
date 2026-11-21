//! The beacon filter's reports on the live chip (M3 Task 18, spec S15,
//! ruling R2): `rtw89_mac_bcn_fltr_rpt`'s BEACON_LOSS handed on and the
//! filter re-armed while associated, a report an operation drained held for
//! the next poll, and everything else ignored.
use super::*;
use crate::wifi::realtek::ax::station::chip::{Chip as StationChip, HardwareKey, Incoming};
use crate::wifi::realtek::ax::tests::firmware::c2h;
use core::cell::RefCell;

/// An obvious test key, no real material: 00 01 .. 0F.
const PTK: [u8; 16] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
];

/// `RTW89_BCN_FLTR_BEACON_LOSS`, `_RSSI` and `_NOTIFY` (`fw.h:273-277`), and
/// `RTW89_BCN_FLTR_RSSI_LOW` (`fw.h:279-283`).
const LOSS: u8 = 0;
const RSSI: u8 = 1;
const NOTIFY: u8 = 2;
const RSSI_LOW: u8 = 1;

/// A BCNFLTR_RPT (cat 1, class 1, func 0xD) of `kind` and `event` for MAC
/// ID `macid`, its moving average `ma` raw (`fw.h:4005-4014`).
fn report(macid: u8, kind: u8, event: u8, ma: u8) -> Vec<u8> {
    let w2 = u32::from(macid) | u32::from(kind) << 8 | u32::from(event) << 10 | u32::from(ma) << 16;
    c2h(1, 1, 0xD, &w2.to_le_bytes())
}

/// A beacon loss for MAC ID 0, at an average of 40, -70 dBm.
fn lost() -> Vec<u8> {
    report(0, LOSS, 0, 40)
}

/// A live chip whose firmware `script` sets up, its BSSID set and
/// associated on 2.4 GHz channel 1 with AID 1: the next H2C is seq 31.
fn associated(script: &dyn Fn(&mut Firmware)) -> (Live<RegisterModel>, Arc<Mutex<Model>>) {
    let (mut live, m) = live_hearing(script);
    // SAFETY: as in `start_on`.
    unsafe { live.set_bssid(AP) }.unwrap();
    let a = association(&live);
    // SAFETY: as above.
    unsafe { live.associate(&a) }.unwrap();
    assert_eq!(live.h2c(), H2c::at(31));
    (live, m)
}

/// Every event `n` polls hand on.
fn polls(live: &mut Live<RegisterModel>, n: usize) -> Vec<Incoming> {
    let mut seen = Vec::new();
    for _ in 0..n {
        StationChip::poll(live, &mut |e| seen.push(e)).unwrap();
    }
    seen
}

/// The beacon filter H2Cs the firmware read, as their w0.
fn filters(log: &RefCell<Vec<Vec<u8>>>) -> Vec<u32> {
    log.borrow()
        .iter()
        .filter(|h2c| {
            let hdr0 = u32::from_le_bytes(h2c[..4].try_into().unwrap());
            (hdr0 & 0x3, (hdr0 >> 2) & 0x3F, (hdr0 >> 8) & 0xFF) == (1, 9, 0x1E)
        })
        .map(|h2c| u32::from_le_bytes(h2c[8..12].try_into().unwrap()))
        .collect()
}

#[test]
fn beacon_loss_while_associated_rearms_and_reports() {
    // rtw89_mac_bcn_fltr_rpt (mac.c:5415-5425): a BEACON_LOSS for the
    // vif's MAC ID, no scan running, goes to mac80211 (ieee80211_beacon_loss),
    // then the filter is armed again in its connect form, without a wait.
    let (mut live, m) = live_hearing(&|fw| fw.after(0xC, 0, vec![lost()]));
    // The core start's dynamic BA CAM H2Cs are class 0xC too: their reports
    // come while no BSSID is set, and are dropped.
    assert!(polls(&mut live, 8).is_empty());
    // SAFETY: as in `start_on`.
    unsafe { live.set_bssid(AP) }.unwrap();
    let a = association(&live);
    // SAFETY: as above.
    unsafe { live.associate(&a) }.unwrap();
    assert_eq!(live.h2c(), H2c::at(31));
    // The BA CAM's acks, then the loss, which the poll drains itself.
    // SAFETY: as above.
    unsafe { live.start_rx_block_ack(0, 0, 64) }.unwrap();
    let before = live.io().trace.len();
    let seen = polls(&mut live, 4);
    assert_eq!(seen, [Incoming::BeaconLost]);
    // Seq 32, after the BA CAM's 31: the test image's 0x00284F07, MON_RSSI,
    // MON_BCN and MON_EN with the loss count capped at 15 (0.27.66.1 lacks
    // BEACON_LOSS_COUNT_V1), as the association armed it.
    assert_eq!(
        h2c_bytes(&m, 9, 0x1E, 32),
        H2c::at(32).bcn_fltr(0, true, false).1
    );
    assert_eq!(
        h2c_bytes(&m, 9, 0x1E, 32)[24..28],
        0x0028_4F07u32.to_le_bytes()
    );
    assert_eq!(kicks(&live.io().trace[before..]), 1);
    assert_eq!(live.h2c(), H2c::at(33));
    assert!(live.associated());
}

#[test]
fn a_beacon_loss_an_operation_drains_is_kept_for_the_next_poll() {
    // The report comes right after SEC_UPD's REC_ACK, so add_key's waits
    // drain it: held as the frames drained meanwhile are, it is taken on
    // the next poll, and nothing is sent before.
    let (mut live, m) = associated(&|fw| fw.after(0xA, 1, vec![lost()]));
    let key = HardwareKey {
        pairwise: true,
        index: 0,
        material: &PTK,
    };
    // SAFETY: as in `start_on`.
    assert_eq!(unsafe { live.add_key(&key) }, Ok(0));
    assert_eq!(live.h2c(), H2c::at(34));
    assert_eq!(polls(&mut live, 1), [Incoming::BeaconLost]);
    assert_eq!(
        h2c_bytes(&m, 9, 0x1E, 34),
        H2c::at(34).bcn_fltr(0, true, false).1
    );
    assert_eq!(live.h2c(), H2c::at(35));
    assert_eq!(live.held_dropped(), 0);
}

#[test]
fn beacon_loss_while_scanning_is_ignored() {
    // A scan runs only while no BSSID is set (spec S1.15): during it and
    // after it, a loss reaches neither mac80211 (`rtwdev->scanning`, and no
    // association) nor the filter, which Linux arms only for an INFRA vif
    // (fw.c:5413). Nor does one held by the BSSID's address CAM.
    let log = RefCell::new(None);
    let (mut live, _m) = live_hearing(&|fw| {
        *log.borrow_mut() = Some(fw.record());
        fw.buffers(0, vec![lost()]);
        fw.after_end(vec![lost()]);
        fw.after(6, 0, vec![lost()]);
    });
    let log = log.into_inner().unwrap();
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    let mut seen = Vec::new();
    for _ in 0..100 {
        StationChip::poll(&mut live, &mut |e| seen.push(e)).unwrap();
        if seen.contains(&Incoming::ScanEnded { completed: true }) {
            break;
        }
    }
    assert!(seen.contains(&Incoming::ScanEnded { completed: true }));
    // SAFETY: as above.
    unsafe { live.end_scan() }.unwrap();
    seen.extend(polls(&mut live, 4));
    // SAFETY: as above.
    unsafe { live.set_bssid(AP) }.unwrap();
    seen.extend(polls(&mut live, 4));
    assert!(!seen.contains(&Incoming::BeaconLost), "{seen:?}");
    assert!(filters(&log).is_empty());
}

#[test]
fn rssi_reports_are_ignored() {
    // No CQM RSSI monitor is set (ieee80211_cqm_rssi_notify has no
    // listener): an RSSI report, low at a raw average of 20 (-90 dBm), and
    // a NOTIFY, are dropped without an H2C.
    let (mut live, _m) = associated(&|fw| {
        fw.after(
            0xC,
            0,
            vec![report(0, RSSI, RSSI_LOW, 20), report(0, NOTIFY, 0, 60)],
        )
    });
    // SAFETY: as in `start_on`.
    unsafe { live.start_rx_block_ack(0, 0, 64) }.unwrap();
    assert!(polls(&mut live, 4).is_empty());
    assert_eq!(live.h2c(), H2c::at(32));
}

#[test]
fn a_loss_for_another_mac_id_is_ignored() {
    // rtw89_mac_bcn_fltr_rpt returns for a MAC ID not the vif link's.
    let (mut live, _m) = associated(&|fw| fw.after(0xC, 0, vec![report(1, LOSS, 0, 40)]));
    // SAFETY: as in `start_on`.
    unsafe { live.start_rx_block_ack(0, 0, 64) }.unwrap();
    assert!(polls(&mut live, 4).is_empty());
    assert_eq!(live.h2c(), H2c::at(32));
}

#[test]
fn a_loss_after_leaving_is_ignored() {
    // Leaving turns the filter off (spec S11); a loss the leave drained, or
    // one after it, finds no association.
    let log = RefCell::new(None);
    let (mut live, _m) = associated(&|fw| {
        *log.borrow_mut() = Some(fw.record());
        fw.after(8, 0, vec![lost()]);
    });
    let log = log.into_inner().unwrap();
    // The join info of the association brought one: on.
    assert_eq!(polls(&mut live, 4), [Incoming::BeaconLost]);
    let armed = filters(&log);
    // SAFETY: as in `start_on`.
    unsafe { live.leave() }.unwrap();
    assert!(polls(&mut live, 4).is_empty());
    // The association's filter, the re-arm, then the leave's filter off.
    assert_eq!(armed, [0x0028_4F07, 0x0028_4F07]);
    assert_eq!(filters(&log), [0x0028_4F07, 0x0028_4F07, 0x0028_4F00]);
}

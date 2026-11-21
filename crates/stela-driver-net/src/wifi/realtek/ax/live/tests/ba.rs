//! RX Block Ack on the live chip (M3 Task 16, spec S13.2): the BA CAM H2Cs
//! of `rtw89_ops_ampdu_action`, the eight static entries, the refusals, the
//! stops a leave runs before the keys, and `rtw89_core_skb_pn_valid` on what
//! the station receives.
use super::*;
use crate::wifi::realtek::ax::station::chip::{Chip as StationChip, HardwareKey, Incoming};
use crate::wifi::realtek::ax::tests::firmware::rx_buffer;
use core::cell::RefCell;

/// An obvious test key, no real material: 00 01 .. 0F.
const PTK: [u8; 16] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
];

fn ptk() -> HardwareKey<'static> {
    HardwareKey {
        pairwise: true,
        index: 0,
        material: &PTK,
    }
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

/// The 8-byte payload of the BA CAM H2C of `seq`.
fn ba_cam(m: &Arc<Mutex<Model>>, seq: u8) -> Vec<u8> {
    h2c_bytes(m, 0xC, 0, seq)[24..].to_vec()
}

/// The start payload for `tid`, `ssn` and static entry `entry`, a window of
/// 64 or less: VALID | INIT_REQ | TID | SSN, STD_EN | entry.
fn started(tid: u8, ssn: u16, entry: u8) -> Vec<u8> {
    let w0 = 3 | u32::from(tid) << 4 | u32::from(ssn) << 20;
    let w1 = 0x100 | u32::from(entry) << 28;
    [w0.to_le_bytes(), w1.to_le_bytes()].concat()
}

/// The stop payload for static entry `entry`.
fn stopped(entry: u8) -> Vec<u8> {
    [0u32.to_le_bytes(), (u32::from(entry) << 28).to_le_bytes()].concat()
}

#[test]
fn start_and_stop_send_the_ba_cam_h2c() {
    let (mut live, m) = associated(&|_| {});
    let before = live.io().trace.len();
    // SAFETY: as in `start_on`.
    assert_eq!(unsafe { live.start_rx_block_ack(0, 0x123, 64) }, Ok(()));
    // rtw89_fw_h2c_ba_cam (fw.c:2574-2651): static entry 0, MAC ID 0, band
    // 0, sent without a wait (rtw89_h2c_tx(.., false)).
    assert_eq!(
        ba_cam(&m, 31),
        [0x03, 0x00, 0x30, 0x12, 0x00, 0x01, 0x00, 0x00]
    );
    assert_eq!(
        h2c_bytes(&m, 0xC, 0, 31),
        H2c::at(31).ba_cam(0, 0, 0x123, 64, 0, true).1
    );
    assert_eq!(kicks(&live.io().trace[before..]), 1);
    // SAFETY: as above.
    assert_eq!(unsafe { live.stop_rx_block_ack(0) }, Ok(()));
    assert_eq!(ba_cam(&m, 32), [0; 8]);
    assert_eq!(live.h2c(), H2c::at(33));
    // The entry is gone: a second stop finds none (-ENOENT) and sends
    // nothing, as Linux returns 0.
    let before = live.io().trace.len();
    // SAFETY: as above.
    assert_eq!(unsafe { live.stop_rx_block_ack(0) }, Ok(()));
    assert_eq!(live.io().trace.len(), before);
    let d = StationChip::diagnostic(&live);
    assert!(d.contains(", ba started 1 stopped 2"), "{d}");
}

#[test]
fn ninth_agreement_skips_the_h2c() {
    let (mut live, m) = associated(&|_| {});
    for tid in 0..8u8 {
        // SAFETY: as in `start_on`.
        unsafe { live.start_rx_block_ack(tid, u16::from(tid), 64) }.unwrap();
        assert_eq!(ba_cam(&m, 31 + tid), started(tid, u16::from(tid), tid));
    }
    // All eight taken: no H2C (rtw89_core_acquire_sta_ba_entry's -ENOSPC);
    // the hardware's dynamic BA CAM covers the agreement (fw.c:2592-2600).
    let before = live.io().trace.len();
    let h2c = live.h2c();
    // SAFETY: as above.
    assert_eq!(unsafe { live.start_rx_block_ack(8, 0, 64) }, Ok(()));
    // SAFETY: as above.
    assert_eq!(unsafe { live.stop_rx_block_ack(8) }, Ok(()));
    assert_eq!(live.io().trace.len(), before);
    assert_eq!(live.h2c(), h2c);
    // Freed, an entry is the first free again.
    // SAFETY: as above.
    unsafe { live.stop_rx_block_ack(3) }.unwrap();
    assert_eq!(ba_cam(&m, 39), stopped(3));
    // SAFETY: as above.
    unsafe { live.start_rx_block_ack(9, 0x456, 64) }.unwrap();
    assert_eq!(ba_cam(&m, 40), started(9, 0x456, 3));
    assert_eq!(live.ba_tids(), [0, 1, 2, 4, 5, 6, 7, 9]);
}

#[test]
fn a_full_ba_cam_gives_tid_0_another_tids_entry() {
    let (mut live, m) = associated(&|_| {});
    for tid in 8..16u8 {
        // SAFETY: as in `start_on`.
        unsafe { live.start_rx_block_ack(tid, 0, 64) }.unwrap();
    }
    // TID 0 takes entry 0, which TID 8 held (core.c:5615-5633), whose
    // stop then finds no entry.
    // SAFETY: as above.
    unsafe { live.start_rx_block_ack(0, 0x10, 64) }.unwrap();
    assert_eq!(ba_cam(&m, 39), started(0, 0x10, 0));
    let before = live.io().trace.len();
    // SAFETY: as above.
    unsafe { live.stop_rx_block_ack(8) }.unwrap();
    assert_eq!(live.io().trace.len(), before);
    // SAFETY: as above.
    unsafe { live.stop_rx_block_ack(0) }.unwrap();
    assert_eq!(ba_cam(&m, 40), stopped(0));
}

#[test]
fn block_ack_is_refused_before_any_io() {
    let (mut live, _m) = live_hearing(&|_| {});
    let before = live.io().trace.len();
    // RX_START comes only for an associated station.
    // SAFETY: as in `start_on`.
    let got = unsafe { live.start_rx_block_ack(0, 0, 64) };
    assert_eq!(
        got,
        Err(AxError::Mac("a Block Ack agreement without an association"))
    );
    // Nothing to stop.
    // SAFETY: as above.
    assert_eq!(unsafe { live.stop_rx_block_ack(0) }, Ok(()));
    assert_eq!(live.io().trace.len(), before);
    let (mut live, _m) = associated(&|_| {});
    let before = live.io().trace.len();
    // The H2C's TID field is four bits (IEEE80211_NUM_TIDS).
    let past = Err(AxError::Mac("a TID past 15"));
    // SAFETY: as above.
    assert_eq!(unsafe { live.start_rx_block_ack(16, 0, 64) }, past);
    // SAFETY: as above.
    assert_eq!(unsafe { live.stop_rx_block_ack(16) }, past);
    assert_eq!(live.io().trace.len(), before);
    assert!(live.ba_tids().is_empty());
}

#[test]
fn a_ba_cam_that_cannot_be_sent_keeps_its_entry_and_the_chip_on() {
    // Linux takes the entry before the H2C and keeps it when rtw89_h2c_tx
    // fails; nothing powers the chip off.
    let (mut live, _m) = associated(&|_| {});
    live.io_mut().fail_reads_of(0x1080);
    // SAFETY: as in `start_on`.
    let got = unsafe { live.start_rx_block_ack(0, 0, 64) };
    assert_eq!(got, Err(READ_FAILED));
    assert_eq!(live.ba_tids(), [0]);
    assert!(!StationChip::diagnostic(&live).contains("powered off"));
}

#[test]
fn leave_stops_what_the_station_left_before_the_keys() {
    // __sta_info_destroy_part1 tears the agreements down before _part2
    // deletes the keys (sta_info.c:1310-1348, :1543-1578).
    let log = RefCell::new(None);
    let (mut live, _m) = associated(&|fw| *log.borrow_mut() = Some(fw.record()));
    let log = log.into_inner().unwrap();
    // SAFETY: as in `start_on`.
    assert_eq!(unsafe { live.add_key(&ptk()) }, Ok(0));
    // SAFETY: as above.
    unsafe { live.start_rx_block_ack(0, 0, 64) }.unwrap();
    // SAFETY: as above.
    unsafe { live.start_rx_block_ack(6, 0, 64) }.unwrap();
    assert_eq!(live.h2c(), H2c::at(36));
    let from = log.borrow().len();
    // SAFETY: as above.
    unsafe { live.leave() }.unwrap();
    // As the firmware read them, past the CH12 ring's 16 slots, without
    // their descriptors: the stops of entry 0 (36) and 1 (37), then the
    // key's dctl (38) and address CAM (39).
    let sent = log.borrow()[from..].to_vec();
    let named = |at: usize| {
        let hdr0 = u32::from_le_bytes(sent[at][..4].try_into().unwrap());
        ((hdr0 >> 2) & 0x3F, (hdr0 >> 8) & 0xFF, hdr0 >> 24)
    };
    assert_eq!(
        [named(0), named(1), named(2), named(3)],
        [(0xC, 0, 36), (0xC, 0, 37), (5, 9, 38), (6, 0, 39)]
    );
    assert_eq!(sent[0][8..], stopped(0));
    assert_eq!(sent[1][8..], stopped(1));
    assert!(live.ba_tids().is_empty());
    assert!(!live.key_installed(0));
}

/// QoS data of TID 0 from the AP to the station, sequence number `sn`,
/// packet number `pn` in its CCMP header, as the hardware hands it on
/// decrypted in a long RX descriptor: A1_MATCH and HW_DEC in d3, MAC ID 0
/// in d5, `RTW89_SEC_KEY_TYPE_CCMP128` in d7 (`txrx.h:373-424`).
fn decrypted(sn: u16, pn: u64) -> Vec<u8> {
    let mut f = vec![0x88, 0x42, 0, 0];
    f.extend_from_slice(&MAC);
    f.extend_from_slice(&AP);
    f.extend_from_slice(&AP);
    f.extend_from_slice(&(sn << 4).to_le_bytes());
    f.extend_from_slice(&[0, 0]);
    let [p0, p1, p2, p3, p4, p5, _, _] = pn.to_le_bytes();
    f.extend_from_slice(&[p0, p1, 0, 0x20, p2, p3, p4, p5]);
    f.extend_from_slice(&[0xAA; 20]);
    f.extend_from_slice(&[0; 8]);
    f.extend_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
    let mut b = rx_buffer(0, true, 2, 0, &f);
    b[16..20].copy_from_slice(&(1u32 | 1 << 2).to_le_bytes());
    b[32..36].copy_from_slice(&(6u32 << 17).to_le_bytes());
    b
}

/// The sequence numbers of the frames four polls hand on.
fn heard_sns(live: &mut Live<RegisterModel>) -> Vec<u16> {
    let mut sns = Vec::new();
    for _ in 0..4 {
        for e in poll_all(live) {
            if let Incoming::Frame(frame) = e {
                sns.push(u16::from_le_bytes([frame.bytes[22], frame.bytes[23]]) >> 4);
            }
        }
    }
    sns
}

fn poll_all(live: &mut Live<RegisterModel>) -> Vec<Incoming> {
    let mut seen = Vec::new();
    StationChip::poll(live, &mut |e| seen.push(e)).unwrap();
    seen
}

/// Three frames on TID 0 right after the PTK's SEC_UPD: SN 10 under PN 5,
/// SN 9 under PN 6, SN 11 under PN 7.
fn after_the_key(fw: &mut Firmware) {
    fw.after(
        0xA,
        1,
        vec![decrypted(10, 5), decrypted(9, 6), decrypted(11, 7)],
    );
}

#[test]
fn the_pn_check_runs_on_what_the_station_receives() {
    // rtw89_core_rx's rtw89_core_skb_pn_valid (core.c:4551, :3943-4006):
    // with TID 0's agreement started, the later PN under the earlier SN is
    // dropped before mac80211 sees it.
    let (mut live, _m) = associated(&after_the_key);
    // SAFETY: as in `start_on`.
    unsafe { live.start_rx_block_ack(0, 10, 64) }.unwrap();
    // SAFETY: as above.
    unsafe { live.add_key(&ptk()) }.unwrap();
    assert_eq!(heard_sns(&mut live), [10, 11]);
    let d = StationChip::diagnostic(&live);
    assert!(d.contains(", pn dropped 1"), "{d}");
}

#[test]
fn without_an_agreement_every_frame_passes() {
    let (mut live, _m) = associated(&after_the_key);
    // SAFETY: as in `start_on`.
    unsafe { live.add_key(&ptk()) }.unwrap();
    assert_eq!(heard_sns(&mut live), [10, 9, 11]);
    assert!(!StationChip::diagnostic(&live).contains("pn dropped"));
}

#[test]
fn a_stopped_agreement_is_not_checked() {
    let (mut live, _m) = associated(&after_the_key);
    // SAFETY: as in `start_on`.
    unsafe { live.start_rx_block_ack(0, 10, 64) }.unwrap();
    // SAFETY: as above.
    unsafe { live.stop_rx_block_ack(0) }.unwrap();
    // SAFETY: as above.
    unsafe { live.add_key(&ptk()) }.unwrap();
    assert_eq!(heard_sns(&mut live), [10, 9, 11]);
}

/// [`decrypted`] as the receiver hands it on.
fn received(sn: u16, pn: u64) -> Received {
    let buffer = decrypted(sn, pn);
    let mut rx = crate::wifi::realtek::ax::receive::Receiver::new(BOOT_CHANNEL);
    rx.set_mode(Mode::Station);
    let mut out = None;
    rx.buffer(&buffer, &mut |e| {
        if let Event::Frame(frame) = e {
            out = Some(frame);
        }
    })
    .unwrap();
    out.unwrap()
}

#[test]
fn a_pairwise_key_starts_the_checks_over() {
    let (mut live, _m) = associated(&|_| {});
    // SAFETY: as in `start_on`.
    unsafe { live.start_rx_block_ack(0, 10, 64) }.unwrap();
    assert!(live.pn_valid(&received(10, 5)));
    assert!(!live.pn_valid(&received(9, 6)));
    // rtw89_ops_set_key's SET_KEY of a pairwise key resets the started TIDs
    // (mac80211.c:1007-1008): the earlier SN goes.
    // SAFETY: as above.
    unsafe { live.add_key(&ptk()) }.unwrap();
    assert!(live.pn_valid(&received(9, 6)));
    // A group key does not.
    let gtk = HardwareKey {
        pairwise: false,
        index: 1,
        material: &PTK,
    };
    // SAFETY: as above.
    unsafe { live.add_key(&gtk) }.unwrap();
    assert!(!live.pn_valid(&received(8, 7)));
    // The station gone, nothing is checked.
    // SAFETY: as above.
    unsafe { live.leave() }.unwrap();
    assert!(live.pn_valid(&received(7, 8)));
}

#[test]
fn a_frame_the_poll_drains_is_checked_as_it_arrives() {
    // RXQ's own drain (rtw89_pci_poll_rxq_dma to rtw89_core_rx): the three
    // frames right after the agreement's BA CAM H2C reach the next polls.
    let (mut live, _m) = live_hearing(&|fw| {
        fw.after(
            0xC,
            0,
            vec![decrypted(10, 5), decrypted(9, 6), decrypted(11, 7)],
        )
    });
    // The core start's dynamic BA CAM H2Cs brought theirs while nothing was
    // associated, which the check passes.
    heard_sns(&mut live);
    heard_sns(&mut live);
    // SAFETY: as in `start_on`.
    unsafe { live.set_bssid(AP) }.unwrap();
    let a = association(&live);
    // SAFETY: as above.
    unsafe { live.associate(&a) }.unwrap();
    // SAFETY: as above.
    unsafe { live.add_key(&ptk()) }.unwrap();
    // SAFETY: as above.
    unsafe { live.start_rx_block_ack(0, 10, 64) }.unwrap();
    assert_eq!(heard_sns(&mut live), [10, 11]);
    let d = StationChip::diagnostic(&live);
    assert!(d.contains(", pn dropped 1"), "{d}");
}

#[test]
fn a_frame_an_install_drains_is_checked_before_the_install_starts_over() {
    // rtw89_core_rx runs the PN check as a frame arrives (core.c:4551): one
    // that add_key's waits drain is checked before rtw89_ops_set_key's reset
    // after the install (mac80211.c:1007-1008), which then forgets it, and
    // it is not checked again when the next poll hands it on.
    let (mut live, _m) = associated(&|fw| fw.after(0xA, 1, vec![decrypted(20, 5)]));
    // SAFETY: as in `start_on`.
    unsafe { live.start_rx_block_ack(0, 10, 64) }.unwrap();
    // SAFETY: as above.
    unsafe { live.add_key(&ptk()) }.unwrap();
    assert_eq!(heard_sns(&mut live), [20]);
    // TID 0 started over after SN 20: an earlier SN under a later PN is the
    // first the check sees.
    assert!(live.pn_valid(&received(15, 6)));
    assert!(!StationChip::diagnostic(&live).contains("pn dropped"));
}

#[test]
fn a_powered_off_chip_holds_no_agreement() {
    let (mut live, _m) = associated(&|_| {});
    // SAFETY: as in `start_on`.
    unsafe { live.start_rx_block_ack(0, 0, 64) }.unwrap();
    // A channel switch failing after its first access powers the chip off,
    // its BA CAM with the firmware.
    live.io_mut().fail_reads_of(0x156CC);
    // SAFETY: as above.
    assert!(unsafe { live.tune(&g5(36)) }.is_err());
    assert!(live.ba_tids().is_empty());
    // SAFETY: as above.
    let started = unsafe { live.start_rx_block_ack(0, 0, 64) };
    assert_eq!(started, Err(POWERED_OFF));
    // SAFETY: as above.
    assert_eq!(unsafe { live.stop_rx_block_ack(0) }, Err(POWERED_OFF));
}

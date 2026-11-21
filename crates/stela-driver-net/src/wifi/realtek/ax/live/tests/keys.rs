//! Keys on the live chip (M3 Task 14, spec S12): the security CAM install
//! with its two waits, its refusals, the delete, and the deletes a leave
//! runs first.
use super::*;
use crate::wifi::realtek::ax::assoc::CamKeys;
use crate::wifi::realtek::ax::keys::NO_KEY;
use crate::wifi::realtek::ax::station::chip::{Chip as _, HardwareKey};
use crate::wifi::realtek::ax::tests::firmware::done_ack;
use core::cell::RefCell;

/// Obvious test keys, no real material: 00 01 .. 0F and F0 F1 .. FF.
const PTK: [u8; 16] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
];
const GTK: [u8; 16] = [
    0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
];

fn pairwise() -> HardwareKey<'static> {
    HardwareKey {
        pairwise: true,
        index: 0,
        material: &PTK,
    }
}

fn group(index: u8) -> HardwareKey<'static> {
    HardwareKey {
        pairwise: false,
        index,
        material: &GTK,
    }
}

/// A live chip whose firmware `script` sets up, its BSSID set (seq 17) and
/// associated on 2.4 GHz channel 1 with AID 1 (seq 18-30): the next H2C is
/// seq 31.
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

/// The associated station's address CAM with `keys` at `seq`.
fn cam_with(keys: CamKeys, seq: u8) -> Vec<u8> {
    let cam = AddrCam {
        keys,
        ..AddrCam::connected(MAC, AP, 1)
    };
    H2c::at(seq).addr_cam_with(&cam, 0, 0).1
}

/// The positions of the CH12 kicks in `t`.
fn kick_at(t: &[Op]) -> Vec<usize> {
    t.iter()
        .enumerate()
        .filter(|(_, o)| matches!(o, Op::W16(0x1080, _)))
        .map(|(i, _)| i)
        .collect()
}

/// Whether an RXQ index read lies strictly between trace entries `a` and
/// `b`.
fn rxq_read_between(t: &[Op], a: usize, b: usize) -> bool {
    t[a + 1..b].contains(&Op::R32(0x1218))
}

const PTK_KEYS: CamKeys = CamKeys {
    keyids: 0,
    valid_and_ents: 0x0000_0001,
    ents_hi: 0,
};
const BOTH_KEYS: CamKeys = CamKeys {
    keyids: 0x0040_0000,
    valid_and_ents: 0x0100_0005,
    ents_hi: 0,
};

#[test]
fn add_key_order_and_waits() {
    let (mut live, m) = associated(&|_| {});
    let before = live.io().trace.len();
    // SAFETY: as in `start_on`.
    assert_eq!(unsafe { live.add_key(&pairwise()) }, Ok(0));
    // rtw89_cam_sec_key_install (cam.c:399-463): SEC_UPD for sec CAM entry
    // 0 (seq 31), then rtw89_cam_attach_sec_cam's dctl (32) and address CAM
    // (33) with slot 0 valid.
    assert_eq!(h2c_bytes(&m, 0xA, 1, 31), H2c::at(31).sec_upd(0, &PTK).1);
    assert_eq!(
        h2c_bytes(&m, 5, 9, 32),
        H2c::at(32).dctl_sec(0, &PTK_KEYS).1
    );
    assert_eq!(h2c_bytes(&m, 6, 0, 33), cam_with(PTK_KEYS, 33));
    assert_eq!(live.h2c(), H2c::at(34));
    // The SEC_UPD kick, its REC_ACK read from RXQ, the dctl kick, the CAM
    // kick, then its DONE_ACK read: each wait came before what follows it.
    let t = &live.io().trace[before..];
    let k = kick_at(t);
    assert_eq!(k.len(), 3);
    assert!(rxq_read_between(t, k[0], k[1]));
    assert!(t[k[2] + 1..].contains(&Op::R32(0x1218)));
    // Both acks came at once: no wait ran out.
    assert!(delays(t) < 100_000, "{}", delays(t));
    assert!(live.key_installed(0));
    assert_eq!(live.cam_keys(), PTK_KEYS);
    // The group key 1 takes entry 1, in slot 2 (spec S12.3).
    // SAFETY: as above.
    assert_eq!(unsafe { live.add_key(&group(1)) }, Ok(1));
    assert_eq!(h2c_bytes(&m, 0xA, 1, 34), H2c::at(34).sec_upd(1, &GTK).1);
    assert_eq!(
        h2c_bytes(&m, 5, 9, 35),
        H2c::at(35).dctl_sec(0, &BOTH_KEYS).1
    );
    assert_eq!(h2c_bytes(&m, 6, 0, 36), cam_with(BOTH_KEYS, 36));
    assert_eq!(live.cam_keys(), BOTH_KEYS);
}

#[test]
fn sec_upd_without_rec_ack_is_refused() {
    let (mut live, m) = associated(&|fw| fw.silent(0xA, 1));
    let before = live.io().trace.len();
    // SAFETY: as in `start_on`.
    let got = unsafe { live.add_key(&pairwise()) };
    assert!(
        matches!(
            got,
            Err(AxError::NoAck {
                class: 0xA,
                func: 1,
                seq: 31,
                ..
            })
        ),
        "{got:?}"
    );
    // The REC_ACK wait ran out; no dctl and no address CAM followed.
    let t = &live.io().trace[before..];
    assert_eq!(kicks(t), 1);
    assert!(delays(t) >= 100_000, "{}", delays(t));
    // Nothing kept: the entry is free again, no slot is valid, and the chip
    // stays on for the station to leave and reconnect.
    assert!(!live.key_installed(0));
    assert_eq!(live.cam_keys(), CamKeys::default());
    assert!(live.associated());
    // The next attempt takes the same entry.
    // SAFETY: as above.
    assert!(unsafe { live.add_key(&pairwise()) }.is_err());
    assert_eq!(h2c_bytes(&m, 0xA, 1, 32), H2c::at(32).sec_upd(0, &PTK).1);
}

#[test]
fn cam_done_ack_nonzero_is_refused() {
    let (mut live, m) = associated(&|fw| fw.refuse(6, 0, 1));
    // SAFETY: as in `start_on`.
    let got = unsafe { live.add_key(&pairwise()) };
    assert_eq!(
        got,
        Err(AxError::H2cRejected {
            class: 6,
            func: 0,
            seq: 33,
            ret: 1
        })
    );
    // The dctl and the address CAM were sent with the slot valid.
    assert_eq!(h2c_bytes(&m, 6, 0, 33), cam_with(PTK_KEYS, 33));
    // Reverted as rtw89_cam_sec_key_install's error path does (cam.c:280-
    // 286, :455-462): the slot's valid bit cleared, the entry freed.
    assert!(!live.key_installed(0));
    assert_eq!(live.cam_keys(), CamKeys::default());
    assert!(live.associated());
    assert!(!live.diagnostic().contains("powered off"));
}

#[test]
fn cam_without_done_ack_is_refused_after_100_ms() {
    let (mut live, _m) = associated(&|fw| fw.silent(6, 0));
    let before = live.io().trace.len();
    // SAFETY: as in `start_on`.
    let got = unsafe { live.add_key(&pairwise()) };
    assert!(
        matches!(
            got,
            Err(AxError::NoAck {
                class: 6,
                func: 0,
                seq: 33,
                ..
            })
        ),
        "{got:?}"
    );
    assert!(delays(&live.io().trace[before..]) >= 100_000);
    assert!(!live.key_installed(0));
}

#[test]
fn a_late_cam_done_ack_does_not_complete_a_key() {
    // The Task 7 carry: the association's address CAM (seq 24) is answered
    // only after the key's dctl, while the key's own CAM gets no answer.
    // Matched on class and func alone, as Wait::Done is, that late
    // DONE_ACK would complete the install.
    let (mut live, _m) = associated(&|fw| {
        fw.silent(6, 0);
        fw.after(5, 9, vec![done_ack(6, 0, 24, 0)]);
    });
    // SAFETY: as in `start_on`.
    let got = unsafe { live.add_key(&pairwise()) };
    assert!(
        matches!(
            got,
            Err(AxError::NoAck {
                class: 6,
                func: 0,
                seq: 33,
                unmatched_ack: 1..,
                ..
            })
        ),
        "{got:?}"
    );
    assert!(!live.key_installed(0));
}

#[test]
fn remove_key_sends_dctl_then_cam_and_frees() {
    let (mut live, m) = associated(&|_| {});
    // SAFETY: as in `start_on`.
    unsafe { live.add_key(&pairwise()) }.unwrap();
    // SAFETY: as above.
    unsafe { live.add_key(&group(1)) }.unwrap();
    let before = live.io().trace.len();
    // SAFETY: as above.
    unsafe { live.remove_key(1) }.unwrap();
    // rtw89_cam_sec_key_del (cam.c:537-566): slot 2's valid bit cleared, its
    // key ID and entry kept (cam.c:215-246); the dctl (seq 37) and the
    // address CAM (38); no SEC_UPD; entry 1 freed.
    let gone = CamKeys {
        keyids: 0x0040_0000,
        valid_and_ents: 0x0100_0001,
        ents_hi: 0,
    };
    assert_eq!(h2c_bytes(&m, 5, 9, 37), H2c::at(37).dctl_sec(0, &gone).1);
    assert_eq!(h2c_bytes(&m, 6, 0, 38), cam_with(gone, 38));
    assert_eq!(live.h2c(), H2c::at(39));
    let t = &live.io().trace[before..];
    assert_eq!(kicks(t), 2);
    // Linux waits for neither.
    assert_eq!(delays(t), 0);
    assert!(!live.key_installed(1) && live.key_installed(0));
    assert_eq!(live.cam_keys(), gone);
    // A rekey takes the entry and the slot again.
    // SAFETY: as above.
    assert_eq!(unsafe { live.add_key(&group(2)) }, Ok(1));
    assert_eq!(live.cam_keys().keyids, 0x0080_0000);
    assert_eq!(live.cam_keys().valid_and_ents, 0x0100_0005);
    // An entry holding no key is refused before any access.
    let before = live.io().trace.len();
    // SAFETY: as above.
    let got = unsafe { live.remove_key(5) };
    assert_eq!(got, Err(NO_KEY));
    assert_eq!(live.io().trace.len(), before);
}

#[test]
fn keys_need_an_association_and_a_key_id_of_two_bits() {
    let (mut live, _m) = live_hearing(&|_| {});
    let refused = |live: &mut Live<RegisterModel>, key: &HardwareKey<'_>| {
        let (before, h2c) = (live.io().trace.len(), live.h2c());
        // SAFETY: as in `start_on`.
        let got = unsafe { live.add_key(key) };
        assert!(matches!(got, Err(AxError::Mac(_))), "{got:?}");
        assert_eq!((live.io().trace.len(), live.h2c()), (before, h2c));
    };
    // mac80211 refuses a station's key before ASSOC (ieee80211_add_key,
    // net/mac80211/cfg.c: -ENOENT); the chip refuses it too, unjoined and
    // joined.
    refused(&mut live, &pairwise());
    // SAFETY: as in `start_on`.
    unsafe { live.set_bssid(AP) }.unwrap();
    refused(&mut live, &pairwise());
    let a = association(&live);
    // SAFETY: as above.
    unsafe { live.associate(&a) }.unwrap();
    // The key ID fields are two bits wide (cam.h:89-95).
    refused(&mut live, &group(4));
    assert!(!live.key_installed(0));
}

#[test]
fn leave_deletes_the_keys_first() {
    let log = RefCell::new(None);
    let (mut live, m) = associated(&|fw| *log.borrow_mut() = Some(fw.record()));
    let log = log.into_inner().unwrap();
    // SAFETY: as in `start_on`.
    unsafe { live.add_key(&pairwise()) }.unwrap();
    // SAFETY: as above.
    unsafe { live.add_key(&group(1)) }.unwrap();
    let from = log.borrow().len();
    // SAFETY: as above.
    unsafe { live.leave() }.unwrap();
    // The 17 H2Cs of seq 37-53, past the CH12 ring's 16 slots: as the
    // firmware read them, without their descriptors.
    let sent = log.borrow()[from..].to_vec();
    assert_eq!(sent.len(), 17);
    assert_eq!(live.h2c(), H2c::at(54));
    let at = |seq: u8| sent[usize::from(seq - 37)].clone();
    // What the radio left is deleted where mac80211 frees the AP station's
    // keys, before its ASSOC -> AUTH (sta_info.c:1577-1578), in Stela's
    // order, the group key first (KeyCam::deletion_order):
    // the group key's dctl (37) and CAM (38), the pairwise key's (39, 40),
    // each without SEC_UPD; then the beacon filter off (41) and the rest of
    // the leave.
    let no_gtk = CamKeys {
        keyids: 0x0040_0000,
        valid_and_ents: 0x0100_0001,
        ents_hi: 0,
    };
    let stale = CamKeys {
        keyids: 0x0040_0000,
        valid_and_ents: 0x0100_0000,
        ents_hi: 0,
    };
    assert_eq!(at(37), H2c::at(37).dctl_sec(0, &no_gtk).1[16..]);
    assert_eq!(at(38), cam_with(no_gtk, 38)[16..]);
    assert_eq!(at(39), H2c::at(39).dctl_sec(0, &stale).1[16..]);
    assert_eq!(at(40), cam_with(stale, 40)[16..]);
    assert_eq!(at(41), H2c::at(41).bcn_fltr(0, false, false).1[16..]);
    // The CAM without the link (44) and without the BSSID (45) carry the
    // key fields the deletes left.
    let unlinked = AddrCam {
        keys: stale,
        ..AddrCam::with_bssid(MAC, AP)
    };
    assert_eq!(at(44), H2c::at(44).addr_cam_with(&unlinked, 0, 0).1[16..]);
    let cleared = AddrCam {
        keys: stale,
        ..AddrCam::unassociated(MAC)
    };
    assert_eq!(at(45), H2c::at(45).addr_cam_with(&cleared, 0, 0).1[16..]);
    // The ring's slots hold the same bytes for the H2Cs still in them.
    assert_eq!(h2c_bytes(&m, 6, 0, 44)[16..], at(44)[..]);
    // Afterwards nothing is installed, and the CAM is a fresh vif's again,
    // as Linux's idle power-off and vif init leave it.
    assert!(!live.key_installed(0) && !live.key_installed(1));
    assert_eq!(live.cam_keys(), CamKeys::default());
    // The next association installs from entry 0 again, its CAM fresh.
    // SAFETY: as above.
    unsafe { live.set_bssid(AP) }.unwrap();
    let a = association(&live);
    // SAFETY: as above.
    unsafe { live.associate(&a) }.unwrap();
    let seq = 54 + 1 + 13;
    // SAFETY: as above.
    assert_eq!(unsafe { live.add_key(&pairwise()) }, Ok(0));
    assert_eq!(h2c_bytes(&m, 6, 0, seq + 2), cam_with(PTK_KEYS, seq + 2));
}

#[test]
fn key_calls_are_refused_while_a_scan_runs() {
    let (mut live, _m) = live_hearing(&|_| {});
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    let before = live.io().trace.len();
    // SAFETY: as above.
    assert_eq!(unsafe { live.add_key(&pairwise()) }, Err(SCAN_NOT_ENDED));
    // SAFETY: as above.
    assert_eq!(unsafe { live.remove_key(0) }, Err(SCAN_NOT_ENDED));
    assert_eq!(live.io().trace.len(), before);
}

#[test]
fn a_powered_off_chip_forgets_its_keys() {
    let (mut live, _m) = associated(&|_| {});
    // SAFETY: as in `start_on`.
    unsafe { live.add_key(&pairwise()) }.unwrap();
    live.io_mut().fail_reads_of(0x156CC);
    // SAFETY: as above.
    assert!(unsafe { live.tune(&g5(36)) }.is_err());
    assert!(!live.key_installed(0));
    assert_eq!(live.cam_keys(), CamKeys::default());
    // SAFETY: as above.
    assert_eq!(unsafe { live.add_key(&pairwise()) }, Err(POWERED_OFF));
    // SAFETY: as above.
    assert_eq!(unsafe { live.remove_key(0) }, Err(POWERED_OFF));
}

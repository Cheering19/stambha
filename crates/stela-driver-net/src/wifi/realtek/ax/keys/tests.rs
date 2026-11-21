use super::*;
use crate::wifi::realtek::ax::h2c::H2c;
use alloc::vec::Vec;

/// The plan's test addresses: the local MAC and the AP's BSSID.
const LOCAL: [u8; 6] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
const AP: [u8; 6] = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07];

/// The payload words of an address CAM H2C for `keys` on the station
/// associated with AID 1.
fn cam_words(keys: CamKeys) -> Vec<u32> {
    let cam = AddrCam {
        keys,
        ..AddrCam::connected(LOCAL, AP, 1)
    };
    let (_, p) = H2c::at(1).addr_cam_with(&cam, 0, 0);
    p[24..]
        .chunks(4)
        .map(|w| u32::from_le_bytes(w.try_into().unwrap()))
        .collect()
}

#[test]
fn cam_words_after_ptk_and_gtk() {
    // M3 spec S12.3: the pairwise key in slot 0 with key ID 0, then the
    // group key 1 in slot 2 (NORMAL mode, cam.c:166-188), each slot's sec
    // CAM entry the first free one (cam.c:102-119).
    let mut cam = KeyCam::default();
    assert_eq!(cam.allocate(), Some(0));
    let ptk = cam.attach(0, true, 0).unwrap();
    assert_eq!(
        ptk,
        CamKeys {
            keyids: 0,
            valid_and_ents: 0x0000_0001,
            ents_hi: 0,
        }
    );
    let w = cam_words(ptk);
    assert_eq!((w[9], w[10], w[11]), (0x0002_0001, 0x0000_0001, 0));
    assert_eq!(cam.allocate(), Some(1));
    let gtk = cam.attach(1, false, 1).unwrap();
    // SEC_ENT2_KEYID [23:22] = 1; valid map 0b101; SEC_ENT2 [31:24] = 1.
    assert_eq!(
        gtk,
        CamKeys {
            keyids: 0x0040_0000,
            valid_and_ents: 0x0100_0005,
            ents_hi: 0,
        }
    );
    let w = cam_words(gtk);
    assert_eq!(w[9], 0x0002_0001 + 0x0040_0000);
    assert_eq!((w[10], w[11]), (0x0100_0005, 0));
    assert_eq!(cam.keys(), gtk);
    assert!(cam.installed(0) && cam.installed(1) && !cam.installed(2));
}

#[test]
fn the_sec_cam_takes_the_first_free_of_128_entries() {
    // rtw89_cam_get_avail_sec_cam (cam.c:102-119), 8852C's scam_num 128.
    let mut cam = KeyCam::default();
    for want in 0..128u8 {
        assert_eq!(cam.allocate(), Some(want));
    }
    assert_eq!(cam.allocate(), None);
    cam.release(77);
    assert!(!cam.installed(77));
    assert_eq!(cam.allocate(), Some(77));
    assert_eq!(cam.allocate(), None);
    // Releasing an entry past the CAM changes nothing.
    cam.release(200);
    assert_eq!(cam.allocate(), None);
}

#[test]
fn normal_mode_puts_pairwise_keys_in_slots_0_1_and_group_keys_in_2_to_4() {
    // rtw89_cam_get_addr_cam_key_idx, RTW89_ADDR_CAM_SEC_NORMAL
    // (cam.c:166-188): pairwise the first free of 0-1, group the first
    // free of 2-4, else -EBUSY.
    let mut cam = KeyCam::default();
    assert_eq!(cam.free_slot(true), Some(0));
    assert_eq!(cam.free_slot(false), Some(2));
    cam.attach(10, true, 0).unwrap();
    cam.attach(11, true, 1).unwrap();
    assert_eq!(cam.free_slot(true), None);
    assert_eq!(cam.attach(12, true, 0), None);
    // A group key and its rekey under another ID sit side by side.
    cam.attach(20, false, 1).unwrap();
    let rekeyed = cam.attach(21, false, 2).unwrap();
    // Slot 3: SEC_ENT3_KEYID [25:24] = 2, SEC_ENT3 = 21 in w11 [7:0].
    assert_eq!(rekeyed.keyids, 0x0040_0000 | 0x0200_0000 | 0x0010_0000);
    assert_eq!(rekeyed.valid_and_ents, 0x140B_0A0F);
    assert_eq!(rekeyed.ents_hi, 21);
    cam.attach(22, false, 3).unwrap();
    assert_eq!(cam.free_slot(false), None);
    assert_eq!(cam.attach(23, false, 1), None);
    // A refused attach changed nothing.
    assert_eq!(cam.keys().valid_and_ents & 0xFF, 0x1F);
}

#[test]
fn detach_clears_only_the_valid_bit_as_linux_does() {
    // __rtw89_cam_detach_sec_cam (cam.c:215-246) clears the slot's bit of
    // sec_cam_map alone: the slot's key ID and entry stay as they were, and
    // the address CAM and dctl H2Cs carry them so.
    let mut cam = KeyCam::default();
    cam.allocate();
    cam.attach(0, true, 0).unwrap();
    cam.allocate();
    cam.attach(1, false, 1).unwrap();
    let without_gtk = cam.detach(1);
    assert_eq!(
        without_gtk,
        CamKeys {
            keyids: 0x0040_0000,
            valid_and_ents: 0x0100_0001,
            ents_hi: 0,
        }
    );
    // The sec CAM entry is the caller's to release.
    assert!(cam.installed(1));
    cam.release(1);
    let none = cam.detach(0);
    assert_eq!(
        none,
        CamKeys {
            keyids: 0x0040_0000,
            valid_and_ents: 0x0100_0000,
            ents_hi: 0,
        }
    );
    // Detaching an entry no slot names changes nothing.
    assert_eq!(cam.detach(9), none);
    // The next group key takes slot 2 again, overwriting its fields.
    cam.attach(1, false, 2).unwrap();
    assert_eq!(cam.keys().keyids, 0x0080_0000);
    assert_eq!(cam.keys().valid_and_ents, 0x0100_0004);
}

#[test]
fn keys_are_deleted_group_first_then_pairwise() {
    // Stela's order, which is ieee80211_free_sta_keys's for a station's own
    // keys (net/mac80211/key.c:1186-1215): the group keys by index, then
    // the pairwise keys by index.
    let mut cam = KeyCam::default();
    cam.attach(4, true, 0).unwrap();
    cam.attach(5, false, 2).unwrap();
    cam.attach(6, false, 1).unwrap();
    assert_eq!(cam.deletion_order(), [6, 5, 4]);
    cam.detach(5);
    assert_eq!(cam.deletion_order(), [6, 4]);
    assert!(KeyCam::default().deletion_order().is_empty());
}

#[test]
fn check_refuses_before_any_io() {
    let mut cam = KeyCam::default();
    assert_eq!(check(&cam, true, 0), Ok(()));
    assert_eq!(check(&cam, false, 3), Ok(()));
    // The key ID fields are two bits wide (cam.h:89-95).
    assert!(matches!(check(&cam, false, 4), Err(AxError::Mac(_))));
    // No slot left for another pairwise key.
    cam.attach(0, true, 0).unwrap();
    cam.attach(1, true, 1).unwrap();
    assert!(matches!(check(&cam, true, 0), Err(AxError::Mac(_))));
    assert_eq!(check(&cam, false, 1), Ok(()));
    // No sec CAM entry left.
    let mut full = KeyCam::default();
    while full.allocate().is_some() {}
    assert!(matches!(check(&full, false, 1), Err(AxError::Mac(_))));
}

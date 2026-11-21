// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
use super::*;
use crate::wifi::realtek::ax::chan::HwBand;
use crate::wifi::realtek::ax::receive::Tag;
use alloc::vec;

#[test]
fn entries_are_the_first_free_of_eight() {
    // rtw89_core_acquire_sta_ba_entry (core.c:5601-5645): the first free bit
    // of ba_cam_map, bacam_num 8.
    let mut cam = BaCam::default();
    for tid in 0..8 {
        assert_eq!(cam.acquire(tid), Some(tid));
    }
    // Full: a TID other than 0 and 5 gets none (-ENOSPC).
    assert_eq!(cam.acquire(8), None);
    assert_eq!(cam.release(8), None);
    // rtw89_core_release_sta_ba_entry frees the entry its TID held.
    assert_eq!(cam.release(3), Some(3));
    assert_eq!(cam.release(3), None);
    assert_eq!(cam.acquire(9), Some(3));
    assert_eq!(cam.tids(), vec![0, 1, 2, 4, 5, 6, 7, 9]);
}

#[test]
fn a_full_cam_gives_tid_0_or_5_an_entry_another_tid_holds() {
    // "allocate a static BA CAM to tid=0/5, so replace the existing one if
    // BA CAM is full": the first set bit whose TID is neither 0 nor 5.
    let mut cam = BaCam::default();
    for tid in 8..16 {
        cam.acquire(tid).unwrap();
    }
    assert_eq!(cam.acquire(0), Some(0));
    // TID 8 lost its entry: its release finds none (-ENOENT).
    assert_eq!(cam.release(8), None);
    assert_eq!(cam.acquire(5), Some(1));
    assert_eq!(cam.acquire(3), None);
    assert_eq!(cam.acquire(0), Some(2));
    // The stolen entries moved to the end of the station's list.
    assert_eq!(cam.tids(), vec![11, 12, 13, 14, 15, 0, 5, 0]);
    // Every entry held by 0 or 5: none to take.
    let mut full = BaCam::default();
    for tid in [0, 5, 0, 5, 0, 5, 0, 5] {
        full.acquire(tid).unwrap();
    }
    assert_eq!(full.acquire(0), None);
    assert_eq!(full.acquire(5), None);
}

#[test]
fn a_release_frees_the_oldest_entry_of_its_tid() {
    // list_for_each_entry_safe over the station's ba_cam_list, oldest first.
    let mut cam = BaCam::default();
    assert_eq!(cam.acquire(1), Some(0));
    assert_eq!(cam.acquire(2), Some(1));
    assert_eq!(cam.release(1), Some(0));
    // TID 2 again takes the free entry 0, newer in the list than entry 1.
    assert_eq!(cam.acquire(2), Some(0));
    assert_eq!(cam.release(2), Some(1));
    assert_eq!(cam.release(2), Some(0));
    assert!(cam.tids().is_empty());
}

/// QoS data of `tid` to the station, `sn` and `pn` in its CCMP header, as
/// the hardware hands it on decrypted: `HW_DEC`, `A1_MATCH`, MAC ID 0,
/// `RTW89_SEC_KEY_TYPE_CCMP128`.
fn frame(tid: u8, sn: u16, pn: u64) -> Received {
    let mut mpdu = vec![0x88, 0x42, 0, 0];
    mpdu.extend([0x02, 0x11, 0x22, 0x33, 0x44, 0x55]);
    mpdu.extend([0x02, 0x03, 0x04, 0x05, 0x06, 0x07]);
    mpdu.extend([0x02, 0x03, 0x04, 0x05, 0x06, 0x07]);
    mpdu.extend((sn << 4).to_le_bytes());
    mpdu.extend([tid, 0]);
    let [p0, p1, p2, p3, p4, p5, _, _] = pn.to_le_bytes();
    mpdu.extend([p0, p1, 0, 0x20, p2, p3, p4, p5]);
    mpdu.extend([0x55; 8]);
    Received {
        mpdu,
        tag: Tag {
            band: HwBand::G2,
            channel: 1,
        },
        signal: None,
        chains: [None; 2],
        fcs_error: false,
        decrypted: true,
        icv_error: false,
        data_rate: 0,
        gi_ltf: 0,
        bw: 0,
        ampdu: false,
        free_run: 0,
        hw_dec: true,
        a1_match: true,
        mac_id: 0,
        sec_type: SEC_KEY_TYPE_CCMP128,
    }
}

#[test]
fn the_pn_check_drops_a_later_pn_under_an_earlier_sn() {
    // rtw89_core_skb_pn_valid (core.c:3943-4006) on TID 0, which RX_START
    // started (rtw89_core_tid_rx_stats_ctrl, core.c:3906-3920).
    let mut stats = RxStats::default();
    stats.control(0, true);
    assert!(stats.valid(true, &frame(0, 10, 5)));
    // A later PN under an earlier SN is dropped.
    assert!(!stats.valid(true, &frame(0, 9, 6)));
    // An earlier PN is left to mac80211's replay check, and keeps the last.
    assert!(stats.valid(true, &frame(0, 11, 4)));
    assert!(!stats.valid(true, &frame(0, 9, 7)));
    assert!(stats.valid(true, &frame(0, 11, 7)));
    // ieee80211_sn_less is modulo 4096: past 4095 is later, not earlier.
    assert!(stats.valid(true, &frame(0, 0x80A, 8)));
    assert!(stats.valid(true, &frame(0, 0xFF0, 9)));
    assert!(stats.valid(true, &frame(0, 5, 10)));
    assert!(!stats.valid(true, &frame(0, 0xFFF, 11)));
    assert_eq!(stats.dropped(), 3);
    // Each TID on its own.
    stats.control(6, true);
    assert!(stats.valid(true, &frame(6, 1, 1)));
    assert!(stats.valid(true, &frame(0, 6, 12)));
}

#[test]
fn the_pn_check_passes_what_linux_does_not_check() {
    let mut stats = RxStats::default();
    // Not started: TID 0 has no agreement.
    assert!(stats.valid(true, &frame(0, 10, 5)));
    assert!(stats.valid(true, &frame(0, 9, 6)));
    stats.control(0, true);
    assert!(stats.valid(true, &frame(0, 10, 5)));
    let later = || frame(0, 9, 6);
    let pass = |stats: &mut RxStats, associated: bool, f: Received| stats.valid(associated, &f);
    // No association to MAC ID 0 (rtw89_assoc_link_rcu_dereference).
    assert!(pass(&mut stats, false, later()));
    assert!(pass(
        &mut stats,
        true,
        Received {
            mac_id: 1,
            ..later()
        }
    ));
    // Not hardware-decrypted, or not to the station's address.
    assert!(pass(
        &mut stats,
        true,
        Received {
            hw_dec: false,
            ..later()
        }
    ));
    assert!(pass(
        &mut stats,
        true,
        Received {
            a1_match: false,
            ..later()
        }
    ));
    // Not QoS data.
    let mut plain = later();
    plain.mpdu[0] = 0x08;
    assert!(pass(&mut stats, true, plain));
    // A cipher without a PN of this form: none, TKIP; CCMP-256 and the GCMPs
    // are checked.
    assert!(pass(
        &mut stats,
        true,
        Received {
            sec_type: 0,
            ..later()
        }
    ));
    assert!(pass(
        &mut stats,
        true,
        Received {
            sec_type: 3,
            ..later()
        }
    ));
    for sec_type in 7..=9 {
        assert!(!pass(
            &mut stats,
            true,
            Received {
                sec_type,
                ..later()
            }
        ));
    }
    // Too short for its CCMP header: passed on, for the station to refuse.
    let mut short = later();
    short.mpdu.truncate(30);
    assert!(pass(&mut stats, true, short));
    assert_eq!(stats.dropped(), 3);
    // RX_STOP: no longer checked.
    stats.control(0, false);
    assert!(pass(&mut stats, true, later()));
}

#[test]
fn a_restart_forgets_the_last_pn() {
    let mut stats = RxStats::default();
    stats.control(0, true);
    stats.control(1, true);
    assert!(stats.valid(true, &frame(0, 10, 5)));
    assert!(stats.valid(true, &frame(1, 10, 5)));
    // rtw89_core_tid_rx_stats_reset after a pairwise SET_KEY: every started
    // TID starts over, still started.
    stats.reset();
    assert!(stats.valid(true, &frame(0, 9, 6)));
    assert!(!stats.valid(true, &frame(0, 8, 7)));
    // RX_START again starts over too.
    stats.control(1, true);
    assert!(stats.valid(true, &frame(1, 9, 6)));
    // A TID that was never started stays unchecked after a reset.
    stats.reset();
    assert!(stats.valid(true, &frame(2, 10, 5)));
    assert!(stats.valid(true, &frame(2, 9, 6)));
    // A TID past 15 is no TID (IEEE80211_NUM_TIDS).
    stats.control(16, true);
    // The station gone: every TID unstarted, the drops still counted.
    stats.forget();
    assert!(stats.valid(true, &frame(0, 10, 5)));
    assert!(stats.valid(true, &frame(0, 9, 6)));
    assert_eq!(stats.dropped(), 1);
}

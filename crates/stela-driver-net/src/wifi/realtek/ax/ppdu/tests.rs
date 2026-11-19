// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
use super::*;
use alloc::{vec, vec::Vec};

/// An RX descriptor with every field zero (a short WIFI descriptor) but the
/// ones a test sets.
fn desc() -> RxDesc {
    RxDesc {
        pkt_type: 0,
        pkt_len: 0,
        long: false,
        mac_info_valid: false,
        ppdu_type: 0,
        ppdu_cnt: 0,
        data_rate: 0,
        gi_ltf: 0,
        bw: 0,
        free_run: 0,
        a1_match: false,
        sw_dec: false,
        hw_dec: false,
        ampdu: false,
        crc32_err: false,
        icv_err: false,
        frame_type: None,
    }
}

fn ppdu_desc() -> RxDesc {
    RxDesc {
        pkt_type: super::super::rx::RX_TYPE_PPDU_STAT,
        ..desc()
    }
}

fn words(ws: &[u32]) -> Vec<u8> {
    ws.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// A PHY status: the 8-byte header, then `ies` as given.
fn phy_status(w0: u32, w1: u32, ies: &[u8]) -> Vec<u8> {
    let mut b = words(&[w0, w1]);
    b.extend_from_slice(ies);
    b
}

/// IE01 (32 bytes, `phy.c:9045`) with `chan_idx` in w0 [23:16].
fn ie01(chan_idx: u8) -> Vec<u8> {
    let mut b = vec![0u8; 32];
    b[..4].copy_from_slice(&(u32::from(chan_idx) << 16 | 1).to_le_bytes());
    b
}

/// Header w0: VALID, LEN 5 (40 bytes), IE map 12; w1: RSSI A 0x70, B 0x60.
const HDR_W0: u32 = 0x8000_058C;
const HDR_W1: u32 = 0x0000_6070;

fn good_status() -> Vec<u8> {
    phy_status(HDR_W0, HDR_W1, &ie01(6))
}

const GOOD: PhyPpdu = PhyPpdu {
    header: true,
    valid: true,
    rssi: [0x70, 0x60],
    rssi_avg: 0x80,
    chan_idx: 6,
};

/// MAC info w0 (`usr_num` [3:0], `RX_CNT_VLD` b29) and w1 (`PLCP_LEN`
/// [23:16]), then `pad` zero bytes before `status`.
fn with_mac_info(w0: u32, w1: u32, pad: usize, status: &[u8]) -> Vec<u8> {
    let mut b = words(&[w0, w1]);
    b.resize(8 + pad, 0);
    b.extend_from_slice(status);
    b
}

#[test]
fn phy_header_rssi_without_mac_info() {
    let b = good_status();
    assert_eq!(b.len(), 40);
    assert_eq!(parse(&ppdu_desc(), &b), GOOD);
}

#[test]
fn mac_info_offsets_the_phy_status() {
    // usr_num 1 (4 bytes, plus 4 to align to 8), RX_CNT_VLD (96 bytes),
    // PLCP_LEN 2 (16 bytes): the PHY status starts at byte 128.
    let b = with_mac_info(1 | 1 << 29, 2 << 16, 120, &good_status());
    assert_eq!(&b[128..136], &words(&[HDR_W0, HDR_W1])[..]);
    let d = RxDesc {
        mac_info_valid: true,
        ..ppdu_desc()
    };
    assert_eq!(parse(&d, &b), GOOD);
    // Without MAC_INFO_VLD the same bytes are not a PHY status.
    assert!(!parse(&ppdu_desc(), &b).header);
}

#[test]
fn mac_info_even_users_and_no_rx_count_take_no_padding() {
    // usr_num 2 (8 bytes, already aligned), no RX count, no PLCP: byte 16.
    let b = with_mac_info(2, 0, 8, &good_status());
    let d = RxDesc {
        mac_info_valid: true,
        ..ppdu_desc()
    };
    assert_eq!(parse(&d, &b), GOOD);
}

#[test]
fn length_mismatch_leaves_rssi_zero() {
    // LEN 4 is 32 bytes, but 40 remain.
    let b = phy_status(0x8000_048C, HDR_W1, &ie01(6));
    assert_eq!(b.len(), 40);
    assert_eq!(parse(&ppdu_desc(), &b), PhyPpdu::default());
    // VALID clear, the right LEN: refused alike.
    let b = phy_status(HDR_W0 & !(1 << 7), HDR_W1, &ie01(6));
    assert_eq!(parse(&ppdu_desc(), &b), PhyPpdu::default());
}

#[test]
fn ie_walk_past_end_is_invalid_but_keeps_rssi() {
    // IE type 11 is 176 bytes, in a 40-byte status.
    let mut ies = vec![0u8; 32];
    ies[0] = 11;
    let b = phy_status(HDR_W0, HDR_W1, &ies);
    let p = parse(&ppdu_desc(), &b);
    assert_eq!(
        p,
        PhyPpdu {
            valid: false,
            chan_idx: 0,
            ..GOOD
        }
    );
}

#[test]
fn ie01_seen_before_a_failed_walk_keeps_its_index_but_not_valid() {
    // Linux keeps `phy_ppdu->chan_idx` from IE01 when a later IE fails;
    // only `valid` gates its use (`rtw8852c.c:2994-2995`).
    let mut ies = ie01(6);
    ies.extend_from_slice(&[11, 0, 0, 0, 0, 0, 0, 0]);
    let b = phy_status(0x8000_068C, HDR_W1, &ies);
    let p = parse(&ppdu_desc(), &b);
    assert_eq!((p.header, p.valid, p.chan_idx), (true, false, 6));
}

#[test]
fn cck_page_below_11_skips_the_walk() {
    let b = phy_status(0x8000_058A, HDR_W1, &ie01(6));
    let p = parse(&ppdu_desc(), &b);
    assert_eq!(
        p,
        PhyPpdu {
            valid: false,
            chan_idx: 0,
            ..GOOD
        }
    );
    // IE map 11 (CCK_PKT) walks.
    let b = phy_status(0x8000_058B, HDR_W1, &ie01(6));
    assert_eq!(parse(&ppdu_desc(), &b), GOOD);
}

#[test]
fn usr_num_above_8_is_refused() {
    let d = RxDesc {
        mac_info_valid: true,
        ..ppdu_desc()
    };
    // usr_num 9: 36 bytes plus 4 of alignment would put the status at 48.
    let b = with_mac_info(9, 0, 40, &good_status());
    assert!(!parse(&d, &b).header);
    assert_eq!(parse(&d, &b), PhyPpdu::default());
    // usr_num 8 is accepted: 32 bytes, the status at 40.
    let b = with_mac_info(8, 0, 32, &good_status());
    assert_eq!(parse(&d, &b), GOOD);
}

#[test]
fn mac_info_past_the_end_stops() {
    let d = RxDesc {
        mac_info_valid: true,
        ..ppdu_desc()
    };
    // PLCP_LEN 0xFF: the u8 `0xFF << 3` is 0xF8, the status 256 bytes in.
    let b = with_mac_info(0, 0xFF << 16, 0, &good_status());
    assert_eq!(b.len(), 48);
    assert_eq!(parse(&d, &b), PhyPpdu::default());
    // Shorter than the MAC info itself.
    assert_eq!(parse(&d, &[0u8; 7]), PhyPpdu::default());
    // The status exactly at the end: no header to read.
    let b = with_mac_info(0, 0, 0, &[]);
    assert_eq!(parse(&d, &b), PhyPpdu::default());
}

#[test]
fn plcp_offset_truncates_to_a_byte_as_linux_u8_plcp_size() {
    // `u8 plcp_size = PLCP_LEN << 3` (`core.c:1950`, `:1968`): PLCP_LEN
    // 0x21 gives (0x108 & 0xFF) = 8, the same as PLCP_LEN 1.
    let d = RxDesc {
        mac_info_valid: true,
        ..ppdu_desc()
    };
    for plcp_len in [1u32, 0x21, 0x41, 0xE1] {
        let b = with_mac_info(0, plcp_len << 16, 8, &good_status());
        assert_eq!(parse(&d, &b), GOOD, "PLCP_LEN {plcp_len:#x}");
    }
    // PLCP_LEN 0x20 truncates to 0: the status right after the MAC info.
    let b = with_mac_info(0, 0x20 << 16, 0, &good_status());
    assert_eq!(parse(&d, &b), GOOD);
    // Without truncation 0x21 would be 264 bytes in, past this payload.
    let b = with_mac_info(0, 0x21 << 16, 8, &good_status());
    assert!(b.len() < 8 + 0x21 * 8);
}

#[test]
fn hdr_2_en_moves_the_walk_past_a_second_header() {
    // HDR_2_EN (b5): the IEs start at 16 (`core.c:2324-2325`). LEN 6.
    let mut ies = vec![0u8; 8];
    ies.extend_from_slice(&ie01(6));
    let b = phy_status(0x8000_06AC, HDR_W1, &ies);
    assert_eq!(parse(&ppdu_desc(), &b), GOOD);
    // Read without the second header, its zero word is a 16-byte IE00 and
    // the walk runs past the end.
    let b = phy_status(0x8000_068C, HDR_W1, &ies);
    let p = parse(&ppdu_desc(), &b);
    assert_eq!((p.header, p.valid), (true, false));
}

#[test]
fn variable_length_ies_use_their_header_length() {
    // Type 8 is VAR_LEN: LEN [11:5] = 2 gives 16 bytes. Then IE01. LEN 7.
    let mut ies = words(&[8 | 2 << 5, 0, 0, 0]);
    ies.extend_from_slice(&ie01(0x22));
    let b = phy_status(0x8000_078C, HDR_W1, &ies);
    let p = parse(&ppdu_desc(), &b);
    assert_eq!((p.valid, p.chan_idx), (true, 0x22));
}

#[test]
fn zero_length_ies_fail_the_walk() {
    // Type 8 with LEN 0, and type 23 whose table length is 0.
    for w0 in [8u32, 23] {
        let ies = words(&[w0, 0, 0, 0, 0, 0, 0, 0]);
        let b = phy_status(HDR_W0, HDR_W1, &ies);
        let p = parse(&ppdu_desc(), &b);
        assert_eq!((p.header, p.valid), (true, false), "ie {w0}");
    }
}

#[test]
fn ie_length_table_is_linux_ax() {
    assert_eq!(
        IE_LEN,
        [
            16, 32, 24, 24, 8, 8, 8, 8, 0xFF, 8, 0xFF, 176, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 16,
            24, 0xFF, 0xFF, 0xFF, 0, 24, 24, 24, 24, 32, 32, 32, 32
        ]
    );
}

#[test]
fn ie09_trailing_zeros_end_the_walk_only_when_to_self() {
    // IE09 (8 bytes), then 8 zero bytes: Linux stops at the zeros once it
    // has kept an IE09, which it does only for an A1-matched status
    // (`core.c:2234-2236`, `:2334`). LEN 3.
    let ies = words(&[9, 0, 0, 0]);
    let b = phy_status(0x8000_038C, HDR_W1, &ies);
    let to_self = RxDesc {
        a1_match: true,
        ..ppdu_desc()
    };
    assert!(parse(&to_self, &b).valid);
    // Not to self: the zero word is a 16-byte IE00, past the end.
    let p = parse(&ppdu_desc(), &b);
    assert_eq!((p.header, p.valid), (true, false));
    // The last 8 bytes must have a zero first word.
    let ies = words(&[9, 0, 0x100, 0]);
    let b = phy_status(0x8000_038C, HDR_W1, &ies);
    let p = parse(&to_self, &b);
    assert_eq!((p.header, p.valid), (true, false));
}

#[test]
fn every_truncation_and_every_byte_pattern_never_panics() {
    let mac = with_mac_info(1 | 1 << 29, 2 << 16, 120, &good_status());
    let plain = good_status();
    let ones = vec![0xFFu8; 300];
    for d in [
        ppdu_desc(),
        RxDesc {
            mac_info_valid: true,
            a1_match: true,
            ..ppdu_desc()
        },
    ] {
        for b in [&mac, &plain, &ones] {
            for n in 0..=b.len() {
                let _ = parse(&d, &b[..n]);
                let _ = parse(&d, &b[n..]);
            }
        }
        // Every IE type with every LEN at the head of a walk.
        for ie in 0u32..32 {
            for len in [0u32, 1, 2, 3, 4, 5, 0x7F] {
                let ies = words(&[ie | len << 5, 0, 0, 0, 0, 0, 0, 0]);
                let _ = parse(&d, &phy_status(HDR_W0, HDR_W1, &ies));
            }
        }
    }
}

#[test]
fn rssi_dbm_matches_linux() {
    // RTW89_RSSI_RAW_TO_DBM: ((s8)(raw >> 1)) - 110 (`core.h:61`).
    assert_eq!(rssi_dbm(0), -110);
    assert_eq!(rssi_dbm(1), -110);
    assert_eq!(rssi_dbm(0x70), -54);
    assert_eq!(rssi_dbm(0xDC), 0);
    assert_eq!(rssi_dbm(0xFF), 17);
    assert_eq!(MAX_PPDU_CNT, 8);
}

fn rated(rate: u16, gi_ltf: u8, bw: u8, ppdu_cnt: u8) -> RxDesc {
    RxDesc {
        data_rate: rate,
        gi_ltf,
        bw,
        ppdu_cnt,
        ..desc()
    }
}

#[test]
fn match_rule() {
    let frame = frame_key(&rated(0x0B, 0, 0, 5));
    assert_eq!(
        frame,
        RateKey {
            legacy: true,
            rate_idx: 0x0B,
            gi: GI_3_2,
            bw: BW_20,
        }
    );
    let ppdu = |rate, gi, bw, cnt| RxDesc {
        pkt_type: super::super::rx::RX_TYPE_PPDU_STAT,
        ..rated(rate, gi, bw, cnt)
    };
    assert!(matches(5, &ppdu(0x0B, 0, 0, 5), &frame));
    // Differing rate, bandwidth or PPDU count.
    assert!(!matches(5, &ppdu(0x0A, 0, 0, 5), &frame));
    assert!(!matches(5, &ppdu(0x0B, 0, 1, 5), &frame));
    assert!(!matches(5, &ppdu(0x0B, 0, 0, 4), &frame));
    assert!(!matches(4, &ppdu(0x0B, 0, 0, 5), &frame));
    // Differing GI.
    assert!(!matches(5, &ppdu(0x0B, 1, 0, 5), &frame));
    // An invalid GI-LTF is 3.2 us on the frame side and 0xFF on the PPDU
    // side, so it never matches, not even itself.
    let odd = frame_key(&rated(0x0B, 6, 0, 5));
    assert_eq!(odd.gi, GI_3_2);
    assert!(!matches(5, &ppdu(0x0B, 6, 0, 5), &odd));
    assert!(!matches(
        5,
        &ppdu(0x0B, 7, 0, 5),
        &frame_key(&rated(0x0B, 7, 0, 5))
    ));
    // ... while an invalid PPDU GI against a 3.2 us frame fails too.
    assert!(!matches(5, &ppdu(0x0B, 6, 0, 5), &frame));
    // Bandwidth codes past 160 MHz all read as 20 MHz.
    let wide = frame_key(&rated(0x0B, 0, 5, 5));
    assert_eq!(wide.bw, BW_20);
    assert!(matches(5, &ppdu(0x0B, 0, 0, 5), &wide));
    // Linux compares the raw index only, not the mode: OFDM 54 (0x0B) and
    // HT MCS 11 (0x8B) match.
    assert!(matches(5, &ppdu(0x8B, 0, 0, 5), &frame));
}

#[test]
fn rate_gi_and_bw_keys_follow_linux() {
    // HT: MCS [4:0]; VHT/HE: MCS [3:0].
    let ht = frame_key(&rated(0x80 | 0x13, 0, 0, 0));
    assert_eq!((ht.legacy, ht.rate_idx), (false, 0x13));
    let vht = frame_key(&rated(0x100 | 0x15, 0, 0, 0));
    assert_eq!((vht.legacy, vht.rate_idx), (false, 0x05));
    let he = frame_key(&rated(0x180 | 0x1B, 0, 0, 0));
    assert_eq!((he.legacy, he.rate_idx), (false, 0x0B));
    let legacy = frame_key(&rated(0x7F, 0, 0, 0));
    assert_eq!((legacy.legacy, legacy.rate_idx), (true, 0x0F));
    // GI-LTF: 1, 3, 5 -> 0.8; 2, 4 -> 1.6; 0 -> 3.2; 6, 7 -> 3.2 or 0xFF.
    let gis: [(u8, u8, u8); 8] = [
        (0, GI_3_2, GI_3_2),
        (1, GI_0_8, GI_0_8),
        (2, GI_1_6, GI_1_6),
        (3, GI_0_8, GI_0_8),
        (4, GI_1_6, GI_1_6),
        (5, GI_0_8, GI_0_8),
        (6, GI_3_2, GI_NONE),
        (7, GI_3_2, GI_NONE),
    ];
    for (gi_ltf, frame_gi, ppdu_gi) in gis {
        let d = rated(0, gi_ltf, 0, 0);
        assert_eq!((frame_key(&d).gi, ppdu_key(&d).gi), (frame_gi, ppdu_gi));
    }
    // BW: 1 -> 40, 2 -> 80, 3 -> 160, the rest 20 (`core.h:8456-8466`).
    let bws = [BW_20, BW_40, BW_80, BW_160, BW_20, BW_20, BW_20, BW_20];
    for (hw, want) in bws.into_iter().enumerate() {
        let d = rated(0, 0, hw as u8, 0);
        assert_eq!((frame_key(&d).bw, ppdu_key(&d).bw), (want, want));
    }
    assert_eq!((GI_0_8, GI_1_6, GI_3_2, GI_NONE), (0, 1, 2, 0xFF));
    assert_eq!((BW_20, BW_40, BW_80, BW_160), (0, 3, 4, 5));
}

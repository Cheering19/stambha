// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
extern crate std;
use super::*;
use alloc::{vec, vec::Vec};

// The WD body and info of M3 spec S5.3, dword by dword, and the CCMP strip
// of S5.4.

const LOCAL: [u8; 6] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
const AP: [u8; 6] = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
const BROADCAST: [u8; 6] = [0xFF; 6];
const MULTICAST: [u8; 6] = [0x01, 0x00, 0x5E, 0x00, 0x00, 0x01];

/// Frame control: Open System authentication, deauthentication, an ACK.
const AUTH: u16 = 0x00B0;
const DEAUTH: u16 = 0x00C0;
const ACK: u16 = 0x00D4;
/// Frame control of a frame to the AP (ToDS): data, QoS data, null data,
/// QoS null, each with the Protected bit (b14) where named.
const DATA: u16 = 0x0108;
const DATA_PROTECTED: u16 = 0x4108;
const QOS: u16 = 0x0188;
const QOS_PROTECTED: u16 = 0x4188;
const NULL: u16 = 0x0148;
const QOS_NULL: u16 = 0x01C8;

/// RFC 1042 LLC/SNAP for EAPOL, then the start of an EAPOL-Key frame.
const EAPOL_BODY: [u8; 12] = [
    0xAA, 0xAA, 0x03, 0, 0, 0, 0x88, 0x8E, 0x02, 0x03, 0x00, 0x5F,
];
/// Plaintext a frame carries after its CCMP header.
const PLAINTEXT: [u8; 10] = [0xAA, 0xAA, 0x03, 0, 0, 0, 0x08, 0x00, 0x45, 0x00];
/// The PN of S5.3's table, and its CCMP header for key ID 0
/// (`[pn0, pn1, 0, 0x20 | keyid << 6, pn2, pn3, pn4, pn5]`).
const PN: u64 = 0x0605_0403_0201;
const PN_HEADER: [u8; 8] = [0x01, 0x02, 0x00, 0x20, 0x03, 0x04, 0x05, 0x06];

const NO_CAPS: RateCaps = RateCaps {
    stbc: false,
    ldpc: false,
};

/// A 24-byte header with control `fc`, A1 `a1`, A2 the station, A3 the
/// AP and sequence number `seq`, then `body`.
fn frame(fc: u16, a1: [u8; 6], seq: u16, body: &[u8]) -> Vec<u8> {
    let mut f = vec![0; 24];
    f[..2].copy_from_slice(&fc.to_le_bytes());
    f[4..10].copy_from_slice(&a1);
    f[10..16].copy_from_slice(&LOCAL);
    f[16..22].copy_from_slice(&AP);
    f[22..24].copy_from_slice(&(seq << 4).to_le_bytes());
    f.extend_from_slice(body);
    f
}

/// Open System authentication, transaction 1, status 0: 30 bytes.
fn auth(seq: u16) -> Vec<u8> {
    frame(AUTH, AP, seq, &[0, 0, 1, 0, 0, 0])
}

/// A QoS frame to the AP: the header, its QoS control for `tid`, `body`.
fn qos(fc: u16, tid: u8, seq: u16, body: &[u8]) -> Vec<u8> {
    let mut f = frame(fc, AP, seq, &[tid, 0]);
    f.extend_from_slice(body);
    f
}

/// The head's 14 dwords: body 0-7, then info 0-5.
fn words(head: &[u8; WD_HEAD_BYTES]) -> [u32; 14] {
    core::array::from_fn(|i| {
        u32::from_le_bytes([
            head[4 * i],
            head[4 * i + 1],
            head[4 * i + 2],
            head[4 * i + 3],
        ])
    })
}

fn sec(keyid: u8, cam: u8, pn: u64) -> Option<TxSec> {
    Some(TxSec { keyid, cam, pn })
}

/// Index of info dword `n` in [`words`].
const fn info(n: usize) -> usize {
    8 + n
}

#[test]
fn auth_on_2g_matches_linux() {
    let f = auth(5);
    assert_eq!(f.len(), 30);
    let (ch, head) = management(&f, HwBand::G2, 0x00F).unwrap();
    assert_eq!(ch, TxChannel::Ch8);
    assert_eq!(
        words(&head),
        [
            0x0048_0080, // B0: WD_INFO_EN, CHANNEL_DMA 8, WD_PAGE
            0x0400_0000, // B1: ADDR_INFO_NUM 1
            0x0024_001E, // B2: QSEL B0_MGMT 0x12, TXPKT_SIZE 30
            5,           // B3: SW_SEQ
            0,
            0,
            0,
            0x8000_0000, // B7: USE_RATE_V1, DATA_RATE CCK1
            0x0000_0400, // I0: DISDATAFB
            0,
            0,
            0,
            0x8800_0000, // I4: HW_RTS_EN, RTS_EN
            0,
        ]
    );
}

#[test]
fn management_rate_takes_the_lowest_basic() {
    assert_eq!(management_rate(HwBand::G2, 0x150), 4);
    assert_eq!(management_rate(HwBand::G5, 0x15), 4);
    assert_eq!(management_rate(HwBand::G2, 0), 0);
    assert_eq!(management_rate(HwBand::G5, 0), 4);
    // 5 GHz bitmaps are OFDM only: bit 4 is OFDM24 (0x8).
    assert_eq!(management_rate(HwBand::G5, 0x150), 8);
    // The rate reaches B7's DATA_RATE [24:16] with USE_RATE_V1.
    let (_, head) = management(&auth(5), HwBand::G5, 0x150).unwrap();
    assert_eq!(words(&head)[7], 0x8008_0000);
    let (_, head) = management(&auth(5), HwBand::G2, 0x150).unwrap();
    assert_eq!(words(&head)[7], 0x8004_0000);
}

#[test]
fn broadcast_management_has_no_rts() {
    for a1 in [BROADCAST, MULTICAST] {
        let (ch, head) = management(&frame(DEAUTH, a1, 9, &[3, 0]), HwBand::G2, 0).unwrap();
        assert_eq!(ch, TxChannel::Ch8);
        assert_eq!(words(&head)[info(4)], 0x8000_0000);
    }
    let (_, head) = management(&frame(DEAUTH, AP, 9, &[3, 0]), HwBand::G2, 0).unwrap();
    assert_eq!(words(&head)[info(4)], 0x8800_0000);
}

#[test]
fn eapol_on_vo_sets_bk_and_tid_indicate() {
    let f = qos(QOS, 7, 0x123, &EAPOL_BODY);
    let len = f.len() as u32;
    let (ch, head) = data(&f, 7, true, HwBand::G2, 0xFFF, NO_CAPS, None).unwrap();
    assert_eq!(ch, TxChannel::Ach3);
    assert_eq!(
        words(&head),
        [
            0x0043_6880,       // B0: WD_INFO_EN, ACH3, HDR_LLC_LEN 13, WD_PAGE
            0x0400_0000,       // B1: ADDR_INFO_NUM 1
            0x0086_0000 | len, // B2: TID_INDICATE, QSEL VO_0
            0x2000 | 0x123,    // B3: BK, SW_SEQ
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0x8800_0000,
            0,
        ]
    );
    // Not EAPOL: the same frame without BK.
    let (_, head) = data(&f, 7, false, HwBand::G2, 0xFFF, NO_CAPS, None).unwrap();
    assert_eq!(words(&head)[3], 0x123);
}

#[test]
fn ccmp_non_qos_and_qos_match_linux() {
    // Non-QoS, key ID 0, CAM 0: the frame as the strip leaves it.
    let f = frame(DATA_PROTECTED, AP, 0x456, &PLAINTEXT);
    let len = f.len() as u32;
    let (ch, head) = data(&f, 0, false, HwBand::G2, 0xFFF, NO_CAPS, sec(0, 0, PN)).unwrap();
    assert_eq!(ch, TxChannel::Ach0);
    assert_eq!(
        words(&head),
        [
            0x0140_6080, // B0: WP_OFFSET 1, WD_INFO_EN, ACH0, HDR_LLC_LEN 12
            0x0400_0006, // B1: ADDR_INFO_NUM 1, SEC_TYPE CCMP128
            len,         // B2: QSEL BE_0
            0x456,
            0x0201_0000, // B4: IV_L1 PN1, IV_L0 PN0
            0x0605_0403, // B5: PN5, PN4, PN3, PN2
            0,
            0,
            0,
            0,
            0x0000_0100, // I2: FORCE_KEY_EN, SEC_CAM_IDX 0
            0,
            0x8800_0000,
            0,
        ]
    );
    // QoS, key ID 2, CAM 5: the header is 26 bytes.
    let f = qos(QOS_PROTECTED, 0, 0x456, &PLAINTEXT);
    let len = f.len() as u32;
    let (ch, head) = data(&f, 0, false, HwBand::G2, 0xFFF, NO_CAPS, sec(2, 5, PN)).unwrap();
    assert_eq!(ch, TxChannel::Ach0);
    let w = words(&head);
    assert_eq!(w[0], 0x0140_6880);
    assert_eq!(w[1], 0x0400_0026);
    assert_eq!(w[2], len);
    assert_eq!(w[4..6], [0x0201_0000, 0x0605_0403]);
    assert_eq!(w[info(2)], 0x0000_0105);
}

#[test]
fn data_lowest_rate_and_caps() {
    assert_eq!(lowest_data_rate(HwBand::G2, 0xFFF), 0);
    assert_eq!(lowest_data_rate(HwBand::G2, 0xFF0), 4);
    assert_eq!(lowest_data_rate(HwBand::G2, 0), 0);
    assert_eq!(lowest_data_rate(HwBand::G5, 0xFF), 4);
    assert_eq!(lowest_data_rate(HwBand::G5, 0xF0), 8);
    assert_eq!(lowest_data_rate(HwBand::G5, 0), 4);
    let f = frame(DATA, AP, 1, &PLAINTEXT);
    let caps = RateCaps {
        stbc: true,
        ldpc: true,
    };
    let (_, head) = data(&f, 0, false, HwBand::G2, 0xFFF, caps, None).unwrap();
    assert_eq!(words(&head)[info(0)], 0x1800);
    assert_eq!(words(&head)[info(1)], 0);
    let caps = RateCaps {
        stbc: true,
        ldpc: false,
    };
    // 5 GHz, lowest supported OFDM12 (bit 2): 0x4 + 2.
    let (_, head) = data(&f, 0, false, HwBand::G5, 0xFC, caps, None).unwrap();
    assert_eq!(words(&head)[info(0)], 0x1000);
    assert_eq!(words(&head)[info(1)], 0x0006_0000);
    // Data frames leave the rate to the firmware's RA: B7 stays 0.
    assert_eq!(words(&head)[7], 0);
}

#[test]
fn strip_removes_the_header_and_keeps_protected() {
    let mut header = frame(DATA_PROTECTED, AP, 0x456, &PN_HEADER);
    header.extend_from_slice(&PLAINTEXT);
    let mut mpdu = header.clone();
    strip_ccmp(&mut mpdu, PN, 0).unwrap();
    assert_eq!(mpdu, frame(DATA_PROTECTED, AP, 0x456, &PLAINTEXT));
    assert_eq!(u16::from_le_bytes([mpdu[0], mpdu[1]]) & 0x4000, 0x4000);
    // L is the length after the removal.
    let (_, head) = data(&mpdu, 0, false, HwBand::G2, 0xFFF, NO_CAPS, sec(0, 0, PN)).unwrap();
    assert_eq!(words(&head)[2], (header.len() - 8) as u32);

    // A QoS frame's header is 26 bytes; key ID 2 sets b7:6 of byte 3.
    let mut ccmp = PN_HEADER;
    ccmp[3] = 0xA0;
    let mut mpdu = qos(QOS_PROTECTED, 6, 0x456, &ccmp);
    mpdu.extend_from_slice(&PLAINTEXT);
    strip_ccmp(&mut mpdu, PN, 2).unwrap();
    assert_eq!(mpdu, qos(QOS_PROTECTED, 6, 0x456, &PLAINTEXT));
}

#[test]
fn strip_refuses_a_header_that_is_not_the_reservation() {
    const REFUSED: Result<(), AxError> =
        Err(AxError::Tx("CCMP header does not match its reservation"));
    let mut good = frame(DATA_PROTECTED, AP, 0x456, &PN_HEADER);
    good.extend_from_slice(&PLAINTEXT);
    // Each of the eight bytes one off.
    for i in 24..32 {
        let mut mpdu = good.clone();
        mpdu[i] = mpdu[i].wrapping_add(1);
        let before = mpdu.clone();
        assert_eq!(strip_ccmp(&mut mpdu, PN, 0), REFUSED, "byte {i}");
        assert_eq!(mpdu, before, "byte {i}");
    }
    // The header one byte late.
    let mut mpdu = good.clone();
    mpdu.insert(24, 0);
    let before = mpdu.clone();
    assert_eq!(strip_ccmp(&mut mpdu, PN, 0), REFUSED);
    assert_eq!(mpdu, before);
    // Another reservation: the next PN, the previous one, another key ID.
    for (pn, keyid) in [(PN + 1, 0), (PN - 1, 0), (PN, 1)] {
        let mut mpdu = good.clone();
        assert_eq!(strip_ccmp(&mut mpdu, pn, keyid), REFUSED);
        assert_eq!(mpdu, good);
    }
    // A QoS frame whose header sits at 24, where the QoS control is.
    let mut mpdu = frame(QOS_PROTECTED, AP, 0x456, &PN_HEADER);
    mpdu.extend_from_slice(&[0, 0]);
    mpdu.extend_from_slice(&PLAINTEXT);
    let before = mpdu.clone();
    assert_eq!(strip_ccmp(&mut mpdu, PN, 0), REFUSED);
    assert_eq!(mpdu, before);
}

#[test]
fn strip_refuses_short_frames_and_unusable_reservations() {
    let mut good = frame(DATA_PROTECTED, AP, 0x456, &PN_HEADER);
    good.extend_from_slice(&PLAINTEXT);
    // Seven bytes of header after the MAC header.
    let mut mpdu = good[..31].to_vec();
    assert_eq!(
        strip_ccmp(&mut mpdu, PN, 0),
        Err(AxError::Tx("frame shorter than its CCMP header"))
    );
    assert_eq!(mpdu, good[..31]);
    // Shorter than a QoS header.
    let mut mpdu = qos(QOS_PROTECTED, 0, 1, &[])[..25].to_vec();
    let before = mpdu.clone();
    assert_eq!(
        strip_ccmp(&mut mpdu, PN, 0),
        Err(AxError::Tx("frame shorter than its 802.11 header"))
    );
    assert_eq!(mpdu, before);
    for (pn, keyid, error) in [
        (PN, 4, "a CCMP key ID past 3"),
        (0, 0, "a CCMP packet number of 0 or past 48 bits"),
        (1 << 48, 0, "a CCMP packet number of 0 or past 48 bits"),
    ] {
        let mut mpdu = good.clone();
        assert_eq!(strip_ccmp(&mut mpdu, pn, keyid), Err(AxError::Tx(error)));
        assert_eq!(mpdu, good);
    }
    // Only a data frame carries a host-reserved CCMP header.
    let mut mpdu = frame(0x40C0, AP, 1, &PN_HEADER);
    let before = mpdu.clone();
    assert_eq!(
        strip_ccmp(&mut mpdu, PN, 0),
        Err(AxError::Tx("not a data frame"))
    );
    assert_eq!(mpdu, before);
}

#[test]
fn tid_tables_match_linux() {
    // rtw89_core_get_qsel (txrx.h:833-852): BE 0, BK 1, VI 2, VO 3.
    let qsel = [0, 1, 1, 0, 2, 2, 3, 3];
    // rtw89_core_get_tid_indicate (txrx.h:873-890): TIDs 2, 3, 5 and 7.
    let indicate = [false, false, true, true, false, true, false, true];
    let channel = [
        TxChannel::Ach0,
        TxChannel::Ach1,
        TxChannel::Ach1,
        TxChannel::Ach0,
        TxChannel::Ach2,
        TxChannel::Ach2,
        TxChannel::Ach3,
        TxChannel::Ach3,
    ];
    for tid in 0..8u8 {
        let t = usize::from(tid);
        assert_eq!(qsel_of_tid(tid), qsel[t], "tid {tid}");
        assert_eq!(tid_indicate(tid), indicate[t], "tid {tid}");
        let f = qos(QOS, tid, 1, &PLAINTEXT);
        let (ch, head) = data(&f, tid, false, HwBand::G2, 0xFFF, NO_CAPS, None).unwrap();
        assert_eq!(ch, channel[t], "tid {tid}");
        let w = words(&head);
        assert_eq!((w[0] >> 16) & 0xF, ch.ring() as u32, "tid {tid}");
        assert_eq!((w[2] >> 17) & 0x3F, u32::from(qsel[t]), "tid {tid}");
        assert_eq!((w[2] >> 23) & 1 == 1, indicate[t], "tid {tid}");
    }
    // Past 7, Linux falls back to BE and no indication.
    assert_eq!(qsel_of_tid(8), 0);
    assert!(!tid_indicate(8));
    // `skb->priority & IEEE80211_QOS_CTL_TAG1D_MASK`: TID 15 is TID 7.
    let f = qos(QOS, 7, 1, &PLAINTEXT);
    let (ch, head) = data(&f, 15, false, HwBand::G2, 0xFFF, NO_CAPS, None).unwrap();
    assert_eq!(ch, TxChannel::Ach3);
    assert_eq!(words(&head)[2] & 0x00FE_0000, 0x0086_0000);
}

#[test]
fn every_head_sets_wd_info_en() {
    // Txq::submit refuses a head without WD_INFO_EN (body dword 0 b22).
    let heads = [
        management(&auth(1), HwBand::G2, 0).unwrap(),
        management(&frame(DEAUTH, BROADCAST, 1, &[3, 0]), HwBand::G5, 0).unwrap(),
        management(&frame(NULL, AP, 1, &[]), HwBand::G2, 0).unwrap(),
        data(
            &frame(DATA, AP, 1, &PLAINTEXT),
            0,
            false,
            HwBand::G2,
            0,
            NO_CAPS,
            None,
        )
        .unwrap(),
        data(
            &qos(QOS, 7, 1, &EAPOL_BODY),
            7,
            true,
            HwBand::G5,
            0,
            NO_CAPS,
            None,
        )
        .unwrap(),
        data(
            &frame(DATA_PROTECTED, AP, 1, &PLAINTEXT),
            0,
            false,
            HwBand::G2,
            0,
            NO_CAPS,
            sec(0, 0, 1),
        )
        .unwrap(),
    ];
    for (i, (_, head)) in heads.iter().enumerate() {
        assert_eq!(words(head)[0] & (1 << 22), 1 << 22, "head {i}");
    }
}

#[test]
fn each_encoder_takes_only_its_frames() {
    const NOT_MANAGEMENT: Result<(TxChannel, [u8; WD_HEAD_BYTES]), AxError> =
        Err(AxError::Tx("not a management or null data frame"));
    const NOT_DATA: Result<(TxChannel, [u8; WD_HEAD_BYTES]), AxError> =
        Err(AxError::Tx("not a data frame with a body"));
    // rtw89_core_get_tx_type (core.c:624-634): null data goes as management.
    for fc in [NULL, QOS_NULL] {
        let f = if fc == QOS_NULL {
            qos(fc, 0, 3, &[])
        } else {
            frame(fc, AP, 3, &[])
        };
        let (ch, head) = management(&f, HwBand::G2, 0).unwrap();
        assert_eq!(ch, TxChannel::Ch8);
        assert_eq!(
            words(&head)[..4],
            [0x0048_0080, 0x0400_0000, 0x0024_0000 | f.len() as u32, 3]
        );
        assert_eq!(data(&f, 0, false, HwBand::G2, 0, NO_CAPS, None), NOT_DATA);
    }
    for f in [frame(DATA, AP, 1, &PLAINTEXT), qos(QOS, 0, 1, &PLAINTEXT)] {
        assert_eq!(management(&f, HwBand::G2, 0), NOT_MANAGEMENT);
    }
    assert_eq!(
        data(&auth(1), 0, false, HwBand::G2, 0, NO_CAPS, None),
        NOT_DATA
    );
    let ack = frame(ACK, AP, 0, &[]);
    assert_eq!(management(&ack, HwBand::G2, 0), NOT_MANAGEMENT);
    assert_eq!(data(&ack, 0, false, HwBand::G2, 0, NO_CAPS, None), NOT_DATA);
}

#[test]
fn encoders_refuse_short_and_long_frames() {
    const SHORT: Result<(TxChannel, [u8; WD_HEAD_BYTES]), AxError> =
        Err(AxError::Tx("frame shorter than its 802.11 header"));
    const LONG: Result<(TxChannel, [u8; WD_HEAD_BYTES]), AxError> =
        Err(AxError::Tx("frame longer than one TX address entry"));
    let a = auth(1);
    assert_eq!(management(&a[..23], HwBand::G2, 0), SHORT);
    assert_eq!(management(&a[..1], HwBand::G2, 0), SHORT);
    assert_eq!(management(&[], HwBand::G2, 0), SHORT);
    // A QoS data frame needs its QoS control too.
    let q = qos(QOS, 0, 1, &PLAINTEXT);
    assert_eq!(
        data(&q[..25], 0, false, HwBand::G2, 0, NO_CAPS, None),
        SHORT
    );
    assert_eq!(data(&[], 0, false, HwBand::G2, 0, NO_CAPS, None), SHORT);
    // 2044 bytes fit one TX address entry (TXPKT_SIZE in B2); 2045 do not.
    let mut big = frame(AUTH, AP, 1, &[]);
    big.resize(MAX_TX_FRAME, 0);
    let (_, head) = management(&big, HwBand::G2, 0).unwrap();
    assert_eq!(words(&head)[2], 0x0024_0000 | 2044);
    big.push(0);
    assert_eq!(management(&big, HwBand::G2, 0), LONG);
    let mut big = frame(DATA, AP, 1, &[]);
    big.resize(MAX_TX_FRAME, 0);
    let (_, head) = data(&big, 0, false, HwBand::G2, 0, NO_CAPS, None).unwrap();
    assert_eq!(words(&head)[2], 2044);
    big.push(0);
    assert_eq!(data(&big, 0, false, HwBand::G2, 0, NO_CAPS, None), LONG);
}

#[test]
fn data_header_length_is_ieee80211_hdrlen() {
    // HDR_LLC_LEN [15:11] in units of 2 bytes: 24, 26 (QoS), 30 (four
    // addresses), 32 (both), 30 (QoS with HT Control, Order b15).
    for (fc, header, len) in [
        (DATA, 24, 12),
        (QOS, 26, 13),
        (0x0308, 30, 15),
        (0x0388, 32, 16),
        (0x8188, 30, 15),
    ] {
        let mut f = frame(fc, AP, 1, &[]);
        f.resize(header, 0);
        f.extend_from_slice(&PLAINTEXT);
        let (_, head) = data(&f, 0, false, HwBand::G2, 0, NO_CAPS, None).unwrap();
        assert_eq!(words(&head)[0], 0x0040_0080 | (len << 11), "fc {fc:#06x}");
        let mut short = f.clone();
        short.truncate(header - 1);
        assert_eq!(
            data(&short, 0, false, HwBand::G2, 0, NO_CAPS, None),
            Err(AxError::Tx("frame shorter than its 802.11 header")),
            "fc {fc:#06x}"
        );
    }
}

#[test]
fn data_refuses_protection_without_its_key() {
    let protected = frame(DATA_PROTECTED, AP, 1, &PLAINTEXT);
    assert_eq!(
        data(&protected, 0, false, HwBand::G2, 0, NO_CAPS, None),
        Err(AxError::Tx("a protected data frame without its key"))
    );
    let plain = frame(DATA, AP, 1, &PLAINTEXT);
    assert_eq!(
        data(&plain, 0, false, HwBand::G2, 0, NO_CAPS, sec(0, 0, PN)),
        Err(AxError::Tx("a key for an unprotected data frame"))
    );
}

#[test]
fn data_refuses_keys_the_descriptor_cannot_carry() {
    let f = frame(DATA_PROTECTED, AP, 1, &PLAINTEXT);
    for (s, error) in [
        (sec(4, 0, PN), "a CCMP key ID past 3"),
        (sec(0, 0, 0), "a CCMP packet number of 0 or past 48 bits"),
        (
            sec(0, 0, 1 << 48),
            "a CCMP packet number of 0 or past 48 bits",
        ),
        (sec(0, 128, PN), "a security CAM index past 127"),
    ] {
        assert_eq!(
            data(&f, 0, false, HwBand::G2, 0, NO_CAPS, s),
            Err(AxError::Tx(error))
        );
    }
    // The largest of each fits its field.
    let (_, head) = data(
        &f,
        0,
        false,
        HwBand::G2,
        0,
        NO_CAPS,
        sec(3, 127, (1 << 48) - 1),
    )
    .unwrap();
    let w = words(&head);
    assert_eq!(w[1], 0x0400_0036);
    assert_eq!(w[4..6], [0xFFFF_0000, 0xFFFF_FFFF]);
    assert_eq!(w[info(2)], 0x0000_017F);
}

#[test]
fn strip_refuses_an_unprotected_frame() {
    // Its 8 bytes after the header are plaintext, even when they read as
    // the reservation's header.
    let mut mpdu = frame(DATA, AP, 0x456, &PN_HEADER);
    mpdu.extend_from_slice(&PLAINTEXT);
    let before = mpdu.clone();
    assert_eq!(
        strip_ccmp(&mut mpdu, PN, 0),
        Err(AxError::Tx("not a protected frame"))
    );
    assert_eq!(mpdu, before);
}

#[test]
fn data_takes_the_tid_its_frame_carries() {
    const OTHER: Result<(TxChannel, [u8; WD_HEAD_BYTES]), AxError> =
        Err(AxError::Tx("a TID other than the frame's own"));
    // skb->priority's TID is the QoS control's (ieee80211_set_qos_hdr).
    let f = qos(QOS, 5, 1, &PLAINTEXT);
    let (ch, _) = data(&f, 5, false, HwBand::G2, 0, NO_CAPS, None).unwrap();
    assert_eq!(ch, TxChannel::Ach2);
    assert_eq!(data(&f, 0, false, HwBand::G2, 0, NO_CAPS, None), OTHER);
    assert_eq!(data(&f, 6, false, HwBand::G2, 0, NO_CAPS, None), OTHER);
    // A frame without QoS goes at priority 0 (__ieee80211_select_queue):
    // TID 8 is TID 0, best effort on ACH0; TID 7 is not its own.
    let f = frame(DATA, AP, 1, &PLAINTEXT);
    let (ch, head) = data(&f, 8, false, HwBand::G2, 0, NO_CAPS, None).unwrap();
    assert_eq!(ch, TxChannel::Ach0);
    assert_eq!(words(&head)[2] & 0x00FE_0000, 0);
    assert_eq!(data(&f, 7, false, HwBand::G2, 0, NO_CAPS, None), OTHER);
    // The QoS control's TID field is four bits: a TSPEC TID 15 is no TID 7.
    let f = qos(QOS, 15, 1, &PLAINTEXT);
    assert_eq!(data(&f, 7, false, HwBand::G2, 0, NO_CAPS, None), OTHER);
}

#[test]
fn eapol_under_the_ptk_matches_linux() {
    // A group handshake's message 2 under the PTK, as the strip leaves it:
    // VO with BK and the TID indication, and the key's dwords.
    let f = qos(QOS_PROTECTED, 7, 0x123, &EAPOL_BODY);
    let len = f.len() as u32;
    let (ch, head) = data(&f, 7, true, HwBand::G2, 0xFFF, NO_CAPS, sec(0, 0, PN)).unwrap();
    assert_eq!(ch, TxChannel::Ach3);
    assert_eq!(
        words(&head),
        [
            0x0143_6880,       // B0: WP_OFFSET 1, WD_INFO_EN, ACH3, HDR_LLC_LEN 13
            0x0400_0006,       // B1: ADDR_INFO_NUM 1, SEC_TYPE CCMP128
            0x0086_0000 | len, // B2: TID_INDICATE, QSEL VO_0
            0x2000 | 0x123,    // B3: BK, SW_SEQ
            0x0201_0000,       // B4: PN1, PN0
            0x0605_0403,       // B5: PN5..PN2
            0,
            0,
            0,
            0,
            0x0000_0100, // I2: FORCE_KEY_EN, SEC_CAM_IDX 0
            0,
            0x8800_0000,
            0,
        ]
    );
}

// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2020-2022  Realtek Corporation
use super::*;
use alloc::{vec, vec::Vec};

/// The descriptor fields a test sets; everything else is zero.
#[derive(Clone, Copy)]
struct Desc {
    pkt_type: u8,
    long: bool,
    ppdu_cnt: u8,
    rate: u16,
    gi_ltf: u8,
    bw: u8,
    free_run: u32,
    d3: u32,
    frame_type: u8,
}

/// A long MGNT WIFI descriptor at PPDU 1, legacy OFDM6 (rate 4), 3.2 us
/// GI-LTF code 0, 20 MHz.
fn mgnt(ppdu_cnt: u8) -> Desc {
    Desc {
        pkt_type: 0,
        long: true,
        ppdu_cnt,
        rate: 4,
        gi_ltf: 0,
        bw: 0,
        free_run: 0x1234_5678,
        d3: 0,
        frame_type: 0,
    }
}

/// A PPDU_STAT descriptor with the same rate, GI and bandwidth as `mgnt`.
fn stat(ppdu_cnt: u8) -> Desc {
    Desc {
        pkt_type: 1,
        long: false,
        frame_type: 0,
        ..mgnt(ppdu_cnt)
    }
}

/// One RXQ buffer, shaped like `command/tests.rs`'s `rx_buffer`: the RX BD
/// info word (FS, LS, WRITE_SIZE), a short (16-byte) or long (32-byte)
/// descriptor, then `payload`.
fn rx_buffer(d: &Desc, payload: &[u8]) -> Vec<u8> {
    let rxd = if d.long { 32 } else { 16 };
    let write = 4 + rxd + payload.len();
    let mut b = vec![0u8; write];
    b[..4].copy_from_slice(&(0xC000 | write as u32).to_le_bytes());
    let d0 = payload.len() as u32 | u32::from(d.pkt_type) << 24 | if d.long { 1 << 31 } else { 0 };
    let d1 = u32::from(d.ppdu_cnt) << 4
        | u32::from(d.rate) << 16
        | u32::from(d.gi_ltf) << 25
        | u32::from(d.bw) << 29;
    b[4..8].copy_from_slice(&d0.to_le_bytes());
    b[8..12].copy_from_slice(&d1.to_le_bytes());
    b[12..16].copy_from_slice(&d.free_run.to_le_bytes());
    b[16..20].copy_from_slice(&d.d3.to_le_bytes());
    if d.long {
        b[20..24].copy_from_slice(&u32::from(d.frame_type).to_le_bytes());
    }
    b[4 + rxd..].copy_from_slice(payload);
    b
}

const FCS: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

/// An 802.11 frame with frame-control byte `fc0`: a 24-byte header, a
/// 12-byte fixed body, then the FCS. 40 bytes.
fn frame(fc0: u8, tag: u8) -> Vec<u8> {
    let mut b = vec![fc0, 0];
    b.extend_from_slice(&[0, 0]);
    b.extend_from_slice(&[0xFF; 6]);
    b.extend_from_slice(&[0x02, 0x11, 0x22, 0x33, 0x44, tag]);
    b.extend_from_slice(&[0x02, 0x11, 0x22, 0x33, 0x44, tag]);
    b.extend_from_slice(&[0, 0]);
    b.extend_from_slice(&[tag; 12]);
    b.extend_from_slice(&FCS);
    b
}

fn beacon(tag: u8) -> Vec<u8> {
    frame(0x80, tag)
}

/// A PPDU status (no MAC info): header w0 VALID, LEN 5 (40 bytes), IE map
/// 12; w1 RSSI A `a`, B `b`; then a 32-byte IE01 carrying `chan_idx`.
fn phy_status(a: u8, b: u8, chan_idx: u8) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&0x8000_058Cu32.to_le_bytes());
    p.extend_from_slice(&(u32::from(b) << 8 | u32::from(a)).to_le_bytes());
    let mut ie01 = vec![0u8; 32];
    ie01[..4].copy_from_slice(&(u32::from(chan_idx) << 16 | 1).to_le_bytes());
    p.extend_from_slice(&ie01);
    p
}

/// A SCANOFLD_RSP C2H (cat 1, class 1, func 9) with `reason`, `pri_ch` and
/// `band`, 24 bytes.
fn scanofld(reason: u8, band: u8, pri_ch: u8) -> Vec<u8> {
    let w = [
        1 | 1 << 2 | 9 << 8,
        24,
        u32::from(pri_ch) | u32::from(reason) << 16,
        0,
        0,
        u32::from(band) << 24,
    ];
    let p: Vec<u8> = w.iter().flat_map(|w: &u32| w.to_le_bytes()).collect();
    rx_buffer(
        &Desc {
            pkt_type: 10,
            long: false,
            ..mgnt(0)
        },
        &p,
    )
}

/// What the sink saw, owned.
#[derive(Debug, PartialEq, Eq)]
enum Seen {
    C2h(C2h<'static>),
    Frame(Received),
}

fn feed(rx: &mut Receiver, buffers: &[Vec<u8>]) -> Vec<Seen> {
    let mut seen = Vec::new();
    for b in buffers {
        rx.buffer(b, &mut |e| {
            seen.push(match e {
                Event::C2h(C2h::ScanOfld(s)) => Seen::C2h(C2h::ScanOfld(s)),
                Event::C2h(C2h::RecAck {
                    cat,
                    class,
                    func,
                    seq,
                }) => Seen::C2h(C2h::RecAck {
                    cat,
                    class,
                    func,
                    seq,
                }),
                Event::C2h(C2h::DoneAck {
                    cat,
                    class,
                    func,
                    ret,
                    seq,
                }) => Seen::C2h(C2h::DoneAck {
                    cat,
                    class,
                    func,
                    ret,
                    seq,
                }),
                Event::C2h(C2h::Other { cat, class, func }) => {
                    Seen::C2h(C2h::Other { cat, class, func })
                }
                Event::C2h(C2h::Log(_)) => Seen::C2h(C2h::Log(&[])),
                Event::C2h(C2h::FormattedLog) => Seen::C2h(C2h::FormattedLog),
                Event::C2h(C2h::BeaconFilter(b)) => Seen::C2h(C2h::BeaconFilter(b)),
                Event::Frame(f) => Seen::Frame(f),
            })
        })
        .expect("well-formed buffer");
    }
    seen
}

fn frames(seen: &[Seen]) -> Vec<&Received> {
    seen.iter()
        .filter_map(|s| match s {
            Seen::Frame(f) => Some(f),
            Seen::C2h(_) => None,
        })
        .collect()
}

const G2_1: Tag = Tag {
    band: HwBand::G2,
    channel: 1,
};

#[test]
fn matched_beacon_gets_signal_and_chains() {
    let mut rx = Receiver::new(G2_1);
    let b = beacon(1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &b),
            rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0)),
        ],
    );
    assert_eq!(
        seen,
        [Seen::Frame(Received {
            mpdu: b[..b.len() - 4].to_vec(),
            tag: G2_1,
            signal: Some(-54),
            chains: [Some(-54), Some(-62)],
            fcs_error: false,
            decrypted: false,
            icv_error: false,
            data_rate: 4,
            gi_ltf: 0,
            bw: 0,
            free_run: 0x1234_5678,
            hw_dec: false,
            a1_match: false,
            mac_id: 0,
            sec_type: 0,
        })]
    );
    assert_eq!(rx.counts.ppdu, 1);
    assert_eq!(rx.pending(), 0);
}

#[test]
fn signal_is_the_stronger_path() {
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &beacon(1)),
            rx_buffer(&stat(1), &phy_status(0x40, 0x90, 0)),
        ],
    );
    let f = frames(&seen);
    assert_eq!(f[0].signal, Some(-38));
    assert_eq!(f[0].chains, [Some(-78), Some(-38)]);
}

#[test]
fn matched_ppdu_with_a_failed_header_still_gives_minus_110() {
    // LEN 4 (32 bytes) against a 40-byte status: the header is refused, the
    // RSSI stays 0, and Linux still reports -110 dBm (S11.5).
    let mut status = phy_status(0x70, 0x60, 0);
    status[1] = 4;
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &beacon(1)),
            rx_buffer(&stat(1), &status),
        ],
    );
    let f = frames(&seen);
    assert_eq!(f[0].signal, Some(-110));
    assert_eq!(f[0].chains, [Some(-110), Some(-110)]);
}

#[test]
fn zero_dbm_signal_reads_as_no_signal() {
    // Raw 0xDC is 0 dBm, which `rtw89_core_validate_rx_signal` flags as
    // RX_FLAG_NO_SIGNAL_VAL; the chains are still reported. 0xDE (1 dBm)
    // is a signal.
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &beacon(1)),
            rx_buffer(&stat(1), &phy_status(0xDC, 0x10, 0)),
            rx_buffer(&mgnt(2), &beacon(2)),
            rx_buffer(&stat(2), &phy_status(0x10, 0xDE, 0)),
        ],
    );
    let f = frames(&seen);
    assert_eq!(f[0].signal, None);
    assert_eq!(f[0].chains, [Some(0), Some(-102)]);
    assert_eq!(f[1].signal, Some(1));
}

#[test]
fn beacon_flushed_by_next_ppdu_has_no_signal() {
    let mut rx = Receiver::new(G2_1);
    let seen = feed(&mut rx, &[rx_buffer(&mgnt(1), &beacon(1))]);
    assert!(seen.is_empty());
    assert_eq!(rx.pending(), 1);
    let seen = feed(&mut rx, &[rx_buffer(&mgnt(2), &beacon(2))]);
    let f = frames(&seen);
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].mpdu[10..16], [0x02, 0x11, 0x22, 0x33, 0x44, 1]);
    assert_eq!(f[0].signal, None);
    assert_eq!(f[0].chains, [None, None]);
    assert_eq!(rx.pending(), 1);
    // Its own status arrives late, after the next PPDU's frame: the second
    // beacon pairs with PPDU 2's status, the first is long gone.
    let seen = feed(&mut rx, &[rx_buffer(&stat(2), &phy_status(0x70, 0x60, 0))]);
    let f = frames(&seen);
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].mpdu[15], 2);
    assert_eq!(f[0].signal, Some(-54));
}

#[test]
fn status_of_another_ppdu_delivers_the_queue_without_signal() {
    // A status whose PPDU_CNT is not `curr` does not match, but still
    // drains the queue (`rtw89_core_rx_pending_skb`).
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &beacon(1)),
            rx_buffer(&stat(3), &phy_status(0x70, 0x60, 0)),
        ],
    );
    let f = frames(&seen);
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].signal, None);
    assert_eq!(rx.pending(), 0);
}

#[test]
fn rate_gi_or_bw_mismatch_gives_no_signal() {
    for stat_desc in [
        Desc { rate: 5, ..stat(1) },
        Desc {
            gi_ltf: 1,
            ..stat(1)
        },
        Desc { bw: 1, ..stat(1) },
    ] {
        let mut rx = Receiver::new(G2_1);
        let seen = feed(
            &mut rx,
            &[
                rx_buffer(&mgnt(1), &beacon(1)),
                rx_buffer(&stat_desc, &phy_status(0x70, 0x60, 0)),
            ],
        );
        let f = frames(&seen);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].signal, None);
    }
}

#[test]
fn short_descriptor_beacon_is_delivered_at_once_without_signal() {
    let mut rx = Receiver::new(G2_1);
    let short = Desc {
        long: false,
        ..mgnt(1)
    };
    let b = beacon(1);
    let seen = feed(&mut rx, &[rx_buffer(&short, &b)]);
    let f = frames(&seen);
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].mpdu, b[..36]);
    assert_eq!(f[0].signal, None);
    assert_eq!(f[0].chains, [None, None]);
    assert_eq!(rx.pending(), 0);
    // The status that follows finds nothing to pair.
    let seen = feed(&mut rx, &[rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0))]);
    assert!(seen.is_empty());
    assert_eq!(rx.counts.ppdu, 1);
}

#[test]
fn long_ctrl_typed_beacon_is_delivered_at_once() {
    // Only MGNT and DATA descriptor types are queued (`PPDU_FILTER_BITMAP`).
    for frame_type in [1, 3] {
        let mut rx = Receiver::new(G2_1);
        let seen = feed(
            &mut rx,
            &[rx_buffer(
                &Desc {
                    frame_type,
                    ..mgnt(1)
                },
                &beacon(1),
            )],
        );
        assert_eq!(frames(&seen).len(), 1);
        assert_eq!(rx.pending(), 0);
    }
    let mut rx = Receiver::new(G2_1);
    feed(
        &mut rx,
        &[rx_buffer(
            &Desc {
                frame_type: 2,
                ..mgnt(1)
            },
            &beacon(1),
        )],
    );
    assert_eq!(rx.pending(), 1);
}

#[test]
fn probe_response_is_queued_like_a_beacon() {
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &frame(0x50, 7)),
            rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0)),
        ],
    );
    let f = frames(&seen);
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].mpdu[0], 0x50);
    assert_eq!(f[0].signal, Some(-54));
}

#[test]
fn ppdu_stat_before_any_frame_is_ignored() {
    let mut rx = Receiver::new(G2_1);
    let seen = feed(&mut rx, &[rx_buffer(&stat(0), &phy_status(0x70, 0x60, 0))]);
    assert!(seen.is_empty());
    assert_eq!(rx.counts.ppdu_ignored, 1);
    assert_eq!(rx.counts.ppdu, 0);
}

#[test]
fn enter_ch_retags_following_frames() {
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            scanofld(c2h::SCAN_ENTER_CH, 1, 36),
            rx_buffer(&mgnt(1), &beacon(1)),
            rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0)),
        ],
    );
    assert_eq!(seen.len(), 2);
    let Seen::C2h(C2h::ScanOfld(s)) = &seen[0] else {
        panic!("the C2H comes first: {seen:?}");
    };
    assert_eq!((s.reason, s.band, s.pri_ch), (3, 1, 36));
    let g5_36 = Tag {
        band: HwBand::G5,
        channel: 36,
    };
    assert_eq!(frames(&seen)[0].tag, g5_36);
    assert_eq!(rx.tag(), g5_36);
}

#[test]
fn enter_op_retags_and_leave_and_end_do_not() {
    let mut rx = Receiver::new(G2_1);
    feed(&mut rx, &[scanofld(c2h::SCAN_ENTER_OP, 0, 6)]);
    assert_eq!(
        rx.tag(),
        Tag {
            band: HwBand::G2,
            channel: 6
        }
    );
    let seen = feed(
        &mut rx,
        &[
            scanofld(c2h::SCAN_LEAVE_CH, 1, 36),
            scanofld(c2h::SCAN_LEAVE_OP, 1, 36),
            scanofld(c2h::SCAN_END, 1, 36),
            scanofld(0, 1, 36),
        ],
    );
    assert_eq!(seen.len(), 4);
    assert_eq!(
        rx.tag(),
        Tag {
            band: HwBand::G2,
            channel: 6
        }
    );
}

#[test]
fn enter_ch_band_2_is_6g_and_band_3_reads_as_2g() {
    // `band` is used as given (8852C supports 6 GHz, `mac.c:5310-5311`);
    // the undefined 3 is 2.4 GHz, as `rtw89_hw_to_nl80211_band`'s default
    // (`core.h:8469-8480`).
    let mut rx = Receiver::new(G2_1);
    feed(&mut rx, &[scanofld(c2h::SCAN_ENTER_CH, 2, 37)]);
    assert_eq!(
        rx.tag(),
        Tag {
            band: HwBand::G6,
            channel: 37
        }
    );
    feed(&mut rx, &[scanofld(c2h::SCAN_ENTER_CH, 3, 5)]);
    assert_eq!(
        rx.tag(),
        Tag {
            band: HwBand::G2,
            channel: 5
        }
    );
}

#[test]
fn queued_frame_keeps_its_arrival_tag_across_a_retag() {
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &beacon(1)),
            scanofld(c2h::SCAN_ENTER_CH, 1, 36),
            rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0)),
        ],
    );
    let f = frames(&seen);
    assert_eq!(f[0].tag, G2_1);
    assert_eq!(f[0].signal, Some(-54));
}

#[test]
fn ie01_chan_idx_overrides_the_tag() {
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &beacon(1)),
            rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0x30)),
        ],
    );
    assert_eq!(
        frames(&seen)[0].tag,
        Tag {
            band: HwBand::G5,
            channel: 100
        }
    );
    // The receiver's own tag is untouched.
    assert_eq!(rx.tag(), G2_1);
}

#[test]
fn ie01_index_of_an_invalid_walk_is_not_applied() {
    // A zero-length IE (type 23) after IE01 fails the walk: `valid` is
    // false, so `rtw8852c_query_ppdu` does not use the index.
    let mut status = phy_status(0x70, 0x60, 0x30);
    status[1] = 6;
    status.extend_from_slice(&[23, 0, 0, 0, 0, 0, 0, 0]);
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &beacon(1)),
            rx_buffer(&stat(1), &status),
        ],
    );
    let f = frames(&seen);
    assert_eq!(f[0].signal, Some(-54));
    assert_eq!(f[0].tag, G2_1);
}

#[test]
fn cck_frame_tagged_5g_takes_the_previous_channel() {
    let g2_13 = Tag {
        band: HwBand::G2,
        channel: 13,
    };
    let g5_36 = Tag {
        band: HwBand::G5,
        channel: 36,
    };
    let cck = |cnt| Desc {
        rate: 0,
        ..mgnt(cnt)
    };
    let cck_stat = |cnt| Desc {
        rate: 0,
        ..stat(cnt)
    };
    let mut rx = Receiver::new(g2_13);
    let seen = feed(
        &mut rx,
        &[
            scanofld(c2h::SCAN_ENTER_CH, 1, 36),
            rx_buffer(&cck(1), &beacon(1)),
            rx_buffer(&cck_stat(1), &phy_status(0x70, 0x60, 0)),
        ],
    );
    let f = frames(&seen);
    assert_eq!(f[0].tag, g2_13);
    assert_eq!(f[0].signal, Some(-54));

    // CCK 11M (rate 3) unmatched is still corrected on the PPDU path...
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&Desc { rate: 3, ..mgnt(2) }, &beacon(2)),
            rx_buffer(&cck_stat(2), &phy_status(0x70, 0x60, 0)),
        ],
    );
    let f = frames(&seen);
    assert_eq!((f[0].tag, f[0].signal), (g2_13, None));

    // ...OFDM 6M (rate 4) is not...
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(3), &beacon(3)),
            rx_buffer(&stat(3), &phy_status(0x70, 0x60, 0)),
        ],
    );
    assert_eq!(frames(&seen)[0].tag, g5_36);

    // ...nor is HT MCS 0 (mode 1, index 0)...
    let ht = 1 << 7;
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(
                &Desc {
                    rate: ht,
                    ..mgnt(4)
                },
                &beacon(4),
            ),
            rx_buffer(
                &Desc {
                    rate: ht,
                    ..stat(4)
                },
                &phy_status(0x70, 0x60, 0),
            ),
        ],
    );
    assert_eq!(frames(&seen)[0].tag, g5_36);

    // ...nor a CCK frame flushed by the next PPDU, nor one delivered at
    // once (Linux corrects only in `rtw89_core_rx_pending_skb`).
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&cck(5), &beacon(5)),
            rx_buffer(&cck(6), &beacon(6)),
            rx_buffer(
                &Desc {
                    long: false,
                    ..cck(6)
                },
                &beacon(7),
            ),
        ],
    );
    let f = frames(&seen);
    assert_eq!(f.len(), 2);
    assert!(f.iter().all(|f| f.tag == g5_36));
}

#[test]
fn cck_correction_uses_the_previous_channel_at_delivery() {
    // `rtw89_correct_cck_chan` reads the channel record when the status
    // arrives, not when the frame did.
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            scanofld(c2h::SCAN_ENTER_CH, 1, 36),
            rx_buffer(&Desc { rate: 0, ..mgnt(1) }, &beacon(1)),
            scanofld(c2h::SCAN_ENTER_CH, 0, 6),
            scanofld(c2h::SCAN_ENTER_CH, 1, 40),
            rx_buffer(&Desc { rate: 0, ..stat(1) }, &phy_status(0x70, 0x60, 0)),
        ],
    );
    // At arrival `prev` was 2G 1; at delivery it is 2G 6.
    assert_eq!(
        frames(&seen)[0].tag,
        Tag {
            band: HwBand::G2,
            channel: 6
        }
    );
}

#[test]
fn cck_correction_applies_after_the_ie01_override() {
    // IE01 says 5G 100; the frame is CCK, so it takes `prev` (2G 1).
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&Desc { rate: 1, ..mgnt(1) }, &beacon(1)),
            rx_buffer(&Desc { rate: 1, ..stat(1) }, &phy_status(0x70, 0x60, 0x30)),
        ],
    );
    assert_eq!(frames(&seen)[0].tag, G2_1);
}

#[test]
fn non_beacon_frames_are_discarded() {
    let mut rx = Receiver::new(G2_1);
    let data = Desc {
        frame_type: 2,
        ..mgnt(1)
    };
    let ctrl = Desc {
        frame_type: 1,
        ..mgnt(1)
    };
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&data, &frame(0x08, 1)),
            rx_buffer(
                &ctrl,
                &[0xD4, 0, 0, 0, 2, 0x11, 0x22, 0x33, 0x44, 0x55, 0, 0, 0, 0],
            ),
        ],
    );
    assert!(seen.is_empty());
    assert_eq!(rx.counts.other_frames, 2);
    assert_eq!(rx.counts.short_frames, 0);
    assert_eq!(rx.pending(), 0);
}

#[test]
fn other_management_subtypes_are_discarded() {
    // Probe request 0x40, authentication 0xB0, action 0xD0; a beacon with
    // protocol version bits set is still a beacon (`ieee80211_is_beacon`
    // masks only type and subtype).
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &frame(0x40, 1)),
            rx_buffer(&mgnt(1), &frame(0xB0, 1)),
            rx_buffer(&mgnt(1), &frame(0xD0, 1)),
            rx_buffer(&mgnt(1), &frame(0x83, 1)),
        ],
    );
    assert!(seen.is_empty());
    assert_eq!(rx.counts.other_frames, 3);
    assert_eq!(rx.pending(), 1);
}

#[test]
fn a_discarded_frame_still_advances_the_ppdu() {
    // Linux flushes on every WIFI frame whose PPDU_CNT changed, whatever
    // it is (`core.c:4541-4544`).
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &beacon(1)),
            rx_buffer(
                &Desc {
                    frame_type: 2,
                    ..mgnt(2)
                },
                &frame(0x08, 1),
            ),
        ],
    );
    let f = frames(&seen);
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].signal, None);
}

#[test]
fn payload_shorter_than_header_and_fcs_is_counted() {
    let mut rx = Receiver::new(G2_1);
    let b = beacon(1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &b[..20]),
            rx_buffer(&mgnt(1), &b[..27]),
            rx_buffer(&mgnt(1), &b[..1]),
            rx_buffer(&mgnt(1), &[]),
        ],
    );
    assert!(seen.is_empty());
    assert_eq!(rx.counts.short_frames, 4);
    assert_eq!(rx.pending(), 0);
    // 28 bytes is a header and an FCS: delivered with an empty body.
    let seen = feed(
        &mut rx,
        &[rx_buffer(
            &Desc {
                long: false,
                ..mgnt(1)
            },
            &b[..28],
        )],
    );
    assert_eq!(frames(&seen)[0].mpdu, b[..24]);
}

#[test]
fn crc_error_beacon_is_flagged() {
    for d3 in [1 << 9, 1 << 10, 1 << 9 | 1 << 10] {
        let mut rx = Receiver::new(G2_1);
        let seen = feed(
            &mut rx,
            &[
                rx_buffer(&Desc { d3, ..mgnt(1) }, &beacon(1)),
                rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0)),
            ],
        );
        let f = frames(&seen);
        assert!(f[0].fcs_error, "d3 {d3:#x}");
        assert_eq!(f[0].signal, Some(-54));
    }
}

#[test]
fn descriptor_rate_fields_are_carried_raw() {
    let mut rx = Receiver::new(G2_1);
    let d = Desc {
        long: false,
        rate: 0x1A5,
        gi_ltf: 7,
        bw: 6,
        free_run: 0xFFFF_FFFF,
        ..mgnt(1)
    };
    let f = feed(&mut rx, &[rx_buffer(&d, &beacon(1))]);
    let f = frames(&f);
    assert_eq!(
        (f[0].data_rate, f[0].gi_ltf, f[0].bw, f[0].free_run),
        (0x1A5, 7, 6, 0xFFFF_FFFF)
    );
}

#[test]
fn queue_overflow_delivers_without_signal() {
    let mut rx = Receiver::new(G2_1);
    let buffers: Vec<_> = (0..MAX_PENDING as u8)
        .map(|i| rx_buffer(&mgnt(1), &beacon(i)))
        .collect();
    assert!(feed(&mut rx, &buffers).is_empty());
    assert_eq!(rx.pending(), MAX_PENDING);
    let seen = feed(&mut rx, &[rx_buffer(&mgnt(1), &beacon(0xEE))]);
    let f = frames(&seen);
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].mpdu[15], 0xEE);
    assert_eq!(f[0].signal, None);
    assert_eq!(rx.counts.overflow, 1);
    assert_eq!(rx.pending(), MAX_PENDING);
    // The 64 queued ones still pair with their status, in order.
    let seen = feed(&mut rx, &[rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0))]);
    let f = frames(&seen);
    assert_eq!(f.len(), MAX_PENDING);
    for (i, f) in f.iter().enumerate() {
        assert_eq!(f.mpdu[15], i as u8);
        assert_eq!(f.signal, Some(-54));
    }
    assert_eq!(rx.pending(), 0);
}

#[test]
fn a_flood_never_grows_the_queue_and_counters_saturate() {
    let mut rx = Receiver::new(G2_1);
    let b = rx_buffer(&mgnt(1), &beacon(1));
    let mut delivered = 0usize;
    for _ in 0..70_000 {
        rx.buffer(&b, &mut |_| delivered += 1).unwrap();
        assert!(rx.pending() <= MAX_PENDING);
    }
    assert_eq!(delivered, 70_000 - MAX_PENDING);
    assert_eq!(rx.counts.overflow, u16::MAX);
}

#[test]
fn flush_delivers_what_never_got_a_status() {
    let mut rx = Receiver::new(G2_1);
    feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &beacon(1)),
            rx_buffer(&mgnt(1), &beacon(2)),
        ],
    );
    let mut seen = Vec::new();
    rx.flush(&mut |e| {
        if let Event::Frame(f) = e {
            seen.push(f)
        }
    });
    assert_eq!(seen.len(), 2);
    assert!(seen.iter().all(|f| f.signal.is_none() && f.tag == G2_1));
    assert_eq!(seen[1].mpdu[15], 2);
    assert_eq!(rx.pending(), 0);
    // A late status then pairs with nothing.
    let mut n = 0;
    rx.buffer(
        &rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0)),
        &mut |_| n += 1,
    )
    .unwrap();
    assert_eq!(n, 0);
}

#[test]
fn c2hs_reach_the_sink_and_short_ones_are_skipped() {
    let c2h_desc = Desc {
        pkt_type: 10,
        long: false,
        ..mgnt(0)
    };
    let done_ack: Vec<u8> = [1u32 | 1 << 8, 12, 0x0700_1425]
        .iter()
        .flat_map(|w| w.to_le_bytes())
        .collect();
    let mut rx = Receiver::new(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&c2h_desc, &done_ack),
            rx_buffer(&c2h_desc, &done_ack[..6]),
            // A SCANOFLD_RSP cut at 20 bytes.
            rx_buffer(&c2h_desc, &scanofld(3, 1, 36)[20..40]),
        ],
    );
    assert_eq!(
        seen,
        [Seen::C2h(C2h::DoneAck {
            cat: 1,
            class: 9,
            func: 0x14,
            ret: 0,
            seq: 7
        })]
    );
    assert_eq!(rx.counts.c2h_skipped, 2);
    assert_eq!(rx.tag(), G2_1);
}

#[test]
fn segments_and_other_report_types_are_dropped() {
    let mut rx = Receiver::new(G2_1);
    let mut seg = rx_buffer(&mgnt(1), &beacon(1));
    seg[1] &= !0x40; // clear LS
    let other = rx_buffer(
        &Desc {
            pkt_type: 2,
            long: false,
            ..mgnt(5)
        },
        &beacon(1),
    );
    let seen = feed(&mut rx, &[seg, other]);
    assert!(seen.is_empty());
    assert_eq!(rx.counts.segments, 1);
    assert_eq!(rx.pending(), 0);
    // Neither moved `curr`: a status for PPDU 0 is still ignored.
    feed(&mut rx, &[rx_buffer(&stat(0), &phy_status(0x70, 0x60, 0))]);
    assert_eq!(rx.counts.ppdu_ignored, 1);
}

#[test]
fn malformed_buffer_is_an_error() {
    let mut rx = Receiver::new(G2_1);
    let whole = rx_buffer(&mgnt(1), &beacon(1));
    // Shorter than its long descriptor.
    let r = rx.buffer(&whole[..20], &mut |_| panic!("no event"));
    assert!(matches!(r, Err(AxError::Rx(_))), "{r:?}");
    // WRITE_SIZE below the descriptor.
    let mut b = whole.clone();
    b[..4].copy_from_slice(&(0xC000u32 | 30).to_le_bytes());
    let r = rx.buffer(&b, &mut |_| panic!("no event"));
    assert!(matches!(r, Err(AxError::Rx(_))), "{r:?}");
    assert_eq!(rx.counts, RxCounts::default());
    assert_eq!(rx.pending(), 0);
}

#[test]
fn every_truncation_and_byte_pattern_never_panics() {
    let samples = [
        rx_buffer(&mgnt(1), &beacon(1)),
        rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0x30)),
        scanofld(c2h::SCAN_ENTER_CH, 1, 36),
        vec![0xFF; 300],
    ];
    for s in &samples {
        for cut in 0..=s.len() {
            for b in [&s[..cut], &s[cut..]] {
                let mut rx = Receiver::new(G2_1);
                let _ = rx.buffer(&s[..], &mut |_| {});
                let _ = rx.buffer(b, &mut |_| {});
                let mut f = b.to_vec();
                if f.len() >= 4 {
                    // Force FS, LS and a WRITE_SIZE of the slice.
                    let write = 0xC000 | (f.len() as u32 & 0x3FFF);
                    f[..4].copy_from_slice(&write.to_le_bytes());
                }
                let _ = rx.buffer(&f, &mut |_| {});
                assert!(rx.pending() <= MAX_PENDING);
            }
        }
    }
    // Every packet type, every PPDU_CNT, all-ones descriptor words.
    for pkt_type in 0..16u8 {
        let mut rx = Receiver::new(G2_1);
        for cnt in 0..8 {
            let mut b = rx_buffer(
                &Desc {
                    pkt_type,
                    ..mgnt(cnt)
                },
                &[0xFF; 64],
            );
            b[8..20].fill(0xFF);
            let _ = rx.buffer(&b, &mut |_| {});
        }
        rx.flush(&mut |_| {});
    }
}

// The station receiver (M3 spec S7).

/// A receiver in [`Mode::Station`] tagging frames with `tag`.
fn station(tag: Tag) -> Receiver {
    let mut rx = Receiver::new(tag);
    rx.set_mode(Mode::Station);
    rx
}

/// A long DATA WIFI descriptor at `ppdu_cnt`, otherwise `mgnt`'s.
fn data(ppdu_cnt: u8) -> Desc {
    Desc {
        frame_type: 2,
        ..mgnt(ppdu_cnt)
    }
}

/// `RX_DESC` dword 3: SW_DEC b1, HW_DEC b2, CRC32_ERR b9, ICV_ERR b10.
const SW_DEC: u32 = 1 << 1;
const HW_DEC: u32 = 1 << 2;
const CRC32_ERR: u32 = 1 << 9;
const ICV_ERR: u32 = 1 << 10;

const G5_36: Tag = Tag {
    band: HwBand::G5,
    channel: 36,
};

#[test]
fn station_mode_delivers_auth_and_data_at_once() {
    let mut rx = station(G2_1);
    let auth = frame(0xB0, 1);
    let seen = feed(&mut rx, &[rx_buffer(&mgnt(1), &auth)]);
    assert_eq!(
        seen,
        [Seen::Frame(Received {
            mpdu: auth[..auth.len() - 4].to_vec(),
            tag: G2_1,
            signal: None,
            chains: [None, None],
            fcs_error: false,
            decrypted: false,
            icv_error: false,
            data_rate: 4,
            gi_ltf: 0,
            bw: 0,
            free_run: 0x1234_5678,
            hw_dec: false,
            a1_match: false,
            mac_id: 0,
            sec_type: 0,
        })]
    );
    let data_frame = frame(0x08, 2);
    let seen = feed(&mut rx, &[rx_buffer(&data(1), &data_frame)]);
    let f = frames(&seen);
    assert_eq!(f.len(), 1);
    assert_eq!(f[0].mpdu, data_frame[..data_frame.len() - 4]);
    assert_eq!((f[0].signal, f[0].chains), (None, [None, None]));
    assert_eq!(rx.pending(), 0);
    // Their status pairs nothing: each was delivered once.
    let seen = feed(&mut rx, &[rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0))]);
    assert!(seen.is_empty(), "{seen:?}");
    assert_eq!(rx.counts.other_frames, 0);
    // Back in scan mode they are discarded again, as M2c's receiver does.
    rx.set_mode(Mode::Scan);
    let seen = feed(
        &mut rx,
        &[rx_buffer(&mgnt(2), &auth), rx_buffer(&data(2), &data_frame)],
    );
    assert!(seen.is_empty(), "{seen:?}");
    assert_eq!(rx.counts.other_frames, 2);
}

#[test]
fn station_mode_still_pairs_beacons() {
    let mut rx = station(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &beacon(1)),
            rx_buffer(&mgnt(1), &frame(0xB0, 2)),
            rx_buffer(&data(1), &frame(0x08, 3)),
            rx_buffer(&mgnt(1), &frame(0x50, 4)),
            rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0)),
        ],
    );
    let f = frames(&seen);
    // The auth and data frames go at once, ahead of the beacon and the
    // probe response waiting for their status.
    let order: Vec<(u8, Option<i8>)> = f.iter().map(|f| (f.mpdu[0], f.signal)).collect();
    assert_eq!(
        order,
        [
            (0xB0, None),
            (0x08, None),
            (0x80, Some(-54)),
            (0x50, Some(-54))
        ]
    );
    assert_eq!(f[2].chains, [Some(-54), Some(-62)]);
    assert_eq!(rx.pending(), 0);
}

#[test]
fn station_mode_queues_only_beacons_and_probe_responses() {
    let mut rx = station(G2_1);
    let beacons: Vec<_> = (0..MAX_PENDING as u8)
        .map(|i| rx_buffer(&mgnt(1), &beacon(i)))
        .collect();
    assert!(feed(&mut rx, &beacons).is_empty());
    // A full queue does not hold back the frames that never wait.
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&data(1), &frame(0x08, 1)),
            rx_buffer(&mgnt(1), &frame(0xD0, 2)),
        ],
    );
    assert_eq!(frames(&seen).len(), 2);
    assert_eq!(rx.counts.overflow, 0);
    assert_eq!(rx.pending(), MAX_PENDING);
    // One more beacon overflows, as M2c's rule.
    let seen = feed(&mut rx, &[rx_buffer(&mgnt(1), &beacon(0xEE))]);
    assert_eq!(frames(&seen)[0].mpdu[15], 0xEE);
    assert_eq!(rx.counts.overflow, 1);
}

#[test]
fn fcs_error_frames_are_dropped_and_counted() {
    let mut rx = station(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&mgnt(1), &beacon(1)),
            // PPDU 2: still flushes the waiting beacon, then is dropped.
            rx_buffer(
                &Desc {
                    d3: CRC32_ERR,
                    ..mgnt(2)
                },
                &beacon(2),
            ),
        ],
    );
    let f = frames(&seen);
    assert_eq!(f.len(), 1, "{seen:?}");
    assert_eq!((f[0].mpdu[15], f[0].signal), (1, None));
    assert_eq!(rx.pending(), 0);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(
                &Desc {
                    d3: ICV_ERR | HW_DEC,
                    ..data(2)
                },
                &frame(0x08, 3),
            ),
            rx_buffer(
                &Desc {
                    d3: CRC32_ERR | ICV_ERR,
                    long: false,
                    ..mgnt(2)
                },
                &frame(0xB0, 4),
            ),
            // Too short for anything, but failed first.
            rx_buffer(
                &Desc {
                    d3: CRC32_ERR,
                    ..mgnt(2)
                },
                &[],
            ),
            rx_buffer(&stat(2), &phy_status(0x70, 0x60, 0)),
        ],
    );
    assert!(seen.is_empty(), "{seen:?}");
    assert_eq!(rx.counts.dropped_fcs, 4);
    assert_eq!(rx.counts.short_frames, 0);
    assert_eq!(rx.pending(), 0);
    // The scan keeps M2c's rule: flagged and delivered, nothing dropped.
    let mut scan = Receiver::new(G2_1);
    let seen = feed(
        &mut scan,
        &[rx_buffer(
            &Desc {
                d3: CRC32_ERR,
                long: false,
                ..mgnt(1)
            },
            &beacon(1),
        )],
    );
    assert!(frames(&seen)[0].fcs_error);
    assert_eq!(scan.counts.dropped_fcs, 0);
}

#[test]
fn decrypted_needs_hw_dec_without_sw_dec_or_icv() {
    for (d3, decrypted) in [
        (HW_DEC, true),
        (HW_DEC | SW_DEC, false),
        (HW_DEC | ICV_ERR, false),
        (0, false),
        (SW_DEC, false),
        (ICV_ERR, false),
    ] {
        let icv = d3 & ICV_ERR != 0;
        // The scan delivers every beacon with its flags.
        let mut rx = Receiver::new(G2_1);
        let seen = feed(
            &mut rx,
            &[rx_buffer(
                &Desc {
                    d3,
                    long: false,
                    ..mgnt(1)
                },
                &beacon(1),
            )],
        );
        let f = frames(&seen);
        assert_eq!(
            (f[0].decrypted, f[0].icv_error, f[0].fcs_error),
            (decrypted, icv, icv),
            "d3 {d3:#x}"
        );
        // The station: a protected data frame, an ICV failure dropped.
        let mut protected = frame(0x08, 1);
        protected[1] = 0x40;
        let mut rx = station(G2_1);
        let seen = feed(&mut rx, &[rx_buffer(&Desc { d3, ..data(1) }, &protected)]);
        let f = frames(&seen);
        if icv {
            assert!(f.is_empty(), "d3 {d3:#x}");
            assert_eq!(rx.counts.dropped_fcs, 1);
        } else {
            assert_eq!(
                (f[0].decrypted, f[0].icv_error),
                (decrypted, false),
                "d3 {d3:#x}"
            );
            assert_eq!(f[0].mpdu, protected[..protected.len() - 4]);
        }
    }
}

#[test]
fn station_mode_counts_frames_shorter_than_their_header() {
    let mut rx = station(G2_1);
    let plain = frame(0x08, 1);
    let mut qos = frame(0x88, 1);
    qos.splice(24..24, [0, 0]);
    // ACK: frame control, duration, RA; then the FCS.
    let ack = [0xD4, 0, 0, 0, 2, 0x11, 0x22, 0x33, 0x44, 0x55, 1, 2, 3, 4];
    let short = |p: &[u8]| {
        rx_buffer(
            &Desc {
                long: false,
                ..mgnt(1)
            },
            p,
        )
    };
    let seen = feed(
        &mut rx,
        &[
            short(&[]),
            short(&plain[..1]),
            short(&plain[..27]),
            short(&qos[..29]),
            short(&ack[..13]),
        ],
    );
    assert!(seen.is_empty(), "{seen:?}");
    assert_eq!(rx.counts.short_frames, 5);
    // Exactly a header and an FCS each: delivered without the FCS. The
    // ACK's whole 10 bytes are under mac80211's 16 (`should_drop_frame`).
    let seen = feed(
        &mut rx,
        &[short(&plain[..28]), short(&qos[..30]), short(&ack)],
    );
    let f = frames(&seen);
    assert_eq!(f.len(), 2, "{seen:?}");
    assert_eq!(f[0].mpdu, plain[..24]);
    assert_eq!(f[1].mpdu, qos[..26]);
    assert_eq!(rx.counts.short_frames, 6);
}

/// A control frame with frame-control byte `fc0`, `len` bytes before its
/// FCS: a 16-byte header (RA, TA), then the BAR control and starting
/// sequence or zeros.
fn control(fc0: u8, len: usize) -> Vec<u8> {
    let mut b = vec![fc0, 0, 0x05, 0xC0];
    b.extend_from_slice(&[0x02, 0x11, 0x22, 0x33, 0x44, 0x55]);
    b.extend_from_slice(&[0x02, 0x03, 0x04, 0x05, 0x06, 0x07]);
    b.resize(len, 0);
    b.extend_from_slice(&FCS);
    b
}

#[test]
fn station_mode_drops_control_frames_but_bar_and_ps_poll() {
    // mac80211's should_drop_frame: under 16 bytes without the FCS, or a
    // control frame other than PS-Poll and BlockAckReq.
    let mut rx = station(G2_1);
    let short = |p: &[u8]| {
        rx_buffer(
            &Desc {
                long: false,
                ..mgnt(1)
            },
            p,
        )
    };
    let seen = feed(
        &mut rx,
        &[
            // RTS, Block Ack, CTS-to-self padded to 16, a control wrapper.
            short(&control(0xB4, 16)),
            short(&control(0x94, 24)),
            short(&control(0xC4, 16)),
            short(&control(0x74, 24)),
            // PS-Poll and BlockAckReq go on.
            short(&control(0xA4, 16)),
            short(&control(0x84, 20)),
        ],
    );
    let kept: Vec<u8> = frames(&seen).iter().map(|f| f.mpdu[0]).collect();
    assert_eq!(kept, [0xA4, 0x84], "{seen:?}");
    assert_eq!(rx.counts.other_frames, 4);
    assert_eq!(rx.counts.short_frames, 0);
    // Under 16 bytes: a CTS (10) and an extension frame (12), counted short.
    let mut cts = control(0xC4, 10);
    cts.truncate(10);
    cts.extend_from_slice(&FCS);
    let mut extension = vec![0x0C, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    extension.extend_from_slice(&FCS);
    let seen = feed(&mut rx, &[short(&cts), short(&extension)]);
    assert!(seen.is_empty(), "{seen:?}");
    assert_eq!(rx.counts.short_frames, 2);
    assert_eq!(rx.counts.other_frames, 4);
    // A scan still counts every control frame as another frame, as M2c.
    let mut scan = Receiver::new(G2_1);
    assert!(feed(&mut scan, &[short(&control(0xA4, 16))]).is_empty());
    assert_eq!(scan.counts.other_frames, 1);
}

#[test]
fn set_tag_moves_the_tag_as_a_channel_switch() {
    let g2_11 = Tag {
        band: HwBand::G2,
        channel: 11,
    };
    let mut rx = station(G2_1);
    rx.set_tag(g2_11);
    rx.set_tag(G5_36);
    assert_eq!(rx.tag(), G5_36);
    let seen = feed(&mut rx, &[rx_buffer(&mgnt(1), &frame(0xB0, 1))]);
    assert_eq!(frames(&seen)[0].tag, G5_36);
    // `rtw89_assign_entity_chan` records the channel it leaves, so a CCK
    // beacon tagged 5 GHz takes 2.4 GHz channel 11, the last one left.
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&Desc { rate: 0, ..mgnt(2) }, &beacon(2)),
            rx_buffer(&Desc { rate: 0, ..stat(2) }, &phy_status(0x70, 0x60, 0)),
        ],
    );
    assert_eq!(frames(&seen)[0].tag, g2_11);
    // A scan still moves it, and a tune afterwards sets it again.
    feed(&mut rx, &[scanofld(c2h::SCAN_ENTER_CH, 0, 6)]);
    assert_eq!(
        rx.tag(),
        Tag {
            band: HwBand::G2,
            channel: 6
        }
    );
    rx.set_tag(G5_36);
    let seen = feed(&mut rx, &[rx_buffer(&data(3), &frame(0x08, 3))]);
    assert_eq!(frames(&seen)[0].tag, G5_36);
}

#[test]
fn bcnfltr_rpt_reaches_the_sink() {
    let c2h_desc = Desc {
        pkt_type: 10,
        long: false,
        ..mgnt(0)
    };
    let report: Vec<u8> = [1u32 | 1 << 2 | 0xD << 8, 12, 0x0028_0000]
        .iter()
        .flat_map(|w| w.to_le_bytes())
        .collect();
    let mut rx = station(G2_1);
    let seen = feed(
        &mut rx,
        &[
            rx_buffer(&c2h_desc, &report),
            rx_buffer(&c2h_desc, &report[..11]),
        ],
    );
    assert_eq!(
        seen,
        [Seen::C2h(C2h::BeaconFilter(c2h::BcnFltr {
            macid: 0,
            kind: c2h::BCNFLTR_BEACON_LOSS,
            event: 0,
            ma: 0x28
        }))]
    );
    assert_eq!(rx.counts.c2h_skipped, 1);
    assert_eq!(rx.tag(), G2_1);
}

#[test]
fn station_mode_never_panics_on_any_truncation() {
    let mut protected = frame(0x88, 1);
    protected[1] = 0x40;
    let samples = [
        rx_buffer(&mgnt(1), &beacon(1)),
        rx_buffer(&data(1), &protected),
        rx_buffer(&stat(1), &phy_status(0x70, 0x60, 0x30)),
        vec![0xFF; 300],
    ];
    for s in &samples {
        for cut in 0..=s.len() {
            for b in [&s[..cut], &s[cut..]] {
                let mut rx = station(G2_1);
                let _ = rx.buffer(&s[..], &mut |_| {});
                let mut f = b.to_vec();
                if f.len() >= 4 {
                    let write = 0xC000 | (f.len() as u32 & 0x3FFF);
                    f[..4].copy_from_slice(&write.to_le_bytes());
                }
                let _ = rx.buffer(&f, &mut |_| {});
                assert!(rx.pending() <= MAX_PENDING);
            }
        }
    }
}

#[test]
fn station_frames_carry_what_the_pn_check_reads() {
    // rtw89_core_skb_pn_valid reads HW_DEC and A1_MATCH (d3 b2, b0), and of
    // a long descriptor the MAC ID (d5 [23:16]) and the security type (d7
    // [20:17]).
    let mut rx = station(G2_1);
    let mut b = rx_buffer(
        &Desc {
            d3: HW_DEC | 1,
            ..data(1)
        },
        &frame(0x88, 1),
    );
    b[24..28].copy_from_slice(&0x0012_0000u32.to_le_bytes());
    b[32..36].copy_from_slice(&(6u32 << 17).to_le_bytes());
    let seen = feed(&mut rx, &[b]);
    let f = frames(&seen);
    assert_eq!(
        (f[0].hw_dec, f[0].a1_match, f[0].mac_id, f[0].sec_type),
        (true, true, 0x12, 6)
    );
    assert!(f[0].decrypted);
    // SW_DEC: decrypted on the host, yet HW_DEC as the descriptor has it.
    let seen = feed(
        &mut rx,
        &[rx_buffer(
            &Desc {
                d3: HW_DEC | SW_DEC,
                ..data(1)
            },
            &frame(0x88, 2),
        )],
    );
    let f = frames(&seen);
    assert_eq!(
        (f[0].hw_dec, f[0].decrypted, f[0].a1_match),
        (true, false, false)
    );
}

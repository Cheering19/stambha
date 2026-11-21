//! The live chip through the chip-neutral station interface (M3 Task 11):
//! the scripted [`Firmware`] behind RXQ and CH12, and [`Air`] behind the
//! data and management channels and RPQ.
use super::*;
use crate::Mac;
use crate::wifi::realtek::ax::assoc::{HtPeer, edca_param, qos_defaults};
use crate::wifi::realtek::ax::pci::TxChannel;
use crate::wifi::realtek::ax::pci::tests::rpp;
use crate::wifi::realtek::ax::station::chip::{
    Association, Chip, Data, HardwareKey, Incoming, Management, Received as Heard, Token,
};
use crate::wifi::realtek::ax::tests::firmware::{Air, Sent, rx_buffer};
use crate::wifi::realtek::ax::txd::{self, RateCaps, TxSec};
use crate::wifi::{Band, Channel};
use alloc::string::{String, ToString};

/// A live chip whose firmware `script` sets up, with [`Air`] on its TX side.
fn on_air(script: &dyn Fn(&mut Firmware)) -> (Live<RegisterModel>, Arc<Mutex<Model>>, Air) {
    let (mut live, m) = live_hearing(script);
    let air = Air::install(live.io_mut(), &m);
    (live, m, air)
}

/// A 2.4 or 5 GHz channel as the station layer names it.
fn channel(band: HwBand, number: u8) -> Channel {
    let chan = Chan::new20(band, number).unwrap();
    Channel {
        band: match band {
            HwBand::G2 => Band::Ghz2,
            _ => Band::Ghz5,
        },
        number: u16::from(number),
        frequency_mhz: chan.freq,
    }
}

/// Every event one [`Chip::poll`] hands on.
fn poll(live: &mut Live<RegisterModel>) -> Vec<Incoming> {
    let mut seen = Vec::new();
    Chip::poll(live, &mut |e| seen.push(e)).unwrap();
    seen
}

/// An open-system authentication request from the chip to [`AP`] with
/// sequence number `seq`.
fn auth(seq: u16) -> Vec<u8> {
    let mut f = vec![0xB0, 0x00, 0, 0];
    f.extend_from_slice(&AP);
    f.extend_from_slice(&MAC);
    f.extend_from_slice(&AP);
    f.extend_from_slice(&(seq << 4).to_le_bytes());
    f.extend_from_slice(&[0, 0, 1, 0, 0, 0]);
    f
}

/// A data frame from the chip to [`AP`] (ToDS), sequence number `seq`,
/// whose body is `n` repeated.
fn data_frame(seq: u16, n: u8) -> Vec<u8> {
    let mut f = vec![0x08, 0x01, 0, 0];
    f.extend_from_slice(&AP);
    f.extend_from_slice(&MAC);
    f.extend_from_slice(&AP);
    f.extend_from_slice(&(seq << 4).to_le_bytes());
    f.extend_from_slice(&[n; 20]);
    f
}

/// [`data_frame`] as QoS data of `tid`, its QoS control after the header.
fn qos_frame(seq: u16, tid: u8, n: u8) -> Vec<u8> {
    let mut f = data_frame(seq, n);
    f[0] = 0x88;
    f.splice(24..24, [tid, 0]);
    f
}

/// [`qos_frame`] as the CCMP strip leaves a frame under a key: Protected,
/// no CCMP header.
fn protected_frame(seq: u16, tid: u8, n: u8) -> Vec<u8> {
    let mut f = qos_frame(seq, tid, n);
    f[1] |= 0x40;
    f
}

fn data(bytes: &[u8], tid: u8) -> Data<'_> {
    Data {
        bytes,
        tid,
        eapol: false,
        supported_rates: 0xFFF,
        key: None,
    }
}

fn token(ch: TxChannel, page: u8) -> Token {
    Token((ch.ring() as u16) << 8 | u16::from(page))
}

/// The association with [`AP`] on `ch`, legacy rates `legacy`, AID 1.
fn joined(ch: Channel, legacy: u16, ht: Option<HtPeer>) -> Association {
    Association {
        bssid: Mac(AP),
        aid: 1,
        channel: ch,
        beacon_interval: 100,
        dtim_period: 1,
        legacy,
        ht,
        edca: None,
        short_slot: true,
    }
}

/// The CH12 packet of cat `cat`, `class`, `func` and `seq`, its
/// descriptor included.
fn h2c_of(m: &Arc<Mutex<Model>>, cat: u32, class: u8, func: u8, seq: u8) -> Vec<u8> {
    let state = m.lock().unwrap();
    for n in 0..TX_ENTRIES {
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
        if named == (cat, u32::from(class), u32::from(func), u32::from(seq)) {
            return device_read(&state, at, 16 + (hdr1 & 0x3FFF) as usize);
        }
    }
    panic!("no H2C {cat}/{class}/{func} seq {seq} in the CH12 slots");
}

/// The TX kicks of TX ring IDX `idx` in a trace.
fn tx_kicks(t: &[Op], idx: u32) -> usize {
    t.iter()
        .filter(|o| matches!(o, Op::W16(r, _) if *r == idx))
        .count()
}

#[test]
fn transmit_management_kicks_ch8_and_reports_the_ack() {
    let (mut live, _m, mut air) = on_air(&|_| {});
    let frame = auth(1);
    let before = live.io().trace.len();
    let sent = Chip::transmit_management(
        &mut live,
        &Management {
            bytes: &frame,
            basic_rates: 0,
        },
    );
    // CH8's first WD page, kicked through 0x1078.
    let first = token(TxChannel::Ch8, 0);
    assert_eq!(sent, Ok(Some(first)));
    assert_eq!(first, Token(0x0800));
    assert_eq!(live.io().trace[before..].last(), Some(&Op::W16(0x1078, 1)));
    // Its DONE report: acknowledged.
    assert_eq!(
        poll(&mut live),
        [Incoming::Transmitted {
            token: first,
            acknowledged: true
        }]
    );
    assert!(poll(&mut live).is_empty());
    // The device read the WD head the encoder built and the frame itself.
    let (_, head) = txd::management(&frame, HwBand::G2, 0).unwrap();
    assert_eq!(
        air.sent(),
        [Sent {
            ch: TxChannel::Ch8,
            head,
            page: 0,
            bytes: frame.clone(),
        }]
    );
    // RETRY_LIMIT (1): not acknowledged. The page came back, so the next
    // frame takes it again.
    air.drop_tx(1);
    let again = Chip::transmit_management(
        &mut live,
        &Management {
            bytes: &auth(2),
            basic_rates: 0,
        },
    );
    assert_eq!(again, Ok(Some(first)));
    assert_eq!(
        poll(&mut live),
        [Incoming::Transmitted {
            token: first,
            acknowledged: false
        }]
    );
    assert_eq!(air.sent().len(), 2);
}

#[test]
fn transmit_data_maps_tid_to_its_channel() {
    let (mut live, _m, air) = on_air(&|_| {});
    for (tid, ch, idx) in [(0, TxChannel::Ach0, 0x1058), (7, TxChannel::Ach3, 0x1064)] {
        let frame = qos_frame(u16::from(tid), tid, tid);
        let before = live.io().trace.len();
        let sent = Chip::transmit_data(&mut live, &data(&frame, tid));
        assert_eq!(sent, Ok(Some(token(ch, 0))));
        assert_eq!(tx_kicks(&live.io().trace[before..], idx), 1);
    }
    // EAPOL goes with BK on the queue of its TID, the unassociated peer
    // with neither STBC nor LDPC.
    let eapol = qos_frame(9, 7, 0x88);
    let frame = Data {
        eapol: true,
        ..data(&eapol, 7)
    };
    assert_eq!(
        Chip::transmit_data(&mut live, &frame),
        Ok(Some(token(TxChannel::Ach3, 1)))
    );
    poll(&mut live);
    let sent = air.sent();
    let chs: Vec<TxChannel> = sent.iter().map(|s| s.ch).collect();
    assert_eq!(chs, [TxChannel::Ach0, TxChannel::Ach3, TxChannel::Ach3]);
    let caps = RateCaps {
        stbc: false,
        ldpc: false,
    };
    let (_, head) = txd::data(&eapol, 7, true, HwBand::G2, 0xFFF, caps, None).unwrap();
    assert_eq!(sent[2].head, head);
    assert_eq!(sent[2].bytes, eapol);
}

#[test]
fn poll_drains_rpq_before_rxq() {
    let (mut live, _m, _air) = on_air(&|_| {});
    let frame = auth(1);
    Chip::transmit_management(
        &mut live,
        &Management {
            bytes: &frame,
            basic_rates: 0,
        },
    )
    .unwrap();
    let before = live.io().trace.len();
    poll(&mut live);
    let t = &live.io().trace[before..];
    // RPQ (rtw89_pci_poll_rpq_dma), CH8's TX reclaim, then RXQ.
    let rpq = first_of(t, |o| *o == Op::R32(0x121C));
    let reclaim = after(t, rpq, |o| *o == Op::R32(0x1078));
    let rxq = after(t, reclaim, |o| *o == Op::R32(0x1218));
    assert!(rpq < reclaim && reclaim < rxq);
    // A ring with nothing outstanding is not read.
    for idx in [0x1058, 0x105C, 0x1060, 0x1064] {
        assert!(!t.contains(&Op::R32(idx)), "{idx:#x}");
    }
}

#[test]
fn tune_sets_channel_then_tx_power() {
    let (mut live, _m, _air) = on_air(&|_| {});
    let tuned = live.io().trace.len();
    Chip::tune(&mut live, channel(HwBand::G5, 36)).unwrap();
    let t = &live.io().trace[tuned..];
    let enter = first_of(t, |o| matches!(o, Op::W32(0xC398, _)));
    let window = txpwr_window(t, enter);
    assert!(same_access(&txpwr_alone(g5(36)), &t[window.clone()]));
    assert_eq!(
        t[window.end..window.end + 2],
        [Op::R32(0xC000), Op::W32(0xCE40, 0x2B)]
    );
    assert_eq!(live.chan(), g5(36));
    assert_eq!(live.receiver().tag().channel, 36);
    // A channel that is no 20 MHz 2.4 or 5 GHz one, or whose frequency is
    // not its number's, is refused before any access, named by its step.
    let before = live.io().trace.len();
    for wrong in [
        Channel {
            band: Band::Ghz6,
            number: 1,
            frequency_mhz: 5955,
        },
        Channel {
            frequency_mhz: 5200,
            ..channel(HwBand::G5, 36)
        },
    ] {
        let refused = Chip::tune(&mut live, wrong).unwrap_err();
        assert!(refused.starts_with("set channel: "), "{refused}");
    }
    assert_eq!(live.io().trace.len(), before);
    // A failure names its step: here set_txpwr's (path A's SAR ceiling).
    live.io_mut().fail_reads_of(0x156CC);
    let failed = Chip::tune(&mut live, channel(HwBand::G2, 6)).unwrap_err();
    assert_eq!(failed, alloc::format!("tx power: {READ_FAILED}"));
}

#[test]
fn set_bssid_sends_the_pre_auth_cam_and_suspends_dig() {
    let (mut live, m, _air) = on_air(&|_| {});
    let before = live.io().trace.len();
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    // Seq 17: the address CAM with the BSSID, NO_LINK.
    let cam = H2c::at(17)
        .addr_cam_with(&AddrCam::with_bssid(MAC, AP), 0, 0)
        .1;
    assert_eq!(h2c_of(&m, 1, 6, 0, 17)[24..], cam[24..]);
    // Then DIG suspended: BB 0x481C [10:6] and [29] cleared.
    let t = &live.io().trace[before..];
    let dig: Vec<u32> = t
        .iter()
        .filter_map(|o| match o {
            Op::W32(0x1481C, v) => Some(*v),
            _ => None,
        })
        .collect();
    assert_eq!(dig.len(), 2);
    assert!(
        dig.iter().all(|v| v & (0x1F << 6 | 1 << 29) == 0),
        "{dig:x?}"
    );
    let kick = first_of(t, |o| matches!(o, Op::W16(0x1080, _)));
    let first_dig = first_of(t, |o| matches!(o, Op::W32(0x1481C, _)));
    assert!(kick < first_dig);
    assert_eq!(live.bssid(), Some(AP));
}

#[test]
fn associate_takes_the_negotiated_ht() {
    let (mut live, m, _air) = on_air(&|_| {});
    let ch36 = channel(HwBand::G5, 36);
    Chip::tune(&mut live, ch36).unwrap();
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    let ht = HtPeer {
        rx_mask: [0xFF, 0xFF, 0, 0],
        ldpc: false,
        rx_stbc: true,
        sgi20: true,
    };
    Chip::associate(&mut live, &joined(ch36, 0xFF, Some(ht))).unwrap();
    assert!(live.associated());
    // Seq 18-21 EDCA, 22 the CMAC table, 23 join, 24 the CAM, 25 the RA.
    let ra = h2c_of(&m, 2, 1, 0, 25);
    let w = |i: usize| u32::from_le_bytes(ra[24 + 4 * i..28 + 4 * i].try_into().unwrap());
    assert_eq!((w(0), w(1), w(2), w(3)), (0x01B0_000C, 0xFF0F_F010, 0, 0));
    // The AID went into the connected address CAM.
    let cam = H2c::at(24)
        .addr_cam_with(&AddrCam::connected(MAC, AP, 1), 0, 0)
        .1;
    assert_eq!(h2c_of(&m, 1, 6, 0, 24)[24..], cam[24..]);
    // Data now carries the peer's STBC.
    let frame = data_frame(1, 1);
    let air = Air::install(live.io_mut(), &m);
    Chip::transmit_data(&mut live, &data(&frame, 0)).unwrap();
    poll(&mut live);
    let info0 = u32::from_le_bytes(air.sent()[0].head[32..36].try_into().unwrap());
    assert_eq!(info0 & (1 << 12 | 1 << 11), 1 << 12);
}

#[test]
fn an_association_without_wmm_programs_the_qos_defaults() {
    // mac80211 still enables QoS without WMM (`mlme.c:6308-6323`):
    // `ieee80211_set_wmm_default(.., true)` hands rtw89 802.11's defaults,
    // as it does for a set it finds invalid (`mlme.c:4228-4245`).
    let mut invalid = qos_defaults(false);
    invalid[2].ecw_min = 0;
    for edca in [None, Some(invalid)] {
        let (mut live, m, _air) = on_air(&|_| {});
        let ch36 = channel(HwBand::G5, 36);
        Chip::tune(&mut live, ch36).unwrap();
        Chip::set_bssid(&mut live, Mac(AP)).unwrap();
        let a = Association {
            edca,
            ..joined(ch36, 0xFF, None)
        };
        Chip::associate(&mut live, &a).unwrap();
        assert_qos_defaults(&live, &m);
    }
}

/// The EDCA H2Cs of an association on 5 GHz channel 36 carried the QoS
/// defaults.
fn assert_qos_defaults(live: &Live<RegisterModel>, m: &Arc<Mutex<Model>>) {
    let defaults = qos_defaults(false);
    // VO, VI, BE, BK with the long slot at seq 18-21, then with the short
    // slot after the port update at 27-30.
    for (n, aci) in [3u8, 2, 0, 1].into_iter().enumerate() {
        for (seq, short) in [(18 + n as u8, false), (27 + n as u8, true)] {
            let param = edca_param(&defaults[usize::from(aci)], short, HwBand::G5);
            let want = H2c::at(seq).edca(aci, param).1;
            assert_eq!(h2c_of(m, 1, 9, 0x0F, seq)[24..], want[24..], "seq {seq}");
        }
    }
    // Spec S10.8's vectors: 5 GHz BE 0x0000A42B, VO 0x002F3222.
    assert_eq!(edca_param(&defaults[0], true, HwBand::G5), 0x0000_A42B);
    assert_eq!(edca_param(&defaults[3], true, HwBand::G5), 0x002F_3222);
    assert_eq!(live.h2c(), H2c::at(31));
}

#[test]
fn associate_tunes_only_off_the_joined_channel() {
    let (mut live, _m, _air) = on_air(&|_| {});
    let ch36 = channel(HwBand::G5, 36);
    Chip::tune(&mut live, ch36).unwrap();
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    // Linux's channel context is the BSS's since the join: no switch.
    let before = live.io().trace.len();
    Chip::associate(&mut live, &joined(ch36, 0xFF, None)).unwrap();
    let t = &live.io().trace[before..];
    assert!(!t.iter().any(|o| matches!(o, Op::W32(0xC398, _))));
    Chip::disassociate(&mut live).unwrap();
    // Another channel than the tuned one: switched first, then programmed.
    Chip::tune(&mut live, ch36).unwrap();
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    let before = live.io().trace.len();
    Chip::associate(&mut live, &joined(channel(HwBand::G5, 40), 0xFF, None)).unwrap();
    let t = &live.io().trace[before..];
    let switch = first_of(t, |o| matches!(o, Op::W32(0xC398, _)));
    let kick = first_of(t, |o| matches!(o, Op::W16(0x1080, _)));
    assert!(switch < kick);
    assert_eq!(live.chan(), g5(40));
}

#[test]
fn disassociate_releases_the_channel_to_the_default() {
    let (mut live, m, _air) = on_air(&|_| {});
    let ch36 = channel(HwBand::G5, 36);
    // Nothing set: nothing to leave, no access.
    let idle = live.io().trace.len();
    Chip::disassociate(&mut live).unwrap();
    assert_eq!(live.io().trace.len(), idle);
    Chip::tune(&mut live, ch36).unwrap();
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    Chip::associate(&mut live, &joined(ch36, 0xFF, None)).unwrap();
    let before = live.io().trace.len();
    Chip::disassociate(&mut live).unwrap();
    assert!(!live.associated());
    assert_eq!(live.bssid(), None);
    // Seq 31-43: S11 with the QoS defaults as the AP's parameters.
    assert_eq!(live.h2c(), H2c::at(44));
    let t = &live.io().trace[before..];
    // Then rtw89_chanctx_ops_unassign_vif's rtw89_set_channel, still on
    // the BSS's channel (chan.c:3483), after the last H2C.
    let last_kick = t
        .iter()
        .rposition(|o| matches!(o, Op::W16(0x1080, _)))
        .unwrap();
    let switch = first_of(t, |o| matches!(o, Op::W32(0xC398, _)));
    assert!(last_kick < switch);
    let window = txpwr_window(t, switch);
    assert!(same_access(&txpwr_alone(g5(36)), &t[window]));
    assert_eq!(live.chan(), g5(36));
    // The context is gone (chan.c:3367-3373): the next restore goes to the
    // default chandef, 2.4 GHz channel 1 (chan.c:614-616).
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    drain_until_ended(&mut live);
    // SAFETY: as above.
    unsafe { live.end_scan() }.unwrap();
    assert_eq!(live.chan(), g2(1));
    assert_eq!(live.receiver().tag().channel, 1);
    // A join that never authenticated releases it too: the AP's station
    // goes from NONE to NOTEXIST, which touches nothing on this chip.
    Chip::tune(&mut live, ch36).unwrap();
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    let mut next = live.h2c();
    let before = live.io().trace.len();
    Chip::disassociate(&mut live).unwrap();
    let t = &live.io().trace[before..];
    // The CAM without the BSSID, then the switch.
    assert_eq!(kicks(t), 1);
    let switch = first_of(t, |o| matches!(o, Op::W32(0xC398, _)));
    assert!(first_of(t, |o| matches!(o, Op::W16(0x1080, _))) < switch);
    let (seq, cleared) = next.addr_cam(MAC, 0, 0);
    assert_eq!(live.h2c(), next);
    assert_eq!(h2c_of(&m, 1, 6, 0, seq)[24..], cleared[24..]);
    // One whose AP authenticated the station, and that never associated:
    // AUTH to NONE's `rtw89_core_sta_link_disconnect` (core.c:5818-5859),
    // the beamformee off, the CMAC table, join info disconnected and the
    // CAM without the link; then the BSSID cleared and the switch.
    Chip::tune(&mut live, ch36).unwrap();
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    Chip::authenticated(&mut live).unwrap();
    let mut next = live.h2c();
    let before = live.io().trace.len();
    Chip::disassociate(&mut live).unwrap();
    let t = &live.io().trace[before..];
    assert_eq!(kicks(t), 4);
    let bfee = first_of(t, |o| matches!(o, Op::W32(0xCD80, _)));
    let first_kick = first_of(t, |o| matches!(o, Op::W16(0x1080, _)));
    let last_kick = t
        .iter()
        .rposition(|o| matches!(o, Op::W16(0x1080, _)))
        .unwrap();
    let switch = first_of(t, |o| matches!(o, Op::W32(0xC398, _)));
    assert!(bfee < first_kick && last_kick < switch);
    // No beacon filter, port or EDCA: none was programmed.
    let (s0, cmac) = next.assoc_cmac_tbl(0, 0, HwBand::G5);
    let (s1, join) = next.join_info(0, 0, false);
    let (s2, cam) = next.addr_cam_with(&AddrCam::with_bssid(MAC, AP), 0, 0);
    let (s3, cleared) = next.addr_cam(MAC, 0, 0);
    assert_eq!(live.h2c(), next);
    for (class, func, seq, want) in [
        (5, 0x0A, s0, cmac),
        (8, 0, s1, join),
        (6, 0, s2, cam),
        (6, 0, s3, cleared),
    ] {
        assert_eq!(
            h2c_of(&m, 1, class, func, seq)[24..],
            want[24..],
            "seq {seq}"
        );
    }
    assert_eq!(live.bssid(), None);
    // The next join starts unauthenticated again.
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    let before = live.io().trace.len();
    Chip::disassociate(&mut live).unwrap();
    assert_eq!(kicks(&live.io().trace[before..]), 1);
}

#[test]
fn authenticated_needs_a_bssid_and_touches_nothing() {
    let (mut live, _m, _air) = on_air(&|_| {});
    let before = live.io().trace.len();
    assert!(Chip::authenticated(&mut live).is_err());
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    let set = live.io().trace.len();
    assert!(set > before);
    Chip::authenticated(&mut live).unwrap();
    assert_eq!(live.io().trace.len(), set);
    // Another BSSID is another AP's station, not authenticated yet: leaving
    // it only clears the BSSID.
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    let before = live.io().trace.len();
    Chip::disassociate(&mut live).unwrap();
    assert_eq!(kicks(&live.io().trace[before..]), 1);
}

#[test]
fn a_failed_reclaim_still_drains_rxq_and_says_so() {
    let frame = rx_buffer(
        0,
        false,
        0,
        3,
        &mgnt_frame(0x08, MAC, AP, b"m1", None, 3)[4 + 32..],
    );
    let (mut live, _m, _air) = on_air(&move |fw| fw.after(6, 0, vec![frame.clone()]));
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    let request = auth(1);
    let mgmt = Management {
        bytes: &request,
        basic_rates: 0,
    };
    assert!(
        Chip::transmit_management(&mut live, &mgmt)
            .unwrap()
            .is_some()
    );
    // CH8's index cannot be read: its TXBD stays outstanding.
    live.io_mut().fail_reads_of(0x1078);
    let (mut frames, mut reports) = (0, 0);
    for _ in 0..4 {
        let polled = Chip::poll(&mut live, &mut |e| match e {
            Incoming::Frame(_) => frames += 1,
            Incoming::Transmitted { .. } => reports += 1,
            _ => {}
        });
        assert_eq!(polled, Err(READ_FAILED.to_string()));
    }
    // RPQ before the reclaim and RXQ after it went on: the frame's report
    // and the AP's frame came, and the chip is still on.
    assert_eq!((frames, reports), (1, 1));
    assert!(!Chip::diagnostic(&live).contains("powered off"));
}

#[test]
fn a_malformed_rxq_buffer_fails_the_poll_after_the_scans_end() {
    // After END_SCAN, a buffer too short for its descriptor: the drain that
    // brings the end fails on it, and the end is still handed on.
    let (mut live, _m, _air) = on_air(&|fw| fw.after_end(vec![vec![0x08, 0xC0, 0, 0, 0, 0, 0, 0]]));
    let list = [channel(HwBand::G2, 1), channel(HwBand::G2, 6)];
    Chip::begin_scan(&mut live, &list).unwrap();
    for _ in 0..100 {
        let mut seen = Vec::new();
        let polled = Chip::poll(&mut live, &mut |e| seen.push(e));
        if let Err(error) = polled {
            let short = AxError::Rx("RX buffer shorter than its descriptor");
            assert_eq!(error, short.to_string());
            assert_eq!(seen.last(), Some(&Incoming::ScanEnded { completed: true }));
            assert!(!Chip::diagnostic(&live).contains("powered off"));
            Chip::end_scan(&mut live).unwrap();
            assert!(poll(&mut live).is_empty());
            return;
        }
        assert!(!seen.iter().any(|e| matches!(e, Incoming::ScanEnded { .. })));
    }
    panic!("the tail never failed");
}

#[test]
fn management_leaves_at_the_lowest_basic_rate_of_the_tuned_band() {
    let (mut live, _m, air) = on_air(&|_| {});
    let rate = |s: &Sent| (u32::from_le_bytes(s.head[28..32].try_into().unwrap()) >> 16) & 0x1FF;
    let send = |live: &mut Live<RegisterModel>, basic: u16| {
        Chip::transmit_management(
            live,
            &Management {
                bytes: &auth(1),
                basic_rates: basic,
            },
        )
        .unwrap()
        .unwrap();
        poll(live);
    };
    // 2.4 GHz: CCK1 with no basic rate known, else the lowest basic one
    // (here 5.5 Mb/s, bit 2) above CCK1.
    send(&mut live, 0);
    send(&mut live, 0x0FC);
    Chip::tune(&mut live, channel(HwBand::G5, 36)).unwrap();
    // 5 GHz: OFDM6, and OFDM 12 Mb/s (bit 2) above it.
    send(&mut live, 0);
    send(&mut live, 0x054);
    let rates: Vec<u32> = air.sent().iter().map(rate).collect();
    assert_eq!(rates, [0x0, 0x2, 0x4, 0x6]);
}

#[test]
fn transmit_waits_while_a_scan_runs() {
    // `ieee80211_stop_queues` from the scan start to its completion
    // (fw.c:10419, :10460): frames wait, nothing is kicked.
    let (mut live, _m, air) = on_air(&|_| {});
    Chip::begin_scan(&mut live, &[channel(HwBand::G2, 1), channel(HwBand::G2, 6)]).unwrap();
    let frame = auth(1);
    let mgmt = Management {
        bytes: &frame,
        basic_rates: 0,
    };
    let body = data_frame(1, 1);
    let before = live.io().trace.len();
    assert_eq!(Chip::transmit_management(&mut live, &mgmt), Ok(None));
    assert_eq!(Chip::transmit_data(&mut live, &data(&body, 0)), Ok(None));
    assert_eq!(live.io().trace.len(), before);
    let mut ended = false;
    for _ in 0..100 {
        ended |= poll(&mut live)
            .iter()
            .any(|e| matches!(e, Incoming::ScanEnded { .. }));
        if ended {
            break;
        }
    }
    assert!(ended);
    // Ended but not restored: still waiting.
    assert_eq!(Chip::transmit_management(&mut live, &mgmt), Ok(None));
    Chip::end_scan(&mut live).unwrap();
    assert!(
        Chip::transmit_management(&mut live, &mgmt)
            .unwrap()
            .is_some()
    );
    poll(&mut live);
    assert_eq!(air.sent().len(), 1);
}

/// RX buffer `buffer` with RX descriptor dword 3 ORed with `d3`.
fn with_d3(mut buffer: Vec<u8>, d3: u32) -> Vec<u8> {
    let word = u32::from_le_bytes(buffer[16..20].try_into().unwrap()) | d3;
    buffer[16..20].copy_from_slice(&word.to_le_bytes());
    buffer
}

#[test]
fn a_chip_scan_reports_its_end_once_after_its_frames() {
    let (mut live, _m, _air) = on_air(&|fw| {
        fw.beacon(1, HOME, b"Home", 0x70);
        // A beacon failing its CRC on 5 GHz 36: a scan hands it on flagged.
        fw.buffers(
            2,
            vec![with_d3(
                mgnt_frame(0x80, BROADCAST, OFFICE, b"Bad", None, 6),
                1 << 9,
            )],
        );
    });
    let list = [
        channel(HwBand::G2, 1),
        channel(HwBand::G2, 6),
        channel(HwBand::G5, 36),
    ];
    Chip::begin_scan(&mut live, &list).unwrap();
    let mut seen = Vec::new();
    for _ in 0..100 {
        seen.extend(poll(&mut live));
        if seen.iter().any(|e| matches!(e, Incoming::ScanEnded { .. })) {
            break;
        }
    }
    let ch6 = channel(HwBand::G2, 6);
    let ch36 = channel(HwBand::G5, 36);
    let heard: Vec<(Channel, Option<i8>, bool)> = seen
        .iter()
        .filter_map(|e| match e {
            Incoming::Frame(f) => Some((f.channel, f.signal_dbm, f.fcs_error)),
            _ => None,
        })
        .collect();
    assert_eq!(heard, [(ch6, Some(-54), false), (ch36, None, true)]);
    // The end comes last, once.
    assert_eq!(seen.last(), Some(&Incoming::ScanEnded { completed: true }));
    for _ in 0..3 {
        assert!(poll(&mut live).is_empty());
    }
    Chip::end_scan(&mut live).unwrap();
    assert!(!live.scanning());
    // The next scan reports its own end.
    Chip::begin_scan(&mut live, &list[..1]).unwrap();
    let mut ends = 0;
    for _ in 0..100 {
        ends += poll(&mut live)
            .iter()
            .filter(|e| matches!(e, Incoming::ScanEnded { .. }))
            .count();
    }
    assert_eq!(ends, 1);
}

#[test]
fn a_scan_past_its_budget_ends_not_completed() {
    let (mut live, _m, _air) = on_air(&|fw| fw.no_end());
    let list = [channel(HwBand::G2, 1), channel(HwBand::G2, 6)];
    Chip::begin_scan(&mut live, &list).unwrap();
    let begun = live.io().now;
    for _ in 0..20 {
        assert!(poll(&mut live).is_empty());
    }
    live.io_mut().now = begun + scan_budget_us(2);
    assert_eq!(poll(&mut live), [Incoming::ScanEnded { completed: false }]);
    assert_eq!(live.scan_state(), Some(ScanEnd::Stopped { ended: false }));
    Chip::end_scan(&mut live).unwrap();
}

#[test]
fn poll_hands_on_a_station_frame_with_its_channel_and_status() {
    // A protected data frame from the AP that the hardware decrypted
    // (HW_DEC), right after the BSSID's address CAM.
    let mut body = mgnt_frame(0x08, MAC, AP, b"m1", None, 3);
    body[4 + 32 + 1] = 0x42;
    let frame = with_d3(body, 1 << 2);
    let (mut live, _m, _air) = on_air(&move |fw| fw.after(6, 0, vec![frame.clone()]));
    Chip::tune(&mut live, channel(HwBand::G5, 36)).unwrap();
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    // RXQ gives one batch a read: the acks still queued go first.
    let mut seen = Vec::new();
    for _ in 0..4 {
        seen.extend(poll(&mut live));
    }
    let [Incoming::Frame(heard)] = &seen[..] else {
        panic!("{seen:?}");
    };
    let Heard {
        bytes,
        channel: ch,
        signal_dbm,
        tsf: _,
        decrypted,
        icv_error,
        fcs_error,
    } = heard;
    assert_eq!(bytes[..2], [0x08, 0x42]);
    assert_eq!(bytes[16..22], AP);
    assert_eq!(*ch, channel(HwBand::G5, 36));
    assert_eq!(
        (*signal_dbm, *decrypted, *icv_error, *fcs_error),
        (None, true, false, false)
    );
}

#[test]
fn a_failed_rpq_poll_still_drains_rxq_and_says_so() {
    let frame = rx_buffer(
        0,
        false,
        0,
        3,
        &mgnt_frame(0x08, MAC, AP, b"m1", None, 3)[4 + 32..],
    );
    let (mut live, _m, _air) = on_air(&move |fw| fw.after(6, 0, vec![frame.clone()]));
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    live.io_mut().fail_reads_of(0x121C);
    let mut frames = 0;
    for _ in 0..4 {
        let polled = Chip::poll(&mut live, &mut |e| {
            if matches!(e, Incoming::Frame(_)) {
                frames += 1;
            }
        });
        assert_eq!(polled, Err(READ_FAILED.to_string()));
    }
    // RXQ went on: the frame came, and the chip is still on.
    assert_eq!(frames, 1);
    assert!(!Chip::diagnostic(&live).contains("powered off"));
}

#[test]
fn silent_rpq_stalls_tx_without_loss() {
    let (mut live, _m, mut air) = on_air(&|_| {});
    air.silent_rpq();
    let frames: Vec<Vec<u8>> = (0..30).map(|n| data_frame(n, n as u8)).collect();
    let mut tokens = Vec::new();
    for frame in &frames[..15] {
        let sent = Chip::transmit_data(&mut live, &data(frame, 0)).unwrap();
        tokens.push(sent.expect("room for the first 15"));
    }
    // The 16th finds the ring full: no room, and nothing touched.
    let before = live.io().trace.len();
    assert_eq!(
        Chip::transmit_data(&mut live, &data(&frames[15], 0)),
        Ok(None)
    );
    assert_eq!(live.io().trace.len(), before);
    // While RPQ is silent the device sends, but no page comes back.
    for _ in 0..3 {
        assert!(poll(&mut live).is_empty());
    }
    assert_eq!(air.sent().len(), 15);
    // RPQ speaks: every frame's report, once, each acknowledged.
    air.ack_all();
    let reported: Vec<Incoming> = poll(&mut live);
    let want: Vec<Incoming> = tokens
        .iter()
        .map(|&token| Incoming::Transmitted {
            token,
            acknowledged: true,
        })
        .collect();
    assert_eq!(reported, want);
    for frame in &frames[15..] {
        let sent = Chip::transmit_data(&mut live, &data(frame, 0)).unwrap();
        assert!(sent.is_some());
    }
    assert_eq!(poll(&mut live).len(), 15);
    // All 30 reached the air, in order, each once.
    let payloads: Vec<Vec<u8>> = air.sent().into_iter().map(|s| s.bytes).collect();
    assert_eq!(payloads, frames);
}

#[test]
fn silent_rpq_runs_the_pages_out_without_reusing_one() {
    let (mut live, _m, mut air) = on_air(&|_| {});
    air.silent_rpq();
    let mut tokens = Vec::new();
    for n in 0..40u16 {
        let frame = qos_frame(n, 5, n as u8);
        if let Some(token) = Chip::transmit_data(&mut live, &data(&frame, 5)).unwrap() {
            tokens.push(token);
        }
        assert!(poll(&mut live).is_empty());
    }
    // Every TXBD came back, but no page without its report: 16, each once.
    assert_eq!(tokens.len(), 16);
    let mut pages: Vec<u16> = tokens.iter().map(|t| t.0).collect();
    pages.sort_unstable();
    pages.dedup();
    assert_eq!(pages.len(), 16);
    assert!(tokens.iter().all(|t| t.0 >> 8 == 2), "VI is ACH2");
    assert!(
        Chip::diagnostic(&live).contains("tx sent 16 reported 0 acked 0 full 24"),
        "{}",
        Chip::diagnostic(&live)
    );
    air.ack_all();
    assert_eq!(poll(&mut live).len(), 16);
    let frame = qos_frame(40, 5, 40);
    assert!(
        Chip::transmit_data(&mut live, &data(&frame, 5))
            .unwrap()
            .is_some()
    );
    assert_eq!(air.sent().len(), 16);
}

#[test]
fn skipped_and_stale_reports_are_counted_not_reported() {
    let (mut live, _m, mut air) = on_air(&|_| {});
    // Seq 16 is past the pools, qsel 4 names no channel, and page 3 of CH8
    // was never sent.
    air.report(rpp(16, 0, 0x00));
    air.report(rpp(0, 0, 0x04));
    air.report(rpp(3, 0, 0x12));
    assert!(poll(&mut live).is_empty());
    let d = Chip::diagnostic(&live);
    assert!(d.contains("rpq skipped 2 stale 1"), "{d}");
    // A report repeated for a frame already reported is stale too.
    let frame = auth(1);
    let mgmt = Management {
        bytes: &frame,
        basic_rates: 0,
    };
    Chip::transmit_management(&mut live, &mgmt).unwrap();
    assert_eq!(poll(&mut live).len(), 1);
    Chip::transmit_management(&mut live, &mgmt).unwrap();
    air.silent_rpq();
    poll(&mut live);
    air.report(rpp(0, 0, 0x12));
    air.ack_all();
    // Page 0's own report, then its repeat.
    assert_eq!(poll(&mut live).len(), 1);
    let d = Chip::diagnostic(&live);
    assert!(d.contains("rpq skipped 2 stale 2"), "{d}");
    assert!(!d.contains(':'), "{d}");
}

#[test]
fn the_chip_installs_and_removes_keys() {
    let (mut live, _m, _air) = on_air(&|_| {});
    // An obvious test key.
    let material: [u8; 16] = core::array::from_fn(|i| i as u8);
    let key = |pairwise: bool, index: u8| HardwareKey {
        pairwise,
        index,
        material: &material,
    };
    let unassociated = Chip::add_key(&mut live, &key(true, 0));
    assert_eq!(
        unassociated,
        Err(String::from("a key without an association"))
    );
    let ch1 = channel(HwBand::G2, 1);
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    Chip::associate(&mut live, &joined(ch1, 0xFFF, None)).unwrap();
    // The security CAM entries, the first free each time (spec S12.1).
    assert_eq!(Chip::add_key(&mut live, &key(true, 0)), Ok(0));
    assert_eq!(Chip::add_key(&mut live, &key(false, 1)), Ok(1));
    assert_eq!(Chip::remove_key(&mut live, 0), Ok(()));
    assert_eq!(
        Chip::remove_key(&mut live, 0),
        Err(String::from("no key in that security CAM entry"))
    );
    assert_eq!(Chip::add_key(&mut live, &key(true, 0)), Ok(0));
    // The disassociation deletes what is left first.
    Chip::disassociate(&mut live).unwrap();
    assert!(!live.key_installed(0) && !live.key_installed(1));
}

#[test]
fn keyed_data_goes_under_its_cam_entry_and_pn() {
    let (mut live, _m, air) = on_air(&|_| {});
    let material: [u8; 16] = core::array::from_fn(|i| i as u8);
    let ch1 = channel(HwBand::G2, 1);
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    Chip::associate(&mut live, &joined(ch1, 0xFFF, None)).unwrap();
    let ptk = HardwareKey {
        pairwise: true,
        index: 0,
        material: &material,
    };
    assert_eq!(Chip::add_key(&mut live, &ptk), Ok(0));
    // rtw89_core_tx_update_sec_key: the frame names CAM entry 0, key ID 0
    // and its PN in BODY4/5 (spec S5.3).
    let frame = protected_frame(5, 0, 0x5A);
    let keyed = Data {
        key: Some((0, 0, 0x0605_0403_0201)),
        ..data(&frame, 0)
    };
    assert_eq!(
        Chip::transmit_data(&mut live, &keyed),
        Ok(Some(token(TxChannel::Ach0, 0)))
    );
    poll(&mut live);
    let sent = air.sent();
    let caps = RateCaps {
        stbc: false,
        ldpc: false,
    };
    let sec = TxSec {
        keyid: 0,
        cam: 0,
        pn: 0x0605_0403_0201,
    };
    let (_, head) = txd::data(&frame, 0, false, HwBand::G2, 0xFFF, caps, Some(sec)).unwrap();
    assert_eq!(sent.len(), 1);
    assert_eq!(sent[0].head, head);
    assert_eq!(sent[0].bytes, frame);
    // A key the security CAM does not hold is refused before any access:
    // entry 1 holds nothing, and entry 0 is gone once removed.
    let before = live.io().trace.len();
    let unheld = Err(String::from("a key not in the security CAM"));
    let other = Data {
        key: Some((1, 0, 1)),
        ..data(&frame, 0)
    };
    assert_eq!(Chip::transmit_data(&mut live, &other), unheld.clone());
    assert_eq!(live.io().trace.len(), before);
    Chip::remove_key(&mut live, 0).unwrap();
    assert_eq!(Chip::transmit_data(&mut live, &keyed), unheld);
    // RX Block Ack programs the BA CAM's entry and frees it (M3 Task 16).
    let before = live.io().trace.len();
    assert_eq!(Chip::start_rx_block_ack(&mut live, 0, 0x123, 64), Ok(()));
    assert_eq!(Chip::stop_rx_block_ack(&mut live, 0), Ok(()));
    assert_eq!(kicks(&live.io().trace[before..]), 2);
    assert_eq!(Chip::mac(&live), Mac(MAC));
}

/// The TX rings `rtw89_pci_ops_flush_queues` waits on: every one but CH12,
/// ACH0-7, CH8, CH9, CH10 and CH11 (`pci.h:556-569`), in ring order.
const FLUSHED: [u32; 12] = [
    0x1058, 0x105C, 0x1060, 0x1064, 0x1068, 0x106C, 0x1070, 0x1074, 0x1078, 0x107C, 0x11D0, 0x11D4,
];
/// `R_AX_DLE_EMPTY0` and the queues `dle_is_txq_empty` wants empty
/// (`mac.c:2388-2412`, `reg.h:631-651`).
const DLE_EMPTY0: u32 = 0x8430;
const TXQ_EMPTY: u32 = 0x07FF_079F;

/// The 32-bit reads of a trace.
fn reads32(t: &[Op]) -> Vec<u32> {
    t.iter()
        .filter_map(|o| match o {
            Op::R32(r) => Some(*r),
            _ => None,
        })
        .collect()
}

#[test]
fn a_flush_without_drop_waits_on_the_rings_then_the_mac() {
    let (mut live, _m, _air) = on_air(&|_| {});
    live.io_mut().set32(DLE_EMPTY0, TXQ_EMPTY);
    let before = live.io().trace.len();
    assert_eq!(Chip::flush(&mut live, false), Ok(()));
    let t = &live.io().trace[before..];
    // __pci_flush_txch: each ring's index already at its host's, so one
    // read each; then rtw89_mac_flush_txq's first sample, already empty.
    let mut want = FLUSHED.to_vec();
    want.push(DLE_EMPTY0);
    assert_eq!(reads32(t), want);
    assert_eq!(delays(t), 0);
    assert_eq!(kicks(t), 0);
    // The MAC's queues never empty: read_poll_timeout every 10 ms for
    // 200 ms, one last sample after it, and the flush ends without error
    // as Linux's only logs.
    live.io_mut().set32(DLE_EMPTY0, TXQ_EMPTY & !1);
    let before = live.io().trace.len();
    assert_eq!(Chip::flush(&mut live, false), Ok(()));
    let t = &live.io().trace[before..];
    let samples = reads32(t).iter().filter(|&&r| r == DLE_EMPTY0).count();
    assert!((21..=22).contains(&samples), "{samples}");
    assert!((200_000..=210_000).contains(&delays(t)), "{}", delays(t));
}

#[test]
fn a_flush_waits_60_us_on_a_ring_the_dma_engine_left() {
    // No Air: nothing reads the TXBD, so ACH0's hardware index stays 0
    // behind its host index of 1.
    let (mut live, _m) = live_hearing(&|_| {});
    live.io_mut().set32(DLE_EMPTY0, TXQ_EMPTY);
    let frame = qos_frame(1, 0, 1);
    assert!(matches!(
        Chip::transmit_data(&mut live, &data(&frame, 0)),
        Ok(Some(_))
    ));
    let before = live.io().trace.len();
    assert_eq!(Chip::flush(&mut live, true), Ok(()));
    let t = &live.io().trace[before..];
    let reads = reads32(t);
    assert_eq!(reads.iter().filter(|&&r| r == 0x1058).count(), 60);
    assert!(FLUSHED[1..].iter().all(|r| reads.contains(r)));
    // 60 waits of 1 us between them, then on (pci.c:1401-1407).
    let ach0: u64 = t
        .iter()
        .filter_map(|o| match o {
            Op::Delay(us) => Some(*us),
            _ => None,
        })
        .take(60)
        .sum();
    assert_eq!(ach0, 60);
    // With drop and no AP's station, no PKT_DROP and no MAC wait.
    assert_eq!(kicks(t), 0);
    assert!(!reads.contains(&DLE_EMPTY0));
}

#[test]
fn a_flush_with_drop_drops_the_stations_four_queues() {
    let (mut live, m, _air) = on_air(&|_| {});
    let ch1 = channel(HwBand::G2, 1);
    Chip::set_bssid(&mut live, Mac(AP)).unwrap();
    Chip::associate(&mut live, &joined(ch1, 0xFFF, None)).unwrap();
    let seq = live.h2c();
    let before = live.io().trace.len();
    assert_eq!(Chip::flush(&mut live, true), Ok(()));
    let t = &live.io().trace[before..];
    // __rtw89_drop_packets for the AP's station: PKT_DROP of BE, BK, VI
    // and VO once for macid 0 (rtw89_mac_pkt_drop_sta, mac.c:7255-7278),
    // as the 8852C's firmware lacks NO_PACKET_DROP (fw.c:870); no wait for
    // the MAC's queues (mac80211.c:1132-1135).
    assert_eq!(kicks(t), 4);
    for sel in 0..4u8 {
        let mut h2c = seq;
        for _ in 0..sel {
            h2c.pkt_drop(0, 0, 0);
        }
        let (at, want) = h2c.pkt_drop(sel, 0, 0);
        assert_eq!(h2c_of(&m, 1, 9, 0x1B, at), want);
    }
    assert!(!reads32(t).contains(&DLE_EMPTY0));
    // A flush is refused while a scan has not ended, and on a chip off.
    Chip::disassociate(&mut live).unwrap();
    Chip::begin_scan(&mut live, &[ch1]).unwrap();
    assert_eq!(
        Chip::flush(&mut live, false),
        Err(SCAN_NOT_ENDED.to_string())
    );
}

#[test]
fn a_powered_off_chip_refuses_every_call() {
    let (mut live, _m, _air) = on_air(&|_| {});
    live.io_mut().fail_reads_of(0x156CC);
    assert!(Chip::tune(&mut live, channel(HwBand::G5, 36)).is_err());
    let off = live.io().trace.len();
    let dead = POWERED_OFF.to_string();
    let frame = auth(1);
    let ch = channel(HwBand::G2, 1);
    assert_eq!(Chip::poll(&mut live, &mut |_| {}), Err(dead.clone()));
    assert!(Chip::tune(&mut live, ch).unwrap_err().ends_with(&dead));
    assert_eq!(Chip::begin_scan(&mut live, &[ch]), Err(dead.clone()));
    assert_eq!(Chip::end_scan(&mut live), Err(dead.clone()));
    assert_eq!(Chip::set_bssid(&mut live, Mac(AP)), Err(dead.clone()));
    assert_eq!(Chip::authenticated(&mut live), Err(dead.clone()));
    let mgmt = Management {
        bytes: &frame,
        basic_rates: 0,
    };
    assert_eq!(
        Chip::transmit_management(&mut live, &mgmt),
        Err(dead.clone())
    );
    assert_eq!(
        Chip::transmit_data(&mut live, &data(&data_frame(1, 1), 0)),
        Err(dead.clone())
    );
    assert_eq!(
        Chip::associate(&mut live, &joined(ch, 0xFFF, None)),
        Err(dead.clone())
    );
    assert_eq!(Chip::disassociate(&mut live), Err(dead.clone()));
    let material = [0; 16];
    let key = HardwareKey {
        pairwise: true,
        index: 0,
        material: &material,
    };
    assert_eq!(Chip::add_key(&mut live, &key), Err(dead.clone()));
    assert_eq!(Chip::remove_key(&mut live, 0), Err(dead.clone()));
    assert_eq!(Chip::flush(&mut live, true), Err(dead.clone()));
    assert_eq!(Chip::flush(&mut live, false), Err(dead.clone()));
    assert_eq!(
        Chip::start_rx_block_ack(&mut live, 0, 0, 64),
        Err(dead.clone())
    );
    assert_eq!(Chip::stop_rx_block_ack(&mut live, 0), Err(dead));
    assert_eq!(live.io().trace.len(), off);
    assert!(Chip::diagnostic(&live).ends_with(", powered off"));
}

// The Task 9 review's minors.

#[test]
fn held_frames_past_64_are_dropped_and_counted() {
    // 70 beacons without a PPDU status wait (short descriptors) in
    // ADD_SCANOFLD_CH's wait: the scan keeps them, 64 are held.
    let beacons: Vec<Vec<u8>> = (0..70u8)
        .map(|n| {
            let frame = mgnt_frame(0x80, BROADCAST, [0x02, 0xBB, 0, 0, 0, n], b"x", None, 1);
            rx_buffer(0, false, 0, 1, &frame[4 + 32..])
        })
        .collect();
    let (mut live, _m) = live_hearing(&move |fw| fw.after(9, 0x16, beacons.clone()));
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    assert_eq!(live.held_dropped(), 6);
    let mut first = Vec::new();
    // SAFETY: as above.
    unsafe {
        live.drain(&mut |e| {
            if let Event::Frame(f) = e {
                first.push(f.mpdu[15]);
            }
        })
    }
    .unwrap();
    // The oldest 64 first, in order.
    assert_eq!(first[..64], (0..64).collect::<Vec<u8>>()[..]);
}

#[test]
fn a_data_frame_drained_by_the_scan_start_is_a_station_frame() {
    // A data frame after vif init's last H2C reaches RXQ before the scan's
    // filter: the scan start's address CAM drains it while the receiver
    // still keeps a station's frames.
    let frame = rx_buffer(
        0,
        false,
        0,
        2,
        &mgnt_frame(0x08, MAC, AP, b"before", None, 2)[4 + 32..],
    );
    let (mut live, _m) = live_hearing(&move |fw| fw.after(5, 0x0A, vec![frame.clone()]));
    let others = live.receiver().counts.other_frames;
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    let drains = drain_until_ended(&mut live);
    assert_eq!(drains[0].len(), 1);
    assert_eq!(drains[0][0].mpdu[0], 0x08);
    assert_eq!(live.receiver().counts.other_frames, others);
}

#[test]
fn an_end_scan_in_the_stops_tail_counts_as_ended() {
    // After the stop's acks: three batches, then END_SCAN. With every read
    // costing 200 ms, the stop's 100 ms wait drains two of them, and the
    // tail the third and END_SCAN.
    let filler = rx_buffer(
        0,
        false,
        0,
        4,
        &mgnt_frame(0x08, MAC, AP, b"filler", None, 4)[4 + 32..],
    );
    let (mut live, _m) = live_hearing(&move |fw| {
        fw.no_end();
        fw.end_on_stop();
        for _ in 0..3 {
            fw.after(9, 0x17, vec![filler.clone()]);
        }
    });
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    let begun = live.io().now;
    for _ in 0..20 {
        // SAFETY: as above.
        unsafe { live.drain(&mut |_| {}) }.unwrap();
    }
    live.io_mut().now = begun + scan_budget_us(3);
    live.io_mut().read_cost_us = 200_000;
    // SAFETY: as above.
    unsafe { live.expire_scan(&mut |_| {}) }.unwrap();
    assert_eq!(live.scan_state(), Some(ScanEnd::Stopped { ended: true }));
}

#[test]
fn a_scan_whose_tail_fails_still_ends() {
    // After END_SCAN, a buffer too short for its descriptor.
    let (mut live, _m) = live_hearing(&|fw| fw.after_end(vec![vec![0x08, 0xC0, 0, 0, 0, 0, 0, 0]]));
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    for _ in 0..100 {
        // SAFETY: as above.
        if let Err(error) = unsafe { live.drain(&mut |_| {}) } {
            assert_eq!(error, AxError::Rx("RX buffer shorter than its descriptor"));
            assert_eq!(live.scan_state(), Some(ScanEnd::Ok));
            return;
        }
        assert_eq!(live.scan_state(), None);
    }
    panic!("the tail never failed");
}

#[test]
fn a_failed_stop_ends_the_scan_stopped_without_resending() {
    let (mut live, _m) = live_hearing(&|fw| fw.no_end());
    // SAFETY: as in `start_on`.
    unsafe { live.begin_scan(&three()) }.unwrap();
    let begun = live.io().now;
    live.io_mut().now = begun + scan_budget_us(3);
    // CH12's index cannot be read: the stop's send fails.
    live.io_mut().fail_reads_of(0x1080);
    // SAFETY: as above.
    assert_eq!(unsafe { live.expire_scan(&mut |_| {}) }, Err(READ_FAILED));
    assert_eq!(live.scan_state(), Some(ScanEnd::Stopped { ended: false }));
    let k = kicks(&live.io().trace);
    live.io_mut().now += 10 * scan_budget_us(3);
    // SAFETY: as above.
    unsafe { live.expire_scan(&mut |_| {}) }.unwrap();
    assert_eq!(kicks(&live.io().trace), k);
}

// The Task 10 review's failure tests.

#[test]
fn a_failed_set_bssid_powers_off_without_a_bssid() {
    let (mut live, _m) = live_hearing(&|_| {});
    // DIG's register cannot be read when the join suspends it, after the
    // address CAM took the BSSID.
    live.io_mut().fail_reads_of(0x1481C);
    // SAFETY: as in `start_on`.
    assert_eq!(unsafe { live.set_bssid(AP) }, Err(READ_FAILED));
    assert_eq!(live.bssid(), None);
    assert_powered_off_for_good(live);
}

#[test]
fn a_failed_leave_powers_off_without_a_bssid() {
    let (mut live, _m) = live_hearing(&|_| {});
    // SAFETY: as in `start_on`.
    unsafe { live.set_bssid(AP) }.unwrap();
    let a = association(&live);
    // SAFETY: as above.
    unsafe { live.associate(&a) }.unwrap();
    // The beamformee's register cannot be read when leaving turns it off.
    live.io_mut().fail_reads_of(0xCD80);
    // SAFETY: as above.
    assert_eq!(unsafe { live.leave() }, Err(READ_FAILED));
    assert_eq!(live.bssid(), None);
    assert!(!live.associated());
    assert_powered_off_for_good(live);
}

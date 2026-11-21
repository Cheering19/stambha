extern crate std;
use super::*;
use crate::wifi::realtek::ax::h2c::H2c;
use crate::wifi::realtek::ax::pci::FWCMD_SLOT_BYTES;
use crate::wifi::realtek::ax::pci::tests::{FWCMD, device_read, running};
use crate::wifi::realtek::ax::receive::{Mode, Tag};
use crate::wifi::realtek::ax::tests::firmware::{Firmware, mgnt_frame};
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::{vec, vec::Vec};

/// The plan's test addresses: the local MAC and the AP's BSSID.
const LOCAL: [u8; 6] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
const AP: [u8; 6] = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
const G5_36: Tag = Tag {
    band: HwBand::G5,
    channel: 36,
};
/// The firmware's features from 0.27.129.4, the pinned file's version.
const FULL: Features = Features {
    beacon_filter: true,
    loss_count_v1: true,
};

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

/// 802.11's default WMM parameter set for an AP, in ACI order: BE, BK, VI,
/// VO.
fn wmm() -> [AccessCategory; 4] {
    [
        ac(0, 3, 4, 10, 0),
        ac(1, 7, 4, 10, 0),
        ac(2, 2, 3, 4, 94),
        ac(3, 2, 2, 3, 47),
    ]
}

/// An HT peer with two streams, SGI at 20 MHz and RX STBC, no LDPC.
const HT_2SS: HtPeer = HtPeer {
    rx_mask: [0xFF, 0xFF, 0, 0],
    ldpc: false,
    rx_stbc: true,
    sgi20: true,
};

/// The association of spec S10's vectors: AID 5, beacon interval 100,
/// DTIM period 1, on `band` with every legacy rate of that band.
fn inputs(band: HwBand) -> AssocInputs {
    AssocInputs {
        mac: LOCAL,
        bssid: AP,
        aid: 5,
        band,
        beacon_int: 100,
        dtim: 1,
        peer: RaPeer {
            legacy: if band == HwBand::G2 { 0xFFF } else { 0xFF },
            ht: Some(HT_2SS),
            rssi: 0,
        },
        edca: Some(wmm()),
        short_slot: true,
    }
}

/// The RA H2C's payload words for `ra` (macid 0).
fn ra_words(ra: &RaInfo) -> Vec<u32> {
    let (_, p) = H2c::at(1).ra(0, ra);
    assert_eq!(p.len(), 16 + 8 + 16);
    // cat 2 (OUTSRC) / class 1 (RA) / func 0 (MACIDCFG), no acks.
    assert_eq!(p[16..18], [0x06, 0x00]);
    assert_eq!(p[20..24], [0x18, 0x00, 0x00, 0x00]);
    (0..4)
        .map(|i| u32::from_le_bytes(p[24 + 4 * i..28 + 4 * i].try_into().unwrap()))
        .collect()
}

fn ra(peer: RaPeer, band: HwBand) -> Vec<u32> {
    ra_words(&ra_info(&peer, band))
}

#[test]
fn ra_info_matches_the_linux_vectors() {
    // Spec S10.4's four rows (phy.c:322-503 with phy.c:731-776).
    let rows = [
        (0xFF, Some(HT_2SS), HwBand::G5, 0x01B0_000C, 0xFF0F_F010),
        (0xFFF, Some(HT_2SS), HwBand::G2, 0x01B0_000E, 0xFF0F_F015),
        (0xFF, None, HwBand::G5, 0x0010_0004, 0x0000_0FF0),
        (0xFFF, None, HwBand::G2, 0x0010_0006, 0x0000_0FF5),
    ];
    for (legacy, ht, band, w0, w1) in rows {
        let peer = RaPeer {
            legacy,
            ht,
            rssi: 0,
        };
        assert_eq!(
            ra(peer, band),
            [w0, w1, 0, 0],
            "{legacy:#x} {ht:?} {band:?}"
        );
    }
    // The fields behind the first row.
    let info = ra_info(
        &RaPeer {
            legacy: 0xFF,
            ht: Some(HT_2SS),
            rssi: 0,
        },
        HwBand::G5,
    );
    assert_eq!(
        info,
        RaInfo {
            mode: 0x6,
            ra_mask: 0xFF0F_F010,
            sgi: true,
            ldpc: false,
            stbc: true,
            ss_num: 1,
            init_rate_lv: 0,
        }
    );
}

#[test]
fn ra_info_counts_streams_and_masks_rates_as_linux_does() {
    let peer = |legacy, ht| RaPeer {
        legacy,
        ht,
        rssi: 0,
    };
    // 802.11b alone: CCK mode, no rate mask and no RSSI floor.
    assert_eq!(ra(peer(0xF, None), HwBand::G2), [0x0010_0002, 0xF, 0, 0]);
    // HT with CCK but no OFDM rate: SUBCCK kept, no SUBOFDM.
    assert_eq!(
        ra(peer(0xF, Some(HT_2SS)), HwBand::G2),
        [0x01B0_000A, 0xFF0F_F005, 0, 0]
    );
    // mac80211 counts the non-zero RX mask bytes (sta_info.c:3570-3579), so
    // MCS 0-7 and 16-23 make two streams, and the third stream's rates go
    // (tx_nss 2). LDPC b22, no SGI, no STBC.
    let gap = HtPeer {
        rx_mask: [0xFF, 0, 0xFF, 0],
        ldpc: true,
        rx_stbc: false,
        sgi20: false,
    };
    assert_eq!(
        ra(peer(0xFF, Some(gap)), HwBand::G5),
        [0x0150_000C, 0x000F_F010, 0, 0]
    );
    // Four streams are cut to two; one stream is SS_NUM 0.
    let four = HtPeer {
        rx_mask: [0xFF; 4],
        ..HT_2SS
    };
    assert_eq!(
        ra(peer(0xFF, Some(four)), HwBand::G5),
        [0x01B0_000C, 0xFF0F_F010, 0, 0]
    );
    let one = HtPeer {
        rx_mask: [0xFF, 0, 0, 0],
        ..HT_2SS
    };
    assert_eq!(
        ra(peer(0xFF, Some(one)), HwBand::G5),
        [0x00B0_000C, 0x000F_F010, 0, 0]
    );
    // The MCS bits as the peer gives them, [12+] and [24+].
    let some = HtPeer {
        rx_mask: [0x0F, 0x03, 0, 0],
        ..HT_2SS
    };
    assert_eq!(ra(peer(0xFF, Some(some)), HwBand::G5)[1], 0x0300_F010);
    // 5 GHz is always OFDM, even with no legacy rate.
    assert_eq!(
        ra(peer(0, Some(HT_2SS)), HwBand::G5)[..2],
        [0x01B0_000C, 0xFF0F_F000]
    );
}

#[test]
fn ra_info_applies_linuxs_rssi_floor_and_initial_rate_level() {
    let peer = |legacy, ht, rssi| RaPeer { legacy, ht, rssi };
    // rtw89_phy_ra_mask_rssi(rssi, 0) (phy.c:158-188) halves the average
    // and finds its level in 33, 47, 51, 55, 59, 63, 103; the initial rate
    // level (phy.c:736-757) is > 40: 1, > 20: 2, > 1: 3, else 0.
    // 60: level 5, floor 0x...0F00; level 1.
    assert_eq!(
        ra(peer(0xFF, Some(HT_2SS), 120), HwBand::G5),
        [0x01B4_000C, 0xFF0F_0000, 0, 0]
    );
    // 33: level 1, floor 0x...FFF0, which drops CCK; level 2.
    assert_eq!(
        ra(peer(0xFFF, None, 66), HwBand::G2),
        [0x0018_0006, 0x0000_0FF0, 0, 0]
    );
    // 32: no floor; level 2. 2: level 3. 1: level 0.
    assert_eq!(
        ra(peer(0xFFF, None, 64), HwBand::G2)[..2],
        [0x0018_0006, 0xFF5]
    );
    assert_eq!(
        ra(peer(0xFFF, None, 4), HwBand::G2)[..2],
        [0x001C_0006, 0xFF5]
    );
    assert_eq!(
        ra(peer(0xFFF, None, 3), HwBand::G2)[..2],
        [0x0010_0006, 0xFF5]
    );
    // 127 is past every level: Linux keeps level 0, no floor; level 1.
    assert_eq!(
        ra(peer(0xFFF, None, 255), HwBand::G2)[..2],
        [0x0014_0006, 0xFF5]
    );
    // A floor that leaves nothing is recovered (phy.c:190-199): HT rates
    // first, then the legacy ones.
    let one = HtPeer {
        rx_mask: [0x0F, 0, 0, 0],
        ldpc: false,
        rx_stbc: false,
        sgi20: false,
    };
    assert_eq!(
        ra(peer(0xFF, Some(one), 120), HwBand::G5)[..2],
        [0x0014_000C, 0x0000_F000]
    );
    assert_eq!(
        ra(peer(0x0F, None, 120), HwBand::G5)[..2],
        [0x0014_0004, 0xF0]
    );
    // 802.11b alone takes no floor, but the initial level.
    assert_eq!(
        ra(peer(0xF, None, 120), HwBand::G2)[..2],
        [0x0014_0002, 0xF]
    );
}

#[test]
fn edca_vectors() {
    // Spec S10.8 (mac80211.c:393-431): AIFS = aifsn * slot + SIFS, slot 9
    // or 20, SIFS 10 (2.4 GHz) or 16; ECWMIN [11:8], ECWMAX [15:12], TXOP
    // [26:16].
    let w = wmm();
    assert_eq!(edca_param(&w[0], true, HwBand::G5), 0x0000_A42B);
    assert_eq!(edca_param(&w[3], true, HwBand::G5), 0x002F_3222);
    assert_eq!(edca_param(&w[0], true, HwBand::G2) & 0xFF, 0x25);
    assert_eq!(edca_param(&w[0], true, HwBand::G2), 0x0000_A425);
    // The long slot.
    assert_eq!(edca_param(&w[0], false, HwBand::G5), 0x0000_A44C);
    assert_eq!(edca_param(&w[0], false, HwBand::G2), 0x0000_A446);
    assert_eq!(edca_param(&w[1], true, HwBand::G5), 0x0000_A44F);
    assert_eq!(edca_param(&w[2], true, HwBand::G5), 0x005E_4322);
    // rtw89_aifsn_to_aifs returns a u8: 15 * 20 + 10 = 310 wraps to 54.
    assert_eq!(
        edca_param(&ac(0, 15, 4, 10, 0), false, HwBand::G2),
        0x0000_A436
    );
    // TXOPLMT keeps 11 bits.
    assert_eq!(
        edca_param(&ac(0, 2, 0, 0, 0xFFFF), true, HwBand::G5),
        0x07FF_0022
    );
}

#[test]
fn features_follow_the_firmware_version() {
    // fw.c:880-905: BEACON_FILTER from 0.27.56.10, BEACON_LOSS_COUNT_V1 from
    // 0.27.128.0 on the RTL8852C.
    let of = |v| Features::of(v);
    let none = Features {
        beacon_filter: false,
        loss_count_v1: false,
    };
    assert_eq!(of((0, 27, 56, 9)), none);
    assert_eq!(
        of((0, 27, 56, 10)),
        Features {
            beacon_filter: true,
            loss_count_v1: false
        }
    );
    assert!(!of((0, 27, 127, 255)).loss_count_v1);
    assert_eq!(of((0, 27, 128, 0)), FULL);
    assert_eq!(of((0, 27, 129, 4)), FULL);
    assert_eq!(of((0, 28, 0, 0)), FULL);
}

/// What a sequence left: the trace, every CH12 packet in kick order, and
/// the H2C sequence after it.
struct Ran {
    io: RegisterModel,
    packets: Vec<Vec<u8>>,
    h2c: H2c,
    frames: usize,
}

/// `f` on a session over fresh rings and the scripted firmware, its H2Cs
/// numbered from `seq`, the port on as vif init leaves it and DIG's
/// register all ones, so that every DIG write shows.
fn on_session<R>(
    seq: u8,
    script: &dyn Fn(&mut Firmware),
    f: impl FnOnce(&mut RegisterModel, &mut Session<'_>, &mut dyn FnMut(Event<'_>)) -> R,
) -> (R, Ran) {
    let (m, mut rings, mut io) = running();
    io.set32(0xC400, 0x4);
    io.set32(0x1481C, 0xFFFF_FFFF);
    io.set32(0xCD80, 0xFFFF_FFFF);
    let mut fw = Firmware::new(&m);
    script(&mut fw);
    fw.install(&mut io);
    let mut h2c = H2c::at(seq);
    let mut frames = 0;
    let got = {
        let mut session = Session::new(&mut rings, &mut h2c, G5_36);
        // The live chip's receiver after its start (M3 spec S7.1).
        session.rx.set_mode(Mode::Station);
        f(&mut io, &mut session, &mut |e| {
            if matches!(e, Event::Frame(_)) {
                frames += 1;
            }
        })
    };
    let k = kicks(&io.trace).len();
    assert!(k <= 16, "{k} kicks overran the CH12 slots");
    let state = m.lock().unwrap();
    let packets = (0..k)
        .map(|n| {
            let at = u64::from(FWCMD) + (n * FWCMD_SLOT_BYTES) as u64;
            let head = device_read(&state, at, 24);
            let hdr1 = u32::from_le_bytes(head[20..24].try_into().unwrap());
            device_read(&state, at, 16 + (hdr1 & 0x3FFF) as usize)
        })
        .collect();
    drop(state);
    (
        got,
        Ran {
            io,
            packets,
            h2c,
            frames,
        },
    )
}

/// The trace indices of the CH12 kicks.
fn kicks(trace: &[Op]) -> Vec<usize> {
    trace
        .iter()
        .enumerate()
        .filter(|(_, o)| matches!(o, Op::W16(0x1080, _)))
        .map(|(i, _)| i)
        .collect()
}

fn first(trace: &[Op], want: impl Fn(&Op) -> bool) -> usize {
    trace
        .iter()
        .position(want)
        .unwrap_or_else(|| panic!("not in the trace"))
}

/// mac80211's AC order (VO, VI, BE, BK) as the ACIs the firmware takes.
const MAC80211_ACS: [u8; 4] = [3, 2, 0, 1];

/// The four EDCA H2Cs `b` builds next for `params` in mac80211's order.
fn edcas(b: &mut H2c, params: &[AccessCategory; 4], short: bool, band: HwBand) -> Vec<Vec<u8>> {
    MAC80211_ACS
        .iter()
        .map(|&aci| {
            b.edca(aci, edca_param(&params[usize::from(aci)], short, band))
                .1
        })
        .collect()
}

/// The payload word `i` of a CH12 packet.
fn payload_word(p: &[u8], i: usize) -> u32 {
    u32::from_le_bytes(p[24 + 4 * i..28 + 4 * i].try_into().unwrap())
}

#[test]
fn set_bssid_sends_the_pre_auth_cam_then_suspends_dig() {
    let mut dig = Dig::default();
    // SAFETY: `running` programmed these rings through this modeled window
    // with `pre_init`; the model domain lets no real device reach the
    // published IOVAs, and this test is the only worker.
    let (got, ran) = on_session(17, &|_| {}, |io, s, sink| unsafe {
        set_bssid(io, s, LOCAL, AP, &mut dig, sink)
    });
    assert_eq!(got, Ok(()));
    // Spec S9.2: the BSSID's address CAM (BSS_CHANGED_BSSID, mac80211.c:
    // 771-776), then sta_add's DIG suspend (core.c:5772), as mac80211 tells
    // the BSSID before it inserts the AP's station (mlme.c:9773-9789).
    assert_eq!(
        ran.packets,
        [H2c::at(17)
            .addr_cam_with(&AddrCam::with_bssid(LOCAL, AP), 0, 0)
            .1]
    );
    let t = &ran.io.trace;
    let cam = kicks(t)[0];
    let dig_at = first(t, |o| *o == Op::R32(0x1481C));
    assert!(cam < dig_at);
    assert_eq!(
        t[dig_at..],
        [
            Op::R32(0x1481C),
            Op::W32(0x1481C, 0xFFFF_F83F),
            Op::R32(0x1481C),
            Op::W32(0x1481C, 0xDFFF_F83F),
        ]
    );
    assert!(dig.paused());
    assert_eq!(ran.h2c, H2c::at(18));
    // A second suspend finds DIG paused and does nothing (phy.c:7859).
    // SAFETY: as above.
    let (got, ran) = on_session(18, &|_| {}, |io, s, sink| unsafe {
        set_bssid(io, s, LOCAL, AP, &mut dig, sink)
    });
    assert_eq!(got, Ok(()));
    assert_eq!(ran.packets.len(), 1);
    assert!(!ran.io.trace.iter().any(|o| matches!(o, Op::R32(0x1481C))));
}

#[test]
fn associate_runs_linux_order() {
    let a = inputs(HwBand::G5);
    let mut dig = Dig::default();
    let mut split = 0;
    // SAFETY: as in `set_bssid_sends_the_pre_auth_cam_then_suspends_dig`.
    let (got, ran) = on_session(16, &|_| {}, |io, s, sink| unsafe {
        set_bssid(io, s, LOCAL, AP, &mut dig, sink)?;
        io.set32(0x1481C, 0xFFFF_FFFF);
        split = io.trace.len();
        associate(io, s, &a, FULL, &mut dig, sink)
    });
    assert_eq!(got, Ok(()));
    let mut b = H2c::at(17);
    // mac80211 programs the AP's WMM parameters from the association response
    // first (ieee80211_assoc_config_link, mlme.c:6310, conf_tx for VO, VI,
    // BE, BK, mlme.c:4094-4114), with the long slot it reset at the last
    // disassociation; then BSS_CHANGED_ASSOC: rtw89_core_sta_link_assoc
    // (core.c:5878-5979) and the port update (mac80211.c:678-697); then
    // BSS_CHANGED_ERP_SLOT on the same notify (mlme.c:4323-4326,
    // mac80211.c:781-782): conf_tx again with the short slot.
    let mut want = edcas(&mut b, &wmm(), false, HwBand::G5);
    want.push(b.assoc_cmac_tbl(0, 0, HwBand::G5).1);
    want.push(b.join_info(0, 0, true).1);
    want.push(b.addr_cam_with(&AddrCam::connected(LOCAL, AP, 5), 0, 0).1);
    want.push(b.ra(0, &ra_info(&a.peer, HwBand::G5)).1);
    want.push(b.bcn_fltr(0, true, true).1);
    want.extend(edcas(&mut b, &wmm(), true, HwBand::G5));
    assert_eq!(ran.packets[1..], want[..]);
    assert_eq!(ran.h2c, b);
    assert_eq!(ran.h2c, H2c::at(30));
    // The vectors behind them: VO's long-slot AIFS 2 * 20 + 16, the AC
    // fields 3, 2, 0, 1, BE's short-slot word last but one.
    let p = &ran.packets[1..];
    assert_eq!(payload_word(&p[0], 1), 0x002F_3238);
    let acs: Vec<u32> = p[..4].iter().map(|p| payload_word(p, 0) >> 5).collect();
    assert_eq!(acs, [3, 2, 0, 1]);
    assert_eq!(payload_word(&p[11], 1), 0x0000_A42B);
    assert_eq!(payload_word(&p[7], 0), 0x01B0_000C);
    assert_eq!(payload_word(&p[8], 0), 0x0028_4C67);
    // By first index: EDCA x4 < CMAC table < join < CAM < RA < beacon filter
    // < DIG resume < the port update's msleep(beacon_int + 1) < EDCA x4.
    let t = &ran.io.trace[split..];
    let k = kicks(t);
    assert_eq!(k.len(), 13);
    let dig_at = first(t, |o| matches!(o, Op::W32(0x1481C, _)));
    let sleep = first(t, |o| *o == Op::Delay(101_000));
    assert!(
        k[8] < dig_at && dig_at < sleep && sleep < k[9],
        "{k:?} {dig_at} {sleep}"
    );
    // dig_resume(false): PD lower bound 0, and spatial reuse off, as
    // total_sta_assoc counts this station only after rtw89_core_sta_link_assoc
    // returns (mac80211.c:569-575).
    assert_eq!(
        t[dig_at - 1..dig_at + 3],
        [
            Op::R32(0x1481C),
            Op::W32(0x1481C, 0xFFFF_F83F),
            Op::R32(0x1481C),
            Op::W32(0x1481C, 0xDFFF_F83F),
        ]
    );
    assert!(!dig.paused());
    // The port is INFRA with the AP's beacon interval and DTIM period.
    assert_eq!(ran.io.get32(0xC400) & 0xC00, 0x800);
    assert_eq!(ran.io.get32(0xC414) & 0xFFFF, 100);
    assert_eq!((ran.io.get32(0xC424) >> 16) & 0xFF00, 0x100);
}

#[test]
fn associate_on_2g_with_the_long_slot_sends_edca_once() {
    let a = AssocInputs {
        short_slot: false,
        ..inputs(HwBand::G2)
    };
    // SAFETY: as in `set_bssid_sends_the_pre_auth_cam_then_suspends_dig`.
    let (got, ran) = on_session(17, &|_| {}, |io, s, sink| unsafe {
        associate(io, s, &a, FULL, &mut Dig::default(), sink)
    });
    assert_eq!(got, Ok(()));
    let mut b = H2c::at(17);
    let mut want = edcas(&mut b, &wmm(), false, HwBand::G2);
    want.push(b.assoc_cmac_tbl(0, 0, HwBand::G2).1);
    want.push(b.join_info(0, 0, true).1);
    want.push(b.addr_cam_with(&AddrCam::connected(LOCAL, AP, 5), 0, 0).1);
    want.push(b.ra(0, &ra_info(&a.peer, HwBand::G2)).1);
    want.push(b.bcn_fltr(0, true, true).1);
    // The slot stays long: no ERP_SLOT, no second set.
    assert_eq!(ran.packets, want);
    assert_eq!(payload_word(&ran.packets[2], 1), 0x0000_A446);
    // DIG was not paused: nothing to resume.
    assert!(!ran.io.trace.iter().any(|o| matches!(o, Op::R32(0x1481C))));
}

#[test]
fn associate_on_5g_takes_the_short_slot_mac80211_forces() {
    // ieee80211_handle_bss_capability (mlme.c:4308-4311): 5 GHz is always
    // the short slot, whatever the AP's capability says.
    let a = AssocInputs {
        short_slot: false,
        ..inputs(HwBand::G5)
    };
    // SAFETY: as in `set_bssid_sends_the_pre_auth_cam_then_suspends_dig`.
    let (got, ran) = on_session(17, &|_| {}, |io, s, sink| unsafe {
        associate(io, s, &a, FULL, &mut Dig::default(), sink)
    });
    assert_eq!(got, Ok(()));
    assert_eq!(ran.packets.len(), 13);
    assert_eq!(payload_word(&ran.packets[11], 1), 0x0000_A42B);
}

#[test]
fn associate_without_wmm_sends_the_qos_defaults() {
    // mac80211 still enables QoS without WMM ("we might have HT/VHT",
    // mlme.c:6308-6323): ieee80211_set_wmm_default(.., true) hands
    // conf_tx 802.11's defaults, then the short slot sends them again. An
    // old firmware gets no beacon filter.
    let a = AssocInputs {
        edca: None,
        ..inputs(HwBand::G5)
    };
    let old = Features {
        beacon_filter: false,
        loss_count_v1: false,
    };
    // SAFETY: as in `set_bssid_sends_the_pre_auth_cam_then_suspends_dig`.
    let (got, ran) = on_session(17, &|_| {}, |io, s, sink| unsafe {
        associate(io, s, &a, old, &mut Dig::default(), sink)
    });
    assert_eq!(got, Ok(()));
    let mut b = H2c::at(17);
    let defaults = qos_defaults(false);
    let mut want = edcas(&mut b, &defaults, false, HwBand::G5);
    want.extend([
        b.assoc_cmac_tbl(0, 0, HwBand::G5).1,
        b.join_info(0, 0, true).1,
        b.addr_cam_with(&AddrCam::connected(LOCAL, AP, 5), 0, 0).1,
        b.ra(0, &ra_info(&a.peer, HwBand::G5)).1,
    ]);
    want.extend(edcas(&mut b, &defaults, true, HwBand::G5));
    assert_eq!(ran.packets, want);
    // The port update still runs.
    assert!(ran.io.trace.contains(&Op::Delay(101_000)));
}

#[test]
fn qos_defaults_are_set_wmm_defaults_with_qos() {
    // util.c:1054-1117: aCWmin 15 (31 for 11b), aCWmax 1023; BE AIFS 3, BK
    // 7; VI CW 7-15 (15-31), TXOP 3008 us (6016); VO CW 3-7 (7-15), TXOP
    // 1504 us (3264); in ACI order, BE, BK, VI, VO.
    assert_eq!(
        qos_defaults(false),
        [
            ac(0, 3, 4, 10, 0),
            ac(1, 7, 4, 10, 0),
            ac(2, 2, 3, 4, 94),
            ac(3, 2, 2, 3, 47),
        ]
    );
    assert_eq!(
        qos_defaults(true),
        [
            ac(0, 3, 5, 10, 0),
            ac(1, 7, 5, 10, 0),
            ac(2, 2, 4, 5, 188),
            ac(3, 2, 3, 4, 102),
        ]
    );
}

#[test]
fn an_invalid_edca_set_falls_back_to_the_qos_defaults() {
    // _ieee80211_sta_wmm_params (mlme.c:4214-4245): CWmin 0, CWmin above
    // CWmax, or an AC missing (here out of ACI order) is "using defaults";
    // an AIFSN under 2 is taken as 2.
    let w = wmm();
    let mut zero = w;
    zero[1].ecw_min = 0;
    let mut above = w;
    above[2].ecw_min = 5;
    for edca in [[w[3], w[2], w[0], w[1]], zero, above] {
        // On 2.4 GHz with an 11b-only peer: the 11b defaults.
        let mut a = AssocInputs {
            edca: Some(edca),
            short_slot: false,
            ..inputs(HwBand::G2)
        };
        a.peer.legacy = 0xF;
        a.peer.ht = None;
        // SAFETY: as in `set_bssid_sends_the_pre_auth_cam_then_suspends_dig`.
        let (got, ran) = on_session(17, &|_| {}, |io, s, sink| unsafe {
            associate(io, s, &a, FULL, &mut Dig::default(), sink)
        });
        assert_eq!(got, Ok(()));
        let want = edcas(&mut H2c::at(17), &qos_defaults(true), false, HwBand::G2);
        assert_eq!(ran.packets[..4], want[..]);
        assert_eq!(ran.packets.len(), 9);
    }
    let mut low = w;
    low[0].aifsn = 1;
    let a = AssocInputs {
        edca: Some(low),
        ..inputs(HwBand::G5)
    };
    // SAFETY: as above.
    let (got, ran) = on_session(17, &|_| {}, |io, s, sink| unsafe {
        associate(io, s, &a, FULL, &mut Dig::default(), sink)
    });
    assert_eq!(got, Ok(()));
    // BE (seq 19, mac80211's third) with AIFS 2.
    let mut clamped = w;
    clamped[0].aifsn = 2;
    let want = edcas(&mut H2c::at(17), &clamped, false, HwBand::G5);
    assert_eq!(ran.packets[..4], want[..]);
}

#[test]
fn associate_hands_what_it_drains_to_the_sink() {
    // A frame from the AP right after the beacon filter's acks.
    let script = |fw: &mut Firmware| {
        fw.after(9, 0x1E, vec![mgnt_frame(0x08, LOCAL, AP, b"m1", None, 3)]);
    };
    let a = inputs(HwBand::G5);
    // SAFETY: as in `set_bssid_sends_the_pre_auth_cam_then_suspends_dig`.
    let (got, ran) = on_session(17, &script, |io, s, sink| unsafe {
        associate(io, s, &a, FULL, &mut Dig::default(), sink)
    });
    assert_eq!(got, Ok(()));
    assert_eq!(ran.frames, 1);
}

#[test]
fn leave_runs_linux_order() {
    let a = inputs(HwBand::G5);
    // SAFETY: as in `set_bssid_sends_the_pre_auth_cam_then_suspends_dig`.
    let (got, ran) = on_session(31, &|_| {}, |io, s, sink| unsafe {
        leave(io, s, &a, FULL, CamKeys::default(), sink)
    });
    assert_eq!(got, Ok(()));
    let mut b = H2c::at(31);
    // mac80211's teardown (ieee80211_set_disassoc, mlme.c:4506-4718): the
    // station's ASSOC -> AUTH (core.c:5799-5816: the beacon filter off),
    // AUTH -> NONE (core.c:5818-5859: the beamformee off, the CMAC table,
    // join disconnect, the CAM with NO_LINK), NONE -> NOTEXIST (nothing on
    // the chip); then BSS_CHANGED_BSSID with the BSSID cleared
    // (mac80211.c:771-776), BSS_CHANGED_ERP_SLOT (ieee80211_reset_erp_info,
    // main.c:430-438: conf_tx with the AP's parameters and the long slot),
    // BSS_CHANGED_MU_GROUPS (the GID table, mac.c:6914-6953); then the
    // defaults without QoS (util.c:1025-1135).
    let mut want = vec![
        b.bcn_fltr(0, false, true).1,
        b.assoc_cmac_tbl(0, 0, HwBand::G5).1,
        b.join_info(0, 0, false).1,
        b.addr_cam_with(&AddrCam::with_bssid(LOCAL, AP), 0, 0).1,
        b.addr_cam(LOCAL, 0, 0).1,
    ];
    want.extend(edcas(&mut b, &wmm(), false, HwBand::G5));
    let defaults = [
        ac(0, 2, 4, 10, 0),
        ac(1, 2, 4, 10, 0),
        ac(2, 2, 4, 10, 0),
        ac(3, 2, 4, 10, 0),
    ];
    want.extend(edcas(&mut b, &defaults, false, HwBand::G5));
    assert_eq!(ran.packets, want);
    assert_eq!(ran.h2c, H2c::at(44));
    assert_eq!(payload_word(&ran.packets[0], 0), 0x0028_4C60);
    assert_eq!(payload_word(&ran.packets[2], 0), 0x0400_0100);
    assert_eq!(payload_word(&ran.packets[12], 1), 0x0000_A438);
    let t = &ran.io.trace;
    let k = kicks(t);
    assert_eq!(k.len(), 13);
    // rtw89_mac_bf_disassoc (mac.c:6907-6912): NDPA off on MAC 0, between the
    // beacon filter and the CMAC table.
    let bfee = first(t, |o| matches!(o, Op::W32(0xCD80, _)));
    assert!(k[0] < bfee && bfee < k[1]);
    assert_eq!(
        t[bfee - 1..=bfee],
        [Op::R32(0xCD80), Op::W32(0xCD80, 0xFFFF_FFF8)]
    );
    // The GID table zeroed between the two EDCA sets: membership, then
    // position (plain writes).
    let gid: Vec<Op> = t[k[8]..k[9]]
        .iter()
        .copied()
        .filter(|o| matches!(o, Op::W32(0xC070..=0xC084, _)))
        .collect();
    assert_eq!(
        gid,
        [
            Op::W32(0xC080, 0),
            Op::W32(0xC084, 0),
            Op::W32(0xC070, 0),
            Op::W32(0xC074, 0),
            Op::W32(0xC078, 0),
            Op::W32(0xC07C, 0),
        ]
    );
    // Linux leaves the port as association set it (spec S11.6), and DIG.
    assert!(!t.iter().any(|o| matches!(o, Op::W32(0xC400, _))));
    assert!(
        !t.iter()
            .any(|o| matches!(o, Op::R32(0x1481C) | Op::W32(0x1481C, _)))
    );
}

#[test]
fn leave_takes_the_defaults_of_the_bss_it_left() {
    // ieee80211_set_wmm_default without QoS: CWmin 31 on an 11b-only 2.4 GHz
    // BSS (no rate above 11 Mbps, mlme.c:6405-6406), else 15; AIFSN 2;
    // CWmax 1023; the long slot.
    for (legacy, word) in [
        (0xF, 0x0000_A532),
        (0xFFF, 0x0000_A432),
        (0x3F, 0x0000_A532),
    ] {
        let mut a = inputs(HwBand::G2);
        a.peer.legacy = legacy;
        a.peer.ht = None;
        // SAFETY: as in `set_bssid_sends_the_pre_auth_cam_then_suspends_dig`.
        let (got, ran) = on_session(31, &|_| {}, |io, s, sink| unsafe {
            leave(io, s, &a, FULL, CamKeys::default(), sink)
        });
        assert_eq!(got, Ok(()));
        assert_eq!(ran.packets.len(), 13);
        for p in &ran.packets[9..] {
            assert_eq!(payload_word(p, 1), word, "{legacy:#x}");
        }
    }
    // Without WMM, ERP_SLOT still sends the QoS defaults the association
    // programmed, then the defaults without QoS; without the feature no
    // beacon filter.
    let a = AssocInputs {
        edca: None,
        ..inputs(HwBand::G5)
    };
    let old = Features {
        beacon_filter: false,
        loss_count_v1: false,
    };
    // SAFETY: as above.
    let (got, ran) = on_session(31, &|_| {}, |io, s, sink| unsafe {
        leave(io, s, &a, old, CamKeys::default(), sink)
    });
    assert_eq!(got, Ok(()));
    let mut b = H2c::at(31);
    let mut want = vec![
        b.assoc_cmac_tbl(0, 0, HwBand::G5).1,
        b.join_info(0, 0, false).1,
        b.addr_cam_with(&AddrCam::with_bssid(LOCAL, AP), 0, 0).1,
        b.addr_cam(LOCAL, 0, 0).1,
    ];
    want.extend(edcas(&mut b, &qos_defaults(false), false, HwBand::G5));
    let defaults = [
        ac(0, 2, 4, 10, 0),
        ac(1, 2, 4, 10, 0),
        ac(2, 2, 4, 10, 0),
        ac(3, 2, 4, 10, 0),
    ];
    want.extend(edcas(&mut b, &defaults, false, HwBand::G5));
    assert_eq!(ran.packets, want);
}

#[test]
fn clear_bssid_sends_the_unassociated_cam() {
    // SAFETY: as in `set_bssid_sends_the_pre_auth_cam_then_suspends_dig`.
    let (got, ran) = on_session(18, &|_| {}, |io, s, sink| unsafe {
        clear_bssid(io, s, LOCAL, CamKeys::default(), sink)
    });
    assert_eq!(got, Ok(()));
    assert_eq!(ran.packets, [H2c::at(18).addr_cam(LOCAL, 0, 0).1]);
    // DIG stays as it was: Linux resumes it only at association.
    assert!(!ran.io.trace.iter().any(|o| matches!(o, Op::R32(0x1481C))));
}

#[test]
fn leave_carries_the_address_cams_key_fields() {
    // rtw89_cam_fill_addr_cam_info (cam.c:906-927) reads the vif's address
    // CAM entry, whose key IDs and entries a key delete leaves in place
    // with only the valid map cleared (cam.c:215-246): the leave's CAM with
    // NO_LINK and its CAM without the BSSID both carry them.
    let stale = CamKeys {
        keyids: 0x0040_0000,
        valid_and_ents: 0x0100_0000,
        ents_hi: 0,
    };
    let a = inputs(HwBand::G5);
    // SAFETY: as in `set_bssid_sends_the_pre_auth_cam_then_suspends_dig`.
    let (got, ran) = on_session(31, &|_| {}, |io, s, sink| unsafe {
        leave(io, s, &a, FULL, stale, sink)
    });
    assert_eq!(got, Ok(()));
    let mut b = H2c::at(34);
    let unlinked = AddrCam {
        keys: stale,
        ..AddrCam::with_bssid(LOCAL, AP)
    };
    let cleared = AddrCam {
        keys: stale,
        ..AddrCam::unassociated(LOCAL)
    };
    assert_eq!(ran.packets[3], b.addr_cam_with(&unlinked, 0, 0).1);
    assert_eq!(ran.packets[4], b.addr_cam_with(&cleared, 0, 0).1);
    // w9: SEC_ENT_MODE NORMAL and SEC_ENT2_KEYID 1; w10: SEC_ENT2 1, no
    // valid slot.
    assert_eq!(payload_word(&ran.packets[3], 9), 0x0042_0000);
    assert_eq!(payload_word(&ran.packets[4], 10), 0x0100_0000);
    // The auth-only teardown's disconnect carries them too.
    // SAFETY: as above.
    let (got, ran) = on_session(18, &|_| {}, |io, s, sink| unsafe {
        disconnect(io, s, LOCAL, AP, HwBand::G5, stale, sink)
    });
    assert_eq!(got, Ok(()));
    assert_eq!(ran.packets[2], H2c::at(20).addr_cam_with(&unlinked, 0, 0).1);
}

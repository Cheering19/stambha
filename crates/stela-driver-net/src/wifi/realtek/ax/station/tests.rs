// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright (C) 2026 Stela contributors
//! The rtw89 radio through the keeper, over a model of the chip that plays
//! the AP, shaped as rtw88's `station/tests.rs` `Air`: a firmware scan
//! hears the AP's beacon on its channel and ends, the AP answers
//! authentication and association, and EAPOL runs against the supplicant
//! tests' `Authenticator`.
use super::chip::{Association, Chip, Data, HardwareKey, Incoming, Management, Received, Token};
use super::*;
use crate::LinkError;
use crate::wifi::connection::security::Security;
use crate::wifi::intel::policy::tests::regdb;
use crate::wifi::network::{Network, Secret};
use crate::wifi::station::phy::AccessCategory;
use crate::wifi::supplicant::tests::Authenticator;
use alloc::{rc::Rc, vec};
use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, Ordering};
use stela_driver_base::entropy::EntropyError;
use zeroize::Zeroizing;

const CLOCK: Clock = Clock {
    ticks_per_microsecond: 1,
};
/// The plan's test addresses.
const LOCAL: [u8; 6] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
const AP: [u8; 6] = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07];
const SSID: &[u8] = b"test";
const RSN: [u8; 22] = [
    48, 20, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 2, 0, 0,
];
/// An AP's HT capabilities (SGI 20, one RX STBC stream, SM power save
/// static) and operation, one stream at 20 MHz.
const HT: [u8; 52] = [
    45, 26, 0x2c, 1, 0x17, 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    61, 22, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
];
/// A WMM parameter element: best effort at the defaults of each class.
const WMM: [u8; 26] = [
    221, 24, 0, 0x50, 0xf2, 2, 1, 1, 0, 0, 3, 0xa4, 0, 0, 0x27, 0xa4, 0, 0, 0x42, 0x43, 0x5e, 0,
    0x62, 0x32, 0x2f, 0,
];
/// A US-like country: 2.4 GHz channels 1-11, and 5 GHz 36-64, 100-144 and
/// 149-165.
const US: [(u32, u32, u8, u16); 5] = [
    (2_402_000, 2_472_000, 0, 3000),
    (5_170_000, 5_250_000, 0, 2300),
    (5_250_000, 5_330_000, 0, 2300),
    (5_490_000, 5_730_000, 0, 2300),
    (5_735_000, 5_835_000, 0, 3000),
];

/// Each read one byte over the last, from 0x5a.
#[derive(Default)]
struct FixedEntropy(AtomicU8);

impl Entropy for FixedEntropy {
    fn fill(&self, bytes: &mut [u8]) -> Result<(), EntropyError> {
        bytes.fill(0x5a + self.0.fetch_add(1, Ordering::Relaxed));
        Ok(())
    }
}

/// An association as the chip was handed it.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Joined {
    bssid: Mac,
    aid: u16,
    channel: Channel,
    beacon_interval: u16,
    dtim_period: u8,
    legacy: u16,
    ht: Option<HtPeer>,
    edca: Option<[AccessCategory; 4]>,
    short_slot: bool,
}

/// One call the chip took, in order.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Call {
    Tune(Channel),
    BeginScan(Vec<Channel>),
    EndScan,
    SetBssid(Mac),
    Authenticated,
    /// A management frame and the basic rates it went with.
    Management(Vec<u8>, u16),
    Data {
        bytes: Vec<u8>,
        tid: u8,
        eapol: bool,
        supported: u16,
        keyed: bool,
    },
    Associate(Joined),
    Disassociate,
}

/// What the chip was told, and what it holds for the host.
#[derive(Default)]
struct Air {
    /// The AP's channel.
    channel: Option<Channel>,
    calls: Vec<Call>,
    incoming: VecDeque<Incoming>,
    /// The channels of a scan begun and not yet ended.
    scanning: Option<Vec<Channel>>,
    next_token: u16,
    /// Frames the chip took that the AP has yet to answer.
    unanswered: VecDeque<(Token, Vec<u8>)>,
    /// The AP neither acknowledges nor answers an authentication.
    deaf_to_auth: bool,
    /// The AP acknowledges an association request but never answers.
    mute_on_assoc: bool,
    /// The AP has no WMM.
    no_wmm: bool,
    /// The scan's beacon fails its FCS.
    bad_fcs: bool,
    /// The chip has no room for management or for data frames.
    full_management: bool,
    full_data: bool,
    /// Frames refused for no room.
    refused: usize,
}

impl Air {
    fn hear(&mut self, bytes: Vec<u8>) {
        let channel = self.channel.unwrap();
        self.incoming.push_back(Incoming::Frame(Received {
            bytes,
            channel,
            signal_dbm: None,
            tsf: 0,
            decrypted: false,
            icv_error: false,
            fcs_error: false,
        }));
    }

    /// Each frame's release report, and the AP's answer to it.
    fn serve(&mut self) {
        while let Some((token, bytes)) = self.unanswered.pop_front() {
            let deaf = bytes[0] == 0xb0 && self.deaf_to_auth;
            self.incoming.push_back(Incoming::Transmitted {
                token,
                acknowledged: !deaf,
            });
            if deaf {
                continue;
            }
            match bytes[0] {
                0xb0 => self.hear(management(0xb0, &[0, 0, 2, 0, 0, 0])),
                0x00 if !self.mute_on_assoc => {
                    let mut body = vec![0x11, 0, 0, 0, 1, 0xc0];
                    body.extend(RATES);
                    body.extend(HT);
                    if !self.no_wmm {
                        body.extend(WMM);
                    }
                    self.hear(management(0x10, &body));
                }
                _ => {}
            }
        }
    }

    fn token(&mut self) -> Token {
        self.next_token += 1;
        Token(self.next_token)
    }

    fn position(&self, want: impl FnMut(&Call) -> bool) -> Option<usize> {
        self.calls.iter().position(want)
    }

    /// The EAPOL bodies the station sent, past their LLC.
    fn eapol(&self) -> Vec<Vec<u8>> {
        self.calls
            .iter()
            .filter_map(|c| match c {
                Call::Data { bytes, .. } => eapol(bytes).map(<[u8]>::to_vec),
                _ => None,
            })
            .collect()
    }

    /// The management frames of subtype `fc0` the chip took.
    fn management(&self, fc0: u8) -> Vec<Vec<u8>> {
        self.calls
            .iter()
            .filter_map(|c| match c {
                Call::Management(bytes, _) if bytes[0] == fc0 => Some(bytes.clone()),
                _ => None,
            })
            .collect()
    }

    fn count(&self, want: impl Fn(&Call) -> bool) -> usize {
        self.calls.iter().filter(|c| want(c)).count()
    }
}

#[derive(Clone)]
struct Model(Rc<RefCell<Air>>);

impl Chip for Model {
    fn mac(&self) -> Mac {
        Mac(LOCAL)
    }
    fn poll(&mut self, sink: &mut dyn FnMut(Incoming)) -> Result<(), String> {
        let mut air = self.0.borrow_mut();
        if let Some(channels) = air.scanning.take() {
            let channel = air.channel.unwrap();
            if channels.contains(&channel) {
                let bad = air.bad_fcs;
                let bytes = beacon(&air);
                air.incoming.push_back(Incoming::Frame(Received {
                    bytes,
                    channel,
                    signal_dbm: Some(-40),
                    tsf: 0,
                    decrypted: false,
                    icv_error: false,
                    fcs_error: bad,
                }));
            }
            air.incoming
                .push_back(Incoming::ScanEnded { completed: true });
        }
        air.incoming.drain(..).for_each(sink);
        Ok(())
    }
    fn tune(&mut self, channel: Channel) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::Tune(channel));
        Ok(())
    }
    fn begin_scan(&mut self, channels: &[Channel]) -> Result<(), String> {
        let mut air = self.0.borrow_mut();
        air.calls.push(Call::BeginScan(channels.to_vec()));
        air.scanning = Some(channels.to_vec());
        Ok(())
    }
    fn end_scan(&mut self) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::EndScan);
        Ok(())
    }
    fn set_bssid(&mut self, bssid: Mac) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::SetBssid(bssid));
        Ok(())
    }
    fn authenticated(&mut self) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::Authenticated);
        Ok(())
    }
    fn transmit_management(&mut self, frame: &Management<'_>) -> Result<Option<Token>, String> {
        let mut air = self.0.borrow_mut();
        if air.full_management {
            air.refused += 1;
            return Ok(None);
        }
        air.calls
            .push(Call::Management(frame.bytes.to_vec(), frame.basic_rates));
        let token = air.token();
        air.unanswered.push_back((token, frame.bytes.to_vec()));
        Ok(Some(token))
    }
    fn transmit_data(&mut self, frame: &Data<'_>) -> Result<Option<Token>, String> {
        let mut air = self.0.borrow_mut();
        if air.full_data {
            air.refused += 1;
            return Ok(None);
        }
        air.calls.push(Call::Data {
            bytes: frame.bytes.to_vec(),
            tid: frame.tid,
            eapol: frame.eapol,
            supported: frame.supported_rates,
            keyed: frame.key.is_some(),
        });
        let token = air.token();
        air.unanswered.push_back((token, frame.bytes.to_vec()));
        Ok(Some(token))
    }
    fn associate(&mut self, association: &Association) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::Associate(Joined {
            bssid: association.bssid,
            aid: association.aid,
            channel: association.channel,
            beacon_interval: association.beacon_interval,
            dtim_period: association.dtim_period,
            legacy: association.legacy,
            ht: association.ht,
            edca: association.edca,
            short_slot: association.short_slot,
        }));
        Ok(())
    }
    fn disassociate(&mut self) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::Disassociate);
        Ok(())
    }
    fn add_key(&mut self, _: &HardwareKey<'_>) -> Result<u8, String> {
        Err("the model holds no key".into())
    }
    fn remove_key(&mut self, _: u8) -> Result<(), String> {
        Err("the model holds no key".into())
    }
    fn start_rx_block_ack(&mut self, _: u8, _: u16) -> Result<(), String> {
        Ok(())
    }
    fn stop_rx_block_ack(&mut self, _: u8) -> Result<(), String> {
        Ok(())
    }
    fn diagnostic(&self) -> String {
        "chip".into()
    }
}

/// The AP's rates on 2.4 GHz: 1 Mb/s (basic) and 6 Mb/s.
const RATES: [u8; 4] = [1, 2, 0x82, 0x0c];

/// An AP's management frame to the station.
fn management(subtype: u8, body: &[u8]) -> Vec<u8> {
    let mut frame = vec![0; 24];
    frame[0] = subtype;
    frame[4..10].copy_from_slice(&LOCAL);
    frame[10..16].copy_from_slice(&AP);
    frame[16..22].copy_from_slice(&AP);
    frame.extend(body);
    frame
}

/// The AP's beacon on its channel.
fn beacon(air: &Air) -> Vec<u8> {
    let channel = air.channel.unwrap().number as u8;
    let mut frame = vec![0; 36];
    frame[0] = 0x80;
    frame[4..10].fill(0xff);
    frame[10..16].copy_from_slice(&AP);
    frame[16..22].copy_from_slice(&AP);
    frame[32..34].copy_from_slice(&100u16.to_le_bytes());
    frame[34] = 0x11;
    frame.extend([0, SSID.len() as u8]);
    frame.extend(SSID);
    frame.extend(RATES);
    // DS parameter set, then a TIM with DTIM period 2.
    frame.extend([3, 1, channel, 5, 4, 0, 2, 0, 0]);
    frame.extend(RSN);
    frame.extend(HT);
    // HT Operation's primary channel.
    let at = frame.len() - 22;
    frame[at] = channel;
    if !air.no_wmm {
        frame.extend(WMM);
    }
    frame
}

/// A data frame from the AP to `receiver`, without protection.
fn data(receiver: [u8; 6], sequence: u16, kind: [u8; 2], body: &[u8]) -> Vec<u8> {
    let mut frame = vec![0; 26];
    frame[0] = 0x88;
    frame[1] = 0x02;
    frame[4..10].copy_from_slice(&receiver);
    frame[10..16].copy_from_slice(&AP);
    frame[16..22].copy_from_slice(&AP);
    frame[22..24].copy_from_slice(&(sequence << 4).to_le_bytes());
    frame.extend([0xaa, 0xaa, 3, 0, 0, 0]);
    frame.extend(kind);
    frame.extend(body);
    frame
}

/// What the station sent past `frame`'s header and LLC when it is EAPOL.
fn eapol(frame: &[u8]) -> Option<&[u8]> {
    if frame[0] & 0x0c != 8 {
        return None;
    }
    let header = if frame[0] & 0x80 != 0 { 26 } else { 24 };
    (frame.get(header + 6..header + 8)? == [0x88, 0x8e]).then(|| &frame[header + 8..])
}

fn channel(number: u16) -> Channel {
    let (band, frequency_mhz) = if number <= 14 {
        (Band::Ghz2, 2407 + 5 * number)
    } else {
        (Band::Ghz5, 5000 + 5 * number)
    };
    Channel {
        band,
        number,
        frequency_mhz,
    }
}

/// The radio kept by a keeper over a model whose AP sits on 2.4 GHz
/// channel 1, under [`US`].
fn wifi() -> (Wifi<Model>, Model) {
    wifi_with(|_| {})
}

/// [`wifi`], with the model set up by `set` first.
fn wifi_with(set: impl FnOnce(&mut Air)) -> (Wifi<Model>, Model) {
    core::mem::forget(crate::test_clock::virtualize());
    let database = regdb(&[(*b"US", &US[..])]);
    let mut air = Air {
        channel: Some(channel(1)),
        ..Air::default()
    };
    set(&mut air);
    let model = Model(Rc::new(RefCell::new(air)));
    let country = database.select(*b"US").unwrap();
    let network = Network {
        ssid: SSID.to_vec(),
        secret: Secret::Psk(Zeroizing::new([0; 32])),
        country: None,
    };
    let radio = Rtl8852c::new(model.clone(), Mac(LOCAL), country, CLOCK).unwrap();
    (
        radio.keep(network, Arc::new(FixedEntropy::default())),
        model,
    )
}

/// Serve the air until `done`, with five milliseconds passing each round.
fn run(
    wifi: &mut Wifi<Model>,
    model: &Model,
    mut done: impl FnMut(&mut Wifi<Model>, &Air) -> bool,
) -> bool {
    for _ in 0..4000 {
        wifi.carrier();
        model.0.borrow_mut().serve();
        if done(wifi, &model.0.borrow()) {
            return true;
        }
        crate::test_clock::advance(CLOCK, 5_000);
    }
    false
}

/// The radio's progress line.
fn progress(wifi: &mut Wifi<Model>) -> String {
    wifi.diagnostic().unwrap()
}

/// Join the model's AP up to the association, its configuration done.
fn associated() -> (Wifi<Model>, Model) {
    let (mut wifi, model) = wifi();
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    (wifi, model)
}

/// The AP's half of the 4-way handshake.
fn authenticator() -> Authenticator {
    let security = Security::select(&RSN, true).unwrap();
    Authenticator::between(
        security.suites(),
        Zeroizing::new([0; 32]),
        Mac(AP),
        Mac(LOCAL),
        security.element(),
    )
}

/// [`associated`], then messages 1 to 4 of the 4-way handshake, m4's
/// release report included.
fn handshaken() -> (Wifi<Model>, Model) {
    let (mut wifi, model) = associated();
    let mut ap = authenticator();
    model
        .0
        .borrow_mut()
        .hear(data(LOCAL, 0, [0x88, 0x8e], &ap.message_1()));
    assert!(run(&mut wifi, &model, |_, air| !air.eapol().is_empty()));
    ap.message_2(&model.0.borrow().eapol()[0]);
    model
        .0
        .borrow_mut()
        .hear(data(LOCAL, 1, [0x88, 0x8e], &ap.message_3(&RSN)));
    assert!(run(&mut wifi, &model, |wifi, air| {
        air.eapol().len() > 1 && progress(wifi).contains("acked 2")
    }));
    (wifi, model)
}

/// Whether `text` holds six `:`-separated pairs of hex digits.
fn names_an_address(text: &str) -> bool {
    let bytes = text.as_bytes();
    bytes.windows(17).any(|w| {
        w.chunks(3).enumerate().all(|(i, part)| {
            part[..2].iter().all(u8::is_ascii_hexdigit) && (i == 5 || part[2] == b':')
        })
    })
}

#[test]
fn a_scan_is_one_passive_offload_of_the_countrys_channels() {
    let (mut wifi, model) = wifi();
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| matches!(c, Call::SetBssid(_))).is_some()
    }));
    let air = model.0.borrow();
    let scans: Vec<&Vec<Channel>> = air
        .calls
        .iter()
        .filter_map(|c| match c {
            Call::BeginScan(channels) => Some(channels),
            _ => None,
        })
        .collect();
    // One offload of every channel the country leaves, 2.4 GHz first.
    let mut want: Vec<u16> = (1..=11).collect();
    want.extend((36..=64).step_by(4));
    want.extend((100..=144).step_by(4));
    want.extend((149..=165).step_by(4));
    let want: Vec<Channel> = want.into_iter().map(channel).collect();
    assert_eq!(scans, [&want]);
    // Each passive: no probe templates (spec S1.3).
    assert!(want.iter().all(|&c| !wifi.radio().probes(c)));
    // Its end restored the chip before the keeper retired it and joined.
    let begun = air.position(|c| matches!(c, Call::BeginScan(_))).unwrap();
    let ended = air.position(|c| *c == Call::EndScan).unwrap();
    let joined = air.position(|c| matches!(c, Call::Tune(_))).unwrap();
    assert!(begun < ended && ended < joined);
    assert_eq!(air.count(|c| *c == Call::EndScan), 1);
    assert_ne!(wifi.radio().stage(), Stage::Scanned);
}

#[test]
fn the_join_tunes_sets_the_bssid_and_authenticates() {
    let (mut wifi, model) = wifi();
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associating
    }));
    let air = model.0.borrow();
    let tune = air.position(|c| *c == Call::Tune(channel(1))).unwrap();
    let bssid = air.position(|c| *c == Call::SetBssid(Mac(AP))).unwrap();
    let auth = air
        .position(|c| matches!(c, Call::Management(b, _) if b[0] == 0xb0))
        .unwrap();
    assert!(tune < bssid && bssid < auth);
    // Open system, transaction 1, to the AP, at the BSS's basic rates
    // (1 Mb/s, bit 0).
    let Call::Management(frame, basic) = &air.calls[auth] else {
        unreachable!()
    };
    assert_eq!(frame[4..10], AP);
    assert_eq!(frame[24..28], [0, 0, 1, 0]);
    assert_eq!(*basic, 0x001);
    // Its token's ack and the AP's answer moved the station on: the chip
    // heard of the AP's station authenticated, then the association
    // request left.
    let authenticated = air.position(|c| *c == Call::Authenticated).unwrap();
    let request = air
        .position(|c| matches!(c, Call::Management(b, _) if b[0] == 0x00))
        .unwrap();
    assert!(auth < authenticated && authenticated < request);
    drop(air);
    assert!(progress(&mut wifi).contains("auth 1/1 status 0"));
}

#[test]
fn association_configures_the_chip_with_the_negotiated_peer() {
    let (_wifi, model) = associated();
    let air = model.0.borrow();
    let at = air.position(|c| matches!(c, Call::Associate(_))).unwrap();
    let Call::Associate(joined) = &air.calls[at] else {
        unreachable!()
    };
    let ac = |aci, aifsn, ecw_min, ecw_max, txop_32us| AccessCategory {
        aci,
        admission_control: false,
        aifsn,
        ecw_min,
        ecw_max,
        txop_32us,
    };
    assert_eq!(
        *joined,
        Joined {
            bssid: Mac(AP),
            aid: 1,
            channel: channel(1),
            beacon_interval: 100,
            dtim_period: 2,
            // 1 and 6 Mb/s.
            legacy: 0x011,
            // The AP's one stream, its SGI 20 and RX STBC, under ours.
            ht: Some(HtPeer {
                rx_mask: [0xff, 0, 0, 0],
                ldpc: false,
                rx_stbc: true,
                sgi20: true,
            }),
            // The AP's WMM parameters, in ACI order.
            edca: Some([
                ac(0, 3, 4, 10, 0),
                ac(1, 7, 4, 10, 0),
                ac(2, 2, 3, 4, 94),
                ac(3, 2, 2, 3, 47),
            ]),
            // The response's capability has no short slot on 2.4 GHz.
            short_slot: false,
        }
    );
    // The join's switch stands: nothing tuned between the response and
    // the association (Chip::associate switches only off its channel).
    let response = air
        .position(|c| matches!(c, Call::Management(b, _) if b[0] == 0x00))
        .unwrap();
    assert!(
        !air.calls[response..at]
            .iter()
            .any(|c| matches!(c, Call::Tune(_)))
    );
}

#[test]
fn an_ap_without_wmm_leaves_its_edca_to_the_chip() {
    // `Negotiated::edca` would hand the chip the no-QoS defaults as the
    // AP's own; the chip programs mac80211's QoS defaults for None.
    let (mut wifi, model) = wifi_with(|air| air.no_wmm = true);
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| matches!(c, Call::Associate(_))).is_some()
    }));
    let air = model.0.borrow();
    let joined = air
        .calls
        .iter()
        .find_map(|c| match c {
            Call::Associate(joined) => Some(joined),
            _ => None,
        })
        .unwrap();
    assert_eq!(joined.edca, None);
}

#[test]
fn eapol_flows_both_ways_and_m3a_stops_at_the_key() {
    let (mut wifi, model) = handshaken();
    let air = model.0.borrow();
    // m2 and m4 went as EAPOL on TID 7, without a key, at the peer's rates.
    let sent: Vec<(u8, bool, u16, bool)> = air
        .calls
        .iter()
        .filter_map(|c| match c {
            Call::Data {
                bytes,
                tid,
                eapol: is_eapol,
                supported,
                keyed,
            } if eapol(bytes).is_some() => Some((*tid, *is_eapol, *supported, *keyed)),
            _ => None,
        })
        .collect();
    assert_eq!(sent, [(7, true, 0x011, false), (7, true, 0x011, false)]);
    drop(air);
    let line = progress(&mut wifi);
    assert!(line.contains("eapol rx 2 tx 2 acked 2"), "{line}");
    assert!(line.contains("keys 0, port closed"), "{line}");
    assert!(line.contains("last \"keys before M3b\""), "{line}");
    // The keeper reports the refusal and the port stays closed.
    let Err(LinkError::Adapter(failure)) = wifi.receive() else {
        panic!("no failure")
    };
    assert!(failure.contains("keys before M3b"), "{failure}");
    assert!(!wifi.carrier());
}

#[test]
fn unacked_authentication_is_retried_and_reported() {
    let (mut wifi, model) = wifi_with(|air| air.deaf_to_auth = true);
    let mut lines = Vec::new();
    let mut gave_up = None;
    let mut joins = Vec::new();
    assert!(run(&mut wifi, &model, |wifi, air| {
        lines.push(progress(wifi));
        let set = air.count(|c| matches!(c, Call::SetBssid(_)));
        if joins.len() < set {
            joins.push(CLOCK.now_us());
        }
        if gave_up.is_none() && air.count(|c| *c == Call::Disassociate) == 1 {
            gave_up = Some(CLOCK.now_us());
        }
        set == 2 && air.management(0xb0).len() == 4
    }));
    // Sent and never acknowledged: auth 1/0, then mac80211's retries at once.
    assert!(lines.iter().any(|l| l.contains("auth 1/0 status -")));
    assert!(lines.iter().any(|l| l.contains("auth 3/0 status -")));
    // The third ended the join; the keeper waited wpa_supplicant's 10 s and
    // joined again.
    assert!(joins[1] - gave_up.unwrap() >= 10_000_000);
    let air = model.0.borrow();
    let first = air.position(|c| matches!(c, Call::SetBssid(_))).unwrap();
    let left = air.position(|c| *c == Call::Disassociate).unwrap();
    let auths = air.calls[first..left]
        .iter()
        .filter(|c| matches!(c, Call::Management(b, _) if b[0] == 0xb0))
        .count();
    assert_eq!(auths, 3);
    // The AP's station never authenticated: the chip was not told it was.
    assert_eq!(air.count(|c| *c == Call::Authenticated), 0);
    drop(air);
    assert!(progress(&mut wifi).contains("auth 4/0 status -"));
    let Err(LinkError::Adapter(failure)) = wifi.receive() else {
        panic!("no failure")
    };
    assert!(failure.contains("authentication or association failed"));
}

#[test]
fn progress_reports_each_stage() {
    let (mut wifi, model) = wifi();
    let chip = "; chip";
    assert_eq!(
        progress(&mut wifi),
        alloc::format!(
            "scans 0 (0 networks, target -), auth -, assoc -, eapol -, keys -, port closed{chip}"
        )
    );
    // The scan found the target, and the join began.
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| matches!(c, Call::SetBssid(_))).is_some()
    }));
    assert_eq!(
        progress(&mut wifi),
        alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), auth 0/0 status -, assoc -, \
             eapol -, keys -, port closed{chip}"
        )
    );
    // Authenticated; the association request is out.
    assert!(run(&mut wifi, &model, |_, air| !air
        .management(0x00)
        .is_empty()));
    assert_eq!(
        progress(&mut wifi),
        alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), auth 1/1 status 0, \
             assoc 1/0 status - aid -, eapol -, keys -, port closed{chip}"
        )
    );
    // Associated.
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    assert_eq!(
        progress(&mut wifi),
        alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), auth 1/1 status 0, \
             assoc 1/1 status 0 aid 1, eapol rx 0 tx 0 acked 0, keys -, port closed{chip}"
        )
    );
    // The handshake, to M3a's planned stop.
    drop((wifi, model));
    let (mut wifi, _model) = handshaken();
    assert_eq!(
        progress(&mut wifi),
        alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), auth 1/1 status 0, \
             assoc 1/1 status 0 aid 1, eapol rx 2 tx 2 acked 2, keys 0, port closed, \
             last \"keys before M3b\"{chip}"
        )
    );
}

#[test]
fn diagnostic_names_no_address() {
    let (mut wifi, model) = associated();
    let mut lines = vec![progress(&mut wifi)];
    let mut ap = authenticator();
    model
        .0
        .borrow_mut()
        .hear(data(LOCAL, 0, [0x88, 0x8e], &ap.message_1()));
    assert!(run(&mut wifi, &model, |wifi, air| {
        lines.push(progress(wifi));
        !air.eapol().is_empty()
    }));
    ap.message_2(&model.0.borrow().eapol()[0]);
    model
        .0
        .borrow_mut()
        .hear(data(LOCAL, 1, [0x88, 0x8e], &ap.message_3(&RSN)));
    assert!(run(&mut wifi, &model, |wifi, air| {
        lines.push(progress(wifi));
        air.count(|c| *c == Call::Disassociate) == 1
    }));
    assert!(names_an_address("ab:cd:ef:01:23:45"));
    for line in &lines {
        assert!(!names_an_address(line), "{line}");
        assert!(!line.contains("test"), "{line}");
    }
}

#[test]
fn a_deauthentication_sends_and_disassociates() {
    let (mut wifi, model) = associated();
    wifi.radio_mut().deauthenticate(3).unwrap();
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    let air = model.0.borrow();
    let deauth = air
        .position(|c| matches!(c, Call::Management(b, _) if b[0] == 0xc0))
        .unwrap();
    let left = air.position(|c| *c == Call::Disassociate).unwrap();
    assert!(deauth < left);
    let Call::Management(frame, _) = &air.calls[deauth] else {
        unreachable!()
    };
    assert_eq!((&frame[4..10], &frame[24..]), (&AP[..], &[3, 0][..]));
}

#[test]
fn a_join_that_authenticated_but_never_associated_leaves_through_the_chip() {
    // mac80211 moves the AP's station AUTH, then, at the association's
    // timeout, AUTH to NONE: the chip's disconnect H2Cs (spec S11).
    let (mut wifi, model) = wifi_with(|air| air.mute_on_assoc = true);
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    let air = model.0.borrow();
    let authenticated = air.position(|c| *c == Call::Authenticated).unwrap();
    let left = air.position(|c| *c == Call::Disassociate).unwrap();
    assert!(authenticated < left);
    assert_eq!(air.management(0x00).len(), 3);
    assert_eq!(air.count(|c| matches!(c, Call::Associate(_))), 0);
}

#[test]
fn a_full_chip_holds_frames_in_order_without_loss() {
    // Review Focus 2 on the radio's side: management frames wait in the
    // outbox, data in `held` (at most 256, the oldest dropped).
    let (mut wifi, model) = wifi_with(|air| air.full_management = true);
    assert!(run(&mut wifi, &model, |_, air| air.refused >= 10));
    assert!(model.0.borrow().management(0xb0).is_empty());
    model.0.borrow_mut().full_management = false;
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    // The authentication waited and went once.
    assert_eq!(model.0.borrow().management(0xb0).len(), 1);
    // Data waits while the queues are full, the oldest dropped past 256.
    model.0.borrow_mut().full_data = true;
    for n in 0..300u16 {
        let mut frame = Vec::from(AP);
        frame.extend(LOCAL);
        frame.extend([0x88, 0x8e]);
        frame.extend(n.to_be_bytes());
        wifi.radio_mut().send_control(&frame, None).unwrap();
    }
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    assert_eq!(model.0.borrow().eapol().len(), 0);
    model.0.borrow_mut().full_data = false;
    assert!(run(&mut wifi, &model, |_, air| air.eapol().len() == 256));
    let sent: Vec<u16> = model
        .0
        .borrow()
        .eapol()
        .iter()
        .map(|body| u16::from_be_bytes([body[0], body[1]]))
        .collect();
    assert_eq!(sent, (44..300).collect::<Vec<u16>>());
    // Each sequence number once, in order.
    let air = model.0.borrow();
    let sequences: Vec<u16> = air
        .calls
        .iter()
        .filter_map(|c| match c {
            Call::Data { bytes, .. } => Some(u16::from_le_bytes([bytes[22], bytes[23]]) >> 4),
            _ => None,
        })
        .collect();
    assert!(sequences.windows(2).all(|w| w[1] == (w[0] + 1) & 0xfff));
}

#[test]
fn a_deauthentication_drops_the_data_still_held() {
    // ieee80211_set_disassoc's flush with drop: what the chip has not
    // taken never leaves after the decision to deauthenticate.
    let (mut wifi, model) = associated();
    model.0.borrow_mut().full_data = true;
    let mut frame = Vec::from(AP);
    frame.extend(LOCAL);
    frame.extend([0x88, 0x8e, 1, 2, 3]);
    wifi.radio_mut().send_control(&frame, None).unwrap();
    wifi.radio_mut().deauthenticate(3).unwrap();
    model.0.borrow_mut().full_data = false;
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    let air = model.0.borrow();
    assert_eq!(air.count(|c| matches!(c, Call::Data { .. })), 0);
    assert_eq!(air.management(0xc0).len(), 1);
}

#[test]
fn a_beacon_failing_its_fcs_finds_no_network() {
    let (mut wifi, model) = wifi_with(|air| air.bad_fcs = true);
    assert!(run(&mut wifi, &model, |wifi, _| {
        progress(wifi).starts_with("scans 1 ")
    }));
    assert!(progress(&mut wifi).starts_with("scans 1 (0 networks, target missing)"));
    assert_eq!(
        model.0.borrow().count(|c| matches!(c, Call::SetBssid(_))),
        0
    );
}

#[test]
fn the_radio_waits_30_s_for_carrier_and_polls() {
    let (wifi, _model) = wifi();
    assert_eq!(wifi.carrier_wait_us(), WIFI_CARRIER_WAIT_US);
    assert_eq!(WIFI_CARRIER_WAIT_US, 30_000_000);
    assert!(!wifi.signals());
    assert_eq!(wifi.due_us(), None);
}

#[test]
fn the_advertisement_is_ht20_with_two_streams() {
    let (wifi, _model) = wifi();
    for band in [Band::Ghz2, Band::Ghz5] {
        let advertisement = wifi.radio().advertisement(band).unwrap();
        let elements = &advertisement.elements;
        let ht = elements.windows(2).position(|w| w == [45, 26]).unwrap();
        // HT capability 0x09AD, A-MPDU 0x03, MCS 0-15, 144 Mb/s, TX defined.
        assert_eq!(elements[ht + 2..ht + 5], [0xAD, 0x09, 0x03]);
        assert_eq!(
            elements[ht + 5..ht + 15],
            [0xff, 0xff, 0, 0, 0, 0, 0, 0, 0, 0]
        );
        assert_eq!(elements[ht + 15..ht + 18], [144, 0, 1]);
        // No VHT or HE (spec S1.7).
        assert!(!elements.windows(2).any(|w| w == [191, 12]));
        assert!(advertisement.capabilities.vht().is_none());
    }
}

#[test]
fn a_frame_the_station_refuses_is_reported_but_is_no_refusal() {
    // The AP's association response again, once associated: the station
    // refuses it, the keeper hears of it once, and the progress line's
    // `last` stays the radio's own refusals.
    let (mut wifi, model) = associated();
    let mut body = vec![0x11, 0, 0, 0, 1, 0xc0];
    body.extend(RATES);
    model.0.borrow_mut().hear(management(0x10, &body));
    assert!(run(&mut wifi, &model, |_, _| true));
    assert!(!progress(&mut wifi).contains("last"));
    let Err(LinkError::Adapter(failure)) = wifi.receive() else {
        panic!("no failure")
    };
    assert_eq!(
        failure,
        "rtw89 unsupported: a management frame the station refuses"
    );
}

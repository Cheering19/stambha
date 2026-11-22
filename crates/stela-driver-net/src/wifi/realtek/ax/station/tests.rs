// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright (C) 2026 Stela contributors
//! The rtw89 radio through the keeper, over a model of the chip that plays
//! the AP, shaped as rtw88's `station/tests.rs` `Air`: a firmware scan
//! hears the AP's beacon on its channel and ends, the AP answers
//! authentication and association, and EAPOL runs against the supplicant
//! tests' `Authenticator`. The model decrypts nothing: it records the keys
//! the station installs and the key each data frame names, and hands back
//! the AP's protected frames as the hardware leaves them, marked decrypted
//! with their CCMP header and MIC in place.
use super::chip::{
    Association, Chip, Data, HardwareKey, Incoming, LinkCounts, Management, Received, Token,
    TxCounts,
};
use super::*;
use crate::LinkError;
use crate::wifi::connection::security::Security;
use crate::wifi::intel::policy::tests::regdb;
use crate::wifi::network::{Network, Secret};
use crate::wifi::realtek::ax::c2h::RaReport;
use crate::wifi::realtek::ax::regd::Regd;
use crate::wifi::realtek::ax::rx::{Rate, RateMode};
use crate::wifi::realtek::ccmp;
use crate::wifi::regulatory::channel::NO_IR;
use crate::wifi::sae::{self, Derivation, Exchange};
use crate::wifi::station::multi_link::Mld;
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
/// [`RSN`] with management frame protection capable (MFPC, b7), so the
/// station protects its management frames (`ieee80211w=1`).
const RSN_PMF: [u8; 22] = [
    48, 20, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 2, 0x80, 0,
];
/// The IGTK KDE a PMF message 3 carries: key ID 4, IPN 9, the key 0x77s.
const IGTK_KDE: [u8; 30] = [
    0xdd, 28, 0, 0x0f, 0xac, 9, 4, 0, 9, 0, 0, 0, 0, 0, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77,
    0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77,
];
/// [`RSN`] of a WPA3-Personal AP: SAE (00-0F-AC:8), management frame
/// protection required and capable (MFPR b6, MFPC b7).
const RSN_SAE: [u8; 22] = [
    48, 20, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 4, 1, 0, 0, 0x0f, 0xac, 8, 0xc0, 0,
];
/// An SAE network's passphrase.
const PASSPHRASE: &[u8] = b"correct horse battery";
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

/// The CAM entry, key ID and PN a data frame goes under, if any.
type FrameKey = Option<(u8, u8, u64)>;

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
    /// A data frame, with the CAM entry, key ID and PN it went under, and
    /// the A-MPDU it went in.
    Data {
        bytes: Vec<u8>,
        tid: u8,
        eapol: bool,
        supported: u16,
        key: FrameKey,
        aggregate: Option<txd::Aggregate>,
    },
    Associate(Joined),
    Disassociate,
    /// A key the chip installed, in CAM entry `cam`.
    AddKey {
        pairwise: bool,
        index: u8,
        material: Vec<u8>,
        cam: u8,
    },
    RemoveKey(u8),
    /// mac80211's flush, with drop or without.
    Flush(bool),
    /// `rtw89_ops_ampdu_action`'s RX_START and RX_STOP.
    StartBa {
        tid: u8,
        ssn: u16,
        window: u16,
    },
    StopBa(u8),
    /// `rtw89_ops_ampdu_action`'s TX_OPERATIONAL with its window, and
    /// TX_STOP (`None`).
    Aggregation {
        tid: u8,
        window: Option<u16>,
    },
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
    /// The AP's association response grants the short slot (capability
    /// b10), which its beacon does not.
    short_slot_response: bool,
    /// The scan's beacon fails its FCS.
    bad_fcs: bool,
    /// The chip has no room for management or for data frames.
    full_management: bool,
    full_data: bool,
    /// Frames refused for no room.
    refused: usize,
    /// The AP protects management frames (beacons [`RSN_PMF`]).
    pmf: bool,
    /// The firmware never acknowledges a key.
    refuse_keys: bool,
    /// The CAM entries holding a key.
    cam: Vec<u8>,
    /// An authentication's report comes only after the association
    /// request has been taken, as the M3a boot saw it.
    late_auth_report: bool,
    /// Reports held for [`Air::late_auth_report`].
    late: Vec<Incoming>,
    /// The AP asks for a Block Ack agreement right after its association
    /// response, heard in the same poll.
    addba_with_assoc: bool,
    /// The chip cannot send its BA CAM H2C.
    ba_fails: bool,
    /// The AP is WPA3-Personal (beacons [`RSN_SAE`]) and answers SAE under
    /// [`PASSPHRASE`]: a station's commit gets its commit, the station's
    /// confirm its confirm ([`Air::sae_confirm`] until then).
    sae: bool,
    /// The AP's half of the last exchange, and its confirm, held.
    sae_ap: Option<Exchange>,
    sae_confirm: Option<sae::Frame>,
    /// The AP sends its confirm with its commit, before the station's.
    sae_eager: bool,
    /// The AP sends a deauthentication before its answer to a commit.
    sae_deauth: bool,
    /// The AP's commit to another station comes before its answer to the
    /// station's commit.
    sae_stray: bool,
    /// The AP's confirm does not verify, as under another passphrase.
    sae_wrong_confirm: bool,
    /// The chip reports beacon loss during a scan, before the beacon it
    /// hears, and with the AP's answer to an authentication.
    lost_unassociated: bool,
    /// The rate and A-MPDU bit the chip hears the AP's frames at (M4 spec S3.1).
    rx_rate: Option<Rate>,
    rx_ampdu: bool,
    /// Elements the AP's beacon carries past its HT ones.
    beacon_extra: Vec<u8>,
    /// What the chip counts (M4 spec S3.2, S3.4).
    link: LinkCounts,
    /// Data frames the chip took and has not kicked yet, and the kicks that
    /// handed any over (M4 spec S9.1): only a kicked frame reaches the AP.
    unkicked: Vec<(Token, Vec<u8>)>,
    kicks: usize,
    /// The AP has no HT: it carries neither HT element.
    no_ht: bool,
    /// The AP answers the station's unprotected ADDBA Requests with
    /// `addba_status` (0, success) at the request's window, or at
    /// `addba_window` when set; it never answers when `addba_silent`.
    addba_window: Option<u16>,
    addba_status: u16,
    addba_silent: bool,
    /// Every frame in the order the AP hears it: a management frame as the
    /// chip takes it, data as a kick hands it over.
    heard: Vec<Vec<u8>>,
}

impl Air {
    fn hear(&mut self, bytes: Vec<u8>) {
        self.hear_as(bytes, false, false);
    }

    /// A frame the hardware `decrypted`, or found failing its ICV.
    fn hear_as(&mut self, bytes: Vec<u8>, decrypted: bool, icv_error: bool) {
        let channel = self.channel.unwrap();
        self.incoming.push_back(Incoming::Frame(Received {
            bytes,
            channel,
            signal_dbm: None,
            tsf: 0,
            decrypted,
            icv_error,
            fcs_error: false,
            rate: self.rx_rate,
            ampdu: self.rx_ampdu,
        }));
    }

    /// Each frame's release report, and the AP's answer to it.
    fn serve(&mut self) {
        while let Some((token, bytes)) = self.unanswered.pop_front() {
            let deaf = bytes[0] == 0xb0 && self.deaf_to_auth;
            let report = Incoming::Transmitted {
                token,
                acknowledged: !deaf,
            };
            if bytes[0] == 0x00 {
                let late = core::mem::take(&mut self.late);
                self.incoming.extend(late);
            }
            if bytes[0] == 0xb0 && self.late_auth_report {
                self.late.push(report);
            } else {
                self.incoming.push_back(report);
            }
            if deaf {
                continue;
            }
            match bytes[0] {
                0xb0 if self.sae => self.answer_sae(&bytes),
                0xb0 => {
                    self.hear(management(0xb0, &[0, 0, 2, 0, 0, 0]));
                    if self.lost_unassociated {
                        self.incoming.push_back(Incoming::BeaconLost);
                    }
                }
                0x00 if !self.mute_on_assoc => {
                    let slot = if self.short_slot_response { 0x04 } else { 0 };
                    let mut body = vec![0x11, slot, 0, 0, 1, 0xc0];
                    body.extend(rates(self.channel.unwrap()));
                    // HT Operation's primary channel, as the beacon's.
                    let mut ht = HT;
                    ht[30] = self.channel.unwrap().number as u8;
                    if !self.no_ht {
                        body.extend(ht);
                    }
                    if !self.no_wmm {
                        body.extend(WMM);
                    }
                    self.hear(management(0x10, &body));
                    if self.addba_with_assoc {
                        self.hear(addba(1, 0, 64, 0));
                    }
                }
                0xd0 if bytes[1] & 0x40 == 0 && bytes[24..26] == [3, 0] && !self.addba_silent => {
                    self.hear(addba_response(&bytes, self.addba_window, self.addba_status));
                }
                _ => {}
            }
        }
    }

    /// The AP's side of SAE (algorithm 3) for the station's frame `bytes`:
    /// each commit begins a new exchange, as hostapd's for a new attempt.
    fn answer_sae(&mut self, bytes: &[u8]) {
        let field = |at: usize| u16::from_le_bytes([bytes[at], bytes[at + 1]]);
        assert_eq!(field(24), 3, "not SAE");
        let (transaction, status, data) = (field(26), field(28), &bytes[30..]);
        match transaction {
            1 => {
                if self.sae_deauth {
                    self.hear(management(0xc0, &[1, 0]));
                }
                if self.sae_stray {
                    let mut stray = sae_frame(&sae_ap_for(OTHER_STATION).commit());
                    stray[4..10].copy_from_slice(&OTHER_STATION);
                    self.hear(stray);
                }
                let mut ap = sae_ap();
                let mut confirm = ap.receive(1, status, data).unwrap().unwrap();
                if self.sae_wrong_confirm {
                    // The confirm field, past the send-confirm counter.
                    confirm.data[2] ^= 1;
                }
                self.hear(sae_frame(&ap.commit()));
                if self.sae_eager {
                    self.hear(sae_frame(&confirm));
                } else {
                    self.sae_confirm = Some(confirm);
                }
                self.sae_ap = Some(ap);
            }
            2 => {
                let ap = self.sae_ap.as_mut().unwrap();
                assert_eq!(ap.receive(2, status, data), Ok(None));
                assert!(ap.accepted());
                if let Some(confirm) = self.sae_confirm.take() {
                    self.hear(sae_frame(&confirm));
                }
            }
            _ => panic!("SAE transaction {transaction}"),
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

    /// The data frames the chip took, with the key each named.
    fn data(&self) -> Vec<(Vec<u8>, FrameKey)> {
        self.calls
            .iter()
            .filter_map(|c| match c {
                Call::Data { bytes, key, .. } => Some((bytes.clone(), *key)),
                _ => None,
            })
            .collect()
    }

    /// The keys installed, as (pairwise, index, CAM entry).
    fn keys(&self) -> Vec<(bool, u8, u8)> {
        self.calls
            .iter()
            .filter_map(|c| match c {
                Call::AddKey {
                    pairwise,
                    index,
                    cam,
                    ..
                } => Some((*pairwise, *index, *cam)),
                _ => None,
            })
            .collect()
    }

    /// The pairwise key's material.
    fn ptk(&self) -> [u8; 16] {
        self.calls
            .iter()
            .find_map(|c| match c {
                Call::AddKey {
                    pairwise: true,
                    material,
                    ..
                } => material[..].try_into().ok(),
                _ => None,
            })
            .unwrap()
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
            if air.lost_unassociated {
                air.incoming.push_back(Incoming::BeaconLost);
            }
            if channels.contains(&channel) {
                let bad = air.bad_fcs;
                let rate = air.rx_rate;
                let bytes = beacon(&air);
                air.incoming.push_back(Incoming::Frame(Received {
                    bytes,
                    channel,
                    signal_dbm: Some(-40),
                    tsf: 0,
                    decrypted: false,
                    icv_error: false,
                    fcs_error: bad,
                    rate,
                    ampdu: false,
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
        air.heard.push(frame.bytes.to_vec());
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
        if let Some((cam, _, _)) = frame.key
            && !air.cam.contains(&cam)
        {
            return Err("a key the CAM does not hold".into());
        }
        air.calls.push(Call::Data {
            bytes: frame.bytes.to_vec(),
            tid: frame.tid,
            eapol: frame.eapol,
            supported: frame.supported_rates,
            key: frame.key,
            aggregate: frame.aggregate,
        });
        let token = air.token();
        air.unkicked.push((token, frame.bytes.to_vec()));
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
    /// Leaving deletes what keys are left, as `Live::leave` does.
    fn disassociate(&mut self) -> Result<(), String> {
        let mut air = self.0.borrow_mut();
        air.calls.push(Call::Disassociate);
        air.cam.clear();
        Ok(())
    }
    /// The lowest free CAM entry, unless the firmware never acknowledges.
    fn add_key(&mut self, key: &HardwareKey<'_>) -> Result<u8, String> {
        let mut air = self.0.borrow_mut();
        if air.refuse_keys {
            return Err("the firmware never acknowledged the key".into());
        }
        let cam = (0..).find(|cam| !air.cam.contains(cam)).unwrap();
        air.cam.push(cam);
        air.calls.push(Call::AddKey {
            pairwise: key.pairwise,
            index: key.index,
            material: key.material.to_vec(),
            cam,
        });
        Ok(cam)
    }
    fn remove_key(&mut self, cam: u8) -> Result<(), String> {
        let mut air = self.0.borrow_mut();
        air.calls.push(Call::RemoveKey(cam));
        let at = air
            .cam
            .iter()
            .position(|&held| held == cam)
            .ok_or("no key in that entry")?;
        air.cam.remove(at);
        Ok(())
    }
    fn flush(&mut self, drop: bool) -> Result<(), String> {
        // Live::flush kicks what was submitted first.
        self.kick()?;
        self.0.borrow_mut().calls.push(Call::Flush(drop));
        Ok(())
    }
    fn kick(&mut self) -> Result<(), String> {
        let mut air = self.0.borrow_mut();
        if !air.unkicked.is_empty() {
            air.kicks += 1;
            let kicked = core::mem::take(&mut air.unkicked);
            air.heard
                .extend(kicked.iter().map(|(_, bytes)| bytes.clone()));
            air.unanswered.extend(kicked);
        }
        Ok(())
    }
    /// Recorded, then refused when the BA CAM H2C cannot be sent.
    fn start_rx_block_ack(&mut self, tid: u8, ssn: u16, window: u16) -> Result<(), String> {
        let mut air = self.0.borrow_mut();
        air.calls.push(Call::StartBa { tid, ssn, window });
        if air.ba_fails {
            return Err("the BA CAM H2C could not be sent".into());
        }
        Ok(())
    }
    fn stop_rx_block_ack(&mut self, tid: u8) -> Result<(), String> {
        self.0.borrow_mut().calls.push(Call::StopBa(tid));
        Ok(())
    }
    fn aggregation(&mut self, tid: u8, window: Option<u16>) -> Result<(), String> {
        self.0
            .borrow_mut()
            .calls
            .push(Call::Aggregation { tid, window });
        Ok(())
    }
    fn diagnostic(&self) -> String {
        "chip".into()
    }
    fn link_counts(&self) -> LinkCounts {
        self.0.borrow().link
    }
}

/// The AP's rates on 2.4 GHz: 1 Mb/s (basic) and 6 Mb/s.
const RATES: [u8; 4] = [1, 2, 0x82, 0x0c];

/// The AP's rates on `channel`: [`RATES`] on 2.4 GHz, 6 Mb/s (basic) and 9 Mb/s
/// elsewhere.
fn rates(channel: Channel) -> [u8; 4] {
    match channel.band {
        Band::Ghz2 => RATES,
        _ => [1, 2, 0x8c, 0x12],
    }
}

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

/// Another station of the AP's BSS.
const OTHER_STATION: [u8; 6] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x99];

/// The AP's half of an SAE exchange with the station under [`PASSPHRASE`].
fn sae_ap() -> Exchange {
    sae_ap_for(LOCAL)
}

/// The AP's half of an SAE exchange with `station` under [`PASSPHRASE`].
fn sae_ap_for(station: [u8; 6]) -> Exchange {
    let mut next = 0x30u8;
    let mut random = || {
        next += 1;
        [next; 32]
    };
    Exchange::new(
        Derivation::HuntingAndPecking,
        PASSPHRASE,
        None,
        None,
        Mac(AP),
        Mac(station),
        &mut random,
    )
    .unwrap()
}

/// The AP's SAE authentication frame carrying `frame`.
fn sae_frame(frame: &sae::Frame) -> Vec<u8> {
    let mut body = vec![3, 0];
    body.extend(frame.transaction.to_le_bytes());
    body.extend(frame.status.to_le_bytes());
    body.extend(&frame.data);
    management(0xb0, &body)
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
    frame.extend(rates(air.channel.unwrap()));
    // DS parameter set, then a TIM with DTIM period 2.
    frame.extend([3, 1, channel, 5, 4, 0, 2, 0, 0]);
    frame.extend(if air.sae {
        RSN_SAE
    } else if air.pmf {
        RSN_PMF
    } else {
        RSN
    });
    if !air.no_ht {
        frame.extend(HT);
        // HT Operation's primary channel.
        let at = frame.len() - 22;
        frame[at] = channel;
    }
    frame.extend(&air.beacon_extra);
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

/// [`data`] protected under key ID `keyid` at packet number `pn`, as the
/// hardware hands it on decrypted: its CCMP header after the MAC header,
/// the plaintext, then the MIC (zeros here).
fn protected(receiver: [u8; 6], sequence: u16, keyid: u8, pn: u64, body: &[u8]) -> Vec<u8> {
    protected_as(receiver, sequence, keyid, pn, [8, 0], body)
}

/// [`protected`] with the LLC's EtherType `kind`.
fn protected_as(
    receiver: [u8; 6],
    sequence: u16,
    keyid: u8,
    pn: u64,
    kind: [u8; 2],
    body: &[u8],
) -> Vec<u8> {
    let mut frame = data(receiver, sequence, kind, body);
    frame[1] |= 0x40;
    let [p0, p1, p2, p3, p4, p5, _, _] = pn.to_le_bytes();
    let header = [p0, p1, 0, 0x20 | keyid << 6, p2, p3, p4, p5];
    frame.splice(26..26, header);
    frame.extend([0; 8]);
    frame
}

/// An IPv4 frame from the station to the AP, its body `fill`.
fn ip(fill: u8) -> Vec<u8> {
    let mut ethernet = Vec::from(AP);
    ethernet.extend(LOCAL);
    ethernet.extend([8, 0, 0x45, 0]);
    ethernet.extend([fill; 44]);
    ethernet
}

/// What the station holds for the network, as each frame's first payload
/// byte past its IPv4 type.
fn delivered(wifi: &mut Wifi<Model>) -> Vec<u8> {
    let mut values = Vec::new();
    while let Some(ethernet) = wifi.receive().unwrap() {
        values.push(ethernet[14]);
    }
    values
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
    // SAE needs the passphrase (`Security::select_on`).
    let secret = if air.sae {
        Secret::parse(PASSPHRASE).unwrap()
    } else {
        Secret::Psk(Zeroizing::new([0; 32]))
    };
    let model = Model(Rc::new(RefCell::new(air)));
    let country = database.select(*b"US").unwrap();
    let network = Network {
        ssid: SSID.to_vec(),
        secret,
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

/// The progress line's link fields (M4 spec S3.4) before any join, and for
/// the model's AP on 2.4 GHz channel 1 at HT20: no RA report, rate or TX
/// count yet.
const LINK_IDLE: &str = "ap -, regd -, rx -, ampdu -, rxq full 0, tx ra -, \
     kicks 0 rel 0 (-/kick), addba 0/0, agg -";
const LINK_CH1: &str = "ap ht20 ch 1 (c 1), regd -, rx -, ampdu -, rxq full 0, tx ra -, \
     kicks 0 rel 0 (-/kick), addba 0/0, agg -";

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
    authenticator_for(&RSN)
}

/// The AP's half of the 4-way handshake under `rsn`.
fn authenticator_for(rsn: &[u8]) -> Authenticator {
    let security = Security::select(rsn, true).unwrap();
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
    shake(&mut wifi, &model, &RSN, &[]);
    (wifi, model)
}

/// Messages 1 to 4 of the 4-way handshake under `rsn`, message 3 carrying
/// `extra` KDEs, m4's release report included.
fn shake(wifi: &mut Wifi<Model>, model: &Model, rsn: &[u8], extra: &[u8]) {
    shake_with(wifi, model, authenticator_for(rsn), rsn, extra);
}

/// [`shake`] with `ap` as the AP's half.
fn shake_with(
    wifi: &mut Wifi<Model>,
    model: &Model,
    mut ap: Authenticator,
    rsn: &[u8],
    extra: &[u8],
) {
    model
        .0
        .borrow_mut()
        .hear(data(LOCAL, 0, [0x88, 0x8e], &ap.message_1()));
    assert!(run(wifi, model, |_, air| !air.eapol().is_empty()));
    ap.message_2(&model.0.borrow().eapol()[0]);
    model
        .0
        .borrow_mut()
        .hear(data(LOCAL, 1, [0x88, 0x8e], &ap.message_3_with(rsn, extra)));
    assert!(run(wifi, model, |wifi, air| {
        air.eapol().len() > 1 && progress(wifi).contains("acked 2")
    }));
}

/// [`handshaken`], the port open.
fn connected() -> (Wifi<Model>, Model) {
    let (mut wifi, model) = handshaken();
    assert!(wifi.carrier(), "{}", progress(&mut wifi));
    (wifi, model)
}

/// A join to an AP protecting management frames, through its handshake
/// with an IGTK to an open port.
fn connected_with_pmf() -> (Wifi<Model>, Model) {
    let (mut wifi, model) = wifi_with(|air| air.pmf = true);
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    assert!(wifi.radio().station().requires_protected_management());
    shake(&mut wifi, &model, &RSN_PMF, &IGTK_KDE);
    assert!(wifi.carrier(), "{}", progress(&mut wifi));
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
fn the_slot_is_the_association_responses_not_the_beacons() {
    // At association the slot is the response's, not the scan beacon's:
    // mac80211 keeps the response's capability as `assoc_capability`
    // (`mlme.c:6339`) and takes the slot from it
    // (`ieee80211_handle_bss_capability`, `mlme.c:4344`). mac80211 then
    // follows changed beacons (`mlme.c:8395`), which this radio does not.
    let (mut wifi, model) = wifi_with(|air| air.short_slot_response = true);
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    let air = model.0.borrow();
    assert_eq!(beacon(&air)[35] & 0x04, 0);
    let joined = air
        .calls
        .iter()
        .find_map(|c| match c {
            Call::Associate(joined) => Some(joined),
            _ => None,
        })
        .unwrap();
    assert_eq!(joined.channel.band, Band::Ghz2);
    assert!(joined.short_slot);
}

#[test]
fn eapol_flows_both_ways_before_any_key() {
    let (mut wifi, model) = handshaken();
    let air = model.0.borrow();
    // m2 and m4 went as EAPOL on TID 7, without a key, at the peer's rates:
    // m4 was handed to the chip before wpa_supplicant installs the PTK.
    let sent: Vec<(u8, bool, u16, FrameKey)> = air
        .calls
        .iter()
        .filter_map(|c| match c {
            Call::Data {
                bytes,
                tid,
                eapol: is_eapol,
                supported,
                key,
                ..
            } if eapol(bytes).is_some() => Some((*tid, *is_eapol, *supported, *key)),
            _ => None,
        })
        .collect();
    assert_eq!(sent, [(7, true, 0x011, None), (7, true, 0x011, None)]);
    let data: Vec<usize> = (0..air.calls.len())
        .filter(|&at| matches!(air.calls[at], Call::Data { .. }))
        .collect();
    let m4 = data[1];
    let ptk = air.position(|c| matches!(c, Call::AddKey { .. })).unwrap();
    assert!(m4 < ptk);
    drop(air);
    let line = progress(&mut wifi);
    assert!(line.contains("eapol rx 2 tx 2 acked 2"), "{line}");
}

#[test]
fn the_handshake_installs_ptk_then_gtk_and_opens_the_port() {
    let (mut wifi, model) = connected();
    // `rtw89_ops_set_key` for the PTK (entry 0), then the GTK of key ID 1
    // (entry 1), each with its own material (spec S12).
    let air = model.0.borrow();
    assert_eq!(air.keys(), [(true, 0, 0), (false, 1, 1)]);
    let gtk: Vec<u8> = (0x50..0x60).collect();
    assert!(air.calls.iter().any(|c| matches!(
        c,
        Call::AddKey { pairwise: false, material, .. } if *material == gtk
    )));
    drop(air);
    assert!(wifi.carrier());
    let line = progress(&mut wifi);
    assert!(line.contains("keys 2, port open"), "{line}");
    assert!(!line.contains("last"), "{line}");
    assert_eq!(wifi.receive(), Ok(None));
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
            "scans 0 (0 networks, target -), auth -, assoc -, eapol -, keys -, port closed, \
             {LINK_IDLE}{chip}"
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
             eapol -, keys -, port closed, {LINK_CH1}{chip}"
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
             assoc 1/0 status - aid -, eapol -, keys -, port closed, {LINK_CH1}{chip}"
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
             assoc 1/1 status 0 aid 1, eapol rx 0 tx 0 acked 0, keys -, port closed, \
             {LINK_CH1}, rx data 0 sealed 0 nokey 0 refused 0, tx data 0{chip}"
        )
    );
    // The handshake, its two keys and the port (M3b's pass).
    drop((wifi, model));
    let (mut wifi, _model) = connected();
    assert_eq!(
        progress(&mut wifi),
        alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), auth 1/1 status 0, \
             assoc 1/1 status 0 aid 1, eapol rx 2 tx 2 acked 2, keys 2, port open, \
             {LINK_CH1}, rx data 0 sealed 0 nokey 0 refused 0, tx data 0{chip}"
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
    assert!(run(&mut wifi, &model, |wifi, _| {
        let line = progress(wifi);
        lines.push(line.clone());
        line.contains("port open")
    }));
    // Then the station leaves.
    wifi.radio_mut().deauthenticate(3).unwrap();
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
fn beacon_loss_while_associated_reconnects() {
    // Review Focus 5, spec S15 (ruling R2): the firmware's loss of the AP's
    // beacons ends the association as `ieee80211_sta_connection_lost` does
    // with tx false: the flush with drop and no frame of the station's own,
    // then the keys and the leaving. The keeper reconnects after
    // RECONNECT_US (100 ms), and the progress line counts the loss.
    let (mut wifi, model) = connected();
    let from = model.0.borrow().calls.len();
    let scans = |air: &Air| air.count(|c| matches!(c, Call::BeginScan(_)));
    let before = scans(&model.0.borrow());
    model
        .0
        .borrow_mut()
        .incoming
        .push_back(Incoming::BeaconLost);
    assert!(run(&mut wifi, &model, |_, air| {
        air.calls[from..].contains(&Call::Disassociate)
    }));
    let left = CLOCK.now_us();
    assert_eq!(
        leaving(&model.0.borrow(), from),
        [
            "flush true",
            "flush false",
            "remove 0",
            "flush false",
            "remove 1",
            "leave"
        ]
    );
    assert_eq!(
        wifi.receive(),
        Err(LinkError::Adapter(String::from(
            "rtw89: the AP ended the association"
        )))
    );
    let line = progress(&mut wifi);
    assert!(
        line.contains(&alloc::format!(
            ", port closed, {LINK_CH1}, rx data 0 sealed 0 nokey 0 refused 0, tx data 0, \
             beacon lost 1; "
        )),
        "{line}"
    );
    assert!(run(&mut wifi, &model, |_, air| scans(air) > before));
    assert!(CLOCK.now_us() - left >= 90_000);
    // The keeper joins again; the count stays.
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    assert_eq!(
        model.0.borrow().count(|c| matches!(c, Call::Associate(_))),
        2
    );
    assert!(progress(&mut wifi).contains("beacon lost 1"));
}

#[test]
fn beacon_loss_while_scanning_is_ignored() {
    // Linux hands mac80211 a beacon loss only outside a scan
    // (`rtwdev->scanning`), and mac80211 takes one only while associated:
    // a loss during the scan, or with the authentication's answer, changes
    // nothing, and the join goes on.
    let (mut wifi, model) = wifi_with(|air| air.lost_unassociated = true);
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    let air = model.0.borrow();
    assert_eq!(air.count(|c| matches!(c, Call::BeginScan(_))), 1);
    assert_eq!(air.count(|c| *c == Call::Disassociate), 0);
    drop(air);
    assert_eq!(wifi.receive(), Ok(None));
    let line = progress(&mut wifi);
    assert!(!line.contains("beacon lost"), "{line}");
}

#[test]
fn a_beacon_loss_while_leaving_is_not_counted() {
    // The station already departs: the loss is not the connection's end.
    let (mut wifi, model) = connected();
    wifi.radio_mut().deauthenticate(3).unwrap();
    model
        .0
        .borrow_mut()
        .incoming
        .push_back(Incoming::BeaconLost);
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    let line = progress(&mut wifi);
    assert!(!line.contains("beacon lost"), "{line}");
    let Err(LinkError::Adapter(failure)) = wifi.receive() else {
        panic!("no failure")
    };
    assert!(!failure.contains("the AP ended"), "{failure}");
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

#[test]
fn no_frame_leaves_encrypted_before_the_key_is_acked() {
    // Review Focus 3: the firmware never acknowledges the PTK. The key is
    // never completed, nothing goes under it, and the keeper fails the
    // handshake and joins again.
    let (mut wifi, model) = wifi_with(|air| air.refuse_keys = true);
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    shake(&mut wifi, &model, &RSN, &[]);
    assert!(run(&mut wifi, &model, |_, air| {
        air.count(|c| matches!(c, Call::SetBssid(_))) == 2
    }));
    let Err(LinkError::Adapter(failure)) = wifi.receive() else {
        panic!("no failure")
    };
    assert!(failure.contains("never acknowledged"), "{failure}");
    let air = model.0.borrow();
    assert!(air.keys().is_empty());
    assert!(air.data().iter().all(|(_, key)| key.is_none()));
    assert!(air.count(|c| *c == Call::Disassociate) >= 1);
    drop(air);
    assert!(!wifi.carrier());
    assert!(progress(&mut wifi).contains("keys 0, port closed"));
}

#[test]
fn data_goes_with_its_pn_and_without_iv() {
    let (mut wifi, model) = connected();
    let before = model.0.borrow().data().len();
    wifi.transmit(&ip(1)).unwrap();
    wifi.transmit(&ip(2)).unwrap();
    // The second waits for TID 0's agreement (M4 spec S10.1).
    assert!(run(&mut wifi, &model, |_, air| air.data().len() == before + 2));
    let air = model.0.borrow();
    let sent = &air.data()[before..];
    // The PTK's CAM entry and key ID 0, under PNs 1 and 2: its
    // `TransmitSeed::host(0)`, as `atomic64_inc_return` counts from 0.
    let keys: Vec<FrameKey> = sent.iter().map(|(_, key)| *key).collect();
    assert_eq!(keys, [Some((0, 0, 1)), Some((0, 0, 2))]);
    for (bytes, _) in sent {
        // Protected QoS data whose 26-byte header is followed by the LLC:
        // the CCMP header is the hardware's to insert (spec S5.4).
        assert_eq!(bytes[..2], [0x88, 0x41]);
        assert_eq!(bytes[26..34], [0xaa, 0xaa, 3, 0, 0, 0, 8, 0]);
        assert_eq!(bytes.len(), 26 + 8 + 46);
    }
}

#[test]
fn pns_strictly_increase_across_a_full_ring() {
    // Review Focus 4: a frame the chip has no room for waits in `held` with
    // the PN reserved for it; one dropped from `held` still spent its PN.
    let (mut wifi, model) = connected();
    let before = model.0.borrow().data().len();
    let fill = |bytes: &[u8]| bytes[36];
    wifi.transmit(&ip(1)).unwrap();
    model.0.borrow_mut().full_data = true;
    wifi.transmit(&ip(2)).unwrap();
    wifi.transmit(&ip(3)).unwrap();
    assert_eq!(model.0.borrow().data().len(), before + 1);
    model.0.borrow_mut().full_data = false;
    assert!(run(&mut wifi, &model, |_, air| air.data().len() == before + 3));
    let sent: Vec<(u8, u64)> = model.0.borrow().data()[before..]
        .iter()
        .map(|(bytes, key)| (fill(bytes), key.unwrap().2))
        .collect();
    assert_eq!(sent, [(1, 1), (2, 2), (3, 3)]);
    // 257 frames wait: the oldest (PN 4) is dropped, the rest leave in
    // order, and the next frame takes PN 261, never a PN given before.
    model.0.borrow_mut().full_data = true;
    for n in 0..257u16 {
        wifi.transmit(&ip(n as u8)).unwrap();
    }
    model.0.borrow_mut().full_data = false;
    assert!(run(&mut wifi, &model, |_, air| air.data().len() == before + 259));
    wifi.transmit(&ip(9)).unwrap();
    let pns: Vec<u64> = model.0.borrow().data()[before + 3..]
        .iter()
        .map(|(_, key)| key.unwrap().2)
        .collect();
    assert_eq!(pns, (5..=261).collect::<Vec<u64>>());
}

#[test]
fn received_data_passes_replay_and_reaches_the_link() {
    let (mut wifi, model) = connected();
    let hear = |frame: Vec<u8>| model.0.borrow_mut().hear_as(frame, true, false);
    hear(protected(LOCAL, 10, 0, 1, &[0x45; 46]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x45]);
    // Replayed, then the next packet number.
    hear(protected(LOCAL, 10, 0, 1, &[0x45; 46]));
    hear(protected(LOCAL, 11, 0, 2, &[0x46; 46]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x46]);
    // A group frame under the GTK of key ID 1, past its RSC of 7.
    hear(protected([0xff; 6], 12, 1, 8, &[0x47; 46]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x47]);
    // Unprotected data is refused once the port is controlled.
    model
        .0
        .borrow_mut()
        .hear(data(LOCAL, 13, [8, 0], &[0x48; 46]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), []);
}

#[test]
fn an_undecrypted_protected_frame_is_dropped() {
    let (mut wifi, model) = connected();
    let body = [0x45; 46];
    let mut air = model.0.borrow_mut();
    // Not decrypted by the hardware, failing its ICV, and under a key ID
    // no key has (spec S13.3).
    air.hear_as(protected(LOCAL, 10, 0, 1, &body), false, false);
    air.hear_as(protected(LOCAL, 11, 0, 2, &body), true, true);
    air.hear_as(protected(LOCAL, 12, 2, 3, &body), true, false);
    drop(air);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), []);
    // None of them spent a packet number.
    model
        .0
        .borrow_mut()
        .hear_as(protected(LOCAL, 13, 0, 1, &body), true, false);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x45]);
}

#[test]
fn a_frame_failing_its_icv_or_fcs_leaves_no_sequence_for_its_retry() {
    // mac80211's `should_drop_frame` drops RX_FLAG_FAILED_FCS_CRC, which
    // rtw89 sets for a CRC32 or ICV error (core.c:4396-4397), before any
    // handler: `ieee80211_rx_h_check_dup` never records its sequence
    // control, so the AP's good retry of it is no duplicate.
    let (mut wifi, model) = connected();
    let retried = |sequence: u16, pn: u64, fill: u8| {
        let mut frame = protected(LOCAL, sequence, 0, pn, &[fill; 46]);
        frame[1] |= 0x08;
        frame
    };
    let mut air = model.0.borrow_mut();
    air.hear_as(protected(LOCAL, 10, 0, 1, &[0x45; 46]), true, true);
    air.hear_as(retried(10, 1, 0x45), true, false);
    drop(air);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x45]);
    // A frame failing its FCS, as the chip hands one on while it scans.
    let mut air = model.0.borrow_mut();
    let channel = air.channel.unwrap();
    air.incoming.push_back(Incoming::Frame(Received {
        bytes: protected(LOCAL, 11, 0, 2, &[0x46; 46]),
        channel,
        signal_dbm: None,
        tsf: 0,
        decrypted: true,
        icv_error: false,
        fcs_error: true,
        rate: None,
        ampdu: false,
    }));
    air.hear_as(retried(11, 2, 0x46), true, false);
    drop(air);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x46]);
}

#[test]
fn the_progress_line_counts_data_both_ways() {
    // M3 Task 20: with keys 2 and the port open but no lease, the line
    // tells data that never came from data the hardware left sealed, data
    // under a key ID with no key and data the station's checks refused, and
    // shows whether the station's own data left. EAPOL counts apart.
    let (mut wifi, model) = wifi();
    assert!(!progress(&mut wifi).contains("rx data"));
    drop((wifi, model));
    let (mut wifi, model) = connected();
    assert!(
        progress(&mut wifi).contains(&alloc::format!(
            ", keys 2, port open, {LINK_CH1}, rx data 0 sealed 0 nokey 0 refused 0, \
                 tx data 0; chip"
        )),
        "{}",
        progress(&mut wifi)
    );
    let body = [0x45; 46];
    let keyless = |mut frame: Vec<u8>| {
        frame[29] = 0x20 | 2 << 6;
        frame
    };
    let mut air = model.0.borrow_mut();
    // Delivered; left sealed; under key ID 2, which has no key.
    air.hear_as(protected(LOCAL, 10, 0, 1, &body), true, false);
    air.hear_as(protected(LOCAL, 11, 0, 2, &body), false, false);
    air.hear_as(keyless(protected(LOCAL, 12, 0, 3, &body)), true, false);
    // Unprotected once the port is controlled, then a replay of PN 1: the
    // station refuses both.
    air.hear(data(LOCAL, 13, [8, 0], &body));
    air.hear_as(protected(LOCAL, 14, 0, 1, &body), true, false);
    // A fragment left sealed, and one under key ID 2; one more left sealed.
    air.hear_as(fragment(15, 0, true, 4, &body), false, false);
    air.hear_as(keyless(fragment(16, 0, true, 5, &body)), true, false);
    air.hear_as(protected(LOCAL, 17, 0, 6, &body), false, false);
    drop(air);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x45]);
    wifi.transmit(&ip(1)).unwrap();
    wifi.transmit(&ip(2)).unwrap();
    // The second waits for TID 0's agreement (M4 spec S10.1), which the AP
    // accepts.
    assert!(run(&mut wifi, &model, |wifi, _| progress(wifi)
        .contains("tx data 2;")));
    let link = LINK_CH1.replace("addba 0/0, agg -", "addba 1/1, agg 0");
    let line = progress(&mut wifi);
    assert!(
        line.contains(&alloc::format!(
            ", port open, {link}, rx data 1 sealed 3 nokey 2 refused 2, tx data 2; "
        )),
        "{line}"
    );
}

#[test]
fn a_robust_management_frame_is_sealed_on_the_host() {
    // Spec S1.2: sealed under the PTK with `realtek::ccmp`, handed to the
    // chip as any management frame (no hardware key).
    let (mut wifi, model) = connected_with_pmf();
    let ptk = model.0.borrow().ptk();
    wifi.radio_mut().deauthenticate(3).unwrap();
    let air = model.0.borrow();
    let deauth = air.management(0xc0);
    let [frame] = &deauth[..] else {
        panic!("{} deauthentications", deauth.len())
    };
    // Protected, under the PTK's next PN (nothing went under it before),
    // its reason sealed and its MIC appended.
    assert_eq!(frame[1] & 0x40, 0x40);
    assert_eq!(frame[24..32], [1, 0, 0, 0x20, 0, 0, 0, 0]);
    assert_eq!(frame.len(), 24 + 8 + 2 + 8);
    let opened = ccmp::open(&ptk, frame, PacketNumber::new(1).unwrap()).unwrap();
    assert_eq!(opened[32..34], [3, 0]);
}

#[test]
fn under_pmf_an_unprotected_departure_is_ignored() {
    // `ieee80211_drop_unencrypted_mgmt`: neither a unicast nor a group
    // deauthentication without its protection ends a PMF association.
    let (mut wifi, model) = connected_with_pmf();
    let mut group = management(0xc0, &[7, 0]);
    group[4..10].fill(0xff);
    model.0.borrow_mut().hear(management(0xc0, &[7, 0]));
    model.0.borrow_mut().hear(group);
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    assert_eq!(model.0.borrow().count(|c| *c == Call::Disassociate), 0);
    assert!(wifi.carrier());
    // No SA Query request follows: starting one is wpa_supplicant's
    // (`sme_event_unprot_disconnect`), which the shared keeper does not
    // port (the module documentation's gap).
    assert!(model.0.borrow().management(0xd0).is_empty());
}

/// The AP's deauthentication to every station, reason 7, under BIP with
/// the IGTK of [`IGTK_KDE`] (key ID 4, the 0x77s) at IPN `ipn`, as
/// `ieee80211_crypto_aes_cmac_encrypt` signs it: the MMIE last, its MIC the
/// first 8 bytes of the CMAC over the AAD (the frame control without Retry,
/// PwrMgt and MoreData, then the three addresses) and the body, the MIC
/// zeroed.
fn signed_group_deauthentication(ipn: u64) -> Vec<u8> {
    let mut frame = management(0xc0, &[7, 0]);
    frame[4..10].fill(0xff);
    frame.extend([76, 16, 4, 0]);
    frame.extend(&ipn.to_le_bytes()[..6]);
    frame.extend([0; 8]);
    let control = u16::from_le_bytes([frame[0], frame[1]]) & !0x3800;
    let mut message = control.to_le_bytes().to_vec();
    message.extend(&frame[4..22]);
    message.extend(&frame[24..]);
    let mic =
        crate::wifi::mediatek::mailbox::key::bip::tests::rotation::cmac(&[0x77; 16], &message);
    let end = frame.len();
    frame[end - 8..].copy_from_slice(&mic[..8]);
    frame
}

#[test]
fn under_pmf_a_group_deauthentication_needs_its_mic_and_a_fresh_ipn() {
    // The host checks a group robust frame's BIP (spec S1.2,
    // `ieee80211_crypto_aes_cmac_decrypt`) under the IGTK message 3 gave,
    // key ID 4 from IPN 9: one whose MIC fails, or whose IPN is not past
    // the last, is dropped; a valid one ends the association as
    // `ieee80211_rx_mgmt_deauth` does, without a frame of the station's own.
    let (mut wifi, model) = connected_with_pmf();
    let mut flipped = signed_group_deauthentication(10);
    let end = flipped.len();
    flipped[end - 1] ^= 1;
    model.0.borrow_mut().hear(flipped);
    model.0.borrow_mut().hear(signed_group_deauthentication(9));
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    assert_eq!(model.0.borrow().count(|c| *c == Call::Disassociate), 0);
    assert!(wifi.carrier());
    let from = model.0.borrow().calls.len();
    model.0.borrow_mut().hear(signed_group_deauthentication(10));
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    assert_eq!(
        leaving(&model.0.borrow(), from),
        [
            "flush true",
            "flush false",
            "remove 0",
            "flush false",
            "remove 1",
            "leave"
        ]
    );
    assert!(!wifi.carrier());
    let Err(LinkError::Adapter(failure)) = wifi.receive() else {
        panic!("no failure")
    };
    assert!(
        failure.contains("the AP ended the association"),
        "{failure}"
    );
}

#[test]
fn a_tkip_or_ccmp256_key_is_unsupported() {
    let (mut wifi, model) = associated();
    for (pairwise, index, length) in [(true, 0, 32), (false, 1, 32), (false, 1, 13)] {
        let got = wifi
            .radio_mut()
            .install_key(pairwise, index, &vec![0; length], ReplaySeed::ZERO);
        assert!(
            matches!(
                got,
                Err(Error::Unsupported(
                    "a key other than CCMP-128 on an RTL8852C"
                ))
            ),
            "{got:?}"
        );
    }
    assert!(model.0.borrow().keys().is_empty());
    let line = progress(&mut wifi);
    assert!(
        line.contains("last \"a key other than CCMP-128 on an RTL8852C\""),
        "{line}"
    );
}

#[test]
fn an_authentication_reported_late_still_counts_its_ack() {
    // The M3a boot read 'auth 1/0' though the chip saw every frame
    // acknowledged: the authentication's report came after the association
    // request had replaced it as the frame the station awaits.
    let (mut wifi, model) = wifi_with(|air| air.late_auth_report = true);
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    let line = progress(&mut wifi);
    assert!(
        line.contains("auth 1/1 status 0, assoc 1/1 status 0"),
        "{line}"
    );
}

/// The flushes, management frames, key calls and the leaving among the
/// chip's calls from `from` to the first disassociation.
fn leaving(air: &Air, from: usize) -> Vec<String> {
    let end = from
        + air.calls[from..]
            .iter()
            .position(|c| *c == Call::Disassociate)
            .unwrap();
    air.calls[from..=end]
        .iter()
        .filter_map(|c| match c {
            Call::Flush(drop) => Some(alloc::format!("flush {drop}")),
            Call::Management(bytes, _) => Some(alloc::format!("mgmt {:02x}", bytes[0])),
            Call::RemoveKey(cam) => Some(alloc::format!("remove {cam}")),
            Call::Disassociate => Some(String::from("leave")),
            Call::AddKey { .. } => Some(String::from("add")),
            Call::StopBa(tid) => Some(alloc::format!("stop {tid}")),
            Call::Aggregation { tid, window: None } => Some(alloc::format!("agg stop {tid}")),
            _ => None,
        })
        .collect()
}

#[test]
fn a_deauthentication_flushes_and_deletes_the_keys_before_leaving() {
    // ieee80211_set_disassoc: a flush with drop, the deauthentication, a
    // flush without; then sta_info_flush deletes the PTK (DISABLE_KEY:
    // rtw89_ops_set_key's flushes, then the CAM delete), the GTK after it
    // (spec S14.5), and the AP's station leaves.
    let (mut wifi, model) = connected();
    let from = model.0.borrow().calls.len();
    wifi.radio_mut().deauthenticate(3).unwrap();
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    assert_eq!(
        leaving(&model.0.borrow(), from),
        [
            "flush true",
            "mgmt c0",
            "flush false",
            "flush false",
            "remove 0",
            "flush false",
            "remove 1",
            "leave"
        ]
    );
    assert!(!wifi.carrier());
}

#[test]
fn a_deauthentication_from_the_ap_drops_what_is_queued_and_deletes_the_keys() {
    // ieee80211_set_disassoc without a frame of its own: the flush with
    // drop alone, then the keys and the leaving.
    let (mut wifi, model) = connected();
    let from = model.0.borrow().calls.len();
    model.0.borrow_mut().hear(management(0xc0, &[7, 0]));
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    assert_eq!(
        leaving(&model.0.borrow(), from),
        [
            "flush true",
            "flush false",
            "remove 0",
            "flush false",
            "remove 1",
            "leave"
        ]
    );
}

#[test]
fn a_group_rekey_replaces_the_key_of_its_index() {
    // ieee80211_key_replace: the old key of the same index goes first
    // (DISABLE_KEY), then the new one; a key of another index is added.
    // The port stays open on the PTK and the GTK last installed.
    let (mut wifi, model) = connected();
    let hear = |frame: Vec<u8>| model.0.borrow_mut().hear_as(frame, true, false);
    let from = model.0.borrow().calls.len();
    wifi.radio_mut()
        .install_key(false, 1, &[0x66; 16], ReplaySeed::ZERO)
        .unwrap();
    {
        let air = model.0.borrow();
        let calls: Vec<&Call> = air.calls[from..].iter().collect();
        assert_eq!(calls.len(), 3, "{calls:?}");
        assert_eq!(*calls[0], Call::Flush(false));
        assert_eq!(*calls[1], Call::RemoveKey(1));
        assert!(matches!(
            calls[2],
            Call::AddKey {
                pairwise: false,
                index: 1,
                cam: 1,
                ..
            }
        ));
    }
    hear(protected([0xff; 6], 20, 1, 1, &[0x51; 46]));
    hear(protected(LOCAL, 21, 0, 1, &[0x52; 46]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x51, 0x52]);
    let from = model.0.borrow().calls.len();
    wifi.radio_mut()
        .install_key(false, 2, &[0x67; 16], ReplaySeed::ZERO)
        .unwrap();
    {
        let air = model.0.borrow();
        let calls: Vec<&Call> = air.calls[from..].iter().collect();
        assert_eq!(calls.len(), 1, "{calls:?}");
        assert!(matches!(
            calls[0],
            Call::AddKey {
                pairwise: false,
                index: 2,
                cam: 2,
                ..
            }
        ));
    }
    hear(protected([0xff; 6], 22, 2, 1, &[0x53; 46]));
    hear(protected(LOCAL, 23, 0, 2, &[0x54; 46]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x53, 0x54]);
    assert!(wifi.carrier());
    // The same key again is no new install (KRACK, ieee80211_key_identical).
    let from = model.0.borrow().calls.len();
    wifi.radio_mut()
        .install_key(false, 2, &[0x67; 16], ReplaySeed::ZERO)
        .unwrap();
    assert_eq!(model.0.borrow().calls.len(), from);
    assert!(progress(&mut wifi).contains("keys 4"));
}

#[test]
fn a_rekey_the_firmware_refuses_closes_the_port_until_its_key_is_installed() {
    // ieee80211_key_replace deletes the old key first: when the chip then
    // refuses the new one, no key of that index is left, so the port
    // closes, and opens again only on an installed key.
    let (mut wifi, model) = connected();
    model.0.borrow_mut().refuse_keys = true;
    let got = wifi
        .radio_mut()
        .install_key(false, 1, &[0x66; 16], ReplaySeed::ZERO);
    assert!(matches!(got, Err(Error::Chip(_))), "{got:?}");
    assert!(!wifi.radio_mut().carrier());
    assert_eq!(wifi.radio_mut().transmit(&ip(1)), Err(LinkError::NoCarrier));
    model.0.borrow_mut().refuse_keys = false;
    wifi.radio_mut()
        .install_key(false, 1, &[0x66; 16], ReplaySeed::ZERO)
        .unwrap();
    assert!(wifi.radio_mut().carrier());
    // A key of a new index deletes nothing: refused, the port stays open.
    model.0.borrow_mut().refuse_keys = true;
    let got = wifi
        .radio_mut()
        .install_key(false, 2, &[0x67; 16], ReplaySeed::ZERO);
    assert!(got.is_err(), "{got:?}");
    assert!(wifi.radio_mut().carrier());
    // A pairwise rekey alike: the old PTK gone, nothing leaves under it.
    let got = wifi
        .radio_mut()
        .install_key(true, 0, &[0x99; 16], ReplaySeed::ZERO);
    assert!(got.is_err(), "{got:?}");
    assert!(!wifi.radio_mut().carrier());
    model.0.borrow_mut().refuse_keys = false;
    wifi.radio_mut()
        .install_key(true, 0, &[0x99; 16], ReplaySeed::ZERO)
        .unwrap();
    assert!(wifi.radio_mut().carrier());
    let before = model.0.borrow().data().len();
    wifi.radio_mut().transmit(&ip(2)).unwrap();
    assert_eq!(model.0.borrow().data()[before].1, Some((0, 0, 1)));
}

#[test]
fn under_pmf_a_deauthentication_without_its_ptk_never_leaves_in_the_clear() {
    // A pairwise rekey the chip refuses has deleted the old PTK first, while
    // the station layer still holds the port authorized under PMF. The
    // deauthentication that follows is robust and protected, with no PTK
    // left to seal it: it is dropped and reported to the keeper once, never
    // sent with the Protected bit over a plain body. Only "never in the
    // clear" is the property; the drop itself is a known gap (mac80211
    // would send it protected under a software key).
    let (mut wifi, model) = connected_with_pmf();
    model.0.borrow_mut().refuse_keys = true;
    let got = wifi
        .radio_mut()
        .install_key(true, 0, &[0x99; 16], ReplaySeed::ZERO);
    assert!(got.is_err(), "{got:?}");
    assert!(wifi.radio().station().protected_port_authorized());
    let from = model.0.borrow().calls.len();
    wifi.radio_mut().deauthenticate(3).unwrap();
    let air = model.0.borrow();
    let sent: Vec<&Vec<u8>> = air.calls[from..]
        .iter()
        .filter_map(|c| match c {
            Call::Management(bytes, _) => Some(bytes),
            _ => None,
        })
        .collect();
    assert!(sent.is_empty(), "{sent:02x?}");
    drop(air);
    let unsealed = |e: &keeper::Event| {
        matches!(
            e,
            keeper::Event::Unsupported("a protected management frame without its key")
        )
    };
    assert_eq!(
        wifi.radio().events.iter().filter(|e| unsealed(e)).count(),
        1
    );
}

#[test]
fn the_keepers_group_key_handshake_installs_the_new_gtk_under_the_ptk() {
    // After the 4-way handshake the AP rotates its group key: group message
    // 1, EAPOL under the PTK, is answered with message 2 under the PTK, and
    // the GTK of index 2 goes in beside index 1, receive-only, the port
    // open throughout.
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
    assert!(wifi.carrier());
    let from = model.0.borrow().calls.len();
    let group = protected_as(LOCAL, 2, 0, 1, [0x88, 0x8e], &ap.group_message_1());
    model.0.borrow_mut().hear_as(group, true, false);
    assert!(run(&mut wifi, &model, |_, air| air.eapol().len() > 2));
    {
        let air = model.0.borrow();
        // MIC | SECURE, at the request's replay counter, under the KCK.
        ap.signed_reply(&air.eapol()[2], 0x300);
        let answer = air.calls[from..]
            .iter()
            .find_map(|c| match c {
                Call::Data {
                    bytes,
                    eapol: true,
                    key,
                    ..
                } if eapol(bytes).is_some() => Some(*key),
                _ => None,
            })
            .unwrap();
        // Under the PTK's CAM entry 0, key ID 0.
        assert!(matches!(answer, Some((0, 0, _))), "{answer:?}");
        assert!(air.calls[from..].iter().any(|c| matches!(
            c,
            Call::AddKey {
                pairwise: false,
                index: 2,
                ..
            }
        )));
        assert!(
            !air.calls[from..]
                .iter()
                .any(|c| matches!(c, Call::RemoveKey(_)))
        );
    }
    assert!(wifi.carrier());
    // The authenticator hands every key RSC 7, so each GTK takes PN 8 on.
    let hear = |frame: Vec<u8>| model.0.borrow_mut().hear_as(frame, true, false);
    hear(protected([0xff; 6], 3, 2, 8, &[0x5a; 46]));
    hear(protected([0xff; 6], 4, 1, 8, &[0x5b; 46]));
    hear(protected([0xff; 6], 5, 2, 7, &[0x5c; 46]));
    wifi.carrier();
    // The new GTK's frame is delivered and its replay refused. So is the
    // old GTK's: the shared station layer authorizes the port on the GTK
    // last installed alone, where mac80211 takes either index until the
    // AP's switch. That is a known gap in the shared station layer, outside
    // this radio, pinned here so a fix there shows up as this test failing.
    assert_eq!(delivered(&mut wifi), [0x5a]);
    let line = progress(&mut wifi);
    assert!(
        line.contains(&alloc::format!(
            ", keys 3, port open, {LINK_CH1}, rx data 1 sealed 0 nokey 0 refused 2, "
        )),
        "{line}"
    );
}

/// Fragment `number` of frame `sequence` under the PTK at packet number
/// `pn`, with `more` to follow.
fn fragment(sequence: u16, number: u8, more: bool, pn: u64, payload: &[u8]) -> Vec<u8> {
    let mut frame = protected(LOCAL, sequence, 0, pn, &[]);
    frame.truncate(34);
    frame.extend_from_slice(payload);
    frame.extend([0; 8]);
    frame[1] |= u8::from(more) << 2;
    frame[22] |= number;
    frame
}

#[test]
fn fragments_under_consecutive_numbers_make_one_frame() {
    let (mut wifi, model) = connected();
    let first = [&[0xaa, 0xaa, 3, 0, 0, 0, 8, 0][..], &[0x45; 4]].concat();
    let hear = |frame: Vec<u8>| model.0.borrow_mut().hear_as(frame, true, false);
    hear(fragment(30, 0, true, 1, &first));
    hear(fragment(30, 1, false, 2, &[0x46; 4]));
    wifi.carrier();
    let ethernet = wifi.receive().unwrap().unwrap();
    assert_eq!(
        ethernet[14..],
        [0x45, 0x45, 0x45, 0x45, 0x46, 0x46, 0x46, 0x46]
    );
    // Numbers that skip one are dropped.
    hear(fragment(31, 0, true, 5, &first));
    hear(fragment(31, 1, false, 7, &[0x46; 4]));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), []);
}

#[test]
fn an_a_msdu_gives_up_each_of_its_frames() {
    let (mut wifi, model) = connected();
    let mut frame = protected(LOCAL, 20, 0, 1, &[]);
    frame.truncate(34);
    frame[24] = 0x80;
    for value in [0x31, 0x32] {
        let padding = (4 - frame[34..].len() % 4) % 4;
        frame.extend(vec![0; padding]);
        frame.extend(LOCAL);
        frame.extend(AP);
        frame.extend(54u16.to_be_bytes());
        frame.extend([0xaa, 0xaa, 3, 0, 0, 0, 8, 0]);
        frame.extend([value; 46]);
    }
    frame.extend([0; 8]);
    model.0.borrow_mut().hear_as(frame, true, false);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x31, 0x32]);
    let line = progress(&mut wifi);
    assert!(
        line.contains(", rx data 2 sealed 0 nokey 0 refused 0, "),
        "{line}"
    );
}

/// An SA Query request at packet number `number` as the hardware leaves
/// it decrypted, its MIC in place.
fn sa_query(number: u8) -> Vec<u8> {
    let mut frame = management(0xd0, &[number, 0, 0, 0x20, 0, 0, 0, 0, 8, 0, 1, 2]);
    frame[1] = 0x40;
    frame.extend([0; 8]);
    frame
}

#[test]
fn a_protected_management_frame_is_taken_once_under_its_packet_number() {
    let (mut wifi, model) = connected();
    let answers = |air: &Air| air.management(0xd0);
    model.0.borrow_mut().hear_as(sa_query(5), true, false);
    assert!(run(&mut wifi, &model, |_, air| answers(air).len() == 1));
    model.0.borrow_mut().hear_as(sa_query(5), true, false);
    assert!(!run(&mut wifi, &model, |_, air| answers(air).len() > 1));
    model.0.borrow_mut().hear_as(sa_query(6), true, false);
    assert!(run(&mut wifi, &model, |_, air| answers(air).len() == 2));
    // Each answer is sealed under the PTK and answers its request's ID.
    let air = model.0.borrow();
    let ptk = air.ptk();
    for (n, answer) in answers(&air).iter().enumerate() {
        let pn = PacketNumber::new(n as u64 + 1).unwrap();
        let opened = ccmp::open(&ptk, answer, pn).unwrap();
        assert_eq!(opened[32..36], [8, 1, 1, 2]);
    }
}

#[test]
fn a_pairwise_rekey_drops_what_waits_under_the_old_key() {
    // The new PTK takes the CAM entry the old one left: a frame held under
    // the old key and its PN must not leave under the new one.
    let (mut wifi, model) = connected();
    let before = model.0.borrow().data().len();
    model.0.borrow_mut().full_data = true;
    wifi.transmit(&ip(1)).unwrap();
    wifi.radio_mut()
        .install_key(true, 0, &[0x99; 16], ReplaySeed::ZERO)
        .unwrap();
    assert_eq!(model.0.borrow().keys().last(), Some(&(true, 0, 0)));
    model.0.borrow_mut().full_data = false;
    wifi.transmit(&ip(2)).unwrap();
    assert!(run(&mut wifi, &model, |_, air| air.data().len() > before));
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    let sent: Vec<(u8, FrameKey)> = model.0.borrow().data()[before..]
        .iter()
        .map(|(bytes, key)| (bytes[36], *key))
        .collect();
    assert_eq!(sent, [(2, Some((0, 0, 1)))]);
}

// RX Block Ack (M3 Task 16, spec S13.2).

/// The AP's ADDBA Request: dialog `dialog`, immediate Block Ack on `tid`
/// with a window of `window`, no timeout, from `ssn`.
fn addba(dialog: u8, tid: u8, window: u16, ssn: u16) -> Vec<u8> {
    let parameters = 2 | u16::from(tid) << 2 | window << 6;
    let mut body = vec![3, 0, dialog];
    body.extend(parameters.to_le_bytes());
    body.extend(0u16.to_le_bytes());
    body.extend((ssn << 4).to_le_bytes());
    management(0xd0, &body)
}

/// The AP's ADDBA Response to the station's ADDBA Request `request`, in the
/// clear: its dialog, `status`, its parameters at a window of `window` (the
/// request's own when `None`), and its timeout.
fn addba_response(request: &[u8], window: Option<u16>, status: u16) -> Vec<u8> {
    let asked = u16::from_le_bytes([request[27], request[28]]);
    let parameters = window.map_or(asked, |window| asked & 0x3f | window << 6);
    let mut body = vec![3, 1, request[26]];
    body.extend(status.to_le_bytes());
    body.extend(parameters.to_le_bytes());
    body.extend_from_slice(&request[29..31]);
    management(0xd0, &body)
}

/// The AP's DELBA of its agreement on `tid`, as its originator, reason 37
/// (any reason does).
fn delba(tid: u8) -> Vec<u8> {
    let parameters = u16::from(tid) << 12 | 1 << 11;
    let mut body = vec![3, 2];
    body.extend(parameters.to_le_bytes());
    body.extend(37u16.to_le_bytes());
    management(0xd0, &body)
}

/// The AP's compressed single-TID Block Ack Request for `tid` from `ssn`.
fn bar(tid: u8, ssn: u16) -> Vec<u8> {
    let mut frame = vec![0x84, 0, 0, 0];
    frame.extend(LOCAL);
    frame.extend(AP);
    frame.extend((u16::from(tid) << 12 | 4).to_le_bytes());
    frame.extend((ssn << 4).to_le_bytes());
    frame
}

/// QoS data of `tid` under the PTK at packet number `pn`, sequence number
/// `sequence`, its first payload byte past the IPv4 type `fill`.
fn qos(tid: u8, sequence: u16, pn: u64, fill: u8) -> Vec<u8> {
    let mut frame = protected(LOCAL, sequence, 0, pn, &[fill; 46]);
    frame[24] = tid;
    frame
}

/// `frame`, a management frame, protected at packet number `pn` as the
/// hardware leaves it decrypted: its CCMP header in place, its MIC zeros.
fn sealed(mut frame: Vec<u8>, pn: u8) -> Vec<u8> {
    frame[1] |= 0x40;
    frame.splice(24..24, [pn, 0, 0, 0x20, 0, 0, 0, 0]);
    frame.extend([0; 8]);
    frame
}

/// The station's ADDBA Responses (category 3, action 1), unprotected.
fn responses(air: &Air) -> Vec<Vec<u8>> {
    air.management(0xd0)
        .into_iter()
        .filter(|frame| frame[24..26] == [3, 1])
        .collect()
}

/// The chip's RX Block Ack calls from call `from` on.
fn agreements(air: &Air, from: usize) -> Vec<Call> {
    air.calls[from..]
        .iter()
        .filter(|c| matches!(c, Call::StartBa { .. } | Call::StopBa(_)))
        .cloned()
        .collect()
}

/// [`connected`], then the AP's agreement on TID 0 from SSN 100 accepted.
fn aggregating() -> (Wifi<Model>, Model) {
    let (mut wifi, model) = connected();
    model.0.borrow_mut().hear(addba(1, 0, 64, 100));
    assert!(run(&mut wifi, &model, |_, air| responses(air).len() == 1));
    (wifi, model)
}

#[test]
fn an_addba_is_accepted_programmed_and_reorders() {
    let (mut wifi, model) = connected();
    let from = model.0.borrow().calls.len();
    model.0.borrow_mut().hear(addba(1, 0, 64, 100));
    assert!(run(&mut wifi, &model, |_, air| !responses(air).is_empty()));
    {
        let air = model.0.borrow();
        // rtw89_ops_ampdu_action's RX_START programs the BA CAM (spec
        // S13.2) before mac80211 answers (agg-rx.c's
        // __ieee80211_start_rx_ba_session).
        let start = air
            .position(|c| {
                *c == Call::StartBa {
                    tid: 0,
                    ssn: 100,
                    window: 64,
                }
            })
            .unwrap();
        let answer = air
            .position(|c| matches!(c, Call::Management(b, _) if b[0] == 0xd0))
            .unwrap();
        assert!(from <= start && start < answer);
        let [response] = &responses(&air)[..] else {
            panic!("{} responses", responses(&air).len())
        };
        // To the AP: ADDBA Response, dialog 1, success, A-MSDU | immediate |
        // TID 0 | a window of 64, no timeout.
        assert_eq!(response[4..10], AP);
        assert_eq!(response[24..33], [3, 1, 1, 0, 0, 0x03, 0x10, 0, 0]);
        assert_eq!(agreements(&air, from).len(), 1);
    }
    // Out of order on TID 0, delivered in order: each passes the replay
    // check, which an earlier packet number delivered last would fail.
    let hear = |frame: Vec<u8>| model.0.borrow_mut().hear_as(frame, true, false);
    hear(qos(0, 101, 11, 0x51));
    hear(qos(0, 100, 10, 0x50));
    hear(qos(0, 102, 12, 0x52));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x50, 0x51, 0x52]);
    // A hole holds what follows it until the reorder timeout releases it.
    hear(qos(0, 104, 14, 0x54));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), []);
    let mut got = Vec::new();
    let mut waited = 0;
    while got.is_empty() && waited < 200_000 {
        crate::test_clock::advance(CLOCK, 5_000);
        waited += 5_000;
        wifi.carrier();
        got.extend(delivered(&mut wifi));
    }
    assert_eq!(got, [0x54]);
    // TID 0's 100 ms, as HT_RX_REORDER_BUF_TIMEOUT's, of which the virtual
    // clock's own reads (100 us each) took some.
    assert!((80_000..=110_000).contains(&waited), "{waited}");
    // Another TID is not reordered.
    hear(qos(6, 7, 1, 0x61));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x61]);
}

#[test]
fn a_delba_stops_the_entry() {
    let (mut wifi, model) = aggregating();
    let hear = |frame: Vec<u8>| model.0.borrow_mut().hear_as(frame, true, false);
    // SSN 102 waits for 100 and 101.
    hear(qos(0, 102, 12, 0x52));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), []);
    let from = model.0.borrow().calls.len();
    model.0.borrow_mut().hear(delba(0));
    wifi.carrier();
    // RX_STOP frees the BA CAM entry; the AP's own DELBA is not answered.
    assert_eq!(agreements(&model.0.borrow(), from), [Call::StopBa(0)]);
    let ours = model.0.borrow().calls[from..]
        .iter()
        .filter(|c| matches!(c, Call::Management(b, _) if b[0] == 0xd0))
        .count();
    assert_eq!(ours, 0);
    // What the agreement held went with it (ieee80211_free_tid_rx), past
    // any reorder timeout.
    for _ in 0..30 {
        run(&mut wifi, &model, |_, _| true);
    }
    assert_eq!(delivered(&mut wifi), []);
    // TID 0 now goes as it comes: the earlier packet number arriving second
    // fails the replay check.
    hear(qos(0, 111, 21, 0x61));
    hear(qos(0, 110, 20, 0x60));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x61]);
    // A new agreement on the TID is programmed again.
    let from = model.0.borrow().calls.len();
    model.0.borrow_mut().hear(addba(2, 0, 32, 200));
    assert!(run(&mut wifi, &model, |_, air| responses(air).len() == 2));
    assert_eq!(
        agreements(&model.0.borrow(), from),
        [Call::StartBa {
            tid: 0,
            ssn: 200,
            window: 32
        }]
    );
}

#[test]
fn a_bar_releases_what_waits_before_its_ssn() {
    let (mut wifi, model) = aggregating();
    model
        .0
        .borrow_mut()
        .hear_as(qos(0, 102, 12, 0x52), true, false);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), []);
    // ieee80211_rx_h_ctrl: the BAR moves the window to its SSN at once.
    model.0.borrow_mut().hear(bar(0, 102));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x52]);
}

#[test]
fn the_departure_stops_each_agreement_before_the_keys() {
    // __sta_info_destroy_part1's ieee80211_sta_tear_down_BA_sessions
    // (AGG_STOP_DESTROY_STA: RX_STOP for each, no DELBA) before
    // _part2's key deletes and the AP's station leaving.
    let (mut wifi, model) = aggregating();
    model.0.borrow_mut().hear(addba(2, 6, 64, 0));
    assert!(run(&mut wifi, &model, |_, air| responses(air).len() == 2));
    let from = model.0.borrow().calls.len();
    wifi.radio_mut().deauthenticate(3).unwrap();
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    assert_eq!(
        leaving(&model.0.borrow(), from),
        [
            "flush true",
            "mgmt c0",
            "flush false",
            "stop 0",
            "stop 6",
            "flush false",
            "remove 0",
            "flush false",
            "remove 1",
            "leave"
        ]
    );
    // The next association starts with none.
    let from = model.0.borrow().calls.len();
    assert!(run(&mut wifi, &model, |wifi, air| {
        air.count(|c| matches!(c, Call::Associate(_))) == 2
            && wifi.radio().station().phase() == Peering::Associated
    }));
    assert!(agreements(&model.0.borrow(), from).is_empty());
}

#[test]
fn an_addba_with_the_association_response_is_answered_once_associated() {
    // The AP asks in the poll that brings its association response.
    // mac80211 takes the response, BSS_CHANGED_ASSOC and all, before the
    // next frame, so the request finds the station associated.
    let (mut wifi, model) = wifi_with(|air| air.addba_with_assoc = true);
    assert!(run(&mut wifi, &model, |_, air| !responses(air).is_empty()));
    let air = model.0.borrow();
    let associate = air.position(|c| matches!(c, Call::Associate(_))).unwrap();
    let start = air
        .position(|c| matches!(c, Call::StartBa { tid: 0, .. }))
        .unwrap();
    assert!(associate < start);
    assert_eq!(responses(&air)[0][27..29], [0, 0]);
}

#[test]
fn a_retried_addba_is_dropped_as_a_duplicate() {
    // ieee80211_rx_h_check_dup: the AP retries the same frame when our ACK
    // was lost; answering it again would decline the agreement just made.
    let (mut wifi, model) = connected();
    let mut request = addba(1, 0, 64, 100);
    request[22..24].copy_from_slice(&(7u16 << 4).to_le_bytes());
    model.0.borrow_mut().hear(request.clone());
    assert!(run(&mut wifi, &model, |_, air| responses(air).len() == 1));
    request[1] |= 0x08;
    model.0.borrow_mut().hear(request);
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    assert_eq!(responses(&model.0.borrow()).len(), 1);
    assert_eq!(
        model
            .0
            .borrow()
            .count(|c| matches!(c, Call::StartBa { .. })),
        1
    );
    // A retry of another frame is no duplicate.
    let mut next = addba(2, 6, 64, 0);
    next[1] |= 0x08;
    next[22..24].copy_from_slice(&(8u16 << 4).to_le_bytes());
    model.0.borrow_mut().hear(next);
    assert!(run(&mut wifi, &model, |_, air| responses(air).len() == 2));
    assert_eq!(responses(&model.0.borrow())[1][27..29], [0, 0]);
}

#[test]
fn a_chip_that_cannot_program_the_ba_cam_still_accepts() {
    // rtw89_ops_ampdu_action returns 0 whatever its BA CAM H2C did: the
    // hardware's dynamic BA CAM covers the agreement (fw.c:2592-2600), and
    // the host reorders. The chip's error is reported, not fatal.
    let (mut wifi, model) = connected();
    model.0.borrow_mut().ba_fails = true;
    model.0.borrow_mut().hear(addba(1, 0, 64, 100));
    assert!(run(&mut wifi, &model, |_, air| responses(air).len() == 1));
    assert_eq!(responses(&model.0.borrow())[0][27..29], [0, 0]);
    let Err(LinkError::Adapter(failure)) = wifi.receive() else {
        panic!("no failure")
    };
    assert!(
        failure.contains("the BA CAM H2C could not be sent"),
        "{failure}"
    );
    let hear = |frame: Vec<u8>| model.0.borrow_mut().hear_as(frame, true, false);
    hear(qos(0, 101, 11, 0x51));
    hear(qos(0, 100, 10, 0x50));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x50, 0x51]);
    assert!(wifi.carrier());
}

#[test]
fn under_pmf_the_agreement_comes_and_goes_protected() {
    // Block Ack action frames are robust: an unprotected request is dropped
    // (ieee80211_drop_unencrypted_mgmt), a protected one answered sealed
    // under the PTK.
    let (mut wifi, model) = connected_with_pmf();
    let ptk = model.0.borrow().ptk();
    model.0.borrow_mut().hear(addba(1, 0, 64, 100));
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    assert!(model.0.borrow().management(0xd0).is_empty());
    model
        .0
        .borrow_mut()
        .hear_as(sealed(addba(2, 0, 64, 100), 1), true, false);
    assert!(run(&mut wifi, &model, |_, air| !air
        .management(0xd0)
        .is_empty()));
    let air = model.0.borrow();
    let [response] = &air.management(0xd0)[..] else {
        panic!("{} action frames", air.management(0xd0).len())
    };
    assert_eq!(response[1] & 0x40, 0x40);
    let opened = ccmp::open(&ptk, response, PacketNumber::new(1).unwrap()).unwrap();
    assert_eq!(opened[32..41], [3, 1, 2, 0, 0, 0x03, 0x10, 0, 0]);
    assert!(
        air.position(|c| matches!(c, Call::StartBa { tid: 0, .. }))
            .is_some()
    );
}

#[test]
fn only_the_aps_acknowledged_qos_data_is_reordered() {
    // ieee80211_rx_reorder_ampdu: the agreement is the AP's station's (its
    // TA), and a frame sent with no acknowledgement is no part of it.
    let (mut wifi, model) = aggregating();
    let hear = |frame: Vec<u8>| model.0.borrow_mut().hear_as(frame, true, false);
    // SSN 102 waits for 100 and 101.
    hear(qos(0, 102, 12, 0x52));
    // Far ahead of the window, from another transmitter: it would flush
    // the window if it went in; it goes on to the station, which refuses it.
    let mut foreign = qos(0, 400, 40, 0x70);
    foreign[10..16].copy_from_slice(&[0x02, 0xAA, 0, 0, 0, 9]);
    hear(foreign);
    // Far ahead too, sent without an acknowledgement: on at once.
    let mut unacked = qos(0, 500, 11, 0x71);
    unacked[24] |= 0x20;
    hear(unacked);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x71]);
    // 102 still waited for its hole, and goes at the BAR.
    model.0.borrow_mut().hear(bar(0, 102));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x52]);
}

#[test]
fn a_frame_the_station_cannot_open_still_takes_its_place_in_the_window() {
    // mac80211 reorders before it decrypts (ieee80211_rx_reorder_ampdu,
    // then ieee80211_rx_h_decrypt): an agreement's frame the hardware left
    // encrypted, or one under a key ID with no key, fills its slot and is
    // dropped as it leaves the buffer, so what follows waits for no hole.
    let (mut wifi, model) = aggregating();
    let mut air = model.0.borrow_mut();
    air.hear_as(qos(0, 100, 10, 0x50), false, false);
    let mut keyless = qos(0, 101, 11, 0x51);
    keyless[29] = 0x20 | 2 << 6;
    air.hear_as(keyless, true, false);
    air.hear_as(qos(0, 102, 12, 0x52), true, false);
    drop(air);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x52]);
    // The window moved past them: 104 waits for 103 alone.
    let mut air = model.0.borrow_mut();
    air.hear_as(qos(0, 104, 14, 0x54), true, false);
    air.hear_as(qos(0, 103, 13, 0x53), true, false);
    drop(air);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x53, 0x54]);
    // A frame failing its ICV is no frame to mac80211 (rtw89's
    // RX_FLAG_FAILED_FCS_CRC, core.c:4396-4397): it leaves a hole, which
    // what follows waits behind until the BAR.
    let mut air = model.0.borrow_mut();
    air.hear_as(qos(0, 105, 15, 0x55), true, true);
    air.hear_as(qos(0, 106, 16, 0x56), true, false);
    drop(air);
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), []);
    model.0.borrow_mut().hear(bar(0, 106));
    wifi.carrier();
    assert_eq!(delivered(&mut wifi), [0x56]);
}

/// The (transaction, status) of each SAE frame the station sent.
fn sae_sent(air: &Air) -> Vec<(u16, u16)> {
    air.management(0xb0)
        .iter()
        .map(|frame| {
            assert_eq!(frame[24..26], [3, 0], "not SAE");
            let field = |at: usize| u16::from_le_bytes([frame[at], frame[at + 1]]);
            (field(26), field(28))
        })
        .collect()
}

/// Where the chip took each authentication frame, in order.
fn authentications(air: &Air) -> Vec<usize> {
    (0..air.calls.len())
        .filter(|&at| matches!(&air.calls[at], Call::Management(b, _) if b[0] == 0xb0))
        .collect()
}

#[test]
fn an_sae_network_authenticates_associates_and_keys() {
    // wpa_supplicant's SAE through mac80211's external authentication
    // (spec S1.16): the station's commit, the AP's, the station's confirm,
    // the AP's; then the association under required PMF, the PTK and GTK
    // in the CAM and the IGTK on the host (spec S1.2).
    let (mut wifi, model) = wifi_with(|air| air.sae = true);
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    assert!(wifi.radio().station().requires_protected_management());
    let air = model.0.borrow();
    assert_eq!(sae_sent(&air), [(1, 0), (2, 0)]);
    assert!(
        air.management(0xb0)
            .iter()
            .all(|frame| frame[4..10] == AP && frame[16..22] == AP)
    );
    // `ieee80211_prep_connection`'s channel and BSSID came first. The AP's
    // confirm of the station's moved the AP's station to AUTH
    // (`ieee80211_mark_sta_auth`), once, before the association request.
    let tune = air.position(|c| *c == Call::Tune(channel(1))).unwrap();
    let bssid = air.position(|c| *c == Call::SetBssid(Mac(AP))).unwrap();
    let sent = authentications(&air);
    let authenticated = air.position(|c| *c == Call::Authenticated).unwrap();
    let request = air
        .position(|c| matches!(c, Call::Management(b, _) if b[0] == 0x00))
        .unwrap();
    assert!(tune < bssid && bssid < sent[0]);
    assert!(sent[1] < authenticated && authenticated < request);
    assert_eq!(air.count(|c| *c == Call::Authenticated), 1);
    // The request names SAE, with MFPC.
    let own = Security::select(&RSN_SAE, true).unwrap().element();
    assert!(air.management(0x00)[0].windows(own.len()).any(|w| w == own));
    let pmk = air.sae_ap.as_ref().unwrap().pmk().unwrap();
    drop(air);
    // The 4-way handshake under the SAE PMK, with the IGTK.
    let security = Security::select(&RSN_SAE, true).unwrap();
    let ap = Authenticator::between(
        security.suites(),
        pmk,
        Mac(AP),
        Mac(LOCAL),
        security.element(),
    );
    shake_with(&mut wifi, &model, ap, &RSN_SAE, &IGTK_KDE);
    assert!(wifi.carrier(), "{}", progress(&mut wifi));
    assert_eq!(model.0.borrow().keys(), [(true, 0, 0), (false, 1, 1)]);
    assert!(wifi.radio().integrity.is_some());
    let line = progress(&mut wifi);
    assert!(
        line.starts_with(&alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), sae 2/2, assoc 1/1 status 0 \
             aid 1, eapol rx 2 tx 2 acked 2, keys 2, port open, {LINK_CH1}, rx data 0 \
             sealed 0 nokey 0 refused 0, tx data 0; "
        )),
        "{line}"
    );
}

#[test]
fn a_failed_sae_ends_and_retries() {
    // An AP that never acknowledges: each commit goes again at once on its
    // TX status (`ieee80211_sta_work`), three tries
    // (IEEE80211_AUTH_MAX_TRIES); the exchange then ends, the chip leaves
    // the BSS it was set to, and the keeper waits wpa_supplicant's 10 s
    // before it commits again.
    let (mut wifi, model) = wifi_with(|air| {
        air.sae = true;
        air.deaf_to_auth = true;
    });
    let mut lines = Vec::new();
    let mut sent = Vec::new();
    let mut joins = Vec::new();
    let mut gave_up = None;
    assert!(run(&mut wifi, &model, |wifi, air| {
        lines.push(progress(wifi));
        let now = CLOCK.now_us();
        while sent.len() < air.management(0xb0).len() {
            sent.push(now);
        }
        let set = air.count(|c| matches!(c, Call::SetBssid(_)));
        if joins.len() < set {
            joins.push(now);
        }
        if gave_up.is_none() && air.count(|c| *c == Call::Disassociate) == 1 {
            gave_up = Some(now);
        }
        set == 2 && sent.len() == 4
    }));
    assert!(lines.iter().any(|l| l.contains(", sae 1/0, ")));
    assert!(lines.iter().any(|l| l.contains(", sae 3/0, ")));
    // Unacknowledged, each try left at once, well inside the 500 ms a frame
    // waits for its TX status (IEEE80211_AUTH_TIMEOUT_LONG).
    assert!(sent[2] - sent[0] < 100_000, "{sent:?}");
    let air = model.0.borrow();
    assert_eq!(sae_sent(&air), [(1, 0); 4]);
    let first = air.position(|c| matches!(c, Call::SetBssid(_))).unwrap();
    let left = air.position(|c| *c == Call::Disassociate).unwrap();
    let commits = authentications(&air);
    assert!(first < commits[0] && commits[2] < left && left < commits[3]);
    // The AP's station never authenticated.
    assert_eq!(air.count(|c| *c == Call::Authenticated), 0);
    drop(air);
    assert!(joins[1] - gave_up.unwrap() >= 10_000_000);
    let Err(LinkError::Adapter(failure)) = wifi.receive() else {
        panic!("no failure")
    };
    assert!(
        failure.contains("SAE authentication ended: Timeout"),
        "{failure}"
    );
}

#[test]
fn an_ap_confirming_first_authenticates_the_station_as_it_confirms() {
    // `ieee80211_mgd_auth`: once the AP confirmed, the station's confirm
    // completes the exchange, the AP's station AUTH before it leaves.
    let (mut wifi, model) = wifi_with(|air| {
        air.sae = true;
        air.sae_eager = true;
    });
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    let air = model.0.borrow();
    assert_eq!(sae_sent(&air), [(1, 0), (2, 0)]);
    let sent = authentications(&air);
    let authenticated = air.position(|c| *c == Call::Authenticated).unwrap();
    assert!(sent[0] < authenticated && authenticated < sent[1]);
    assert_eq!(air.count(|c| *c == Call::Authenticated), 1);
}

#[test]
fn a_deauthentication_during_sae_is_ignored_and_the_exchange_completes() {
    // `ieee80211_rx_mgmt_deauth` acts only with `ifmgd->associated` or
    // `assoc_data` (`mlme.c:5646-5690` at the donor's mac80211): with only
    // `auth_data` it returns, and the exchange runs on.
    let (mut wifi, model) = wifi_with(|air| {
        air.sae = true;
        air.sae_deauth = true;
    });
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    let air = model.0.borrow();
    assert_eq!(sae_sent(&air), [(1, 0), (2, 0)]);
    assert_eq!(air.count(|c| *c == Call::Disassociate), 0);
    assert_eq!(air.count(|c| *c == Call::Authenticated), 1);
    drop(air);
    assert_eq!(wifi.receive(), Ok(None));
}

#[test]
fn an_sae_frame_to_another_station_is_not_the_exchanges() {
    // `ieee80211_accept_frame` hands mlme only frames to the station's own
    // address or a group address: the AP's commit to another station never
    // reaches the keeper's exchange, which completes with the AP's answer.
    let (mut wifi, model) = wifi_with(|air| {
        air.sae = true;
        air.sae_stray = true;
    });
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    let air = model.0.borrow();
    assert_eq!(sae_sent(&air), [(1, 0), (2, 0)]);
    assert_eq!(air.count(|c| *c == Call::Disassociate), 0);
    drop(air);
    assert_eq!(wifi.receive(), Ok(None));
}

#[test]
fn an_sae_exchange_for_an_mld_link_is_unsupported() {
    // The chip has one address, the one its address CAM holds.
    let (_, model) = wifi();
    let database = regdb(&[(*b"US", &US[..])]);
    let country = database.select(*b"US").unwrap();
    let mut radio = Rtl8852c::new(model.clone(), Mac(LOCAL), country, CLOCK).unwrap();
    let peer = Peer {
        bssid: Mac(AP),
        channel: channel(1),
        basic_rates: 1,
        supported_rates: 0x11,
        beacon_interval: 100,
        dtim_period: 2,
        slot_time: 20,
    };
    let affiliation = Affiliation {
        mld: Mld {
            local: Mac(LOCAL),
            ap: Mac(AP),
        },
        link_id: 0,
        local: Mac([0x02, 0x11, 0x22, 0x33, 0x44, 0x66]),
    };
    let got = radio.begin_external_authentication(0, peer, Some(affiliation), 10_000_000);
    assert!(
        matches!(got, Err(Error::Unsupported("MLO on an RTL8852C"))),
        "{got:?}"
    );
    assert!(model.0.borrow().calls.is_empty());
}

#[test]
fn a_wrong_sae_confirm_deauthenticates_and_leaves() {
    // mac80211 takes the AP's confirm (status 0) as the exchange's end, its
    // station AUTH; wpa_supplicant's check of it fails, and
    // `ieee80211_mgd_deauth` aborts the authentication with a
    // deauthentication to the AP before the chip leaves.
    let (mut wifi, model) = wifi_with(|air| {
        air.sae = true;
        air.sae_wrong_confirm = true;
    });
    assert!(run(&mut wifi, &model, |_, air| {
        air.count(|c| *c == Call::Disassociate) == 1
    }));
    let air = model.0.borrow();
    assert_eq!(sae_sent(&air), [(1, 0), (2, 0)]);
    let authenticated = air.position(|c| *c == Call::Authenticated).unwrap();
    let deauth = air
        .position(|c| matches!(c, Call::Management(b, _) if b[0] == 0xc0))
        .unwrap();
    let left = air.position(|c| *c == Call::Disassociate).unwrap();
    assert!(authenticated < deauth && deauth < left);
    let Call::Management(frame, _) = &air.calls[deauth] else {
        unreachable!()
    };
    // Unprotected, from the station to the AP, reason 3 (leaving).
    assert_eq!(frame[1] & 0x40, 0);
    assert_eq!((&frame[4..10], &frame[10..16]), (&AP[..], &LOCAL[..]));
    assert_eq!(frame[24..], [3, 0]);
    assert_eq!(air.management(0x00).len(), 0);
    drop(air);
    let Err(LinkError::Adapter(failure)) = wifi.receive() else {
        panic!("no failure")
    };
    assert!(failure.contains("SAE confirm failed"), "{failure}");
}

#[test]
fn a_deauthentication_from_another_ap_leaves_the_exchange_running() {
    // `ieee80211_rx_mgmt_deauth` takes no deauthentication while only
    // authenticating, and `ieee80211_rx_mgmt_auth` ignores another BSSID's
    // frame.
    let (mut wifi, model) = wifi_with(|air| air.sae = true);
    assert!(run(&mut wifi, &model, |_, air| !air
        .management(0xb0)
        .is_empty()));
    let other = [0x02, 0xaa, 0, 0, 0, 9, 0x02, 0xaa, 0, 0, 0, 9];
    let mut deauth = management(0xc0, &[1, 0]);
    deauth[10..22].copy_from_slice(&other);
    let mut commit = sae_frame(&sae_ap().commit());
    commit[10..22].copy_from_slice(&other);
    // And the AP's to another station.
    let mut elsewhere = management(0xc0, &[1, 0]);
    elsewhere[4..10].copy_from_slice(&other[..6]);
    model.0.borrow_mut().hear(deauth);
    model.0.borrow_mut().hear(commit);
    model.0.borrow_mut().hear(elsewhere);
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    let air = model.0.borrow();
    assert_eq!(air.count(|c| *c == Call::Disassociate), 0);
    assert_eq!(sae_sent(&air), [(1, 0), (2, 0)]);
    drop(air);
    // Dropped as mac80211 drops them, nothing reported.
    assert_eq!(wifi.receive(), Ok(None));
}

#[test]
fn the_progress_line_names_the_last_joins_authentication() {
    // An SAE attempt that failed, then the network's AP offering WPA2 only:
    // the counts add up, under the open system's name.
    let (mut wifi, model) = wifi_with(|air| {
        air.sae = true;
        air.deaf_to_auth = true;
    });
    assert!(run(&mut wifi, &model, |_, air| {
        air.count(|c| *c == Call::Disassociate) == 1
    }));
    assert!(progress(&mut wifi).contains(", sae 3/0, "));
    {
        let mut air = model.0.borrow_mut();
        air.sae = false;
        air.deaf_to_auth = false;
    }
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    let line = progress(&mut wifi);
    assert!(
        line.contains(", auth 4/1 status 0, assoc 1/1 status 0"),
        "{line}"
    );
}

/// A rate the model hears the AP's frames at: VHT, two streams.
fn vht(mcs: u8, bw: u8, sgi: bool) -> Rate {
    Rate {
        mode: RateMode::Vht,
        nss: 2,
        mcs,
        bw,
        sgi,
    }
}

#[test]
fn histogram_counts_delivered_data_only() {
    // M4 spec S3.1: every data frame the chip hands on addressed to the
    // station, with its rate and A-MPDU bit; nothing else.
    let (mut wifi, model) = associated();
    let body = [0x45; 46];
    let mut air = model.0.borrow_mut();
    air.rx_rate = Some(vht(9, 2, true));
    air.rx_ampdu = true;
    air.hear(data(LOCAL, 1, [8, 0], &body));
    air.hear(data(LOCAL, 2, [8, 0], &body));
    air.rx_ampdu = false;
    air.rx_rate = Some(vht(7, 2, true));
    air.hear(data(LOCAL, 3, [8, 0], &body));
    // A beacon, another station's data, and a frame without a rate.
    let beacon = beacon(&air);
    air.hear(beacon);
    air.hear(data(OTHER_STATION, 4, [8, 0], &body));
    air.rx_rate = None;
    air.hear(data(LOCAL, 5, [8, 0], &body));
    drop(air);
    wifi.carrier();
    let histogram = wifi.radio().rx_rates();
    assert_eq!((histogram.total, histogram.ampdu), (3, 2));
    assert_eq!(histogram.counts.len(), 2);
    assert_eq!(histogram.top(), Some((vht(9, 2, true), 2)));
    let line = progress(&mut wifi);
    assert!(
        line.contains(", rx vht80 2ss mcs9 sgi 66% (3), ampdu 66%, "),
        "{line}"
    );
}

#[test]
fn progress_line_has_the_m4_fields() {
    // M4 spec S3.4 on 5 GHz channel 157: the AP's VHT operation is 80 MHz
    // around 155, while the station (HT, M3) joins at 20 MHz.
    let (mut wifi, model) = wifi_with(|air| {
        air.channel = Some(channel(157));
        air.beacon_extra = vec![192, 5, 1, 155, 0, 0xfc, 0xff];
        air.link = LinkCounts {
            regd: Some((*b"IN", Regd::Etsi)),
            rxq_full: 0,
            tx: TxCounts {
                kicks: [4, 0, 0, 0, 0],
                released: [10, 0, 0, 0, 0],
                ..TxCounts::default()
            },
        };
    });
    // Before the join, what is not known yet prints `-`.
    let line = progress(&mut wifi);
    assert!(
        line.ends_with(
            ", port closed, ap -, regd IN ETSI, rx -, ampdu -, rxq full 0, tx ra -, \
             kicks 4 rel 10 (2.5/kick), addba 0/0, agg -; chip"
        ),
        "{line}"
    );
    let joined = run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    });
    assert!(joined, "{}", progress(&mut wifi));
    let mut air = model.0.borrow_mut();
    air.rx_rate = Some(vht(8, 0, false));
    air.hear(data(LOCAL, 1, [8, 0], &[0x45; 46]));
    // The firmware's RA report: VHT 2SS MCS 7, short GI, 20 MHz.
    air.incoming.push_back(Incoming::RaReport(RaReport {
        macid: 0,
        mode: 2,
        rate: 0x17,
        giltf: 1,
        bw: 0,
    }));
    drop(air);
    wifi.carrier();
    let line = progress(&mut wifi);
    assert!(
        line.contains(
            ", ap vht80 ch 157 (c 155), regd IN ETSI, rx vht20 2ss mcs8 100% (1), ampdu 0%, \
             rxq full 0, tx ra vht20 2ss mcs7 sgi retry -, kicks 4 rel 10 (2.5/kick), \
             addba 0/0, agg -, rx data "
        ),
        "{line}"
    );
    assert!(!names_an_address(&line), "{line}");
}

/// wireless-regdb's IN rules (M4 spec S14.1: 5725-5875 MHz at 80 MHz, so
/// 149-173 by the rules), and a US-like 5 GHz whose 5835-5895 MHz rule is
/// NO-IR, as [`regdb`] builds rules (regulatory.db's flags: DFS 1 << 2,
/// NO-IR 1 << 3).
const IN_RULES: [(u32, u32, u8, u16); 5] = [
    (2_402_000, 2_482_000, 0, 3000),
    (5_150_000, 5_250_000, 0, 3000),
    (5_250_000, 5_350_000, 1 << 2, 2400),
    (5_470_000, 5_725_000, 1 << 2, 2400),
    (5_725_000, 5_875_000, 0, 3000),
];
const UNII4_RULES: [(u32, u32, u8, u16); 3] = [
    (2_402_000, 2_472_000, 0, 3000),
    (5_735_000, 5_835_000, 0, 3000),
    (5_835_000, 5_895_000, 1 << 3, 2700),
];

/// The 5 GHz channel numbers `channels` offers, and their flags.
fn five_ghz(offered: &[(Channel, u32)]) -> Vec<(u16, u32)> {
    offered
        .iter()
        .filter(|(c, _)| c.band == Band::Ghz5)
        .map(|(c, flags)| (c.number, *flags))
        .collect()
}

#[test]
fn unii4_is_us_only_without_acpi() {
    // rtw89_regd_apply_policy_unii4 (regd.c:770-797) disables channels 169,
    // 173 and 177 (from RTW89_5GHZ_UNII4_START_INDEX 25, core.h:6991-6992)
    // for every regd but US, whatever the country's rules allow.
    // A database's countries are in alpha2 order.
    let database = regdb(&[
        (*b"CA", &UNII4_RULES[..]),
        (*b"IN", &IN_RULES[..]),
        (*b"US", &UNII4_RULES[..]),
    ]);
    let india = channels(&database.clone().select(*b"IN").unwrap(), *b"IN");
    assert!(five_ghz(&india).iter().all(|&(n, _)| n < 169), "{india:?}");
    let us = five_ghz(&channels(&database.clone().select(*b"US").unwrap(), *b"US"));
    let unii4: Vec<u16> = us.iter().map(|&(n, _)| n).filter(|&n| n >= 169).collect();
    assert_eq!(unii4, [169, 173, 177]);
    // Each keeps its rule's flags (NO-IR here).
    assert!(
        us.iter()
            .filter(|&&(n, _)| n >= 169)
            .all(|&(_, f)| f & NO_IR != 0)
    );
    let canada = channels(&database.clone().select(*b"CA").unwrap(), *b"CA");
    assert!(five_ghz(&canada).iter().all(|&(n, _)| n < 169));
    // The world regd, and a US channel plan under the world regd's index.
    let world = channels(&database.clone().select(*b"US").unwrap(), *b"00");
    assert!(five_ghz(&world).iter().all(|&(n, _)| n < 169));
    // The radio takes the regd its chip found (Live::regds' alpha2).
    let model = |alpha2: [u8; 2]| {
        Model(Rc::new(RefCell::new(Air {
            link: LinkCounts {
                regd: Some((alpha2, Regd::Fcc)),
                ..LinkCounts::default()
            },
            ..Air::default()
        })))
    };
    for (alpha2, unii4) in [(*b"US", 3), (*b"IN", 0)] {
        let country = database.clone().select(*b"US").unwrap();
        let radio = Rtl8852c::new(model(alpha2), Mac(LOCAL), country, CLOCK).unwrap();
        let offered = radio.channels(Band::Ghz5);
        let past = offered.iter().filter(|(c, _)| c.number >= 169).count();
        assert_eq!(past, unii4, "{alpha2:?}");
    }
}

#[test]
fn in_keeps_149_to_165() {
    let database = regdb(&[(*b"IN", &IN_RULES[..])]);
    let india = five_ghz(&channels(&database.clone().select(*b"IN").unwrap(), *b"IN"));
    let upper: Vec<u16> = india
        .iter()
        .map(|&(n, _)| n)
        .filter(|&n| n >= 149)
        .collect();
    assert_eq!(upper, [149, 153, 157, 161, 165]);
    assert!(india.iter().all(|&(_, f)| f & NO_IR == 0));
}

#[test]
fn kick_once_per_pass_covers_every_submitted_frame() {
    // M4 spec S9.1: the frames the network hands over go to the chip as they
    // come (`rtw89_pci_tx_write`), and the radio's service pass kicks once
    // (`rtw89_core_txq_schedule`'s `rtw89_core_tx_kick_off`). TID 0's
    // agreement is made first, as data waits while it is made (spec S10.1).
    let (mut wifi, model) = connected();
    wifi.transmit(&ip(9)).unwrap();
    assert!(run(&mut wifi, &model, |wifi, _| wifi.radio().aggregating
        [0]
    .is_some()));
    let (before, kicks) = {
        let air = model.0.borrow();
        assert!(air.unkicked.is_empty());
        (air.data().len(), air.kicks)
    };
    for fill in 0..5 {
        wifi.transmit(&ip(fill)).unwrap();
    }
    {
        let air = model.0.borrow();
        assert_eq!(air.data().len(), before + 5);
        assert_eq!(air.unkicked.len(), 5);
        assert_eq!(air.kicks, kicks);
    }
    wifi.carrier();
    let air = model.0.borrow();
    assert_eq!(air.kicks, kicks + 1);
    assert!(air.unkicked.is_empty());
    // Every frame once, in order.
    let fills: Vec<u8> = air.data()[before..]
        .iter()
        .map(|(bytes, _)| *bytes.last().unwrap())
        .collect();
    assert_eq!(fills, [0, 1, 2, 3, 4]);
    assert_eq!(air.data().len(), before + 5);
}

// TX Block Ack (M4 Task 8, spec S10).

/// An IPv4 frame from the station to the AP of type of service `tos`, its
/// body `fill`: by RFC 8325, 0xb8 (EF) is TID 6 and 0xc0 (CS6) TID 7.
fn ip_tos(tos: u8, fill: u8) -> Vec<u8> {
    let mut ethernet = ip(fill);
    ethernet[15] = tos;
    ethernet
}

/// The station's Block Ack action frames of action `action` (0 the ADDBA
/// Request, 2 the DELBA), unprotected.
fn block_acks(air: &Air, action: u8) -> Vec<Vec<u8>> {
    air.management(0xd0)
        .into_iter()
        .filter(|frame| frame[24..26] == [3, action])
        .collect()
}

/// The chip's TX Block Ack calls from call `from` on.
fn aggregations(air: &Air, from: usize) -> Vec<(u8, Option<u16>)> {
    air.calls[from..]
        .iter()
        .filter_map(|c| match c {
            Call::Aggregation { tid, window } => Some((*tid, *window)),
            _ => None,
        })
        .collect()
}

/// The data frames but EAPOL from call `from` on: each one's first payload
/// byte past its IPv4 type of service, its TID and its A-MPDU.
fn aggregates(air: &Air, from: usize) -> Vec<(u8, u8, Option<txd::Aggregate>)> {
    air.calls[from..]
        .iter()
        .filter_map(|c| match c {
            Call::Data {
                bytes,
                tid,
                aggregate,
                ..
            } if eapol(bytes).is_none() => Some((bytes[36], *tid, *aggregate)),
            _ => None,
        })
        .collect()
}

/// A data frame's sequence number.
fn sequence(bytes: &[u8]) -> u16 {
    u16::from_le_bytes([bytes[22], bytes[23]]) >> 4
}

/// The A-MPDU of an agreement of 64 with the model's AP, whose HT A-MPDU
/// parameters (`HT[4]`, 0x17) are the length exponent 3 and the spacing 5.
const AGGREGATE_64: Option<txd::Aggregate> = Some(txd::Aggregate {
    agg_num: 64,
    factor: 3,
    density: 5,
});

/// Whether the radio's agreement on `tid` is operational.
fn agreed(wifi: &Wifi<Model>, tid: usize) -> bool {
    wifi.radio().aggregating[tid].is_some()
}

#[test]
fn first_qos_data_starts_an_agreement() {
    // rtw89_core_txq_check_agg (core.c:4707-4741): the first QoS data on a
    // TID with no agreement queues ba_work, whose
    // ieee80211_start_tx_ba_session sends an ADDBA Request (agg-tx.c), 64
    // for an HT peer (M4 spec S10.1), immediate, from the TID's next
    // sequence number. The AP's success is TX_OPERATIONAL
    // (Chip::aggregation), and the TID's next frames go in A-MPDUs of the
    // agreement's window at the AP's spacing.
    let (mut wifi, model) = connected();
    let from = model.0.borrow().calls.len();
    wifi.transmit(&ip(1)).unwrap();
    assert!(run(&mut wifi, &model, |wifi, _| agreed(wifi, 0)));
    {
        let air = model.0.borrow();
        let [request] = &block_acks(&air, 0)[..] else {
            panic!("{} requests", block_acks(&air, 0).len())
        };
        assert_eq!(request[4..10], AP);
        // Immediate (b1) on TID 0 (b2-5), a window of 64 (b6-15), no
        // timeout, from the sequence number after the first frame's.
        let parameters = u16::from_le_bytes([request[27], request[28]]);
        assert_eq!(parameters & 0x3e, 0x02);
        assert_eq!(parameters >> 6, 64);
        assert_eq!(request[29..31], [0, 0]);
        let first = air.calls[from..]
            .iter()
            .find_map(|c| match c {
                Call::Data { bytes, .. } => Some(sequence(bytes)),
                _ => None,
            })
            .unwrap();
        let ssn = u16::from_le_bytes([request[31], request[32]]) >> 4;
        assert_eq!(ssn, (first + 1) & 0xfff);
        assert_eq!(aggregations(&air, from), [(0, Some(64))]);
    }
    for fill in 2..5 {
        wifi.transmit(&ip(fill)).unwrap();
    }
    assert_eq!(
        aggregates(&model.0.borrow(), from),
        [
            (1, 0, None),
            (2, 0, AGGREGATE_64),
            (3, 0, AGGREGATE_64),
            (4, 0, AGGREGATE_64)
        ]
    );
    let line = progress(&mut wifi);
    assert!(line.contains(", addba 1/1, agg 0, rx data"), "{line}");
}

#[test]
fn the_ap_hears_the_data_before_the_request_it_started() {
    // rtw89_core_txq_push writes the frame, rtw89_core_txq_schedule kicks
    // it (core.c:4743-4767, :4854-4858), and only then does ba_work send the
    // ADDBA Request whose SSN follows it. The batched kick (M4 spec S9.1)
    // must not let the request, on CH8 at once, overtake that frame, often a
    // DHCP Discover, on ACH0: an AP would take it for one before the window.
    let (mut wifi, model) = connected();
    let from = model.0.borrow().heard.len();
    wifi.transmit(&ip(1)).unwrap();
    assert!(run(&mut wifi, &model, |wifi, _| agreed(wifi, 0)));
    let air = model.0.borrow();
    let heard = &air.heard[from..];
    let data = heard
        .iter()
        .position(|f| f[0] == 0x88 && eapol(f).is_none() && f[36] == 1)
        .unwrap();
    let request = heard
        .iter()
        .position(|f| f[0] == 0xd0 && f[24..26] == [3, 0])
        .unwrap();
    assert!(data < request, "data {data}, request {request}");
}

/// The position among `heard` of the first frame `want` takes.
fn heard_at(heard: &[Vec<u8>], want: impl Fn(&[u8]) -> bool) -> usize {
    heard.iter().position(|f| want(f)).unwrap()
}

/// Whether `f` is the station's data whose first payload byte is `fill`.
fn data_of(f: &[u8], fill: u8) -> bool {
    f[0] == 0x88 && eapol(f).is_none() && f[36] == fill
}

/// Whether `f` is a Block Ack action frame of action `action`.
fn block_ack_of(f: &[u8], action: u8) -> bool {
    f[0] == 0xd0 && f[24..26] == [3, action]
}

#[test]
fn a_silent_ap_times_the_request_out() {
    // ieee80211_addba_resp_timer_expired: no answer within
    // ADDBA_RESP_INTERVAL, 1 s, stops the session with a DELBA
    // (ieee80211_stop_tx_ba_session: TX_STOP, then the DELBA); the frame
    // that waited leaves after it, singly, and asks again as the next
    // frame of a TID with no session (rtw89_core_txq_check_agg).
    let (mut wifi, model) = connected();
    model.0.borrow_mut().addba_silent = true;
    let (from, heard_from) = {
        let air = model.0.borrow();
        (air.calls.len(), air.heard.len())
    };
    let start = CLOCK.now_us();
    for fill in 1..=2 {
        wifi.transmit(&ip(fill)).unwrap();
    }
    assert!(run(&mut wifi, &model, |_, air| !block_acks(air, 2).is_empty()));
    let waited = CLOCK.now_us() - start;
    assert!((1_000_000..1_100_000).contains(&waited), "{waited}");
    assert!(run(&mut wifi, &model, |_, air| block_acks(air, 0).len() == 2));
    let air = model.0.borrow();
    assert_eq!(aggregations(&air, from), [(0, None)]);
    assert_eq!(aggregates(&air, from), [(1, 0, None), (2, 0, None)]);
    let [frame] = &block_acks(&air, 2)[..] else {
        panic!("{} DELBAs", block_acks(&air, 2).len())
    };
    // The initiator's, on TID 0.
    assert_eq!(u16::from_le_bytes([frame[26], frame[27]]), 1 << 11);
    let heard = &air.heard[heard_from..];
    let second = heard
        .iter()
        .enumerate()
        .filter(|(_, f)| block_ack_of(f, 0))
        .nth(1)
        .unwrap()
        .0;
    let order = [
        heard_at(heard, |f| data_of(f, 1)),
        heard_at(heard, |f| block_ack_of(f, 0)),
        heard_at(heard, |f| block_ack_of(f, 2)),
        heard_at(heard, |f| data_of(f, 2)),
        second,
    ];
    assert!(order.windows(2).all(|w| w[0] < w[1]), "{order:?}");
}

#[test]
fn a_declined_request_leaves_the_tid_single() {
    // ieee80211_process_addba_resp with a failure status:
    // ___ieee80211_stop_tx_ba_session(AGG_STOP_DECLINED), TX_STOP and no
    // DELBA (tx_stop is for a local request only); the data goes singly.
    // Each next frame asks again until HT_AGG_BURST_RETRIES (3) requests
    // are past: then ieee80211_start_tx_ba_session is -EBUSY for
    // HT_AGG_RETRIES_PERIOD, 15 s from the last (agg-tx.c), and asks again
    // after it.
    let (mut wifi, model) = connected();
    model.0.borrow_mut().addba_status = 37;
    let from = model.0.borrow().calls.len();
    for fill in 1..=6u8 {
        wifi.transmit(&ip(fill)).unwrap();
        assert!(run(&mut wifi, &model, |wifi, air| {
            aggregates(air, from).len() == usize::from(fill)
                && !wifi.radio().station().tx_block_ack_blocks_data(0)
        }));
    }
    {
        let air = model.0.borrow();
        assert_eq!(block_acks(&air, 0).len(), 4);
        assert_eq!(aggregations(&air, from), [(0, None); 4]);
        assert!(block_acks(&air, 2).is_empty());
        assert!(aggregates(&air, from).iter().all(|(_, _, a)| a.is_none()));
    }
    let line = progress(&mut wifi);
    assert!(line.contains(", addba 0/4, agg -, "), "{line}");
    crate::test_clock::advance(CLOCK, 15_000_000);
    wifi.carrier();
    wifi.transmit(&ip(7)).unwrap();
    assert!(run(&mut wifi, &model, |_, air| block_acks(air, 0).len() == 5));
}

/// The AP's DELBA of the station's agreement on `tid`, as its recipient
/// (initiator b11 clear), reason 37.
fn recipient_delba(tid: u8) -> Vec<u8> {
    let mut body = vec![3, 2];
    body.extend((u16::from(tid) << 12).to_le_bytes());
    body.extend(37u16.to_le_bytes());
    management(0xd0, &body)
}

#[test]
fn the_aps_delba_ends_the_agreement() {
    // ieee80211_process_delba from the recipient:
    // __ieee80211_stop_tx_ba_session(AGG_STOP_PEER_REQUEST), TX_STOP and no
    // DELBA of the station's. The next frame goes singly and, the TID
    // neither forbidden nor blocked, asks again (rtw89_core_txq_check_agg).
    let (mut wifi, model) = connected();
    wifi.transmit(&ip(1)).unwrap();
    assert!(run(&mut wifi, &model, |wifi, _| agreed(wifi, 0)));
    let from = model.0.borrow().calls.len();
    model.0.borrow_mut().hear(recipient_delba(0));
    assert!(run(&mut wifi, &model, |wifi, _| !agreed(wifi, 0)
        && !wifi.radio().station().tx_block_ack_blocks_data(0)));
    {
        let air = model.0.borrow();
        assert_eq!(aggregations(&air, from), [(0, None)]);
        assert!(block_acks(&air, 2).is_empty());
    }
    wifi.transmit(&ip(2)).unwrap();
    assert_eq!(aggregates(&model.0.borrow(), from), [(2, 0, None)]);
    assert!(run(&mut wifi, &model, |wifi, air| {
        block_acks(air, 0).len() == 2 && agreed(wifi, 0)
    }));
    assert_eq!(
        aggregations(&model.0.borrow(), from),
        [(0, None), (0, Some(64))]
    );
}

#[test]
fn a_reassociation_makes_a_fresh_agreement() {
    // The departure ends the agreement (TX_STOP_FLUSH); the next
    // association configures its own limits and its first data asks again.
    let (mut wifi, model) = connected();
    wifi.transmit(&ip(1)).unwrap();
    assert!(run(&mut wifi, &model, |wifi, _| agreed(wifi, 0)));
    wifi.radio_mut().deauthenticate(3).unwrap();
    assert!(run(&mut wifi, &model, |wifi, air| {
        air.count(|c| matches!(c, Call::Associate(_))) == 2
            && wifi.radio().station().phase() == Peering::Associated
    }));
    // The new association's 4-way handshake, counted from here.
    model.0.borrow_mut().calls.clear();
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
    assert!(run(&mut wifi, &model, |wifi, _| wifi.radio().port_open));
    wifi.transmit(&ip(2)).unwrap();
    assert!(run(&mut wifi, &model, |wifi, _| agreed(wifi, 0)));
    let air = model.0.borrow();
    assert_eq!(block_acks(&air, 0).len(), 1);
    assert_eq!(aggregations(&air, 0), [(0, Some(64))]);
    drop(air);
    let line = progress(&mut wifi);
    assert!(!line.contains("Block Ack limits"), "{line}");
    assert!(line.contains(", addba 2/2, agg 0, "), "{line}");
}

#[test]
fn a_prepare_waits_only_for_its_own_tids_numbered_data() {
    // mac80211 synchronizes a TID's own TX before its ADDBA
    // (ieee80211_agg_start_txq after the stop); a non-QoS frame, held on
    // the BE queue as TID 0's would be, is no frame of TID 0's session.
    let (mut wifi, model) = connected();
    model.0.borrow_mut().full_data = true;
    wifi.transmit(&ip(1)).unwrap();
    // The held QoS frame is replaced by a non-QoS one on the same queue.
    let radio = wifi.radio_mut();
    let mut held = radio.held.pop_front().unwrap();
    held.bytes[0] = 0x08;
    held.qos = false;
    radio.held.push_back(held);
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    assert_eq!(block_acks(&model.0.borrow(), 0).len(), 1);
}

#[test]
fn an_agreement_takes_the_responses_window_up_to_128() {
    // __ieee80211_start_tx_ba_session's agreement takes min(the response's
    // window, the 128 of TRANSMIT_WINDOW) (agg-tx.c:992-1000), and
    // TX_OPERATIONAL carries it; a smaller answer is kept as it is.
    for (answer, window) in [(256, 128), (32, 32)] {
        let (mut wifi, model) = connected();
        model.0.borrow_mut().addba_window = Some(answer);
        let from = model.0.borrow().calls.len();
        wifi.transmit(&ip(1)).unwrap();
        assert!(run(&mut wifi, &model, |wifi, _| agreed(wifi, 0)));
        wifi.transmit(&ip(2)).unwrap();
        let air = model.0.borrow();
        assert_eq!(aggregations(&air, from), [(0, Some(window))]);
        let aggregate = aggregates(&air, from)[1].2.unwrap();
        assert_eq!(aggregate.agg_num, window);
    }
}

#[test]
fn parked_frames_keep_their_order_and_pns() {
    // ieee80211_agg_stop_txq: while the agreement is made, the TID's data
    // waits in its TXQ unnumbered, then leaves in order behind the ADDBA
    // Request (ieee80211_agg_start_txq), each frame taking its sequence
    // number and PN as it leaves.
    let (mut wifi, model) = connected();
    let from = model.0.borrow().calls.len();
    for fill in 1..=6 {
        wifi.transmit(&ip(fill)).unwrap();
    }
    assert_eq!(aggregates(&model.0.borrow(), from).len(), 1);
    assert!(run(&mut wifi, &model, |_, air| {
        aggregates(air, from).len() == 6
    }));
    let air = model.0.borrow();
    let request = air
        .position(|c| matches!(c, Call::Management(b, _) if b[0] == 0xd0 && b[24..26] == [3, 0]))
        .unwrap();
    let data: Vec<usize> = (from..air.calls.len())
        .filter(|&at| matches!(air.calls[at], Call::Data { .. }))
        .collect();
    assert!(data[0] < request && request < data[1]);
    let sent: Vec<(u8, u64, u16)> = data
        .iter()
        .map(|&at| match &air.calls[at] {
            Call::Data { bytes, key, .. } => (bytes[36], key.unwrap().2, sequence(bytes)),
            _ => unreachable!(),
        })
        .collect();
    let fills: Vec<u8> = sent.iter().map(|s| s.0).collect();
    assert_eq!(fills, [1, 2, 3, 4, 5, 6]);
    let pns: Vec<u64> = sent.iter().map(|s| s.1).collect();
    assert_eq!(pns, [1, 2, 3, 4, 5, 6]);
    assert!(sent.windows(2).all(|w| w[1].2 == (w[0].2 + 1) & 0xfff));
    // The request's SSN is the second frame's.
    let request = &block_acks(&air, 0)[0];
    assert_eq!(
        u16::from_le_bytes([request[31], request[32]]) >> 4,
        sent[1].2
    );
    let aggregated: Vec<bool> = aggregates(&air, from)
        .iter()
        .map(|(_, _, aggregate)| aggregate.is_some())
        .collect();
    assert_eq!(aggregated, [false, true, true, true, true, true]);
}

#[test]
fn eapol_stops_the_session_and_forbids_it_for_4_s() {
    // rtw89_core_stop_tx_ba_session (core.c:4682-4705): EAPOL on a TID whose
    // agreement is operational stops it with a DELBA
    // (ieee80211_stop_tx_ba_session: TX_STOP, then the DELBA from
    // ieee80211_stop_tx_ba_cb) and forbids the TID for
    // RTW89_FORBID_BA_TIMER, 4 s (core.h:56), until rtw89_forbid_ba_work
    // (core.c:4900-4912).
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
    assert!(wifi.carrier());
    wifi.transmit(&ip_tos(0xc0, 1)).unwrap();
    assert!(run(&mut wifi, &model, |wifi, _| agreed(wifi, 7)));
    // The AP's group rekey: the station's answer is EAPOL on TID 7.
    let from = model.0.borrow().calls.len();
    let heard_from = model.0.borrow().heard.len();
    let group = protected_as(LOCAL, 2, 0, 1, [0x88, 0x8e], &ap.group_message_1());
    model.0.borrow_mut().hear_as(group, true, false);
    assert!(run(&mut wifi, &model, |_, air| !block_acks(air, 2).is_empty()));
    {
        let air = model.0.borrow();
        let answer = air.calls[from..].iter().find_map(|c| match c {
            Call::Data {
                bytes,
                tid,
                aggregate,
                ..
            } if eapol(bytes).is_some() => Some((*tid, *aggregate)),
            _ => None,
        });
        assert_eq!(answer, Some((7, None)));
        assert_eq!(aggregations(&air, from), [(7, None)]);
        let stop = air
            .position(|c| {
                *c == Call::Aggregation {
                    tid: 7,
                    window: None,
                }
            })
            .unwrap();
        let delete = air
            .position(
                |c| matches!(c, Call::Management(b, _) if b[0] == 0xd0 && b[24..26] == [3, 2]),
            )
            .unwrap();
        assert!(stop < delete);
        // The initiator's (b11) on TID 7, WLAN_REASON_QSTA_NOT_USE.
        let [frame] = &block_acks(&air, 2)[..] else {
            panic!("{} DELBAs", block_acks(&air, 2).len())
        };
        assert_eq!(frame[4..10], AP);
        assert_eq!(
            u16::from_le_bytes([frame[26], frame[27]]),
            7 << 12 | 1 << 11
        );
        assert_eq!(frame[28..30], 37u16.to_le_bytes());
        // The AP hears the answer, written before the DELBA, first.
        let heard = &air.heard[heard_from..];
        let answer = heard.iter().position(|f| eapol(f).is_some()).unwrap();
        let delete = heard
            .iter()
            .position(|f| f[0] == 0xd0 && f[24..26] == [3, 2])
            .unwrap();
        assert!(answer < delete, "answer {answer}, DELBA {delete}");
    }
    // Within the 4 s, TID 7's data starts nothing and goes singly.
    let from = model.0.borrow().calls.len();
    wifi.transmit(&ip_tos(0xc0, 2)).unwrap();
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    {
        let air = model.0.borrow();
        assert_eq!(aggregates(&air, from), [(2, 7, None)]);
        assert_eq!(block_acks(&air, 0).len(), 1);
    }
    // Past them, the next frame asks again.
    crate::test_clock::advance(CLOCK, 4_000_000);
    wifi.carrier();
    wifi.transmit(&ip_tos(0xc0, 3)).unwrap();
    assert!(run(&mut wifi, &model, |wifi, air| {
        block_acks(air, 0).len() == 2 && agreed(wifi, 7)
    }));
}

#[test]
fn vo_aggregates_as_rtw89_does() {
    // rtw89_core_txq_check_agg has no exception for the voice queue, where
    // rtw88's rtw_txq_check_agg has one: TID 6 data starts an agreement.
    let (mut wifi, model) = connected();
    wifi.transmit(&ip_tos(0xb8, 1)).unwrap();
    assert!(run(&mut wifi, &model, |wifi, _| agreed(wifi, 6)));
    let air = model.0.borrow();
    let [request] = &block_acks(&air, 0)[..] else {
        panic!("{} requests", block_acks(&air, 0).len())
    };
    assert_eq!(u16::from_le_bytes([request[27], request[28]]) >> 2 & 0xf, 6);
}

#[test]
fn a_refused_start_blocks_the_tid() {
    // rtw89_core_ba_work (core.c:4626-4632): ieee80211_start_tx_ba_session's
    // -EINVAL, here for an AP without HT, sets RTW89_TXQ_F_BLOCK_BA, and the
    // TID is not tried again in the association.
    let (mut wifi, model) = wifi_with(|air| air.no_ht = true);
    assert!(run(&mut wifi, &model, |wifi, _| {
        wifi.radio().station().phase() == Peering::Associated
    }));
    shake(&mut wifi, &model, &RSN, &[]);
    assert!(wifi.carrier(), "{}", progress(&mut wifi));
    let from = model.0.borrow().calls.len();
    wifi.transmit(&ip(1)).unwrap();
    let mut blocked = [false; 8];
    blocked[0] = true;
    assert_eq!(wifi.radio().blocked, blocked);
    wifi.transmit(&ip(2)).unwrap();
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    let air = model.0.borrow();
    assert!(block_acks(&air, 0).is_empty());
    assert_eq!(aggregates(&air, from), [(1, 0, None), (2, 0, None)]);
}

#[test]
fn pmf_addba_goes_under_the_ptk() {
    // Block Ack action frames are robust: under PMF the ADDBA Request leaves
    // sealed under the PTK (rtw88's send_agreement_frame), at the PN after
    // the data's, as one key's tx_pn counts both, and the AP's protected
    // response makes the agreement.
    let (mut wifi, model) = connected_with_pmf();
    let ptk = model.0.borrow().ptk();
    let from = model.0.borrow().calls.len();
    wifi.transmit(&ip(1)).unwrap();
    assert!(run(&mut wifi, &model, |_, air| !air
        .management(0xd0)
        .is_empty()));
    let opened = {
        let air = model.0.borrow();
        let [request] = &air.management(0xd0)[..] else {
            panic!("{} action frames", air.management(0xd0).len())
        };
        assert_eq!(request[1] & 0x40, 0x40);
        let h = &request[24..32];
        let pn = u64::from_le_bytes([h[0], h[1], h[4], h[5], h[6], h[7], 0, 0]);
        assert_eq!(pn, 2);
        ccmp::open(&ptk, request, PacketNumber::new(pn).unwrap()).unwrap()
    };
    assert_eq!(opened[32..34], [3, 0]);
    // The request as the AP reads it: no CCMP header or MIC.
    let mut clear = opened[..24].to_vec();
    clear.extend_from_slice(&opened[32..opened.len() - 8]);
    clear[1] &= !0x40;
    model
        .0
        .borrow_mut()
        .hear_as(sealed(addba_response(&clear, None, 0), 1), true, false);
    assert!(run(&mut wifi, &model, |wifi, _| agreed(wifi, 0)));
    assert_eq!(aggregations(&model.0.borrow(), from), [(0, Some(64))]);
}

#[test]
fn departure_stops_every_agreement_without_a_frame() {
    // __sta_info_destroy_part1's ieee80211_sta_tear_down_BA_sessions
    // (AGG_STOP_DESTROY_STA): each RX agreement's RX_STOP, then each TX
    // agreement's TX_STOP_FLUSH, no DELBA, before _part2's key deletes. The
    // next association configures its own.
    let (mut wifi, model) = aggregating();
    wifi.transmit(&ip(1)).unwrap();
    wifi.transmit(&ip_tos(0xb8, 2)).unwrap();
    assert!(run(&mut wifi, &model, |wifi, _| agreed(wifi, 0)
        && agreed(wifi, 6)));
    let line = progress(&mut wifi);
    assert!(line.contains(", addba 2/2, agg 0 6, "), "{line}");
    let from = model.0.borrow().calls.len();
    wifi.radio_mut().deauthenticate(3).unwrap();
    assert!(run(&mut wifi, &model, |_, air| {
        air.position(|c| *c == Call::Disassociate).is_some()
    }));
    assert_eq!(
        leaving(&model.0.borrow(), from),
        [
            "flush true",
            "mgmt c0",
            "flush false",
            "stop 0",
            "agg stop 0",
            "agg stop 6",
            "flush false",
            "remove 0",
            "flush false",
            "remove 1",
            "leave"
        ]
    );
    assert!(wifi.radio().aggregating.iter().all(Option::is_none));
    let from = model.0.borrow().calls.len();
    assert!(run(&mut wifi, &model, |wifi, air| {
        air.count(|c| matches!(c, Call::Associate(_))) == 2
            && wifi.radio().station().phase() == Peering::Associated
    }));
    for _ in 0..3 {
        run(&mut wifi, &model, |_, _| true);
    }
    assert!(aggregations(&model.0.borrow(), from).is_empty());
    let line = progress(&mut wifi);
    assert!(!line.contains("Block Ack limits"), "{line}");
    assert!(line.contains(", addba 2/2, agg -, "), "{line}");
}

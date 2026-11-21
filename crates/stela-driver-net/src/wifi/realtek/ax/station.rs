// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2018-2019 Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! An RTL8852CE as the connection keeper's radio (M3 spec S14). rtw89
//! leaves the MAC's soft half to mac80211, as rtw88 does, so the station's
//! orchestration is Stela's, after rtw88's `realtek/station.rs` (its
//! `Rtl8822c`, cited per method); the hardware beneath it is
//! [`chip::Chip`], which a live chip implements.
//!
//! - A scan is one firmware offload (`rtw89_ops_hw_scan`, spec S8) of every
//!   channel the country leaves, each passive (no probe templates, spec
//!   S1.3); its end restores the chip (`rtw89_hw_scan_complete`) before
//!   the keeper retires it.
//! - A join tunes to the BSS and sets its BSSID (`ieee80211_prep_connection`
//!   over `rtw89_set_channel` and `BSS_CHANGED_BSSID`), then authenticates
//!   and associates through the shared `Station`, whose frames leave on the
//!   chip's management queue at the BSS's lowest basic rate, each ticketed
//!   one's TX status from its release report. The AP's acceptance of the
//!   authentication moves its station to AUTH ([`chip::Chip::authenticated`]).
//! - The association programs the chip with the negotiated peer as
//!   `BSS_CHANGED_ASSOC` does (spec S10), on the channel the join switched
//!   to.
//! - EAPOL runs over the data queues, without a key; data frames wait in
//!   `held` while the chip has no room, management frames in the outbox.
//! - A departure leaves through the chip (spec S11), which releases its
//!   channel.
//!
//! The radio polls (spec S1.4): it never signals, and the keeper services
//! it from `carrier()` and `receive()`. Its diagnostic is the progress line
//! of spec S14.6, which names no address, then the chip's own counts.
//!
//! This module does not grant (M3a): keys (`install_key` refuses with
//! "keys before M3b", so the port never opens), encrypted or protected
//! frames, the IGTK, SAE, RX Block Ack and reordering, fragments and
//! A-MSDUs received, beacon loss, A-MSDU or aggregated TX, power save,
//! templates, the watchdog, or following the AP's beacons and channel
//! switches. Nor the chip half of mac80211's TX flush before a
//! deauthentication (`rtw89_ops_flush`'s ring wait and PKT_DROP H2Cs).
pub mod chip;

use self::chip::{Association, Chip, Data, Incoming, Management, Received, Token};
use super::assoc::HtPeer;
use super::live::KEYS_BEFORE_M3B;
use crate::{
    Clock, Link, LinkError, Mac,
    wifi::{
        Band, Channel, RadioStatus, RawRate, Timestamp,
        connection::keeper::{self, Advertisement, Keeper, Peer, Radio, Stage},
        frame::Mpdu,
        network::Network,
        regulatory::{channel::DISABLED, database::Country},
        scan::{self, Outcome, Retirement, Scanner, result::Bss},
        security::{
            self, Cipher, Integrity, KeyIdentity, KeySlot, PacketNumber, ReplaySeed,
            SecurityStatus, management as protection,
        },
        station::{
            Action, AssociationContext, Completion, Failure, Join, NativeBssObservation,
            Phase as Peering, RegulatoryPolicy, Station, Ticket,
            data::{self, Ingress, Protection, Qos, SequenceOwner, TxClass},
            external,
            multi_link::Affiliation,
            phy::{Capabilities, McsMaps, Role},
            wire::six_ghz::Selection,
        },
    },
};
use alloc::{collections::VecDeque, format, string::String, sync::Arc, vec::Vec};
use core::fmt;
use stela_driver_base::entropy::Entropy;

/// How long the radio may take from its start to carrier (M3 spec S1.5):
/// one scan's budget, `SAE_BUDGET_US` and `HANDSHAKE_TIMEOUT_US`, 28.98 s,
/// rounded up.
pub const WIFI_CARRIER_WAIT_US: u64 = 30_000_000;
/// `IEEE80211_SCAN_RESULT_EXPIRE`.
const BSS_EXPIRY_US: u64 = 30_000_000;
/// The data frames that wait for room, the oldest dropped past it.
const MAX_HELD: usize = 256;
/// The EAPOL frames whose release reports the progress line awaits.
const MAX_EAPOL_REPORTS: usize = 64;
/// What SAE gets in M3a (M3 Task 17).
const SAE_BEFORE_M3B: &str = "SAE before M3b";
/// `ieee80211_add_wmm_info_ie`, without U-APSD.
const WMM_ELEMENT: [u8; 9] = [221, 7, 0, 0x50, 0xf2, 2, 0, 1, 0];
/// `rtw89_init_ht_cap` (`core.c:6063-6090`) at 20 MHz with two streams
/// (spec S1.7): capability info 0x09AD (Linux's 0x19E3 without 20/40,
/// SGI 40 and DSSS/CCK 40, SM power save disabled), A-MPDU 64K without
/// density, MCS 0-15 without MCS 32, 144 Mb/s, TX MCS defined.
const HT_ELEMENT: [u8; 28] = [
    45, 26, 0xAD, 0x09, 0x03, 0xff, 0xff, 0, 0, 0, 0, 0, 0, 0, 0, 144, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0,
];
/// `rtw89_ext_capa_sta` (`core.c:208-212`), the station's extended
/// capabilities: extended channel switching, multiple BSSID, operating mode
/// notification.
const EXTENDED_CAPABILITIES: [u8; 10] = [127, 8, 0x04, 0, 0x40, 0, 0, 0, 0, 0x40];
/// `rtw89_channels_2ghz` and `rtw89_channels_5ghz` (`core.c:40-86`).
const CHANNELS_2GHZ: [u16; 14] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14];
const CHANNELS_5GHZ: [u16; 28] = [
    36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
    149, 153, 157, 161, 165, 169, 173, 177,
];
/// The Supported Channels a spectrum-managed request names (spec S14.4).
const SUPPORTED_2GHZ: [u8; 2] = [1, 14];
const SUPPORTED_5GHZ: [u8; 4] = [36, 28, 149, 9];

#[derive(Debug)]
pub enum Error {
    /// The chip's own error text, which names no address.
    Chip(String),
    Station(crate::wifi::station::Error),
    Data(data::Error),
    Key(security::Error),
    Scan(scan::Error),
    Refused(&'static str),
    Unsupported(&'static str),
    /// Frames must drain before this can go on.
    Pending,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

/// The keeper's handle on the IGTK the host would hold (M3b).
pub struct ManagementKey;

/// A BSS and when it was last heard, a `cfg80211_internal_bss`.
struct Heard {
    observation: Arc<NativeBssObservation>,
    at: u64,
}

/// A data frame waiting for room: its MPDU, the queue's TID, whether it is
/// EAPOL, and the peer's rates.
struct Held {
    bytes: Vec<u8>,
    tid: u8,
    eapol: bool,
    supported: u16,
}

/// What a management frame is to the progress line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Authentication,
    Association,
    Other,
}

impl Kind {
    fn of(bytes: &[u8]) -> Self {
        match bytes[0] & 0xfc {
            0xb0 => Self::Authentication,
            0x00 | 0x20 => Self::Association,
            _ => Self::Other,
        }
    }
}

/// Frames of one exchange the chip took and saw acknowledged, and the
/// AP's last answer.
#[derive(Clone, Copy, Debug, Default)]
struct Exchange {
    sent: u32,
    acked: u32,
    status: Option<u16>,
}

#[derive(Clone, Copy, Debug, Default)]
struct Eapol {
    received: u32,
    sent: u32,
    acked: u32,
}

/// The progress line's counts (spec S14.6), each `None` until reached.
#[derive(Clone, Debug, Default)]
struct Progress {
    scans: u32,
    auth: Option<Exchange>,
    assoc: Option<Exchange>,
    aid: Option<u16>,
    eapol: Option<Eapol>,
    keys: Option<u32>,
    last: Option<&'static str>,
}

fn bump(counter: &mut u32) {
    *counter = counter.saturating_add(1);
}

/// `-` for a field not reached yet.
fn or_dash<T: fmt::Display>(value: Option<T>) -> String {
    value.map_or_else(|| String::from("-"), |value| format!("{value}"))
}

fn exchange(exchange: Option<Exchange>) -> String {
    exchange.map_or_else(
        || String::from("-"),
        |e| format!("{}/{} status {}", e.sent, e.acked, or_dash(e.status)),
    )
}

/// The channels the country admits, which the real check gates at each frame.
struct Admitted<'a>(&'a [(Channel, u32)]);

impl RegulatoryPolicy for Admitted<'_> {
    fn allow_management_tx(&mut self, channel: Channel) -> bool {
        self.0.iter().any(|(known, _)| *known == channel)
    }
    fn allow_data_tx(&mut self, channel: Channel) -> bool {
        self.allow_management_tx(channel)
    }
}

impl scan::Policy for Admitted<'_> {
    fn allow_receive(&mut self, channel: Channel) -> bool {
        self.0.iter().any(|(known, _)| *known == channel)
    }
    fn allow_probe(&mut self, _: Channel) -> bool {
        false
    }
    fn lookup_channel(&mut self, band: Band, number: u16) -> Option<Channel> {
        self.0
            .iter()
            .map(|(channel, _)| *channel)
            .find(|channel| channel.band == band && channel.number == number)
    }
}

/// A management frame from `local` to `ap` (rtw88's `management`).
fn management(local: Mac, ap: Mac, subtype: u16, protected: bool, body: &[u8]) -> Vec<u8> {
    let mut bytes = alloc::vec![0; 24];
    let control = subtype | if protected { 0x4000 } else { 0 };
    bytes[..2].copy_from_slice(&control.to_le_bytes());
    bytes[4..10].copy_from_slice(&ap.0);
    bytes[10..16].copy_from_slice(&local.0);
    bytes[16..22].copy_from_slice(&ap.0);
    bytes.extend_from_slice(body);
    bytes
}

/// The rates of `rtw89_bitrates` (`core.c:153-166`) on `band`, as the
/// Supported and Extended Supported Rates elements.
fn rate_elements(band: Band) -> &'static [u8] {
    match band {
        Band::Ghz2 => &[1, 8, 2, 4, 11, 22, 12, 18, 24, 36, 50, 4, 48, 72, 96, 108],
        _ => &[1, 8, 12, 18, 24, 36, 48, 72, 96, 108],
    }
}

/// The channels mac80211 registers for rtw89, as `country` leaves them.
fn channels(country: &Country) -> Vec<(Channel, u32)> {
    let mut out = Vec::new();
    let mut band = |band: Band, numbers: &[u16], frequency: fn(u16) -> u16| {
        let centers: Vec<u16> = numbers.iter().map(|&number| frequency(number)).collect();
        for (&number, regulated) in numbers
            .iter()
            .zip(country.band(&centers).unwrap_or_default())
        {
            if regulated.flags & DISABLED == 0 {
                out.push((
                    Channel {
                        band,
                        number,
                        frequency_mhz: frequency(number),
                    },
                    regulated.flags,
                ));
            }
        }
    };
    band(Band::Ghz2, &CHANNELS_2GHZ, |number| {
        if number == 14 {
            2484
        } else {
            2407 + 5 * number
        }
    });
    band(Band::Ghz5, &CHANNELS_5GHZ, |number| 5000 + 5 * number);
    out
}

/// `context` as `BSS_CHANGED_ASSOC` finds it (spec S10): the AP, its AID
/// (`& 0x7ff`, as mac80211 keeps it), the channel joined, the BSS's beacon
/// interval and DTIM period, the peer's rates, its negotiated HT
/// (`ieee80211_ht_cap_ie_to_sta_ht_cap`), the AP's WMM parameters when it
/// has WMM (none otherwise: the chip programs mac80211's QoS defaults),
/// and the slot `ieee80211_handle_bss_capability` takes from the
/// response's capability, always short off 2.4 GHz.
fn association(context: &AssociationContext) -> Association {
    let negotiated = context.negotiated();
    let channel = context.channel();
    Association {
        bssid: context.bssid(),
        aid: context.response().aid & 0x7ff,
        channel,
        beacon_interval: context.beacon().beacon_interval,
        dtim_period: context.dtim_period(),
        legacy: negotiated.legacy.supported,
        ht: negotiated.ht.as_ref().map(|ht| HtPeer {
            rx_mask: [
                ht.peer_rx_mcs[0],
                ht.peer_rx_mcs[1],
                ht.peer_rx_mcs[2],
                ht.peer_rx_mcs[3],
            ],
            ldpc: ht.capability & 0x0001 != 0,
            rx_stbc: ht.capability & 0x0300 != 0,
            sgi20: ht.capability & 0x0020 != 0,
        }),
        edca: negotiated.wmm.as_ref().map(|wmm| wmm.access_categories),
        short_slot: channel.band != Band::Ghz2 || context.response().capabilities & (1 << 10) != 0,
    }
}

/// An RTL8852CE kept connected to one network.
pub type Wifi<C> = Keeper<Rtl8852c<C>>;

/// An RTL8852CE's association state over its chip.
pub struct Rtl8852c<C: Chip> {
    chip: C,
    address: Mac,
    clock: Clock,
    country: Country,
    channels: Vec<(Channel, u32)>,
    station: Station,
    scanner: Scanner,
    /// The scan the chip runs, until it ends.
    scanning: Option<scan::Ticket>,
    /// A scan ended and waits for the keeper.
    scanned: Option<scan::Ticket>,
    observed: Vec<Heard>,
    outbox: VecDeque<(Option<Ticket>, Vec<u8>)>,
    events: VecDeque<keeper::Event>,
    received: VecDeque<Vec<u8>>,
    held: VecDeque<Held>,
    admitted: Option<(Ticket, Arc<AssociationContext>)>,
    context: Option<Arc<AssociationContext>>,
    /// The BSS joined, as the keeper's profile of it.
    peer: Option<Peer>,
    /// The ticketed management frame whose report the station awaits.
    reporting: Option<(Token, Kind)>,
    /// The EAPOL frames the chip took, until their reports.
    eapol_reports: VecDeque<Token>,
    port_open: bool,
    departing: Option<Ticket>,
    left: bool,
    reported: Vec<&'static str>,
    /// The configured network's name, compared and never printed.
    ssid: Vec<u8>,
    progress: Progress,
}

impl<C: Chip> Rtl8852c<C> {
    /// `chip`, running at `address` under `country`, as a keeper's radio.
    pub fn new(chip: C, address: Mac, country: Country, clock: Clock) -> Result<Self, Error> {
        let channels = channels(&country);
        Ok(Self {
            chip,
            address,
            clock,
            country,
            channels,
            station: Station::new(address).map_err(Error::Station)?,
            scanner: Scanner::new(address).map_err(Error::Scan)?,
            scanning: None,
            scanned: None,
            observed: Vec::new(),
            outbox: VecDeque::new(),
            events: VecDeque::new(),
            received: VecDeque::new(),
            held: VecDeque::new(),
            admitted: None,
            context: None,
            peer: None,
            reporting: None,
            eapol_reports: VecDeque::new(),
            port_open: false,
            departing: None,
            left: false,
            reported: Vec::new(),
            ssid: Vec::new(),
            progress: Progress::default(),
        })
    }

    /// Keep `self` connected to `network`; `entropy` gives each handshake's
    /// SNonce.
    pub fn keep(mut self, network: Network, entropy: Arc<dyn Entropy>) -> Wifi<C> {
        let clock = self.clock;
        self.ssid = network.ssid.clone();
        Keeper::keep(self, network, entropy, clock)
    }

    /// The chip, as the radio left it.
    pub fn into_chip(self) -> C {
        self.chip
    }

    fn chip<T>(result: Result<T, String>) -> Result<T, Error> {
        result.map_err(Error::Chip)
    }

    /// `result`, its refusal kept as the progress line's `last`: what the
    /// radio itself refused the keeper.
    fn note<T>(&mut self, result: Result<T, Error>) -> Result<T, Error> {
        if let Err(Error::Refused(why) | Error::Unsupported(why)) = &result {
            self.progress.last = Some(why);
        }
        result
    }

    /// `operation` declined, reported once. A frame dropped is no refusal
    /// of the keeper's: the progress line's `last` stays.
    fn unsupported(&mut self, operation: &'static str) {
        if !self.reported.contains(&operation) {
            self.reported.push(operation);
            self.events.push_back(keeper::Event::Unsupported(operation));
        }
    }

    fn act(&mut self, action: Action) {
        match action {
            Action::Transmit { ticket, bytes, .. } => {
                self.outbox.push_back((Some(ticket), bytes));
            }
            Action::Configure {
                ticket,
                context: Some(context),
                ..
            } => self.admitted = Some((ticket, context)),
            // Every join here is a modern one, which carries its context.
            Action::Configure { .. } => {
                self.admitted = None;
                if let Ok(Action::Remove { ticket, .. }) = self.station.disconnect() {
                    self.depart(ticket);
                }
            }
            Action::Remove { ticket, .. } => self.depart(ticket),
        }
    }

    /// The station left: nothing it queued leaves after it.
    fn depart(&mut self, ticket: Ticket) {
        self.departing = Some(ticket);
        self.left = true;
        self.port_open = false;
        self.admitted = None;
        self.outbox.clear();
        self.held.clear();
    }

    /// The station's state gone after its retirement.
    fn clear(&mut self) {
        self.context = None;
        self.peer = None;
        self.admitted = None;
        self.departing = None;
        self.left = false;
        self.reporting = None;
        self.port_open = false;
        self.outbox.clear();
        self.held.clear();
    }

    fn observed(&self, handle: usize) -> Result<Arc<NativeBssObservation>, Error> {
        self.observed
            .get(handle)
            .map(|heard| heard.observation.clone())
            .ok_or(Error::Refused("the BSS is gone"))
    }

    /// The basic rates management frames leave at: the BSS's, once joined
    /// (`link->conf->basic_rates`, which mac80211 sets as it prepares the
    /// connection).
    fn basic_rates(&self) -> u16 {
        self.peer.map_or(0, |peer| peer.basic_rates)
    }

    /// rtw88's `send_management` (`station.rs:889`): the outbox, oldest
    /// first, while the chip has room; a ticketed frame's token is kept
    /// for its report.
    fn send_management(&mut self) -> Result<(), Error> {
        let basic_rates = self.basic_rates();
        while let Some((_, bytes)) = self.outbox.front() {
            // A protected frame needs the PTK the host would seal it under.
            if bytes[1] & 0x40 != 0 {
                self.outbox.pop_front();
                self.unsupported("a protected management frame before M3b");
                continue;
            }
            let frame = Management { bytes, basic_rates };
            let Some(token) = Self::chip(self.chip.transmit_management(&frame))? else {
                return Ok(());
            };
            let Some((ticket, bytes)) = self.outbox.pop_front() else {
                break;
            };
            let kind = Kind::of(&bytes);
            match kind {
                Kind::Authentication => bump(&mut self.progress.auth.get_or_insert_default().sent),
                Kind::Association => bump(&mut self.progress.assoc.get_or_insert_default().sent),
                Kind::Other => {}
            }
            if let Some(ticket) = ticket {
                self.reporting = Some((token, kind));
                self.station.submitted(ticket).map_err(Error::Station)?;
            }
        }
        Ok(())
    }

    /// rtw88's `send_data` (`station.rs:943`): `held`, oldest first, while
    /// the chip has room. Each EAPOL frame's token is kept for its report.
    fn send_data(&mut self) -> Result<(), Error> {
        while let Some(held) = self.held.front() {
            let frame = Data {
                bytes: &held.bytes,
                tid: held.tid,
                eapol: held.eapol,
                supported_rates: held.supported,
                key: None,
            };
            let Some(token) = Self::chip(self.chip.transmit_data(&frame))? else {
                return Ok(());
            };
            let eapol = held.eapol;
            self.held.pop_front();
            if eapol {
                bump(&mut self.progress.eapol.get_or_insert_default().sent);
                if self.eapol_reports.len() >= MAX_EAPOL_REPORTS {
                    self.eapol_reports.pop_front();
                }
                self.eapol_reports.push_back(token);
            }
        }
        Ok(())
    }

    /// rtw88's `send` (`station.rs:1121`, spec S13.1) without A-MSDU, TX
    /// Block Ack or keys: the MSDU as the station prepares it, held, then
    /// sent while the chip has room, as mac80211 hands a frame to the
    /// driver at once.
    fn send(&mut self, ethernet: &[u8], class: TxClass) -> Result<(), Error> {
        let epoch = self
            .station
            .association_epoch()
            .ok_or(Error::Refused("no association"))?;
        let context = self.context.clone().ok_or(Error::Refused("no context"))?;
        let tid = data::data_tid(&context, None, None, ethernet);
        let mut policy = Admitted(&self.channels);
        let mut keys: Vec<&mut KeySlot> = Vec::new();
        let plan = match tid {
            Some(tid) => self.station.prepare_qos_msdu_refs(
                epoch,
                ethernet,
                class,
                Qos {
                    context: &context,
                    tid,
                },
                &mut keys,
                &mut policy,
            ),
            None => self.station.prepare_msdu_refs(
                epoch,
                ethernet,
                class,
                SequenceOwner::Host,
                &mut keys,
                &mut policy,
            ),
        }
        .map_err(Error::Data)?;
        let (_, _, mpdu, protection, _, _) = plan.into_parts();
        match protection {
            Protection::None => {}
            Protection::HostIv { .. } => return Err(Error::Unsupported(KEYS_BEFORE_M3B)),
            Protection::Firmware { .. } => return Err(Error::Refused("a firmware key")),
        }
        if self.held.len() >= MAX_HELD {
            self.held.pop_front();
        }
        self.held.push_back(Held {
            bytes: mpdu,
            tid: tid.unwrap_or(0),
            eapol: class != TxClass::Data,
            supported: context.negotiated().legacy.supported,
        });
        self.send_data()
    }

    /// What the chip handed on since the last poll, every item taken; then
    /// the poll's own error, or the first an item gave.
    fn take_incoming(&mut self, now: u64) -> Result<(), Error> {
        let mut incoming = Vec::new();
        let mut first = Self::chip(self.chip.poll(&mut |item| incoming.push(item)));
        for item in incoming {
            let taken = match item {
                Incoming::Transmitted {
                    token,
                    acknowledged,
                } => self.transmitted(token, acknowledged, now),
                Incoming::Frame(received) => match Self::mpdu(received) {
                    Some(mpdu) => match self.frame(mpdu, now) {
                        // mac80211 drops a frame the station cannot use;
                        // the exchange it answers retries or times out.
                        Err(Error::Station(_)) => {
                            self.unsupported("a management frame the station refuses");
                            Ok(())
                        }
                        taken => taken,
                    },
                    None => Ok(()),
                },
                Incoming::ScanEnded { completed } => self.scan_ended(completed),
                // Beacon loss is M3 Task 18's; no chip reports it yet.
                Incoming::BeaconLost => Ok(()),
            };
            if first.is_ok() {
                first = taken;
            }
        }
        first
    }

    /// A frame's release report: the ticketed management frame's goes to
    /// the station as its TX status (rtw88's `Report::Transmitted`,
    /// `station.rs:1275-1288`, spec S1.10); an EAPOL frame's is counted.
    fn transmitted(&mut self, token: Token, acknowledged: bool, now: u64) -> Result<(), Error> {
        if let Some((_, kind)) = self.reporting.filter(|&(awaited, _)| awaited == token) {
            self.reporting = None;
            if acknowledged {
                let exchange = match kind {
                    Kind::Authentication => self.progress.auth.as_mut(),
                    Kind::Association => self.progress.assoc.as_mut(),
                    Kind::Other => None,
                };
                if let Some(exchange) = exchange {
                    bump(&mut exchange.acked);
                }
            }
            return self
                .station
                .management_status(acknowledged, now)
                .map_err(Error::Station);
        }
        if let Some(at) = self.eapol_reports.iter().position(|&sent| sent == token) {
            self.eapol_reports.remove(at);
            if acknowledged && let Some(eapol) = &mut self.progress.eapol {
                bump(&mut eapol.acked);
            }
        }
        Ok(())
    }

    /// The scan ended (spec S8.2): its ticket completed, the chip restored
    /// (`rtw89_hw_scan_complete`, spec S8.3), and the scan left for the
    /// keeper to retire, even when the restore failed, which powered the
    /// chip off.
    fn scan_ended(&mut self, completed: bool) -> Result<(), Error> {
        let Some(ticket) = self.scanning.take() else {
            return Ok(());
        };
        let outcome = if completed {
            Outcome::Completed
        } else {
            Outcome::Aborted
        };
        let recorded = self.scanner.completed(ticket, outcome).map_err(Error::Scan);
        let restored = Self::chip(self.chip.end_scan());
        self.scanned = Some(ticket);
        bump(&mut self.progress.scans);
        recorded.and(restored)
    }

    /// `received` as an MPDU on the channel the chip heard it on, its
    /// security status (spec S7.3) and FCS status with it.
    fn mpdu(received: Received) -> Option<Mpdu> {
        let channel = received.channel;
        let radio = RadioStatus {
            band: channel.band,
            channel: channel.number,
            frequency_mhz: channel.frequency_mhz,
            signal_dbm: received.signal_dbm,
            chain_signal_dbm: [None; 4],
            rate: RawRate {
                layout: 0,
                words: [0; 4],
            },
            rx_rate: None,
            tsf: Some(Timestamp::Low32(received.tsf)),
        };
        let security = SecurityStatus {
            decrypted: received.decrypted,
            integrity: if received.decrypted {
                Integrity::Verified
            } else {
                Integrity::Unknown
            },
            icv_error: received.icv_error,
            ..SecurityStatus::default()
        };
        let mut mpdu = Mpdu::new(received.bytes, radio, security).ok()?;
        mpdu.fcs_error = received.fcs_error;
        Some(mpdu)
    }

    /// rtw88's `frame` (`station.rs:1344`) without keys: a scan's beacons
    /// and probe responses to the scanner, management frames to
    /// [`Self::management_frame`], unprotected whole MSDUs to the station.
    /// A protected frame has no key to be taken under in M3a, and
    /// fragments and A-MSDUs are M3b's (spec S13.3).
    fn frame(&mut self, frame: Mpdu, now: u64) -> Result<(), Error> {
        let header = *frame.header();
        if let Some(ticket) = self.scanning
            && matches!(header.control & 0xfc, 0x80 | 0x50)
        {
            let mut policy = Admitted(&self.channels);
            if let Ok(bss) = self.scanner.observe(ticket, &frame, &mut policy) {
                self.hear(bss, now);
            }
            return Ok(());
        }
        if header.control & 12 == 0 {
            return self.management_frame(frame, now);
        }
        if header.control & 12 != 8
            || !header.data_present()
            || header.protected()
            || header.fragmented()
            || header.amsdu
        {
            return Ok(());
        }
        self.deliver(&frame);
        Ok(())
    }

    /// rtw88's `deliver` (`station.rs:1451`) of one MSDU: mac80211 drops
    /// what its port check refuses. EAPOL goes to the keeper.
    fn deliver(&mut self, frame: &Mpdu) {
        let Some(epoch) = self.station.association_epoch() else {
            return;
        };
        let mut keys: Vec<&mut KeySlot> = Vec::new();
        match self.station.receive_msdu_refs(epoch, frame, &mut keys) {
            Ok(Ingress::Data(ethernet)) => self.received.push_back(ethernet),
            Ok(Ingress::ControlPort { ethernet, .. }) => {
                bump(&mut self.progress.eapol.get_or_insert_default().received);
                self.events.push_back(keeper::Event::Eapol(ethernet));
            }
            Err(_) => {}
        }
    }

    fn hear(&mut self, mut bss: Bss, now: u64) {
        let known = self
            .observed
            .iter_mut()
            .find(|heard| heard.observation.bss().bssid == bss.bssid);
        match known {
            Some(heard) => {
                bss.keep_name_of(heard.observation.bss());
                heard.observation = Arc::new(NativeBssObservation { bss });
                heard.at = now;
            }
            None => self.observed.push(Heard {
                observation: Arc::new(NativeBssObservation { bss }),
                at: now,
            }),
        }
    }

    /// rtw88's `management_frame` (`station.rs:1514`) of M3a: what the
    /// station's authentication and association take, and the AP's
    /// departures. Beacons and probe responses outside a scan, action
    /// frames (Block Ack is M3 Task 16's; channel switches are not
    /// followed), protected frames, and, under PMF, robust frames to
    /// everyone (BIP is M3b's) are dropped, as is everything before a join.
    fn management_frame(&mut self, frame: Mpdu, now: u64) -> Result<(), Error> {
        let header = *frame.header();
        let subtype = header.control & 0xfc;
        if matches!(subtype, 0x80 | 0x50 | 0xd0) || header.protected() {
            return Ok(());
        }
        if self.station.phase() == Peering::ExternalAuthenticating {
            return Ok(());
        }
        let Some(epoch) = self.station.association_epoch() else {
            return Ok(());
        };
        // `ieee80211_drop_unencrypted_mgmt` of an MFP station.
        if self.station.requires_protected_management()
            && header.receiver.is_multicast()
            && protection::robust(&frame)
        {
            return Ok(());
        }
        let before = self.station.phase();
        let mut policy = Admitted(&self.channels);
        let action = self
            .station
            .receive(epoch, &frame, now, &mut policy)
            .map_err(Error::Station)?;
        let authenticated = self.answered(before, subtype, &action);
        self.act(action);
        if authenticated {
            Self::chip(self.chip.authenticated())?;
        }
        Ok(())
    }

    /// The AP's answer `action` came of, for the progress line: whether it
    /// accepted the authentication.
    fn answered(&mut self, before: Peering, subtype: u16, action: &Action) -> bool {
        let rejected = match action {
            Action::Remove {
                failure: Some(Failure::PeerRejected(status)),
                ..
            } => Some(*status),
            _ => None,
        };
        match (before, subtype) {
            (Peering::Authenticating, 0xb0) => {
                let accepted = self.station.phase() == Peering::Associating;
                let status = if accepted { Some(0) } else { rejected };
                if let (Some(status), Some(auth)) = (status, &mut self.progress.auth) {
                    auth.status = Some(status);
                }
                if accepted {
                    self.progress.assoc.get_or_insert_default();
                }
                accepted
            }
            (Peering::Associating, 0x10 | 0x30) => {
                let assoc = self.progress.assoc.get_or_insert_default();
                if let Action::Configure { response, .. } = action {
                    assoc.status = Some(response.status);
                    self.progress.aid = Some(response.aid & 0x7ff);
                } else if rejected.is_some() {
                    assoc.status = rejected;
                }
                false
            }
            _ => false,
        }
    }

    fn operate(&mut self) -> Result<(), Error> {
        let now = self.clock.now_us();
        let mut first = self.take_incoming(now);
        let mut policy = Admitted(&self.channels);
        if !self.left {
            match self.station.poll(now, &mut policy) {
                Ok(Some(action)) => self.act(action),
                Ok(None) => {}
                Err(error) => {
                    if first.is_ok() {
                        first = Err(Error::Station(error));
                    }
                }
            }
        }
        let sent = self.send_management();
        if first.is_ok() {
            first = sent;
        }
        let sent = self.send_data();
        if first.is_ok() {
            first = sent;
        }
        first
    }

    /// The progress line (spec S14.6), then the chip's counts.
    fn progress(&self) -> String {
        let p = &self.progress;
        let target = if p.scans == 0 {
            String::from("-")
        } else {
            self.observed
                .iter()
                .map(|heard| heard.observation.bss())
                .filter(|bss| bss.ssid.as_deref() == Some(&self.ssid[..]))
                .max_by_key(|bss| bss.signal_dbm.unwrap_or(i8::MIN))
                .map_or_else(
                    || String::from("missing"),
                    |bss| {
                        format!(
                            "found ch {} {} dBm",
                            bss.channel.number,
                            or_dash(bss.signal_dbm)
                        )
                    },
                )
        };
        let mut line = format!(
            "scans {} ({} networks, target {target}), auth {}",
            p.scans,
            self.observed.len(),
            exchange(p.auth),
        );
        match p.assoc {
            Some(_) => line.push_str(&format!(
                ", assoc {} aid {}",
                exchange(p.assoc),
                or_dash(p.aid)
            )),
            None => line.push_str(", assoc -"),
        }
        match p.eapol {
            Some(e) => line.push_str(&format!(
                ", eapol rx {} tx {} acked {}",
                e.received, e.sent, e.acked
            )),
            None => line.push_str(", eapol -"),
        }
        line.push_str(&format!(", keys {}", or_dash(p.keys)));
        line.push_str(if self.port_open {
            ", port open"
        } else {
            ", port closed"
        });
        if let Some(last) = p.last {
            line.push_str(&format!(", last \"{last}\""));
        }
        line.push_str("; ");
        line.push_str(&self.chip.diagnostic());
        line
    }

    fn begin_scan_with(&mut self, request: scan::Request, budget_us: u64) -> Result<(), Error> {
        let now = self.clock.now_us();
        self.observed
            .retain(|heard| now.saturating_sub(heard.at) < BSS_EXPIRY_US);
        let channels = request.channels().to_vec();
        let mut policy = Admitted(&self.channels);
        let ticket = self
            .scanner
            .begin(request, now, budget_us, &mut policy)
            .map_err(Error::Scan)?;
        let validated = self
            .scanner
            .validate_submission(ticket, now, &mut policy)
            .and_then(|()| self.scanner.submitted(ticket));
        let begun = match validated {
            Ok(()) => Self::chip(self.chip.begin_scan(&channels)),
            Err(error) => Err(Error::Scan(error)),
        };
        if let Err(error) = begun {
            // Nothing runs: the chip refused before its scan started, or
            // aborted and restored it (`rtw89_hw_scan_abort`).
            let _ = self.scanner.start_rejected(ticket, 0);
            let _ = self.scanner.retire(ticket, Retirement::Confirmed);
            return Err(error);
        }
        self.scanning = Some(ticket);
        Ok(())
    }

    fn join_with(&mut self, handle: usize, join: Join, peer: Peer) -> Result<(), Error> {
        let observed = self.observed(handle)?;
        let bss = observed.bss().clone();
        // `ieee80211_prep_connection`: the channel, then the BSSID.
        Self::chip(self.chip.tune(bss.channel))?;
        Self::chip(self.chip.set_bssid(bss.bssid))?;
        self.peer = Some(peer);
        self.progress.auth.get_or_insert_default();
        let now = self.clock.now_us();
        let mut policy = Admitted(&self.channels);
        let begun = if join.request.pmf().is_some() {
            self.station
                .begin_modern_pmf(join, observed, now, &mut policy)
        } else {
            self.station.begin_modern(join, bss, now, &mut policy)
        };
        match begun {
            Ok(action) => {
                self.act(action);
                Ok(())
            }
            Err(error) => {
                // No station began: the BSSID goes again, so a scan may run.
                self.peer = None;
                let _ = self.chip.disassociate();
                Err(Error::Station(error))
            }
        }
    }

    /// rtw88's `configure` (`station.rs:2113`) on rtw89 (spec S14.5): the
    /// chip's association on the join's channel, without a switch of the
    /// radio's own, then the station configured. RX Block Ack is M3 Task
    /// 16's.
    fn configure_admitted(&mut self) -> Result<(), Error> {
        let (ticket, context) = self
            .admitted
            .take()
            .ok_or(Error::Refused("configuration"))?;
        Self::chip(self.chip.associate(&association(&context)))?;
        self.context = Some(context);
        self.station
            .complete_configuration(ticket, Completion::Accepted, self.clock.now_us())
            .map_err(Error::Station)?;
        self.progress.eapol.get_or_insert_default();
        Ok(())
    }

    /// rtw88's `deauthenticate` (`station.rs:2296`): the deauthentication
    /// after what the outbox holds, then the station's departure, even when
    /// the chip refused the frame. Held data never leaves after it:
    /// mac80211 flushes with drop first (`ieee80211_set_disassoc`), and
    /// the departure drops what the chip has not taken.
    fn deauthenticate_with(&mut self, reason: u16) -> Result<(), Error> {
        if self.left || self.station.phase() == Peering::Idle {
            return Ok(());
        }
        if let Ok((local, ap)) = self.station.addresses() {
            let protected = self.station.protected_port_authorized();
            let bytes = management(local, ap, 0xc0, protected, &reason.to_le_bytes());
            self.outbox.push_back((None, bytes));
        }
        let sent = self.send_management();
        let action = self.station.disconnect().map_err(Error::Station)?;
        self.act(action);
        sent
    }

    /// rtw88's `retire_peer` (`station.rs:2311`), the chip first: its
    /// disassociation (spec S11) releases the channel, then the removal is
    /// accepted. A failed disassociation powered the chip off, which holds
    /// no association either.
    fn retire(&mut self) -> Result<(), Error> {
        let left = Self::chip(self.chip.disassociate());
        if let Some(ticket) = self.departing.take() {
            self.station
                .complete_removal(ticket, Completion::Accepted)
                .map_err(Error::Station)?;
        }
        self.clear();
        left
    }
}

impl<C: Chip> Link for Rtl8852c<C> {
    fn mac(&self) -> Mac {
        self.address
    }
    fn carrier(&mut self) -> bool {
        self.port_open
    }
    /// Spec S1.5, through the keeper.
    fn carrier_wait_us(&self) -> u64 {
        WIFI_CARRIER_WAIT_US
    }
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        if !self.port_open {
            return Err(LinkError::NoCarrier);
        }
        let sent = self.send(frame, TxClass::Data);
        self.note(sent)
            .map_err(|error| LinkError::Adapter(format!("{error}")))
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        Ok(self.received.pop_front())
    }
    fn diagnostic(&mut self) -> Option<String> {
        Some(self.progress())
    }
}

impl<C: Chip> Radio for Rtl8852c<C> {
    type Handle = usize;
    type Profile = Peer;
    type Management = Arc<ManagementKey>;
    type Error = Error;

    const NAME: &'static str = "rtw89";
    /// rtw89 leaves `max_listen_interval` at mac80211's default, as rtw88.
    const LISTEN_INTERVAL: u16 = 5;

    fn deferred(error: &Error) -> bool {
        matches!(error, Error::Pending)
    }

    fn service(&mut self) -> Result<(), Error> {
        let served = self.operate();
        self.note(served)
    }

    fn take_event(&mut self) -> Option<keeper::Event> {
        self.events.pop_front()
    }

    fn stage(&self) -> Stage {
        if self.left {
            Stage::NeedsDrain
        } else if self.scanned.is_some() {
            Stage::Scanned
        } else if self.admitted.is_some() {
            Stage::Admitted
        } else if self.scanning.is_none() && self.station.phase() == Peering::Idle {
            Stage::Idle
        } else {
            Stage::Busy
        }
    }

    fn station(&self) -> &Station {
        &self.station
    }

    fn channels(&self, band: Band) -> Vec<(Channel, u32)> {
        self.channels
            .iter()
            .filter(|(channel, _)| channel.band == band)
            .copied()
            .collect()
    }

    /// Never: the firmware has no probe template (spec S1.3).
    fn probes(&self, _: Channel) -> bool {
        false
    }

    fn supported_channels(&self, band: Band) -> &'static [u8] {
        match band {
            Band::Ghz2 => &SUPPORTED_2GHZ,
            Band::Ghz5 => &SUPPORTED_5GHZ,
            Band::Ghz6 => &[],
        }
    }

    fn regulated_power_dbm(&self, frequency_mhz: u16) -> i32 {
        self.country.channel(frequency_mhz).max_reg_dbm
    }

    /// HT 20 MHz with two streams, no VHT or HE (spec S1.7), WMM.
    fn advertisement(&self, band: Band) -> Result<Advertisement, Error> {
        let mut elements = rate_elements(band).to_vec();
        elements.extend_from_slice(&HT_ELEMENT);
        elements.extend_from_slice(&EXTENDED_CAPABILITIES);
        elements.extend_from_slice(&WMM_ELEMENT);
        let capabilities = Capabilities::parse(band, Role::Station, &elements)
            .map_err(|_| Error::Refused("advertisement"))?;
        Ok(Advertisement {
            capabilities,
            elements,
            he_source_maps: [McsMaps {
                rx: 0xffff,
                tx: 0xffff,
            }; 3],
            multi_link: Some(false),
        })
    }

    /// The BSS's profile, kept for its basic rates (spec S1.9).
    fn profile(&self, peer: Peer) -> Result<Peer, Error> {
        Ok(peer)
    }

    /// rtw88's `begin_scan` (`station.rs:1968`) as one firmware offload of
    /// the request's channels, which the chip bounds by its own budget.
    fn begin_scan(&mut self, request: scan::Request, budget_us: u64) -> Result<(), Error> {
        let begun = self.begin_scan_with(request, budget_us);
        self.note(begun)
    }

    /// The scan's end handed on every frame it heard, and the chip was
    /// restored: retirement is confirmed.
    unsafe fn retire_scan(&mut self) -> Result<(), Error> {
        let ticket = self.scanned.take().ok_or(Error::Refused("scan"));
        let ticket = self.note(ticket)?;
        self.scanner
            .retire(ticket, Retirement::Confirmed)
            .map_err(Error::Scan)?;
        Ok(())
    }

    fn six_ghz_scan(
        &self,
        _: &scan::Request,
        _: scan::six_ghz::Options,
    ) -> Result<Option<scan::Request>, Error> {
        Ok(None)
    }

    fn bss(&self) -> Vec<(usize, &Bss)> {
        self.observed
            .iter()
            .enumerate()
            .map(|(handle, heard)| (handle, heard.observation.bss()))
            .collect()
    }

    fn observation(&self, handle: usize) -> Result<Arc<NativeBssObservation>, Error> {
        self.observed(handle)
    }

    /// rtw88's `join` (`station.rs:2022`): the channel and BSSID, then
    /// authentication.
    fn join(&mut self, handle: usize, join: Join, peer: Peer) -> Result<(), Error> {
        let joined = self.join_with(handle, join, peer);
        self.note(joined)
    }

    fn begin_external_authentication(
        &mut self,
        _: usize,
        _: Peer,
        _: Option<Affiliation>,
        _: u64,
    ) -> Result<Arc<external::Session>, Error> {
        self.note(Err(Error::Unsupported(SAE_BEFORE_M3B)))
    }

    fn begin_six_ghz_external_authentication(
        &mut self,
        _: usize,
        _: Selection,
        _: Peer,
        _: Option<Affiliation>,
        _: u64,
    ) -> Result<Arc<external::Session>, Error> {
        self.note(Err(Error::Unsupported("6 GHz on an RTL8852C")))
    }

    fn transmit_external_authentication(
        &mut self,
        _: &external::Session,
        _: external::Frame<'_>,
    ) -> Result<(), Error> {
        self.note(Err(Error::Unsupported(SAE_BEFORE_M3B)))
    }

    fn associate_external(
        &mut self,
        _: &external::Session,
        _: usize,
        _: Join,
    ) -> Result<(), Error> {
        self.note(Err(Error::Unsupported(SAE_BEFORE_M3B)))
    }

    fn configure(&mut self) -> Result<(), Error> {
        let configured = self.configure_admitted();
        self.note(configured)
    }

    fn send_control(&mut self, ethernet: &[u8], key: Option<KeyIdentity>) -> Result<(), Error> {
        let class = key.map_or(TxClass::ControlPortPlaintext, TxClass::ControlPortProtected);
        let sent = self.send(ethernet, class);
        self.note(sent)
    }

    /// `rtw89_ops_set_key` takes CCMP-128 alone here (spec S1.2); in M3a
    /// none reaches the chip (spec S14.5).
    fn install_key(
        &mut self,
        _pairwise: bool,
        _index: u8,
        material: &[u8],
        _receive: ReplaySeed,
    ) -> Result<KeyIdentity, Error> {
        self.progress.keys.get_or_insert(0);
        let refused = if material.len() != Cipher::Ccmp128.key_length() {
            Error::Unsupported("a key other than CCMP-128 on an RTL8852C")
        } else {
            Error::Unsupported(KEYS_BEFORE_M3B)
        };
        self.note(Err(refused))
    }

    fn install_link_key(&mut self, _: u8, _: u8, _: &[u8], _: ReplaySeed) -> Result<(), Error> {
        self.note(Err(Error::Unsupported("MLO on an RTL8852C")))
    }

    /// The IGTK would stay on the host (spec S1.2); M3a has no PTK to
    /// hold it beside.
    unsafe fn install_management_key(
        &mut self,
        _: u64,
        _: u8,
        _: &[u8],
        _: PacketNumber,
    ) -> Result<Arc<ManagementKey>, Error> {
        self.note(Err(Error::Unsupported(KEYS_BEFORE_M3B)))
    }

    unsafe fn rotate_management_key(
        &mut self,
        _: &Arc<ManagementKey>,
        _: u8,
        _: &[u8],
        _: PacketNumber,
    ) -> Result<Arc<ManagementKey>, Error> {
        self.note(Err(Error::Unsupported(KEYS_BEFORE_M3B)))
    }

    /// The port opens on an installed PTK and GTK, which M3a never has.
    fn authorize_port(&mut self, _: u64) -> Result<(), Error> {
        self.note(Err(Error::Refused("no pairwise key")))
    }

    fn deauthenticate(&mut self, reason: u16) -> Result<(), Error> {
        let left = self.deauthenticate_with(reason);
        self.note(left)
    }

    unsafe fn retire_peer(&mut self) -> Result<(), Error> {
        let retired = self.retire();
        self.note(retired)
    }
}

#[cfg(test)]
mod tests;

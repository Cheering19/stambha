// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) AND GPL-2.0-only
// Copyright(c) 2018-2019 Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
// The duplicate check is adapted from pinned Linux net/mac80211/rx.c
// (ieee80211_rx_h_check_dup): Copyright 2002-2005, Instant802 Networks, Inc.;
// 2005-2006, Devicescape Software, Inc.; 2006-2007 Jiri Benc; 2007-2010
// Johannes Berg; 2013-2014 Intel Mobile Communications GmbH; 2015-2017 Intel
// Deutschland GmbH; 2018-2026 Intel Corporation. See NOTICE.
//! An RTL8852CE as the connection keeper's radio (M3 spec S14). rtw89
//! leaves the MAC's soft half to mac80211, as rtw88 does, so the station's
//! orchestration is Stela's, after rtw88's `realtek/station.rs` (its
//! `Rtl8822c`, cited per method); the hardware beneath it is
//! [`chip::Chip`], which a live chip implements. mac80211's and cfg80211's
//! functions are cited at the donor's `net/mac80211/` and `net/wireless/`
//! (Linux `a804c5742bff`).
//!
//! - A scan is one firmware offload (`rtw89_ops_hw_scan`, spec S8) of every
//!   channel the country leaves, each passive (no probe templates, spec
//!   S1.3); its end restores the chip (`rtw89_hw_scan_complete`) before
//!   the keeper retires it.
//! - A join tunes to the BSS and sets its BSSID (`ieee80211_prep_connection`,
//!   `mlme.c:9596`, over `rtw89_set_channel` and `BSS_CHANGED_BSSID`), then
//!   authenticates and associates through the shared `Station`, whose frames
//!   leave on the chip's management queue at the BSS's lowest basic rate,
//!   each ticketed one's TX status from its release report. The AP's
//!   acceptance of the authentication moves its station to AUTH
//!   ([`chip::Chip::authenticated`]).
//! - A WPA3-Personal network is joined by SAE as `ieee80211_mgd_auth`
//!   (`mlme.c:9898`) runs wpa_supplicant's exchange (spec S1.16): the channel
//!   and BSSID as a join's, then the keeper's commits and confirms, which the
//!   station builds, on the management queue, each one's TX status timing its
//!   next try (`ieee80211_sta_work`, `mlme.c:9097`), and the AP's handed back
//!   to the keeper. The AP's station goes to AUTH once the exchange ends
//!   authenticated, with the AP's confirm of the station's or the station's
//!   confirm to an AP that confirmed first (`ieee80211_mark_sta_auth`,
//!   `mlme.c:5386`); the association follows under the PMF SAE requires. Only
//!   the AP's authentication frames addressed to the station reach the
//!   exchange (`ieee80211_accept_frame`, `rx.c:4517`;
//!   `ieee80211_rx_mgmt_auth`, `mlme.c:5414`). A denied or timed-out exchange
//!   ends, the keeper told; a deauthentication does not end it, as
//!   `ieee80211_rx_mgmt_deauth` (`mlme.c:5646`) takes one only once
//!   associated or associating; and the keeper's own end of it leaves with a
//!   deauthentication (`ieee80211_mgd_deauth`, `mlme.c:10719`).
//! - The association programs the chip with the negotiated peer as
//!   `BSS_CHANGED_ASSOC` does (spec S10), on the channel the join switched
//!   to.
//! - EAPOL runs over the data queues; data frames wait in `held` while the
//!   chip has no room, management frames in the outbox.
//! - The handshake's keys go to the chip's security CAM (`rtw89_ops_set_key`,
//!   spec S12), CCMP-128 only (spec S1.2): the PTK with a host packet number,
//!   the GTK receive-only; a key of an index already held replaces it as
//!   `ieee80211_key_replace` (`key.c:440`) does, the old one deleted first,
//!   and an identical one is not installed again (`ieee80211_key_identical`,
//!   `key.c:813`). The port opens on both, and under PMF on the IGTK too,
//!   which stays on the host (`BipKey`, spec S1.2); a rekey while it is open
//!   authorizes it again on the keys then held, as mac80211's station stays
//!   authorized across one, but a replacement closes it from the old key's
//!   delete until the new key is installed, so one the chip refuses leaves it
//!   closed. rtw89 puts the IGTK in the CAM as BIP_CCMP128 (`cam.c:516-517`),
//!   but reads neither BIP bit of the RX descriptor (`AX_RXD_BIP_KEYID`,
//!   `AX_RXD_BIP_ENC`, `txrx.h:427-428`), and mac80211 checks a group robust
//!   frame's MIC itself unless the hardware marked it decrypted
//!   (`ieee80211_crypto_aes_cmac_decrypt`, `wpa.c:911`): here every one's is
//!   checked on the host.
//! - Data under the PTK leaves as `Protection::HostIv` plans it: the
//!   station reserves its PN and writes its CCMP header, which
//!   [`super::txd::strip_ccmp`] checks against the reservation and removes;
//!   the chip puts the PN in the TX descriptor and the hardware encrypts
//!   (`hw_sec_hdr`, spec S5.4). A reservation is spent whether the frame
//!   leaves, waits or is dropped, so PNs only increase. Robust management
//!   frames are sealed on the host under the PTK (`realtek::ccmp`), as
//!   rtw88 seals them, and leave without a hardware key.
//! - Received data the hardware decrypted (its CCMP header and MIC kept, spec
//!   S7.3) is taken under the key its key ID names, through the station's
//!   replay and port checks, fragments reassembled and A-MSDUs split as rtw88
//!   takes them; protected data the station cannot open is dropped where
//!   mac80211 decrypts, past the reorder buffer, so it still takes its place
//!   in its agreement's window. Protected management frames are opened on the
//!   host when the hardware left them sealed. A retried unicast frame of the
//!   AP's station repeating the last sequence control its counter saw is a
//!   duplicate, dropped (`ieee80211_rx_h_check_dup`, `rx.c:1521`). Frames
//!   heard after an accepted association response wait until the keeper
//!   configures the association, as mac80211 takes the response,
//!   `BSS_CHANGED_ASSOC` and all, before the next frame: an ADDBA Request or
//!   EAPOL in the same poll then finds the station associated.
//! - The AP's RX Block Ack agreements (spec S13.2) go through the station's
//!   Block Ack layer as mac80211's `agg-rx.c` runs them, with A-MSDUs in
//!   A-MPDUs and a window of at most `max_rx_agg_num`, 64: an agreement
//!   accepted gets its reorder buffer, then the chip's RX_START
//!   ([`chip::Chip::start_rx_block_ack`]), then its ADDBA Response, success
//!   whatever the chip returned, as `rtw89_ops_ampdu_action` returns 0 and
//!   the hardware's dynamic BA CAM covers an agreement without an entry. QoS
//!   data from the AP on its TID is reordered (`ieee80211_rx_reorder_ampdu`,
//!   `rx.c:1445`, the shared `Reorder`), a BAR releasing what waits before
//!   its SSN; a DELBA, a replacement or the departure stops it on the chip
//!   and drops what it held. Block Ack action frames are robust: under PMF
//!   they come and go protected.
//! - The station's TX Block Ack agreements (M4 spec S10) go through the
//!   station's `ba::tx` as rtw88 drives it, at rtw89's rules
//!   (`rtw89_core_txq_check_agg`, `core.c:4707`): the first QoS data on a
//!   TID without one asks the AP, VO included, under the pairwise key with
//!   PMF; the TID's data waits unnumbered while its agreement changes; one
//!   in force is the chip's TX_OPERATIONAL ([`chip::Chip::aggregation`]),
//!   and the TID's data goes in A-MPDUs of its window. A start that
//!   `ieee80211_start_tx_ba_session` refuses with -EINVAL blocks the TID for
//!   the association; EAPOL on an aggregating TID stops its agreement with a
//!   DELBA and forbids the TID for 4 s (`rtw89_core_stop_tx_ba_session`).
//! - A departure leaves through the chip (spec S11), which releases its
//!   channel. From an association, mac80211's `ieee80211_set_disassoc`
//!   (`mlme.c:4506`) flushes first ([`chip::Chip::flush`] with drop, then
//!   without it after a deauthentication of the station's own); the
//!   agreements stop on the chip without a DELBA, RX then TX
//!   (`ieee80211_sta_tear_down_BA_sessions`, `ht.c:286`); each key is deleted
//!   after a flush without drop, as `rtw89_ops_set_key`'s DISABLE_KEY
//!   flushes, the PTK first, then the group keys, before the chip
//!   disassociates.
//! - The firmware's beacon loss, which the chip reports while associated and
//!   not scanning (spec S15), loses the connection: the keeper hears of it
//!   and reconnects after `RECONNECT_US`, and the station departs as
//!   `ieee80211_sta_connection_lost` (`mlme.c:8937`) leaves an AP that
//!   stopped answering, the flush with drop and no frame of its own. The
//!   progress line counts each.
//!
//! The radio polls (spec S1.4): it never signals, and the keeper services
//! it from `carrier()` and `receive()`. Its diagnostic is the progress line
//! of spec S14.6, which names no address, then the chip's own counts. It
//! takes 30 s from its start to carrier ([`WIFI_CARRIER_WAIT_US`], spec
//! S1.5), which the keeper forwards.
//!
//! This module does not grant: any cipher but CCMP-128 (TKIP, GCMP, CCMP-256,
//! WEP), hardware-encrypted management frames (`hw_mgmt_tx_encrypt`), the
//! IGTK in the CAM, SAE on 6 GHz or on an MLD link, the probe of the AP
//! mac80211 sends on a beacon loss before it gives the AP up
//! (`ieee80211_mgd_probe_ap`, `mlme.c:4914`), A-MSDU TX, power
//! save, templates, the watchdog, SA Query requests of its own, or following
//! the AP's beacons and channel switches. Under PMF an unprotected
//! deauthentication or disassociation is dropped
//! (`ieee80211_drop_unencrypted_mgmt`, `rx.c:2568`), and the SA Query
//! wpa_supplicant then starts for one of reason 6 or 7, to learn whether the
//! AP still knows the station (`sme_event_unprot_disconnect`), is the shared
//! keeper's to send, which sends none for any radio: a station whose AP lost
//! its state keeps a link that carries nothing until the keeper leaves it. Of
//! RX Block Ack it does not grant: an agreement's inactivity timeout
//! (`sta_rx_agg_session_timer_expired`, `agg-rx.c:143`); the DELBA mac80211
//! sends for an aggregate on a TID without an agreement (`unexpected_agg`,
//! `rx.c:1472-1479`); ending an agreement for a fragment under it (fragments
//! are reassembled, never reordered, as rtw88 takes them); mac80211's reorder
//! timeout, as the shared `Reorder` releases after 100 ms on TIDs 0-3 and 40
//! ms on 4-7 where `HT_RX_REORDER_BUF_TIMEOUT` is 100 ms for all; and two
//! answers of mac80211 the station layer gives otherwise: an ADDBA Request
//! repeated under its agreement's dialog token without a MAC retry is
//! declined (mac80211 answers success), and one in an association without HT
//! gets no answer (mac80211 declines it).
//!
//! The GTK is deleted before the chip disassociates, where Linux
//! deletes a station's group key (a vif key in station mode, `cfg.c:617`'s
//! `ieee80211_add_key` without an address) only after cfg80211's
//! disconnect (`__cfg80211_disconnected`'s `rdev_del_key` loop,
//! `net/wireless/sme.c:1375-1391`); either way the chip ends keyless.
pub mod chip;

use self::chip::{Association, Chip, Data, HardwareKey, Incoming, Management, Received, Token};
use super::assoc::HtPeer;
use super::c2h::{RaReport, ra_rate};
use super::regd::unii4_allowed;
use super::rx::{Rate, RateMode};
use super::txd;
use crate::{
    Clock, Link, LinkError, Mac,
    wifi::{
        Band, Channel, RadioStatus, RawRate, Timestamp,
        connection::keeper::{self, Advertisement, Keeper, Peer, Radio, Stage},
        frame::Mpdu,
        mediatek::reorder::Reorder,
        network::Network,
        realtek::ccmp,
        regulatory::{channel::DISABLED, database::Country},
        scan::{self, Outcome, Retirement, Scanner, result::Bss},
        security::{
            self, Cipher, InstallOutcome, Integrity, KeyIdentity, KeyScope, KeySlot, PacketNumber,
            ReplaySeed, SecurityStatus,
            aggregate::{OrderedGroup, Shape},
            management::{self as protection, BipKey, ManagementCipher},
            tx::TransmitSeed,
        },
        station::{
            Action, AssociationContext, Completion, Failure, Join, NativeBssObservation,
            Phase as Peering, RegulatoryPolicy, Station, Ticket,
            ba::{self, tx},
            data::{self, Ingress, Protection, Qos, SequenceOwner, TxClass},
            external,
            fragments::Fragments,
            multi_link::Affiliation,
            operations::Operations,
            phy::{Capabilities, McsMaps, Role},
            wire::six_ghz::Selection,
        },
    },
};
use alloc::{
    boxed::Box,
    collections::{BTreeMap, VecDeque},
    format,
    string::String,
    sync::Arc,
    vec::Vec,
};
use core::fmt;
use stela_driver_base::entropy::Entropy;
use zeroize::Zeroizing;

/// How long the radio may take from its start to carrier (M3 spec S1.5):
/// one scan's budget, `SAE_BUDGET_US` and `HANDSHAKE_TIMEOUT_US`, 28.98 s,
/// rounded up.
pub const WIFI_CARRIER_WAIT_US: u64 = 30_000_000;
/// `IEEE80211_SCAN_RESULT_EXPIRE`.
const BSS_EXPIRY_US: u64 = 30_000_000;
/// The data frames that wait for room, the oldest dropped past it.
const MAX_HELD: usize = 256;
/// The EAPOL frames, and the ticketed management frames the station no
/// longer awaits, whose release reports the progress line still counts.
const MAX_EAPOL_REPORTS: usize = 64;
/// What a key the chip cannot take gets (spec S1.2).
const CCMP_128_ONLY: &str = "a key other than CCMP-128 on an RTL8852C";
/// `IEEE80211_CCMP_HDR_LEN`, and a management frame's header, before which
/// a sealed frame's CCMP header goes.
const CCMP_HEADER: usize = 8;
const MANAGEMENT_HEADER: usize = 24;
/// What an MLD link gets: the chip has one address, its address CAM's.
const NO_MLO: &str = "MLO on an RTL8852C";
/// 8852C's `max_rx_agg_num` (`rtw8852c.c:3157`), which rtw89 makes
/// `max_rx_aggregation_subframes` (`core.c:7483`): an agreement's largest
/// window.
const RECEIVE_WINDOW: u16 = 64;
/// A reorder clock of microseconds, as rtw88's.
const REORDER_TICKS_PER_SECOND: u32 = 1_000_000;
/// 8852C's `max_tx_agg_num` (`rtw8852c.c:3156`), which rtw89 makes
/// `max_tx_aggregation_subframes` (`core.c:7484`): a TX agreement's largest
/// window (M4 spec S10.1).
const TRANSMIT_WINDOW: u16 = 128;
/// `RTW89_FORBID_BA_TIMER` (`core.h:56`): how long EAPOL on an aggregating
/// TID keeps the TID from asking again (M4 spec S10.5).
const FORBID_BA_US: u64 = 4_000_000;
/// `WLAN_CATEGORY_BACK`.
const BLOCK_ACK_CATEGORY: u8 = 3;
/// The frames that wait behind an accepted association response, the
/// newest dropped past it.
const MAX_DEFERRED: usize = 64;
/// `sta->last_seq_ctrl`: one counter per TID, and one for management and
/// non-QoS data (`IEEE80211_NUM_TIDS + 1`).
const SEQUENCES: usize = 17;
/// `sta_info_alloc`'s `last_seq_ctrl`: the TIDs' at `USHRT_MAX`, the last
/// counter's zeroed.
const FRESH_SEQUENCES: [u16; SEQUENCES] = {
    let mut fresh = [u16::MAX; SEQUENCES];
    fresh[SEQUENCES - 1] = 0;
    fresh
};
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
/// `RTW89_5GHZ_UNII4_START_INDEX` (`core.h:6991`): channel 169's.
const UNII4_START_INDEX: usize = 25;
const _: () = assert!(CHANNELS_5GHZ[UNII4_START_INDEX] == 169);
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
    BlockAck(ba::Error),
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

/// The keeper's handle on the IGTK the host holds.
pub struct ManagementKey;

/// One key the chip's security CAM holds: its slot, kind, key ID, entry
/// and material (for the host's sealing and opening under the PTK).
struct Key {
    slot: KeySlot,
    pairwise: bool,
    index: u8,
    cam: u8,
    material: Zeroizing<Vec<u8>>,
}

/// Whether `a` and `b` are the same key material, compared in time that
/// does not depend on where they differ.
fn same_material(a: &[u8], b: &[u8]) -> bool {
    a.len() == b.len() && a.iter().zip(b).fold(0u8, |d, (a, b)| d | (a ^ b)) == 0
}

/// A BSS and when it was last heard, a `cfg80211_internal_bss`.
struct Heard {
    observation: Arc<NativeBssObservation>,
    at: u64,
}

/// A data frame waiting for room: its MPDU (without its CCMP header when
/// it goes under a key), the queue's TID (0 for non-QoS data) and whether
/// it is QoS data of that TID, whether it is EAPOL, the peer's rates, the
/// key it goes under: CAM entry, key ID and its reserved PN, and the A-MPDU
/// it goes in.
struct Held {
    bytes: Vec<u8>,
    tid: u8,
    qos: bool,
    eapol: bool,
    supported: u16,
    key: Option<(u8, u8, u64)>,
    aggregate: Option<txd::Aggregate>,
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

/// The data frames past the handshake's: the MSDUs the station took
/// (`delivered`), the protected frames dropped because the hardware left
/// them encrypted (`sealed`) or because no key holds their key ID
/// (`keyless`), and those the station's replay and port checks refused
/// (`refused`); then the frames of the host's own the chip took (`sent`).
/// EAPOL counts in [`Eapol`] instead.
#[derive(Clone, Copy, Debug, Default)]
struct Traffic {
    delivered: u32,
    sealed: u32,
    keyless: u32,
    refused: u32,
    sent: u32,
}

/// The progress line's counts (spec S14.6), each `None` until reached.
/// `auth` counts the SAE frames when the last join was by SAE (`sae`).
/// `data` counts once the first association is configured.
#[derive(Clone, Debug, Default)]
struct Progress {
    scans: u32,
    auth: Option<Exchange>,
    sae: bool,
    assoc: Option<Exchange>,
    aid: Option<u16>,
    eapol: Option<Eapol>,
    keys: Option<u32>,
    data: Option<Traffic>,
    beacon_lost: Option<u32>,
    last: Option<&'static str>,
    /// The ADDBA Requests the station sent, and those the AP answered with
    /// success (M4 spec S3.4).
    addba_sent: u32,
    addba_ok: u32,
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

/// The rates of the data frames the chip handed on addressed to the
/// station, each with the A-MPDU bit of its RX descriptor (M4 spec S3.1):
/// `counts` by rate, `ampdu` those that came in an A-MPDU, `total` all,
/// each saturating. The key space is the descriptor's, so the map stays
/// bounded.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RateHistogram {
    pub counts: BTreeMap<Rate, u32>,
    pub ampdu: u32,
    pub total: u32,
}

impl RateHistogram {
    pub fn record(&mut self, rate: Rate, ampdu: bool) {
        bump(self.counts.entry(rate).or_default());
        if ampdu {
            bump(&mut self.ampdu);
        }
        bump(&mut self.total);
    }

    /// The rate most frames came at, the highest of a tie, and its count.
    pub fn top(&self) -> Option<(Rate, u32)> {
        self.counts
            .iter()
            .max_by_key(|&(_, &count)| count)
            .map(|(&rate, &count)| (rate, count))
    }
}

/// `rate` as the progress line prints it: mode and width, streams, MCS (a
/// legacy rate's index) and a short GI.
fn rate_text(rate: &Rate) -> String {
    let mode = match rate.mode {
        RateMode::Legacy => "legacy",
        RateMode::Ht => "ht",
        RateMode::Vht => "vht",
        RateMode::He => "he",
    };
    let sgi = if rate.sgi { " sgi" } else { "" };
    format!(
        "{mode}{} {}ss mcs{}{sgi}",
        width_text(rate.bw),
        rate.nss,
        rate.mcs
    )
}

/// `enum rtw89_bandwidth`'s 20 to 160 MHz.
fn width_text(bw: u8) -> &'static str {
    match bw {
        0 => "20",
        1 => "40",
        2 => "80",
        3 => "160",
        _ => "?",
    }
}

/// `part` of `whole` in whole percent, rounded down.
fn percent(part: u32, whole: u32) -> u64 {
    u64::from(part) * 100 / u64::from(whole.max(1))
}

/// The joined AP's operation as its beacon states it (M4 spec S3.4): its
/// mode (`he` with an HE operation, else `vht` with a VHT operation on 5
/// GHz, else `ht` with an HT operation, else `legacy`), its width and
/// centre channel from the HT operation's secondary channel offset and the
/// VHT operation's width and centre segments (`ieee80211_chandef_vht_oper`,
/// read as the AP states it, not as the station joined), its primary
/// channel, and the extended NSS bandwidth support of its VHT capability
/// (bits 31:30) when it has one (M4 spec S7.2, SL-1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ApOperation {
    mode: &'static str,
    width: &'static str,
    primary: u16,
    centre: u16,
    ext_nss_bw: Option<u8>,
}

impl ApOperation {
    fn of(bss: &Bss) -> Self {
        let primary = bss.channel.number;
        let operations = Operations::parse(&bss.elements).ok();
        let ht = operations.as_ref().and_then(|o| o.ht.as_ref());
        let vht = operations
            .as_ref()
            .and_then(|o| o.vht.as_ref())
            .filter(|_| bss.channel.band == Band::Ghz5);
        let he = operations.as_ref().is_some_and(|o| o.he.is_some());
        let mode = match (he, vht.is_some(), ht.is_some()) {
            (true, _, _) => "he",
            (false, true, _) => "vht",
            (false, false, true) => "ht",
            _ => "legacy",
        };
        let (mut width, mut centre) = ("20", primary);
        match ht.map(|ht| ht.parameter & 3) {
            Some(1) => (width, centre) = ("40", primary + 2),
            Some(3) => (width, centre) = ("40", primary.saturating_sub(2)),
            _ => {}
        }
        if let Some(vht) = vht {
            let (c0, c1) = (u16::from(vht.center0), u16::from(vht.center1));
            match vht.width {
                1 if c1 != 0 && c0.abs_diff(c1) == 8 => (width, centre) = ("160", c1),
                1 if c1 != 0 && c0.abs_diff(c1) > 8 => (width, centre) = ("80p80", c0),
                1 => (width, centre) = ("80", c0),
                2 => (width, centre) = ("160", c0),
                3 => (width, centre) = ("80p80", c0),
                _ => {}
            }
        }
        // The VHT Capabilities element (191) and its capability's bits
        // 31:30.
        let mut ext_nss_bw = None;
        let mut rest = &bss.elements[..];
        while let [id, len, tail @ ..] = rest {
            let Some(body) = tail.get(..usize::from(*len)) else {
                break;
            };
            if *id == 191
                && let Some(cap) = body.get(..4)
            {
                ext_nss_bw = Some(cap[3] >> 6);
            }
            rest = &tail[usize::from(*len)..];
        }
        Self {
            mode,
            width,
            primary,
            centre,
            ext_nss_bw,
        }
    }
}

impl fmt::Display for ApOperation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}{} ch {} (c {})",
            self.mode, self.width, self.primary, self.centre
        )?;
        if let Some(bits) = self.ext_nss_bw {
            write!(f, " ext nss bw {bits}")?;
        }
        Ok(())
    }
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

/// The channels mac80211 registers for rtw89, as `country` leaves them,
/// then UNII-4 (169, 173 and 177, from `RTW89_5GHZ_UNII4_START_INDEX`,
/// `core.h:6991-6992`) dropped unless the regd `alpha2` names may use it
/// ([`unii4_allowed`], M4 spec S14.2): `rtw89_regd_apply_policy_unii4`
/// disables them from the regd notifier, after cfg80211's own channel and
/// HT40 flags (`wiphy_update_regulatory`'s `reg_call_notifier` comes last),
/// so the channels below keep the flags they had with UNII-4 present.
fn channels(country: &Country, alpha2: [u8; 2]) -> Vec<(Channel, u32)> {
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
    if !unii4_allowed(alpha2) {
        let unii4 = &CHANNELS_5GHZ[UNII4_START_INDEX..];
        out.retain(|(channel, _)| !(channel.band == Band::Ghz5 && unii4.contains(&channel.number)));
    }
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
    /// Data kept from a TID whose TX agreement is changing, as
    /// `ieee80211_agg_stop_txq` keeps it in the TXQ.
    parked: VecDeque<(u8, Vec<u8>, TxClass)>,
    /// What TX agreements have yet to do, in turn.
    changes: VecDeque<tx::Action>,
    /// Each TID's TX agreement in force, `RTW89_TXQ_F_AMPDU`.
    aggregating: [Option<Arc<tx::Session>>; 8],
    /// The TIDs an agreement cannot start on in this association,
    /// `RTW89_TXQ_F_BLOCK_BA`.
    blocked: [bool; 8],
    /// The TIDs EAPOL stopped an agreement on, `RTW89_TXQ_F_FORBID_BA`,
    /// until `forbid_ends`: one timer for all of them, as rtw89's
    /// `forbid_ba_work` is one delayed work restarted on each stop.
    forbidden: [bool; 8],
    forbid_ends: Option<u64>,
    admitted: Option<(Ticket, Arc<AssociationContext>)>,
    context: Option<Arc<AssociationContext>>,
    /// The BSS joined, as the keeper's profile of it.
    peer: Option<Peer>,
    /// The ticketed management frame whose report the station awaits.
    reporting: Option<(Token, Kind)>,
    /// Ticketed management frames the chip took before the one the station
    /// awaits, until their reports: counted, not the station's.
    earlier: VecDeque<(Token, Kind)>,
    /// The EAPOL frames the chip took, until their reports.
    eapol_reports: VecDeque<Token>,
    /// The keys the chip's security CAM holds.
    keys: Vec<Key>,
    /// The next `KeySlot` number: each key gets its own.
    next_slot: u16,
    /// The join's management cipher, and the IGTK the host checks group
    /// frames under.
    management_cipher: Option<ManagementCipher>,
    integrity: Option<BipKey>,
    fragments: Fragments,
    /// The frames each TID's RX Block Ack agreement holds in order.
    agreements: [Option<Reorder<Mpdu>>; 8],
    /// Frames heard after an accepted association response, until the
    /// keeper configures the association.
    deferred: VecDeque<Mpdu>,
    /// The AP's station, from the join until the station's retirement, and
    /// its `last_seq_ctrl`.
    ap: Option<Mac>,
    last_sequence: [u16; SEQUENCES],
    port_open: bool,
    /// The keeper authorized the port in this association; a replaced
    /// key's delete closes it until a key is installed again.
    authorized: bool,
    departing: Option<Ticket>,
    left: bool,
    reported: Vec<&'static str>,
    /// The configured network's name, compared and never printed.
    ssid: Vec<u8>,
    progress: Progress,
    /// The rates the station's data came at, and the last RA report of the
    /// firmware's for the AP, across every association (M4 spec S3.1,
    /// S3.3).
    rx_rates: RateHistogram,
    tx_rate: Option<RaReport>,
    /// The last BSS joined's operation.
    ap_operation: Option<ApOperation>,
}

impl<C: Chip> Rtl8852c<C> {
    /// `chip`, running at `address` under `country`, as a keeper's radio.
    pub fn new(chip: C, address: Mac, country: Country, clock: Clock) -> Result<Self, Error> {
        let alpha2 = chip.link_counts().regd.map_or(*b"00", |(alpha2, _)| alpha2);
        let channels = channels(&country, alpha2);
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
            parked: VecDeque::new(),
            changes: VecDeque::new(),
            aggregating: Default::default(),
            blocked: [false; 8],
            forbidden: [false; 8],
            forbid_ends: None,
            admitted: None,
            context: None,
            peer: None,
            reporting: None,
            earlier: VecDeque::new(),
            eapol_reports: VecDeque::new(),
            keys: Vec::new(),
            next_slot: 0,
            management_cipher: None,
            integrity: None,
            fragments: Fragments::default(),
            agreements: Default::default(),
            deferred: VecDeque::new(),
            ap: None,
            last_sequence: FRESH_SEQUENCES,
            port_open: false,
            authorized: false,
            departing: None,
            left: false,
            reported: Vec::new(),
            ssid: Vec::new(),
            progress: Progress::default(),
            rx_rates: RateHistogram::default(),
            tx_rate: None,
            ap_operation: None,
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

    /// The rates of the data the chip handed on addressed to the station
    /// (M4 spec S3.1).
    pub fn rx_rates(&self) -> &RateHistogram {
        &self.rx_rates
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
            Action::Transmit { ticket, bytes, .. } => self.queue_management(Some(ticket), bytes),
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
            Action::Remove { ticket, failure } => {
                // An SAE exchange the AP denied or that timed out, which
                // mac80211 destroys and cfg80211 reports to wpa_supplicant
                // (rtw88's `depart`).
                if let Some(failure) = failure
                    && self.station.external_authentication().is_some()
                {
                    self.events
                        .push_back(keeper::Event::SaeEnded(format!("{failure:?}")));
                }
                self.depart(ticket);
            }
        }
    }

    /// `bytes` to the outbox, a protected frame sealed first: mac80211
    /// encrypts a frame as it enters its TX path. One that cannot be sealed
    /// (no PTK) is dropped, reported once.
    fn queue_management(&mut self, ticket: Option<Ticket>, bytes: Vec<u8>) {
        if bytes.get(1).is_some_and(|fc| fc & 0x40 != 0) {
            match self.seal(&bytes) {
                Ok(sealed) => self.outbox.push_back((ticket, sealed)),
                Err(_) => self.unsupported("a protected management frame without its key"),
            }
        } else {
            self.outbox.push_back((ticket, bytes));
        }
    }

    /// The station left of the AP's doing or its own: an association's
    /// `ieee80211_set_disassoc` without a frame of its own flushes with
    /// drop first, its failure passed over as Linux's is.
    fn depart(&mut self, ticket: Ticket) {
        if self.context.is_some() {
            let _ = self.chip.flush(true);
        }
        self.leave(ticket);
    }

    /// The station left: nothing it queued leaves after it.
    fn leave(&mut self, ticket: Ticket) {
        self.departing = Some(ticket);
        self.left = true;
        self.port_open = false;
        self.admitted = None;
        self.outbox.clear();
        self.held.clear();
        self.parked.clear();
        self.deferred.clear();
    }

    /// The station's state gone after its retirement, the keys with it. A
    /// report still awaited is counted when it comes.
    fn clear(&mut self) {
        self.context = None;
        self.peer = None;
        self.admitted = None;
        self.departing = None;
        self.left = false;
        if let Some(report) = self.reporting.take() {
            self.keep_earlier(report);
        }
        self.port_open = false;
        self.authorized = false;
        self.outbox.clear();
        self.held.clear();
        self.parked.clear();
        // `rtw89_core_free_sta_pending_ba` and `_forbid_ba`
        // (`mac80211.c:619-620`): the AP's station takes its TXQ flags with it.
        self.changes.clear();
        self.aggregating = Default::default();
        self.blocked = [false; 8];
        self.forbidden = [false; 8];
        self.forbid_ends = None;
        self.keys.clear();
        self.management_cipher = None;
        self.integrity = None;
        self.fragments.clear();
        for tid in 0..self.agreements.len() {
            self.close_agreement(tid);
        }
        self.deferred.clear();
        self.ap = None;
        self.last_sequence = FRESH_SEQUENCES;
    }

    /// A report the station no longer awaits, kept for the progress line.
    fn keep_earlier(&mut self, report: (Token, Kind)) {
        if self.earlier.len() >= MAX_EAPOL_REPORTS {
            self.earlier.pop_front();
        }
        self.earlier.push_back(report);
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
    /// for its report, the token it replaces among the earlier ones, and
    /// so is an SAE frame's, the outbox's only authentication without a
    /// ticket: mac80211 sends the exchange's frames with
    /// `IEEE80211_TX_CTL_REQ_TX_STATUS`, rtw89 reporting TX status
    /// (`REPORTS_TX_ACK_STATUS`, `core.c:7495`), and the status times the
    /// exchange's next try (`ieee80211_mgd_conn_tx_status`). A protected
    /// frame was sealed as it was queued. A frame the chip refuses (a chip
    /// off, or one the encoder or the submit refuses) stays at the head and
    /// its error is returned, as rtw88's does, so the frames behind it wait
    /// until a departure clears the outbox; rtw89 frees just that frame
    /// (`ieee80211_free_txskb` after `rtw89_core_tx_write` fails,
    /// `mac80211.c:39-43`).
    fn send_management(&mut self) -> Result<(), Error> {
        let basic_rates = self.basic_rates();
        while let Some((_, bytes)) = self.outbox.front() {
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
            if (ticket.is_some() || kind == Kind::Authentication)
                && let Some(earlier) = self.reporting.replace((token, kind))
            {
                self.keep_earlier(earlier);
            }
            if let Some(ticket) = ticket {
                self.station.submitted(ticket).map_err(Error::Station)?;
            }
        }
        Ok(())
    }

    /// rtw88's `send_data` (`station.rs:943`): `held`, oldest first, while
    /// the chip has room, each written to the chip and left for the service
    /// pass's kick (M4 spec S9.1). Each EAPOL frame's token is kept for its
    /// report; an A-MPDU's frame is published to its agreement, which
    /// restarts the agreement's inactivity timeout.
    fn send_data(&mut self) -> Result<(), Error> {
        while let Some(held) = self.held.front() {
            let frame = Data {
                bytes: &held.bytes,
                tid: held.tid,
                eapol: held.eapol,
                supported_rates: held.supported,
                key: held.key,
                aggregate: held.aggregate,
            };
            let Some(token) = Self::chip(self.chip.transmit_data(&frame))? else {
                return Ok(());
            };
            let eapol = held.eapol;
            let session = held.aggregate.and(
                self.aggregating
                    .get(usize::from(held.tid))
                    .cloned()
                    .flatten(),
            );
            self.held.pop_front();
            if let Some(session) = session {
                let now = self.clock.now_us();
                // SAFETY: this frame just went to the chip under `session`.
                // A stop under way has revoked it, and that refusal is right.
                let _ = unsafe { self.station.tx_block_ack_published(&session, now) };
            }
            if eapol {
                bump(&mut self.progress.eapol.get_or_insert_default().sent);
                if self.eapol_reports.len() >= MAX_EAPOL_REPORTS {
                    self.eapol_reports.pop_front();
                }
                self.eapol_reports.push_back(token);
            } else {
                self.count_data(|data| &mut data.sent);
            }
        }
        Ok(())
    }

    /// rtw88's `seal` (`station.rs:918`), `IEEE80211_KEY_FLAG_SW_MGMT_TX`'s
    /// host half: `bytes`, a robust management frame, sealed under the PTK
    /// at the next PN of its slot, which data shares, as one key's
    /// `tx_pn` counts both in Linux.
    fn seal(&mut self, bytes: &[u8]) -> Result<Vec<u8>, Error> {
        let key = self
            .keys
            .iter_mut()
            .find(|key| key.pairwise)
            .ok_or(Error::Refused("no pairwise key"))?;
        let identity = key.slot.installed().ok_or(Error::Refused("no key"))?;
        if bytes.len() < MANAGEMENT_HEADER {
            return Err(Error::Refused("a management frame shorter than its header"));
        }
        let nonce = key
            .slot
            .reserve_host_tx_nonce(identity)
            .map_err(Error::Key)?;
        let pn = nonce.packet_number();
        let header: [u8; CCMP_HEADER] = nonce.into_header();
        let material: &[u8; 16] = key.material[..]
            .try_into()
            .map_err(|_| Error::Unsupported(CCMP_128_ONLY))?;
        Ok(ccmp::seal(
            material,
            &bytes[..MANAGEMENT_HEADER],
            &header,
            pn,
            &bytes[MANAGEMENT_HEADER..],
        ))
    }

    /// rtw88's `send` (`station.rs:1121`, spec S13.1) without A-MSDU: data
    /// of a TID whose TX agreement is changing parked, unnumbered, as
    /// `ieee80211_agg_stop_txq` keeps it (M4 spec S10.1); otherwise the MSDU
    /// as the station prepares it under the keys held, a `HostIv` frame's
    /// CCMP header checked against its reservation and removed (spec S5.4),
    /// in an A-MPDU when its TID aggregates, held, then written to the chip
    /// while it has room, as mac80211 hands a frame to the driver at once;
    /// the service pass's kick sends it. `rtw89_core_txq_check_agg` then
    /// weighs the TID's agreement (M4 spec S10.4, S10.5). A frame the strip
    /// refuses is dropped, its PN spent.
    fn send(&mut self, ethernet: &[u8], class: TxClass) -> Result<(), Error> {
        let epoch = self
            .station
            .association_epoch()
            .ok_or(Error::Refused("no association"))?;
        let context = self.context.clone().ok_or(Error::Refused("no context"))?;
        let tid = data::data_tid(&context, None, None, ethernet);
        if let Some(tid) = tid
            && self.station.tx_block_ack_blocks_data(tid)
        {
            if self.parked.len() >= MAX_HELD {
                self.parked.pop_front();
            }
            self.parked.push_back((tid, ethernet.to_vec(), class));
            return Ok(());
        }
        let mut policy = Admitted(&self.channels);
        let mut keys: Vec<&mut KeySlot> = self.keys.iter_mut().map(|key| &mut key.slot).collect();
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
        let (_, _, mut mpdu, protection, _, _) = plan.into_parts();
        let key = match protection {
            Protection::None => None,
            Protection::HostIv { key, pn } => {
                let cam = self
                    .keys
                    .iter()
                    .find(|held| held.slot.installed() == Some(key))
                    .map(|held| held.cam)
                    .ok_or(Error::Refused("a key the chip does not hold"))?;
                txd::strip_ccmp(&mut mpdu, pn.value(), key.index())
                    .map_err(|error| Error::Chip(format!("{error}")))?;
                Some((cam, key.index(), pn.value()))
            }
            Protection::Firmware { .. } => return Err(Error::Refused("a firmware key")),
        };
        let eapol = class != TxClass::Data;
        let aggregate = tid
            .filter(|_| !eapol)
            .and_then(|tid| self.aggregate(&context, tid));
        if self.held.len() >= MAX_HELD {
            self.held.pop_front();
        }
        self.held.push_back(Held {
            bytes: mpdu,
            tid: tid.unwrap_or(0),
            qos: tid.is_some(),
            eapol,
            supported: context.negotiated().legacy.supported,
            key,
            aggregate,
        });
        if let Some(tid) = tid {
            self.check_aggregation(tid, eapol);
        }
        self.send_data()
    }

    /// `rtw89_core_txq_check_agg`'s `IEEE80211_TX_CTL_AMPDU` and
    /// `rtw89_core_tx_update_ampdu_info` (`core.c:636-675`, M4 spec S10.3):
    /// data on `tid` goes in an A-MPDU while its agreement is operational and
    /// the TID is not forbidden, of the agreement's window at the AP's HT
    /// A-MPDU length exponent and MPDU spacing.
    fn aggregate(&self, context: &AssociationContext, tid: u8) -> Option<txd::Aggregate> {
        let index = usize::from(tid);
        if *self.forbidden.get(index)? {
            return None;
        }
        let session = self.aggregating.get(index)?.as_ref()?;
        let agreement = self.station.validate_tx_block_ack(session).ok()?;
        let ht = context.negotiated().ht.as_ref()?;
        Some(txd::Aggregate {
            agg_num: agreement.window(),
            factor: ht.peer_ampdu_base_exponent,
            density: ht.peer_mpdu_spacing,
        })
    }

    /// `rtw89_core_txq_check_agg` (`core.c:4707-4741`) of a frame just
    /// queued on `tid`, then `rtw89_core_ba_work` (`:4601-4637`): nothing
    /// while the TID is forbidden; EAPOL stops an operational agreement
    /// ([`Self::forbid`]); otherwise a TID neither blocked nor aggregating
    /// starts one, unless the station is leaving (`rtwsta->disassoc`). Under
    /// PMF the ADDBA goes under the pairwise key, so none starts before it
    /// (rtw88's `start_aggregation`). There is no VO exclusion: rtw88 has one,
    /// rtw89 none. `ieee80211_start_tx_ba_session`'s -EINVAL
    /// (`Unsupported`, `InvalidLimits`) blocks the TID for the association;
    /// its other refusals leave the next frame to ask again.
    fn check_aggregation(&mut self, tid: u8, eapol: bool) {
        let index = usize::from(tid);
        if self.forbidden.get(index).is_none_or(|&forbidden| forbidden) {
            return;
        }
        if eapol {
            self.forbid(tid);
            return;
        }
        if self.blocked[index] || self.aggregating[index].is_some() || self.left {
            return;
        }
        let now = self.clock.now_us();
        let started = if !self.station.requires_protected_management() {
            self.station.begin_tx_block_ack(tid, 0, now)
        } else if self.keys.iter().any(|key| key.pairwise) {
            self.station.begin_authenticated_tx_block_ack(tid, 0, now)
        } else {
            return;
        };
        match started {
            Ok(action) => self.changes.push_back(action),
            Err(ba::Error::Unsupported | ba::Error::InvalidLimits) => self.blocked[index] = true,
            Err(_) => {}
        }
    }

    /// `rtw89_core_stop_tx_ba_session` (`core.c:4682-4705`, M4 spec S10.5):
    /// EAPOL on a TID whose agreement is operational (`RTW89_TXQ_F_AMPDU`)
    /// forbids the TID and stops the agreement with a DELBA
    /// (`ieee80211_stop_tx_ba_session`); the one forbid timer restarts
    /// (`cancel_delayed_work`, then `RTW89_FORBID_BA_TIMER` again), and its
    /// end clears every TID it forbade (`rtw89_forbid_ba_work`,
    /// `:4900-4912`).
    fn forbid(&mut self, tid: u8) {
        let index = usize::from(tid);
        if self.left || self.aggregating[index].is_none() {
            return;
        }
        self.forbidden[index] = true;
        if let Ok(action) = self.station.stop_tx_block_ack(tid, true) {
            self.changes.push_back(action);
        }
        self.forbid_ends = self.clock.now_us().checked_add(FORBID_BA_US);
    }

    /// mac80211's aggregation work over `rtw89_ops_ampdu_action`
    /// (`mac80211.c:1026-1075`, M4 spec S10.2) after rtw88's
    /// `change_agreements` (`station.rs:992`), each action in turn:
    /// - Prepare waits while QoS data of its TID the station numbered is
    ///   held (non-QoS data on the same queue is no frame of the TID's), as
    ///   mac80211 numbers a frame as it leaves the TXQ, then gives the
    ///   TID's next sequence number (TX_START: IMMEDIATE, no I/O).
    /// - A frame goes to the chip at once ([`Self::send_agreement_frame`]),
    ///   waiting at the head for room; one the station no longer sends, or
    ///   one the chip refuses, is rejected.
    /// - Programming is [`Chip::aggregation`]: TX_OPERATIONAL with the
    ///   agreement's window, or TX_STOP; a stop takes the TID's A-MPDUs off
    ///   what is held. rtw89 returns 0 whatever it did, so the agreement goes
    ///   on, and the chip's error is returned after the rest.
    /// - Operational puts the agreement in force and counts it.
    fn change_agreements(&mut self) -> Result<(), Error> {
        let mut first = Ok(());
        while let Some(action) = self.changes.pop_front() {
            let now = self.clock.now_us();
            let next = match action {
                tx::Action::Prepare { ticket, session } => {
                    let tid = session.tid();
                    if self.held.iter().any(|held| held.qos && held.tid == tid) {
                        self.changes
                            .push_front(tx::Action::Prepare { ticket, session });
                        return first;
                    }
                    let sequence = self
                        .context
                        .as_ref()
                        .and_then(|context| self.station.next_qos_sequence(context, tid).ok());
                    // SAFETY: new data of this TID waits in `parked`, none of
                    // it is held, and `sequence` is the station's own counter.
                    unsafe {
                        self.station.complete_tx_block_ack_prepare(
                            ticket,
                            Completion::Accepted,
                            sequence,
                            now,
                        )
                    }
                }
                tx::Action::Transmit {
                    ticket,
                    session,
                    bytes,
                } => {
                    let result = if self.left
                        || self
                            .station
                            .validate_tx_block_ack_transmit(ticket, &session, &bytes)
                            .is_err()
                    {
                        Completion::RejectedAndIdle
                    } else {
                        match self.send_agreement_frame(&bytes) {
                            Ok(true) => Completion::Accepted,
                            Ok(false) => {
                                self.changes.push_front(tx::Action::Transmit {
                                    ticket,
                                    session,
                                    bytes,
                                });
                                return first;
                            }
                            Err(error) => {
                                if first.is_ok() {
                                    first = Err(error);
                                }
                                Completion::RejectedAndIdle
                            }
                        }
                    };
                    // SAFETY: the frame reached the chip, or never will.
                    unsafe {
                        self.station
                            .complete_tx_block_ack_transmit(ticket, result, now)
                    }
                }
                tx::Action::Program {
                    ticket,
                    session,
                    agreement,
                } => {
                    let tid = session.tid();
                    let window = agreement.map(tx::Agreement::window);
                    let programmed = Self::chip(self.chip.aggregation(tid, window));
                    if first.is_ok() {
                        first = programmed;
                    }
                    if agreement.is_none() {
                        if let Some(slot) = self.aggregating.get_mut(usize::from(tid)) {
                            *slot = None;
                        }
                        for held in &mut self.held {
                            if held.qos && held.tid == tid {
                                held.aggregate = None;
                            }
                        }
                    }
                    // SAFETY: rtw89 holds no agreement in firmware (the
                    // 8852C's `h2c_ampdu_cmac_tbl` is NULL), only the TID's
                    // `agg_num` and the RA aggregation limit. As under
                    // `IEEE80211_AMPDU_TX_STOP_CONT`, a frame the chip took
                    // goes as its descriptor says and later ones go singly.
                    unsafe {
                        self.station
                            .complete_tx_block_ack_program(ticket, Completion::Accepted)
                    }
                }
                tx::Action::Operational { session, .. } => {
                    if let Some(slot) = self.aggregating.get_mut(usize::from(session.tid())) {
                        *slot = Some(session);
                        bump(&mut self.progress.addba_ok);
                    }
                    continue;
                }
                tx::Action::ResumeLegacy { .. } | tx::Action::None => continue,
            }
            .map_err(Error::BlockAck)?;
            self.changes.push_front(next);
        }
        first
    }

    /// rtw88's `send_agreement_frame` (`station.rs:1085`): `bytes` to the
    /// chip's management queue at once, as `ieee80211_tx_skb` sends an ADDBA
    /// or a DELBA, sealed under PMF, as Block Ack action frames are robust;
    /// false when the chip has no room. An ADDBA Request taken is counted.
    ///
    /// The data written since the pass's last kick is kicked first. rtw89
    /// kicks a TID's frames as it pushes them (`rtw89_core_txq_schedule`,
    /// `core.c:4854-4858`) and `ba_work` sends the ADDBA only after, so the
    /// AP hears the frame that started the agreement before the request
    /// whose SSN follows it, and a DELBA after the frames before it; here the
    /// management queue's at-once kick would otherwise let it overtake them.
    fn send_agreement_frame(&mut self, bytes: &[u8]) -> Result<bool, Error> {
        Self::chip(self.chip.kick())?;
        let context = self.context.clone().ok_or(Error::Refused("no context"))?;
        let frame = if context.pmf().is_some() {
            let mut robust = bytes.to_vec();
            if let Some(fc1) = robust.get_mut(1) {
                *fc1 |= 0x40;
            }
            self.seal(&robust)?
        } else {
            bytes.to_vec()
        };
        let management = Management {
            bytes: &frame,
            basic_rates: self.basic_rates(),
        };
        let taken = Self::chip(self.chip.transmit_management(&management))?.is_some();
        if taken
            && bytes.get(MANAGEMENT_HEADER..MANAGEMENT_HEADER + 2) == Some(&[BLOCK_ACK_CATEGORY, 0])
        {
            bump(&mut self.progress.addba_sent);
        }
        Ok(taken)
    }

    /// rtw88's `release_parked` (`station.rs:1100`), `ieee80211_agg_start_txq`
    /// of each TID no longer changing: its parked data sent in order while
    /// `held` has room. A frame the link refuses now is dropped, as mac80211
    /// drops one it can no longer take.
    fn release_parked(&mut self) -> Result<(), Error> {
        let mut at = 0;
        while at < self.parked.len() && self.held.len() < MAX_HELD {
            let (tid, ..) = self.parked[at];
            if self.station.tx_block_ack_blocks_data(tid) {
                at += 1;
                continue;
            }
            let Some((_, ethernet, class)) = self.parked.remove(at) else {
                break;
            };
            match self.send(&ethernet, class) {
                Err(Error::Data(_)) => self.unsupported("a data frame its link refused once held"),
                result => result?,
            }
        }
        Ok(())
    }

    /// `ieee80211_sta_tear_down_BA_sessions`' TX half
    /// (`AGG_STOP_DESTROY_STA`, rtw88's `end_aggregation`,
    /// `station.rs:689`): each TID's agreement stopped without a DELBA
    /// (TX_STOP_FLUSH), so the next association's
    /// `configure_tx_block_ack` finds none.
    fn end_aggregation(&mut self) -> Result<(), Error> {
        for tid in 0..8 {
            if let Ok(action) = self.station.stop_tx_block_ack(tid, false) {
                self.changes.push_back(action);
            }
        }
        let ended = self.change_agreements();
        if ended.is_err() || !self.changes.is_empty() {
            self.unsupported("a TX Block Ack agreement that would not end");
        }
        ended
    }

    /// What the chip handed on since the last poll, every item taken: once
    /// the keeper configured the association, the frames deferred behind
    /// its response first; then each item the chip handed on, a frame heard
    /// while an accepted association response waits for the keeper
    /// deferred. The first error a deferred frame, the poll or an item gave
    /// is returned.
    fn take_incoming(&mut self, now: u64) -> Result<(), Error> {
        let mut first = Ok(());
        if self.admitted.is_none() {
            for frame in core::mem::take(&mut self.deferred) {
                let taken = self.take_frame(frame, now);
                if first.is_ok() {
                    first = taken;
                }
            }
        }
        let mut incoming = Vec::new();
        let polled = Self::chip(self.chip.poll(&mut |item| incoming.push(item)));
        if first.is_ok() {
            first = polled;
        }
        for item in incoming {
            let taken = match item {
                Incoming::Transmitted {
                    token,
                    acknowledged,
                } => self.transmitted(token, acknowledged, now),
                Incoming::Frame(received) => match self.count_rate(received) {
                    Some(mpdu) if self.admitted.is_some() => {
                        if self.deferred.len() < MAX_DEFERRED {
                            self.deferred.push_back(mpdu);
                        }
                        Ok(())
                    }
                    Some(mpdu) => self.take_frame(mpdu, now),
                    None => Ok(()),
                },
                Incoming::ScanEnded { completed } => self.scan_ended(completed),
                Incoming::BeaconLost => self.beacon_lost(),
                Incoming::RaReport(report) => {
                    self.tx_rate = Some(report);
                    Ok(())
                }
            };
            if first.is_ok() {
                first = taken;
            }
        }
        first
    }

    /// [`Self::frame`]: mac80211 drops a frame the station cannot use, and
    /// the exchange it answers retries or times out; reported once.
    fn take_frame(&mut self, frame: Mpdu, now: u64) -> Result<(), Error> {
        match self.frame(frame, now) {
            Err(Error::Station(_)) => {
                self.unsupported("a management frame the station refuses");
                Ok(())
            }
            taken => taken,
        }
    }

    /// A frame's release report: the awaited management frame's goes to
    /// the station as its TX status (rtw88's `Report::Transmitted`,
    /// `station.rs:1275-1288`, spec S1.10), an SAE frame's to its exchange
    /// (`ieee80211_sta_work`: unacknowledged, it goes again at once;
    /// acknowledged, the AP gets IEEE80211_AUTH_TIMEOUT_SAE), and is
    /// counted; one the station no longer awaits, and an EAPOL frame's, is
    /// counted only.
    fn transmitted(&mut self, token: Token, acknowledged: bool, now: u64) -> Result<(), Error> {
        if let Some((_, kind)) = self.reporting.filter(|&(awaited, _)| awaited == token) {
            self.reporting = None;
            self.count_ack(kind, acknowledged);
            return self
                .station
                .management_status(acknowledged, now)
                .map_err(Error::Station);
        }
        if let Some(at) = self.earlier.iter().position(|&(sent, _)| sent == token) {
            if let Some((_, kind)) = self.earlier.remove(at) {
                self.count_ack(kind, acknowledged);
            }
            return Ok(());
        }
        if let Some(at) = self.eapol_reports.iter().position(|&sent| sent == token) {
            self.eapol_reports.remove(at);
            if acknowledged && let Some(eapol) = &mut self.progress.eapol {
                bump(&mut eapol.acked);
            }
        }
        Ok(())
    }

    /// An acknowledged authentication or association, counted.
    fn count_ack(&mut self, kind: Kind, acknowledged: bool) {
        let exchange = match kind {
            Kind::Authentication => self.progress.auth.as_mut(),
            Kind::Association => self.progress.assoc.as_mut(),
            Kind::Other => None,
        };
        if acknowledged && let Some(exchange) = exchange {
            bump(&mut exchange.acked);
        }
    }

    /// One more of the data counts `field` names, once they are reached.
    fn count_data(&mut self, field: impl FnOnce(&mut Traffic) -> &mut u32) {
        if let Some(data) = &mut self.progress.data {
            bump(field(data));
        }
    }

    /// A protected data frame no key opened, dropped: one the hardware
    /// left encrypted, or one it decrypted under a key ID no key holds.
    fn count_keyless(&mut self, frame: &Mpdu) {
        if frame.security.decrypted {
            self.count_data(|data| &mut data.keyless);
        } else {
            self.count_data(|data| &mut data.sealed);
        }
    }

    /// The firmware's beacon loss (spec S15, ruling R2): while associated,
    /// not leaving and not scanning, the keeper hears the connection lost,
    /// which it reconnects from after `RECONNECT_US`, the loss is counted,
    /// and the station departs without a frame of its own, as
    /// `ieee80211_sta_connection_lost(.., tx false)` ends it once the probe
    /// of the AP that `ieee80211_beacon_loss` starts goes unanswered: the
    /// flush with drop, then the departure. Stela leaves at once, as a lost
    /// AP needs a new association either way (spec S1.11). Anything else
    /// is ignored.
    fn beacon_lost(&mut self) -> Result<(), Error> {
        if self.context.is_none() || self.left || self.scanning.is_some() {
            return Ok(());
        }
        bump(self.progress.beacon_lost.get_or_insert(0));
        self.events.push_back(keeper::Event::ConnectionLost);
        let action = self.station.disconnect().map_err(Error::Station)?;
        self.act(action);
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

    /// [`Self::mpdu`], a data frame addressed to the station that passed
    /// its FCS counted at its rate first (M4 spec S3.1).
    fn count_rate(&mut self, received: Received) -> Option<Mpdu> {
        let to_station = received.bytes.len() >= 10
            && received.bytes[0] & 0x0c == 0x08
            && received.bytes[4..10] == self.address.0;
        if to_station
            && !received.fcs_error
            && let Some(rate) = received.rate
        {
            self.rx_rates.record(rate, received.ampdu);
        }
        Self::mpdu(received)
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

    /// rtw88's `frame` (`station.rs:1344`) without TKIP (spec S13.3): a
    /// scan's beacons and probe responses to the scanner, which refuses one
    /// failing its FCS; then a frame failing its FCS or ICV dropped, as
    /// rtw89 marks either `RX_FLAG_FAILED_FCS_CRC` (`core.c:4396-4397`) and
    /// mac80211's `should_drop_frame` (`rx.c:93-105`, from
    /// `ieee80211_rx_monitor`, `rx.c:913-918`, at the donor's mac80211)
    /// drops it before any handler, so the duplicate check never records
    /// its sequence control; then a duplicate dropped
    /// ([`Self::duplicate`]); a Block Ack Request to the agreement it names;
    /// management frames to [`Self::management_frame`]; and data with a
    /// body to the station. A protected frame the hardware decrypted goes
    /// under the key its key ID names (pairwise for unicast, a group key for
    /// a group address). One left encrypted (mac80211 would decrypt it in
    /// software; no Stela station does), or under a key ID without a key,
    /// goes on keyless and is dropped as it is delivered
    /// ([`Self::deliver`]): mac80211 decrypts in `ieee80211_rx_h_decrypt`,
    /// after `ieee80211_rx_reorder_ampdu`, so such a frame still takes its
    /// place in its agreement's window. Fragments are reassembled, a
    /// keyless protected one dropped at once; other data goes through its
    /// agreement's reorder buffer ([`Self::order`]).
    fn frame(&mut self, mut frame: Mpdu, now: u64) -> Result<(), Error> {
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
        if frame.fcs_error || frame.security.icv_error {
            return Ok(());
        }
        if self.duplicate(&frame) {
            return Ok(());
        }
        if header.control & 0xfc == 0x84 {
            return self.block_ack_request(&frame, now);
        }
        if header.control & 12 == 0 {
            return self.management_frame(frame, now);
        }
        if header.control & 12 != 8 || !header.data_present() {
            return Ok(());
        }
        if header.protected() {
            let multicast = header.receiver.is_multicast();
            let index = frame.bytes().get(header.length + 3).map(|byte| byte >> 6);
            let decrypted = frame.security.decrypted;
            frame.security.key = self
                .keys
                .iter()
                .find(|key| key.pairwise != multicast && Some(key.index) == index)
                .and_then(|key| key.slot.installed())
                .filter(|_| decrypted);
        }
        if header.fragmented() {
            if header.protected() && frame.security.key.is_none() {
                self.count_keyless(&frame);
                return Ok(());
            }
            self.reassemble(&frame, now);
            return Ok(());
        }
        self.order(frame, now);
        Ok(())
    }

    /// `ieee80211_rx_h_check_dup` (`rx.c:1521-1576` at the donor's mac80211)
    /// on the AP's station's `last_seq_ctrl`: for a unicast frame from the
    /// AP joined that is neither a control frame nor a null function,
    /// whether it is retried with the sequence control its counter last saw
    /// (a duplicate, dropped); otherwise its sequence control becomes the
    /// last. QoS data counts per TID, everything else on one counter
    /// (`ieee80211_parse_qos`).
    fn duplicate(&mut self, frame: &Mpdu) -> bool {
        let header = frame.header();
        let bytes = frame.bytes();
        if bytes.len() < MANAGEMENT_HEADER
            || header.control & 12 == 4
            || matches!(header.control & 0xfc, 0x48 | 0xc8)
            || header.receiver.is_multicast()
            || self.ap.is_none()
            || header.transmitter != self.ap
        {
            return false;
        }
        // A TID is four bits: 0-15.
        let index = header.tid.map_or(SEQUENCES - 1, usize::from);
        let control = u16::from_le_bytes([bytes[22], bytes[23]]);
        if header.control & 0x0800 != 0 && self.last_sequence[index] == control {
            return true;
        }
        self.last_sequence[index] = control;
        false
    }

    /// `ieee80211_rx_reorder_ampdu` (`rx.c:1445-1518` at the donor's
    /// mac80211), rtw88's `order` (`station.rs:1427`): QoS data from the AP
    /// to the station on a TID under an agreement, unless sent without an
    /// acknowledgement, goes through the agreement's reorder buffer
    /// (`ieee80211_sta_manage_reorder_buf`), and what it releases goes on in
    /// order; everything else goes on at once.
    fn order(&mut self, frame: Mpdu, now: u64) {
        let header = *frame.header();
        let from_ap = self
            .context
            .as_ref()
            .is_some_and(|context| header.transmitter == Some(context.peer()));
        // IEEE80211_QOS_CTL_ACK_POLICY_NOACK, in the QoS control
        // `ieee80211_get_qos_ctl` finds.
        let qos_at = if header.control & 0x0300 == 0x0300 {
            30
        } else {
            24
        };
        let acknowledged = frame
            .bytes()
            .get(qos_at)
            .is_some_and(|qc| qc & 0x60 != 0x20);
        let agreement = header
            .tid
            .filter(|_| from_ap && acknowledged && !header.receiver.is_multicast())
            .and_then(|tid| self.agreements.get_mut(usize::from(tid)))
            .and_then(Option::as_mut);
        let (Some(agreement), Some(sequence)) = (agreement, header.sequence) else {
            self.deliver(frame);
            return;
        };
        let mut ready = Vec::new();
        if let Err(rejected) =
            agreement.push(sequence, now as u32, frame, |frame| ready.push(frame))
        {
            ready.push(rejected.frame);
        }
        for frame in ready {
            self.deliver(frame);
        }
    }

    /// `ieee80211_rx_h_ctrl`'s Block Ack Request, through the station's
    /// Block Ack layer: from the associated AP for an agreement it holds,
    /// what waits before its SSN is released at once; anything else is
    /// dropped.
    fn block_ack_request(&mut self, frame: &Mpdu, now: u64) -> Result<(), Error> {
        let Some(epoch) = self.station.association_epoch() else {
            return Ok(());
        };
        match self.station.receive_block_ack(epoch, frame, now) {
            Ok(action) => self.block_ack(action),
            Err(_) => Ok(()),
        }
    }

    /// rtw88's `block_ack` (`station.rs:1671-1740`), the station's Block Ack
    /// layer over mac80211's `agg-rx.c` with `rtw89_ops_ampdu_action`'s RX
    /// half beneath (spec S13.2):
    ///
    /// - A start (`__ieee80211_start_rx_ba_session`): the reorder buffer for
    ///   the agreement's window, then the chip's RX_START, then the
    ///   agreement accepted and its ADDBA Response queued. The chip's error
    ///   is returned after it, the agreement accepted all the same, as
    ///   `rtw89_ops_ampdu_action` returns 0 whatever its H2C did. A window
    ///   the host cannot hold is declined.
    /// - A stop (`___ieee80211_stop_rx_ba_session`): the chip's RX_STOP,
    ///   whose error is returned as mac80211 only logs it, then the buffer
    ///   and what it held dropped (`ieee80211_free_tid_rx`), then the stop
    ///   completed, which may bring a DELBA of the station's own or a
    ///   replacement agreement.
    /// - A frame to send: queued, protected under PMF, as Block Ack action
    ///   frames are robust.
    /// - A BAR: what waits before its SSN released.
    /// - A TX agreement's step (the AP's ADDBA Response or DELBA): kept for
    ///   [`Self::change_agreements`].
    fn block_ack(&mut self, action: ba::Action) -> Result<(), Error> {
        match action {
            ba::Action::Program {
                ticket,
                session,
                enable: true,
            } => {
                let tid = session.tid();
                let ssn = session.start_sequence();
                let window = session.window().min(RECEIVE_WINDOW);
                let mut chip = Ok(());
                let completion = match Reorder::new(tid, ssn, window, REORDER_TICKS_PER_SECOND) {
                    Ok(reorder) if usize::from(tid) < self.agreements.len() => {
                        chip = Self::chip(self.chip.start_rx_block_ack(tid, ssn, window));
                        self.agreements[usize::from(tid)] = Some(reorder);
                        Completion::Accepted
                    }
                    _ => {
                        self.unsupported("a Block Ack window the host cannot hold");
                        Completion::RejectedAndIdle
                    }
                };
                // SAFETY: the host alone holds the agreement's reorder state,
                // which is ready for this ticket; rtw89 waits for no BA CAM
                // acknowledgement (its H2C goes without a wait), and the
                // hardware's dynamic BA CAM answers the AP without an entry.
                let completed = unsafe { self.station.complete_rx_block_ack(ticket, completion) };
                let next = completed.map_or(Ok(()), |action| self.block_ack(action));
                chip.and(next)
            }
            ba::Action::Program {
                ticket,
                session,
                enable: false,
            } => {
                let tid = session.tid();
                let chip = Self::chip(self.chip.stop_rx_block_ack(tid));
                self.close_agreement(usize::from(tid));
                // SAFETY: the reorder buffer and what it held are gone, and
                // no frame of the agreement is delivered after this.
                let completed = unsafe {
                    self.station
                        .complete_rx_block_ack(ticket, Completion::Accepted)
                };
                let next = completed.map_or(Ok(()), |action| self.block_ack(action));
                chip.and(next)
            }
            ba::Action::Transmit {
                context, mut bytes, ..
            } => {
                if context.pmf().is_some()
                    && let Some(fc1) = bytes.get_mut(1)
                {
                    *fc1 |= 0x40;
                }
                self.queue_management(None, bytes);
                Ok(())
            }
            ba::Action::Bar {
                session,
                start_sequence,
            } => {
                let mut ready = Vec::new();
                if let Some(Some(agreement)) = self.agreements.get_mut(usize::from(session.tid())) {
                    let _ = agreement.bar(start_sequence, |frame| ready.push(frame));
                }
                for frame in ready {
                    self.deliver(frame);
                }
                Ok(())
            }
            // The AP's answer to an ADDBA Request, or its DELBA of a TX
            // agreement, for the next pass's `change_agreements`.
            ba::Action::Tx(action) => {
                self.changes.push_back(action);
                Ok(())
            }
            ba::Action::None => Ok(()),
        }
    }

    /// `ieee80211_free_tid_rx`: what the agreement on `tid` still held goes
    /// with it.
    fn close_agreement(&mut self, tid: usize) {
        if let Some(mut agreement) = self.agreements.get_mut(tid).and_then(Option::take) {
            agreement.stop();
        }
    }

    /// `sta_rx_agg_reorder_timer_expired`, releasing what waited too long.
    fn release_expired(&mut self, now: u64) {
        let mut ready = Vec::new();
        for agreement in self.agreements.iter_mut().flatten() {
            let _ = agreement.poll(now as u32, |frame| ready.push(frame));
        }
        for frame in ready {
            self.deliver(frame);
        }
    }

    /// rtw88's `reassemble` (`station.rs:1415`): `frame`, one fragment;
    /// the station has the MSDU with the last of them.
    fn reassemble(&mut self, frame: &Mpdu, now: u64) {
        let key = frame.security.key;
        let slot = self
            .keys
            .iter_mut()
            .map(|key| &mut key.slot)
            .find(|slot| key.is_some() && slot.installed() == key);
        if let Some(whole) = self.fragments.add(frame, slot, now) {
            self.deliver(whole);
        }
    }

    /// rtw88's `deliver` (`station.rs:1451`): `frame`, an MSDU or an
    /// A-MSDU, to the station; mac80211 drops what its replay and port
    /// checks refuse. EAPOL goes to the keeper. A protected frame without
    /// the key that opened it ([`Self::frame`]) is dropped first, as
    /// `ieee80211_rx_h_decrypt` drops it once the reorder buffer released
    /// it. The progress line counts each outcome but EAPOL's.
    fn deliver(&mut self, frame: Mpdu) {
        if frame.header().protected() && frame.security.key.is_none() {
            self.count_keyless(&frame);
            return;
        }
        let refused = |radio: &mut Self| radio.count_data(|data| &mut data.refused);
        let ingress = if frame.header().amsdu {
            let Some(context) = self.context.clone() else {
                refused(self);
                return;
            };
            let key = frame.security.key;
            // SAFETY: an A-MSDU is one whole MPDU, one RX queue record, and
            // comes here once its agreement's reorder buffer, if any,
            // released it in order (`Self::order`); `clear` closes every
            // agreement and drops what an older association left, so none
            // reaches a newer one.
            let group =
                unsafe { OrderedGroup::new(context, alloc::vec![frame], Shape::WholeMpdu, ()) };
            let slot = self
                .keys
                .iter_mut()
                .map(|key| &mut key.slot)
                .find(|slot| key.is_some() && slot.installed() == key);
            let Ok(received) = self.station.receive_ordered_group(&group, slot) else {
                refused(self);
                return;
            };
            let keys: Vec<&KeySlot> = self.keys.iter().map(|key| &key.slot).collect();
            match self.station.receive_group_refs(&received, &keys) {
                Ok(ingress) => ingress,
                Err(_) => {
                    refused(self);
                    return;
                }
            }
        } else {
            let Some(epoch) = self.station.association_epoch() else {
                refused(self);
                return;
            };
            let mut keys: Vec<&mut KeySlot> =
                self.keys.iter_mut().map(|key| &mut key.slot).collect();
            match self.station.receive_msdu_refs(epoch, &frame, &mut keys) {
                Ok(ingress) => alloc::vec![ingress],
                Err(_) => {
                    refused(self);
                    return;
                }
            }
        };
        for ingress in ingress {
            match ingress {
                Ingress::Data(ethernet) => {
                    self.count_data(|data| &mut data.delivered);
                    self.received.push_back(ethernet);
                }
                Ingress::ControlPort { ethernet, .. } => {
                    bump(&mut self.progress.eapol.get_or_insert_default().received);
                    self.events.push_back(keeper::Event::Eapol(ethernet));
                }
            }
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

    /// rtw88's `management_frame` (`station.rs:1514`): what the station's
    /// authentication and association take, the AP's departures, and its
    /// Block Ack action frames; a protected frame to
    /// [`Self::protected_management`]. Under PMF a robust frame to everyone
    /// must carry its IGTK's MIC (BIP on the host), a departure to the
    /// station alone must come protected once keys are installed, and a
    /// Block Ack action frame always (`ieee80211_drop_unencrypted_mgmt`,
    /// which the station's Block Ack layer applies). Beacons and probe
    /// responses outside a scan and other unprotected action frames (channel
    /// switches are not followed) are dropped, as is everything before a
    /// join.
    fn management_frame(&mut self, frame: Mpdu, now: u64) -> Result<(), Error> {
        let header = *frame.header();
        let subtype = header.control & 0xfc;
        if matches!(subtype, 0x80 | 0x50) {
            return Ok(());
        }
        if header.protected() {
            return self.protected_management(frame, now);
        }
        if self.station.phase() == Peering::ExternalAuthenticating {
            return self.exchange_frame(frame, now);
        }
        let Some(epoch) = self.station.association_epoch() else {
            return Ok(());
        };
        if subtype == 0xd0 {
            if frame.bytes().get(header.length) == Some(&BLOCK_ACK_CATEGORY)
                && let Ok(action) = self.station.receive_block_ack(epoch, &frame, now)
            {
                return self.block_ack(action);
            }
            return Ok(());
        }
        let pmf = self.station.requires_protected_management();
        if pmf && header.receiver.is_multicast() && protection::robust(&frame) {
            let Some(integrity) = &mut self.integrity else {
                return Ok(());
            };
            let Ok(authenticated) = integrity.authenticate(&frame) else {
                return Ok(());
            };
            if matches!(subtype, 0xa0 | 0xc0) {
                let body = authenticated.body();
                let action = self
                    .station
                    .receive_protected_departure(epoch, &frame, body)
                    .map_err(Error::Station)?;
                self.events.push_back(keeper::Event::ConnectionLost);
                self.act(action);
            }
            return Ok(());
        }
        if pmf && matches!(subtype, 0xa0 | 0xc0) && !self.keys.is_empty() {
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

    /// `ieee80211_rx_mgmt_auth` while the keeper's SAE exchange runs
    /// (rtw88's `management_frame`, `station.rs:1526`): an authentication
    /// frame from its AP (the BSSID its `auth_data` names) to the station or
    /// to a group address, as `ieee80211_accept_frame` lets one through to
    /// mlme, goes to the exchange, and one it takes to the keeper, which
    /// answers it. The AP's confirm of the station's ends the exchange
    /// authenticated, and the AP's station goes to AUTH on the chip
    /// (`ieee80211_mark_sta_auth`). Anything else is dropped, a
    /// deauthentication too: `ieee80211_rx_mgmt_deauth` acts only once
    /// associated or associating (`mlme.c:5646-5690` at the donor's
    /// mac80211), and the exchange runs on to its end.
    fn exchange_frame(&mut self, frame: Mpdu, now: u64) -> Result<(), Error> {
        let session = self
            .station
            .external_authentication()
            .cloned()
            .ok_or(Error::Refused("exchange"))?;
        let header = *frame.header();
        let peer = session.peer();
        let from_peer =
            header.transmitter == Some(peer) && frame.bytes().get(16..22) == Some(&peer.0[..]);
        let to_us = header.receiver == self.address || header.receiver.is_multicast();
        match header.control & 0xfc {
            0xb0 if from_peer && to_us => {
                let received = self
                    .station
                    .receive_external_authentication(&session, frame, now)
                    .map_err(Error::Station)?;
                // A frame after the exchange's end is not taken, so it
                // ends once.
                if let Some(received) = received {
                    self.events
                        .push_back(keeper::Event::Sae(Box::new(received)));
                    if self.station.external_authenticated() {
                        Self::chip(self.chip.authenticated())?;
                    }
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }

    /// rtw88's `protected_management` (`station.rs:1598`) without channel
    /// switches: a frame under the PTK, opened on the host when the
    /// hardware left it sealed, taken once under its packet number
    /// (`ieee80211_crypto_ccmp_decrypt` keeps management's apart from
    /// data's); a departure ends the association, a Block Ack action frame
    /// goes to the station's Block Ack layer, and an SA Query request is
    /// answered, sealed in turn.
    fn protected_management(&mut self, mut frame: Mpdu, now: u64) -> Result<(), Error> {
        let local = self.address;
        let Some(epoch) = self.station.association_epoch() else {
            return Ok(());
        };
        let header = *frame.header();
        let Some(key) = self.keys.iter_mut().find(|key| key.pairwise) else {
            return Ok(());
        };
        if !frame.security.decrypted {
            let number = frame
                .bytes()
                .get(header.length..)
                .and_then(|iv| PacketNumber::from_header(iv).ok());
            let material: Option<&[u8; 16]> = key.material[..].try_into().ok();
            let (Some((number, _)), Some(material)) = (number, material) else {
                return Ok(());
            };
            let Some(opened) = ccmp::open(material, frame.bytes(), number) else {
                return Ok(());
            };
            let security = SecurityStatus {
                decrypted: true,
                integrity: Integrity::Verified,
                ..SecurityStatus::default()
            };
            let Ok(clear) = Mpdu::new(opened, frame.radio.clone(), security) else {
                return Ok(());
            };
            frame = clear;
        }
        frame.security.key = key.slot.installed();
        let Ok(authenticated) = key.slot.authorize_unicast_management(&frame, local) else {
            return Ok(());
        };
        let body = authenticated.body();
        match header.control & 0xfc {
            0xa0 | 0xc0 => {
                let action = self
                    .station
                    .receive_protected_departure(epoch, &frame, body)
                    .map_err(Error::Station)?;
                self.events.push_back(keeper::Event::ConnectionLost);
                self.act(action);
            }
            0xd0 if body.first() == Some(&BLOCK_ACK_CATEGORY) => {
                let Some(context) = self.context.clone() else {
                    return Ok(());
                };
                if let Ok(action) = self
                    .station
                    .receive_authenticated_block_ack(&context, &frame, body, now)
                {
                    return self.block_ack(action);
                }
            }
            0xd0 if body.len() >= 4 && body[..2] == [8, 0] => {
                let response = [8, 1, body[2], body[3]];
                let Some(ap) = header.transmitter else {
                    return Ok(());
                };
                let bytes = management(local, ap, 0xd0, true, &response);
                self.queue_management(None, bytes);
            }
            _ => {}
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
        self.release_expired(now);
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
        // `ieee80211_auth`: the exchange's last frame again once its wait
        // passed (rtw88's `operate`).
        if !self.left && self.station.phase() == Peering::ExternalAuthenticating {
            let mut policy = Admitted(&self.channels);
            match self.station.external_retransmission(now, &mut policy) {
                Ok(Some(bytes)) => self.queue_management(None, bytes),
                Ok(None) => {}
                Err(error) => {
                    if first.is_ok() {
                        first = Err(Error::Station(error));
                    }
                }
            }
        }
        // `rtw89_forbid_ba_work` once its 4 s passed.
        if self.forbid_ends.is_some_and(|ends| ends <= now) {
            self.forbid_ends = None;
            self.forbidden = [false; 8];
        }
        // The TX agreements' due timers (`ieee80211_addba_resp_timer_expired`,
        // `sta_tx_agg_session_timer_expired`), each due one in turn, as
        // rtw88's `operate`.
        while !self.left
            && self
                .station
                .next_tx_block_ack_deadline()
                .is_some_and(|deadline| deadline <= now)
        {
            match self.station.poll_tx_block_ack(now) {
                Ok(tx::Action::None) => break,
                Ok(action) => self.changes.push_back(action),
                Err(error) => {
                    if first.is_ok() {
                        first = Err(Error::BlockAck(error));
                    }
                    break;
                }
            }
        }
        // rtw88's order: management, data, the agreements' changes, the
        // data they released, then data again.
        let sent = self.send_management();
        if first.is_ok() {
            first = sent;
        }
        let sent = self.send_data();
        if first.is_ok() {
            first = sent;
        }
        let changed = self.change_agreements();
        if first.is_ok() {
            first = changed;
        }
        let released = self.release_parked();
        if first.is_ok() {
            first = released;
        }
        let sent = self.send_data();
        if first.is_ok() {
            first = sent;
        }
        // The pass's kick hands over every data frame queued since the last
        // (`rtw89_core_txq_schedule`'s `rtw89_core_tx_kick_off`, M4 spec
        // S9.1): the network's frames and the station's own, written to the
        // chip as they came.
        let kicked = Self::chip(self.chip.kick());
        if first.is_ok() {
            first = kicked;
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
        let auth = if p.sae {
            let counts = p
                .auth
                .map_or_else(|| String::from("-"), |e| format!("{}/{}", e.sent, e.acked));
            format!("sae {counts}")
        } else {
            format!("auth {}", exchange(p.auth))
        };
        let mut line = format!(
            "scans {} ({} networks, target {target}), {auth}",
            p.scans,
            self.observed.len(),
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
        self.link_progress(&mut line);
        if let Some(d) = p.data {
            line.push_str(&format!(
                ", rx data {} sealed {} nokey {} refused {}, tx data {}",
                d.delivered, d.sealed, d.keyless, d.refused, d.sent
            ));
        }
        if let Some(lost) = p.beacon_lost {
            line.push_str(&format!(", beacon lost {lost}"));
        }
        if let Some(last) = p.last {
            line.push_str(&format!(", last \"{last}\""));
        }
        line.push_str("; ");
        line.push_str(&self.chip.diagnostic());
        line
    }

    /// The progress line's link fields (M4 spec S3.4), each `-` until known:
    /// the joined AP's operation; the TX power's map entry and regulatory
    /// index; the rate most of the station's data came at, its share and
    /// the data counted, and the share in A-MPDUs; RXQ's full polls; the
    /// firmware's last RA report (the AX report has no retry ratio, which
    /// Linux reads on BE only, `phy.c:3263`); ACH0's kicks and releases, and
    /// the releases per kick; the ADDBA Requests answered with success and
    /// sent; and the TIDs aggregating.
    fn link_progress(&self, line: &mut String) {
        let link = self.chip.link_counts();
        line.push_str(&format!(", ap {}", or_dash(self.ap_operation)));
        match link.regd {
            Some((alpha2, regd)) => line.push_str(&format!(
                ", regd {}{} {}",
                char::from(alpha2[0]),
                char::from(alpha2[1]),
                regd.name()
            )),
            None => line.push_str(", regd -"),
        }
        let h = &self.rx_rates;
        match h.top() {
            Some((rate, count)) => line.push_str(&format!(
                ", rx {} {}% ({}), ampdu {}%",
                rate_text(&rate),
                percent(count, h.total),
                h.total,
                percent(h.ampdu, h.total)
            )),
            None => line.push_str(", rx -, ampdu -"),
        }
        line.push_str(&format!(", rxq full {}", link.rxq_full));
        match &self.tx_rate {
            Some(report) => {
                line.push_str(&format!(", tx ra {} retry -", rate_text(&ra_rate(report))))
            }
            None => line.push_str(", tx ra -"),
        }
        let (kicks, released) = (link.tx.kicks[0], link.tx.released[0]);
        let mean = match kicks {
            0 => String::from("-"),
            kicks => {
                let tenths = (u64::from(released) * 10 + u64::from(kicks) / 2) / u64::from(kicks);
                format!("{}.{}", tenths / 10, tenths % 10)
            }
        };
        line.push_str(&format!(", kicks {kicks} rel {released} ({mean}/kick)"));
        let p = &self.progress;
        line.push_str(&format!(", addba {}/{}", p.addba_ok, p.addba_sent));
        line.push_str(&format!(", agg {}", self.aggregating_tids()));
    }

    /// The TIDs whose TX Block Ack agreement is operational, space-separated,
    /// `-` for none.
    fn aggregating_tids(&self) -> String {
        let tids: Vec<String> = (0..self.aggregating.len())
            .filter(|&tid| self.aggregating[tid].is_some())
            .map(|tid| format!("{tid}"))
            .collect();
        if tids.is_empty() {
            String::from("-")
        } else {
            tids.join(" ")
        }
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

    /// `ieee80211_prep_connection`, for a join and an SAE exchange alike:
    /// the channel, then the BSSID; the BSS's profile kept, and the AP's
    /// station new (`sta_info_alloc`).
    fn approach(&mut self, bss: &Bss, peer: Peer) -> Result<(), Error> {
        self.ap_operation = Some(ApOperation::of(bss));
        Self::chip(self.chip.tune(bss.channel))?;
        Self::chip(self.chip.set_bssid(bss.bssid))?;
        self.peer = Some(peer);
        self.ap = Some(bss.bssid);
        self.last_sequence = FRESH_SEQUENCES;
        Ok(())
    }

    /// No station began after [`Self::approach`]: the BSSID goes again, so
    /// a scan may run.
    fn withdraw(&mut self) {
        self.peer = None;
        self.ap = None;
        let _ = self.chip.disassociate();
    }

    fn join_with(&mut self, handle: usize, join: Join, peer: Peer) -> Result<(), Error> {
        let observed = self.observed(handle)?;
        let bss = observed.bss().clone();
        self.approach(&bss, peer)?;
        self.management_cipher = join.request.pmf().map(|pmf| pmf.management());
        self.progress.sae = false;
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
                self.withdraw();
                Err(Error::Station(error))
            }
        }
    }

    /// rtw88's `begin_external_authentication` (`station.rs:2041`), the
    /// half of `ieee80211_mgd_auth` before its first frame: the channel and
    /// BSSID as a join's ([`Self::approach`]), then the keeper's SAE
    /// exchange begun on the station. An MLD link's is refused first.
    fn begin_external_with(
        &mut self,
        handle: usize,
        peer: Peer,
        multi_link: Option<Affiliation>,
        budget_us: u64,
    ) -> Result<Arc<external::Session>, Error> {
        if multi_link.is_some() {
            return Err(Error::Unsupported(NO_MLO));
        }
        let bss = self.observed(handle)?.bss().clone();
        self.approach(&bss, peer)?;
        self.progress.sae = true;
        self.progress.auth.get_or_insert_default();
        let now = self.clock.now_us();
        let mut policy = Admitted(&self.channels);
        let begun = self.station.begin_external_authentication(
            external::Peer {
                bssid: bss.bssid,
                channel: bss.channel,
                ssid: bss.ssid.clone().unwrap_or_default(),
                multi_link: None,
            },
            now,
            budget_us,
            &mut policy,
        );
        begun.map_err(|error| {
            self.withdraw();
            Error::Station(error)
        })
    }

    /// rtw88's `transmit_external_authentication` (`station.rs:2079`): the
    /// keeper's SAE frame, built by the station, to the outbox. A confirm
    /// to an AP that confirmed first ends the exchange authenticated, and
    /// the AP's station goes to AUTH on the chip before it leaves, as
    /// `ieee80211_mgd_auth` marks it before `ieee80211_auth` sends it.
    fn transmit_external_with(
        &mut self,
        session: &external::Session,
        frame: external::Frame<'_>,
    ) -> Result<(), Error> {
        let now = self.clock.now_us();
        let mut policy = Admitted(&self.channels);
        let bytes = self
            .station
            .external_authentication_transmit(session, frame, now, &mut policy)
            .map_err(Error::Station)?;
        // The station builds no frame after the exchange's end, so this is
        // the frame that ended it.
        if self.station.external_authenticated() {
            Self::chip(self.chip.authenticated())?;
        }
        self.queue_management(None, bytes);
        Ok(())
    }

    /// rtw88's `associate_external` (`station.rs:2094`): the association
    /// once the exchange authenticated, on the channel and BSSID its start
    /// set, under the join's management cipher, as [`Self::join_with`]
    /// takes it.
    fn associate_external_with(
        &mut self,
        session: &external::Session,
        handle: usize,
        join: Join,
    ) -> Result<(), Error> {
        let observed = self.observed(handle)?;
        let now = self.clock.now_us();
        self.management_cipher = join.request.pmf().map(|pmf| pmf.management());
        let mut policy = Admitted(&self.channels);
        let action = self
            .station
            .associate_external_pmf(session, join, observed, now, &mut policy)
            .map_err(Error::Station)?;
        self.act(action);
        Ok(())
    }

    /// rtw88's `configure` (`station.rs:2113`) on rtw89 (spec S14.5): the
    /// chip's association on the join's channel, without a switch of the
    /// radio's own, then the station configured, and with HT its RX Block
    /// Ack limits: `max_rx_agg_num` and A-MSDUs in A-MPDUs
    /// (`SUPPORTS_AMSDU_IN_AMPDU`, `core.c:7500`); with a peer that
    /// aggregates, its TX Block Ack limit, `max_tx_agg_num` (M4 spec S10.1).
    fn configure_admitted(&mut self) -> Result<(), Error> {
        let (ticket, context) = self
            .admitted
            .take()
            .ok_or(Error::Refused("configuration"))?;
        Self::chip(self.chip.associate(&association(&context)))?;
        let ht = context.negotiated().ht.is_some();
        let aggregates = context.negotiated().aggregates();
        self.context = Some(context);
        self.station
            .complete_configuration(ticket, Completion::Accepted, self.clock.now_us())
            .map_err(Error::Station)?;
        if ht {
            self.station
                .configure_rx_block_ack(RECEIVE_WINDOW, true)
                .map_err(|_| Error::Refused("Block Ack limits"))?;
        }
        if aggregates {
            self.station
                .configure_tx_block_ack(TRANSMIT_WINDOW)
                .map_err(|_| Error::Refused("Block Ack limits"))?;
        }
        self.progress.eapol.get_or_insert_default();
        self.progress.data.get_or_insert_default();
        Ok(())
    }

    /// rtw88's `deauthenticate` (`station.rs:2296`): the deauthentication
    /// after what the outbox holds, then the station's departure, even when
    /// the chip refused the frame. Held data never leaves after it. From an
    /// association, as `ieee80211_set_disassoc` sends it: a flush with drop
    /// first (mac80211 purges its queues, the chip drops what it holds),
    /// the frame, then a flush without drop so it leaves (`mlme.c:4594-4604`
    /// at the donor's mac80211); `ieee80211_mgd_deauth` flushes neither
    /// during an authentication or association. A flush's failure is passed
    /// over, as Linux's is. An SAE exchange, before any join the station
    /// could address the frame from, is left with a deauthentication
    /// between the exchange's addresses, as `ieee80211_mgd_deauth` aborts
    /// an authentication "by local choice".
    fn deauthenticate_with(&mut self, reason: u16) -> Result<(), Error> {
        if self.left || self.station.phase() == Peering::Idle {
            return Ok(());
        }
        let associated = self.context.is_some();
        if associated {
            self.held.clear();
            let _ = self.chip.flush(true);
        }
        let addresses = self.station.addresses().ok().or_else(|| {
            self.station
                .external_authentication()
                .map(|session| (session.local(), session.peer()))
        });
        if let Some((local, ap)) = addresses {
            let protected = self.station.protected_port_authorized();
            let bytes = management(local, ap, 0xc0, protected, &reason.to_le_bytes());
            self.queue_management(None, bytes);
        }
        let sent = self.send_management();
        if associated {
            let _ = self.chip.flush(false);
        }
        match self.station.disconnect().map_err(Error::Station)? {
            Action::Remove { ticket, .. } => self.leave(ticket),
            action => self.act(action),
        }
        sent
    }

    /// rtw88's `retire_peer` (`station.rs:2311`) on rtw89 (spec S14.5):
    /// `sta_info_flush`'s `__sta_info_destroy_part1` first stops each RX
    /// Block Ack agreement on the chip without a DELBA and drops what it
    /// held, then each TX agreement ([`Self::end_aggregation`])
    /// (`ieee80211_sta_tear_down_BA_sessions(AGG_STOP_DESTROY_STA)`,
    /// `sta_info.c:1348` at the donor's mac80211); then `_part2`'s key
    /// deletes, the PTK, then the group keys by key ID, each as
    /// `rtw89_ops_set_key`'s DISABLE_KEY runs it, a flush without drop, then
    /// the CAM delete (`mac80211.c:1010-1015`); then the chip's
    /// disassociation (spec S11), which releases the channel; then the
    /// removal is accepted. Every step runs whatever an earlier one
    /// returned, as mac80211 goes on past a driver's failures, and the
    /// first error is returned. A failed disassociation powered the chip
    /// off, which holds no association either.
    fn retire(&mut self) -> Result<(), Error> {
        let mut first = Ok(());
        for tid in 0..self.agreements.len() {
            if self.agreements[tid].is_some() {
                let stopped = Self::chip(self.chip.stop_rx_block_ack(tid as u8));
                self.close_agreement(tid);
                if first.is_ok() {
                    first = stopped;
                }
            }
        }
        let ended = self.end_aggregation();
        if first.is_ok() {
            first = ended;
        }
        let mut keys = core::mem::take(&mut self.keys);
        keys.sort_by_key(|key| (!key.pairwise, key.index));
        for key in &keys {
            let _ = self.chip.flush(false);
            let removed = Self::chip(self.chip.remove_key(key.cam));
            if first.is_ok() {
                first = removed;
            }
        }
        drop(keys);
        let left = Self::chip(self.chip.disassociate());
        if first.is_ok() {
            first = left;
        }
        // rtw88's order: the ticket stays until the removal is completed,
        // and `clear` drops it with the rest.
        if let Some(ticket) = self.departing {
            self.station
                .complete_removal(ticket, Completion::Accepted)
                .map_err(Error::Station)?;
        }
        self.clear();
        first
    }

    /// `rtw89_ops_set_key`'s SET_KEY (spec S12, S14.5) of a CCMP-128 key:
    /// the pairwise key with `TransmitSeed::host(0)`, a group key
    /// receive-only. As `ieee80211_key_link`: an identical key of the same
    /// kind and index is not installed again (-EALREADY, which mac80211
    /// takes as success); another replaces it (`ieee80211_key_replace`),
    /// the old one deleted first (a flush without drop, then the CAM
    /// delete, its failure passed over), then the new one installed. The
    /// key counts as installed only once the chip's `add_key` returned its
    /// CAM entry, the firmware having acknowledged it; otherwise its slot
    /// is never completed and goes. An open port is authorized again on the
    /// keys then held. A replacement closes it from the old key's delete
    /// on, so it opens again only on the new key installed: where the chip
    /// refuses that, mac80211 would go on under a software key, which no
    /// Stela station has, and nothing of that kind and index is left.
    fn install(
        &mut self,
        pairwise: bool,
        index: u8,
        material: &[u8],
        receive: ReplaySeed,
    ) -> Result<KeyIdentity, Error> {
        self.progress.keys.get_or_insert(0);
        let temporal: &[u8; 16] = material
            .try_into()
            .map_err(|_| Error::Unsupported(CCMP_128_ONLY))?;
        let peer = self
            .station
            .pairwise_peer()
            .ok_or(Error::Refused("no peer"))?;
        if let Some(at) = self
            .keys
            .iter()
            .position(|key| key.pairwise == pairwise && key.index == index)
        {
            if same_material(&self.keys[at].material, material)
                && let Some(identity) = self.keys[at].slot.installed()
            {
                return Ok(identity);
            }
            let old = self.keys.remove(at);
            self.port_open = false;
            // What still waits under the old key goes with it: its CAM
            // entry may hold the new key by the time the chip has room.
            self.held
                .retain(|held| held.key.is_none_or(|(cam, _, _)| cam != old.cam));
            let _ = self.chip.flush(false);
            let _ = self.chip.remove_key(old.cam);
        }
        let scope = if pairwise {
            KeyScope::Pairwise(peer)
        } else {
            KeyScope::Group(peer)
        };
        let transmit = if pairwise {
            TransmitSeed::host(PacketNumber::new(0).ok_or(Error::Refused("packet number"))?)
        } else {
            TransmitSeed::RECEIVE_ONLY
        };
        // SAFETY: the counter gives each key of the radio its own slot,
        // which no other `KeySlot` controls.
        let mut slot = unsafe { KeySlot::new(self.next_slot) }.map_err(Error::Key)?;
        self.next_slot = self.next_slot.wrapping_add(1);
        let ticket = slot
            .begin_install(index, Cipher::Ccmp128, material, receive, scope, transmit)
            .map_err(Error::Key)?;
        let identity = ticket.identity();
        let cam = Self::chip(self.chip.add_key(&HardwareKey {
            pairwise,
            index,
            material: temporal,
        }))?;
        slot.complete_install(ticket, InstallOutcome::Accepted)
            .map_err(Error::Key)?;
        self.keys.push(Key {
            slot,
            pairwise,
            index,
            cam,
            material: Zeroizing::new(material.to_vec()),
        });
        if let Some(keys) = &mut self.progress.keys {
            bump(keys);
        }
        // mac80211 keeps the AP's station authorized across a rekey; the
        // station layer authorizes one PTK and GTK, which a replaced or
        // newer key leaves behind, so the port is authorized again on the
        // keys now held. The keeper installs a key only from an
        // authenticated handshake.
        if self.authorized {
            let epoch = self
                .station
                .association_epoch()
                .ok_or(Error::Refused("no association"))?;
            self.authorize(epoch)?;
        }
        Ok(identity)
    }

    /// The host's IGTK (spec S1.2), rtw88's `install_management_key`
    /// (`station.rs:2236`): BIP under the join's management cipher, beside
    /// the GTK, which must be installed first (`Pending` until then).
    fn install_integrity(
        &mut self,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<Arc<ManagementKey>, Error> {
        let cipher = self
            .management_cipher
            .ok_or(Error::Refused("no management cipher"))?;
        let gtk = self
            .keys
            .iter()
            .rev()
            .find(|key| !key.pairwise)
            .and_then(|key| key.slot.installed())
            .ok_or(Error::Pending)?;
        self.integrity = Some(
            BipKey::new(gtk, cipher, index, material, receive)
                .map_err(|_| Error::Refused("the IGTK"))?,
        );
        Ok(Arc::new(ManagementKey))
    }

    /// rtw88's `authorize_port` (`station.rs:2271`): the PTK and the last
    /// GTK, under PMF with the association's protection.
    fn authorize(&mut self, epoch: u64) -> Result<(), Error> {
        let context = self.context.clone().ok_or(Error::Refused("no context"))?;
        let ptk = self
            .keys
            .iter()
            .find(|key| key.pairwise)
            .ok_or(Error::Refused("no pairwise key"))?;
        let gtk = self
            .keys
            .iter()
            .rev()
            .find(|key| !key.pairwise)
            .ok_or(Error::Refused("no group key"))?;
        let links = [(&ptk.slot, &gtk.slot)];
        if self.station.requires_protected_management() {
            self.station
                .authorize_protected_port(&context, epoch, &links)
        } else {
            self.station.authorize_port(epoch, &links)
        }
        .map_err(Error::Station)?;
        self.port_open = true;
        self.authorized = true;
        Ok(())
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

    /// SAE's start (spec S1.16): the channel and BSSID, then the exchange.
    fn begin_external_authentication(
        &mut self,
        handle: usize,
        peer: Peer,
        multi_link: Option<Affiliation>,
        budget_us: u64,
    ) -> Result<Arc<external::Session>, Error> {
        let begun = self.begin_external_with(handle, peer, multi_link, budget_us);
        self.note(begun)
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
        session: &external::Session,
        frame: external::Frame<'_>,
    ) -> Result<(), Error> {
        let sent = self.transmit_external_with(session, frame);
        self.note(sent)
    }

    fn associate_external(
        &mut self,
        session: &external::Session,
        handle: usize,
        join: Join,
    ) -> Result<(), Error> {
        let associated = self.associate_external_with(session, handle, join);
        self.note(associated)
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

    /// `rtw89_ops_set_key` of CCMP-128 alone (spec S1.2), into the chip's
    /// security CAM (its `install`).
    fn install_key(
        &mut self,
        pairwise: bool,
        index: u8,
        material: &[u8],
        receive: ReplaySeed,
    ) -> Result<KeyIdentity, Error> {
        let installed = self.install(pairwise, index, material, receive);
        self.note(installed)
    }

    fn install_link_key(&mut self, _: u8, _: u8, _: &[u8], _: ReplaySeed) -> Result<(), Error> {
        self.note(Err(Error::Unsupported(NO_MLO)))
    }

    /// The IGTK stays on the host, where mac80211 runs BIP (spec S1.2).
    unsafe fn install_management_key(
        &mut self,
        _: u64,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<Arc<ManagementKey>, Error> {
        let installed = self.install_integrity(index, material, receive);
        self.note(installed)
    }

    /// A rekeyed IGTK replaces the host's.
    unsafe fn rotate_management_key(
        &mut self,
        _: &Arc<ManagementKey>,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<Arc<ManagementKey>, Error> {
        let installed = self.install_integrity(index, material, receive);
        self.note(installed)
    }

    /// The port opens on an installed PTK and GTK.
    fn authorize_port(&mut self, epoch: u64) -> Result<(), Error> {
        let authorized = self.authorize(epoch);
        self.note(authorized)
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

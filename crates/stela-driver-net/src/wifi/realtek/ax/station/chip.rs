// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2018-2019 Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! The chip-neutral hardware half of an rtw89 station (M3 spec S14.1),
//! Stela's own: what rtw88's `Device` (`realtek/station.rs:132-192`) is to
//! its `Rtl8822c`, generalized from rtw88's descriptor, queue and H2C word
//! types to frames, channels and peers, so a radio drives any chip beneath
//! it. Each call is the hardware half of the mac80211 operation it names, as
//! rtw89's `mac80211.c` callbacks are: `rtw89_ops_tx`
//! ([`Chip::transmit_management`], [`Chip::transmit_data`]), the PCI NAPI
//! poll of RPQ, then RXQ (`rtw89_pci_napi_poll`, `pci.c:4528-4534`,
//! [`Chip::poll`]), `rtw89_ops_hw_scan` and its completion
//! ([`Chip::begin_scan`], [`Chip::end_scan`]), the channel context's
//! `rtw89_set_channel` ([`Chip::tune`]), `BSS_CHANGED_BSSID`
//! ([`Chip::set_bssid`]), the AP's station moved from NONE to AUTH
//! (`__rtw89_ops_sta_state`, [`Chip::authenticated`]),
//! `BSS_CHANGED_ASSOC` ([`Chip::associate`]), the
//! station's way down to NOTEXIST and the channel context's release
//! ([`Chip::disassociate`]), `rtw89_ops_set_key` ([`Chip::add_key`],
//! [`Chip::remove_key`]), `rtw89_ops_flush` ([`Chip::flush`]) and
//! `rtw89_ops_ampdu_action`'s RX half
//! ([`Chip::start_rx_block_ack`], [`Chip::stop_rx_block_ack`]).
//!
//! The trait grants nothing by itself: a call's success is what its
//! implementation documents, and an error is that implementation's own
//! text, which names no address.
use crate::Mac;
use crate::wifi::Channel;
use crate::wifi::realtek::ax::assoc::HtPeer;
use crate::wifi::realtek::ax::c2h::RaReport;
use crate::wifi::realtek::ax::regd::Regd;
use crate::wifi::realtek::ax::rx::Rate;
use crate::wifi::realtek::ax::txd::Aggregate;
use crate::wifi::station::phy::AccessCategory;
use alloc::{string::String, vec::Vec};

/// A frame the chip took, until its [`Incoming::Transmitted`]: on an
/// RTL8852C its TX channel's ring index (`enum rtw89_tx_channel`) in bits
/// 15:8 and its WD page's sequence number in bits 7:0. A token is reported
/// once; it may name a later frame after that.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Token(pub u16);

/// A frame the chip received. `bytes` is the MPDU without its FCS; a frame
/// the hardware decrypted keeps its security header and MIC, as no
/// `RX_FLAG_*_STRIPPED` is set (M3 spec S7.3). `channel` is the channel it
/// was heard on, `signal_dbm` its signal when one was measured, and `tsf`
/// the chip's free-running counter at its reception. `fcs_error` is set only
/// on a scan's frames, which the scanner refuses (M2c's scan); a station's
/// frames failing their FCS or ICV are dropped beneath this interface.
/// `rate` is its PPDU's rate as the chip decoded it, when it did, and
/// `ampdu` whether it came in an A-MPDU (M4 spec S3.1).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Received {
    pub bytes: Vec<u8>,
    pub channel: Channel,
    pub signal_dbm: Option<i8>,
    pub tsf: u32,
    /// `RX_FLAG_DECRYPTED`.
    pub decrypted: bool,
    pub icv_error: bool,
    /// `RX_FLAG_FAILED_FCS_CRC`.
    pub fcs_error: bool,
    pub rate: Option<Rate>,
    pub ampdu: bool,
}

/// What [`Chip::poll`] hands its sink.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Incoming {
    Frame(Received),
    /// The frame `token` names left the chip, acknowledged by its receiver
    /// or not (`IEEE80211_TX_STAT_ACK`).
    Transmitted {
        token: Token,
        acknowledged: bool,
    },
    /// The scan [`Chip::begin_scan`] began has ended: `completed` unless it
    /// was aborted or failed (`ieee80211_scan_completed`'s `aborted`). Every
    /// frame it heard was handed on before this.
    ScanEnded {
        completed: bool,
    },
    /// The firmware reported the associated AP's beacons lost, while no
    /// scan ran (M3 spec S15).
    BeaconLost,
    /// The firmware's rate adaptation reported the rate it sends the AP at
    /// (`rtw89_phy_c2h_ra_rpt`, M4 spec S3.3), for the station's MAC ID.
    RaReport(RaReport),
}

/// A management frame (or null data frame) to send: `bytes` from its MAC
/// header on, without FCS. `basic_rates` is the joined BSS's basic-rate
/// bitmap in mac80211's band order (on 2.4 GHz CCK 1-11 in bits 0-3 and
/// OFDM 6-54 in bits 4-11, elsewhere OFDM 6-54 in bits 0-7, as
/// `LegacyRates` holds it), 0 before one is known; the frame leaves at the
/// lowest of them.
pub struct Management<'a> {
    pub bytes: &'a [u8],
    pub basic_rates: u16,
}

/// A data frame to send: `bytes` from its MAC header on, without FCS and
/// without its security header, which the hardware inserts. `tid` picks its
/// queue; `eapol` marks an EAPOL frame. `supported_rates` is the peer's, in
/// [`Management::basic_rates`]' order: the lowest is the retry floor. `key`
/// is the security CAM entry, key ID and packet number the frame is
/// encrypted under (`Protection::HostIv`'s, the IV already removed).
/// `aggregate` is the A-MPDU it goes in, when its TID's TX Block Ack agreement
/// is operational (M4 spec S10.3).
pub struct Data<'a> {
    pub bytes: &'a [u8],
    pub tid: u8,
    pub eapol: bool,
    pub supported_rates: u16,
    pub key: Option<(u8, u8, u64)>,
    pub aggregate: Option<Aggregate>,
}

/// An association as mac80211 holds it at `BSS_CHANGED_ASSOC`: the AP's
/// address and AID, the channel joined, the beacon interval (TU) and DTIM
/// period, the peer's legacy rates (in [`Management::basic_rates`]' order)
/// and negotiated HT, its EDCA parameters in ACI order when it has WMM, and
/// whether the BSS uses the short slot.
pub struct Association {
    pub bssid: Mac,
    pub aid: u16,
    pub channel: Channel,
    pub beacon_interval: u16,
    pub dtim_period: u8,
    pub legacy: u16,
    pub ht: Option<HtPeer>,
    pub edca: Option<[AccessCategory; 4]>,
    pub short_slot: bool,
}

/// A key to install: pairwise or the group key `index`, its 16 bytes of
/// CCMP-128 material.
pub struct HardwareKey<'a> {
    pub pairwise: bool,
    pub index: u8,
    pub material: &'a [u8; 16],
}

/// What an RTL8852C's TX side did (M4 spec S3.2), saturating, each array
/// indexed by channel: ACH0 to ACH3 (BE, BK, VI, VO), then CH8, the
/// management queue. `submitted` counts the frames each channel took,
/// `kicks` the writes of its TX ring index (`__rtw89_pci_tx_kick_off`),
/// `released` the RPQ release reports for it (`rtw89_pci_release_rpp`), and
/// `failed` the releases of any channel whose TX status was not
/// `RTW89_TX_DONE` (statuses 1 to 3: retry limit, lifetime, MACID drop).
/// `rpq_full` counts the RPQ polls that found every BD filled. The mean
/// releases per kick on ACH0 is the aggregation's proxy: the firmware does
/// not report its aggregates' sizes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TxCounts {
    pub submitted: [u32; 5],
    pub kicks: [u32; 5],
    pub released: [u32; 5],
    pub failed: u32,
    pub rpq_full: u32,
}

/// What the chip counts for the station's progress line (M4 spec S3.4):
/// the alpha2 of the TX power's regulatory map entry (`00` for the
/// world's) and its regulatory index on the band tuned, once known; the
/// RXQ polls that found the ring full; and [`TxCounts`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LinkCounts {
    pub regd: Option<([u8; 2], Regd)>,
    pub rxq_full: u32,
    pub tx: TxCounts,
}

/// The hardware beneath an rtw89 station (module documentation).
pub trait Chip {
    /// The chip's own address.
    fn mac(&self) -> Mac;
    /// What arrived since the last poll, in Linux's NAPI order: RPQ's
    /// release reports first, then RXQ's frames and reports. An error is
    /// not fatal: every step of the poll ran, every event before the
    /// error was handed on, and the chip stays as it was; the next poll
    /// tries again.
    fn poll(&mut self, sink: &mut dyn FnMut(Incoming)) -> Result<(), String>;
    /// Switch to `channel`, 20 MHz (M3 spec S9.1).
    fn tune(&mut self, channel: Channel) -> Result<(), String>;
    /// Begin a passive firmware scan of `channels` (M3 spec S8.1). The chip
    /// bounds it by its own budget from the list; [`Incoming::ScanEnded`]
    /// says when it ended.
    fn begin_scan(&mut self, channels: &[Channel]) -> Result<(), String>;
    /// Restore the chip after the scan [`Incoming::ScanEnded`] reported
    /// (M3 spec S8.3).
    fn end_scan(&mut self) -> Result<(), String>;
    /// Point the chip at `bssid` before authentication (M3 spec S9.2-S9.3).
    fn set_bssid(&mut self, bssid: Mac) -> Result<(), String>;
    /// The AP accepted the authentication: mac80211 moves the AP's station
    /// from NONE to AUTH (`ieee80211_mark_sta_auth`), which rtw89 programs
    /// nothing for, so a [`Chip::disassociate`] before an association runs
    /// AUTH to NONE's disconnect (M3 spec S11) instead of NONE's nothing.
    fn authenticated(&mut self) -> Result<(), String>;
    /// Queue a management frame; `Ok(None)` when there is no room now.
    fn transmit_management(&mut self, frame: &Management<'_>) -> Result<Option<Token>, String>;
    /// Queue a data frame; `Ok(None)` when there is no room now. It may wait
    /// for the next [`Chip::kick`] to leave; a management frame does not.
    fn transmit_data(&mut self, frame: &Data<'_>) -> Result<Option<Token>, String>;
    /// Hand over every data frame queued since the last kick, the end of a
    /// service pass (`rtw89_core_tx_kick_off` after
    /// `rtw89_core_txq_schedule` pushed a TID's frames, M4 spec S9.1); none
    /// for a chip whose frames leave as they are queued.
    fn kick(&mut self) -> Result<(), String> {
        Ok(())
    }
    /// Program the association (M3 spec S10).
    fn associate(&mut self, association: &Association) -> Result<(), String>;
    /// Leave the BSS set, authenticated to or associated with (M3 spec
    /// S11), and release its channel.
    fn disassociate(&mut self) -> Result<(), String>;
    /// Install a key; its security CAM entry (M3 spec S12).
    fn add_key(&mut self, key: &HardwareKey<'_>) -> Result<u8, String>;
    fn remove_key(&mut self, cam: u8) -> Result<(), String>;
    /// mac80211's `drv_flush`: wait for the frames the chip took to leave
    /// its TX rings, then with `drop` drop what it still queues for the AP,
    /// or without it wait for its TX queues to empty. Neither wait fails
    /// for time, as Linux's only log theirs; nothing is reported for a
    /// dropped frame beyond its release report, if any.
    fn flush(&mut self, drop: bool) -> Result<(), String>;
    /// Program the AP's RX Block Ack agreement on `tid` from `ssn` with a
    /// window of `window` (M3 spec S13.2), mac80211's
    /// `IEEE80211_AMPDU_RX_START`. The host reorders what it delivers.
    fn start_rx_block_ack(&mut self, tid: u8, ssn: u16, window: u16) -> Result<(), String>;
    /// End the agreement on `tid`, `IEEE80211_AMPDU_RX_STOP`.
    fn stop_rx_block_ack(&mut self, tid: u8) -> Result<(), String>;
    /// The station's TX Block Ack agreement on `tid` is operational with a
    /// window of `window` (`IEEE80211_AMPDU_TX_OPERATIONAL`), or, `None`,
    /// stopped (`IEEE80211_AMPDU_TX_STOP_*`), M4 spec S10.2; nothing for a chip
    /// that keeps no aggregation state.
    fn aggregation(&mut self, tid: u8, window: Option<u16>) -> Result<(), String> {
        let _ = (tid, window);
        Ok(())
    }
    /// The chip's own counts, naming no address.
    fn diagnostic(&self) -> String;
    /// The counts the station's progress line prints (M4 spec S3.4); none
    /// for a chip that keeps none.
    fn link_counts(&self) -> LinkCounts {
        LinkCounts::default()
    }
}

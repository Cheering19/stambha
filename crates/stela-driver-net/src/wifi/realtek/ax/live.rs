// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2020  Realtek Corporation
//! An RTL8852CE kept powered after its bring-up (M3 spec S2-S3): [`start`]
//! runs the bring-up of [`super::bringup`] through `rtw89_phy_dm_init`
//! (`core.c:6697`), then what `rtw89_leave_ips` (`ps.c:257-278`) runs from
//! idle, [`leave_ips`]: the rest of `rtw89_core_start` (`core.c:6658-6725`,
//! [`core_tail`]), `rtw89_set_channel` (`core.c:600-618`) to 2.4 GHz channel
//! 1 with `__rtw89_set_channel`'s TX power between the RF part and the
//! help's exit (`core.c:568-598`, [`channel::set_channel`]), and
//! `rtw89_mac_vif_init` (`mac.c:5045`, [`vif`]); then the WD pages and
//! payload slots of the data and management queues, as
//! `rtw89_pci_alloc_tx_wd_ring` (`pci.c:3539`) allocates them at probe
//! ([`TxPools`]). The result, [`Live`], owns the register window, the rings,
//! the pools, the H2C sequence, the command session's receiver and ledger,
//! the bring-up's report, the inputs of every channel switch, the TX power
//! tables with their regulatory index, and the security CAM. [`Live::tune`]
//! switches channel as `__rtw89_set_channel` does, TX power included;
//! [`Live::stop`] is `rtw89_core_stop` (`core.c:6727-6767`): the coex half,
//! `rtw89_pci_ops_deinit` (`pci.c:3053`), then `rtw89_mac_pwr_off`
//! (`mac.c:1603`).
//!
//! A failure of any step after the power-on powers the chip off as the
//! bring-up's failures do and hands the window and the rings back
//! ([`StartError`]). A failed channel switch leaves the TX scheduler stopped
//! and the BB held in reset, so [`Live::tune`] powers the chip off on any
//! failure after its first access, and every later call is refused with
//! [`POWERED_OFF`].
//!
//! A firmware scan over any list is an operation on the live chip (M3 spec
//! S1.3, S8), each step from [`scan`]: [`Live::begin_scan`] is
//! `rtw89_hw_scan_start` and `rtw89_hw_scan_offload(true)`
//! (`fw.c:10367-10437`, `:10531-10571`); [`Live::drain`] reads RXQ, where
//! END_SCAN ends the scan (`rtw89_mac_c2h_scanofld_rsp`, `mac.c:5353-5367`)
//! and the drain that brings it also drains what follows, as M2c's scan
//! does after END_SCAN; [`Live::expire_scan`]
//! stops a scan past its budget as `rtw89_hw_scan_abort` does
//! (`fw.c:10494-10511`); [`Live::end_scan`] is `rtw89_hw_scan_complete`
//! (`fw.c:10473-10492`): `rtw89_chanctx_proceed`'s `rtw89_set_channel`
//! (`chan.c:3269-3302`) back to the operating channel through
//! [`Live::tune`], TX power and receiver tag included, then
//! `rtw89_hw_scan_complete_cb`. The operating channel is the one the last
//! switch set, as Linux returns to its channel context's chandef, or, once
//! [`Live::release_channel`] has released that context, to the default
//! chandef, 2.4 GHz channel 1 (`chan.c:614-616`).
//!
//! While the scan's RX filter is on, from `rtw89_hw_scan_start` to the
//! restore's filter write, the receiver keeps only beacons and probe
//! responses ([`Mode::Scan`], M2c's), and every other frame is counted, not
//! handed on. The scan's filter clears `B_AX_A_A1_MATCH`, `B_AX_A_BC` and
//! `B_AX_A_BCN_CHK_EN` (`fw.c:10424-10428`), so the chip then passes other
//! BSSes' frames too; Linux hands them all to mac80211, whose scan takes
//! only beacons and probe responses, and an unassociated station, the only
//! one that scans here (spec S1.15), acts on no other frame from a BSS it
//! has not joined. A station receiver would hand each to the station's
//! scanner, which refuses it, counted as dropped from the scan. An
//! associated scan would need the station receiver, as the AP's frames go on
//! during it in Linux.
//!
//! A scan whose offload fails is aborted as `rtw89_ops_hw_scan` aborts it
//! (`mac80211.c:1315-1319`): the stop, whose failure Linux only warns of,
//! then the restore. A failure in the scan start's own steps, which Linux's
//! cannot fail, or in the restore, powers the chip off, as a failed channel
//! switch does: the chip is left with the scan's RX filter, EDCCA, TSF sync
//! or DIG, or a stopped TX scheduler.
//!
//! A station's join, association and leaving are operations too (M3 spec
//! S9-S11, each step from [`assoc`]): [`Live::set_bssid`] before
//! authentication, [`Live::associate`] at the association response,
//! [`Live::leave`] afterwards, or after a join that never associated. Their
//! H2Cs go through the command session without a wait; frames drained
//! meanwhile are held for the next [`Live::drain`]. Each is refused while a
//! scan has not been ended, so no drain outside the scan's tracking can
//! take its END_SCAN; and a scan is refused while a BSSID is set
//! ([`CONNECTED`]), Linux's connected-or-connecting test
//! (`rtw89_is_any_vif_connected_or_connecting`, `fw.c:10514-10529`): the
//! connected scan is not ported, and the restore's address CAM would clear
//! the BSSID entry. DIG's pause ([`Dig`]) is kept across all of them, as
//! Linux's `pause_dig` is. A failure after the first access powers the chip
//! off: the firmware's role is left half programmed.
//!
//! Frames go out on the data and management queues as `rtw89_ops_tx` hands
//! them to `rtw89_core_tx_write` (`core.c:1453-1478`): the WD head of
//! [`txd`], submitted on the TX channel the encoder names ([`super::pci::Txq::submit`]),
//! [`Live::queue_management`] and [`Live::queue_data`]. A full channel is
//! "no room", and so is any frame while a scan has not been ended, as
//! `rtw89_hw_scan_start` stops mac80211's queues until
//! `rtw89_hw_scan_complete` wakes them (`fw.c:10419`, `:10460`). Each frame
//! is named by a [`Token`] until its release report. [`Live::service`] is
//! the radio's poll, in `rtw89_pci_napi_poll`'s order (`pci.c:4528-4534`):
//! RPQ's release reports (`rtw89_pci_poll_rpq_dma`, `pci.c:720`), each
//! returning its page as `rtw89_pci_release_rpp` does and naming its token,
//! then the TX reclaim of every channel with TXBDs outstanding
//! (`rtw89_pci_reclaim_txbd`, `pci.c:501-520`), then [`Live::drain`], then
//! [`Live::expire_scan`]. A report the pools cannot take (a seq past them,
//! a qsel without a channel, a page not in flight) is counted, never
//! reported. The live chip is the station's [`Chip`] (M3 spec S14.1).
//!
//! The associated station's keys are operations too (M3 spec S12, each step
//! from [`keys`]): [`Live::add_key`] installs one in the security CAM and
//! points the address CAM at it, waiting for the firmware's receipt and
//! acknowledgement, and [`Live::remove_key`] deletes one; [`Live::leave`]
//! deletes what is left first. A refused key leaves the chip on. A data
//! frame goes under a key as its TX descriptor names the key's CAM entry,
//! key ID and host PN ([`Live::queue_data`]), only an entry
//! [`Live::add_key`] installed ([`Chip::transmit_data`]). [`Live::flush`]
//! is `rtw89_ops_flush`: the TX rings' wait, then with drop the PKT_DROP
//! H2Cs for the AP's station, without it the wait for the MAC's TX queues.
//!
//! The associated station's RX Block Ack agreements are operations too (M3
//! spec S13.2, each step from [`ba`]): [`Live::start_rx_block_ack`] and
//! [`Live::stop_rx_block_ack`] are `rtw89_ops_ampdu_action`'s RX_START and
//! RX_STOP, a static BA CAM entry and its H2C, sent without a wait as
//! Linux's, and the TID's PN check started or stopped; [`Live::leave`]
//! stops what is left before the key deletes, as mac80211 tears the
//! agreements down first. Every frame the chip drains passes
//! `rtw89_core_skb_pn_valid` as it is drained, by [`Live::drain`] or by an
//! operation that holds it, as `rtw89_core_rx` runs it on reception before
//! mac80211 (`core.c:4551`), and is not checked again when handed on; so a
//! pairwise key's install starts the started TIDs' checks over
//! (`mac80211.c:1007-1008`) after the frames its own waits drained.
//!
//! The beacon filter the association arms reports the AP's beacons lost
//! (M3 spec S15, ruling R2): [`Live::service`] takes each BCNFLTR_RPT as
//! `rtw89_mac_bcn_fltr_rpt` does, a report an operation drained held for
//! the next poll as its frames are. A loss for the associated vif's MAC ID
//! re-arms the filter, and, unless a scan has not been ended, is handed on
//! as [`Incoming::BeaconLost`]; anything else is ignored.
//!
//! This module grants no TX status beyond RPQ's: a report says the MAC is
//! done with a frame, acknowledged or not, nothing more; a flush grants no
//! empty queue either, as Linux's waits only log their timeouts. It
//! encrypts no management frame (a robust one is sealed on the host, spec
//! S1.2). It runs no IQK, DPK, TSSI, watchdog, interrupts, probe
//! request or idle power save, and no CQM: the beacon filter's RSSI reports
//! are ignored, and a beacon loss is the AP gone, not a reason to probe it
//! (`ieee80211_mgd_probe_ap`). It has no `Drop`: a live chip that is
//! dropped keeps running, its rings and pools quarantined, as rtw88's
//! `Hardware` keeps running for the boot.
use super::assoc::{self, AddrCam, AssocInputs, Features, RaPeer};
use super::ba::{self, BaCam, RxStats};
use super::bb::BbGain;
use super::bringup::{self, BringUpError, Calibrated, FirmwareFile, Host, Running};
use super::c2h::{BCNFLTR_BEACON_LOSS, BcnFltr, C2h, RaReport, SCAN_END};
use super::chan::{Chan, frequency};
use super::channel::{self, ChannelError, ChannelInputs, TxPower};
use super::command::{Parked, Session};
use super::core_tail;
use super::dm::Dig;
use super::efuse::RxGain;
use super::h2c::{H2c, ScanChannel};
use super::io::{AxError, AxIo, poll32};
use super::keys::{self, KeyCam};
use super::pci::{Rings, TX_DONE, TxChannel, TxPools, WD_HEAD_BYTES, releases};
use super::receive::{Event, MAX_PENDING, Mode, Received, Tag};
use super::regd::Regds;
use super::rx::decode_rate;
use super::scan::{self, EdccaBak, ScanEnd, ScanError, hw_band, wifi_band};
use super::station::chip::{
    self, Association, Chip, Data, HardwareKey, Incoming, LinkCounts, Management, Token, TxCounts,
};
use super::txd::{self, RateCaps, TxSec};
use super::txpwr_tables::TxPowerTables;
use super::vif;
use crate::Mac;
use crate::bus::dma::{mapping::DmaDomain, memory::PageSource};
use crate::wifi::Channel;
use alloc::collections::VecDeque;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

/// The channel `rtw89_leave_ips` sets from idle: an unassociated channel
/// context is the default chandef, 2.4 GHz channel 1 (M2c spec S1.2).
pub const BOOT_CHANNEL: Tag = Tag {
    band: super::chan::HwBand::G2,
    channel: 1,
};

/// What every call on a [`Live`] chip returns once a failed channel switch,
/// scan start or scan restore powered it off.
pub const POWERED_OFF: AxError =
    AxError::Phy("the chip was powered off after a failed channel switch or scan");

/// What a scan, a channel switch, [`scan::run_once`] or an association step
/// gets while the last scan has not been ended with [`Live::end_scan`].
pub const SCAN_NOT_ENDED: AxError = AxError::Scan("the last scan has not been ended");

/// What a scan or [`scan::run_once`] gets while a BSSID is set
/// ([`Live::set_bssid`] until [`Live::leave`]).
pub const CONNECTED: AxError =
    AxError::Scan("a BSSID is set: the station is connecting or connected");

/// The most frames, and the most beacon filter reports, a [`Live`] holds
/// for the next [`Live::drain`], as the receiver's own queue is bounded;
/// past it they are dropped and counted ([`Live::held_dropped`]).
const MAX_HELD: usize = MAX_PENDING;

/// What [`Chip::transmit_data`] gets for a frame naming a security CAM
/// entry that holds no key [`Live::add_key`] installed.
pub const KEY_NOT_IN_CAM: &str = "a key not in the security CAM";

/// `R_AX_DLE_EMPTY0` (`reg.h:631`), and the queues `dle_is_txq_empty`
/// wants empty in it (`mac.c:2393-2405`, bits `reg.h:632-651`).
const R_AX_DLE_EMPTY0: u32 = 0x8430;
const DLE_TXQ_EMPTY: u32 = 0x07FF_079F;
/// `rtw89_mac_flush_txq`'s `read_poll_timeout`: every 10 ms for up to
/// 200 ms (`mac.c:6360-6361`).
const FLUSH_TXQ_STEP_US: u64 = 10_000;
const FLUSH_TXQ_WAIT_US: u64 = 200_000;
/// `rtw89_mac_pkt_drop_sta`'s selections (`mac.c:7259-7264`),
/// `RTW89_PKT_DROP_SEL_MACID_BE_ONCE` to `_VO_ONCE` (`core.h:6485-6488`).
const PKT_DROP_SELS: [u8; 4] = [0, 1, 2, 3];

/// The data and management channels, in the order the reclaim reads them.
const TX_CHANNELS: [TxChannel; 5] = [
    TxChannel::Ach0,
    TxChannel::Ach1,
    TxChannel::Ach2,
    TxChannel::Ach3,
    TxChannel::Ch8,
];

/// What the TX side did, saturating, for [`Chip::diagnostic`]: frames
/// queued, refused for no room, reported, reported acknowledged; release
/// reports skipped (`Releases::invalid`) and stale (for a page not in
/// flight or already reported). [`Live::tx_counts`] counts by channel.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct TxTally {
    sent: u32,
    full: u32,
    reported: u32,
    acked: u32,
    skipped: u32,
    stale: u32,
}

/// The RX Block Ack starts and stops the chip took, saturating, for
/// [`Chip::diagnostic`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct BaCounts {
    started: u32,
    stopped: u32,
}

fn bump(counter: &mut u32) {
    *counter = counter.saturating_add(1);
}

/// What [`Live::start_rx_block_ack`] gets while not associated.
pub const BA_WITHOUT_ASSOCIATION: AxError =
    AxError::Mac("a Block Ack agreement without an association");

/// An RTL8852CE that [`start`] left running (M3 spec S3.1).
pub struct Live<I: AxIo> {
    io: I,
    rings: Rings,
    pools: TxPools,
    h2c: H2c,
    /// The command session's receiver and ledger between two sessions;
    /// `None` only while [`Live::session`] runs one.
    parked: Option<Parked>,
    running: Running,
    host: Host,
    gain: BbGain,
    rx_gain: RxGain,
    offset_base: i8,
    tables: TxPowerTables,
    regds: (Regds, [u8; 2]),
    /// The channel the last switch set.
    chan: Chan,
    /// The channel context's chandef, which a scan's restore returns to:
    /// the last switch's, until [`Live::release_channel`] makes it the
    /// default chandef.
    home: Chan,
    /// The firmware scan begun, until [`Live::end_scan`] restores after it.
    scan: Option<LiveScan>,
    /// Frames drained outside [`Live::drain`] and [`Live::expire_scan`].
    held: Held,
    /// `rtwvif_link->bssid`, from [`Live::set_bssid`] until [`Live::leave`].
    bssid: Option<[u8; 6]>,
    /// The AP's station is AUTH or past it, from [`Live::authenticated`]
    /// until [`Live::leave`] or the next [`Live::set_bssid`].
    authenticated: bool,
    /// The association made, until [`Live::leave`].
    associated: Option<AssocInputs>,
    /// DIG's pause.
    dig: Dig,
    /// The security CAM and the station's address CAM key fields, empty
    /// unless associated with keys installed.
    keys: KeyCam,
    /// The static BA CAM entries the associated station's agreements hold.
    ba: BaCam,
    /// The AP's station's `tid_rx_stats`, for the PN check.
    rx_stats: RxStats,
    tx: TxTally,
    /// [`Live::tx_counts`].
    counts: TxCounts,
    ba_counts: BaCounts,
    /// A failure powered the chip off.
    off: bool,
}

/// A firmware scan on a [`Live`] chip.
struct LiveScan {
    /// The EDCCA levels the scan start raised.
    edcca: EdccaBak,
    /// SCANOFLD start's DONE_ACK plus the list's budget.
    deadline_us: u64,
    /// The first END_SCAN's status.
    status: Option<u8>,
    /// How the scan ended, once what followed END_SCAN, or the stop, was
    /// drained.
    end: Option<ScanEnd>,
    /// [`Live::service`] handed its end on.
    announced: bool,
}

impl LiveScan {
    /// Record the first END_SCAN of a scan that has not ended.
    fn track(&mut self, event: &Event<'_>) {
        if let Event::C2h(C2h::ScanOfld(report)) = event
            && report.reason == SCAN_END
            && self.status.is_none()
            && self.end.is_none()
        {
            self.status = Some(report.status);
        }
    }
}

/// What [`Held`] keeps: a frame, a beacon filter report, or an RA report.
enum Kept {
    Frame(Received),
    Report(BcnFltr),
    Rate(RaReport),
}

/// Frames, beacon filter reports and RA reports drained outside
/// [`Live::drain`] and [`Live::expire_scan`], oldest first, for the next
/// [`Live::drain`] to hand out, at most [`MAX_HELD`] of each; past it they are
/// dropped and counted. Other C2Hs are not held: the ledger counts them.
#[derive(Default)]
struct Held {
    kept: VecDeque<Kept>,
    frames: usize,
    reports: usize,
    rates: usize,
    dropped: u16,
}

impl Held {
    fn keep(&mut self, event: Event<'_>) {
        let (kept, count) = match event {
            Event::Frame(frame) => (Kept::Frame(frame), &mut self.frames),
            Event::C2h(C2h::BeaconFilter(report)) => (Kept::Report(report), &mut self.reports),
            Event::C2h(C2h::RaReport(report)) => (Kept::Rate(report), &mut self.rates),
            Event::C2h(_) => return,
        };
        if *count < MAX_HELD {
            *count += 1;
            self.kept.push_back(kept);
        } else {
            self.dropped = self.dropped.saturating_add(1);
        }
    }

    /// The oldest kept, as the event it was.
    fn next(&mut self) -> Option<Event<'static>> {
        Some(match self.kept.pop_front()? {
            Kept::Frame(frame) => {
                self.frames -= 1;
                Event::Frame(frame)
            }
            Kept::Report(report) => {
                self.reports -= 1;
                Event::C2h(C2h::BeaconFilter(report))
            }
            Kept::Rate(report) => {
                self.rates -= 1;
                Event::C2h(C2h::RaReport(report))
            }
        })
    }
}

/// A failed [`start`]: the step and its error, the register window, and the
/// rings once they were installed. The chip has been powered off; the rings
/// must be kept, as nothing proves the DMA stopped.
pub struct StartError<I> {
    pub error: BringUpError,
    pub io: I,
    pub rings: Option<Rings>,
}

/// `rtw89_leave_ips` (`ps.c:257-278`) after the bring-up's
/// `rtw89_phy_dm_init`, on `session` (M3 spec S2 steps 1-3):
/// 1. `"core start"`: the rest of `rtw89_core_start` ([`core_tail::run`]);
/// 2. `"set channel"` and `"tx power"`: `rtw89_set_channel` to `chan`,
///    `set_txpwr` between its RF part and its help's exit
///    ([`channel::set_channel`]);
/// 3. `"vif init"`: `rtw89_mac_vif_init` for `mac` ([`vif::init`]).
///
/// The first failure is returned and nothing after it runs; the caller
/// powers the chip off.
///
/// # Safety
/// The contract of `Ch12::submit` and `Rxq::poll` for `io` and the
/// session's rings: `io` is the exclusive register window of the same
/// RTL8852C requester that `pci::pre_init` programmed with them, `pre_init`
/// succeeded, and its contract still holds.
pub unsafe fn leave_ips<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    mac: [u8; 6],
    chan: &Chan,
    inputs: &ChannelInputs<'_>,
    power: TxPower<'_>,
) -> Result<(), ScanError> {
    let at = |step: &'static str| move |error: AxError| ScanError { step, error };
    // SAFETY: the caller's contract is `core_tail::run`'s, word for word.
    unsafe { core_tail::run(io, session) }.map_err(at("core start"))?;
    channel::set_channel(io, chan, inputs, power).map_err(|e| at(e.step)(e.error))?;
    // SAFETY: as above, `vif::init`'s.
    unsafe { vif::init(io, session, mac) }.map_err(at("vif init"))
}

/// Bring an RTL8852CE of cut `cut` up on `firmware` for `host` and keep it
/// running (M3 spec S2-S3): the bring-up through `rtw89_phy_dm_init`
/// ([`super::bringup::bring_up`]'s steps 1-14, with the file's TX power
/// tables and the regulatory index of the efuse country, else
/// `network_country`, else the world's), then [`leave_ips`] on 2.4 GHz
/// channel 1 with one command session from the sequence state the RF pages
/// left (seq 3), then `"tx rings"`: the TX pools ([`TxPools::install`]), new
/// for this start, as pages in flight from an earlier run can never return.
/// The live chip's receiver then keeps every frame a station receives
/// ([`Mode::Station`]), tagged with the channel set; the H2C sequence goes
/// on at seq 17.
///
/// On a failure the chip is powered off as [`super::bringup::bring_up`]
/// powers it off for that step (a failure from `"core start"` on as a
/// calibration failure is, `bringup::shut_down`), and the window and the
/// rings come back in the error.
///
/// # Safety
/// [`super::bringup::bring_up`]'s contract, for as long as the live chip is
/// used: `io` is the exclusive register window of exactly the RTL8852C PCI
/// requester that `domain` isolates; `source` yields real coherent WB DMA
/// RAM for that requester, with NoSnoop and ATS disabled; one worker owns
/// the device and its queues, and no other engine, IRQ or reset worker
/// touches them.
#[expect(
    clippy::result_large_err,
    reason = "A failure hands the window and the rings back by value: the rings are never freed, as nothing proves the DMA stopped."
)]
pub unsafe fn start<I: AxIo>(
    mut io: I,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    cut: u8,
    firmware: FirmwareFile<'_>,
    host: Host,
    network_country: Option<[u8; 2]>,
) -> Result<Live<I>, StartError<I>> {
    let pools_domain = domain.clone();
    // SAFETY: the caller's contract is `power_up`'s, word for word.
    let up = unsafe {
        bringup::power_up(
            &mut io,
            source,
            domain,
            cut,
            firmware,
            host,
            network_country,
        )
    };
    let (calibrated, mut rings) = match up {
        Ok(up) => up,
        Err((error, rings)) => return Err(StartError { error, io, rings }),
    };
    let Calibrated {
        running,
        mut h2c,
        gain,
        rx_gain,
        offset_base,
        tables,
        regds,
    } = calibrated;
    // Every failure from here on follows the calibration, so it keeps RF
    // 0x18 as the bring-up's do.
    let failed = |step: &'static str, error: AxError| BringUpError {
        rf18: running.rf18,
        ..BringUpError::new(step, error)
    };
    let inputs = ChannelInputs {
        gain: &gain,
        rx_gain: &rx_gain,
        offset_base,
        cut,
    };
    let power = TxPower {
        tables: &tables,
        regds: regds.0,
    };
    let left = match Chan::new20(BOOT_CHANNEL.band, BOOT_CHANNEL.channel) {
        Ok(chan) => {
            let mut session = Session::new(&mut rings, &mut h2c, BOOT_CHANNEL);
            // SAFETY: the caller's contract is `leave_ips`'s: `power_up`
            // programmed these rings through this window with a
            // `pci::pre_init` that succeeded, and its OFLD_CFG wait and radio
            // steps submitted and polled on them since.
            let left =
                unsafe { leave_ips(&mut io, &mut session, running.mac, &chan, &inputs, power) };
            left.map(|()| (chan, session.park()))
        }
        Err(error) => Err(ScanError {
            step: "set channel",
            error,
        }),
    };
    let left = left.map_err(|e| failed(e.step, e.error)).and_then(|left| {
        // SAFETY: the caller's `Rings::install` contract, which is
        // `TxPools::install`'s, for the rings' own requester and domain.
        let pools = unsafe { TxPools::install(source, pools_domain) };
        pools
            .map(|pools| (left, pools))
            .map_err(|e| failed("tx rings", e))
    });
    let ((chan, mut parked), pools) = match left {
        Ok(left) => left,
        Err(error) => {
            bringup::shut_down(&mut io, &rings, host);
            return Err(StartError {
                error,
                io,
                rings: Some(rings),
            });
        }
    };
    parked.rx.set_mode(Mode::Station);
    parked.rx.set_tag(tag_of(&chan));
    Ok(Live {
        io,
        rings,
        pools,
        h2c,
        parked: Some(parked),
        running,
        host,
        gain,
        rx_gain,
        offset_base,
        tables,
        regds,
        chan,
        home: chan,
        scan: None,
        held: Held::default(),
        bssid: None,
        authenticated: false,
        associated: None,
        dig: Dig::default(),
        keys: KeyCam::default(),
        ba: BaCam::default(),
        rx_stats: RxStats::default(),
        tx: TxTally::default(),
        counts: TxCounts::default(),
        ba_counts: BaCounts::default(),
        off: false,
    })
}

/// The receiver's tag for frames on `chan`.
fn tag_of(chan: &Chan) -> Tag {
    Tag {
        band: chan.band,
        channel: chan.channel,
    }
}

/// `first`, unless it is `Ok` and `next` failed: the first error wins.
fn keep(first: &mut Result<(), AxError>, next: Result<(), AxError>) {
    if first.is_ok() {
        *first = next;
    }
}

/// The token of WD page `page` of `ch`: its ring in bits 15:8.
fn token(ch: TxChannel, page: u8) -> Token {
    Token((ch.ring() as u16) << 8 | u16::from(page))
}

/// A received frame as the station hears it: the tag as a channel at its
/// cfg80211 frequency, the free-running counter as its TSF, the
/// descriptor's rate decoded ([`decode_rate`]) with its A-MPDU bit.
fn heard(frame: Received) -> chip::Received {
    let Tag { band, channel } = frame.tag;
    chip::Received {
        bytes: frame.mpdu,
        channel: Channel {
            band: wifi_band(band),
            number: u16::from(channel),
            frequency_mhz: frequency(band, channel),
        },
        signal_dbm: frame.signal,
        tsf: frame.free_run,
        decrypted: frame.decrypted,
        icv_error: frame.icv_error,
        fcs_error: frame.fcs_error,
        rate: Some(decode_rate(frame.data_rate, frame.bw, frame.gi_ltf)),
        ampdu: frame.ampdu,
    }
}

/// `channel` as a 20 MHz [`Chan`]: refused, as [`Chan::new20`] refuses, for
/// a number past 255, one that is no 2.4 or 5 GHz channel, and a frequency
/// that is not its number's.
fn chan_of(channel: Channel) -> Result<Chan, AxError> {
    const NOT_20: AxError = AxError::Phy("not a 20 MHz 2.4 or 5 GHz channel");
    let number = u8::try_from(channel.number).map_err(|_| NOT_20)?;
    let chan = Chan::new20(hw_band(channel.band), number)?;
    if chan.freq != channel.frequency_mhz {
        return Err(NOT_20);
    }
    Ok(chan)
}

/// A failure of `step` as [`Chip`] reports it.
fn failed(step: &str, error: AxError) -> String {
    format!("{step}: {error}")
}

/// Whether an RA report goes to the station: for its MAC ID
/// (`__rtw89_phy_c2h_ra_rpt_iter` matches `rtwsta_link->mac_id`,
/// `phy.c:3247-3249`), and kept as Linux keeps one ([`RaReport::valid`]).
fn station_rate(report: &RaReport) -> bool {
    report.macid == u16::from(vif::MAC_ID) && report.valid()
}

/// Whether `event` goes on from the drain that brought it: a frame only
/// when `rtw89_core_skb_pn_valid` passes it ([`RxStats::valid`] under
/// `stats`, `associated` saying whether MAC ID 0 is the associated
/// station's), as `rtw89_core_rx` runs it on reception (`core.c:4551`),
/// whichever drain brought it and whatever the operation then changes;
/// every C2H.
fn checked(stats: &mut RxStats, associated: bool, event: &Event<'_>) -> bool {
    match event {
        Event::Frame(frame) => stats.valid(associated, frame),
        Event::C2h(_) => true,
    }
}

impl<I: AxIo> Live<I> {
    /// What the bring-up reported (M2b2's report, no scan).
    pub fn running(&self) -> &Running {
        &self.running
    }

    /// The efuse MAC address.
    pub fn mac(&self) -> [u8; 6] {
        self.running.mac
    }

    /// The TX power's regulatory index and the alpha2 it was found by
    /// (`00` for the world's).
    pub fn regds(&self) -> (Regds, [u8; 2]) {
        self.regds
    }

    /// What the TX side did by channel (M4 spec S3.2): each frame a submit
    /// took, each write of a ring's index, each release report and those
    /// not `RTW89_TX_DONE`, and the RPQ polls that found RPQ full
    /// ([`super::pci::Rings::rpq_full_batches`]).
    pub fn tx_counts(&self) -> TxCounts {
        TxCounts {
            rpq_full: u32::from(self.rings.rpq_full_batches()),
            ..self.counts
        }
    }

    /// The channel the last switch set: [`start`]'s 2.4 GHz channel 1, then
    /// each [`Live::tune`]'s. A firmware scan takes the radio off it until
    /// [`Live::end_scan`] switches back to it, or to 2.4 GHz channel 1 after
    /// [`Live::release_channel`].
    pub fn chan(&self) -> Chan {
        self.chan
    }

    /// Whether a firmware scan was begun and not yet ended with
    /// [`Live::end_scan`].
    pub fn scanning(&self) -> bool {
        self.scan.is_some()
    }

    /// How the firmware scan ended: `None` while it runs, and when none was
    /// begun since the last [`Live::end_scan`].
    pub fn scan_state(&self) -> Option<ScanEnd> {
        self.scan.as_ref().and_then(|scan| scan.end)
    }

    /// The frames, and the beacon filter reports, drained outside
    /// [`Live::drain`] and [`Live::expire_scan`] past the 64 of each it holds
    /// for the next [`Live::drain`], which were dropped (saturating).
    pub fn held_dropped(&self) -> u16 {
        self.held.dropped
    }

    /// The BSSID [`Live::set_bssid`] set, until [`Live::leave`].
    pub fn bssid(&self) -> Option<[u8; 6]> {
        self.bssid
    }

    /// Whether [`Live::associate`] associated the station and it has not
    /// left.
    pub fn associated(&self) -> bool {
        self.associated.is_some()
    }

    /// Refused with [`POWERED_OFF`] on a chip a failure powered off, and
    /// with [`SCAN_NOT_ENDED`] while a scan was begun and not ended.
    pub(super) fn idle(&self) -> Result<(), AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        if self.scan.is_some() {
            return Err(SCAN_NOT_ENDED);
        }
        Ok(())
    }

    /// Refused with [`CONNECTED`] while a BSSID is set.
    pub(super) fn unconnected(&self) -> Result<(), AxError> {
        match self.bssid {
            Some(_) => Err(CONNECTED),
            None => Ok(()),
        }
    }

    /// DIG's pause, for [`scan::run_once`]'s scan start.
    pub(super) fn dig(&self) -> Dig {
        self.dig
    }

    /// DIG's pause after [`scan::run_once`]'s scan start.
    pub(super) fn set_dig(&mut self, dig: Dig) {
        self.dig = dig;
    }

    /// `__rtw89_set_channel` to `chan` ([`channel::set_channel`]): the TX
    /// scheduler stopped, the MAC, BB and RF parts, `set_txpwr` from the
    /// file's tables at [`Live::regds`], then the help's exit; then the
    /// receiver tags frames with `chan`. Refused before any I/O, changing
    /// nothing, on a chip already off ([`POWERED_OFF`]), while a scan has not
    /// been ended ([`SCAN_NOT_ENDED`]: the firmware moves the radio), and for
    /// a channel [`Chan::new20`] would not build. Any later failure leaves
    /// the TX scheduler stopped, so the chip is powered off
    /// (`bringup::shut_down`) and every later call fails with
    /// [`POWERED_OFF`]. The channel becomes the one a scan's restore
    /// returns to.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn tune(&mut self, chan: &Chan) -> Result<(), AxError> {
        // SAFETY: the caller's contract is `switch`'s, word for word.
        unsafe { self.switch(chan) }.map_err(|failed| failed.error)
    }

    /// [`Live::tune`], its failure with its step: `"set channel"` for a
    /// refusal and for every step of the switch but `set_txpwr`'s, `"tx
    /// power"` for that.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    unsafe fn switch(&mut self, chan: &Chan) -> Result<(), ChannelError> {
        let refused = |error| ChannelError {
            step: "set channel",
            error,
        };
        self.idle().map_err(refused)?;
        channel::check_chan(chan).map_err(refused)?;
        let inputs = ChannelInputs {
            gain: &self.gain,
            rx_gain: &self.rx_gain,
            offset_base: self.offset_base,
            cut: self.running.cut,
        };
        let power = TxPower {
            tables: &self.tables,
            regds: self.regds.0,
        };
        if let Err(failed) = channel::set_channel(&mut self.io, chan, &inputs, power) {
            self.power_off();
            return Err(failed);
        }
        self.chan = *chan;
        self.home = *chan;
        if let Some(parked) = self.parked.as_mut() {
            parked.rx.set_tag(tag_of(chan));
        }
        Ok(())
    }

    /// Begin a firmware scan of `list` (M3 spec S8.1): `rtw89_hw_scan_start`'s
    /// chip steps (`scan::hw_scan_start`), then the scan's receiver
    /// ([`Mode::Scan`], see the module documentation), into which the frames
    /// still waiting for their PPDU status, received before the scan, are
    /// flushed for the next [`Live::drain`]; then `rtw89_hw_scan_offload(true)`
    /// (`scan::offload`: ADD_SCANOFLD_CH with `list`, SCANOFLD start, each
    /// with its 100 ms DONE_ACK wait). The scan's budget,
    /// [`scan::scan_budget_us`] of the list, runs from SCANOFLD start's
    /// DONE_ACK ([`Live::expire_scan`]). Frames drained meanwhile are held
    /// for the next [`Live::drain`], END_SCAN recorded.
    ///
    /// Refused before any I/O, changing nothing: on a chip off
    /// ([`POWERED_OFF`]), while the last scan has not been ended
    /// ([`SCAN_NOT_ENDED`]), while a BSSID is set ([`CONNECTED`]), for a list
    /// [`scan::scan_list`] would refuse, and on a firmware without
    /// SCAN_OFFLOAD. A failure in the scan start's own steps powers the chip
    /// off. A failed offload is aborted as Linux aborts it: `scan::stop`,
    /// its failure ignored, then the restore of [`Live::end_scan`]; its error
    /// is returned, and the chip stays on unless the restore failed.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn begin_scan(&mut self, list: &[ScanChannel]) -> Result<(), AxError> {
        self.idle()?;
        self.unconnected()?;
        scan::check_list(list)?;
        scan::check_firmware(self.running.firmware)?;
        let mac = self.running.mac;
        let budget_us = scan::scan_budget_us(list.len());
        let mut dig = self.dig;
        let started = self.holding(|io, session, held| {
            // SAFETY: the caller's contract is `start`'s, which
            // `hw_scan_start`'s is, for the live chip's own window and rings.
            let edcca = unsafe { scan::hw_scan_start(io, session, mac, &mut dig, held) }?;
            session.rx.set_mode(Mode::Scan);
            session.rx.flush(held);
            let mut scan = LiveScan {
                edcca,
                deadline_us: u64::MAX,
                status: None,
                end: None,
                announced: false,
            };
            // SAFETY: as above, `offload`'s.
            let offloaded = unsafe {
                scan::offload(io, session, list, &mut |e| {
                    scan.track(&e);
                    held(e)
                })
            }
            .and_then(|()| io.now_us());
            Ok((scan, offloaded))
        });
        self.dig = dig;
        let (mut scan, offloaded) = match started {
            Ok(Ok(started)) => started,
            Ok(Err(error)) => {
                self.power_off();
                return Err(error);
            }
            Err(off) => return Err(off),
        };
        match offloaded {
            Ok(now_us) => {
                scan.deadline_us = now_us.saturating_add(budget_us);
                self.scan = Some(scan);
                Ok(())
            }
            Err(error) => {
                self.scan = Some(scan);
                // SAFETY: the caller's contract is `abort`'s.
                unsafe { self.abort() };
                Err(error)
            }
        }
    }

    /// One RXQ poll for `sink` (the RXQ step of the radio's poll): first the
    /// frames and beacon filter reports an operation held, oldest first
    /// (see [`Live::service`]), then every RXQ entry through the receiver,
    /// the session's ledger and `rtw89_core_skb_pn_valid` (`checked`: a
    /// frame it drops is counted, not handed on), as `rtw89_pci_poll_rxq_dma`
    /// hands them to `rtw89_core_rx`; a held frame passed that check as it
    /// was drained. A running
    /// scan's first END_SCAN ends it: the same call then drains what
    /// followed END_SCAN and flushes the receiver into `sink`
    /// (`scan::tail`), so every frame the scan heard is handed on before
    /// [`Live::scan_state`] says how it ended. A malformed buffer ends the
    /// drain with its error after the buffers before it were handed on, and
    /// a later call goes on. Refused with [`POWERED_OFF`] on a chip off.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn drain(&mut self, sink: &mut dyn FnMut(Event<'_>)) -> Result<(), AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        self.release(sink);
        let mut scan = self.scan.take();
        let drained = self.passing(sink, |io, session, sink| {
            // SAFETY: the caller's contract is `start`'s, which `drain`'s
            // is, for the live chip's own window and rings.
            unsafe {
                session.drain(io, &mut |e| {
                    if let Some(scan) = scan.as_mut() {
                        scan.track(&e);
                    }
                    sink(e)
                })
            }?;
            if let Some(scan) = scan.as_mut()
                && scan.end.is_none()
                && let Some(status) = scan.status
            {
                // SAFETY: as above, `tail`'s.
                let tail = unsafe { scan::tail(io, session, sink) };
                // The scan is over whether or not its tail could be read.
                scan.end = Some(scan::ended(status));
                tail?;
            }
            Ok(())
        });
        self.scan = scan;
        drained.and_then(|drained| drained)
    }

    /// The scan budget (the radio poll's last step, M3 spec S8.1): a scan
    /// without END_SCAN at its deadline is stopped as `rtw89_hw_scan_abort`
    /// does (`scan::stop`: SCANOFLD stop, then up to `STOP_WAIT_US` for
    /// END_SCAN), what followed drained into `sink` (`scan::tail`) as
    /// [`Live::drain`] drains it, after the frames held, and it ends
    /// [`ScanEnd::Stopped`], `ended` when
    /// END_SCAN came in the stop's wait or its tail. A stop or tail that
    /// fails still ends it so, as Linux completes an abort whatever the
    /// stop returned (`fw.c:10502-10510`), and returns its error; the stop
    /// is never sent twice. Anything else changes nothing and does no I/O
    /// but the clock read: no scan, one that ended or brought END_SCAN, one
    /// within its budget. Refused with [`POWERED_OFF`] on a chip off.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn expire_scan(&mut self, sink: &mut dyn FnMut(Event<'_>)) -> Result<(), AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        let deadline_us = match &self.scan {
            Some(scan) if scan.status.is_none() && scan.end.is_none() => scan.deadline_us,
            _ => return Ok(()),
        };
        if self.io.now_us()? < deadline_us {
            return Ok(());
        }
        self.release(sink);
        let mut scan = self.scan.take();
        let stopped = self.passing(sink, |io, session, sink| {
            let mut track = |e: Event<'_>| {
                if let Some(scan) = scan.as_mut() {
                    scan.track(&e);
                }
                sink(e)
            };
            // SAFETY: the caller's contract is `start`'s, which `stop`'s is,
            // for the live chip's own window and rings.
            let stop = unsafe { scan::stop(io, session, &mut track) };
            let (ended, result) = match stop {
                Ok(ended) => {
                    // SAFETY: as above, `tail`'s.
                    let tail = unsafe { scan::tail(io, session, &mut track) };
                    (ended, tail.map(|_| ()))
                }
                Err(error) => (false, Err(error)),
            };
            if let Some(scan) = scan.as_mut() {
                let ended = ended || scan.status.is_some();
                scan.end = Some(ScanEnd::Stopped { ended });
            }
            result
        });
        self.scan = scan;
        stopped.and_then(|stopped| stopped)
    }

    /// `rtw89_hw_scan_complete` for the ended scan (M3 spec S8.3; see the
    /// module documentation): the switch back to the operating channel through
    /// [`Live::tune`], with its TX power and the receiver's tag, then
    /// `scan::hw_scan_complete` (the RX filter, the station receiver
    /// again, the address CAM, EDCCA, TSF sync, DIG), frames its CAM send
    /// drains held for the next [`Live::drain`]. The scan is over either way;
    /// a failure powers the chip off. Refused before any I/O on a chip off
    /// ([`POWERED_OFF`]), without a scan, and while the scan runs.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn end_scan(&mut self) -> Result<(), AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        let edcca = match &self.scan {
            None => return Err(AxError::Scan("no scan to end")),
            Some(LiveScan { end: None, .. }) => {
                return Err(AxError::Scan("the scan is still running"));
            }
            Some(scan) => scan.edcca,
        };
        self.scan = None;
        // SAFETY: the caller's contract is `restore`'s.
        unsafe { self.restore(&edcca) }
    }

    /// [`scan::run_once`]'s scan, ended as `end`, with the EDCCA levels its
    /// start raised, for [`Live::end_scan`].
    pub(super) fn scanned(&mut self, edcca: EdccaBak, end: ScanEnd) {
        self.scan = Some(LiveScan {
            edcca,
            deadline_us: 0,
            status: None,
            end: Some(end),
            announced: false,
        });
    }

    /// `rtw89_ops_hw_scan`'s abort of a failed offload: [`scan::stop`], whose
    /// failure `rtw89_hw_scan_abort` only warns of (`fw.c:10502-10504`), then
    /// the restore, whose failure powers the chip off and is left for the
    /// next call's [`POWERED_OFF`].
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    unsafe fn abort(&mut self) {
        // SAFETY: the caller's contract is `start`'s, which `stop`'s is, for
        // the live chip's own window and rings.
        let _ = self.holding(|io, session, held| unsafe { scan::stop(io, session, held) });
        if let Some(scan) = self.scan.take() {
            // SAFETY: as above, `restore`'s.
            let _ = unsafe { self.restore(&scan.edcca) };
        }
    }

    /// [`Live::end_scan`]'s restore, once the scan record is gone: the
    /// switch back, then [`scan::hw_scan_complete`]; a failure of the latter
    /// powers the chip off, as one of the switch does.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    unsafe fn restore(&mut self, edcca: &EdccaBak) -> Result<(), AxError> {
        let chan = self.home;
        // SAFETY: the caller's contract is `tune`'s, word for word.
        unsafe { self.tune(&chan) }?;
        let mac = self.running.mac;
        let mut dig = self.dig;
        let completed = self.holding(|io, session, held| {
            // SAFETY: as above, `hw_scan_complete`'s, for the live chip's own
            // window and rings.
            unsafe { scan::hw_scan_complete(io, session, mac, edcca, &mut dig, held) }
        });
        self.dig = dig;
        match completed {
            Ok(Ok(())) => Ok(()),
            Ok(Err(error)) => {
                self.power_off();
                Err(error)
            }
            Err(off) => Err(off),
        }
    }

    /// Set the BSSID before authentication (M3 spec S9.2-S9.3,
    /// [`assoc::set_bssid`]): its address CAM, then DIG suspended unless it
    /// already is. A station that has not associated may set another: the
    /// CAM follows the last, whose AP's station is new and not
    /// [`Live::authenticated`]. Refused before any I/O, changing nothing: on a
    /// chip off ([`POWERED_OFF`]), while a scan has not been ended
    /// ([`SCAN_NOT_ENDED`]), for a zero BSSID (Linux's "no BSSID"), and
    /// while associated (`Mac`). Any later failure powers the chip off.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn set_bssid(&mut self, bssid: [u8; 6]) -> Result<(), AxError> {
        self.idle()?;
        if bssid == [0; 6] {
            return Err(AxError::Mac("a zero BSSID is no BSSID"));
        }
        if self.associated.is_some() {
            return Err(AxError::Mac("a new BSSID while associated"));
        }
        let mac = self.running.mac;
        let mut dig = self.dig;
        let done = self.holding(|io, session, held| {
            // SAFETY: the caller's contract is `start`'s, which `set_bssid`'s
            // is, for the live chip's own window and rings.
            unsafe { assoc::set_bssid(io, session, mac, bssid, &mut dig, held) }
        });
        self.dig = dig;
        self.done(done)?;
        self.bssid = Some(bssid);
        self.authenticated = false;
        Ok(())
    }

    /// The AP of the BSSID set accepted the authentication: mac80211's
    /// NONE to AUTH (`__rtw89_ops_sta_state`, `mac80211.c:939-971`), which
    /// rtw89 programs nothing for. [`Live::leave`] then runs AUTH to NONE's
    /// disconnect even when the station never associated. No I/O. Refused,
    /// changing nothing, on a chip off ([`POWERED_OFF`]) and with `Mac`
    /// while no BSSID is set.
    pub fn authenticated(&mut self) -> Result<(), AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        if self.bssid.is_none() {
            return Err(AxError::Mac("an authentication without a BSSID set"));
        }
        self.authenticated = true;
        Ok(())
    }

    /// Associate the station with the BSS [`Live::set_bssid`] set (M3 spec
    /// S10, [`assoc::associate`]), with this chip's firmware features and
    /// DIG's pause; frames drained meanwhile are held for the next
    /// [`Live::drain`]. Refused before any I/O, changing nothing: on a chip
    /// off ([`POWERED_OFF`]), while a scan has not been ended
    /// ([`SCAN_NOT_ENDED`]), and with `Mac` when already associated, when
    /// `inputs` names another BSSID than the one set, another address than
    /// the chip's, or another band than the channel tuned (Linux takes it
    /// from the vif's channel context). EDCA parameters the station layer
    /// gives none of, or an invalid set of, are 802.11's QoS defaults
    /// ([`AssocInputs::params`]). Any later failure powers the chip off.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn associate(&mut self, inputs: &AssocInputs) -> Result<(), AxError> {
        self.idle()?;
        if self.associated.is_some() {
            return Err(AxError::Mac("already associated"));
        }
        if self.bssid != Some(inputs.bssid) {
            return Err(AxError::Mac("an association without its BSSID set"));
        }
        if inputs.mac != self.running.mac {
            return Err(AxError::Mac("an association for another address"));
        }
        if inputs.band != self.chan.band {
            return Err(AxError::Mac("an association on a band not tuned"));
        }
        let features = Features::of(self.running.firmware);
        let mut dig = self.dig;
        let done = self.holding(|io, session, held| {
            // SAFETY: the caller's contract is `start`'s, which `associate`'s
            // is, for the live chip's own window and rings.
            unsafe { assoc::associate(io, session, inputs, features, &mut dig, held) }
        });
        self.dig = dig;
        self.done(done)?;
        self.associated = Some(*inputs);
        Ok(())
    }

    /// Leave (M3 spec S11): from an association, first every RX Block Ack
    /// agreement's static BA CAM entry still held stopped as
    /// [`Live::stop_rx_block_ack`] stops it, oldest first, as mac80211 tears
    /// the AP station's agreements down before its keys
    /// (`__sta_info_destroy_part1`'s `ieee80211_sta_tear_down_BA_sessions`,
    /// `sta_info.c:1348`), then every key still
    /// installed deleted as [`Live::remove_key`] deletes it, in Stela's
    /// order, the group keys first ([`keys::KeyCam::deletion_order`]):
    /// mac80211 frees the AP station's keys there, before its ASSOC to AUTH
    /// (`sta_info.c:1577-1578`, `ieee80211_free_sta_keys`), but in station
    /// mode those are the PTK alone, the GTK being the vif's, and the radio
    /// deletes every key itself, the PTK first, before it leaves; then
    /// [`assoc::leave`] for it, whose address CAMs carry the key fields the
    /// deletes left; from a BSSID set without one, [`assoc::clear_bssid`],
    /// after [`assoc::disconnect`] when the AP's station was
    /// [`Live::authenticated`] (`ieee80211_destroy_assoc_data` destroys it
    /// from AUTH); with neither, nothing, without I/O. Either way the BSSID,
    /// the authentication, the association, the agreements and the keys are
    /// gone afterwards, the AP's station's PN checks with them, and the
    /// address CAM's key fields are a fresh vif's, as
    /// Linux's idle power-off and its vif init leave them (`ps.c:233-278`,
    /// `rtw89_cam_init_addr_cam`, `cam.c:646-684`) before the station's next
    /// scan or join; DIG is left as it is. Refused before any I/O, changing
    /// nothing, on a chip off ([`POWERED_OFF`]) and while a scan has not
    /// been ended ([`SCAN_NOT_ENDED`]). A failure powers the chip off.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn leave(&mut self) -> Result<(), AxError> {
        self.idle()?;
        if self.bssid.is_none() {
            return Ok(());
        }
        let associated = self.associated.take();
        let bssid = self.bssid.take();
        let authenticated = core::mem::take(&mut self.authenticated);
        let mut cam = core::mem::take(&mut self.keys);
        let mut agreements = core::mem::take(&mut self.ba);
        self.rx_stats.forget();
        let features = Features::of(self.running.firmware);
        let mac = self.running.mac;
        let band = self.chan.band;
        let done = self.holding(|io, session, sink| {
            match (&associated, bssid) {
                (Some(inputs), _) => {
                    for tid in agreements.tids() {
                        // SAFETY: the caller's contract is `start`'s, which
                        // `ba::stop`'s is, for the live chip's own window and
                        // rings.
                        unsafe { ba::stop(io, session, &mut agreements, tid, sink) }?;
                    }
                    let addr = AddrCam::connected(inputs.mac, inputs.bssid, inputs.aid);
                    for idx in cam.deletion_order() {
                        // SAFETY: the caller's contract is `start`'s, which
                        // `delete`'s is, for the live chip's own window and
                        // rings.
                        unsafe { keys::delete(io, session, &mut cam, &addr, idx, sink) }?;
                    }
                    let left = cam.keys();
                    // SAFETY: as above, `leave`'s.
                    unsafe { assoc::leave(io, session, inputs, features, left, sink) }
                }
                (None, Some(bssid)) if authenticated => {
                    let left = cam.keys();
                    // SAFETY: as above, `disconnect`'s and `clear_bssid`'s.
                    unsafe { assoc::disconnect(io, session, mac, bssid, band, left, sink) }?;
                    // SAFETY: as above.
                    unsafe { assoc::clear_bssid(io, session, mac, left, sink) }
                }
                // SAFETY: as above, `clear_bssid`'s.
                (None, _) => unsafe { assoc::clear_bssid(io, session, mac, cam.keys(), sink) },
            }
        });
        self.done(done)
    }

    /// Whether security CAM entry `cam` holds a key [`Live::add_key`]
    /// installed and nothing has deleted since.
    pub fn key_installed(&self, cam: u8) -> bool {
        self.keys.installed(cam)
    }

    /// Install `key` for the associated station (M3 spec S12.5,
    /// [`keys::install`]): SEC_UPD into the first free security CAM entry
    /// and its REC_ACK, then the dctl and the address CAM, connected with the
    /// association's AID, and that CAM's own DONE_ACK with return 0, each
    /// wait `ACK_WAIT_US`; frames drained meanwhile are held for the next
    /// [`Live::drain`]. Returns the entry, which a frame's TX descriptor
    /// names. Refused before any I/O, changing nothing: on a chip off
    /// ([`POWERED_OFF`]), while a scan has not been ended
    /// ([`SCAN_NOT_ENDED`]), with `Mac` while not associated (mac80211
    /// refuses a station's key before ASSOC, `ieee80211_add_key`), and as
    /// [`keys::check`] refuses. Any later failure, a firmware that never
    /// acknowledged the key included, is returned with nothing installed:
    /// the entry and the slot are free again, and the chip stays on, for
    /// the station to leave and join again. An installed pairwise key
    /// starts the started TIDs' PN checks over
    /// (`rtw89_core_tid_rx_stats_reset`, `mac80211.c:1007-1008`).
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn add_key(&mut self, key: &HardwareKey<'_>) -> Result<u8, AxError> {
        self.idle()?;
        let Some(inputs) = &self.associated else {
            return Err(AxError::Mac("a key without an association"));
        };
        let addr = AddrCam::connected(inputs.mac, inputs.bssid, inputs.aid);
        keys::check(&self.keys, key.pairwise, key.index)?;
        let mut cam = self.keys;
        let done = self.holding(|io, session, held| {
            // SAFETY: the caller's contract is `start`'s, which `install`'s
            // is, for the live chip's own window and rings.
            unsafe {
                keys::install(
                    io,
                    session,
                    &mut cam,
                    &addr,
                    key.pairwise,
                    key.index,
                    key.material,
                    held,
                )
            }
        });
        self.keys = cam;
        let installed = done.and_then(|done| done)?;
        if key.pairwise {
            self.rx_stats.reset();
        }
        Ok(installed)
    }

    /// Delete the key in security CAM entry `cam` (M3 spec S12.6,
    /// [`keys::delete`]): its slot detached, the dctl and the address CAM,
    /// neither waited on; the entry is free afterwards whatever they
    /// returned, as Linux frees it, and their first failure is returned
    /// with the chip left on. Refused before any I/O, changing nothing, on
    /// a chip off ([`POWERED_OFF`]), while a scan has not been ended
    /// ([`SCAN_NOT_ENDED`]), and for an entry holding no key
    /// ([`keys::NO_KEY`]).
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn remove_key(&mut self, cam: u8) -> Result<(), AxError> {
        self.idle()?;
        // A key is installed only while associated, and leaving deletes it.
        let (true, Some(inputs)) = (self.keys.installed(cam), &self.associated) else {
            return Err(keys::NO_KEY);
        };
        let addr = AddrCam::connected(inputs.mac, inputs.bssid, inputs.aid);
        let mut state = self.keys;
        let done = self.holding(|io, session, held| {
            // SAFETY: the caller's contract is `start`'s, which `delete`'s
            // is, for the live chip's own window and rings.
            unsafe { keys::delete(io, session, &mut state, &addr, cam, held) }
        });
        self.keys = state;
        done.and_then(|done| done)
    }

    /// `rtw89_ops_ampdu_action`'s RX_START (M3 spec S13.2,
    /// `mac80211.c:1062-1065`) of the associated station's agreement on
    /// `tid` from `ssn` with a window of `window`: the TID's PN check
    /// started over (`rtw89_core_tid_rx_stats_ctrl`), then [`ba::start`], a
    /// static BA CAM entry and its H2C, sent without a wait; with all eight
    /// entries taken nothing is sent, the hardware's dynamic BA CAM covering
    /// the agreement. Frames drained meanwhile are held for the next
    /// [`Live::drain`]. Refused before any I/O, changing nothing: on a chip
    /// off ([`POWERED_OFF`]), while a scan has not been ended
    /// ([`SCAN_NOT_ENDED`]), for a TID past 15 ([`ba::TID_PAST_15`]), and
    /// while not associated ([`BA_WITHOUT_ASSOCIATION`]: mac80211 starts
    /// none before). A failed send is returned with the entry taken, as
    /// Linux leaves it, and the chip left on.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn start_rx_block_ack(
        &mut self,
        tid: u8,
        ssn: u16,
        window: u16,
    ) -> Result<(), AxError> {
        self.idle()?;
        if usize::from(tid) >= ba::TIDS {
            return Err(ba::TID_PAST_15);
        }
        if self.associated.is_none() {
            return Err(BA_WITHOUT_ASSOCIATION);
        }
        self.rx_stats.control(tid, true);
        let mut cam = self.ba;
        let done = self.holding(|io, session, held| {
            // SAFETY: the caller's contract is `start`'s, which
            // `ba::start`'s is, for the live chip's own window and rings.
            unsafe { ba::start(io, session, &mut cam, tid, ssn, window, held) }
        });
        self.ba = cam;
        done.and_then(|done| done)?;
        bump(&mut self.ba_counts.started);
        Ok(())
    }

    /// `rtw89_ops_ampdu_action`'s RX_STOP (`mac80211.c:1066-1069`) of the
    /// agreement on `tid`: its PN check stopped, then [`ba::stop`], the
    /// oldest entry it holds freed and its stop H2C sent without a wait; an
    /// agreement holding no entry sends nothing. Frames drained meanwhile
    /// are held for the next [`Live::drain`]. Refused before any I/O,
    /// changing nothing, on a chip off ([`POWERED_OFF`]), while a scan has
    /// not been ended ([`SCAN_NOT_ENDED`]), and for a TID past 15
    /// ([`ba::TID_PAST_15`]). A failed send is returned with the entry
    /// already free, as Linux frees it, and the chip left on.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn stop_rx_block_ack(&mut self, tid: u8) -> Result<(), AxError> {
        self.idle()?;
        if usize::from(tid) >= ba::TIDS {
            return Err(ba::TID_PAST_15);
        }
        self.rx_stats.control(tid, false);
        let mut cam = self.ba;
        let done = self.holding(|io, session, held| {
            // SAFETY: the caller's contract is `start`'s, which `ba::stop`'s
            // is, for the live chip's own window and rings.
            unsafe { ba::stop(io, session, &mut cam, tid, held) }
        });
        self.ba = cam;
        done.and_then(|done| done)?;
        bump(&mut self.ba_counts.stopped);
        Ok(())
    }

    /// `rtw89_ops_flush` (`mac80211.c:1122-1136`), as mac80211's
    /// `ieee80211_flush_queues` and `rtw89_ops_set_key`'s DISABLE_KEY
    /// (`mac80211.c:1010-1013`) call it: no LPS to leave, then
    /// `rtw89_hci_flush_queues`, the TX rings' wait of [`Rings::flush_tx`];
    /// then with `drop`, as the 8852C's firmware lacks `NO_PACKET_DROP`
    /// (`fw.c:870`), `__rtw89_drop_packets` (`mac80211.c:1109-1120`): for
    /// the AP's station, which exists while a BSSID is set (mac80211 adds it
    /// before authentication and iterates it as uploaded), the four
    /// PKT_DROP H2Cs of `rtw89_mac_pkt_drop_sta` (`mac.c:7255-7278`), BE, BK,
    /// VI then VO once for macid 0, each sent without a wait and its failure
    /// passed over as Linux ignores it; without `drop`,
    /// `rtw89_mac_flush_txq` (`mac.c:6352-6363`): `R_AX_DLE_EMPTY0` sampled
    /// every 10 ms for up to 200 ms until `dle_is_txq_empty`'s queues are
    /// all empty (`mac.c:2388-2412`), a timeout passed over as Linux only
    /// logs it. Frames drained meanwhile are held for the next
    /// [`Live::drain`]. Refused before any I/O on a chip off
    /// ([`POWERED_OFF`]) and while a scan has not been ended
    /// ([`SCAN_NOT_ENDED`]); a failed register read is returned, the chip
    /// left on, as is the first PKT_DROP that could not be sent.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn flush(&mut self, drop: bool) -> Result<(), AxError> {
        self.idle()?;
        self.rings.flush_tx(&mut self.io)?;
        if !drop {
            return match poll32(
                &mut self.io,
                R_AX_DLE_EMPTY0,
                DLE_TXQ_EMPTY,
                DLE_TXQ_EMPTY,
                FLUSH_TXQ_STEP_US,
                FLUSH_TXQ_WAIT_US,
            ) {
                Ok(()) | Err(AxError::Timeout { .. }) => Ok(()),
                Err(error) => Err(error),
            };
        }
        if self.bssid.is_none() {
            return Ok(());
        }
        let sent = self.holding(|io, session, held| {
            let mut first = Ok(());
            for sel in PKT_DROP_SELS {
                let packet = session.h2c.pkt_drop(sel, vif::MAC_ID, vif::PORT);
                // SAFETY: the caller's contract is `start`'s, which `send`'s
                // is, for the live chip's own window and rings.
                let result = unsafe { session.send(io, packet, held) };
                if first.is_ok() {
                    first = result.map(|_| ());
                }
            }
            first
        });
        sent.and_then(|sent| sent)
    }

    /// The channel context released after the station left (mac80211's
    /// `ieee80211_link_release_channel` at the end of
    /// `ieee80211_set_disassoc` and `ieee80211_destroy_auth_data`):
    /// `rtw89_chanctx_ops_unassign_vif`'s `rtw89_set_channel`
    /// (`chan.c:3432-3500`), which still finds the context registered and
    /// switches to its channel again ([`Live::tune`] to [`Live::chan`]),
    /// then `rtw89_chanctx_ops_remove` (`chan.c:3367-3373`): no context is
    /// left, so the next scan's restore goes to the default chandef, 2.4
    /// GHz channel 1 (`rtw89_entity_recalc`, `chan.c:614-616`), without a
    /// switch now. Refused and failing as [`Live::tune`].
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn release_channel(&mut self) -> Result<(), AxError> {
        let chan = self.chan;
        // SAFETY: the caller's contract is `tune`'s, word for word.
        unsafe { self.tune(&chan) }?;
        self.home = Chan::new20(BOOT_CHANNEL.band, BOOT_CHANNEL.channel)?;
        Ok(())
    }

    /// Queue a management (or null data) frame, `bytes` from its MAC header
    /// on without FCS, as `rtw89_core_tx_write` does for a station's frame:
    /// the WD head of [`txd::management`] for the tuned band and the BSS's
    /// basic rates `basic` (mac80211's band order, 0 before one is known),
    /// submitted on the channel it names (CH8). `Ok(None)` is "no room":
    /// the channel is full ([`super::pci::Txq::free`] as of the last reclaim), or a scan
    /// has not been ended, which stops the queues; nothing is touched. The
    /// token names the frame until its [`Incoming::Transmitted`]. Refused
    /// with [`POWERED_OFF`] on a chip off and for what the encoder or
    /// [`super::pci::Txq::submit`] refuses; a submit failure leaves the chip on.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn queue_management(
        &mut self,
        bytes: &[u8],
        basic: u16,
    ) -> Result<Option<Token>, AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        if self.scan.is_some() {
            return Ok(None);
        }
        let (ch, head) = txd::management(bytes, self.chan.band, basic)?;
        // SAFETY: the caller's contract is `submit`'s, word for word.
        unsafe { self.submit(ch, &head, bytes) }
    }

    /// Queue a data frame, as [`Live::queue_management`] queues a
    /// management frame: the WD head of [`txd::data`] for the queue of
    /// `tid`, `eapol`, the tuned band, the peer's `supported` rates, once
    /// associated the STBC and LDPC its RA H2C took (`rtwsta_link->ra`,
    /// `core.c:1145-1146`; neither before), and `sec`, the hardware key a
    /// Protected frame goes under (`rtw89_core_tx_update_sec_key`), whose
    /// CCMP header [`txd::strip_ccmp`] removed. The key's CAM entry is the
    /// caller's to have installed ([`Chip::transmit_data`] checks it).
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn queue_data(
        &mut self,
        bytes: &[u8],
        tid: u8,
        eapol: bool,
        supported: u16,
        sec: Option<TxSec>,
    ) -> Result<Option<Token>, AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        if self.scan.is_some() {
            return Ok(None);
        }
        let caps = match &self.associated {
            Some(a) => {
                let ra = assoc::ra_info(&a.peer, a.band);
                RateCaps {
                    stbc: ra.stbc,
                    ldpc: ra.ldpc,
                }
            }
            None => RateCaps {
                stbc: false,
                ldpc: false,
            },
        };
        let (ch, head) = txd::data(bytes, tid, eapol, self.chan.band, supported, caps, sec)?;
        // SAFETY: the caller's contract is `submit`'s, word for word.
        unsafe { self.submit(ch, &head, bytes) }
    }

    /// One frame on `ch`, the channel its encoder named ([`super::pci::Txq::submit`]
    /// checks no head against its queue), counted.
    ///
    /// # Safety
    /// [`start`]'s contract: [`super::pci::Txq::submit`]'s for the live chip's own
    /// window, rings and pools.
    unsafe fn submit(
        &mut self,
        ch: TxChannel,
        head: &[u8; WD_HEAD_BYTES],
        bytes: &[u8],
    ) -> Result<Option<Token>, AxError> {
        // SAFETY: the caller's contract; `start` installed the pools for
        // these rings' own requester.
        let queued = unsafe {
            self.rings
                .txq(&mut self.pools, ch)
                .submit(&mut self.io, head, bytes)
        }?;
        match queued {
            Some(page) => {
                bump(&mut self.tx.sent);
                // Txq::submit writes the ring index for each frame it takes.
                bump(&mut self.counts.submitted[ch.slot()]);
                bump(&mut self.counts.kicks[ch.slot()]);
                Ok(Some(token(ch, page)))
            }
            None => {
                bump(&mut self.tx.full);
                Ok(None)
            }
        }
    }

    /// The radio's poll (M3 spec S1.4), `rtw89_pci_napi_poll`'s order: RPQ,
    /// each release report returning its page ([`TxPools::release`]) and
    /// handed on as [`Incoming::Transmitted`], acknowledged for
    /// `RTW89_TX_DONE`; the TX reclaim of every channel with TXBDs
    /// outstanding; RXQ ([`Live::drain`]), its frames handed on, each past
    /// `rtw89_core_skb_pn_valid` as it was drained ([`RxStats::valid`], a
    /// drop counted), with each RA report for the station's MAC ID as
    /// [`Incoming::RaReport`] (`rtw89_phy_c2h_ra_rpt`, M4 spec S3.3); the
    /// scan budget ([`Live::expire_scan`]); then each beacon filter report
    /// the drains brought, held ones first, as `rtw89_mac_bcn_fltr_rpt`
    /// takes it (`Live::beacon_filter`); then, once,
    /// [`Incoming::ScanEnded`] when the scan begun has ended, after every
    /// frame it heard. Every step runs whatever an earlier one returned,
    /// and the first error is returned. Such an error is not fatal: no step
    /// powers the chip off, every event before the error was handed on, and
    /// the next poll tries again. Refused with [`POWERED_OFF`] on a chip
    /// off.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn service(&mut self, sink: &mut dyn FnMut(Incoming)) -> Result<(), AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        // SAFETY: the caller's contract is `reports`'s, word for word.
        let mut first = unsafe { self.reports(sink) };
        for ch in TX_CHANNELS {
            // SAFETY: as above, `reclaim`'s, for the live chip's own window,
            // rings and pools.
            let reclaimed = unsafe { self.rings.txq(&mut self.pools, ch).reclaim(&mut self.io) };
            keep(&mut first, reclaimed);
        }
        let mut filter = Vec::new();
        let mut frames = |e: Event<'_>| match e {
            Event::Frame(frame) => sink(Incoming::Frame(heard(frame))),
            Event::C2h(C2h::BeaconFilter(report)) => filter.push(report),
            Event::C2h(C2h::RaReport(report)) if station_rate(&report) => {
                sink(Incoming::RaReport(report))
            }
            Event::C2h(_) => {}
        };
        // SAFETY: as above, `drain`'s.
        keep(&mut first, unsafe { self.drain(&mut frames) });
        // SAFETY: as above, `expire_scan`'s.
        keep(&mut first, unsafe { self.expire_scan(&mut frames) });
        for report in filter {
            // SAFETY: as above, `beacon_filter`'s.
            keep(&mut first, unsafe { self.beacon_filter(report, sink) });
        }
        if let Some(scan) = self.scan.as_mut()
            && !scan.announced
            && let Some(end) = scan.end
        {
            scan.announced = true;
            sink(Incoming::ScanEnded {
                completed: end == ScanEnd::Ok,
            });
        }
        first
    }

    /// One RPQ poll ([`super::pci::Rpq::poll`]): every report of every
    /// buffer, as `rtw89_pci_release_rpp` takes it.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    unsafe fn reports(&mut self, sink: &mut dyn FnMut(Incoming)) -> Result<(), AxError> {
        let Self {
            io,
            rings,
            pools,
            tx,
            counts,
            ..
        } = self;
        let mut rpq = rings.rpq();
        // SAFETY: the caller's contract is `poll`'s, for the live chip's own
        // window and rings.
        let polled = unsafe {
            rpq.poll(io, &mut |buffer| {
                let found = releases(buffer)?;
                tx.skipped = tx.skipped.saturating_add(u32::from(found.invalid));
                for release in found.reports {
                    // `releases` keeps only seqs below the pools' 16 and the
                    // qsels of a channel.
                    let (Some(ch), Ok(page)) =
                        (TxChannel::of_qsel(release.qsel), u8::try_from(release.seq))
                    else {
                        continue;
                    };
                    if !pools.release(ch, page) {
                        bump(&mut tx.stale);
                        continue;
                    }
                    let acknowledged = release.status == TX_DONE;
                    bump(&mut tx.reported);
                    bump(&mut counts.released[ch.slot()]);
                    if acknowledged {
                        bump(&mut tx.acked);
                    } else {
                        bump(&mut counts.failed);
                    }
                    sink(Incoming::Transmitted {
                        token: token(ch, page),
                        acknowledged,
                    });
                }
                Ok(())
            })
        };
        polled.map(|_| ())
    }

    /// `rtw89_mac_bcn_fltr_rpt` (`mac.c:5389-5443`) of `report` for the one
    /// vif (M3 spec S15): a report for another MAC ID than the vif's is
    /// ignored, and so are the RSSI and NOTIFY ones, as nothing asks for
    /// CQM (`ieee80211_cqm_rssi_notify`); `report.ma` is raw, its signal
    /// `ma - MAX_RSSI`, which nothing reads. A BEACON_LOSS while associated
    /// is handed on as [`Incoming::BeaconLost`] unless a scan has not been
    /// ended (`rtwdev->scanning`, as `ieee80211_beacon_loss` is called only
    /// then), and then re-arms the filter as Linux does on every loss
    /// report, `rtw89_fw_h2c_set_bcn_fltr_cfg(.., true)` ([`H2c::bcn_fltr`]
    /// of this firmware's [`Features`]) sent without a wait when the
    /// firmware has BEACON_FILTER (`fw.c:5409-5410`); frames drained
    /// meanwhile are held for the next [`Live::drain`]. While not
    /// associated nothing happens: mac80211 takes no beacon loss of a
    /// station not associated, and Linux sends no filter for a vif not
    /// INFRA (`fw.c:5413`). A failed send is returned with the chip left
    /// on, where Linux only logs it.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    unsafe fn beacon_filter(
        &mut self,
        report: BcnFltr,
        sink: &mut dyn FnMut(Incoming),
    ) -> Result<(), AxError> {
        if report.macid != vif::MAC_ID
            || report.kind != BCNFLTR_BEACON_LOSS
            || self.associated.is_none()
        {
            return Ok(());
        }
        if self.scan.is_none() {
            sink(Incoming::BeaconLost);
        }
        let features = Features::of(self.running.firmware);
        if !features.beacon_filter {
            return Ok(());
        }
        let sent = self.holding(|io, session, held| {
            let packet = session
                .h2c
                .bcn_fltr(vif::MAC_ID, true, features.loss_count_v1);
            // SAFETY: the caller's contract is `start`'s, which `send`'s is,
            // for the live chip's own window and rings.
            unsafe { session.send(io, packet, held) }.map(|_| ())
        });
        sent.and_then(|sent| sent)
    }

    /// The end of an association step run on the session: its error, the
    /// chip powered off for it.
    fn done(&mut self, done: Result<Result<(), AxError>, AxError>) -> Result<(), AxError> {
        match done {
            Ok(Ok(())) => Ok(()),
            Ok(Err(error)) => {
                self.power_off();
                Err(error)
            }
            Err(off) => Err(off),
        }
    }

    /// Hand the held frames and beacon filter reports to `sink`, oldest
    /// first.
    fn release(&mut self, sink: &mut dyn FnMut(Event<'_>)) {
        while let Some(event) = self.held.next() {
            sink(event);
        }
    }

    /// `rtw89_core_stop` after a failure ([`bringup::shut_down`]): every
    /// later call is refused with [`POWERED_OFF`], and a scan, a BSSID, an
    /// association, the agreements and the keys are gone with the firmware.
    fn power_off(&mut self) {
        bringup::shut_down(&mut self.io, &self.rings, self.host);
        self.off = true;
        self.scan = None;
        self.bssid = None;
        self.authenticated = false;
        self.associated = None;
        self.keys = KeyCam::default();
        self.ba = BaCam::default();
        self.rx_stats.forget();
    }

    /// [`Live::session`] for an operation's `f`, with a sink that holds what
    /// it drains for the next [`Live::drain`] ([`Held`]), each frame past
    /// the PN check first ([`Live::passing`]).
    fn holding<R>(
        &mut self,
        f: impl FnOnce(&mut I, &mut Session<'_>, &mut dyn FnMut(Event<'_>)) -> R,
    ) -> Result<R, AxError> {
        let mut held = core::mem::take(&mut self.held);
        let result = self.passing(&mut |e| held.keep(e), f);
        self.held = held;
        result
    }

    /// [`Live::session`] for a drain's `f`, with `sink` behind the PN check
    /// ([`checked`]).
    fn passing<R>(
        &mut self,
        sink: &mut dyn FnMut(Event<'_>),
        f: impl FnOnce(&mut I, &mut Session<'_>, &mut dyn FnMut(Event<'_>)) -> R,
    ) -> Result<R, AxError> {
        let mut stats = core::mem::take(&mut self.rx_stats);
        let associated = self.associated.is_some();
        let result = self.session(|io, session| {
            f(io, session, &mut |e| {
                if checked(&mut stats, associated, &e) {
                    sink(e);
                }
            })
        });
        self.rx_stats = stats;
        result
    }

    /// Run `f` on the command session, resumed from where the last one was
    /// parked, then park it again. Refused with [`POWERED_OFF`] on a chip a
    /// failure powered off.
    pub(super) fn session<R>(
        &mut self,
        f: impl FnOnce(&mut I, &mut Session<'_>) -> R,
    ) -> Result<R, AxError> {
        if self.off {
            return Err(POWERED_OFF);
        }
        // Taken only here, and `f` cannot reach `self`: always `Some`.
        let parked = self
            .parked
            .take()
            .ok_or(AxError::Mac("command session already in use"))?;
        let mut session = Session::resume(&mut self.rings, &mut self.h2c, parked);
        let result = f(&mut self.io, &mut session);
        self.parked = Some(session.park());
        Ok(result)
    }

    /// `rtw89_core_stop` (M3 spec S3.2, `bringup::shut_down`): the coex
    /// half, `pci::deinit`, then the power-off, every result ignored as
    /// Linux ignores them; nothing on a chip a failure already powered
    /// off. The window, the rings and the pools come back and must be kept:
    /// nothing proves the DMA stopped.
    ///
    /// # Safety
    /// [`start`]'s contract, still holding for this chip.
    pub unsafe fn stop(self) -> (I, Rings, TxPools) {
        let Self {
            mut io,
            rings,
            pools,
            host,
            off,
            ..
        } = self;
        if !off {
            bringup::shut_down(&mut io, &rings, host);
        }
        (io, rings, pools)
    }
}

/// The live chip beneath an rtw89 station (M3 spec S14.1). Each call is the
/// [`Live`] operation it names, its error that operation's `AxError` text,
/// a channel switch's prefixed with its step. Every call but [`Chip::mac`]
/// and [`Chip::diagnostic`] is refused with [`POWERED_OFF`] once a failure
/// powered the chip off.
///
/// Safe to call as the trait is: [`start`]'s contract is its caller's for
/// as long as the live chip is used, which every call here is.
impl<I: AxIo> Chip for Live<I> {
    fn mac(&self) -> Mac {
        Mac(self.running.mac)
    }

    /// [`Live::service`].
    fn poll(&mut self, sink: &mut dyn FnMut(Incoming)) -> Result<(), String> {
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { self.service(sink) }.map_err(|e| e.to_string())
    }

    /// [`Live::tune`], a failure named by its step (`"set channel"` or
    /// `"tx power"`).
    fn tune(&mut self, channel: Channel) -> Result<(), String> {
        let chan = chan_of(channel).map_err(|e| failed("set channel", e))?;
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { self.switch(&chan) }.map_err(|e| failed(e.step, e.error))
    }

    /// [`Live::begin_scan`] of `channels`, each passive with M2c's period
    /// and dwell ([`scan::scan_list`]), within its own budget
    /// ([`scan::scan_budget_us`] of the list).
    fn begin_scan(&mut self, channels: &[Channel]) -> Result<(), String> {
        if self.off {
            return Err(POWERED_OFF.to_string());
        }
        let mut pairs = Vec::new();
        for c in channels {
            let number = u8::try_from(c.number)
                .map_err(|_| AxError::Scan("scan channel not a 20 MHz 2.4 or 5 GHz channel"))
                .map_err(|e| e.to_string())?;
            pairs.push((hw_band(c.band), number));
        }
        let list = scan::scan_list(&pairs).map_err(|e| e.to_string())?;
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { Live::begin_scan(self, &list) }.map_err(|e| e.to_string())
    }

    fn end_scan(&mut self) -> Result<(), String> {
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { Live::end_scan(self) }.map_err(|e| e.to_string())
    }

    fn set_bssid(&mut self, bssid: Mac) -> Result<(), String> {
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { Live::set_bssid(self, bssid.0) }.map_err(|e| e.to_string())
    }

    /// [`Live::authenticated`].
    fn authenticated(&mut self) -> Result<(), String> {
        Live::authenticated(self).map_err(|e| e.to_string())
    }

    /// [`Live::queue_management`].
    fn transmit_management(&mut self, frame: &Management<'_>) -> Result<Option<Token>, String> {
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { self.queue_management(frame.bytes, frame.basic_rates) }.map_err(|e| e.to_string())
    }

    /// [`Live::queue_data`], a frame's key as its CAM entry, key ID and PN
    /// ([`TxSec`]). A frame naming an entry that holds no key
    /// [`Live::add_key`] installed and nothing deleted since is refused
    /// with [`KEY_NOT_IN_CAM`] before any access: the hardware would
    /// encrypt it under whatever that entry held, or under nothing.
    fn transmit_data(&mut self, frame: &Data<'_>) -> Result<Option<Token>, String> {
        if self.off {
            return Err(POWERED_OFF.to_string());
        }
        let sec = match frame.key {
            Some((cam, _, _)) if !self.key_installed(cam) => return Err(KEY_NOT_IN_CAM.into()),
            Some((cam, keyid, pn)) => Some(TxSec { keyid, cam, pn }),
            None => None,
        };
        let Data {
            bytes,
            tid,
            eapol,
            supported_rates,
            ..
        } = *frame;
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { self.queue_data(bytes, tid, eapol, supported_rates, sec) }
            .map_err(|e| e.to_string())
    }

    /// [`Live::associate`] with the AP's peer at RSSI 0 (none averaged, M3
    /// spec S1.9). Linux's channel context is the joined BSS's since the
    /// join's switch, so there is none here unless `association` names
    /// another channel than the one tuned, which is switched to first (a
    /// failure named as [`Chip::tune`] names it).
    fn associate(&mut self, association: &Association) -> Result<(), String> {
        if self.off {
            return Err(POWERED_OFF.to_string());
        }
        let chan = chan_of(association.channel).map_err(|e| failed("set channel", e))?;
        if chan != self.chan {
            // SAFETY: `start`'s contract holds for the live chip's every use.
            unsafe { self.switch(&chan) }.map_err(|e| failed(e.step, e.error))?;
        }
        let inputs = AssocInputs {
            mac: self.running.mac,
            bssid: association.bssid.0,
            aid: association.aid,
            band: chan.band,
            beacon_int: association.beacon_interval,
            dtim: association.dtim_period,
            peer: RaPeer {
                legacy: association.legacy,
                ht: association.ht,
                rssi: 0,
            },
            edca: association.edca,
            short_slot: association.short_slot,
        };
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { Live::associate(self, &inputs) }.map_err(|e| e.to_string())
    }

    /// [`Live::leave`] (AUTH to NONE's disconnect too after
    /// [`Chip::authenticated`]), then, when a BSSID was set, the channel
    /// context released ([`Live::release_channel`]).
    fn disassociate(&mut self) -> Result<(), String> {
        let joined = self.bssid.is_some();
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { self.leave() }.map_err(|e| e.to_string())?;
        if joined {
            // SAFETY: as above.
            unsafe { self.release_channel() }.map_err(|e| e.to_string())?;
        }
        Ok(())
    }

    /// [`Live::add_key`].
    fn add_key(&mut self, key: &HardwareKey<'_>) -> Result<u8, String> {
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { Live::add_key(self, key) }.map_err(|e| e.to_string())
    }

    /// [`Live::remove_key`].
    fn remove_key(&mut self, cam: u8) -> Result<(), String> {
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { Live::remove_key(self, cam) }.map_err(|e| e.to_string())
    }

    /// [`Live::flush`].
    fn flush(&mut self, drop: bool) -> Result<(), String> {
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { Live::flush(self, drop) }.map_err(|e| e.to_string())
    }

    /// [`Live::start_rx_block_ack`].
    fn start_rx_block_ack(&mut self, tid: u8, ssn: u16, window: u16) -> Result<(), String> {
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { Live::start_rx_block_ack(self, tid, ssn, window) }.map_err(|e| e.to_string())
    }

    /// [`Live::stop_rx_block_ack`].
    fn stop_rx_block_ack(&mut self, tid: u8) -> Result<(), String> {
        // SAFETY: `start`'s contract holds for the live chip's every use.
        unsafe { Live::stop_rx_block_ack(self, tid) }.map_err(|e| e.to_string())
    }

    /// The TX counts, RPQ's skipped and stale reports, the receiver's
    /// drops and the held frames dropped, then, once any came, the RX Block
    /// Ack starts and stops and the frames the PN check dropped, naming no
    /// address.
    fn diagnostic(&self) -> String {
        let TxTally {
            sent,
            full,
            reported,
            acked,
            skipped,
            stale,
        } = self.tx;
        let mut line = format!(
            "tx sent {sent} reported {reported} acked {acked} full {full}, \
             rpq skipped {skipped} stale {stale}"
        );
        if let Some(parked) = &self.parked {
            let c = parked.rx.counts;
            line.push_str(&format!(
                ", rx fcs {} other {} short {} overflow {}",
                c.dropped_fcs, c.other_frames, c.short_frames, c.overflow
            ));
        }
        line.push_str(&format!(", held dropped {}", self.held.dropped));
        let BaCounts { started, stopped } = self.ba_counts;
        if started != 0 || stopped != 0 {
            line.push_str(&format!(", ba started {started} stopped {stopped}"));
        }
        let dropped = self.rx_stats.dropped();
        if dropped != 0 {
            line.push_str(&format!(", pn dropped {dropped}"));
        }
        if self.off {
            line.push_str(", powered off");
        }
        line
    }

    /// The TX power's alpha2 and its index on the band tuned
    /// ([`Regds::txpwr`]), the RXQ polls that found RXQ full and
    /// [`Live::tx_counts`].
    fn link_counts(&self) -> LinkCounts {
        let (regds, alpha2) = self.regds;
        LinkCounts {
            regd: Some((alpha2, regds.txpwr(self.chan.band))),
            rxq_full: u32::from(self.rings.rxq_full_batches()),
            tx: self.tx_counts(),
        }
    }
}

#[cfg(test)]
impl<I: AxIo> Live<I> {
    pub(crate) fn io(&self) -> &I {
        &self.io
    }

    pub(crate) fn io_mut(&mut self) -> &mut I {
        &mut self.io
    }

    pub(crate) fn h2c(&self) -> H2c {
        self.h2c
    }

    pub(crate) fn receiver(&self) -> &super::receive::Receiver {
        &self.parked.as_ref().unwrap().rx
    }

    pub(crate) fn cam_keys(&self) -> super::assoc::CamKeys {
        self.keys.keys()
    }

    pub(crate) fn ba_tids(&self) -> Vec<u8> {
        self.ba.tids()
    }

    /// The PN check every drain runs on what it brings (`Live::passing`,
    /// through `checked`), on `frame`.
    pub(crate) fn pn_valid(&mut self, frame: &Received) -> bool {
        self.rx_stats.valid(self.associated.is_some(), frame)
    }
}

#[cfg(test)]
mod tests;

// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2020-2022  Realtek Corporation
//! An RTL8852C's first passive scan, from the end of `rtw89_core_start` to
//! the end of the firmware's scan offload (M2c spec S2, S8-S13), as
//! `rtw89_ops_hw_scan` (`mac80211.c:1279-1320`) runs it from idle.
//!
//! In `rtw89_leave_ips`'s order (`ps.c:257-278`): the rest of
//! `rtw89_core_start` ([`core_tail`]), `rtw89_set_channel` to 2.4 GHz
//! channel 1 ([`channel::set_channel`]) and `rtw89_mac_vif_init` ([`vif`]).
//! Then `rtw89_hw_scan_start` (`fw.c:10367-10437`) after the SCAN_OFFLOAD
//! firmware gate (`fw.c:898`): `rtw89_mac_port_cfg_rx_sync(false)`
//! (`mac.c:4674-4684`), `rtw89_core_scan_start` (`core.c:7063-7081`) with
//! `rtw89_pci_recalc_int_mit` (`pci.c:4261-4283`),
//! `rtw89_phy_config_edcca(scan)` (`phy.c:8641-8663`) and the address CAM
//! H2C, `rtw89_mac_set_rx_fltr` (`mac.c:2678-2690`) and
//! `rtw89_phy_dig_suspend` (`phy.c:7850-7893`). Then
//! `rtw89_hw_scan_offload(true)` (`fw.c:10531-10571`): ADD_SCANOFLD_CH with
//! [`SCAN_CHANNELS`], one batch, then SCANOFLD, each waiting for its
//! DONE_ACK as `rtw89_mac_c2h_done_ack` completes them (`mac.c:5532-5554`).
//! Each SCANOFLD_RSP is read as `rtw89_mac_c2h_scanofld_rsp` does
//! (`mac.c:5283-5385`): ENTER_CH hops are counted, END_SCAN ends the scan.
//! Without END_SCAN in [`SCAN_WAIT_US`], the scan is stopped as
//! `rtw89_hw_scan_abort` does (`fw.c:10494-10511`): SCANOFLD stop, then up
//! to [`STOP_WAIT_US`] for END_SCAN, as the stop's wait condition is
//! completed only by END_SCAN (`mac.c:6131-6157`); the result is reported,
//! not failed. The beacons and probe responses received meanwhile
//! ([`receive`](super::receive)) go through `wifi::scan::Scanner::observe` (cfg80211's BSS
//! parsing, spec S1.12) and are kept by BSSID with `Bss::keep_name_of`.
//! Frames still queued from core start and vif init are flushed away before
//! `Scanner::begin`; frames drained between ADD_SCANOFLD_CH and SCANOFLD's
//! DONE_ACK are held until `Scanner::submitted`; after END_SCAN or the stop,
//! RXQ is drained until a poll finds it empty, or for at most 64 polls
//! (`MAX_TAIL_POLLS`), which the report then says
//! ([`ScanReport::tail_capped`]); only then is the receiver flushed into the
//! scan, so frames still waiting for their PPDU status are kept, without
//! RSSI. Between ADD_SCANOFLD_CH's DONE_ACK and SCANOFLD start the request
//! is rechecked with `Scanner::validate_submission`.
//!
//! Where Linux aborts a failed `rtw89_hw_scan_offload` (`mac80211.c:1315`),
//! the scan start fails instead and the caller powers the chip off.
//!
//! The SCANOFLD_RSP `band` is used as given (8852C supports 6 GHz,
//! `mac.c:5310-5311`); no listed channel is 6 GHz, so a frame the receiver
//! tags with 6 GHz (band 2) fails the scanner's channel check and is
//! counted as dropped. The undefined band 3 tags 2.4 GHz, as
//! `rtw89_hw_to_nl80211_band`'s default does (`core.h:8469-8480`), so its
//! frames are kept only where 2.4 GHz and its channel are listed. Every
//! ENTER_CH report is counted as a hop, whatever its band.
//!
//! Not ported (spec S1.2, S1.5, S1.17): PKT_OFLD probe templates, the
//! software steps of `rtw89_hw_scan_start`, `rtw89_hw_scan_complete`'s
//! restores (the chip is powered off after the scan), and the 6 GHz, MCC
//! and connected-scan branches. This module grants no transmission request,
//! no association, no interrupts and no channel switch beyond the boot
//! channel; its only H2Cs are those of the steps above.
use alloc::vec::Vec;
use core::fmt;

use super::bb;
use super::c2h::{C2h, SCAN_END, SCAN_ENTER_CH, SCAN_STATUS_SUCCESS, ScanOfld};
use super::chan::{Chan, HwBand, frequency};
use super::channel::{self, ChannelInputs};
use super::command::{CommandError, Refused, Session};
use super::core_tail;
use super::h2c::{H2C_FUNC_ADD_SCANOFLD_CH, H2C_FUNC_SCANOFLD, H2c, ScanChannel};
use super::io::{AxError, AxIo, clear32};
use super::pci::Rings;
use super::receive::{Event, Received, Tag};
use super::vif::{self, MAC_ID, PORT};
use crate::Mac;
use crate::wifi::frame::Mpdu;
use crate::wifi::scan::{self as wscan, Outcome, Policy, Request, Retirement, Scanner, Ticket};
use crate::wifi::security::SecurityStatus;
use crate::wifi::{Band, Channel, RadioStatus, RawRate, Timestamp};

/// `RTW89_DFS_CHAN_TIME` (`fw.h:333`): every channel is NO_IR, so DFS
/// (`fw.c:9955-9959`), and its period is `max(RTW89_CHANNEL_TIME,
/// RTW89_DFS_CHAN_TIME)` (spec S1.4).
const SCAN_PERIOD: u8 = 105;
/// `RTW89_DWELL_TIME` (`fw.h:336`).
const SCAN_DWELL: u8 = 20;

const fn g2(channel: u8) -> ScanChannel {
    ScanChannel {
        band: HwBand::G2,
        channel,
        period: SCAN_PERIOD,
        dwell: SCAN_DWELL,
        dfs: true,
    }
}

const fn g5(channel: u8) -> ScanChannel {
    ScanChannel {
        band: HwBand::G5,
        ..g2(channel)
    }
}

/// The passive scan's channels in ADD_SCANOFLD_CH order (spec S1.4): 2.4 GHz
/// 1-13, then 5 GHz 36-64, 100-144 and 149-165, every one NO_IR.
pub const SCAN_CHANNELS: [ScanChannel; 38] = [
    g2(1),
    g2(2),
    g2(3),
    g2(4),
    g2(5),
    g2(6),
    g2(7),
    g2(8),
    g2(9),
    g2(10),
    g2(11),
    g2(12),
    g2(13),
    g5(36),
    g5(40),
    g5(44),
    g5(48),
    g5(52),
    g5(56),
    g5(60),
    g5(64),
    g5(100),
    g5(104),
    g5(108),
    g5(112),
    g5(116),
    g5(120),
    g5(124),
    g5(128),
    g5(132),
    g5(136),
    g5(140),
    g5(144),
    g5(149),
    g5(153),
    g5(157),
    g5(161),
    g5(165),
];
/// The END_SCAN budget from SCANOFLD's DONE_ACK: `2 * 38 * 105 ms + 1 s`
/// (spec S1.13). Linux has no driver timeout here.
pub const SCAN_WAIT_US: u64 = 8_980_000;
/// The stop's wait for END_SCAN, `RTW89_WAIT_FOR_COND_TIMEOUT`
/// (`core.h:5930`).
pub const STOP_WAIT_US: u64 = 100_000;
/// The networks a [`ScanReport`] lists.
pub const MAX_LISTED: usize = 32;

/// How the scan ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScanEnd {
    /// END_SCAN with status SUCCESS (1).
    Ok,
    /// END_SCAN with another status.
    Status(u8),
    /// No END_SCAN within [`SCAN_WAIT_US`]: the scan was stopped, and
    /// `ended` says whether END_SCAN came after the stop.
    Stopped { ended: bool },
}

/// One BSS the scan heard, as cfg80211 corrected its channel. `signal` is
/// the last RSSI heard on that channel, `None` if none was; `ssid` holds
/// `ssid_len` bytes; `hidden` is `Bss::hides_ssid`; `frames` counts the
/// beacons and probe responses merged into it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Network {
    pub bssid: [u8; 6],
    pub band: HwBand,
    pub channel: u8,
    pub signal: Option<i8>,
    pub ssid: [u8; 32],
    pub ssid_len: u8,
    pub hidden: bool,
    pub frames: u16,
}

/// What the scan found (spec S13.3). `elapsed_ms` runs from SCANOFLD
/// start's DONE_ACK to END_SCAN, or to the end of the stop's wait. `frames`
/// counts the beacons and probe responses the scanner accepted, of which
/// `without_rssi` had no matched PPDU status. `dropped` counts, from
/// `Scanner::begin` on, those it refused, those too short to be a frame,
/// those that are no MPDU, and those past a bound: a new BSSID past 512, or
/// more than 64 frames before `Scanner::submitted`.
/// `rxq_full` is [`Rings::rxq_full_batches`]; `tail_capped` says the drain
/// after END_SCAN or the stop hit its cap, 64 polls that each found RXQ
/// entries, so RXQ was never seen empty and frames may have been left in
/// it; `refused` the session ledger's first refused H2C, its return raw.
/// `networks` holds the strongest [`MAX_LISTED`] of `networks_total`,
/// strongest first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScanReport {
    pub channels: u8,
    pub entered: u8,
    pub elapsed_ms: u32,
    pub end: ScanEnd,
    pub frames: u16,
    pub without_rssi: u16,
    pub dropped: u16,
    pub rxq_full: u16,
    pub tail_capped: bool,
    pub refused: Option<Refused>,
    pub networks_total: u16,
    pub networks: [Option<Network>; MAX_LISTED],
}

/// One listed network with its 1-based place `index` among `total`.
#[derive(Clone, Copy, Debug)]
pub struct Listed<'a> {
    pub index: usize,
    pub total: u16,
    pub network: &'a Network,
}

/// What the scan needs from the bring-up: the efuse MAC address, the
/// firmware version (major, minor, sub, idx) and the channel inputs of
/// [`channel::set_channel`].
#[derive(Clone, Copy, Debug)]
pub struct ScanInputs<'a> {
    pub mac: [u8; 6],
    pub firmware: (u8, u8, u8, u8),
    pub channel: ChannelInputs<'a>,
}

/// A failed step (spec S2): `"core start"`, `"set channel"`, `"vif init"`,
/// `"scan start"` or `"scan"`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScanError {
    pub step: &'static str,
    pub error: AxError,
}

impl ScanReport {
    /// The listed networks, strongest first, numbered from 1.
    pub fn listed(&self) -> impl Iterator<Item = Listed<'_>> {
        let total = self.networks_total;
        self.networks
            .iter()
            .flatten()
            .enumerate()
            .map(move |(i, network)| Listed {
                index: i + 1,
                total,
                network,
            })
    }
}

impl fmt::Display for ScanEnd {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Ok => f.write_str("end ok"),
            Self::Status(status) => write!(f, "end status {status}"),
            Self::Stopped { ended: false } => f.write_str("no end, stopped"),
            Self::Stopped { ended: true } => f.write_str("no end, stopped and ended"),
        }
    }
}

/// Spec S13.3: the suffix the host check appends to the bring-up line, with
/// Stela's own `, RXQ not drained after 64 polls` after `RXQ full` when
/// [`ScanReport::tail_capped`].
impl fmt::Display for ScanReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            ", scan {}/{} channels in {} ms, {}: {} networks from {} frames ({} without RSSI)",
            self.entered,
            self.channels,
            self.elapsed_ms,
            self.end,
            self.networks_total,
            self.frames,
            self.without_rssi
        )?;
        if self.dropped != 0 {
            write!(f, ", dropped {}", self.dropped)?;
        }
        if self.rxq_full != 0 {
            write!(f, ", RXQ full {}", self.rxq_full)?;
        }
        if self.tail_capped {
            write!(f, ", RXQ not drained after {MAX_TAIL_POLLS} polls")?;
        }
        if let Some(r) = self.refused {
            write!(
                f,
                ", H2C {}/{} seq {} returned {}",
                r.class, r.func, r.seq, r.ret
            )?;
        }
        Ok(())
    }
}

/// Spec S13.4: `WIFI NETWORK <i>/<n> <band> ch <c> <rssi> dBm <bssid>
/// "<ssid>"`, `no RSSI` for `<rssi> dBm`, a hidden SSID as `<hidden>`. SSID
/// bytes 0x20-0x7E print as they are except `"` and `\`, which, as every
/// other byte, print as `\xNN` in lower-case hex.
impl fmt::Display for Listed<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let n = self.network;
        let band = match n.band {
            HwBand::G2 => "2G",
            HwBand::G5 => "5G",
            HwBand::G6 => "6G",
        };
        write!(
            f,
            "WIFI NETWORK {}/{} {band} ch {} ",
            self.index, self.total, n.channel
        )?;
        match n.signal {
            Some(dbm) => write!(f, "{dbm} dBm ")?,
            None => f.write_str("no RSSI ")?,
        }
        let b = n.bssid;
        write!(
            f,
            "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ",
            b[0], b[1], b[2], b[3], b[4], b[5]
        )?;
        if n.hidden {
            return f.write_str("<hidden>");
        }
        f.write_str("\"")?;
        let len = usize::from(n.ssid_len).min(n.ssid.len());
        for &byte in &n.ssid[..len] {
            match byte {
                b'"' | b'\\' => write!(f, "\\x{byte:02x}")?,
                0x20..=0x7E => write!(f, "{}", char::from(byte))?,
                _ => write!(f, "\\x{byte:02x}")?,
            }
        }
        f.write_str("\"")
    }
}

/// `RTW89_FW_VER_CODE(0, 27, 36, 0)`: 8852C's first firmware with
/// `SCAN_OFFLOAD` (`fw.c:898`).
const SCAN_OFFLOAD_MIN: (u8, u8, u8, u8) = (0, 27, 36, 0);
/// `R_AX_PORT_CFG_P0` and `B_AX_TSF_UDT_EN` (`reg.h:2381`, `:2398`).
const R_AX_PORT_CFG_P0: u32 = 0xC400;
const B_AX_TSF_UDT_EN: u32 = 1 << 3;
/// 8852CE's `mit_addr`, `R_AX_INT_MIT_RX_V1` (`rtw8852ce.c:61`,
/// `pci.h:916`).
const R_AX_INT_MIT_RX_V1: u32 = 0x1184;
/// 8852C's `edcca_regs` (`rtw8852c.c:252-257`): `R_SEG0R_EDCCA_LVL` with
/// `edcca_mask` `B_EDCCA_LVL_MSK0`, `edcca_p_mask` `B_EDCCA_LVL_MSK1` and
/// `ppdu_mask` `B_EDCCA_LVL_MSK3` (`reg.h:9689-9693`); `EDCCA_MAX`
/// (`phy.h:149`).
const R_SEG0R_EDCCA_LVL: u32 = 0x4840;
const B_EDCCA_LVL_MSK0: u32 = 0x0000_00FF;
const B_EDCCA_LVL_MSK1: u32 = 0x0000_FF00;
const B_EDCCA_LVL_MSK3: u32 = 0xFF00_0000;
const EDCCA_MAX: u32 = 249;
/// `R_AX_RX_FLTR_OPT` and `B_AX_RX_FLTR_CFG_MASK`, everything but
/// `B_AX_RX_MPDU_MAX_LEN_MASK` [21:16] (`reg.h:3327-3353`).
const R_AX_RX_FLTR_OPT: u32 = 0xCE20;
const B_AX_RX_FLTR_CFG_MASK: u32 = !0x003F_0000;
/// `DEFAULT_AX_RX_FLTR` (`reg.h:3348-3352`), `hal.rx_fltr` (`core.c:7020`),
/// less `B_AX_A_BCN_CHK_EN` b7, `B_AX_A_BC` b2 and `B_AX_A_A1_MATCH` b1
/// (`fw.c:10423-10425`).
const DEFAULT_AX_RX_FLTR: u32 = 0x0300_44BE;
const SCAN_RX_FLTR: u32 = DEFAULT_AX_RX_FLTR & !(1 << 7 | 1 << 2 | 1 << 1);
/// 8852C's `dig_regs` (`rtw8852c.c:228-231`): `R_SEG0R_PD` with
/// `B_SEG0R_PD_LOWER_BOUND_MSK` and `B_SEG0R_PD_SPATIAL_REUSE_EN_MSK`
/// (`reg.h:9685-9696`).
const R_SEG0R_PD: u32 = 0x481C;
const B_SEG0R_PD_LOWER_BOUND_MSK: u32 = 0x0000_07C0;
const B_SEG0R_PD_SPATIAL_REUSE_EN_MSK: u32 = 1 << 29;
/// `RTW89_C2H_SCAN_DONE_ACK_RETURN` (`fw.h:3962`): the bits of
/// ADD_SCANOFLD_CH's DONE_ACK return that fail it (`mac.c:5537-5539`).
const SCAN_DONE_ACK_RETURN: u8 = 0x3F;
/// SCANOFLD fails on any non-zero return (`data.err = !!h2c_return`).
const SCANOFLD_RETURN: u8 = 0xFF;
/// The tag of spec S6's channel, 2.4 GHz channel 1, until the first
/// ENTER_CH.
const BOOT_CHANNEL: Tag = Tag {
    band: HwBand::G2,
    channel: 1,
};
/// The most RXQ polls after END_SCAN or the stop: the drain ends earlier at
/// the first poll that finds RXQ empty.
const MAX_TAIL_POLLS: usize = 64;
/// The most frames held between ADD_SCANOFLD_CH's submit and SCANOFLD's
/// DONE_ACK, before the scanner takes them; past it they are dropped.
const MAX_EARLY: usize = super::receive::MAX_PENDING;
/// The most BSSIDs kept; a frame from a new BSSID past it is dropped.
const MAX_NETWORKS: usize = 512;

fn wifi_band(band: HwBand) -> Band {
    match band {
        HwBand::G2 => Band::Ghz2,
        HwBand::G5 => Band::Ghz5,
        HwBand::G6 => Band::Ghz6,
    }
}

fn hw_band(band: Band) -> HwBand {
    match band {
        Band::Ghz2 => HwBand::G2,
        Band::Ghz5 => HwBand::G5,
        Band::Ghz6 => HwBand::G6,
    }
}

/// A listed channel as `wifi::Channel`, at its cfg80211 frequency.
fn wifi_channel(c: &ScanChannel) -> Channel {
    Channel {
        band: wifi_band(c.band),
        number: u16::from(c.channel),
        frequency_mhz: frequency(c.band, c.channel),
    }
}

/// Spec S13.1's policy: reception on the listed channels only, and never a
/// probe (every channel is NO_IR).
struct PassiveList;

impl Policy for PassiveList {
    fn allow_receive(&mut self, channel: Channel) -> bool {
        self.lookup_channel(channel.band, channel.number) == Some(channel)
    }
    fn allow_probe(&mut self, _: Channel) -> bool {
        false
    }
    fn lookup_channel(&mut self, band: Band, number: u16) -> Option<Channel> {
        SCAN_CHANNELS
            .iter()
            .map(wifi_channel)
            .find(|c| c.band == band && c.number == number)
    }
}

/// A `wifi::scan` refusal as the step's error.
fn refusal(error: wscan::Error) -> AxError {
    AxError::Scan(match error {
        wscan::Error::Invalid(what) | wscan::Error::Unsupported(what) => what,
        wscan::Error::Busy => "scanner busy",
        wscan::Error::Stale => "stale scan ticket",
        wscan::Error::WrongPhase => "scanner in the wrong phase",
        wscan::Error::ClockRegressed => "scan clock went backwards",
        wscan::Error::Overflow => "scan counter overflow",
        wscan::Error::Regulatory => "scan channel not admitted",
        wscan::Error::Integrity => "scan frame failed its integrity check",
        wscan::Error::WrongPeer => "scan frame from the wrong peer",
        wscan::Error::WrongChannel => "scan frame on an unscanned channel",
        wscan::Error::TimedOut => "scan timed out",
    })
}

fn command(error: CommandError) -> AxError {
    match error {
        CommandError::Submit(e) | CommandError::Wait(e) => e,
    }
}

fn bump(counter: &mut u16) {
    *counter = counter.saturating_add(1);
}

/// One BSSID's record: the last `Bss` heard (its elements dropped), the
/// last RSSI measured on its channel, and the frames merged.
struct Heard {
    bss: wscan::result::Bss,
    signal: Option<i8>,
    frames: u16,
}

impl Heard {
    fn network(&self) -> Network {
        let bss = &self.bss;
        let name = bss.ssid.as_deref().unwrap_or_default();
        let mut ssid = [0; 32];
        let len = name.len().min(ssid.len());
        ssid[..len].copy_from_slice(&name[..len]);
        Network {
            bssid: bss.bssid.0,
            band: hw_band(bss.channel.band),
            channel: u8::try_from(bss.channel.number).unwrap_or(u8::MAX),
            signal: self.signal,
            ssid,
            ssid_len: len as u8,
            hidden: bss.hides_ssid(),
            frames: self.frames,
        }
    }
}

/// The running scan: the scanner and its ticket, the SCANOFLD_RSPs seen,
/// and the table of what was heard.
struct Scan {
    scanner: Scanner,
    ticket: Ticket,
    /// Whether `Scanner::submitted` ran; frames before it wait in `early`.
    submitted: bool,
    early: Vec<Received>,
    entered: u8,
    /// END_SCAN's status, once it arrived.
    end: Option<u8>,
    frames: u16,
    without_rssi: u16,
    dropped: u16,
    table: Vec<Heard>,
    /// When SCANOFLD start's DONE_ACK arrived.
    started_us: u64,
    /// The receiver's short frames before `Scanner::begin`.
    short_before: u16,
}

impl Scan {
    /// One event the session hands on: a SCANOFLD_RSP or a delivered frame.
    fn event(&mut self, event: Event<'_>) {
        match event {
            Event::C2h(C2h::ScanOfld(report)) => self.report(&report),
            // The ledger keeps every other C2H.
            Event::C2h(_) => {}
            Event::Frame(frame) if self.submitted => self.frame(frame),
            Event::Frame(frame) if self.early.len() < MAX_EARLY => self.early.push(frame),
            Event::Frame(_) => bump(&mut self.dropped),
        }
    }

    /// `rtw89_mac_c2h_scanofld_rsp`'s part that is not the receiver's tag:
    /// ENTER_CH counts a hop, the first END_SCAN ends the scan. LEAVE_CH,
    /// the operating-channel reasons and the rest change nothing here.
    fn report(&mut self, report: &ScanOfld) {
        match report.reason {
            SCAN_ENTER_CH => self.entered = self.entered.saturating_add(1),
            SCAN_END if self.end.is_none() => self.end = Some(report.status),
            _ => {}
        }
    }

    /// Spec S11.7 and S13.1-S13.2: the frame as an MPDU with its radio
    /// status, through `Scanner::observe`, into the table.
    fn frame(&mut self, frame: Received) {
        let radio = RadioStatus {
            band: wifi_band(frame.tag.band),
            channel: u16::from(frame.tag.channel),
            frequency_mhz: frequency(frame.tag.band, frame.tag.channel),
            signal_dbm: frame.signal,
            chain_signal_dbm: [frame.chains[0], frame.chains[1], None, None],
            rate: RawRate {
                layout: 0,
                words: [
                    u32::from(frame.data_rate),
                    u32::from(frame.gi_ltf),
                    u32::from(frame.bw),
                    0,
                ],
            },
            rx_rate: None,
            tsf: Some(Timestamp::Low32(frame.free_run)),
        };
        let measured = frame.signal.is_some();
        let Ok(mut mpdu) = Mpdu::new(frame.mpdu, radio, SecurityStatus::default()) else {
            bump(&mut self.dropped);
            return;
        };
        mpdu.fcs_error = frame.fcs_error;
        let Ok(mut bss) = self.scanner.observe(self.ticket, &mpdu, &mut PassiveList) else {
            bump(&mut self.dropped);
            return;
        };
        bss.elements = Vec::new();
        if let Some(heard) = self.table.iter_mut().find(|h| h.bss.bssid == bss.bssid) {
            bss.keep_name_of(&heard.bss);
            heard.signal = bss.signal_dbm.or(heard.signal);
            heard.bss = bss;
            bump(&mut heard.frames);
        } else if self.table.len() < MAX_NETWORKS {
            self.table.push(Heard {
                signal: bss.signal_dbm,
                bss,
                frames: 1,
            });
        } else {
            bump(&mut self.dropped);
            return;
        }
        bump(&mut self.frames);
        if !measured {
            bump(&mut self.without_rssi);
        }
    }

    /// Spec S13.2: every BSSID heard, strongest first (no RSSI last), then
    /// by BSSID; the first [`MAX_LISTED`] are kept.
    fn networks(&self) -> (u16, [Option<Network>; MAX_LISTED]) {
        let mut sorted: Vec<&Heard> = self.table.iter().collect();
        sorted.sort_by(|a, b| {
            b.signal
                .cmp(&a.signal)
                .then_with(|| a.bss.bssid.0.cmp(&b.bss.bssid.0))
        });
        let mut networks = [None; MAX_LISTED];
        for (slot, heard) in networks.iter_mut().zip(sorted) {
            *slot = Some(heard.network());
        }
        let total = u16::try_from(self.table.len()).unwrap_or(u16::MAX);
        (total, networks)
    }
}

/// `rtw89_phy_config_edcca(scan = true)` on PHY 0: the three levels read
/// (Linux keeps them for the restore Stela does not port), then each set
/// to `EDCCA_MAX`.
fn config_edcca<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    for mask in [B_EDCCA_LVL_MSK0, B_EDCCA_LVL_MSK1, B_EDCCA_LVL_MSK3] {
        bb::read32_mask(io, R_SEG0R_EDCCA_LVL, mask)?;
    }
    for mask in [B_EDCCA_LVL_MSK0, B_EDCCA_LVL_MSK1, B_EDCCA_LVL_MSK3] {
        bb::field32(io, R_SEG0R_EDCCA_LVL, mask, EDCCA_MAX)?;
    }
    Ok(())
}

/// `rtw89_mac_set_rx_fltr(0, rx_fltr)`: the MPDU length field kept, every
/// other bit from `rx_fltr`.
fn set_rx_fltr<I: AxIo + ?Sized>(io: &mut I, rx_fltr: u32) -> Result<(), AxError> {
    let val = io.read32(R_AX_RX_FLTR_OPT)?;
    io.write32(
        R_AX_RX_FLTR_OPT,
        (val & !B_AX_RX_FLTR_CFG_MASK) | (rx_fltr & B_AX_RX_FLTR_CFG_MASK),
    )
}

/// `rtw89_phy_dig_ctrl(pause = true)` on PHY 0, from `pause_dig` false: PD
/// lower bound 0 (`disabled_dm_bitmap` is 0), spatial reuse off.
fn dig_suspend<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    bb::field32(io, R_SEG0R_PD, B_SEG0R_PD_LOWER_BOUND_MSK, 0)?;
    bb::field32(io, R_SEG0R_PD, B_SEG0R_PD_SPATIAL_REUSE_EN_MSK, 0)
}

/// Whether `event` is END_SCAN.
fn is_end(event: &Event<'_>) -> bool {
    matches!(event, Event::C2h(C2h::ScanOfld(r)) if r.reason == SCAN_END)
}

/// Step `"scan start"`: the gate, spec S8, `Scanner::begin`, then
/// ADD_SCANOFLD_CH with its wait, `Scanner::validate_submission`, SCANOFLD
/// with its wait, and `submitted`.
///
/// # Safety
/// As [`run`].
unsafe fn start<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    inputs: &ScanInputs<'_>,
) -> Result<Scan, AxError> {
    if inputs.firmware < SCAN_OFFLOAD_MIN {
        return Err(AxError::Scan("firmware predates SCAN_OFFLOAD (0.27.36.0)"));
    }
    let mut scanner = Scanner::new(Mac(inputs.mac)).map_err(refusal)?;
    // `rtw89_mac_port_cfg_rx_sync(false)`.
    clear32(io, R_AX_PORT_CFG_P0, B_AX_TSF_UDT_EN)?;
    // `rtw89_core_scan_start`: RX interrupt mitigation off while scanning,
    // EDCCA at its maximum, then the address CAM (`INFO_CHANGE`, which v0
    // does not carry, so the bytes are vif init's).
    io.write32(R_AX_INT_MIT_RX_V1, 0)?;
    config_edcca(io)?;
    let drop = &mut |_: Event<'_>| {};
    let packet = session.h2c.addr_cam(inputs.mac, MAC_ID, PORT);
    // SAFETY: the caller's contract is `send`'s, word for word.
    unsafe { session.send(io, packet, drop) }?;
    set_rx_fltr(io, SCAN_RX_FLTR)?;
    dig_suspend(io)?;
    // Frames still waiting from core start and vif init are no results.
    session.rx.flush(drop);
    let mut policy = PassiveList;
    let request = Request::new(
        SCAN_CHANNELS.iter().map(wifi_channel).collect(),
        Vec::new(),
        Vec::new(),
    )
    .map_err(refusal)?;
    let ticket = scanner
        .begin(request, io.now_us()?, SCAN_WAIT_US, &mut policy)
        .map_err(refusal)?;
    let mut scan = Scan {
        scanner,
        ticket,
        submitted: false,
        early: Vec::new(),
        entered: 0,
        end: None,
        frames: 0,
        without_rssi: 0,
        dropped: 0,
        table: Vec::new(),
        started_us: 0,
        short_before: session.rx.counts.short_frames,
    };
    let packet = session.h2c.add_scanofld_ch(&SCAN_CHANNELS)?;
    // SAFETY: as above, `send_and_wait`'s.
    unsafe {
        session.send_and_wait(
            io,
            packet,
            H2C_FUNC_ADD_SCANOFLD_CH,
            SCAN_DONE_ACK_RETURN,
            &mut |e| scan.event(e),
        )
    }
    .map_err(command)?;
    // `Scanner`'s contract: the request is rechecked (budget and channel
    // policy) just before it is handed to the transport, SCANOFLD start,
    // which `submitted` then records. A budget already spent cancels it.
    scan.scanner
        .validate_submission(scan.ticket, io.now_us()?, &mut PassiveList)
        .map_err(refusal)?;
    let packet = session.h2c.scanofld(MAC_ID, PORT, true);
    // SAFETY: as above.
    unsafe {
        session.send_and_wait(io, packet, H2C_FUNC_SCANOFLD, SCANOFLD_RETURN, &mut |e| {
            scan.event(e)
        })
    }
    .map_err(command)?;
    scan.started_us = io.now_us()?;
    scan.scanner.submitted(scan.ticket).map_err(refusal)?;
    scan.submitted = true;
    for frame in core::mem::take(&mut scan.early) {
        scan.frame(frame);
    }
    Ok(scan)
}

/// Step `"scan"`: wait for END_SCAN, else stop as `rtw89_hw_scan_abort`
/// does; then drain RXQ until it is empty and only then flush the
/// receiver, so the last frames, and those whose PPDU status never came,
/// are kept; then the scanner's completion and the report.
///
/// # Safety
/// As [`run`].
unsafe fn finish<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    mut scan: Scan,
) -> Result<ScanReport, AxError> {
    if scan.end.is_none() {
        // SAFETY: the caller's contract is `wait`'s, word for word.
        unsafe { session.wait(io, SCAN_WAIT_US, &mut |e| scan.event(e), &mut is_end) }?;
    }
    let stopped = scan.end.is_none();
    if stopped {
        scan.scanner.cancel(scan.ticket).map_err(refusal)?;
        // `rtw89_hw_scan_offload(false)`: the stop's DONE_ACK completes
        // nothing in Linux (`mac.c:5540-5541` names the start's condition);
        // only END_SCAN ends its wait.
        let packet = session.h2c.scanofld(MAC_ID, PORT, false);
        // SAFETY: as above, `send`'s.
        unsafe { session.send(io, packet, &mut |e| scan.event(e)) }?;
        if scan.end.is_none() {
            // SAFETY: as above.
            unsafe { session.wait(io, STOP_WAIT_US, &mut |e| scan.event(e), &mut is_end) }?;
        }
    }
    let ended_us = io.now_us()?;
    // Capped unless a poll finds RXQ empty.
    let mut tail_capped = true;
    for _ in 0..MAX_TAIL_POLLS {
        // SAFETY: as above, `drain`'s.
        if unsafe { session.drain(io, &mut |e| scan.event(e)) }? == 0 {
            tail_capped = false;
            break;
        }
    }
    session.rx.flush(&mut |e| scan.event(e));
    let end = match scan.end {
        _ if stopped => ScanEnd::Stopped {
            ended: scan.end.is_some(),
        },
        Some(SCAN_STATUS_SUCCESS) => ScanEnd::Ok,
        Some(status) => ScanEnd::Status(status),
        None => ScanEnd::Stopped { ended: false },
    };
    let outcome = match end {
        ScanEnd::Ok => Outcome::Completed,
        ScanEnd::Status(status) => Outcome::Failed(u32::from(status)),
        ScanEnd::Stopped { .. } => Outcome::Aborted,
    };
    scan.scanner
        .completed(scan.ticket, outcome)
        .map_err(refusal)?;
    let proof = if scan.end.is_some() {
        Retirement::Confirmed
    } else {
        Retirement::Uncertain
    };
    scan.scanner.retire(scan.ticket, proof).map_err(refusal)?;
    let (networks_total, networks) = scan.networks();
    let elapsed_us = ended_us.saturating_sub(scan.started_us);
    Ok(ScanReport {
        channels: SCAN_CHANNELS.len() as u8,
        entered: scan.entered,
        elapsed_ms: u32::try_from(elapsed_us / 1000).unwrap_or(u32::MAX),
        end,
        frames: scan.frames,
        without_rssi: scan.without_rssi,
        dropped: scan.dropped.saturating_add(
            session
                .rx
                .counts
                .short_frames
                .saturating_sub(scan.short_before),
        ),
        rxq_full: session.rings().rxq_full_batches(),
        tail_capped,
        refused: session.ledger.refused,
        networks_total,
        networks,
    })
}

/// Spec S2 steps 1-7 (see the module documentation), each failing with its
/// step: `"core start"` ([`core_tail::run`]), `"set channel"` (2.4 GHz
/// channel 1), `"vif init"` ([`vif::init`] with `inputs.mac`), `"scan
/// start"` and `"scan"`. One [`Session`] carries every H2C, numbered on from
/// `h2c`. The first failure is returned and nothing after it runs; the
/// caller powers the chip off. A scan that never ends is stopped and
/// reported, not failed.
///
/// # Safety
/// The contract of `Ch12::submit` and `Rxq::poll` for `io` and `rings`:
/// `io` is the exclusive register window of the same RTL8852C requester
/// that `pci::pre_init` programmed with `rings`, `pre_init` succeeded, and
/// its contract still holds: its DMA domain isolates that requester, the
/// rings are coherent WB RAM with NoSnoop and ATS disabled, and one worker
/// owns the device and its queues.
pub unsafe fn run<I: AxIo + ?Sized>(
    io: &mut I,
    rings: &mut Rings,
    h2c: &mut H2c,
    inputs: &ScanInputs<'_>,
) -> Result<ScanReport, ScanError> {
    let at = |step: &'static str| move |error: AxError| ScanError { step, error };
    let mut session = Session::new(rings, h2c, BOOT_CHANNEL);
    // SAFETY: the caller's contract is `core_tail::run`'s, word for word.
    unsafe { core_tail::run(io, &mut session) }.map_err(at("core start"))?;
    let chan = Chan::new20(BOOT_CHANNEL.band, BOOT_CHANNEL.channel).map_err(at("set channel"))?;
    channel::set_channel(io, &chan, &inputs.channel).map_err(at("set channel"))?;
    // SAFETY: as above, `vif::init`'s.
    unsafe { vif::init(io, &mut session, inputs.mac) }.map_err(at("vif init"))?;
    // SAFETY: as above, `start`'s.
    let scan = unsafe { start(io, &mut session, inputs) }.map_err(at("scan start"))?;
    // SAFETY: as above, `finish`'s.
    unsafe { finish(io, &mut session, scan) }.map_err(at("scan"))
}

#[cfg(test)]
mod tests;

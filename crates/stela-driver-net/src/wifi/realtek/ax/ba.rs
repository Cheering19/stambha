// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! RX Block Ack on an RTL8852C station (M3 spec S13.2), the driver's half of
//! `rtw89_ops_ampdu_action`'s `IEEE80211_AMPDU_RX_START` and `_RX_STOP`
//! (`mac80211.c:1062-1069`):
//!
//! - [`BaCam`] is the static BA CAM's allocation for the station's one
//!   link: `rtw89_core_acquire_sta_ba_entry` and
//!   `rtw89_core_release_sta_ba_entry` (`core.c:5601-5670`) over 8852C's
//!   eight entries (`bacam_num`, `rtw8852c.c:3221`), the station's
//!   `ba_cam_list` kept in the order its entries were taken.
//! - [`start`] and [`stop`] are `rtw89_chip_h2c_ba_cam` for that link,
//!   8852C's `rtw89_fw_h2c_ba_cam` (`fw.c:2574-2651`, V0_EXT, built by
//!   `H2c::ba_cam`): an entry taken or freed, then its H2C sent without a
//!   wait, as `rtw89_h2c_tx(.., false)` sends it. Without an entry, all
//!   eight taken on a start or none held on a stop, nothing is sent and
//!   the call succeeds, as Linux's returns 0: the hardware's dynamic BA
//!   CAM, which `rtw89_fw_h2c_init_dynamic_ba_cam_v0_ext` set up at core
//!   start, covers the agreement. A send that fails leaves the entry as
//!   Linux leaves it, taken on a start and freed on a stop.
//! - [`RxStats`] is `rtwsta->tid_rx_stats`: `rtw89_core_tid_rx_stats_ctrl`
//!   (`core.c:3906-3920`) as each start and stop runs it,
//!   `rtw89_core_tid_rx_stats_reset` (`core.c:3922-3941`) after a pairwise
//!   SET_KEY (`mac80211.c:1007-1008`), and `rtw89_core_skb_pn_valid`
//!   (`core.c:3943-4006`), which `rtw89_core_rx` runs on every frame before
//!   mac80211 sees it (`core.c:4551`): on a TID with an agreement, a
//!   hardware-decrypted QoS data frame to the station whose CCMP or GCMP
//!   packet number is past the last one taken but whose sequence number is
//!   before the last one's is dropped.
//!
//! This module does not grant: an agreement (the station's own Block Ack
//! layer negotiates it, and the host reorders), an acknowledgement of any
//! H2C, TX aggregation (`rtw89_chip_h2c_ampdu_cmac_tbl`, NULL for 8852C),
//! the BA CAM v1 of other chips, a second link, or a replay check: a frame
//! the PN check passes still goes through the station's.
use super::command::Session;
use super::io::{AxError, AxIo};
use super::receive::{Event, Received};
use super::vif::MAC_ID;
use crate::wifi::frame::header_len;
use alloc::vec::Vec;

/// 8852C's `bacam_num` (`rtw8852c.c:3221`): the static BA CAM's entries.
pub const BA_CAM_ENTRIES: usize = 8;
/// `IEEE80211_NUM_TIDS`, the TIDs a station's statistics keep.
pub const TIDS: usize = 16;
/// `RTW89_SEC_KEY_TYPE_CCMP128` to `_GCMP256` (`core.h:321-324`): the
/// ciphers whose header `ccmp_hdr2pn` reads.
pub const SEC_KEY_TYPE_CCMP128: u8 = 6;
const SEC_KEY_TYPE_GCMP256: u8 = 9;
/// `IEEE80211_SN_MASK` and half of `IEEE80211_SN_MODULO`.
const SN_MASK: u16 = 0xFFF;
const SN_HALF: u16 = 0x800;
/// `IEEE80211_CCMP_HDR_LEN`.
const CCMP_HDR_LEN: usize = 8;

/// What a start or stop of a TID past 15 gets: the H2C's TID field is four
/// bits, and no station has more TIDs.
pub const TID_PAST_15: AxError = AxError::Mac("a TID past 15");

/// The static BA CAM entries the station holds (`cam_info->ba_cam_map`
/// with each `ba_cam_entry[i].tid`), and its `ba_cam_list`, oldest first.
/// With one station the two hold the same entries.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BaCam {
    /// Entry `i`'s TID while it is taken.
    tids: [Option<u8>; BA_CAM_ENTRIES],
    /// The taken entries in the order taken; `list[..len]`.
    list: [u8; BA_CAM_ENTRIES],
    len: usize,
}

impl BaCam {
    /// `rtw89_core_acquire_sta_ba_entry`: the first free entry, given to
    /// `tid`. When all are taken a TID of 0 or 5 takes the first entry whose
    /// TID is neither ("hardware will process the original tid
    /// automatically"), which leaves the station's list and goes to its end;
    /// any other TID gets `None` (`-ENOSPC`), as does 0 or 5 when every
    /// entry holds 0 or 5.
    pub fn acquire(&mut self, tid: u8) -> Option<u8> {
        let entry = match self.tids.iter().position(Option::is_none) {
            Some(free) => free,
            None if tid == 0 || tid == 5 => {
                let taken = self
                    .tids
                    .iter()
                    .position(|held| !matches!(held, Some(0 | 5)))?;
                self.unlist(taken as u8);
                taken
            }
            None => return None,
        };
        self.tids[entry] = Some(tid);
        self.list[self.len] = entry as u8;
        self.len += 1;
        Some(entry as u8)
    }

    /// `rtw89_core_release_sta_ba_entry`: the oldest entry of the
    /// station's list that `tid` holds, freed; `None` when it holds none
    /// (`-ENOENT`).
    pub fn release(&mut self, tid: u8) -> Option<u8> {
        let entry = self.list[..self.len]
            .iter()
            .copied()
            .find(|&entry| self.tids[usize::from(entry)] == Some(tid))?;
        self.unlist(entry);
        self.tids[usize::from(entry)] = None;
        Some(entry)
    }

    /// The TIDs of the entries held, in the station's list order.
    pub fn tids(&self) -> Vec<u8> {
        self.list[..self.len]
            .iter()
            .filter_map(|&entry| self.tids[usize::from(entry)])
            .collect()
    }

    /// `list_del` of `entry` from the station's list.
    fn unlist(&mut self, entry: u8) {
        if let Some(at) = self.list[..self.len].iter().position(|&e| e == entry) {
            self.list.copy_within(at + 1..self.len, at);
            self.len -= 1;
        }
    }
}

/// `struct rtw89_tid_stats` (`core.h:7530-7534`): the last packet number
/// taken, `None` for Linux's -1, and its sequence number.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct TidStats {
    last_pn: Option<u64>,
    last_sn: u16,
    started: bool,
}

impl Default for TidStats {
    fn default() -> Self {
        Self {
            last_pn: None,
            last_sn: SN_MASK,
            started: false,
        }
    }
}

impl TidStats {
    /// `__rtw89_core_tid_rx_stats_reset`.
    fn reset(&mut self) {
        self.last_pn = None;
        self.last_sn = SN_MASK;
    }
}

/// `rtwsta->tid_rx_stats` of the station's AP, and the frames
/// [`RxStats::valid`] dropped (saturating). A new station's are all
/// unstarted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RxStats {
    tids: [TidStats; TIDS],
    dropped: u32,
}

/// `ieee80211_sn_less`.
fn sn_less(a: u16, b: u16) -> bool {
    a.wrapping_sub(b) & SN_MASK > SN_HALF
}

impl RxStats {
    /// `rtw89_core_tid_rx_stats_ctrl`: a start resets `tid` and starts it,
    /// a stop stops it. A TID past 15 changes nothing.
    pub fn control(&mut self, tid: u8, enable: bool) {
        let Some(stats) = self.tids.get_mut(usize::from(tid)) else {
            return;
        };
        if enable {
            stats.reset();
            stats.started = true;
        } else {
            stats.started = false;
        }
    }

    /// `rtw89_core_tid_rx_stats_reset`: every started TID starts over.
    pub fn reset(&mut self) {
        for stats in self.tids.iter_mut().filter(|stats| stats.started) {
            stats.reset();
        }
    }

    /// The AP's station gone: a new one's statistics, every TID unstarted.
    /// The drops counted stay.
    pub fn forget(&mut self) {
        self.tids = [TidStats::default(); TIDS];
    }

    /// The frames [`RxStats::valid`] dropped.
    pub fn dropped(&self) -> u32 {
        self.dropped
    }

    /// `rtw89_core_skb_pn_valid` for `frame`, `associated` saying whether
    /// MAC ID 0 is the associated station's (`rtw89_assoc_link_rcu_dereference`):
    /// `false`, counted, for QoS data that the hardware decrypted (`HW_DEC`)
    /// for the station's own address (`A1_MATCH`), of the associated
    /// station's MAC ID, on a started TID, under CCMP or GCMP, whose packet
    /// number is past the TID's last but whose sequence number is before
    /// the last's; such a frame with a later sequence number becomes the
    /// last. Everything else is `true`, a frame too short for its CCMP
    /// header too, which Linux reads without a check and the station's own
    /// checks drop.
    pub fn valid(&mut self, associated: bool, frame: &Received) -> bool {
        let mpdu = &frame.mpdu;
        let Some(&[fc0, fc1]) = mpdu.get(..2) else {
            return true;
        };
        let fc = u16::from_le_bytes([fc0, fc1]);
        // ieee80211_is_data_qos.
        if fc & 0x008C != 0x0088 {
            return true;
        }
        if !frame.hw_dec || !frame.a1_match || !associated || frame.mac_id != MAC_ID {
            return true;
        }
        let hdrlen = header_len(fc);
        // The QoS control ends the header but for an HT control after it.
        let qos_at = if fc & 0x0300 == 0x0300 { 30 } else { 24 };
        let (Some(&qc), Some(sc), Some(ccmp)) = (
            mpdu.get(qos_at),
            mpdu.get(22..24),
            mpdu.get(hdrlen..hdrlen + CCMP_HDR_LEN),
        ) else {
            return true;
        };
        let Some(stats) = self.tids.get_mut(usize::from(qc & 0xF)) else {
            return true;
        };
        if !stats.started
            || !(SEC_KEY_TYPE_CCMP128..=SEC_KEY_TYPE_GCMP256).contains(&frame.sec_type)
        {
            return true;
        }
        // ieee80211_get_sn, and ccmp_hdr2pn (util.h:83-91): PN0, PN1, then
        // PN2-PN5 past the reserved and key ID bytes.
        let sn = u16::from_le_bytes([sc[0], sc[1]]) >> 4;
        let pn = u64::from_le_bytes([ccmp[0], ccmp[1], ccmp[4], ccmp[5], ccmp[6], ccmp[7], 0, 0]);
        if stats.last_pn.is_none_or(|last| pn > last) {
            if stats.last_pn.is_some() && sn_less(sn, stats.last_sn) {
                self.dropped = self.dropped.saturating_add(1);
                return false;
            }
            stats.last_sn = sn;
            stats.last_pn = Some(pn);
        }
        true
    }
}

/// RX_START's BA CAM for `tid` from `ssn` with a window of `window`:
/// [`BaCam::acquire`], then its H2C through `session` without a wait;
/// frames drained meanwhile go to `sink`. Without a free entry nothing is
/// sent. A failed send is returned with the entry still taken.
///
/// # Safety
/// The contract of `Ch12::submit` and `Rxq::poll` for `io` and `session`'s
/// rings, as `Session::send`'s.
pub unsafe fn start<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    cam: &mut BaCam,
    tid: u8,
    ssn: u16,
    window: u16,
    sink: &mut dyn FnMut(Event<'_>),
) -> Result<(), AxError> {
    let Some(entry) = cam.acquire(tid) else {
        return Ok(());
    };
    let packet = session.h2c.ba_cam(MAC_ID, tid, ssn, window, entry, true);
    // SAFETY: the caller's contract is `send`'s, word for word.
    unsafe { session.send(io, packet, sink) }.map(|_| ())
}

/// RX_STOP's BA CAM for `tid`: [`BaCam::release`], then the entry's stop
/// H2C as [`start`] sends one. Without an entry held nothing is sent. A
/// failed send is returned with the entry already free.
///
/// # Safety
/// As [`start`].
pub unsafe fn stop<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    cam: &mut BaCam,
    tid: u8,
    sink: &mut dyn FnMut(Event<'_>),
) -> Result<(), AxError> {
    let Some(entry) = cam.release(tid) else {
        return Ok(());
    };
    let packet = session.h2c.ba_cam(MAC_ID, tid, 0, 0, entry, false);
    // SAFETY: the caller's contract is `send`'s, word for word.
    unsafe { session.send(io, packet, sink) }.map(|_| ())
}

#[cfg(test)]
mod tests;

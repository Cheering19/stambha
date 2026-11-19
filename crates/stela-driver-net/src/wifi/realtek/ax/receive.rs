// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2020-2022  Realtek Corporation
//! An RTL8852C's RX dispatcher for the scan: each RXQ buffer goes where
//! `rtw89_core_rx` (`core.c:4527-4559`) sends it. A C2H is parsed and
//! handed on (`rtw89_core_rx_process_report`, `core.c:4087-4103`), and a
//! SCANOFLD_RSP with reason ENTER_CH or ENTER_OP first moves the channel
//! tag as `rtw89_mac_c2h_scanofld_rsp` does with `rtw89_chan_create` and
//! `rtw89_assign_entity_chan` (`mac.c:5369-5383`, `chan.c:199-215`). A
//! PPDU status is parsed by [`ppdu::parse`] and pairs the frames waiting
//! for it as `rtw89_core_rx_pending_skb` does (`core.c:4036-4057`), with
//! `rtw8852c_query_ppdu` and `rtw8852c_fill_freq_with_ppdu`
//! (`rtw8852c.c:2963-2996`) for the RSSI and the IE01 channel, and
//! `rtw89_correct_cck_chan` (`core.c:3187-3201`). A WIFI frame flushes the
//! queue when its PPDU count changes (`rtw89_core_flush_ppdu_rx_queue`,
//! `core.c:4477-4491`), is tagged with the scan channel as
//! `rtw89_core_update_rx_status` does during an offloaded scan
//! (`core.c:4363-4397`), and waits for its status only when its descriptor
//! is long and typed MGNT or DATA (`PPDU_FILTER_BITMAP`, `phy.h:147`). A
//! frame delivered without a matched status has no signal, as mac80211
//! gets `RX_FLAG_NO_SIGNAL_VAL` (`core.c:3816-3820`). Spec S10.2 and
//! S11.4-S11.7.
//!
//! Stela keeps only beacons and probe responses (FC type 0, subtype 8 or
//! 5) and counts every other WIFI frame as dropped (spec S1.10); Linux
//! matches and flushes each queued frame on its own, so the restriction
//! changes no kept frame's outcome. Unlike Linux's unbounded `rx_queue`,
//! at most [`MAX_PENDING`] frames wait; past that a frame is delivered at
//! once without a signal, ahead of the older frames still waiting, so
//! delivery is then out of arrival order. Each delivered frame loses its
//! 4-byte FCS (`RX_INCLUDES_FCS`, `core.c:7497`). Rate conversion,
//! radiotap, station statistics, the PN check and the MCC channel fix are
//! not ported: none applies to an unassociated scan. Device input is bounds-checked: a
//! malformed buffer is an error, a short frame or C2H is counted, and the
//! queue never grows past its bound. This module reads no register and
//! grants no DMA, no channel switch and no frame parsing past the frame
//! control byte.
use alloc::collections::VecDeque;
use alloc::vec::Vec;

use super::c2h::{self, C2h, SCAN_ENTER_CH, SCAN_ENTER_OP};
use super::chan::{HwBand, decode_chan_idx};
use super::io::AxError;
use super::ppdu::{self, MAX_PPDU_CNT, PhyPpdu, RateKey};
use super::rx::{
    self, FRAME_DATA, FRAME_MGNT, RX_TYPE_C2H, RX_TYPE_PPDU_STAT, RX_TYPE_WIFI, RxBuffer, RxDesc,
};

/// The most frames that wait for their PPDU status.
pub const MAX_PENDING: usize = 64;
/// `ppdu_sts.curr_rx_ppdu_cnt`'s initial `U8_MAX` (`core.c:6467`).
const NO_PPDU: u8 = 0xFF;
/// The 802.11 header and the FCS (`FCS_LEN`).
const HDR_LEN: usize = 24;
const FCS_LEN: usize = 4;
/// Frame-control byte 0: type [3:2] and subtype [7:4]; beacon and probe
/// response are management (type 0) subtypes 8 and 5.
const FC0_TYPE_SUBTYPE: u8 = 0xFC;
const FC0_BEACON: u8 = 0x80;
const FC0_PROBE_RESP: u8 = 0x50;
/// `RTW89_HW_RATE_OFDM6` (`core.h:349`): legacy indices below it are CCK.
const HW_RATE_OFDM6: u8 = 4;

/// The channel a frame is tagged with: `rx_status->band` and the channel
/// of `rx_status->freq`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tag {
    pub band: HwBand,
    pub channel: u8,
}

/// A received beacon or probe response. `mpdu` has no FCS. `signal` is
/// `None` when no PPDU status matched (`RX_FLAG_NO_SIGNAL_VAL`); `chains`
/// are paths A and B, `Some` exactly when a status matched. The rate
/// fields are the descriptor's raw `RX_DATARATE`, `RX_GI_LTF` and `BW`;
/// `free_run` is `FREERUN_CNT`. `fcs_error` is `CRC32_ERR || ICV_ERR`
/// (`RX_FLAG_FAILED_FCS_CRC`, `core.c:4396-4397`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Received {
    pub mpdu: Vec<u8>,
    pub tag: Tag,
    pub signal: Option<i8>,
    pub chains: [Option<i8>; 2],
    pub fcs_error: bool,
    pub data_rate: u16,
    pub gi_ltf: u8,
    pub bw: u8,
    pub free_run: u32,
}

/// What [`Receiver::buffer`] hands on: every parsed C2H, and every kept
/// frame when it is delivered.
#[derive(Debug)]
pub enum Event<'a> {
    C2h(C2h<'a>),
    Frame(Received),
}

/// What was dropped or seen, all saturating. `segments`: multi-buffer
/// frames, not assembled. `other_frames`: WIFI frames that are not beacons
/// or probe responses. `short_frames`: beacons and probe responses shorter
/// than a header and an FCS (or WIFI payloads without a frame-control
/// byte). `ppdu`: PPDU statuses processed (`curr` ≤ 8), including those
/// that found the queue empty. `ppdu_ignored`: PPDU statuses that came
/// before any WIFI frame (`curr` above `RTW89_MAX_PPDU_CNT`). `overflow`: frames delivered without a signal
/// because the queue was full; such a frame goes out ahead of the older
/// frames still waiting. `c2h_skipped`: C2Hs too short to parse.
/// Other packet types are dropped uncounted, as Linux drops them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RxCounts {
    pub segments: u16,
    pub other_frames: u16,
    pub short_frames: u16,
    pub ppdu: u16,
    pub ppdu_ignored: u16,
    pub overflow: u16,
    pub c2h_skipped: u16,
}

struct Pending {
    frame: Received,
    key: RateKey,
}

/// The dispatcher's state: the current and previous channel tag (Linux's
/// channel context and its `rtw89_chan_rcd`), `curr_rx_ppdu_cnt` for PHY 0
/// and the frames waiting for their status, oldest first.
pub struct Receiver {
    pub counts: RxCounts,
    cur: Tag,
    prev: Tag,
    curr_ppdu: u8,
    pending: VecDeque<Pending>,
}

fn bump(counter: &mut u16) {
    *counter = counter.saturating_add(1);
}

/// A SCANOFLD_RSP `band` as a tag band. The RTL8852C supports 6 GHz, so the
/// band is used as given (`mac.c:5310-5311`); the undefined 3 reads as
/// 2.4 GHz, as `rtw89_hw_to_nl80211_band`'s default does (`core.h:8469-8480`)
/// wherever Linux turns the channel into `rx_status`.
fn scan_band(band: u8) -> HwBand {
    match band {
        1 => HwBand::G5,
        2 => HwBand::G6,
        _ => HwBand::G2,
    }
}

/// `rtw8852c_query_ppdu` for a matched frame: the stronger path's RSSI as
/// the signal, both paths as chains, and the IE01 channel of a completed
/// walk. A signal of exactly 0 dBm is no signal, as
/// `rtw89_core_validate_rx_signal` flags it (`core.c:3816-3820`).
fn query_ppdu(phy: &PhyPpdu, frame: &mut Received) {
    let [a, b] = phy.rssi;
    let signal = ppdu::rssi_dbm(a.max(b));
    frame.signal = (signal != 0).then_some(signal);
    frame.chains = [Some(ppdu::rssi_dbm(a)), Some(ppdu::rssi_dbm(b))];
    if phy.valid && phy.chan_idx != 0 {
        let (band, channel) = decode_chan_idx(phy.chan_idx);
        frame.tag = Tag { band, channel };
    }
}

impl Receiver {
    /// A receiver tagging frames with `tag` (spec S6's channel) until the
    /// first ENTER_CH, with no PPDU seen yet.
    pub fn new(tag: Tag) -> Self {
        Self {
            counts: RxCounts::default(),
            cur: tag,
            prev: tag,
            curr_ppdu: NO_PPDU,
            pending: VecDeque::with_capacity(MAX_PENDING),
        }
    }

    /// The channel the next WIFI frame is tagged with.
    pub fn tag(&self) -> Tag {
        self.cur
    }

    /// How many frames wait for their PPDU status (tests only).
    #[cfg(test)]
    pub(crate) fn pending(&self) -> usize {
        self.pending.len()
    }

    /// Route one RXQ buffer (spec S11.4). A buffer `rx::parse` refuses is
    /// `Err(AxError::Rx(..))` and changes nothing; everything else is
    /// `Ok`, with what was dropped in [`Receiver::counts`].
    pub fn buffer(
        &mut self,
        buffer: &[u8],
        sink: &mut dyn FnMut(Event<'_>),
    ) -> Result<(), AxError> {
        let frame = match rx::parse(buffer)? {
            RxBuffer::Frame(frame) => frame,
            RxBuffer::Segment => {
                bump(&mut self.counts.segments);
                return Ok(());
            }
        };
        match frame.pkt_type {
            RX_TYPE_WIFI => self.wifi(&frame.desc, frame.payload, sink),
            RX_TYPE_PPDU_STAT => self.ppdu_stat(&frame.desc, frame.payload, sink),
            RX_TYPE_C2H => self.c2h(frame.payload, sink),
            // `rtw89_core_rx_process_report`'s default: freed.
            _ => {}
        }
        Ok(())
    }

    /// Deliver every waiting frame without a signal, as
    /// `rtw89_core_flush_ppdu_rx_queue` does. The scan calls it once RX
    /// ends, so frames whose status never came are not lost.
    pub fn flush(&mut self, sink: &mut dyn FnMut(Event<'_>)) {
        while let Some(p) = self.pending.pop_front() {
            sink(Event::Frame(p.frame));
        }
    }

    fn c2h(&mut self, payload: &[u8], sink: &mut dyn FnMut(Event<'_>)) {
        let Some(c2h) = c2h::parse(payload) else {
            bump(&mut self.counts.c2h_skipped);
            return;
        };
        if let C2h::ScanOfld(s) = c2h
            && matches!(s.reason, SCAN_ENTER_CH | SCAN_ENTER_OP)
        {
            // Not connected, an ENTER on 2G 1 finds the default `op_chan`
            // (`mac.c:5372-5376`); assigning it gives the same tag.
            self.prev = self.cur;
            self.cur = Tag {
                band: scan_band(s.band),
                channel: s.pri_ch,
            };
        }
        sink(Event::C2h(c2h));
    }

    fn wifi(&mut self, desc: &RxDesc, payload: &[u8], sink: &mut dyn FnMut(Event<'_>)) {
        if self.curr_ppdu != desc.ppdu_cnt {
            self.flush(sink);
            self.curr_ppdu = desc.ppdu_cnt;
        }
        let Some(&fc0) = payload.first() else {
            bump(&mut self.counts.short_frames);
            return;
        };
        if !matches!(fc0 & FC0_TYPE_SUBTYPE, FC0_BEACON | FC0_PROBE_RESP) {
            bump(&mut self.counts.other_frames);
            return;
        }
        let Some(len) = payload
            .len()
            .checked_sub(FCS_LEN)
            .filter(|&len| len >= HDR_LEN)
        else {
            bump(&mut self.counts.short_frames);
            return;
        };
        let frame = Received {
            mpdu: payload[..len].to_vec(),
            tag: self.cur,
            signal: None,
            chains: [None; 2],
            fcs_error: desc.crc32_err || desc.icv_err,
            data_rate: desc.data_rate,
            gi_ltf: desc.gi_ltf,
            bw: desc.bw,
            free_run: desc.free_run,
        };
        if !(desc.long && matches!(desc.frame_type, Some(FRAME_MGNT | FRAME_DATA))) {
            sink(Event::Frame(frame));
            return;
        }
        if self.pending.len() >= MAX_PENDING {
            bump(&mut self.counts.overflow);
            sink(Event::Frame(frame));
            return;
        }
        self.pending.push_back(Pending {
            frame,
            key: ppdu::frame_key(desc),
        });
    }

    fn ppdu_stat(&mut self, desc: &RxDesc, payload: &[u8], sink: &mut dyn FnMut(Event<'_>)) {
        if self.curr_ppdu > MAX_PPDU_CNT {
            bump(&mut self.counts.ppdu_ignored);
            return;
        }
        bump(&mut self.counts.ppdu);
        let phy = ppdu::parse(desc, payload);
        while let Some(Pending { mut frame, key }) = self.pending.pop_front() {
            if ppdu::matches(self.curr_ppdu, desc, &key) {
                query_ppdu(&phy, &mut frame);
            }
            // `rtw89_correct_cck_chan`, with the record as it is now.
            if frame.tag.band != HwBand::G2 && key.legacy && key.rate_idx < HW_RATE_OFDM6 {
                frame.tag = self.prev;
            }
            sink(Event::Frame(frame));
        }
    }
}

#[cfg(test)]
mod tests;

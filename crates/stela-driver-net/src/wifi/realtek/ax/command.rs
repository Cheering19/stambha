// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! The firmware command channel's first H2C: `rtw89_fw_h2c_set_ofld_cfg`
//! (`fw.c:5320`) submitted on CH12 as `rtw89_h2c_tx` does (`core.c:1373`),
//! then a polled RXQ wait for its acknowledgements, matched as the class 0
//! handlers `rtw89_mac_c2h_rec_ack` and `rtw89_mac_c2h_done_ack`
//! (`mac.c:5459`, `:5495`) decode them after `rtw89_fw_c2h_parse_attr`
//! (`fw.c:9043`). [`count_c2h`] reads RXQ once more without waiting and
//! only counts the C2Hs it holds.
//!
//! Linux does not wait here: `rtw89_h2c_tx` returns after the kick, and
//! `rtw89_fw_c2h_cmd_handle` drops every C2H before `RTW89_FLAG_RUNNING`
//! (`fw.c:9097`). The wait is Stela's own (spec S1.9-S1.10), budgeted at
//! Linux's `RTW89_WAIT_FOR_COND_TIMEOUT` (`core.h:5930`). Nothing received is
//! acted on: unrelated C2Hs and frames are only counted, a plain firmware log
//! is kept as text, and formatted logs are not rendered. This module grants
//! no interrupts, no C2H dispatch, no other H2C and no DMA beyond the
//! contracts of `Ch12::submit` and `Rxq::poll`.
use super::c2h::{self, C2h};
use super::h2c::{H2C_CAT_MAC, H2C_CL_MAC_FW_OFLD, H2C_FUNC_OFLD_CFG, H2c};
use super::io::{AxError, AxIo, poll_sample};
use super::pci::Rings;
use super::rx::{self, RxBuffer};

/// `RTW89_WAIT_FOR_COND_TIMEOUT` (`core.h:5930`): 100 ms.
pub const ACK_WAIT_US: u64 = 100_000;
/// The RXQ index sampling interval (spec S1.9).
pub const ACK_POLL_US: u64 = 50;

/// `RTW89_CORE_RX_TYPE_C2H` (`core.h:236`).
const RX_TYPE_C2H: u8 = 10;
const LOG_TEXT_BYTES: usize = 48;

/// The first `LOG_TEXT_BYTES` printable ASCII bytes of a plain firmware
/// log; every other byte is dropped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogText {
    bytes: [u8; LOG_TEXT_BYTES],
    len: u8,
}

impl LogText {
    fn new(raw: &[u8]) -> Self {
        let mut text = Self {
            bytes: [0; LOG_TEXT_BYTES],
            len: 0,
        };
        for &b in raw.iter().filter(|b| matches!(b, 0x20..=0x7E)) {
            if usize::from(text.len) == LOG_TEXT_BYTES {
                break;
            }
            text.bytes[usize::from(text.len)] = b;
            text.len += 1;
        }
        text
    }

    pub fn as_str(&self) -> &str {
        // Only printable ASCII is stored, which is always UTF-8.
        core::str::from_utf8(&self.bytes[..usize::from(self.len)]).unwrap_or_default()
    }
}

/// What the firmware answered to an H2C within the wait.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct H2cAck {
    /// A REC_ACK matching the H2C's cat, class, func and seq arrived.
    pub rec: bool,
    /// A matching DONE_ACK arrived (with return 0, or the command fails).
    pub done: bool,
    /// C2Hs that matched nothing kept and were not acks: other classes and
    /// functions, formatted logs and plain logs after the first. Saturates.
    pub other_c2h: u8,
    /// Class 0 REC_ACKs and DONE_ACKs for another cat, class, func or seq.
    /// Saturates.
    pub unmatched_ack: u8,
    /// RXQ buffers that were not one whole C2H: multi-buffer segments,
    /// non-C2H packet types and C2Hs too short to parse. Saturates.
    pub skipped: u8,
    /// The first plain firmware log.
    pub log: Option<LogText>,
}

/// Why a command failed, split as the bring-up names its steps (spec S11).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandError {
    /// The H2C was not sent, or the firmware refused it (DONE_ACK return
    /// not 0): step `"h2c"`.
    Submit(AxError),
    /// No matching ack arrived in time, or RXQ failed: step `"c2h"`.
    Wait(AxError),
}

/// The wait's record of what RXQ delivered.
struct Seen {
    ack: H2cAck,
    /// The first matching DONE_ACK's `H2C_RETURN`.
    done_ret: u8,
}

impl Seen {
    /// One RXQ buffer. Only a malformed buffer is an error.
    fn record(&mut self, seq: u8, buffer: &[u8]) -> Result<(), AxError> {
        let ours = |cat, class, func, s| {
            (cat, class, func, s) == (H2C_CAT_MAC, H2C_CL_MAC_FW_OFLD, H2C_FUNC_OFLD_CFG, seq)
        };
        let ack = &mut self.ack;
        let frame = match rx::parse(buffer)? {
            RxBuffer::Frame(frame) if frame.pkt_type == RX_TYPE_C2H => frame,
            RxBuffer::Frame(_) | RxBuffer::Segment => {
                ack.skipped = ack.skipped.saturating_add(1);
                return Ok(());
            }
        };
        match c2h::parse(frame.payload) {
            None => ack.skipped = ack.skipped.saturating_add(1),
            Some(C2h::RecAck {
                cat,
                class,
                func,
                seq: s,
            }) if ours(cat, class, func, s) => ack.rec = true,
            Some(C2h::DoneAck {
                cat,
                class,
                func,
                ret,
                seq: s,
            }) if ours(cat, class, func, s) => {
                if !ack.done {
                    ack.done = true;
                    self.done_ret = ret;
                }
            }
            Some(C2h::RecAck { .. } | C2h::DoneAck { .. }) => {
                ack.unmatched_ack = ack.unmatched_ack.saturating_add(1)
            }
            Some(C2h::Log(text)) if ack.log.is_none() => ack.log = Some(LogText::new(text)),
            Some(C2h::Log(_) | C2h::FormattedLog | C2h::Other { .. }) => {
                ack.other_c2h = ack.other_c2h.saturating_add(1)
            }
        }
        Ok(())
    }
}

/// Send OFLD_CFG with `h2c`'s next sequence number on CH12, then poll RXQ
/// every `ACK_POLL_US` until its DONE_ACK arrives or `ACK_WAIT_US` passes
/// (spec S1.9-S1.10). A REC_ACK alone succeeds once the wait expires; a
/// DONE_ACK with a non-zero return is `Submit(H2cRejected)`; no matching ack
/// is `Wait(NoAck)` with what did arrive (the counts of [`H2cAck`]) and the
/// last RXQ hardware index read; an RXQ or clock failure is `Wait`. A malformed RXQ
/// buffer wins over any ack already seen, in the same batch or an earlier
/// one: a DONE_ACK followed by a buffer shorter than its descriptor is
/// `Wait(Rx)`, not success. A failed submit leaves RXQ untouched.
///
/// # Safety
/// The contract of `Ch12::submit` and `Rxq::poll`: `io` is the exclusive
/// register window of the same RTL8852C requester that `pci::pre_init`
/// programmed with `rings`, and the `pre_init` contract still holds: its DMA
/// domain isolates that requester, the rings are coherent WB RAM with
/// NoSnoop and ATS disabled, and one worker owns the device and its queues.
pub unsafe fn ofld_cfg<I: AxIo + ?Sized>(
    io: &mut I,
    rings: &mut Rings,
    h2c: &mut H2c,
) -> Result<H2cAck, CommandError> {
    let (seq, packet) = h2c.ofld_cfg();
    // SAFETY: the caller's contract is `submit`'s, word for word.
    unsafe { rings.ch12().submit(io, &packet) }.map_err(CommandError::Submit)?;
    let mut seen = Seen {
        ack: H2cAck {
            rec: false,
            done: false,
            other_c2h: 0,
            unmatched_ack: 0,
            skipped: 0,
            log: None,
        },
        done_ret: 0,
    };
    // One RXQ for the whole wait: it holds the reusable buffer copy.
    let mut rxq = rings.rxq();
    // Met or expired, `seen` holds every ack that arrived.
    poll_sample(
        io,
        ACK_POLL_US,
        ACK_WAIT_US,
        |io| {
            // SAFETY: the caller's contract is `poll`'s, word for word, and
            // the submit above succeeded, which only a successful `pre_init`
            // of these rings allows: RXQ is programmed and started.
            unsafe { rxq.poll(io, &mut |buffer| seen.record(seq, buffer)) }?;
            Ok(seen.ack.done)
        },
        |done| done,
    )
    .map_err(CommandError::Wait)?;
    let ack = seen.ack;
    if ack.done && seen.done_ret != 0 {
        return Err(CommandError::Submit(AxError::H2cRejected {
            class: H2C_CL_MAC_FW_OFLD,
            func: H2C_FUNC_OFLD_CFG,
            seq,
            ret: seen.done_ret,
        }));
    }
    if !ack.done && !ack.rec {
        return Err(CommandError::Wait(AxError::NoAck {
            class: H2C_CL_MAC_FW_OFLD,
            func: H2C_FUNC_OFLD_CFG,
            seq,
            other_c2h: ack.other_c2h,
            unmatched_ack: ack.unmatched_ack,
            skipped: ack.skipped,
            rxq_hw: rxq.hardware_index(),
        }));
    }
    Ok(ack)
}

/// The C2Hs that arrived since RXQ was last read: one `Rxq::poll` with no
/// wait, counting the frames of the C2H packet type (spec S1.10, S11.3).
/// Nothing is parsed past the RX descriptor and nothing is acted on; other
/// frames and segments are consumed uncounted. A malformed buffer, or an
/// RXQ failure, is the error. The count saturates. Stela's own: Linux
/// reads RXQ only under NAPI.
///
/// # Safety
/// The contract of `Rxq::poll`: `io` is the exclusive register window of
/// the same RTL8852C requester that `pci::pre_init` programmed with `rings`,
/// and the `pre_init` contract still holds: its DMA domain isolates that
/// requester, the rings are coherent WB RAM with NoSnoop and ATS disabled,
/// and one worker owns the device and its queues. `pre_init` must have
/// succeeded on these rings.
pub unsafe fn count_c2h<I: AxIo + ?Sized>(io: &mut I, rings: &mut Rings) -> Result<u8, AxError> {
    let mut count: u8 = 0;
    // SAFETY: the caller's contract is `poll`'s, word for word.
    unsafe {
        rings.rxq().poll(io, &mut |buffer| {
            if let RxBuffer::Frame(frame) = rx::parse(buffer)?
                && frame.pkt_type == RX_TYPE_C2H
            {
                count = count.saturating_add(1);
            }
            Ok(())
        })
    }?;
    Ok(count)
}

#[cfg(test)]
mod tests;

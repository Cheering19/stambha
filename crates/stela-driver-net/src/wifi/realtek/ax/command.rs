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
//! is kept as text, and formatted logs are not rendered.
//!
//! Every H2C after `rtw89_phy_dm_init` goes through one [`Session`] (M2c
//! spec S9.1): `rtw89_h2c_tx`'s CH12 submit, then RXQ drained through the
//! session's [`Receiver`], so acks, scan reports, beacon filter reports
//! and frames are taken in RXQ order whatever wait is running, and none is
//! dropped. Its acks are decoded as `rtw89_mac_c2h_rec_ack` and
//! `rtw89_mac_c2h_done_ack` do (`mac.c:5459-5554`).
//! [`Session::send_and_wait_for`] waits for one of two things ([`Wait`]):
//!
//! - a DONE_ACK matched on cat, class and func but not seq, as Linux's
//!   `rtw89_complete_cond` does for ADD_SCANOFLD_CH and SCANOFLD (M2c spec
//!   S1.13); [`Session::send_and_wait`] is that wait on FW_OFLD. The same
//!   wait on another class (the address CAM's, M3 spec S12.5) is Stela's
//!   own: Linux waits for no other DONE_ACK.
//! - the H2C's REC_ACK, matched on cat, class, func and seq (M3 spec S1.12,
//!   S12.5). Also Stela's own: Linux's REC_ACK handler only counts SCANOFLD
//!   receipts (`mac.c:5459-5492`).
//!
//! Every other DONE_ACK, which Linux ignores, is recorded in the [`Ledger`],
//! the first non-zero return as its refusal (M2c spec S1.14).
//!
//! This module grants no interrupts, no C2H handler beyond the ack
//! decoding, no H2C it builds itself past OFLD_CFG (the session sends what
//! its caller built) and no DMA beyond the contracts of `Ch12::submit` and
//! `Rxq::poll`.
use super::c2h::{self, C2h};
use super::h2c::{H2C_CAT_MAC, H2C_CL_MAC_FW_OFLD, H2C_FUNC_OFLD_CFG, H2c};
use super::io::{AxError, AxIo, poll_sample};
use super::pci::Rings;
use super::receive::{Event, Receiver, Tag};
use super::rx::{self, RX_TYPE_C2H, RxBuffer};
use alloc::vec::Vec;

/// `RTW89_WAIT_FOR_COND_TIMEOUT` (`core.h:5930`): 100 ms.
pub const ACK_WAIT_US: u64 = 100_000;
/// The RXQ index sampling interval (spec S1.9).
pub const ACK_POLL_US: u64 = 50;

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
            Some(
                C2h::Log(_)
                | C2h::FormattedLog
                | C2h::ScanOfld(_)
                | C2h::BeaconFilter(_)
                | C2h::Other { .. },
            ) => ack.other_c2h = ack.other_c2h.saturating_add(1),
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

/// The first DONE_ACK with a non-zero `H2C_RETURN` that no
/// [`Session::send_and_wait`] consumed (spec S1.14): the H2C's class, func
/// and seq as the DONE_ACK names them, and the return.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Refused {
    pub class: u8,
    pub func: u8,
    pub seq: u8,
    pub ret: u8,
}

/// Every C2H a [`Session`] has drained, counted by kind (saturating), and
/// the first refusal. An ack a wait consumed is counted in `rec_acks` or
/// `done_acks`, and a consumed DONE_ACK never becomes `refused`.
/// SCANOFLD_RSPs and BCNFLTR_RPTs are not counted here: they go on to the
/// sink, as frames do.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Ledger {
    pub rec_acks: u16,
    pub done_acks: u16,
    /// Plain and formatted firmware logs.
    pub logs: u16,
    /// C2Hs of any other cat, class or func.
    pub other_c2h: u16,
    pub refused: Option<Refused>,
}

/// What [`Session::send_and_wait_for`] waits for after the submit, within
/// `ACK_WAIT_US`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wait {
    /// A DONE_ACK for H2C cat 1 (`H2C_CAT_MAC`) / `class` / `func`, whatever
    /// its seq, as `rtw89_complete_cond` completes a wait. It fails the
    /// command when `ret & mask` is not 0 (0x3F is
    /// `RTW89_C2H_SCAN_DONE_ACK_RETURN`, `mac.c:5537-5539`; 0xFF the whole
    /// return).
    Done { class: u8, func: u8, mask: u8 },
    /// The REC_ACK naming the packet itself: the cat, class and func of its
    /// FWCMD header and its seq.
    Received,
}

/// The ack a running [`Session::send_and_wait_for`] waits for, and what
/// else arrived meanwhile (saturating, for `NoAck`).
struct Awaited {
    cat: u8,
    class: u8,
    func: u8,
    /// `Some` for a REC_ACK wait: the seq it must name.
    receipt: Option<u8>,
    /// The waited-for ack, once it arrived: a DONE_ACK's raw `H2C_RETURN`,
    /// or 0 for a REC_ACK.
    met: Option<u8>,
    /// C2Hs that were not acks, scan and beacon filter reports included.
    other_c2h: u8,
    /// Acks for another H2C.
    unmatched_ack: u8,
}

impl Awaited {
    /// Whether an ack names the awaited H2C: its cat, class and func, and
    /// for a REC_ACK wait its seq. A REC_ACK naming the H2C a DONE_ACK wait
    /// waits for is that H2C's own receipt, so it is not unmatched.
    fn names(&self, cat: u8, class: u8, func: u8, seq: u8) -> bool {
        (cat, class, func) == (self.cat, self.class, self.func)
            && self.receipt.is_none_or(|s| s == seq)
    }
}

/// `H2C_HDR_CAT` [1:0], `H2C_HDR_CLASS` [7:2] and `H2C_HDR_FUNC` [15:8] of
/// the FWCMD header after a CH12 packet's 16-byte descriptor
/// (`fw.h:4673-4676`), as `H2c::packet_with` lays it out.
fn h2c_header(bytes: &[u8]) -> Option<(u8, u8, u8)> {
    let hdr0 = u32::from_le_bytes(bytes.get(16..20)?.try_into().ok()?);
    Some((
        (hdr0 & 0x3) as u8,
        ((hdr0 >> 2) & 0x3F) as u8,
        ((hdr0 >> 8) & 0xFF) as u8,
    ))
}

fn bump(counter: &mut u16) {
    *counter = counter.saturating_add(1);
}

fn bump8(counter: &mut u8) {
    *counter = counter.saturating_add(1);
}

/// One event from the receiver: C2Hs pass the ledger (and the running wait)
/// first; SCANOFLD_RSPs, BCNFLTR_RPTs and frames go on to `sink`.
fn account(
    ledger: &mut Ledger,
    awaited: &mut Option<Awaited>,
    event: Event<'_>,
    sink: &mut dyn FnMut(Event<'_>),
) {
    let c2h = match event {
        Event::Frame(_) => return sink(event),
        Event::C2h(c2h) => c2h,
    };
    match c2h {
        C2h::RecAck {
            cat,
            class,
            func,
            seq,
        } => {
            bump(&mut ledger.rec_acks);
            if let Some(a) = awaited {
                if !a.names(cat, class, func, seq) {
                    bump8(&mut a.unmatched_ack);
                } else if a.receipt.is_some() && a.met.is_none() {
                    a.met = Some(0);
                }
            }
        }
        C2h::DoneAck {
            cat,
            class,
            func,
            ret,
            seq,
        } => {
            bump(&mut ledger.done_acks);
            if let Some(a) = awaited {
                if a.receipt.is_none() && a.met.is_none() && a.names(cat, class, func, seq) {
                    a.met = Some(ret);
                    return;
                }
                bump8(&mut a.unmatched_ack);
            }
            if ret != 0 && ledger.refused.is_none() {
                ledger.refused = Some(Refused {
                    class,
                    func,
                    seq,
                    ret,
                });
            }
        }
        C2h::Log(_) | C2h::FormattedLog => {
            bump(&mut ledger.logs);
            if let Some(a) = awaited {
                bump8(&mut a.other_c2h);
            }
        }
        C2h::Other { .. } => {
            bump(&mut ledger.other_c2h);
            if let Some(a) = awaited {
                bump8(&mut a.other_c2h);
            }
        }
        C2h::ScanOfld(_) | C2h::BeaconFilter(_) => {
            if let Some(a) = awaited {
                bump8(&mut a.other_c2h);
            }
            sink(Event::C2h(c2h));
        }
    }
}

/// A [`Session`]'s own state between two sessions on the same rings and H2C
/// sequence ([`Session::park`], [`Session::resume`]): the receiver, the
/// ledger, and the last in-range RXQ hardware index a drain read.
pub struct Parked {
    pub rx: Receiver,
    pub ledger: Ledger,
    rxq_hw: u16,
}

/// The command channel after `rtw89_phy_dm_init`: CH12 for every H2C, RXQ
/// drained through one [`Receiver`] after each and during every wait. The
/// H2C sequence is the caller's `h2c`, so it continues from wherever the
/// earlier steps left it.
pub struct Session<'r> {
    rings: &'r mut Rings,
    pub h2c: &'r mut H2c,
    pub rx: Receiver,
    pub ledger: Ledger,
    awaited: Option<Awaited>,
    /// The last in-range RXQ hardware index a drain read.
    rxq_hw: u16,
}

impl<'r> Session<'r> {
    /// A session on `rings`, whose RXQ `pci::pre_init` started, numbering
    /// H2Cs from `h2c`, with a receiver tagging frames with `tag` until the
    /// firmware reports a channel.
    pub fn new(rings: &'r mut Rings, h2c: &'r mut H2c, tag: Tag) -> Self {
        Self {
            rings,
            h2c,
            rx: Receiver::new(tag),
            ledger: Ledger::default(),
            awaited: None,
            rxq_hw: 0,
        }
    }

    /// A session on the same rings and H2C sequence as the one `parked`
    /// ended: its receiver (with the frames still waiting for their PPDU
    /// status), its ledger and its last RXQ hardware index go on.
    pub fn resume(rings: &'r mut Rings, h2c: &'r mut H2c, parked: Parked) -> Self {
        let Parked { rx, ledger, rxq_hw } = parked;
        Self {
            rings,
            h2c,
            rx,
            ledger,
            awaited: None,
            rxq_hw,
        }
    }

    /// End the session, keeping what [`Session::resume`] goes on from. No
    /// wait runs between two calls, so nothing is awaited.
    pub fn park(self) -> Parked {
        Parked {
            rx: self.rx,
            ledger: self.ledger,
            rxq_hw: self.rxq_hw,
        }
    }

    pub fn rings(&self) -> &Rings {
        self.rings
    }

    /// Submit `packet` (a builder's `(seq, bytes)`) on CH12 as
    /// `rtw89_h2c_tx` does, then drain RXQ once without waiting. Returns
    /// the seq. A failed submit leaves RXQ untouched.
    ///
    /// # Safety
    /// The contract of `Ch12::submit` and `Rxq::poll`: `io` is the exclusive
    /// register window of the same RTL8852C requester that `pci::pre_init`
    /// programmed with this session's rings, and the `pre_init` contract
    /// still holds: its DMA domain isolates that requester, the rings are
    /// coherent WB RAM with NoSnoop and ATS disabled, and one worker owns the
    /// device and its queues.
    pub unsafe fn send<I: AxIo + ?Sized>(
        &mut self,
        io: &mut I,
        packet: (u8, Vec<u8>),
        sink: &mut dyn FnMut(Event<'_>),
    ) -> Result<u8, AxError> {
        let (seq, bytes) = packet;
        // SAFETY: the caller's contract is `submit`'s, word for word.
        unsafe { self.rings.ch12().submit(io, &bytes) }?;
        // SAFETY: the caller's contract is `poll`'s; the submit succeeded,
        // which only a successful `pre_init` of these rings allows.
        unsafe { self.drain(io, sink) }?;
        Ok(seq)
    }

    /// One `Rxq::poll`: every filled RXQ entry goes through the receiver,
    /// then the ledger, then `sink` (spec S9.1). Returns the number of RXQ
    /// entries consumed, 0 when RXQ held none. A malformed buffer ends the
    /// drain with its error after the buffers before it were handled.
    ///
    /// # Safety
    /// As [`Session::send`], and `pre_init` must have succeeded on these
    /// rings.
    pub unsafe fn drain<I: AxIo + ?Sized>(
        &mut self,
        io: &mut I,
        sink: &mut dyn FnMut(Event<'_>),
    ) -> Result<usize, AxError> {
        let Self {
            rings,
            rx,
            ledger,
            awaited,
            rxq_hw,
            ..
        } = self;
        let mut rxq = rings.rxq();
        // SAFETY: the caller's contract is `poll`'s, word for word.
        let polled = unsafe {
            rxq.poll(io, &mut |buffer| {
                rx.buffer(buffer, &mut |event| account(ledger, awaited, event, sink))
            })
        };
        *rxq_hw = rxq.hardware_index();
        polled
    }

    /// Drain RXQ every `ACK_POLL_US` until `done` accepts an event handed to
    /// `sink` (`Ok(true)`, after the rest of that batch) or `budget_us`
    /// passes (`Ok(false)`). `done` sees each SCANOFLD_RSP and frame just
    /// before `sink` does. A drain or clock failure is the error.
    ///
    /// # Safety
    /// As [`Session::drain`].
    pub unsafe fn wait<I: AxIo + ?Sized>(
        &mut self,
        io: &mut I,
        budget_us: u64,
        sink: &mut dyn FnMut(Event<'_>),
        done: &mut dyn FnMut(&Event<'_>) -> bool,
    ) -> Result<bool, AxError> {
        let mut met = false;
        poll_sample(
            io,
            ACK_POLL_US,
            budget_us,
            |io| {
                let mut each = |event: Event<'_>| {
                    if !met && done(&event) {
                        met = true;
                    }
                    sink(event);
                };
                // SAFETY: the caller's contract is `drain`'s.
                unsafe { self.drain(io, &mut each) }?;
                Ok(met)
            },
            |met| met,
        )?;
        Ok(met)
    }

    /// [`Session::send_and_wait_for`] waiting for a DONE_ACK for cat 1 /
    /// FW_OFLD (class 9) / `func` whatever its seq, as
    /// `rtw89_mac_c2h_done_ack` completes the wait for ADD_SCANOFLD_CH and
    /// SCANOFLD (`mac.c:5532-5554`, M2c spec S1.13), failing on
    /// `ret & ret_mask` (0x3F for ADD_SCANOFLD_CH,
    /// `RTW89_C2H_SCAN_DONE_ACK_RETURN`; 0xFF for SCANOFLD).
    ///
    /// # Safety
    /// As [`Session::send`].
    pub unsafe fn send_and_wait<I: AxIo + ?Sized>(
        &mut self,
        io: &mut I,
        packet: (u8, Vec<u8>),
        func: u8,
        ret_mask: u8,
        sink: &mut dyn FnMut(Event<'_>),
    ) -> Result<u8, CommandError> {
        let wait = Wait::Done {
            class: H2C_CL_MAC_FW_OFLD,
            func,
            mask: ret_mask,
        };
        // SAFETY: the caller's contract is `send_and_wait_for`'s.
        unsafe { self.send_and_wait_for(io, packet, wait, sink) }
    }

    /// Send `packet`, then drain RXQ every `ACK_POLL_US` for up to
    /// `ACK_WAIT_US` until what `wait` names arrives (M2c spec S1.13, M3
    /// spec S12.5). Returns the seq.
    ///
    /// - [`Wait::Received`] for a packet too short for its FWCMD header:
    ///   `Submit(Dma)`, before anything is sent.
    /// - The H2C not sent: `Submit` with the submit's error; RXQ untouched.
    /// - [`Wait::Done`] and `ret & mask != 0`: `Submit(H2cRejected)` with
    ///   the wait's class and func and the raw return. The wait consumed that
    ///   DONE_ACK, so it never becomes the ledger's refusal.
    /// - Nothing matching in time: `Wait(NoAck)` with the class and func
    ///   waited for, the C2Hs that were not acks, the unmatched acks and the
    ///   skipped buffers of this wait, and the last RXQ hardware index. A
    ///   DONE_ACK never ends a [`Wait::Received`], nor a REC_ACK a
    ///   [`Wait::Done`].
    /// - A drain or clock failure: `Wait` with it.
    ///
    /// Frames, scan reports and beacon filter reports drained meanwhile
    /// reach `sink`.
    ///
    /// # Safety
    /// As [`Session::send`].
    pub unsafe fn send_and_wait_for<I: AxIo + ?Sized>(
        &mut self,
        io: &mut I,
        packet: (u8, Vec<u8>),
        wait: Wait,
        sink: &mut dyn FnMut(Event<'_>),
    ) -> Result<u8, CommandError> {
        let (seq, bytes) = packet;
        let (cat, class, func, receipt, mask) = match wait {
            Wait::Done { class, func, mask } => (H2C_CAT_MAC, class, func, None, mask),
            Wait::Received => {
                let (cat, class, func) = h2c_header(&bytes).ok_or(CommandError::Submit(
                    AxError::Dma("CH12 packet without an H2C header"),
                ))?;
                (cat, class, func, Some(seq), 0)
            }
        };
        let fresh = || Awaited {
            cat,
            class,
            func,
            receipt,
            met: None,
            other_c2h: 0,
            unmatched_ack: 0,
        };
        // SAFETY: the caller's contract is `submit`'s, word for word.
        unsafe { self.rings.ch12().submit(io, &bytes) }.map_err(CommandError::Submit)?;
        let skipped = |rx: &Receiver| {
            u32::from(rx.counts.segments).saturating_add(u32::from(rx.counts.c2h_skipped))
        };
        let skipped_before = skipped(&self.rx);
        self.awaited = Some(fresh());
        let waited = poll_sample(
            io,
            ACK_POLL_US,
            ACK_WAIT_US,
            |io| {
                // SAFETY: the caller's contract is `drain`'s; the submit
                // succeeded, which only a successful `pre_init` allows.
                unsafe { self.drain(io, sink) }?;
                Ok(self.awaited.as_ref().is_some_and(|a| a.met.is_some()))
            },
            |acked| acked,
        );
        // Set above and taken only here: the default is never used.
        let awaited = self.awaited.take().unwrap_or_else(fresh);
        waited.map_err(CommandError::Wait)?;
        match awaited.met {
            Some(ret) if ret & mask != 0 => Err(CommandError::Submit(AxError::H2cRejected {
                class,
                func,
                seq,
                ret,
            })),
            Some(_) => Ok(seq),
            None => Err(CommandError::Wait(AxError::NoAck {
                class,
                func,
                seq,
                other_c2h: awaited.other_c2h,
                unmatched_ack: awaited.unmatched_ack,
                skipped: u8::try_from(skipped(&self.rx).saturating_sub(skipped_before))
                    .unwrap_or(u8::MAX),
                rxq_hw: self.rxq_hw,
            })),
        }
    }
}

#[cfg(test)]
mod tests;

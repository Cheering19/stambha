// SPDX-License-Identifier: GPL-2.0-only
// The multitouch feature protocol follows VoodooInput d8978137
// (VoodooInputSimulatorDevice.cpp, GPL-2.0; Copyright (c) 2018-2019
// Alexandre Daoud and Kishor Prins) and the mode switch Linux
// hid-magicmouse.c (Copyright (c) 2010 Michael Poole and Chase Douglas).
// See NOTICE.
//! The emulated Magic Trackpad 2 itself, as a hub port's full-speed
//! [`Function`], built with the `trackpad` feature alone (see the parent
//! module).
//!
//! Evidence: the identity 05AC:0265 (Linux hid-ids.h
//! `USB_DEVICE_ID_APPLE_MAGICTRACKPAD2`; Windows' AmtPtpDevice.inf binds its
//! `MI_01`), the mode switch SET_REPORT feature 0x02 `{02, 01}` on the
//! multitouch interface (hid-magicmouse.c `magicmouse_enable_multitouch`),
//! the report layout (hid-magicmouse.c and VoodooInput,
//! [`report`](super::report)), and VoodooInput's simulator, whose report
//! descriptor and answers to Apple's multitouch feature requests Intel
//! macOS's multitouch driver accepts ([`descriptors`]). VoodooInput
//! presents itself over I2C under an internal trackpad's ID; that macOS
//! takes the same frames and answers from a USB 05AC:0265 is not shown by
//! any source, so `Identity::Builtin` gives VoodooInput's own ID, one
//! constant away.
//!
//! The function sends frames whatever mode the guest set: VoodooInput never
//! waits for a switch, and waiting would make a missed switch a dead pad.
//! Every request the tables do not answer STALLs and is reported once
//! (`TrackpadEvent::Unknown`), which is how the device converges on what
//! macOS asks.
//!
//! Host frames queue until the guest's interrupt IN on endpoint 0x83, and
//! are encoded only then, so a frame's touch states follow what the guest
//! was sent: a frame dropped from the queue can delay a Start or a Stop but
//! never lose one. Once two frames are queued, a frame of the newest one's
//! contacts and buttons replaces it (their motion coalesces, counted apart),
//! so a guest reading slower than the pad lags two frames, not a queue's
//! worth; frames of changing contacts queue up to [`QUEUE`], past which the
//! oldest is dropped, counted. An unconfigured guest keeps the newest frame
//! only. A bus reset
//! or a new configuration forgets every contact, so a finger still down
//! starts again. A report longer than the endpoint's 64-byte packets (six
//! or more fingers) goes out over consecutive packets, the last one short.
use super::{
    Config, Counters, TrackpadEvent,
    descriptors::{
        self, DECLARED, ENDPOINTS, FEATURE_REPORT, HID, INTERFACES, MAX_PACKET, MODE, MULTITOUCH,
        QUERY, REPORT,
    },
    report::{Encoder, Mapping},
};
use crate::Result;
use crate::usb::{
    BusPeer, Function, Reply, Token,
    control::{
        CLASS, CLEAR_FEATURE, Control, ENDPOINT_HALT, GET_DESCRIPTOR, GET_INTERFACE, GET_STATUS,
        Outcome, RECIPIENT_ENDPOINT, RECIPIENT_INTERFACE, Requests, SET_FEATURE, SET_INTERFACE,
        STANDARD, Setup, Stall,
    },
};
use alloc::{collections::VecDeque, vec, vec::Vec};
use stela_drivers::input::multitouch::{Frame, Surface};

/// Host frames waiting for the guest at most.
pub const QUEUE: usize = 32;
/// Queued frames from which a frame of the newest one's contacts and
/// buttons replaces it: the guest's next report and the newest motion
/// stay, so a guest reading slower than the pad lags two frames at most.
const COALESCE_FROM: usize = 2;
/// Trace entries kept between the owner's takes.
const TRACE: usize = 64;
/// Distinct unknown requests reported.
const UNKNOWN: usize = 64;

const GET_REPORT: u8 = 0x01;
const GET_IDLE: u8 = 0x02;
const GET_PROTOCOL: u8 = 0x03;
const SET_REPORT: u8 = 0x09;
const SET_IDLE: u8 = 0x0a;
const SET_PROTOCOL: u8 = 0x0b;
const REPORT_PROTOCOL: u8 = 1;

/// The gap between frames under which they are one stretch of reporting.
const REPORTING_GAP_US: u64 = 100_000;

pub struct Trackpad {
    control: Control,
    state: State,
}

struct State {
    configuration: u8,
    map: Mapping,
    encoder: Encoder,
    queue: VecDeque<Frame>,
    /// The rest of a report longer than a packet, for the next IN.
    pending: Vec<u8>,
    /// The information query armed by SET_REPORT feature 0x01.
    query: Option<u8>,
    /// The mode the guest last set, which the trace says once per change.
    mode: Option<u8>,
    halted: [bool; INTERFACES],
    /// Feature reports and queries already reported.
    features: Vec<u8>,
    queries: Vec<u8>,
    unknown: Vec<(u8, u8, u16, u16)>,
    trace: Vec<TrackpadEvent>,
    counters: Counters,
    /// Drops since the last take.
    dropped: u64,
    last_time: Option<u64>,
}

impl Trackpad {
    pub fn new(config: Config, serial: &str) -> Self {
        Self {
            control: Control::new(descriptors::descriptors(config.identity.product(), serial)),
            state: State {
                configuration: 0,
                map: Mapping::mt2(config.surface),
                encoder: Encoder::new(),
                queue: VecDeque::with_capacity(QUEUE),
                pending: Vec::new(),
                query: None,
                mode: None,
                halted: [false; INTERFACES],
                features: Vec::new(),
                queries: Vec::new(),
                unknown: Vec::new(),
                trace: Vec::new(),
                counters: Counters::default(),
                dropped: 0,
                last_time: None,
            },
        }
    }

    /// The host surface the function maps.
    pub fn surface(&self) -> Surface {
        self.state.map.surface()
    }

    /// Map `surface` from now on: a touchpad that arrived after the
    /// cabling, which was made with the MT2's own size. Feature 0xD9 and the
    /// frames that follow take it.
    pub fn set_surface(&mut self, surface: Surface) {
        self.state.map = Mapping::mt2(surface);
    }

    /// One host frame, for the guest's next interrupt IN.
    pub fn frame(&mut self, frame: Frame) {
        let state = &mut self.state;
        state.counters.received += 1;
        if let Some(last) = state.last_time.replace(frame.time_us) {
            let gap = frame.time_us.saturating_sub(last);
            if gap < REPORTING_GAP_US {
                state.counters.reporting_us += gap;
            }
        }
        if state.configuration == 0 {
            state.queue.clear();
            state.queue.push_back(frame);
            return;
        }
        if state.queue.len() >= COALESCE_FROM
            && let Some(back) = state.queue.back_mut()
            && same_contacts(back, &frame)
        {
            *back = frame;
            state.counters.coalesced += 1;
            return;
        }
        if state.queue.len() == QUEUE {
            state.queue.pop_front();
            state.counters.dropped += 1;
            state.dropped += 1;
        }
        state.queue.push_back(frame);
    }

    /// What the function did since the last take: configurations, the mode
    /// and features the guest asked for, each unknown request once, and the
    /// frames dropped meanwhile.
    pub fn take_trace(&mut self) -> Vec<TrackpadEvent> {
        let mut trace = core::mem::take(&mut self.state.trace);
        if self.state.dropped != 0 {
            trace.push(TrackpadEvent::Dropped(core::mem::take(
                &mut self.state.dropped,
            )));
        }
        trace
    }

    pub fn counters(&self) -> Counters {
        self.state.counters
    }

    /// Whether the guest has the function configured.
    pub fn configured(&self) -> bool {
        self.state.configuration != 0
    }
}

/// Whether two frames list the same contacts, touching and confident
/// alike, and the same buttons.
fn same_contacts(a: &Frame, b: &Frame) -> bool {
    a.buttons == b.buttons
        && a.fingers().len() == b.fingers().len()
        && a.fingers()
            .iter()
            .zip(b.fingers())
            .all(|(a, b)| (a.id, a.tip, a.confident) == (b.id, b.tip, b.confident))
}

impl State {
    fn note(&mut self, event: TrackpadEvent) {
        if self.trace.len() < TRACE {
            self.trace.push(event);
        }
    }

    /// A request no table answers: reported once, then STALLed.
    fn unknown(&mut self, setup: Setup) -> Result<Outcome, Stall> {
        let key = (setup.request_type, setup.request, setup.value, setup.index);
        if !self.unknown.contains(&key) && self.unknown.len() < UNKNOWN {
            self.unknown.push(key);
            self.note(TrackpadEvent::Unknown {
                request_type: key.0,
                request: key.1,
                value: key.2,
                index: key.3,
            });
        }
        Err(Stall)
    }

    /// Forget what the guest was sent: a bus reset or a new configuration.
    fn forget(&mut self) {
        self.encoder.reset();
        self.pending.clear();
        self.query = None;
        self.halted = [false; INTERFACES];
        // Only the newest frame is still the pad's state.
        if let Some(newest) = self.queue.pop_back() {
            self.queue.clear();
            self.queue.push_back(newest);
        }
    }

    fn interface(setup: &Setup) -> Option<usize> {
        let interface = usize::from(setup.index & 0xff);
        (interface < INTERFACES && setup.index >> 8 == 0).then_some(interface)
    }

    fn endpoint(setup: &Setup) -> Option<usize> {
        ENDPOINTS
            .iter()
            .position(|&endpoint| u16::from(endpoint) == setup.index)
    }

    /// A HID class request to interface `interface`.
    fn class(&mut self, setup: Setup, interface: usize, data: &[u8]) -> Result<Outcome, Stall> {
        let (kind, id) = ((setup.value >> 8) as u8, setup.value as u8);
        let multitouch = interface == usize::from(MULTITOUCH);
        match (setup.request, setup.device_to_host()) {
            (GET_REPORT, true) if multitouch && kind == FEATURE_REPORT => {
                let answer = if id == QUERY {
                    self.query
                        .and_then(|query| descriptors::query(query, &self.map))
                } else {
                    descriptors::feature(id, &self.map)
                };
                let Some(answer) = answer else {
                    return self.unknown(setup);
                };
                if !self.features.contains(&id) {
                    self.features.push(id);
                    self.note(TrackpadEvent::Feature(id));
                }
                Ok(Outcome::Data(answer))
            }
            (SET_REPORT, false) if multitouch && kind == FEATURE_REPORT => match (id, data) {
                (QUERY, [QUERY, query, ..]) => {
                    let answered = descriptors::query(*query, &self.map).is_some();
                    if !self.queries.contains(query) {
                        self.queries.push(*query);
                        self.note(TrackpadEvent::Query {
                            id: *query,
                            answered,
                        });
                    }
                    if !answered {
                        self.query = None;
                        return self.unknown(setup);
                    }
                    self.query = Some(*query);
                    Ok(Outcome::Done)
                }
                (MODE, [MODE, mode, ..]) => {
                    // Linux sends it again after each reset-resume.
                    if self.mode.replace(*mode) != Some(*mode) {
                        self.note(TrackpadEvent::Mode(*mode));
                    }
                    Ok(Outcome::Done)
                }
                _ => self.unknown(setup),
            },
            (GET_REPORT, true) if !multitouch => {
                let declared = DECLARED
                    .iter()
                    .find(|&&(n, declared_kind, declared_id, _)| {
                        usize::from(n) == interface && declared_kind == kind && declared_id == id
                    });
                match declared {
                    Some(&(_, _, id, bytes)) => {
                        let mut answer = vec![0u8; bytes + usize::from(id != 0)];
                        answer[0] = id;
                        Ok(Outcome::Data(answer))
                    }
                    None => self.unknown(setup),
                }
            }
            // The vendor interface's feature and the actuator's output
            // reports are taken and ignored: the pad's click is mechanical.
            (SET_REPORT, false) if !multitouch => Ok(Outcome::Done),
            (GET_IDLE, true) => Ok(Outcome::Data(vec![0])),
            // Reports go out on change alone (as `usb::hid`'s).
            (SET_IDLE, false) if setup.value >> 8 == 0 => Ok(Outcome::Done),
            (GET_PROTOCOL, true) => Ok(Outcome::Data(vec![REPORT_PROTOCOL])),
            (SET_PROTOCOL, false) if setup.value == u16::from(REPORT_PROTOCOL) => Ok(Outcome::Done),
            _ => self.unknown(setup),
        }
    }

    /// One IN packet on the multitouch endpoint: the rest of a long report,
    /// else the next frame's report, else the header-only report owed after
    /// a lift.
    fn interrupt_in(&mut self, buffer: &mut [u8]) -> Reply {
        if self.pending.is_empty() {
            let mut report = Vec::new();
            if let Some(frame) = self.queue.pop_front() {
                self.encoder.encode(&frame, &self.map, &mut report);
            } else if !self.encoder.tail(&mut report) {
                return Reply::Nak;
            }
            self.counters.sent += 1;
            self.pending = report;
        }
        let n = self.pending.len().min(MAX_PACKET).min(buffer.len());
        buffer[..n].copy_from_slice(&self.pending[..n]);
        self.pending.drain(..n);
        Reply::Ack(n)
    }
}

impl Requests for State {
    fn configure(&mut self, value: u8) -> Result<(), Stall> {
        if value != self.configuration {
            self.note(TrackpadEvent::Configured(value));
        }
        self.configuration = value;
        self.forget();
        Ok(())
    }

    fn request(&mut self, setup: Setup, data: &[u8]) -> Result<Outcome, Stall> {
        let read = setup.device_to_host();
        match (setup.kind(), setup.recipient(), setup.request, read) {
            (CLASS, RECIPIENT_INTERFACE, ..) => match Self::interface(&setup) {
                Some(interface) => self.class(setup, interface, data),
                None => self.unknown(setup),
            },
            (STANDARD, RECIPIENT_INTERFACE, GET_DESCRIPTOR, true) => {
                let descriptor = Self::interface(&setup).and_then(|interface| {
                    match ((setup.value >> 8) as u8, setup.value as u8) {
                        (HID, 0) => descriptors::report_descriptor(interface)
                            .map(|report| descriptors::hid_descriptor(report.len()).to_vec()),
                        (REPORT, 0) => {
                            descriptors::report_descriptor(interface).map(<[u8]>::to_vec)
                        }
                        _ => None,
                    }
                });
                match descriptor {
                    Some(bytes) => Ok(Outcome::Data(bytes)),
                    None => self.unknown(setup),
                }
            }
            (STANDARD, RECIPIENT_INTERFACE, GET_STATUS, true)
                if Self::interface(&setup).is_some() =>
            {
                Ok(Outcome::Data(vec![0, 0]))
            }
            (STANDARD, RECIPIENT_INTERFACE, GET_INTERFACE, true)
                if Self::interface(&setup).is_some() =>
            {
                Ok(Outcome::Data(vec![0]))
            }
            (STANDARD, RECIPIENT_INTERFACE, SET_INTERFACE, false)
                if setup.value == 0 && Self::interface(&setup).is_some() =>
            {
                Ok(Outcome::Done)
            }
            (STANDARD, RECIPIENT_ENDPOINT, GET_STATUS, true) => match Self::endpoint(&setup) {
                Some(n) => Ok(Outcome::Data(vec![u8::from(self.halted[n]), 0])),
                None => self.unknown(setup),
            },
            (STANDARD, RECIPIENT_ENDPOINT, CLEAR_FEATURE | SET_FEATURE, false)
                if setup.value == ENDPOINT_HALT =>
            {
                match Self::endpoint(&setup) {
                    Some(n) => {
                        self.halted[n] = setup.request == SET_FEATURE;
                        Ok(Outcome::Done)
                    }
                    None => self.unknown(setup),
                }
            }
            _ => self.unknown(setup),
        }
    }
}

impl BusPeer for Trackpad {
    fn reset(&mut self) -> Result<()> {
        self.control.reset(&mut self.state)?;
        self.state.forget();
        Ok(())
    }

    fn packet(&mut self, endpoint: u8, token: Token, data: &mut [u8]) -> Result<Reply> {
        if endpoint == 0 {
            return self.control.packet(token, data, &mut self.state);
        }
        let state = &mut self.state;
        let Some(n) = ENDPOINTS.iter().position(|&ours| ours == endpoint | 0x80) else {
            return Ok(Reply::Stall);
        };
        Ok(match token {
            _ if state.configuration == 0 || state.halted[n] => Reply::Stall,
            Token::In if n == usize::from(MULTITOUCH) => state.interrupt_in(data),
            // The vendor and actuator interfaces never report.
            Token::In => Reply::Nak,
            Token::Out | Token::Setup => Reply::Stall,
        })
    }
}

impl Function for Trackpad {
    fn set_time(&mut self, _now: u64) {}

    fn ready_at(&self, endpoint: u8) -> Option<u64> {
        let state = &self.state;
        let multitouch = ENDPOINTS[usize::from(MULTITOUCH)];
        if endpoint | 0x80 != multitouch
            || state.configuration == 0
            || state.halted[usize::from(MULTITOUCH)]
        {
            return None;
        }
        (!state.queue.is_empty() || !state.pending.is_empty() || state.encoder.owes_tail())
            .then_some(0)
    }
}

#[cfg(test)]
mod tests;

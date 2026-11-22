// SPDX-License-Identifier: GPL-2.0-only
// The report layout and the constants of a pad without pressure follow
// VoodooInput d8978137 (VoodooInputSimulatorDevice.{hpp,cpp}, GPL-2.0;
// Copyright (c) 2018-2019 Alexandre Daoud and Kishor Prins), and the range
// and decoding Linux hid-magicmouse.c (Copyright (c) 2010 Michael Poole and
// Chase Douglas). See NOTICE.
//! The Magic Trackpad 2's multitouch input report (report 0x02 on its
//! multitouch interface), from host touch [`Frame`]s.
//!
//! A report is a 12-byte header and 9 bytes per finger, at most 15 fingers,
//! as Linux `__magicmouse_raw_event` checks it (`TRACKPAD2_USB_REPORT_ID`).
//! Header: the report ID, the button in byte 1 bit 0 (Linux `clicks`), five
//! zero bytes, 0x03 while a finger touches or the button is down and 0x02
//! otherwise, the inner multitouch report ID 0x31 and a 21-bit millisecond
//! timestamp packed as `ms << 3 | 4`, `ms >> 5`, `ms >> 13` (VoodooInput
//! `constructReportGated`). Finger: X and Y as signed 13-bit values, Y
//! upward, the finger type and touch state, touch major and minor, size,
//! pressure, and the identifier with orientation 4 (Linux
//! `magicmouse_emit_touch` reads every field back).
//!
//! Positions use the MT2's own resolution (hid-magicmouse.c
//! `TRACKPAD2_MIN_X`..`TRACKPAD2_DIMENSION_Y`: 7612 units over 160.00 mm,
//! 5065 over 114.90 mm) with the host pad centred on the MT2's centre, so a
//! millimetre of finger travel is a millimetre whether macOS scales by the
//! reported surface (feature 0xD9) or by the MT2's own size.
//!
//! Each host contact takes a slot when it first touches and keeps it until
//! one Stop follows its lift; the identifier is the slot plus one, and new
//! contacts take slots round-robin, so a new touch never reuses the
//! identifier a lift has just freed. A contact's first report is Start,
//! then Active while it touches; the report of the frame where it lifts or
//! leaves the frame carries Stop, at its last position and type, with its
//! sizes and pressure 0. When the pad empties, the next report owed is the
//! 12-byte header alone ([`Encoder::tail`]).
//!
//! The pad states no pressure or size, so a touching finger has
//! VoodooInput's constants for such pads: pressure 5, size 10, major and
//! minor 20, and pressure 120 on every finger while the button is down.
//! Finger types: index, middle, ring and little by report order, palm for a
//! contact the pad marks without confidence, and thumb for the lowest
//! confident finger while four or more contacts touch or the button is
//! down, so a thumb's click and the index finger's drag, and a four-finger
//! pinch, read as macOS expects. That is the behavior VoodooI2C's native
//! engine shows (VoodooI2C is GPL-3.0, so none of its code is used here),
//! except that a palm is never made the thumb.
use alloc::vec::Vec;
use stela_drivers::input::multitouch::{Frame, Surface};

pub const REPORT_ID: u8 = 0x02;
pub const HEADER: usize = 12;
pub const FINGER: usize = 9;
/// The most fingers Linux reads from one report.
pub const MAX_FINGERS: usize = 15;
const MULTITOUCH_REPORT_ID: u8 = 0x31;
/// hid-magicmouse.c `TRACKPAD2_MIN_X`/`MAX_X` and `MIN_Y`/`MAX_Y`, Y
/// downward as Linux reports it.
const MT2_X: (i64, i64) = (-3678, 3934);
const MT2_Y: (i64, i64) = (-2478, 2587);
/// `TRACKPAD2_DIMENSION_X`/`_Y`, in micrometres.
const MT2_UM: (i64, i64) = (160_000, 114_900);
/// The millisecond timestamp's width.
const MS_BITS: u32 = 21;

const MAJOR: u8 = 20;
const MINOR: u8 = 20;
const SIZE: u8 = 10;
const PRESSURE: u8 = 5;
const CLICK_PRESSURE: u8 = 120;
/// VoodooInput's angle: pi/2, Linux orientation 0.
const ORIENTATION: u8 = 4;
/// VoodooInput `TouchStates`.
const START: u8 = 3;
const ACTIVE: u8 = 4;
const STOP: u8 = 7;
/// VoodooInput `MT2FingerType`.
const THUMB: u8 = 1;
const INDEX: u8 = 2;
const PALM: u8 = 6;
/// Contacts touching at once from which the lowest is the thumb.
const THUMB_CONTACTS: usize = 4;

/// Round `n / d` half away from zero.
fn rdiv(n: i64, d: i64) -> i64 {
    let q = n.abs() / d;
    let q = if 2 * (n.abs() % d) >= d { q + 1 } else { q };
    if n < 0 { -q } else { q }
}

/// The least and greatest X and Y feature 0xD9 states after the surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum D9Range {
    /// VoodooInput's bytes verbatim: X -7356 to 7869 (about twice
    /// hid-magicmouse.c's `TRACKPAD2_MIN_X`/`MAX_X`), Y -174 to 9956. Intel
    /// macOS takes them, though they are not the range the reports carry.
    VoodooInput,
    /// The range the reports carry: the host surface's corners.
    Emitted,
}

/// The range feature 0xD9 sends: VoodooInput's, the one Intel macOS is
/// known to take, for the first boot. One edit makes it
/// [`D9Range::Emitted`].
pub const D9_RANGE: D9Range = D9Range::VoodooInput;

/// VoodooInput's 0xD9 range bytes (`VoodooInputSimulatorDevice.cpp`).
const VOODOO_INPUT_D9_RANGE: [u8; 8] = [0x44, 0xe3, 0x52, 0xff, 0xbd, 0x1e, 0xe4, 0x26];

/// Where a host surface's positions land on the MT2's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mapping {
    surface: Surface,
}

impl Mapping {
    /// The host surface at the MT2's resolution, centred.
    pub fn mt2(surface: Surface) -> Self {
        Self { surface }
    }

    pub fn surface(&self) -> Surface {
        self.surface
    }

    /// The MT2 position of host position `x_um`, `y_um` (from the top-left
    /// corner, y downward): X, and Y upward as the wire carries it.
    pub fn to_mt2(&self, x_um: u32, y_um: u32) -> (i16, i16) {
        let centre = |(min, max): (i64, i64)| (min + max).div_euclid(2);
        let span = |(min, max): (i64, i64)| max - min;
        let width = i64::from(self.surface.width_um);
        let height = i64::from(self.surface.height_um);
        let x = centre(MT2_X) + rdiv((i64::from(x_um) - width / 2) * span(MT2_X), MT2_UM.0);
        let y = centre(MT2_Y) + rdiv((i64::from(y_um) - height / 2) * span(MT2_Y), MT2_UM.1);
        (
            x.clamp(MT2_X.0, MT2_X.1) as i16,
            -(y.clamp(MT2_Y.0, MT2_Y.1)) as i16,
        )
    }

    /// Feature report 0xD9, the sensor surface, with [`D9_RANGE`].
    pub fn d9(&self) -> [u8; 17] {
        self.d9_with(D9_RANGE)
    }

    /// Feature report 0xD9: the surface's width and height in 0.01 mm
    /// (u32), then the least and greatest X and Y (i16, Y upward) of
    /// `range`, all little-endian.
    pub fn d9_with(&self, range: D9Range) -> [u8; 17] {
        let hundredths = |um: u32| um.saturating_add(5) / 10;
        let mut report = [0u8; 17];
        report[0] = 0xd9;
        report[1..5].copy_from_slice(&hundredths(self.surface.width_um).to_le_bytes());
        report[5..9].copy_from_slice(&hundredths(self.surface.height_um).to_le_bytes());
        match range {
            D9Range::VoodooInput => report[9..].copy_from_slice(&VOODOO_INPUT_D9_RANGE),
            D9Range::Emitted => {
                let (min_x, max_y) = self.to_mt2(0, 0);
                let (max_x, min_y) = self.to_mt2(self.surface.width_um, self.surface.height_um);
                for (at, value) in [(9, min_x), (11, min_y), (13, max_x), (15, max_y)] {
                    report[at..at + 2].copy_from_slice(&value.to_le_bytes());
                }
            }
        }
        report
    }
}

/// A contact the guest has seen touch: the host's identifier, and what its
/// last report said of it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Slot {
    host: u8,
    x: i16,
    y: i16,
    kind: u8,
}

/// The MT2 state the guest has been sent.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Encoder {
    slots: [Option<Slot>; MAX_FINGERS],
    /// Where the next new contact's slot search starts.
    next: usize,
    /// The timestamp of the header-only report owed after the last lift.
    tail: Option<u32>,
}

/// One finger's 9 bytes.
fn finger(x: i16, y: i16, kind: u8, state: u8, touching: Option<u8>, identifier: u8) -> [u8; 9] {
    let x = (x as u16) & 0x1fff;
    let y = (y as u16) & 0x1fff;
    let (major, minor, size, pressure) = match touching {
        Some(pressure) => (MAJOR, MINOR, SIZE, pressure),
        None => (0, 0, 0, 0),
    };
    [
        x as u8,
        ((x >> 8) as u8 & 0x1f) | ((y & 0x7) as u8) << 5,
        (y >> 3) as u8,
        ((y >> 11) as u8 & 0x3) | (kind & 7) << 2 | (state & 7) << 5,
        major,
        minor,
        size,
        pressure,
        (identifier & 0xf) | ORIENTATION << 5,
    ]
}

fn header(button: bool, active: bool, ms: u32, out: &mut Vec<u8>) {
    out.extend_from_slice(&[
        REPORT_ID,
        u8::from(button),
        0,
        0,
        0,
        0,
        0,
        if active { 0x03 } else { 0x02 },
        MULTITOUCH_REPORT_ID,
        (ms << 3 | 4) as u8,
        (ms >> 5) as u8,
        (ms >> 13) as u8,
    ]);
}

/// A frame's time as the report's millisecond counter.
fn millis(time_us: u64) -> u32 {
    ((time_us / 1000) % (1 << MS_BITS)) as u32
}

impl Encoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Forget every contact: a reconfigured guest has seen none.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Append the report of `frame` to `out`, through `map`, and note what
    /// the guest was told.
    pub fn encode(&mut self, frame: &Frame, map: &Mapping, out: &mut Vec<u8>) {
        let ms = millis(frame.time_us);
        let button = frame.buttons & 1 != 0;
        let touching = |host: u8| {
            frame
                .fingers()
                .iter()
                .find(|finger| finger.id == host && finger.tip)
        };
        // Contacts that lift in this frame: their slot's last report, then
        // Stop. Slots free only after this report, so a new contact cannot
        // take the identifier of one stopping beside it.
        let mut stopping = [false; MAX_FINGERS];
        for (n, slot) in self.slots.iter().enumerate() {
            if let Some(slot) = slot {
                stopping[n] = touching(slot.host).is_none();
            }
        }
        // Contacts landing: the next free slot, round-robin; one that finds
        // none waits for a later frame.
        let mut started = [false; MAX_FINGERS];
        for finger in frame.fingers().iter().filter(|finger| finger.tip) {
            if self
                .slots
                .iter()
                .flatten()
                .any(|slot| slot.host == finger.id)
            {
                continue;
            }
            let Some(n) = (0..MAX_FINGERS)
                .map(|step| (self.next + step) % MAX_FINGERS)
                .find(|&n| self.slots[n].is_none())
            else {
                break;
            };
            self.slots[n] = Some(Slot {
                host: finger.id,
                x: 0,
                y: 0,
                kind: INDEX,
            });
            started[n] = true;
            self.next = (n + 1) % MAX_FINGERS;
        }
        // Positions and types of the touching contacts, in report order.
        let mut order = 0;
        let mut touched = 0;
        for (n, slot) in self.slots.iter_mut().enumerate() {
            let Some(slot) = slot else {
                continue;
            };
            if !stopping[n] {
                let finger = touching(slot.host).expect("a touching slot's contact touches");
                (slot.x, slot.y) = map.to_mt2(finger.x_um, finger.y_um);
                slot.kind = if finger.confident {
                    INDEX + (order % 4) as u8
                } else {
                    PALM
                };
                touched += 1;
            }
            order += 1;
        }
        if touched >= THUMB_CONTACTS || button {
            // The lowest confident finger: the least Y upward, the last of
            // equals, as VoodooI2C's `>=` picks it.
            let thumb = self
                .slots
                .iter()
                .enumerate()
                .filter_map(|(n, slot)| {
                    slot.filter(|s| !stopping[n] && s.kind != PALM)
                        .map(|s| (n, s.y))
                })
                .fold(None, |lowest: Option<(usize, i16)>, (n, y)| match lowest {
                    Some((_, least)) if y > least => lowest,
                    _ => Some((n, y)),
                });
            if let Some((n, _)) = thumb
                && let Some(slot) = &mut self.slots[n]
            {
                slot.kind = THUMB;
            }
        }
        let pressure = if button { CLICK_PRESSURE } else { PRESSURE };
        header(button, touched > 0 || button, ms, out);
        let mut stopped = false;
        for (n, entry) in self.slots.iter_mut().enumerate() {
            let Some(slot) = *entry else {
                continue;
            };
            let identifier = n as u8 + 1;
            if stopping[n] {
                out.extend_from_slice(&finger(slot.x, slot.y, slot.kind, STOP, None, identifier));
                *entry = None;
                stopped = true;
            } else {
                let state = if started[n] { START } else { ACTIVE };
                out.extend_from_slice(&finger(
                    slot.x,
                    slot.y,
                    slot.kind,
                    state,
                    Some(pressure),
                    identifier,
                ));
            }
        }
        // The header alone says the button is up too, so it is owed only
        // once the button is.
        self.tail = (stopped && touched == 0 && !button).then_some(ms);
    }

    /// Append the header-only report owed after the last contact's Stop, at
    /// that report's time; false, appending nothing, when none is owed.
    pub fn tail(&mut self, out: &mut Vec<u8>) -> bool {
        let Some(ms) = self.tail.take() else {
            return false;
        };
        header(false, false, ms, out);
        true
    }

    /// Whether a header-only report is owed.
    pub fn owes_tail(&self) -> bool {
        self.tail.is_some()
    }
}

#[cfg(test)]
mod tests;

// SPDX-License-Identifier: GPL-2.0-only
// Linux hid-multitouch.c reference: Copyright (c) 2010-2012 Stephane
// Chatty, Benjamin Tissoires and Ecole Nationale de l'Aviation Civile.
// See NOTICE.
//! A precision touchpad's whole frames ([`Input::Touch`]) for a guest that
//! models a multitouch device, in place of the mouse path's motion
//! ([`super::ptp`] in frames mode).
//!
//! Each contact goes into the frame as the pad listed it, a palm
//! (Confidence 0) included: Linux hid-multitouch keeps such a contact as
//! `MT_TOOL_PALM` (`mt_process_slot`), and dropping it would read as a lift,
//! which a multitouch consumer takes for a tap. Positions are micrometres
//! from the pad's top-left corner over its [`Surface`], kept exact to the
//! micrometre (126.90 mm stays 126 900 µm, where the mouse path's
//! [`super::touchpad::Geometry`] rounds to whole millimetres).
//!
//! A frame's time follows the pad's Scan Time (100 µs units, HID Usage
//! Tables 16.3), unwrapped past its logical maximum, as hid-multitouch's
//! `mt_compute_timestamp` does, and resynchronized to the poll clock after a
//! gap longer than its `MAX_TIMESTAMP_INTERVAL` of one second. The host polls
//! in bursts, so poll times would put velocity noise into the frames.
//! A Scan Time counter of 0 to `max` wraps modulo `max + 1`; Linux adds
//! `max` itself, 100 µs short at each wrap.
use super::multitouch::{Finger, Frame, Surface};
use crate::usb::hid::{descriptor::ReportField, events::Contact};
use alloc::vec::Vec;

/// hid-multitouch's `MAX_TIMESTAMP_INTERVAL`, in microseconds.
const MAX_GAP_US: u64 = 1_000_000;
/// Scan Time's unit.
const SCAN_UNIT_US: u64 = 100;
/// A Scan Time field's logical maximum when it states none: a 16-bit
/// counter, as the Windows precision touchpad requirement has it.
const SCAN_MAX: i64 = 0xffff;
/// libinput's assumed size of a pad that states none, as the mouse path
/// assumes it.
const ASSUMED: Surface = Surface {
    width_um: 69_000,
    height_um: 50_000,
};

/// The micrometres a field's physical range spans, from its unit: SI linear
/// (centimetres) or English linear (inches), length to the first power,
/// scaled by its unit exponent.
fn micrometres(field: &ReportField) -> Option<u32> {
    let extent = i128::from(field.physical_maximum) - i128::from(field.physical_minimum);
    if extent <= 0 {
        return None;
    }
    let per_unit: i128 = match field.unit & 0xff {
        0x11 => 10_000,
        0x13 => 25_400,
        _ => return None,
    };
    let exponent = i32::from(field.unit_exponent);
    let mut um = extent * per_unit;
    if exponent >= 0 {
        um = um.checked_mul(10i128.checked_pow(exponent as u32)?)?;
    } else {
        // Rounded to the nearest micrometre.
        let divisor = 10i128.checked_pow(exponent.unsigned_abs())?;
        um = (um + divisor / 2) / divisor;
    }
    u32::try_from(um).ok().filter(|um| *um != 0)
}

/// The surface the pad's X and Y fields span, when both state their size.
pub fn surface(x: &ReportField, y: &ReportField) -> Option<Surface> {
    Some(Surface {
        width_um: micrometres(x)?,
        height_um: micrometres(y)?,
    })
}

/// What frames mode reads of a pad's fields: the logical ranges of X and Y,
/// the surface they span and Scan Time's maximum.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Scale {
    x: (i64, i64),
    y: (i64, i64),
    stated: Option<Surface>,
    scan_max: i64,
}

impl Scale {
    /// The scale of a pad's finger X and Y fields and its Scan Time field.
    pub fn of(x: &ReportField, y: &ReportField, scan: Option<&ReportField>) -> Self {
        Self {
            x: (x.minimum, x.maximum),
            y: (y.minimum, y.maximum),
            stated: surface(x, y),
            scan_max: scan
                .map(|field| field.maximum)
                .filter(|max| *max > 0)
                .unwrap_or(SCAN_MAX),
        }
    }

    /// The surface the pad states.
    pub fn stated(&self) -> Option<Surface> {
        self.stated
    }

    /// The surface the frames' positions span: the stated one, else
    /// libinput's assumed 69 by 50 mm.
    pub fn surface(&self) -> Surface {
        self.stated.unwrap_or(ASSUMED)
    }

    pub fn scan_max(&self) -> i64 {
        self.scan_max
    }

    /// `value` of the logical range `range` in micrometres of `extent`,
    /// rounded to the nearest, clamped to the range.
    fn position(value: i64, range: (i64, i64), extent: u32) -> u32 {
        let (min, max) = range;
        let span = i128::from(max) - i128::from(min);
        if span <= 0 {
            return 0;
        }
        let offset = (i128::from(value.clamp(min, max)) - i128::from(min)) * i128::from(extent);
        ((offset + span / 2) / span) as u32
    }
}

/// The pad's scan clock, read as frame times.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Clock {
    /// The last frame's time, its Scan Time and the poll time it was read
    /// at.
    last: Option<(u64, Option<i64>, u64)>,
}

impl Clock {
    /// The time of a frame of Scan Time `scan` (0..=`scan_max`, if the pad
    /// reports one) read at poll time `now_us`: the first frame's poll time,
    /// then the last frame's plus the scan counter's advance, wrapping past
    /// `scan_max`; a gap of more than a second between polls, or a frame
    /// without a scan time, resynchronizes to the poll time. Never before
    /// the last frame's time.
    pub fn time_us(&mut self, scan: Option<i64>, scan_max: i64, now_us: u64) -> u64 {
        let time = match (self.last, scan) {
            (None, _) => now_us,
            (Some((time, Some(last_scan), polled)), Some(scan))
                if now_us.saturating_sub(polled) <= MAX_GAP_US =>
            {
                let period = i128::from(scan_max.max(1)) + 1;
                let advance = (i128::from(scan) - i128::from(last_scan)).rem_euclid(period);
                time.saturating_add(advance as u64 * SCAN_UNIT_US)
            }
            (Some((time, ..)), _) => now_us.max(time),
        };
        self.last = Some((time, scan, now_us));
        time
    }
}

/// The frame of `contacts`, the pad's contacts of one scan in the order it
/// listed them, with `buttons` held at `time_us`. A contact without a
/// position is left out; one without an identifier takes its place in the
/// list as one; one without a confidence is confident.
pub fn frame(contacts: &[Contact], buttons: u8, time_us: u64, scale: &Scale) -> Frame {
    let surface = scale.surface();
    let fingers: Vec<Finger> = contacts
        .iter()
        .enumerate()
        .filter_map(|(index, contact)| {
            let (x, y) = (contact.x?, contact.y?);
            Some(Finger {
                // Contact identifiers fit a byte on every precision
                // touchpad; a wider one keeps its low byte.
                id: contact.id.map_or(index as u8, |id| id as u8),
                x_um: Scale::position(x, scale.x, surface.width_um),
                y_um: Scale::position(y, scale.y, surface.height_um),
                tip: contact.tip == Some(true),
                confident: contact.confidence != Some(false),
            })
        })
        .collect();
    Frame::new(time_us, buttons, &fingers)
}

#[cfg(test)]
mod tests;

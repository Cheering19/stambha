// SPDX-License-Identifier: GPL-2.0-or-later
//! An emulated USB Magic Trackpad 2 fed by the host's precision touchpads,
//! so macOS drives them with its own multitouch stack: its gestures,
//! scrolling, pinch and rotate and its Trackpad settings, where the guest's
//! mouse gets only motion and wheel detents.
//!
//! This module holds what the machine and its owner name of the trackpad
//! in every build: its identity, what is cabled ([`Config`]), its serial
//! number, its trace ([`TrackpadEvent`]) and its counters. The device
//! itself (`Trackpad`, a hub port's full-speed function), its descriptors
//! and feature answers (`descriptors`) and its report encoder (`report`)
//! follow GPL-2.0-only VoodooInput material, so they are built only with
//! this crate's `trackpad` feature, which stela.efi alone enables, and in
//! this crate's own tests; without it the machine refuses to cable one
//! ([`super::TRACKPAD`]).
use alloc::{format, string::String};
use stela_drivers::input::multitouch::Surface;

#[cfg(any(feature = "trackpad", test))]
pub mod descriptors;
#[cfg(any(feature = "trackpad", test))]
mod function;
#[cfg(any(feature = "trackpad", test))]
pub mod report;
#[cfg(any(feature = "trackpad", test))]
pub use function::{QUEUE, Trackpad};

/// Apple's USB vendor ID (Linux hid-ids.h `USB_VENDOR_ID_APPLE`).
pub const VENDOR: u16 = 0x05ac;
/// The Magic Trackpad 2 (hid-ids.h `USB_DEVICE_ID_APPLE_MAGICTRACKPAD2`).
pub const MAGIC_TRACKPAD_2: u16 = 0x0265;
/// The MacBookAir10,1 internal trackpad's product ID, the one VoodooInput
/// presents on macOS 12 and later: the fallback identity.
pub const BUILTIN: u16 = 0x0281;

/// Which Apple trackpad the device presents.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Identity {
    /// The Magic Trackpad 2, 05AC:0265.
    MagicTrackpad2,
    /// The macOS 12+ internal trackpad ID, 05AC:0281.
    Builtin,
}

impl Identity {
    pub fn product(self) -> u16 {
        match self {
            Self::MagicTrackpad2 => MAGIC_TRACKPAD_2,
            Self::Builtin => BUILTIN,
        }
    }
}

/// What the owner cables: the identity, and the host surface the frames'
/// positions span.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    pub identity: Identity,
    pub surface: Surface,
}

/// The serial number of the trackpad of the board whose ECID is `ecid`:
/// stable for the machine, "STELA" and 7 hexadecimal digits of SHA-256
/// of a label and the ECID, so it is no Apple serial and names no ECID.
pub fn serial(ecid: u64) -> String {
    use sha2::{Digest, Sha256};
    let mut hash = Sha256::new();
    hash.update(b"Stela Magic Trackpad 2 serial");
    hash.update(ecid.to_le_bytes());
    let digest = hash.finalize();
    let mut serial = String::from("STELA");
    for byte in &digest[..4] {
        serial.push_str(&format!("{byte:02X}"));
    }
    serial.truncate(12);
    serial
}

/// What the device tells its owner's log.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackpadEvent {
    /// The guest selected this configuration (0: deconfigured).
    Configured(u8),
    /// The guest set the mode feature (1: multitouch).
    Mode(u8),
    /// The guest first read this feature report of the multitouch
    /// interface.
    Feature(u8),
    /// The guest first armed this information query (SET_REPORT feature
    /// 0x01), answered or refused.
    Query { id: u8, answered: bool },
    /// The first of these requests, which STALLed.
    Unknown {
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
    },
    /// Host frames dropped from the queue since the last take.
    Dropped(u64),
}

impl core::fmt::Display for TrackpadEvent {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            Self::Configured(value) => write!(f, "configured {value}"),
            Self::Mode(mode) => write!(f, "mode {mode}"),
            Self::Feature(id) => write!(f, "feature {id:#04x}"),
            Self::Query { id, answered: true } => write!(f, "query {id:#04x}"),
            Self::Query {
                id,
                answered: false,
            } => write!(f, "query {id:#04x} refused"),
            Self::Unknown {
                request_type,
                request,
                value,
                index,
            } => write!(
                f,
                "unknown request_type={request_type:#04x} request={request:#04x} \
                 value={value:#06x} index={index}"
            ),
            Self::Dropped(frames) => write!(f, "dropped {frames} frames"),
        }
    }
}

/// How many frames came from the host and went to the guest, and the time
/// the host frames spanned while fingers were down, for a frame rate.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    pub received: u64,
    pub sent: u64,
    pub dropped: u64,
    /// The sum of the gaps between consecutive received frames under
    /// 100 ms, by the frames' own times: the time the pad was reporting.
    pub reporting_us: u64,
}

#[cfg(test)]
mod tests;

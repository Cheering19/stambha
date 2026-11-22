// SPDX-License-Identifier: GPL-2.0-only
// The multitouch and actuator report descriptors and the multitouch
// feature answers are VoodooInput d8978137's, verbatim
// (VoodooInputSimulatorDevice.cpp `report_descriptor`, `setReport`,
// `getReport`; VoodooInputActuatorDevice.cpp `actuator_report_descriptor`;
// GPL-2.0; Copyright (c) 2018-2019 Alexandre Daoud and Kishor Prins). The
// identity is Linux hid-ids.h's and the mode switch hid-magicmouse.c's.
// See NOTICE.
//! What the emulated Magic Trackpad 2 says about itself: its device,
//! configuration and HID descriptors, and the answers of its multitouch
//! interface's feature reports. The layout is data: an interface or ID
//! changes here alone.
//!
//! | Interface | Class | Report descriptor | Endpoint |
//! |---|---|---|---|
//! | 0 | HID 3/0/0 | a vendor 0xFF00:0x0B collection with one 1-byte feature report | 0x81 interrupt IN, 64 bytes, every 8 ms |
//! | 1 | HID 3/0/2 | VoodooInput's 110-byte simulator descriptor | 0x83 interrupt IN, 64 bytes, every 1 ms |
//! | 2 | HID 3/0/0 | VoodooInput's 36-byte actuator descriptor | 0x84 interrupt IN, 64 bytes, every 8 ms |
//!
//! Interface 1 is the multitouch one: Windows' driver binds the MT2's
//! `MI_01` and reads endpoint 0x83, and Linux sends the mode switch to the
//! interface whose protocol is mouse. Its descriptor declares the mouse
//! report 0x02 that the multitouch reports reuse, a touch pad application
//! and the vendor input report 0x44 of 1387 bytes, which makes a host size
//! its input transfers for the longest multitouch report.
use super::{VENDOR, report::Mapping};
use crate::usb::control::{self, CONFIGURATION, DEVICE, Descriptors, INTERFACE};
use alloc::{string::String, vec, vec::Vec};

/// One MT2 firmware's revision (`HID\VID_05AC&PID_0265&REV_0855`).
pub const RELEASE: u16 = 0x0855;

pub const INTERFACES: usize = 3;
/// The multitouch interface.
pub const MULTITOUCH: u16 = 1;
/// Each interface's interrupt IN endpoint, by interface number.
pub const ENDPOINTS: [u8; INTERFACES] = [0x81, 0x83, 0x84];
/// Full-speed polling intervals in frames, by interface number.
const INTERVALS: [u8; INTERFACES] = [8, 1, 8];
pub const MAX_PACKET: usize = 64;

pub const HID: u8 = 0x21;
pub const REPORT: u8 = 0x22;
/// HID report types in GET/SET_REPORT's wValue high byte.
pub const INPUT_REPORT: u8 = 1;
pub const OUTPUT_REPORT: u8 = 2;
pub const FEATURE_REPORT: u8 = 3;

/// Interface 0: a vendor collection with one feature report of a byte and
/// no report ID.
pub const VENDOR_DESCRIPTOR: [u8; 21] = [
    0x06, 0x00, 0xff, // Usage Page (Vendor 0xFF00)
    0x09, 0x0b, // Usage (0x0B)
    0xa1, 0x01, // Collection (Application)
    0x09, 0x0b, //   Usage (0x0B)
    0x15, 0x00, 0x26, 0xff, 0x00, // Logical 0..255
    0x75, 0x08, 0x95, 0x01, 0xb1, 0x02, // one byte, Feature
    0xc0,
];

/// Interface 1: VoodooInput's simulator descriptor.
pub const MULTITOUCH_DESCRIPTOR: [u8; 110] = [
    0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01, 0xa1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
    0x15, 0x00, 0x25, 0x01, 0x85, 0x02, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05,
    0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x02,
    0x81, 0x06, 0x95, 0x04, 0x75, 0x08, 0x81, 0x01, 0xc0, 0xc0, 0x05, 0x0d, 0x09, 0x05, 0xa1, 0x01,
    0x06, 0x00, 0xff, 0x09, 0x0c, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x10, 0x85, 0x3f,
    0x81, 0x22, 0xc0, 0x06, 0x00, 0xff, 0x09, 0x0c, 0xa1, 0x01, 0x06, 0x00, 0xff, 0x09, 0x0c, 0x15,
    0x00, 0x26, 0xff, 0x00, 0x85, 0x44, 0x75, 0x08, 0x96, 0x6b, 0x05, 0x81, 0x00, 0xc0,
];

/// Interface 2: VoodooInput's actuator descriptor: input 0x3F of 15 bytes,
/// output 0x53 of 63.
pub const ACTUATOR_DESCRIPTOR: [u8; 36] = [
    0x06, 0x00, 0xff, 0x09, 0x0d, 0xa1, 0x01, 0x06, 0x00, 0xff, 0x09, 0x0d, 0x15, 0x00, 0x26, 0xff,
    0x00, 0x75, 0x08, 0x85, 0x3f, 0x96, 0x0f, 0x00, 0x81, 0x02, 0x09, 0x0d, 0x85, 0x53, 0x96, 0x3f,
    0x00, 0x91, 0x02, 0xc0,
];

/// The report descriptor of interface `interface`.
pub fn report_descriptor(interface: usize) -> Option<&'static [u8]> {
    match interface {
        0 => Some(&VENDOR_DESCRIPTOR),
        1 => Some(&MULTITOUCH_DESCRIPTOR),
        2 => Some(&ACTUATOR_DESCRIPTOR),
        _ => None,
    }
}

/// The reports interfaces 0 and 2 declare, as (interface, type, ID, bytes
/// after the ID): GET_REPORT answers them with zeros.
pub const DECLARED: [(u16, u8, u8, usize); 3] = [
    (0, FEATURE_REPORT, 0, 1),
    (2, INPUT_REPORT, 0x3f, 15),
    (2, OUTPUT_REPORT, 0x53, 63),
];

/// HID descriptor for a report descriptor of `bytes` (HID 1.11 6.2.1).
pub fn hid_descriptor(bytes: usize) -> [u8; 9] {
    [
        9,
        HID,
        0x11,
        0x01, // HID 1.11
        0,    // no country
        1,
        REPORT,
        bytes as u8,
        (bytes >> 8) as u8,
    ]
}

fn device(product: u16) -> [u8; 18] {
    let [vendor_low, vendor_high] = VENDOR.to_le_bytes();
    let [product_low, product_high] = product.to_le_bytes();
    let [release_low, release_high] = RELEASE.to_le_bytes();
    [
        18,
        DEVICE,
        0x00,
        0x02, // USB 2.0
        0,
        0,
        0, // class per interface
        control::EP0_PACKET as u8,
        vendor_low,
        vendor_high,
        product_low,
        product_high,
        release_low,
        release_high,
        1, // manufacturer
        2, // product
        3, // serial
        1, // one configuration
    ]
}

fn configuration() -> Vec<u8> {
    let parts: Vec<Vec<u8>> = (0..INTERFACES)
        .flat_map(|n| {
            let protocol = if n == usize::from(MULTITOUCH) { 2 } else { 0 };
            let descriptor = report_descriptor(n).expect("each interface has one");
            [
                vec![9, INTERFACE, n as u8, 0, 1, 3, 0, protocol, 0],
                hid_descriptor(descriptor.len()).to_vec(),
                vec![
                    7,
                    control::ENDPOINT,
                    ENDPOINTS[n],
                    3, // interrupt
                    MAX_PACKET as u8,
                    0,
                    INTERVALS[n],
                ],
            ]
        })
        .collect();
    let parts: Vec<&[u8]> = parts.iter().map(Vec::as_slice).collect();
    // Self-powered, no remote wakeup.
    let bytes = control::configuration(1, 0, &parts);
    debug_assert_eq!(bytes[1], CONFIGURATION);
    bytes
}

/// The descriptors of a trackpad of product ID `product` with serial
/// `serial`.
pub fn descriptors(product: u16, serial: &str) -> Descriptors {
    Descriptors {
        device: device(product),
        configuration: configuration(),
        // Full speed only: device qualifier and other-speed configuration
        // STALL.
        other_speed: None,
        strings: vec![
            String::from("Apple Inc."),
            String::from("Magic Trackpad 2"),
            String::from(serial),
        ],
    }
}

/// The multitouch information queries SET_REPORT feature 0x01 `{01, id}`
/// arms, with the length GET_REPORT feature 0x01 then states:
/// `{01, id, 00, length, 00}`. Query 0xD9 is answered with the whole 0xD9
/// report instead, as VoodooInput answers it.
pub const QUERIES: [(u8, u8); 7] = [
    (0xdb, 0x49),
    (0xd1, 0x01),
    (0xd3, 0x0c),
    (0xd0, 0x0f),
    (0xa1, 0x06),
    (0x7f, 0x04),
    (0xc8, 0x01),
];
pub const QUERY: u8 = 0x01;
pub const SENSOR_SURFACE: u8 = 0xd9;
/// The MT2's mode switch (hid-magicmouse.c `feature_mt_trackpad2_usb`).
pub const MODE: u8 = 0x02;

const D1: [u8; 2] = [0xd1, 0x81];
const D3: [u8; 13] = [
    0xd3, 0x01, 0x16, 0x1e, 0x03, 0x95, 0x00, 0x14, 0x1e, 0x62, 0x05, 0x00, 0x00,
];
const D0: [u8; 16] = [
    0xd0, 0x02, 0x01, 0x00, 0x14, 0x01, 0x00, 0x1e, 0x00, 0x02, 0x14, 0x02, 0x01, 0x0e, 0x02, 0x00,
];
const A1: [u8; 7] = [0xa1, 0x00, 0x00, 0x05, 0x00, 0xfc, 0x01];
const SEVEN_F: [u8; 5] = [0x7f, 0x00, 0x00, 0x00, 0x00];

/// The answer to GET_REPORT feature `id` on the multitouch interface, for
/// a surface mapped by `map`: VoodooInput's bytes, with the pad's width and
/// height in 0xD9 ([`Mapping::d9`]).
pub fn feature(id: u8, map: &Mapping) -> Option<Vec<u8>> {
    Some(match id {
        0x00 => vec![0x00, 0x01],
        MODE => vec![MODE, 0x01],
        0xd1 => D1.to_vec(),
        0xd3 => D3.to_vec(),
        0xd0 => D0.to_vec(),
        0xa1 => A1.to_vec(),
        SENSOR_SURFACE => map.d9().to_vec(),
        0x7f => SEVEN_F.to_vec(),
        0xc8 => vec![0xc8, 0x08],
        0xdb => {
            // D1, D3, D0, A1 and D9, each after its length as a 16-bit
            // word (D1's in the header), then 7F with none.
            let mut all = vec![0xdb, 0x01];
            for part in [&D1[..], &D3, &D0, &A1, &map.d9()] {
                all.extend_from_slice(&(part.len() as u16).to_le_bytes());
                all.extend_from_slice(part);
            }
            all.extend_from_slice(&SEVEN_F);
            all
        }
        _ => return None,
    })
}

/// The answer GET_REPORT feature 0x01 gives after query `id` was armed.
pub fn query(id: u8, map: &Mapping) -> Option<Vec<u8>> {
    if id == SENSOR_SURFACE {
        return Some(map.d9().to_vec());
    }
    QUERIES
        .iter()
        .find(|(query, _)| *query == id)
        .map(|&(_, length)| vec![QUERY, id, 0x00, length, 0x00])
}

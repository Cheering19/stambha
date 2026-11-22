// SPDX-License-Identifier: GPL-2.0-only
// The expected descriptors and feature answers are VoodooInput's (see
// the parent module).
use super::*;
use crate::usb::control::{
    CONFIGURATION, DEVICE, GET_CONFIGURATION, INTERFACE, SET_ADDRESS, SET_CONFIGURATION, STRING,
    string,
    tests::{setup, transfer},
};
use crate::usb::trackpad::{Identity, serial};
use alloc::vec::Vec;
use stela_drivers::input::multitouch::Finger;

const ELAN: Surface = Surface {
    width_um: 126_900,
    height_um: 82_000,
};
const ENGLISH: u16 = 0x0409;

fn trackpad(identity: Identity) -> Trackpad {
    Trackpad::new(
        Config {
            identity,
            surface: ELAN,
        },
        &serial(0x0011_2233_4455_6677),
    )
}

fn enumerate(t: &mut Trackpad) {
    transfer(t, setup(0, SET_ADDRESS, 3, 0, 0), &[]).unwrap();
    transfer(t, setup(0, SET_CONFIGURATION, 1, 0, 0), &[]).unwrap();
}

fn configured() -> Trackpad {
    let mut t = trackpad(Identity::MagicTrackpad2);
    enumerate(&mut t);
    t
}

fn get_feature(t: &mut Trackpad, id: u8, length: u16) -> Result<Vec<u8>, Stall> {
    transfer(
        t,
        setup(0xa1, GET_REPORT, 0x0300 | u16::from(id), 1, length),
        &[],
    )
}

fn set_feature(t: &mut Trackpad, id: u8, data: &[u8]) -> Result<Vec<u8>, Stall> {
    transfer(
        t,
        setup(
            0x21,
            SET_REPORT,
            0x0300 | u16::from(id),
            1,
            data.len() as u16,
        ),
        data,
    )
}

/// One interrupt transfer on endpoint `endpoint`, packet by packet until a
/// short one: `None` for a NAK, `Err` for a STALL.
fn read(t: &mut Trackpad, endpoint: u8) -> core::result::Result<Option<Vec<u8>>, ()> {
    let mut report = Vec::new();
    loop {
        let mut packet = [0u8; 64];
        match t.packet(endpoint, Token::In, &mut packet).unwrap() {
            Reply::Ack(n) => {
                report.extend_from_slice(&packet[..n]);
                if n < 64 {
                    return Ok(Some(report));
                }
            }
            Reply::Nak if report.is_empty() => return Ok(None),
            Reply::Nak => panic!("a NAK inside a report"),
            Reply::Stall => return Err(()),
        }
    }
}

fn report(t: &mut Trackpad) -> Vec<u8> {
    read(t, 0x83).unwrap().expect("a report is due")
}

/// Pad units of the ELAN over its millimetres, as the host's frames round
/// them.
fn elan(id: u8, x: u64, y: u64, tip: bool, confident: bool) -> Finger {
    Finger {
        id,
        x_um: ((x * 126_900 + 3987 / 2) / 3987) as u32,
        y_um: ((y * 82_000 + 2579 / 2) / 2579) as u32,
        tip,
        confident,
    }
}

fn hex(text: &str) -> Vec<u8> {
    text.split_whitespace()
        .map(|byte| u8::from_str_radix(byte, 16).unwrap())
        .collect()
}

#[test]
fn enumerates_as_05ac_0265() {
    let mut t = trackpad(Identity::MagicTrackpad2);
    let device = transfer(&mut t, setup(0x80, GET_DESCRIPTOR, 0x100, 0, 18), &[]).unwrap();
    assert_eq!(
        device,
        [
            18, DEVICE, 0x00, 0x02, 0, 0, 0, 64, 0xac, 0x05, 0x65, 0x02, 0x55, 0x08, 1, 2, 3, 1
        ]
    );
    enumerate(&mut t);
    assert_eq!(
        transfer(&mut t, setup(0x80, GET_CONFIGURATION, 0, 0, 1), &[]).unwrap(),
        [1]
    );
    let config = transfer(&mut t, setup(0x80, GET_DESCRIPTOR, 0x200, 0, 255), &[]).unwrap();
    assert_eq!(config[1], CONFIGURATION);
    assert_eq!(
        usize::from(u16::from_le_bytes([config[2], config[3]])),
        config.len()
    );
    assert_eq!(config[4], 3, "three interfaces");
    assert_eq!(config[7], 0xc0, "self-powered, no remote wakeup");
    // Walk it: each interface, its HID descriptor and its endpoint.
    let (mut interfaces, mut hids, mut endpoints) = (Vec::new(), Vec::new(), Vec::new());
    let mut at = usize::from(config[0]);
    while at < config.len() {
        let d = &config[at..at + usize::from(config[at])];
        match d[1] {
            INTERFACE => interfaces.push((d[2], d[4], d[5], d[6], d[7])),
            HID => hids.push(u16::from_le_bytes([d[7], d[8]])),
            0x05 => endpoints.push((d[2], d[3], u16::from_le_bytes([d[4], d[5]]), d[6])),
            other => panic!("descriptor type {other:#x}"),
        }
        at += d.len();
    }
    assert_eq!(at, config.len());
    assert_eq!(
        interfaces,
        [(0, 1, 3, 0, 0), (1, 1, 3, 0, 2), (2, 1, 3, 0, 0)],
        "HID; the multitouch interface's protocol is mouse"
    );
    assert_eq!(hids, [21, 110, 36]);
    assert_eq!(
        endpoints,
        [(0x81, 3, 64, 8), (0x83, 3, 64, 1), (0x84, 3, 64, 8)]
    );
    // The report descriptors, VoodooInput's on interfaces 1 and 2.
    for (interface, bytes) in [
        (0, &descriptors::VENDOR_DESCRIPTOR[..]),
        (1, &descriptors::MULTITOUCH_DESCRIPTOR[..]),
        (2, &descriptors::ACTUATOR_DESCRIPTOR[..]),
    ] {
        let got = transfer(
            &mut t,
            setup(0x81, GET_DESCRIPTOR, 0x2200, interface, 512),
            &[],
        )
        .unwrap();
        assert_eq!(got, bytes);
        let hid = transfer(
            &mut t,
            setup(0x81, GET_DESCRIPTOR, 0x2100, interface, 9),
            &[],
        )
        .unwrap();
        assert_eq!(hid, descriptors::hid_descriptor(bytes.len()));
    }
    assert_eq!(
        &descriptors::MULTITOUCH_DESCRIPTOR[..4],
        [0x05, 0x01, 0x09, 0x02]
    );
    assert_eq!(
        &descriptors::MULTITOUCH_DESCRIPTOR[104..],
        [0x96, 0x6b, 0x05, 0x81, 0x00, 0xc0]
    );
    // The strings.
    for (n, text) in [
        (1, "Apple Inc."),
        (2, "Magic Trackpad 2"),
        (3, serial(0x0011_2233_4455_6677).as_str()),
    ] {
        assert_eq!(
            transfer(
                &mut t,
                setup(
                    0x80,
                    GET_DESCRIPTOR,
                    u16::from(STRING) << 8 | n,
                    ENGLISH,
                    255
                ),
                &[]
            )
            .unwrap(),
            string(text)
        );
    }
    // Full speed only.
    assert!(transfer(&mut t, setup(0x80, GET_DESCRIPTOR, 0x600, 0, 10), &[]).is_err());
}

#[test]
fn builtin_identity_is_0281() {
    let mut t = trackpad(Identity::Builtin);
    let device = transfer(&mut t, setup(0x80, GET_DESCRIPTOR, 0x100, 0, 18), &[]).unwrap();
    assert_eq!(
        (device[8], device[9], device[10], device[11]),
        (0xac, 0x05, 0x81, 0x02)
    );
}

#[test]
fn feature_protocol() {
    let mut t = configured();
    let map = Mapping::mt2(ELAN);
    set_feature(&mut t, 0x01, &[0x01, 0xd3]).unwrap();
    assert_eq!(
        get_feature(&mut t, 0x01, 64).unwrap(),
        [0x01, 0xd3, 0x00, 0x0c, 0x00]
    );
    // Every query VoodooInput answers, with its length.
    for (query, length) in [
        (0xdb, 0x49),
        (0xd1, 0x01),
        (0xd0, 0x0f),
        (0xa1, 0x06),
        (0x7f, 0x04),
        (0xc8, 0x01),
    ] {
        set_feature(&mut t, 0x01, &[0x01, query]).unwrap();
        assert_eq!(
            get_feature(&mut t, 0x01, 64).unwrap(),
            [0x01, query, 0x00, length, 0x00]
        );
    }
    // Query 0xD9 is answered with the surface itself.
    set_feature(&mut t, 0x01, &[0x01, 0xd9]).unwrap();
    assert_eq!(get_feature(&mut t, 0x01, 64).unwrap(), map.d9());
    assert_eq!(get_feature(&mut t, 0x00, 64).unwrap(), [0x00, 0x01]);
    assert_eq!(get_feature(&mut t, 0xd1, 64).unwrap(), [0xd1, 0x81]);
    assert_eq!(
        get_feature(&mut t, 0xd3, 64).unwrap(),
        hex("d3 01 16 1e 03 95 00 14 1e 62 05 00 00")
    );
    assert_eq!(
        get_feature(&mut t, 0xd0, 64).unwrap(),
        hex("d0 02 01 00 14 01 00 1e 00 02 14 02 01 0e 02 00")
    );
    assert_eq!(
        get_feature(&mut t, 0xa1, 64).unwrap(),
        hex("a1 00 00 05 00 fc 01")
    );
    assert_eq!(get_feature(&mut t, 0xd9, 64).unwrap(), map.d9());
    assert_eq!(
        get_feature(&mut t, 0x7f, 64).unwrap(),
        hex("7f 00 00 00 00")
    );
    assert_eq!(get_feature(&mut t, 0xc8, 64).unwrap(), [0xc8, 0x08]);
    assert_eq!(get_feature(&mut t, 0x02, 64).unwrap(), [0x02, 0x01]);
    // 0xDB: VoodooInput's composite with our surface, 72 bytes over two
    // packets of endpoint 0.
    let mut db = hex(
        "db 01 02 00 d1 81 0d 00 d3 01 16 1e 03 95 00 14 1e 62 05 00 00 10 00 \
         d0 02 01 00 14 01 00 1e 00 02 14 02 01 0e 02 00 07 00 a1 00 00 05 00 fc 01 11 00",
    );
    db.extend_from_slice(&map.d9());
    db.extend_from_slice(&hex("7f 00 00 00 00"));
    assert_eq!(db.len(), 72);
    assert_eq!(get_feature(&mut t, 0xdb, 255).unwrap(), db);
    // Each feature once in the trace, in the order first asked.
    let trace = t.take_trace();
    let features: Vec<u8> = trace
        .iter()
        .filter_map(|event| match event {
            TrackpadEvent::Feature(id) => Some(*id),
            _ => None,
        })
        .collect();
    assert_eq!(
        features,
        [
            0x01, 0x00, 0xd1, 0xd3, 0xd0, 0xa1, 0xd9, 0x7f, 0xc8, 0x02, 0xdb
        ]
    );
    assert!(trace.contains(&TrackpadEvent::Query {
        id: 0xd3,
        answered: true
    }));
    assert!(t.take_trace().is_empty());
}

#[test]
fn mode_switch_logged() {
    let mut t = configured();
    t.take_trace();
    // Linux's and Windows' switch: SET_REPORT wValue 0x0302, wIndex 1.
    transfer(&mut t, setup(0x21, SET_REPORT, 0x0302, 1, 2), &[0x02, 0x01]).unwrap();
    assert_eq!(t.take_trace(), [TrackpadEvent::Mode(1)]);
    // Windows reads it first.
    assert_eq!(get_feature(&mut t, 0x02, 2).unwrap(), [0x02, 0x01]);
}

#[test]
fn unknown_stalls_once() {
    let mut t = configured();
    t.take_trace();
    for _ in 0..3 {
        assert!(get_feature(&mut t, 0x42, 64).is_err());
    }
    let unknown = TrackpadEvent::Unknown {
        request_type: 0xa1,
        request: GET_REPORT,
        value: 0x0342,
        index: 1,
    };
    assert_eq!(t.take_trace(), [unknown]);
    // An input report the tables do not hold, and an unknown query.
    assert!(transfer(&mut t, setup(0xa1, GET_REPORT, 0x0102, 1, 64), &[]).is_err());
    assert!(set_feature(&mut t, 0x01, &[0x01, 0x42]).is_err());
    assert!(get_feature(&mut t, 0x01, 64).is_err(), "nothing armed");
    assert!(set_feature(&mut t, 0x01, &[0x01, 0x42]).is_err());
    let trace = t.take_trace();
    assert!(trace.contains(&TrackpadEvent::Query {
        id: 0x42,
        answered: false
    }));
    assert_eq!(
        trace
            .iter()
            .filter(|e| matches!(e, TrackpadEvent::Unknown { .. }))
            .count(),
        3,
        "GET input 0x02, SET query, GET query: each once"
    );
    // The pipe works after each STALL.
    assert_eq!(get_feature(&mut t, 0xd1, 64).unwrap(), [0xd1, 0x81]);
}

#[test]
fn the_other_interfaces_answer_their_declared_reports() {
    let mut t = configured();
    assert_eq!(
        transfer(&mut t, setup(0xa1, GET_REPORT, 0x0300, 0, 8), &[]).unwrap(),
        [0]
    );
    let actuator = transfer(&mut t, setup(0xa1, GET_REPORT, 0x013f, 2, 64), &[]).unwrap();
    assert_eq!(actuator.len(), 16);
    assert_eq!(actuator[0], 0x3f);
    assert!(actuator[1..].iter().all(|b| *b == 0));
    // The actuator's haptic output is taken and ignored.
    let mut haptic = vec![0x53];
    haptic.extend([0x11; 63]);
    transfer(&mut t, setup(0x21, SET_REPORT, 0x0253, 2, 64), &haptic).unwrap();
    assert!(transfer(&mut t, setup(0xa1, GET_REPORT, 0x0301, 0, 8), &[]).is_err());
    // Their endpoints never report.
    assert_eq!(read(&mut t, 0x81), Ok(None));
    assert_eq!(read(&mut t, 0x84), Ok(None));
    assert_eq!(t.ready_at(0x81), None);
    // Idle and protocol requests.
    transfer(&mut t, setup(0x21, SET_IDLE, 0, 1, 0), &[]).unwrap();
    assert!(transfer(&mut t, setup(0x21, SET_IDLE, 0x0400, 1, 0), &[]).is_err());
    assert_eq!(
        transfer(&mut t, setup(0xa1, GET_PROTOCOL, 0, 1, 1), &[]).unwrap(),
        [1]
    );
}

#[test]
fn frames_flow_without_switch() {
    let mut t = configured();
    assert_eq!(t.ready_at(0x83), None);
    assert_eq!(read(&mut t, 0x83), Ok(None));
    t.frame(Frame::new(1_000_000, 0, &[elan(0, 1993, 1289, true, true)]));
    assert_eq!(t.ready_at(0x83), Some(0));
    assert_eq!(
        report(&mut t),
        hex("02 00 00 00 00 00 00 03 31 44 1f 00 7f 60 f9 6b 14 14 0a 05 81")
    );
    assert_eq!(t.ready_at(0x83), None);
    // The lift, then the header alone, then nothing.
    t.frame(Frame::new(
        1_008_000,
        0,
        &[elan(0, 1993, 1289, false, true)],
    ));
    let stop = report(&mut t);
    assert_eq!(stop[7], 0x02);
    assert_eq!(stop[15] >> 5, 7, "Stop");
    assert_eq!(t.ready_at(0x83), Some(0), "the header-only report is owed");
    assert_eq!(report(&mut t), hex("02 00 00 00 00 00 00 02 31 84 1f 00"));
    assert_eq!(read(&mut t, 0x83), Ok(None));
    assert_eq!(t.counters().received, 2);
    assert_eq!(t.counters().sent, 3);
    assert_eq!(t.counters().reporting_us, 8_000);
}

#[test]
fn an_unconfigured_guest_keeps_the_newest_frame() {
    let mut t = trackpad(Identity::MagicTrackpad2);
    for n in 0..50 {
        t.frame(Frame::new(
            n * 8_000,
            0,
            &[elan(0, 1000 + n, 1000, true, true)],
        ));
    }
    // Endpoints of an unconfigured device STALL.
    assert_eq!(read(&mut t, 0x83), Err(()));
    assert_eq!(t.ready_at(0x83), None);
    enumerate(&mut t);
    let r = report(&mut t);
    assert_eq!(r.len(), 21);
    assert_eq!(r[15] >> 5, 3, "Start");
    assert_eq!(read(&mut t, 0x83), Ok(None), "one frame was kept");
}

#[test]
fn queue_bound_keeps_lift() {
    let mut t = configured();
    t.take_trace();
    // One finger moving for 100 frames without an IN: its motion coalesces
    // at the full queue's back.
    for n in 0..100u64 {
        t.frame(Frame::new(
            n * 8_000,
            0,
            &[elan(0, 500 + 10 * n, 1000, true, true)],
        ));
    }
    // The lift differs from the back: the oldest frame goes, counted.
    t.frame(Frame::new(800_000, 0, &[]));
    assert_eq!(t.take_trace(), [TrackpadEvent::Dropped(1)]);
    assert_eq!(t.counters().dropped, 1);
    let mut reports = Vec::new();
    while let Some(r) = read(&mut t, 0x83).unwrap() {
        reports.push(r);
    }
    assert_eq!(reports.len(), QUEUE + 1, "the queue, then the header alone");
    assert_eq!(reports[0][15] >> 5, 3, "Start");
    assert!(
        reports[1..QUEUE - 1].iter().all(|r| r[15] >> 5 == 4),
        "Active"
    );
    let stop = &reports[QUEUE - 1];
    assert_eq!((stop.len(), stop[15] >> 5), (21, 7), "Stop");
    assert_eq!(reports[QUEUE].len(), 12, "the header alone");
    // The last Active report is the newest motion, coalesced.
    let last = &reports[QUEUE - 2];
    let moved = Mapping::mt2(ELAN)
        .to_mt2(elan(0, 500 + 990, 1000, true, true).x_um, 0)
        .0;
    assert_eq!(
        i16::from_le_bytes([last[12], last[13] & 0x1f]) << 3 >> 3,
        moved
    );
    // Contacts that keep changing drop one frame each past the bound.
    let mut t = configured();
    t.take_trace();
    for n in 0..100u64 {
        let fingers = [
            elan(0, 500, 1000, true, true),
            elan(1, 900, 1000, true, true),
        ];
        let count = 1 + (n % 2) as usize;
        t.frame(Frame::new(n * 8_000, 0, &fingers[..count]));
    }
    assert_eq!(t.take_trace(), [TrackpadEvent::Dropped(100 - QUEUE as u64)]);
}

#[test]
fn reset_frees_slots() {
    let mut t = configured();
    let finger = |time| Frame::new(time, 0, &[elan(3, 2000, 1000, true, true)]);
    t.frame(finger(0));
    report(&mut t);
    t.frame(finger(8_000));
    assert_eq!(report(&mut t)[15] >> 5, 4, "Active");
    // A bus reset (the guest slept), then a new enumeration: the finger
    // still down starts again, as identifier 1, with no stale Stop.
    t.reset().unwrap();
    assert_eq!(read(&mut t, 0x83), Err(()), "unconfigured");
    enumerate(&mut t);
    t.frame(finger(500_000));
    let again = report(&mut t);
    assert_eq!(again.len(), 21, "one finger, no Stop beside it");
    assert_eq!((again[15] >> 5, again[20] & 0xf), (3, 1));
    let trace = t.take_trace();
    assert!(trace.contains(&TrackpadEvent::Configured(0)));
    assert_eq!(
        trace
            .iter()
            .filter(|e| **e == TrackpadEvent::Configured(1))
            .count(),
        2
    );
}

#[test]
fn a_resting_palm_stays_a_palm() {
    let mut t = configured();
    // A palm the pad never trusts, resting through typing, then lifting.
    for (n, tip) in [true, true, true, false].into_iter().enumerate() {
        t.frame(Frame::new(
            n as u64 * 8_000,
            0,
            &[elan(2, 700, 2300, tip, false)],
        ));
        let r = report(&mut t);
        assert_eq!(r[1], 0, "no click");
        assert_eq!(r[15] >> 2 & 7, 6, "typed Palm in every report");
    }
}

#[test]
fn six_fingers_span_two_packets() {
    let mut t = configured();
    let six: Vec<Finger> = (0..6)
        .map(|n| elan(n, 400 + 500 * u64::from(n), 1200, true, true))
        .collect();
    t.frame(Frame::new(0, 0, &six));
    let mut first = [0u8; 64];
    assert_eq!(
        t.packet(0x83, Token::In, &mut first).unwrap(),
        Reply::Ack(64)
    );
    assert_eq!(t.ready_at(0x83), Some(0), "the rest is due");
    let mut rest = [0u8; 64];
    assert_eq!(
        t.packet(0x83, Token::In, &mut rest).unwrap(),
        Reply::Ack(12 + 6 * 9 - 64)
    );
    assert_eq!(read(&mut t, 0x83), Ok(None));
}

#[test]
fn halted_endpoint_stalls_until_cleared() {
    let mut t = configured();
    transfer(&mut t, setup(0x02, SET_FEATURE, 0, 0x83, 0), &[]).unwrap();
    assert_eq!(
        transfer(&mut t, setup(0x82, GET_STATUS, 0, 0x83, 2), &[]).unwrap(),
        [1, 0]
    );
    t.frame(Frame::new(0, 0, &[elan(0, 100, 100, true, true)]));
    assert_eq!(t.ready_at(0x83), None);
    assert_eq!(read(&mut t, 0x83), Err(()));
    transfer(&mut t, setup(0x02, CLEAR_FEATURE, 0, 0x83, 0), &[]).unwrap();
    assert_eq!(report(&mut t).len(), 21);
}

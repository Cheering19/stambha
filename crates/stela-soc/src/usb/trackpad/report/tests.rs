use super::*;
use alloc::{vec, vec::Vec};
use stela_drivers::input::multitouch::Finger;

/// The ELAN 04F3:32E2: 3987 by 2579 units over 126.90 by 82.00 mm.
const ELAN: Surface = Surface {
    width_um: 126_900,
    height_um: 82_000,
};
/// `ptp/tests.rs`'s fixture pad: 1000 by 600 units over 100 by 60 mm.
const FIXTURE: Surface = Surface {
    width_um: 100_000,
    height_um: 60_000,
};

/// One touch as Linux reads it, with the finger type, which it does not.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Touch {
    id: u8,
    x: i32,
    y: i32,
    size: u8,
    orientation: i32,
    major: u8,
    minor: u8,
    pressure: u8,
    state: u8,
    down: bool,
    kind: u8,
}

/// Linux hid-magicmouse.c's MT2 USB path, line for line:
/// `__magicmouse_raw_event`'s `TRACKPAD2_USB_REPORT_ID` case (the size
/// checks and `clicks = data[1]`) and `magicmouse_emit_touch`'s MT2 branch.
/// `None` where Linux drops the report.
fn linux_decode(data: &[u8]) -> Option<(u8, Vec<Touch>)> {
    if data.first() != Some(&REPORT_ID) {
        return None;
    }
    let size = data.len();
    if size < 12 || !(size - 12).is_multiple_of(9) {
        return None;
    }
    let npoints = (size - 12) / 9;
    if npoints > 15 {
        return None;
    }
    let touches = (0..npoints)
        .map(|ii| {
            let t = &data[12 + ii * 9..12 + ii * 9 + 9];
            let b = |n: usize| u32::from(t[n]);
            // `int` shifts: the bytes' bits shifted to the top, then an
            // arithmetic shift down.
            let x = ((b(1) << 27 | b(0) << 19) as i32) >> 19;
            let y = -(((b(3) << 30 | b(2) << 22 | b(1) << 14) as i32) >> 19);
            let state = t[3] & 0xc0;
            Touch {
                id: t[8] & 0xf,
                x,
                y,
                size: t[6],
                orientation: i32::from(t[8] >> 5) - 4,
                major: t[4],
                minor: t[5],
                pressure: t[7],
                state,
                down: state == 0x80,
                kind: t[3] >> 2 & 7,
            }
        })
        .collect();
    Some((data[1], touches))
}

fn hex(text: &str) -> Vec<u8> {
    text.split_whitespace()
        .map(|byte| u8::from_str_radix(byte, 16).unwrap())
        .collect()
}

/// Pad units over a pad's millimetres, rounded as the host's frames round
/// them (`ptp_frames`).
fn on(units: (u64, u64), surface: Surface, x: u64, y: u64) -> (u32, u32) {
    let scale =
        |value: u64, extent: u32, span: u64| ((value * u64::from(extent) + span / 2) / span) as u32;
    (
        scale(x, surface.width_um, units.0),
        scale(y, surface.height_um, units.1),
    )
}

fn elan(id: u8, x: u64, y: u64, tip: bool, confident: bool) -> Finger {
    let (x_um, y_um) = on((3987, 2579), ELAN, x, y);
    Finger {
        id,
        x_um,
        y_um,
        tip,
        confident,
    }
}

fn encoded(encoder: &mut Encoder, frame: &Frame, map: &Mapping) -> Vec<u8> {
    let mut out = Vec::new();
    encoder.encode(frame, map, &mut out);
    out
}

#[test]
fn golden_elan_a_to_f() {
    let map = Mapping::mt2(ELAN);
    let mut e = Encoder::new();
    let ms = |ms: u64| ms * 1000;
    // A: a finger lands at the centre.
    let a = Frame::new(ms(1000), 0, &[elan(0, 1993, 1289, true, true)]);
    assert_eq!(
        encoded(&mut e, &a, &map),
        hex("02 00 00 00 00 00 00 03 31 44 1f 00 7f 60 f9 6b 14 14 0a 05 81")
    );
    // B: it moves.
    let b = Frame::new(ms(1008), 0, &[elan(0, 2500, 1000, true, true)]);
    assert_eq!(
        encoded(&mut e, &b, &map),
        hex("02 00 00 00 00 00 00 03 31 84 1f 00 7f 03 2c 88 14 14 0a 05 81")
    );
    // C: a second finger lands lower, the button goes down: pressure 120,
    // and the lower finger is the thumb.
    let c = Frame::new(
        ms(1016),
        1,
        &[
            elan(0, 2500, 1000, true, true),
            elan(1, 1000, 2000, true, true),
        ],
    );
    assert_eq!(
        encoded(&mut e, &c, &map),
        hex(
            "02 01 00 00 00 00 00 03 31 c4 1f 00 7f 03 2c 88 14 14 0a 78 81 \
             a0 da 7c 67 14 14 0a 78 82"
        )
    );
    // D: both lift, listed with their tips up.
    let d = Frame::new(
        ms(1024),
        0,
        &[
            elan(0, 2500, 1000, false, true),
            elan(1, 1000, 2000, false, true),
        ],
    );
    assert_eq!(
        encoded(&mut e, &d, &map),
        hex(
            "02 00 00 00 00 00 00 02 31 04 20 00 7f 03 2c e8 00 00 00 00 81 \
             a0 da 7c e7 00 00 00 00 82"
        )
    );
    assert!(e.owes_tail());
    // E: nothing on the pad: the header alone, which is the tail owed.
    let empty = Frame::new(ms(1034), 0, &[]);
    assert_eq!(
        encoded(&mut e, &empty, &map),
        hex("02 00 00 00 00 00 00 02 31 54 20 00")
    );
    assert!(!e.owes_tail());
    // F: a palm, kept and typed Palm, takes the next identifier.
    let f = Frame::new(ms(1042), 0, &[elan(7, 300, 2400, true, false)]);
    assert_eq!(
        encoded(&mut e, &f, &map),
        hex("02 00 00 00 00 00 00 03 31 94 20 00 7c d6 36 7b 14 14 0a 05 83")
    );
}

#[test]
fn golden_fixture_e2e() {
    let map = Mapping::mt2(FIXTURE);
    let mut e = Encoder::new();
    let at = |x: u64, y: u64, tip: bool| {
        let (x_um, y_um) = on((1000, 600), FIXTURE, x, y);
        Finger {
            id: 1,
            x_um,
            y_um,
            tip,
            confident: true,
        }
    };
    let one = Frame::new(1_000_000, 0, &[at(500, 300, true)]);
    assert_eq!(
        encoded(&mut e, &one, &map),
        hex("02 00 00 00 00 00 00 03 31 44 1f 00 80 40 f9 6b 14 14 0a 05 81")
    );
    let two = Frame::new(1_008_000, 0, &[at(600, 300, true)]);
    assert_eq!(
        encoded(&mut e, &two, &map),
        hex("02 00 00 00 00 00 00 03 31 84 1f 00 5c 42 f9 8b 14 14 0a 05 81")
    );
    let lift = Frame::new(1_016_000, 0, &[at(600, 300, false)]);
    assert_eq!(
        encoded(&mut e, &lift, &map),
        hex("02 00 00 00 00 00 00 02 31 c4 1f 00 5c 42 f9 eb 00 00 00 00 81")
    );
    let mut tail = Vec::new();
    assert!(e.tail(&mut tail));
    assert_eq!(tail, hex("02 00 00 00 00 00 00 02 31 c4 1f 00"));
    tail.clear();
    assert!(!e.tail(&mut tail), "once");
    assert!(tail.is_empty());
}

#[test]
fn a_contact_left_out_of_the_frame_stops() {
    let map = Mapping::mt2(ELAN);
    let mut e = Encoder::new();
    let down = Frame::new(0, 0, &[elan(4, 2000, 1000, true, true)]);
    encoded(&mut e, &down, &map);
    let gone = Frame::new(8_000, 0, &[]);
    let (clicks, touches) = linux_decode(&encoded(&mut e, &gone, &map)).unwrap();
    assert_eq!(clicks, 0);
    assert_eq!(touches.len(), 1);
    assert_eq!((touches[0].id, touches[0].state), (1, 0xc0));
    assert_eq!(
        (touches[0].major, touches[0].size, touches[0].pressure),
        (0, 0, 0)
    );
    assert!(e.owes_tail());
    // A lift with the button still down owes no header-only report, which
    // would say the button is up.
    let mut e = Encoder::new();
    let held = Frame::new(0, 1, &[elan(4, 2000, 1000, true, true)]);
    encoded(&mut e, &held, &map);
    let lifted = Frame::new(8_000, 1, &[]);
    let report = encoded(&mut e, &lifted, &map);
    assert_eq!((report[1], report[7]), (1, 0x03));
    assert!(!e.owes_tail());
}

#[test]
fn linux_round_trip() {
    let map = Mapping::mt2(ELAN);
    for i in 0..=20u32 {
        for j in 0..=20u32 {
            let (x_um, y_um) = (ELAN.width_um * i / 20, ELAN.height_um * j / 20);
            let (x, y) = map.to_mt2(x_um, y_um);
            for button in [0u8, 1] {
                let mut e = Encoder::new();
                let finger = Finger {
                    id: 9,
                    x_um,
                    y_um,
                    tip: true,
                    confident: true,
                };
                for (time, state, down) in [(0, 0x40, false), (8_000, 0x80, true)] {
                    let report = encoded(&mut e, &Frame::new(time, button, &[finger]), &map);
                    let (clicks, touches) = linux_decode(&report).expect("Linux takes it");
                    assert_eq!(clicks, button);
                    let t = touches[0];
                    assert_eq!((t.x, t.y), (i32::from(x), -i32::from(y)), "{i},{j}");
                    assert_eq!((t.state, t.down), (state, down));
                    assert_eq!((t.id, t.orientation), (1, 0));
                    assert_eq!(t.pressure, if button == 1 { 120 } else { 5 });
                    assert_eq!((t.major, t.minor, t.size), (20, 20, 10));
                }
            }
        }
    }
}

#[test]
fn corners_centre_and_aspect() {
    let map = Mapping::mt2(ELAN);
    // The corners land inside the MT2's range, the pad centred on its
    // centre (128, 54 downward).
    assert_eq!(map.to_mt2(0, 0), (-2891, 1753));
    assert_eq!(map.to_mt2(126_900, 82_000), (3147, -1861));
    assert_eq!(map.to_mt2(63_450, 41_000), (128, -54));
    // One resolution per axis: 47.575 units per mm across and 44.08 down,
    // the MT2's own, so the pad's 126.9 by 82.0 mm keep their ratio in
    // millimetres.
    let (left, top) = map.to_mt2(0, 0);
    let (right, bottom) = map.to_mt2(126_900, 82_000);
    let width_um = i64::from(right - left) * 160_000 / 7612;
    let height_um = i64::from(top - bottom) * 114_900 / 5065;
    assert!((width_um - 126_900).abs() < 50, "{width_um}");
    assert!((height_um - 82_000).abs() < 50, "{height_um}");
    // A surface larger than the MT2's is clamped to its range.
    let large = Mapping::mt2(Surface {
        width_um: 300_000,
        height_um: 200_000,
    });
    assert_eq!(large.to_mt2(0, 0), (-3678, 2478));
    assert_eq!(large.to_mt2(300_000, 200_000), (3934, -2587));
}

#[test]
fn d9_is_surface_and_range() {
    let mut expected = hex("d9 92 31 00 00 08 20 00 00");
    for value in [-2891i16, -1861, 3147, 1753] {
        expected.extend(value.to_le_bytes());
    }
    assert_eq!(Mapping::mt2(ELAN).d9().to_vec(), expected);
}

#[test]
fn palm_type_and_thumb() {
    let map = Mapping::mt2(ELAN);
    let kinds = |report: &[u8]| -> Vec<u8> {
        let (_, touches) = linux_decode(report).unwrap();
        touches.iter().map(|t| t.kind).collect()
    };
    // A palm alone, for its whole life: Palm, then Stop as Palm.
    let mut e = Encoder::new();
    let palm = |tip| elan(3, 1000, 2400, tip, false);
    assert_eq!(
        kinds(&encoded(&mut e, &Frame::new(0, 0, &[palm(true)]), &map)),
        [6]
    );
    assert_eq!(
        kinds(&encoded(&mut e, &Frame::new(8_000, 0, &[palm(true)]), &map)),
        [6]
    );
    assert_eq!(
        kinds(&encoded(
            &mut e,
            &Frame::new(16_000, 0, &[palm(false)]),
            &map
        )),
        [6]
    );
    // The button with two fingers: the lower (larger y) is the thumb; the
    // other keeps its order's type.
    let mut e = Encoder::new();
    let two = [
        elan(1, 2000, 2000, true, true),
        elan(2, 2200, 500, true, true),
    ];
    assert_eq!(
        kinds(&encoded(&mut e, &Frame::new(0, 1, &two), &map)),
        [1, 3]
    );
    // Without the button and with fewer than four contacts, no thumb.
    assert_eq!(
        kinds(&encoded(&mut e, &Frame::new(8_000, 0, &two), &map)),
        [2, 3]
    );
    // A palm lowest of all is never the thumb: the lowest finger is.
    let mut e = Encoder::new();
    let with_palm = [
        elan(1, 2000, 1500, true, true),
        elan(2, 2200, 500, true, true),
        elan(3, 1000, 2500, true, false),
    ];
    assert_eq!(
        kinds(&encoded(&mut e, &Frame::new(0, 1, &with_palm), &map)),
        [1, 3, 6]
    );
    // Four fingers touching make the lowest a thumb without the button.
    let mut e = Encoder::new();
    let four: Vec<Finger> = (0..4)
        .map(|n| {
            elan(
                n,
                500 + 700 * u64::from(n),
                1000 + 100 * u64::from(n),
                true,
                true,
            )
        })
        .collect();
    assert_eq!(
        kinds(&encoded(&mut e, &Frame::new(0, 0, &four), &map)),
        [2, 3, 4, 1]
    );
}

#[test]
fn fifteen_finger_cap() {
    let map = Mapping::mt2(ELAN);
    let mut e = Encoder::new();
    let fingers = |base: u8| -> Vec<Finger> {
        (0..10)
            .map(|n| elan(base + n, 300 * u64::from(n) + 100, 1200, true, true))
            .collect()
    };
    let first = encoded(&mut e, &Frame::new(0, 0, &fingers(0)), &map);
    assert_eq!(linux_decode(&first).unwrap().1.len(), 10);
    // Ten others replace them at once: ten Stops, and five of the new
    // contacts in the slots left; Linux takes the 15.
    let second = encoded(&mut e, &Frame::new(8_000, 0, &fingers(20)), &map);
    assert_eq!(second.len(), HEADER + MAX_FINGERS * FINGER);
    let (_, touches) = linux_decode(&second).unwrap();
    assert_eq!(touches.iter().filter(|t| t.state == 0xc0).count(), 10);
    assert_eq!(touches.iter().filter(|t| t.state == 0x40).count(), 5);
    // The other five start once the Stops freed their slots.
    let third = encoded(&mut e, &Frame::new(16_000, 0, &fingers(20)), &map);
    let (_, touches) = linux_decode(&third).unwrap();
    assert_eq!(touches.len(), 10);
    assert_eq!(touches.iter().filter(|t| t.state == 0x40).count(), 5);
    assert_eq!(touches.iter().filter(|t| t.state == 0x80).count(), 5);
    let mut ids: Vec<u8> = touches.iter().map(|t| t.id).collect();
    ids.sort_unstable();
    ids.dedup();
    assert_eq!(ids.len(), 10, "every contact its own identifier");
}

#[test]
fn identifiers_rotate_and_reset_starts_over() {
    let map = Mapping::mt2(ELAN);
    let mut e = Encoder::new();
    let tap = |e: &mut Encoder, id: u8, time: u64| -> u8 {
        let down = encoded(
            e,
            &Frame::new(time, 0, &[elan(id, 2000, 1000, true, true)]),
            &map,
        );
        encoded(e, &Frame::new(time + 8_000, 0, &[]), &map);
        linux_decode(&down).unwrap().1[0].id
    };
    assert_eq!(tap(&mut e, 0, 0), 1);
    assert_eq!(tap(&mut e, 0, 20_000), 2, "a new touch, a new identifier");
    // A contact still down when the guest forgets everything starts again.
    let down = Frame::new(40_000, 0, &[elan(5, 2000, 1000, true, true)]);
    encoded(&mut e, &down, &map);
    e.reset();
    let again = encoded(
        &mut e,
        &Frame::new(48_000, 0, &[elan(5, 2000, 1000, true, true)]),
        &map,
    );
    let t = linux_decode(&again).unwrap().1[0];
    assert_eq!((t.id, t.state), (1, 0x40));
}

#[test]
fn timestamps_wrap_at_21_bits() {
    let map = Mapping::mt2(ELAN);
    let mut e = Encoder::new();
    let report = encoded(&mut e, &Frame::new(((1 << 21) + 5) * 1000, 0, &[]), &map);
    assert_eq!(&report[9..12], &[5 << 3 | 4, 0, 0]);
    let report = encoded(&mut e, &Frame::new(((1 << 21) - 1) * 1000, 0, &[]), &map);
    assert_eq!(&report[9..12], &[0xfc, 0xff, 0xff]);
    assert_eq!(vec![report[0], report[8]], [REPORT_ID, 0x31]);
}

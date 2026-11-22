use super::*;
use crate::input::{
    Input,
    ptp::tests::{EMPTY, Pad, report},
};
use crate::usb::hid::descriptor::ReportField;
use alloc::{vec, vec::Vec};

fn touches(inputs: &[Input]) -> Vec<Frame> {
    inputs
        .iter()
        .map(|input| match input {
            Input::Touch(frame) => *frame,
            other => panic!("frames mode gave {other:?}"),
        })
        .collect()
}

fn frames_pad() -> Pad {
    let mut pad = Pad::new();
    pad.ptp.set_frames(true);
    pad
}

/// An axis field of `maximum` units over `physical` in `unit` and
/// `exponent`.
fn axis(maximum: i64, physical: i64, unit: u32, exponent: i8) -> ReportField {
    ReportField {
        offset: 0,
        size: 16,
        count: 1,
        flags: 2,
        minimum: 0,
        maximum,
        physical_minimum: 0,
        physical_maximum: physical,
        unit,
        unit_exponent: exponent,
        collection: None,
        usages: vec![0x0001_0030],
    }
}

#[test]
fn surface_is_micrometres() {
    let pad = Pad::new();
    assert_eq!(
        pad.ptp.surface(),
        Surface {
            width_um: 100_000,
            height_um: 60_000
        }
    );
    // The ELAN 04F3:32E2's X: 0..3987 over 12690 at 10^-3 cm, 126.90 mm,
    // which whole millimetres would round to 127.
    let x = axis(3987, 12690, 0x11, -3);
    let y = axis(2579, 8200, 0x11, -3);
    assert_eq!(
        surface(&x, &y),
        Some(Surface {
            width_um: 126_900,
            height_um: 82_000
        })
    );
    // English linear: 4 inches at 10^0 is 101.6 mm.
    let inch = axis(1000, 4, 0x13, 0);
    assert_eq!(surface(&inch, &inch).map(|s| s.width_um), Some(101_600));
    // No unit, or no physical range: no stated size, and libinput's assumed
    // 69 by 50 mm for the frames.
    let bare = axis(1000, 0, 0, 0);
    assert_eq!(surface(&bare, &bare), None);
    let scale = Scale::of(&bare, &bare, None);
    assert_eq!(scale.stated(), None);
    assert_eq!(
        scale.surface(),
        Surface {
            width_um: 69_000,
            height_um: 50_000
        }
    );
    assert_eq!(scale.scan_max(), 0xffff);
}

#[test]
fn frames_mode_emits_touch_not_motion() {
    let mut pad = frames_pad();
    let first = report([(true, true, 1, 500, 300), EMPTY], 10, 1, false);
    let moved = report([(true, true, 1, 600, 300), EMPTY], 20, 1, false);
    let mut inputs = pad.feed(&first, 1_000_000);
    inputs.extend(pad.feed(&moved, 1_001_000));
    let frames = touches(&inputs);
    assert_eq!(frames.len(), 2, "one frame per report, nothing else");
    let at = |frame: &Frame| {
        let f = frame.fingers()[0];
        (f.id, f.x_um, f.y_um, f.tip, f.confident)
    };
    assert_eq!(at(&frames[0]), (1, 50_000, 30_000, true, true));
    assert_eq!(at(&frames[1]), (1, 60_000, 30_000, true, true));
    // The lift reaches the frame with its tip up; a button-only report is
    // a frame of no contacts.
    let lift = report([(true, false, 1, 600, 300), EMPTY], 30, 1, false);
    let lifted = touches(&pad.feed(&lift, 1_002_000));
    assert_eq!(lifted[0].fingers().len(), 1);
    assert!(!lifted[0].fingers()[0].tip);
    let click = report([EMPTY, EMPTY], 40, 0, true);
    let clicked = touches(&pad.feed(&click, 1_003_000));
    assert_eq!(clicked.len(), 1);
    assert!(clicked[0].fingers().is_empty());
    assert_eq!(clicked[0].buttons, 1);
}

#[test]
fn non_confident_contact_is_kept() {
    let mut pad = frames_pad();
    // A palm, its whole life without confidence, beside a finger.
    let palm = report(
        [(false, true, 3, 100, 100), (true, true, 4, 900, 500)],
        5,
        2,
        false,
    );
    let frames = touches(&pad.feed(&palm, 0));
    let fingers = frames[0].fingers();
    assert_eq!(fingers.len(), 2);
    assert_eq!(
        (fingers[0].id, fingers[0].tip, fingers[0].confident),
        (3, true, false)
    );
    assert_eq!(
        (fingers[1].id, fingers[1].tip, fingers[1].confident),
        (4, true, true)
    );
    assert_eq!((fingers[0].x_um, fingers[0].y_um), (10_000, 10_000));
}

#[test]
fn scan_time_drives_time() {
    // Through the pad: scan 100 then 180 is 8 ms, whatever the polls say.
    let mut pad = frames_pad();
    let one = report([(true, true, 1, 500, 300), EMPTY], 100, 1, false);
    let two = report([(true, true, 1, 510, 300), EMPTY], 180, 1, false);
    let a = touches(&pad.feed(&one, 1_000_000))[0].time_us;
    let b = touches(&pad.feed(&two, 1_000_100))[0].time_us;
    assert_eq!((a, b), (1_000_000, 1_008_000));
    // The counter wraps at 65535: 65500 to 30 is 66 counts.
    let mut clock = Clock::default();
    assert_eq!(clock.time_us(Some(65_500), 0xffff, 5_000_000), 5_000_000);
    assert_eq!(clock.time_us(Some(30), 0xffff, 5_000_200), 5_006_600);
    // A second scan time within a frame of hybrid reports adds nothing.
    assert_eq!(clock.time_us(Some(30), 0xffff, 5_000_300), 5_006_600);
    // A poll gap over a second resynchronizes to the poll clock, which is
    // past the scan clock here.
    assert_eq!(clock.time_us(Some(40), 0xffff, 6_500_000), 6_500_000);
    assert_eq!(clock.time_us(Some(50), 0xffff, 6_500_100), 6_501_000);
    // A resync behind the scan clock keeps time from running back.
    let mut fast = Clock::default();
    assert_eq!(fast.time_us(Some(0), 0xffff, 0), 0);
    assert_eq!(fast.time_us(Some(60_000), 0xffff, 900_000), 6_000_000);
    assert_eq!(fast.time_us(Some(0), 0xffff, 2_000_000), 6_000_000);
    // A pad without Scan Time runs on the poll clock.
    let mut bare = Clock::default();
    assert_eq!(bare.time_us(None, 0xffff, 10), 10);
    assert_eq!(bare.time_us(None, 0xffff, 25), 25);
}

#[test]
fn three_fingers_reach_the_frame() {
    let mut pad = frames_pad();
    let start = report(
        [(true, true, 1, 100, 100), (true, true, 2, 200, 100)],
        8,
        3,
        true,
    );
    assert!(pad.feed(&start, 10_000).is_empty(), "frame incomplete");
    let rest = report([(true, true, 3, 300, 100), EMPTY], 8, 0, true);
    let frames = touches(&pad.feed(&rest, 10_000));
    assert_eq!(frames.len(), 1);
    assert_eq!(
        frames[0].fingers().iter().map(|f| f.id).collect::<Vec<_>>(),
        [1, 2, 3]
    );
    assert_eq!(frames[0].buttons, 1);
}

#[test]
fn frames_off_is_unchanged() {
    // The same reports through a pad with frames mode set and cleared, and
    // through one never set: the mouse path's inputs, identical.
    let sequence = [
        (report([(true, true, 1, 100, 300), EMPTY], 10, 1, false), 0),
        (
            report([(true, true, 1, 350, 300), EMPTY], 20, 1, false),
            300_000,
        ),
        (
            report([(true, true, 1, 350, 300), EMPTY], 30, 1, true),
            310_000,
        ),
        (report([EMPTY, EMPTY], 40, 0, false), 320_000),
    ];
    let mut plain = Pad::new();
    let mut toggled = Pad::new();
    toggled.ptp.set_frames(true);
    toggled.ptp.set_frames(false);
    for (bytes, now) in &sequence {
        let expected = plain.feed(bytes, *now);
        assert!(
            !expected
                .iter()
                .any(|input| matches!(input, Input::Touch(_))),
            "no frame without frames mode"
        );
        assert_eq!(toggled.feed(bytes, *now), expected);
    }
}

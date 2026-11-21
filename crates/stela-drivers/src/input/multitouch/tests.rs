use super::*;
use crate::input::{HeldInputs, Input};
use alloc::vec::Vec;

fn finger(id: u8, tip: bool) -> Finger {
    Finger {
        id,
        x_um: 1000 * u32::from(id),
        y_um: 500,
        tip,
        confident: true,
    }
}

#[test]
fn frame_keeps_ten_fingers_in_order() {
    let twelve: Vec<Finger> = (0..12).map(|id| finger(id, true)).collect();
    let frame = Frame::new(7, 1, &twelve);
    assert_eq!(frame.fingers().len(), MAX_FINGERS);
    assert_eq!(
        frame.fingers().iter().map(|f| f.id).collect::<Vec<_>>(),
        (0..10).collect::<Vec<_>>()
    );
    assert_eq!((frame.time_us, frame.buttons), (7, 1));
    let lifted = Frame::lifted(9);
    assert!(lifted.fingers().is_empty());
    assert_eq!((lifted.time_us, lifted.buttons), (9, 0));
    assert!(!lifted.holds());
}

#[test]
fn held_releases_a_touch() {
    // A finger down, then the device stops: one lifted frame at the last
    // frame's time.
    let mut held = HeldInputs::new();
    let mut inputs = alloc::vec![Input::Touch(Frame::new(1_000, 0, &[finger(0, true)]))];
    held.polled(3, &mut inputs, 0);
    let mut out = Vec::new();
    held.stopped(3, &mut out);
    assert_eq!(out, [Input::Touch(Frame::lifted(1_000))]);
    out.clear();
    held.stopped(3, &mut out);
    assert!(out.is_empty(), "once");
    // Button 1 held with no finger listed.
    let mut inputs = alloc::vec![Input::Touch(Frame::new(2_000, 1, &[]))];
    held.polled(4, &mut inputs, 0);
    held.stopped(4, &mut out);
    assert_eq!(out, [Input::Touch(Frame::lifted(2_000))]);
    // A touch that lifted holds nothing.
    out.clear();
    let mut inputs = alloc::vec![
        Input::Touch(Frame::new(3_000, 0, &[finger(0, true)])),
        Input::Touch(Frame::new(3_008, 0, &[finger(0, false)])),
    ];
    held.polled(5, &mut inputs, 0);
    held.stopped(5, &mut out);
    assert!(out.is_empty());
}

#[test]
fn a_touch_passes_through_beside_another_device_s_buttons() {
    let mut held = HeldInputs::new();
    let mut mouse = alloc::vec![Input::MouseButtons(1)];
    held.polled(0, &mut mouse, 0);
    let frame = Frame::new(5, 0, &[finger(1, true)]);
    let mut pad = alloc::vec![Input::Touch(frame)];
    held.polled(1, &mut pad, 0);
    assert_eq!(pad, [Input::Touch(frame)]);
    // The pad stops: its touch lifts, the mouse's button stays.
    let mut out = Vec::new();
    held.stopped(1, &mut out);
    assert_eq!(out, [Input::Touch(Frame::lifted(5))]);
}

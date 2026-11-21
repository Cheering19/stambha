//! A precision touchpad's frames mode through the I²C-HID input.
use super::*;
use crate::input::{
    multitouch::Surface,
    ptp::tests::{EMPTY, descriptor as touchpad_descriptor, report},
};

#[test]
fn a_precision_touchpad_passes_frames_through_on_request() {
    let (mut d, h) = device(0x04f3, 0x32e2);
    {
        let mut h = h.borrow_mut();
        h.report_desc = touchpad_descriptor();
        // wReportDescLength; the reports fit wMaxInputLength's 32 bytes.
        let length = h.report_desc.len() as u16;
        h.desc[4..6].copy_from_slice(&length.to_le_bytes());
    }
    d.initialize().unwrap();
    let mut input = HidInput::new(d, String::from("pad")).unwrap();
    assert!(input.is_precision_touchpad());
    assert_eq!(
        input.touch_surface(),
        Some(Surface {
            width_um: 100_000,
            height_um: 60_000
        })
    );
    let feed = |input: &mut HidInput<_, _, _>, bytes: Vec<u8>| {
        h.borrow_mut().input.push_back(Hardware::frame(&bytes));
        let mut inputs = Vec::new();
        input.poll(&mut inputs).unwrap();
        inputs
    };
    // The mouse path until frames mode is set: a finger moving 10 mm is
    // motion.
    feed(
        &mut input,
        report([(true, true, 1, 500, 300), EMPTY], 10, 1, false),
    );
    let moved = feed(
        &mut input,
        report([(true, true, 1, 600, 300), EMPTY], 20, 1, false),
    );
    assert!(matches!(moved[..], [Input::Motion { .. }]), "{moved:?}");
    assert!(input.set_touch_frames(true));
    let frames = feed(
        &mut input,
        report([(true, true, 1, 700, 300), EMPTY], 30, 1, false),
    );
    let [Input::Touch(frame)] = frames[..] else {
        panic!("{frames:?}");
    };
    assert_eq!(frame.fingers()[0].x_um, 70_000);
}

#[test]
fn another_device_has_no_frames_mode() {
    let (d, _h) = ready();
    let mut input = HidInput::new(d, String::from("vendor")).unwrap();
    assert!(!input.is_precision_touchpad());
    assert!(!input.set_touch_frames(true));
    assert_eq!(input.touch_surface(), None);
}

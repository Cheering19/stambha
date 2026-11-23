//! Reset responses no command waits for, and a precision touchpad's mode
//! switch: an expected reset after the switch, a pad that resets itself,
//! and a switch that fails and leaves the pad a mouse.
use super::*;
use crate::input::ptp::tests::{EMPTY, descriptor as touchpad_descriptor, report};

const INPUT_MODE_ID: u8 = 3;
const TOUCHPAD_MODE: u8 = 3;

/// An ELAN precision touchpad that keeps each feature report apart, started.
fn pad() -> (Device<Model, Pin, Time>, Rc<RefCell<Hardware>>) {
    let (mut d, h) = device(0x04f3, 0x32e2);
    {
        let mut h = h.borrow_mut();
        h.report_desc = touchpad_descriptor();
        let length = h.report_desc.len() as u16;
        h.desc[4..6].copy_from_slice(&length.to_le_bytes());
        h.keyed = true;
    }
    d.initialize().unwrap();
    (d, h)
}

fn input_mode(h: &Rc<RefCell<Hardware>>) -> Option<u8> {
    h.borrow()
        .features
        .iter()
        .find(|(id, _)| *id == INPUT_MODE_ID)
        .map(|(_, bytes)| bytes[0])
}

fn touch(h: &Rc<RefCell<Hardware>>, input: &mut HidInput<Model, Pin, Time>) -> Vec<Input> {
    h.borrow_mut().input.push_back(Hardware::frame(&report(
        [(true, true, 1, 700, 300), EMPTY],
        30,
        1,
        false,
    )));
    let mut inputs = Vec::new();
    input.poll(&mut inputs).unwrap();
    inputs
}

#[test]
fn a_reset_the_mode_switch_causes_is_waited_for_and_the_mode_set_again() {
    let (d, h) = pad();
    // The pad answers its Input Mode report by resetting out of it.
    h.borrow_mut().reset_on_feature = 1;
    let mut input = HidInput::new(d, String::from("pad")).unwrap();
    assert!(input.is_precision_touchpad());
    assert_eq!(input.name, "pad precision touchpad");
    assert!(input.device().is_ready());
    // Started again (a second host RESET, its response awaited on the pin)
    // and switched once more.
    assert_eq!(h.borrow().reset_count, 2);
    assert_eq!(input_mode(&h), Some(TOUCHPAD_MODE));
    assert!(input.take_notices().is_empty());
    assert!(input.set_touch_frames(true));
    assert!(matches!(touch(&h, &mut input)[..], [Input::Touch(_)]));
}

#[test]
fn a_switch_answered_by_reset_twice_leaves_a_working_mouse() {
    let (d, h) = pad();
    h.borrow_mut().reset_on_feature = usize::MAX;
    let mut input = HidInput::new(d, String::from("pad")).unwrap();
    assert!(!input.is_precision_touchpad());
    assert_eq!(input.name, "pad");
    assert_eq!(
        input.take_notices(),
        vec![Notice::MouseMode(Fallback::ResetAfterSwitch)]
    );
    // Started a last time, in its firmware's mouse mode, and polled.
    assert!(input.device().is_ready());
    assert_eq!(input_mode(&h), None);
    h.borrow_mut().reset_on_feature = 0;
    assert!(!input.set_touch_frames(true));
    let mut inputs = Vec::new();
    input.poll(&mut inputs).unwrap();
    assert!(input.device().is_ready());
}

#[test]
fn a_failed_switch_falls_back_to_mouse_mode() {
    let (d, h) = pad();
    // The Input Mode report is written, then the transfer fails.
    h.borrow_mut().fail_after_opcode = Some(SET_REPORT);
    let resets = h.borrow().reset_count;
    let mut input = HidInput::new(d, String::from("pad")).unwrap();
    assert!(!input.is_precision_touchpad());
    assert_eq!(input.name, "pad");
    let notices = input.take_notices();
    assert!(
        matches!(
            notices[..],
            [Notice::MouseMode(Fallback::Switch(Error {
                kind: ErrorKind::Bus(_),
                ..
            }))]
        ),
        "{notices:?}"
    );
    // The half-applied mode is cleared by starting the pad again, and the
    // device is ready, not removed.
    assert_eq!(h.borrow().reset_count, resets + 1);
    assert_eq!(input_mode(&h), None);
    assert!(input.device().is_ready());
    assert!(!input.device().needs_reset());
    h.borrow_mut().input.push_back(Hardware::frame(&[1, 0]));
    let mut inputs = Vec::new();
    input.poll(&mut inputs).unwrap();
    assert!(input.device().is_ready());
}

#[test]
fn a_pad_that_resets_itself_is_started_again_in_touchpad_mode() {
    let (d, h) = pad();
    let mut input = HidInput::new(d, String::from("pad")).unwrap();
    assert!(input.set_touch_frames(true));
    let resets = h.borrow().reset_count;
    h.borrow_mut().device_reset();
    assert_eq!(input_mode(&h), None);
    let mut inputs = Vec::new();
    input.poll(&mut inputs).unwrap();
    assert!(input.device().is_ready());
    assert!(input.is_precision_touchpad());
    assert_eq!(
        input.take_notices(),
        vec![Notice::ModeRestored { count: 1 }]
    );
    assert_eq!(h.borrow().reset_count, resets + 1);
    assert_eq!(input_mode(&h), Some(TOUCHPAD_MODE));
    // Frames mode is kept across the restore.
    assert!(matches!(touch(&h, &mut input)[..], [Input::Touch(_)]));
}

#[test]
fn a_reset_response_with_the_mode_held_is_dropped_once_logged() {
    let (d, h) = pad();
    let mut input = HidInput::new(d, String::from("pad")).unwrap();
    let resets = h.borrow().reset_count;
    for _ in 0..2 {
        h.borrow_mut().input.push_back(vec![0, 0]);
        let mut inputs = Vec::new();
        input.poll(&mut inputs).unwrap();
    }
    assert_eq!(input.take_notices(), vec![Notice::ResetDropped]);
    assert_eq!(h.borrow().reset_count, resets);
    assert!(input.is_precision_touchpad());
    assert!(input.device().is_ready());
}

#[test]
fn another_device_keeps_running_through_its_own_reset() {
    let (d, h) = ready();
    let mut input = HidInput::new(d, String::from("vendor")).unwrap();
    let resets = h.borrow().reset_count;
    h.borrow_mut().input.push_back(vec![0, 0]);
    h.borrow_mut()
        .input
        .push_back(Hardware::frame(&[17, 0xaa, 0xbb]));
    let mut inputs = Vec::new();
    input.poll(&mut inputs).unwrap();
    input.poll(&mut inputs).unwrap();
    assert_eq!(input.last_report(), [17, 0xaa, 0xbb]);
    assert_eq!(input.take_notices(), vec![Notice::ResetDropped]);
    assert_eq!(h.borrow().reset_count, resets);
    assert!(input.device().is_ready());
}

#[test]
fn a_pad_that_keeps_resetting_is_read_as_a_mouse() {
    let (d, h) = pad();
    let mut input = HidInput::new(d, String::from("pad")).unwrap();
    for count in 1..=MAX_RESTORES {
        h.borrow_mut().device_reset();
        let mut inputs = Vec::new();
        input.poll(&mut inputs).unwrap();
        assert_eq!(input.take_notices(), vec![Notice::ModeRestored { count }]);
    }
    h.borrow_mut().device_reset();
    let mut inputs = Vec::new();
    input.poll(&mut inputs).unwrap();
    assert_eq!(
        input.take_notices(),
        vec![Notice::MouseMode(Fallback::Resets(MAX_RESTORES + 1))]
    );
    assert!(!input.is_precision_touchpad());
    assert!(input.device().is_ready());
}

#[test]
fn reinitialize_refuses_a_device_whose_timing_failed_or_never_started() {
    let (mut d, _h) = device(0x06cb, 1);
    assert_eq!(d.reinitialize().unwrap_err().kind, ErrorKind::NotReady);
    let (mut d, h) = ready();
    h.borrow_mut().fail_next = true;
    assert!(d.set_report(ReportType::Feature, 1, &[1]).is_err());
    assert!(d.needs_reset());
    d.reinitialize().unwrap();
    assert!(d.is_ready());
    assert_eq!(h.borrow().reset_count, 2);
    d.sleep().unwrap();
    assert_eq!(d.reinitialize().unwrap_err().kind, ErrorKind::NotReady);
}

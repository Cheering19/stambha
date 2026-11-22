//! The host's lid, with `--lid method` ([`stela_boot_core::lid_panel`]):
//! the lid's notifications (`aml`) and the built-in keyboard and touchpad
//! (`input`) move its request of the built-in panel, which the scanout
//! follows (`gpu::display`); with `--lid-sleep` each move also asks the
//! guest to sleep or to wake ([`take_request`]). With `--lid ignore`, the
//! default, nothing here moves.
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use stela_boot_core::lid_panel::LidRequest;

/// `--lid method`.
static METHOD: AtomicBool = AtomicBool::new(false);
/// `--lid-sleep`.
static SLEEP: AtomicBool = AtomicBool::new(false);
/// The lid asks the built-in panel dark.
static DARK: AtomicBool = AtomicBool::new(false);
/// The guest's request a move left for the next service turn.
static REQUEST: AtomicU8 = AtomicU8::new(NONE);
const NONE: u8 = 0;
const SLEEP_REQUEST: u8 = 1;
const WAKE_REQUEST: u8 = 2;

/// What a move of the lid's request asks of the guest.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(in crate::engine) enum Request {
    /// The panel went dark: a HID System Sleep.
    Sleep,
    /// The panel lit again: the power button's wake, where the guest sleeps.
    Wake,
}

/// Follow the lid (`--lid method`), and with it the guest's sleep
/// (`--lid-sleep`).
pub(in crate::engine) fn configure(method: bool, sleep: bool) {
    METHOD.store(method, Ordering::Release);
    SLEEP.store(method && sleep, Ordering::Release);
}

/// Whether the lid is followed (`--lid method`).
pub(in crate::engine) fn method() -> bool {
    METHOD.load(Ordering::Acquire)
}

/// A lid notification, after which `_LID` read `open`: the request it
/// changed to, where it did.
pub(in crate::engine) fn notified(open: bool) -> Option<bool> {
    step(|request| request.notified(open))
}

/// Input of the built-in keyboard or touchpad: `Some(false)` where it lit
/// a panel the lid had darkened.
pub(in crate::engine) fn built_in_input() -> Option<bool> {
    if !DARK.load(Ordering::Acquire) {
        return None;
    }
    step(LidRequest::input)
}

fn step(change: impl Fn(&mut LidRequest) -> Option<bool>) -> Option<bool> {
    if !method() {
        return None;
    }
    let mut changed = None;
    let _ = DARK.try_update(Ordering::AcqRel, Ordering::Acquire, |dark| {
        let mut request = LidRequest::with(dark);
        changed = change(&mut request);
        changed.map(|_| request.dark())
    });
    if let Some(dark) = changed
        && SLEEP.load(Ordering::Acquire)
    {
        let request = if dark { SLEEP_REQUEST } else { WAKE_REQUEST };
        REQUEST.store(request, Ordering::Release);
    }
    changed
}

/// Whether the lid asks the built-in panel dark.
#[cfg_attr(not(feature = "nvk"), allow(dead_code))]
pub(in crate::engine) fn dark() -> bool {
    DARK.load(Ordering::Acquire)
}

/// The request the last move left, once: a later move replaces one not
/// yet taken, as only the last counts.
pub(in crate::engine) fn take_request() -> Option<Request> {
    match REQUEST.swap(NONE, Ordering::AcqRel) {
        SLEEP_REQUEST => Some(Request::Sleep),
        WAKE_REQUEST => Some(Request::Wake),
        _ => None,
    }
}

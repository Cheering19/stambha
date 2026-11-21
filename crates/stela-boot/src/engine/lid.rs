//! The host's lid as ACPI last read it (`aml`), for the scanout and the
//! guest: while it is closed, a guest display shown on built-in panels
//! alone stays dark (`gpu::display`, [`stela_boot_core::lid_panel`]); with
//! `--lid-sleep` a close asks macOS to sleep and an open wakes it
//! ([`take_request`]).
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

static CLOSED: AtomicBool = AtomicBool::new(false);
/// `--lid-sleep`.
static SLEEP: AtomicBool = AtomicBool::new(false);
/// The guest's request a lid change left for the next service turn.
static REQUEST: AtomicU8 = AtomicU8::new(NONE);
const NONE: u8 = 0;
const SLEEP_REQUEST: u8 = 1;
const WAKE_REQUEST: u8 = 2;

/// What a lid change asks of the guest.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(in crate::engine) enum Request {
    /// The lid closed: a HID System Sleep.
    Sleep,
    /// The lid opened: the power button's wake, where the guest sleeps.
    Wake,
}

/// Follow the lid with the guest's sleep (`--lid-sleep`).
pub(in crate::engine) fn enable_sleep() {
    SLEEP.store(true, Ordering::Release);
}

/// The lid was read open, or closed; `changed` where it moved since the
/// last reading, not the state it starts in, which asks the guest nothing.
pub(in crate::engine) fn set(open: bool, changed: bool) {
    CLOSED.store(!open, Ordering::Release);
    if changed && SLEEP.load(Ordering::Acquire) {
        let request = if open { WAKE_REQUEST } else { SLEEP_REQUEST };
        REQUEST.store(request, Ordering::Release);
    }
}

/// Whether the lid was last read closed; open until a reading says not.
#[cfg_attr(not(feature = "nvk"), allow(dead_code))]
pub(in crate::engine) fn closed() -> bool {
    CLOSED.load(Ordering::Acquire)
}

/// The request the last lid change left, once: a later change replaces
/// one not yet taken, as only the lid's last state counts.
pub(in crate::engine) fn take_request() -> Option<Request> {
    match REQUEST.swap(NONE, Ordering::AcqRel) {
        SLEEP_REQUEST => Some(Request::Sleep),
        WAKE_REQUEST => Some(Request::Wake),
        _ => None,
    }
}

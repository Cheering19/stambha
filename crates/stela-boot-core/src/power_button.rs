//! What a press of the host's power button does to the guest.
//!
//! A PC's ACPI power button status records a press, not how long the
//! button was held, so a held button cannot be told from a short press. A
//! short press of a Mac's button asks macOS to sleep or to show its
//! shutdown choice; holding it cuts the power, whatever the guest does. The
//! host gives the guest a press and a release for each press, and stands in
//! for a hold with a second press within [`FORCE_WINDOW_MILLIS`] of the
//! first, while that request is still pending: the guest has not slept
//! since. A press while the guest sleeps wakes it and starts over, as does
//! any press once the window passed, so a press minutes later, or after a
//! sleep and wake, is a new request and never cuts the guest off.

/// How soon after a press a second one stands for a hold of the button.
pub const FORCE_WINDOW_MILLIS: u64 = 3000;

/// What one press does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Press {
    /// A press and a release for the guest.
    Normal,
    /// The button held: the machine turns off, whatever the guest does.
    ForceOff,
}

/// The last press that may still be pending.
#[derive(Clone, Copy, Debug, Default)]
pub struct Presses {
    /// Its counter value and the guest's sleeps then.
    last: Option<(u64, u64)>,
}

impl Presses {
    pub const fn new() -> Self {
        Self { last: None }
    }

    /// A press at counter value `now`, with the window `window` ticks long,
    /// the guest having slept `sleeps` times since the boot and `asleep`
    /// now.
    pub fn press(&mut self, now: u64, window: u64, sleeps: u64, asleep: bool) -> Press {
        if asleep {
            // A wake: the request before it was answered by the sleep.
            self.last = None;
            return Press::Normal;
        }
        let pending = self
            .last
            .is_some_and(|(at, slept)| slept == sleeps && now.wrapping_sub(at) < window);
        if pending {
            self.last = None;
            Press::ForceOff
        } else {
            self.last = Some((now, sleeps));
            Press::Normal
        }
    }
}

/// How long after the service's first look a control-method power
/// button's notification stands for one its firmware raised before (an EC
/// query pending at start), not a press.
pub const NOTIFY_SETTLE_MILLIS: u64 = 5000;
/// How long a power button's notifications must stop before the next is a
/// new press: firmware may notify again and again while the button is
/// held (the FA608PP's EC query `_QE7` notifies, sleeps 500 ms and is
/// queried again), or with the fixed event of the same press.
pub const NOTIFY_DEBOUNCE_MILLIS: u64 = 1000;

/// What a control-method power button's notification is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Notified {
    /// A press: a press and a release for the guest. A notification never
    /// stands for a hold, as a repeat of one press may follow it.
    Press,
    /// The press a notification or a fixed event just before stood for.
    Repeat,
    /// Within the settling after the first look.
    Settling,
}

/// A control-method power button's notifications, taken as presses.
#[derive(Clone, Copy, Debug, Default)]
pub struct Notifications {
    /// The service's first look, the settling's start.
    start: Option<u64>,
    /// The last notification and the last fixed-event press.
    notified: Option<u64>,
    fixed: Option<u64>,
}

impl Notifications {
    pub const fn new() -> Self {
        Self {
            start: None,
            notified: None,
            fixed: None,
        }
    }

    /// The service looked at `now`: the first look starts the settling.
    pub fn look(&mut self, now: u64) {
        self.start.get_or_insert(now);
    }

    /// One or more notifications seen at `now`, with `settle` and
    /// `debounce` ticks long: a press after `debounce` of quiet from every
    /// notification and fixed press, else a repeat of the last.
    pub fn notified(&mut self, now: u64, settle: u64, debounce: u64) -> Notified {
        let start = *self.start.get_or_insert(now);
        let recent = |at: Option<u64>| at.is_some_and(|at| now.wrapping_sub(at) < debounce);
        let notified = if now.wrapping_sub(start) < settle {
            Notified::Settling
        } else if recent(self.notified) || recent(self.fixed) {
            Notified::Repeat
        } else {
            Notified::Press
        };
        self.notified = Some(now);
        notified
    }

    /// A fixed-event press at `now`: whether it is one of its own, not the
    /// press a notification within `debounce` before already stood for.
    /// Fixed presses keep their own pairing ([`Presses`]).
    pub fn fixed(&mut self, now: u64, debounce: u64) -> bool {
        let own = !self
            .notified
            .is_some_and(|at| now.wrapping_sub(at) < debounce);
        self.fixed = Some(now);
        own
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WINDOW: u64 = 3000;

    #[test]
    fn a_press_after_the_window_is_a_new_request() {
        let mut presses = Presses::new();
        assert_eq!(presses.press(1000, WINDOW, 0, false), Press::Normal);
        assert_eq!(presses.press(4000, WINDOW, 0, false), Press::Normal);
        // Minutes later, and again.
        assert_eq!(presses.press(600_000, WINDOW, 0, false), Press::Normal);
        assert_eq!(presses.press(900_000, WINDOW, 0, false), Press::Normal);
    }

    #[test]
    fn a_second_press_within_the_window_forces_the_machine_off() {
        let mut presses = Presses::new();
        assert_eq!(presses.press(1000, WINDOW, 0, false), Press::Normal);
        assert_eq!(presses.press(3999, WINDOW, 0, false), Press::ForceOff);
        // The forced press starts over.
        assert_eq!(presses.press(4500, WINDOW, 0, false), Press::Normal);
    }

    #[test]
    fn a_sleep_and_wake_start_over() {
        let mut presses = Presses::new();
        // A press, the guest sleeps, a press wakes it, a press soon after
        // asks again.
        assert_eq!(presses.press(1000, WINDOW, 0, false), Press::Normal);
        assert_eq!(presses.press(1500, WINDOW, 1, true), Press::Normal);
        assert_eq!(presses.press(2000, WINDOW, 1, false), Press::Normal);
        // A sleep since the last press, the guest awake again (an RTC
        // wake): a new request.
        assert_eq!(presses.press(2500, WINDOW, 2, false), Press::Normal);
        // Two presses with no sleep between still force.
        assert_eq!(presses.press(2600, WINDOW, 2, false), Press::ForceOff);
    }

    const SETTLE: u64 = 5000;
    const DEBOUNCE: u64 = 1000;

    #[test]
    fn notifications_settle_after_the_first_look() {
        let mut notes = Notifications::new();
        notes.look(10_000);
        // Queued before the boot looked, or pending EC queries at start.
        assert_eq!(notes.notified(10_000, SETTLE, DEBOUNCE), Notified::Settling);
        assert_eq!(notes.notified(14_800, SETTLE, DEBOUNCE), Notified::Settling);
        // A burst that ran past the settling stays one.
        assert_eq!(notes.notified(15_500, SETTLE, DEBOUNCE), Notified::Repeat);
        assert_eq!(notes.notified(17_000, SETTLE, DEBOUNCE), Notified::Press);
    }

    #[test]
    fn a_held_buttons_repeated_notifications_are_one_press() {
        let mut notes = Notifications::new();
        notes.look(0);
        // The FA608PP's _QE7 notifies, sleeps 500 ms and runs again while
        // the button is held: 0, 0.5, 1.0 and 1.5 s are one press.
        assert_eq!(notes.notified(10_000, SETTLE, DEBOUNCE), Notified::Press);
        for at in [10_500, 11_000, 11_500] {
            assert_eq!(notes.notified(at, SETTLE, DEBOUNCE), Notified::Repeat);
        }
        // A second of quiet ends it.
        assert_eq!(notes.notified(12_500, SETTLE, DEBOUNCE), Notified::Press);
    }

    #[test]
    fn a_fixed_event_and_a_notification_of_one_press_are_one_press() {
        let mut notes = Notifications::new();
        notes.look(0);
        assert_eq!(notes.notified(10_000, SETTLE, DEBOUNCE), Notified::Press);
        assert!(!notes.fixed(10_400, DEBOUNCE));
        assert!(notes.fixed(20_000, DEBOUNCE));
        assert_eq!(notes.notified(20_300, SETTLE, DEBOUNCE), Notified::Repeat);
        // Fixed presses alone keep their own pairing, however close.
        let mut fixed = Notifications::new();
        assert!(fixed.fixed(1000, DEBOUNCE));
        assert!(fixed.fixed(1200, DEBOUNCE));
    }

    #[test]
    fn a_counter_that_wraps_keeps_the_window() {
        let mut presses = Presses::new();
        assert_eq!(
            presses.press(u64::MAX - 10, WINDOW, 0, false),
            Press::Normal
        );
        assert_eq!(presses.press(100, WINDOW, 0, false), Press::ForceOff);
    }
}

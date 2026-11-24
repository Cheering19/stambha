//! How the guest's pointer follows the host's in real time: the rate of the
//! display's presents that moved a layer (a pointer moves its layer whole),
//! and the time from a host pointer report to the first such present after
//! it, both on the host's clock. The display paces itself on the guest's
//! counter, so its own rate (`DCP PACING`) is per guest second.
//!
//! The service notes the host read of the first pointer report a present
//! has not yet answered ([`inputs`]); the boot PE looks at the display's
//! count of moved presents after each slice ([`Cursor::look`]), so a
//! latency is late by one slice at most. A pointer report is a mouse's or
//! a touchpad's motion, or, with `--trackpad`, a touch frame whose one
//! finger moved: the guest's trackpad moves its pointer by one finger's
//! travel.

use alloc::string::String;
use core::sync::atomic::{AtomicU64, Ordering};
use stela_drivers::input::{Input, multitouch::Frame};
use stela_soc::dcp::iomfb::moved_presents;

/// When the host read the oldest pointer report no moved present followed
/// yet, in TSC ticks; zero for none.
static MOTION_AT: AtomicU64 = AtomicU64::new(0);

/// The last touch frame's one finger ([`touched`]); zero for none.
static TOUCHED: AtomicU64 = AtomicU64::new(0);

/// The host read `inputs` at TSC `now`.
pub(super) fn inputs(inputs: &[Input], now: u64) {
    for input in inputs {
        let moved = match input {
            Input::Move { .. } | Input::Motion { .. } => true,
            Input::Touch(frame) => {
                let finger = touched(frame);
                let last = TOUCHED.swap(finger, Ordering::Relaxed);
                finger != 0 && last != 0 && finger != last
            }
            _ => false,
        };
        if moved {
            let _ = MOTION_AT.compare_exchange(0, now.max(1), Ordering::Relaxed, Ordering::Relaxed);
        }
    }
}

/// Where `frame`'s one finger touching the pad is, as `x << 32 | y` in
/// micrometres with bit 63 set; zero unless one finger touches it.
fn touched(frame: &Frame) -> u64 {
    let mut tips = frame.fingers().iter().filter(|finger| finger.tip);
    match (tips.next(), tips.next()) {
        (Some(finger), None) => {
            1 << 63 | u64::from(finger.x_um & 0x7fff_ffff) << 32 | u64::from(finger.y_um)
        }
        _ => 0,
    }
}

/// The moved presents and latencies since the last report.
pub(super) struct Cursor {
    seen: u64,
    since: u64,
    moved: u64,
    samples: u64,
    total: u64,
    longest: u64,
}

impl Cursor {
    pub(super) fn new(now: u64) -> Self {
        Self {
            seen: moved_presents(),
            since: now,
            moved: 0,
            samples: 0,
            total: 0,
            longest: 0,
        }
    }

    /// Note the moved presents since the last look, at TSC `now`.
    pub(super) fn look(&mut self, now: u64) {
        let seen = moved_presents();
        if seen == self.seen {
            return;
        }
        self.moved += seen.wrapping_sub(self.seen);
        self.seen = seen;
        let at = MOTION_AT.swap(0, Ordering::Relaxed);
        if at != 0 {
            let latency = now.saturating_sub(at);
            self.samples += 1;
            self.total += latency;
            self.longest = self.longest.max(latency);
        }
    }

    /// The window's line, and a new window from TSC `now`.
    pub(super) fn report(&mut self, now: u64, tsc_hz: u64) -> String {
        let tsc_hz = u128::from(tsc_hz.max(1));
        let millis = |ticks: u64| u128::from(ticks) * 1000 / tsc_hz;
        let wall = now.saturating_sub(self.since).max(1);
        let per_ten = u128::from(self.moved) * 10 * tsc_hz / u128::from(wall);
        let pending = MOTION_AT.load(Ordering::Relaxed);
        let line = alloc::format!(
            "CURSOR moved_per_wall_second={}.{} input_to_present_ms={}/{} samples={} unanswered_ms={}\n",
            per_ten / 10,
            per_ten % 10,
            millis(self.total / self.samples.max(1)),
            millis(self.longest),
            self.samples,
            if pending == 0 {
                0
            } else {
                millis(now.saturating_sub(pending))
            },
        );
        *self = Self {
            seen: self.seen,
            since: now,
            moved: 0,
            samples: 0,
            total: 0,
            longest: 0,
        };
        line
    }
}

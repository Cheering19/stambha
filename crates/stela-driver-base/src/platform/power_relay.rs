//! The host's power readings on their way to the guest: the processor that
//! runs the firmware's AML publishes each reading ([`Relay::publish`]), and
//! whichever processor serves the guest takes the newest ([`Relay::turn`])
//! and passes it to the guest's SMC, which says when its notifications are
//! due ([`Relay::delivered`]). A turn also says when the host's battery is
//! due to be read again, every [`REFRESH_MILLIS`], for firmware that does
//! not notify each change of charge. Every value is an atomic or behind
//! [`Latest`]'s flag, so a relay can be a `static` that both processors
//! reach.
use super::power_snapshot::PowerSnapshot;
use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicBool, AtomicU8, AtomicU32, AtomicU64, Ordering},
};

/// How often the host's battery is read again with no notification.
pub const REFRESH_MILLIS: u64 = 30_000;
/// How often it is read while the guest waits for a complete reading, and
/// how long the guest waits: the SMC takes a battery only until macOS has
/// read its key table, early in its boot.
pub const WAITING_REFRESH_MILLIS: u64 = 1_000;
pub const WAIT_MILLIS: u64 = 60_000;
/// A reading that fails, or takes this long, doubles the period, up to
/// [`MAX_REFRESH_MILLIS`].
pub const SLOW_REFRESH_MILLIS: u64 = 100;
pub const MAX_REFRESH_MILLIS: u64 = 300_000;

/// What a timed reading's outcome means for the log and the next one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Polled {
    /// It failed, the first since one succeeded: say so.
    pub failed: bool,
    /// It took [`SLOW_REFRESH_MILLIS`] or more, the first that did: say so.
    pub slow: bool,
    /// It succeeded after a failure: say so.
    pub recovered: bool,
    /// The period until the next.
    pub period_ms: u64,
}

/// Where the host's battery is on its way to the guest.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// No host battery goes to the guest.
    Off,
    /// The guest waits for a complete reading to attach.
    Waiting,
    /// The guest's SMC has the battery.
    Attached,
}

/// The latest of a value one processor writes and another reads: a flag
/// one processor at a time holds for one copy in or out, and a count of
/// the writes.
pub struct Latest<T: Copy> {
    held: AtomicBool,
    generation: AtomicU32,
    value: UnsafeCell<Option<T>>,
}

// SAFETY: `value` is reached only by the processor that set `held`, for one
// copy of a `Copy` value in or out.
unsafe impl<T: Copy + Send> Sync for Latest<T> {}

impl<T: Copy> Latest<T> {
    pub const fn new() -> Self {
        Self {
            held: AtomicBool::new(false),
            generation: AtomicU32::new(0),
            value: UnsafeCell::new(None),
        }
    }

    fn hold(&self) {
        while self
            .held
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            core::hint::spin_loop();
        }
    }

    /// Replace the value, waiting for a reader's copy.
    pub fn store(&self, value: T) {
        self.hold();
        // SAFETY: `held` gives this processor the only reference.
        unsafe { *self.value.get() = Some(value) };
        self.generation.fetch_add(1, Ordering::Relaxed);
        self.held.store(false, Ordering::Release);
    }

    /// The value and the count of writes that made it, waiting for a
    /// writer's copy.
    pub fn load(&self) -> (Option<T>, u32) {
        self.hold();
        // SAFETY: as in `store`.
        let value = unsafe { *self.value.get() };
        let generation = self.generation.load(Ordering::Relaxed);
        self.held.store(false, Ordering::Release);
        (value, generation)
    }

    /// A value newer than the write `seen` counts, which it then counts;
    /// `None` while a writer holds it, for a later call.
    pub fn take_newer(&self, seen: &AtomicU32) -> Option<T> {
        if self.generation.load(Ordering::Acquire) == seen.load(Ordering::Relaxed) {
            return None;
        }
        if self
            .held
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            return None;
        }
        // SAFETY: as in `store`.
        let value = unsafe { *self.value.get() };
        seen.store(self.generation.load(Ordering::Relaxed), Ordering::Relaxed);
        self.held.store(false, Ordering::Release);
        value
    }
}

impl<T: Copy> Default for Latest<T> {
    fn default() -> Self {
        Self::new()
    }
}

/// What one service turn does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Turn {
    /// Ask the firmware for a new reading.
    pub refresh: bool,
    /// A reading newer than the guest's, to pass on.
    pub reading: Option<PowerSnapshot>,
    /// No newer reading, but the guest's notifications are due.
    pub flush: bool,
    /// The guest waited [`WAIT_MILLIS`] for a complete reading in vain: no
    /// host battery goes to it now.
    pub expired: bool,
}

/// Readings from the host's firmware to the guest's SMC.
pub struct Relay {
    latest: Latest<PowerSnapshot>,
    /// The write of the reading the SMC has.
    seen: AtomicU32,
    /// When a notification the SMC holds is due, in milliseconds; 0 none.
    due: AtomicU64,
    /// When the firmware was last asked for a reading, in milliseconds;
    /// [`NEVER`] before the first.
    refreshed: AtomicU64,
    /// The last reading logged's summary (`Summary::pack`); 0 none.
    logged: AtomicU32,
    /// [`Mode`], as [`OFF`], [`WAITING`] or [`ATTACHED`].
    mode: AtomicU8,
    /// When a wait for a complete reading ends, in milliseconds.
    until: AtomicU64,
    /// The period a failed or slow reading set; 0 for the mode's own.
    period: AtomicU64,
    /// The last timed reading failed.
    failing: AtomicBool,
    /// A slow timed reading was said.
    slow_said: AtomicBool,
}

const NEVER: u64 = u64::MAX;
const OFF: u8 = 0;
const WAITING: u8 = 1;
const ATTACHED: u8 = 2;

impl Relay {
    pub const fn new() -> Self {
        Self {
            latest: Latest::new(),
            seen: AtomicU32::new(0),
            due: AtomicU64::new(0),
            refreshed: AtomicU64::new(NEVER),
            logged: AtomicU32::new(0),
            mode: AtomicU8::new(OFF),
            until: AtomicU64::new(0),
            period: AtomicU64::new(0),
            failing: AtomicBool::new(false),
            slow_said: AtomicBool::new(false),
        }
    }

    pub fn mode(&self) -> Mode {
        match self.mode.load(Ordering::Acquire) {
            WAITING => Mode::Waiting,
            ATTACHED => Mode::Attached,
            _ => Mode::Off,
        }
    }

    /// The guest waits for a complete reading, from `now_ms` for at most
    /// [`WAIT_MILLIS`].
    pub fn wait(&self, now_ms: u64) {
        self.until
            .store(now_ms.saturating_add(WAIT_MILLIS), Ordering::Relaxed);
        self.mode.store(WAITING, Ordering::Release);
    }

    /// The guest's SMC took the battery.
    pub fn attached(&self) {
        self.mode.store(ATTACHED, Ordering::Release);
    }

    /// No host battery goes to the guest any more.
    pub fn give_up(&self) {
        self.mode.store(OFF, Ordering::Release);
    }

    /// The firmware's newest reading.
    pub fn publish(&self, reading: PowerSnapshot) {
        self.latest.store(reading);
    }

    /// The newest reading, which the SMC then counts as having.
    pub fn take(&self) -> Option<PowerSnapshot> {
        let (reading, generation) = self.latest.load();
        self.seen.store(generation, Ordering::Relaxed);
        reading
    }

    /// One service turn at `now_ms`.
    pub fn turn(&self, now_ms: u64) -> Turn {
        if self.mode() == Mode::Waiting && now_ms >= self.until.load(Ordering::Relaxed) {
            self.give_up();
            return Turn {
                refresh: false,
                reading: None,
                flush: false,
                expired: true,
            };
        }
        let period = self.period.load(Ordering::Relaxed).max(self.base_period());
        let refreshed = self.refreshed.load(Ordering::Relaxed);
        let refresh = refreshed == NEVER || now_ms.wrapping_sub(refreshed) >= period;
        if refresh {
            self.refreshed.store(now_ms, Ordering::Relaxed);
        }
        let reading = self.latest.take_newer(&self.seen);
        let due = self.due.load(Ordering::Relaxed);
        Turn {
            refresh,
            reading,
            flush: reading.is_none() && due != 0 && now_ms >= due,
            expired: false,
        }
    }

    /// A timed reading the turns asked for ended, `failed` or not, after
    /// `took_ms` of the firmware's AML.
    pub fn polled(&self, failed: bool, took_ms: u64) -> Polled {
        let slow = took_ms >= SLOW_REFRESH_MILLIS;
        let base = self.base_period();
        let period = if failed || slow {
            let before = self.period.load(Ordering::Relaxed).max(base);
            before.saturating_mul(2).min(MAX_REFRESH_MILLIS)
        } else {
            base
        };
        self.period
            .store(if period == base { 0 } else { period }, Ordering::Relaxed);
        let was_failing = self.failing.swap(failed, Ordering::Relaxed);
        Polled {
            failed: failed && !was_failing,
            slow: slow && !self.slow_said.swap(true, Ordering::Relaxed),
            recovered: !failed && was_failing,
            period_ms: period,
        }
    }

    /// The period between timed readings with none failing.
    fn base_period(&self) -> u64 {
        match self.mode() {
            Mode::Waiting => WAITING_REFRESH_MILLIS,
            Mode::Off | Mode::Attached => REFRESH_MILLIS,
        }
    }

    /// The SMC took the turn's reading or notifications; it holds more
    /// that are due at `next_ms`.
    pub fn delivered(&self, next_ms: Option<u64>) {
        self.due.store(next_ms.unwrap_or(0), Ordering::Relaxed);
    }

    /// Whether `reading` changes what macOS acts on since the last one
    /// asked about: a line for the log.
    pub fn changed(&self, reading: &PowerSnapshot) -> bool {
        let summary = reading.summary().pack();
        self.logged.swap(summary, Ordering::Relaxed) != summary
    }
}

impl Default for Relay {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests;

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
    sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering},
};

/// How often the host's battery is read again with no notification.
pub const REFRESH_MILLIS: u64 = 30_000;

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
}

const NEVER: u64 = u64::MAX;

impl Relay {
    pub const fn new() -> Self {
        Self {
            latest: Latest::new(),
            seen: AtomicU32::new(0),
            due: AtomicU64::new(0),
            refreshed: AtomicU64::new(NEVER),
            logged: AtomicU32::new(0),
        }
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
        let refreshed = self.refreshed.load(Ordering::Relaxed);
        let refresh = refreshed == NEVER || now_ms.wrapping_sub(refreshed) >= REFRESH_MILLIS;
        if refresh {
            self.refreshed.store(now_ms, Ordering::Relaxed);
        }
        let reading = self.latest.take_newer(&self.seen);
        let due = self.due.load(Ordering::Relaxed);
        Turn {
            refresh,
            reading,
            flush: reading.is_none() && due != 0 && now_ms >= due,
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

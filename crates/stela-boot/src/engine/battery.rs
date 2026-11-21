//! The host's battery and AC adapter for the guest's SMC (`--battery`), or
//! a synthetic battery in their place (`--test-battery`), which a machine
//! with none, such as a virtual one, can boot with.
//!
//! The AML host reads the batteries and the AC adapter as they attach and
//! after every notification ([`super::aml`]) and publishes each reading here
//! ([`publish`]): the first battery with a reading and the adapter, in the
//! SMC's units (`power_snapshot`). Before the guest starts, [`attach`] gives
//! the SMC that reading, and each service turn ([`serve`]) passes a newer
//! one on, and sends the battery's notifications when the SMC says they are
//! due. A reading is published on the boot processor, which runs the AML,
//! and taken by whichever core serves the host. Firmware need not notify a
//! change of charge: the AML host reads them again every
//! [`REFRESH_SECONDS`].
//!
//! The reading at attach, and each later one that changes what macOS acts
//! on (the AC adapter, the battery's status, percentage or critical flag),
//! is a `POWER battery ...` line.
use super::{aml::HostAcpi, host, output, parallel};
use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, AtomicU64, Ordering},
};
use stela_boot_core::options::Battery;
use stela_drivers::platform::power_snapshot::{BatteryState, PowerSnapshot, Readings};

/// How often the host's batteries and AC adapter are read again with no
/// notification.
const REFRESH_SECONDS: u64 = 30;

/// The latest reading, the boot processor writing it and one serving core
/// at a time reading it; its generation counts the writes.
struct Latest {
    held: AtomicBool,
    generation: AtomicU32,
    reading: UnsafeCell<Option<PowerSnapshot>>,
}

// SAFETY: `reading` is reached only by the core that set `held`, for one
// copy in or out; the snapshot is plain `Copy` data.
unsafe impl Sync for Latest {}

impl Latest {
    const fn new() -> Self {
        Self {
            held: AtomicBool::new(false),
            generation: AtomicU32::new(0),
            reading: UnsafeCell::new(None),
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

    fn store(&self, reading: PowerSnapshot) {
        self.hold();
        // SAFETY: `held` gives this core the only reference.
        unsafe { *self.reading.get() = Some(reading) };
        self.generation.fetch_add(1, Ordering::Relaxed);
        self.held.store(false, Ordering::Release);
    }

    /// The reading and its generation.
    fn load(&self) -> (Option<PowerSnapshot>, u32) {
        self.hold();
        // SAFETY: as in `store`.
        let reading = unsafe { *self.reading.get() };
        let generation = self.generation.load(Ordering::Relaxed);
        self.held.store(false, Ordering::Release);
        (reading, generation)
    }

    /// A reading newer than the one `seen` names, which it then names;
    /// `None` while a write holds it, for a later turn.
    fn take_newer(&self, seen: &AtomicU32) -> Option<PowerSnapshot> {
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
        let reading = unsafe { *self.reading.get() };
        seen.store(self.generation.load(Ordering::Relaxed), Ordering::Relaxed);
        self.held.store(false, Ordering::Release);
        reading
    }
}

static LATEST: Latest = Latest::new();
/// The SMC has the host's battery: the service passes readings on.
static HOST: AtomicBool = AtomicBool::new(false);
/// The AML host, which reads the battery again on request.
static ACPI: AtomicPtr<HostAcpi> = AtomicPtr::new(core::ptr::null_mut());
/// The generation of the reading the SMC has.
static SEEN: AtomicU32 = AtomicU32::new(0);
/// When a notification the SMC holds is due, in milliseconds; 0 for none.
static DUE: AtomicU64 = AtomicU64::new(0);
/// When the battery was last read again on request, in milliseconds.
static REFRESHED: AtomicU64 = AtomicU64::new(0);
/// The summary of the last reading logged (`Summary::pack`).
static LOGGED: AtomicU32 = AtomicU32::new(0);
/// A refused update was reported.
static REFUSED: AtomicBool = AtomicBool::new(false);

fn now_ms() -> u64 {
    host::fresh_counter() / (host::COUNTER_HZ / 1000)
}

/// The AML host read the batteries and AC adapter `power` holds: its first
/// battery with a reading, and the adapter, become the latest reading. On
/// the boot processor, while it holds them.
pub(super) fn publish(power: &stela_drivers::probe::Power) {
    let ac_online = power.ac.as_ref().and_then(|ac| ac.online());
    let battery = power
        .batteries
        .iter()
        .find_map(|battery| battery.snapshot())
        .map(|snapshot| BatteryState::from_readings(&Readings::acpi(snapshot), ac_online));
    LATEST.store(PowerSnapshot { ac_online, battery });
}

/// Give the guest's SMC the battery `choice` names, before the guest starts:
/// the host's latest reading, or the synthetic battery. Without a reading of
/// a host battery the SMC keeps the Mac mini's keys, and the line says why.
pub(super) fn attach(
    devices: &parallel::Board,
    choice: Option<Battery>,
    acpi: Option<&'static HostAcpi>,
) {
    let Some(choice) = choice else {
        return;
    };
    let (snapshot, source) = match choice {
        Battery::Host => {
            let (reading, generation) = LATEST.load();
            let Some(reading) = reading.filter(|reading| reading.battery.is_some()) else {
                output::arguments(format_args!(
                    "POWER battery: --battery, but the host reported no ACPI battery reading{}; \
                     the SMC answers as the Mac mini's\n",
                    if acpi.is_none() {
                        " (its ACPI namespace did not load)"
                    } else {
                        ""
                    }
                ));
                return;
            };
            SEEN.store(generation, Ordering::Relaxed);
            (reading, "the host's ACPI battery")
        }
        Battery::Test { percent, state } => (PowerSnapshot::synthetic(percent, state), "--test-battery"),
    };
    if let Err(error) = devices.with(|devices| devices.add_battery(&snapshot)) {
        output::arguments(format_args!(
            "POWER battery refused: {error}; the SMC answers as the Mac mini's\n"
        ));
        return;
    }
    output::arguments(format_args!(
        "POWER {snapshot} (the SMC's battery, one pack, from {source})\n"
    ));
    LOGGED.store(snapshot.summary().pack(), Ordering::Relaxed);
    if choice == Battery::Host {
        if let Some(acpi) = acpi {
            ACPI.store(core::ptr::from_ref(acpi).cast_mut(), Ordering::Release);
        }
        HOST.store(true, Ordering::Release);
    }
}

/// One service turn: ask the AML host for a new reading every
/// [`REFRESH_SECONDS`], pass a newer reading to the SMC, and send the
/// notifications it holds once they are due. Nothing without the host's
/// battery.
pub(super) fn serve(devices: &parallel::Board) {
    if !HOST.load(Ordering::Acquire) {
        return;
    }
    let now = now_ms();
    if now.wrapping_sub(REFRESHED.load(Ordering::Relaxed)) >= REFRESH_SECONDS * 1000 {
        REFRESHED.store(now, Ordering::Relaxed);
        let acpi = ACPI.load(Ordering::Acquire);
        // SAFETY: the AML host is leaked for the whole program.
        if let Some(acpi) = unsafe { acpi.as_ref() } {
            acpi.refresh_power();
        }
    }
    let reading = LATEST.take_newer(&SEEN);
    let due = DUE.load(Ordering::Relaxed);
    if reading.is_none() && (due == 0 || now < due) {
        return;
    }
    let next = devices.serve(|devices| match &reading {
        Some(reading) => devices.power_update(reading, now),
        None => devices.power_due(now),
    });
    match next {
        Ok(next) => DUE.store(next.unwrap_or(0), Ordering::Relaxed),
        Err(error) => {
            DUE.store(0, Ordering::Relaxed);
            if !REFUSED.swap(true, Ordering::Relaxed) {
                output::arguments(format_args!("POWER battery update refused: {error}\n"));
            }
        }
    }
    if let Some(reading) = reading {
        let summary = reading.summary().pack();
        if LOGGED.swap(summary, Ordering::Relaxed) != summary {
            output::arguments(format_args!("POWER {reading}\n"));
        }
    }
}

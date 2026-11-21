//! The host's battery and AC adapter for the guest's SMC (`--battery`), or
//! a synthetic battery in their place (`--test-battery`), which a machine
//! with none, such as a virtual one, can boot with.
//!
//! The AML host reads the batteries and the AC adapter as they attach and
//! after every notification ([`super::aml`]) and publishes each reading
//! ([`publish`]): the first battery with a reading and the adapter, in the
//! SMC's units (`power_snapshot`). Before the guest starts, [`attach`] gives
//! the SMC that reading; each service turn ([`serve`]) then does what the
//! relay (`power_relay`) says: ask the AML host for a new reading, pass a
//! newer one on, or send the notifications the SMC holds once due.
//!
//! The reading at attach, and each later one that changes what macOS acts
//! on (the AC adapter, the battery's status, percentage or critical flag),
//! is a `POWER battery ...` line.
use super::{aml::HostAcpi, host, output, parallel};
use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};
use stela_boot_core::options::Battery;
use stela_drivers::platform::{
    power_relay::Relay,
    power_snapshot::{BatteryState, PowerSnapshot, Readings},
};

static RELAY: Relay = Relay::new();
/// The SMC has the host's battery: the service passes readings on.
static HOST: AtomicBool = AtomicBool::new(false);
/// The AML host, which reads the battery again on request.
static ACPI: AtomicPtr<HostAcpi> = AtomicPtr::new(core::ptr::null_mut());
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
    RELAY.publish(PowerSnapshot { ac_online, battery });
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
            let Some(reading) = RELAY.take().filter(|reading| reading.battery.is_some()) else {
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
    if let Some(battery) = &snapshot.battery {
        output::arguments(format_args!(
            "POWER {snapshot} {} (the SMC's battery, one pack, from {source})\n",
            battery.thresholds()
        ));
    }
    RELAY.changed(&snapshot);
    if choice == Battery::Host {
        if let Some(acpi) = acpi {
            ACPI.store(core::ptr::from_ref(acpi).cast_mut(), Ordering::Release);
        }
        HOST.store(true, Ordering::Release);
    }
}

/// One service turn, as the relay says. Nothing without the host's battery.
pub(super) fn serve(devices: &parallel::Board) {
    if !HOST.load(Ordering::Acquire) {
        return;
    }
    let now = now_ms();
    let turn = RELAY.turn(now);
    if turn.refresh {
        // SAFETY: the AML host is leaked for the whole program.
        if let Some(acpi) = unsafe { ACPI.load(Ordering::Acquire).as_ref() } {
            acpi.refresh_power();
        }
    }
    if turn.reading.is_none() && !turn.flush {
        return;
    }
    let next = devices.serve(|devices| match &turn.reading {
        Some(reading) => devices.power_update(reading, now),
        None => devices.power_due(now),
    });
    match next {
        Ok(next) => RELAY.delivered(next),
        Err(error) => {
            RELAY.delivered(None);
            if !REFUSED.swap(true, Ordering::Relaxed) {
                output::arguments(format_args!("POWER battery update refused: {error}\n"));
            }
        }
    }
    if let Some(reading) = turn.reading
        && RELAY.changed(&reading)
    {
        output::arguments(format_args!("POWER {reading}\n"));
    }
}

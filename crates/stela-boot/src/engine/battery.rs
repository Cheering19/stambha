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
//! is a `POWER battery ...` line, as is one the SMC holds back, a fall to 0
//! without the firmware's critical bit.
use super::{aml::HostAcpi, host, output, parallel};
use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};
use stela_boot_core::options::Battery;
use stela_drivers::platform::{
    power_relay::{self, Attached, Change, Held, Mode, Relay, Source},
    power_snapshot::{BatteryState, PowerSnapshot, Readings},
};

static RELAY: Relay = Relay::new();
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

/// The AML host's timed reading, which a turn asked for, ended: `failure`
/// names what failed, and it ran `took_ms` of AML. A failure or a slow
/// reading lengthens the period (`Relay::polled`); the first of each is
/// said, and a reading that succeeds again.
pub(super) fn polled(failure: Option<&str>, took_ms: u64) {
    let polled = RELAY.polled(failure.is_some(), took_ms);
    if let Some(failure) = failure.filter(|_| polled.failed) {
        output::arguments(format_args!(
            "POWER battery: reading the host's battery failed ({failure}); the next reading is \
             in {} s, longer each failure up to {} s, and this is not said again until one \
             succeeds\n",
            polled.period_ms / 1000,
            power_relay::MAX_REFRESH_MILLIS / 1000
        ));
    }
    if polled.slow {
        output::arguments(format_args!(
            "POWER battery: reading the host's battery took {took_ms} ms on the boot processor; \
             the next is in {} s\n",
            polled.period_ms / 1000
        ));
    }
    if polled.recovered {
        output::arguments(format_args!(
            "POWER battery: the host's battery reads again\n"
        ));
    }
}

/// Give the guest's SMC the battery `choice` names, before the guest starts:
/// the synthetic battery, or the host's latest reading. A host reading
/// without a percentage, a remaining or a full capacity, or at 0, would read
/// as an empty battery, so the SMC keeps the Mac mini's keys, the line says
/// why, and the service tries each new reading ([`serve`]) until macOS has
/// read the SMC's key table or [`power_relay::WAIT_MILLIS`] pass. The boot
/// `withheld` names gets none (`System::battery_withheld`): an install or
/// restore, a staged update, a recoveryOS or the Metal plugin repair.
pub(super) fn attach(
    devices: &parallel::Board,
    choice: Option<Battery>,
    acpi: Option<&'static HostAcpi>,
    withheld: Option<&'static str>,
) {
    if let (Some(_), Some(boot)) = (choice, withheld) {
        output::arguments(format_args!(
            "POWER battery: not given to {boot}; its SMC answers as the Mac mini's\n"
        ));
        return;
    }
    match choice {
        None => {}
        Some(Battery::Test { percent, state }) => {
            let snapshot = PowerSnapshot::synthetic(percent, state);
            match devices.with(|devices| devices.add_battery(&snapshot)) {
                Ok(()) => attached(&snapshot, Source::Test),
                Err(error) => output::arguments(format_args!(
                    "POWER battery refused: {error}; the SMC answers as the Mac mini's\n"
                )),
            }
        }
        Some(Battery::Host) => {
            let Some(acpi) = acpi else {
                output::arguments(format_args!(
                    "POWER battery: --battery, but the host's ACPI namespace did not load; \
                     the SMC answers as the Mac mini's\n"
                ));
                return;
            };
            ACPI.store(core::ptr::from_ref(acpi).cast_mut(), Ordering::Release);
            let reading = RELAY.take();
            if let Some(missing) = missing(reading.as_ref()) {
                output::arguments(format_args!(
                    "POWER battery: waiting up to {} s for a reading of the host's battery with \
                     a percentage, a remaining and a full capacity and a charge above 0; this one \
                     has no {missing}; until then the SMC answers as the Mac mini's\n",
                    power_relay::WAIT_MILLIS / 1000
                ));
                RELAY.wait(now_ms());
                return;
            }
            attach_host(devices, reading.as_ref().expect("a complete reading"));
        }
    }
}

/// What a host reading lacks before the SMC may take it, if anything.
fn missing(reading: Option<&PowerSnapshot>) -> Option<&'static str> {
    match reading.and_then(|reading| reading.battery) {
        None => Some("battery"),
        Some(battery) => battery.missing(),
    }
}

fn attach_host(devices: &parallel::Board, reading: &PowerSnapshot) {
    match devices.with(|devices| devices.add_battery(reading)) {
        Ok(()) => {
            // SAFETY: the AML host is leaked for the whole program.
            let follows = unsafe { ACPI.load(Ordering::Acquire).as_ref() }
                .is_some_and(HostAcpi::hardware_active);
            attached(reading, Source::Host { follows });
            RELAY.attached();
        }
        Err(error) => {
            RELAY.give_up();
            output::arguments(format_args!(
                "POWER battery refused: {error}; the SMC stays the Mac mini's for this boot\n"
            ));
        }
    }
}

fn attached(snapshot: &PowerSnapshot, source: Source) {
    output::arguments(format_args!("POWER {}\n", Attached { snapshot, source }));
    RELAY.changed(snapshot);
}

/// One service turn, as the relay says: while the guest waits for a
/// complete reading, try each new one; once its SMC has the battery, pass
/// each new reading on. Nothing without the host's battery.
pub(super) fn serve(devices: &parallel::Board) {
    let mode = RELAY.mode();
    if mode == Mode::Off {
        return;
    }
    let now = now_ms();
    let turn = RELAY.turn(now);
    if turn.expired {
        output::arguments(format_args!(
            "POWER battery: no complete reading of the host's battery within {} s; \
             the SMC stays the Mac mini's for this boot\n",
            power_relay::WAIT_MILLIS / 1000
        ));
        return;
    }
    if turn.refresh {
        // SAFETY: the AML host is leaked for the whole program.
        if let Some(acpi) = unsafe { ACPI.load(Ordering::Acquire).as_ref() } {
            acpi.refresh_power();
        }
    }
    if mode == Mode::Waiting {
        if let Some(reading) = turn.reading {
            if RELAY.changed(&reading) {
                output::arguments(format_args!("POWER {reading} (not attached)\n"));
            }
            if missing(Some(&reading)).is_none() {
                attach_host(devices, &reading);
            }
        }
        return;
    }
    if turn.keys {
        // Without waiting for a PE that holds the devices: the next drain
        // takes what this one could not.
        let requests = devices
            .try_with(|devices| {
                core::iter::from_fn(|| devices.take_smc_key_request())
                    .collect::<alloc::vec::Vec<_>>()
            })
            .unwrap_or_default();
        for request in requests {
            output::arguments(format_args!("POWER battery: macOS {request}\n"));
        }
    }
    if turn.reading.is_none() && !turn.flush {
        return;
    }
    let next = devices.serve(|devices| match &turn.reading {
        Some(reading) => devices
            .power_update(reading, now)
            .map(|next| (next, devices.power_held())),
        None => devices.power_due(now).map(|next| (next, false)),
    });
    let held = match next {
        Ok((next, held)) => {
            RELAY.delivered(next);
            held
        }
        Err(error) => {
            RELAY.delivered(None);
            if !REFUSED.swap(true, Ordering::Relaxed) {
                output::arguments(format_args!("POWER battery update refused: {error}\n"));
            }
            false
        }
    };
    let Some(reading) = turn.reading else {
        return;
    };
    if held {
        // Left out of the logged summary, so the reading that confirms it
        // is logged as a change.
        output::arguments(format_args!("POWER {}\n", Held(&reading)));
    } else if RELAY.changed(&reading) {
        output::arguments(format_args!("POWER {}\n", Change(&reading)));
    }
}

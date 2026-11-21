// SPDX-License-Identifier: GPL-2.0-or-later
// ACPI power button notifications, following Linux drivers/acpi/button.c.
// Copyright (C) 2001, 2002 Andy Grover and Paul Diefenbaugh
//! The ACPI control-method power button (PNP0C0C): its press is the
//! firmware's `Notify (button, 0x80)`, for which Linux's
//! `acpi_button_notify` reports `KEY_POWER` down and up. A firmware whose
//! FADT sets PWR_BUTTON has no fixed power button event and reports its
//! presses only this way, typically from an embedded controller query.
use super::acpi::{self, Notification, Session};
use alloc::{string::String, vec::Vec};
use core::sync::atomic::{AtomicBool, Ordering};

/// `ACPI_BUTTON_HID_POWER`.
pub const POWER: &str = "PNP0C0C";
/// `ACPI_BUTTON_NOTIFY_STATUS`: the button was pressed.
pub const NOTIFY_STATUS: u32 = 0x80;

/// Whether a power button's notification `value` is a press, as
/// `acpi_button_notify` reports one: STATUS alone. WAKE (0x02) only marks
/// a wakeup, and any other value is unsupported.
pub fn press(value: u32) -> bool {
    value == NOTIFY_STATUS
}

/// A control-method power button, whose presses set the flag
/// [`attach_power_buttons`] was given while this lives.
pub struct PowerButton<'a> {
    pub path: String,
    _notification: Notification<'a>,
}

/// What [`attach_power_buttons`] found.
pub struct PowerButtons<'a> {
    pub buttons: Vec<PowerButton<'a>>,
    /// Buttons whose handler ACPICA refused, by path.
    pub refused: Vec<(String, acpi::Error)>,
}

/// On every present PNP0C0C device, a notify handler that sets `pressed`
/// at each press, as Linux's `acpi_button_add_event_handler` installs
/// `acpi_button_notify` on a control-method button. The handler takes the
/// device notifications, where STATUS is; Linux's takes the system ones
/// too, of which it only marks WAKE as a wakeup.
pub fn attach_power_buttons<'a>(
    session: &'a Session,
    pressed: &'static AtomicBool,
) -> Result<PowerButtons<'a>, acpi::Error> {
    let mut found = PowerButtons {
        buttons: Vec::new(),
        refused: Vec::new(),
    };
    for handle in session.devices(POWER)? {
        let path = handle.path().unwrap_or_else(|_| String::from(POWER));
        let notified = handle.notify(move |value| {
            if press(value) {
                pressed.store(true, Ordering::Release);
            }
        });
        match notified {
            Ok(notification) => found.buttons.push(PowerButton {
                path,
                _notification: notification,
            }),
            Err(error) => found.refused.push((path, error)),
        }
    }
    Ok(found)
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::platform::acpi::{
        Argument, Mode,
        test_support::{self, TEST_SERIAL, aml},
    };

    unsafe extern "C" {
        fn acpi_os_wait_events_complete();
    }

    /// Run the notifications the firmware queued.
    fn drain() {
        // SAFETY: serialized test process context outside an AML frame.
        unsafe { acpi_os_wait_events_complete() };
    }

    /// A present button, an absent one and a lid, each able to notify.
    fn namespace() -> Vec<u8> {
        let fire = |path| aml::method("FIRE", 1, &aml::notify(path, &[aml::ARG0]));
        aml::scope(
            "\\_SB_",
            &[
                aml::device(
                    "PWRB",
                    &[aml::named("_HID", &aml::string(POWER)), fire("PWRB")].concat(),
                ),
                aml::device(
                    "PWR2",
                    &[
                        aml::named("_HID", &aml::string(POWER)),
                        aml::named("_STA", &aml::integer(0)),
                    ]
                    .concat(),
                ),
                aml::device(
                    "LID0",
                    &[aml::named("_HID", &aml::string("PNP0C0D")), fire("LID0")].concat(),
                ),
            ]
            .concat(),
        )
    }

    #[test]
    fn only_a_status_notification_is_a_press() {
        assert!(press(0x80));
        for value in [0x00, 0x02, 0x81, 0xff] {
            assert!(!press(value), "{value:#x}");
        }
    }

    #[test]
    fn a_buttons_status_notification_sets_the_flag() {
        static PRESSED: AtomicBool = AtomicBool::new(false);
        let _serial = TEST_SERIAL.lock().unwrap();
        // SAFETY: the fixture owns every ACPICA service; serialized singleton.
        let session = unsafe {
            Session::start(
                test_support::dsdt_fixture(&namespace()),
                Mode::InterpreterOnly,
            )
        }
        .unwrap();
        let found = attach_power_buttons(&session, &PRESSED).unwrap();
        let paths: Vec<_> = found.buttons.iter().map(|button| &button.path).collect();
        assert_eq!(paths, ["\\_SB_.PWRB"]);
        assert!(found.refused.is_empty());
        let button = session.handle("\\_SB.PWRB").unwrap();
        for value in [0x81, 0x02] {
            button
                .evaluate(Some("FIRE"), &[Argument::Integer(value)])
                .unwrap();
        }
        session
            .handle("\\_SB.LID0")
            .unwrap()
            .evaluate(Some("FIRE"), &[Argument::Integer(0x80)])
            .unwrap();
        drain();
        assert!(!PRESSED.load(Ordering::Acquire));
        button
            .evaluate(Some("FIRE"), &[Argument::Integer(0x80)])
            .unwrap();
        drain();
        assert!(PRESSED.swap(false, Ordering::AcqRel));
        drop(found);
        session.shutdown().unwrap();
    }

    /// `STELA_TEST_DSDT` names the ASUS TUF A16 FA608PP's DSDT; skipped
    /// without one. Its EC query `_QE7` notifies `\_SB.PWRB`, its one
    /// power button, which presses it.
    #[test]
    fn the_fa608pps_ec_query_presses_its_power_button() {
        static PRESSED: AtomicBool = AtomicBool::new(false);
        let Some(services) = test_support::named_dsdt_fixture() else {
            return;
        };
        let _serial = TEST_SERIAL.lock().unwrap();
        // SAFETY: the fixture owns every ACPICA service; serialized singleton.
        let session = unsafe { Session::start(services, Mode::InterpreterOnly) }.unwrap();
        let found = attach_power_buttons(&session, &PRESSED).unwrap();
        let paths: Vec<_> = found.buttons.iter().map(|button| &button.path).collect();
        assert_eq!(paths, ["\\_SB_.PWRB"]);
        assert!(found.refused.is_empty());
        session
            .handle("\\_SB.PCI0.SBRG.EC0._QE7")
            .unwrap()
            .invoke(None, &[])
            .unwrap();
        drain();
        assert!(PRESSED.swap(false, Ordering::AcqRel));
        drop(found);
        session.shutdown().unwrap();
    }
}

// SPDX-License-Identifier: GPL-2.0-or-later
// ACPI lid state, following Linux drivers/acpi/button.c.
// Copyright (C) 2001, 2002 Andy Grover and Paul Diefenbaugh
//! The native ACPI lid switch (PNP0C0D): whether the lid is open, from
//! `_LID` as Linux's `acpi_lid_evaluate_state` reads it.
use super::acpi::{self, Handle, Value};

/// ACPICA's AE_TYPE: a method returned something other than an integer.
const TYPE: acpi::Error = acpi::Error(0x0008);

/// The lid; every refresh evaluates `_LID` anew.
pub struct Lid<'a> {
    handle: Handle<'a>,
    open: Option<bool>,
}

impl<'a> Lid<'a> {
    /// Use a handle from the session's PNP0C0D device discovery that has
    /// `_LID`.
    pub fn attach(handle: Handle<'a>) -> Result<Self, acpi::Error> {
        handle.child("_LID")?;
        Ok(Self { handle, open: None })
    }

    /// Whether the lid is open, from `_LID`.
    pub fn refresh(&mut self) -> Result<bool, acpi::Error> {
        let open = state(self.handle.evaluate(Some("_LID"), &[])?)?;
        self.open = Some(open);
        Ok(open)
    }

    /// The last [`Self::refresh`]'s answer.
    pub fn open(&self) -> Option<bool> {
        self.open
    }

    /// [`Self::refresh`], and what it read where that differs from the
    /// state known before: Linux's `acpi_lid_notify_state` reports the lid
    /// switch at every notification, and its input layer passes a change
    /// on alone. The first reading is one, as Linux reports the state the
    /// lid starts in (`lid_init_state=method`, its default).
    pub fn change(&mut self) -> Result<Option<bool>, acpi::Error> {
        let before = self.open;
        let open = self.refresh()?;
        Ok((before != Some(open)).then_some(open))
    }

    /// The lid device's absolute path.
    pub fn path(&self) -> Result<alloc::string::String, acpi::Error> {
        self.handle.path()
    }
}

/// `_LID`'s answer: nonzero is open.
fn state(value: Option<Value<'_>>) -> Result<bool, acpi::Error> {
    match value {
        Some(Value::Integer(state)) => Ok(state != 0),
        _ => Err(TYPE),
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::platform::acpi::{
        Argument, Mode, Session,
        test_support::{self, TEST_SERIAL, aml},
    };
    use alloc::vec::Vec;
    use core::sync::atomic::{AtomicU32, Ordering};

    unsafe extern "C" {
        fn acpi_os_wait_events_complete();
    }

    #[test]
    fn nonzero_lid_state_is_open_and_others_are_malformed() {
        assert_eq!(state(Some(Value::Integer(1))), Ok(true));
        assert_eq!(state(Some(Value::Integer(0))), Ok(false));
        assert_eq!(state(Some(Value::String(alloc::vec![b'1']))), Err(TYPE));
        assert_eq!(state(None), Err(TYPE));
    }

    /// A lid whose `_LID` answers what `SETL` stored, open at first.
    fn namespace() -> Vec<u8> {
        aml::scope(
            "\\_SB_",
            &aml::device(
                "LID0",
                &[
                    aml::named("_HID", &aml::string("PNP0C0D")),
                    aml::named("LSTA", &aml::integer(1)),
                    aml::method("_LID", 0, &aml::ret(&aml::name("LSTA"))),
                    aml::method("SETL", 1, &aml::store(&[aml::ARG0], &aml::name("LSTA"))),
                ]
                .concat(),
            ),
        )
    }

    #[test]
    fn a_change_is_reported_once_and_the_first_reading_is_one() {
        let _serial = TEST_SERIAL.lock().unwrap();
        // SAFETY: the fixture owns every ACPICA service; serialized singleton.
        let session = unsafe {
            Session::start(
                test_support::dsdt_fixture(&namespace()),
                Mode::InterpreterOnly,
            )
        }
        .unwrap();
        let handle = session.devices("PNP0C0D").unwrap().remove(0);
        let mut lid = Lid::attach(handle).unwrap();
        assert_eq!(lid.path().unwrap(), "\\_SB_.LID0");
        let set = |open: u64| {
            handle
                .invoke(Some("SETL"), &[Argument::Integer(open)])
                .unwrap()
        };
        assert_eq!(lid.change(), Ok(Some(true)));
        assert_eq!(lid.change(), Ok(None));
        set(0);
        assert_eq!(lid.change(), Ok(Some(false)));
        assert_eq!(lid.change(), Ok(None));
        assert_eq!(lid.open(), Some(false));
        set(1);
        assert_eq!(lid.change(), Ok(Some(true)));
        session.shutdown().unwrap();
    }

    /// `STELA_TEST_DSDT` names the ASUS TUF A16 FA608PP's DSDT; skipped
    /// without one. Its EC query `_Q2F` notifies `\_SB.LID0` with 0x80,
    /// whose `_LID` reads open while the EC reports nothing (ECOK clear).
    #[test]
    fn the_fa608pps_ec_query_notifies_its_lid() {
        static NOTIFIED: AtomicU32 = AtomicU32::new(0);
        let Some(services) = test_support::named_dsdt_fixture() else {
            return;
        };
        let _serial = TEST_SERIAL.lock().unwrap();
        // SAFETY: the fixture owns every ACPICA service; serialized singleton.
        let session = unsafe { Session::start(services, Mode::InterpreterOnly) }.unwrap();
        let handles = session.devices("PNP0C0D").unwrap();
        assert_eq!(handles.len(), 1);
        let mut lid = Lid::attach(handles[0]).unwrap();
        assert_eq!(lid.path().unwrap(), "\\_SB_.LID0");
        let notification = handles[0]
            .notify(|value| NOTIFIED.store(value, Ordering::Release))
            .unwrap();
        session
            .handle("\\_SB.PCI0.SBRG.EC0._Q2F")
            .unwrap()
            .invoke(None, &[])
            .unwrap();
        // SAFETY: serialized test process context outside an AML frame.
        unsafe { acpi_os_wait_events_complete() };
        assert_eq!(NOTIFIED.load(Ordering::Acquire), 0x80);
        assert_eq!(lid.change(), Ok(Some(true)));
        drop(notification);
        session.shutdown().unwrap();
    }
}

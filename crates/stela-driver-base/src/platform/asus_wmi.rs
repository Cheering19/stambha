// SPDX-License-Identifier: GPL-2.0-or-later
// ASUS WMI hotkey events and keyboard backlight, following Linux
// drivers/platform/x86/{asus-wmi,asus-nb-wmi}.c through the ACPI-WMI
// mapping of drivers/platform/wmi/core.c.
// Copyright(C) 2010 Intel Corporation.
// Copyright(C) 2010-2011 Corentin Chary <corentin.chary@gmail.com>
// Copyright (C) 2007-2008 Carlos Corbacho <carlos@strangeworlds.co.uk>
//! An ASUS laptop's WMI device (PNP0C14, `_UID` "ATK" or "ASUSWMI"): its
//! management methods, which asus-wmi calls through the `WM` method its
//! `_WDG` names for `ASUS_WMI_MGMT_GUID`, and its notebook events
//! (`ASUS_NB_WMI_EVENT_GUID`), whose codes `_WED` answers at each of the
//! device's notifications.
//!
//! [`AsusWmi::init`] is asus-wmi's INIT, after which the firmware reports
//! the Fn hotkeys as events; the keyboard backlight (device 0x00050021)
//! is read with DSTS and set with DEVS to `0x80 | level`, its levels 0 to
//! 3, and its keys (0xC4 up, 0xC5 down, 0xC7 toggle) step it as
//! `asus_wmi_handle_event_code` does.
use super::acpi::{self, Argument, Handle, Session, Value};
use alloc::{string::String, vec::Vec};

/// `ASUS_WMI_MGMT_GUID`, 97845ED0-4E6D-11DE-8A39-0800200C9A66, in `_WDG`'s
/// byte order.
pub const MANAGEMENT: [u8; 16] = [
    0xd0, 0x5e, 0x84, 0x97, 0x6d, 0x4e, 0xde, 0x11, 0x8a, 0x39, 0x08, 0x00, 0x20, 0x0c, 0x9a, 0x66,
];
/// `ASUS_NB_WMI_EVENT_GUID`, 0B3CBB35-E3C2-45ED-91C2-4C5A6D195D1C.
pub const EVENTS: [u8; 16] = [
    0x35, 0xbb, 0x3c, 0x0b, 0xc2, 0xe3, 0xed, 0x45, 0x91, 0xc2, 0x4c, 0x5a, 0x6d, 0x19, 0x5d, 0x1c,
];
/// The WMI device's hardware ID.
pub const WMI: &str = "PNP0C14";
/// `ACPI_WMI_METHOD`, `_STRING` and `_EVENT` block flags.
pub const BLOCK_METHOD: u8 = 1 << 1;
pub const BLOCK_STRING: u8 = 1 << 2;
pub const BLOCK_EVENT: u8 = 1 << 3;
/// One `_WDG` block (`struct guid_block`).
const BLOCK_BYTES: usize = 20;
/// `ASUS_WMI_METHODID_*`.
const INIT: u32 = 0x5449_4e49;
const DSTS: u32 = 0x5354_5344;
const DCTS: u32 = 0x5354_4344;
const DEVS: u32 = 0x5356_4544;
/// `ASUS_WMI_DEVID_KBD_BACKLIGHT`.
const KEYBOARD_BACKLIGHT: u32 = 0x0005_0021;
/// `ASUS_WMI_DSTS_PRESENCE_BIT` and `ASUS_WMI_UNSUPPORTED_METHOD`.
const PRESENCE: u32 = 0x0001_0000;
const UNSUPPORTED: u32 = 0xffff_fffe;
/// `ASUS_EV_MAX_BRIGHTNESS`.
pub const KEYBOARD_MAX: u8 = 3;
/// `NOTIFY_KBD_BRTUP`, `_BRTDWN` and `_BRTTOGGLE`.
pub const KEYBOARD_UP: u32 = 0xc4;
pub const KEYBOARD_DOWN: u32 = 0xc5;
pub const KEYBOARD_TOGGLE: u32 = 0xc7;
/// `WMI_EVENT_MASK`.
const EVENT_MASK: u64 = 0xffff;
/// What `_WED` answers once the firmware's queue is empty: the FA608PP's
/// `GANQ` returns Ones, masked to this (an unknown key to Linux, which
/// reads one event per notification).
pub const NO_EVENT: u32 = 0xffff;

/// One `_WDG` block: its GUID, object ID (or notification and a reserved
/// byte for an event), instance count and flags.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Block {
    pub guid: [u8; 16],
    pub id: [u8; 2],
    pub instances: u8,
    pub flags: u8,
}

/// `_WDG`'s blocks, as Linux's `parse_wdg` reads them; a partial block at
/// the end is ignored.
pub fn blocks(wdg: &[u8]) -> Vec<Block> {
    let (whole, _) = wdg.as_chunks::<BLOCK_BYTES>();
    whole
        .iter()
        .map(|block| Block {
            guid: block[..16].try_into().expect("16 bytes"),
            id: [block[16], block[17]],
            instances: block[18],
            flags: block[19],
        })
        .collect()
}

/// Why an ASUS WMI call gave no answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Acpi(acpi::Error),
    /// The method answered `ASUS_WMI_UNSUPPORTED_METHOD`.
    Unsupported,
    /// `_WED` answered no integer.
    Malformed,
}

impl From<acpi::Error> for Error {
    fn from(error: acpi::Error) -> Self {
        Self::Acpi(error)
    }
}

/// An ASUS WMI device with the management methods.
pub struct AsusWmi<'a> {
    pub path: String,
    handle: Handle<'a>,
    /// `WM` and the management block's object ID.
    method: String,
    /// The notebook events' notification.
    pub notify: u32,
    /// The device-status method: DCTS for `_UID` "ASUSWMI", else DSTS.
    status: u32,
    /// The keyboard backlight's level as last read or set (`kbd_led_wk`);
    /// `None` where the firmware has none.
    keyboard: Option<u8>,
}

impl<'a> AsusWmi<'a> {
    /// The first present WMI device whose `_WDG` has the ASUS management
    /// methods and the notebook events, as asus-nb-wmi binds only where
    /// both GUIDs exist (`asus_wmi_probe`); `None` without one.
    pub fn find(session: &'a Session) -> Result<Option<Self>, acpi::Error> {
        for handle in session.devices(WMI)? {
            let wdg = match handle.evaluate(Some("_WDG"), &[]) {
                Ok(Some(Value::Buffer(bytes))) => bytes,
                _ => continue,
            };
            let blocks = blocks(&wdg);
            let Some(management) = blocks.iter().find(|block| {
                block.guid == MANAGEMENT
                    && block.flags & BLOCK_METHOD != 0
                    && block.flags & BLOCK_STRING == 0
                    && block.instances > 0
            }) else {
                continue;
            };
            let Some(events) = blocks
                .iter()
                .find(|block| block.guid == EVENTS && block.flags & BLOCK_EVENT != 0)
            else {
                continue;
            };
            let notify = u32::from(events.id[0]);
            let uid = match handle.evaluate(Some("_UID"), &[]) {
                Ok(Some(Value::String(uid))) => uid,
                _ => Vec::new(),
            };
            let [first, second] = management.id;
            return Ok(Some(Self {
                path: handle.path()?,
                handle,
                method: alloc::format!("WM{}{}", char::from(first), char::from(second)),
                notify,
                status: if uid == b"ASUSWMI" { DCTS } else { DSTS },
                keyboard: None,
            }));
        }
        Ok(None)
    }

    /// The device, whose notifications its handler takes.
    pub fn handle(&self) -> Handle<'a> {
        self.handle
    }

    /// The management method `WM..` it calls.
    pub fn method(&self) -> &str {
        &self.method
    }

    /// `asus_wmi_evaluate_method3` with `arg0` and `arg1`: instance 0,
    /// `method_id` and a buffer of six 32-bit arguments (`struct
    /// bios_args`, whose size some firmware's fields read up to). An
    /// answer other than an integer reads as 0.
    fn evaluate(&self, method_id: u32, arg0: u32, arg1: u32) -> Result<u32, Error> {
        let mut args = [0; 24];
        args[..4].copy_from_slice(&arg0.to_le_bytes());
        args[4..8].copy_from_slice(&arg1.to_le_bytes());
        let answer = self.handle.evaluate(
            Some(&self.method),
            &[
                Argument::Integer(0),
                Argument::Integer(u64::from(method_id)),
                Argument::Buffer(&args),
            ],
        )?;
        let value = match answer {
            Some(Value::Integer(value)) => value as u32,
            _ => 0,
        };
        if value == UNSUPPORTED {
            return Err(Error::Unsupported);
        }
        Ok(value)
    }

    /// `asus_wmi_platform_init`'s INIT, which "enable[s] hotkeys on some
    /// models": what it answered.
    pub fn init(&mut self) -> Result<u32, Error> {
        self.evaluate(INIT, 0, 0)
    }

    /// `kbd_led_read`: the keyboard backlight's level from its device
    /// status (its low seven bits, an unknown 0x8000 as 0), kept as the
    /// level known; `None` where the firmware has no keyboard backlight
    /// (no answer, all ones or no presence bit).
    pub fn read_keyboard(&mut self) -> Result<Option<u8>, Error> {
        let status = match self.evaluate(self.status, KEYBOARD_BACKLIGHT, 0) {
            Ok(status) => status,
            Err(Error::Unsupported) => return Ok(None),
            Err(error) => return Err(error),
        };
        if status == u32::MAX || status & PRESENCE == 0 {
            self.keyboard = None;
            return Ok(None);
        }
        let bits = match status & 0xffff {
            0x8000 => 0,
            bits => bits,
        };
        self.keyboard = Some((bits & 0x7f) as u8);
        Ok(self.keyboard)
    }

    /// The keyboard backlight's level as last read or set.
    pub fn keyboard(&self) -> Option<u8> {
        self.keyboard
    }

    /// `do_kbd_led_set`: `level` clamped to 0..=3, kept, and set with DEVS
    /// as `0x80 | level`; the level set.
    pub fn set_keyboard(&mut self, level: i32) -> Result<u8, Error> {
        let level = level.clamp(0, i32::from(KEYBOARD_MAX)) as u8;
        self.keyboard = Some(level);
        self.evaluate(DEVS, KEYBOARD_BACKLIGHT, 0x80 | u32::from(level))?;
        Ok(level)
    }

    /// The event code of one notification of the event block: `_WED` of
    /// its notification, masked (`asus_wmi_get_event_code`). Linux reads it
    /// at every notification, as a firmware queue of events needs.
    pub fn event(&self) -> Result<u32, Error> {
        match self
            .handle
            .evaluate(Some("_WED"), &[Argument::Integer(u64::from(self.notify))])?
        {
            Some(Value::Integer(code)) => Ok((code & EVENT_MASK) as u32),
            _ => Err(Error::Malformed),
        }
    }

    /// Every event the firmware's queue holds, read through `_WED`
    /// ([`Self::event`]) until it answers [`NO_EVENT`], at most `bound`. A
    /// read that fails is passed on and the draining goes on: the queue
    /// fills while events wait, and a full one costs each keypress a stall
    /// in the firmware's EC query (the FA608PP's holds 16).
    pub fn drain(&self, bound: usize) -> Vec<Result<u32, Error>> {
        let mut events = Vec::new();
        for _ in 0..bound {
            match self.event() {
                Ok(NO_EVENT) => break,
                read => events.push(read),
            }
        }
        events
    }

    /// `asus_wmi_handle_event_code` for the keyboard backlight's keys: one
    /// level up or down, or a toggle that wraps after the highest, from the
    /// level known; the levels before and after, or `None` for another
    /// code or without a keyboard backlight.
    pub fn keyboard_key(&mut self, code: u32) -> Result<Option<(u8, u8)>, Error> {
        let Some(level) = self.keyboard else {
            return Ok(None);
        };
        let next = match code {
            KEYBOARD_UP => i32::from(level) + 1,
            KEYBOARD_DOWN => i32::from(level) - 1,
            KEYBOARD_TOGGLE if level >= KEYBOARD_MAX => 0,
            KEYBOARD_TOGGLE => i32::from(level) + 1,
            _ => return Ok(None),
        };
        Ok(Some((level, self.set_keyboard(next)?)))
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::platform::acpi::{
        Mode,
        test_support::{self, TEST_SERIAL, aml},
    };
    use core::sync::atomic::{AtomicU32, Ordering};

    unsafe extern "C" {
        fn acpi_os_wait_events_complete();
    }

    /// `CreateDWordField (source, index, name)`.
    fn dword_field(source: u8, index: u64, name: &str) -> Vec<u8> {
        [&[0x8a, source][..], &aml::integer(index), &aml::name(name)].concat()
    }

    /// `Or (left, right)` and `And (left, right)`, with no target.
    fn or(left: &[u8], right: &[u8]) -> Vec<u8> {
        [&[0x7d], left, right, &[0]].concat()
    }
    fn and(left: &[u8], right: &[u8]) -> Vec<u8> {
        [&[0x7b], left, right, &[0]].concat()
    }

    /// `_WDG` with the management methods as object "NB" and the
    /// notebook events on notification 0xFF.
    fn wdg() -> Vec<u8> {
        [
            &MANAGEMENT[..],
            b"NB",
            &[1, BLOCK_METHOD],
            &EVENTS,
            &[0xff, 0, 1, BLOCK_EVENT],
        ]
        .concat()
    }

    /// An ATK device as an ASUS notebook's, whose WMNB reads five of the
    /// buffer's arguments, its keyboard backlight at level 2; and a WMI
    /// device of another vendor.
    fn namespace() -> Vec<u8> {
        let keyboard = aml::integer(u64::from(KEYBOARD_BACKLIGHT));
        let unsupported = aml::ret(&aml::integer(u64::from(UNSUPPORTED)));
        let wmnb = [
            dword_field(aml::ARG2, 0, "IIA0"),
            dword_field(aml::ARG2, 4, "IIA1"),
            dword_field(aml::ARG2, 0x10, "IIA4"),
            aml::if_equal(
                &[aml::ARG1],
                &aml::integer(u64::from(INIT)),
                &[
                    aml::store(&aml::integer(1), &aml::name("ATKP")),
                    aml::ret(&aml::integer(1)),
                ]
                .concat(),
            ),
            aml::if_equal(
                &[aml::ARG1],
                &aml::integer(u64::from(DSTS)),
                &[
                    aml::if_equal(
                        &aml::name("IIA0"),
                        &keyboard,
                        &aml::ret(&or(&aml::name("KBLL"), &aml::integer(0x5_0000))),
                    ),
                    unsupported.clone(),
                ]
                .concat(),
            ),
            aml::if_equal(
                &[aml::ARG1],
                &aml::integer(u64::from(DEVS)),
                &[
                    aml::if_equal(
                        &aml::name("IIA0"),
                        &keyboard,
                        &[
                            aml::store(
                                &and(&aml::name("IIA1"), &aml::integer(0x83)),
                                &aml::name("KBLL"),
                            ),
                            aml::ret(&aml::integer(1)),
                        ]
                        .concat(),
                    ),
                    unsupported.clone(),
                ]
                .concat(),
            ),
            unsupported,
        ]
        .concat();
        let wed = [
            aml::if_equal(
                &[aml::ARG0],
                &aml::integer(0xff),
                &aml::ret(&aml::name("EVNT")),
            ),
            aml::ret(&aml::integer(1)),
        ]
        .concat();
        aml::scope(
            "\\_SB_",
            &[
                aml::device(
                    "ATKD",
                    &[
                        aml::named("_HID", &aml::string(WMI)),
                        aml::named("_UID", &aml::string("ATK")),
                        aml::named("_WDG", &aml::buffer(&wdg())),
                        aml::named("ATKP", &aml::integer(0)),
                        aml::named("KBLL", &aml::integer(2)),
                        aml::named("EVNT", &aml::integer(u64::from(KEYBOARD_UP))),
                        aml::method("WMNB", 3, &wmnb),
                        aml::method("_WED", 1, &wed),
                        aml::method("FIRE", 1, &aml::notify("ATKD", &[aml::ARG0])),
                    ]
                    .concat(),
                ),
                aml::device(
                    "WMI2",
                    &[
                        aml::named("_HID", &aml::string(WMI)),
                        aml::named("_WDG", &aml::buffer(&[0x11; BLOCK_BYTES])),
                    ]
                    .concat(),
                ),
            ]
            .concat(),
        )
    }

    fn integer(session: &Session, path: &str) -> u64 {
        match session.handle(path).unwrap().evaluate(None, &[]).unwrap() {
            Some(Value::Integer(value)) => value,
            other => panic!("{path}: {other:?}"),
        }
    }

    #[test]
    fn wdg_blocks_are_read_as_linux_reads_them() {
        let read = blocks(&[wdg(), alloc::vec![0; 7]].concat());
        assert_eq!(read.len(), 2);
        assert_eq!(read[0].guid, MANAGEMENT);
        assert_eq!(
            (read[0].id, read[0].instances, read[0].flags),
            (*b"NB", 1, 2)
        );
        assert_eq!((read[1].id[0], read[1].flags), (0xff, BLOCK_EVENT));
    }

    #[test]
    fn init_reads_and_steps_the_keyboard_backlight_and_takes_events() {
        static NOTIFIED: AtomicU32 = AtomicU32::new(0);
        let _serial = TEST_SERIAL.lock().unwrap();
        // SAFETY: the fixture owns every ACPICA service; serialized singleton.
        let session = unsafe {
            Session::start(
                test_support::dsdt_fixture(&namespace()),
                Mode::InterpreterOnly,
            )
        }
        .unwrap();
        let mut wmi = AsusWmi::find(&session).unwrap().unwrap();
        assert_eq!(
            (wmi.path.as_str(), wmi.method(), wmi.notify),
            ("\\_SB_.ATKD", "WMNB", 0xff)
        );
        assert_eq!(wmi.init(), Ok(1));
        assert_eq!(integer(&session, "\\_SB.ATKD.ATKP"), 1);
        assert_eq!(wmi.read_keyboard(), Ok(Some(2)));
        // Up past the highest, down past the lowest: clamped, set each time.
        assert_eq!(wmi.keyboard_key(KEYBOARD_UP), Ok(Some((2, 3))));
        assert_eq!(integer(&session, "\\_SB.ATKD.KBLL"), 0x83);
        assert_eq!(wmi.keyboard_key(KEYBOARD_UP), Ok(Some((3, 3))));
        for (from, to) in [(3, 2), (2, 1), (1, 0), (0, 0)] {
            assert_eq!(wmi.keyboard_key(KEYBOARD_DOWN), Ok(Some((from, to))));
        }
        assert_eq!(integer(&session, "\\_SB.ATKD.KBLL"), 0x80);
        assert_eq!(wmi.keyboard_key(KEYBOARD_TOGGLE), Ok(Some((0, 1))));
        assert_eq!(wmi.set_keyboard(3), Ok(3));
        assert_eq!(wmi.keyboard_key(KEYBOARD_TOGGLE), Ok(Some((3, 0))));
        assert_eq!(wmi.read_keyboard(), Ok(Some(0)));
        assert_eq!(wmi.keyboard_key(0x6b), Ok(None));
        // A notification of the event block names its code through _WED.
        let notification = wmi
            .handle()
            .notify(|value| {
                NOTIFIED.fetch_add(value, Ordering::AcqRel);
            })
            .unwrap();
        session
            .handle("\\_SB.ATKD")
            .unwrap()
            .invoke(Some("FIRE"), &[Argument::Integer(0xff)])
            .unwrap();
        // SAFETY: serialized test process context outside an AML frame.
        unsafe { acpi_os_wait_events_complete() };
        assert_eq!(NOTIFIED.load(Ordering::Acquire), 0xff);
        assert_eq!(wmi.event(), Ok(KEYBOARD_UP));
        // A bound ends a queue that never empties.
        assert_eq!(wmi.drain(3), [Ok(KEYBOARD_UP); 3]);
        drop(notification);
        session.shutdown().unwrap();
    }

    #[test]
    fn a_wmi_device_without_the_asus_methods_is_not_taken() {
        let _serial = TEST_SERIAL.lock().unwrap();
        let body = aml::scope(
            "\\_SB_",
            &aml::device(
                "WMI2",
                &[
                    aml::named("_HID", &aml::string(WMI)),
                    aml::named("_WDG", &aml::buffer(&[0x11; BLOCK_BYTES])),
                ]
                .concat(),
            ),
        );
        // SAFETY: the fixture owns every ACPICA service; serialized singleton.
        let session =
            unsafe { Session::start(test_support::dsdt_fixture(&body), Mode::InterpreterOnly) }
                .unwrap();
        assert!(AsusWmi::find(&session).unwrap().is_none());
        session.shutdown().unwrap();
    }

    /// `STELA_TEST_DSDT` names the ASUS TUF A16 FA608PP's DSDT; skipped
    /// without one. Its ATK device's WMNB takes INIT, reads the keyboard
    /// backlight from EC RAM (zero here) and sets it, and an event its
    /// firmware queues (`IANE`, which `_Q76`/`_Q77` call for Fn+F2/F3 once
    /// INIT set ATKP) comes back from `_WED` at its notification.
    #[test]
    fn the_fa608pps_atk_device() {
        static NOTIFIED: AtomicU32 = AtomicU32::new(0);
        let Some(services) = test_support::named_dsdt_fixture() else {
            return;
        };
        let _serial = TEST_SERIAL.lock().unwrap();
        // SAFETY: the fixture owns every ACPICA service; serialized singleton.
        let session = unsafe { Session::start(services, Mode::InterpreterOnly) }.unwrap();
        let mut wmi = AsusWmi::find(&session).unwrap().unwrap();
        assert_eq!(
            (wmi.path.as_str(), wmi.method(), wmi.notify),
            ("\\_SB_.ATKD", "WMNB", 0xff)
        );
        assert_eq!(integer(&session, "\\_SB.ATKP"), 0);
        assert_eq!(wmi.init(), Ok(1));
        assert_eq!(integer(&session, "\\_SB.ATKP"), 1);
        assert_eq!(wmi.read_keyboard(), Ok(Some(0)));
        assert_eq!(wmi.keyboard_key(KEYBOARD_UP), Ok(Some((0, 1))));
        let notification = wmi
            .handle()
            .notify(|value| {
                if value == 0xff {
                    NOTIFIED.fetch_add(1, Ordering::AcqRel);
                }
            })
            .unwrap();
        for code in [KEYBOARD_DOWN, KEYBOARD_UP] {
            session
                .handle("\\_SB.ATKD")
                .unwrap()
                .invoke(Some("IANE"), &[Argument::Integer(u64::from(code))])
                .unwrap();
        }
        // SAFETY: serialized test process context outside an AML frame.
        unsafe { acpi_os_wait_events_complete() };
        assert_eq!(NOTIFIED.load(Ordering::Acquire), 2);
        // Both, in order, then GANQ's Ones: the queue is empty.
        assert_eq!(wmi.drain(32), [Ok(KEYBOARD_DOWN), Ok(KEYBOARD_UP)]);
        assert!(wmi.drain(32).is_empty());
        assert_eq!(wmi.keyboard_key(KEYBOARD_DOWN), Ok(Some((1, 0))));
        drop(notification);
        session.shutdown().unwrap();
    }

    /// A WMI device with the management methods and no notebook events, as
    /// asus-nb-wmi does not bind to.
    #[test]
    fn a_wmi_device_without_the_notebook_events_is_not_taken() {
        let _serial = TEST_SERIAL.lock().unwrap();
        let wdg = [&MANAGEMENT[..], b"NB", &[1, BLOCK_METHOD]].concat();
        let body = aml::scope(
            "\\_SB_",
            &aml::device(
                "ATKD",
                &[
                    aml::named("_HID", &aml::string(WMI)),
                    aml::named("_WDG", &aml::buffer(&wdg)),
                    aml::method("WMNB", 3, &aml::ret(&aml::integer(1))),
                ]
                .concat(),
            ),
        );
        // SAFETY: the fixture owns every ACPICA service; serialized singleton.
        let session =
            unsafe { Session::start(test_support::dsdt_fixture(&body), Mode::InterpreterOnly) }
                .unwrap();
        assert!(AsusWmi::find(&session).unwrap().is_none());
        session.shutdown().unwrap();
    }

    /// A drain reads `_WED` until it answers the empty queue's Ones, past a
    /// read that fails, and within its bound.
    #[test]
    fn a_drain_reads_until_the_queue_is_empty_past_a_bad_read() {
        let _serial = TEST_SERIAL.lock().unwrap();
        // _WED: a buffer at its first read, 0xC4 at its second, then Ones.
        let increment = [&[0x75][..], &aml::name("CNT_")].concat();
        let wed = [
            increment,
            aml::if_equal(
                &aml::name("CNT_"),
                &aml::integer(1),
                &aml::ret(&aml::buffer(&[0])),
            ),
            aml::if_equal(
                &aml::name("CNT_"),
                &aml::integer(2),
                &aml::ret(&aml::integer(u64::from(KEYBOARD_UP))),
            ),
            aml::ret(&[0xff]),
        ]
        .concat();
        let body = aml::scope(
            "\\_SB_",
            &aml::device(
                "ATKD",
                &[
                    aml::named("_HID", &aml::string(WMI)),
                    aml::named("_WDG", &aml::buffer(&wdg())),
                    aml::named("CNT_", &aml::integer(0)),
                    aml::method("WMNB", 3, &aml::ret(&aml::integer(1))),
                    aml::method("_WED", 1, &wed),
                ]
                .concat(),
            ),
        );
        // SAFETY: the fixture owns every ACPICA service; serialized singleton.
        let session =
            unsafe { Session::start(test_support::dsdt_fixture(&body), Mode::InterpreterOnly) }
                .unwrap();
        let wmi = AsusWmi::find(&session).unwrap().unwrap();
        assert_eq!(wmi.drain(8), [Err(Error::Malformed), Ok(KEYBOARD_UP)]);
        assert_eq!(integer(&session, "\\_SB.ATKD.CNT_"), 3);
        session.shutdown().unwrap();
    }
}

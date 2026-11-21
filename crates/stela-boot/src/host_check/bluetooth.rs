//! `--bluetooth-check`: the first USB Bluetooth controller a Realtek setup
//! takes, as the host check's storage probe brought it up, then a passive LE
//! scan through it (`stela_drivers::usb::bluetooth::realtek::scan`), each
//! step one `CHECK` line (`stela_boot_core::bluetooth_check`).
//!
//! Before ExitBootServices ([`prepare`]) the USB stick's host firmware is
//! verified as a storage boot verifies it (`host_firmware::identity_catalog`).
//! The probe then gives USB Bluetooth requests alone the bundle files their
//! recipes pin (`BundleProvider`, through `bluetooth_check::BluetoothOnly`);
//! every other device starts as a plain host check starts it. The probe
//! starts each xHCI once, the stick's and the controller's alike, and this
//! check resets no xHCI, hub or port: it sends HCI commands through the
//! controller's own transport, then HCI Reset, and closes that transport. It
//! reads nothing from the stick; its lines reach the log the probe opened
//! and, with `--keep-lines`, the firmware's variables.
use super::{Machine, Verdict};
use alloc::{boxed::Box, format, string::String, sync::Arc, vec::Vec};
use stela_boot_core::bluetooth_check::{self, BluetoothOnly, Controller};
use stela_drivers::{
    Handoff,
    firmware::{Catalog, Provider},
    usb::bluetooth::{UsbHci, realtek},
};

/// How long the scan listens.
const SCAN_US: u64 = 8_000_000;

/// What [`prepare`] found before ExitBootServices.
pub(super) enum Plan {
    /// No `--bluetooth-check`.
    Off,
    /// The check runs, with the verified catalog, or why there is none.
    On(core::result::Result<Arc<Catalog>, String>),
}

/// Before ExitBootServices: the verified catalog of the USB stick's host
/// firmware. A check without one still reports what the controller read.
pub(super) fn prepare(resources: &crate::host_firmware::Resources) -> Plan {
    // SAFETY: every immutable host-firmware page stays reserved LOADER_DATA,
    // and the host check's tables identity-map all such RAM for the rest of
    // the boot.
    Plan::On(
        match unsafe { crate::host_firmware::identity_catalog(resources) } {
            Ok(Some(catalog)) => Ok(catalog),
            Ok(None) => Err(String::from("the USB stick has no host firmware")),
            Err(error) => Err(format!("{error}")),
        },
    )
}

/// The firmware the probe gives its drivers: for USB Bluetooth requests the
/// verified catalog as a normal boot's drivers ask it (a request with no
/// exact binding gets the bundle file its recipe pins), for any other the
/// handoff's own, as a plain host check gives it. All of it lives for the
/// rest of the boot, as attached devices keep borrowing their images.
pub(super) fn firmware(plan: &Plan, handoff: &Handoff) -> Option<&'static dyn Provider> {
    let Plan::On(Ok(catalog)) = plan else {
        return None;
    };
    let held: &'static Arc<Catalog> = Box::leak(Box::new(Arc::clone(catalog)));
    let catalog: &'static Catalog = held;
    let bundle: &'static dyn Provider = Box::leak(Box::new(
        stela_loader::host_firmware::bundle::BundleProvider::new(catalog),
    ));
    let rest: &'static dyn Provider = stela_drivers::probe::handoff_provider(handoff);
    Some(Box::leak(Box::new(BluetoothOnly {
        bluetooth: bundle,
        rest,
    })))
}

fn controller(hci: &UsbHci) -> Controller<'_> {
    let device = hci.binding().device();
    let d = &device.descriptor;
    Controller {
        location: &device.location,
        vendor: u16::from_le_bytes([d[8], d[9]]),
        product: u16::from_le_bytes([d[10], d[11]]),
        revision: u16::from_le_bytes([d[12], d[13]]),
        initialization: hci.initialization(),
    }
}

/// After the storage, GPT and log checks: the lines for the transports the
/// probe kept, with the scan of the chosen one where it came up.
#[expect(
    clippy::result_large_err,
    reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
)]
pub(super) fn check(machine: &mut Machine) {
    let catalog = match core::mem::replace(&mut machine.bluetooth, Plan::Off) {
        Plan::Off => {
            machine.record(
                "bluetooth",
                Verdict::Skip,
                String::from("no --bluetooth-check"),
            );
            return;
        }
        Plan::On(catalog) => catalog,
    };
    if let Err(why) = &catalog {
        crate::output::arguments(format_args!(
            "HOST CHECK bluetooth: no verified host firmware ({why}); the controller is \
             identified without it\n"
        ));
    }
    let mut transports = core::mem::take(&mut machine.usb_bluetooth);
    let chosen = {
        let controllers: Vec<Controller<'_>> = transports.iter().map(controller).collect();
        bluetooth_check::chosen(&controllers)
            .filter(|&index| bluetooth_check::scannable(&controllers[index]))
    };
    let scan = chosen.map(|index| realtek::scan(&mut transports[index], SCAN_US));
    let controllers: Vec<Controller<'_>> = transports.iter().map(controller).collect();
    for line in bluetooth_check::lines(&controllers, scan.as_ref()) {
        let verdict = match line.verdict {
            bluetooth_check::Verdict::Pass => Verdict::Pass,
            bluetooth_check::Verdict::Fail => Verdict::Fail,
            bluetooth_check::Verdict::Skip => Verdict::Skip,
        };
        machine.record(line.name, verdict, line.detail);
    }
    drop(controllers);
    // Kept, as the probe kept them: a closed one holds its DMA for the boot.
    machine.usb_bluetooth = transports;
}

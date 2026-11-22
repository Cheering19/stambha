//! `--bluetooth-check`: the xHCI the firmware's USB stack found a Realtek
//! USB Bluetooth controller on, started alone, the controller brought up on
//! it with the USB stick's firmware, then a passive LE scan through it
//! (`stela_drivers::usb::bluetooth::realtek::scan`), each step one `CHECK`
//! line (`stela_boot_core::bluetooth_check`).
//!
//! The host check's probe gives its drivers no native services, so it starts
//! no USB controller (`probe::probe`); and with them it would start every
//! driver, the Windows disk's NVMe among them. This check runs no probe at
//! all. Before ExitBootServices ([`prepare`]) it reads the firmware's PCI
//! census and its USB interfaces, and `bluetooth_target::choose` names the
//! one function it may start, or refuses: when that xHCI also holds a USB
//! disk other than the boot stick, or the firmware's USB devices could not
//! all be read. After it ([`start`]) only that function is touched: its
//! configuration is read, firmware hands it over (`usb::handoff::early`),
//! the xHCI driver starts it with the TSC's minimum delays, and the devices
//! on it are enumerated, the Bluetooth controller's initialization asking
//! the stick's verified firmware for USB Bluetooth requests alone. No other
//! function gets a driver, a BAR access or Bus Master, and no bridge is
//! written: the firmware's own xHCI driver left the port above it open.
//!
//! On this PC that xHCI (0000:05:00.4) also holds the boot stick, so its
//! start resets the stick and enumerates it again. Nothing writes to it: the
//! controller is started without mass storage
//! (`xhci::Started::without_mass_storage`), so the stick's disk, Bulk-Only
//! or UAS, is reported unused with no configuration or SCSI command, and
//! the lines go to the screen and, with `--keep-lines`, the firmware's
//! variables.
use super::{Machine, Verdict};
use alloc::{boxed::Box, format, rc::Rc, string::String, sync::Arc, vec::Vec};
use core::any::Any;
use stela_boot_core::{
    bluetooth_check::{self, BluetoothOnly, Controller},
    bluetooth_target::{self, PciFunction, Target, UsbInterface},
};
use stela_drivers::{
    Handoff,
    bus::{
        dma::RegionAllocator,
        pci::{Address, Function, power::Timer},
    },
    firmware::{Catalog, Provider},
    input::InputDevice,
    probe::Devices,
    usb::{
        bluetooth::{UsbHci, realtek},
        handoff, xhci,
    },
};
use uefi::{
    boot::{self, OpenProtocolAttributes, OpenProtocolParams},
    proto::{loaded_image::LoadedImage, usb::io::UsbIo},
};

/// How long the scan listens.
const SCAN_US: u64 = 8_000_000;

/// The check's state, from [`prepare`] to [`check`].
pub(super) struct Check {
    plan: Plan,
    /// The USB Bluetooth transports the started xHCI brought.
    transports: Vec<UsbHci>,
    /// What the started controller's DMA may still reach, for the boot.
    kept: Vec<Box<dyn Any>>,
}

enum Plan {
    /// No `--bluetooth-check`.
    Off,
    /// Nothing is started, for this reason.
    Refused(String),
    /// Chosen before ExitBootServices, with the verified catalog or why
    /// there is none.
    Chosen {
        target: Target,
        catalog: Result<Arc<Catalog>, String>,
    },
    /// Its xHCI was started.
    Started,
}

impl Check {
    pub(super) fn off() -> Self {
        Self {
            plan: Plan::Off,
            transports: Vec::new(),
            kept: Vec::new(),
        }
    }

    /// Whether the check runs, instead of the probe.
    pub(super) fn on(&self) -> bool {
        !matches!(self.plan, Plan::Off)
    }
}

/// Before ExitBootServices: the one xHCI to start, from the firmware's PCI
/// census and USB interfaces, and the verified catalog of the USB stick's
/// host firmware. One line says what will be started, or the check's line
/// later says why nothing is.
pub(super) fn prepare(resources: &crate::host_firmware::Resources) -> Check {
    let functions = match crate::pci_io::capture() {
        Ok(census) => census
            .functions
            .iter()
            .map(|function| {
                let identity = function.identity();
                PciFunction {
                    address: identity.address,
                    vendor: identity.vendor,
                    device: identity.device,
                    class: identity.class,
                }
            })
            .collect::<Vec<_>>(),
        Err(error) => {
            return Check {
                plan: Plan::Refused(format!(
                    "the firmware's PCI census is unavailable ({error}); nothing is started"
                )),
                ..Check::off()
            };
        }
    };
    let interfaces = usb_interfaces();
    let target = match bluetooth_target::choose(&functions, interfaces.as_deref()) {
        Ok(target) => target,
        Err(why) => {
            return Check {
                plan: Plan::Refused(why),
                ..Check::off()
            };
        }
    };
    crate::output::arguments(format_args!("{}\n", target.line()));
    // SAFETY: every immutable host-firmware page stays reserved LOADER_DATA,
    // and the host check's tables identity-map all such RAM for the rest of
    // the boot.
    let catalog = match unsafe { crate::host_firmware::identity_catalog(resources) } {
        Ok(Some(catalog)) => Ok(catalog),
        Ok(None) => Err(String::from("the USB stick has no host firmware")),
        Err(error) => Err(format!("{error}")),
    };
    Check {
        plan: Plan::Chosen { target, catalog },
        ..Check::off()
    }
}

/// Every USB interface the firmware's USB stack holds: the PCI function its
/// device path leaves the PCI tree at, its path below, its device's IDs, its
/// class and whether the disk Stela started from lies on it. `None` when
/// any of them cannot be read, so the check cannot tell what a controller
/// holds.
fn usb_interfaces() -> Option<Vec<UsbInterface>> {
    let started = boot::open_protocol_exclusive::<LoadedImage>(boot::image_handle())
        .ok()
        .and_then(|image| image.device())
        .and_then(crate::payload::device_path);
    let mut interfaces = Vec::new();
    for handle in boot::find_handles::<UsbIo>().ok()? {
        let location = crate::pci_io::disk_location(handle)?;
        let (controller, path) = location.split_once(' ')?;
        let full = crate::payload::device_path(handle)?;
        // SAFETY: descriptors are only read, while no driver changes them.
        let mut io = unsafe {
            boot::open_protocol::<UsbIo>(
                OpenProtocolParams {
                    handle,
                    agent: boot::image_handle(),
                    controller: None,
                },
                OpenProtocolAttributes::GetProtocol,
            )
        }
        .ok()?;
        let device = io.device_descriptor().ok()?;
        let interface = io.interface_descriptor().ok()?;
        interfaces.push(UsbInterface {
            controller: Address::parse(controller)?,
            path: path.into(),
            vendor: device.id_vendor,
            product: device.id_product,
            class: [
                interface.interface_class,
                interface.interface_subclass,
                interface.interface_protocol,
            ],
            boot: started
                .as_deref()
                .is_some_and(|disk| disk == full || disk.starts_with(&format!("{full}/"))),
        });
    }
    Some(interfaces)
}

/// The firmware the controller's devices get: for USB Bluetooth requests
/// the verified catalog as a normal boot's drivers ask it (a request with
/// no exact binding gets the bundle file its recipe pins), for any other
/// the handoff's own. All of it lives for the rest of the boot, as attached
/// devices keep borrowing their images.
fn firmware(catalog: &Result<Arc<Catalog>, String>, handoff: &Handoff) -> &'static dyn Provider {
    let rest: &'static dyn Provider = stela_drivers::probe::handoff_provider(handoff);
    let Ok(catalog) = catalog else {
        return rest;
    };
    let held: &'static Arc<Catalog> = Box::leak(Box::new(Arc::clone(catalog)));
    let catalog: &'static Catalog = held;
    let bundle: &'static dyn Provider = Box::leak(Box::new(
        stela_loader::host_firmware::bundle::BundleProvider::new(catalog),
    ));
    Box::leak(Box::new(BluetoothOnly {
        bluetooth: bundle,
        rest,
    }))
}

/// After ExitBootServices, in place of the probe: the storage, GPT and log
/// checks skipped, then the chosen xHCI started alone. Gives the keyboards
/// on it.
pub(super) fn start(machine: &mut Machine, handoff: &Handoff) -> Vec<InputDevice> {
    for (name, why) in [
        (
            "storage",
            "--bluetooth-check starts no driver but its one xHCI's",
        ),
        ("gpt", "--bluetooth-check reads no disk"),
        ("log", "--bluetooth-check writes nothing to the stick"),
    ] {
        machine.record(name, Verdict::Skip, String::from(why));
    }
    let Plan::Chosen { target, catalog } = &machine.bluetooth.plan else {
        return Vec::new();
    };
    let target = target.clone();
    if let Err(why) = catalog {
        crate::output::arguments(format_args!(
            "HOST CHECK bluetooth: no verified host firmware ({why}); the controller is \
             identified without it\n"
        ));
    }
    let provider = firmware(catalog, handoff);
    match bring_up(&target, handoff, provider, machine.timing.tsc_hz) {
        Ok(mut devices) => {
            for (location, error) in &devices.usb_failures {
                crate::output::arguments(format_args!(
                    "{}",
                    stela_drivers::usb::xhci::failure_line(location, error)
                ));
            }
            for (location, why) in &devices.usb_unused {
                crate::output::arguments(format_args!("USB UNUSED {location}: {why}\n"));
            }
            for device in &devices.inputs {
                crate::output::arguments(format_args!("INPUT {}\n", device.name()));
            }
            let check = &mut machine.bluetooth;
            check.transports = core::mem::take(&mut devices.bluetooth);
            let inputs = core::mem::take(&mut devices.inputs);
            check.kept.push(Box::new(devices));
            check.plan = Plan::Started;
            inputs
        }
        Err(why) => {
            machine.bluetooth.plan =
                Plan::Refused(format!("pci={} did not start: {why}", target.controller));
            Vec::new()
        }
    }
}

/// The target's xHCI started alone: its configuration read and checked
/// against the census, firmware's ownership handed over, the controller
/// started and the devices on it enumerated. Nothing else is touched.
fn bring_up(
    target: &Target,
    handoff: &Handoff,
    provider: &'static dyn Provider,
    tsc_hz: u64,
) -> Result<Devices, String> {
    let address = target.controller;
    let window = handoff
        .ecam
        .iter()
        .copied()
        .find(|window| {
            window.segment == address.segment
                && (window.start_bus..=window.end_bus).contains(&address.bus)
        })
        .ok_or_else(|| String::from("no ECAM window reaches it"))?;
    // SAFETY: one CPU after ExitBootServices, and no probe runs in this
    // mode, so nothing else reaches this function's configuration; the
    // window is mapped uncached (`storage::discover` maps every ECAM
    // window). It is the one function the target admits.
    let config = unsafe { window.config(address.bus, address.device, address.function) };
    let mut function = Function::new(address, config)
        .map_err(|refusal| format!("it does not answer: {:?}", refusal.reason))?;
    if !target.admits(function.address)
        || (function.vendor, function.device, function.class)
            != (target.vendor, target.device, xhci::CLASS)
    {
        return Err(format!(
            "it is {:04x}:{:04x} of class {:02x?} now, not the xHCI the firmware listed",
            function.vendor, function.device, function.class
        ));
    }
    let minimum =
        crate::native_clock::tsc::Minimum::timer(tsc_hz, crate::cpu::INVARIANT_TSC.present())
            .ok_or_else(|| {
                String::from("the TSC is not invariant, so no device delay can be timed")
            })?;
    let minimum: Rc<dyn Timer> = Rc::new(stela_drivers::budget::Bounded(minimum));
    // SAFETY: the handoff describes the running machine after
    // ExitBootServices, and nothing else drives this controller.
    match unsafe { handoff::early(&mut function, handoff, minimum.as_ref()) } {
        Some(Err(error)) => return Err(format!("the firmware's handoff failed: {error}")),
        Some(Ok(handed)) if handed.forced => crate::output::arguments(format_args!(
            "HOST CHECK bluetooth: pci={address} firmware did not release it in time; \
             ownership was taken with its SMIs off\n"
        )),
        _ => {}
    }
    // SAFETY: one CPU after ExitBootServices; `dma` was reserved for the
    // host check, and no probe allocates from it in this mode.
    let mut dma = unsafe { RegionAllocator::new(handoff.dma.clone()) };
    // Its devices enumerated and their class drivers bound, the Bluetooth
    // controller's among them. Its mass storage, the stick's disk, is left
    // unused, whether its binding would be Bulk-Only or UAS: the firmware's
    // descriptors show only the setting it uses, so the census cannot tell
    // which, and either way the disk gets no configuration, SCSI command or
    // settle wait. Nothing is left settling either, so none is discovered.
    let found = xhci::begin_with_provider(&mut function, handoff, &mut dma, provider, minimum)
        .map_err(|error| format!("its xHCI driver failed: {error}"))?
        .without_mass_storage()
        .finish_unsettled(&mut dma, provider);
    let mut devices = Devices::default();
    devices.absorb_usb(Some(address), found, provider);
    devices.owners.push(Box::new(function));
    Ok(devices)
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

/// The check's lines: one `CHECK bluetooth SKIP` without the option, the
/// refusal when nothing was started, else one per step of the controller
/// the started xHCI brought, with the scan of the chosen one where it came
/// up.
#[expect(
    clippy::result_large_err,
    reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
)]
pub(super) fn check(machine: &mut Machine) {
    let lines = match &machine.bluetooth.plan {
        Plan::Off => {
            machine.record(
                "bluetooth",
                Verdict::Skip,
                String::from("no --bluetooth-check"),
            );
            return;
        }
        Plan::Refused(why) => bluetooth_check::refused(why),
        Plan::Chosen { .. } => bluetooth_check::refused(
            "no PCI access (see the cpu lines), so its xHCI was not started; nothing is started",
        ),
        Plan::Started => {
            let transports = &mut machine.bluetooth.transports;
            let chosen = {
                let controllers: Vec<Controller<'_>> = transports.iter().map(controller).collect();
                bluetooth_check::chosen(&controllers)
                    .filter(|&index| bluetooth_check::scannable(&controllers[index]))
            };
            let scan = chosen.map(|index| realtek::scan(&mut transports[index], SCAN_US));
            let controllers: Vec<Controller<'_>> = transports.iter().map(controller).collect();
            bluetooth_check::lines(&controllers, scan.as_ref())
        }
    };
    for line in lines {
        let verdict = match line.verdict {
            bluetooth_check::Verdict::Pass => Verdict::Pass,
            bluetooth_check::Verdict::Fail => Verdict::Fail,
            bluetooth_check::Verdict::Skip => Verdict::Skip,
        };
        machine.record(line.name, verdict, line.detail);
    }
}

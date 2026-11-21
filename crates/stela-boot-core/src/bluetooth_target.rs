//! Which one PCI function `--host-check --bluetooth-check` may start: the
//! xHCI the firmware's own USB stack found a Realtek USB Bluetooth controller
//! on, read before ExitBootServices, and only when nothing on that xHCI but
//! the disk Stela started from could be a disk.
//!
//! The check runs none of the probe's drivers. It is handed one [`Target`],
//! whose [`Target::admits`] takes its controller alone: every other function
//! (the PC's NVMe and SATA disks, network and Wi-Fi cards, GPUs, USB4 hosts,
//! the other xHCIs and the bridges) gets no driver, no BAR access and no Bus
//! Master. A controller that also holds another USB disk, which its start
//! would reset, or a firmware USB inventory that could not all be read, so
//! that the check cannot tell, starts nothing.
//!
//! On this PC the Bluetooth controller's xHCI (0000:05:00.4) also holds the
//! boot stick. Its start resets the controller, so the stick is enumerated
//! again; nothing writes to it, as the check's lines go to the screen and,
//! with `--keep-lines`, the firmware's variables.
use alloc::{format, string::String};
use stela_drivers::{
    bus::pci::Address,
    usb::{bluetooth::Setup, xhci},
};

/// USB Bluetooth's interface class, subclass and protocol.
const BLUETOOTH: [u8; 3] = [0xe0, 0x01, 0x01];
/// The mass storage interface class, every disk's, UAS's too.
const MASS_STORAGE: u8 = 0x08;

/// A PCI function of the firmware's census.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PciFunction {
    pub address: Address,
    pub vendor: u16,
    pub device: u16,
    /// Base class, subclass and programming interface.
    pub class: [u8; 3],
}

/// A USB interface the firmware's USB stack enumerated.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UsbInterface {
    /// The PCI function its device path leaves the PCI tree at.
    pub controller: Address,
    /// Its device path below that controller, as text.
    pub path: String,
    pub vendor: u16,
    pub product: u16,
    /// Interface class, subclass and protocol.
    pub class: [u8; 3],
    /// Its path leads to the disk Stela started from.
    pub boot: bool,
}

/// The one function the check may start.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Target {
    pub controller: Address,
    /// Its PCI IDs, checked again before it is started.
    pub vendor: u16,
    pub device: u16,
    /// The Bluetooth interface found on it.
    pub bluetooth: UsbInterface,
    /// The boot stick's path, when it is on this controller too.
    pub stick: Option<String>,
    /// The USB interfaces on it the firmware found.
    pub interfaces: usize,
}

impl Target {
    /// Whether the check may touch the PCI function at `address`: only its
    /// one controller.
    pub fn admits(&self, address: Address) -> bool {
        address == self.controller
    }

    /// The line said before the firmware exits.
    pub fn line(&self) -> String {
        let stick = match &self.stick {
            Some(path) => format!(
                ", and the boot stick at {path}, which its start resets and enumerates again"
            ),
            None => String::from("; the boot stick is on another controller"),
        };
        format!(
            "BLUETOOTH CHECK starts pci={} ({:04x}:{:04x}, xHCI) alone: it holds {:04x}:{:04x} at {} \
             among {} USB interfaces{stick}; no other PCI function is touched, and nothing writes \
             to the stick",
            self.controller,
            self.vendor,
            self.device,
            self.bluetooth.vendor,
            self.bluetooth.product,
            self.bluetooth.path,
            self.interfaces
        )
    }
}

/// The function to start among `functions`, the firmware's PCI census, from
/// `usb`, the firmware's USB interfaces (`None` when not all could be read);
/// or why nothing is started.
pub fn choose(functions: &[PciFunction], usb: Option<&[UsbInterface]>) -> Result<Target, String> {
    let refuse = |why: String| Err(format!("{why}; nothing is started"));
    let Some(usb) = usb else {
        return refuse(String::from(
            "the firmware's USB devices could not all be read, so the check cannot tell what else \
             the Bluetooth controller's xHCI holds",
        ));
    };
    let Some(bluetooth) = usb.iter().find(|interface| {
        interface.class == BLUETOOTH
            && Setup::of(interface.vendor, interface.product, false) == Some(Setup::Realtek)
    }) else {
        return refuse(format!(
            "the firmware's USB stack lists no Bluetooth controller a Realtek setup takes, among \
             {} USB interfaces",
            usb.len()
        ));
    };
    let controller = bluetooth.controller;
    let Some(function) = functions
        .iter()
        .find(|function| function.address == controller)
    else {
        return refuse(format!(
            "the Bluetooth controller's controller pci={controller} is not in the firmware's PCI \
             census"
        ));
    };
    if function.class != xhci::CLASS {
        let [base, sub, interface] = function.class;
        return refuse(format!(
            "the Bluetooth controller's controller pci={controller} is of class \
             {base:02x}{sub:02x}{interface:02x}, not an xHCI"
        ));
    }
    let on = || {
        usb.iter()
            .filter(move |interface| interface.controller == controller)
    };
    if let Some(disk) = on().find(|interface| interface.class[0] == MASS_STORAGE && !interface.boot)
    {
        return refuse(format!(
            "the Bluetooth controller's xHCI pci={controller} also holds the USB disk \
             {:04x}:{:04x} at {}, which its start would reset",
            disk.vendor, disk.product, disk.path
        ));
    }
    Ok(Target {
        controller,
        vendor: function.vendor,
        device: function.device,
        bluetooth: bluetooth.clone(),
        stick: on()
            .find(|interface| interface.class[0] == MASS_STORAGE && interface.boot)
            .map(|stick| stick.path.clone()),
        interfaces: on().count(),
    })
}

#[cfg(test)]
mod tests;

use super::*;
use alloc::{string::ToString, vec, vec::Vec};

fn at(text: &str) -> Address {
    Address::parse(text).unwrap()
}
fn pci(address: &str, vendor: u16, device: u16, class: [u8; 3]) -> PciFunction {
    PciFunction {
        address: at(address),
        vendor,
        device,
        class,
    }
}
fn usb(controller: &str, path: &str, ids: (u16, u16), class: [u8; 3], boot: bool) -> UsbInterface {
    UsbInterface {
        controller: at(controller),
        path: path.to_string(),
        vendor: ids.0,
        product: ids.1,
        class,
        boot,
    }
}

const XHCI: [u8; 3] = [0x0c, 0x03, 0x30];
const BLUETOOTH: [u8; 3] = [0xe0, 0x01, 0x01];
const DISK: [u8; 3] = [0x08, 0x06, 0x50];
const STICK: (u16, u16) = (0x0781, 0x5581);

/// This laptop's PCI functions, as its firmware's census lists them, with
/// one of each kind the check must never start.
fn census() -> Vec<PciFunction> {
    vec![
        pci("0000:00:00.0", 0x1022, 0x14e8, [0x06, 0x00, 0x00]),
        pci("0000:00:08.1", 0x1022, 0x14eb, [0x06, 0x04, 0x00]),
        pci("0000:01:00.0", 0x10ec, 0xc852, [0x02, 0x80, 0x00]),
        // The Windows disk.
        pci("0000:02:00.0", 0x144d, 0xa80c, [0x01, 0x08, 0x02]),
        pci("0000:03:00.0", 0x1022, 0x7901, [0x01, 0x06, 0x01]),
        pci("0000:04:00.0", 0x10ec, 0x8168, [0x02, 0x00, 0x00]),
        pci("0000:05:00.0", 0x10de, 0x28a0, [0x03, 0x00, 0x00]),
        pci("0000:05:00.3", 0x1022, 0x15b6, XHCI),
        pci("0000:05:00.4", 0x1022, 0x15b7, XHCI),
        pci("0000:06:00.0", 0x1022, 0x1668, [0x0c, 0x03, 0x40]),
        pci("0000:07:00.0", 0x1022, 0x15c0, XHCI),
    ]
}
/// The firmware's USB interfaces: the stick and, behind a hub, the
/// RTL8852CE's Bluetooth (two interfaces) on 05:00.4; a keyboard and another
/// disk on 05:00.3.
fn interfaces() -> Vec<UsbInterface> {
    vec![
        usb("0000:05:00.4", "USB(0x0,0x0)", STICK, DISK, true),
        usb(
            "0000:05:00.4",
            "USB(0x1,0x0)",
            (0x05e3, 0x0610),
            [0x09, 0, 0],
            false,
        ),
        usb(
            "0000:05:00.4",
            "USB(0x1,0x0)/USB(0x2,0x0)",
            (0x13d3, 0x3612),
            BLUETOOTH,
            false,
        ),
        usb(
            "0000:05:00.4",
            "USB(0x1,0x0)/USB(0x2,0x1)",
            (0x13d3, 0x3612),
            BLUETOOTH,
            false,
        ),
        usb(
            "0000:05:00.3",
            "USB(0x2,0x0)",
            (0x046d, 0xc31c),
            [0x03, 1, 1],
            false,
        ),
        usb(
            "0000:05:00.3",
            "USB(0x3,0x0)",
            (0x0bc2, 0x2322),
            DISK,
            false,
        ),
    ]
}

#[test]
fn only_the_bluetooth_controllers_xhci_is_admitted_among_every_other_function() {
    let functions = census();
    let usb = interfaces();
    let target = choose(&functions, Ok(&usb)).unwrap();
    assert_eq!(target.controller, at("0000:05:00.4"));
    assert_eq!((target.vendor, target.device), (0x1022, 0x15b7));
    assert_eq!(
        (target.bluetooth.vendor, target.bluetooth.product),
        (0x13d3, 0x3612)
    );
    assert_eq!(target.bluetooth.path, "USB(0x1,0x0)/USB(0x2,0x0)");
    assert_eq!(target.stick.as_deref(), Some("USB(0x0,0x0)"));
    assert_eq!(target.interfaces, 4);
    // Default deny: of every function the census lists, the NVMe, AHCI,
    // network, Wi-Fi, GPU, USB4, bridges and the other xHCIs among them,
    // the target alone is admitted.
    let admitted: Vec<Address> = functions
        .iter()
        .map(|function| function.address)
        .filter(|&address| target.admits(address))
        .collect();
    assert_eq!(admitted, [at("0000:05:00.4")]);
    assert!(!target.admits(at("0000:02:00.0")));
    assert!(!target.admits(at("0000:05:00.3")));
    assert_eq!(
        target.line(),
        "BLUETOOTH CHECK starts pci=0000:05:00.4 (1022:15b7, xHCI) alone: it holds 13d3:3612 at \
         USB(0x1,0x0)/USB(0x2,0x0) among 4 USB interfaces, and the boot stick at USB(0x0,0x0), \
         which its start resets and enumerates again; no other PCI function is touched, and \
         nothing writes to the stick"
    );
}

#[test]
fn a_controller_with_another_disk_or_an_unreadable_census_is_never_started() {
    let functions = census();
    // What could not be read is named.
    let unread = "UsbIo handle 3 of 7 at 0000:05:00.4 USB(0x1,0x0): its interface descriptor \
                  (DEVICE_ERROR)";
    assert_eq!(
        choose(&functions, Err(unread)).unwrap_err(),
        "the firmware's USB devices could not all be read (UsbIo handle 3 of 7 at 0000:05:00.4 \
         USB(0x1,0x0): its interface descriptor (DEVICE_ERROR)), so the check cannot tell what \
         else the Bluetooth controller's xHCI holds; nothing is started"
    );
    // A USB disk beside the stick on the same controller.
    let mut usb = interfaces();
    usb.push(usb_disk_on_target());
    let other = choose(&functions, Ok(&usb)).unwrap_err();
    assert_eq!(
        other,
        "the Bluetooth controller's xHCI pci=0000:05:00.4 also holds the USB disk 152d:0583 at \
         USB(0x1,0x0)/USB(0x3,0x0), which its start would reset; nothing is started"
    );
    // The stick alone is no other disk, wherever it is; a disk on another
    // controller is not reset.
    assert!(choose(&functions, Ok(&interfaces())).is_ok());
}

fn usb_disk_on_target() -> UsbInterface {
    usb(
        "0000:05:00.4",
        "USB(0x1,0x0)/USB(0x3,0x0)",
        (0x152d, 0x0583),
        [0x08, 0x06, 0x62],
        false,
    )
}

#[test]
fn no_realtek_bluetooth_or_a_controller_that_is_no_listed_xhci_starts_nothing() {
    let functions = census();
    // A MediaTek controller is not this check's.
    let mediatek = vec![usb(
        "0000:05:00.4",
        "USB(0x2,0x0)",
        (0x0e8d, 0x7922),
        BLUETOOTH,
        false,
    )];
    let none = choose(&functions, Ok(&mediatek)).unwrap_err();
    assert_eq!(
        none,
        "the firmware's USB stack lists no Bluetooth controller a Realtek setup takes, among 1 \
         USB interfaces; nothing is started"
    );
    // A Realtek ID on an interface of another class is no Bluetooth one.
    let wrong = vec![usb(
        "0000:05:00.4",
        "USB(0x2,0x0)",
        (0x13d3, 0x3612),
        [0xff, 0, 0],
        false,
    )];
    assert!(choose(&functions, Ok(&wrong)).is_err());
    // Its controller absent from the census, or no xHCI.
    let usb = interfaces();
    let missing: Vec<PciFunction> = functions
        .iter()
        .copied()
        .filter(|function| function.address != at("0000:05:00.4"))
        .collect();
    assert_eq!(
        choose(&missing, Ok(&usb)).unwrap_err(),
        "the Bluetooth controller's controller pci=0000:05:00.4 is not in the firmware's PCI \
         census; nothing is started"
    );
    let mut ehci = functions.clone();
    for function in &mut ehci {
        if function.address == at("0000:05:00.4") {
            function.class = [0x0c, 0x03, 0x20];
        }
    }
    assert_eq!(
        choose(&ehci, Ok(&usb)).unwrap_err(),
        "the Bluetooth controller's controller pci=0000:05:00.4 is of class 0c0320, not an \
         xHCI; nothing is started"
    );
}

#[test]
fn the_stick_on_another_controller_is_not_said_to_be_reset() {
    let functions = census();
    let mut usb = interfaces();
    usb[0].controller = at("0000:05:00.3");
    let target = choose(&functions, Ok(&usb)).unwrap();
    assert_eq!(target.stick, None);
    assert_eq!(target.interfaces, 3);
    assert!(
        target
            .line()
            .contains("the boot stick is on another controller")
    );
}

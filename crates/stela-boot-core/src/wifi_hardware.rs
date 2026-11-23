//! Whether the startup menu offers Wi-Fi (`stela-boot`'s `choice::Menu`):
//! its "Wi-Fi: ..." line and the `W` key that opens the Wi-Fi page. It does
//! only on a PC with a Wi-Fi function the scan can start a station on today
//! (`stela_drivers::probe::wifi_starts`), found in the firmware's PCI census
//! before ExitBootServices. A PC whose census lists every function and none
//! is one, such as a desktop with no Wi-Fi card or one whose card no driver
//! here starts (an MT7920 or an Intel 9000-series radio), is offered none. A census that
//! is unavailable or could not read every function offers Wi-Fi as the
//! configuration alone did, so a firmware quirk never hides a Wi-Fi option
//! that works.
use alloc::{format, string::String, vec::Vec};
use stela_drivers::{bus::pci::Identity, probe::wifi_starts};

/// The firmware's PCI census, as the menu reads it.
#[derive(Clone, Copy, Debug)]
pub struct Census<'a> {
    /// The functions read.
    pub functions: &'a [Identity],
    /// The functions the firmware has a handle for that could not be read.
    pub unread: usize,
    /// The PCI ports that show nothing below them
    /// (`stela_loader::host_firmware::bundle::empty_ports`): a Wi-Fi card
    /// the firmware left powered off there answers only after the engine
    /// powers the port on, after the firmware's exit.
    pub empty_ports: usize,
}

/// What the census says of the PC's Wi-Fi.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Hardware {
    /// The first function a station starts on.
    Supported(Identity),
    /// Every function was read and none is one. The other wireless-class
    /// functions found, which the driver cannot start yet, if any.
    Absent(Vec<Identity>),
    /// No complete census: why.
    Unknown(String),
}

/// What `census` says, or why there is none.
pub fn decide(census: Result<Census<'_>, &str>) -> Hardware {
    let census = match census {
        Ok(census) => census,
        Err(why) => {
            return Hardware::Unknown(format!("the firmware's PCI census is unavailable: {why}"));
        }
    };
    if let Some(function) = census
        .functions
        .iter()
        .find(|function| wifi_starts(function))
    {
        return Hardware::Supported(*function);
    }
    if census.unread != 0 {
        return Hardware::Unknown(format!(
            "{} PCI function(s) of the firmware's census could not be read",
            census.unread
        ));
    }
    if census.functions.is_empty() {
        return Hardware::Unknown("the firmware's PCI census lists no function".into());
    }
    if census.empty_ports != 0 {
        return Hardware::Unknown(format!(
            "{} PCI port(s) show nothing below them, where a Wi-Fi card the firmware left \
             powered off answers only after the firmware's exit",
            census.empty_ports
        ));
    }
    Hardware::Absent(
        census
            .functions
            .iter()
            .filter(|function| function.class[..2] == [2, 0x80])
            .copied()
            .collect(),
    )
}

impl Hardware {
    /// Whether the menu shows its Wi-Fi line and `W` opens the page.
    pub fn offered(&self) -> bool {
        !matches!(self, Self::Absent(_))
    }

    /// The boot log's one line saying which way the menu decided.
    pub fn line(&self) -> String {
        let named = |function: &Identity| {
            format!(
                "{} ({:04x}:{:04x})",
                function.address, function.vendor, function.device
            )
        };
        match self {
            Self::Supported(function) => format!(
                "CHOICE Wi-Fi offered: {} is a Wi-Fi chip the driver starts\n",
                named(function)
            ),
            Self::Absent(others) if others.is_empty() => String::from(
                "CHOICE Wi-Fi hidden: no PCI function is a Wi-Fi chip the driver starts\n",
            ),
            Self::Absent(others) => format!(
                "CHOICE Wi-Fi hidden: no PCI function is a Wi-Fi chip the driver starts; \
                 not started yet: {}\n",
                others.iter().map(named).collect::<Vec<_>>().join(", ")
            ),
            Self::Unknown(why) => {
                format!("CHOICE Wi-Fi offered: the PC's Wi-Fi chip is not known ({why})\n")
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use stela_drivers::bus::pci::Address;

    fn function(bus: u8, vendor: u16, device: u16, class: [u8; 3]) -> Identity {
        Identity {
            address: Address {
                segment: 0,
                bus,
                device: 0,
                function: 0,
            },
            vendor,
            device,
            class,
        }
    }
    const WIFI: [u8; 3] = [2, 0x80, 0];
    /// A PC's other functions: a host bridge, an NVMe disk, an xHCI and an
    /// Ethernet controller.
    fn others() -> Vec<Identity> {
        Vec::from([
            function(0, 0x1022, 0x14e8, [6, 0, 0]),
            function(1, 0x144d, 0xa80a, [1, 8, 2]),
            function(4, 0x1022, 0x15b6, [0x0c, 3, 0x30]),
            function(5, 0x10ec, 0x8168, [2, 0, 0]),
        ])
    }
    fn census(functions: &[Identity], unread: usize) -> Result<Census<'_>, &'static str> {
        Ok(Census {
            functions,
            unread,
            empty_ports: 0,
        })
    }

    #[test]
    fn a_port_with_nothing_below_it_keeps_wifi_offered() {
        // A laptop whose firmware left its M.2 Wi-Fi slot powered off: no
        // Wi-Fi function yet, and one port shows nothing below it.
        let functions = others();
        let powered_off = Census {
            functions: &functions,
            unread: 0,
            empty_ports: 1,
        };
        let hardware = decide(Ok(powered_off));
        assert!(matches!(hardware, Hardware::Unknown(_)), "{hardware:?}");
        assert!(hardware.offered());
        // Every port answered and still no Wi-Fi function: none offered.
        assert!(!decide(census(&functions, 0)).offered());
    }

    #[test]
    fn supported_chip_offers_wifi() {
        let mut functions = others();
        let rtl8852ce = function(3, 0x10ec, 0xc852, WIFI);
        functions.push(rtl8852ce);
        let hardware = decide(census(&functions, 0));
        assert_eq!(hardware, Hardware::Supported(rtl8852ce));
        assert!(hardware.offered());
        assert_eq!(
            hardware.line(),
            "CHOICE Wi-Fi offered: 0000:03:00.0 (10ec:c852) is a Wi-Fi chip the driver starts\n"
        );
        // A function that could not be read does not hide one that was.
        assert!(decide(census(&functions, 2)).offered());
    }

    #[test]
    fn no_wifi_chip_hides_wifi() {
        let functions = others();
        let hardware = decide(census(&functions, 0));
        assert_eq!(hardware, Hardware::Absent(Vec::new()));
        assert!(!hardware.offered());
        assert_eq!(
            hardware.line(),
            "CHOICE Wi-Fi hidden: no PCI function is a Wi-Fi chip the driver starts\n"
        );
    }

    #[test]
    fn failed_census_offers_wifi() {
        let hardware = decide(Err("enumerate protocol: NOT_FOUND"));
        assert!(hardware.offered());
        assert_eq!(
            hardware.line(),
            "CHOICE Wi-Fi offered: the PC's Wi-Fi chip is not known (the firmware's PCI \
             census is unavailable: enumerate protocol: NOT_FOUND)\n"
        );
        // A census missing a function might be missing the chip.
        let functions = others();
        let partial = decide(census(&functions, 1));
        assert!(matches!(partial, Hardware::Unknown(_)));
        assert!(partial.offered());
        // As is one that lists nothing.
        assert!(decide(census(&[], 0)).offered());
    }

    #[test]
    fn unsupported_wifi_chips_hide_wifi() {
        // An Intel 9560, an MT7920 and an RTL8852BE have no driver that
        // starts them.
        let ac9560 = function(2, 0x8086, 0x9df0, WIFI);
        let mt7920 = function(3, 0x14c3, 0x7920, WIFI);
        let rtl8852be = function(6, 0x10ec, 0xb852, WIFI);
        for chip in [ac9560, mt7920, rtl8852be] {
            let mut functions = others();
            functions.push(chip);
            let hardware = decide(census(&functions, 0));
            assert_eq!(hardware, Hardware::Absent(Vec::from([chip])));
            assert!(!hardware.offered());
        }
        let mut functions = others();
        functions.extend([ac9560, mt7920]);
        assert_eq!(
            decide(census(&functions, 0)).line(),
            "CHOICE Wi-Fi hidden: no PCI function is a Wi-Fi chip the driver starts; not \
             started yet: 0000:02:00.0 (8086:9df0), 0000:03:00.0 (14c3:7920)\n"
        );
        // An MT7921, an MT7922, an MT7925 and an RTL8822CE start, their
        // claims quiesced by a reset.
        for chip in [
            function(3, 0x14c3, 0x7961, WIFI),
            function(3, 0x14c3, 0x0616, WIFI),
            function(3, 0x0b48, 0x7922, WIFI),
            function(3, 0x14c3, 0x7925, WIFI),
            function(6, 0x10ec, 0xc822, WIFI),
        ] {
            let mut functions = others();
            functions.push(chip);
            let hardware = decide(census(&functions, 0));
            assert_eq!(hardware, Hardware::Supported(chip));
            assert!(hardware.offered());
        }
        // An RTL8852CE's ID on a function of another class is no Wi-Fi chip.
        let mut functions = others();
        functions.push(function(3, 0x10ec, 0xc852, [2, 0, 0]));
        assert!(!decide(census(&functions, 0)).offered());
    }
}

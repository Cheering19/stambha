// SPDX-License-Identifier: GPL-2.0-or-later
//! What a Bluetooth driver's failure says of the bytes it kept, once
//! printed. A failure line reaches the boot log, which owners post, and the
//! bytes a controller sent can be anything: a controller dump is its RAM,
//! which can hold link keys; an event can carry a neighbour's address and
//! name; a version or address read carries the controller's own address.
//! So the errors and observations print what failed and how many bytes it
//! saw, never their values, while the types keep the bytes for the code
//! that reads them:
//!
//! - [`Bytes`] prints a byte string as its length;
//! - [`Address`] prints a BD_ADDR as its vendor half and last byte, as
//!   `stela_contracts::masked::Mac` prints a MAC address;
//! - [`Lists`] prints a value's Debug text with every list of plain
//!   numbers, the way Debug writes a byte array, slice or vector, as its
//!   length, so evidence no rule above names (an attempt's observations,
//!   nested causes) prints no byte either.
use alloc::string::String;
use core::fmt::{self, Write as _};

/// A byte string, printed as `<N bytes>`.
pub(crate) struct Bytes<'a>(pub &'a [u8]);

impl fmt::Debug for Bytes<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "<{} bytes>", self.0.len())
    }
}

/// A BD_ADDR in HCI order (its least significant byte first), printed most
/// significant first as its vendor half and last byte: `aa:bb:cc:..:..:ff`.
pub(crate) struct Address<'a>(pub &'a [u8; 6]);

impl fmt::Debug for Address<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut address = *self.0;
        address.reverse();
        write!(f, "{}", stela_contracts::masked::Mac(&address))
    }
}

/// `T`'s Debug text, each list of plain numbers in it (`[1, 2, 3]`) written
/// as `<3 bytes>`. Lists of anything else, and text in quotes, are kept.
pub(crate) struct Lists<'a, T: ?Sized>(pub &'a T);

impl<T: fmt::Debug + ?Sized> fmt::Display for Lists<'_, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut filter = Filter {
            out: f,
            list: None,
            quote: None,
            escaped: false,
        };
        write!(filter, "{:?}", self.0)?;
        filter.flush()
    }
}

/// Writes Debug text on, holding back each list until it is known to be one
/// of plain numbers.
struct Filter<'a, 'b> {
    out: &'a mut fmt::Formatter<'b>,
    /// The list held back, from its `[`.
    list: Option<String>,
    /// The quote of the string or character being written.
    quote: Option<char>,
    escaped: bool,
}

impl Filter<'_, '_> {
    /// Writes the list held back as it was.
    fn flush(&mut self) -> fmt::Result {
        match self.list.take() {
            Some(list) => self.out.write_str(&list),
            None => Ok(()),
        }
    }
}

impl fmt::Write for Filter<'_, '_> {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        for c in text.chars() {
            self.write_char(c)?;
        }
        Ok(())
    }

    fn write_char(&mut self, c: char) -> fmt::Result {
        if let Some(quote) = self.quote {
            if self.escaped {
                self.escaped = false;
            } else if c == '\\' {
                self.escaped = true;
            } else if c == quote {
                self.quote = None;
            }
            return self.out.write_char(c);
        }
        if let Some(list) = self.list.as_mut() {
            match c {
                ']' => {
                    let numbers = list
                        .split(|c: char| !c.is_ascii_digit())
                        .filter(|run| !run.is_empty())
                        .count();
                    self.list = None;
                    return if numbers == 0 {
                        self.out.write_str("[]")
                    } else {
                        write!(self.out, "<{numbers} bytes>")
                    };
                }
                '0'..='9' | ',' | '-' => {
                    list.push(c);
                    return Ok(());
                }
                c if c.is_whitespace() => {
                    list.push(c);
                    return Ok(());
                }
                // Not a list of plain numbers: it goes out as it is.
                _ => self.flush()?,
            }
        }
        match c {
            '[' => self.list = Some(String::from("[")),
            '"' | '\'' => {
                self.quote = Some(c);
                self.escaped = false;
                self.out.write_char(c)?;
            }
            c => self.out.write_char(c)?,
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::{format, vec, vec::Vec};

    #[derive(Debug)]
    #[expect(dead_code, reason = "Only its Debug text is read.")]
    struct Evidence {
        requirement: &'static str,
        observed: Vec<u8>,
        address: Option<[u8; 6]>,
        opcodes: Vec<u16>,
        nested: Vec<Vec<u8>>,
        registers: [Option<u32>; 2],
        empty: Vec<u8>,
        offset: u32,
        name: char,
    }

    /// Every byte list leaves the text as its length, nested ones too;
    /// numbers outside lists, lists of anything else and quoted text stay.
    #[test]
    fn lists_of_numbers_print_as_their_length() {
        let evidence = Evidence {
            requirement: "a \"[1, 2]\" quoted",
            observed: vec![0x47, 0x61, 0x72, 0x76],
            address: Some([0x10, 0xa7, 0xb8, 0x30, 0x31, 0x32]),
            opcodes: vec![0x0c03, 0x1009],
            nested: vec![vec![1, 2], vec![3]],
            registers: [Some(7), None],
            empty: Vec::new(),
            offset: 4660,
            name: '[',
        };
        let text = format!("{}", Lists(&evidence));
        assert_eq!(
            text,
            "Evidence { requirement: \"a \\\"[1, 2]\\\" quoted\", observed: <4 bytes>, \
             address: Some(<6 bytes>), opcodes: <2 bytes>, nested: [<2 bytes>, <1 bytes>], \
             registers: [Some(7), None], empty: [], offset: 4660, name: '[' }"
        );
        for secret in ["71", "167", "184", "3075"] {
            assert!(!text.contains(secret), "{text}");
        }
        // Pretty Debug text alike.
        assert_eq!(format!("{}", Lists(&PrettyBytes)), "<3 bytes>");
    }

    struct PrettyBytes;
    impl fmt::Debug for PrettyBytes {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            write!(f, "{:#?}", [9u8, 8, 7])
        }
    }

    /// A synthetic name and BD_ADDR, as a controller's frames could carry.
    const NAME: &[u8] = b"Tester";
    const ADDRESS: [u8; 6] = [0x10, 0xa7, 0xb8, 0x30, 0x31, 0x32];

    /// `secret`: the name, the address and a controller dump's bytes.
    fn secret() -> Vec<u8> {
        [NAME, &ADDRESS[..], &[0x5a; 8]].concat()
    }

    /// Whether `text` shows any of [`secret`]'s values, as Debug writes
    /// them or in hex.
    fn leaks(text: &str) -> bool {
        [
            "84, 101, 115",
            "167, 184",
            "90, 90",
            "a7:b8",
            "b8:a7",
            "Tester",
        ]
        .iter()
        .any(|value| text.contains(value))
    }

    /// The failure lines of the transports and initializers, Display and
    /// Debug alike: a Qualcomm controller dump, a Broadcom PCIe frame,
    /// event and return, an Intel PCIe frame and a Realtek response, alone,
    /// nested and inside an initialization's failure, show their lengths
    /// and never a byte.
    #[test]
    fn failure_lines_print_no_byte_a_controller_sent() {
        use crate::bluetooth::{bcm4377, intel_pcie};
        use crate::usb::bluetooth::{self as usb, InitializationError, broadcom, realtek};
        let dump = || usb::Error {
            kind: usb::ErrorKind::ControllerDump {
                driver: "qualcomm-usb",
                packet: usb::Packet::Diagnostic(secret()),
            },
            cleanup: None,
        };
        let broadcom_pcie = || bcm4377::Error::Cleanup {
            cause: alloc::boxed::Box::new(bcm4377::Error::Packet {
                kind: bcm4377::Kind::Event,
                bytes: secret(),
            }),
            cleanup: alloc::boxed::Box::new(bcm4377::Error::Controller {
                opcode: 0x1009,
                status: 0,
                returned: ADDRESS.to_vec(),
            }),
        };
        let hci = bcm4377::Error::Hci {
            requirement: "event",
            bytes: secret(),
        };
        let intel = intel_pcie::Error::Packet {
            requirement: "complete RFH",
            raw: secret(),
        };
        let realtek = || realtek::Error::Cleanup {
            cause: alloc::boxed::Box::new(realtek::Error::Protocol {
                requirement: "local version",
                observed: secret(),
            }),
            cleanup: dump(),
        };
        let controller = realtek::Error::Controller {
            opcode: 0x1009,
            status: 0x0c,
            parameters: ADDRESS.to_vec(),
        };
        let texts = [
            format!("{}", dump()),
            format!("{:?}", dump()),
            format!("{}", broadcom_pcie()),
            format!("{:?}", broadcom_pcie()),
            format!("{hci} {hci:?}"),
            format!("{intel} {intel:?}"),
            format!("{}", realtek()),
            format!("{:?}", realtek()),
            format!("{controller} {controller:?}"),
            format!("{}", InitializationError::Realtek(realtek())),
            format!("{}", InitializationError::Transport(dump())),
            format!(
                "{}",
                InitializationError::Broadcom(broadcom::Error::InvalidAddress(ADDRESS))
            ),
        ];
        for text in &texts {
            assert!(!leaks(text), "{text}");
        }
        assert!(texts[0].contains("Diagnostic(<20 bytes>)"), "{}", texts[0]);
        assert!(texts[3].contains("returned: <6 bytes>"), "{}", texts[3]);
        assert!(texts[5].contains("raw: <20 bytes>"), "{}", texts[5]);
        assert!(texts[8].contains("parameters: <6 bytes>"), "{}", texts[8]);
        // The packets themselves print their lengths.
        let packets = [
            format!("{:?}", usb::Packet::Event(secret())),
            format!(
                "{:?}",
                bcm4377::Packet {
                    kind: bcm4377::Kind::Acl,
                    bytes: secret(),
                    completion: [0x5a; 16],
                    footer: secret(),
                }
            ),
            format!(
                "{:?}",
                intel_pcie::Packet {
                    kind: intel_pcie::Kind::Event,
                    bytes: secret(),
                    padding: secret(),
                    receive_header: [0; 8],
                    completion: 0,
                    diagnostic: false,
                }
            ),
        ];
        for text in &packets {
            assert!(!leaks(text), "{text}");
        }
    }

    #[test]
    fn bytes_and_addresses_print_masked() {
        assert_eq!(format!("{:?}", Bytes(&[1, 2, 3])), "<3 bytes>");
        // 32:31:30:b8:a7:10 in HCI order.
        let address = [0x10, 0xa7, 0xb8, 0x30, 0x31, 0x32];
        assert_eq!(format!("{:?}", Address(&address)), "32:31:30:..:..:10");
    }
}

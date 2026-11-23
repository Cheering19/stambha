//! The Wi-Fi network the mount's configuration names (`--wifi`, `--wifi-key`,
//! `--wifi-country`), for whichever Wi-Fi adapter joins it. The secret
//! never appears in `Debug` output.
use alloc::{string::String, vec::Vec};
use core::fmt;
use zeroize::Zeroizing;

/// The longest SSID IEEE 802.11 allows, in bytes.
pub const SSID_BYTES: usize = 32;

/// The network to join.
#[derive(Clone)]
pub struct Network {
    /// One to [`SSID_BYTES`] bytes.
    pub ssid: Vec<u8>,
    pub secret: Secret,
    /// An ISO 3166 alpha-2 country code in uppercase ASCII; none leaves the
    /// adapter firmware's default regulatory domain.
    pub country: Option<[u8; 2]>,
    /// The other saved networks (`wifi.saved`), joined when this one, the
    /// preferred network, is not in range: the strongest of them a scan
    /// finds. Their names never appear in a failure the keeper reports.
    pub alternates: Vec<Alternate>,
}

impl fmt::Debug for Network {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Network")
            .field("ssid", &String::from_utf8_lossy(&self.ssid))
            .field("secret", &self.secret)
            .field(
                "country",
                &self.country.map(|code| [code[0] as char, code[1] as char]),
            )
            .field("alternates", &self.alternates.len())
            .finish()
    }
}

/// A saved network other than the preferred one.
#[derive(Clone)]
pub struct Alternate {
    /// One to [`SSID_BYTES`] bytes.
    pub ssid: Vec<u8>,
    pub secret: Secret,
}

impl fmt::Debug for Alternate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Alternate")
            .field("ssid", &"..")
            .field("secret", &self.secret)
            .finish()
    }
}

impl Network {
    /// Make the saved alternate named `ssid` the preferred network, the one
    /// preferred until now its first alternate; whether `ssid` names this
    /// network or one of its alternates. A name neither carries changes
    /// nothing.
    pub fn prefer(&mut self, ssid: &[u8]) -> bool {
        if self.ssid == ssid {
            return true;
        }
        let Some(index) = self
            .alternates
            .iter()
            .position(|alternate| alternate.ssid == ssid)
        else {
            return false;
        };
        let chosen = self.alternates.remove(index);
        let previous = Alternate {
            ssid: core::mem::replace(&mut self.ssid, chosen.ssid),
            secret: core::mem::replace(&mut self.secret, chosen.secret),
        };
        self.alternates.insert(0, previous);
        true
    }
}

/// A WPA2-PSK or WPA3-SAE credential, or none for an open network.
#[derive(Clone)]
pub enum Secret {
    /// Eight to 63 printable ASCII characters.
    Passphrase(Passphrase),
    /// The 256-bit pre-shared key, from 64 hex digits.
    Psk(Zeroizing<[u8; 32]>),
    /// No credential: the network is open, wpa_supplicant's `key_mgmt=NONE`.
    /// Only a BSS without privacy, RSN or WPA matches it.
    Open,
}

impl fmt::Debug for Secret {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Passphrase(_) => "Passphrase(..)",
            Self::Psk(_) => "Psk(..)",
            Self::Open => "Open",
        })
    }
}

/// A WPA passphrase of eight to 63 printable ASCII characters, cleared when
/// dropped.
#[derive(Clone)]
pub struct Passphrase {
    bytes: Zeroizing<[u8; 63]>,
    len: u8,
}

impl Passphrase {
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..usize::from(self.len)]
    }
}

/// Why a key file holds no usable secret.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SecretError {
    /// Neither 8 to 63 characters nor 64 hex digits.
    Length,
    /// A character outside printable ASCII.
    NotAscii,
    /// 64 characters that are not all hex digits.
    BadHex,
}

impl fmt::Display for SecretError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Length => {
                "Wi-Fi key is neither an 8 to 63 character passphrase nor 64 hex digits"
            }
            Self::NotAscii => "Wi-Fi passphrase has a character outside printable ASCII",
            Self::BadHex => "Wi-Fi key of 64 characters is not all hex digits",
        })
    }
}

impl Secret {
    /// The secret a key file holds: a passphrase, or a PSK as 64 hex digits.
    /// One trailing `\n` or `\r\n` is dropped.
    pub fn parse(file: &[u8]) -> Result<Self, SecretError> {
        let text = file
            .strip_suffix(b"\r\n")
            .or_else(|| file.strip_suffix(b"\n"))
            .unwrap_or(file);
        if !text.iter().all(|byte| (0x20..0x7f).contains(byte)) {
            return Err(SecretError::NotAscii);
        }
        match text.len() {
            64 => {
                let mut key = Zeroizing::new([0u8; 32]);
                for (byte, pair) in key.iter_mut().zip(text.as_chunks::<2>().0) {
                    let digit = |c: u8| (c as char).to_digit(16).ok_or(SecretError::BadHex);
                    *byte = (digit(pair[0])? << 4 | digit(pair[1])?) as u8;
                }
                Ok(Self::Psk(key))
            }
            len @ 8..=63 => {
                let mut bytes = Zeroizing::new([0u8; 63]);
                bytes[..len].copy_from_slice(text);
                Ok(Self::Passphrase(Passphrase {
                    bytes,
                    len: len as u8,
                }))
            }
            _ => Err(SecretError::Length),
        }
    }
}

impl Secret {
    /// The pairwise master key for `ssid`: the PSK itself, or the
    /// passphrase's PBKDF2 derivation (IEEE 802.11-2020 J.4.1). An open
    /// network has none and keys nothing; its value is zero.
    pub fn pmk(&self, ssid: &[u8]) -> Zeroizing<[u8; 32]> {
        match self {
            Self::Passphrase(passphrase) => {
                super::supplicant::passphrase_psk(passphrase.as_bytes(), ssid)
            }
            Self::Psk(psk) => psk.clone(),
            Self::Open => Zeroizing::new([0; 32]),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_passphrase_derives_its_pmk_and_a_psk_is_its_own() {
        let pmk = Secret::parse(b"password").unwrap().pmk(b"IEEE");
        assert_eq!(pmk[..4], [0xf4, 0x2c, 0x6f, 0xc5]);
        let psk = Secret::parse(&b"0f".repeat(32)).unwrap();
        assert_eq!(*psk.pmk(b"any"), [0x0f; 32]);
    }

    #[test]
    fn a_passphrase_keeps_its_characters_without_the_line_end() {
        let Secret::Passphrase(phrase) = Secret::parse(b"correct horse\r\n").unwrap() else {
            panic!("passphrase expected");
        };
        assert_eq!(phrase.as_bytes(), b"correct horse");
    }

    #[test]
    fn sixty_four_hex_digits_are_a_psk() {
        let text = b"0f".repeat(32);
        let Secret::Psk(key) = Secret::parse(&text).unwrap() else {
            panic!("PSK expected");
        };
        assert_eq!(*key, [0x0f; 32]);
        let mut bad = text.clone();
        bad[5] = b'g';
        assert_eq!(Secret::parse(&bad).err(), Some(SecretError::BadHex));
    }

    #[test]
    fn short_long_and_control_character_keys_are_refused() {
        assert_eq!(Secret::parse(b"short").err(), Some(SecretError::Length));
        assert_eq!(Secret::parse(&[b'a'; 65]).err(), Some(SecretError::Length));
        assert_eq!(
            Secret::parse(b"tab\there!").err(),
            Some(SecretError::NotAscii)
        );
    }

    #[test]
    fn debug_output_hides_the_secret() {
        let config = Network {
            alternates: Default::default(),
            ssid: b"Home".to_vec(),
            secret: Secret::parse(b"hunter22").unwrap(),
            country: Some(*b"US"),
        };
        let shown = alloc::format!("{config:?}");
        assert!(shown.contains("Home") && !shown.contains("hunter22"));
    }

    fn psk(byte: u8) -> Secret {
        Secret::Psk(Zeroizing::new([byte; 32]))
    }

    fn named(network: &Network) -> Vec<&[u8]> {
        core::iter::once(&network.ssid[..])
            .chain(
                network
                    .alternates
                    .iter()
                    .map(|alternate| &alternate.ssid[..]),
            )
            .collect()
    }

    /// The menu's choice becomes the preferred network, the one before it
    /// the first alternate; a name no network carries changes nothing.
    #[test]
    fn preferring_a_saved_network_puts_it_first() {
        let mut network = Network {
            ssid: b"home".to_vec(),
            secret: psk(1),
            country: None,
            alternates: alloc::vec![
                Alternate {
                    ssid: b"cafe".to_vec(),
                    secret: psk(2),
                },
                Alternate {
                    ssid: b"phone".to_vec(),
                    secret: psk(3),
                },
            ],
        };
        assert!(network.prefer(b"phone"));
        assert_eq!(named(&network), [&b"phone"[..], &b"home"[..], &b"cafe"[..]]);
        assert!(matches!(&network.secret, Secret::Psk(key) if **key == [3; 32]));
        assert!(network.prefer(b"phone"));
        assert!(!network.prefer(b"elsewhere"));
        assert_eq!(named(&network), [&b"phone"[..], &b"home"[..], &b"cafe"[..]]);
    }
}

//! The saved Wi-Fi networks file (`\EFI\stela\wifi.saved`): the networks a
//! boot joins when the preferred one (`--wifi` and its `wifi.key`) is not in
//! range, the strongest of them a scan finds (`Keeper::choose`).
//!
//! One network per line: its name, then its secret (a passphrase of 8 to 63
//! printable characters, or a PSK as 64 hex digits), as two words separated
//! by spaces or tabs, each writing `%`, whitespace and control characters as
//! `%XX`, as a `--wifi` word does. Empty lines and lines starting with `#`
//! are skipped; a line that is not one usable network is counted, never
//! echoed, since it may hold a secret.
use alloc::vec::Vec;
use stela_drivers::net::wifi::network::{Alternate, SSID_BYTES, Secret};

/// What a saved networks file holds.
pub struct Saved {
    /// The usable networks, in the file's order.
    pub networks: Vec<Alternate>,
    /// Lines that are not one usable network.
    pub unusable: usize,
}

/// The networks `file` saves. The caller wipes `file` itself; the decoded
/// secrets are wiped here once parsed.
pub fn parse(file: &[u8]) -> Saved {
    let mut networks = Vec::new();
    let mut unusable = 0;
    for line in file.split(|&byte| byte == b'\n') {
        let line = line.strip_suffix(b"\r").unwrap_or(line);
        let line = line.trim_ascii();
        if line.is_empty() || line[0] == b'#' {
            continue;
        }
        match network(line) {
            Some(network) => networks.push(network),
            None => unusable += 1,
        }
    }
    Saved { networks, unusable }
}

/// One line's network: exactly two words, a 1 to 32 byte name and a secret.
fn network(line: &[u8]) -> Option<Alternate> {
    let mut words = line
        .split(|&byte| byte == b' ' || byte == b'\t')
        .filter(|word| !word.is_empty());
    let (Some(name), Some(secret), None) = (words.next(), words.next(), words.next()) else {
        return None;
    };
    let ssid = decode(name).filter(|ssid| (1..=SSID_BYTES).contains(&ssid.len()))?;
    let mut key = decode(secret)?;
    let parsed = Secret::parse(&key);
    wipe(&mut key);
    Some(Alternate {
        ssid,
        secret: parsed.ok()?,
    })
}

/// A word with its `%XX` escapes decoded; none for a `%` without two hex
/// digits after it.
fn decode(word: &[u8]) -> Option<Vec<u8>> {
    let mut bytes = Vec::with_capacity(word.len());
    let mut rest = word;
    while let Some((&byte, tail)) = rest.split_first() {
        rest = tail;
        if byte != b'%' {
            bytes.push(byte);
            continue;
        }
        let digits = core::str::from_utf8(rest.get(..2)?).ok()?;
        bytes.push(u8::from_str_radix(digits, 16).ok()?);
        rest = &rest[2..];
    }
    Some(bytes)
}

/// Overwrite a decoded secret before its memory is freed.
pub fn wipe(bytes: &mut [u8]) {
    for byte in bytes.iter_mut() {
        // SAFETY: a byte of a buffer this function borrows exclusively.
        unsafe { core::ptr::write_volatile(byte, 0) };
    }
}

/// The network lines of `file`, each with its name decoded: blank, comment
/// and unnamed lines left out; the secret is not decoded.
fn named_lines(file: &[u8]) -> impl Iterator<Item = (&[u8], Option<Vec<u8>>)> {
    file.split(|&byte| byte == b'\n').filter_map(|line| {
        let trimmed = line.strip_suffix(b"\r").unwrap_or(line).trim_ascii();
        if trimmed.is_empty() || trimmed[0] == b'#' {
            return None;
        }
        let name = trimmed
            .split(|&byte| byte == b' ' || byte == b'\t')
            .find(|word| !word.is_empty())
            .and_then(decode);
        Some((line, name))
    })
}

/// The names `file` saves, in order, for the startup menu's Wi-Fi page: a
/// name each line holds, whether or not its secret is usable, its secret
/// never decoded; each name once.
pub fn names(file: &[u8]) -> Vec<Vec<u8>> {
    listing(file)
        .into_iter()
        .map(|listed| listed.name)
        .collect()
}

/// One saved network as the startup menu's Wi-Fi page lists it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Listed {
    /// One to [`SSID_BYTES`] bytes, decoded.
    pub name: Vec<u8>,
    /// A line of it is one usable network ([`parse`]); its secret is
    /// decoded to tell, and wiped, as [`parse`] does.
    pub usable: bool,
}

/// The networks `file` saves, in order, each once: the names [`names`]
/// gives, each with whether a line of it is one a boot can join.
pub fn listing(file: &[u8]) -> Vec<Listed> {
    let mut listed: Vec<Listed> = Vec::new();
    for (line, name) in named_lines(file) {
        let Some(name) = name.filter(|name| (1..=SSID_BYTES).contains(&name.len())) else {
            continue;
        };
        let line = line.strip_suffix(b"\r").unwrap_or(line).trim_ascii();
        let usable = network(line).is_some();
        match listed.iter_mut().find(|known| known.name == name) {
            Some(known) => known.usable |= usable,
            None => listed.push(Listed { name, usable }),
        }
    }
    listed
}

/// `file` without the lines of the network `ssid`, as the menu forgets it
/// or saves it again; every other line, comments included, as it was.
pub fn without(file: &[u8], ssid: &[u8]) -> Vec<u8> {
    let mut kept = Vec::with_capacity(file.len());
    for line in file.split(|&byte| byte == b'\n') {
        let named = named_lines(line).next().and_then(|(_, name)| name);
        if named.as_deref() == Some(ssid) || line.is_empty() {
            continue;
        }
        kept.extend_from_slice(line);
        kept.push(b'\n');
    }
    kept
}

/// The line that saves `ssid` with `secret`, both written as `--wifi` words
/// are: `%`, whitespace, control and non-ASCII bytes as `%XX`.
pub fn line(ssid: &[u8], secret: &[u8]) -> Vec<u8> {
    let mut line = Vec::with_capacity(3 * (ssid.len() + secret.len()) + 2);
    for (n, word) in [ssid, secret].into_iter().enumerate() {
        if n == 1 {
            line.push(b' ');
        }
        for &byte in word {
            if byte == b'%' || byte <= b' ' || byte >= 0x7f {
                line.extend_from_slice(alloc::format!("%{byte:02X}").as_bytes());
            } else {
                line.push(byte);
            }
        }
    }
    line.push(b'\n');
    line
}

#[cfg(test)]
mod tests {
    use super::*;

    fn saved_names(saved: &Saved) -> Vec<&[u8]> {
        saved
            .networks
            .iter()
            .map(|network| &network.ssid[..])
            .collect()
    }

    #[test]
    fn each_line_saves_a_name_and_a_secret_in_order() {
        let saved = parse(b"GarvitRoom_5G hunter2hunter2\r\nMom%27s%20phone pass%20word%2512\n");
        assert_eq!(saved.unusable, 0);
        assert_eq!(
            saved_names(&saved),
            [&b"GarvitRoom_5G"[..], &b"Mom's phone"[..]]
        );
        let Secret::Passphrase(phrase) = &saved.networks[1].secret else {
            panic!("passphrase expected");
        };
        assert_eq!(phrase.as_bytes(), b"pass word%12");
    }

    #[test]
    fn blank_and_comment_lines_are_skipped_and_tabs_separate() {
        let saved = parse(b"\n# home networks\n   \n\tcafe\t\tcoffee-time  \n");
        assert_eq!(saved.unusable, 0);
        assert_eq!(saved_names(&saved), [&b"cafe"[..]]);
    }

    #[test]
    fn a_64_hex_digit_secret_is_a_psk() {
        let line = alloc::format!("lab {}\n", "ab".repeat(32));
        let saved = parse(line.as_bytes());
        assert!(matches!(saved.networks[0].secret, Secret::Psk(_)));
    }

    #[test]
    fn unusable_lines_are_counted_and_the_rest_kept() {
        let long = "n".repeat(33);
        let text = alloc::format!(
            "one-word\nshort short\ngood password1\n{long} password1\nthree words here\nbad%zz password1\nbad password%2\n"
        );
        let saved = parse(text.as_bytes());
        assert_eq!(saved_names(&saved), [&b"good"[..]]);
        assert_eq!(saved.unusable, 6);
    }

    #[test]
    fn the_menu_lists_names_without_their_secrets_and_each_once() {
        let file = b"# mine\nhome secret-one\nMom%27s%20phone bad\nhome secret-two\n\n";
        assert_eq!(
            super::names(file),
            [b"home".to_vec(), b"Mom's phone".to_vec()]
        );
    }

    #[test]
    fn the_listing_tells_which_saved_keys_a_boot_can_use() {
        let file = b"# mine\nhome bad\ncafe coffee-time\nhome secret-two\nlab short\n";
        let listed = listing(file);
        let names: Vec<(&[u8], bool)> = listed
            .iter()
            .map(|listed| (&listed.name[..], listed.usable))
            .collect();
        assert_eq!(
            names,
            [
                (&b"home"[..], true),
                (&b"cafe"[..], true),
                (&b"lab"[..], false)
            ]
        );
    }

    #[test]
    fn a_password_with_spaces_and_percent_signs_round_trips() {
        let written = line(b"Cafe 5G", b"100% pass word");
        let saved = parse(&written);
        assert_eq!(saved.unusable, 0);
        let Secret::Passphrase(phrase) = &saved.networks[0].secret else {
            panic!("passphrase expected");
        };
        assert_eq!(phrase.as_bytes(), b"100% pass word");
        assert_eq!(listing(&written)[0].name, b"Cafe 5G".to_vec());
    }

    #[test]
    fn forgetting_a_network_keeps_every_other_line() {
        let file = b"# mine\nhome secret-one\r\ncafe coffee-time\nhome again-here\n";
        assert_eq!(
            without(file, b"home"),
            b"# mine\ncafe coffee-time\n".to_vec()
        );
        assert_eq!(without(file, b"elsewhere"), file.to_vec());
    }

    #[test]
    fn a_saved_line_reads_back_as_its_network() {
        let written = line(b"Mom's phone", b"pass word%12");
        assert_eq!(written, b"Mom's%20phone pass%20word%2512\n".to_vec());
        let saved = parse(&written);
        assert_eq!(saved_names(&saved), [&b"Mom's phone"[..]]);
        let Secret::Passphrase(phrase) = &saved.networks[0].secret else {
            panic!("passphrase expected");
        };
        assert_eq!(phrase.as_bytes(), b"pass word%12");
    }

    #[test]
    fn an_empty_file_saves_nothing() {
        let saved = parse(b"");
        assert!(saved.networks.is_empty());
        assert_eq!(saved.unusable, 0);
    }
}

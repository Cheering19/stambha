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
fn wipe(bytes: &mut [u8]) {
    for byte in bytes.iter_mut() {
        // SAFETY: a byte of a buffer this function borrows exclusively.
        unsafe { core::ptr::write_volatile(byte, 0) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn names(saved: &Saved) -> Vec<&[u8]> {
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
        assert_eq!(names(&saved), [&b"GarvitRoom_5G"[..], &b"Mom's phone"[..]]);
        let Secret::Passphrase(phrase) = &saved.networks[1].secret else {
            panic!("passphrase expected");
        };
        assert_eq!(phrase.as_bytes(), b"pass word%12");
    }

    #[test]
    fn blank_and_comment_lines_are_skipped_and_tabs_separate() {
        let saved = parse(b"\n# home networks\n   \n\tcafe\t\tcoffee-time  \n");
        assert_eq!(saved.unusable, 0);
        assert_eq!(names(&saved), [&b"cafe"[..]]);
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
        assert_eq!(names(&saved), [&b"good"[..]]);
        assert_eq!(saved.unusable, 6);
    }

    #[test]
    fn an_empty_file_saves_nothing() {
        let saved = parse(b"");
        assert!(saved.networks.is_empty());
        assert_eq!(saved.unusable, 0);
    }
}

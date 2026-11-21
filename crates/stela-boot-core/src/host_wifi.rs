//! What the host check's `--wifi` station check (M3 spec S17) reads and
//! prints: the EAPOL counts and refusal in an rtw89 radio's progress line
//! (spec S14.6), every MAC address in text it prints masked (S17.5), and
//! its `stela.cfg` line with the network's name left out.
use crate::options::WIFI;
use alloc::string::String;

/// What a masked address reads as.
const MASKED: &str = "xx:xx:xx:xx:xx:xx";

/// Whether `bytes` is six `:`-separated pairs of hex digits, either case.
fn is_address(bytes: &[u8]) -> bool {
    bytes.len() == MASKED.len()
        && bytes.chunks(3).enumerate().all(|(i, part)| {
            part[..2].iter().all(u8::is_ascii_hexdigit) && (i == 5 || part[2] == b':')
        })
}

/// `text` with each `hh:hh:hh:hh:hh:hh` (hex digits in either case) written
/// `xx:xx:xx:xx:xx:xx`, left to right, and everything else kept.
pub fn mask_addresses(text: &str) -> String {
    let bytes = text.as_bytes();
    let mut masked = String::with_capacity(text.len());
    let mut at = 0;
    while at < bytes.len() {
        if bytes.get(at..at + MASKED.len()).is_some_and(is_address) {
            masked.push_str(MASKED);
            at += MASKED.len();
            continue;
        }
        // An address is ASCII, so any other character is copied whole.
        let next = text[at..].chars().next().map_or(1, char::len_utf8);
        masked.push_str(&text[at..at + next]);
        at += next;
    }
    masked
}

/// `text`, a `stela.cfg` line, with the word after each `--wifi` (the
/// network's name) written `<ssid>` and every word separated by one space;
/// a line without `--wifi` comes back as it was written.
pub fn hide_network(text: &str) -> String {
    let mut words = text.split_ascii_whitespace();
    if !text.split_ascii_whitespace().any(|word| word == WIFI) {
        return String::from(text);
    }
    let mut line = String::with_capacity(text.len());
    while let Some(word) = words.next() {
        if !line.is_empty() {
            line.push(' ');
        }
        line.push_str(word);
        if word == WIFI && words.next().is_some() {
            line.push_str(" <ssid>");
        }
    }
    line
}

/// The EAPOL frames a radio's progress line counts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Eapol {
    pub received: u32,
    pub sent: u32,
    pub acked: u32,
}

/// What the station check needs of an rtw89 radio's progress line (M3 spec
/// S14.6): its EAPOL counts, once the handshake began, and whether the
/// radio recorded a refusal of its own (`last "…"`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Progress {
    pub eapol: Option<Eapol>,
    pub last: bool,
}

impl Progress {
    /// Read `line`; a field it does not hold, or holds malformed, reads as
    /// not reached.
    pub fn read(line: &str) -> Self {
        Self {
            eapol: eapol(line),
            last: line.contains(", last \""),
        }
    }

    /// M3a's stop (spec S17.3): messages 1 and 3 received, 2 and 4 sent,
    /// and the radio's refusal recorded, which in M3a is the first key's.
    pub fn handshaken(&self) -> bool {
        self.last
            && self
                .eapol
                .is_some_and(|eapol| eapol.received >= 2 && eapol.sent >= 2)
    }
}

/// How long the station check keeps the keeper going for the handshake
/// (`WIFI_STATION_BUDGET_US`, M3 plan Global Constraints).
pub const STATION_BUDGET_US: u64 = 60_000_000;

/// How long the station check keeps polling after the handshake, so m4's
/// release report is counted, and again after its deauthentication, so the
/// frame leaves the ring before the chip stops: the longest
/// `rtw89_mac_flush_txq` waits for the queues to empty (200 ms,
/// `mac.c:6352-6361`, from `rtw89_ops_flush`, `mac80211.c:1122-1135`).
pub const FLUSH_US: u64 = 200_000;

/// The station check's stop: polling goes on until the handshake
/// ([`Progress::handshaken`]) and [`FLUSH_US`] after it, or until
/// [`STATION_BUDGET_US`] without it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Watch {
    start_us: u64,
    handshaken_us: Option<u64>,
}

impl Watch {
    /// A watch begun at `now_us`.
    pub fn new(now_us: u64) -> Self {
        Self {
            start_us: now_us,
            handshaken_us: None,
        }
    }

    /// After one poll at `now_us` whose progress line read `progress`:
    /// `None` to poll on, `Some(true)` once the flush after the handshake
    /// passed, `Some(false)` once the budget passed without it. A
    /// handshake reached is kept, whatever later lines read.
    pub fn after(&mut self, now_us: u64, progress: &Progress) -> Option<bool> {
        if self.handshaken_us.is_none() && progress.handshaken() {
            self.handshaken_us = Some(now_us);
        }
        match self.handshaken_us {
            Some(at) => (now_us.saturating_sub(at) >= FLUSH_US).then_some(true),
            None => (now_us.saturating_sub(self.start_us) >= STATION_BUDGET_US).then_some(false),
        }
    }
}

/// `eapol rx <r> tx <t> acked <k>` in `line`.
fn eapol(line: &str) -> Option<Eapol> {
    let (_, rest) = line.split_once("eapol rx ")?;
    let mut words = rest.split([' ', ',', ';']);
    let mut number = |label: Option<&str>| {
        if let Some(label) = label
            && words.next() != Some(label)
        {
            return None;
        }
        words.next()?.parse().ok()
    };
    Some(Eapol {
        received: number(None)?,
        sent: number(Some("tx"))?,
        acked: number(Some("acked"))?,
    })
}

#[cfg(test)]
mod tests;

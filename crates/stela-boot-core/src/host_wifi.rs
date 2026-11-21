//! What the host check's `--wifi` station check (M3 spec S17) reads and
//! prints: the EAPOL counts and refusal in an rtw89 radio's progress line
//! (spec S14.6), every MAC address in text it prints masked (S17.5), and
//! its `stela.cfg` line and its own line with the network's name left out.
use crate::options::WIFI;
use alloc::string::String;
use stela_drivers::net::wifi::realtek::ax::live::KEYS_BEFORE_M3B;

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

/// What a hidden network name reads as.
const HIDDEN: &str = "<ssid>";

/// `text` with the network's name `ssid` written `<ssid>` wherever it
/// occurs, byte for byte, as `String::from_utf8_lossy` writes it (the
/// connection keeper's failures name the network so): occurrences that
/// overlap are hidden as one, and a name that is also part of other words
/// is hidden there too, so the line gives up legibility, never the name. A
/// `<ssid>` already in `text` stays as it is, even for a name that is part
/// of it, so hiding twice changes nothing; an empty name hides nothing.
pub fn hide_ssid(text: &str, ssid: &[u8]) -> String {
    let name = String::from_utf8_lossy(ssid);
    let (name, bytes) = (name.as_bytes(), text.as_bytes());
    if name.is_empty() || name.len() > bytes.len() {
        return String::from(text);
    }
    // Whether `bytes[start..end]` lies inside a `<ssid>` of `text`.
    let placed = |start: usize, end: usize| {
        (start.saturating_sub(HIDDEN.len())..=start)
            .any(|at| at + HIDDEN.len() >= end && bytes[at..].starts_with(HIDDEN.as_bytes()))
    };
    let mut hidden = String::with_capacity(text.len());
    // `text[..copied]` is in `hidden`, written out or hidden.
    let mut copied = 0;
    // The occurrences being hidden as one: `text[start..end]`.
    let mut run: Option<(usize, usize)> = None;
    let mut flush = |hidden: &mut String, (start, end): (usize, usize)| {
        // An occurrence of UTF-8 in UTF-8 starts and ends on characters.
        hidden.push_str(&text[copied..start]);
        hidden.push_str(HIDDEN);
        copied = end;
    };
    for start in 0..=bytes.len() - name.len() {
        let end = start + name.len();
        if !bytes[start..].starts_with(name) || placed(start, end) {
            continue;
        }
        match &mut run {
            Some((_, until)) if start < *until => *until = end,
            _ => {
                if let Some(done) = run.replace((start, end)) {
                    flush(&mut hidden, done);
                }
            }
        }
    }
    if let Some(done) = run {
        flush(&mut hidden, done);
    }
    hidden.push_str(&text[copied..]);
    hidden
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
/// radio's last refusal of its own is M3a's, the first key's.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Progress {
    pub eapol: Option<Eapol>,
    /// The station's part of the line (before its first `; `, the chip's
    /// counts) ends `, last "keys before M3b"` ([`KEYS_BEFORE_M3B`]).
    pub keys_refused: bool,
}

impl Progress {
    /// Read `line`; a field it does not hold, or holds malformed, reads as
    /// not reached.
    pub fn read(line: &str) -> Self {
        let station = line.split_once("; ").map_or(line, |(station, _)| station);
        let keys_refused = station
            .strip_suffix('"')
            .and_then(|rest| rest.strip_suffix(KEYS_BEFORE_M3B))
            .is_some_and(|rest| rest.ends_with(", last \""));
        Self {
            eapol: eapol(line),
            keys_refused,
        }
    }

    /// M3a's stop (spec S17.3): messages 1 and 3 received, 2 and 4 sent,
    /// and the radio's last refusal the first key's.
    pub fn handshaken(&self) -> bool {
        self.keys_refused
            && self
                .eapol
                .is_some_and(|eapol| eapol.received >= 2 && eapol.sent >= 2)
    }
}

/// How long the station check keeps the keeper going for the handshake
/// (`WIFI_STATION_BUDGET_US`, M3 plan Global Constraints).
pub const STATION_BUDGET_US: u64 = 60_000_000;

/// How long before `--deadline` the station check stops without the
/// handshake, so that its teardown (the flush after the handshake, the
/// deauthentication's and the chip's stop) ends before the deadline stops
/// the host check with the chip still up.
pub const DEADLINE_MARGIN_US: u64 = 10_000_000;
const _: () = assert!(DEADLINE_MARGIN_US > 10 * FLUSH_US);

/// How long the station check keeps polling after the handshake, so m4's
/// release report is counted, and again after its deauthentication, so the
/// frame leaves the ring before the chip stops: the longest
/// `rtw89_mac_flush_txq` waits for the queues to empty (200 ms,
/// `mac.c:6352-6361`, from `rtw89_ops_flush`, `mac80211.c:1122-1135`).
pub const FLUSH_US: u64 = 200_000;

/// Why the station check stopped polling ([`Watch::after`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum End {
    /// The handshake ran, and [`FLUSH_US`] passed after it.
    Handshaken,
    /// [`STATION_BUDGET_US`] passed without the handshake.
    Budget,
    /// The deadline came within [`DEADLINE_MARGIN_US`] first.
    Deadline,
}

/// The station check's stop: polling goes on until the handshake
/// ([`Progress::handshaken`]) and [`FLUSH_US`] after it, or without it
/// until [`STATION_BUDGET_US`] or [`DEADLINE_MARGIN_US`] before the
/// deadline, whichever comes first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Watch {
    start_us: u64,
    /// [`DEADLINE_MARGIN_US`] before the deadline, when that comes before
    /// the budget ends.
    cut_us: Option<u64>,
    handshaken_us: Option<u64>,
}

impl Watch {
    /// A watch begun at `now_us`, with the deadline at `deadline_us` on the
    /// same clock when there is one.
    pub fn new(now_us: u64, deadline_us: Option<u64>) -> Self {
        let budget_us = now_us.saturating_add(STATION_BUDGET_US);
        Self {
            start_us: now_us,
            cut_us: deadline_us
                .map(|deadline| deadline.saturating_sub(DEADLINE_MARGIN_US))
                .filter(|&cut| cut < budget_us),
            handshaken_us: None,
        }
    }

    /// After one poll at `now_us` whose progress line read `progress`:
    /// `None` to poll on, else why to stop. A handshake reached is kept,
    /// whatever later lines read, and gets its flush even past the cut,
    /// which the margin holds.
    pub fn after(&mut self, now_us: u64, progress: &Progress) -> Option<End> {
        if self.handshaken_us.is_none() && progress.handshaken() {
            self.handshaken_us = Some(now_us);
        }
        if let Some(at) = self.handshaken_us {
            return (now_us.saturating_sub(at) >= FLUSH_US).then_some(End::Handshaken);
        }
        if self.cut_us.is_some_and(|cut| now_us >= cut) {
            return Some(End::Deadline);
        }
        (now_us.saturating_sub(self.start_us) >= STATION_BUDGET_US).then_some(End::Budget)
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

//! What the host check's `--wifi` station check (M3 spec S17) reads and
//! prints: the EAPOL counts, keys and port in an rtw89 radio's progress
//! line (spec S14.6), when the network stack's wait for a DHCP lease is
//! cut, the stack's failures, the line's ending (M3b's "Done means"),
//! every MAC address in text it prints masked (S17.5), and its `stela.cfg`
//! line and its own line with the network's name left out.
use crate::options::WIFI;
use alloc::string::String;
use core::{fmt, net::Ipv4Addr};
use stela_drivers::net::wifi::realtek::ax::WIFI_CARRIER_WAIT_US;

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
/// S14.6), read from the station's own fields: before the first `; ` (the
/// chip's counts follow it) and before `, last "` (the radio's last
/// refusal, which could read as anything). The authentication's field,
/// `auth t/a status s` or `sae t/a`, and the data, beacon loss, Block Ack
/// and PN counts are printed, not judged.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Progress {
    /// Once the association was configured.
    pub eapol: Option<Eapol>,
    /// The keys installed, once the first install began (`keys -` before).
    pub keys: Option<u32>,
    pub port_open: bool,
}

impl Progress {
    /// Read `line`; a field it does not hold, or holds malformed, reads as
    /// not reached.
    pub fn read(line: &str) -> Self {
        let station = line.split_once("; ").map_or(line, |(station, _)| station);
        let station = station
            .split_once(", last \"")
            .map_or(station, |(fields, _)| fields);
        let fields = || station.split(", ");
        Self {
            eapol: eapol(station),
            keys: fields()
                .find_map(|field| field.strip_prefix("keys "))
                .and_then(|keys| keys.parse().ok()),
            port_open: fields().any(|field| field == "port open"),
        }
    }

    /// M3b's (spec "Done means"): the pairwise and group keys in the
    /// hardware and the port open, `keys 2, port open`; a rekey counts on.
    pub fn keyed(&self) -> bool {
        self.port_open && self.keys.is_some_and(|keys| keys >= 2)
    }
}

/// `stela_runtime`'s `DHCP_WAIT_US`: how long the network stack's `dhcp`
/// waits for a lease once the link has carrier.
pub const DHCP_WAIT_US: u64 = 60_000_000;

/// How long the station check lets the network stack go for a lease (M3
/// spec S17.3): its wait for the radio's carrier ([`WIFI_CARRIER_WAIT_US`],
/// the scan, authentication and 4-way handshake) and then for the lease
/// ([`DHCP_WAIT_US`]). The stack is driven again past a failure of its own
/// (the keeper's failures reach it as the link's) until this passes.
pub const STATION_BUDGET_US: u64 = WIFI_CARRIER_WAIT_US + DHCP_WAIT_US;

/// How long before `--deadline` the station check stops without a lease,
/// so that its teardown (the deauthentication, its flush and the chip's
/// stop) ends before the deadline stops the host check with the chip up.
pub const DEADLINE_MARGIN_US: u64 = 10_000_000;
const _: () = assert!(DEADLINE_MARGIN_US > 10 * FLUSH_US);

/// How long the station check keeps polling after its deauthentication,
/// so the frame leaves the ring before the chip stops: the longest
/// `rtw89_mac_flush_txq` waits for the queues to empty (200 ms,
/// `mac.c:6352-6361`, from `rtw89_ops_flush`, `mac80211.c:1122-1135`).
pub const FLUSH_US: u64 = 200_000;

/// Which bound cut the network stack's wait ([`Cut`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum End {
    /// [`STATION_BUDGET_US`] passed.
    Budget,
    /// The deadline came within [`DEADLINE_MARGIN_US`] first.
    Deadline,
}

/// When the station check stops waiting for a lease: [`STATION_BUDGET_US`]
/// after it began, or [`DEADLINE_MARGIN_US`] before `--deadline` when that
/// comes first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cut {
    at_us: u64,
    end: End,
}

impl Cut {
    /// The cut of a wait begun at `now_us`, with the deadline at
    /// `deadline_us` on the same clock when there is one.
    pub fn new(now_us: u64, deadline_us: Option<u64>) -> Self {
        let budget_us = now_us.saturating_add(STATION_BUDGET_US);
        match deadline_us
            .map(|deadline| deadline.saturating_sub(DEADLINE_MARGIN_US))
            .filter(|&cut| cut < budget_us)
        {
            Some(at_us) => Self {
                at_us,
                end: End::Deadline,
            },
            None => Self {
                at_us: budget_us,
                end: End::Budget,
            },
        }
    }

    /// Whether the wait is over at `now_us`.
    pub fn reached(&self, now_us: u64) -> bool {
        now_us >= self.at_us
    }

    pub fn end(&self) -> End {
        self.end
    }
}

/// The network stack's failures within the wait, which the check drives
/// the stack on past: how many, the first and the last.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Failures {
    count: u32,
    first: Option<String>,
    last: Option<String>,
}

impl Failures {
    pub fn note(&mut self, failure: String) {
        self.count = self.count.saturating_add(1);
        if self.first.is_none() {
            self.first = Some(failure);
        } else {
            self.last = Some(failure);
        }
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
}

/// Nothing without a failure; else `stack failed once: <first>`, `stack
/// failed <n> times: <first>` when the last was the first's words, or
/// `stack failed <n> times, first: <first>, last: <last>`.
impl fmt::Display for Failures {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let Some(first) = &self.first else {
            return Ok(());
        };
        match &self.last {
            None => write!(f, "stack failed once: {first}"),
            Some(last) if last == first => write!(f, "stack failed {} times: {first}", self.count),
            Some(last) => write!(
                f,
                "stack failed {} times, first: {first}, last: {last}",
                self.count
            ),
        }
    }
}

/// A DHCPv4 lease, `after_us` after the station began.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Lease {
    pub address: Ipv4Addr,
    pub prefix: u8,
    pub router: Option<Ipv4Addr>,
    pub after_us: u64,
}

/// How the network stack's wait ended.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    Leased(Lease),
    /// Cut without a lease ([`Cut`]).
    Cut(End),
    /// The stack failed in a way driving it again cannot mend.
    Failed(String),
}

/// The station check's verdict and the line's ending after the progress
/// (M3 spec "Done means", M3b): a pass is a lease with
/// [`Progress::keyed`], `; DHCP lease <a.b.c.d>/<p> router <r|-> after <s>
/// s`; anything else fails and says which way.
pub fn ending(outcome: &Outcome, progress: &Progress) -> (bool, String) {
    match outcome {
        Outcome::Leased(lease) => {
            let router = lease
                .router
                .map_or_else(|| String::from("-"), |router| alloc::format!("{router}"));
            let line = alloc::format!(
                "; DHCP lease {}/{} router {router} after {} s",
                lease.address,
                lease.prefix,
                lease.after_us / 1_000_000
            );
            if progress.keyed() {
                (true, line)
            } else {
                (
                    false,
                    alloc::format!("{line}, but not \"keys 2, port open\""),
                )
            }
        }
        Outcome::Cut(End::Budget) => (
            false,
            alloc::format!("; no DHCP lease within {} s", STATION_BUDGET_US / 1_000_000),
        ),
        Outcome::Cut(End::Deadline) => (
            false,
            alloc::format!(
                "; stopped {} s before --deadline without a DHCP lease",
                DEADLINE_MARGIN_US / 1_000_000
            ),
        ),
        Outcome::Failed(why) => (false, alloc::format!("; no DHCP lease: {why}")),
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

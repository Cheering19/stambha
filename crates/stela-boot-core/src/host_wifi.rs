//! What the host check's `--wifi` station check (M3 spec S17) reads and
//! prints: the EAPOL counts, keys and port in an rtw89 radio's progress
//! line (spec S14.6), when the network stack's wait for a DHCP lease is
//! cut, the stack's failures, the line's ending (M3b's "Done means"),
//! every MAC address in text it prints masked (S17.5), and its `stela.cfg`
//! line and its own line with the network's name left out; then the
//! `--network-check` POST's verdict and line ([`network_check`]).
/// The network's name hidden as a normal boot hides it in what a Wi-Fi
/// adapter's link says.
pub use crate::network_link::hide_ssid;
use crate::options::WIFI;
use alloc::string::String;
use core::{fmt, net::Ipv4Addr};
use stela_drivers::net::wifi::realtek::ax::WIFI_CARRIER_WAIT_US;
use stela_runtime::net::NetError;
/// How long the network stack's `dhcp` waits for a lease once the link has
/// carrier: the stack's own.
pub use stela_runtime::net::stack::DHCP_WAIT_US;

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

/// How long the station check lets the network stack go for a lease (M3
/// spec S17.3): its wait for the radio's carrier ([`WIFI_CARRIER_WAIT_US`],
/// the scan, authentication and 4-way handshake) and then for the lease
/// ([`DHCP_WAIT_US`]). The stack is driven again past a failure of its own
/// (the keeper's failures reach it as the link's) until this passes. The
/// `--network-check` POST that follows a lease gets what is left of it.
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

/// What an address that may name the owner's network beyond the LAN reads
/// as.
const PUBLIC: &str = "<public>";

/// An IPv4 address as the line prints it: itself when it names no network
/// beyond the owner's own, a private (RFC 1918), link-local (RFC 3927) or
/// shared (RFC 6598, 100.64.0.0/10) one, else `<public>`.
pub struct Shown(pub Ipv4Addr);

impl fmt::Display for Shown {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [first, second, ..] = self.0.octets();
        let shared = first == 100 && second & 0xc0 == 0x40;
        match self.0.is_private() || self.0.is_link_local() || shared {
            true => write!(f, "{}", self.0),
            false => f.write_str(PUBLIC),
        }
    }
}

/// `account`, the network stack's DHCP account (`Stack::dhcp_summary`),
/// with the address after each `server=` and `router=` (the lease's) as
/// [`Shown`] writes it; anything else, a word that is no address among it,
/// as it was.
pub fn hide_public(account: &str) -> String {
    let mut shown = String::with_capacity(account.len());
    let mut rest = account;
    while let Some(at) = ["server=", "router="]
        .iter()
        .filter_map(|name| rest.find(name).map(|at| at + name.len()))
        .min()
    {
        shown.push_str(&rest[..at]);
        rest = &rest[at..];
        let end = rest
            .find(|c: char| !(c.is_ascii_digit() || c == '.'))
            .unwrap_or(rest.len());
        match rest[..end].parse::<Ipv4Addr>() {
            Ok(address) => shown.push_str(&alloc::format!("{}", Shown(address))),
            Err(_) => shown.push_str(&rest[..end]),
        }
        rest = &rest[end..];
    }
    shown.push_str(rest);
    shown
}

/// `text` with every IPv4 address in it, a run of digits and dots that
/// reads as one once the dots at its ends are left aside, as [`Shown`]
/// writes it; anything else as it was.
pub fn hide_public_addresses(text: &str) -> String {
    let part = |c: char| c.is_ascii_digit() || c == '.';
    let mut shown = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(start) = rest.find(part) {
        shown.push_str(&rest[..start]);
        rest = &rest[start..];
        let end = rest.find(|c: char| !part(c)).unwrap_or(rest.len());
        let run = &rest[..end];
        let word = run.trim_matches('.');
        let lead = run.len() - run.trim_start_matches('.').len();
        match word.parse::<Ipv4Addr>() {
            Ok(address) => {
                shown.push_str(&run[..lead]);
                shown.push_str(&alloc::format!("{}", Shown(address)));
                shown.push_str(&run[lead + word.len()..]);
            }
            Err(_) => shown.push_str(run),
        }
        rest = &rest[end..];
    }
    shown.push_str(rest);
    shown
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
/// s`, each address as [`Shown`] writes it; anything else fails and says
/// which way.
pub fn ending(outcome: &Outcome, progress: &Progress) -> (bool, String) {
    match outcome {
        Outcome::Leased(lease) => {
            let router = lease.router.map_or_else(
                || String::from("-"),
                |router| alloc::format!("{}", Shown(router)),
            );
            let line = alloc::format!(
                "; DHCP lease {}/{} router {router} after {} s",
                Shown(lease.address),
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

/// How the network check's POST (`--network-check`), made over the station
/// once it has its lease, ended.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Post {
    /// The network stack's `post_url`: the 2xx answer's body, or how it
    /// failed, `after_us` after the POST began.
    Ended {
        result: Result<alloc::vec::Vec<u8>, NetError>,
        after_us: u64,
    },
    /// The station check's cut ([`Cut`]) came first.
    Cut(End),
}

/// The network check's verdict and its `CHECK network` line for a POST to
/// `url` (`post`; none when the station check got no lease), judged as a
/// normal boot judges its `--network-check`: any HTTP answer proves the
/// path end to end and passes, a 2xx body given by its length and SHA-256
/// as the normal boot prints it (`NETWORK CHECK body bytes=… sha256=…`);
/// only a transport failure (the link, DNS, the connection, TLS, a reply
/// malformed or asking for a login) fails, in the stack's words, as does
/// the station check's cut. The network's name `ssid` is hidden, then
/// every MAC address masked and every public IPv4 address written
/// `<public>` ([`hide_public_addresses`]): the server's, and the network
/// and AP a keeper failure reaching the POST as the link's names.
pub fn network_check(url: &str, post: Option<&Post>, ssid: &[u8]) -> (bool, String) {
    let (passed, line) = match post {
        None => (
            false,
            String::from("not run: the station check got no DHCP lease (see CHECK wifi)"),
        ),
        Some(Post::Ended { result, after_us }) => {
            let ms = after_us / 1_000;
            match result {
                Ok(body) => (
                    true,
                    alloc::format!(
                        "POST {url} answered 2xx in {ms} ms: body bytes={} sha256={}",
                        body.len(),
                        stela_formats::sha256(body)
                    ),
                ),
                Err(NetError::Status(status)) => (
                    true,
                    alloc::format!(
                        "POST {url} answered HTTP {status} in {ms} ms, which proves the path"
                    ),
                ),
                Err(error) => (
                    false,
                    alloc::format!("POST {url} failed after {ms} ms: {error}"),
                ),
            }
        }
        Some(Post::Cut(End::Budget)) => (
            false,
            alloc::format!(
                "POST {url}: no answer within the station check's {} s",
                STATION_BUDGET_US / 1_000_000
            ),
        ),
        Some(Post::Cut(End::Deadline)) => (
            false,
            alloc::format!(
                "POST {url}: stopped {} s before --deadline without an answer",
                DEADLINE_MARGIN_US / 1_000_000
            ),
        ),
    };
    let line = hide_public_addresses(&mask_addresses(&hide_ssid(&line, ssid)));
    (passed, line)
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

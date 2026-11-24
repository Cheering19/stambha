//! Whether the startup menu opens the Wi-Fi page by itself, and the record
//! a boot leaves for the next one's menu to decide it by.
//!
//! On a PC with a Wi-Fi chip the driver starts, a boot with no network to
//! join opens the page (`stela-boot`'s `wifi_menu`) before the menu, and a
//! boot after one whose join failed opens it on that network with its
//! password field ready. Neither ever holds a boot that has to run without
//! anyone at the PC: the one after a restart macOS asked for (the
//! installer's restarts, an update's, any boot an install chains into), one
//! after a stop or a boot that never ended (a crash, a hang, power lost),
//! one whose earlier boot had a network up, a menu that holds for setup or
//! opens with no menu at all, and one whose earlier boot's log could not be
//! read. A console with no input leaves the page at once.
//!
//! The record is the boot log's own lines ([`RECORD`]), read back from the
//! log on the USB as the earlier boots' device-start record is
//! (`stela-boot`'s `log_file`). They name a network only by where it is
//! set: the `--wifi` one ([`Which::Configured`]) or the saved one of that
//! place in `wifi.saved` ([`Which::Saved`]), never by its name; the menu
//! shows the name, from those files.
use alloc::{format, string::String};

/// What each record line starts with, after the log's time stamp.
pub const RECORD: &str = "WIFI RECORD ";
/// The marker line each boot's log starts with
/// (`stela_runtime::pc::device_starts::BOOT`).
pub const BOOT: &str = "=== STELA BOOT ";
/// The lines a boot ends with (`stela-boot`'s `engine::power` and
/// `output::terminate`).
const RESTART: &str = "STELA RESTART code=";
const POWER_OFF: &str = "STELA POWER-OFF code=";
const STOPPED: &str = "STELA STOPPED code=";

/// A network, by where it is set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Which {
    /// The startup entry's `--wifi` network.
    Configured,
    /// The usable network of this place in `wifi.saved`, from 1
    /// (`wifi_saved::parse`'s order).
    Saved(usize),
}

impl Which {
    fn word(self) -> String {
        match self {
            Self::Configured => String::from("configured"),
            Self::Saved(place) => format!("saved-{place}"),
        }
    }

    fn parse(word: &str) -> Option<Self> {
        if word == "configured" {
            return Some(Self::Configured);
        }
        let place: usize = word.strip_prefix("saved-")?.parse().ok()?;
        (place != 0).then_some(Self::Saved(place))
    }
}

/// Why a join failed, as far as the driver tells.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cause {
    /// The handshake failed: the password may be wrong.
    Password,
    /// No scan found the network.
    Range,
}

impl Cause {
    fn word(self) -> &'static str {
        match self {
            Self::Password => "password",
            Self::Range => "range",
        }
    }

    fn parse(word: &str) -> Option<Self> {
        match word {
            "password" => Some(Self::Password),
            "range" => Some(Self::Range),
            _ => None,
        }
    }
}

/// What a boot's Wi-Fi came to, or what the menu did to it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Record {
    Joined(Which),
    Failed(Which, Cause),
    /// A wired adapter's link came up.
    Wired,
    /// The page saved or forgot a network: an earlier failure is no longer
    /// the network's.
    Cleared,
}

impl Record {
    /// The log line, without the time stamp the log adds.
    pub fn line(self) -> String {
        let what = match self {
            Self::Joined(which) => format!("joined network={}", which.word()),
            Self::Failed(which, cause) => {
                format!("failed network={} cause={}", which.word(), cause.word())
            }
            Self::Wired => String::from("wired up"),
            Self::Cleared => String::from("cleared at the menu"),
        };
        format!("{RECORD}{what}\n")
    }

    /// The record in one log line, wherever in it [`RECORD`] starts.
    fn parse(line: &str) -> Option<Self> {
        let (_, rest) = line.split_once(RECORD)?;
        fn field<'a>(word: Option<&'a str>, key: &str) -> Option<&'a str> {
            word?.strip_prefix(key)
        }
        let mut words = rest.split_ascii_whitespace();
        match words.next()? {
            "joined" => Some(Self::Joined(Which::parse(field(
                words.next(),
                "network=",
            )?)?)),
            "failed" => {
                let which = Which::parse(field(words.next(), "network=")?)?;
                let cause = Cause::parse(field(words.next(), "cause=")?)?;
                Some(Self::Failed(which, cause))
            }
            "wired" => Some(Self::Wired),
            "cleared" => Some(Self::Cleared),
            _ => None,
        }
    }
}

/// How the earlier boot ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ending {
    /// The log holds no earlier boot: a new USB's first.
    First,
    /// macOS turned the Mac off.
    PowerOff,
    /// macOS restarted the Mac: an installer's, an update's or the user's
    /// restart, whose next boot nobody may be at the PC for.
    Restart,
    /// Stambha stopped.
    Stopped,
    /// No line ends it: a hang, a crash past the log, or power lost.
    Unfinished,
}

/// The earlier boot, as its log tells.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Earlier {
    pub ending: Ending,
    /// Its newest Wi-Fi record, other than [`Record::Wired`].
    pub wifi: Option<Record>,
    /// A wired adapter's link came up.
    pub wired: bool,
}

/// The earlier boot in `stream`, the log's newest bytes, read before this
/// boot's marker line joins it: the part after its last marker line. Bytes
/// with no marker line at all are either nothing (a new USB) or the end of
/// a boot whose marker the ring wrote over, which ended as its lines say.
pub fn earlier(stream: &[u8]) -> Earlier {
    let text = String::from_utf8_lossy(stream);
    let boot = match text.rfind(BOOT) {
        Some(at) => &text[at..],
        None if text.trim().is_empty() => {
            return Earlier {
                ending: Ending::First,
                wifi: None,
                wired: false,
            };
        }
        None => &text[..],
    };
    let mut earlier = Earlier {
        ending: Ending::Unfinished,
        wifi: None,
        wired: false,
    };
    for line in boot.lines() {
        if line.contains(RESTART) {
            earlier.ending = Ending::Restart;
        } else if line.contains(POWER_OFF) {
            earlier.ending = Ending::PowerOff;
        } else if line.contains(STOPPED) {
            earlier.ending = Ending::Stopped;
        }
        match Record::parse(line) {
            Some(Record::Wired) => earlier.wired = true,
            Some(record) => earlier.wifi = Some(record),
            None => {}
        }
    }
    earlier
}

/// What this boot's menu knows when it decides.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Facts {
    /// The earlier boot; `None` when the log could not be read.
    pub earlier: Option<Earlier>,
    /// The census found a Wi-Fi chip the driver starts
    /// (`wifi_hardware::Hardware::Supported`).
    pub supported: bool,
    /// The boot waits at the menu for setup (`choice::hold`).
    pub held: bool,
    /// The startup choices open no menu at all (`timeout_seconds` 0).
    pub no_menu: bool,
    /// The boot has a network to join: `--wifi`, or a usable saved one.
    pub network: bool,
}

/// The page opens by itself.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Prompt {
    /// No network is set up: choose one.
    Choose,
    /// The earlier boot could not join this network.
    Retry(Which, Cause),
}

/// Why the page stays closed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Closed {
    Unsupported,
    Held,
    NoMenu,
    Unread,
    Restart,
    Stopped,
    Unfinished,
    Wired,
    Joined,
    NetworkSet,
}

impl Closed {
    fn why(self) -> &'static str {
        match self {
            Self::Unsupported => "no Wi-Fi chip the driver starts",
            Self::Held => "the menu holds for setup",
            Self::NoMenu => "the startup choices open no menu",
            Self::Unread => "the earlier boot's log could not be read",
            Self::Restart => "the earlier boot ended in a restart macOS asked for",
            Self::Stopped => "the earlier boot stopped",
            Self::Unfinished => "the earlier boot never ended",
            Self::Wired => "a wired network was up the earlier boot",
            Self::Joined => "the earlier boot joined Wi-Fi",
            Self::NetworkSet => "a network is set up",
        }
    }
}

/// Whether the page opens by itself this boot, and how.
pub fn decide(facts: &Facts) -> Result<Prompt, Closed> {
    if !facts.supported {
        return Err(Closed::Unsupported);
    }
    if facts.held {
        return Err(Closed::Held);
    }
    if facts.no_menu {
        return Err(Closed::NoMenu);
    }
    let earlier = facts.earlier.ok_or(Closed::Unread)?;
    match earlier.ending {
        Ending::First | Ending::PowerOff => {}
        Ending::Restart => return Err(Closed::Restart),
        Ending::Stopped => return Err(Closed::Stopped),
        Ending::Unfinished => return Err(Closed::Unfinished),
    }
    if earlier.wired {
        return Err(Closed::Wired);
    }
    match earlier.wifi {
        Some(Record::Failed(which, cause)) if facts.network => Ok(Prompt::Retry(which, cause)),
        Some(Record::Joined(_)) => Err(Closed::Joined),
        _ if facts.network => Err(Closed::NetworkSet),
        _ => Ok(Prompt::Choose),
    }
}

/// The log's line for [`decide`]'s answer.
pub fn line(decided: &Result<Prompt, Closed>) -> String {
    match decided {
        Ok(Prompt::Choose) => {
            String::from("CHOICE Wi-Fi page: opens by itself: no network is set up\n")
        }
        Ok(Prompt::Retry(which, cause)) => format!(
            "CHOICE Wi-Fi page: opens by itself on network={}: the earlier boot's join failed \
             (cause={})\n",
            which.word(),
            cause.word()
        ),
        Err(closed) => format!("CHOICE Wi-Fi page: stays closed: {}\n", closed.why()),
    }
}

/// The page's line when it opens by itself to choose a network.
pub const CHOOSE: &str = "Connect to Wi-Fi so macOS has internet. Esc: start without Wi-Fi.";

/// The page's line when it opens on `name`, whose join failed for `cause`.
pub fn retry_notice(name: &str, cause: Cause) -> String {
    match cause {
        Cause::Password => format!(
            "Couldn't join {name}: the password may be wrong. Type it again, or press Esc to \
             start without Wi-Fi."
        ),
        Cause::Range => format!(
            "{name} isn't in range. Check its name and password, or press Esc to start without \
             Wi-Fi."
        ),
    }
}

#[cfg(test)]
mod tests;

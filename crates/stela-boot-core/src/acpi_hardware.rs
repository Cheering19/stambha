//! ACPI hardware mode (`--acpi-hardware`): what a boot says about it, and
//! the two guards that keep a misbehaving firmware from costing the guest
//! its time once it runs.
//!
//! Hardware mode is ACPICA with its events on: the firmware switched into
//! ACPI mode (`SMI_CMD` ← `ACPI_ENABLE`, unless `SCI_EN` is already set),
//! the SCI routed to the boot processor, the fixed events and GPEs
//! dispatched, the embedded controller's `_REG` run and its queries
//! (`_Qxx`) taken, and the notifications those send delivered as deferred
//! work. The engine runs all of it on the work owner's continuation
//! (`stela-boot`'s `engine::aml`); this module holds only what a host test
//! can check: the lines, the FADT's ACPI-mode registers, the SCI's storm
//! guard ([`SciGovernor`]) and the EC's query budget and failure breaker
//! ([`EcGovernor`]).
//!
//! It is off by default until a laptop has proven it: the option is a
//! hand-written opt-in setup never writes, so it keeps the options
//! revision (`stela_loader::mount_interface::OPTIONS_REVISION`), and a
//! laptop's boot does not take it unasked ([`crate::laptop`]).
use alloc::{format, string::String};

/// What stays off while hardware mode does: the list the off lines name.
pub const WITHOUT: &str = "the embedded controller and what firmware reads through it (battery, \
     AC adapter, lid), ACPI device notifications and USB-C port reports (UCSI)";

/// The line of a boot that leaves hardware mode off because the option is
/// not given.
pub fn off_line() -> String {
    format!(
        "UNSUPPORTED ACPI hardware mode: off unless --acpi-hardware is given, until it is proven \
         on hardware (no ACPI interrupt, ACPI events or deferred firmware work); off because of \
         it: {WITHOUT}\n"
    )
}

/// The line of a boot whose `--acpi-hardware` cannot start, for `reason`.
pub fn refused_line(reason: &str) -> String {
    format!(
        "UNSUPPORTED ACPI hardware mode: --acpi-hardware is given, but {reason}, so it stays off; \
         off because of it: {WITHOUT}\n"
    )
}

/// The FADT's legacy fixed-hardware registers ACPI mode involves.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Registers {
    /// `SCI_INT` (offset 46): the SCI's legacy interrupt number.
    pub sci_interrupt: u16,
    /// `SMI_CMD` (offset 48), 0 where the platform has none.
    pub smi_command: u32,
    /// `ACPI_ENABLE` (offset 52): written to `SMI_CMD` to enter ACPI mode.
    pub acpi_enable: u8,
    /// `PM1a_CNT_BLK` (offset 64), whose bit 0 is `SCI_EN`.
    pub pm1a_control: u32,
    /// HW_REDUCED_ACPI (flags bit 20): no SCI and no legacy fixed hardware.
    pub reduced: bool,
}

/// `SCI_EN`, bit 0 of PM1_CNT: the platform is in ACPI mode, and its
/// events raise the SCI rather than an SMI.
pub const SCI_EN: u16 = 1;

impl Registers {
    /// The registers of a FADT, `None` when it is shorter than its flags.
    pub fn read(fadt: &[u8]) -> Option<Self> {
        let word = |at: usize| Some(u16::from_le_bytes(fadt.get(at..at + 2)?.try_into().ok()?));
        let dword = |at: usize| Some(u32::from_le_bytes(fadt.get(at..at + 4)?.try_into().ok()?));
        Some(Self {
            sci_interrupt: word(46)?,
            smi_command: dword(48)?,
            acpi_enable: *fadt.get(52)?,
            pm1a_control: dword(64)?,
            reduced: dword(112)? & (1 << 20) != 0,
        })
    }

    /// The PM1a control port, where the platform has one in I/O space.
    pub fn control_port(&self) -> Option<u16> {
        u16::try_from(self.pm1a_control)
            .ok()
            .filter(|&port| port != 0)
    }

    /// What entering ACPI mode takes, with PM1a_CNT reading `control`
    /// (`None` where it was not read): none when `SCI_EN` is set, as
    /// ACPICA's `acpi_enable` skips the write then.
    pub fn entry(&self, control: Option<u16>) -> String {
        match control {
            Some(value) if value & SCI_EN != 0 => {
                String::from("the firmware left SCI_EN set, so no SMI_CMD write enters ACPI mode")
            }
            _ if self.smi_command == 0 || self.acpi_enable == 0 => {
                String::from("SCI_EN is clear and the FADT has no SMI_CMD or ACPI_ENABLE")
            }
            Some(_) => format!(
                "SCI_EN is clear, so ACPI mode is entered with SMI_CMD {:#x} <- {:#x}",
                self.smi_command, self.acpi_enable
            ),
            None => format!(
                "PM1a_CNT unread; ACPI mode is entered with SMI_CMD {:#x} <- {:#x} if SCI_EN is \
                 clear",
                self.smi_command, self.acpi_enable
            ),
        }
    }
}

/// The line of a boot that starts hardware mode: its SCI's route and what
/// entering ACPI mode takes.
pub fn starting_line(registers: &Registers, route: &str, control: Option<u16>) -> String {
    format!(
        "ACPI hardware mode starting (--acpi-hardware): SCI_INT {} on {route}; {}\n",
        registers.sci_interrupt,
        registers.entry(control)
    )
}

/// A window of the SCI's storm guard, in milliseconds.
pub const WINDOW_MS: u64 = 1_000;
/// SCIs in one window that count toward a storm. The SCI is taken at most
/// once per host service turn (about 250 a second), and a laptop's events
/// (battery, lid, hotkeys) come a few a second at most, so a level held
/// asserted shows as nearly every turn's.
pub const STORM_PER_WINDOW: u32 = 100;
/// Windows in a row past [`STORM_PER_WINDOW`] that make a storm.
pub const STORM_WINDOWS: u32 = 3;
/// How often ACPICA's SCI handler runs while the SCI stays masked.
pub const POLL_MS: u64 = 100;
/// How long the first storm keeps the SCI masked before it is tried again;
/// a second storm keeps it masked for the rest of the boot.
pub const RETRY_MS: u64 = 30_000;

/// What the SCI's owner does this turn ([`SciGovernor::turn`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Turn {
    /// Run ACPICA's SCI handler.
    pub dispatch: bool,
    /// Unmask the SCI's line again once the handler ran.
    pub rearm: bool,
    /// A change to report, once.
    pub change: Option<Change>,
}

/// A change of the SCI's mode, for its line ([`Change::line`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Change {
    /// The SCI came `per_window` times in the last window, and as often in
    /// the windows before it: it stays masked and is polled, for
    /// [`RETRY_MS`] or, `lasting`, for good.
    Storm { per_window: u32, lasting: bool },
    /// The masked SCI is tried again after [`RETRY_MS`].
    Retry,
}

impl Change {
    pub fn line(&self) -> String {
        match *self {
            Self::Storm {
                per_window,
                lasting,
            } => format!(
                "HOST ACPI SCI storm: {per_window} interrupts in the last second, past \
                 {STORM_PER_WINDOW} for {STORM_WINDOWS} s; the SCI stays masked and ACPICA's \
                 handler runs every {POLL_MS} ms {}\n",
                if lasting {
                    String::from("for the rest of the boot (the second storm)")
                } else {
                    format!("for {} s, then interrupts are tried again", RETRY_MS / 1000)
                }
            ),
            Self::Retry => format!(
                "HOST ACPI SCI interrupts tried again after {} s polled\n",
                RETRY_MS / 1000
            ),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    Interrupts,
    Polled { since: u64, lasting: bool },
}

/// The SCI's storm guard. A level SCI that no handler clears (a GPE
/// without one that ACPICA cannot disable, a chipset's stuck status) would
/// otherwise run ACPICA's handler every turn for the rest of the boot.
/// Past [`STORM_PER_WINDOW`] for [`STORM_WINDOWS`] windows in a row the SCI
/// stays masked and the handler runs every [`POLL_MS`] instead, which still
/// takes every event, late; interrupts are tried again once after
/// [`RETRY_MS`], and a second storm keeps the polling for good.
#[derive(Clone, Copy, Debug)]
pub struct SciGovernor {
    mode: Mode,
    window_start: u64,
    in_window: u32,
    stormy: u32,
    last_poll: u64,
    storms: u32,
}

impl SciGovernor {
    pub fn new(now: u64) -> Self {
        Self {
            mode: Mode::Interrupts,
            window_start: now,
            in_window: 0,
            stormy: 0,
            last_poll: now,
            storms: 0,
        }
    }

    /// Whether the SCI is masked and polled.
    pub fn polled(&self) -> bool {
        matches!(self.mode, Mode::Polled { .. })
    }

    /// The storms so far.
    pub fn storms(&self) -> u32 {
        self.storms
    }

    /// One turn at `now` (milliseconds), `interrupted` when the SCI's
    /// interrupt arrived since the last turn.
    pub fn turn(&mut self, now: u64, interrupted: bool) -> Turn {
        match self.mode {
            Mode::Interrupts => {
                self.in_window += u32::from(interrupted);
                let mut change = None;
                if now.saturating_sub(self.window_start) >= WINDOW_MS {
                    if self.in_window >= STORM_PER_WINDOW {
                        self.stormy += 1;
                    } else {
                        self.stormy = 0;
                    }
                    let per_window = self.in_window;
                    self.window_start = now;
                    self.in_window = 0;
                    if self.stormy >= STORM_WINDOWS {
                        self.storms += 1;
                        let lasting = self.storms > 1;
                        self.mode = Mode::Polled {
                            since: now,
                            lasting,
                        };
                        self.stormy = 0;
                        self.last_poll = now;
                        change = Some(Change::Storm {
                            per_window,
                            lasting,
                        });
                    }
                }
                Turn {
                    dispatch: interrupted,
                    rearm: interrupted && !self.polled(),
                    change,
                }
            }
            Mode::Polled { since, lasting } => {
                if !lasting && now.saturating_sub(since) >= RETRY_MS {
                    self.mode = Mode::Interrupts;
                    self.window_start = now;
                    self.in_window = 0;
                    return Turn {
                        dispatch: true,
                        rearm: true,
                        change: Some(Change::Retry),
                    };
                }
                let due = now.saturating_sub(self.last_poll) >= POLL_MS;
                if due {
                    self.last_poll = now;
                }
                Turn {
                    dispatch: due || interrupted,
                    rearm: false,
                    change: None,
                }
            }
        }
    }
}

/// EC queries one window takes at most; the rest wait for the next. An EC
/// that keeps `SCI_EVT` set and answers each `QR_EC` with an event would
/// otherwise run its `_Qxx` methods every turn.
pub const QUERIES_PER_WINDOW: u32 = 64;
/// Failed EC transactions in a row that stop the EC's events. Each one has
/// waited out the EC's timeout (500 ms), so a dead EC would otherwise cost
/// the guest that each turn.
pub const EC_FAILURES: u32 = 3;

/// Whether one more EC query is taken now ([`EcGovernor::admit`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Admit {
    Take,
    /// The window's budget is spent; `first` for the window's first refusal,
    /// which the boot reports once.
    Wait {
        first: bool,
    },
    /// The EC's events are stopped.
    Stopped,
}

/// The EC's query budget and failure breaker, and which query numbers the
/// boot has named, each once.
#[derive(Clone, Debug)]
pub struct EcGovernor {
    window_start: u64,
    in_window: u32,
    refused: bool,
    failures: u32,
    stopped: bool,
    named: [u64; 4],
}

impl EcGovernor {
    pub fn new(now: u64) -> Self {
        Self {
            window_start: now,
            in_window: 0,
            refused: false,
            failures: 0,
            stopped: false,
            named: [0; 4],
        }
    }

    pub fn stopped(&self) -> bool {
        self.stopped
    }

    /// One more query at `now` (milliseconds).
    pub fn admit(&mut self, now: u64) -> Admit {
        if self.stopped {
            return Admit::Stopped;
        }
        if now.saturating_sub(self.window_start) >= WINDOW_MS {
            self.window_start = now;
            self.in_window = 0;
            self.refused = false;
        }
        if self.in_window >= QUERIES_PER_WINDOW {
            let first = !self.refused;
            self.refused = true;
            return Admit::Wait { first };
        }
        self.in_window += 1;
        Admit::Take
    }

    /// A transaction that completed.
    pub fn succeeded(&mut self) {
        self.failures = 0;
    }

    /// A transaction that failed: true when this one stops the EC's events.
    pub fn failed(&mut self) -> bool {
        if self.stopped {
            return false;
        }
        self.failures += 1;
        if self.failures >= EC_FAILURES {
            self.stopped = true;
            return true;
        }
        false
    }

    /// Whether query `number` is named for the first time, so its line is
    /// written: a laptop's few events each get one, and an event that
    /// repeats does not fill the log.
    pub fn first(&mut self, number: u8) -> bool {
        let (word, bit) = (usize::from(number / 64), 1u64 << (number % 64));
        let first = self.named[word] & bit == 0;
        self.named[word] |= bit;
        first
    }
}

#[cfg(test)]
mod tests;

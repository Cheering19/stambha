//! A host controller lent to the guest behind Apple's module
//! (`--bluetooth-lend`): the function keeps the module's identity, a shadow
//! of the controller model ([`Model`]) answers every vendor command and
//! every command the milestone keeps local, and only the standard commands
//! on [`policy`]'s list go to the host's controller, whose answers the guest
//! gets.
//!
//! The bridge decides, packet by packet, and does no I/O: [`Bridge::command`]
//! and [`Bridge::chip_event`] return [`Step`]s, an event for the guest or a
//! command for the host's controller, which the function queues for its
//! owner to carry.
//!
//! - The shadow executes every command, so it always holds the state the
//!   guest set. Its own later events (an inquiry or page that ends) are
//!   discarded while the bridge forwards.
//! - One forwarded command is in flight. The guest always sees one command
//!   credit, as from the model; later commands wait their turn.
//! - Every forwarded command is answered by [`DEADLINE`]. If the host's
//!   controller has not answered by then, fails, or answers with an error the
//!   model would not give or of another kind or length than the model's, the
//!   bridge falls back: the guest gets the shadow's answer to that command
//!   and the shadow answers everything after it, for the rest of the boot.
//!   Nothing goes to the host's controller again. So bluetoothd's bring-up,
//!   and Setup's first screen, which waits for it, never wait on a stalled
//!   controller.
use super::model::Model;
use crate::counter::FREQUENCY_HZ;
use alloc::{collections::VecDeque, vec, vec::Vec};
use core::fmt;

pub mod policy;

pub use policy::ChipCommand;

/// System counts the host's controller has to answer a forwarded command:
/// one second.
pub const DEADLINE: u64 = FREQUENCY_HZ;

/// Commands that wait behind the one in flight; more stall the guest's
/// control transfer until one is answered.
pub const BACKLOG: usize = 64;

// Events (Bluetooth Core v5.4 Vol 4 Part E 7.7).
const COMMAND_COMPLETE: u8 = 0x0e;
const COMMAND_STATUS: u8 = 0x0f;

/// What the bridge does with a packet: an event for the guest, or a command
/// for the host's controller.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Step {
    Guest(Vec<u8>),
    Chip(ChipCommand),
}

/// Why the bridge fell back to the model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cause {
    /// No answer by [`DEADLINE`].
    Timeout,
    /// This error status where the model answers success.
    Refused(u8),
    /// An answer of another kind or length than the model's.
    Malformed,
    /// The host's controller failed.
    HostFailed,
}

impl fmt::Display for Cause {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Timeout => f.write_str("no answer in 1 s"),
            Self::Refused(status) => write!(f, "status {status:#04x} where the model succeeds"),
            Self::Malformed => f.write_str("an answer of another kind or length than the model's"),
            Self::HostFailed => f.write_str("the host controller failed"),
        }
    }
}

/// The command the bridge fell back at, if one was in flight, and why.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Fallback {
    pub opcode: Option<u16>,
    pub cause: Cause,
}

impl fmt::Display for Fallback {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.opcode {
            Some(opcode) => write!(f, "{opcode:#06x} {}", self.cause),
            None => write!(f, "between commands: {}", self.cause),
        }
    }
}

/// Counters for the owner's log.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counters {
    /// Commands sent to the host's controller.
    pub forwarded: u64,
    /// Commands the shadow answered while the bridge forwarded.
    pub local: u64,
    /// Events of the host's controller the guest got.
    pub passed: u64,
    /// Events of the host's controller dropped: vendor events, credits
    /// alone, stale answers, and every event after the fallback.
    pub dropped: u64,
}

/// The forwarded command in flight.
#[derive(Debug, Clone)]
struct Pending {
    opcode: u16,
    kind: policy::Forward,
    /// The shadow's answer, the first of its events, given on a fallback.
    shadow: Vec<Vec<u8>>,
    deadline: u64,
}

/// The bridge.
#[derive(Debug, Clone)]
pub struct Bridge {
    shadow: Model,
    fallen: Option<Fallback>,
    pending: Option<Pending>,
    backlog: VecDeque<Vec<u8>>,
    counters: Counters,
}

/// The parts of a Command Complete or Command Status event that answers a
/// command, with its status.
pub(super) struct Completion<'a> {
    pub code: u8,
    pub opcode: u16,
    pub status: u8,
    pub returns: &'a [u8],
}

/// Whether `event` is a Command Complete or Command Status, which answers a
/// command or only gives credits.
fn completes(event: &[u8]) -> bool {
    matches!(event.first(), Some(&(COMMAND_COMPLETE | COMMAND_STATUS)))
}

impl<'a> Completion<'a> {
    pub(super) fn of(event: &'a [u8]) -> Option<Self> {
        if event.len() < 2 || usize::from(event[1]) + 2 != event.len() {
            return None;
        }
        match *event {
            [COMMAND_COMPLETE, _, _, low, high, status, ref returns @ ..] => Some(Self {
                code: COMMAND_COMPLETE,
                opcode: u16::from_le_bytes([low, high]),
                status,
                returns,
            }),
            [COMMAND_STATUS, 4, status, _, low, high] => Some(Self {
                code: COMMAND_STATUS,
                opcode: u16::from_le_bytes([low, high]),
                status,
                returns: &[],
            }),
            _ => None,
        }
    }

    /// The event, with the one command credit the guest always sees.
    fn event(&self, returns: &[u8]) -> Vec<u8> {
        let [low, high] = self.opcode.to_le_bytes();
        if self.code == COMMAND_STATUS {
            return vec![COMMAND_STATUS, 4, self.status, 1, low, high];
        }
        let mut event = vec![COMMAND_COMPLETE, (4 + returns.len()) as u8, 1, low, high];
        event.push(self.status);
        event.extend_from_slice(returns);
        event
    }
}

/// The opcode of a command or of the completion event that answers one.
fn opcode(packet: &[u8]) -> u16 {
    u16::from_le_bytes([packet[0], packet[1]])
}

fn guest(events: Vec<Vec<u8>>) -> Vec<Step> {
    events.into_iter().map(Step::Guest).collect()
}

impl Bridge {
    /// A bridge whose shadow's power-on BD_ADDR is `address`.
    pub fn new(address: [u8; 6]) -> Self {
        Self {
            shadow: Model::new(address),
            fallen: None,
            pending: None,
            backlog: VecDeque::new(),
            counters: Counters::default(),
        }
    }

    /// Why the bridge fell back to the model, once it has.
    pub fn fallen(&self) -> Option<Fallback> {
        self.fallen
    }

    pub fn counters(&self) -> Counters {
        self.counters
    }

    /// The shadow's BD_ADDR now.
    pub fn address(&self) -> [u8; 6] {
        self.shadow.address()
    }

    /// Opcodes the shadow answered Unknown HCI Command that were not taken
    /// before, each once.
    pub fn take_unknown(&mut self) -> Vec<u16> {
        self.shadow.take_unknown()
    }

    /// Whether another command can wait for its turn.
    pub fn has_room(&self) -> bool {
        self.backlog.len() < BACKLOG
    }

    /// One command of the guest, header included, whose length its header
    /// already matches.
    pub fn command(&mut self, packet: &[u8], now: u64) -> Vec<Step> {
        if self.pending.is_some() {
            self.backlog.push_back(packet.to_vec());
            return Vec::new();
        }
        self.start(packet, now)
    }

    fn start(&mut self, packet: &[u8], now: u64) -> Vec<Step> {
        let shadow = self.shadow.command(packet, now);
        if self.fallen.is_some() {
            return guest(shadow);
        }
        let Some((command, kind)) = policy::forward(packet) else {
            self.counters.local += 1;
            return guest(shadow);
        };
        self.counters.forwarded += 1;
        self.pending = Some(Pending {
            opcode: opcode(packet),
            kind,
            shadow,
            deadline: now.saturating_add(DEADLINE),
        });
        vec![Step::Chip(command)]
    }

    /// The commands that waited, until one is in flight again.
    fn next(&mut self, now: u64) -> Vec<Step> {
        let mut steps = Vec::new();
        while self.pending.is_none()
            && let Some(packet) = self.backlog.pop_front()
        {
            steps.extend(self.start(&packet, now));
        }
        steps
    }

    /// Falls back for good; the guest gets the shadow's answer to the
    /// command in flight, and the commands that waited are the shadow's.
    fn fall(&mut self, cause: Cause, now: u64) -> Vec<Step> {
        let pending = self.pending.take();
        self.fallen.get_or_insert(Fallback {
            opcode: pending.as_ref().map(|p| p.opcode),
            cause,
        });
        let mut steps = pending.map(|p| guest(p.shadow)).unwrap_or_default();
        steps.extend(self.next(now));
        steps
    }

    /// One event from the host's controller.
    pub fn chip_event(&mut self, event: &[u8], now: u64) -> Vec<Step> {
        if self.fallen.is_some() {
            self.counters.dropped += 1;
            return Vec::new();
        }
        if completes(event) {
            let completion = Completion::of(event);
            let pending = completion
                .as_ref()
                .and_then(|completion| self.pending.take_if(|p| p.opcode == completion.opcode));
            let (Some(completion), Some(pending)) = (completion, pending) else {
                // Credits alone (opcode 0, no status), or the answer to a
                // command no longer in flight.
                self.counters.dropped += 1;
                return Vec::new();
            };
            return self.answer(pending, &completion, now);
        }
        self.counters.passed += 1;
        vec![Step::Guest(event.to_vec())]
    }

    fn answer(&mut self, pending: Pending, chip: &Completion<'_>, now: u64) -> Vec<Step> {
        let shadow = pending.shadow.first().and_then(|e| Completion::of(e));
        match policy::answer(pending.kind, shadow.as_ref(), chip) {
            Ok(returns) => {
                self.counters.passed += 1;
                let mut steps = vec![Step::Guest(chip.event(&returns))];
                steps.extend(self.next(now));
                steps
            }
            Err(cause) => {
                self.pending = Some(pending);
                self.fall(cause, now)
            }
        }
    }

    /// The system count is `now`: a forwarded command past its deadline
    /// falls back, and the shadow's procedures that ended give their events
    /// once it has.
    pub fn advance(&mut self, now: u64) -> Vec<Step> {
        let mut steps = Vec::new();
        if self.pending.as_ref().is_some_and(|p| p.deadline <= now) {
            steps = self.fall(Cause::Timeout, now);
        }
        let later = self.shadow.advance(now);
        if self.fallen.is_some() {
            steps.extend(guest(later));
        }
        steps
    }

    /// The earliest system count at which [`Self::advance`] has something
    /// to do.
    pub fn deadline(&self) -> Option<u64> {
        let pending = self.pending.as_ref().map(|p| p.deadline);
        let shadow = self.fallen.and(self.shadow.deadline());
        pending.into_iter().chain(shadow).min()
    }

    /// The host's controller failed: the bridge falls back.
    pub fn chip_failed(&mut self, now: u64) -> Vec<Step> {
        if self.fallen.is_some() {
            return Vec::new();
        }
        self.fall(Cause::HostFailed, now)
    }
}

#[cfg(test)]
mod chip;
#[cfg(test)]
mod tests;

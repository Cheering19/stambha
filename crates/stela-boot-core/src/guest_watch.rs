//! What the home PE watches for in the guest's console while macOS runs:
//! a kernel panic that never resets the machine, and a console that has
//! gone quiet before macOS showed anything.
//!
//! XNU prints `panic(cpu N caller 0x...): message` at the start of a line
//! and then the panic log: the backtrace, `Panicked task`, the kernel
//! extensions in it, often a nested panic. On a Mac its watchdog or the PMU
//! resets the machine after that, which ends the run (`Stop::Reset`). A
//! panic whose handler hangs, as one with its other CPUs unanswered and no
//! watchdog armed yet, spins for good: a 2026-11-23 boot printed its panic
//! at 42.7 s and ran on for twenty minutes at one PC. [`Watch`] ends such a
//! run [`PANIC_HOLD_MILLIS`] after the panic's first line, time enough for
//! the rest of the panic log and for a reset that does come, unless the
//! boot asked to keep running (`--keep-running-on-panic`).
//!
//! A console with no line for [`STALL_MILLIS`] before the guest's first
//! frame reached the screen is only named, once a quiet stretch, as
//! `BOOT STALLED?`: macOS may still be working.
use alloc::string::String;

/// How long a panic is left to finish printing, or to reset the machine,
/// before the run stops: XNU writes its panic log within about a second,
/// and its watchdog mechanism resets within 10 s of arming
/// (`guest_timeouts::WATCHDOG_MECHANISM_SECONDS`).
pub const PANIC_HOLD_MILLIS: u64 = 30_000;

/// The lines after a panic's first that [`Watch`] counts as its log; more
/// are counted as this many.
pub const PANIC_LINES: u32 = 4096;

/// How long the console may stay quiet before the guest showed a frame
/// before the log names it.
pub const STALL_MILLIS: u64 = 120_000;

/// The most of a line [`Watch`] keeps to name it.
const KEPT_BYTES: usize = 240;

/// Whether `line`, from a line's start, begins XNU's panic header
/// `panic(cpu N caller ...)`. Only a line's start counts: a driver that
/// prints the word, or a header inside another line, is no panic.
pub fn begins_panic(line: &str) -> bool {
    line.strip_prefix("panic(cpu ")
        .is_some_and(|rest| rest.starts_with(|c: char| c.is_ascii_digit()))
}

/// The first panic header seen.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Panic {
    /// When its line was written, in milliseconds.
    pub at: u64,
    /// Its line, at most [`KEPT_BYTES`] of it.
    pub line: String,
    /// The lines after it so far, at most [`PANIC_LINES`].
    pub lines: u32,
}

/// What [`Watch::poll`] found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    /// A panic's first line arrived: the run stops after the hold, or with
    /// `keep_running` goes on.
    Panicked { line: String, keep_running: bool },
    /// The hold after the panic ended with no reset: stop the run.
    Stop(Panic),
    /// The console has been quiet this many seconds before the guest's
    /// first frame, since `last` (none: no line yet).
    Stalled { seconds: u64, last: Option<String> },
}

#[derive(Debug, Default)]
pub struct Watch {
    keep_running: bool,
    /// When watching began, for a console that never printed.
    began: Option<u64>,
    panic: Option<Panic>,
    /// [`Event::Panicked`] was given.
    said: bool,
    /// [`Event::Stop`] was given.
    stopped: bool,
    /// The last line with text, and when it came.
    last: Option<(u64, String)>,
    /// This quiet stretch was named.
    warned: bool,
}

impl Watch {
    pub const fn new() -> Self {
        Self {
            keep_running: false,
            began: None,
            panic: None,
            said: false,
            stopped: false,
            last: None,
            warned: false,
        }
    }

    /// Go on after a panic instead of stopping (`--keep-running-on-panic`).
    pub fn keep_running(&mut self, keep: bool) {
        self.keep_running = keep;
    }

    /// The first panic, once one was seen.
    pub fn panic(&self) -> Option<&Panic> {
        self.panic.as_ref()
    }

    /// A console line or fragment written at `now` milliseconds; `beginning`
    /// when it starts a line rather than continuing a long one.
    pub fn line(&mut self, now: u64, text: &str, beginning: bool) {
        if text.trim().is_empty() {
            return;
        }
        self.last = Some((now, kept(text)));
        self.warned = false;
        match &mut self.panic {
            Some(panic) => panic.lines = panic.lines.saturating_add(1).min(PANIC_LINES),
            None if beginning && begins_panic(text) => {
                self.panic = Some(Panic {
                    at: now,
                    line: kept(text),
                    lines: 0,
                })
            }
            None => {}
        }
    }

    /// What is due at `now` milliseconds; `shown` once the guest's first
    /// frame reached the screen. Each event is given once (a stall once a
    /// quiet stretch); a stop before a stall.
    pub fn poll(&mut self, now: u64, shown: bool) -> Option<Event> {
        let began = *self.began.get_or_insert(now);
        if let Some(panic) = &self.panic {
            if !self.said {
                self.said = true;
                return Some(Event::Panicked {
                    line: panic.line.clone(),
                    keep_running: self.keep_running,
                });
            }
            if !self.keep_running
                && !self.stopped
                && now.saturating_sub(panic.at) >= PANIC_HOLD_MILLIS
            {
                self.stopped = true;
                return Some(Event::Stop(panic.clone()));
            }
        }
        let since = self.last.as_ref().map_or(began, |(at, _)| *at);
        let quiet = now.saturating_sub(since);
        if !shown && !self.warned && quiet >= STALL_MILLIS {
            self.warned = true;
            return Some(Event::Stalled {
                seconds: quiet / 1000,
                last: self.last.as_ref().map(|(_, line)| line.clone()),
            });
        }
        None
    }
}

/// At most [`KEPT_BYTES`] of `text`, cut at a character's boundary.
fn kept(text: &str) -> String {
    let mut end = text.len().min(KEPT_BYTES);
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    let mut line = String::from(text[..end].trim_end());
    if end < text.len() {
        line.push_str(" ...");
    }
    line
}

#[cfg(test)]
mod tests;

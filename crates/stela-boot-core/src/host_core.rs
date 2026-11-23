//! `--host-core on` (`options::HOST_CORE`): one host processor beyond the
//! guest's PEs that runs the host's own work, the service turns and the
//! GPUs' pump, so that PE 0's core runs guest code where it served before.
//!
//! The core is started like a PE's (its record, descriptors, shootdown slot
//! and counter agreement), then given no PE: macOS sees exactly the PEs it
//! sees without the option. Every step that cannot be checked leaves the
//! boot as it is without it, with a line saying why:
//!
//! - [`plan`]: before anything is reserved, whether a processor is left
//!   beyond the PEs, and the run is parallel.
//! - [`confirm`]: once the processors started, whether the last one, the
//!   host core's, did, and the reservation kept the count it was planned
//!   with.
//! - [`Channel::self_check`]: before the guest starts, the host core must
//!   answer a ping through the [`Channel`] within its limit, or it never
//!   serves and PE 0 keeps everything ([`State::Fallback`]).
//! - [`Channel::covers`]: PE 0 leaves the GPUs' pump to the host core only
//!   while the host core serves and its beat is fresh; a stalled host core
//!   gives the work back to PE 0 at once.
//!
//! The [`Channel`] has one poster (the boot processor) and one taker (the
//! host core): a request word with a sequence number, an answer word, the
//! state and the beat.
use core::sync::atomic::{AtomicU8, AtomicU64, Ordering};

/// What [`plan`] makes of the option.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Plan {
    /// Not asked for: the boot reserves its PEs alone.
    Off,
    /// Asked for but not possible: the boot reserves its PEs alone and
    /// says why.
    Refused(&'static str),
    /// Reserve `executors` processors: the PEs and the host core, last.
    Reserve { executors: usize },
}

impl Plan {
    /// The processors to reserve for `pes` PEs under this plan.
    pub const fn executors(self, pes: usize) -> usize {
        match self {
            Self::Reserve { executors } => executors,
            Self::Off | Self::Refused(_) => pes,
        }
    }
}

/// Whether a host core is reserved beside `pes` PEs: `wanted` is the
/// option, `parallel` whether every PE runs on a core of its own, and
/// `assignable` how many processors the capture can start (the boot
/// processor among them).
pub fn plan(wanted: bool, parallel: bool, pes: usize, assignable: usize) -> Plan {
    if !wanted {
        return Plan::Off;
    }
    if !parallel || pes < 1 {
        return Plan::Refused("the PEs run serialized on one processor");
    }
    match pes.checked_add(1) {
        Some(executors) if executors <= assignable => Plan::Reserve { executors },
        _ => Plan::Refused("no processor is left beyond the guest's PEs"),
    }
}

/// Whether the host core exists once the processors started: the plan
/// reserved `planned` (PEs and the host core), the reservation kept
/// `reserved` (fewer without a startup page), and `started` started, the
/// boot processor among them. The host core is the last slot, so a host
/// core that did not start loses only itself.
pub fn confirm(planned: usize, reserved: usize, started: usize) -> Result<(), &'static str> {
    if reserved != planned {
        return Err("the reservation kept fewer processors than planned");
    }
    if started == planned {
        return Ok(());
    }
    if started + 1 == planned {
        Err("the host core's processor did not start")
    } else {
        Err("a PE's processor did not start, and the ones after it with it")
    }
}

/// Where the host core stands ([`Channel::state`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// No host core: PE 0 serves as without the option.
    Off,
    /// The self-check's ping is out.
    Checking,
    /// It answered: it serves and pumps.
    Serving,
    /// It did not answer in time, or was stopped: it serves nothing and PE
    /// 0 keeps everything. A late answer does not change this.
    Fallback,
    /// The run ended and it acknowledged its stop.
    Stopped,
}

impl State {
    const ALL: [Self; 5] = [
        Self::Off,
        Self::Checking,
        Self::Serving,
        Self::Fallback,
        Self::Stopped,
    ];

    pub const fn name(self) -> &'static str {
        match self {
            Self::Off => "off",
            Self::Checking => "checking",
            Self::Serving => "serving",
            Self::Fallback => "fallback",
            Self::Stopped => "stopped",
        }
    }
}

/// A request to the host core.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Request {
    /// Answer, and nothing else: the self-check.
    Ping,
    /// Answer, then serve nothing more.
    Stop,
}

const PING: u64 = 1;
const STOP: u64 = 2;
const KIND_BITS: u32 = 2;

/// What the host core does in one pass of its loop ([`Channel::turn`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Turn {
    /// Serve the host's work and pump.
    Serve,
    /// Nothing to do: wait a little.
    Idle,
    /// Leave the loop: the run ends.
    Leave,
}

/// The boot processor's line to the host core.
pub struct Channel {
    /// The last request: its sequence number above [`KIND_BITS`], then its
    /// kind. Zero before the first.
    request: AtomicU64,
    /// The sequence number of the last request answered.
    answer: AtomicU64,
    state: AtomicU8,
    /// The counter value at the host core's last pass while serving.
    beat: AtomicU64,
}

impl Channel {
    pub const fn new() -> Self {
        Self {
            request: AtomicU64::new(0),
            answer: AtomicU64::new(0),
            state: AtomicU8::new(0),
            beat: AtomicU64::new(0),
        }
    }

    pub fn state(&self) -> State {
        State::ALL[usize::from(self.state.load(Ordering::Acquire)).min(State::ALL.len() - 1)]
    }

    fn set(&self, state: State) {
        self.state.store(state as u8, Ordering::Release);
    }

    /// Post `request`, the boot processor's side: its sequence number. One
    /// poster only.
    pub fn post(&self, request: Request) -> u64 {
        let seq = (self.request.load(Ordering::Relaxed) >> KIND_BITS) + 1;
        let kind = match request {
            Request::Ping => PING,
            Request::Stop => STOP,
        };
        self.request
            .store(seq << KIND_BITS | kind, Ordering::Release);
        seq
    }

    /// Whether the request of `seq` was answered.
    pub fn answered(&self, seq: u64) -> bool {
        self.answer.load(Ordering::Acquire) >= seq
    }

    /// The host core's side: the request not answered yet, if any.
    pub fn take(&self) -> Option<(u64, Request)> {
        let word = self.request.load(Ordering::Acquire);
        let seq = word >> KIND_BITS;
        if seq == 0 || self.answered(seq) {
            return None;
        }
        let request = match word & ((1 << KIND_BITS) - 1) {
            STOP => Request::Stop,
            _ => Request::Ping,
        };
        Some((seq, request))
    }

    fn answer(&self, seq: u64) {
        self.answer.fetch_max(seq, Ordering::Release);
    }

    /// One pass of the host core's loop at counter value `now`: it answers
    /// the request waiting, and serves only while [`State::Serving`],
    /// beating as it does.
    pub fn turn(&self, now: u64) -> Turn {
        if let Some((seq, request)) = self.take() {
            self.answer(seq);
            if request == Request::Stop {
                return Turn::Leave;
            }
        }
        match self.state() {
            State::Serving => {
                self.beat.store(now, Ordering::Release);
                Turn::Serve
            }
            State::Stopped => Turn::Leave,
            State::Off | State::Checking | State::Fallback => Turn::Idle,
        }
    }

    /// The boot processor's self-check before the guest starts: a ping,
    /// then up to `limit` ticks of `now` for its answer, `wait` between
    /// looks. Serving with the ticks it took, or [`State::Fallback`] with
    /// why.
    pub fn self_check(
        &self,
        mut now: impl FnMut() -> u64,
        limit: u64,
        mut wait: impl FnMut(),
    ) -> Result<u64, &'static str> {
        self.set(State::Checking);
        let seq = self.post(Request::Ping);
        let start = now();
        loop {
            let at = now();
            if self.answered(seq) {
                self.beat.store(at, Ordering::Release);
                self.set(State::Serving);
                return Ok(at.wrapping_sub(start));
            }
            if at.wrapping_sub(start) >= limit {
                self.set(State::Fallback);
                return Err("the host core did not answer its ping in time");
            }
            wait();
        }
    }

    /// Whether the host core takes the work PE 0 would leave it at counter
    /// value `now`: serving, and its last pass less than `stale` ago.
    pub fn covers(&self, now: u64, stale: u64) -> bool {
        if self.state() != State::Serving {
            return false;
        }
        let since = now.wrapping_sub(self.beat.load(Ordering::Acquire)) as i64;
        (0..stale as i64).contains(&since) || since < 0 && since > -(stale as i64)
    }

    /// The run's end, on the boot processor: a host core that may serve is
    /// told to stop and given up to `limit` ticks to acknowledge, after
    /// which nothing has it serve. Whether it acknowledged (true when there
    /// was nothing to stop).
    pub fn stop(&self, mut now: impl FnMut() -> u64, limit: u64, mut wait: impl FnMut()) -> bool {
        let state = self.state();
        if matches!(state, State::Off | State::Stopped) {
            return true;
        }
        // Nothing serves from here on, whether or not it answers.
        self.set(State::Fallback);
        let seq = self.post(Request::Stop);
        let start = now();
        let acknowledged = loop {
            if self.answered(seq) {
                break true;
            }
            if now().wrapping_sub(start) >= limit {
                break false;
            }
            wait();
        };
        self.set(State::Stopped);
        acknowledged
    }
}

impl Default for Channel {
    fn default() -> Self {
        Self::new()
    }
}

/// Who ran a service turn, for each one's share of the host's work.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Taker {
    /// A PE that waits for an interrupt.
    Waiting = 0,
    /// The boot PE, PE 0.
    Boot = 1,
    /// The host core.
    HostCore = 2,
}

/// The ticks each [`Taker`] served since the last [`Shares::take`].
pub struct Shares([AtomicU64; 3]);

impl Shares {
    pub const fn new() -> Self {
        Self([const { AtomicU64::new(0) }; 3])
    }

    pub fn add(&self, taker: Taker, ticks: u64) {
        self.0[taker as usize].fetch_add(ticks, Ordering::Relaxed);
    }

    /// Each taker's ticks, in [`Taker`] order, and their sum, cleared.
    pub fn take(&self) -> ([u64; 3], u64) {
        let ticks = self.0.each_ref().map(|t| t.swap(0, Ordering::Relaxed));
        (
            ticks,
            ticks.iter().fold(0u64, |sum, t| sum.saturating_add(*t)),
        )
    }
}

impl Default for Shares {
    fn default() -> Self {
        Self::new()
    }
}

/// `part` of `total` in whole percent, 0 of nothing.
pub fn percent(part: u64, total: u64) -> u64 {
    if total == 0 {
        return 0;
    }
    (u128::from(part) * 100 / u128::from(total)) as u64
}

#[cfg(test)]
mod tests;

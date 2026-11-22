//! `--profile`'s AMX lines: what every PE's AMX unit did since the last
//! beat (`PES AMX`) and, with the hot-block totals, since the boot began
//! (`PES AMX TOTAL`): register operations and moves completed in blocks
//! and by the engine's service, the share host kernels ran, and the forms
//! that ran most with their mean cost
//! (`stela_engine::machine::engine::AmxCounts::line`).
use super::{output, parallel};
use stela_engine::machine::{
    Host, Machine,
    engine::{AmxCounts, FORMS},
};

/// The forms a beat's line shows; the total shows every one that ran.
const SHOWN: usize = 16;

/// The beat's AMX lines, and every PE's counts now in `seen`, which holds
/// those of the last beat.
pub(super) fn report<H: Host>(
    machine: &Machine<H, parallel::Board>,
    seen: &mut AmxCounts,
    total: bool,
    tsc_hz: u64,
) {
    let mut now = AmxCounts::default();
    for counts in parallel::counts(machine) {
        now += counts.amx;
    }
    output::arguments(format_args!("PES AMX {}\n", now.line(seen, tsc_hz, SHOWN)));
    if total {
        output::arguments(format_args!(
            "PES AMX TOTAL {}\n",
            now.line(&AmxCounts::default(), tsc_hz, FORMS)
        ));
    }
    *seen = now;
}

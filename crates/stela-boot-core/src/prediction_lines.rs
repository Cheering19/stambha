//! `PES PREDICTION`: the guest context switches since the last beat and the
//! prediction barriers the host issued for them, so that a boot with
//! `--no-ibpb` and one without show what the barrier costs. Every count is
//! one the engine keeps anyway, but for the engine path's barriers, which
//! the host counts as it issues them.
use alloc::{format, string::String};

/// One beat's counts, every PE's together.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Contexts {
    /// Whether the host isolates predictions between guest contexts: IBPB
    /// and a return stack refill at each switch (`speculation::isolates`).
    pub isolating: bool,
    /// The engine's `Statistics::prediction_isolations`: every guest context
    /// change the engine made and, where the host isolates, every native
    /// TTBR0 switch too.
    pub isolations: u64,
    /// The TTBR0 switches native code made (`Statistics::native_table_switches`).
    pub native: u64,
    /// The barriers the engine's path issued (`speculation::isolate`).
    pub engine_barriers: u64,
}

impl Contexts {
    /// `PES PREDICTION isolating=1 switches=E/N ibpb=B`: the guest context
    /// switches the engine made (E) and native code made (N), and the IBPBs
    /// the host issued for them, each with its return stack refill: the
    /// engine path's, and where the host isolates one per native switch,
    /// which the switch thunk issues itself.
    pub fn line(&self) -> String {
        let native_barriers = if self.isolating { self.native } else { 0 };
        let engine = self.isolations.saturating_sub(native_barriers);
        format!(
            "PES PREDICTION isolating={} switches={engine}/{} ibpb={}\n",
            u8::from(self.isolating),
            self.native,
            self.engine_barriers + native_barriers,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The same workload counts the same switches in both boots of an A/B;
    /// only the isolating one issues barriers, one per switch.
    #[test]
    fn both_boots_count_the_switches_and_only_an_isolating_one_barriers() {
        let isolating = Contexts {
            isolating: true,
            isolations: 150,
            native: 50,
            engine_barriers: 100,
        };
        assert_eq!(
            isolating.line(),
            "PES PREDICTION isolating=1 switches=100/50 ibpb=150\n"
        );
        let shared = Contexts {
            isolating: false,
            isolations: 100,
            native: 50,
            engine_barriers: 0,
        };
        assert_eq!(
            shared.line(),
            "PES PREDICTION isolating=0 switches=100/50 ibpb=0\n"
        );
        assert_eq!(
            Contexts::default().line(),
            "PES PREDICTION isolating=0 switches=0/0 ibpb=0\n"
        );
    }
}

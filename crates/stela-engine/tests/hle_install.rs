//! HLE in the engine: with HLE off the engine compiles exactly the blocks
//! it did without it; with a table, the compile of a signed function's
//! entry matches it, counts what it refuses, and installs a composite block
//! in its place, a native fast path in front of its first instruction
//! compiled alone, or translates it as before when it refuses.
#![cfg(all(
    feature = "test-support",
    feature = "std",
    target_arch = "x86_64",
    target_os = "linux"
))]
use anyhow::Result;
use std::sync::Arc;
use stela_engine::{
    hle::{Config, Id, Refusal, Signature, Statistics},
    machine::{Stop, system::Register},
};
#[path = "support/hle_harness.rs"]
mod hle_harness;
use hle_harness::*;

/// A case for `id` whose call returns.
fn returning(id: Id, seed: u64) -> Case {
    cases(id, seed, 64)
        .into_iter()
        .find(|case| reference(id, case).is_some() && case.x[2] > 16)
        .expect("a returning case")
}

/// `stand_in`'s function under the Bail routine, enabled.
fn bail(stand_in: &StandIn) -> Config {
    Config::with_table(table(stand_in, Id::Bail), &[Id::Bail])
}

/// The compile counts of a session's machine.
fn compiled(session: &mut Session) -> (u64, u64) {
    let statistics = &session.machine().statistics;
    (statistics.compiled, statistics.compiled_instructions)
}

#[test]
fn hle_off_compiles_the_same_blocks_as_without_it() -> Result<()> {
    let stand_in = stand_in(Id::Memmove);
    let mut without = Session::new(&stand_in, None)?;
    // Off, a boot without --hle, and one with --hle and the generated table,
    // which holds no signature yet.
    let mut configured = [
        Session::new(&stand_in, Some(Config::off()))?,
        Session::new(&stand_in, Some(Config::boot(false, &[])))?,
        Session::new(&stand_in, Some(Config::boot(true, &[])))?,
    ];
    for case in cases(Id::Memmove, 3, 50) {
        let expected = without.run(&case)?;
        for session in &mut configured {
            assert_eq!(session.run(&case)?, expected);
        }
    }
    let expected = compiled(&mut without);
    assert!(expected.0 > 0);
    for session in &mut configured {
        assert_eq!(compiled(session), expected);
        assert_eq!(session.hle(), Statistics::default());
    }
    Ok(())
}

#[test]
fn a_matched_production_routine_installs_its_native_once_and_runs_it() -> Result<()> {
    // Each production routine's stand-in under its own native: one match
    // and one install for the whole session, and a call that returns runs
    // the fast path (the composite and the `ret`, two instructions).
    for id in [Id::Memmove, Id::Bzero, Id::Memset, Id::Memcmp, Id::Strlen] {
        let stand_in = stand_in(id);
        let config = Config::with_table(table(&stand_in, id), &[id]).with_max_bytes(CASE_MAX_BYTES);
        let mut on = Session::new(&stand_in, Some(config))?;
        let mut off = Session::new(&stand_in, None)?;
        let mut native = 0;
        for case in cases(id, 6, 40) {
            let (a, b) = (off.run(&case)?, on.run(&case)?);
            assert_eq!(agree(&stand_in, &case, &a, &b), Ok(()), "{id:?}");
            native += usize::from(b.retired == 2);
        }
        let statistics = on.hle();
        assert_eq!(
            (statistics.matched, statistics.installed),
            (1, 1),
            "{id:?} {statistics:?}"
        );
        assert_eq!(statistics.matched_signatures, 1);
        assert!(native > 0, "{id:?}");
    }
    Ok(())
}

/// Whether every page of `case`'s data mapping is readable and writable.
fn mapped(case: &Case) -> bool {
    case.unreadable.is_none() && case.read_only.is_none()
}

#[test]
fn a_bail_only_native_agrees_with_the_original_for_every_stand_in() -> Result<()> {
    // Direct calls, and calls through `blr x16` on a guarded page, where
    // memset's `bti c` admits the call and the rest take a Branch Target
    // exception at the composite's entry.
    let direct = STAND_INS.map(stand_in);
    let called = STAND_INS.map(|id| stand_in(id).through_blr());
    for stand_in in direct.iter().chain(&called) {
        let id = stand_in.id;
        let count = match stand_in.caller {
            Some(_) => case_count(4000) / 16,
            None => case_count(4000),
        };
        let mut on = Session::new(stand_in, Some(bail(stand_in)))?;
        let mut off = Session::new(stand_in, None)?;
        let (mut returned, mut aborted) = (0, 0);
        for case in case_stream(id, 0xba11 + id.bit() as u64, count) {
            let (expected, got) = (off.run(&case)?, on.run(&case)?);
            assert_eq!(agree(stand_in, &case, &expected, &got), Ok(()), "{id:?}");
            // The original runs in both: they end alike in full.
            assert_eq!(got, expected, "{id:?} {:x?}", case.x);
            returned += usize::from(got.abort.is_none());
            aborted += usize::from(got.abort.is_some());
            assert!(!mapped(&case) || on.hle().bypassed == 0);
        }
        let statistics = on.hle();
        assert!(statistics.installed >= 1, "{id:?}: {statistics:?}");
        assert_eq!(statistics.bypassed, 0, "{id:?}");
        assert_eq!(statistics.refused, [0; Refusal::COUNT], "{id:?}");
        let refused = stand_in.caller.is_some() && id != Id::Memset;
        assert!(
            aborted > 0 && (refused || returned > aborted),
            "{id:?}: {returned} returned, {aborted} aborted"
        );
    }
    Ok(())
}

#[test]
fn a_composite_is_installed_once_and_reused() -> Result<()> {
    let stand_in = stand_in(Id::Memmove);
    let case = returning(Id::Memmove, 19);
    let mut session = Session::new(&stand_in, Some(bail(&stand_in)))?;
    let first = session.run(&case)?;
    assert_eq!(first.stop, Stop::Breakpoint(stand_in.end()));
    for _ in 0..2 {
        assert_eq!(session.run(&case)?, first);
    }
    let statistics = session.hle();
    assert_eq!((statistics.matched, statistics.installed), (1, 1));
    Ok(())
}

#[test]
fn a_fault_in_the_bail_path_is_the_originals_own() -> Result<()> {
    // Touch's fast path loads X0's byte, which is readable, and bails; the
    // original's first instruction, its bail, loads X1's, which is not: the
    // fault is taken through the bail's recovery row, moved past the fast
    // path.
    let stand_in = stand_in(Id::Touch);
    let base = cases(Id::Touch, 21, 1).remove(0);
    let case = Case {
        x: [100, 2 * PAGE as u64 + 5, 0],
        unreadable: Some(2),
        read_only: None,
        ..base
    };
    let touch = Config::with_table(table(&stand_in, Id::Touch), &[Id::Touch]);
    let (on, statistics) = run(&stand_in, &case, Some(touch))?;
    let (off, _) = run(&stand_in, &case, None)?;
    assert_eq!(on, off);
    let (_, far, elr, _) = on.abort.expect("the original's data abort");
    assert_eq!((far, elr), (data_base() + case.x[1], stand_in.pc()));
    assert_eq!(on.retired, 0);
    assert!(statistics.installed >= 1, "{statistics:?}");
    assert_eq!(statistics.bypassed, 0);
    Ok(())
}

#[test]
fn a_host_breakpoint_inside_the_function_refuses_it() -> Result<()> {
    let stand_in = stand_in(Id::Memmove);
    let case = returning(Id::Memmove, 7);
    let inside = stand_in.pc() + 8;
    let mut outcomes = Vec::new();
    for hle in [None, Some(bail(&stand_in))] {
        let mut session = Session::new(&stand_in, hle)?;
        let vector = session.machine().host.lower_synchronous();
        session
            .machine()
            .set_breakpoints(&[vector, stand_in.end(), inside]);
        let outcome = session.run(&case)?;
        assert_eq!(outcome.stop, Stop::Breakpoint(inside));
        outcomes.push((outcome, session.hle()));
    }
    let (off, on) = (&outcomes[0], &outcomes[1]);
    assert_eq!(on.0, off.0);
    assert!(on.1.matched >= 1, "{:?}", on.1);
    assert!(on.1.refusals(Refusal::Breakpoint) >= 1, "{:?}", on.1);
    assert_eq!(on.1.installed, 0);
    // A trace point refuses it too, and the call returns.
    let mut session = Session::new(&stand_in, Some(bail(&stand_in)))?;
    session.machine().set_traces(&[stand_in.pc() + 60]);
    let traced = session.run(&case)?;
    assert_eq!(
        agree(&stand_in, &case, &run(&stand_in, &case, None)?.0, &traced),
        Ok(())
    );
    assert!(session.hle().refusals(Refusal::Breakpoint) >= 1);
    Ok(())
}

#[test]
fn a_function_crossing_its_fetch_page_is_refused() -> Result<()> {
    let case = returning(Id::Memmove, 9);
    let length = stand_in(Id::Memmove).function().len();
    // The last word that fits ends the function at the page's end.
    let fits = (4096 - length) / 4;
    for (words, matched, crosses) in [
        (fits, true, false),
        (fits + 1, false, true),
        // The prefix is the page's last 32 bytes.
        (1016, false, true),
        // Fewer than a prefix's bytes are left: no candidate is read.
        (1017, false, false),
        (1018, false, false),
    ] {
        let placed = stand_in(Id::Memmove).at(words);
        let (on, statistics) = run(&placed, &case, Some(bail(&placed)))?;
        let (off, _) = run(&placed, &case, None)?;
        assert_eq!(on, off, "at word {words}");
        assert_eq!(
            statistics.matched >= 1,
            matched,
            "at word {words}: {statistics:?}"
        );
        assert_eq!(
            statistics.refusals(Refusal::CrossesPage) >= 1,
            crosses,
            "at word {words}: {statistics:?}"
        );
        // A function that fits takes its composite.
        assert_eq!(statistics.installed >= 1, matched, "at word {words}");
    }
    Ok(())
}

#[test]
fn a_disabled_id_never_matches() -> Result<()> {
    let memmove = stand_in(Id::Memmove);
    let bzero = stand_in(Id::Bzero);
    let none = Config::with_table(table(&memmove, Id::Memmove), &[]);
    let (_, statistics) = run(&memmove, &returning(Id::Memmove, 11), Some(none))?;
    assert_eq!(statistics, Statistics::default());
    // With another routine on, only its function matches.
    let signature = |stand_in: &StandIn, id| {
        Signature::of(id, &stand_in.function(), stand_in.exit).expect("an eligible stand-in")
    };
    let both: Arc<[Signature]> = Arc::from([
        signature(&memmove, Id::Memmove),
        signature(&bzero, Id::Bzero),
    ]);
    let only_bzero = Config::with_table(both, &[Id::Bzero]);
    let (_, statistics) = run(
        &memmove,
        &returning(Id::Memmove, 11),
        Some(only_bzero.clone()),
    )?;
    assert_eq!(statistics.matched, 0);
    let (_, statistics) = run(&bzero, &returning(Id::Bzero, 12), Some(only_bzero))?;
    assert!(statistics.matched >= 1);
    assert_eq!(statistics.matched_signatures, 0b10);
    Ok(())
}

/// The Touch routine over touch's stand-in, enabled.
fn touch() -> Config {
    Config::with_table(table(&stand_in(Id::Touch), Id::Touch), &[Id::Touch])
}

/// A call of touch's stand-in with X0 and X1 at data offsets `x0` and `x1`
/// (offsets wrap: any address is one) and page `unreadable` unreadable.
fn touching(x0: u64, x1: u64, unreadable: Option<usize>) -> Case {
    Case {
        x: [x0, x1, 0],
        unreadable,
        read_only: None,
        ..cases(Id::Touch, 21, 1).remove(0)
    }
}

/// Touch's X1, readable on the data mapping's third page.
const X1: u64 = 2 * PAGE as u64 + 5;

#[test]
fn a_fault_in_the_native_prefix_is_taken_by_the_original() -> Result<()> {
    // X0's page is unreadable: the fast path's load of it faults, and the
    // original then loads X1 and faults at its second instruction.
    let stand_in = stand_in(Id::Touch);
    let case = touching(100, X1, Some(0));
    let (on, statistics) = run(&stand_in, &case, Some(touch()))?;
    let (off, _) = run(&stand_in, &case, None)?;
    assert_eq!(on, off);
    let (_, far, elr, _) = on.abort.expect("the original's data abort");
    assert_eq!((far, elr), (data_base() + 100, stand_in.pc() + 4));
    assert_eq!(on.retired, 1);
    assert_eq!((statistics.installed, statistics.bypassed), (1, 1));
    Ok(())
}

#[test]
fn a_call_that_always_faults_bypasses_every_time_and_stays_exact() -> Result<()> {
    let stand_in = stand_in(Id::Touch);
    let case = touching(100, X1, Some(0));
    let off = run(&stand_in, &case, None)?.0;
    let mut session = Session::new(&stand_in, Some(touch()))?;
    for call in 1..=3 {
        assert_eq!(session.run(&case)?, off, "call {call}");
        assert_eq!(session.hle().bypassed, call);
    }
    assert_eq!(session.hle().installed, 1);
    Ok(())
}

#[test]
fn every_touch_combination_agrees() -> Result<()> {
    let stand_in = stand_in(Id::Touch);
    // X0 and X1 apart with neither, X0's or X1's page unreadable; both on
    // one unreadable page; and X0 on a read-only page, which reads.
    let read_only = Case {
        read_only: Some(0),
        ..touching(100, X1, None)
    };
    for (case, bypassed) in [
        (touching(100, X1, None), 0),
        (touching(100, X1, Some(0)), 1),
        (touching(100, X1, Some(2)), 0),
        (touching(100, 107, Some(0)), 1),
        (read_only, 0),
    ] {
        let (on, statistics) = run(&stand_in, &case, Some(touch()))?;
        let (off, _) = run(&stand_in, &case, None)?;
        assert_eq!(agree(&stand_in, &case, &off, &on), Ok(()), "{case:?}");
        assert_eq!(statistics.bypassed, bypassed, "{:x?}", case.x);
        assert_eq!(statistics.installed, 1);
    }
    Ok(())
}

#[test]
fn a_touch_native_agrees_with_the_original_on_random_cases() -> Result<()> {
    // Unreadable and read-only pages anywhere: a call bypasses exactly when
    // X0's page is unreadable, the only fault its fast path can take.
    let stand_in = stand_in(Id::Touch);
    let mut on = Session::new(&stand_in, Some(touch()))?;
    let mut off = Session::new(&stand_in, None)?;
    let (mut bypassed, mut aborted) = (0, 0);
    for case in case_stream(Id::Touch, 0x70c4, case_count(4000)) {
        let before = on.hle().bypassed;
        let (expected, got) = (off.run(&case)?, on.run(&case)?);
        assert_eq!(
            agree(&stand_in, &case, &expected, &got),
            Ok(()),
            "{case:x?}"
        );
        let faults = case.unreadable == Some(case.x[0] as usize / PAGE);
        assert_eq!(
            on.hle().bypassed - before,
            u64::from(faults),
            "{:x?}",
            case.x
        );
        assert!(!mapped(&case) || !faults);
        bypassed += usize::from(faults);
        aborted += usize::from(got.abort.is_some());
    }
    assert!(bypassed > 0 && aborted > bypassed, "{bypassed} bypassed");
    assert_eq!(on.hle().installed, 1);
    Ok(())
}

#[test]
fn an_out_of_range_pointer_bails_before_the_fast_path_loads() -> Result<()> {
    use stela_engine::machine::Host as _;
    let stand_in = stand_in(Id::Touch);
    let limit = Session::new(&stand_in, None)?
        .machine()
        .host
        .placement()
        .check()
        .lower_limit();
    let at = |address: u64| address.wrapping_sub(data_base());
    for (address, faults) in [
        // At and past the limit, in the upper half and top-byte tagged: the
        // range check bails and the original's access takes the fault.
        (limit, false),
        (limit + PAGE as u64, false),
        (u64::MAX - 7, false),
        ((data_base() + 100) | 0x5a << 56, false),
        // In range and not mapped: the fast path's load faults and the
        // original takes it.
        (0x10, true),
        // In the test host's five-level range but past 2^47: on a
        // four-level host the fast path's load takes a #GP, which bypasses
        // as a #PF does, and the original's access stops the engine as an
        // unsupported address check, alike with HLE.
        (limit - 64, true),
    ] {
        let case = touching(at(address), X1, None);
        let (on, statistics) = run(&stand_in, &case, Some(touch()))?;
        let (off, _) = run(&stand_in, &case, None)?;
        assert_eq!(on, off, "{address:#x}");
        let canonical = !(1 << 47..0xffff_8000_0000_0000).contains(&address);
        assert!(
            on.abort.is_some() || !canonical,
            "{address:#x}: {:?}",
            on.stop
        );
        assert_eq!(statistics.bypassed, u64::from(faults), "{address:#x}");
    }
    Ok(())
}

#[test]
fn software_step_never_enters_the_native() -> Result<()> {
    // MDSCR_EL1.SS with the OS lock clear, and PSTATE.SS: each instruction
    // is a block of its own, which HLE never matches.
    let stand_in = stand_in(Id::Touch);
    let case = touching(100, X1, None);
    let mut outcomes = Vec::new();
    for hle in [None, Some(touch())] {
        let mut session = Session::new(&stand_in, hle)?;
        let machine = session.machine();
        machine
            .cpu
            .registers
            .set_raw(Register::OSLSR_EL1, 0, 0b1000);
        machine.cpu.registers.set_raw(Register::MDSCR_EL1, 0, 1);
        machine.cpu.pstate.ss = true;
        let outcome = session.run(&case)?;
        outcomes.push((outcome, session.hle()));
    }
    let ((off, _), (on, statistics)) = (&outcomes[0], &outcomes[1]);
    assert_eq!(on, off);
    let (esr, ..) = on.abort.expect("a software step exception");
    assert_eq!(exception_class(esr), 0x32, "{esr:#x}");
    assert_eq!(on.retired, 1);
    assert_eq!(*statistics, Statistics::default());
    Ok(())
}

#[test]
fn a_function_longer_than_the_first_window_is_matched_once_per_compile() -> Result<()> {
    // 39 NOPs and a `ret`: 160 bytes, past the 128 a compile reads first,
    // so its block is compiled from the whole window after the first.
    let mut words = vec![a64::NOP; 39];
    words.push(a64::RET);
    let long = StandIn {
        id: Id::Bail,
        words,
        entry: 0,
        exit: 156,
        returns: false,
        caller: None,
        driven: false,
    };
    let case = &cases(Id::Memmove, 17, 1)[0];
    let (on, statistics) = run(&long, case, Some(bail(&long)))?;
    let (off, _) = run(&long, case, None)?;
    assert_eq!(on, off);
    assert_eq!(on.stop, Stop::Breakpoint(long.end()));
    assert_eq!(statistics.matched, 1, "{statistics:?}");
    Ok(())
}

#[test]
fn a_word_the_chip_runs_as_another_refuses_the_function() -> Result<()> {
    // abs x9, x9 (FEAT_CSSC, from the M5) is UDF on the test host's M2,
    // where bzero's padding NOP was: every call that stores reaches it.
    let mut lacking = stand_in(Id::Bzero);
    lacking.words[5] = 0xdac0_2129;
    let case = cases(Id::Bzero, 15, 64)
        .into_iter()
        .find(|case| reference(Id::Bzero, case).is_some() && case.x[1] > 0)
        .expect("a storing case");
    let (on, statistics) = run(&lacking, &case, Some(bail(&lacking)))?;
    let (off, _) = run(&lacking, &case, None)?;
    assert_eq!(on, off);
    assert!(off.abort.is_some(), "the original's Undefined exception");
    assert!(statistics.matched >= 1, "{statistics:?}");
    assert!(
        statistics.refusals(Refusal::Admitted) >= 1,
        "{statistics:?}"
    );
    assert_eq!(statistics.installed, 0);
    // The same function with the NOP matches with no refusal.
    let (_, statistics) = run(
        &stand_in(Id::Bzero),
        &case,
        Some(bail(&stand_in(Id::Bzero))),
    )?;
    assert!(statistics.matched >= 1, "{statistics:?}");
    assert_eq!(statistics.refused, [0; Refusal::COUNT]);
    Ok(())
}

#[test]
fn an_fp_trapped_or_alignment_checked_compile_never_matches() -> Result<()> {
    let stand_in = stand_in(Id::Memmove);
    let case = returning(Id::Memmove, 13);
    let reference = run(&stand_in, &case, None)?.0;
    for (register, set) in [
        // CPACR_EL1.FPEN traps EL0's FP and SIMD.
        (Register::CPACR_EL1, None),
        // SCTLR_EL1.A checks data alignment.
        (Register::SCTLR_EL1, Some(1 << 1)),
    ] {
        let mut session = Session::new(&stand_in, Some(bail(&stand_in)))?;
        let registers = &mut session.machine().cpu.registers;
        let value = set.map_or(0, |bits| registers.get(register, 0) | bits);
        registers.set_raw(register, 0, value);
        assert_eq!(session.run(&case)?, reference, "{register:?}");
        assert_eq!(session.hle(), Statistics::default(), "{register:?}");
    }
    Ok(())
}

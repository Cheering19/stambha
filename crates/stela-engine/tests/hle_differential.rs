//! HLE's differential suites: each stand-in original against the Rust model
//! of its C function, and the harness's comparison of two runs, which every
//! native routine's suite will run the translated original against.
#![cfg(all(
    feature = "test-support",
    feature = "std",
    target_arch = "x86_64",
    target_os = "linux"
))]
use anyhow::Result;
use stela_engine::{
    hle::{self, Config, Id, Signature},
    machine::Stop,
};
#[path = "support/hle_harness.rs"]
mod hle_harness;
use hle_harness::{a64, *};

#[test]
fn a64_encoders_produce_known_words() {
    assert_eq!(a64::ldrb_post(8, 0, 1), 0x3840_1408);
    assert_eq!(a64::subs_imm(2, 2, 1), 0xf100_0442);
    assert_eq!(a64::RET, 0xd65f_03c0);
    assert_eq!(a64::BTI_C, 0xd503_245f);
    // The rest as llvm-mc encodes them, the branches' offsets by their
    // fields.
    assert_eq!(a64::ldrb_pre(5, 1, -1), 0x385f_fc25); // ldrb w5,[x1,#-1]!
    assert_eq!(a64::strb_pre(5, 3, -1), 0x381f_fc65); // strb w5,[x3,#-1]!
    assert_eq!(a64::strb_post(a64::ZR, 3, 1), 0x3800_147f); // strb wzr,[x3],#1
    assert_eq!(a64::ldrb(2, 0), 0x3940_0002); // ldrb w2,[x0]
    assert_eq!(a64::subs_w(3, 3, 4), 0x6b04_0063); // subs w3,w3,w4
    assert_eq!(a64::sub(4, 0, 1), 0xcb01_0004); // sub x4,x0,x1
    assert_eq!(a64::add(0, 8, 9), 0x8b09_0100); // add x0,x8,x9
    assert_eq!(a64::cmp(4, 2), 0xeb02_009f); // cmp x4,x2
    assert_eq!(a64::mov(3, 0), 0xaa00_03e3); // mov x3,x0
    assert_eq!(a64::mov_w(0, 3), 0x2a03_03e0); // mov w0,w3
    assert_eq!(a64::movz_w(0, 0), 0x5280_0000); // mov w0,#0
    assert_eq!(a64::cbz(2, 16), 0xb400_0202); // cbz x2,.+64
    assert_eq!(a64::cbz_w(2, 3), 0x3400_0062); // cbz w2,.+12
    assert_eq!(a64::cbnz_w(2, -1), 0x35ff_ffe2); // cbnz w2,.-4
    assert_eq!(a64::b_cond(a64::LO, 6), 0x5400_00c3); // b.lo .+24
    assert_eq!(a64::b_cond(a64::NE, -3), 0x54ff_ffa1); // b.ne .-12
    assert_eq!(a64::b(7), 0x1400_0007); // b .+28
}

#[test]
fn stand_ins_are_eligible() {
    for id in STAND_INS.into_iter().chain([Id::Bail]) {
        let stand_in = stand_in(id);
        let function = stand_in.function();
        assert!(
            Signature::of(id, &function, stand_in.exit).is_ok(),
            "{id:?}: {:?}",
            hle::eligible(&function, stand_in.exit)
        );
        // Placed further into its page, it is the same function.
        assert_eq!(stand_in.at(5).function(), function, "{id:?}");
    }
}

/// A case's arguments and faulting page, for a failure's message.
fn named(case: &Case) -> String {
    format!(
        "x {:x?} unreadable {:?} read-only {:?}",
        case.x, case.unreadable, case.read_only
    )
}

#[test]
fn stand_ins_match_the_reference_model() -> Result<()> {
    for id in STAND_INS {
        let stand_in = stand_in(id);
        let mut session = Session::new(&stand_in, None)?;
        let (mut returned, mut faulted) = (0, 0);
        for (index, case) in cases(id, 0x51ab_1e00 + id.bit() as u64, case_count(4000))
            .iter()
            .enumerate()
        {
            let outcome = session.run(case)?;
            match reference(id, case) {
                Some((x0, memory)) => {
                    assert_eq!(
                        outcome.stop,
                        Stop::Breakpoint(stand_in.end()),
                        "{id:?} case {index}: {}",
                        named(case)
                    );
                    if stand_in.returns {
                        assert_eq!(
                            Some(outcome.state.x[0]),
                            x0,
                            "{id:?} case {index}: {}",
                            named(case)
                        );
                    }
                    assert!(
                        outcome.memory == memory,
                        "{id:?} case {index}: memory, {}",
                        named(case)
                    );
                    returned += 1;
                }
                None => {
                    assert!(
                        outcome.abort.is_some(),
                        "{id:?} case {index}: {:?} where the model faults, {}",
                        outcome.stop,
                        named(case)
                    );
                    faulted += 1;
                }
            }
        }
        // The cases reach both ends: most calls return and some fault.
        assert!(
            returned > 2 * faulted && faulted > 0,
            "{id:?}: {returned} returned, {faulted} faulted"
        );
    }
    Ok(())
}

/// The harness's own check: `agree` refuses a run that changed what the
/// ABI keeps, an X0 result or memory, and allows caller-saved registers
/// and flags to differ; a faulting call must match in full.
#[test]
fn agree_refuses_a_changed_result() -> Result<()> {
    let stand_in = stand_in(Id::Memmove);
    let all = cases(Id::Memmove, 7, 256);
    let returning = all
        .iter()
        .find(|case| reference(Id::Memmove, case).is_some() && case.x[2] > 8)
        .expect("a returning case");
    let off = run(&stand_in, returning, None)?.0;
    assert_eq!(
        agree(
            &stand_in,
            returning,
            &off,
            &run(&stand_in, returning, None)?.0
        ),
        Ok(())
    );
    let mut on = run(&stand_in, returning, None)?.0;
    on.state.x[9] ^= 1;
    on.state.nzcv ^= 0x4000_0000;
    on.state.v[16][0] ^= 1;
    assert_eq!(
        agree(&stand_in, returning, &off, &on),
        Ok(()),
        "caller-saved"
    );
    type Change = fn(&mut Outcome);
    let changes: [(&str, Change); 8] = [
        ("x19", |on| on.state.x[19] ^= 1),
        ("x29", |on| on.state.x[29] ^= 1),
        ("sp", |on| on.state.sp ^= 16),
        ("x30", |on| on.state.x[30] ^= 4),
        ("v8", |on| on.state.v[8][0] ^= 1),
        ("x0", |on| on.state.x[0] ^= 1),
        ("memory", |on| on.memory[0] ^= 1),
        ("btype", |on| on.pstate.btype = 0b10),
    ];
    for (name, change) in changes {
        let mut on = run(&stand_in, returning, None)?.0;
        change(&mut on);
        assert!(agree(&stand_in, returning, &off, &on).is_err(), "{name}");
    }
    // Both runs alike, X0 and memory against the model.
    for (name, change) in [changes[5], changes[6]] {
        let (mut off, mut on) = (
            run(&stand_in, returning, None)?.0,
            run(&stand_in, returning, None)?.0,
        );
        change(&mut off);
        change(&mut on);
        assert!(
            agree(&stand_in, returning, &off, &on).is_err(),
            "{name} against the model"
        );
    }
    let faulting = all
        .iter()
        .find(|case| reference(Id::Memmove, case).is_none())
        .expect("a faulting case");
    let off = run(&stand_in, faulting, None)?.0;
    let mut on = run(&stand_in, faulting, None)?.0;
    assert_eq!(agree(&stand_in, faulting, &off, &on), Ok(()));
    on.state.x[9] ^= 1;
    assert!(
        agree(&stand_in, faulting, &off, &on).is_err(),
        "a fault in full"
    );
    let mut on = run(&stand_in, faulting, None)?.0;
    let spsr = &mut on.abort.as_mut().expect("an abort").3;
    *spsr ^= 0b10 << 10;
    assert!(
        agree(&stand_in, faulting, &off, &on).is_err(),
        "an abort's SPSR (BTYPE)"
    );
    Ok(())
}

/// A call through `blr x16` into a guarded page enters with BTYPE 0b10:
/// memset's `bti c` admits it and the call runs as the model says; every
/// other stand-in's first word takes a Branch Target exception at its entry,
/// with BTYPE saved in SPSR and memory untouched.
#[test]
fn a_call_through_blr_on_a_guarded_page_lands_as_its_first_word_admits() -> Result<()> {
    for id in STAND_INS {
        let called = stand_in(id).through_blr();
        let mut session = Session::new(&called, None)?;
        let (mut returned, mut refused) = (0, 0);
        for case in cases(id, 0x0b71 + id.bit() as u64, 64) {
            let outcome = session.run(&case)?;
            if id == Id::Memset {
                match reference(id, &case) {
                    Some((x0, memory)) => {
                        assert_eq!(
                            outcome.stop,
                            Stop::Breakpoint(called.end()),
                            "{}",
                            named(&case)
                        );
                        assert_eq!(Some(outcome.state.x[0]), x0, "{}", named(&case));
                        assert!(outcome.memory == memory, "{}", named(&case));
                        assert_eq!(outcome.pstate.btype, 0);
                        returned += 1;
                    }
                    None => assert!(outcome.abort.is_some(), "{}", named(&case)),
                }
                continue;
            }
            let (esr, _, elr, spsr) = outcome.abort.expect("a Branch Target exception");
            assert_eq!(exception_class(esr), BRANCH_TARGET, "{id:?} {esr:#x}");
            assert_eq!(esr & 3, 0b10, "{id:?}: ESR's BTYPE");
            assert_eq!((elr, spsr >> 10 & 3), (called.pc(), 0b10), "{id:?}");
            assert!(outcome.memory == case.contents, "{id:?}");
            refused += 1;
        }
        assert!(returned + refused > 32, "{id:?}");
    }
    Ok(())
}

/// A wrong original shows up against the model: memmove that copies
/// forward only, run on backward overlaps.
#[test]
fn a_wrong_stand_in_disagrees_with_the_reference_model() -> Result<()> {
    let mut forward = stand_in(Id::Memmove);
    forward.words[4] = a64::NOP; // b.lo back
    let mut session = Session::new(&forward, None)?;
    let mut wrong = 0;
    for case in cases(Id::Memmove, 11, 200) {
        let (dst, src, n) = (case.x[0], case.x[1], case.x[2]);
        let backward = dst > src && dst < src + n;
        if let (true, Some((_, memory))) = (backward, reference(Id::Memmove, &case)) {
            wrong += usize::from(session.run(&case)?.memory != memory);
        }
    }
    assert!(wrong > 0, "no backward overlap told the stand-ins apart");
    Ok(())
}

/// The harness's own check: a session's calls, after returns and aborts
/// alike, end as each would on a machine of its own, entered directly or
/// through `blr x16` on a guarded page (where a Branch Target exception
/// leaves FAR as it was, and its ERET brings BTYPE back).
#[test]
fn a_reused_session_runs_as_a_fresh_machine() -> Result<()> {
    let direct = [Id::Memmove, Id::Strlen, Id::Touch].map(stand_in);
    let called = [Id::Memset, Id::Touch].map(|id| stand_in(id).through_blr());
    for stand_in in direct.iter().chain(&called) {
        let id = stand_in.id;
        // Off, and on with each test routine that suits the stand-in: Bail
        // for all, Touch for touch's, whose fast path faults and bypasses
        // where X0's page is unreadable.
        let mut routines = vec![None, Some(Id::Bail)];
        if id == Id::Touch {
            routines.push(Some(Id::Touch));
        }
        for routine in routines {
            let hle = routine.map(|routine| {
                Config::with_table(table(stand_in, routine), &[routine])
                    .with_max_bytes(CASE_MAX_BYTES)
            });
            let mut session = Session::new(stand_in, hle.clone())?;
            let (mut returned, mut aborted, mut after_abort) = (0, 0, false);
            let count = if stand_in.caller.is_some() { 48 } else { 120 };
            for case in cases(id, 23, count) {
                let bypassed = session.hle().bypassed;
                let reused = session.run(&case)?;
                // A fast path faults only on a page that is not mapped.
                let mapped = case.unreadable.is_none() && case.read_only.is_none();
                assert!(!mapped || session.hle().bypassed == bypassed);
                // The first calls, every abort and every call after one.
                if returned < 4 || after_abort || reused.abort.is_some() {
                    let fresh = run(stand_in, &case, hle.clone())?.0;
                    assert_eq!(reused, fresh, "{id:?} {routine:?}: {}", named(&case));
                }
                after_abort = reused.abort.is_some();
                returned += usize::from(reused.abort.is_none());
                aborted += usize::from(reused.abort.is_some());
            }
            // Touch through `blr` always takes its Branch Target exception.
            let refused = stand_in.caller.is_some() && id == Id::Touch;
            assert!(
                aborted > 0 && (refused || returned > aborted),
                "{id:?} {routine:?}: {aborted} aborted"
            );
        }
    }
    Ok(())
}

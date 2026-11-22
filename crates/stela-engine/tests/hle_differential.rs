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
    hle::{self, Id, Signature},
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
        let mut session = Session::new(&stand_in)?;
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
    let off = run(&stand_in, returning)?;
    assert_eq!(
        agree(&stand_in, returning, &off, &run(&stand_in, returning)?),
        Ok(())
    );
    let mut on = run(&stand_in, returning)?;
    on.state.x[9] ^= 1;
    on.state.nzcv ^= 0x4000_0000;
    on.state.v[16][0] ^= 1;
    assert_eq!(
        agree(&stand_in, returning, &off, &on),
        Ok(()),
        "caller-saved"
    );
    type Change = fn(&mut Outcome);
    let changes: [(&str, Change); 7] = [
        ("x19", |on| on.state.x[19] ^= 1),
        ("x29", |on| on.state.x[29] ^= 1),
        ("sp", |on| on.state.sp ^= 16),
        ("x30", |on| on.state.x[30] ^= 4),
        ("v8", |on| on.state.v[8][0] ^= 1),
        ("x0", |on| on.state.x[0] ^= 1),
        ("memory", |on| on.memory[0] ^= 1),
    ];
    for (name, change) in changes {
        let mut on = run(&stand_in, returning)?;
        change(&mut on);
        assert!(agree(&stand_in, returning, &off, &on).is_err(), "{name}");
    }
    let faulting = all
        .iter()
        .find(|case| reference(Id::Memmove, case).is_none())
        .expect("a faulting case");
    let off = run(&stand_in, faulting)?;
    let mut on = run(&stand_in, faulting)?;
    assert_eq!(agree(&stand_in, faulting, &off, &on), Ok(()));
    on.state.x[9] ^= 1;
    assert!(
        agree(&stand_in, faulting, &off, &on).is_err(),
        "a fault in full"
    );
    Ok(())
}

/// A wrong original shows up against the model: memmove that copies
/// forward only, run on backward overlaps.
#[test]
fn a_wrong_stand_in_disagrees_with_the_reference_model() -> Result<()> {
    let mut forward = stand_in(Id::Memmove);
    forward.words[4] = a64::NOP; // b.lo back
    let mut session = Session::new(&forward)?;
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
/// alike, end as each would on a machine of its own.
#[test]
fn a_reused_session_runs_as_a_fresh_machine() -> Result<()> {
    for id in [Id::Memmove, Id::Strlen, Id::Touch] {
        let stand_in = stand_in(id);
        let mut session = Session::new(&stand_in)?;
        let (mut returned, mut aborted, mut after_abort) = (0, 0, false);
        for case in cases(id, 23, 120) {
            let reused = session.run(&case)?;
            // The first calls, every abort and every call after one.
            if returned < 4 || after_abort || reused.abort.is_some() {
                assert_eq!(reused, run(&stand_in, &case)?, "{id:?}: {}", named(&case));
            }
            after_abort = reused.abort.is_some();
            returned += usize::from(reused.abort.is_none());
            aborted += usize::from(reused.abort.is_some());
        }
        assert!(
            aborted > 0 && returned > aborted,
            "{id:?}: {aborted} aborted"
        );
    }
    Ok(())
}

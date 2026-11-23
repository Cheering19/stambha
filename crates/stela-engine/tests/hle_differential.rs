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
        for (index, case) in
            case_stream(id, 0x51ab_1e00 + id.bit() as u64, case_count(4000)).enumerate()
        {
            let case = &case;
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

// The natives against the originals. Each suite runs the original once
// per case and the native on every vector set the host has (SSE2, AVX2,
// AVX-512F), on machines reused across the cases.

const PAGE64: u64 = PAGE as u64;

/// The random suite of `id` (`case_count(4000)` cases): every case agrees,
/// some ran natively and some faulted.
fn random_suite(id: Id, seed: u64) -> Result<()> {
    let mut differential = Differential::new(id, CASE_MAX_BYTES)?;
    let (mut native, mut faulted) = (0, 0);
    for case in case_stream(id, seed, case_count(4000)) {
        let checked = differential.check(&case)?;
        native += usize::from(checked.native);
        faulted += usize::from(checked.off.abort.is_some());
    }
    assert!(
        native > 0 && faulted > 0,
        "{id:?}: {native} native, {faulted} faulted"
    );
    Ok(())
}

/// The sizes an exhaustive suite covers: every one to 300 and the class
/// and bound edges past it.
fn sizes() -> Vec<u64> {
    let mut sizes: Vec<u64> = (0..=300).collect();
    sizes.extend([
        383, 384, 385, 511, 512, 513, 1000, 1023, 1024, 1025, 2047, 2048, 2049, 4031, 4095, 4096,
        4097,
    ]);
    sizes
}

/// The byte positions a page boundary takes in a buffer of `n` bytes:
/// every one for small buffers (and in the acceptance run), both ends and
/// eighths for larger ones.
fn positions(n: u64) -> Vec<u64> {
    if n <= 80 || exhaustive() {
        return (0..=n).collect();
    }
    let mut at: Vec<u64> = (0..=40).chain(n - 40..=n).collect();
    at.extend((1..8).map(|eighth| n * eighth / 8));
    at
}

/// `x` as `id` takes its arguments from (dst, src, n): bzero's n is X1,
/// memset's byte X1, strlen's string X0.
fn arguments(id: Id, x: [u64; 3]) -> [u64; 3] {
    match id {
        Id::Bzero => [x[0], x[2], 0],
        Id::Memset => [x[0], 0x1234_56ab, x[2]],
        _ => x,
    }
}

/// Make a case of `id` over `n` bytes the one a suite means: memcmp's two
/// buffers equal (a compare that runs to `n`), strlen's string `n` nonzero
/// bytes and a NUL.
fn prepare(id: Id, case: &mut Case, n: u64) {
    let [x0, x1, _] = case.x;
    let (x0, x1, n) = (x0 as usize, x1 as usize, n as usize);
    match id {
        Id::Memcmp if x0 + n <= BYTES && x1 + n <= BYTES => {
            case.contents.copy_within(x0..x0 + n, x1);
        }
        Id::Strlen if x0 < BYTES => {
            let end = (x0 + n).min(BYTES);
            for byte in &mut case.contents[x0..end] {
                *byte |= 1;
            }
            if x0 + n < BYTES {
                case.contents[x0 + n] = 0;
            }
        }
        _ => {}
    }
}

/// Whether a fault-free, in-range call of `id` on `n` bytes runs natively,
/// where that is certain: to the bound for the others, and for strlen a
/// NUL well inside its bound.
fn expected(id: Id, n: u64) -> Option<bool> {
    let bound = u64::from(CASE_MAX_BYTES);
    match id {
        Id::Strlen if n + 64 >= bound && n <= bound + 64 => None,
        Id::Strlen => Some(n < bound),
        _ => Some(n <= bound),
    }
}

#[test]
fn memmove_agrees_with_the_original_on_random_cases() -> Result<()> {
    random_suite(Id::Memmove, 0x00c0_0001)
}

/// `id` (a copy, fill or compare) at every size and every alignment pair
/// mod 16 (in the acceptance run; sixteen pairs per size otherwise): each
/// agrees, and runs natively up to the bound.
fn size_suite(id: Id) -> Result<()> {
    let mut differential = Differential::new(id, CASE_MAX_BYTES)?;
    for n in sizes() {
        for dst in 0..16u64 {
            let pairs: Vec<u64> = match exhaustive() {
                true => (0..16).collect(),
                false => vec![(dst * 7 + n) % 16],
            };
            for src in pairs {
                let x = arguments(id, [64 + dst, 2 * PAGE64 + 64 + src, n]);
                let mut case = case_of(x, n << 8 | dst << 4 | src, None, None);
                prepare(id, &mut case, n);
                let checked = differential.check(&case)?;
                if let Some(native) = expected(id, n) {
                    assert_eq!(checked.native, native, "{id:?} n {n} dst {dst} src {src}");
                }
            }
        }
    }
    Ok(())
}

#[test]
fn memmove_agrees_at_every_size_and_every_alignment_pair_mod_16() -> Result<()> {
    size_suite(Id::Memmove)
}

#[test]
fn memmove_agrees_for_forward_backward_exact_and_adjacent_overlaps() -> Result<()> {
    let mut differential = Differential::new(Id::Memmove, CASE_MAX_BYTES)?;
    let mut checked = 0;
    for n in sizes().into_iter().filter(|&n| n <= 4096) {
        let apart: Vec<u64> = [
            0, 1, 2, 7, 8, 15, 16, 17, 31, 32, 33, 63, 64, 65, 127, 128, 129, 255, 256, 4095,
        ]
        .into_iter()
        .chain([n.saturating_sub(1), n, n + 1])
        .filter(|&apart| apart <= n + 1 && 3000 + 13 + apart + n <= 4 * PAGE64)
        .collect();
        for apart in apart {
            let low = 3000 + (n % 13);
            for (dst, src) in [(low, low + apart), (low + apart, low)] {
                let case = case_of([dst, src, n], n ^ apart << 20, None, None);
                assert!(
                    differential.check(&case)?.native,
                    "n {n} dst {dst} src {src}"
                );
                checked += 1;
            }
        }
    }
    assert!(checked > 1000);
    Ok(())
}

/// `id` with the buffer of argument `which` crossing into page 2 at every
/// byte position (`positions`), that page unreadable or read-only, the
/// other buffer at `other`: each call agrees with the original, the
/// original faults in some, the native bypasses in some, and a buffer
/// whose first page faults is left untouched.
fn fault_suite(id: Id, which: usize, other: u64, sizes: &[u64]) -> Result<()> {
    let mut differential = Differential::new(id, CASE_MAX_BYTES)?;
    let (mut faulted, mut bypassed) = (0, 0);
    for &n in sizes {
        for k in positions(n) {
            for kind in 0..2u64 {
                let mut x = [other, other, n];
                x[which] = 2 * PAGE64 - k;
                let x = arguments(id, x);
                let (unreadable, read_only) = match kind {
                    0 => (Some(2), None),
                    _ => (None, Some(2)),
                };
                let mut case = case_of(x, n << 16 | k << 1 | kind, unreadable, read_only);
                prepare(id, &mut case, n);
                let checked = differential.check(&case)?;
                faulted += usize::from(checked.off.abort.is_some());
                bypassed += usize::from(checked.bypassed);
                if checked.off.abort.is_some() && k == 0 && which == 0 {
                    for on in &checked.on {
                        assert!(on.memory == case.contents, "{id:?} n {n}");
                    }
                }
            }
        }
    }
    assert!(
        faulted > 0 && bypassed > 0,
        "{id:?}: {faulted} faulted, {bypassed} bypassed"
    );
    Ok(())
}

const FAULT_SIZES: [u64; 22] = [
    1, 2, 3, 4, 5, 7, 8, 9, 15, 16, 17, 31, 32, 33, 64, 100, 255, 256, 257, 300, 1024, 4096,
];

#[test]
fn memmove_faults_as_the_original_at_every_byte_of_a_page_crossing() -> Result<()> {
    // The destination crossing, the source inside page 0; then the source
    // crossing, the destination inside page 3.
    fault_suite(Id::Memmove, 0, 16, &FAULT_SIZES)?;
    fault_suite(Id::Memmove, 1, 3 * PAGE64 + 16, &FAULT_SIZES[..21])
}

/// `id` (a store) with its destination on a read-only page 2, at its start,
/// inside it, and crossing into it: each agrees, faults, and where its
/// first page is read-only leaves memory untouched.
fn read_only_suite(id: Id) -> Result<()> {
    let mut differential = Differential::new(id, CASE_MAX_BYTES)?;
    for n in [1, 16, 100, 256, 257, 4096] {
        for dst in [2 * PAGE64, 2 * PAGE64 + 100, 2 * PAGE64 - 8] {
            let src = if dst < 2 * PAGE64 { 64 } else { 3 * PAGE64 };
            let x = arguments(id, [dst, src.min(4 * PAGE64 - n), n]);
            let case = case_of(x, n ^ dst, None, Some(2));
            let checked = differential.check(&case)?;
            let reaches = dst + n > 2 * PAGE64;
            assert_eq!(
                checked.off.abort.is_some(),
                reaches,
                "{id:?} n {n} dst {dst}"
            );
            if dst >= 2 * PAGE64 {
                for on in &checked.on {
                    assert!(on.memory == case.contents, "{id:?} n {n} dst {dst}");
                }
            }
        }
    }
    Ok(())
}

#[test]
fn a_read_only_destination_page_faults_as_the_original_with_memory_untouched() -> Result<()> {
    read_only_suite(Id::Memmove)
}

/// The range's limit a session's machine runs with.
fn limit() -> Result<u64> {
    use stela_engine::machine::Host as _;
    let mut session = Session::new(&stand_in(Id::Touch), None)?;
    Ok(session.machine().host.placement().check().lower_limit())
}

/// The arguments every native bails on: `n = u64::MAX`, a buffer at
/// `2^64 - 8`, a top-byte-tagged pointer and a buffer ending past the
/// range's limit, in each pointer argument: each agrees with the original
/// and none runs natively.
fn bail_suite(id: Id) -> Result<()> {
    let mut differential = Differential::new(id, CASE_MAX_BYTES)?;
    let base = data_base();
    let limit = limit()?;
    let absolute = |address: u64| address.wrapping_sub(base);
    let mut all: Vec<[u64; 3]> = vec![
        [64, 4096, u64::MAX],
        [absolute(u64::MAX - 7), 64, 16],
        [64 | 0x5a << 56, 4096, 16],
        [absolute(limit - 8), 64, 16],
    ];
    if matches!(id, Id::Memmove | Id::Memcmp) {
        all.extend([
            [64, absolute(u64::MAX - 7), 16],
            [64, 4096 | 0x5a << 56, 16],
            [64, absolute(limit - 8), 16],
        ]);
    }
    if id == Id::Strlen {
        // Within max_bytes of the limit, even with its NUL in reach.
        all.push([absolute(limit - 100), 0, 0]);
    }
    for (index, x) in all.into_iter().enumerate() {
        let case = case_of(arguments(id, x), index as u64, None, None);
        // The original runs in both, which the model need not describe.
        let checked = differential.check_bailed(&case)?;
        assert!(!checked.native, "{id:?} {x:x?}");
    }
    Ok(())
}

#[test]
fn wrapping_tagged_or_out_of_range_arguments_bail() -> Result<()> {
    bail_suite(Id::Memmove)
}

#[test]
fn memmove_runs_natively_to_max_bytes_and_bails_past_it() -> Result<()> {
    let mut differential = Differential::new(Id::Memmove, CASE_MAX_BYTES)?;
    for (n, native) in [
        (0, true),
        (1, true),
        (256, true),
        (257, true),
        (4096, true),
        (4097, false),
    ] {
        for (dst, src) in [(64, 8192 + 64), (8192 + 64, 64), (100, 101), (101, 100)] {
            let case = case_of([dst, src, n], n, None, None);
            let checked = differential.check(&case)?;
            assert_eq!(checked.native, native, "n {n} dst {dst} src {src}");
            if !native {
                assert!(
                    checked
                        .on
                        .iter()
                        .all(|on| on.retired == checked.off.retired)
                );
            }
        }
    }
    // A zero-length call touches nothing, wherever its pointers point.
    let case = case_of([u64::MAX - 100, 1 << 40, 0], 9, None, None);
    assert!(differential.check(&case)?.native);
    Ok(())
}

#[test]
fn large_overlapping_moves_agree_in_both_directions() -> Result<()> {
    let mut differential = Differential::new(Id::Memmove, CASE_MAX_BYTES)?;
    for n in [257, 300, 511, 1000, 2048, 4000, 4096] {
        for apart in [1, 15, 16, 17, 31, 32, 33, 63, 64, 65, 255, 4095] {
            for skew in [0, 1, 7, 33] {
                let low = 512 + skew;
                if low + apart + n > 4 * PAGE64 {
                    continue;
                }
                for (dst, src) in [(low, low + apart), (low + apart, low)] {
                    let case = case_of([dst, src, n], n ^ apart ^ skew, None, None);
                    assert!(
                        differential.check(&case)?.native,
                        "n {n} dst {dst} src {src}"
                    );
                }
            }
        }
    }
    Ok(())
}

#[test]
fn a_page_unreadable_or_read_only_anywhere_in_a_large_move_faults_as_the_original() -> Result<()> {
    let mut differential = Differential::new(Id::Memmove, CASE_MAX_BYTES)?;
    let mut faulted = 0;
    for n in [257, 1000, 4096] {
        for page in 0..4 {
            for (dst, src) in [(100, 8292), (8292, 100), (4000, 4100), (4100, 4000)] {
                for kind in 0..2 {
                    let (unreadable, read_only) = match kind {
                        0 => (Some(page), None),
                        _ => (None, Some(page)),
                    };
                    let case = case_of([dst, src, n], n ^ dst ^ page as u64, unreadable, read_only);
                    let checked = differential.check(&case)?;
                    faulted += usize::from(checked.off.abort.is_some());
                }
            }
        }
    }
    assert!(faulted > 20, "{faulted}");
    Ok(())
}

#[test]
fn bzero_agrees_with_the_original_on_random_cases() -> Result<()> {
    random_suite(Id::Bzero, 0x00c0_0002)
}

#[test]
fn memset_agrees_with_the_original_on_random_cases() -> Result<()> {
    random_suite(Id::Memset, 0x00c0_0003)
}

#[test]
fn fills_agree_at_every_size_and_alignment() -> Result<()> {
    size_suite(Id::Bzero)?;
    size_suite(Id::Memset)
}

#[test]
fn fills_fault_as_the_original_at_every_byte_of_a_page_crossing() -> Result<()> {
    fault_suite(Id::Bzero, 0, 0, &FAULT_SIZES)?;
    fault_suite(Id::Memset, 0, 0, &FAULT_SIZES)
}

#[test]
fn a_read_only_destination_page_faults_as_the_original_with_memory_untouched_in_fills() -> Result<()>
{
    read_only_suite(Id::Bzero)?;
    read_only_suite(Id::Memset)
}

#[test]
fn wrapping_tagged_or_out_of_range_arguments_bail_in_fills() -> Result<()> {
    bail_suite(Id::Bzero)?;
    bail_suite(Id::Memset)
}

#[test]
fn memset_stores_only_the_low_byte_of_w1() -> Result<()> {
    let mut differential = Differential::new(Id::Memset, CASE_MAX_BYTES)?;
    for n in [1u64, 7, 16, 33, 256, 257, 4096] {
        let case = case_of([100, 0x1234_56ab, n], n, None, None);
        let checked = differential.check(&case)?;
        assert!(checked.native, "n {n}");
        let end = 100 + n as usize;
        for on in &checked.on {
            assert!(on.memory[100..end].iter().all(|&byte| byte == 0xab));
            assert_eq!(on.memory[..100], case.contents[..100]);
            assert_eq!(on.memory[end..], case.contents[end..]);
        }
    }
    Ok(())
}

#[test]
fn memcmp_agrees_on_random_cases_including_x0_bit_for_bit() -> Result<()> {
    random_suite(Id::Memcmp, 0x00c0_0004)
}

#[test]
fn memcmp_agrees_at_every_size_and_alignment_of_equal_buffers() -> Result<()> {
    size_suite(Id::Memcmp)
}

#[test]
fn memcmp_finds_a_difference_at_every_offset_with_extreme_bytes() -> Result<()> {
    let mut differential = Differential::new(Id::Memcmp, CASE_MAX_BYTES)?;
    let mut results = std::collections::BTreeSet::new();
    for n in [
        1u64, 2, 3, 5, 8, 9, 15, 16, 17, 31, 32, 33, 47, 64, 100, 255, 256, 257, 1000, 4096,
    ] {
        for at in positions(n).into_iter().filter(|&at| at < n) {
            for (a, b) in [(0x00, 0xff), (0xff, 0x00), (0x7f, 0x80), (1, 2)] {
                let (x0, x1) = (64 + at % 5, 2 * PAGE64 + 100);
                let mut case = case_of([x0, x1, n], n ^ at << 12, None, None);
                prepare(Id::Memcmp, &mut case, n);
                case.contents[(x0 + at) as usize] = a;
                case.contents[(x1 + at) as usize] = b;
                let checked = differential.check(&case)?;
                assert!(checked.native, "n {n} at {at}");
                results.insert(checked.on[0].state.x[0]);
            }
        }
    }
    // u32(a) - u32(b), zero-extended: both signs of the difference.
    assert!(
        results.contains(&0xffff_ff01) && results.contains(&0xff),
        "{results:x?}"
    );
    Ok(())
}

#[test]
fn memcmp_faults_as_the_original_at_every_byte_of_a_page_crossing() -> Result<()> {
    fault_suite(Id::Memcmp, 0, 16, &FAULT_SIZES)?;
    fault_suite(Id::Memcmp, 1, 3 * PAGE64 + 16, &FAULT_SIZES[..21])
}

#[test]
fn wrapping_tagged_or_out_of_range_arguments_bail_memcmp() -> Result<()> {
    bail_suite(Id::Memcmp)
}

#[test]
fn strlen_agrees_on_random_cases() -> Result<()> {
    random_suite(Id::Strlen, 0x00c0_0005)
}

#[test]
fn strlen_agrees_at_every_length_and_alignment() -> Result<()> {
    size_suite(Id::Strlen)
}

#[test]
fn strlen_agrees_with_nuls_at_and_around_page_ends() -> Result<()> {
    let mut differential = Differential::new(Id::Strlen, CASE_MAX_BYTES)?;
    for start in [
        0u64, 1, 15, 16, 17, 31, 32, 33, 63, 64, 4000, 4095, 4096, 8191,
    ] {
        for page_end in [PAGE64, 2 * PAGE64, 3 * PAGE64] {
            for around in [-2i64, -1, 0, 1, 2] {
                let nul = page_end as i64 + around;
                if nul < start as i64 {
                    continue;
                }
                let n = nul as u64 - start;
                let mut case = case_of([start, 0, 0], start ^ nul as u64, None, None);
                prepare(Id::Strlen, &mut case, n);
                let checked = differential.check(&case)?;
                if let Some(native) = expected(Id::Strlen, n) {
                    assert_eq!(checked.native, native, "start {start} n {n}");
                }
            }
        }
    }
    Ok(())
}

#[test]
fn strlen_faults_as_the_original_when_the_string_runs_into_an_unreadable_page() -> Result<()> {
    fault_suite(Id::Strlen, 0, 0, &FAULT_SIZES)
}

#[test]
fn strlen_bails_without_a_nul_in_max_bytes() -> Result<()> {
    let mut differential = Differential::new(Id::Strlen, 256)?;
    for (n, native) in [
        (100u64, true),
        (180, true),
        (300, false),
        (1000, false),
        (4000, false),
    ] {
        let mut case = case_of([7, 0, 0], n, None, None);
        prepare(Id::Strlen, &mut case, n);
        assert_eq!(differential.check(&case)?.native, native, "n {n}");
    }
    Ok(())
}

#[test]
fn strlen_of_an_out_of_range_pointer_bails_to_the_original() -> Result<()> {
    bail_suite(Id::Strlen)
}

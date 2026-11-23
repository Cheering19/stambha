//! HLE natives reached as a real guest reaches them: called in a hot loop
//! through `bl` (paired host frames, the composite's forward link and the
//! `ret` block's paired return), entered by falling through from the word
//! before, rewritten by the guest and invalidated with IC IVAU, and called
//! between a load-exclusive and its store-exclusive. Each driver program
//! lies ahead of its stand-in in one code page.
#![cfg(all(
    feature = "test-support",
    feature = "std",
    target_arch = "x86_64",
    target_os = "linux"
))]
use anyhow::Result;
use stela_engine::{
    hle::{Config, Id},
    machine::{Stop, system::Register},
};
#[path = "support/hle_harness.rs"]
mod hle_harness;
use hle_harness::{a64::*, *};

/// `bl` by `words`.
fn bl(words: i32) -> u32 {
    0x9400_0000 | (words as u32 & 0x03ff_ffff)
}
/// `mov xD, #immediate` (`movz`).
fn movz(rd: u32, immediate: u32) -> u32 {
    0xd280_0000 | immediate << 5 | rd
}
/// `movk wD, #immediate, lsl 16`.
fn movk_w16(rd: u32, immediate: u32) -> u32 {
    0x72a0_0000 | immediate << 5 | rd
}
/// `adr xD, .+bytes`.
fn adr(rd: u32, bytes: u32) -> u32 {
    0x1000_0000 | (bytes & 3) << 29 | (bytes >> 2) << 5 | rd
}
/// `str wT,[xN]`.
fn str_w(rt: u32, rn: u32) -> u32 {
    0xb900_0000 | rn << 5 | rt
}
/// `ldxr xT,[xN]`.
fn ldxr(rt: u32, rn: u32) -> u32 {
    0xc85f_7c00 | rn << 5 | rt
}
/// `stxr wS, xT,[xN]`.
fn stxr(rs: u32, rt: u32, rn: u32) -> u32 {
    0xc800_7c00 | rs << 16 | rn << 5 | rt
}
const DSB_ISH: u32 = 0xd503_3b9f;
const ISB: u32 = 0xd503_3fdf;
/// `dc cvau, xN` and `ic ivau, xN`.
fn dc_cvau(rn: u32) -> u32 {
    0xd50b_7b20 | rn
}
fn ic_ivau(rn: u32) -> u32 {
    0xd50b_7520 | rn
}

/// `b` from word `from` to the word after the driven stand-in's last.
fn to_end(from: usize, driver: usize, function: &StandIn) -> u32 {
    b((driver + function.words.len() - from) as i32)
}

#[test]
fn the_call_encoders_match_llvm_mc() {
    assert_eq!(bl(4), 0x9400_0004);
    assert_eq!(movz(19, 64), 0xd280_0813); // mov x19,#64
    assert_eq!(movk_w16(9, 0x3800), 0x72a7_0009); // movk w9,#0x3800,lsl #16
    assert_eq!(adr(10, 56), 0x1000_01ca); // adr x10,.+56
    assert_eq!(str_w(9, 10), 0xb900_0149); // str w9,[x10]
    assert_eq!(ldxr(9, 10), 0xc85f_7d49); // ldxr x9,[x10]
    assert_eq!(stxr(11, 9, 10), 0xc80b_7d49); // stxr w11,x9,[x10]
    assert_eq!(dc_cvau(10), 0xd50b_7b2a);
    assert_eq!(ic_ivau(10), 0xd50b_752a);
}

/// `function` driven by `driver` under its own routine, bounded at the
/// cases' bound.
fn native(function: &StandIn, id: Id) -> Config {
    Config::with_table(table(function, id), &[id]).with_max_bytes(CASE_MAX_BYTES)
}

/// The registers and memory a caller may rely on after both runs: all
/// memory, X19-X30, SP, the low halves of V8-V15, and the stop.
fn kept(off: &Outcome, on: &Outcome) -> Result<(), String> {
    let (a, b) = (&off.state, &on.state);
    let checks = [
        ("stop", off.stop == on.stop),
        ("pc", off.pc == on.pc),
        ("memory", off.memory == on.memory),
        ("x19-x30", a.x[19..=30] == b.x[19..=30]),
        ("sp", a.sp == b.sp),
        ("v8-v15", (8..16).all(|v| a.v[v][0] == b.v[v][0])),
        ("abort", off.abort == on.abort),
    ];
    match checks.iter().find(|(_, equal)| !equal) {
        Some((name, _)) => Err(format!("{name} differs")),
        None => Ok(()),
    }
}

#[test]
fn calls_in_a_loop_link_through_the_native() -> Result<()> {
    const CALLS: u64 = 64;
    let memmove = stand_in(Id::Memmove);
    let driver = [
        mov(20, 0),          // 0: mov x20,x0
        mov(21, 1),          // 1: mov x21,x1
        mov(22, 2),          // 2: mov x22,x2
        movz(19, 64),        // 3: mov x19,#64
        mov(0, 20),          // 4: loop: mov x0,x20
        mov(1, 21),          // 5: mov x1,x21
        mov(2, 22),          // 6: mov x2,x22
        bl(5),               // 7: bl memmove (word 12)
        subs_imm(19, 19, 1), // 8: subs x19,x19,#1
        b_cond(NE, -5),      // 9: b.ne loop
        0,                   // 10: b end
        NOP,                 // 11
    ];
    let mut driver = driver.to_vec();
    driver[10] = to_end(10, driver.len(), &memmove);
    let function = memmove.driven_by(&driver);
    let mut off = Session::new(&function, None)?;
    let mut on = Session::new(&function, Some(native(&function, Id::Memmove)))?;
    let mut ran = 0;
    for case in cases(Id::Memmove, 0xca11, 48) {
        let (blocks_off, blocks_on) = (
            off.machine().statistics.blocks,
            on.machine().statistics.blocks,
        );
        let (a, b) = (off.run(&case)?, on.run(&case)?);
        kept(&a, &b).map_err(|why| anyhow::anyhow!("{why}: {:x?}", case.x))?;
        let returned = a.stop == Stop::Breakpoint(function.end());
        let in_bound = case.x[2] <= u64::from(CASE_MAX_BYTES) && case.x[2] > 0;
        if returned && in_bound && case.unreadable.is_none() && case.read_only.is_none() {
            // The prologue, then per call the driver's four, the composite,
            // the `ret` and the loop's two, then the branch to the end.
            assert_eq!(b.retired, 4 + CALLS * 8 + 1, "{:x?}", case.x);
            assert!(
                on.machine().statistics.blocks - blocks_on
                    < off.machine().statistics.blocks - blocks_off
            );
            ran += 1;
        }
    }
    assert!(ran > 8, "{ran}");
    assert_eq!(on.hle().installed, 1, "{:?}", on.hle());
    Ok(())
}

/// The entry reached without a call (Review Focus 4): with the caller's
/// X30, by a tail branch (`b memset`), whose target a block starts at, the
/// native runs; by falling through from the word before, the block that
/// word starts runs on into the function and no compile starts at its
/// entry, so the original runs translated, as without HLE. Both agree.
#[test]
fn an_entry_reached_by_a_tail_branch_runs_the_native_and_a_fall_through_the_original() -> Result<()>
{
    let memset = stand_in(Id::Memset);
    // mov x2,#40; b memset (word 2) | mov x2,#40 into memset's first word.
    let branched = memset.driven_by(&[movz(2, 40), b(2), NOP]);
    let fallen = memset.driven_by(&[movz(2, 40)]);
    for (function, natively) in [(branched, true), (fallen, false)] {
        let mut off = Session::new(&function, None)?;
        let mut on = Session::new(&function, Some(native(&function, Id::Memset)))?;
        let mut ran = 0;
        for mut case in cases(Id::Memset, 0xfa11, 64) {
            case.x[2] = 40;
            let (a, b) = (off.run(&case)?, on.run(&case)?);
            assert_eq!(agree(&function, &case, &a, &b), Ok(()), "{:x?}", case.x);
            if b.abort.is_none() && b.stop == Stop::Breakpoint(function.end()) {
                // The mov, the b, the composite and the ret.
                ran += usize::from(b.retired == 4);
            }
        }
        let statistics = on.hle();
        assert_eq!(statistics.installed, u64::from(natively), "{statistics:?}");
        assert_eq!(ran > 8, natively, "{ran} ran natively: {statistics:?}");
    }
    Ok(())
}

#[test]
fn a_rewritten_function_runs_its_new_body_after_ic_ivau() -> Result<()> {
    let memset = stand_in(Id::Memset);
    // memset's word 3 is `strb w1,[x3],#1`; the driver stores `strb
    // wzr,[x3],#1` over it between two calls.
    let rewritten = strb_post(ZR, 3, 1);
    let driver_words = 19;
    let target = 4 * (driver_words + 3) as u32;
    let driver = vec![
        mov(20, 0),                   // 0: mov x20,x0
        mov(21, 1),                   // 1: mov x21,x1
        mov(22, 2),                   // 2: mov x22,x2
        bl(driver_words as i32 - 3),  // 3: bl memset
        mov(0, 20),                   // 4: mov x0,x20
        mov(1, 21),                   // 5: mov x1,x21
        mov(2, 22),                   // 6: mov x2,x22
        adr(10, target - 4 * 7),      // 7: adr x10, memset's word 3
        movz(9, rewritten & 0xffff),  // 8: mov x9,#low
        movk_w16(9, rewritten >> 16), // 9: movk w9,#high,lsl #16
        str_w(9, 10),                 // 10: str w9,[x10]
        dc_cvau(10),                  // 11: dc cvau,x10
        DSB_ISH,                      // 12: dsb ish
        ic_ivau(10),                  // 13: ic ivau,x10
        DSB_ISH,                      // 14: dsb ish
        ISB,                          // 15: isb
        bl(driver_words as i32 - 16), // 16: bl memset
        0,                            // 17: b end
        NOP,                          // 18
    ];
    assert_eq!(driver.len(), driver_words);
    let mut driver = driver;
    driver[17] = to_end(17, driver_words, &memset);
    let function = memset.driven_by(&driver);
    let mut ran = 0;
    for case in cases(Id::Memset, 0x1c1, 24) {
        let (dst, n) = (case.x[0] as usize, case.x[2] as usize);
        let fits = dst + n <= BYTES && n > 0 && n <= CASE_MAX_BYTES as usize;
        if !fits || case.unreadable.is_some() || case.read_only.is_some() {
            continue;
        }
        // A machine per call: the first rewrites the code page.
        let mut outcomes = Vec::new();
        let mut statistics = None;
        for hle in [None, Some(native(&function, Id::Memset))] {
            let mut session = Session::new(&function, hle)?;
            let sctlr = session.machine().cpu.registers.get(Register::SCTLR_EL1, 0);
            session
                .machine()
                .cpu
                .registers
                .set_raw(Register::SCTLR_EL1, 0, sctlr | 1 << 26);
            outcomes.push(session.run(&case)?);
            statistics = Some(session.hle());
        }
        let (a, b) = (&outcomes[0], &outcomes[1]);
        kept(a, b).map_err(|why| anyhow::anyhow!("{why}: {:x?}", case.x))?;
        assert_eq!(b.stop, Stop::Breakpoint(function.end()));
        // The second call stored zeros.
        assert!(b.memory[dst..dst + n].iter().all(|&byte| byte == 0));
        let statistics = statistics.expect("an HLE run");
        // The rewritten function fails its digest at its second compile.
        assert_eq!(
            (statistics.matched, statistics.installed),
            (1, 1),
            "{statistics:?}"
        );
        ran += 1;
    }
    assert!(ran > 4, "{ran}");
    Ok(())
}

#[test]
fn an_exclusive_pair_across_a_copy_ends_as_without_hle() -> Result<()> {
    let memmove = stand_in(Id::Memmove);
    let driver = vec![
        mov(10, 0),      // 0: mov x10,x0
        ldxr(9, 10),     // 1: ldxr x9,[x10]
        bl(4),           // 2: bl memmove (word 6)
        stxr(11, 9, 10), // 3: stxr w11,x9,[x10]
        0,               // 4: b end
        NOP,             // 5
    ];
    let mut driver = driver;
    driver[4] = to_end(4, 6, &memmove);
    let function = memmove.driven_by(&driver);
    let mut off = Session::new(&function, None)?;
    let mut on = Session::new(&function, Some(native(&function, Id::Memmove)))?;
    let mut checked = 0;
    for mut case in cases(Id::Memmove, 0xe7c1, 96) {
        // An exclusive needs an aligned address.
        case.x[0] &= !7;
        if case.x[0] + case.x[2] > BYTES as u64 || case.x[2] == 0 {
            continue;
        }
        let (a, b) = (off.run(&case)?, on.run(&case)?);
        kept(&a, &b).map_err(|why| anyhow::anyhow!("{why}: {:x?}", case.x))?;
        assert_eq!(a.state.x[11], b.state.x[11], "{:x?}", case.x);
        checked += 1;
    }
    assert!(checked > 16, "{checked}");
    Ok(())
}

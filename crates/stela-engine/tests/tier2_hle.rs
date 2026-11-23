//! Tier 2 (`--tier2`) beside HLE (`--hle`): a composite block is never
//! promoted or replaced, and no superblock runs into a function entry, so a
//! call branched to the entry still runs the composite. Every block entered
//! is made hot at once (threshold 1); each call ends as it does with HLE
//! alone, instruction for instruction, and agrees with HLE off.
#![cfg(all(
    feature = "test-support",
    feature = "std",
    target_arch = "x86_64",
    target_os = "linux"
))]
use anyhow::Result;
use stela_engine::{
    hle::{Config, Id},
    machine::{
        Stop,
        engine::tier2::{Tier2Counts, Tier2Options},
    },
};
#[path = "support/hle_harness.rs"]
mod hle_harness;
use hle_harness::{a64::*, *};

/// `mov xD, #immediate` (`movz`).
fn movz(rd: u32, immediate: u32) -> u32 {
    0xd280_0000 | immediate << 5 | rd
}

/// `function`'s routine under `id`, enabled.
fn native(function: &StandIn, id: Id) -> Config {
    Config::with_table(table(function, id), &[id])
}

/// Sessions of `function`: HLE off, HLE alone, and HLE with tier 2
/// promoting every block it enters.
fn sessions(function: &StandIn, id: Id) -> Result<[Session; 3]> {
    let off = Session::new(function, None)?;
    let hle = Session::new(function, Some(native(function, id)))?;
    let mut tier2 = Session::new(function, Some(native(function, id)))?;
    tier2.machine().set_tier2(Some(Tier2Options {
        check: false,
        threshold: 1,
        check_every: 1,
    }));
    Ok([off, hle, tier2])
}

/// Run memset cases of 40 bytes on `function` in each of [`sessions`]:
/// the calls that ran the composite (returned within a few instructions,
/// where the original's loop retires three per byte), and the third
/// session's tier-2 counts.
fn memset_calls(function: &StandIn) -> Result<(usize, Tier2Counts)> {
    let [mut off, mut hle, mut tier2] = sessions(function, Id::Memset)?;
    let mut natively = 0;
    for mut case in cases(Id::Memset, 0xfa11, 64) {
        case.x[2] = 40;
        let (a, b, c) = (off.run(&case)?, hle.run(&case)?, tier2.run(&case)?);
        assert_eq!(agree(function, &case, &a, &c), Ok(()), "{:x?}", case.x);
        // Tier 2 changes nothing HLE does, to the instruction retired.
        assert_eq!(b, c, "{:x?}", case.x);
        let returned = c.abort.is_none() && c.stop == Stop::Breakpoint(function.end());
        natively += usize::from(returned && c.retired < 8);
    }
    assert_eq!(tier2.hle().installed, 1, "{:?}", tier2.hle());
    assert_eq!(hle.hle().installed, 1, "{:?}", hle.hle());
    Ok((natively, tier2.machine().statistics.tier2))
}

/// Calls entering the composite directly, every block hot at once: the
/// composite is never promoted and runs each call. (A call its fast path
/// bails from runs the original's words, which tier 2 may promote.)
#[test]
fn tier2_never_promotes_a_composite() -> Result<()> {
    let memset = stand_in(Id::Memset);
    let (natively, counts) = memset_calls(&memset)?;
    assert!(natively > 8, "{natively} ran the composite: {counts:?}");
    Ok(())
}

/// `mov x2,#40; b memset; nop` ahead of memset: tier 2 promotes the driver's
/// block, whose superblock would run on through the B into memset's words
/// and their loop, past the composite. It ends before the entry instead:
/// the driver's block stays as tier 1 compiled it, and the B reaches the
/// composite in every call.
#[test]
fn a_superblock_ends_before_an_hle_entry() -> Result<()> {
    let branched = stand_in(Id::Memset).driven_by(&[movz(2, 40), b(2), NOP]);
    let (natively, counts) = memset_calls(&branched)?;
    assert!(natively > 8, "{natively} ran the composite: {counts:?}");
    assert!(counts.promoted >= 1 && counts.unchanged >= 1, "{counts:?}");
    Ok(())
}

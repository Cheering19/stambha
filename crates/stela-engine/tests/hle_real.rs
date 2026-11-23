//! HLE's natives against the real macOS platform functions (opt-in): the
//! functions `cargo xtask hle-signatures --extract` wrote, each a `.bin` of
//! its bytes and a `.json` of its routine, address, length and exit, in the
//! private directory `STELA_HLE_FUNCTIONS` names. Apple's bytes are read
//! here at test time only; none are in the repository.
//!
//! - `real_platform_functions_match_their_natives`: each eligible function,
//!   placed at its own page offset, stands in for the stand-in and runs the
//!   differential cases (`STELA_HLE_CASES`, default 10000) against its
//!   routine's native on every vector set the host has. A failure for
//!   memmove, bzero or memset is a bug in the native; for memcmp or strlen
//!   it may be the real function's result form, which then keeps that
//!   routine out of the committed table.
//! - `real_platform_functions_time_by_size`: host ns per call, translated
//!   and native, by size, through a `bl` loop. A report; no assert.
//!
//! Run: `STELA_HLE_FUNCTIONS=<dir> cargo test --release -p stela-engine
//! --features test-support --test hle_real -- --ignored --nocapture`.
#![cfg(all(
    feature = "test-support",
    feature = "std",
    target_arch = "x86_64",
    target_os = "linux"
))]
use anyhow::{Context, Result, ensure};
use std::path::{Path, PathBuf};
use stela_engine::{
    hle::{Config, Id, Signature},
    machine::system::Register,
};
#[path = "support/hle_harness.rs"]
mod hle_harness;
use hle_harness::{a64::*, *};

/// One extracted function.
struct Real {
    name: String,
    id: Id,
    address: u64,
    exit: u16,
    bytes: Vec<u8>,
    eligible: bool,
}

/// The value of `"key": value` in a flat JSON object, unquoted.
fn field<'a>(json: &'a str, key: &str) -> Option<&'a str> {
    let at = json.find(&format!("\"{key}\""))?;
    let rest = json[at + key.len() + 2..]
        .trim_start()
        .strip_prefix(':')?
        .trim_start();
    let end = match rest.strip_prefix('"') {
        Some(quoted) => return quoted.split('"').next(),
        None => rest.find([',', '}', '\n']).unwrap_or(rest.len()),
    };
    Some(rest[..end].trim())
}

/// The functions in `STELA_HLE_FUNCTIONS`.
fn functions() -> Result<Vec<Real>> {
    let directory = PathBuf::from(
        std::env::var("STELA_HLE_FUNCTIONS")
            .context("STELA_HLE_FUNCTIONS names the directory hle-signatures --extract wrote")?,
    );
    let mut functions = Vec::new();
    let mut entries: Vec<PathBuf> = std::fs::read_dir(&directory)?
        .map(|entry| entry.map(|entry| entry.path()))
        .collect::<std::io::Result<_>>()?;
    entries.sort();
    for json_path in entries
        .iter()
        .filter(|path| path.extension().is_some_and(|e| e == "json"))
    {
        functions.push(read(json_path)?);
    }
    ensure!(
        !functions.is_empty(),
        "{} holds no functions",
        directory.display()
    );
    Ok(functions)
}

fn read(json_path: &Path) -> Result<Real> {
    let json = std::fs::read_to_string(json_path)?;
    let named =
        |key| field(&json, key).with_context(|| format!("{}: no {key}", json_path.display()));
    let id = Id::from_name(named("id")?).context("an unknown routine")?;
    let exit = named("exit")?;
    let bytes = std::fs::read(json_path.with_extension("bin"))?;
    Ok(Real {
        name: json_path
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .into_owned(),
        id,
        address: named("address")?.parse()?,
        exit: if exit == "null" { 0 } else { exit.parse()? },
        bytes,
        eligible: named("verdict")? == "eligible",
    })
}

/// `real` as a stand-in, at its own offset in its page.
fn standing(real: &Real) -> StandIn {
    let (words, _) = real.bytes.as_chunks::<4>();
    StandIn {
        id: real.id,
        words: words.iter().map(|word| u32::from_le_bytes(*word)).collect(),
        entry: 0,
        exit: real.exit,
        returns: real.id != Id::Bzero,
        caller: None,
        driven: false,
    }
    .at(((real.address & 0xfff) / 4) as usize)
}

/// SCTLR_EL1.DZE and .UCI on every machine: a real bzero may run DC ZVA.
fn system_registers(differential: &mut Differential) {
    for machine in differential.machines() {
        let sctlr = machine.cpu.registers.get(Register::SCTLR_EL1, 0);
        machine
            .cpu
            .registers
            .set_raw(Register::SCTLR_EL1, 0, sctlr | 1 << 14 | 1 << 26);
    }
}

#[test]
#[ignore = "oracle: STELA_HLE_FUNCTIONS"]
fn real_platform_functions_match_their_natives() -> Result<()> {
    let mut failures = Vec::new();
    for real in functions()? {
        if !real.eligible {
            println!("{}: not eligible, skipped", real.name);
            continue;
        }
        let stand_in = standing(&real);
        Signature::of(real.id, &stand_in.function(), real.exit)
            .map_err(|why| anyhow::anyhow!("{}: {why:?}", real.name))?;
        let mut differential = Differential::of(stand_in, real.id, CASE_MAX_BYTES)?;
        system_registers(&mut differential);
        let (mut checked, mut native, mut failed) = (0, 0, 0);
        for case in case_stream(real.id, 0x2ea1 ^ real.address, case_count(10_000)) {
            match differential.check(&case) {
                Ok(outcome) => native += usize::from(outcome.native),
                Err(why) => {
                    if failed < 5 {
                        println!("{}: {why:#}", real.name);
                    }
                    failed += 1;
                }
            }
            checked += 1;
        }
        let statistics = differential.statistics();
        println!(
            "{} ({}): {checked} cases, {native} native, {failed} disagreed; {statistics:?}",
            real.name,
            real.id.name()
        );
        if failed != 0 {
            failures.push(format!("{}: {failed} of {checked}", real.name));
        }
    }
    ensure!(failures.is_empty(), "disagreed: {failures:?}");
    Ok(())
}

/// `bl` by `words`.
fn bl(words: i32) -> u32 {
    0x9400_0000 | (words as u32 & 0x03ff_ffff)
}

/// `function` called `calls` times from a loop ahead of it in its page,
/// which ends at its end.
fn driven(function: &StandIn, calls: u32) -> Result<StandIn> {
    let offset = function.entry;
    let driver = [
        mov(20, 0),
        mov(21, 1),
        mov(22, 2),
        0xd280_0000 | calls << 5 | 19, // mov x19,#calls
        mov(0, 20),                    // loop: mov x0,x20
        mov(1, 21),
        mov(2, 22),
        bl((offset - 7) as i32), // bl the function
        subs_imm(19, 19, 1),
        b_cond(NE, -5),
        b((function.words.len() - 10) as i32), // b end
    ];
    ensure!(driver.len() < offset, "no room for the driver");
    let mut words = function.words.clone();
    words[..driver.len()].copy_from_slice(&driver);
    Ok(StandIn {
        words,
        driven: true,
        ..function.clone()
    })
}

#[test]
#[ignore = "manual: STELA_HLE_FUNCTIONS"]
fn real_platform_functions_time_by_size() -> Result<()> {
    for real in functions()?.into_iter().filter(|real| real.eligible) {
        let function = standing(&real);
        print!("{} ({}) ns/call", real.name, real.id.name());
        for n in [8u64, 16, 32, 64, 100, 127, 128, 129, 200, 256, 4096] {
            // Within a run's block budget translated: a block a byte.
            let calls = (150_000 / (n as u32 + 8)).clamp(16, 50_000);
            let driven = driven(&function, calls)?;
            let x = match real.id {
                Id::Bzero => [64, n, 0],
                Id::Memset => [64, 0xab, n],
                _ => [64, 2 * PAGE as u64 + 64, n],
            };
            let mut case = case_of(x, n, None, None);
            let at = 64 + n as usize;
            if real.id == Id::Strlen {
                case.contents[64..at].iter_mut().for_each(|byte| *byte |= 1);
                case.contents[at] = 0;
            }
            if real.id == Id::Memcmp {
                case.contents.copy_within(64..at, 2 * PAGE + 64);
            }
            let mut times = Vec::new();
            for hle in [
                None,
                Some(Config::with_table(table(&driven, real.id), &[real.id])),
            ] {
                let mut session = Session::new(&driven, hle)?;
                let machine = session.machine();
                let sctlr = machine.cpu.registers.get(Register::SCTLR_EL1, 0);
                machine
                    .cpu
                    .registers
                    .set_raw(Register::SCTLR_EL1, 0, sctlr | 1 << 14);
                // The first run compiles; the fastest of five runs after.
                session.run(&case)?;
                let mut fastest = f64::MAX;
                for _ in 0..5 {
                    let start = std::time::Instant::now();
                    let outcome = session.run(&case)?;
                    ensure!(outcome.abort.is_none(), "{}: n {n} aborted", real.name);
                    fastest = fastest.min(start.elapsed().as_nanos() as f64 / f64::from(calls));
                }
                times.push(fastest);
            }
            print!(
                " n={n}: {:.1} -> {:.1} ({:.2}x)",
                times[0],
                times[1],
                times[0] / times[1]
            );
        }
        println!();
    }
    Ok(())
}

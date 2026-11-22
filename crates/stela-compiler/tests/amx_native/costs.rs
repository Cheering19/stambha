//! What each AMX form costs on this host, through the model and through
//! each native path that has F16C: the forms native code covers and those
//! the plan's later tasks target, and the dispatch alone, per call of
//! `Native::execute` and of the routine blocks call
//! (`x86_64::amx_routine::routine`):
//! `cargo test -p stela-compiler --release --test amx_native costs -- --ignored --nocapture`.
use crate::harness::{self, NoMemory, Paths, Random};
use std::time::Instant;
use stela_compiler::{
    amx::{AmxOperation, AmxRegisters, Generation, model, native::Tier},
    x86_64::{amx_routine, amx_transfer::Cell},
};

const VECTOR: u64 = 1 << 63;
const WIDEN_16_32: u64 = 1 << 62;
const Y_HALF: u64 = 1 << 60;
/// vecfp and matfp type (bits 45:42) and mode (bits 52:47).
const fn float(kind: u64, mode: u64) -> u64 {
    kind << 42 | mode << 47
}

/// Each form: its name, the plan task that targets it (0 where native code
/// covers it already), the operation and an operand of it.
const CASES: [(&str, u8, AmxOperation, u64); 28] = [
    (
        "fma32 matrix f16-Y",
        0,
        AmxOperation::Fma32,
        Y_HALF | 1 << 20,
    ),
    ("fma32 vector", 0, AmxOperation::Fma32, VECTOR | 5 << 20),
    ("fma64 matrix", 0, AmxOperation::Fma64, 0),
    (
        "vecfp f16 broadcast",
        0,
        AmxOperation::VectorFloat,
        float(2, 0) | 1 << 38 | 3 << 32 | 8 << 20,
    ),
    (
        "fma16 widened clear (mode 7)",
        0,
        AmxOperation::Fma16,
        0x4000_0000_3800_0000,
    ),
    (
        "fma32 constant (mode 7)",
        0,
        AmxOperation::Fma32,
        7 << 27 | 3 << 20,
    ),
    (
        "dispatch only (fma32 mode 6)",
        20,
        AmxOperation::Fma32,
        6 << 27,
    ),
    ("fma32 matrix z+x (mode 2)", 5, AmxOperation::Fma32, 2 << 27),
    (
        "fma64 vector z+y (mode 4)",
        5,
        AmxOperation::Fma64,
        VECTOR | 4 << 27,
    ),
    (
        "fma16 vector product (mode 1)",
        6,
        AmxOperation::Fma16,
        VECTOR | 1 << 27,
    ),
    ("fma16 matrix z+x (mode 2)", 6, AmxOperation::Fma16, 2 << 27),
    ("fma16 vector", 7, AmxOperation::Fma16, VECTOR),
    ("fma16 matrix", 7, AmxOperation::Fma16, 0),
    ("fma16 widened matrix", 7, AmxOperation::Fma16, WIDEN_16_32),
    ("mac16 vector", 8, AmxOperation::Mac16, VECTOR),
    ("mac16 matrix", 9, AmxOperation::Mac16, 0),
    ("mac16 widened matrix", 10, AmxOperation::Mac16, WIDEN_16_32),
    ("matfp f32", 11, AmxOperation::MatrixFloat, float(4, 0)),
    ("matfp f64", 11, AmxOperation::MatrixFloat, float(7, 0)),
    (
        "matfp f32 select (mode 4)",
        12,
        AmxOperation::MatrixFloat,
        float(4, 4),
    ),
    ("matfp f16", 12, AmxOperation::MatrixFloat, float(2, 0)),
    ("matfp f16-f32", 13, AmxOperation::MatrixFloat, float(3, 0)),
    ("vecfp f32", 14, AmxOperation::VectorFloat, float(4, 0)),
    (
        "vecfp f64 multiply (mode 10)",
        14,
        AmxOperation::VectorFloat,
        float(7, 10),
    ),
    (
        "vecfp f16 multiply (mode 10)",
        15,
        AmxOperation::VectorFloat,
        float(2, 10),
    ),
    (
        "vecfp f16 min (mode 5)",
        15,
        AmxOperation::VectorFloat,
        float(2, 5),
    ),
    ("vecfp bf16", 19, AmxOperation::VectorFloat, float(0, 0)),
    ("matfp bf16", 19, AmxOperation::MatrixFloat, float(0, 0)),
];

const ROUNDS: u32 = 20_000;
const GENERATION: Generation = Generation::M2;

/// Nanoseconds per call of `step` over [`ROUNDS`] calls.
fn per_call(mut step: impl FnMut()) -> u128 {
    let start = Instant::now();
    for _ in 0..ROUNDS {
        step();
    }
    start.elapsed().as_nanos() / u128::from(ROUNDS)
}

/// One case on one native path: ns per `Native::execute` and per routine
/// call, and whether the path completed it (else the cost is its refusal's
/// and the routine's includes the model).
fn native_costs(
    paths: &Paths,
    initial: &AmxRegisters,
    operation: AmxOperation,
    operand: u64,
) -> (u128, u128, bool) {
    let mut registers = initial.clone();
    let covered = paths
        .native
        .execute(GENERATION, &mut registers, operation, operand);
    let execute = per_call(|| {
        paths
            .native
            .execute(GENERATION, &mut registers, operation, operand);
    });
    let mut registers = initial.clone();
    let mut cell = Cell {
        registers: &raw mut registers as u64,
        generation: Some(GENERATION),
        native: paths.native,
        ..Cell::default()
    };
    let number = operation.encoding().0;
    let routine = per_call(|| {
        // SAFETY: the cell names a live register file nothing else touches
        // during the call, its generation and paths.
        let done = unsafe { amx_routine::routine(&raw mut cell, number, operand) };
        assert_eq!(done, 1, "the routine completes register operations");
    });
    (execute, routine, covered)
}

/// The cost table, printed as rows of the plan's results log.
#[test]
#[ignore = "manual: opt-in, in a release build"]
fn costs() {
    let initial = harness::fill(&mut Random(0x1234_5678_9abc_def1));
    let paths: Vec<Paths> = harness::tiers()
        .into_iter()
        .filter(|paths| paths.tier() != Tier::Model && paths.f16c)
        .collect();
    let mut header = String::from("| form | task | model ns |");
    for paths in &paths {
        header.push_str(&format!(" {0} ns | {0} routine ns |", paths.tier().name()));
    }
    eprintln!("{header}");
    for (name, task, operation, operand) in CASES {
        let mut registers = initial.clone();
        let model = per_call(|| {
            model::execute_for(
                GENERATION,
                &mut registers,
                operation,
                operand,
                &mut NoMemory,
            )
            .unwrap();
        });
        let task = if task == 0 {
            String::from("native")
        } else {
            format!("{task}")
        };
        let mut row = format!("| {name} | {task} | {model} |");
        for paths in &paths {
            let (execute, routine, covered) = native_costs(paths, &initial, operation, operand);
            if covered {
                row.push_str(&format!(" {execute} | {routine} |"));
            } else {
                row.push_str(&format!(" refused ({execute}) | {routine} (model) |"));
            }
        }
        eprintln!("{row}");
    }
}

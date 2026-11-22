//! `fma32`, `fms32`, `fma64` and `fms64` in every tier.
use crate::harness::{self, Case, Form, GENERATIONS, Paths, check, differential, native};
use stela_compiler::amx::{AmxOperation, AmxRegisters, Generation, native::Tier};

const VECTOR: u64 = 1 << 63;
const X_HALF: u64 = 1 << 61;
const Y_HALF: u64 = 1 << 60;

/// Where the fused paths leave an operand to the model: the model tier,
/// which completes only copies and constants; copies of f16-widened halves,
/// whose NaN payloads fms32 negates; and fma32's widening on the VEX tier
/// without F16C.
fn may_refuse(paths: &Paths, operation: AmxOperation, _: Generation, operand: u64) -> bool {
    let mode = (operand >> 27) & 7;
    let single = matches!(operation, AmxOperation::Fma32 | AmxOperation::Fms32);
    let widened = single && operand & (X_HALF | Y_HALF) != 0;
    let copies_half =
        single && (mode == 3 && operand & X_HALF != 0 || mode == 5 && operand & Y_HALF != 0);
    paths.tier() == Tier::Model
        || copies_half
        || widened && paths.tier() == Tier::Fma && !paths.f16c
}

/// The unit test's seed, operands and floor (more than 5000 of 16000 on
/// M2), with its 16000 operands spread over the four generations, and a
/// floor for each shape and mode class: about half of what each ran.
const FUSED: Case = Case {
    name: "fused",
    seed: 0x9e37_79b9_7f4a_7c15,
    operations: &[
        AmxOperation::Fma32,
        AmxOperation::Fms32,
        AmxOperation::Fma64,
        AmxOperation::Fms64,
    ],
    generations: &GENERATIONS,
    iterations: 1000,
    fill: harness::fill,
    operand: harness::operand,
    runs_on: |_| true,
    may_refuse,
    forms: &[
        Form {
            name: "any operand",
            of: |_, _| true,
            floor: |paths| with_fma(paths, 1251),
        },
        Form {
            name: "matrix z ± x*y or x*y",
            of: |_, operand| operand & VECTOR == 0 && arithmetic(operand),
            floor: |paths| with_fma(paths, 400),
        },
        Form {
            name: "vector z ± x*y or x*y",
            of: |_, operand| operand & VECTOR != 0 && arithmetic(operand),
            floor: |paths| with_fma(paths, 400),
        },
        Form {
            name: "f16-widened fma32/fms32",
            of: |operation, operand| {
                matches!(operation, AmxOperation::Fma32 | AmxOperation::Fms32)
                    && operand & (X_HALF | Y_HALF) != 0
                    && arithmetic(operand)
            },
            floor: |paths| {
                if paths.tier() == Tier::Fma && !paths.f16c {
                    0
                } else {
                    with_fma(paths, 450)
                }
            },
        },
        Form {
            name: "copies and constants",
            of: |_, operand| matches!((operand >> 27) & 7, 3 | 5 | 6 | 7),
            floor: |_| 400,
        },
        Form {
            name: "z ± x (mode 2)",
            of: |_, operand| (operand >> 27) & 7 == 2,
            floor: |paths| with_fma(paths, 70),
        },
        Form {
            name: "z ± y (mode 4)",
            of: |_, operand| (operand >> 27) & 7 == 4,
            floor: |paths| with_fma(paths, 70),
        },
    ],
};

/// z ± x*y or x*y: bits 29:27 of 0 or 1.
fn arithmetic(operand: u64) -> bool {
    (operand >> 27) & 7 <= 1
}

/// `floor` on the tiers that multiply-add, none on the model's.
fn with_fma(paths: &Paths, floor: usize) -> usize {
    if paths.tier() == Tier::Model {
        0
    } else {
        floor
    }
}

#[test]
fn every_native_tier_matches_the_model_bit_for_bit() {
    differential(&FUSED, &native);
}

/// Special f32 values: both zeros, ±1, the denormal and normal limits, the
/// largest finite values, infinities, quiet, signalling and negative NaNs,
/// and 1's neighbours with the steps 2^-24 and 2^-23 that tie or round
/// against them, and 2^24.
const SINGLES: [u32; 20] = [
    0,
    0x8000_0000,
    0x3f80_0000,
    0xbf80_0000,
    0x0000_0001,
    0x8000_0001,
    0x007f_ffff,
    0x0080_0000,
    0x7f7f_ffff,
    0xff7f_ffff,
    0x7f80_0000,
    0xff80_0000,
    0x7fc0_0000,
    0x7f80_0001,
    0xffc1_2345,
    0x3f80_0001,
    0x3380_0000,
    0x3400_0000,
    0x4b80_0000,
    0x3f7f_ffff,
];

/// The f64 counterparts of [`SINGLES`].
const DOUBLES: [u64; 18] = [
    0,
    1 << 63,
    0x3ff0_0000_0000_0000,
    0xbff0_0000_0000_0000,
    1,
    0x8000_0000_0000_0001,
    0x000f_ffff_ffff_ffff,
    0x0010_0000_0000_0000,
    0x7fef_ffff_ffff_ffff,
    0xffef_ffff_ffff_ffff,
    0x7ff0_0000_0000_0000,
    0xfff0_0000_0000_0000,
    0x7ff8_0000_0000_0000,
    0x7ff0_0000_0000_0001,
    0xfff8_0000_0000_1234,
    0x3ff0_0000_0000_0001,
    0x3ca0_0000_0000_0000,
    0x4340_0000_0000_0000,
];

/// z ± x and z ± y (bits 29:27 of 2 and 4) over every pair of special
/// values, as the vector's lane pairs and the matrix's outer pairs, which
/// every multiply-add tier completes natively.
#[test]
fn add_only_modes_round_once_over_every_special_pair() {
    let singles: Vec<u64> = SINGLES.iter().map(|&value| u64::from(value)).collect();
    for paths in harness::tiers()
        .iter()
        .filter(|paths| paths.tier() != Tier::Model)
    {
        for (operations, values, bytes) in [
            ([AmxOperation::Fma32, AmxOperation::Fms32], &singles[..], 4),
            ([AmxOperation::Fma64, AmxOperation::Fms64], &DOUBLES[..], 8),
        ] {
            let pairs: Vec<(u64, u64)> = values
                .iter()
                .flat_map(|&z| values.iter().map(move |&other| (z, other)))
                .collect();
            for chunk in pairs.chunks(64 / bytes) {
                let mut registers = AmxRegisters::zeroed();
                for (lane, &(z, other)) in chunk.iter().enumerate() {
                    let at = lane * bytes..lane * bytes + bytes;
                    for row in registers.z.iter_mut() {
                        row[at.clone()].copy_from_slice(&z.to_le_bytes()[..bytes]);
                    }
                    registers.x[0][at.clone()].copy_from_slice(&other.to_le_bytes()[..bytes]);
                    registers.y[0][at].copy_from_slice(&other.to_le_bytes()[..bytes]);
                }
                for operation in operations {
                    for operand in [
                        VECTOR | 2 << 27,
                        VECTOR | 4 << 27,
                        2 << 27,
                        4 << 27 | 1 << 20,
                    ] {
                        assert!(
                            check(
                                &native,
                                paths,
                                Generation::M2,
                                &registers,
                                operation,
                                operand
                            ),
                            "{paths:?} {operation:?} {operand:#x} left to the model"
                        );
                    }
                }
            }
        }
    }
}

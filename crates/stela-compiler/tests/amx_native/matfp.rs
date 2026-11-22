//! `matfp` on IEEE types: f32, f64, f16 and f16 into f32.
use crate::{
    fused::{DOUBLES, SINGLES},
    half::HALVES,
    harness::{self, Case, Form, GENERATIONS, Paths, Random, check, differential, native},
};
use stela_compiler::amx::{AmxOperation, AmxRegisters, Generation, native::Tier};

/// No bits 56:54: the forms covered.
fn plain(operand: u64) -> bool {
    (operand >> 54) & 7 == 0
}

const INDEXED: u64 = 1 << 53;
/// Under an indexed load: the indices are Y's.
const INDEXED_Y: u64 = 1 << 47;

/// The ALU mode: bits 52:47, or 0 under an indexed load (bit 53).
fn mode(operand: u64) -> u64 {
    if operand & INDEXED != 0 {
        0
    } else {
        (operand >> 47) & 0x3f
    }
}

fn shuffled(operand: u64) -> bool {
    (operand >> 27) & 0xf != 0
}

fn kind(operand: u64) -> u64 {
    (operand >> 42) & 0xf
}

/// The lane-enable value 3 (zeros) in X's or Y's field (bits 37 and 63
/// ignored, as the model does).
fn zeros(operand: u64) -> bool {
    let operand = operand & !(1 << 37 | 1 << 63);
    (operand >> 38) & 7 == 0 && (operand >> 32) & 0x3f == 3
        || (operand >> 23) & 7 == 0 && (operand >> 58) & 0x3f == 3
}

/// Operands over every field `matfp` reads, mostly in the covered forms:
/// modes 0, 1 and 4, the IEEE types, a quarter with an indexed load and
/// half shuffled, lane fields often all on or one of the values 3 (zeros),
/// 4 and 5 (a zero operand).
fn operand(random: &mut Random) -> u64 {
    let mut operand = random.next();
    let choice = random.next();
    if choice & 7 != 0 {
        operand &= !(7 << 54);
    }
    if (choice >> 32) & 3 != 0 {
        operand &= !INDEXED;
    }
    if (choice >> 34) & 1 != 0 {
        operand &= !(0xf << 27);
    }
    if choice & 0x38 != 0 {
        operand &= !(0x3f << 47);
        operand |= [0u64, 1, 4][(choice >> 6) as usize % 3] << 47;
    }
    if choice & 0x300 != 0 {
        operand &= !(0xf << 42);
        operand |= [4u64, 7, 3, 2, 5, 9, 0, 1][(choice >> 10) as usize & 7] << 42;
    }
    let special = [3u64, 4, 5][(choice >> 20) as usize % 3];
    match (choice >> 13) % 4 {
        0 => operand &= !(0x1ff << 32),
        1 => operand = operand & !(0x1ff << 32) | special << 32,
        _ => {}
    }
    match (choice >> 16) % 4 {
        0 => operand &= !(7 << 23 | 0x3f << 58),
        1 => operand = operand & !(7 << 23 | 0x3f << 58) | special << 58,
        _ => {}
    }
    operand
}

/// Where `matfp` is left to the model: the model tier; bits 56:54; a mode
/// other than 0, 1 and 4; bf16 (types 0/1 from the M2); and without F16C
/// the f16 arithmetic and anything of type 3 but zeros
/// (`native::x86::matrix_float`).
fn may_refuse(paths: &Paths, _: AmxOperation, generation: Generation, operand: u64) -> bool {
    let operand = operand & !(1 << 37 | 1 << 63);
    let m2 = generation != Generation::M1;
    let bf16 = m2 && kind(operand) <= 1;
    let widened = kind(operand) == 3;
    let half = !matches!(kind(operand), 3 | 4 | 7) && !bf16;
    paths.tier() == Tier::Model
        || !plain(operand)
        || !matches!(mode(operand), 0 | 1 | 4)
        || bf16
        || !zeros(operand) && !paths.f16c && (widened || half && mode(operand) != 4)
}

/// `floor` on the multiply-add tiers, and with F16C where `f16c`.
fn floor(paths: &Paths, floor: usize, f16c: bool) -> usize {
    if paths.tier() == Tier::Model || f16c && !paths.f16c {
        0
    } else {
        floor
    }
}

/// Covered, with neither zeros nor a mode outside 0/1 (arithmetic) or 4.
fn covered(operand: u64, kinds: &[u64], modes: &[u64]) -> bool {
    plain(operand)
        && !zeros(operand)
        && kinds.contains(&kind(operand))
        && modes.contains(&mode(operand))
}

const MATFP: Case = Case {
    name: "matfp",
    seed: 0xa54f_f53a_5f1d_36f1,
    operations: &[AmxOperation::MatrixFloat],
    generations: &GENERATIONS,
    iterations: 6000,
    fill: harness::fill,
    operand,
    runs_on: |_| true,
    may_refuse,
    forms: &[
        Form {
            name: "f32 z ± x*y",
            of: |_, operand| covered(operand, &[4], &[0, 1]),
            floor: |paths| floor(paths, 60, false),
        },
        Form {
            name: "f64 z ± x*y",
            of: |_, operand| covered(operand, &[7], &[0, 1]),
            floor: |paths| floor(paths, 60, false),
        },
        Form {
            name: "f32/f64 select",
            of: |_, operand| covered(operand, &[4, 7], &[4]),
            floor: |paths| floor(paths, 60, false),
        },
        Form {
            name: "f16 z ± x*y",
            of: |_, operand| covered(operand, &[2, 5, 9], &[0, 1]),
            floor: |paths| floor(paths, 100, true),
        },
        Form {
            name: "f16 select",
            of: |_, operand| covered(operand, &[2, 5, 9], &[4]),
            floor: |paths| floor(paths, 50, false),
        },
        Form {
            name: "f16-f32 z ± x*y",
            of: |_, operand| covered(operand, &[3], &[0, 1]),
            floor: |paths| floor(paths, 60, true),
        },
        Form {
            name: "f16-f32 select",
            of: |_, operand| covered(operand, &[3], &[4]),
            floor: |paths| floor(paths, 30, true),
        },
        Form {
            name: "zeros",
            of: |_, operand| plain(operand) && zeros(operand),
            floor: |paths| floor(paths, 100, false),
        },
        Form {
            name: "shuffled f32/f64 z ± x*y",
            of: |_, operand| shuffled(operand) && covered(operand, &[4, 7], &[0, 1]),
            floor: |paths| floor(paths, 80, false),
        },
        Form {
            name: "indexed X f32",
            of: |_, operand| {
                operand & (INDEXED | INDEXED_Y) == INDEXED && covered(operand, &[4], &[0])
            },
            floor: |paths| floor(paths, 15, false),
        },
        Form {
            name: "indexed Y f16",
            of: |_, operand| {
                operand & (INDEXED | INDEXED_Y) == INDEXED | INDEXED_Y
                    && covered(operand, &[2, 5, 9], &[0])
            },
            floor: |paths| floor(paths, 15, true),
        },
        Form {
            name: "a zero operand",
            of: |_, operand| {
                let operand = operand & !(1 << 37 | 1 << 63);
                plain(operand)
                    && !zeros(operand)
                    && ((operand >> 38) & 7 == 0 && matches!((operand >> 32) & 0x3f, 4 | 5)
                        || (operand >> 23) & 7 == 0 && matches!((operand >> 58) & 0x3f, 4 | 5))
            },
            floor: |paths| floor(paths, 100, false),
        },
    ],
};

#[test]
fn matfp_matches_the_model_bit_for_bit() {
    differential(&MATFP, &native);
}

/// Special values in X, Y and Z lanes, rotated so that many triples
/// meet, in modes 0, 1 and 4, for f32, f64, f16 and f16 into f32, with
/// every lane on and the Z row offsets 0 and 3: each completes natively
/// (F16C for the f16 forms) and matches the model.
#[test]
fn matfp_special_values_match_the_model() {
    let singles: Vec<u64> = SINGLES.iter().map(|&value| u64::from(value)).collect();
    let halves: Vec<u64> = HALVES.iter().map(|&value| u64::from(value)).collect();
    let mut ran = 0;
    for paths in harness::tiers()
        .iter()
        .filter(|paths| paths.tier() != Tier::Model)
    {
        ran += 1;
        for (kind, values, bytes, f16c) in [
            (4u64, &singles[..], 4, false),
            (7, &DOUBLES[..], 8, false),
            (2, &halves[..], 2, true),
            (3, &halves[..], 2, true),
        ] {
            if f16c && !paths.f16c {
                continue;
            }
            let count = values.len();
            for rotation in 0..count {
                let mut registers = AmxRegisters::zeroed();
                let value = |index: usize| values[index % count].to_le_bytes();
                for lane in 0..64 / bytes {
                    let at = lane * bytes..lane * bytes + bytes;
                    registers.x[0][at.clone()].copy_from_slice(&value(lane + rotation)[..bytes]);
                    registers.y[0][at.clone()]
                        .copy_from_slice(&value(lane * 3 + rotation)[..bytes]);
                    for (row, z) in registers.z.iter_mut().enumerate() {
                        z[at.clone()].copy_from_slice(&value(lane * 7 + row + rotation)[..bytes]);
                    }
                }
                for mode in [0u64, 1, 4] {
                    for z_row in [0u64, 3] {
                        let operand = kind << 42 | mode << 47 | z_row << 20;
                        for generation in [Generation::M1, Generation::M2] {
                            assert!(
                                check(
                                    &native,
                                    paths,
                                    generation,
                                    &registers,
                                    AmxOperation::MatrixFloat,
                                    operand
                                ),
                                "{paths:?} {generation:?} matfp {operand:#x} left to the model"
                            );
                        }
                    }
                }
            }
        }
    }
    assert!(ran > 0, "no multiply-add path ran");
}

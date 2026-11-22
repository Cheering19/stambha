//! `vecfp` on IEEE types: f16, f32, f64 and f16 into f32, every mode the
//! model computes on one Z row.
use crate::{
    fused::{DOUBLES, SINGLES},
    half::HALVES,
    harness::{self, Case, Form, GENERATIONS, Paths, Random, check, differential, native},
};
use stela_compiler::amx::{AmxOperation, AmxRegisters, Generation, native::Tier};

fn mode(operand: u64) -> u64 {
    (operand >> 47) & 0x3f
}

fn kind(operand: u64) -> u64 {
    (operand >> 42) & 0xf
}

/// No bits 56:54, indexed load (53), shuffle (30:27) or multiple rows
/// (31, which M1 ignores): the one-row forms covered.
fn plain(operand: u64) -> bool {
    (operand >> 54) & 7 == 0 && operand & (1 << 53 | 1 << 31) == 0 && (operand >> 27) & 0xf == 0
}

/// The lane field (bits 40:38) and value (37:32, bit 37 ignored).
fn field(operand: u64) -> (u64, u64) {
    ((operand >> 38) & 7, (operand >> 32) & 0x1f)
}

/// The lane value 3 of field 0: zeros.
fn zeros(operand: u64) -> bool {
    field(operand) == (0, 3)
}

/// Operands over every field `vecfp` reads, mostly one-row forms of
/// `modes` on the IEEE types, lane fields often all on, a broadcast Y
/// lane, or the values 3 (zeros), 4 and 5 (a zero operand).
fn operand_of(random: &mut Random, modes: &[u64]) -> u64 {
    let mut operand = random.next();
    let choice = random.next();
    if choice & 7 != 0 {
        operand &= !(7 << 54 | 1 << 53 | 0xf << 27);
    }
    if choice & 0x8_0000 == 0 {
        operand &= !(1 << 31);
    }
    if choice & 0x38 != 0 {
        operand &= !(0x3f << 47);
        operand |= modes[(choice >> 6) as usize % modes.len()] << 47;
    }
    if choice & 0x600 != 0 {
        operand &= !(0xf << 42);
        operand |= [4u64, 7, 3, 2, 5, 9, 0, 1][(choice >> 11) as usize & 7] << 42;
    }
    match (choice >> 14) % 4 {
        0 => operand &= !(0x1ff << 32),
        1 => operand = operand & !(0x1ff << 32) | 1 << 38 | ((choice >> 20) % 32) << 32,
        2 => operand = operand & !(0x1ff << 32) | [3u64, 4, 5][(choice >> 26) as usize % 3] << 32,
        _ => {}
    }
    operand
}

/// Where `vecfp` is left to the model: bits 56:54, an indexed load, a
/// shuffle, the M2's multiple rows, a mode the model leaves as a no-op,
/// bf16 (types 0/1 from the M2), and arithmetic (modes 0, 1, 10-12 that
/// write) on f32/f64 lanes on the model tier or on f16 lanes without F16C
/// (`native::x86::vector_float`).
pub fn may_refuse(paths: &Paths, _: AmxOperation, generation: Generation, operand: u64) -> bool {
    let operand = operand & !(1 << 37);
    let m2 = generation != Generation::M1;
    let mode = mode(operand);
    let arithmetic = matches!(mode, 0 | 1 | 10..=12);
    let wide_float = matches!(kind(operand), 4 | 7);
    (operand >> 54) & 7 != 0
        || operand & 1 << 53 != 0
        || (operand >> 27) & 0xf != 0
        || m2 && operand & 1 << 31 != 0
        || !(matches!(mode, 0 | 1 | 4 | 5 | 7) || m2 && matches!(mode, 10..=12))
        || m2 && kind(operand) <= 1
        || !zeros(operand)
            && arithmetic
            && if wide_float {
                paths.tier() == Tier::Model
            } else {
                !paths.f16c
            }
}

/// The arithmetic floor, on the paths that compute it.
fn arithmetic(paths: &Paths, wide_float: bool, floor: usize) -> usize {
    let able = if wide_float {
        paths.tier() != Tier::Model
    } else {
        paths.f16c
    };
    if able { floor } else { 0 }
}

/// Covered and neither zeros nor another kind or mode.
fn covered(operand: u64, kinds: &[u64], modes: &[u64]) -> bool {
    plain(operand)
        && !zeros(operand)
        && kinds.contains(&kind(operand))
        && modes.contains(&mode(operand))
}

const VECFP: Case = Case {
    name: "vecfp",
    seed: 0x510e_527f_ade6_82d1,
    operations: &[AmxOperation::VectorFloat],
    generations: &GENERATIONS,
    iterations: 6000,
    fill: harness::fill,
    operand: |random| operand_of(random, &[0, 1, 4, 5, 7]),
    runs_on: |_| true,
    may_refuse,
    forms: &[
        Form {
            name: "f32/f64 z ± x*y",
            of: |_, operand| covered(operand, &[4, 7], &[0, 1]),
            floor: |paths| arithmetic(paths, true, 150),
        },
        Form {
            name: "f16 z ± x*y",
            of: |_, operand| covered(operand, &[2, 5, 9], &[0, 1]),
            floor: |paths| arithmetic(paths, false, 150),
        },
        Form {
            name: "f16-f32 z ± x*y",
            of: |_, operand| covered(operand, &[3], &[0, 1]),
            floor: |paths| arithmetic(paths, false, 50),
        },
        Form {
            name: "select (4)",
            of: |_, operand| covered(operand, &[4, 7, 3, 2, 5, 9], &[4]),
            floor: |_| 150,
        },
        Form {
            name: "min (5)",
            of: |_, operand| covered(operand, &[4, 7, 3, 2, 5, 9], &[5]),
            floor: |_| 150,
        },
        Form {
            name: "max (7)",
            of: |_, operand| covered(operand, &[4, 7, 3, 2, 5, 9], &[7]),
            floor: |_| 150,
        },
        Form {
            name: "zeros",
            of: |_, operand| plain(operand) && zeros(operand),
            floor: |_| 100,
        },
        Form {
            name: "broadcast Y lane",
            of: |_, operand| plain(operand) && field(operand).0 == 1,
            floor: |_| 300,
        },
    ],
};

#[test]
fn vecfp_matches_the_model_bit_for_bit() {
    differential(&VECFP, &native);
}

/// Modes 10-12, which M1 leaves as no-ops.
const VECFP_M2: Case = Case {
    name: "vecfp M2",
    seed: 0x9b05_688c_2b3e_6c1f,
    operations: &[AmxOperation::VectorFloat],
    generations: &[Generation::M2, Generation::M3, Generation::M4],
    iterations: 6000,
    fill: harness::fill,
    operand: |random| operand_of(random, &[10, 11, 12]),
    runs_on: |_| true,
    may_refuse,
    forms: &[
        Form {
            name: "f32/f64 x*y",
            of: |_, operand| covered(operand, &[4, 7], &[10]),
            floor: |paths| arithmetic(paths, true, 100),
        },
        Form {
            name: "f32/f64 z + x, z + y",
            of: |_, operand| covered(operand, &[4, 7], &[11, 12]),
            floor: |paths| arithmetic(paths, true, 200),
        },
        Form {
            name: "f16 x*y, z + x, z + y",
            of: |_, operand| covered(operand, &[2, 5, 9], &[10, 11, 12]),
            floor: |paths| arithmetic(paths, false, 200),
        },
        Form {
            name: "f16-f32 x*y, z + x, z + y",
            of: |_, operand| covered(operand, &[3], &[10, 11, 12]),
            floor: |paths| arithmetic(paths, false, 100),
        },
    ],
};

#[test]
fn vecfp_m2_modes_match_the_model_bit_for_bit() {
    differential(&VECFP_M2, &native);
}

/// Special values in X, Y and Z lanes, rotated so that many triples meet,
/// in every mode, for f32, f64, f16 and f16 into f32, with every lane on,
/// in Z rows 4 and 9: each completes natively where its path computes it
/// and matches the model.
#[test]
fn vecfp_special_values_match_the_model() {
    let singles: Vec<u64> = SINGLES.iter().map(|&value| u64::from(value)).collect();
    let halves: Vec<u64> = HALVES.iter().map(|&value| u64::from(value)).collect();
    for paths in harness::tiers() {
        for (kind, values, bytes) in [
            (4u64, &singles[..], 4),
            (7, &DOUBLES[..], 8),
            (2, &halves[..], 2),
            (3, &halves[..], 2),
        ] {
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
                // Z widened from f16 holds f32 lanes: give it f32 specials.
                if kind == 3 {
                    for (row, z) in registers.z.iter_mut().enumerate() {
                        for lane in 0..16 {
                            let single = SINGLES[(lane * 7 + row + rotation) % SINGLES.len()];
                            z[4 * lane..4 * lane + 4].copy_from_slice(&single.to_le_bytes());
                        }
                    }
                }
                for mode in [0u64, 1, 4, 5, 7, 10, 11, 12] {
                    for z_row in [4u64, 9] {
                        let operand = kind << 42 | mode << 47 | z_row << 20;
                        let covered = check(
                            &native,
                            &paths,
                            Generation::M2,
                            &registers,
                            AmxOperation::VectorFloat,
                            operand,
                        );
                        assert!(
                            covered
                                || may_refuse(
                                    &paths,
                                    AmxOperation::VectorFloat,
                                    Generation::M2,
                                    operand
                                ),
                            "{paths:?} vecfp {operand:#x} left to the model"
                        );
                    }
                }
            }
        }
    }
}

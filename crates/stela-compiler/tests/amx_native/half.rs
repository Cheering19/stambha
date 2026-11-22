//! `fma16`/`fms16` and `vecfp` on f16 lanes, which need F16C.
use crate::harness::{self, Case, Form, GENERATIONS, Paths, Random, check, differential, native};
use stela_compiler::amx::{AmxOperation, AmxRegisters, Generation};

const VECTOR: u64 = 1 << 63;
/// `fma16`/`fms16` bit 62: f32 accumulation into interleaved row pairs.
const WIDEN_16_32: u64 = 1 << 62;

/// `vecfp` operands over the covered f16 forms, with some outside them.
fn vector_operand(random: &mut Random) -> u64 {
    let mut operand = random.next();
    let choice = random.next();
    // Mostly no 54..56, no indexed load, no shuffle, mode 0 or 1.
    if choice & 7 != 0 {
        operand &= !(7 << 54 | 1 << 53 | 0xf << 27 | 0x3f << 47);
        operand |= (choice >> 3 & 1) << 47;
    }
    // Mostly f16 types (2, 5, 6, 8..15; 0/1 are f16 only on M1).
    if choice & 0x30 != 0 {
        operand &= !(0xf << 42);
        operand |= [2u64, 5, 6, 8, 9, 15, 0, 1][(choice >> 6) as usize & 7] << 42;
    }
    if choice & 0x300 != 0 {
        operand &= !(1 << 31);
    }
    // Broadcast Y lane often.
    if choice & 0xc00 != 0 {
        operand &= !(7 << 38);
        operand |= 1 << 38;
    }
    operand
}

/// Where the f16 `vecfp` path leaves an operand to the model: without
/// F16C; bits 56:54, an indexed load, a mode past 1, a type that is not
/// f16 (bf16 on M2 and later), the M2 multi-row form, a shuffle, and the
/// lane-enable values 3 to 5 of mode 0 (`native::x86::vector_half`).
fn vecfp_may_refuse(paths: &Paths, _: AmxOperation, generation: Generation, operand: u64) -> bool {
    let m2 = generation != Generation::M1;
    let operand = operand & !(1 << 37);
    let mode = (operand >> 47) & 0x3f;
    let kind = (operand >> 42) & 0xf;
    let wider = matches!(kind, 3 | 4 | 7) || m2 && kind <= 1;
    let field = (operand >> 38) & 7;
    let value = (operand >> 32) & 0x3f;
    !paths.f16c
        || (operand >> 54) & 7 != 0
        || operand & 1 << 53 != 0
        || mode > 1
        || wider
        || m2 && operand & 1 << 31 != 0
        || (operand >> 27) & 0xf != 0
        || field == 0 && matches!(value, 3..=5)
}

/// The unit test's seed, operands and floor (more than 2000 of 20000 per
/// generation), on M3 as well, and a floor for each lane form and for
/// `z - x*y`: about half of what each ran.
const VECFP_HALF: Case = Case {
    name: "vecfp f16",
    seed: 0x3c6e_f372_fe94_f82b,
    operations: &[AmxOperation::VectorFloat],
    generations: &GENERATIONS,
    iterations: 20000,
    fill: harness::fill,
    operand: vector_operand,
    runs_on: |paths| paths.f16c,
    may_refuse: vecfp_may_refuse,
    forms: &[
        Form {
            name: "f16 vecfp",
            of: |_, _| true,
            floor: |_| 2001,
        },
        Form {
            name: "broadcast Y lane",
            of: |_, operand| (operand >> 38) & 7 == 1,
            floor: |_| 4000,
        },
        Form {
            name: "lane enables",
            of: |_, operand| (operand >> 38) & 7 != 1,
            floor: |_| 1100,
        },
        Form {
            name: "z - x*y",
            of: |_, operand| (operand >> 47) & 1 != 0,
            floor: |_| 2600,
        },
    ],
};

#[test]
fn vecfp_half_matches_the_model_bit_for_bit() {
    differential(&VECFP_HALF, &native);
}

/// The fused-operation operands, mostly with bits 29:27 clear.
fn half_operand(random: &mut Random) -> u64 {
    let mut operand = harness::operand(random);
    if !random.next().is_multiple_of(4) {
        operand &= !(7 << 27);
    }
    operand
}

/// Where the f16 fused path leaves an operand to the model: copies of the
/// bit-62 widened matrix (`native::x86::fused_half`).
fn fma16_may_refuse(_: &Paths, _: AmxOperation, _: Generation, operand: u64) -> bool {
    let mode = (operand >> 27) & 7;
    let widened = operand & VECTOR == 0 && operand & WIDEN_16_32 != 0;
    widened && matches!(mode, 3 | 5)
}

/// The fma16 shape of an operand: vector, f16 matrix, f32-widening matrix.
fn shape(operand: u64) -> usize {
    if operand & VECTOR != 0 {
        0
    } else if operand & WIDEN_16_32 == 0 {
        1
    } else {
        2
    }
}

/// The fused-operation operands with bits 29:27 of 1 (product), 2 (z ± x)
/// or 4 (z ± y).
fn alu_operand(random: &mut Random) -> u64 {
    let operand = harness::operand(random) & !(7 << 27);
    operand | [1u64, 2, 4][(random.next() % 3) as usize] << 27
}

/// Each mode's floor per shape: about half of what it ran.
const fn mode_shape(mode: u64, shape_index: usize) -> Form {
    const NAMES: [[&str; 3]; 3] = [
        ["vector x*y", "f16 matrix x*y", "widened matrix x*y"],
        ["vector z ± x", "f16 matrix z ± x", "widened matrix z ± x"],
        ["vector z ± y", "f16 matrix z ± y", "widened matrix z ± y"],
    ];
    Form {
        name: NAMES[mode as usize / 2][shape_index],
        of: match (mode, shape_index) {
            (1, 0) => |_, operand| (operand >> 27) & 7 == 1 && shape(operand) == 0,
            (1, 1) => |_, operand| (operand >> 27) & 7 == 1 && shape(operand) == 1,
            (1, _) => |_, operand| (operand >> 27) & 7 == 1 && shape(operand) == 2,
            (2, 0) => |_, operand| (operand >> 27) & 7 == 2 && shape(operand) == 0,
            (2, 1) => |_, operand| (operand >> 27) & 7 == 2 && shape(operand) == 1,
            (2, _) => |_, operand| (operand >> 27) & 7 == 2 && shape(operand) == 2,
            (_, 0) => |_, operand| (operand >> 27) & 7 == 4 && shape(operand) == 0,
            (_, 1) => |_, operand| (operand >> 27) & 7 == 4 && shape(operand) == 1,
            _ => |_, operand| (operand >> 27) & 7 == 4 && shape(operand) == 2,
        },
        floor: if shape_index == 0 { |_| 80 } else { |_| 40 },
    }
}

/// fma16/fms16 product-only and add-only modes in every shape: none is
/// left to the model.
const FMA16_ALU: Case = Case {
    name: "fma16 alu",
    seed: 0xbb67_ae85_84ca_a73b,
    operations: &[AmxOperation::Fma16, AmxOperation::Fms16],
    generations: &GENERATIONS,
    iterations: 600,
    fill: harness::fill,
    operand: alu_operand,
    runs_on: |paths| paths.f16c,
    may_refuse: |_, _, _, _| false,
    forms: &[
        mode_shape(1, 0),
        mode_shape(1, 1),
        mode_shape(1, 2),
        mode_shape(2, 0),
        mode_shape(2, 1),
        mode_shape(2, 2),
        mode_shape(4, 0),
        mode_shape(4, 1),
        mode_shape(4, 2),
    ],
};

#[test]
fn fma16_product_and_add_modes_match_the_model_bit_for_bit() {
    differential(&FMA16_ALU, &native);
}

/// fma16/fms16 modes 1, 2 and 4 over every pair of the special halves of
/// [`HALVES`] (in the vector's lane pairs, the f16 matrix's and the
/// widened matrix's outer pairs), with Z the first of each pair, in every
/// shape: each completes natively and rounds as the model does.
#[test]
fn fma16_product_and_add_modes_round_once_over_every_special_pair() {
    let pairs: Vec<(u16, u16)> = HALVES
        .iter()
        .flat_map(|&z| HALVES.iter().map(move |&other| (z, other)))
        .collect();
    let mut ran = 0;
    for paths in harness::tiers().iter().filter(|paths| paths.f16c) {
        ran += 1;
        for chunk in pairs.chunks(32) {
            let mut registers = AmxRegisters::zeroed();
            for (lane, &(z, other)) in chunk.iter().enumerate() {
                for row in registers.z.iter_mut() {
                    row[2 * lane..2 * lane + 2].copy_from_slice(&z.to_le_bytes());
                }
                registers.x[0][2 * lane..2 * lane + 2].copy_from_slice(&other.to_le_bytes());
                registers.y[0][2 * lane..2 * lane + 2].copy_from_slice(&other.to_le_bytes());
            }
            for operation in [AmxOperation::Fma16, AmxOperation::Fms16] {
                for mode in [1u64, 2, 4] {
                    for shape in [VECTOR, 1 << 20, WIDEN_16_32] {
                        let operand = shape | mode << 27;
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
    assert!(ran > 0, "no F16C path ran");
}

/// The unit test's seed, operands and floors (more than 500 of 6000 per
/// shape on M2), with its 6000 operands spread over the four generations.
const FMA16: Case = Case {
    name: "fma16",
    seed: 0x6a09_e667_f3bc_c908,
    operations: &[AmxOperation::Fma16, AmxOperation::Fms16],
    generations: &GENERATIONS,
    iterations: 750,
    fill: harness::fill,
    operand: half_operand,
    runs_on: |paths| paths.f16c,
    may_refuse: fma16_may_refuse,
    forms: &[
        Form {
            name: "vector",
            of: |_, operand| operand & VECTOR != 0,
            floor: |_| 126,
        },
        Form {
            name: "f16 matrix",
            of: |_, operand| operand & (VECTOR | WIDEN_16_32) == 0,
            floor: |_| 126,
        },
        Form {
            name: "f32-widening matrix",
            of: |_, operand| operand & VECTOR == 0 && operand & WIDEN_16_32 != 0,
            floor: |_| 126,
        },
    ],
};

/// fma16/fms16 in vector, f16 matrix and f32-widening matrix shapes, with
/// all lanes on or random enables.
#[test]
fn fma16_matches_the_model_bit_for_bit() {
    differential(&FMA16, &native);
}

/// Special halves: zeros, denormals and their limits, ±1 and its
/// neighbours, the largest finite values, infinities and NaNs, and values
/// whose exact sums fall on, near and between f16 rounding points.
const HALVES: [u16; 24] = [
    0x0000, 0x8000, 0x0001, 0x8001, 0x03ff, 0x0400, 0x3c00, 0xbc00, 0x3c01, 0x3bff, 0x7bff, 0xfbff,
    0x7c00, 0xfc00, 0x7e00, 0x7c01, 0x1400, 0x1001, 0x5bff, 0x2400, 0x0200, 0x6800, 0xe800, 0x0c00,
];

/// Exhaustive-style f16 corner cases: every special pairing and values
/// whose exact sums fall on, near and between f16 rounding points, which
/// every F16C path must complete natively in every generation.
#[test]
fn vecfp_half_rounds_once_at_ties_and_extremes() {
    let mut ran = 0;
    for paths in harness::tiers().iter().filter(|paths| paths.f16c) {
        ran += 1;
        for generation in GENERATIONS {
            let mut registers = AmxRegisters::zeroed();
            for (index, &x) in HALVES.iter().enumerate() {
                for lane in 0..32 {
                    let y = HALVES[lane % HALVES.len()];
                    let z = HALVES[(lane * 7 + index) % HALVES.len()];
                    registers.x[0][2 * lane..2 * lane + 2].copy_from_slice(&x.to_le_bytes());
                    registers.y[0][2 * lane..2 * lane + 2].copy_from_slice(&y.to_le_bytes());
                    registers.z[3][2 * lane..2 * lane + 2].copy_from_slice(&z.to_le_bytes());
                }
                for mode in [0u64, 1] {
                    let operand = 2 << 42 | mode << 47 | 3 << 20;
                    assert!(
                        check(
                            &native,
                            paths,
                            generation,
                            &registers,
                            AmxOperation::VectorFloat,
                            operand
                        ),
                        "{paths:?} {generation:?} x {x:#06x} mode {mode} left to the model"
                    );
                }
            }
        }
    }
    assert!(ran > 0, "no F16C path ran");
}

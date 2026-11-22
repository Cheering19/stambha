//! `mac16` on the AVX-512F tier, in every shape and field.
use crate::harness::{Case, Form, GENERATIONS, Paths, Random, check, differential, native};
use stela_compiler::amx::{AmxOperation, AmxRegisters, Generation, native::Tier};

const VECTOR: u64 = 1 << 63;
const WIDEN_16_32: u64 = 1 << 62;
const X_INT8: u64 = 1 << 61;
const Y_INT8: u64 = 1 << 60;

/// i16 extremes and neighbours, and values whose low bytes are int8's
/// extremes (0x80, 0x7f, 0xff) under other high bytes.
const EDGES: [u16; 12] = [
    0x8000, 0xffff, 0x0000, 0x0001, 0x7fff, 0x8001, 0x7ffe, 0x0080, 0x007f, 0x00ff, 0x1280, 0xfe7f,
];

/// Registers of edge i16 lanes between random ones.
fn fill(random: &mut Random) -> AmxRegisters {
    let mut registers = AmxRegisters::zeroed();
    for lane in registers.as_bytes_mut().as_chunks_mut::<2>().0 {
        let value = random.next();
        *lane = if value.is_multiple_of(3) {
            EDGES[(value >> 8) as usize % EDGES.len()]
        } else {
            (value >> 16) as u16
        }
        .to_le_bytes();
    }
    registers
}

/// Operands over every field `mac16` reads: shifts often 0, 1, 15, 16 or
/// 31, lane enables often all on, every shape and selection.
fn operand(random: &mut Random) -> u64 {
    let mut operand = random.next();
    let choice = random.next();
    if choice & 3 != 0 {
        operand &= !(0x1f << 55);
        operand |= [0u64, 1, 15, 16, 31, 7][(choice >> 2) as usize % 6] << 55;
    }
    if choice & 0x30 != 0 {
        operand &= !(0x7f << 41);
    }
    if choice & 0xc0 != 0 {
        operand &= !(0x7f << 32);
    }
    operand
}

fn shape(operand: u64) -> usize {
    if operand & VECTOR != 0 {
        0
    } else if operand & WIDEN_16_32 == 0 {
        1
    } else {
        2
    }
}

/// `floor` on the AVX-512F tier, none below it.
fn on_avx512(paths: &Paths, floor: usize) -> usize {
    if paths.tier() == Tier::Avx512 {
        floor
    } else {
        0
    }
}

/// Every `mac16` operand completes natively on the AVX-512F tier and none
/// below it; floors per shape, selection, int8 input and Z input.
const MAC16: Case = Case {
    name: "mac16",
    seed: 0x3c6e_f372_fe94_f82b,
    operations: &[AmxOperation::Mac16],
    generations: &GENERATIONS,
    iterations: 3000,
    fill,
    operand,
    runs_on: |_| true,
    may_refuse: |paths, _, _, _| paths.tier() != Tier::Avx512,
    forms: &[
        Form {
            name: "vector",
            of: |_, operand| shape(operand) == 0,
            floor: |paths| on_avx512(paths, 700),
        },
        Form {
            name: "matrix",
            of: |_, operand| shape(operand) == 1,
            floor: |paths| on_avx512(paths, 350),
        },
        Form {
            name: "widened matrix",
            of: |_, operand| shape(operand) == 2,
            floor: |paths| on_avx512(paths, 350),
        },
        Form {
            name: "x*y",
            of: |_, operand| (operand >> 28) & 3 == 0,
            floor: |paths| on_avx512(paths, 350),
        },
        Form {
            name: "x",
            of: |_, operand| (operand >> 28) & 3 == 1,
            floor: |paths| on_avx512(paths, 350),
        },
        Form {
            name: "y",
            of: |_, operand| (operand >> 28) & 3 == 2,
            floor: |paths| on_avx512(paths, 350),
        },
        Form {
            name: "0",
            of: |_, operand| (operand >> 28) & 3 == 3,
            floor: |paths| on_avx512(paths, 350),
        },
        Form {
            name: "int8 x and y",
            of: |_, operand| operand & (X_INT8 | Y_INT8) == X_INT8 | Y_INT8,
            floor: |paths| on_avx512(paths, 350),
        },
        Form {
            name: "no z input",
            of: |_, operand| operand & 1 << 27 != 0,
            floor: |paths| on_avx512(paths, 700),
        },
    ],
};

#[test]
fn mac16_matches_the_model_bit_for_bit() {
    differential(&MAC16, &native);
}

/// Every pair of edge lanes, as the vector's lane pairs and the matrices'
/// outer pairs, in every selection and int8 choice, with shifts 0, 1, 15,
/// 16 and 31, with and without Z: each completes natively on the AVX-512F
/// tier and matches the model.
#[test]
fn mac16_edges_match_the_model_in_every_shape() {
    let pairs: Vec<(u16, u16)> = EDGES
        .iter()
        .flat_map(|&a| EDGES.iter().map(move |&b| (a, b)))
        .collect();
    let mut ran = 0;
    for paths in crate::harness::tiers()
        .iter()
        .filter(|paths| paths.tier() == Tier::Avx512)
    {
        ran += 1;
        for chunk in pairs.chunks(32) {
            let mut registers = AmxRegisters::zeroed();
            for (lane, &(a, b)) in chunk.iter().enumerate() {
                registers.x[0][2 * lane..2 * lane + 2].copy_from_slice(&a.to_le_bytes());
                registers.y[0][2 * lane..2 * lane + 2].copy_from_slice(&b.to_le_bytes());
                for (row, z) in registers.z.iter_mut().enumerate() {
                    let value = EDGES[(lane + row) % EDGES.len()];
                    z[2 * lane..2 * lane + 2].copy_from_slice(&value.to_le_bytes());
                }
            }
            for shape in [VECTOR, 1 << 20, WIDEN_16_32] {
                for selection in 0..4u64 {
                    for int8 in [0, X_INT8, Y_INT8, X_INT8 | Y_INT8] {
                        for shift in [0u64, 1, 15, 16, 31] {
                            for skip in [0, 1 << 27] {
                                let operand = shape | int8 | shift << 55 | selection << 28 | skip;
                                assert!(
                                    check(
                                        &native,
                                        paths,
                                        Generation::M2,
                                        &registers,
                                        AmxOperation::Mac16,
                                        operand
                                    ),
                                    "{paths:?} mac16 {operand:#x} left to the model"
                                );
                            }
                        }
                    }
                }
            }
        }
    }
    assert!(ran > 0, "no AVX-512F path ran");
}

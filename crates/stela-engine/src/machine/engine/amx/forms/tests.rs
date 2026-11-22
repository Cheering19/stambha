use super::*;
use alloc::{collections::BTreeSet, string::ToString};
use stela_compiler::{amx::AmxOperation::*, x86_64::amx_routine};

const GENERATIONS: [Generation; 4] = [
    Generation::M1,
    Generation::M2,
    Generation::M3,
    Generation::M4,
];
const Y_HALF: u64 = 1 << 60;

struct Random(u64);

impl Random {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
}

/// Random operands, often with the fields that stage or select a form clear.
fn operand(random: &mut Random) -> u64 {
    let mut operand = random.next();
    let choice = random.next();
    if choice & 1 != 0 {
        operand &= !(0xf << 27);
    }
    if choice & 2 != 0 {
        operand &= !(7 << 54 | 1 << 53);
    }
    if choice & 4 != 0 {
        operand &= !(1 << 31);
    }
    operand
}

fn register_operations() -> impl Iterator<Item = AmxOperation> {
    AmxOperation::ALL
        .into_iter()
        .filter(|&operation| amx_routine::calls(operation))
}

#[test]
fn every_form_is_taken_and_only_register_operations_have_one() {
    let mut random = Random(0x9e37_79b9_7f4a_7c15);
    let mut taken = [false; FORMS];
    for operation in register_operations() {
        for generation in GENERATIONS {
            for _ in 0..4000 {
                let form = key(operation, operand(&mut random), generation).unwrap();
                taken[form] = true;
            }
        }
    }
    let untaken: Vec<_> = (0..FORMS).filter(|&form| !taken[form]).collect();
    assert!(untaken.is_empty(), "{untaken:?}");
    for operation in [LoadX, StoreZ, LoadZInterleaved, Set, Clear, Unallocated] {
        assert_eq!(
            key(operation, u64::MAX, Generation::M2),
            None,
            "{operation:?}"
        );
    }
}

#[test]
fn form_names_are_distinct_and_name_their_operation() {
    let names: BTreeSet<_> = (0..FORMS).map(|form| FormName(form).to_string()).collect();
    assert_eq!(names.len(), FORMS);
    let mut random = Random(0x3c6e_f372_fe94_f82b);
    for operation in register_operations() {
        let family = match operation {
            Fms32 => "fma32",
            Fms64 => "fma64",
            Fms16 => "fma16",
            _ => operation.mnemonic(),
        };
        for _ in 0..256 {
            let form = key(operation, operand(&mut random), Generation::M2).unwrap();
            let name = FormName(form).to_string();
            assert!(name.starts_with(family), "{operation:?} {name}");
        }
    }
}

/// The hot BNNS and Espresso forms `amx::native`'s timing names: each its
/// own key.
#[test]
fn the_named_forms_have_keys_of_their_own() {
    let named = [
        ("fma32/matrix/fused/f16", Fma32, Y_HALF | 1 << 20),
        ("fma32/vector/fused", Fma32, VECTOR | 5 << 20),
        ("fma64/matrix/fused", Fma64, 0),
        (
            "vecfp/f16/fma",
            VectorFloat,
            2 << 42 | 1 << 38 | 3 << 32 | 8 << 20,
        ),
        ("fma16/widened/copy", Fma16, 0x4000_0000_3800_0000),
        ("fma32/matrix/copy", Fma32, 7 << 27 | 3 << 20),
    ];
    let keys: BTreeSet<_> = named
        .iter()
        .map(|&(name, operation, operand)| {
            let form = key(operation, operand, Generation::M2).unwrap();
            assert_eq!(FormName(form).to_string(), name);
            form
        })
        .collect();
    assert_eq!(keys.len(), named.len());
}

/// Offsets, the Z row, lane enables and fma's sign choose no form; the
/// precision, shape, mode class, f16 widening, element kind (by
/// generation) and staging do.
#[test]
fn a_form_is_the_fields_that_choose_its_work() {
    let mut random = Random(0x6a09_e667_f3bc_c908);
    for operation in register_operations() {
        for generation in GENERATIONS {
            for _ in 0..512 {
                let operand = operand(&mut random);
                let form = key(operation, operand, generation);
                let moved = operand ^ random.next() & 0x3ff_ffff;
                assert_eq!(key(operation, moved, generation), form, "{operation:?}");
                if matches!(
                    operation,
                    Fma32 | Fms32 | Fma64 | Fms64 | Fma16 | Fms16 | Mac16
                ) {
                    let lanes = operand ^ random.next() & (0x7f << 41 | 0x7f << 32);
                    assert_eq!(key(operation, lanes, generation), form, "{operation:?}");
                }
            }
        }
    }
    for operand in [0, VECTOR, 1 << 27, 6 << 27, Y_HALF] {
        assert_eq!(
            key(Fms32, operand, Generation::M2),
            key(Fma32, operand, Generation::M2)
        );
        assert_eq!(
            key(Fms16, operand, Generation::M2),
            key(Fma16, operand, Generation::M2)
        );
    }
    let name = |operation, operand, generation| {
        FormName(key(operation, operand, generation).unwrap()).to_string()
    };
    assert_eq!(name(Fma32, 2 << 27, Generation::M2), "fma32/matrix/add");
    assert_eq!(
        name(Fma32, 4 << 27 | 1 << 61, Generation::M2),
        "fma32/matrix/add/f16"
    );
    assert_eq!(
        name(Fma64, VECTOR | 1 << 27, Generation::M2),
        "fma64/vector/product"
    );
    assert_eq!(
        name(Fma16, WIDEN_16_32, Generation::M2),
        "fma16/widened/fused"
    );
    assert_eq!(
        name(Fma16, VECTOR | WIDEN_16_32, Generation::M2),
        "fma16/vector/fused"
    );
    assert_eq!(
        name(Mac16, WIDEN_16_32 | 3 << 60, Generation::M2),
        "mac16/widened"
    );
    assert_eq!(name(VectorFloat, 0, Generation::M1), "vecfp/f16/fma");
    assert_eq!(name(VectorFloat, 0, Generation::M2), "vecfp/bf16/fma");
    assert_eq!(
        name(VectorFloat, 1 << 42 | 10 << 47, Generation::M3),
        "vecfp/bf16-f32/alu"
    );
    assert_eq!(
        name(VectorFloat, 3 << 42 | 4 << 47, Generation::M4),
        "vecfp/f16-f32/alu"
    );
    assert_eq!(
        name(VectorFloat, 7 << 42 | 1 << 47, Generation::M2),
        "vecfp/f64/fma"
    );
    assert_eq!(
        name(VectorFloat, 4 << 42 | MULTIPLE_ROWS, Generation::M1),
        "vecfp/f32/fma"
    );
    assert_eq!(
        name(VectorFloat, 4 << 42 | MULTIPLE_ROWS, Generation::M2),
        "vecfp/staged"
    );
    assert_eq!(name(VectorFloat, 1 << 29, Generation::M2), "vecfp/staged");
    assert_eq!(
        name(VectorFloat, INDEXED_LOAD, Generation::M2),
        "vecfp/staged"
    );
    assert_eq!(name(VectorFloat, 1 << 54, Generation::M2), "vecfp/staged");
    assert_eq!(
        name(MatrixFloat, 4 << 42 | 4 << 47, Generation::M2),
        "matfp/f32"
    );
    assert_eq!(name(MatrixFloat, 1 << 42, Generation::M1), "matfp/f16");
    assert_eq!(name(MatrixFloat, 1 << 27, Generation::M2), "matfp/staged");
    assert_eq!(name(GenerateLookup, 0, Generation::M2), "genlut");
}

#[test]
fn the_line_ranks_forms_by_operations_and_prices_their_samples() {
    let fused = key(Fma32, 0, Generation::M2).unwrap();
    let matfp = key(MatrixFloat, 4 << 42, Generation::M2).unwrap();
    let mut before = AmxCounts {
        operations: [10, 1],
        moves: [5, 0],
        ..AmxCounts::default()
    };
    before.forms.counts[fused] = [10, 0];
    before.forms.sampled[fused][0] = [1000, 1];
    let mut now = before;
    now.operations = [1010, 3];
    now.moves = [2005, 1];
    now.forms.counts[fused] = [910, 0];
    // 14 sampled calls of 80 ticks at 2 GHz: 40 ns each.
    now.forms.sampled[fused][0] = [1000 + 14 * 80, 15];
    now.forms.counts[matfp] = [0, 100];
    now.forms.sampled[matfp][1] = [4000, 2];
    let tsc_hz = 2_000_000_000;
    assert_eq!(
        now.line(&before, tsc_hz, 16),
        "ops=1000/2 moves=2000/1 native=90% forms=2 fma32/matrix/fused:900/0/40/- \
         matfp/f32:0/100/-/1000"
    );
    assert_eq!(
        now.line(&before, tsc_hz, 1),
        "ops=1000/2 moves=2000/1 native=90% forms=2 fma32/matrix/fused:900/0/40/-"
    );
    assert_eq!(
        now.line(&now, tsc_hz, 16),
        "ops=0/0 moves=0/0 native=- forms=0"
    );
    let mut sum = before;
    sum += now;
    assert_eq!(sum.operations, [1020, 4]);
    assert_eq!(sum.forms.counts[fused], [920, 0]);
    assert_eq!(sum.forms.sampled[matfp][1], [4000, 2]);
}

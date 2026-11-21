//! Scalar FCVT and vector FCVTL and FCVTN between S and D over every input
//! class against the Arm reference, compiled as blocks for every tier this
//! host has: every inline completion matches the reference in every V
//! register and FPSR, every exit leaves the state unchanged, D to S
//! completes every non-NaN input inline with FZ, FIZ, AH and the trap
//! enables clear (tiny, subnormal and overflowing results included), and
//! both directions flush a subnormal input inline under FZ or FIZ with AH
//! clear.
use crate::{
    GuestAddress, GuestState, Options, TargetCpu,
    aarch64::floating::{
        FPCR_AH, FPCR_DN, FPCR_EXCEPTION_ENABLE_MASK, FPCR_FIZ, FPCR_FZ, FPCR_NEP,
        FPCR_ROUNDING_SHIFT, Format,
        precision::{self, PrecisionRounding},
    },
    block::{BlockProfile, CompilationPlan},
    x86_64::native::run,
};
use alloc::{vec, vec::Vec};

const PC: GuestAddress = GuestAddress(0x1000_0000);
const RETURN: u64 = 0x4000;
const D: u32 = 3;
const N: u32 = 4;

fn targets() -> Vec<TargetCpu> {
    let host = TargetCpu::detect();
    [
        TargetCpu::Baseline,
        TargetCpu::Sse41,
        TargetCpu::X86_64_V2,
        TargetCpu::X86_64_V3,
        TargetCpu::X86_64_V4,
        TargetCpu::X86_64_V4.with_data_independent_timing(),
        host,
    ]
    .into_iter()
    .filter(|target| host.supports(*target))
    .collect()
}

fn compile(word: u32, target: TargetCpu) -> Vec<u8> {
    let source = [word, 0xd65f_03c0].map(u32::to_le_bytes).concat();
    CompilationPlan::new(&source, PC, &Options::new(BlockProfile::ORDINARY, target))
        .unwrap()
        .compile()
        .unwrap()
        .native
        .x86_64_code
}

fn controls() -> [u32; 11] {
    [
        0,
        1 << FPCR_ROUNDING_SHIFT,
        2 << FPCR_ROUNDING_SHIFT,
        3 << FPCR_ROUNDING_SHIFT,
        FPCR_DN,
        FPCR_NEP,
        FPCR_FZ,
        FPCR_FIZ,
        FPCR_FZ | FPCR_NEP,
        FPCR_AH,
        FPCR_EXCEPTION_ENABLE_MASK,
    ]
}

/// Zeros, subnormals, values around both formats' minimum normals and
/// maximums, normals, infinities and NaNs, both signs.
fn floats(format: Format) -> Vec<u64> {
    let fraction_bits = format.fraction_bits();
    let infinity = format.exponent_mask();
    let quiet = 1u64 << (fraction_bits - 1);
    let bias = u64::from(format.exponent_bias());
    let minimum = 1u64 << fraction_bits;
    let mut values = vec![
        0,
        1,
        minimum - 3,
        minimum,
        bias << fraction_bits,
        (bias << fraction_bits) + 1,
        infinity - 1,
        infinity,
        infinity | quiet,
        infinity | 1,
        // NaNs with payloads, signalling and quiet.
        infinity | (quiet - 1),
        infinity | quiet | quiet >> 3 | 5,
    ];
    if format == Format::Double {
        let single_bias = u64::from(Format::Single.exponent_bias());
        // Single's minimum normal, just below and above it, a value that
        // rounds up to it, Single's subnormal range and its overflow edge.
        let single_minimum = (bias - single_bias + 1) << fraction_bits;
        values.extend([
            single_minimum,
            single_minimum - 1,
            single_minimum + (1 << 28),
            single_minimum - (1 << 28),
            (bias - single_bias - 10) << fraction_bits | 0x1234_5678,
            (bias - single_bias - 30) << fraction_bits,
            (bias + single_bias) << fraction_bits | (minimum - 1),
            (bias + single_bias + 1) << fraction_bits,
            (bias + 3) << fraction_bits | 0x1000_0001,
        ]);
    }
    let positive = values.len();
    for index in 0..positive {
        values.push(values[index] | format.sign_mask());
    }
    values
}

fn word(source: Format) -> u32 {
    let registers = N << 5 | D;
    match source {
        Format::Double => 0x1e62_4000 | registers,
        _ => 0x1e22_c000 | registers,
    }
}

#[test]
fn scalar_conversions_complete_subnormal_tiny_and_flushed_inputs_inline() {
    let mut runs = 0;
    let mut inline = 0;
    for target in targets() {
        for (source, destination) in [
            (Format::Double, Format::Single),
            (Format::Single, Format::Double),
        ] {
            let code = compile(word(source), target);
            for bits in floats(source) {
                for fpcr in controls() {
                    let mut initial = GuestState {
                        fpcr,
                        fpsr: 0x0800_0000,
                        ..GuestState::default()
                    };
                    initial.x[30] = RETURN;
                    initial.v[N as usize] = [bits, 0x0123_4567_89ab_cdef];
                    initial.v[D as usize] = [0xdead_beef_0bad_f00d, 0xfeed_face_cafe_d00d];
                    let label = alloc::format!("{source:?} {bits:#x} {fpcr:#x} {target:?}");
                    let magnitude = bits & !source.sign_mask();
                    let nan = magnitude > source.exponent_mask();
                    let subnormal = magnitude != 0 && magnitude < 1 << source.fraction_bits();
                    let flushing = fpcr & (FPCR_FZ | FPCR_FIZ) != 0 && fpcr & FPCR_AH == 0;
                    let reference = precision::evaluate(
                        source,
                        destination,
                        PrecisionRounding::Fpcr,
                        fpcr,
                        bits,
                    );
                    let usable = fpcr & FPCR_EXCEPTION_ENABLE_MASK == 0
                        && !target.data_independent_timing()
                        && !nan;
                    let must_inline = usable
                        && if source == Format::Double {
                            let result = reference.as_ref().unwrap().bits;
                            let single_minimum = 1 << Format::Single.fraction_bits();
                            let rounded = fpcr & (FPCR_FZ | FPCR_FIZ | FPCR_AH) == 0;
                            rounded && result & !Format::Single.sign_mask() != single_minimum
                                || flushing && (subnormal || magnitude == 0)
                        } else {
                            !subnormal || flushing
                        };
                    let mut actual = initial.clone();
                    let exit = run(&code, &mut actual);
                    runs += 1;
                    if exit == PC.0 {
                        assert_eq!(actual, initial, "{label}");
                        assert!(!must_inline, "exited: {label}");
                        continue;
                    }
                    assert_eq!(exit, RETURN, "{label}");
                    inline += 1;
                    let outcome = reference.expect("completed with traps");
                    let mut expected = initial.clone();
                    expected.fpsr |= outcome.exceptions;
                    let mask = if destination == Format::Single {
                        u64::from(u32::MAX)
                    } else {
                        u64::MAX
                    };
                    expected.v[D as usize] = if fpcr & FPCR_NEP != 0 {
                        let old = initial.v[D as usize];
                        [old[0] & !mask | outcome.bits, old[1]]
                    } else {
                        [outcome.bits, 0]
                    };
                    assert_eq!(actual, expected, "{label}");
                }
            }
        }
    }
    // Scalar NaNs take the helper.
    assert!(inline > runs * 2 / 5, "{inline} of {runs} inline");
}

/// The sticky-inexact D to S converts before the code installs any MXCSR:
/// its CVTSD2SS (F2 0F 5A) comes before the full path's first LDMXCSR
/// (0F AE /2), where every conversion took four MXCSR instructions.
#[test]
fn sticky_double_to_single_converts_before_any_mxcsr_load() {
    let code = compile(word(Format::Double), TargetCpu::Baseline);
    let convert = code
        .windows(3)
        .position(|bytes| bytes == [0xf2, 0x0f, 0x5a])
        .expect("a CVTSD2SS");
    let load = code
        .windows(3)
        .position(|bytes| bytes[..2] == [0x0f, 0xae] && bytes[2] >> 3 & 7 == 2)
        .expect("an LDMXCSR");
    assert!(convert < load, "CVTSD2SS at {convert}, LDMXCSR at {load}");
    // FCVT with round to odd (FCVTXN) keeps its private rounding.
    let odd = compile(0x7e61_6800 | N << 5 | D, TargetCpu::Baseline);
    let convert = odd.windows(3).position(|bytes| bytes == [0xf2, 0x0f, 0x5a]);
    let load = odd
        .windows(3)
        .position(|bytes| bytes[..2] == [0x0f, 0xae] && bytes[2] >> 3 & 7 == 2);
    assert!(convert.is_none() || load < convert, "{convert:?} {load:?}");
}

/// Scalar D to S on the sticky-inexact path: with FPCR at its defaults and
/// FPSR.IXC already set, CVTSD2SS runs in the MXCSR native code is entered
/// with, and only a normal result other than the Single minimum normal, or
/// a zero from a zero, completes there. Over every input class, every
/// control and every tier, with IXC alone and every cumulative flag set
/// beforehand, each run ends where the same input with IXC clear ends (the
/// full path, which the sticky path leaves every such state to), with the
/// same V registers and FPSR but IXC, and matches the Arm reference.
#[test]
fn sticky_double_to_single_matches_the_full_path_bit_for_bit() {
    use crate::aarch64::floating::INEXACT;
    const CUMULATIVE: u32 = 0x9f;
    let (mut runs, mut inline) = (0, 0);
    for target in targets() {
        let code = compile(word(Format::Double), target);
        for bits in floats(Format::Double) {
            for fpcr in controls() {
                for cumulative in [INEXACT, CUMULATIVE] {
                    let state = |fpsr: u32| {
                        let mut state = GuestState {
                            fpcr,
                            fpsr: 0x0800_0000 | fpsr,
                            ..GuestState::default()
                        };
                        state.x[30] = RETURN;
                        state.v[N as usize] = [bits, 0x0123_4567_89ab_cdef];
                        state.v[D as usize] = [0xdead_beef_0bad_f00d, 0xfeed_face_cafe_d00d];
                        state
                    };
                    let label = alloc::format!("{bits:#x} {fpcr:#x} {cumulative:#x} {target:?}");
                    let (full_initial, sticky_initial) =
                        (state(cumulative & !INEXACT), state(cumulative));
                    let mut full = full_initial.clone();
                    let full_exit = run(&code, &mut full);
                    let mut sticky = sticky_initial.clone();
                    let exit = run(&code, &mut sticky);
                    runs += 1;
                    assert_eq!(exit, full_exit, "{label}");
                    // What completes inline at FPCR's defaults (DN aside)
                    // outside the DIT profile: a non-NaN input whose result
                    // is not the Single minimum normal (which Arm may find
                    // tiny where the host does not).
                    let magnitude = bits & !Format::Double.sign_mask();
                    let minimum = precision::evaluate(
                        Format::Double,
                        Format::Single,
                        PrecisionRounding::Fpcr,
                        0,
                        bits,
                    )
                    .is_ok_and(|outcome| {
                        outcome.bits & !Format::Single.sign_mask()
                            == 1 << Format::Single.fraction_bits()
                    });
                    let sticky_state = fpcr & !FPCR_DN == 0
                        && !target.data_independent_timing()
                        && magnitude <= Format::Double.exponent_mask()
                        && !minimum;
                    if exit == PC.0 {
                        assert_eq!(sticky, sticky_initial, "{label}");
                        assert!(!sticky_state, "exited: {label}");
                        continue;
                    }
                    inline += 1;
                    let mut expected = full;
                    expected.fpsr |= INEXACT;
                    assert_eq!(sticky, expected, "{label}");
                    let outcome = precision::evaluate(
                        Format::Double,
                        Format::Single,
                        PrecisionRounding::Fpcr,
                        fpcr,
                        bits,
                    )
                    .expect("completed with traps");
                    assert_eq!(
                        sticky.fpsr,
                        sticky_initial.fpsr | outcome.exceptions,
                        "{label}"
                    );
                    assert_eq!(
                        sticky.v[D as usize][0] as u32, outcome.bits as u32,
                        "{label}"
                    );
                }
            }
        }
    }
    assert!(inline > runs * 2 / 5, "{inline} of {runs} inline");
}

/// FCVTL, FCVTL2, FCVTN and FCVTN2 between S and D lanes over every input
/// class, two distinct classes per vector: with the trap enables and AH
/// clear, widening completes every vector inline (zeros, subnormals,
/// infinities and NaNs included, a subnormal flushed under FZ or FIZ), and
/// with FZ, FIZ and AH clear too, narrowing completes every vector without
/// a result exactly the Single minimum normal inline (tiny, subnormal,
/// zero, overflowing and NaN results included). Every
/// inline completion matches the reference, every exit leaves the state
/// unchanged.
#[test]
fn vector_conversions_complete_zero_subnormal_and_tiny_lanes_inline() {
    let mut runs = 0;
    let mut inline = 0;
    let registers = N << 5 | D;
    // (word, source, destination, upper half).
    let forms = [
        (
            0x0e61_7800 | registers,
            Format::Single,
            Format::Double,
            false,
        ),
        (
            0x4e61_7800 | registers,
            Format::Single,
            Format::Double,
            true,
        ),
        (
            0x0e61_6800 | registers,
            Format::Double,
            Format::Single,
            false,
        ),
        (
            0x4e61_6800 | registers,
            Format::Double,
            Format::Single,
            true,
        ),
    ];
    for target in targets() {
        for (word, source, destination, upper) in forms {
            let code = compile(word, target);
            let values = floats(source);
            for (index, &first) in values.iter().enumerate() {
                let second = values[(index * 7 + 3) % values.len()];
                for fpcr in controls() {
                    let mut initial = GuestState {
                        fpcr,
                        fpsr: 0x0800_0000,
                        ..GuestState::default()
                    };
                    initial.x[30] = RETURN;
                    initial.v[N as usize] = [0x0123_4567_89ab_cdef, 0x0fed_cba9_8765_4321];
                    initial.v[D as usize] = [0xdead_beef_0bad_f00d, 0xfeed_face_cafe_d00d];
                    let lanes = [first, second];
                    if source == Format::Single {
                        initial.v[N as usize][usize::from(upper)] = first | second << 32;
                    } else {
                        initial.v[N as usize] = lanes;
                    }
                    let label = alloc::format!("{word:08x} {lanes:x?} {fpcr:#x} {target:?}");
                    let outcomes: Vec<_> = lanes
                        .iter()
                        .map(|&bits| {
                            precision::evaluate(
                                source,
                                destination,
                                PrecisionRounding::Fpcr,
                                fpcr,
                                bits,
                            )
                        })
                        .collect();
                    let usable = fpcr & (FPCR_EXCEPTION_ENABLE_MASK | FPCR_AH) == 0
                        && !target.data_independent_timing();
                    let must_inline = usable
                        && (source == Format::Single
                            || fpcr & (FPCR_FZ | FPCR_FIZ) == 0
                                && outcomes.iter().all(|outcome| {
                                    let bits = outcome.as_ref().unwrap().bits;
                                    bits & !Format::Single.sign_mask()
                                        != 1 << Format::Single.fraction_bits()
                                }));
                    let mut actual = initial.clone();
                    let exit = run(&code, &mut actual);
                    runs += 1;
                    if exit == PC.0 {
                        assert_eq!(actual, initial, "{label}");
                        assert!(!must_inline, "exited: {label}");
                        continue;
                    }
                    assert_eq!(exit, RETURN, "{label}");
                    inline += 1;
                    let mut expected = initial.clone();
                    let results: Vec<u64> = outcomes
                        .into_iter()
                        .map(|outcome| {
                            let outcome = outcome.expect("completed with traps");
                            expected.fpsr |= outcome.exceptions;
                            outcome.bits
                        })
                        .collect();
                    expected.v[D as usize] = if destination == Format::Double {
                        [results[0], results[1]]
                    } else {
                        let packed = results[0] | results[1] << 32;
                        if upper {
                            [initial.v[D as usize][0], packed]
                        } else {
                            [packed, 0]
                        }
                    };
                    assert_eq!(actual, expected, "{label}");
                }
            }
        }
    }
    assert!(inline > runs / 3, "{inline} of {runs} inline");
}

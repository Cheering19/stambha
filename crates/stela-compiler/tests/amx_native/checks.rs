//! The harness itself: it passes the real paths and fails each fault a
//! stand-in for them injects, and a host without a covered tier fails
//! unless the tier is named optional.
use crate::harness::{Case, Form, Paths, differential, fill, native, paths_on, set_mxcsr};
use stela_compiler::{
    TargetCpu,
    amx::{AmxOperation, AmxRegisters, Generation, native::Tier},
};

/// A small case every non-model path completes in full: vector fma32 with
/// bits 29:27 and the widening bits clear. With it the harness must pass
/// the real paths and catch each fault a stand-in injects.
const PROBE: Case = Case {
    name: "probe",
    seed: 0x2545_f491_4f6c_dd1d,
    operations: &[AmxOperation::Fma32],
    generations: &[Generation::M2],
    iterations: 64,
    fill,
    operand: |random| random.next() & !(7 << 27 | 3 << 60) | 1 << 63,
    runs_on: |paths| paths.tier() != Tier::Model,
    may_refuse: |_, _, _, _| false,
    forms: &[Form {
        name: "vector fma32",
        of: |_, _| true,
        floor: |_| 64,
    }],
};

/// [`PROBE`] admitting every refusal, so that only its floor can catch one.
const PERMISSIVE: Case = Case {
    may_refuse: |_, _, _, _| true,
    ..PROBE
};

#[test]
fn harness_passes_the_real_paths() {
    differential(&PROBE, &native);
    differential(&PERMISSIVE, &native);
}

/// The real path, then one Z bit flipped.
fn wrong_lane(
    paths: &Paths,
    generation: Generation,
    registers: &mut AmxRegisters,
    operation: AmxOperation,
    operand: u64,
) -> bool {
    let covered = native(paths, generation, registers, operation, operand);
    registers.z[0][0] ^= 1;
    covered
}

/// The real path, then MXCSR left at the AMX environment.
fn leaves_its_mxcsr(
    paths: &Paths,
    generation: Generation,
    registers: &mut AmxRegisters,
    operation: AmxOperation,
    operand: u64,
) -> bool {
    let covered = native(paths, generation, registers, operation, operand);
    set_mxcsr(0x1f80);
    covered
}

#[test]
#[should_panic(expected = "native differs from the model")]
fn harness_catches_a_wrong_lane() {
    differential(&PROBE, &wrong_lane);
}

#[test]
#[should_panic(expected = "came back as 0x1f80")]
fn harness_catches_a_path_that_leaves_its_mxcsr() {
    differential(&PROBE, &leaves_its_mxcsr);
}

#[test]
#[should_panic(expected = "a refused operand changed the registers")]
fn harness_catches_a_refusal_that_changes_the_registers() {
    differential(&PERMISSIVE, &|_, _, registers, _, _| {
        registers.y[7][63] ^= 0x80;
        false
    });
}

#[test]
#[should_panic(expected = "which it covers")]
fn harness_catches_a_refusal_it_does_not_admit() {
    differential(&PROBE, &|_, _, _, _, _| false);
}

#[test]
#[should_panic(expected = "under its floor of 64")]
fn harness_catches_a_silent_refusal_by_its_floor() {
    differential(&PERMISSIVE, &|_, _, _, _, _| false);
}

/// A host without a covered tier fails the tests unless the tier is
/// named optional, which leaves its paths out with a note.
#[test]
fn a_lacking_tier_fails_unless_named_optional() {
    let fma_host = TargetCpu::X86_64_V3;
    let error = paths_on(fma_host, "").unwrap_err();
    assert!(error.contains("lacks avx512f"), "{error}");
    let (paths, notes) = paths_on(fma_host, "avx512f").unwrap();
    assert_eq!(paths.len(), 4);
    assert_eq!(notes.len(), 2, "{notes:?}");
    assert!(paths.iter().all(|paths| paths.tier() != Tier::Avx512));
    let error = paths_on(TargetCpu::X86_64_V2, "avx512f,f16c").unwrap_err();
    assert!(error.contains("lacks fma"), "{error}");
    let (paths, notes) = paths_on(TargetCpu::X86_64_V2, " fma , avx512f,f16c").unwrap();
    assert_eq!((paths.len(), notes.len()), (1, 5));
    assert!(
        paths_on(fma_host, "avx512bw")
            .unwrap_err()
            .contains("avx512bw")
    );
    let (paths, notes) = paths_on(TargetCpu::X86_64_V4, "").unwrap();
    assert_eq!((paths.len(), notes.len()), (6, 0));
}

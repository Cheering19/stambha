//! The differential harness, the host paths it runs and the register and
//! operand generators the cases share.
use core::arch::asm;
use stela_compiler::{
    TargetCpu,
    amx::{
        AmxOperation, AmxRegisters, Generation,
        model::{self, AmxMemory, MemoryFault},
        native::{Native, Tier},
        state::{ROW_BYTES, Y_OFFSET, Z_OFFSET},
    },
};

/// Every generation the reference model defines, oldest first.
pub const GENERATIONS: [Generation; 4] = [
    Generation::M1,
    Generation::M2,
    Generation::M3,
    Generation::M4,
];

/// A caller's MXCSR no AMX result may depend on and every path must leave
/// as it found it: flush to zero, denormals are zero, round toward zero,
/// every exception masked (as `x86_64::amx`'s kernel tests use).
pub const HOSTILE_MXCSR: u32 = 0xffc0;

/// Register operations reach no memory.
pub struct NoMemory;

impl AmxMemory for NoMemory {
    fn read(&mut self, _: u64, _: &mut [u8]) -> Result<(), MemoryFault> {
        panic!("arithmetic must not read guest memory")
    }
    fn write(&mut self, _: u64, _: &[u8]) -> Result<(), MemoryFault> {
        panic!("arithmetic must not write guest memory")
    }
}

/// xorshift64: the operands and registers of one seed are the same on
/// every host.
pub struct Random(pub u64);

impl Random {
    pub fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
}

/// One host path: what `Native::select` gives a target with only a tier's
/// own feature, and F16C or not.
#[derive(Debug, Clone, Copy)]
pub struct Paths {
    pub native: Native,
    pub f16c: bool,
}

impl Paths {
    pub fn tier(&self) -> Tier {
        self.native.tier()
    }
}

/// The features [`STELA_AMX_OPTIONAL`] can name.
const FEATURES: [&str; 3] = ["fma", "avx512f", "f16c"];

/// The variable that marks tiers a host may lack.
pub const STELA_AMX_OPTIONAL: &str = "STELA_AMX_OPTIONAL";

/// Every path the tests cover, on `host`: the model, FMA3 and AVX-512F
/// tiers, each without and with F16C (a lower tier's instructions all exist
/// on a higher host, so each runs where its features do). A path needing a
/// feature `host` lacks is an error unless `optional` (a comma-separated
/// [`STELA_AMX_OPTIONAL`] value) names that feature; then it is left out
/// with a note. Gives the paths and the notes.
pub fn paths_on(host: TargetCpu, optional: &str) -> Result<(Vec<Paths>, Vec<String>), String> {
    let optional: Vec<&str> = optional
        .split(',')
        .map(str::trim)
        .filter(|name| !name.is_empty())
        .collect();
    if let Some(name) = optional.iter().find(|name| !FEATURES.contains(name)) {
        return Err(format!(
            "{STELA_AMX_OPTIONAL} names {name:?}; it takes {}",
            FEATURES.join(", ")
        ));
    }
    let mut paths = Vec::new();
    let mut notes = Vec::new();
    for (tier, base, feature) in [
        (Tier::Model, TargetCpu::Baseline, None),
        (Tier::Fma, TargetCpu::Baseline.with_fma(), Some("fma")),
        (
            Tier::Avx512,
            TargetCpu::Baseline.with_avx512f(),
            Some("avx512f"),
        ),
    ] {
        for f16c in [false, true] {
            let target = if f16c { base.with_f16c() } else { base };
            let lacking: Vec<&str> = feature
                .filter(|_| !host.supports(base))
                .into_iter()
                .chain((f16c && !host.supports(TargetCpu::F16c)).then_some("f16c"))
                .collect();
            if lacking.is_empty() {
                let native = Native::select(target);
                assert_eq!(native.tier(), tier, "the tier {target:?} selects");
                paths.push(Paths { native, f16c });
            } else if let Some(missing) = lacking.iter().find(|name| !optional.contains(name)) {
                return Err(format!(
                    "the host lacks {missing}, which the AMX native tests cover: run them on a \
                     host with it, or name it in {STELA_AMX_OPTIONAL} to leave its paths out"
                ));
            } else {
                notes.push(format!(
                    "{} f16c={f16c}: the host lacks {}",
                    tier.name(),
                    lacking.join("+")
                ));
            }
        }
    }
    Ok((paths, notes))
}

/// [`paths_on`] this host with [`STELA_AMX_OPTIONAL`], noting each path it
/// leaves out; a lacking tier not named there fails the test.
pub fn tiers() -> Vec<Paths> {
    let optional = std::env::var(STELA_AMX_OPTIONAL).unwrap_or_default();
    match paths_on(TargetCpu::detect(), &optional) {
        Ok((paths, notes)) => {
            for note in notes {
                eprintln!("AMX native path left out ({STELA_AMX_OPTIONAL}): {note}");
            }
            paths
        }
        Err(error) => panic!("{error}"),
    }
}

pub fn mxcsr() -> u32 {
    let mut value = 0u32;
    // SAFETY: STMXCSR stores four bytes to a live local.
    unsafe {
        asm!(
            "stmxcsr [{value}]",
            value = in(reg) &raw mut value,
            options(nostack, preserves_flags),
        );
    }
    value
}

/// Load `value`, an MXCSR with its reserved bits clear.
pub fn set_mxcsr(value: u32) {
    // SAFETY: LDMXCSR of four live bytes; the callers pass MXCSR values
    // with no reserved bit set.
    unsafe {
        asm!(
            "ldmxcsr [{value}]",
            value = in(reg) &raw const value,
            options(nostack, preserves_flags, readonly),
        );
    }
}

/// `run` with [`HOSTILE_MXCSR`] in MXCSR: what it gives, and the MXCSR it
/// left. The thread's own MXCSR is back after.
pub fn hostile<T>(run: impl FnOnce() -> T) -> (T, u32) {
    let own = mxcsr();
    set_mxcsr(HOSTILE_MXCSR);
    let result = run();
    let left = mxcsr();
    set_mxcsr(own);
    (result, left)
}

/// What a path does with one operand: [`native`], or a stand-in that tests
/// the harness itself.
pub type Run = dyn Fn(&Paths, Generation, &mut AmxRegisters, AmxOperation, u64) -> bool;

/// The path's own `Native::execute`.
pub fn native(
    paths: &Paths,
    generation: Generation,
    registers: &mut AmxRegisters,
    operation: AmxOperation,
    operand: u64,
) -> bool {
    paths
        .native
        .execute(generation, registers, operation, operand)
}

/// Where byte `offset` of the register file is.
fn place(offset: usize) -> String {
    let (file, start) = if offset < Y_OFFSET {
        ("x", 0)
    } else if offset < Z_OFFSET {
        ("y", Y_OFFSET)
    } else {
        ("z", Z_OFFSET)
    };
    let within = offset - start;
    format!("{file}[{}][{}]", within / ROW_BYTES, within % ROW_BYTES)
}

fn first_difference(a: &AmxRegisters, b: &AmxRegisters) -> Option<usize> {
    a.as_bytes()
        .iter()
        .zip(b.as_bytes())
        .position(|(a, b)| a != b)
}

/// One operand on one path, against the model on a copy of `initial`:
/// `run` under [`HOSTILE_MXCSR`] must leave MXCSR unchanged and, where it
/// completes the operand, give the model's registers, where it refuses,
/// leave them as they were. Whether it completed the operand.
pub fn check(
    run: &Run,
    paths: &Paths,
    generation: Generation,
    initial: &AmxRegisters,
    operation: AmxOperation,
    operand: u64,
) -> bool {
    let mut expected = initial.clone();
    model::execute_for(generation, &mut expected, operation, operand, &mut NoMemory)
        .expect("register operations complete");
    let mut actual = initial.clone();
    let (covered, left) = hostile(|| run(paths, generation, &mut actual, operation, operand));
    let at = || {
        format!(
            "{paths:?} {generation:?} {} operand {operand:#018x}",
            operation.mnemonic()
        )
    };
    assert_eq!(
        left,
        HOSTILE_MXCSR,
        "{}: the caller's MXCSR {HOSTILE_MXCSR:#x} came back as {left:#x}",
        at()
    );
    if covered {
        if let Some(offset) = first_difference(&actual, &expected) {
            panic!(
                "{}: native differs from the model first at {}",
                at(),
                place(offset)
            );
        }
    } else if let Some(offset) = first_difference(&actual, initial) {
        panic!(
            "{}: a refused operand changed the registers, first at {}",
            at(),
            place(offset)
        );
    }
    covered
}

/// A named form of a case: its operands, and the least number of them a
/// path must complete natively in each generation the case runs.
pub struct Form {
    pub name: &'static str,
    pub of: fn(AmxOperation, u64) -> bool,
    pub floor: fn(&Paths) -> usize,
}

/// One differential test.
pub struct Case {
    pub name: &'static str,
    pub seed: u64,
    pub operations: &'static [AmxOperation],
    pub generations: &'static [Generation],
    /// Operands per path, generation and operation.
    pub iterations: usize,
    pub fill: fn(&mut Random) -> AmxRegisters,
    pub operand: fn(&mut Random) -> u64,
    /// The paths the case runs on.
    pub runs_on: fn(&Paths) -> bool,
    /// Where a path may leave an operand to the model: every refusal
    /// outside it fails the case.
    pub may_refuse: fn(&Paths, AmxOperation, Generation, u64) -> bool,
    pub forms: &'static [Form],
}

/// Run `case` through `run` on each path of [`tiers`] it runs on: for
/// every generation, operation and iteration, fresh registers from `fill`
/// and an operand, then [`check`]. A refusal must be one `may_refuse`
/// admits, and each form must have run natively at least its floor's
/// number of times per path and generation, so that neither a wrong lane,
/// a changed refusal, a leaked MXCSR nor a silent refusal passes. One
/// generator runs through the whole case, in that order.
pub fn differential(case: &Case, run: &Run) {
    let mut random = Random(case.seed);
    let mut ran = 0;
    for paths in tiers().iter().filter(|paths| (case.runs_on)(paths)) {
        ran += 1;
        for &generation in case.generations {
            let mut covered = vec![0usize; case.forms.len()];
            for &operation in case.operations {
                for _ in 0..case.iterations {
                    let initial = (case.fill)(&mut random);
                    let operand = (case.operand)(&mut random);
                    if check(run, paths, generation, &initial, operation, operand) {
                        for (count, form) in covered.iter_mut().zip(case.forms) {
                            *count += usize::from((form.of)(operation, operand));
                        }
                    } else {
                        assert!(
                            (case.may_refuse)(paths, operation, generation, operand),
                            "{}: {paths:?} {generation:?} {} refused operand {operand:#018x}, \
                             which it covers",
                            case.name,
                            operation.mnemonic()
                        );
                    }
                }
            }
            let counts: Vec<String> = case
                .forms
                .iter()
                .zip(&covered)
                .map(|(form, count)| format!("{} {count}", form.name))
                .collect();
            eprintln!(
                "{}: {paths:?} {generation:?} ran natively: {}",
                case.name,
                counts.join(", ")
            );
            for (form, &count) in case.forms.iter().zip(&covered) {
                let floor = (form.floor)(paths);
                assert!(
                    count >= floor,
                    "{}: {paths:?} {generation:?} ran {} natively {count} times, under its \
                     floor of {floor}",
                    case.name,
                    form.name
                );
            }
        }
    }
    assert!(ran > 0, "{}: no path ran", case.name);
}

/// Registers of special f16 values in the low half of each 32-bit lane,
/// and special f32 and f64 values, between random bits.
pub fn fill(random: &mut Random) -> AmxRegisters {
    const SINGLES: [u32; 12] = [
        0,
        0x8000_0000,
        0x7f80_0000,
        0xff80_0000,
        0x7fc0_0000,
        0x7f80_0001,
        0x0000_0001,
        0x807f_ffff,
        0x3f80_0000,
        0x7f7f_ffff,
        0x0080_0000,
        0xffc1_2345,
    ];
    const DOUBLES: [u64; 10] = [
        0,
        1 << 63,
        0x7ff0_0000_0000_0000,
        0xfff0_0000_0000_0000,
        0x7ff8_0000_0000_0000,
        0x7ff0_0000_0000_0001,
        1,
        0x3ff0_0000_0000_0000,
        0x7fef_ffff_ffff_ffff,
        0x0010_0000_0000_0000,
    ];
    const HALVES: [u16; 9] = [
        0, 0x8000, 0x7c00, 0xfc00, 0x7c01, 0x7e00, 0x0001, 0x3c00, 0x7bff,
    ];
    let mut registers = AmxRegisters::zeroed();
    for lane in registers.as_bytes_mut().as_chunks_mut::<8>().0 {
        let value = random.next();
        *lane = match value % 8 {
            0 => {
                let low = SINGLES[(value >> 8) as usize % SINGLES.len()];
                let high = SINGLES[(value >> 16) as usize % SINGLES.len()];
                u64::from(low) | u64::from(high) << 32
            }
            1 => DOUBLES[(value >> 8) as usize % DOUBLES.len()],
            2 => {
                let half = |shift: u32| u64::from(HALVES[(value >> shift) as usize % HALVES.len()]);
                half(8) | half(16) << 32 | (value & 0xffff_0000_ffff_0000)
            }
            // Small exponents make cancellation, denormal and tie results
            // likely.
            3 => value & 0x83ff_ffff_83ff_ffff | 0x3c00_0000_3c00_0000,
            _ => random.next(),
        }
        .to_le_bytes();
    }
    registers
}

/// Fused-operation operands over every field they read, with modes 0/1
/// favoured and the lane-enable fields often all on.
pub fn operand(random: &mut Random) -> u64 {
    let mut operand = random.next();
    let choice = random.next();
    operand &= !(7 << 27);
    let mode = match choice % 8 {
        0..=2 => 0,
        3 | 4 => 1,
        _ => 2 + (choice >> 8) % 6,
    };
    operand |= mode << 27;
    if choice & 0x30 != 0 {
        operand &= !(0x7f << 41);
    }
    if choice & 0xc0 != 0 {
        operand &= !(0x7f << 32);
    }
    operand
}

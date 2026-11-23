//! Tier 2 (`--tier2`, `stela_engine::machine::engine::tier2`) ends where
//! tier 1 and the reference do: registers, flags, memory, the instructions
//! retired and any data abort's syndrome, address and return address.
//!
//! The reference is the engine's own lowest tier: every block alone, linked,
//! with neither regions nor self-loops (the oracle `regions.rs` and
//! `self_loops.rs` use). Tier 1 is the engine's default. Tier 2 promotes
//! every block the engine enters, so the measured run (after a warm-up run
//! that makes them hot) executes tier-2 superblocks wherever they form.
//!
//! The programs are random: integer arithmetic and logic on the pinned and
//! the frame-homed registers, flag setters and every kind of flag reader
//! (B.cond, CSEL, CCMP, ADC, MRS NZCV), forward branches of every form and
//! counted loops back, loads and stores that alias through a fixed and a
//! moving base, pairs, LSE atomics, exclusives, acquire and release, and
//! System instructions (barriers, thread registers, MSR NZCV, DC ZVA, SVC).
//! Their bases sit astride a page boundary, the page on one side made to
//! fault, so accesses fault at every position of a superblock; their code
//! may straddle a page boundary too.
#![cfg(all(
    feature = "test-support",
    feature = "std",
    target_arch = "x86_64",
    target_os = "linux"
))]
use anyhow::{Result, ensure};
use stela_compiler::{GuestState, TargetCpu};
use stela_engine::machine::{Stop, engine::tier2::Tier2Options, system::Register};
#[path = "support/store_host.rs"]
mod store_host;
use store_host::{Mapping, machine};

const PAGE: u64 = 4096;
const BYTES: u64 = 4 * PAGE;
const CSV3_SHIFT: u32 = 60;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, bound: u64) -> u64 {
        self.next() % bound
    }
    fn pick<T: Copy>(&mut self, items: &[T]) -> T {
        items[self.below(items.len() as u64) as usize]
    }
}

/// The data base, the moving pointer and the loop counter.
const BASE: u32 = 20;
const POINTER: u32 = 21;
const COUNTER: u32 = 9;
/// Registers the programs compute with: the pinned ones (X0, X1, X8, X16,
/// X19) and frame-homed ones.
const REGISTERS: [u32; 20] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 16, 17, 19, 22, 23, 29, 30,
];

/// Which tier runs a program.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tier {
    Reference,
    One,
    Two,
    /// Tier 2 with every entry the engine makes into a tier-2 block checked.
    Checked,
}

/// A program's instruction to be placed: a word, or a branch to the
/// instruction `to` places away.
#[derive(Clone, Copy)]
enum Word {
    Plain(u32),
    /// `form` with its offset field empty, and the target index.
    Branch(BranchForm, usize),
}

#[derive(Clone, Copy)]
enum BranchForm {
    Cond(u32),
    Cbz(u32),
    Tbz(u32),
    B,
}

fn place(words: &[Word]) -> Vec<u32> {
    words
        .iter()
        .enumerate()
        .map(|(at, word)| match *word {
            Word::Plain(word) => word,
            Word::Branch(form, to) => {
                let offset = to as i64 - at as i64;
                match form {
                    BranchForm::Cond(word) | BranchForm::Cbz(word) => {
                        word | (((offset as u32) & 0x7ffff) << 5)
                    }
                    BranchForm::Tbz(word) => word | (((offset as u32) & 0x3fff) << 5),
                    BranchForm::B => 0x1400_0000 | ((offset as u32) & 0x03ff_ffff),
                }
            }
        })
        .collect()
}

/// One random instruction, at `at` of `length`.
fn instruction(rng: &mut Rng, at: usize, length: usize, compute: bool) -> Word {
    let r = |rng: &mut Rng| rng.pick(&REGISTERS);
    let any = |rng: &mut Rng| {
        if rng.below(6) == 0 {
            rng.pick(&[BASE, POINTER, COUNTER, 31])
        } else {
            rng.pick(&REGISTERS)
        }
    };
    let sf = |rng: &mut Rng| (rng.below(4) != 0) as u32;
    let cond = |rng: &mut Rng| rng.below(14) as u32;
    // A forward target: the next instruction up to the end.
    let forward = |rng: &mut Rng| at + 1 + rng.below((length - at) as u64) as usize;
    // A compute-only program takes no memory or System instruction.
    let kind = loop {
        let kind = rng.below(34);
        if !(compute && (16..=26).contains(&kind)) {
            break kind;
        }
    };
    match kind {
        // ADD/SUB (immediate), flags or not.
        0..=3 => {
            let (sf, op, s) = (sf(rng), rng.below(2) as u32, rng.below(2) as u32);
            let imm = rng.below(4096) as u32;
            Word::Plain(
                sf << 31 | op << 30 | s << 29 | 0x1100_0000 | imm << 10 | any(rng) << 5 | r(rng),
            )
        }
        // ADD/SUB (shifted register).
        4..=6 => {
            let (sf, op, s) = (sf(rng), rng.below(2) as u32, rng.below(2) as u32);
            let shift = rng.below(3) as u32;
            let amount = rng.below(if sf == 1 { 64 } else { 32 }) as u32;
            Word::Plain(
                sf << 31
                    | op << 30
                    | s << 29
                    | 0x0b00_0000
                    | shift << 22
                    | any(rng) << 16
                    | amount << 10
                    | any(rng) << 5
                    | r(rng),
            )
        }
        // Logical (shifted register), ANDS included.
        7..=8 => {
            let (sf, opc) = (sf(rng), rng.below(4) as u32);
            let amount = rng.below(if sf == 1 { 64 } else { 32 }) as u32;
            let n = rng.below(2) as u32;
            Word::Plain(
                sf << 31
                    | opc << 29
                    | 0x0a00_0000
                    | (rng.below(4) as u32) << 22
                    | n << 21
                    | any(rng) << 16
                    | amount << 10
                    | any(rng) << 5
                    | r(rng),
            )
        }
        // ADC/ADCS/SBC/SBCS: carry readers.
        9 => {
            let (sf, op, s) = (sf(rng), rng.below(2) as u32, rng.below(2) as u32);
            Word::Plain(
                sf << 31
                    | op << 30
                    | s << 29
                    | 0x1a00_0000
                    | any(rng) << 16
                    | any(rng) << 5
                    | r(rng),
            )
        }
        // CSEL/CSINC/CSINV/CSNEG.
        10..=11 => {
            let (sf, op, op2) = (sf(rng), rng.below(2) as u32, rng.below(2) as u32);
            Word::Plain(
                sf << 31
                    | op << 30
                    | 0x1a80_0000
                    | any(rng) << 16
                    | cond(rng) << 12
                    | op2 << 10
                    | any(rng) << 5
                    | r(rng),
            )
        }
        // CCMP/CCMN (immediate).
        12 => {
            let (sf, op) = (sf(rng), rng.below(2) as u32);
            Word::Plain(
                sf << 31
                    | op << 30
                    | 0x3a40_0800
                    | (rng.below(32) as u32) << 16
                    | cond(rng) << 12
                    | any(rng) << 5
                    | rng.below(16) as u32,
            )
        }
        // MADD/MSUB.
        13 => {
            let sf = sf(rng);
            let o0 = rng.below(2) as u32;
            Word::Plain(
                sf << 31
                    | 0x1b00_0000
                    | any(rng) << 16
                    | o0 << 15
                    | any(rng) << 10
                    | any(rng) << 5
                    | r(rng),
            )
        }
        // MOVZ/MOVK.
        14 => {
            let (sf, opc) = (sf(rng), rng.pick(&[2u32, 3]));
            let hw = rng.below(if sf == 1 { 4 } else { 2 }) as u32;
            Word::Plain(
                sf << 31
                    | opc << 29
                    | 0x1280_0000
                    | hw << 21
                    | (rng.below(65536) as u32) << 5
                    | r(rng),
            )
        }
        // UBFM/SBFM.
        15 => {
            let sf = sf(rng);
            let bits = if sf == 1 { 64 } else { 32 };
            let opc = rng.pick(&[0u32, 2]);
            Word::Plain(
                sf << 31
                    | opc << 29
                    | 0x1300_0000
                    | sf << 22
                    | (rng.below(bits) as u32) << 16
                    | (rng.below(bits) as u32) << 10
                    | any(rng) << 5
                    | r(rng),
            )
        }
        // Loads and stores through the fixed base: they alias each other.
        16..=19 => {
            let (load, size) = (rng.below(2) as u32, rng.pick(&[0u32, 2, 3]));
            let imm = rng.below(32) as u32;
            let rt = if load == 1 { r(rng) } else { any(rng) };
            Word::Plain(size << 30 | 0x3900_0000 | load << 22 | imm << 10 | BASE << 5 | rt)
        }
        // Post- and pre-indexed through the moving pointer.
        20..=21 => {
            let load = rng.below(2) as u32;
            let imm = ((rng.below(9) as i32 - 4) * 8) as u32 & 0x1ff;
            let index = rng.pick(&[1u32, 3]);
            let rt = if load == 1 { r(rng) } else { any(rng) };
            Word::Plain(0xf800_0000 | load << 22 | imm << 12 | index << 10 | POINTER << 5 | rt)
        }
        // LDP/STP (signed offset) through the base.
        22 => {
            let load = rng.below(2) as u32;
            let (rt, mut rt2) = (r(rng), r(rng));
            if rt2 == rt {
                rt2 = if rt == 2 { 3 } else { 2 };
            }
            Word::Plain(
                0xa900_0000
                    | load << 22
                    | (rng.below(16) as u32) << 15
                    | rt2 << 10
                    | BASE << 5
                    | rt,
            )
        }
        // LSE atomics: LDADD, LDCLR, LDEOR, LDSET, SWP.
        23 => {
            let op = rng.pick(&[0u32, 1, 2, 3, 8]);
            Word::Plain(0xf820_0000 | any(rng) << 16 | op << 12 | BASE << 5 | r(rng))
        }
        // LDAR/STLR.
        24 => match rng.below(2) {
            0 => Word::Plain(0xc8df_fc00 | BASE << 5 | r(rng)),
            _ => Word::Plain(0xc89f_fc00 | BASE << 5 | any(rng)),
        },
        // An exclusive pair at the base, or a lone CLREX.
        25 => match rng.below(3) {
            0 => Word::Plain(0xc85f_7c00 | BASE << 5 | r(rng)),
            1 => Word::Plain(0xc800_7c00 | rng.pick(&[2u32, 3, 4]) << 16 | BASE << 5 | any(rng)),
            _ => Word::Plain(0xd503_305f),
        },
        // System instructions.
        26 => Word::Plain(rng.pick(&[
            0xd503_3bbf,        // dmb ish
            0xd503_201f,        // nop
            0xd503_3fdf,        // isb
            0xd53b_d040 | 5,    // mrs x5, tpidr_el0
            0xd51b_d040 | 6,    // msr tpidr_el0, x6
            0xd53b_4200 | 7,    // mrs x7, nzcv
            0xd51b_4200 | 12,   // msr nzcv, x12
            0xd50b_7420 | BASE, // dc zva, x20
            0xd503_237f,        // pacibsp
            0xd503_23ff,        // autibsp
            0xd503_20ff,        // xpaclri
            0xd400_0001,        // svc #0
        ])),
        // B.cond.
        27..=29 => Word::Branch(BranchForm::Cond(0x5400_0000 | cond(rng)), forward(rng)),
        // CBZ/CBNZ.
        30 => {
            let (sf, op) = (sf(rng), rng.below(2) as u32);
            Word::Branch(
                BranchForm::Cbz(sf << 31 | 0x3400_0000 | op << 24 | any(rng)),
                forward(rng),
            )
        }
        // TBZ/TBNZ.
        31 => {
            let bit = rng.below(64) as u32;
            let op = rng.below(2) as u32;
            Word::Branch(
                BranchForm::Tbz(
                    (bit >> 5) << 31 | 0x3600_0000 | op << 24 | (bit & 31) << 19 | any(rng),
                ),
                forward(rng),
            )
        }
        // B.
        _ => Word::Branch(BranchForm::B, forward(rng)),
    }
}

/// A random program: its words, and where a counted loop starts, if any.
fn program(rng: &mut Rng) -> Vec<u32> {
    let length = 4 + rng.below(36) as usize;
    let compute = rng.below(3) == 0;
    let mut words: Vec<Word> = Vec::with_capacity(length + 8);
    while words.len() < length {
        let at = words.len();
        // A flag idiom in a third of the places: a setter, instructions
        // that neither set nor read NZCV, then a reader.
        if rng.below(3) == 0 && at + 4 < length {
            let r = |rng: &mut Rng| rng.pick(&REGISTERS);
            let setter = match rng.below(3) {
                0 => 0xeb00_001f | r(rng) << 16 | r(rng) << 5, // cmp xn,xm
                1 => 0xb100_0000 | (rng.below(4096) as u32) << 10 | r(rng) << 5 | r(rng), // adds imm
                _ => 0xea00_0000 | r(rng) << 16 | r(rng) << 5 | r(rng),                   // ands
            };
            words.push(Word::Plain(setter));
            for _ in 0..rng.below(3) {
                // add xd,xn,#imm: no flags.
                words.push(Word::Plain(
                    0x9100_0000 | (rng.below(4096) as u32) << 10 | r(rng) << 5 | r(rng),
                ));
            }
            let at = words.len();
            let reader = match rng.below(3) {
                0 => Word::Branch(
                    BranchForm::Cond(0x5400_0000 | rng.below(14) as u32),
                    at + 1 + rng.below((length.max(at + 1) - at) as u64) as usize,
                ),
                1 => Word::Plain(
                    0x9a80_0000
                        | r(rng) << 16
                        | (rng.below(14) as u32) << 12
                        | r(rng) << 5
                        | r(rng),
                ),
                _ => Word::Plain(0x9a00_0000 | r(rng) << 16 | r(rng) << 5 | r(rng)), // adc
            };
            words.push(reader);
            continue;
        }
        words.push(instruction(rng, at, length, compute));
    }
    let length = words.len();
    // Branches past the end land on its last word.
    for word in &mut words {
        if let Word::Branch(_, to) = word {
            *to = (*to).min(length);
        }
    }
    // A counted loop back over part of the program, in a third of them.
    if rng.below(3) == 0 {
        let start = rng.below(length as u64) as usize;
        let at = words.len();
        words.push(Word::Plain(0xf100_0400 | COUNTER << 5 | COUNTER)); // subs x9,x9,#1
        words.push(Word::Branch(BranchForm::Cond(0x5400_0001), start)); // b.ne
        let _ = at;
    }
    place(&words)
}

/// What a run ends in.
#[derive(Debug, PartialEq)]
struct Outcome {
    stop: Stop,
    pc: u64,
    state: GuestState,
    retired: u64,
    memory: Vec<u8>,
    abort: Option<(u64, u64, u64)>,
}

struct Case<'a> {
    words: &'a [u32],
    /// Where the program starts in its code page, past a B over padding.
    offset: u64,
    initial: &'a GuestState,
    contents: &'a [u8],
    protected: Option<u64>,
    masked: bool,
}

/// The tier-2 counts of the measured run.
#[derive(Default)]
struct Ran {
    installed: u64,
    exits: u64,
    checks: u64,
    check_mismatches: u64,
    check_skipped: u64,
}

fn run(code: &Mapping, data: &Mapping, tier: Tier, case: &Case) -> Result<(Outcome, Ran)> {
    // A B from the code's start over NOPs to the program at `offset`.
    let mut image = vec![0xd503_201f_u32; (case.offset / 4) as usize];
    if case.offset != 0 {
        image[0] = 0x1400_0000 | (case.offset / 4) as u32;
    }
    image.extend_from_slice(case.words);
    let mut machine = machine(code, data, &image, TargetCpu::detect())?;
    match tier {
        Tier::Reference => {
            machine.set_regions(false);
            machine.set_self_loops(false);
        }
        Tier::One => {}
        Tier::Two => machine.set_tier2(Some(Tier2Options {
            check: false,
            threshold: 1,
            check_every: 1,
        })),
        Tier::Checked => machine.set_tier2(Some(Tier2Options {
            check: true,
            threshold: 1,
            check_every: 1,
        })),
    }
    let pfr0 = machine.cpu.registers.get(Register::ID_AA64PFR0_EL1, 0);
    let pfr0 = pfr0 & !(0xf << CSV3_SHIFT) | u64::from(case.masked) << CSV3_SHIFT;
    machine
        .cpu
        .registers
        .set_raw(Register::ID_AA64PFR0_EL1, 0, pfr0);
    let pstate = machine.cpu.pstate;
    let vector = machine.host.lower_synchronous();
    // A warm-up run makes the program's blocks hot; then the measured one.
    let mut outcome = None;
    for measured in [false, true] {
        ensure!(data.write(data.0, case.contents), "write data");
        for register in [Register::ESR_EL1, Register::FAR_EL1, Register::ELR_EL1] {
            machine.cpu.registers.set_raw(register, 0, 0);
        }
        machine.cpu.pstate = pstate;
        machine.cpu.monitor = None;
        if measured && let Some(page) = case.protected {
            data.protect(page as usize * PAGE as usize, PAGE as usize, false)?;
        }
        machine.cpu.state = case.initial.clone();
        machine.cpu.pc = code.0;
        let retired = machine.cpu.retired;
        let before = machine.statistics.tier2;
        let stop = machine.run(machine.statistics.blocks + 20_000);
        if measured && let Some(page) = case.protected {
            data.protect(page as usize * PAGE as usize, PAGE as usize, true)?;
        }
        if !measured {
            continue;
        }
        let (stop, pc) = match stop == Stop::Breakpoint(vector) {
            true => (Stop::Breakpoint(0), machine.cpu.pc.wrapping_sub(vector)),
            false => (stop, machine.cpu.pc),
        };
        let abort = (stop == Stop::Breakpoint(0)).then(|| {
            (
                machine.cpu.registers.get(Register::ESR_EL1, 0),
                machine.cpu.registers.get(Register::FAR_EL1, 0),
                machine.cpu.registers.get(Register::ELR_EL1, 0),
            )
        });
        let after = machine.statistics.tier2;
        outcome = Some((
            Outcome {
                stop,
                pc,
                state: machine.cpu.state.clone(),
                retired: machine.cpu.retired - retired,
                memory: data.read(data.0, BYTES as usize).expect("readable data"),
                abort,
            },
            Ran {
                installed: after.installed,
                exits: after.exits - before.exits,
                checks: after.checks,
                check_mismatches: after.check_mismatches,
                check_skipped: after.check_skipped,
            },
        ));
    }
    Ok(outcome.expect("a measured run"))
}

/// One random case's start.
fn start(rng: &mut Rng, data: &Mapping) -> (GuestState, Vec<u8>, Option<u64>) {
    let mut initial = GuestState {
        x: std::array::from_fn(|_| rng.next()),
        v: std::array::from_fn(|_| [rng.next(), rng.next()]),
        nzcv: (rng.next() as u32) & 0xf000_0000,
        ..GuestState::default()
    };
    // The base within a few words of a page boundary, 8-byte aligned, so
    // its accesses reach both pages.
    let boundary = 1 + rng.below(3);
    let base = data.0 + boundary * PAGE - 8 * rng.below(40);
    initial.x[BASE as usize] = base;
    initial.x[POINTER as usize] = base - 8 * rng.below(8);
    initial.x[COUNTER as usize] = 1 + rng.below(6);
    let contents: Vec<u8> = (0..BYTES).map(|_| rng.next() as u8).collect();
    // Two cases in three fault: the page below or above the boundary.
    let protected = match rng.below(3) {
        0 => None,
        1 => Some(boundary - 1),
        _ => Some(boundary),
    };
    (initial, contents, protected)
}

#[test]
fn tier2_ends_where_tier1_and_the_reference_do() -> Result<()> {
    let data = Mapping::new()?;
    let code = Mapping::new()?;
    let mut rng = Rng(0x7132_2a5e_0bad_f00d);
    let (mut cases, mut aborts, mut installed, mut ran_tier2) = (0, 0, 0, 0);
    let mut faulting = std::collections::BTreeSet::new();
    // Mismatches against the reference, by tier, and the first of each.
    let mut mismatches = [0usize; 3];
    let mut firsts: [Option<String>; 3] = [None, None, None];
    let (mut checks, mut check_mismatches, mut check_skipped) = (0, 0, 0);
    for program_index in 0..500 {
        let words = program(&mut rng);
        // Some programs straddle the code page's end.
        let offset = match program_index % 4 {
            0 => PAGE - 4 * (1 + rng.below(words.len() as u64)),
            _ => 4 * rng.below(64),
        };
        for case in 0..2 {
            let (initial, contents, protected) = start(&mut rng, &data);
            let masked = case % 2 == 1;
            let start = Case {
                words: &words,
                offset,
                initial: &initial,
                contents: &contents,
                protected,
                masked,
            };
            let (reference, _) = run(&code, &data, Tier::Reference, &start)?;
            let (one, _) = run(&code, &data, Tier::One, &start)?;
            let (two, ran) = run(&code, &data, Tier::Two, &start)?;
            let (checked, check) = run(&code, &data, Tier::Checked, &start)?;
            checks += check.checks;
            check_mismatches += check.check_mismatches;
            check_skipped += check.check_skipped;
            let what = format!(
                "program {program_index} {words:08x?} offset {offset:#x} case {case} masked {masked} protected {protected:?}"
            );
            for (tier, outcome) in [&one, &two, &checked].into_iter().enumerate() {
                let end = |outcome: &Outcome| {
                    (
                        outcome.stop.clone(),
                        outcome.pc,
                        outcome.retired,
                        outcome.abort,
                    )
                };
                let differs = if end(outcome) != end(&reference) {
                    Some(format!(
                        "end {:x?} against {:x?}",
                        end(outcome),
                        end(&reference)
                    ))
                } else if outcome.state != reference.state {
                    Some(format!(
                        "state {:x?} against {:x?}",
                        outcome.state, reference.state
                    ))
                } else if outcome.memory != reference.memory {
                    Some("memory".to_owned())
                } else {
                    None
                };
                if let Some(differs) = differs {
                    mismatches[tier] += 1;
                    firsts[tier].get_or_insert_with(|| format!("{what}: {differs}"));
                }
            }
            cases += 1;
            if let Some((_, far, elr)) = reference.abort
                && far != 0
            {
                aborts += 1;
                faulting.insert(elr.wrapping_sub(code.0 + offset) / 4);
            }
            installed += usize::from(ran.installed > 0);
            ran_tier2 += usize::from(ran.exits > 0);
        }
    }
    eprintln!(
        "tier 2: {cases} cases, {aborts} aborted at {} positions, {installed} installed, {ran_tier2} left tier-2 blocks; mismatches tier 1 {} tier 2 {} checked {}; checks {checks} check_mismatches {check_mismatches} skipped {check_skipped}",
        faulting.len(),
        mismatches[0],
        mismatches[1],
        mismatches[2]
    );
    assert!(
        mismatches == [0, 0, 0],
        "tier 1: {} mismatches, first {:?}; tier 2: {} mismatches, first {:?}; checked: {} mismatches, first {:?}",
        mismatches[0],
        firsts[0],
        mismatches[1],
        firsts[1],
        mismatches[2],
        firsts[2]
    );
    // The check compares many entries, and finds tier 2 right in each.
    assert!(checks > 200, "{checks} checks");
    assert_eq!(check_mismatches, 0, "check mismatches");
    // The cases reach tier 2, and fault at many positions in their code.
    assert!(
        installed > cases / 2,
        "{installed} of {cases} installed tier-2 blocks"
    );
    assert!(
        ran_tier2 > cases / 3,
        "{ran_tier2} of {cases} left tier-2 blocks"
    );
    assert!(aborts > cases / 5, "{aborts} of {cases} aborted");
    assert!(
        faulting.len() >= 30,
        "faults at {} positions",
        faulting.len()
    );
    Ok(())
}

/// A superblock the guest rewrites (STR, DC CVAU, IC IVAU, ISB) runs its new
/// body at its next call, in every tier: the IC IVAU takes the tier-2 block
/// away with its page's blocks, which are compiled again from the new words.
#[test]
fn a_rewritten_superblock_runs_its_new_body() -> Result<()> {
    let data = Mapping::new()?;
    let code = Mapping::new()?;
    // The caller calls the leaf x2 times, rewrites the leaf's word 17 once
    // and calls it x11 times more. The leaf at 16 is a superblock: a TBZ
    // and a B that land ahead inside it.
    let words: [u32; 22] = [
        0x9400_0010, //  0: bl 16
        0xf100_0442, //  1: subs x2,x2,#1
        0x54ff_ffc1, //  2: b.ne 0
        0x3500_0186, //  3: cbnz w6, 15
        0xb900_0149, //  4: str w9,[x10]
        0xd50b_7b2a, //  5: dc cvau, x10
        0xd503_3b9f, //  6: dsb ish
        0xd50b_752a, //  7: ic ivau, x10
        0xd503_3b9f, //  8: dsb ish
        0xd503_3fdf, //  9: isb
        0xaa0b_03e2, // 10: mov x2,x11
        0x5280_0026, // 11: mov w6,#1
        0x17ff_fff4, // 12: b 0
        0xd503_201f, // 13: nop
        0xd503_201f, // 14: nop
        0x1400_0007, // 15: b 22 (the end)
        0x3600_0062, // 16: tbz w2,#0, 19
        0x9100_04a5, // 17: add x5,x5,#1 (rewritten to #2)
        0x1400_0002, // 18: b 20
        0x9100_04e7, // 19: add x7,x7,#1
        0x9100_058c, // 20: add x12,x12,#1
        0xd65f_03c0, // 21: ret
    ];
    // Calls with x2 odd step x5; x2 counts down from n to 1 in each phase.
    let odd = |n: u64| n.div_ceil(2);
    let mut rng = Rng(0x1357_9bdf_2468_ace0);
    for case in 0..8 {
        let (first, second) = (1 + rng.below(400), 1 + rng.below(400));
        let mut ends = vec![];
        for tier in [Tier::Reference, Tier::One, Tier::Two, Tier::Checked] {
            ensure!(
                code.write(code.0 + 17 * 4, &0x9100_04a5u32.to_le_bytes()),
                "the leaf's first body"
            );
            let mut machine = machine(&code, &data, &words, TargetCpu::detect())?;
            match tier {
                Tier::Reference => {
                    machine.set_regions(false);
                    machine.set_self_loops(false);
                }
                Tier::One => {}
                Tier::Two | Tier::Checked => machine.set_tier2(Some(Tier2Options {
                    check: tier == Tier::Checked,
                    threshold: 1,
                    check_every: 1,
                })),
            }
            // SCTLR_EL1.UCI: EL0 cleans and invalidates by VA.
            let sctlr = machine.cpu.registers.get(Register::SCTLR_EL1, 0);
            machine
                .cpu
                .registers
                .set_raw(Register::SCTLR_EL1, 0, sctlr | 1 << 26);
            let mut state = GuestState::default();
            state.x[2] = first;
            state.x[9] = 0x9100_08a5; // add x5,x5,#2
            state.x[10] = code.0 + 17 * 4;
            state.x[11] = second;
            machine.cpu.state = state;
            machine.cpu.pc = code.0;
            let retired = machine.cpu.retired;
            let stop = machine.run(machine.statistics.blocks + 200_000);
            let state = machine.cpu.state.clone();
            let what = format!("{tier:?} case {case} ({first}, {second})");
            ensure!(
                (state.x[12], state.x[5], state.x[7])
                    == (
                        first + second,
                        odd(first) + 2 * odd(second),
                        first - odd(first) + second - odd(second)
                    ),
                "{what}: calls {} x5 {} x7 {}",
                state.x[12],
                state.x[5],
                state.x[7]
            );
            if matches!(tier, Tier::Two | Tier::Checked) {
                ensure!(
                    machine.statistics.tier2.installed > 0,
                    "{what}: no tier-2 block"
                );
                ensure!(
                    machine.statistics.code_invalidations > 0,
                    "{what}: no invalidation"
                );
            }
            ends.push((stop, machine.cpu.retired - retired, state, machine.cpu.pc));
        }
        for (tier, end) in ends.iter().enumerate().skip(1) {
            ensure!(
                *end == ends[0],
                "tier {tier} case {case}: ends apart from the reference"
            );
        }
    }
    Ok(())
}

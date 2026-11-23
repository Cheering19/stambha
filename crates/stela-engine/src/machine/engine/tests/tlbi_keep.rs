//! `--tlbi-keep on` (`tlbi_keep.rs`): a TLBI of everything keeps the lookup
//! contexts, and a fetch runs only what its walk now gives. Each machine
//! here runs with `--tlbi-verify-check` too, whose count of mismatches must
//! stay zero: a mismatch is an entry verification kept that a full walk
//! contradicts. Expectations are a full walk of the machine's own tables at
//! the fetch ([`walk`]), never what the test thinks it changed.

use super::process_switching::whole_tlbi_acceptance;
use super::*;
use mmu::tlbi::{Everywhere, Reach};

type M = Machine<FakeHost, NoDevices>;

const RET: u32 = 0xd65f_03c0;
const NG: u64 = 1 << 11;
const MSR_TTBR0_EL1_X0: u32 = 0xd518_2000;
/// Frames 0 to 3, each a page with a RET.
const FRAMES: u64 = 4;

fn frame(n: u64) -> u64 {
    BASE + n * 0x1000
}

fn pc(page: u64) -> u64 {
    BASE + page * 0x1000
}

/// Space `space` selected by TTBR0_EL1 with `asid`.
fn select(machine: &mut M, space: usize, asid: u64) {
    let [l1, _, _] = SPACES_TABLES[space];
    machine.cpu.state.x[0] = (BASE + l1) | asid << 48;
    machine.system_instruction(MSR_TTBR0_EL1_X0).unwrap();
}

/// Space `space`'s level 3 entry of page `page`: `frame`, private (nG) unless
/// `global`.
fn map(machine: &mut M, space: usize, page: u64, frame: u64, global: bool) {
    let [_, _, l3] = SPACES_TABLES[space];
    let ng = if global { 0 } else { NG };
    write_entry(machine, l3 + page * 8, page_descriptor(frame, false) | ng);
}

fn all(machine: &mut M) {
    machine.tlb_maintained(Reach::All, Everywhere::All, None);
}

/// A TLBI by VA of the page at `va`, of every ASID.
fn page_tlbi(machine: &mut M, va: u64) {
    let reach = Reach::Containing(va);
    machine.tlb_maintained(reach, Everywhere::Addresses(reach), None);
}

/// What a fetch of `pc` runs: its page's block key, or None for a fault.
fn run(machine: &mut M, pc: u64) -> Option<u64> {
    match machine.fetch_block(pc, false) {
        Ok(Fetched::Block(_)) => Some(machine.current_key(pc).expect("taken")),
        Ok(Fetched::Exception(_)) => None,
        _ => panic!("{pc:#x}: neither a block nor an exception"),
    }
}

/// What a full walk of `pc` in the machine's state gives now, as the key of
/// the frame it translates to (`keys`, by frame), or None for a fault.
fn walk(machine: &mut M, keys: &[u64], pc: u64) -> Option<u64> {
    let controls = machine.cpu.controls(machine.cpu.regime());
    let requester = machine.cpu.requester();
    let walked = mmu::translate(
        &controls,
        pc,
        Access::Execute,
        requester,
        &mut RamTables(&mut machine.host),
    );
    match walked {
        Translated::Mapped(translation) if machine.host.fetch_permitted(pc) => {
            let frame = (translation.physical - BASE) / 0x1000;
            Some(*keys.get(frame as usize).expect("a frame with a key"))
        }
        _ => None,
    }
}

/// `run` and `walk` agree at `pc`: what it ran.
#[track_caller]
fn runs_its_walk(machine: &mut M, keys: &[u64], pc: u64) -> Option<u64> {
    let ran = run(machine, pc);
    assert_eq!(ran, walk(machine, keys, pc), "{pc:#x}");
    ran
}

/// A machine keeping its contexts with the check on, space 0 mapping page
/// `n` at frame `n` (private to ASID 1, or `global`), each page run once,
/// and each frame's key.
fn kept(global: bool) -> (M, Vec<u64>) {
    let mut words = vec![0u32; (FRAMES * 0x1000 / 4) as usize];
    for frame in 0..FRAMES as usize {
        words[frame * 0x400] = RET;
    }
    let mut machine = machine(&words);
    machine.set_tlbi_keep(true, true);
    assert_eq!(machine.tlbi_keep(), (true, true));
    paged_space(&mut machine, 0, &(0..FRAMES).map(frame).collect::<Vec<_>>());
    for page in 0..FRAMES {
        map(&mut machine, 0, page, frame(page), global);
    }
    select(&mut machine, 0, 1);
    let keys: Vec<u64> = (0..FRAMES)
        .map(|page| run(&mut machine, pc(page)).expect("mapped"))
        .collect();
    (machine, keys)
}

/// No kept entry was contradicted by its full walk.
#[track_caller]
fn no_mismatches(machine: &M) {
    let counts = machine.statistics.tlbi_keep;
    assert_eq!(counts.mismatches, 0, "{counts:#?}");
    assert_eq!(counts.checked, counts.kept, "{counts:#?}");
}

/// The acceptance set passes with the contexts kept, with and without the
/// check; the check finds nothing to drop; the keep did keep and drop.
#[test]
fn the_acceptance_set_passes_with_the_contexts_kept() {
    for check in [false, true] {
        let (failed, counts) = whole_tlbi_acceptance(&|machine| machine.set_tlbi_keep(true, check));
        assert!(failed.is_empty(), "check {check}: {failed:#?}");
        assert!(counts.tlbis >= 5, "{counts:#?}");
        assert!(counts.verified > 0 && counts.kept > 0, "{counts:#?}");
        assert!(
            counts.dropped >= 5,
            "every scenario drops a page: {counts:#?}"
        );
        assert_eq!(counts.mismatches, 0, "{counts:#?}");
        assert_eq!(counts.checked, if check { counts.kept } else { 0 });
    }
}

/// The mutation "keep without verifying": every scenario of the acceptance
/// set fails.
#[test]
fn the_acceptance_set_fails_where_kept_contexts_go_unverified() {
    let (failed, _) = whole_tlbi_acceptance(&|machine| {
        machine.set_tlbi_keep(true, false);
        machine.tlbi_keep.unverified = true;
    });
    for name in [
        "remapped page",
        "unmapped page",
        "execute-never page",
        "reused TTBR0 and ASID",
        "one VA in two spaces",
    ] {
        assert!(
            failed.iter().any(|failure| failure.starts_with(name)),
            "{name} passed unverified: {failed:#?}"
        );
    }
}

/// Without `--tlbi-keep on` nothing is kept: the contexts are made anew.
#[test]
fn the_keep_is_off_by_default() {
    let mut machine = machine(&[RET]);
    assert_eq!(machine.tlbi_keep(), (false, false));
    paged_space(&mut machine, 0, &[BASE]);
    run(&mut machine, BASE);
    let before = machine.chain.contexts();
    all(&mut machine);
    run(&mut machine, BASE);
    assert_ne!(machine.chain.contexts()[0], before[0]);
    assert_eq!(machine.statistics.tlbi_keep, TlbiKeepCounts::default());
}

/// ITLB drop epoch of `half` ([`chain::ITLB_DROPS`]).
fn drop_epoch(machine: &M, half: usize) -> u64 {
    let at =
        (machine.itlb.address() as i64 + i64::from(chain::ITLB_DROPS)) as u64 + 8 * half as u64;
    // SAFETY: the ITLB's header words precede its first slot, live while
    // the machine does, and only this thread writes them.
    unsafe { (at as *const u64).read() }
}

/// Between the TLBI and the verifying fetch nothing native can reach a kept
/// context: the chain runs in the void context, every stamp and jump cache
/// entry is void with its drop epoch, and neither the transition records
/// nor the TTBR0 switch table can take its number (`live_contexts`). The
/// fetch then verifies it and runs in it again, under its old number.
#[test]
fn native_code_reaches_no_kept_context_before_its_verification() {
    let (mut machine, keys) = kept(false);
    let before = machine.chain.contexts();
    let epochs = [0, 1].map(|half| drop_epoch(&machine, half));
    let switches = machine.mirrors.table_switch.generation;
    let el = machine.cpu.pstate.el;
    let variant = machine.current_variant(false, false);
    assert_eq!(machine.live_contexts(el, variant), Some(before));
    all(&mut machine);
    assert_eq!(machine.chain.contexts(), [contexts::VOID; 2]);
    for (half, &epoch) in epochs.iter().enumerate() {
        assert!(drop_epoch(&machine, half) > epoch, "half {half}");
    }
    assert_ne!(machine.mirrors.table_switch.generation, switches);
    assert_eq!(machine.live_contexts(el, variant), None);
    for page in 0..FRAMES {
        assert_eq!(machine.current_key(pc(page)), None);
    }
    assert_eq!(runs_its_walk(&mut machine, &keys, pc(1)), Some(keys[1]));
    assert_eq!(machine.chain.contexts(), before);
    assert_eq!(machine.live_contexts(el, variant), Some(before));
    // Every other page was kept without a walk.
    for page in [0, 2, 3] {
        assert_eq!(machine.current_key(pc(page)), Some(keys[page as usize]));
    }
    assert_eq!(machine.statistics.tlbi_keep.dropped, 0);
    assert_eq!(machine.statistics.tlbi_keep.kept, FRAMES);
    no_mismatches(&machine);
}

/// A descriptor changed and changed back before the TLBI holds its old
/// value: the entry is the walk's and stays. Changed, flushed and run, then
/// changed back and flushed, each fetch runs the walk of its time.
#[test]
fn a_descriptor_changed_and_back_runs_its_walk() {
    let (mut machine, keys) = kept(false);
    map(&mut machine, 0, 1, frame(2), false);
    map(&mut machine, 0, 1, frame(1), false);
    all(&mut machine);
    assert_eq!(runs_its_walk(&mut machine, &keys, pc(1)), Some(keys[1]));
    assert_eq!(machine.statistics.tlbi_keep.dropped, 0);
    map(&mut machine, 0, 1, frame(2), false);
    all(&mut machine);
    assert_eq!(runs_its_walk(&mut machine, &keys, pc(1)), Some(keys[2]));
    map(&mut machine, 0, 1, frame(1), false);
    all(&mut machine);
    assert_eq!(runs_its_walk(&mut machine, &keys, pc(1)), Some(keys[1]));
    for page in 0..FRAMES {
        runs_its_walk(&mut machine, &keys, pc(page));
    }
    assert_eq!(machine.statistics.tlbi_keep.dropped, 2);
    no_mismatches(&machine);
}

/// A level 3 table freed and its page reused for another table: the level 2
/// entry names a new table (other mappings), and the old page holds another
/// space's entries; or the level 2 entry stays and its page's contents are
/// another table's.
#[test]
fn a_table_page_freed_and_reused_runs_its_walk() {
    const NEW_TABLE: u64 = 0x9000;
    let [_, l2, l3] = SPACES_TABLES[0];
    for keep_level2 in [false, true] {
        let (mut machine, keys) = kept(false);
        let (table, other) = if keep_level2 {
            (l3, l3)
        } else {
            write_entry(&mut machine, l2, (BASE + NEW_TABLE) | 0b11);
            (NEW_TABLE, l3)
        };
        for page in 0..FRAMES {
            write_entry(
                &mut machine,
                table + page * 8,
                page_descriptor(frame(FRAMES - 1 - page), false) | NG,
            );
            if other != table {
                // The old page, another space's table now.
                write_entry(
                    &mut machine,
                    other + page * 8,
                    page_descriptor(frame((page + 1) % FRAMES), false) | NG,
                );
            }
        }
        all(&mut machine);
        for page in 0..FRAMES {
            let ran = runs_its_walk(&mut machine, &keys, pc(page));
            assert_eq!(ran, Some(keys[(FRAMES - 1 - page) as usize]));
        }
        no_mismatches(&machine);
    }
}

/// A 2 MiB block split into a level 3 table that maps one page elsewhere:
/// every page runs the new walk, and the block's entries, a leaf fill's
/// among them, go.
#[test]
fn a_block_split_into_a_table_runs_its_walk() {
    let mut words = vec![0u32; (FRAMES * 0x1000 / 4) as usize];
    for frame in 0..FRAMES as usize {
        words[frame * 0x400] = RET;
    }
    let mut machine = machine(&words);
    machine.set_tlbi_keep(true, true);
    paged_space(&mut machine, 0, &(0..FRAMES).map(frame).collect::<Vec<_>>());
    let [_, l2, l3] = SPACES_TABLES[0];
    for page in 0..FRAMES {
        map(&mut machine, 0, page, frame(page), false);
    }
    select(&mut machine, 0, 1);
    let keys: Vec<u64> = (0..FRAMES)
        .map(|page| run(&mut machine, pc(page)).expect("mapped"))
        .collect();
    // The block, of the same frames.
    write_entry(&mut machine, l2, BASE | 1 << 10 | 3 << 8 | NG | 0b01);
    all(&mut machine);
    for page in 0..FRAMES {
        assert_eq!(
            runs_its_walk(&mut machine, &keys, pc(page)),
            Some(keys[page as usize])
        );
    }
    // Split: page 0 moves to frame 2.
    write_entry(&mut machine, l2, (BASE + l3) | 0b11);
    map(&mut machine, 0, 0, frame(2), false);
    all(&mut machine);
    assert_eq!(runs_its_walk(&mut machine, &keys, pc(0)), Some(keys[2]));
    for page in 1..FRAMES {
        assert_eq!(
            runs_its_walk(&mut machine, &keys, pc(page)),
            Some(keys[page as usize])
        );
    }
    assert!(machine.statistics.tlbi_keep.dropped >= FRAMES);
    no_mismatches(&machine);
}

/// Only permissions change: UXN (no effect on an EL1 fetch), AP read-only
/// (none either), AP writable at EL0 (privileged execute-never) and PXN.
/// Each page runs what its walk gives, faults included.
#[test]
fn a_permission_change_runs_its_walk() {
    const AP_EL0: u64 = 1 << 6;
    const AP_READ_ONLY: u64 = 1 << 7;
    const UXN: u64 = 1 << 54;
    const PXN: u64 = 1 << 53;
    let (mut machine, keys) = kept(false);
    let [_, _, l3] = SPACES_TABLES[0];
    for (page, bits) in [(0, UXN), (1, AP_READ_ONLY), (2, AP_EL0), (3, PXN)] {
        write_entry(
            &mut machine,
            l3 + page * 8,
            page_descriptor(frame(page), false) | NG | bits,
        );
    }
    all(&mut machine);
    let ran: Vec<Option<u64>> = (0..FRAMES)
        .map(|page| runs_its_walk(&mut machine, &keys, pc(page)))
        .collect();
    assert_eq!(ran[0], Some(keys[0]));
    assert_eq!(ran[3], None, "PXN faults");
    assert_eq!(machine.statistics.tlbi_keep.dropped, FRAMES);
    no_mismatches(&machine);
}

/// The host's answer to `fetch_permitted` is in no descriptor: a page it
/// refuses after the TLBI faults, though its tables are unchanged.
#[test]
fn a_page_the_host_refuses_after_the_tlbi_faults() {
    let (mut machine, keys) = kept(false);
    machine.host.refused.push(pc(2));
    all(&mut machine);
    for page in 0..FRAMES {
        let ran = runs_its_walk(&mut machine, &keys, pc(page));
        assert_eq!(ran.is_none(), page == 2, "page {page}");
    }
    assert_eq!(machine.statistics.tlbi_keep.dropped, 1);
    no_mismatches(&machine);
}

/// A frame whose blocks take another variant (it needs the translated one)
/// under unchanged tables: the kept entry's block key is the old variant's,
/// which a fill would not make now, so it goes, and the fetch runs the new
/// variant's block.
#[test]
fn a_frame_whose_variant_changed_runs_its_new_variant() {
    let mut words = vec![0u32; (FRAMES * 0x1000 / 4) as usize];
    for frame in 0..FRAMES as usize {
        words[frame * 0x400] = RET;
    }
    let mut machine = machine(&words);
    machine.host.placement = mmu::placement::Placement::four_level(2).unwrap();
    machine.host.translation_helper = Some(0x1000);
    machine.derived_changed();
    machine.set_tlbi_keep(true, true);
    paged_space(&mut machine, 0, &(0..FRAMES).map(frame).collect::<Vec<_>>());
    for page in 0..FRAMES {
        map(&mut machine, 0, page, frame(page), false);
    }
    select(&mut machine, 0, 1);
    let keys: Vec<u64> = (0..FRAMES)
        .map(|page| run(&mut machine, pc(page)).expect("mapped"))
        .collect();
    // Only the variant input changes: no IC maintenance tells the ITLB.
    machine.translated_pages.insert(frame(1));
    all(&mut machine);
    let ran = run(&mut machine, pc(1)).expect("mapped");
    assert_ne!(ran, keys[1], "the old variant's key was kept");
    let Ok(Fetched::Block(block)) = machine.fetch_block(pc(1), false) else {
        panic!("a block");
    };
    assert!(machine.variants[machine.code.blocks[block].variant.0 as usize].translated);
    for page in [0, 2, 3] {
        assert_eq!(run(&mut machine, pc(page)), Some(keys[page as usize]));
    }
    assert_eq!(machine.statistics.tlbi_keep.dropped, 1);
    no_mismatches(&machine);
}

/// An ASID a new process takes again, with the same VA layout but tables of
/// its own (another TTBR0 base): its pages run its frames, whether they are
/// private (its own context) or global (the upper half's context holds the
/// lower half's global pages, which the TTBR0 check keeps apart).
#[test]
fn an_asid_reused_with_the_same_layout_runs_its_walk() {
    for global in [false, true] {
        let (mut machine, keys) = kept(global);
        paged_space(
            &mut machine,
            1,
            &(0..FRAMES)
                .map(|n| frame((n + 2) % FRAMES))
                .collect::<Vec<_>>(),
        );
        for page in 0..FRAMES {
            map(&mut machine, 1, page, frame((page + 2) % FRAMES), global);
        }
        select(&mut machine, 0, 1);
        // The first process exits; its ASID goes to the next, after a flush.
        all(&mut machine);
        select(&mut machine, 1, 1);
        for page in 0..FRAMES {
            assert_eq!(
                runs_its_walk(&mut machine, &keys, pc(page)),
                Some(keys[((page + 2) % FRAMES) as usize]),
                "global {global}, page {page}"
            );
        }
        no_mismatches(&machine);
    }
}

/// A stage-2 change under an unchanged stage 1: the EL1&0 regime with
/// HCR_EL2.VM maps the IPA space through a 1 GiB stage-2 block, then the
/// block is made execute-never, and a fetch faults.
#[test]
fn a_stage2_change_runs_its_walk() {
    // 4 KiB, 40-bit IPA from level 1 with two concatenated tables, 48-bit PS.
    const VTCR_4K: u64 = 24 | 1 << 6 | 5 << 16 | 1 << 31;
    const LEVEL1: u64 = 0x6000;
    const HCR_VM: u64 = 1;
    const XN: u64 = 1 << 54;
    let block = |xn: u64| BASE | 1 << 10 | 3 << 8 | 3 << 6 | 0xf << 2 | xn | 0b01;
    let mut words = vec![0u32; (FRAMES * 0x1000 / 4) as usize];
    for frame in 0..FRAMES as usize {
        words[frame * 0x400] = RET;
    }
    let mut machine = machine(&words);
    machine.set_tlbi_keep(true, true);
    paged_space(&mut machine, 0, &(0..FRAMES).map(frame).collect::<Vec<_>>());
    for page in 0..FRAMES {
        map(&mut machine, 0, page, frame(page), false);
    }
    let entry = LEVEL1 + ((BASE >> 30) & 0x3ff) * 8;
    write_entry(&mut machine, entry, block(0));
    let registers = &mut machine.cpu.registers;
    let hcr = registers.get(SystemRegister::HCR_EL2, 0);
    registers.set(SystemRegister::VTCR_EL2, 0, VTCR_4K);
    registers.set(SystemRegister::VTTBR_EL2, 0, BASE + LEVEL1);
    registers.set(SystemRegister::HCR_EL2, 0, hcr | HCR_VM);
    machine.translations_changed();
    select(&mut machine, 0, 1);
    assert!(machine.cpu.stage2(machine.cpu.regime()), "stage 2 applies");
    let keys: Vec<u64> = (0..FRAMES)
        .map(|page| run(&mut machine, pc(page)).expect("mapped"))
        .collect();
    // Stage 1 is unchanged: every entry stays.
    all(&mut machine);
    for page in 0..FRAMES {
        assert_eq!(
            runs_its_walk(&mut machine, &keys, pc(page)),
            Some(keys[page as usize])
        );
    }
    assert_eq!(machine.statistics.tlbi_keep.dropped, 0);
    assert!(machine.statistics.tlbi_keep.kept >= FRAMES);
    write_entry(&mut machine, entry, block(XN));
    all(&mut machine);
    for page in 0..FRAMES {
        assert_eq!(runs_its_walk(&mut machine, &keys, pc(page)), None);
    }
    no_mismatches(&machine);
}

/// A seeded xorshift.
struct Draws(u64);

impl Draws {
    fn next(&mut self, below: u64) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0 % below
    }
}

/// Two machines, one forgetting at a TLBI of everything and one keeping
/// with the check, take one random sequence: descriptor rewrites in either
/// space (another frame, private or global, unmapped, PXN), each followed by
/// a TLBI by VA or of everything; TLBIs of everything alone; address-space
/// switches with ASID reuse, flushed as XNU flushes (the global mappings of
/// the two spaces differ, or the ASID last served another space); and
/// fetches. Every fetch of both runs what its walk gives, and the two agree.
#[test]
fn kept_and_forgotten_contexts_run_alike_across_random_table_changes() {
    const STEPS: usize = 2500;
    let mut words = vec![0u32; (FRAMES * 0x1000 / 4) as usize];
    for frame in 0..FRAMES as usize {
        words[frame * 0x400] = RET;
    }
    let make = |keep: bool| {
        let mut machine = machine(&words);
        machine.set_tlbi_keep(keep, keep);
        for space in [1, 0] {
            paged_space(
                &mut machine,
                space,
                &(0..FRAMES).map(frame).collect::<Vec<_>>(),
            );
            for page in 0..FRAMES {
                map(&mut machine, space, page, frame(page), false);
            }
        }
        select(&mut machine, 0, 1);
        let keys: Vec<u64> = (0..FRAMES)
            .map(|page| run(&mut machine, pc(page)).expect("mapped"))
            .collect();
        (machine, keys)
    };
    let mut totals = TlbiKeepCounts::default();
    for seed in [0x9e37_79b9_7f4a_7c15u64, 7, 0x1234_5678, 0xdead_beef_cafe] {
        let (mut forgetting, forgetting_keys) = make(false);
        let (mut keeping, keeping_keys) = make(true);
        let mut draws = Draws(seed);
        // The model's descriptors, by space and page, to flush as XNU does.
        let mut descriptors = [core::array::from_fn::<u64, { FRAMES as usize }, _>(|page| {
            page_descriptor(frame(page as u64), false) | NG
        }); 2];
        let globals = |descriptors: &[[u64; FRAMES as usize]; 2], space: usize| {
            descriptors[space].map(|descriptor| {
                (descriptor & 1 != 0 && descriptor & NG == 0).then_some(descriptor)
            })
        };
        let mut space = 0usize;
        let mut owners = [0usize; 4];
        let mut fetches = 0;
        for step in 0..STEPS {
            let draw = draws.next(100);
            let mut apply = |action: &dyn Fn(&mut M)| {
                action(&mut forgetting);
                action(&mut keeping);
            };
            if draw < 30 {
                let (at, page) = (draws.next(2) as usize, draws.next(FRAMES));
                let target = frame(draws.next(FRAMES));
                let [_, _, l3] = SPACES_TABLES[at];
                let descriptor = match draws.next(5) {
                    0 => 0,
                    1 => page_descriptor(target, true) | NG,
                    2 => page_descriptor(target, false),
                    _ => page_descriptor(target, false) | NG,
                };
                descriptors[at][page as usize] = descriptor;
                apply(&|machine| write_entry(machine, l3 + page * 8, descriptor));
                if draws.next(2) == 0 {
                    apply(&|machine| page_tlbi(machine, pc(page)));
                } else {
                    apply(&|machine| all(machine));
                }
            } else if draw < 38 {
                apply(&|machine| all(machine));
            } else if draw < 55 {
                let (to, to_asid) = (draws.next(2) as usize, 1 + draws.next(3));
                if globals(&descriptors, to) != globals(&descriptors, space)
                    || owners[to_asid as usize] != to
                {
                    apply(&|machine| all(machine));
                }
                owners[to_asid as usize] = to;
                space = to;
                apply(&|machine| select(machine, to, to_asid));
            } else {
                let at = pc(draws.next(FRAMES));
                let forgot = run(&mut forgetting, at);
                let kept = run(&mut keeping, at);
                let walked = walk(&mut forgetting, &forgetting_keys, at);
                assert_eq!(
                    forgot, walked,
                    "seed {seed:#x} step {step}: forgetting at {at:#x}"
                );
                assert_eq!(
                    kept,
                    walk(&mut keeping, &keeping_keys, at),
                    "seed {seed:#x} step {step}: keeping at {at:#x}"
                );
                // Both machines number frames' blocks alike.
                assert_eq!(
                    forgot.map(|key| forgetting_keys.iter().position(|&held| held == key)),
                    kept.map(|key| keeping_keys.iter().position(|&held| held == key)),
                    "seed {seed:#x} step {step}: at {at:#x}"
                );
                fetches += 1;
            }
        }
        assert!(fetches > STEPS / 3);
        no_mismatches(&keeping);
        assert_eq!(forgetting.statistics.tlbi_keep, TlbiKeepCounts::default());
        totals.add(&keeping.statistics.tlbi_keep);
    }
    assert!(totals.tlbis > 400, "{totals:#?}");
    assert!(totals.kept > 100 && totals.dropped > 100, "{totals:#?}");
    assert!(totals.verified > 100, "{totals:#?}");
}

//! Tier 2 (`tier2.rs`) beside the warm cache's translation store and
//! `--tlbi-keep on` (`tlbi_keep.rs`): a superblock reaches no offer, and a
//! kept context reaches it only through its verified walk, its stamps and
//! jump cache entries gone with the TLBI's drop epochs.

use super::*;
use mmu::tlbi::{Everywhere, Reach};

type M = Machine<FakeHost, NoDevices>;

/// `tbz w2,#0,1f; add x5,x5,#1; b 2f; 1: add x7,x7,#1; 2: add x12,x12,#1;
/// ret`: tier 1 ends a block at the B, tier 2 makes one superblock of it,
/// the TBZ and the B landing ahead inside.
const SUPERBLOCK: [u32; 6] = [
    0x3600_0062,
    0x9100_04a5,
    0x1400_0002,
    0x9100_04e7,
    0x9100_058c,
    0xd65f_03c0,
];
const RET: u32 = 0xd65f_03c0;

/// Every block promoted at its first heat, checked never.
fn tier2_on(machine: &mut M) {
    machine.set_tier2(Some(tier2::Tier2Options {
        check: false,
        threshold: 1,
        check_every: 1,
    }));
}

/// The block a fetch of `pc` runs.
#[track_caller]
fn fetched(machine: &mut M, pc: u64) -> usize {
    match machine.fetch_block(pc, false) {
        Ok(Fetched::Block(index)) => index,
        _ => panic!("{pc:#x}: no block"),
    }
}

/// Make the block at `pc` hot, as a run entering it does, and promote it
/// after the run: the block a fetch of `pc` then runs.
#[track_caller]
fn promoted(machine: &mut M, pc: u64) -> usize {
    let tier1 = fetched(machine, pc);
    machine.tier2_heat(tier1, tier1, false);
    machine.tier2_after_run();
    fetched(machine, pc)
}

/// A TLBI of everything.
fn all(machine: &mut M) {
    machine.tlb_maintained(Reach::All, Everywhere::All, None);
}

/// ITLB drop epoch of `half` ([`chain::ITLB_DROPS`]).
fn drop_epoch(machine: &M, half: usize) -> u64 {
    let at =
        (machine.itlb.address() as i64 + i64::from(chain::ITLB_DROPS)) as u64 + 8 * half as u64;
    // SAFETY: the ITLB's header words precede its first slot, live while
    // the machine does, and only this thread writes them.
    unsafe { (at as *const u64).read() }
}

/// The store receives tier 1's offer of the block and nothing of the
/// superblock that takes its place, nor of its recompile after the
/// superblock goes: every offer holds tier 1's code.
#[test]
fn a_superblock_reaches_no_offer() {
    let mut machine = machine(&SUPERBLOCK);
    machine.host.store = Some(stored::tests::MemoryStore::default());
    tier2_on(&mut machine);
    let tier1 = fetched(&mut machine, BASE);
    assert!(!machine.code.blocks[tier1].tier2);
    let offers = |machine: &M| machine.host.store.as_ref().unwrap().offers.len();
    assert_eq!(offers(&machine), 1);
    let offered = machine.statistics.offered;
    let tier1_code = machine.host.store.as_ref().unwrap().offers[0]
        .emission
        .block
        .native
        .x86_64_code
        .clone();
    let tier2 = promoted(&mut machine, BASE);
    assert_eq!(machine.statistics.tier2.installed, 1);
    assert!(machine.code.blocks[tier2].tier2);
    assert_eq!(offers(&machine), 1, "the superblock is offered");
    assert_eq!(machine.statistics.offered, offered);
    // Its code is not tier 1's.
    assert!(machine.code.blocks[tier2].instructions > machine.code.blocks[tier1].instructions);
    // Taken away, the block is tier 1's again; whatever its recompile
    // offers is tier 1's code.
    machine.tier2_deopt(tier2);
    let again = fetched(&mut machine, BASE);
    assert!(!machine.code.blocks[again].tier2);
    let store = machine.host.store.as_ref().unwrap();
    assert!(
        store
            .offers
            .iter()
            .all(|offer| offer.emission.block.native.x86_64_code == tier1_code),
        "an offer holds other code than tier 1's"
    );
    // A deoptimized block is promoted no more.
    let installed = machine.statistics.tier2.installed;
    let settled = promoted(&mut machine, BASE);
    assert!(!machine.code.blocks[settled].tier2);
    assert_eq!(machine.statistics.tier2.installed, installed);
}

/// With `--tlbi-keep on` a TLBI of everything voids the chain's contexts
/// and grows both drop epochs, so no stamp or jump cache entry of the
/// superblock holds; the fetch then verifies the kept context and reaches
/// the same superblock, its frame unchanged. Remapped, the page walks to its
/// new frame's block; mapped back, to the superblock again; once the
/// superblock goes, a kept context reaches tier 1's block.
#[test]
fn a_kept_context_reaches_a_superblock_only_through_its_walk() {
    let mut words = vec![0u32; 2 * 0x400];
    words[..SUPERBLOCK.len()].copy_from_slice(&SUPERBLOCK);
    words[0x400] = RET;
    let mut machine = machine(&words);
    machine.set_tlbi_keep(true, true);
    tier2_on(&mut machine);
    let frames = [BASE, BASE + 0x1000];
    paged_space(&mut machine, 0, &frames);
    let tier1 = fetched(&mut machine, BASE);
    let tier2 = promoted(&mut machine, BASE);
    assert_ne!(tier1, tier2);
    assert!(machine.code.blocks[tier2].tier2);
    let key = machine.current_key(BASE).expect("taken");
    let epochs = [0, 1].map(|half| drop_epoch(&machine, half));
    all(&mut machine);
    assert_eq!(machine.chain.contexts(), [contexts::VOID; 2]);
    for (half, &epoch) in epochs.iter().enumerate() {
        assert!(drop_epoch(&machine, half) > epoch, "half {half}");
    }
    assert_eq!(machine.current_key(BASE), None);
    // Verified, the kept entry reaches the superblock.
    assert_eq!(fetched(&mut machine, BASE), tier2);
    assert_eq!(machine.current_key(BASE), Some(key));
    let counts = machine.statistics.tlbi_keep;
    assert!(counts.verified > 0 && counts.kept > 0, "{counts:#?}");
    assert_eq!(counts.dropped, 0, "{counts:#?}");
    // Remapped to the RET's frame: the walk's block.
    let [_, _, l3] = SPACES_TABLES[0];
    write_entry(&mut machine, l3, page_descriptor(frames[1], false));
    all(&mut machine);
    let moved = fetched(&mut machine, BASE);
    let block = &machine.code.blocks[moved];
    assert_eq!((block.physical, block.tier2), (frames[1], false));
    assert_eq!(machine.statistics.tlbi_keep.dropped, 1);
    // Mapped back: the superblock, whose words are unchanged.
    write_entry(&mut machine, l3, page_descriptor(frames[0], false));
    all(&mut machine);
    assert_eq!(fetched(&mut machine, BASE), tier2);
    // The superblock goes: the kept context reaches tier 1's block.
    machine.tier2_deopt(tier2);
    all(&mut machine);
    let after = fetched(&mut machine, BASE);
    let block = &machine.code.blocks[after];
    assert_eq!((block.physical, block.tier2), (frames[0], false));
    let counts = machine.statistics.tlbi_keep;
    assert_eq!(counts.mismatches, 0, "{counts:#?}");
    assert_eq!(counts.checked, counts.kept, "{counts:#?}");
}

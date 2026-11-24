use super::super::Fetched;
use super::super::tests::{BASE, machine, make_pending};

#[test]
fn legacy_first_fault_qualification_is_cached_and_reports_dispatcher_cost() {
    let program = [0x91000400u32, 0xf9400043, 0xd4000001];
    let input: Vec<u8> = program.into_iter().flat_map(u32::to_le_bytes).collect();
    let mut machine = machine(&program);
    let Ok(Fetched::Block(index)) = machine.fetch_block(BASE, false) else {
        panic!("actual emission");
    };
    make_pending(&mut machine, index, &input);
    let mut owner = machine.code.blocks.recovery().remove(&index).unwrap();
    let started = std::time::Instant::now();
    let first = owner.qualify(BASE, None, None).unwrap().rows();
    let cold = started.elapsed();
    assert!(
        first
            .iter()
            .any(|row| row.site.kind() == stela_compiler::block::RecoverySiteKind::GuestAccess)
    );
    let started = std::time::Instant::now();
    for _ in 0..32 {
        assert_eq!(owner.qualify(BASE, None, None).unwrap().rows(), first);
    }
    let cached = started.elapsed();
    assert_eq!(
        owner.qualification_attempts, 1,
        "only the first actual fault re-emits the plan"
    );
    eprintln!(
        "dispatcher recovery qualification: source={} native={} cold_ns={} cached_average_ns={} (current host; no native guest entered)",
        input.len(),
        owner.code_bytes(),
        cold.as_nanos(),
        cached.as_nanos() / 32
    );
}

use super::super::{Offer, Prepared, prepare};
use super::{Emission, NativeFailureReason, StoredOrigin, Template};
use crate::translations::{Digest, Record, translator_id};
use alloc::{sync::Arc, vec::Vec};
use stela_compiler::block::{FaultRecovery, RecoveryBinding};

fn store_window() -> (
    Arc<super::Preparation>,
    Vec<u8>,
    stela_compiler::block::CompiledBlock,
) {
    let preparation = Arc::new(prepare::test_preparation());
    let source: Vec<u8> = [0xf900_0041u32; 256]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect();
    let Prepared::Block {
        compiled, input, ..
    } = preparation.prepare(&source, BASE, source.len()).unwrap()
    else {
        panic!("actual store-window compilation");
    };
    assert_eq!(input, source.len());
    assert!(!compiled.fault_recovery.is_empty());
    (preparation, source, compiled)
}

#[test]
fn a_compiled_owner_keeps_only_its_packed_rows() {
    let (preparation, source, compiled) = store_window();
    let expected = compiled.fault_recovery.clone();
    let unpacked = expected.len() * core::mem::size_of::<FaultRecovery>()
        + expected
            .iter()
            .map(|row| row.bindings.len() * core::mem::size_of::<RecoveryBinding>())
            .sum::<usize>();
    let native_bytes = compiled.native.x86_64_code.len();
    let mut owner =
        Template::compiled(Arc::new(Emission::new(source, compiled, preparation))).installed();
    assert_eq!(
        owner.code_bytes(),
        native_bytes,
        "the code's image, not a copy"
    );
    assert_eq!(owner.qualify(BASE, None, None).unwrap().rows(), expected);
    assert!(
        owner.test_packed_bytes() * 3 < unpacked,
        "{} packed bytes of {unpacked}",
        owner.test_packed_bytes()
    );
}

#[test]
fn exact_lazy_qualification_replaces_portable_bank_with_only_minted_rows() {
    let preparation = Arc::new(prepare::test_preparation());
    let mut page = [0u8; crate::translations::PAGE_BYTES];
    let source: Vec<u8> = [0x9100_0400u32, 0xf940_0043, 0xd400_0001]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect();
    page[..source.len()].copy_from_slice(&source);
    let key = Digest::of(&page);
    let context = preparation.context(BASE);
    let offer = Offer::compile(key, BASE, BASE, context, source, preparation.clone()).unwrap();
    let expected = &offer.block().fault_recovery;
    let code_bytes = offer.block().native.x86_64_code.len();
    assert!(!expected.is_empty());
    let translator = translator_id();
    let mut record = Record::new(translator, key);
    record.insert(offer.entry().unwrap());
    let record = Record::decode(&record.encode(), &translator, &key).unwrap();
    let origin = Arc::new(StoredOrigin {
        key,
        context,
        page: BASE,
        preparation,
        kept: None,
    });
    let mut owner = {
        let entry = record.find(0, &context).unwrap();
        let mut relocated = crate::translations::Relocated::default();
        assert!(entry.relocate(BASE, offer.input(), &mut relocated));
        Template::stored(entry, &relocated, origin).installed()
    };
    assert_eq!(owner.stored_key(), Some(key));
    assert_eq!(owner.code_bytes(), code_bytes);
    assert_eq!(
        &owner
            .qualify(
                BASE,
                Some(&crate::translations::StoredRecord::of(&record)),
                None,
            )
            .unwrap()
            .rows(),
        expected
    );
    assert_eq!(owner.stored_key(), None);
    assert_eq!(owner.code_bytes(), code_bytes);
    assert_eq!(owner.qualification_attempts, 1);
    assert_eq!(&owner.qualify(BASE, None, None).unwrap().rows(), expected);
    assert_eq!(owner.qualification_attempts, 1);
}

/// A block installed from a record that the store let go since (a pack
/// read on demand keeps a bounded cache of records): its first fault
/// compiles the entry's bytes in guest memory again and gets the record's
/// rows; bytes that no longer admit to the entry's input give none.
#[test]
fn a_stored_block_whose_record_went_qualifies_from_its_bytes_in_memory() {
    let preparation = Arc::new(prepare::test_preparation());
    let source: Vec<u8> = [0x9100_0400u32, 0xf940_0043, 0xd400_0001]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect();
    let mut page = [0u8; crate::translations::PAGE_BYTES];
    page[..source.len()].copy_from_slice(&source);
    let key = Digest::of(&page);
    let context = preparation.context(BASE);
    let offer = Offer::compile(
        key,
        BASE,
        BASE,
        context,
        source.clone(),
        preparation.clone(),
    )
    .unwrap();
    let expected = offer.block().fault_recovery.clone();
    assert!(!expected.is_empty());
    let translator = translator_id();
    let mut record = Record::new(translator, key);
    record.insert(offer.entry().unwrap());
    let record = Record::decode(&record.encode(), &translator, &key).unwrap();
    let installed = || {
        let origin = Arc::new(StoredOrigin {
            key,
            context,
            page: BASE,
            preparation: preparation.clone(),
            kept: None,
        });
        let entry = record.find(0, &context).unwrap();
        let mut relocated = crate::translations::Relocated::default();
        assert!(entry.relocate(BASE, offer.input(), &mut relocated));
        Template::stored(entry, &relocated, origin).installed()
    };
    let mut owner = installed();
    let bytes = owner.stored_input_bytes().unwrap();
    assert_eq!(bytes, offer.input().len());
    // No record and no bytes: nothing qualifies, as before.
    assert_eq!(
        owner.qualify(BASE, None, None).map(|rows| rows.rows()),
        Err(NativeFailureReason::StoredRecord)
    );
    assert_eq!(
        owner
            .qualify(BASE, None, Some(&source[..bytes]))
            .unwrap()
            .rows(),
        expected
    );
    assert_eq!(owner.stored_input_bytes(), None);
    // Guest bytes rewritten since: refused, never another block's rows.
    let mut owner = installed();
    let mut changed = source.clone();
    changed[4] ^= 1;
    assert_eq!(
        owner
            .qualify(BASE, None, Some(&changed[..bytes]))
            .map(|rows| rows.rows()),
        Err(NativeFailureReason::StoredRecord)
    );
}

/// A record of the block's page that no longer holds its entry (after the
/// pack is retired, a lookup gives only the pieces of the record still in
/// memory) is no record of it: the fault reads the entry's bytes from guest
/// memory, as for a record the store let go, and gets the block's rows.
#[test]
fn a_stored_block_whose_record_lost_its_entry_qualifies_from_its_bytes_in_memory() {
    let preparation = Arc::new(prepare::test_preparation());
    let source: Vec<u8> = [0x9100_0400u32, 0xf940_0043, 0xd400_0001]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect();
    let mut page = [0u8; crate::translations::PAGE_BYTES];
    page[..source.len()].copy_from_slice(&source);
    let key = Digest::of(&page);
    let context = preparation.context(BASE);
    let compile = |pc: u64, input: &[u8]| {
        Offer::compile(key, BASE, pc, context, input.to_vec(), preparation.clone()).unwrap()
    };
    let offer = compile(BASE, &source);
    let expected = offer.block().fault_recovery.clone();
    assert!(!expected.is_empty());
    let translator = translator_id();
    let encode = |offer: &Offer| {
        let mut record = Record::new(translator, key);
        record.insert(offer.entry().unwrap());
        Record::decode(&record.encode(), &translator, &key).unwrap()
    };
    let whole = encode(&offer);
    // The same page's record, holding only another block's entry.
    let partial = encode(&compile(BASE + 8, &source[8..]));
    let (whole, partial) = (
        crate::translations::StoredRecord::of(&whole),
        crate::translations::StoredRecord::of(&partial),
    );
    let origin = Arc::new(StoredOrigin {
        key,
        context,
        page: BASE,
        preparation: preparation.clone(),
        kept: None,
    });
    let entry = whole.find(0, &context).unwrap();
    let mut relocated = crate::translations::Relocated::default();
    assert!(entry.relocate(BASE, offer.input(), &mut relocated));
    let mut owner = Template::stored(&entry, &relocated, origin).installed();
    assert!(owner.stored_in(&whole));
    assert!(!owner.stored_in(&partial));
    let bytes = owner.stored_input_bytes().unwrap();
    // The partial record alone gives no rows: what the fault was.
    assert_eq!(
        owner
            .qualify(BASE, Some(&partial), None)
            .map(|rows| rows.rows()),
        Err(NativeFailureReason::StoredRecord)
    );
    assert_eq!(
        owner
            .qualify(BASE, Some(&partial), Some(&source[..bytes]))
            .unwrap()
            .rows(),
        expected
    );
}

/// Retired, a lookup of a key with a piece left in the file gives the
/// pieces in memory alone, where before it gave none: a record that can lack
/// an entry the boot installed from the whole record, which a fault must not
/// take for that entry's (`CodeRecovery::stored_in`). Asimose's 0.6.0
/// stopped at `NativeFailureReason::StoredRecord` 2 s after a retirement.
#[test]
fn a_retired_store_gives_a_record_of_only_its_pieces_in_memory() {
    use crate::translations::pack::{Layout, Pack, Piece, Store, prefix_bytes};
    let translator = translator_id();
    let keys: Vec<Digest> = (0..64u32).map(|n| Digest::of(&n.to_le_bytes())).collect();
    let layout = |keys: &[Digest]| {
        let records = keys
            .iter()
            .map(|&key| (key, Piece::New(Record::new(translator, key).encode())))
            .collect();
        Layout::new(&translator, records).to_vec()
    };
    // Every key in a segment left in the file, half of them in one in memory.
    let bytes = [layout(&keys), layout(&keys[..32])];
    let prefix = prefix_bytes(&bytes[0], &translator).unwrap();
    let disk = Pack::index_only(&bytes[0][..prefix], &translator).unwrap();
    let whole = Pack::parse(&bytes[1], &translator).unwrap();
    let mut store = Store::over(translator, alloc::vec![disk, whole]);
    assert!(store.stored(&keys[3]).is_none());
    assert!(store.stored(&keys[40]).is_none());
    store.retire();
    assert!(store.stored(&keys[3]).is_some());
    assert!(store.stored(&keys[40]).is_none());
}

/// An entry proven during the boot qualifies from itself, at the page it
/// was rebased to: the rows of the block compiled at that page.
#[test]
fn a_kept_entry_qualifies_at_its_new_page_without_a_record() {
    let preparation = Arc::new(prepare::test_preparation());
    let source: Vec<u8> = [0x1000_0040u32, 0xf940_0043, 0xd400_0001]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect();
    let key = Digest([7; 32]);
    let context = preparation.context(BASE);
    let moved = BASE + 0x5_4000;
    let at = |page| {
        Offer::compile(
            key,
            page,
            page,
            context,
            source.clone(),
            preparation.clone(),
        )
        .unwrap()
    };
    let entry = Arc::new(at(BASE).entry().unwrap());
    let expected = at(moved).block().fault_recovery.clone();
    assert!(!expected.is_empty());
    let origin = Arc::new(StoredOrigin {
        key,
        context,
        page: moved,
        preparation: preparation.clone(),
        kept: Some(Arc::clone(&entry)),
    });
    let mut relocated = crate::translations::Relocated::default();
    assert!(entry.relocate(moved, &source, &mut relocated));
    assert_eq!(relocated.code, at(moved).block().native.x86_64_code);
    let mut owner = Template::stored(&entry, &relocated, origin).installed();
    // No pack record is looked up for it.
    assert_eq!(owner.stored_key(), None);
    assert_eq!(&owner.qualify(moved, None, None).unwrap().rows(), &expected);
}

#[test]
fn fault_rows_survive_packing() {
    let program = [0x9100_0400u32, 0xf900_0020, 0xf940_0043, 0x9100_0c64].repeat(64);
    let mut description = prepare::test_preparation().description();
    description.checks = stela_compiler::x86_64::MemoryChecks::system(true, false, true, false);
    let preparation = Arc::new(super::Preparation::from_description(&description).unwrap());
    let source: Vec<u8> = program.into_iter().flat_map(u32::to_le_bytes).collect();
    let Prepared::Block {
        compiled, input, ..
    } = preparation.prepare(&source, BASE, source.len()).unwrap()
    else {
        panic!("actual load compilation");
    };
    assert_eq!(input, source.len());
    assert!(!compiled.fault_recovery.is_empty());
    let expected = compiled.fault_recovery.clone();
    let mut owner =
        Template::compiled(Arc::new(Emission::new(source, compiled, preparation))).installed();
    assert_eq!(owner.qualify(BASE, None, None).unwrap().rows(), expected);
}

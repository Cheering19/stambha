use super::*;
use alloc::vec::Vec;

const NOP: u32 = 0xd503_201f;
const RET: u32 = 0xd65f_03c0;
/// The synthetic leaf's exit: its `ret` at word 60.
const EXIT: u16 = 240;
/// The word [`replaced`] puts a refused or masked word at.
const SLOT: usize = 30;

/// `add xN, xN, #imm`.
fn add(register: u32, immediate: u32) -> u32 {
    0x9100_0000 | (immediate & 0xfff) << 10 | register << 5 | register
}
/// `b.ne` by `words` from the branch.
fn b_ne(words: i32) -> u32 {
    0x5400_0001 | ((words as u32) & 0x7ffff) << 5
}
/// `cbz x3` by `words` from the branch.
fn cbz_x3(words: i32) -> u32 {
    0xb400_0003 | ((words as u32) & 0x7ffff) << 5
}
/// `b` by `words` from the branch.
fn b(words: i32) -> u32 {
    0x1400_0000 | ((words as u32) & 0x03ff_ffff)
}

fn bytes(words: &[u32]) -> Vec<u8> {
    words.iter().flat_map(|word| word.to_le_bytes()).collect()
}

/// A self-contained 64-word leaf: `add`s, an inside `b.ne` and `cbz`, its
/// `ret` at word 60 and NOP padding.
fn leaf_words() -> Vec<u32> {
    let mut words = vec![NOP; 64];
    for (index, word) in words.iter_mut().enumerate().take(60) {
        *word = add(index as u32 % 8, index as u32 + 1);
    }
    words[10] = b_ne(5);
    words[20] = cbz_x3(30);
    words[60] = RET;
    words
}

fn leaf() -> Vec<u8> {
    bytes(&leaf_words())
}

/// The leaf with word [`SLOT`] replaced by `word`.
fn replaced(word: u32) -> Vec<u8> {
    let mut words = leaf_words();
    words[SLOT] = word;
    bytes(&words)
}

fn masked_stream(function: &[u8]) -> Vec<u32> {
    let length = function.len() as u32;
    let (words, _) = function.as_chunks::<4>();
    words
        .iter()
        .enumerate()
        .map(|(index, word)| mask_word(u32::from_le_bytes(*word), index as u32 * 4, length))
        .collect()
}

#[test]
fn a_signature_matches_its_own_bytes() {
    let function = leaf();
    let signature = Signature::of(Id::Memmove, &function, EXIT).unwrap();
    assert!(signature.matches(&function));
    assert_eq!(signature.id(), Id::Memmove);
    assert_eq!(signature.length(), 256);
    assert_eq!(signature.exit(), 240);
}

#[test]
fn every_changed_unmasked_bit_misses() {
    let function = leaf();
    let signature = Signature::of(Id::Memmove, &function, EXIT).unwrap();
    for word in 0..function.len() / 4 {
        for bit in 0..32 {
            let mut changed = function.clone();
            let mut value = u32::from_le_bytes(changed[word * 4..word * 4 + 4].try_into().unwrap());
            value ^= 1 << bit;
            changed[word * 4..word * 4 + 4].copy_from_slice(&value.to_le_bytes());
            assert!(
                !signature.matches(&changed) || Signature::of(Id::Memmove, &changed, EXIT).is_err(),
                "word {word} bit {bit}: {value:#010x} still matches"
            );
        }
    }
}

#[test]
fn an_outside_branch_is_masked_and_an_inside_one_is_not() {
    // Forward and backward out of the function: only the opcode is left.
    let ahead = replaced(b(100));
    let behind = replaced(b(-100));
    assert_ne!(ahead, behind);
    assert_eq!(masked_stream(&ahead), masked_stream(&behind));
    for function in [&ahead, &behind] {
        assert_eq!(
            eligible(function, EXIT),
            Err(Ineligible::Masked {
                offset: SLOT as u32 * 4
            })
        );
    }
    // An inside branch keeps its immediate: another target misses.
    let function = leaf();
    let signature = Signature::of(Id::Memmove, &function, EXIT).unwrap();
    let mut words = leaf_words();
    words[10] = b_ne(6);
    let retargeted = bytes(&words);
    assert_eq!(eligible(&retargeted, EXIT), Ok(()));
    assert!(!signature.matches(&retargeted));
    // A branch to the word after the last is outside.
    assert_eq!(
        eligible(&replaced(b(64 - SLOT as i32)), EXIT),
        Err(Ineligible::Masked {
            offset: SLOT as u32 * 4
        })
    );
    assert_eq!(eligible(&replaced(b(63 - SLOT as i32)), EXIT), Ok(()));
}

#[test]
fn every_pc_relative_form_is_masked() {
    // B/BL, B.cond and BC.cond, CBZ/CBNZ (W and X) and TBZ/TBNZ, all to
    // outside the function; ADR, ADRP and the literal loads whatever their
    // immediate. Each loses exactly its PC-relative field.
    const IMM26: u32 = 0x03ff_ffff;
    const IMM19: u32 = 0x00ff_ffe0;
    const IMM14: u32 = 0x0007_ffe0;
    const ADR: u32 = 0x60ff_ffe0;
    for (word, field) in [
        (0x1400_0064, IMM26),                       // b +100 words
        (0x9400_0064, IMM26),                       // bl +100 words
        (0x5400_0000 | (0x1000 << 5), IMM19),       // b.eq +0x1000 words
        (0x5400_0010 | (0x1000 << 5), IMM19),       // bc.eq +0x1000 words
        (0x3400_0000 | (0x1000 << 5) | 1, IMM19),   // cbz w1
        (0xb500_0000 | (0x7_f000 << 5) | 1, IMM19), // cbnz x1, backward
        (0x3600_0000 | (0x1000 << 5) | 2, IMM14),   // tbz w2,#0
        (0xb700_0000 | (0x2000 << 5) | 2, IMM14),   // tbnz x2,#32, backward
        (0x1000_0000, ADR),                         // adr x0, .
        (0x9000_0000, ADR),                         // adrp x0, .
        (0xb000_0021, ADR),                         // adrp x1, .+5 pages
        (0x5800_0000, IMM19),                       // ldr x0, .
        (0x1800_0020, IMM19),                       // ldr w0, .+4
        (0x9800_0000, IMM19),                       // ldrsw x0, .
        (0xd800_0000, IMM19),                       // prfm pldl1keep, .
        (0x5c00_0000, IMM19),                       // ldr d0, .
    ] {
        assert_eq!(
            eligible(&replaced(word), EXIT),
            Err(Ineligible::Masked {
                offset: SLOT as u32 * 4
            }),
            "{word:#010x}"
        );
        assert_eq!(
            masked_stream(&replaced(word))[SLOT],
            word & !field,
            "{word:#010x}"
        );
    }
}

#[test]
fn a_prefix_collision_still_needs_the_digest() {
    let function = leaf();
    let signature = Signature::of(Id::Memmove, &function, EXIT).unwrap();
    let head: &[u8; PREFIX_BYTES] = function[..PREFIX_BYTES].try_into().unwrap();
    assert_eq!(signature.prefix(), prefix_hash(head, 256));
    let forged = Signature::generated(Id::Memmove, 256, EXIT, signature.prefix(), [0; 32]);
    assert!(!forged.matches(&function));
    // With its digest too, a generated signature is the computed one.
    let generated = Signature::generated(
        Id::Memmove,
        256,
        EXIT,
        signature.prefix(),
        *signature.digest(),
    );
    assert_eq!(generated, signature);
    assert!(generated.matches(&function));
}

#[test]
fn the_prefix_hash_depends_on_the_length() {
    // A branch inside 64 words is outside 32: the same head masks apart.
    let mut words = leaf_words();
    words[2] = b(40);
    let head: [u8; PREFIX_BYTES] = bytes(&words[..8]).try_into().unwrap();
    assert_ne!(prefix_hash(&head, 256), prefix_hash(&head, 128));
}

#[test]
fn eligibility_names_each_refused_class() {
    let offset = SLOT as u32 * 4;
    for (word, refusal) in [
        (0x9400_0000, Ineligible::Call { offset }), // bl . (inside)
        (0xd61f_0200, Ineligible::RegisterBranch { offset }), // br x16
        (0xd400_0001, Ineligible::Exception { offset }), // svc #0
        (0xd51b_d040, Ineligible::System { offset }), // msr tpidr_el0, x0
        (0xc85f_7c20, Ineligible::Exclusive { offset }), // ldxr x0,[x1]
        (0xf820_8020, Ineligible::Exclusive { offset }), // swp x0,x0,[x1]
        (0xd503_237f, Ineligible::System { offset }), // pacibsp
        (0xdac1_0020, Ineligible::Pointer { offset }), // pacia x0, x1
        (0xf820_0420, Ineligible::Pointer { offset }), // ldraa x0,[x1]
        (0x9ac2_3020, Ineligible::Pointer { offset }), // pacga x0, x1, x2
        (0x9000_0000, Ineligible::Masked { offset }), // adrp x0, 0
    ] {
        assert_eq!(
            eligible(&replaced(word), EXIT),
            Err(refusal),
            "{word:#010x}"
        );
    }
    for word in [
        0xd50b_7423, // dc zva, x3
        0xd503_245f, // bti c
        0xd503_241f, // bti
        0xd53b_00e4, // mrs x4, dczid_el0
        NOP,
    ] {
        assert_eq!(eligible(&replaced(word), EXIT), Ok(()), "{word:#010x}");
    }
    let function = leaf();
    // Too short, not whole words, too long.
    assert_eq!(eligible(&function[..28], 24), Err(Ineligible::Length));
    assert_eq!(eligible(&function[..254], EXIT), Err(Ineligible::Length));
    let long = bytes(&[NOP; 1025]);
    assert_eq!(eligible(&long, EXIT), Err(Ineligible::Length));
    // The exit out of the function, not a word, at the entry, or no `ret`.
    for exit in [256, 1000, 242, 0, 236] {
        assert_eq!(eligible(&function, exit), Err(Ineligible::Exit), "{exit}");
    }
    // A RET through another register is a register branch.
    assert_eq!(
        eligible(&replaced(0xd65f_0200), EXIT),
        Err(Ineligible::RegisterBranch { offset })
    );
}

#[test]
fn bytes_shorter_than_the_length_never_match() {
    let function = leaf();
    let signature = Signature::of(Id::Memmove, &function, EXIT).unwrap();
    for length in [0, 31, 32, 252] {
        assert!(!signature.matches(&function[..length]), "{length}");
    }
    // Bytes past the function are not its own.
    let mut longer = function.clone();
    longer.extend_from_slice(&[0xff; 64]);
    assert!(signature.matches(&longer));
}

#[test]
fn names_round_trip() {
    for routine in Id::PRODUCTION {
        assert_eq!(Id::from_name(routine.name()), Some(routine));
    }
    assert_eq!(Id::from_name("bail"), None);
    assert_eq!(Id::from_name("touch"), None);
    assert_eq!(Id::from_name("Memmove"), None);
    assert_eq!(Id::Bail.name(), "bail");
    assert_eq!(Id::Touch.name(), "touch");
    let bits = Id::PRODUCTION
        .iter()
        .chain(&[Id::Bail, Id::Touch])
        .fold(0, |bits, routine| {
            assert_eq!(bits & routine.bit(), 0, "{routine:?}");
            bits | routine.bit()
        });
    assert_eq!(bits, 0x7f);
}

/// A table of two functions: the leaf as memmove and another as bzero.
fn two_routines() -> alloc::sync::Arc<[Signature]> {
    alloc::sync::Arc::from([
        Signature::of(Id::Memmove, &leaf(), EXIT).unwrap(),
        Signature::of(Id::Bzero, &replaced(NOP), EXIT).unwrap(),
    ])
}

#[test]
fn an_off_config_enables_nothing() {
    let off = Config::off();
    assert_eq!(off.enabled, 0);
    assert!(off.table.is_empty());
    assert_eq!(off.max_bytes, Config::MAX_BYTES);
    assert_eq!(Config::default(), off);
    for routine in Id::PRODUCTION.into_iter().chain([Id::Bail, Id::Touch]) {
        assert!(!off.enables(routine), "{routine:?}");
    }
    // Without --hle a boot is off whatever its table, and the generated
    // table holds nothing yet.
    assert_eq!(config::booted(false, &[], two_routines()), off);
    assert_eq!(Config::boot(false, &[]), off);
    assert_eq!(Config::boot(true, &[]).enabled, 0);
    // A routine without a signature is never enabled.
    assert_eq!(Config::with_table(two_routines(), &[Id::Strlen]).enabled, 0);
}

#[test]
fn hle_off_clears_only_its_ids() {
    let both = config::booted(true, &[], two_routines());
    assert!(both.enables(Id::Memmove) && both.enables(Id::Bzero));
    assert_eq!(both.enabled, Id::Memmove.bit() | Id::Bzero.bit());
    let without_bzero = config::booted(true, &[Id::Bzero], two_routines());
    assert_eq!(without_bzero.enabled, Id::Memmove.bit());
    assert_eq!(without_bzero.table, two_routines());
    // Naming a routine the table lacks clears nothing else.
    let without_strlen = config::booted(true, &[Id::Strlen], two_routines());
    assert_eq!(without_strlen.enabled, both.enabled);
    assert_eq!(
        Config::with_table(two_routines(), &[Id::Memmove, Id::Strlen]).enabled,
        Id::Memmove.bit()
    );
}

#[test]
fn max_bytes_takes_its_bounds() {
    for bytes in [256, 4096, 1 << 20] {
        assert_eq!(Config::off().with_max_bytes(bytes).max_bytes, bytes);
    }
}

#[test]
#[should_panic(expected = "256 bytes to 1 MiB")]
fn max_bytes_below_its_bounds_panics() {
    Config::off().with_max_bytes(255);
}

#[test]
#[should_panic(expected = "256 bytes to 1 MiB")]
fn max_bytes_above_its_bounds_panics() {
    Config::off().with_max_bytes((1 << 20) + 1);
}

#[test]
#[should_panic(expected = "at most 64 signatures")]
fn a_table_of_more_than_64_signatures_panics() {
    let signature = Signature::of(Id::Memmove, &leaf(), EXIT).unwrap();
    Config::with_table(alloc::sync::Arc::from(vec![signature; 65]), &[Id::Memmove]);
}

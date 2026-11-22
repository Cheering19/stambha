//! Function signatures: the SHA-256 of a function's words with every
//! PC-relative field masked, so a function matches wherever its image
//! loads, and a hash of its first 32 bytes that finds candidates at
//! compile time without a table walk per compile.
//!
//! A branch's field is masked only when its target lies outside the
//! function: a branch inside keeps its offset, which the function's bytes
//! decide. So masking depends on the function's length, and so does the
//! prefix hash. ADR, ADRP and literal loads are always masked. A masked
//! word hashes with a flag that says so: an outside branch masks to the
//! word of a branch to itself (`b .`), which a function may hold, and the
//! digest keeps the two apart.
//!
//! The phase-1 eligibility of a function ([`eligible`]) admits only
//! self-contained leaves the native routines can replace whole, and
//! [`Signature::of`] runs it, so every signature built from bytes is of an
//! eligible function.
use super::Id;
use stela_crypto::Sha256;
use stela_crypto::digest::Digest as _;

/// The bytes at a function's entry its prefix hash covers.
pub const PREFIX_BYTES: usize = 32;
/// A function's length: at least its prefix and at most a fetch page.
const LENGTHS: core::ops::RangeInclusive<usize> = PREFIX_BYTES..=4096;
/// `ret` (through X30), the only register branch a function may hold and
/// the word its exit offset names.
const RET: u32 = 0xd65f_03c0;

/// A function a native routine replaces: its routine, its length and the
/// offset of its exit `ret` in bytes, and the hashes of its masked words.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Signature {
    id: Id,
    length: u16,
    exit: u16,
    prefix: u64,
    digest: [u8; 32],
}

/// Why a function cannot be replaced in phase 1, with the byte offset of
/// the word that refuses it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ineligible {
    /// Not whole words, shorter than its prefix or longer than a page.
    Length,
    /// The exit is the entry, not a word of the function, or not `ret`.
    Exit,
    /// Its last word is neither `ret` nor a branch inside it (a `b`), so
    /// it could run on past its end into bytes it does not hold.
    FallsThrough,
    /// A PC-relative word: a branch out of the function, ADR, ADRP or a
    /// literal load.
    Masked { offset: u32 },
    /// BL.
    Call { offset: u32 },
    /// A register branch other than `ret`: BR, BLR, a RET through another
    /// register, ERET or a PAC form.
    RegisterBranch { offset: u32 },
    /// A System instruction other than NOP, BTI, DC ZVA and MRS DCZID_EL0
    /// (the PAC hints such as PACIBSP included).
    System { offset: u32 },
    /// SVC, HVC, SMC, BRK, HLT or DCPS.
    Exception { offset: u32 },
    /// A word of the reserved space (bits 31:25 clear): UDF, an Apple AMX
    /// coprocessor word or another Apple word there, which the original
    /// traps on or hands a coprocessor and no native routine runs.
    Reserved { offset: u32 },
    /// A load or store exclusive, a load-acquire or store-release (LDAR,
    /// STLR, LDAPR, and the RCpc forms with an offset or writeback such as
    /// LDAPUR and STLUR), an LSE or LSE128 atomic, or an MTE tag access of
    /// the RCpc forms' class: an ordering or tag effect a plain copy lacks.
    Exclusive { offset: u32 },
    /// PAC*, AUT*, XPAC*, PACGA or LDRAA/LDRAB.
    Pointer { offset: u32 },
}

impl Signature {
    /// The signature of `bytes`, the whole function, if it is eligible.
    pub fn of(id: Id, bytes: &[u8], exit: u16) -> Result<Self, Ineligible> {
        eligible(bytes, exit)?;
        let length = bytes.len() as u32;
        Ok(Self {
            id,
            length: length as u16,
            exit,
            prefix: prefix_hash(head(bytes), length),
            digest: digest(bytes),
        })
    }

    /// A signature a generated table holds. Its function's bytes were
    /// checked when it was generated; a length or exit no eligible function
    /// has stops the build.
    pub const fn generated(id: Id, length: u16, exit: u16, prefix: u64, digest: [u8; 32]) -> Self {
        let bytes = length as usize;
        assert!(
            bytes >= *LENGTHS.start() && bytes <= *LENGTHS.end() && bytes.is_multiple_of(4),
            "a signature's length is whole words of 32 bytes to a page"
        );
        assert!(
            exit != 0 && exit < length && exit.is_multiple_of(4),
            "a signature's exit is a word of its function past the entry"
        );
        Self {
            id,
            length,
            exit,
            prefix,
            digest,
        }
    }

    pub fn id(&self) -> Id {
        self.id
    }

    /// The function's bytes.
    pub fn length(&self) -> usize {
        usize::from(self.length)
    }

    /// The offset of its exit `ret` from the entry.
    pub fn exit(&self) -> u64 {
        u64::from(self.exit)
    }

    /// The hash of its first [`PREFIX_BYTES`] masked bytes ([`prefix_hash`]).
    pub fn prefix(&self) -> u64 {
        self.prefix
    }

    /// The SHA-256 of its masked words, little-endian.
    pub fn digest(&self) -> &[u8; 32] {
        &self.digest
    }

    /// Whether `function`, the bytes from an entry (at least the length;
    /// those past it are not the function's), is this function.
    pub fn matches(&self, function: &[u8]) -> bool {
        let Some(function) = function.get(..self.length()) else {
            return false;
        };
        prefix_hash(head(function), u32::from(self.length)) == self.prefix
            && digest(function) == self.digest
    }
}

fn head(function: &[u8]) -> &[u8; PREFIX_BYTES] {
    function[..PREFIX_BYTES]
        .try_into()
        .expect("a function holds its prefix")
}

/// The whole words of `bytes`, each with its byte offset.
fn words(bytes: &[u8]) -> impl Iterator<Item = (u32, u32)> + '_ {
    let (words, _) = bytes.as_chunks::<4>();
    words
        .iter()
        .enumerate()
        .map(|(index, word)| (index as u32 * 4, u32::from_le_bytes(*word)))
}

/// `value`'s low `bits` as a signed number.
fn signed(value: u32, bits: u32) -> i64 {
    i64::from(((value << (32 - bits)) as i32) >> (32 - bits))
}

/// The bits of `word`, at byte `offset` of a function of `length` bytes,
/// that hold a PC-relative field it masks (Arm ARM C4.1): B and BL (imm26),
/// B.cond and BC.cond, CBZ and CBNZ (imm19) and TBZ and TBNZ (imm14) whose
/// targets lie outside `[0, length)`; ADR and ADRP (immlo:immhi) and the
/// literal loads LDR, LDRSW and PRFM (imm19) always. 0 for any other word.
fn pc_relative(word: u32, offset: u32, length: u32) -> u32 {
    // A branch's offset `field` of `bits` words, if its target is outside.
    let branch = |field: u32, bits: u32| {
        let words = signed((word & field) >> field.trailing_zeros(), bits);
        match (0..i64::from(length)).contains(&(i64::from(offset) + 4 * words)) {
            true => 0,
            false => field,
        }
    };
    if word & 0x7c00_0000 == 0x1400_0000 {
        return branch(0x03ff_ffff, 26);
    }
    if word & 0xff00_0000 == 0x5400_0000 || word & 0x7e00_0000 == 0x3400_0000 {
        return branch(0x00ff_ffe0, 19);
    }
    if word & 0x7e00_0000 == 0x3600_0000 {
        return branch(0x0007_ffe0, 14);
    }
    if word & 0x1f00_0000 == 0x1000_0000 {
        return 0x60ff_ffe0;
    }
    if word & 0x3b00_0000 == 0x1800_0000 {
        return 0x00ff_ffe0;
    }
    0
}

/// `word` at byte `offset` of a function of `length` bytes with its
/// PC-relative field, if it has one the signature masks, cleared.
pub fn mask_word(word: u32, offset: u32, length: u32) -> u32 {
    word & !pc_relative(word, offset, length)
}

/// Whether the function `bytes`, exiting through the `ret` at `exit`, is a
/// self-contained leaf a phase-1 native routine can replace: it calls
/// nothing, leaves only through that `ret` or another, ends in a `ret` or
/// a branch back inside so it never runs past its last word, reads nothing
/// PC-relative, signs or authenticates no pointer and makes no system,
/// exception, reserved, exclusive, ordered or atomic access. The first
/// word that refuses it names the refusal, its classes checked in this
/// order, and then its last word.
pub fn eligible(bytes: &[u8], exit: u16) -> Result<(), Ineligible> {
    if !bytes.len().is_multiple_of(4) || !LENGTHS.contains(&bytes.len()) {
        return Err(Ineligible::Length);
    }
    let length = bytes.len() as u32;
    let exit = u32::from(exit);
    let ret = words(bytes).any(|(offset, word)| offset == exit && word == RET);
    if exit == 0 || !ret {
        return Err(Ineligible::Exit);
    }
    let mut last = RET;
    for (offset, word) in words(bytes) {
        refusal(word, offset, length)?;
        last = word;
    }
    // Every `b` is inside by now: one out of the function was masked.
    if last != RET && last & 0xfc00_0000 != 0x1400_0000 {
        return Err(Ineligible::FallsThrough);
    }
    Ok(())
}

fn refusal(word: u32, offset: u32, length: u32) -> Result<(), Ineligible> {
    if pc_relative(word, offset, length) != 0 {
        return Err(Ineligible::Masked { offset });
    }
    if word & 0xfc00_0000 == 0x9400_0000 {
        return Err(Ineligible::Call { offset });
    }
    if word & 0xfe00_0000 == 0xd600_0000 && word != RET {
        return Err(Ineligible::RegisterBranch { offset });
    }
    if word & 0xff00_0000 == 0xd400_0000 {
        return Err(Ineligible::Exception { offset });
    }
    let system = word & 0xffc0_0000 == 0xd500_0000;
    let admitted = word == 0xd503_201f // NOP
        || word & 0xffff_ff3f == 0xd503_241f // BTI, any targets
        || word & 0xffff_ffe0 == 0xd50b_7420 // DC ZVA
        || word & 0xffff_ffe0 == 0xd53b_00e0; // MRS DCZID_EL0
    if system && !admitted {
        return Err(Ineligible::System { offset });
    }
    // UDF and the Apple words beside it: AMX (`0x00201000 | op << 5 | Rt`),
    // GXF's and the SPTM's maintenance words.
    if word & 0xfe00_0000 == 0 {
        return Err(Ineligible::Reserved { offset });
    }
    let ordered = word & 0x3f00_0000 == 0x0800_0000 // exclusives, LDAR, STLR, CAS
        || word & 0x3b20_0c00 == 0x3820_0000 // LSE atomics, LDAPR
        || word & 0x3f00_0000 == 0x1900_0000 // LDAPUR, STLUR, RCpc3, LSE128, MTE tags
        || word & 0x3f20_0c00 == 0x1d00_0800; // LDAPUR, STLUR (SIMD&FP)
    if ordered {
        return Err(Ineligible::Exclusive { offset });
    }
    let pointer = word & 0xffff_0000 == 0xdac1_0000 // PAC*, AUT*, XPAC*
        || word & 0xffe0_fc00 == 0x9ac0_3000 // PACGA
        || word & 0xff20_0400 == 0xf820_0400; // LDRAA, LDRAB
    if pointer {
        return Err(Ineligible::Pointer { offset });
    }
    Ok(())
}

/// FNV-1a 64 over the 8 masked words of a function's first
/// [`PREFIX_BYTES`], little-endian, for a function of `length` bytes.
pub fn prefix_hash(head: &[u8; PREFIX_BYTES], length: u32) -> u64 {
    words(head).fold(0xcbf2_9ce4_8422_2325, |hash, (offset, word)| {
        mask_word(word, offset, length)
            .to_le_bytes()
            .iter()
            .fold(hash, |hash, &byte| {
                (hash ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3)
            })
    })
}

/// The SHA-256 of `function`'s masked words, little-endian, each masked as
/// a word of a function of its length and followed by a byte that is 1
/// when the word has a masked field: an outside branch masks to the word of
/// a branch to itself, which the flag tells apart.
fn digest(function: &[u8]) -> [u8; 32] {
    let length = function.len() as u32;
    let mut hasher = Sha256::new();
    for (offset, word) in words(function) {
        let field = pc_relative(word, offset, length);
        hasher.update((word & !field).to_le_bytes());
        hasher.update([u8::from(field != 0)]);
    }
    hasher.finalize().into()
}

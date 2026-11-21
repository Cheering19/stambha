//! Synthetic images for the symbolizer's tests: a little-endian arm64e
//! Mach-O dylib with three pages of `__TEXT` (its header, then two pages of
//! code), two symbols and three function starts, standalone and inside a
//! one-file dyld shared cache, as Apple's tools lay them out. No Apple
//! bytes.

/// The page size the engine keys pages by.
pub(super) const PAGE: usize = 4096;
/// The images' `__TEXT` and `__LINKEDIT` sizes.
const TEXT: usize = 3 * PAGE;
const LINKEDIT: usize = PAGE;
/// Where the images' code starts, and the second function's start, which
/// no symbol names, and the second symbol, `_beta`.
pub(super) const ALPHA: u64 = 0x1000;
pub(super) const UNNAMED: u64 = 0x1400;
pub(super) const BETA: u64 = 0x1800;
/// The dyld cache's image path and its unslid base.
pub(super) const CACHE_PATH: &str = "/usr/lib/system/libsynthetic.dylib";
pub(super) const CACHE_BASE: u64 = 0x1_8000_0000;
/// The standalone image's base in the report's tests.
pub(super) const BASE_FOR_REPORT: u64 = 0x7100_0000;

fn u32s(out: &mut Vec<u8>, words: &[u32]) {
    for word in words {
        out.extend_from_slice(&word.to_le_bytes());
    }
}

fn u64s(out: &mut Vec<u8>, words: &[u64]) {
    for word in words {
        out.extend_from_slice(&word.to_le_bytes());
    }
}

fn name16(out: &mut Vec<u8>, name: &[u8]) {
    let mut field = [0u8; 16];
    field[..name.len()].copy_from_slice(name);
    out.extend_from_slice(&field);
}

/// An `LC_SEGMENT_64` with at most one section, `(name, address, size,
/// file offset, flags)`.
fn segment(
    name: &[u8],
    address: u64,
    size: u64,
    offset: u64,
    protection: u32,
    section: Option<(&[u8], u64, u64, u32, u32)>,
) -> Vec<u8> {
    let sections = u32::from(section.is_some());
    let mut out = Vec::new();
    u32s(&mut out, &[0x19, 72 + 80 * sections]);
    name16(&mut out, name);
    u64s(&mut out, &[address, size, offset, size]);
    u32s(&mut out, &[protection, protection, sections, 0]);
    if let Some((section, address, size, offset, flags)) = section {
        name16(&mut out, section);
        name16(&mut out, name);
        u64s(&mut out, &[address, size]);
        u32s(&mut out, &[offset, 2, 0, 0, flags, 0, 0, 0]);
    }
    out
}

/// The `__TEXT` and `__LINKEDIT` bytes of the image at `base`, whose
/// segments lie at `text_offset` and `linkedit_offset` of their file and
/// whose code bytes `seed` picks.
fn image(base: u64, text_offset: u32, linkedit_offset: u32, seed: u8) -> (Vec<u8>, Vec<u8>) {
    let strings = b"\0_alpha\0_beta\0";
    let mut commands = Vec::new();
    commands.extend(segment(
        b"__TEXT",
        base,
        TEXT as u64,
        text_offset.into(),
        5,
        Some((
            b"__text",
            base + ALPHA,
            (TEXT - PAGE) as u64,
            text_offset + ALPHA as u32,
            0x8000_0400,
        )),
    ));
    commands.extend(segment(
        b"__LINKEDIT",
        base + TEXT as u64,
        LINKEDIT as u64,
        linkedit_offset.into(),
        1,
        None,
    ));
    // LC_SYMTAB: two symbols at 0, their strings at 0x200.
    u32s(
        &mut commands,
        &[
            2,
            24,
            linkedit_offset,
            2,
            linkedit_offset + 0x200,
            strings.len() as u32,
        ],
    );
    // LC_FUNCTION_STARTS at 0x100.
    u32s(&mut commands, &[0x26, 16, linkedit_offset + 0x100, 8]);
    let mut text = Vec::new();
    u32s(
        &mut text,
        &[
            0xfeed_facf,
            0x0100_000c,
            2,
            6,
            4,
            commands.len() as u32,
            0,
            0,
        ],
    );
    text.extend(commands);
    text.resize(TEXT, 0);
    // Two pages of code no other page of the image shares.
    for (index, byte) in text[PAGE..].iter_mut().enumerate() {
        *byte = ((index * 7 + 3) as u8 ^ (index >> 9) as u8).wrapping_add(seed);
    }
    let mut linkedit = Vec::new();
    for (name, value) in [(1u32, base + ALPHA), (8, base + BETA)] {
        u32s(&mut linkedit, &[name]);
        // N_SECT | N_EXT, section 1.
        linkedit.extend_from_slice(&[0x0f, 1, 0, 0]);
        u64s(&mut linkedit, &[value]);
    }
    linkedit.resize(0x100, 0);
    // ULEB128 deltas from `base`: ALPHA, UNNAMED, BETA, then the end.
    linkedit.extend_from_slice(&[0x80, 0x20, 0x80, 0x08, 0x80, 0x08, 0, 0]);
    linkedit.resize(0x200, 0);
    linkedit.extend_from_slice(strings);
    linkedit.resize(LINKEDIT, 0);
    (text, linkedit)
}

/// A standalone Mach-O of [`image`] at `base`: at any base, the same code.
pub(super) fn macho(base: u64) -> Vec<u8> {
    let (mut file, linkedit) = image(base, 0, TEXT as u32, 0);
    file.extend(linkedit);
    file
}

/// A one-file dyld shared cache of the old layout (no subcaches) holding
/// [`image`] at [`CACHE_BASE`] as [`CACHE_PATH`], its code unlike
/// [`macho`]'s.
pub(super) fn cache() -> Vec<u8> {
    let header = PAGE;
    let (text, linkedit) = image(CACHE_BASE, header as u32, (header + TEXT) as u32, 1);
    let mut file = Vec::new();
    file.extend_from_slice(b"dyld_v1  arm64e\0");
    // Mappings at 0x100, two; images at 0x200, one.
    u32s(&mut file, &[0x100, 2, 0x200, 1]);
    file.resize(0x100, 0);
    u64s(&mut file, &[CACHE_BASE, TEXT as u64, header as u64]);
    u32s(&mut file, &[5, 5]);
    u64s(
        &mut file,
        &[
            CACHE_BASE + TEXT as u64,
            LINKEDIT as u64,
            (header + TEXT) as u64,
        ],
    );
    u32s(&mut file, &[1, 1]);
    file.resize(0x200, 0);
    u64s(&mut file, &[CACHE_BASE, 0, 0]);
    u32s(&mut file, &[0x300, 0]);
    file.resize(0x300, 0);
    file.extend_from_slice(CACHE_PATH.as_bytes());
    file.push(0);
    file.resize(header, 0);
    file.extend(text);
    file.extend(linkedit);
    file
}

/// The first 8 bytes of the SHA-256 of `page`, as the lines print them.
pub(super) fn key(page: &[u8]) -> u64 {
    use sha2::{Digest, Sha256};
    let digest = Sha256::digest(page);
    u64::from_be_bytes(digest[..8].try_into().unwrap())
}

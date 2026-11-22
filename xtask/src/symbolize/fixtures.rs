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
/// Where `__text` ends and the `__auth_stubs` section, the rest of the
/// last page, starts.
pub(super) const STUBS: u64 = 0x2c00;
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

/// An `LC_SEGMENT_64` with `sections`, each `(name, address, size, file
/// offset, flags)`.
fn segment(
    name: &[u8],
    address: u64,
    size: u64,
    offset: u64,
    protection: u32,
    sections: &[(&[u8], u64, u64, u32, u32)],
) -> Vec<u8> {
    let count = sections.len() as u32;
    let mut out = Vec::new();
    u32s(&mut out, &[0x19, 72 + 80 * count]);
    name16(&mut out, name);
    u64s(&mut out, &[address, size, offset, size]);
    u32s(&mut out, &[protection, protection, count, 0]);
    for &(section, address, size, offset, flags) in sections {
        name16(&mut out, section);
        name16(&mut out, name);
        u64s(&mut out, &[address, size]);
        u32s(&mut out, &[offset, 2, 0, 0, flags, 0, 0, 0]);
    }
    out
}

/// The `__TEXT` and `__LINKEDIT` bytes of the image at `base`, whose
/// segments lie at `text_offset` and `linkedit_offset` of their file,
/// whose code bytes `seed` picks and whose symbols at [`ALPHA`] and
/// [`BETA`] are `names`.
fn image(
    base: u64,
    text_offset: u32,
    linkedit_offset: u32,
    seed: u8,
    names: [&str; 2],
) -> (Vec<u8>, Vec<u8>) {
    let strings = format!("\0{}\0{}\0", names[0], names[1]);
    let strings = strings.as_bytes();
    let mut commands = Vec::new();
    commands.extend(segment(
        b"__TEXT",
        base,
        TEXT as u64,
        text_offset.into(),
        5,
        &[
            (
                b"__text",
                base + ALPHA,
                STUBS - ALPHA,
                text_offset + ALPHA as u32,
                0x8000_0400,
            ),
            // S_SYMBOL_STUBS, pure instructions.
            (
                b"__auth_stubs",
                base + STUBS,
                TEXT as u64 - STUBS,
                text_offset + STUBS as u32,
                0x8000_0408,
            ),
        ],
    ));
    commands.extend(segment(
        b"__LINKEDIT",
        base + TEXT as u64,
        LINKEDIT as u64,
        linkedit_offset.into(),
        1,
        &[],
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
    let second = 2 + names[0].len() as u32;
    for (name, value) in [(1u32, base + ALPHA), (second, base + BETA)] {
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

/// A standalone Mach-O of [`image`] at `base`: at any base, the same code,
/// `_alpha` and `_beta`.
pub(super) fn macho(base: u64) -> Vec<u8> {
    macho_named(base, 0, ["_alpha", "_beta"])
}

/// A standalone Mach-O of [`image`] at `base`, its code picked by `seed`
/// and its symbols `names`.
pub(super) fn macho_named(base: u64, seed: u8, names: [&str; 2]) -> Vec<u8> {
    let (mut file, linkedit) = image(base, 0, TEXT as u32, seed, names);
    file.extend(linkedit);
    file
}

/// A one-file dyld shared cache of the old layout (no subcaches) holding
/// [`image`] at [`CACHE_BASE`] as [`CACHE_PATH`], its code unlike
/// [`macho`]'s.
pub(super) fn cache() -> Vec<u8> {
    let header = PAGE;
    let (text, linkedit) = image(
        CACHE_BASE,
        header as u32,
        (header + TEXT) as u32,
        1,
        ["_alpha", "_beta"],
    );
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

/// The split cache's image path.
pub(super) const SPLIT_PATH: &str = "/usr/lib/libsplit.dylib";

/// A dyld shared cache of the layout macOS 13 and later ship: a main file
/// whose header names its subcaches (V2 entries, with their suffixes) and
/// a `.symbols` file by UUID, the image's `__TEXT` in the main file and
/// its `__LINKEDIT` (symbols, function starts) in `.01`. Each file's
/// suffix and bytes, the main file's suffix empty; [`image`] at
/// [`CACHE_BASE`] as [`SPLIT_PATH`], its code unlike the others'. The
/// header's fields lie where `object`'s `DyldCacheHeader` reads them.
pub(super) fn split_cache() -> [(&'static str, Vec<u8>); 3] {
    use core::mem::{offset_of, size_of};
    use object::LittleEndian as Le;
    use object::macho::{DyldCacheHeader, DyldCacheImageInfo, DyldSubCacheEntryV2};
    type Header = DyldCacheHeader<Le>;
    const MAIN: [u8; 16] = [1; 16];
    const SUB: [u8; 16] = [2; 16];
    const SYMBOLS: [u8; 16] = [3; 16];
    const MAPPINGS: usize = 0x400;
    const SUBCACHES: usize = 0x500;
    const IMAGES: usize = 0x600;
    const PATHS: usize = 0x700;
    let put32 = |file: &mut Vec<u8>, at: usize, value: u32| {
        file[at..at + 4].copy_from_slice(&value.to_le_bytes());
    };
    let put64 = |file: &mut Vec<u8>, at: usize, value: u64| {
        file[at..at + 8].copy_from_slice(&value.to_le_bytes());
    };
    // A header of `uuid` whose mappings (with slide info, V2) are
    // `(address, size, file offset, protection)`.
    let header = |uuid: [u8; 16], mappings: &[(u64, u64, u64, u32)], bytes: usize| {
        let mut file = vec![0u8; bytes];
        file[..16].copy_from_slice(b"dyld_v1  arm64e\0");
        put32(
            &mut file,
            offset_of!(Header, mapping_offset),
            size_of::<Header>() as u32,
        );
        file[offset_of!(Header, uuid)..][..16].copy_from_slice(&uuid);
        put32(
            &mut file,
            offset_of!(Header, mapping_with_slide_offset),
            MAPPINGS as u32,
        );
        put32(
            &mut file,
            offset_of!(Header, mapping_with_slide_count),
            mappings.len() as u32,
        );
        for (index, &(address, size, offset, protection)) in mappings.iter().enumerate() {
            let at = MAPPINGS + 56 * index;
            put64(&mut file, at, address);
            put64(&mut file, at + 8, size);
            put64(&mut file, at + 16, offset);
            put32(&mut file, at + 48, protection);
            put32(&mut file, at + 52, protection);
        }
        file
    };
    let linkedit_at = PAGE;
    let (text, linkedit) = image(
        CACHE_BASE,
        PAGE as u32,
        linkedit_at as u32,
        3,
        ["_alpha", "_beta"],
    );
    let mut main = header(
        MAIN,
        &[(CACHE_BASE, TEXT as u64, PAGE as u64, 5)],
        PAGE + TEXT,
    );
    put32(
        &mut main,
        offset_of!(Header, sub_cache_array_offset),
        SUBCACHES as u32,
    );
    put32(&mut main, offset_of!(Header, sub_cache_array_count), 1);
    main[SUBCACHES..][..16].copy_from_slice(&SUB);
    put64(
        &mut main,
        SUBCACHES + offset_of!(DyldSubCacheEntryV2<Le>, cache_vm_offset),
        TEXT as u64,
    );
    main[SUBCACHES + offset_of!(DyldSubCacheEntryV2<Le>, file_suffix)..][..3]
        .copy_from_slice(b".01");
    main[offset_of!(Header, symbol_file_uuid)..][..16].copy_from_slice(&SYMBOLS);
    put32(&mut main, offset_of!(Header, images_offset), IMAGES as u32);
    put32(&mut main, offset_of!(Header, images_count), 1);
    put64(&mut main, IMAGES, CACHE_BASE);
    put32(
        &mut main,
        IMAGES + offset_of!(DyldCacheImageInfo<Le>, path_file_offset),
        PATHS as u32,
    );
    main[PATHS..][..SPLIT_PATH.len()].copy_from_slice(SPLIT_PATH.as_bytes());
    main[PAGE..].copy_from_slice(&text);
    let mut sub = header(
        SUB,
        &[(
            CACHE_BASE + TEXT as u64,
            LINKEDIT as u64,
            linkedit_at as u64,
            1,
        )],
        linkedit_at + LINKEDIT,
    );
    sub[linkedit_at..].copy_from_slice(&linkedit);
    let symbols = header(SYMBOLS, &[], PAGE);
    [("", main), (".01", sub), (".symbols", symbols)]
}

/// The first 8 bytes of the SHA-256 of `page`, as the lines print them.
pub(super) fn key(page: &[u8]) -> u64 {
    use sha2::{Digest, Sha256};
    let digest = Sha256::digest(page);
    u64::from_be_bytes(digest[..8].try_into().unwrap())
}

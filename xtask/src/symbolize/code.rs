//! The code a log's page keys name: every executable 4 KiB page of the
//! dyld shared caches and Mach-O images given, hashed as the engine keys
//! the pages it runs (SHA-256 of the page's bytes, as `model.rs` in the
//! engine's `ahead` maps a pack's pages to a cache), and the images'
//! symbols and function starts, which name an address. Only the pages a
//! log names are kept; a page whose bytes more than one place holds is
//! ambiguous. A page found names its function from the file it was found
//! in: kernels of one KDK, kexts linked at 0 and two caches' images share
//! addresses.
use anyhow::{Context, Result, anyhow, bail, ensure};
use object::macho::{self, LinkeditDataCommand};
use object::read::macho::{DyldCache, MachOFile64};
use object::{Endianness, Object, ObjectSegment, ObjectSymbol, SegmentFlags, SymbolKind};
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

/// The bytes the engine keys a page by.
const PAGE: usize = 4096;
/// How deep a directory is searched.
const DEPTH: usize = 6;

/// Where a page key and offset are.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Place {
    /// At this unslid address of the `file`th file read.
    Found { file: usize, address: u64 },
    /// In a page whose bytes more than one place holds.
    Ambiguous,
    /// In no page given.
    Missing,
}

/// Where a wanted page's bytes are: one place, or more.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PageAt {
    One { file: usize, address: u64 },
    Many,
}

/// One file read: its name and its images' executable ranges `(start,
/// end, image)`, ascending.
#[derive(Debug, Default)]
struct File {
    name: String,
    ranges: Vec<(u64, u64, usize)>,
}

/// The function an address is in: its image's path, if an image holds it,
/// and a name, the symbol at the function's start or `sub_<start>` where no
/// symbol names it, with that start.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Named<'a> {
    pub image: Option<&'a str>,
    pub function: String,
    pub start: u64,
}

/// One image's executable ranges' names.
#[derive(Debug, Default)]
pub(super) struct Image {
    pub path: String,
    /// Code symbols and function starts, ascending.
    symbols: Vec<(u64, String)>,
    starts: Vec<u64>,
}

#[derive(Debug, Default)]
pub(super) struct Code {
    /// The wanted pages, by key.
    pages: HashMap<u64, PageAt>,
    /// The files read, in order.
    files: Vec<File>,
    pub images: Vec<Image>,
    /// What was read: each cache and image, for the report.
    pub loaded: Vec<String>,
    /// Keep every executable page, wanted or not (the oracle test).
    #[cfg(test)]
    every_page: bool,
}

impl Code {
    /// The code of `paths`: dyld shared caches (a main file, whose
    /// subcaches lie beside it, or a directory holding them) and Mach-O
    /// images, or directories of them; the pages `wanted` names.
    pub(super) fn load(paths: &[PathBuf], wanted: &HashSet<u64>) -> Result<Self> {
        let mut code = Self::default();
        code.read(paths, wanted)?;
        Ok(code)
    }

    /// Add the code of `paths` ([`Self::load`]).
    fn read(&mut self, paths: &[PathBuf], wanted: &HashSet<u64>) -> Result<()> {
        let mut files = Vec::new();
        for path in paths {
            ensure!(path.exists(), "{} does not exist", path.display());
            collect(path, DEPTH, &mut files)?;
        }
        files.sort();
        files.dedup();
        for file in files {
            self.add_path(&file, wanted)
                .with_context(|| format!("reading {}", file.display()))?;
        }
        Ok(())
    }

    /// Whether the page `key` is kept.
    fn keeps(&self, key: u64, wanted: &HashSet<u64>) -> bool {
        #[cfg(test)]
        if self.every_page {
            return true;
        }
        wanted.contains(&key)
    }

    /// Add `path` by its content: a main dyld cache file with its
    /// subcaches beside it, a 64-bit Mach-O, or nothing.
    fn add_path(&mut self, path: &Path, wanted: &HashSet<u64>) -> Result<()> {
        use std::io::Read;
        let name = path.display().to_string();
        let mut magic = [0u8; 8];
        let read = std::fs::File::open(path)?.read(&mut magic)?;
        if read < magic.len() {
            return Ok(());
        }
        if magic.starts_with(b"dyld_v1") {
            // A subcache (`.01`, `.symbols`) is read with its main file.
            let file_name = path.file_name().and_then(|name| name.to_str());
            if file_name.is_none_or(|file_name| file_name.contains('.')) {
                return Ok(());
            }
            let main = Mapped::open(path)?;
            let suffixes = DyldCache::<Endianness>::subcache_suffixes(&main[..])
                .map_err(|e| anyhow!("{e}"))?;
            let mut subcaches = Vec::new();
            for suffix in suffixes {
                let sub = PathBuf::from(format!("{}{suffix}", path.display()));
                subcaches.push(
                    Mapped::open(&sub)
                        .with_context(|| format!("{name} needs its subcache {}", sub.display()))?,
                );
            }
            let slices: Vec<&[u8]> = subcaches.iter().map(|sub| &sub[..]).collect();
            return self.add_cache(&main, &slices, &name, wanted);
        }
        if u32::from_le_bytes([magic[0], magic[1], magic[2], magic[3]]) == macho::MH_MAGIC_64 {
            let data = Mapped::open(path)?;
            return self.add_macho(&data, &name, wanted);
        }
        Ok(())
    }

    /// Add a standalone 64-bit Mach-O, read as `name`.
    pub(super) fn add_macho(
        &mut self,
        data: &[u8],
        name: &str,
        wanted: &HashSet<u64>,
    ) -> Result<()> {
        let file = MachOFile64::<Endianness>::parse(data).map_err(|e| anyhow!("{name}: {e}"))?;
        // A dSYM's segments hold no code, only the names its image has.
        if file.macho_header().filetype.get(file.endian()) == macho::MH_DSYM {
            return Ok(());
        }
        let Some(index) = self.start_file(name) else {
            return Ok(());
        };
        let before = self.pages.len();
        for segment in file.segments() {
            if executable(segment.flags()) {
                let bytes = segment.data().map_err(|e| anyhow!("{name}: {e}"))?;
                self.hash(index, segment.address(), bytes, wanted);
            }
        }
        let starts = function_starts(&file, |offset, size| data.get(offset..offset + size));
        self.add_image(index, &file, name, starts);
        self.loaded.push(format!(
            "{name}: 1 image, {} of {} wanted pages",
            self.pages.len() - before,
            wanted.len()
        ));
        Ok(())
    }

    /// Add a dyld shared cache: its main file and its subcaches, in the
    /// order the main file names them, `.symbols` last.
    pub(super) fn add_cache(
        &mut self,
        main: &[u8],
        subcaches: &[&[u8]],
        name: &str,
        wanted: &HashSet<u64>,
    ) -> Result<()> {
        let cache =
            DyldCache::<Endianness>::parse(main, subcaches).map_err(|e| anyhow!("{name}: {e}"))?;
        let Some(index) = self.start_file(name) else {
            return Ok(());
        };
        let before = self.pages.len();
        for mapping in cache.mappings() {
            if mapping.init_prot() & macho::VM_PROT_EXECUTE != 0 {
                let bytes = mapping.data().map_err(|e| anyhow!("{name}: {e}"))?;
                self.hash(index, mapping.address(), bytes, wanted);
            }
        }
        let mut images = 0;
        for image in cache.images() {
            let (Ok(path), Ok(object::File::MachO64(file))) = (image.path(), image.parse_object())
            else {
                continue;
            };
            // An image's linkedit offsets are offsets in the cache file
            // that holds its __LINKEDIT.
            let linkedit = file
                .segments()
                .find(|segment| segment.name().ok().flatten() == Some("__LINKEDIT"))
                .and_then(|segment| cache.data_and_offset_for_address(segment.address()))
                .map(|(data, _)| data);
            let starts = function_starts(&file, |offset, size| {
                linkedit.and_then(|data| data.get(offset..offset + size))
            });
            self.add_image(index, &file, path, starts);
            images += 1;
        }
        self.loaded.push(format!(
            "{name}: {images} image{}, {} of {} wanted pages",
            if images == 1 { "" } else { "s" },
            self.pages.len() - before,
            wanted.len()
        ));
        Ok(())
    }

    /// The index of a new file read as `name`, or `None` for a name read
    /// before: the same file twice is one place.
    fn start_file(&mut self, name: &str) -> Option<usize> {
        if self.files.iter().any(|file| file.name == name) {
            return None;
        }
        self.files.push(File {
            name: name.to_owned(),
            ranges: Vec::new(),
        });
        Some(self.files.len() - 1)
    }

    /// Keep the place of each page of `bytes`, at `address` in the `file`th
    /// file, whose key `wanted` names.
    fn hash(&mut self, file: usize, address: u64, bytes: &[u8], wanted: &HashSet<u64>) {
        if wanted.is_empty() && !self.keeps(0, wanted) {
            return;
        }
        for (index, key) in keys(bytes).into_iter().enumerate() {
            if !self.keeps(key, wanted) {
                continue;
            }
            let at = PageAt::One {
                file,
                address: address + (index * PAGE) as u64,
            };
            self.pages
                .entry(key)
                .and_modify(|held| {
                    if *held != at {
                        *held = PageAt::Many;
                    }
                })
                .or_insert(at);
        }
    }

    /// Add an image of the `file`th file: its executable ranges, code
    /// symbols and function starts.
    fn add_image<'data, R: object::ReadRef<'data>>(
        &mut self,
        file_index: usize,
        file: &MachOFile64<'data, Endianness, R>,
        path: &str,
        mut starts: Vec<u64>,
    ) {
        let index = self.images.len();
        let ranges = &mut self.files[file_index].ranges;
        for segment in file.segments() {
            if executable(segment.flags()) && segment.size() != 0 {
                let start = segment.address();
                ranges.push((start, start + segment.size(), index));
            }
        }
        ranges.sort_unstable();
        let mut symbols: Vec<(u64, String)> = file
            .symbols()
            .filter(|symbol| symbol.kind() == SymbolKind::Text && symbol.is_definition())
            .filter_map(|symbol| Some((symbol.address(), symbol.name().ok()?.to_owned())))
            .filter(|(_, name)| !name.is_empty())
            .collect();
        symbols.sort();
        symbols.dedup_by_key(|symbol| symbol.0);
        starts.sort_unstable();
        starts.dedup();
        self.images.push(Image {
            path: path.to_owned(),
            symbols,
            starts,
        });
    }

    /// Where `offset` in the page `key` is.
    pub(super) fn place(&self, key: u64, offset: u16) -> Place {
        match self.pages.get(&key) {
            Some(&PageAt::One { file, address }) => Place::Found {
                file,
                address: address + u64::from(offset),
            },
            Some(PageAt::Many) => Place::Ambiguous,
            None => Place::Missing,
        }
    }

    /// The function at `address` of the `file`th file read: only that
    /// file's images name it.
    pub(super) fn name(&self, file: usize, address: u64) -> Named<'_> {
        let unnamed = Named {
            image: None,
            function: format!("{address:#x}"),
            start: address,
        };
        let Some(file) = self.files.get(file) else {
            return unnamed;
        };
        // The innermost range that holds the address, the last to start
        // at or below it: ranges may nest.
        let below = file.ranges.partition_point(|range| range.0 <= address);
        let range = file.ranges[..below]
            .iter()
            .rev()
            .find(|range| address < range.1);
        let Some(&(_, _, index)) = range else {
            return unnamed;
        };
        let image = &self.images[index];
        let symbol = image
            .symbols
            .partition_point(|symbol| symbol.0 <= address)
            .checked_sub(1)
            .map(|index| &image.symbols[index]);
        let start = image
            .starts
            .partition_point(|&start| start <= address)
            .checked_sub(1)
            .map(|index| image.starts[index]);
        // A function start past the symbol before the address is a
        // function no symbol names.
        let (function, start) = match (symbol, start) {
            (Some((at, _)), Some(start)) if *at < start => (format!("sub_{start:x}"), start),
            (Some((at, name)), _) => (name.clone(), *at),
            (None, Some(start)) => (format!("sub_{start:x}"), start),
            (None, None) => (format!("{address:#x}"), address),
        };
        Named {
            image: Some(&image.path),
            function,
            start,
        }
    }
}

/// Whether a segment's initial protection lets it execute.
fn executable(flags: SegmentFlags) -> bool {
    matches!(flags, SegmentFlags::MachO { initprot, .. } if initprot & macho::VM_PROT_EXECUTE != 0)
}

/// The first 8 bytes of the SHA-256 of each whole page of `bytes`, in
/// their order, hashed on up to 8 threads: a shared cache's text is
/// gigabytes.
fn keys(bytes: &[u8]) -> Vec<u64> {
    use sha2::{Digest, Sha256};
    let pages: Vec<&[u8; PAGE]> = bytes.as_chunks::<PAGE>().0.iter().collect();
    let threads = std::thread::available_parallelism()
        .map_or(1, |n| n.get())
        .min(8);
    let share = pages.len().div_ceil(threads).max(1);
    std::thread::scope(|scope| {
        let workers: Vec<_> = pages
            .chunks(share)
            .map(|pages| {
                scope.spawn(move || {
                    pages
                        .iter()
                        .map(|page| {
                            let digest = Sha256::digest(page);
                            let mut first = [0; 8];
                            first.copy_from_slice(&digest[..8]);
                            u64::from_be_bytes(first)
                        })
                        .collect::<Vec<u64>>()
                })
            })
            .collect();
        workers
            .into_iter()
            .flat_map(|worker| worker.join().expect("a hashing thread"))
            .collect()
    })
}

/// The function starts `LC_FUNCTION_STARTS` gives, its bytes read by
/// `read(file offset, size)` from the file that holds the image's
/// `__LINKEDIT`: ULEB128 deltas from the `__TEXT` segment's address, to a
/// zero.
fn function_starts<'data, 'linkedit, R: object::ReadRef<'data>>(
    file: &MachOFile64<'data, Endianness, R>,
    read: impl Fn(usize, usize) -> Option<&'linkedit [u8]>,
) -> Vec<u64> {
    let endian = file.endian();
    let text = file
        .segments()
        .find(|segment| segment.name().ok().flatten() == Some("__TEXT"))
        .map(|segment| segment.address());
    let (Some(text), Ok(mut commands)) = (text, file.macho_load_commands()) else {
        return Vec::new();
    };
    let mut starts = Vec::new();
    while let Ok(Some(command)) = commands.next() {
        if command.cmd() != macho::LC_FUNCTION_STARTS {
            continue;
        }
        let Ok(data) = command.data::<LinkeditDataCommand<Endianness>>() else {
            continue;
        };
        let (offset, size) = (
            data.dataoff.get(endian) as usize,
            data.datasize.get(endian) as usize,
        );
        let Some(bytes) = read(offset, size) else {
            continue;
        };
        let mut address = text;
        let mut at = 0;
        while at < bytes.len() {
            let (mut delta, mut shift) = (0u64, 0u32);
            while let Some(&byte) = bytes.get(at) {
                at += 1;
                if shift < 64 {
                    delta |= u64::from(byte & 0x7f) << shift;
                }
                shift += 7;
                if byte & 0x80 == 0 {
                    break;
                }
            }
            if delta == 0 {
                break;
            }
            address = address.wrapping_add(delta);
            starts.push(address);
        }
    }
    starts
}

/// The files under `path`, at most `depth` directories down.
fn collect(path: &Path, depth: usize, files: &mut Vec<PathBuf>) -> Result<()> {
    let metadata = std::fs::metadata(path)?;
    if metadata.is_file() {
        files.push(path.to_path_buf());
    } else if metadata.is_dir() && depth > 0 {
        for entry in std::fs::read_dir(path)? {
            collect(&entry?.path(), depth - 1, files)?;
        }
    }
    Ok(())
}

/// A file's bytes, mapped where the host maps files: a shared cache's are
/// gigabytes.
struct Mapped {
    #[cfg(unix)]
    pointer: *mut libc::c_void,
    #[cfg(unix)]
    length: usize,
    #[cfg(not(unix))]
    bytes: Vec<u8>,
}

impl Mapped {
    #[cfg(unix)]
    fn open(path: &Path) -> Result<Self> {
        use std::os::fd::AsRawFd;
        let file = std::fs::File::open(path)?;
        let length = usize::try_from(file.metadata()?.len())?;
        if length == 0 {
            return Ok(Self {
                pointer: core::ptr::null_mut(),
                length,
            });
        }
        // SAFETY: a private read-only mapping of a file opened for
        // reading, unmapped at drop; nothing writes through it.
        let pointer = unsafe {
            libc::mmap(
                core::ptr::null_mut(),
                length,
                libc::PROT_READ,
                libc::MAP_PRIVATE,
                file.as_raw_fd(),
                0,
            )
        };
        if pointer == libc::MAP_FAILED {
            bail!(
                "mapping {}: {}",
                path.display(),
                std::io::Error::last_os_error()
            );
        }
        Ok(Self { pointer, length })
    }

    #[cfg(not(unix))]
    fn open(path: &Path) -> Result<Self> {
        Ok(Self {
            bytes: std::fs::read(path)?,
        })
    }
}

impl core::ops::Deref for Mapped {
    type Target = [u8];

    #[cfg(unix)]
    fn deref(&self) -> &[u8] {
        if self.length == 0 {
            return &[];
        }
        // SAFETY: mapped readable for `length` bytes until drop.
        unsafe { core::slice::from_raw_parts(self.pointer.cast::<u8>(), self.length) }
    }

    #[cfg(not(unix))]
    fn deref(&self) -> &[u8] {
        &self.bytes
    }
}

#[cfg(unix)]
impl Drop for Mapped {
    fn drop(&mut self) {
        if self.length != 0 {
            // SAFETY: the mapping `open` made, unmapped once.
            unsafe { libc::munmap(self.pointer, self.length) };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::fixtures::{self, ALPHA, BETA, CACHE_BASE, CACHE_PATH, PAGE, UNNAMED};
    use super::*;

    const BASE: u64 = 0x7000_0000;

    fn keys(file: &[u8], at: usize) -> [u64; 2] {
        [
            fixtures::key(&file[at + PAGE..at + 2 * PAGE]),
            fixtures::key(&file[at + 2 * PAGE..at + 3 * PAGE]),
        ]
    }

    /// The address a place names, wherever it was found.
    fn address(place: Place) -> Option<u64> {
        match place {
            Place::Found { address, .. } => Some(address),
            _ => None,
        }
    }

    #[test]
    fn a_macho_names_its_pages_and_functions() {
        let file = fixtures::macho(BASE);
        let [first, second] = keys(&file, 0);
        let mut code = Code::default();
        code.add_macho(
            &file,
            "libfixture.dylib",
            &HashSet::from([first, second, 42]),
        )
        .unwrap();
        assert_eq!(
            code.place(first, 0x10),
            Place::Found {
                file: 0,
                address: BASE + ALPHA + 0x10
            }
        );
        assert_eq!(address(code.place(second, 0x20)), Some(BASE + 0x2020));
        assert_eq!(code.place(42, 0), Place::Missing);
        // The header's page was not wanted.
        assert_eq!(code.place(fixtures::key(&file[..PAGE]), 0), Place::Missing);
        let image = Some("libfixture.dylib");
        assert_eq!(
            code.name(0, BASE + ALPHA + 0x10),
            Named {
                image,
                function: "_alpha".into(),
                start: BASE + ALPHA
            }
        );
        // A function start past the symbol before it: a function no
        // symbol names.
        assert_eq!(
            code.name(0, BASE + UNNAMED + 0x100),
            Named {
                image,
                function: format!("sub_{:x}", BASE + UNNAMED),
                start: BASE + UNNAMED
            }
        );
        assert_eq!(code.name(0, BASE + 0x2020).function, "_beta");
        assert_eq!(code.name(0, BASE + 0x2020).start, BASE + BETA);
        // Outside every image.
        assert_eq!(code.name(0, 0x10).image, None);
        assert_eq!(
            code.loaded,
            ["libfixture.dylib: 1 image, 2 of 3 wanted pages"]
        );
    }

    #[test]
    fn a_dyld_cache_names_its_images_pages_and_functions() {
        let file = fixtures::cache();
        let [first, second] = keys(&file, PAGE);
        let mut code = Code::default();
        code.add_cache(
            &file,
            &[],
            "dyld_shared_cache_arm64e",
            &HashSet::from([first, second]),
        )
        .unwrap();
        assert_eq!(
            address(code.place(first, 0x10)),
            Some(CACHE_BASE + ALPHA + 0x10)
        );
        assert_eq!(address(code.place(second, 0x20)), Some(CACHE_BASE + 0x2020));
        assert_eq!(
            code.name(0, CACHE_BASE + ALPHA + 0x10),
            Named {
                image: Some(CACHE_PATH),
                function: "_alpha".into(),
                start: CACHE_BASE + ALPHA
            }
        );
        assert_eq!(
            code.name(0, CACHE_BASE + UNNAMED).function,
            format!("sub_{:x}", CACHE_BASE + UNNAMED)
        );
        assert_eq!(code.name(0, CACHE_BASE + 0x2ffc).function, "_beta");
        assert_eq!(
            code.loaded,
            ["dyld_shared_cache_arm64e: 1 image, 2 of 2 wanted pages"]
        );
    }

    /// The same bytes at two addresses: a key that names either is
    /// ambiguous.
    #[test]
    fn a_page_two_addresses_hold_is_ambiguous() {
        let one = fixtures::macho(BASE);
        let other = fixtures::macho(BASE + 0x10_0000);
        let [first, _] = keys(&one, 0);
        let wanted = HashSet::from([first]);
        let mut code = Code::default();
        code.add_macho(&one, "one", &wanted).unwrap();
        code.add_macho(&other, "other", &wanted).unwrap();
        assert_eq!(code.place(first, 0), Place::Ambiguous);
        // The same file read twice is one place.
        let mut again = Code::default();
        again.add_macho(&one, "one", &wanted).unwrap();
        again.add_macho(&one, "one", &wanted).unwrap();
        assert_eq!(
            again.place(first, 0),
            Place::Found {
                file: 0,
                address: BASE + ALPHA
            }
        );
        assert_eq!(again.loaded.len(), 1);
    }

    /// Images at overlapping addresses in different files (the kernels of
    /// one KDK, kexts linked at 0, two caches): a page names its function
    /// from the file its bytes were found in. A dSYM, which holds names but
    /// no code, is not read.
    #[test]
    fn overlapping_images_name_functions_from_their_own_file() {
        let one = fixtures::macho_named(BASE, 0, ["_alpha", "_beta"]);
        let other = fixtures::macho_named(BASE, 2, ["_gamma", "_delta"]);
        let shifted = fixtures::macho_named(BASE + 0x1000, 4, ["_epsilon", "_zeta"]);
        let [one_first, one_second] = keys(&one, 0);
        let [other_first, _] = keys(&other, 0);
        let [shifted_first, _] = keys(&shifted, 0);
        let wanted = HashSet::from([one_first, one_second, other_first, shifted_first]);
        let mut dsym = fixtures::macho_named(BASE, 6, ["_dsym", "_names"]);
        dsym[12] = macho::MH_DSYM as u8;
        let mut code = Code::default();
        code.add_macho(&dsym, "kernel.dSYM", &wanted).unwrap();
        code.add_macho(&one, "kernel.release.t6000", &wanted)
            .unwrap();
        code.add_macho(&other, "kernel.release.t8112", &wanted)
            .unwrap();
        code.add_macho(&shifted, "kext", &wanted).unwrap();
        assert_eq!(code.loaded.len(), 3, "{:?}", code.loaded);
        let named = |key, offset| match code.place(key, offset) {
            Place::Found { file, address } => code.name(file, address),
            place => panic!("{place:?}"),
        };
        assert_eq!(named(one_first, 0x10).function, "_alpha");
        assert_eq!(named(one_first, 0x10).image, Some("kernel.release.t6000"));
        assert_eq!(named(other_first, 0x10).function, "_gamma");
        assert_eq!(named(other_first, 0x10).image, Some("kernel.release.t8112"));
        assert_eq!(named(one_second, 0x20).function, "_beta");
        // The shifted image's first code page is the others' second.
        assert_eq!(named(shifted_first, 0x10).function, "_epsilon");
        assert_eq!(named(shifted_first, 0x10).image, Some("kext"));
    }

    /// Paths load by content: a directory's main cache and its subcaches
    /// once, a Mach-O, and nothing of other files.
    #[test]
    fn paths_load_caches_and_images_by_content() {
        let directory = tempfile::tempdir().unwrap();
        let cache = fixtures::cache();
        let macho = fixtures::macho(BASE);
        std::fs::write(directory.path().join("dyld_shared_cache_arm64e"), &cache).unwrap();
        std::fs::write(
            directory.path().join("dyld_shared_cache_arm64e.map"),
            "text",
        )
        .unwrap();
        std::fs::create_dir(directory.path().join("kernel")).unwrap();
        std::fs::write(directory.path().join("kernel/kernel.fixture"), &macho).unwrap();
        std::fs::write(directory.path().join("notes.txt"), "not code").unwrap();
        let [cached, _] = keys(&cache, PAGE);
        let [kernel, _] = keys(&macho, 0);
        let code = Code::load(
            &[directory.path().to_path_buf()],
            &HashSet::from([cached, kernel]),
        )
        .unwrap();
        assert_eq!(address(code.place(cached, 4)), Some(CACHE_BASE + ALPHA + 4));
        assert_eq!(address(code.place(kernel, 4)), Some(BASE + ALPHA + 4));
        assert_eq!(code.loaded.len(), 2, "{:?}", code.loaded);
        // A missing path is refused.
        let missing = directory.path().join("absent");
        assert!(Code::load(&[missing], &HashSet::new()).is_err());
    }

    /// The guest's own dyld shared cache (`STELA_SYMBOLIZE_DSC`, its main
    /// file or the directory holding it, its subcaches and `.symbols`):
    /// its images parse, almost all of its code pages are unique, and the
    /// page key and offset of `objc_msgSend` and `_platform_memmove` name
    /// them back in their images. Whether a guest's RAM holds these bytes
    /// at run time only a boot shows.
    #[test]
    #[ignore = "oracle: STELA_SYMBOLIZE_DSC"]
    fn a_real_cache_names_its_own_functions_back() {
        let path =
            PathBuf::from(std::env::var_os("STELA_SYMBOLIZE_DSC").expect("STELA_SYMBOLIZE_DSC"));
        let mut code = Code {
            every_page: true,
            ..Code::default()
        };
        code.read(&[path], &HashSet::new()).unwrap();
        println!("{:?}", code.loaded);
        assert!(code.images.len() > 1000, "{} images", code.images.len());
        let unique = code
            .pages
            .values()
            .filter(|at| **at != PageAt::Many)
            .count();
        assert!(
            unique * 10 > code.pages.len() * 9,
            "{unique} of {} code pages unique",
            code.pages.len()
        );
        let by_address: HashMap<u64, u64> = code
            .pages
            .iter()
            .filter_map(|(&key, &at)| match at {
                PageAt::One { address, .. } => Some((address, key)),
                PageAt::Many => None,
            })
            .collect();
        for wanted in ["_objc_msgSend", "_platform_memmove"] {
            let (image, address) = code
                .images
                .iter()
                .find_map(|image| {
                    let symbol = image
                        .symbols
                        .iter()
                        .find(|(_, name)| name.ends_with(wanted))?;
                    Some((image.path.clone(), symbol.0))
                })
                .unwrap_or_else(|| panic!("no symbol ends with {wanted}"));
            let page = address & !(PAGE as u64 - 1);
            let key = *by_address
                .get(&page)
                .unwrap_or_else(|| panic!("{wanted}'s page {page:#x} is ambiguous or not code"));
            let offset = (address - page) as u16;
            let Place::Found { file, address: at } = code.place(key, offset) else {
                panic!("{wanted} not found");
            };
            assert_eq!(at, address, "{wanted}");
            let named = code.name(file, address);
            assert!(named.function.ends_with(wanted), "{wanted}: {named:?}");
            assert_eq!(named.image, Some(image.as_str()), "{wanted}");
            println!("{wanted}: {address:#x} in {image}, page {key:016x}+{offset:x}");
        }
    }
}

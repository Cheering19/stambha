//! Install the UEFI mount on a USB's EFI system partition as its
//! removable-media boot loader, `\EFI\BOOT\BOOTX64.EFI`, which a PC's
//! firmware starts when it boots from the stick; no other boot manager is
//! involved.
//!
//! `install` copies `stela.efi` there and writes the mount's
//! `EFI/stela/stela.cfg` and `EFI/stela/source.bin`. A partition that
//! already boots another loader is refused and left unchanged: the loader
//! there is replaced only when it is a `stela.efi` (it carries a mount
//! interface marker, [`mount_interface::find`]). Files are replaced
//! atomically. `uninstall` removes what `install` wrote.
//!
//! With `--ipsw`, `install` also writes a restore payload for `stela.efi`
//! under `EFI/stela/images/<build>/` on the ESP or another volume of the
//! same USB: FAT32-safe slices of the model's members, or the whole IPSW,
//! and `payload.json` with every member's SHA-256 and the public AEA key
//! responses. `uninstall` leaves payloads alone.
pub use crate::restore::payload::{
    PayloadLayout, SYNTHETIC_BUILD, SYNTHETIC_MODEL, WriteOptions as PayloadOptions,
    synthetic_ipsw, write as write_payload,
};
use anyhow::{Context, Result, bail, ensure};
use clap::{Args, Subcommand};
use serde_json::json;
use std::{
    fs,
    io::Write,
    path::{Path, PathBuf},
};
use stela_loader::mount_interface;
use stela_runtime::pc::boot_log;

use stela_formats::payload::MAX_SLICE_BYTES;

/// The removable-media boot loader path for x64 (UEFI 2.10, Removable Media
/// Boot Behavior).
pub const LOADER: &str = "EFI/BOOT/BOOTX64.EFI";
const LOADER_DIRECTORY: &str = "EFI/BOOT";
const MOUNT: &str = "EFI/stela";
pub(crate) const OPTIONS: &str = "EFI/stela/stela.cfg";
/// `mount_interface::BOOT_LOG_FILE`.
pub(crate) const BOOT_LOG: &str = "EFI/stela/boot.log";
const SOURCE: &str = "EFI/stela/source.bin";

#[derive(Subcommand)]
pub enum EfiCommand {
    /// Install `stela.efi` as an EFI partition's removable-media boot loader.
    Install(Install),
    /// Remove everything `install` added.
    Uninstall(Uninstall),
}

impl EfiCommand {
    pub fn run(self) -> Result<()> {
        let report = match self {
            Self::Install(command) => install(&command)?,
            Self::Uninstall(command) => uninstall(&command.esp, false)?,
        };
        println!("{report:#}");
        Ok(())
    }
}

#[derive(Args)]
pub struct Install {
    /// The EFI system partition root.
    #[arg(long)]
    pub esp: PathBuf,
    /// The mount application, `stela.efi`.
    #[arg(long)]
    pub image: PathBuf,
    /// Original ARM bytes; omit to keep the installed `source.bin`.
    #[arg(long)]
    pub source: Option<PathBuf>,
    /// Mount options for `stela.cfg`; omit to keep the installed file.
    #[arg(long, allow_hyphen_values = true)]
    pub options: Option<String>,
    /// Write this IPSW's restore payload as well.
    #[arg(long)]
    pub ipsw: Option<PathBuf>,
    /// The Mac model whose identities the payload carries.
    #[arg(long, default_value = crate::setup::PINNED_MODEL, requires = "ipsw")]
    pub model: String,
    /// The volume root for the payload; defaults to `--esp`.
    #[arg(long, requires = "ipsw")]
    pub payload_root: Option<PathBuf>,
    #[arg(long, value_enum, default_value_t = PayloadLayout::Split, requires = "ipsw")]
    pub payload_layout: PayloadLayout,
    /// Largest slice of a split payload; FAT32 allows 4 GiB - 1.
    #[arg(long, default_value_t = MAX_SLICE_BYTES, hide = true)]
    pub slice_bytes: u64,
    /// Leave out the public AEA keys; the restore then needs the network.
    #[arg(long, requires = "ipsw")]
    pub no_aea_keys: bool,
}

#[derive(Args)]
pub struct Uninstall {
    /// The EFI system partition root.
    #[arg(long)]
    esp: PathBuf,
}

/// Replace `path` through a sibling temporary file, so a failed write leaves
/// the old file in place; with `fresh`, only when nothing is there yet.
pub(crate) fn write_atomic(path: &Path, bytes: &[u8], fresh: bool) -> Result<()> {
    let parent = path.parent().context("a file without a parent directory")?;
    (|| -> Result<()> {
        let mut file = tempfile::NamedTempFile::new_in(parent)?;
        file.write_all(bytes)?;
        file.as_file().sync_all()?;
        match fresh {
            true => persist_new(file, path)?,
            false => drop(file.persist(path).map_err(|error| error.error)?),
        };
        Ok(())
    })()
    .with_context(|| format!("cannot write {}", path.display()))?;
    // FAT may refuse to sync a directory; the file itself is already synced.
    let _ = fs::File::open(parent).and_then(|directory| directory.sync_all());
    Ok(())
}

/// Give the synced temporary `file` the name `path`, never replacing a
/// `path` that exists ([`rename_new`]); a refused `file` is removed.
pub(crate) fn persist_new(file: tempfile::NamedTempFile, path: &Path) -> Result<()> {
    let temporary = file.into_temp_path();
    rename_new(&temporary, path)?;
    // It is `path` now: nothing is left to remove.
    let _ = temporary.keep();
    Ok(())
}

/// Give the finished file `from` the name `to`, never replacing a `to` that
/// exists: by a rename that refuses to replace where the file system has
/// one, else, as on FAT and exFAT under macOS (which refuse that rename with
/// ENOTSUP and have no hard links either), by a plain rename once `to` is
/// checked to be absent. A refused `from` stays where it is.
pub(crate) fn rename_new(from: &Path, to: &Path) -> Result<()> {
    rename_new_with(from, to, |from, to| {
        tempfile::TempPath::try_from_path(from)?
            .persist_noclobber(to)
            .map_err(|error| {
                // Kept for the caller, not removed with the handle.
                let _ = error.path.keep();
                error.error
            })
    })
}

fn rename_new_with(
    from: &Path,
    to: &Path,
    no_replace: impl FnOnce(&Path, &Path) -> std::io::Result<()>,
) -> Result<()> {
    match no_replace(from, to) {
        Ok(()) => return Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {
            bail!("{} already exists", to.display())
        }
        Err(_) => {}
    }
    match fs::symlink_metadata(to) {
        Ok(_) => bail!("{} already exists", to.display()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => {
            return Err(error).with_context(|| format!("cannot inspect {}", to.display()));
        }
    }
    fs::rename(from, to)
        .with_context(|| format!("cannot rename {} to {}", from.display(), to.display()))
}

/// Whether the partition's boot loader is a `stela.efi`; `None` when it has
/// none.
pub(crate) fn loader_is_stela(esp: &Path) -> Result<Option<bool>> {
    let path = esp.join(LOADER);
    match fs::read(&path) {
        Ok(image) => Ok(Some(mount_interface::find(&image).is_some())),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(error) => Err(error).with_context(|| format!("cannot read {}", path.display())),
    }
}

/// Refuse a partition whose boot loader is not Stela's, changing nothing.
pub(crate) fn check_loader(esp: &Path) -> Result<()> {
    ensure!(
        loader_is_stela(esp)? != Some(false),
        "{} already boots another loader ({}). Stambha boots the stick itself, as its only boot \
         loader, and does not replace another one: use a stick whose EFI partition has no \
         {LOADER}, or erase the stick first.",
        esp.display(),
        esp.join(LOADER).display()
    );
    Ok(())
}

/// Install or replace the mount, and return a report of what changed.
pub fn install(command: &Install) -> Result<serde_json::Value> {
    let esp = &command.esp;
    let source = match &command.source {
        Some(source) => {
            Some(fs::read(source).with_context(|| format!("cannot read {}", source.display()))?)
        }
        None if esp.join(SOURCE).is_file() => None,
        None => bail!("pass --source: {} has no {SOURCE}", esp.display()),
    };
    // Check the partition before writing the payload, which goes before the
    // loader, so the stick never boots Stela with a half-written payload.
    check_loader(esp)?;
    let payload = match &command.ipsw {
        Some(ipsw) => Some(crate::restore::payload::write(
            &crate::restore::payload::WriteOptions {
                image: ipsw,
                model: &command.model,
                root: command.payload_root.as_deref().unwrap_or(esp),
                layout: command.payload_layout,
                slice_bytes: command.slice_bytes,
                fetch_keys: !command.no_aea_keys,
            },
        )?),
        None => None,
    };
    let image = fs::read(&command.image)
        .with_context(|| format!("cannot read {}", command.image.display()))?;
    let log = keep_boot_log(&crate::setup::host::System, esp, image.len() as u64);
    let mut report = add_mount(esp, &image, source.as_deref(), command.options.as_deref())?;
    report["payload"] = json!(payload);
    report["boot_log_change"] = json!(log.said());
    Ok(report)
}

/// Copy the ARM source when given and the options, then the loader, over
/// whatever loader is there: callers check it first, and make or grow the
/// log file before it ([`keep_boot_log`]). `stela create` uses this without
/// a source: install mode runs none.
pub(crate) fn add_mount(
    esp: &Path,
    image: &[u8],
    source: Option<&[u8]>,
    options: Option<&str>,
) -> Result<serde_json::Value> {
    ensure!(image.starts_with(b"MZ"), "the image is not a PE image");
    ensure!(
        mount_interface::find(image).is_some(),
        "the image is not a stela.efi: it carries no mount interface marker"
    );
    fs::create_dir_all(esp.join(MOUNT))?;
    fs::create_dir_all(esp.join(LOADER_DIRECTORY))?;
    if let Some(source) = source {
        write_atomic(&esp.join(SOURCE), source, false)?;
    }
    if let Some(options) = options {
        write_atomic(&esp.join(OPTIONS), options.as_bytes(), false)?;
    }
    // The loader goes in last, so the stick never boots it without its files.
    write_atomic(&esp.join(LOADER), image, false)?;
    Ok(json!({
        "loader": esp.join(LOADER),
        "source": esp.join(SOURCE).is_file().then(|| esp.join(SOURCE)),
        "options": esp.join(OPTIONS).is_file().then(|| esp.join(OPTIONS)),
        "boot_log": esp.join(BOOT_LOG),
    }))
}

/// Room an EFI partition keeps beside a longer log file: a copy of the
/// loader, which an update writes before it replaces the old one, and this
/// margin.
const LOADER_MARGIN: u64 = 16 << 20;

/// The length to give a `current`-byte log file (0: none yet) on an EFI
/// partition with `free` bytes free, whose `installed`-byte loader (0: none
/// yet) a `loader`-byte one replaces: the longest of
/// [`boot_log::FILE_BYTES`], 16 MiB and 8 MiB that can be written beside
/// the old file and still leaves room for another copy of the loader and
/// [`LOADER_MARGIN`] once the new loader is in place, as the next update
/// needs; else [`boot_log::LOG_BYTES`] for a new file or one shorter than
/// any log (which no boot writes). A file that as it is would leave the
/// next update no room for its copy of a loader this long, as a 32 MiB log
/// beside a loader of 84 MiB or more does, gets the longest of 16 MiB,
/// 8 MiB and [`boot_log::LOG_BYTES`] shorter than it that does. `None`
/// leaves the file as it is.
pub(crate) fn boot_log_bytes(current: u64, free: u64, loader: u64, installed: u64) -> Option<u64> {
    let fits = |bytes: u64| {
        let left = free
            .saturating_add(current)
            .checked_sub(bytes)
            .and_then(|left| left.checked_sub(loader.saturating_sub(installed)));
        free >= bytes && left.is_some_and(|left| left >= loader.saturating_add(LOADER_MARGIN))
    };
    let longest = [boot_log::FILE_BYTES, 16 << 20, 8 << 20]
        .into_iter()
        .find(|&bytes| bytes > current && fits(bytes));
    match longest {
        Some(bytes) => Some(bytes),
        None if current < boot_log::LOG_BYTES => Some(boot_log::LOG_BYTES),
        None if !fits(current) => [16 << 20, 8 << 20, boot_log::LOG_BYTES]
            .into_iter()
            .find(|&bytes| bytes < current && fits(bytes)),
        None => None,
    }
}

/// Why the system refused a file, as it says it, without the paths an
/// error names on the way.
fn reason(error: &anyhow::Error) -> String {
    let io = || {
        error
            .chain()
            .filter_map(|cause| cause.downcast_ref::<std::io::Error>())
    };
    match (io().find_map(std::io::Error::raw_os_error), io().next()) {
        (Some(code), _) => std::io::Error::from_raw_os_error(code).to_string(),
        (None, Some(error)) => error.kind().to_string(),
        (None, None) => error.root_cause().to_string(),
    }
}

/// What [`keep_boot_log`] did with the stick's log file.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum LogFile {
    /// It is as long as `stela` makes it: left as it is.
    Kept,
    /// Made anew, this long: there was none, or none a boot writes.
    Made(u64),
    /// Made longer with every line and boot it held.
    Grown { from: u64, to: u64 },
    /// Made shorter, its newest lines and its newest boots' records kept, so
    /// that the EFI partition keeps room for the next update's copy of the
    /// loader ([`boot_log_bytes`]).
    Shrunk { from: u64, to: u64 },
    /// Left this long: the EFI partition lacks the room for a longer one.
    NoRoom(u64),
    /// Left this long: the EFI partition's free room could not be read.
    RoomUnknown(u64),
    /// Left as it is: it does not read as a log, and `stela log` shows its
    /// lines as they lie.
    Unreadable(u64),
    /// The stick refused the new file; the one before (`bytes` long, 0 for
    /// none) is left as it was.
    Failed { bytes: u64, error: String },
}

impl LogFile {
    /// What a report says of it after `boot log`, in a few plain words;
    /// nothing when it is as `stela` makes it.
    pub(crate) fn said(&self) -> Option<String> {
        use crate::setup::host::bytes_text as size;
        let short = |bytes: u64| match bytes < boot_log::FILE_BYTES {
            true => ", as the EFI partition has no room for a longer one",
            false => "",
        };
        Some(match self {
            Self::Kept => return None,
            Self::Made(bytes) if *bytes == boot_log::FILE_BYTES => {
                format!("made ({})", size(*bytes))
            }
            Self::Made(bytes) => format!("made ({}){}", size(*bytes), short(*bytes)),
            Self::Grown { from, to } => format!(
                "made longer, {} from {}, with every boot it held{}",
                size(*to),
                size(*from),
                short(*to)
            ),
            Self::Shrunk { from, to } => format!(
                "made shorter, {} from {}, with its newest lines and its last boots, so the EFI \
                 partition keeps room for the next update",
                size(*to),
                size(*from)
            ),
            Self::NoRoom(bytes) => format!(
                "stays {}: the EFI partition has no room for a longer one",
                size(*bytes)
            ),
            Self::RoomUnknown(bytes) => format!(
                "stays {}: the EFI partition's free room could not be read",
                size(*bytes)
            ),
            Self::Unreadable(bytes) => format!(
                "left as it is ({}): it does not read as a log; stambha log still shows its lines",
                size(*bytes)
            ),
            Self::Failed { bytes: 0, error } => {
                format!("none: the stick refused the file ({error})")
            }
            Self::Failed { bytes, error } => format!(
                "stays {} as it was: the stick refused the new file ({error})",
                size(*bytes)
            ),
        })
    }
}

/// Make the USB's log file, or make a shorter one longer with its lines
/// ([`boot_log::relayout`]), or one too long for the next update's copy of
/// the loader shorter with its newest lines and boots, as long as
/// [`boot_log_bytes`] allows for a
/// `loader`-byte loader on the EFI partition `host` says is at `esp`. A
/// file whose header does not decode keeps its bytes for `stela log` to
/// read raw; a blank one, or one shorter than any log, is made anew. A
/// file the stick refuses leaves the one before as it was: the log is not
/// worth an update or a setup, and a stick that refuses every write fails
/// at the loader next.
pub(crate) fn keep_boot_log(
    host: &dyn crate::setup::host::Host,
    esp: &Path,
    loader: u64,
) -> LogFile {
    let free = host.volume(esp).ok().map(|volume| volume.free_bytes);
    keep_boot_log_in(esp, loader, free)
}

/// [`keep_boot_log`] on an EFI partition with `free` bytes free, `None`
/// when that is not known.
fn keep_boot_log_in(esp: &Path, loader: u64, free: Option<u64>) -> LogFile {
    let path = esp.join(BOOT_LOG);
    let current = fs::metadata(&path).map_or(0, |metadata| metadata.len());
    let installed = fs::metadata(esp.join(LOADER)).map_or(0, |metadata| metadata.len());
    let bytes = match free {
        Some(free) => boot_log_bytes(current, free, loader, installed),
        // A file a boot writes, without knowing what more fits.
        None if current < boot_log::LOG_BYTES => Some(boot_log::LOG_BYTES),
        None => return LogFile::RoomUnknown(current),
    };
    let Some(bytes) = bytes else {
        return match current >= boot_log::FILE_BYTES {
            true => LogFile::Kept,
            false => LogFile::NoRoom(current),
        };
    };
    let failed = |error: anyhow::Error| LogFile::Failed {
        bytes: current,
        error: reason(&error),
    };
    if let Err(error) = fs::create_dir_all(esp.join(MOUNT)) {
        return failed(error.into());
    }
    let (new, made) = match current < boot_log::LOG_BYTES {
        // `stela.efi` writes its clusters wherever the volume put them.
        true => (vec![0; bytes as usize], true),
        false => {
            let old = match fs::read(&path) {
                Ok(old) => old,
                Err(error) => return failed(error.into()),
            };
            match boot_log::relayout(&old, bytes) {
                Ok(new) => (new, false),
                Err(boot_log::HeaderError::Blank) => (vec![0; bytes as usize], true),
                Err(_) => return LogFile::Unreadable(current),
            }
        }
    };
    match write_atomic(&path, &new, current == 0) {
        Ok(()) if made => LogFile::Made(bytes),
        Ok(()) if bytes < current => LogFile::Shrunk {
            from: current,
            to: bytes,
        },
        Ok(()) => LogFile::Grown {
            from: current,
            to: bytes,
        },
        Err(error) => failed(error),
    }
}

/// Paths relative to the ESP that `install` writes, for reports.
pub(crate) const INSTALLED: [&str; 4] = [LOADER, OPTIONS, SOURCE, BOOT_LOG];

/// Remove the loader, when it is Stela's, and the files `install` wrote,
/// and return a report.
pub fn uninstall(esp: &Path, owned: bool) -> Result<serde_json::Value> {
    let stela = match owned {
        true => Some(true),
        false => loader_is_stela(esp)?,
    };
    let mut remove = vec![esp.join(OPTIONS), esp.join(SOURCE), esp.join(BOOT_LOG)];
    if stela == Some(true) {
        remove.insert(0, esp.join(LOADER));
    }
    let mut files = Vec::new();
    for file in remove {
        if file.is_file() {
            fs::remove_file(&file)?;
            files.push(file);
        }
    }
    // Directories go only when nothing else is left in them.
    for directory in [esp.join(MOUNT), esp.join(LOADER_DIRECTORY)] {
        if fs::read_dir(&directory).is_ok_and(|mut entries| entries.next().is_none()) {
            fs::remove_dir(&directory)?;
        }
    }
    Ok(json!({
        "files": files,
        "kept_loader": (stela == Some(false)).then(|| esp.join(LOADER)),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    const MIB: u64 = 1 << 20;

    #[test]
    fn the_log_file_is_as_long_as_the_efi_partition_keeps_room_for_two_loaders() {
        // A new stick's 200 MiB EFI partition and a 68 MB loader: 32 MiB.
        assert_eq!(
            boot_log_bytes(0, 199 * MIB, 68_400_128, 0),
            Some(boot_log::FILE_BYTES)
        );
        // A loader of 90 MB leaves room for 8 MiB, one of 100 MB for none
        // longer than the 4 MiB log of older sticks.
        assert_eq!(boot_log_bytes(0, 199 * MIB, 90_000_000, 0), Some(8 * MIB));
        assert_eq!(
            boot_log_bytes(0, 199 * MIB, 100_000_000, 0),
            Some(boot_log::LOG_BYTES)
        );
        // An update of a stick with a 4 MiB log and 127.6 MiB free beside
        // its 68 MB loader: 32 MiB, and the new loader still fits after.
        let free = 261_419 * 512;
        assert_eq!(
            boot_log_bytes(boot_log::LOG_BYTES, free, 68_400_128, 68_400_128),
            Some(boot_log::FILE_BYTES)
        );
        // Less room: the longest that fits, or the file as it is.
        assert_eq!(
            boot_log_bytes(boot_log::LOG_BYTES, 100 * MIB, 68_400_128, 68_400_128),
            Some(16 * MIB)
        );
        assert_eq!(
            boot_log_bytes(boot_log::LOG_BYTES, 85 * MIB, 68_400_128, 68_400_128),
            None
        );
        assert_eq!(
            boot_log_bytes(boot_log::FILE_BYTES, 1 << 40, 68_400_128, 68_400_128),
            None
        );
    }

    /// This tester build's loader, 87,715,840 bytes, and the NVIDIA one's.
    const LARGE_LOADER: u64 = 87_715_840;
    const NVIDIA_LOADER: u64 = 53_975_552;

    /// A 32 MiB log beside a loader too long for two copies of it and the
    /// log is made shorter, to the longest that keeps the next update's
    /// copy; one that leaves room, or a loader that needs none, keeps it.
    #[test]
    fn a_log_too_long_for_the_next_updates_loader_is_made_shorter() {
        let partition = 199 * MIB;
        // An NVIDIA stick's 32 MiB log, updated to the larger loader: the
        // update itself fits, the next would not.
        let free = partition - NVIDIA_LOADER - boot_log::FILE_BYTES;
        assert!(free >= LARGE_LOADER);
        assert!(partition - LARGE_LOADER - boot_log::FILE_BYTES < LARGE_LOADER);
        assert_eq!(
            boot_log_bytes(boot_log::FILE_BYTES, free, LARGE_LOADER, NVIDIA_LOADER),
            Some(8 * MIB)
        );
        // Then the next update of that loader keeps the 8 MiB log, and has
        // room for its copy.
        let free = partition - LARGE_LOADER - 8 * MIB;
        assert!(free >= LARGE_LOADER + LOADER_MARGIN);
        assert_eq!(
            boot_log_bytes(8 * MIB, free, LARGE_LOADER, LARGE_LOADER),
            None
        );
        // The NVIDIA loader beside the same log needs no change.
        let free = partition - NVIDIA_LOADER - boot_log::FILE_BYTES;
        assert_eq!(
            boot_log_bytes(boot_log::FILE_BYTES, free, NVIDIA_LOADER, NVIDIA_LOADER),
            None
        );
        // A little more room keeps a 16 MiB log; too little room
        // for any shorter log leaves it.
        assert_eq!(
            boot_log_bytes(boot_log::FILE_BYTES, 90 * MIB, LARGE_LOADER, LARGE_LOADER),
            Some(16 * MIB)
        );
        assert_eq!(
            boot_log_bytes(boot_log::FILE_BYTES, 40 * MIB, LARGE_LOADER, LARGE_LOADER),
            None
        );
    }

    /// An update over a stick with a 32 MiB log of twenty boots, in a folder
    /// for the EFI partition: the log is made 8 MiB with its newest lines
    /// and its newest boots' records, the loader goes in, and the next
    /// update keeps the log and has room for its copy of the loader.
    #[test]
    fn an_update_over_a_32_mib_log_keeps_its_newest_boots_and_room_for_the_next() {
        use stela_runtime::pc::boot_log::read;
        let temp = tempfile::tempdir().unwrap();
        let esp = temp.path();
        let path = esp.join(BOOT_LOG);
        let partition = 199 * MIB;
        // The NVIDIA stick: its loader (as long, never written out) and its
        // 32 MiB log.
        fs::create_dir_all(esp.join(LOADER_DIRECTORY)).unwrap();
        fs::create_dir_all(esp.join(MOUNT)).unwrap();
        fs::File::create(esp.join(LOADER))
            .unwrap()
            .set_len(NVIDIA_LOADER)
            .unwrap();
        let old = crate::boot_log::tests::written_in(
            boot_log::FILE_BYTES,
            &crate::boot_log::tests::boot_lines(1, 20, 12_000),
        );
        fs::write(&path, &old).unwrap();
        let before = read(&old).unwrap();
        assert_eq!(before.boots, 20);
        let free = partition - NVIDIA_LOADER - boot_log::FILE_BYTES;
        let log = keep_boot_log_in(esp, LARGE_LOADER, Some(free));
        assert_eq!(
            log,
            LogFile::Shrunk {
                from: boot_log::FILE_BYTES,
                to: 8 * MIB
            }
        );
        assert!(
            log.said()
                .unwrap()
                .starts_with("made shorter, 8.4 MB from 33.6 MB")
        );
        let short = fs::read(&path).unwrap();
        assert_eq!(short.len() as u64, 8 * MIB);
        let after = read(&short).unwrap();
        // The boots are counted on, the newest lines are kept as they were,
        // and every boot the old log kept a record of that the new one has
        // room for, the newest, keeps its record.
        assert_eq!(after.boots, before.boots);
        assert!(before.stream.ends_with(&after.stream));
        assert!(after.stream.len() < before.stream.len());
        assert!(after.stream.ends_with(b"boot 20 line 011999\n"));
        assert_eq!((before.starts.len(), after.starts.len()), (16, 8));
        let kept: Vec<u64> = after.starts.iter().map(|start| start.boot).collect();
        let newest: Vec<u64> = before
            .starts
            .iter()
            .map(|start| start.boot)
            .rev()
            .take(kept.len())
            .rev()
            .collect();
        assert!(!kept.is_empty());
        assert_eq!(kept, newest);
        // The loader goes in over the old one.
        let image = {
            let mut image = b"MZ".to_vec();
            image.extend_from_slice(&mount_interface::marker(mount_interface::CURRENT));
            image
        };
        add_mount(esp, &image, None, None).unwrap();
        assert_eq!(fs::read(esp.join(LOADER)).unwrap(), image);
        // The next update of a loader as long: the log stays, and the
        // partition holds that update's copy of the loader.
        fs::File::create(esp.join(LOADER))
            .unwrap()
            .set_len(LARGE_LOADER)
            .unwrap();
        let free = partition - LARGE_LOADER - 8 * MIB;
        assert!(free >= LARGE_LOADER);
        assert_eq!(
            keep_boot_log_in(esp, LARGE_LOADER, Some(free)),
            LogFile::NoRoom(8 * MIB)
        );
        assert_eq!(fs::read(&path).unwrap(), short);
    }

    /// A stick that refuses the shorter file keeps the long log as it was.
    #[cfg(unix)]
    #[test]
    fn a_shorter_log_file_the_stick_refuses_leaves_the_long_one() {
        use std::os::unix::fs::PermissionsExt;
        let temp = tempfile::tempdir().unwrap();
        let folder = temp.path().join(MOUNT);
        fs::create_dir_all(&folder).unwrap();
        let path = temp.path().join(BOOT_LOG);
        let four = crate::boot_log::tests::region(&["boot 1\n", "boot 2\n"]);
        let old = boot_log::relayout(&four, boot_log::FILE_BYTES).unwrap();
        fs::write(&path, &old).unwrap();
        fs::set_permissions(&folder, fs::Permissions::from_mode(0o555)).unwrap();
        if tempfile::NamedTempFile::new_in(&folder).is_ok() {
            fs::set_permissions(&folder, fs::Permissions::from_mode(0o755)).unwrap();
            return;
        }
        let free = 199 * MIB - NVIDIA_LOADER - boot_log::FILE_BYTES;
        let log = keep_boot_log_in(temp.path(), LARGE_LOADER, Some(free));
        fs::set_permissions(&folder, fs::Permissions::from_mode(0o755)).unwrap();
        assert!(
            matches!(log, LogFile::Failed { bytes, .. } if bytes == boot_log::FILE_BYTES),
            "{log:?}"
        );
        assert_eq!(fs::read(&path).unwrap(), old);
    }

    #[test]
    fn a_short_log_file_is_made_longer_with_its_lines() {
        use stela_runtime::pc::boot_log::read;
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join(BOOT_LOG);
        let room = Some(1 << 40);
        // No file yet: a long one, all zeros, in a folder made for it.
        assert_eq!(
            keep_boot_log_in(temp.path(), 0, room),
            LogFile::Made(boot_log::FILE_BYTES)
        );
        assert_eq!(
            fs::read(&path).unwrap(),
            vec![0; boot_log::FILE_BYTES as usize]
        );
        assert_eq!(keep_boot_log_in(temp.path(), 0, room), LogFile::Kept);
        // A 4 MiB log of three boots.
        let old = crate::boot_log::tests::region(&["boot 1\n", "boot 2\n", "boot 3\n"]);
        fs::write(&path, &old).unwrap();
        let before = read(&old).unwrap();
        assert_eq!(before.boots, 3);
        assert_eq!(
            keep_boot_log_in(temp.path(), 0, room),
            LogFile::Grown {
                from: boot_log::LOG_BYTES,
                to: boot_log::FILE_BYTES
            }
        );
        let grown = fs::read(&path).unwrap();
        assert_eq!(grown.len() as u64, boot_log::FILE_BYTES);
        let after = read(&grown).unwrap();
        assert_eq!((after.stream, after.starts), (before.stream, before.starts));
        // A file whose header does not decode keeps its bytes.
        let mut torn =
            crate::boot_log::tests::released_region(&crate::boot_log::tests::boot_lines(1, 3, 100));
        torn[16] ^= 0xff;
        fs::write(&path, &torn).unwrap();
        assert_eq!(
            keep_boot_log_in(temp.path(), 0, room),
            LogFile::Unreadable(boot_log::LOG_BYTES)
        );
        assert_eq!(fs::read(&path).unwrap(), torn);
        // So does a log on a partition without the room, or whose room is
        // not known.
        fs::write(&path, &old).unwrap();
        assert_eq!(
            keep_boot_log_in(temp.path(), 0, Some(19 << 20)),
            LogFile::NoRoom(boot_log::LOG_BYTES)
        );
        assert_eq!(
            keep_boot_log_in(temp.path(), 0, None),
            LogFile::RoomUnknown(boot_log::LOG_BYTES)
        );
        assert_eq!(fs::read(&path).unwrap(), old);
        // A file shorter than any log, which no boot writes, is made anew.
        fs::write(&path, b"cut short").unwrap();
        assert_eq!(
            keep_boot_log_in(temp.path(), 0, None),
            LogFile::Made(boot_log::LOG_BYTES)
        );
        assert_eq!(fs::metadata(&path).unwrap().len(), boot_log::LOG_BYTES);
    }

    /// A stick that refuses the longer file keeps the log it had, which
    /// reads as before, and the loader still goes in.
    #[cfg(unix)]
    #[test]
    fn a_log_file_the_stick_refuses_leaves_the_one_before_and_the_update_goes_on() {
        use std::os::unix::fs::PermissionsExt;
        let temp = tempfile::tempdir().unwrap();
        fs::create_dir_all(temp.path().join(MOUNT)).unwrap();
        let path = temp.path().join(BOOT_LOG);
        let old = crate::boot_log::tests::region(&["boot 1\n", "boot 2\n"]);
        fs::write(&path, &old).unwrap();
        let folder = temp.path().join(MOUNT);
        fs::set_permissions(&folder, fs::Permissions::from_mode(0o555)).unwrap();
        // Root writes anyway: nothing to see then.
        if tempfile::NamedTempFile::new_in(&folder).is_ok() {
            fs::set_permissions(&folder, fs::Permissions::from_mode(0o755)).unwrap();
            return;
        }
        let log = keep_boot_log_in(temp.path(), 0, Some(1 << 40));
        let LogFile::Failed { bytes, error } = &log else {
            panic!("{log:?}");
        };
        assert_eq!(*bytes, boot_log::LOG_BYTES);
        // As the system names it, without the path.
        assert!(
            error.eq_ignore_ascii_case("permission denied (os error 13)")
                || error == "permission denied",
            "{error}"
        );
        let said = log.said().unwrap();
        assert!(
            said.starts_with("stays 4.2 MB as it was: the stick refused the new file ("),
            "{said}"
        );
        assert!(!said.contains("EFI/stela"), "{said}");
        // Nothing of the new file is left, and the old one reads as before.
        assert_eq!(fs::read_dir(&folder).unwrap().count(), 1);
        assert_eq!(fs::read(&path).unwrap(), old);
        // The update's next step, the loader, goes on: it is in another folder.
        let image = {
            let mut image = b"MZ".to_vec();
            image.extend_from_slice(&mount_interface::marker(mount_interface::CURRENT));
            image
        };
        add_mount(temp.path(), &image, None, None).unwrap();
        assert_eq!(fs::read(temp.path().join(LOADER)).unwrap(), image);
        fs::set_permissions(&folder, fs::Permissions::from_mode(0o755)).unwrap();
        // A new stick's file refused: none, said so.
        let said = LogFile::Failed {
            bytes: 0,
            error: "No space left on device (os error 28)".into(),
        }
        .said()
        .unwrap();
        assert_eq!(
            said,
            "none: the stick refused the file (No space left on device (os error 28))"
        );
    }

    /// Each outcome is one plain line, without the file's path or format.
    #[test]
    fn what_happened_to_the_log_is_said_in_plain_words() {
        let said = |log: LogFile| log.said();
        assert_eq!(said(LogFile::Kept), None);
        assert_eq!(
            said(LogFile::Shrunk {
                from: boot_log::FILE_BYTES,
                to: 8 << 20
            })
            .unwrap(),
            "made shorter, 8.4 MB from 33.6 MB, with its newest lines and its last boots, so \
             the EFI partition keeps room for the next update"
        );
        assert_eq!(
            said(LogFile::Made(boot_log::FILE_BYTES)).unwrap(),
            "made (33.6 MB)"
        );
        assert_eq!(
            said(LogFile::Made(boot_log::LOG_BYTES)).unwrap(),
            "made (4.2 MB), as the EFI partition has no room for a longer one"
        );
        assert_eq!(
            said(LogFile::Grown {
                from: boot_log::LOG_BYTES,
                to: boot_log::FILE_BYTES
            })
            .unwrap(),
            "made longer, 33.6 MB from 4.2 MB, with every boot it held"
        );
        assert_eq!(
            said(LogFile::Grown {
                from: boot_log::LOG_BYTES,
                to: 16 << 20
            })
            .unwrap(),
            "made longer, 16.8 MB from 4.2 MB, with every boot it held, as the EFI partition \
             has no room for a longer one"
        );
        assert_eq!(
            said(LogFile::NoRoom(boot_log::LOG_BYTES)).unwrap(),
            "stays 4.2 MB: the EFI partition has no room for a longer one"
        );
        assert_eq!(
            said(LogFile::RoomUnknown(boot_log::LOG_BYTES)).unwrap(),
            "stays 4.2 MB: the EFI partition's free room could not be read"
        );
        assert_eq!(
            said(LogFile::Unreadable(boot_log::LOG_BYTES)).unwrap(),
            "left as it is (4.2 MB): it does not read as a log; stambha log still shows its lines"
        );
    }

    /// What macOS answers for a rename that keeps an existing file on FAT
    /// and exFAT: ENOTSUP.
    fn unsupported(_: &Path, _: &Path) -> std::io::Result<()> {
        Err(std::io::Error::from(std::io::ErrorKind::Unsupported))
    }

    fn temporary(directory: &Path, bytes: &[u8]) -> tempfile::NamedTempFile {
        let mut file = tempfile::NamedTempFile::new_in(directory).unwrap();
        file.write_all(bytes).unwrap();
        file
    }

    #[test]
    fn a_volume_without_a_keeping_rename_publishes_a_new_file_by_renaming() {
        let temp = tempfile::tempdir().unwrap();
        let (part, path) = (temp.path().join("a.part"), temp.path().join("boot.log"));
        fs::write(&part, b"new").unwrap();
        rename_new_with(&part, &path, unsupported).unwrap();
        assert_eq!(fs::read(&path).unwrap(), b"new");
        // Only the published file is left: the part became it.
        assert_eq!(fs::read_dir(temp.path()).unwrap().count(), 1);
    }

    #[test]
    fn an_existing_file_is_never_replaced_either_way() {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("boot.log");
        fs::write(&path, b"earlier boots").unwrap();
        // A refused part stays for its caller, either way.
        let part = temp.path().join("a.part");
        fs::write(&part, b"new").unwrap();
        for result in [
            rename_new_with(&part, &path, unsupported),
            rename_new(&part, &path),
        ] {
            let error = result.unwrap_err().to_string();
            assert!(error.contains("already exists"), "{error}");
            assert_eq!(fs::read(&part).unwrap(), b"new");
        }
        fs::remove_file(&part).unwrap();
        let error = persist_new(temporary(temp.path(), b"new"), &path)
            .unwrap_err()
            .to_string();
        assert!(error.contains("already exists"), "{error}");
        assert_eq!(fs::read(&path).unwrap(), b"earlier boots");
        // The refused temporary files are removed.
        assert_eq!(fs::read_dir(temp.path()).unwrap().count(), 1);
    }

    /// The real rename, on whatever file system holds the temporary
    /// directory (`TMPDIR` on a FAT or exFAT volume takes the fallback).
    #[test]
    fn a_new_file_is_published_on_the_temporary_directory_s_file_system() {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("boot.log");
        persist_new(temporary(temp.path(), b"new"), &path).unwrap();
        assert_eq!(fs::read(&path).unwrap(), b"new");
        let image = temp.path().join("fdr.img4");
        write_atomic(&image, b"image", true).unwrap();
        assert_eq!(fs::read(&image).unwrap(), b"image");
        assert!(write_atomic(&image, b"other", true).is_err());
        assert_eq!(fs::read(&image).unwrap(), b"image");
        assert_eq!(fs::read_dir(temp.path()).unwrap().count(), 2);
    }

    #[test]
    fn a_failed_write_names_the_file() {
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("missing").join("boot.log");
        let error = format!("{:#}", write_atomic(&path, b"new", true).unwrap_err());
        assert!(
            error.contains(&format!("cannot write {}", path.display())),
            "{error}"
        );
    }
}

//! The install medium (`--install MANIFEST --install-source GUID`): the
//! IPSW as `ipsw.NNN` pieces on a FAT32 partition of the USB, beside its
//! `payload.json`, which `stela` writes.
//!
//! Before boot services end, [`check_pieces`] confirms through the
//! firmware's own FAT driver that every piece is there with its size. After
//! them, [`open`] finds each piece's clusters through Stela's FAT32 reader
//! on the partition the storage drivers opened by its GUID, and joins them
//! into one IPSW whose every block is checked against the piece hashes the
//! manifest names. A block that fails its check is named once in the boot
//! log ([`DAMAGED_LOG_LINE`]), which `stela status` reads to send the
//! user to `repair`.
use super::bundle::{self, Bytes};
use crate::Result;
use crate::output;
use alloc::{boxed::Box, format, vec::Vec};
use stela_contracts::refuse;
use stela_drivers::block::{Ahead, BlockDevice, Bounded, ReadBound};
use stela_runtime::restore::asr::CHUNK_BYTES;
use uefi::proto::media::file::Directory;
use {
    stela_formats::payload::Layout, stela_formats::payload::PayloadError,
    stela_formats::payload::PayloadManifest, stela_formats::payload::ReadAt,
    stela_formats::payload::piece_name, stela_formats::payload::pieces::DAMAGED_LOG_LINE,
    stela_formats::payload::pieces::Joined, stela_runtime::pc::fat32::Fat32,
};

/// The joined install medium the restore host reads. The first block that
/// fails its hash is named in the boot log and on the screen; the read
/// fails as before, so nothing damaged reaches the restore.
pub(super) struct Medium {
    joined: Joined<Bytes<Box<dyn BlockDevice>>>,
    reported: bool,
    /// The bound of the partition's reads ([`Bounded`]).
    bound: ReadBound,
    /// The partition's read-ahead, which the restore host pumps each turn;
    /// none with `--medium-read-ahead 0`.
    ahead: Option<Ahead>,
}

impl Medium {
    /// The bound the restore host sets on the partition's reads once it
    /// serves on a PE whose guest waits meanwhile.
    pub(super) fn read_bound(&self) -> ReadBound {
        self.bound.clone()
    }

    /// The read-ahead the pieces are read through, if any: pumped, it keeps
    /// the USB's next reads in flight while the service is away.
    pub(super) fn read_ahead(&self) -> Option<Ahead> {
        self.ahead.clone()
    }
}

impl ReadAt for Medium {
    type Error = PayloadError;

    fn len(&self) -> u64 {
        self.joined.len()
    }

    fn read_at(&mut self, offset: u64, buffer: &mut [u8]) -> core::result::Result<(), Self::Error> {
        let result = self.joined.read_at(offset, buffer);
        if let Err(PayloadError::Verification { member, .. }) = &result
            && !self.reported
        {
            self.reported = true;
            output::arguments(format_args!("{DAMAGED_LOG_LINE}{member}\n"));
            crate::status::info(
                "The macOS install files on the USB are damaged: on another computer, plug in \
                 this USB and run `stambha repair`, then install again.",
            );
            output::flush_log();
        }
        result
    }
}

/// Check, through the firmware's FAT driver on `root`, that every piece
/// `manifest` lists is in `directory` with its exact size. Each missing or
/// short piece is reported by name.
pub fn check_pieces(
    root: &mut Directory,
    directory: &str,
    manifest: &PayloadManifest,
) -> Result<()> {
    let pieces = match &manifest.layout {
        Layout::Pieces { pieces, .. } => pieces,
        // Streamed after boot services end, over the mount's own network.
        Layout::Remote { .. } => return Ok(()),
        _ => {
            return Err(refuse!(
                "--install names a payload that is not an install medium"
            ));
        }
    };
    let mut complete = true;
    for (index, piece) in pieces.iter().enumerate() {
        let path = format!("{directory}\\{}", piece_name(index));
        match bundle::file_size(root, &path) {
            Ok(bytes) if bytes == piece.bytes => {}
            Ok(bytes) => {
                output::arguments(format_args!(
                    "INSTALL MEDIUM {path} holds {bytes} bytes, not {}\n",
                    piece.bytes
                ));
                complete = false;
            }
            Err(_) => {
                output::arguments(format_args!("INSTALL MEDIUM {path} is missing\n"));
                complete = false;
            }
        }
    }
    if !complete {
        return Err(refuse!(
            "the install medium is incomplete; run `stambha repair` on another computer to finish it"
        ));
    }
    output::arguments(format_args!(
        "INSTALL MEDIUM {directory}: {} pieces, {} bytes, build {}\n",
        pieces.len(),
        manifest.source.bytes,
        manifest.product_build
    ));
    Ok(())
}

/// The install medium on `partition`: each piece of `manifest` found
/// through FAT32 in `directory` (the manifest's own, as `stela.cfg` names
/// it) and joined, checked against `hashes`, the piece hash file. The
/// partition is read `read_ahead` chunks ahead (`--medium-read-ahead`);
/// with none it is read only when asked, through the bound alone, as it
/// was before the read-ahead.
pub(super) fn open(
    partition: Box<dyn BlockDevice>,
    manifest: &PayloadManifest,
    directory: &str,
    hashes: &'static [u8],
    read_ahead: usize,
) -> Result<Medium> {
    let Layout::Pieces { pieces, .. } = &manifest.layout else {
        return Err(refuse!(
            "--install names a payload that is not an install medium"
        ));
    };
    let identity = partition.identity();
    let sector_bytes = identity.block_size;
    let features = identity.features;
    let read_limit = features
        .max_read_transfer
        .map_or(features.max_transfer, |limit| limit.get());
    // The restore host reads the pieces in service turns, and the USB may
    // hold a read behind its own slow writes (the logs, the translation
    // store) for seconds: once bounded, a read waits for it only briefly.
    // Read ahead, as the study's raw source is, the USB goes on reading the
    // pieces between the turns instead of only within each one's bound.
    let (bound, ahead, partition) = match read_ahead {
        0 => {
            let partition = Bounded::new(partition);
            let bound = partition.bound();
            (bound, None, Box::new(partition) as Box<dyn BlockDevice>)
        }
        depth => {
            let (bound, ahead) = super::restore::read_ahead(partition, depth);
            let partition = Box::new(ahead.clone()) as Box<dyn BlockDevice>;
            (bound, Some(ahead), partition)
        }
    };
    let prefetch = (super::restore::prefetch_chunks(ahead.is_some()) * CHUNK_BYTES) >> 10;
    match read_ahead {
        0 => output::arguments(format_args!(
            "INSTALL MEDIUM read-ahead off (--medium-read-ahead 0): each read waits for the \
             USB within its service, {prefetch} KiB a service, as before the read-ahead\n"
        )),
        depth => output::arguments(format_args!(
            "INSTALL MEDIUM read-ahead {depth} chunks of {} KiB (--medium-read-ahead), \
             {prefetch} KiB a service\n",
            super::restore::READ_AHEAD_BYTES >> 10
        )),
    }
    let mut volume = Fat32::open(Bytes::new(partition))
        .map_err(|error| refuse!("install medium: {error}", %error))?;
    let directory = directory.replace('\\', "/");
    let mut extents = Vec::with_capacity(pieces.len());
    for index in 0..pieces.len() {
        let file = volume
            .file(&format!("{directory}/{}", piece_name(index)))
            .map_err(|error| refuse!("install medium: {error}", %error))?;
        extents.push(file.extents);
    }
    Joined::from_static_hashes(volume.into_inner(), manifest, extents, hashes)
        .map(|joined| Medium {
            joined: joined.with_two_block_reads(read_limit, sector_bytes),
            reported: false,
            bound,
            ahead,
        })
        .map_err(|error| refuse!("install medium: {error}", %error))
}

//! Block devices built on others or on memory: extent runs, read-ahead,
//! volatile overlays and RAM disks.

use super::shared::WINDOW_DEPTH;
use super::*;

/// Runs of blocks of a [`Shared`] disk, one after another as one device.
/// Like a [`Window`], it takes queued commands, each sent to the disk as one
/// command per run it reaches, and never waits to look at the disk.
pub struct Runs {
    pub(super) disk: Shared,
    pub(super) runs: Vec<(u64, u64)>,
    pub(super) identity: Identity,
    /// The commands started and not yet finished.
    pub(super) started: Vec<Spread>,
    /// The next command's tag.
    pub(super) next: u32,
    /// A commit put off while another owner held the disk.
    pub(super) uncommitted: bool,
}

/// A command started on [`Runs`]: the disk's ticket of each piece and the
/// piece's byte offset in the transfer, those not finished yet, and what
/// the finished ones gave.
pub(super) struct Spread {
    tag: u32,
    pending: Vec<(u32, usize)>,
    /// A read's bytes, assembled as its pieces finish.
    read: Option<(usize, Option<Vec<u8>>)>,
    failed: Option<BlockError>,
}

impl Spread {
    fn completed(&mut self, offset: usize, result: Finished) {
        match result {
            Ok(Some(bytes)) => {
                if let Some((total, data)) = &mut self.read {
                    if data.is_none() && offset == 0 && bytes.len() == *total {
                        *data = Some(bytes);
                    } else if offset + bytes.len() <= *total {
                        data.get_or_insert_with(|| vec![0; *total])[offset..offset + bytes.len()]
                            .copy_from_slice(&bytes);
                    } else {
                        self.failed
                            .get_or_insert(BlockError::Protocol("a read returned too much data"));
                    }
                }
            }
            Ok(None) => {}
            Err(error) => {
                self.failed.get_or_insert(error);
            }
        }
    }

    /// The command's result once every piece has finished.
    fn result(&mut self) -> Option<Finished> {
        if !self.pending.is_empty() {
            return None;
        }
        Some(match self.failed.take() {
            Some(error) => Err(error),
            None => Ok(self
                .read
                .take()
                .map(|(total, data)| data.unwrap_or_else(|| vec![0; total]))),
        })
    }
}

impl Runs {
    /// The disk commands `command` becomes, each with its byte offset in
    /// the transfer; refused whole before any reaches the disk.
    fn spread(&self, command: Command) -> Result<Vec<(usize, Command)>, BlockError> {
        let size = self.identity.block_size as usize;
        Ok(match command {
            Command::Read { lba, bytes } => {
                let blocks = self.identity.check(lba, bytes)?;
                self.pieces(lba, blocks)
                    .into_iter()
                    .map(|(at, from, count)| {
                        let bytes = count * size;
                        (from * size, Command::Read { lba: at, bytes })
                    })
                    .collect()
            }
            Command::Write { lba, data, fua } => {
                let blocks = self.identity.check_write(lba, data.len())?;
                self.pieces(lba, blocks)
                    .into_iter()
                    .map(|(at, from, count)| {
                        let data = data[from * size..(from + count) * size].to_vec();
                        (from * size, Command::Write { lba: at, data, fua })
                    })
                    .collect()
            }
            Command::Flush => vec![(0, Command::Flush)],
            Command::Discard { extents } => {
                let mut commands = Vec::new();
                for extent in extents {
                    self.identity
                        .check_write(extent.lba, extent.blocks as usize * size)?;
                    for (at, _, count) in self.pieces(extent.lba, extent.blocks) {
                        commands.push((0, Command::discard(at, count as u64)));
                    }
                }
                commands
            }
            Command::WriteZeroes { lba, blocks, unmap } => {
                self.identity.check_write(lba, blocks as usize * size)?;
                self.pieces(lba, blocks)
                    .into_iter()
                    .map(|(at, _, count)| {
                        let blocks = count as u64;
                        (
                            0,
                            Command::WriteZeroes {
                                lba: at,
                                blocks,
                                unmap,
                            },
                        )
                    })
                    .collect()
            }
        })
    }

    /// Queue `command` on the disk as its pieces; its tag. The device is
    /// told of them at once unless `more` follow ([`BlockDevice::commit`]).
    fn submit(&mut self, command: Command, more: bool) -> Result<u32, BlockError> {
        if self.started.len() >= WINDOW_DEPTH {
            return Err(BlockError::Protocol("the disk runs queue is full"));
        }
        let read = match &command {
            Command::Read { bytes, .. } => Some((*bytes, None)),
            _ => None,
        };
        let pieces = self.spread(command)?;
        let last = pieces.len().saturating_sub(1);
        let mut pending = Vec::with_capacity(pieces.len());
        {
            let mut disk = self.disk.lock();
            for (index, (offset, piece)) in pieces.into_iter().enumerate() {
                pending.push((disk.submit(piece, more || index != last), offset));
            }
        }
        self.uncommitted &= more;
        let tag = self.next;
        self.next = self.next.wrapping_add(1);
        self.started.push(Spread {
            tag,
            pending,
            read,
            failed: None,
        });
        Ok(tag)
    }

    /// Command `tag`'s result once each of its pieces has one, collecting
    /// each with `take`.
    fn collect(
        &mut self,
        tag: u32,
        mut take: impl FnMut(u32) -> Option<Finished>,
    ) -> Option<Finished> {
        let index = self.started.iter().position(|spread| spread.tag == tag)?;
        let spread = &mut self.started[index];
        let mut at = 0;
        while let Some(&(ticket, offset)) = spread.pending.get(at) {
            match take(ticket) {
                Some(result) => {
                    spread.pending.swap_remove(at);
                    spread.completed(offset, result);
                }
                None => at += 1,
            }
        }
        let result = spread.result()?;
        self.started.swap_remove(index);
        Some(result)
    }
    /// The disk's blocks for `blocks` blocks from `lba`, which
    /// [`Identity::check`] has kept inside the device: each piece's first
    /// disk block, and its first block and count in the transfer.
    pub(super) fn pieces(&self, lba: u64, blocks: u64) -> Vec<(u64, usize, usize)> {
        let (mut start, mut pieces) = (0, Vec::new());
        let end = lba + blocks;
        for &(first, count) in &self.runs {
            let (from, to) = (lba.max(start), end.min(start + count));
            if from < to {
                pieces.push((
                    first + from - start,
                    (from - lba) as usize,
                    (to - from) as usize,
                ));
            }
            start += count;
        }
        pieces
    }

    /// Write `data` at `lba` piece by piece, each in its turn among the
    /// disk's queued commands.
    pub(super) fn write_pieces(
        &mut self,
        lba: u64,
        data: &[u8],
        fua: bool,
    ) -> Result<(), BlockError> {
        let blocks = self.identity.check_write(lba, data.len())?;
        let size = self.identity.block_size as usize;
        for (at, from, count) in self.pieces(lba, blocks) {
            self.disk.run(Command::Write {
                lba: at,
                data: data[from * size..(from + count) * size].to_vec(),
                fua,
            })?;
        }
        Ok(())
    }

    /// `command` for each piece of `blocks` blocks from `lba`.
    pub(super) fn each(
        &mut self,
        lba: u64,
        blocks: u64,
        command: impl Fn(u64, u64) -> Command,
    ) -> Result<(), BlockError> {
        let size = self.identity.block_size as usize;
        self.identity.check_write(lba, blocks as usize * size)?;
        for (at, _, count) in self.pieces(lba, blocks) {
            self.disk.run(command(at, count as u64))?;
        }
        Ok(())
    }
}

impl BlockDevice for Runs {
    fn identity(&self) -> &Identity {
        &self.identity
    }

    /// Each piece in its turn among the disk's queued commands.
    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        let blocks = self.identity.check(lba, buffer.len())?;
        let size = self.identity.block_size as usize;
        for (at, from, count) in self.pieces(lba, blocks) {
            let part = &mut buffer[from * size..(from + count) * size];
            let command = Command::Read {
                lba: at,
                bytes: part.len(),
            };
            let data = self.disk.run(command)?;
            part.copy_from_slice(&data.ok_or(BlockError::Protocol("a read returned no data"))?);
        }
        Ok(())
    }

    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.write_pieces(lba, data, false)
    }

    fn write_fua(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.write_pieces(lba, data, true)
    }

    fn flush_cache(&mut self) -> Result<(), BlockError> {
        self.disk.run(Command::Flush).map(drop)
    }

    fn discard(&mut self, lba: u64, blocks: u64) -> Result<(), BlockError> {
        self.each(lba, blocks, Command::discard)
    }

    fn write_zeroes(&mut self, lba: u64, blocks: u64, unmap: bool) -> Result<(), BlockError> {
        self.each(lba, blocks, |lba, blocks| Command::WriteZeroes {
            lba,
            blocks,
            unmap,
        })
    }

    /// Runs take queued commands as a window does: the disk runs their
    /// pieces in turn, so a caller that may not wait, such as a log store
    /// in a service turn, never runs one where it asks.
    fn depth(&self) -> usize {
        WINDOW_DEPTH
    }

    fn room(&self) -> bool {
        self.started.len() < WINDOW_DEPTH && !self.disk.held()
    }

    fn start(&mut self, command: Command) -> Result<u32, BlockError> {
        self.submit(command, false)
    }

    fn start_more(&mut self, command: Command) -> Result<u32, BlockError> {
        self.submit(command, true)
    }

    fn commit(&mut self) {
        match self.disk.try_lock() {
            Some(mut disk) => disk.commit(),
            None => self.uncommitted = true,
        }
    }

    /// Takes the disk's completions and commits what waits.
    fn finish(&mut self, tag: u32) -> Option<Finished> {
        let disk = self.disk.clone();
        let mut held = disk.try_lock()?;
        self.uncommitted = false;
        self.collect(tag, |ticket| held.take(ticket))
    }

    /// A finish collects every completed command of the disk.
    fn reaps_together(&self) -> bool {
        true
    }

    fn finished(&mut self, tag: u32) -> Option<Finished> {
        let disk = self.disk.clone();
        let mut held = disk.try_lock()?;
        self.collect(tag, |ticket| held.requests().finished.remove(&ticket))
    }

    /// One of these runs' pieces has a result, the disk has posted a
    /// completion, or a look would run the next piece of a disk of depth 0:
    /// another owner's results never count.
    fn posted(&self) -> bool {
        let Some(mut disk) = self.disk.try_lock() else {
            return true;
        };
        let finished = &disk.requests().finished;
        self.started
            .iter()
            .flat_map(|spread| &spread.pending)
            .any(|(ticket, _)| finished.contains_key(ticket))
            || disk.runs_waiting()
            || disk.posted()
    }

    fn signals(&self) -> bool {
        self.disk.lock().signals()
    }
}

/// Sequential read-ahead over a device that takes queued commands, as an
/// operating system's page cache reads a file ahead: while its reader
/// streams, each read going on from the last or served ahead,
/// [`ReadAhead::pump`] keeps `depth` chunks after the last read in memory
/// or in flight, never waiting, and a read finds them in memory. A read
/// that catches up with a chunk in flight waits for that chunk, within the
/// bound of a bounded device once one is set ([`ReadAhead::bounded`]),
/// rather than read its blocks again. A read that nothing read ahead holds reads the device
/// directly and starts the read-ahead over after it; one that jumped there
/// is read ahead again once its reader reads on from it, so a reader that
/// moves between two places reads nothing for nothing.
pub struct ReadAhead {
    pub(super) device: Box<dyn BlockDevice>,
    /// The bound of a bounded device's reads, which a read waiting for a
    /// chunk in flight waits within too.
    pub(super) bound: Option<ReadBound>,
    /// Blocks per read-ahead request, and requests kept ahead.
    pub(super) chunk: u64,
    pub(super) depth: usize,
    /// Chunks read ahead, by first block.
    pub(super) ready: BTreeMap<u64, Vec<u8>>,
    /// The first block of the next chunk to read ahead.
    pub(super) next: u64,
    /// The first block of the last read that succeeded: its chunks stay for
    /// a caller whose next read ran past its bound and who reads it again.
    pub(super) last: u64,
    /// The block after it, the first a reader may still want.
    pub(super) wanted: u64,
    /// Whether the last read went on from the one before it or was served
    /// ahead: only then is the device read ahead.
    pub(super) streaming: bool,
    /// The chunks being read, by first block.
    pub(super) running: Vec<(u64, Submission)>,
}

impl ReadAhead {
    /// Read `device` ahead in requests of `chunk_bytes`, keeping `depth`. A
    /// read that catches up with a chunk in flight takes it if it has
    /// arrived and else reads the device directly; over a [`Bounded`]
    /// device take [`Self::bounded`], whose reads wait for it within the
    /// bound.
    pub fn new(device: Box<dyn BlockDevice>, chunk_bytes: usize, depth: usize) -> Self {
        let block = u64::from(device.identity().block_size.max(1));
        Self {
            bound: None,
            chunk: (chunk_bytes as u64 / block).max(1),
            depth,
            ready: BTreeMap::new(),
            next: 0,
            last: 0,
            wanted: 0,
            streaming: true,
            running: Vec::new(),
            device,
        }
    }

    /// [`Self::new`] over a bounded device: a read that waits for a chunk in
    /// flight waits only within the device's bound, as the device's own reads
    /// do, and past it fails as blocked ([`BlockError::Timeout`]).
    pub fn bounded(device: Bounded, chunk_bytes: usize, depth: usize) -> Self {
        let bound = device.bound();
        Self {
            bound: Some(bound),
            ..Self::new(Box::new(device), chunk_bytes, depth)
        }
    }

    /// The blocks `data`, a chunk's bytes, holds.
    fn blocks(&self, data: &[u8]) -> u64 {
        data.len() as u64 / u64::from(self.device.identity().block_size.max(1))
    }

    /// Collect completed read-aheads and, while the reader streams, submit
    /// the next; never waits. A device without queued commands is read only
    /// when asked.
    pub fn pump(&mut self) {
        if self.device.depth() == 0 {
            return;
        }
        let block = self.device.identity().block_size as u64;
        let mut index = 0;
        while index < self.running.len() {
            let Some(finished) = self.running[index].1.poll(&mut *self.device) else {
                index += 1;
                continue;
            };
            let (first, _) = self.running.swap_remove(index);
            self.arrived(first, finished);
        }
        if !self.streaming {
            return;
        }
        let blocks = self.device.identity().blocks;
        while self.ahead() < self.depth && self.next < blocks && self.device.room() {
            let (first, count) = (self.next, self.chunk.min(blocks - self.next));
            let request = Request::Read {
                lba: first,
                bytes: (count * block) as usize,
            };
            let Ok(mut submission) = Submission::new(request, self.device.identity()) else {
                return;
            };
            match submission.poll(&mut *self.device) {
                Some(Err(_)) => return,
                Some(finished) => {
                    self.next += count;
                    self.arrived(first, finished);
                }
                None => {
                    self.running.push((first, submission));
                    self.next += count;
                }
            }
        }
    }

    /// A chunk read ahead from `first` finished: kept while a reader may
    /// still want it. A failed one is read again when asked, directly; one
    /// a read that went elsewhere left behind goes.
    fn arrived(&mut self, first: u64, finished: Finished) {
        if let Ok(Some(data)) = finished
            && first + self.blocks(&data) > self.wanted
            && first < self.next
        {
            self.ready.insert(first, data);
        }
    }

    /// The chunks after the last read, in memory or in flight.
    fn ahead(&self) -> usize {
        let after = |(first, data): &(&u64, &Vec<u8>)| **first + self.blocks(data) > self.wanted;
        self.ready.iter().filter(after).count() + self.running.len()
    }

    /// Wait for chunk `index` in flight, which holds a block a read wants:
    /// whether it arrived. One that failed is read again directly. Within a
    /// bound ([`Self::bounded`]), as a [`Bounded`] read: past it, only a
    /// result the device has by then is taken, else the read is blocked.
    /// With no bound set, nothing limits a wait, so only a result the device
    /// has already is taken, and else the read reads directly, as one of a
    /// [`Bounded`] device does then.
    fn wait(&mut self, index: usize) -> Result<bool, BlockError> {
        loop {
            if let Some(finished) = self.running[index].1.poll(&mut *self.device) {
                let (first, _) = self.running.swap_remove(index);
                return Ok(match finished {
                    Ok(Some(data)) => {
                        self.ready.insert(first, data);
                        true
                    }
                    _ => false,
                });
            }
            let Some(bound) = self.bound.as_ref().filter(|bound| bound.is_set()) else {
                return Ok(false);
            };
            if bound.passed() {
                bound.block();
                return Err(BlockError::Timeout);
            }
            crate::waiting();
            core::hint::spin_loop();
        }
    }

    pub(super) fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        let block = self.device.identity().block_size as usize;
        let blocks = (buffer.len() / block) as u64;
        let end = lba + blocks;
        // Chunks wholly before this read and the last one are not read
        // again: a caller whose read past its bound failed reads the last
        // one again first.
        let keep = lba.min(self.last);
        while let Some((&first, data)) = self.ready.first_key_value() {
            if first + self.blocks(data) > keep {
                break;
            }
            self.ready.pop_first();
        }
        let mut at = 0;
        while at < blocks {
            let want = lba + at;
            let found = self
                .ready
                .range(..=want)
                .next_back()
                .filter(|(first, data)| **first + self.blocks(data) > want)
                .map(|(first, data)| (*first, data));
            if let Some((first, data)) = found {
                let from = (want - first) as usize * block;
                let count = (data.len() - from).min((blocks - at) as usize * block);
                let to = at as usize * block;
                buffer[to..to + count].copy_from_slice(&data[from..from + count]);
                at += (count / block) as u64;
                continue;
            }
            let chunk = self.chunk;
            let Some(index) = self
                .running
                .iter()
                .position(|(first, _)| (*first..*first + chunk).contains(&want))
            else {
                break;
            };
            if !self.wait(index)? {
                break;
            }
        }
        let served = at > 0;
        if at < blocks {
            // A miss: what nothing read ahead holds is read here. The
            // read-ahead goes on after it where a chunk starts there, else
            // over from there, with only the last read's chunks kept.
            let continued = self.ready.contains_key(&end)
                || self.running.iter().any(|(first, _)| *first == end);
            if !continued {
                let (last, wanted) = (self.last, self.wanted.min(end));
                let size = u64::from(self.device.identity().block_size.max(1));
                self.ready.retain(|first, data| {
                    *first < wanted && *first + data.len() as u64 / size > last
                });
                self.next = end;
            }
            let to = at as usize * block;
            self.device.read(lba + at, &mut buffer[to..])?;
        }
        self.streaming = served || (self.last..=self.wanted).contains(&lba);
        self.last = lba;
        self.wanted = end;
        self.next = self.next.max(end);
        Ok(())
    }
}

/// A [`ReadAhead`] shared by its reader and whoever pumps it.
#[derive(Clone)]
pub struct Ahead {
    pub inner: alloc::rc::Rc<core::cell::RefCell<ReadAhead>>,
    pub(super) identity: Identity,
}

impl Ahead {
    pub fn new(ahead: ReadAhead) -> Self {
        let identity = ahead.device.identity().clone();
        Self {
            inner: alloc::rc::Rc::new(core::cell::RefCell::new(ahead)),
            identity,
        }
    }

    pub fn pump(&self) {
        self.inner.borrow_mut().pump();
    }
}

impl BlockDevice for Ahead {
    fn identity(&self) -> &Identity {
        &self.identity
    }

    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.identity.check(lba, buffer.len())?;
        self.inner.borrow_mut().read(lba, buffer)
    }

    fn write(&mut self, _lba: u64, _data: &[u8]) -> Result<(), BlockError> {
        Err(BlockError::ReadOnly)
    }
}

/// Engine memory as a disk that keeps nothing past power-off. Blocks read
/// as zeros until written; a written chunk takes [`Volatile::CHUNK`] bytes
/// of the heap, at most `limit` bytes in all, and a write that needs more
/// refuses whole. Writing zeros where nothing was written takes nothing.
pub struct Volatile {
    pub(super) identity: Identity,
    pub(super) chunks: BTreeMap<u64, Box<[u8]>>,
    pub(super) limit: usize,
}

impl Volatile {
    pub const BLOCK: u32 = 4096;
    pub const CHUNK: usize = 64 * 1024;

    /// A `bytes`-long disk, a nonzero whole number of chunks, named
    /// `serial`, that may keep at most `limit` written bytes.
    pub fn new(serial: &str, bytes: u64, limit: usize) -> Result<Self, BlockError> {
        if bytes == 0 || !bytes.is_multiple_of(Self::CHUNK as u64) {
            return Err(BlockError::Unsupported(
                "a RAM-backed disk is a nonzero whole number of 64 KiB chunks",
            ));
        }
        Ok(Self {
            identity: Identity {
                transport: Transport::Ram,
                location: String::from("engine memory"),
                model: String::from("Stela volatile disk"),
                serial: String::from(serial),
                blocks: bytes / u64::from(Self::BLOCK),
                block_size: Self::BLOCK,
                read_only: false,
                features: Features {
                    discard: Some(DiscardLimits::single(0)),
                    write_zeroes: Some(WriteZeroes {
                        blocks: 0,
                        unmap: true,
                    }),
                    ..Features::default()
                },
            },
            chunks: BTreeMap::new(),
            limit,
        })
    }

    /// The chunks a checked `len`-byte transfer at `lba` touches: each
    /// chunk's index, the offset in it, and the range of the transfer.
    pub(super) fn pieces(
        lba: u64,
        len: usize,
    ) -> impl Iterator<Item = (u64, usize, core::ops::Range<usize>)> {
        let start = lba * u64::from(Self::BLOCK);
        let mut done = 0;
        core::iter::from_fn(move || {
            (done < len).then(|| {
                let at = start + done as u64;
                let within = (at % Self::CHUNK as u64) as usize;
                let n = (Self::CHUNK - within).min(len - done);
                let piece = (at / Self::CHUNK as u64, within, done..done + n);
                done += n;
                piece
            })
        })
    }
}

impl BlockDevice for Volatile {
    fn identity(&self) -> &Identity {
        &self.identity
    }

    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.identity.check(lba, buffer.len())?;
        for (chunk, within, range) in Self::pieces(lba, buffer.len()) {
            let n = range.len();
            match self.chunks.get(&chunk) {
                Some(bytes) => buffer[range].copy_from_slice(&bytes[within..within + n]),
                None => buffer[range].fill(0),
            }
        }
        Ok(())
    }

    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.identity.check_write(lba, data.len())?;
        let written = |range: &core::ops::Range<usize>| data[range.clone()].iter().any(|&b| b != 0);
        let new = Self::pieces(lba, data.len())
            .filter(|(chunk, _, range)| !self.chunks.contains_key(chunk) && written(range))
            .count();
        if (self.chunks.len() + new) * Self::CHUNK > self.limit {
            return Err(BlockError::Unsupported(
                "the RAM-backed disk has used all the memory it may take",
            ));
        }
        for (chunk, within, range) in Self::pieces(lba, data.len()) {
            let n = range.len();
            if let Some(bytes) = self.chunks.get_mut(&chunk) {
                bytes[within..within + n].copy_from_slice(&data[range]);
            } else if written(&range) {
                let mut bytes = vec![0; Self::CHUNK].into_boxed_slice();
                bytes[within..within + n].copy_from_slice(&data[range]);
                self.chunks.insert(chunk, bytes);
            }
        }
        Ok(())
    }

    /// Zeroed chunks give their memory back.
    fn discard(&mut self, lba: u64, blocks: u64) -> Result<(), BlockError> {
        self.write_zeroes(lba, blocks, true)
    }

    fn write_zeroes(&mut self, lba: u64, blocks: u64, _unmap: bool) -> Result<(), BlockError> {
        let len = usize::try_from(blocks)
            .ok()
            .and_then(|blocks| blocks.checked_mul(Self::BLOCK as usize))
            .ok_or_else(|| range(&self.identity, lba, blocks))?;
        self.identity.check_write(lba, len)?;
        for (chunk, within, range) in Self::pieces(lba, len) {
            if range.len() == Self::CHUNK {
                self.chunks.remove(&chunk);
            } else if let Some(bytes) = self.chunks.get_mut(&chunk) {
                bytes[within..within + range.len()].fill(0);
            }
        }
        Ok(())
    }
}

/// A RAM disk for the partition and selection tests.
#[cfg(any(test, feature = "ram-disk"))]
pub struct RamDisk {
    pub identity: Identity,
    pub bytes: Vec<u8>,
    pub flushes: usize,
}

#[cfg(any(test, feature = "ram-disk"))]
impl RamDisk {
    pub fn new(serial: &str, blocks: u64, block_size: u32) -> Self {
        Self {
            identity: Identity {
                transport: Transport::VirtioBlk,
                location: String::from("test"),
                model: String::from("RAM"),
                serial: String::from(serial),
                blocks,
                block_size,
                read_only: false,
                features: Features::default(),
            },
            bytes: vec![0; (blocks * u64::from(block_size)) as usize],
            flushes: 0,
        }
    }

    pub(super) fn span(&self, lba: u64, bytes: usize) -> core::ops::Range<usize> {
        let start = (lba * u64::from(self.identity.block_size)) as usize;
        start..start + bytes
    }
}

#[cfg(any(test, feature = "ram-disk"))]
impl BlockDevice for RamDisk {
    fn identity(&self) -> &Identity {
        &self.identity
    }

    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.identity.check(lba, buffer.len())?;
        buffer.copy_from_slice(&self.bytes[self.span(lba, buffer.len())]);
        Ok(())
    }

    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.identity.check_write(lba, data.len())?;
        let span = self.span(lba, data.len());
        self.bytes[span].copy_from_slice(data);
        Ok(())
    }

    /// Counted, for a test that reports a write cache.
    fn flush_cache(&mut self) -> Result<(), BlockError> {
        self.flushes += 1;
        Ok(())
    }

    fn write_fua(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.write(lba, data)
    }
}

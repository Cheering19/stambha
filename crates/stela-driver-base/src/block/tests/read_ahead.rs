//! The read-ahead a reader in bounded turns streams from: a slow disk's
//! next chunks in flight between the turns, each block read once.
use super::*;

/// A read of `blocks` 512-byte blocks from `lba`, as the disk is given it.
fn read(lba: u64, blocks: usize) -> Command {
    Command::Read {
        lba,
        bytes: blocks * 512,
    }
}

/// A reader that catches up with the read-ahead waits, within its bound,
/// for the chunk in flight that holds what it reads, and never reads those
/// blocks again; what arrives meanwhile stays for the read asked again.
#[test]
fn a_reader_that_catches_up_waits_for_the_chunk_in_flight() {
    let _clock = crate::test_clock::virtualize();
    let (delayed, (released, reads, given)) = Delayed::new(numbered(16), 4);
    released.set(usize::MAX);
    let device = Bounded::new(Box::new(delayed));
    let bound = device.bound();
    let ahead = Ahead::new(ReadAhead::bounded(device, 1024, 2));
    let mut reader = ahead.clone();
    let mut block = [0; 512];
    reader.read(0, &mut block).unwrap();
    assert_eq!(reads.get(), 1, "an unbounded miss reads directly");
    released.set(0);
    ahead.pump();
    assert_eq!(*given.borrow(), [read(1, 2), read(3, 2)]);
    bound.set(1_000);
    let mut two = [0; 1024];
    assert_eq!(reader.read(1, &mut two), Err(BlockError::Timeout));
    assert!(bound.blocked(), "a wait past the bound is blocked");
    assert_eq!(
        given.borrow().len(),
        2,
        "the chunk in flight is not read again"
    );
    released.set(usize::MAX);
    ahead.pump();
    bound.set(1_000);
    reader.read(1, &mut two).unwrap();
    assert_eq!((two[0], two[512]), (1, 2));
    reader.read(3, &mut two).unwrap();
    assert_eq!((two[0], two[512]), (3, 4));
    assert!(!bound.blocked());
    assert_eq!(reads.get(), 1);
    assert_eq!(
        given.borrow()[..2],
        [read(1, 2), read(3, 2)],
        "each read once"
    );
}

/// A disk whose queued commands complete only once looked at `after`
/// times: one a reader with no bound to wait within must not wait for.
struct Slow {
    inner: Delayed,
    released: Rc<Cell<usize>>,
    looks: Rc<Cell<usize>>,
    after: usize,
}

impl BlockDevice for Slow {
    fn identity(&self) -> &Identity {
        self.inner.identity()
    }
    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.inner.read(lba, buffer)
    }
    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.inner.write(lba, data)
    }
    fn depth(&self) -> usize {
        self.inner.depth()
    }
    fn room(&self) -> bool {
        self.inner.room()
    }
    fn start(&mut self, command: Command) -> Result<u32, BlockError> {
        self.inner.start(command)
    }
    fn finish(&mut self, tag: u32) -> Option<Finished> {
        self.looks.set(self.looks.get() + 1);
        if self.looks.get() >= self.after {
            self.released.set(usize::MAX);
        }
        self.inner.finish(tag)
    }
}

/// With no bound set, as before a turn bounds it, a read that catches up
/// with a chunk in flight takes it only if it has arrived and otherwise
/// reads directly, as an unbounded read does, rather than wait for it
/// without any limit.
#[test]
fn a_reader_with_no_bound_reads_directly_rather_than_wait_for_a_chunk() {
    let (delayed, (released, reads, _)) = Delayed::new(numbered(16), 4);
    released.set(usize::MAX);
    let looks = Rc::new(Cell::new(0));
    let slow = Slow {
        inner: delayed,
        released: released.clone(),
        looks: looks.clone(),
        after: 1_000,
    };
    let ahead = Ahead::new(ReadAhead::bounded(Bounded::new(Box::new(slow)), 1024, 2));
    let mut reader = ahead.clone();
    let mut block = [0; 512];
    reader.read(0, &mut block).unwrap();
    released.set(0);
    ahead.pump();
    let mut two = [0; 1024];
    reader.read(1, &mut two).unwrap();
    assert_eq!((two[0], two[512]), (1, 2));
    assert_eq!(reads.get(), 2, "read directly");
    assert_eq!(looks.get(), 1, "the chunk in flight was looked at once");
}

/// A caller whose reads in one turn ran past their bound reads them again
/// from the first: the chunks its last read took stay for it.
#[test]
fn a_caller_reading_again_after_its_bound_finds_its_last_read_in_memory() {
    let _clock = crate::test_clock::virtualize();
    let (delayed, (released, _, given)) = Delayed::new(numbered(16), 4);
    released.set(usize::MAX);
    let device = Bounded::new(Box::new(delayed));
    let bound = device.bound();
    let ahead = Ahead::new(ReadAhead::bounded(device, 1024, 2));
    let mut reader = ahead.clone();
    let mut block = [0; 512];
    reader.read(0, &mut block).unwrap();
    // The disk moves the first chunk only.
    released.set(0);
    ahead.pump();
    released.set(1);
    ahead.pump();
    bound.set(1_000);
    let mut two = [0; 1024];
    reader.read(1, &mut two).unwrap();
    assert_eq!(reader.read(3, &mut two), Err(BlockError::Timeout));
    // The next turn reads both again.
    released.set(usize::MAX);
    ahead.pump();
    bound.set(1_000);
    reader.read(1, &mut two).unwrap();
    assert_eq!((two[0], two[512]), (1, 2));
    reader.read(3, &mut two).unwrap();
    assert_eq!((two[0], two[512]), (3, 4));
    assert_eq!(
        given.borrow()[..2],
        [read(1, 2), read(3, 2)],
        "nothing read twice"
    );
    assert!(
        !given.borrow()[2..].contains(&read(1, 2)),
        "{:?}",
        given.borrow()
    );
}

/// A read served in part from memory whose rest misses keeps the chunk it
/// was served from until the rest is read: a rest that ran past its bound
/// is read again alone, not with the part memory held.
#[test]
fn a_read_served_in_part_keeps_its_chunk_until_its_rest_is_read() {
    let _clock = crate::test_clock::virtualize();
    let (delayed, (released, _, given)) = Delayed::new(numbered(16), 4);
    released.set(usize::MAX);
    let device = Bounded::new(Box::new(delayed));
    let bound = device.bound();
    let ahead = Ahead::new(ReadAhead::bounded(device, 1024, 1));
    let mut reader = ahead.clone();
    let mut block = [0; 512];
    reader.read(0, &mut block).unwrap();
    ahead.pump();
    ahead.pump();
    assert_eq!(*given.borrow(), [read(1, 2)]);
    released.set(0);
    bound.set(1_000);
    let mut four = [0; 2048];
    assert_eq!(reader.read(1, &mut four), Err(BlockError::Timeout));
    released.set(usize::MAX);
    bound.set(1_000);
    reader.read(1, &mut four).unwrap();
    assert_eq!((four[0], four[512], four[1024], four[1536]), (1, 2, 3, 4));
    assert_eq!(
        given.borrow()[1..],
        [read(3, 2)],
        "the rest read once, alone"
    );
}

/// A reader that jumps elsewhere, as one serving two places in turn, is
/// read ahead again only once it reads on from where it landed, so a jump
/// costs no chunks read for nothing.
#[test]
fn a_reader_that_jumps_is_read_ahead_once_it_streams_again() {
    let (delayed, (released, reads, given)) = Delayed::new(numbered(32), 8);
    released.set(usize::MAX);
    let ahead = Ahead::new(ReadAhead::new(Box::new(delayed), 1024, 2));
    let mut reader = ahead.clone();
    let mut block = [0; 512];
    reader.read(0, &mut block).unwrap();
    ahead.pump();
    assert_eq!(*given.borrow(), [read(1, 2), read(3, 2)]);
    reader.read(20, &mut block).unwrap();
    assert_eq!((block[0], reads.get()), (20, 2), "a jump reads directly");
    ahead.pump();
    assert_eq!(given.borrow().len(), 2, "a jump is not read ahead");
    reader.read(21, &mut block).unwrap();
    assert_eq!((block[0], reads.get()), (21, 3));
    ahead.pump();
    assert_eq!(given.borrow()[2..], [read(22, 2), read(24, 2)]);
    ahead.pump();
    let mut two = [0; 1024];
    reader.read(22, &mut two).unwrap();
    assert_eq!((two[0], two[512], reads.get()), (22, 23, 3), "found ahead");
}

/// A disk whose block `bad` cannot be read, queued or directly.
struct Faulty {
    inner: Delayed,
    bad: u64,
    /// The started commands that reach `bad`, by tag.
    failing: Vec<u32>,
}

const FAULT: BlockError = BlockError::Device { status: 3 };

impl Faulty {
    fn reaches(&self, lba: u64, bytes: usize) -> bool {
        (lba..lba + (bytes / 512) as u64).contains(&self.bad)
    }
}

impl BlockDevice for Faulty {
    fn identity(&self) -> &Identity {
        self.inner.identity()
    }
    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        match self.reaches(lba, buffer.len()) {
            true => Err(FAULT),
            false => self.inner.read(lba, buffer),
        }
    }
    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.inner.write(lba, data)
    }
    fn depth(&self) -> usize {
        self.inner.depth()
    }
    fn room(&self) -> bool {
        self.inner.room()
    }
    fn start(&mut self, command: Command) -> Result<u32, BlockError> {
        let fails = matches!(command, Command::Read { lba, bytes } if self.reaches(lba, bytes));
        let tag = self.inner.start(command)?;
        if fails {
            self.failing.push(tag);
        }
        Ok(tag)
    }
    fn finish(&mut self, tag: u32) -> Option<Finished> {
        let finished = self.inner.finish(tag)?;
        Some(match self.failing.contains(&tag) {
            true => Err(FAULT),
            false => finished,
        })
    }
}

/// A chunk that fails is read again directly, so the reader gets the
/// disk's own error, bounded or not, and reads on past it.
#[test]
fn a_failed_chunk_is_read_again_and_its_error_reaches_the_reader() {
    let _clock = crate::test_clock::virtualize();
    for bounded in [false, true] {
        let (delayed, (released, _, _)) = Delayed::new(numbered(16), 4);
        released.set(usize::MAX);
        let faulty = Faulty {
            inner: delayed,
            bad: 4,
            failing: Vec::new(),
        };
        let device = Bounded::new(Box::new(faulty));
        let bound = device.bound();
        let ahead = Ahead::new(ReadAhead::bounded(device, 1024, 2));
        let mut reader = ahead.clone();
        let mut block = [0; 512];
        reader.read(0, &mut block).unwrap();
        released.set(0);
        ahead.pump();
        released.set(usize::MAX);
        if bounded {
            bound.set(1_000);
        } else {
            ahead.pump();
        }
        let mut two = [0; 1024];
        reader.read(1, &mut two).unwrap();
        assert_eq!((two[0], two[512]), (1, 2));
        assert_eq!(reader.read(3, &mut two), Err(FAULT), "bounded={bounded}");
        assert!(!bound.blocked(), "a failure is not a blocked read");
        reader.read(5, &mut two).unwrap();
        assert_eq!((two[0], two[512]), (5, 6));
    }
}

/// A disk that moves one queued command a tick, in order, as a stick
/// moves one read while its reader is away; a tick it has nothing queued
/// for moves nothing.
struct Paced {
    disk: RamDisk,
    /// Each command not yet done: its tag, the command and its start tick.
    queued: VecDeque<(u32, Command, u64)>,
    done: BTreeMap<u32, Finished>,
    next: u32,
    now: Rc<Cell<u64>>,
    /// The tick the last command was done at.
    last: u64,
}

impl Paced {
    fn advance(&mut self) {
        while let Some((_, _, started)) = self.queued.front() {
            let at = (*started).max(self.last) + 1;
            if at > self.now.get() {
                return;
            }
            let (tag, command, _) = self.queued.pop_front().expect("looked at above");
            self.done.insert(tag, self.disk.execute(command));
            self.last = at;
        }
    }
}

impl BlockDevice for Paced {
    fn identity(&self) -> &Identity {
        &self.disk.identity
    }
    fn read(&mut self, _lba: u64, _buffer: &mut [u8]) -> Result<(), BlockError> {
        panic!("a bounded reader reads this disk only through its queue");
    }
    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.disk.write(lba, data)
    }
    fn depth(&self) -> usize {
        8
    }
    fn room(&self) -> bool {
        self.queued.len() < 8
    }
    fn start(&mut self, command: Command) -> Result<u32, BlockError> {
        let tag = self.next;
        self.next += 1;
        self.queued.push_back((tag, command, self.now.get()));
        Ok(tag)
    }
    fn finish(&mut self, tag: u32) -> Option<Finished> {
        self.advance();
        self.done.remove(&tag)
    }
}

/// The turns a reader that reads one 2-block chunk a turn, its reads
/// bounded, takes for the first 32 blocks of a [`Paced`] disk, read ahead
/// four chunks or not at all.
fn turns_to_read(read_ahead: bool) -> u64 {
    let _clock = crate::test_clock::virtualize();
    let now = Rc::new(Cell::new(0));
    let paced = Paced {
        disk: numbered(64),
        queued: VecDeque::new(),
        done: BTreeMap::new(),
        next: 0,
        now: now.clone(),
        last: 0,
    };
    let device = Bounded::new(Box::new(paced));
    let bound = device.bound();
    let (mut reader, ahead): (Box<dyn BlockDevice>, _) = match read_ahead {
        true => {
            let ahead = Ahead::new(ReadAhead::bounded(device, 1024, 4));
            (Box::new(ahead.clone()), Some(ahead))
        }
        false => (Box::new(device), None),
    };
    let (mut lba, mut turns) = (0, 0);
    while lba < 32 {
        turns += 1;
        assert!(turns < 1_000, "the reader made no progress");
        if let Some(ahead) = &ahead {
            ahead.pump();
        }
        bound.set(1_000);
        let mut two = [0; 1024];
        match reader.read(lba, &mut two) {
            Ok(()) => {
                assert_eq!((u64::from(two[0]), u64::from(two[512])), (lba, lba + 1));
                lba += 2;
            }
            Err(error) => assert_eq!(error, BlockError::Timeout),
        }
        // The disk works on while the reader is away.
        now.set(now.get() + 1);
    }
    turns
}

/// Read ahead, a disk that moves a chunk a tick keeps moving the next ones
/// while its reader is away, and the reader takes one a turn; read only
/// when asked, it idles from each read's end to the next turn's ask, and
/// the reader takes one every other turn.
#[test]
fn a_slow_disk_read_ahead_keeps_working_between_its_readers_turns() {
    let (asked, ahead) = (turns_to_read(false), turns_to_read(true));
    assert_eq!(asked, 32, "two turns a chunk");
    assert!(ahead <= 17, "one turn a chunk: {ahead}");
}

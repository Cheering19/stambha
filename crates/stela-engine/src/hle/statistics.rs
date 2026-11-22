//! What a machine's HLE did: the compiles whose entry matched a signature,
//! the composites installed, the calls whose native prefix faulted and ran
//! the original, and the matches it refused, by reason.

/// Why HLE left a function translated after finding it in the table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal {
    /// A host breakpoint or trace point lies inside the function.
    Breakpoint,
    /// A candidate's function would run past its entry's 4 KiB fetch page,
    /// which a composite cannot index (counted before its bytes are read).
    CrossesPage,
    /// Its entry is a device site.
    DeviceSite,
    /// A word of it is one the chip lacks, which it runs as UDF or a NOP
    /// (`Chip::admit`): the original would not run the bytes its signature
    /// was made from.
    Admitted,
    /// Its first instruction did not compile alone into the bail path.
    Bail,
    /// The composite block could not be laid out.
    Compose,
}

impl Refusal {
    pub const COUNT: usize = 6;
    pub const ALL: [Refusal; Self::COUNT] = [
        Self::Breakpoint,
        Self::CrossesPage,
        Self::DeviceSite,
        Self::Admitted,
        Self::Bail,
        Self::Compose,
    ];

    pub const fn name(self) -> &'static str {
        match self {
            Self::Breakpoint => "breakpoint",
            Self::CrossesPage => "crosses-page",
            Self::DeviceSite => "device-site",
            Self::Admitted => "admitted",
            Self::Bail => "bail",
            Self::Compose => "compose",
        }
    }
}

const _: () = assert!(Refusal::Compose as usize + 1 == Refusal::COUNT);

/// One machine's HLE counts since it started.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Statistics {
    /// Compiles of an entry whose function matched an enabled signature.
    pub matched: u64,
    /// The table's signatures that matched, a bit per index.
    pub matched_signatures: u64,
    /// Composite blocks installed.
    pub installed: u64,
    /// Calls whose native prefix faulted, so the original ran them.
    pub bypassed: u64,
    /// Refusals by [`Refusal`].
    pub refused: [u64; Refusal::COUNT],
}

impl Statistics {
    pub fn refusals(&self, refusal: Refusal) -> u64 {
        self.refused[refusal as usize]
    }

    pub(crate) fn refuse(&mut self, refusal: Refusal) {
        self.refused[refusal as usize] += 1;
    }
}

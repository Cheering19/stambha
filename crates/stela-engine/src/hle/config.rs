//! What a boot's HLE matches: a table of signatures, the routines of it
//! that are on, and the most bytes one native call may move, fill, compare
//! or scan.
use super::{Id, Signature, signatures::SIGNATURES};
use alloc::{format, string::String, sync::Arc, vec::Vec};

/// The signatures a table may hold, so a 64-bit set names those that
/// matched.
const SIGNATURES_PER_TABLE: usize = 64;

/// A machine's HLE configuration. The default is [`Config::off`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Config {
    /// The routines matched, a bit each ([`Id::bit`]): only routines with a
    /// signature in `table`, so 0 when it is empty.
    pub enabled: u32,
    /// The most bytes one native call works on; a larger call runs the
    /// original.
    pub max_bytes: u32,
    pub table: Arc<[Signature]>,
}

impl Default for Config {
    fn default() -> Self {
        Self::off()
    }
}

impl Config {
    /// The default [`Self::max_bytes`]: a few microseconds of copying.
    pub const MAX_BYTES: u32 = 65_536;
    /// What [`Self::with_max_bytes`] takes: from the routines' largest
    /// unrolled size class to about 1 MiB.
    const BOUNDS: core::ops::RangeInclusive<u32> = 256..=1 << 20;

    /// No HLE: nothing matches, so every function is translated.
    pub fn off() -> Self {
        Self {
            enabled: 0,
            max_bytes: Self::MAX_BYTES,
            table: Arc::from([]),
        }
    }

    /// A boot's configuration: off without `--hle` (`on`); with it, every
    /// routine of the generated table but those `--hle-off` names (`off`).
    pub fn boot(on: bool, off: &[Id]) -> Self {
        booted(on, off, Arc::from(SIGNATURES))
    }

    /// `table`, with those of `ids` that have a signature in it enabled.
    pub fn with_table(table: Arc<[Signature]>, ids: &[Id]) -> Self {
        Self::of(bits(ids), table)
    }

    /// `self` with native calls bounded at `bytes`.
    pub fn with_max_bytes(self, bytes: u32) -> Self {
        assert!(
            Self::BOUNDS.contains(&bytes),
            "HLE's bound on a call is 256 bytes to 1 MiB, not {bytes}"
        );
        Self {
            max_bytes: bytes,
            ..self
        }
    }

    pub fn enables(&self, id: Id) -> bool {
        self.enabled & id.bit() != 0
    }

    /// The boot's line after `HLE `, for a configuration `--hle` turned on
    /// with `off` named by `--hle-off`: `on signatures=K ids=memmove,bzero
    /// off=strlen max=65536`, `-` for an empty list. With no signatures, it
    /// says that nothing can match.
    pub fn describe(&self, off: &[Id]) -> String {
        let names = |ids: &mut dyn Iterator<Item = Id>| {
            let names: Vec<&str> = ids.map(Id::name).collect();
            match names.is_empty() {
                true => String::from("-"),
                false => names.join(","),
            }
        };
        let ids = names(&mut Id::PRODUCTION.into_iter().filter(|id| self.enables(*id)));
        let off = names(&mut off.iter().copied());
        let mut line = format!(
            "on signatures={} ids={ids} off={off} max={}",
            self.table.len(),
            self.max_bytes
        );
        if self.enabled == 0 {
            line.push_str(": no signature is enabled, so no function is replaced");
        }
        line
    }

    fn of(ids: u32, table: Arc<[Signature]>) -> Self {
        assert!(
            table.len() <= SIGNATURES_PER_TABLE,
            "an HLE table holds at most {SIGNATURES_PER_TABLE} signatures"
        );
        let held = table
            .iter()
            .fold(0, |held, signature| held | signature.id().bit());
        Self {
            enabled: ids & held,
            max_bytes: Self::MAX_BYTES,
            table,
        }
    }
}

/// [`Config::boot`] over `table`.
pub(super) fn booted(on: bool, off: &[Id], table: Arc<[Signature]>) -> Config {
    match on {
        true => Config::of(!bits(off), table),
        false => Config::off(),
    }
}

fn bits(ids: &[Id]) -> u32 {
    ids.iter().fold(0, |bits, id| bits | id.bit())
}

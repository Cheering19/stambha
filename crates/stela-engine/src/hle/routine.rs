//! The native routines a matched function can be replaced with.

/// A native routine, by the C function whose contract it keeps. Its bit in a
/// configuration's enabled set is [`Self::bit`]; its name is the one
/// `--hle-off` takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Routine {
    Memmove,
    Bzero,
    Memset,
    Memcmp,
    Strlen,
    /// For tests: a fast path that only bails to the original.
    #[cfg(any(test, feature = "test-support"))]
    Bail,
    /// For tests: a fast path that loads one byte at X0, then bails.
    #[cfg(any(test, feature = "test-support"))]
    Touch,
}

impl Routine {
    /// The routines a boot can enable, in bit order.
    pub const PRODUCTION: [Routine; 5] = [
        Self::Memmove,
        Self::Bzero,
        Self::Memset,
        Self::Memcmp,
        Self::Strlen,
    ];

    pub const fn name(self) -> &'static str {
        match self {
            Self::Memmove => "memmove",
            Self::Bzero => "bzero",
            Self::Memset => "memset",
            Self::Memcmp => "memcmp",
            Self::Strlen => "strlen",
            #[cfg(any(test, feature = "test-support"))]
            Self::Bail => "bail",
            #[cfg(any(test, feature = "test-support"))]
            Self::Touch => "touch",
        }
    }

    /// The production routine named `name`: never a test routine.
    pub fn from_name(name: &str) -> Option<Self> {
        Self::PRODUCTION
            .into_iter()
            .find(|routine| routine.name() == name)
    }

    /// Its bit in a set of routines: `1 << index`.
    pub const fn bit(self) -> u32 {
        1 << self as u32
    }
}

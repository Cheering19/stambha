//! High-level emulation (HLE): native x86-64 replacements for hot guest
//! library functions (`memmove`, `memset`, `bzero`, ...), found by their
//! bytes when the engine compiles a function's entry and checked against
//! the translated originals.
//!
//! This module holds the parts that need no machine: the routine ids
//! ([`Id`]), the masked function signatures and the phase-1 eligibility of a
//! function ([`Signature`], [`eligible`]). None of it feeds the translation
//! store's translator id (`build.rs` hashes neither this module nor the
//! engine glue), so building it in leaves every stored translation valid.
mod routine;
mod signature;

pub use routine::Routine as Id;
pub use signature::{Ineligible, PREFIX_BYTES, Signature, eligible, mask_word, prefix_hash};

#[cfg(test)]
mod tests;

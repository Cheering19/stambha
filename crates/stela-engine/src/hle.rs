//! High-level emulation (HLE): native x86-64 replacements for hot guest
//! library functions (`memmove`, `memset`, `bzero`, ...), found by their
//! bytes when the engine compiles a function's entry and checked against
//! the translated originals.
//!
//! This module holds the parts that need no machine: the routine ids
//! ([`Id`]), the masked function signatures and the phase-1 eligibility of a
//! function ([`Signature`], [`eligible`]), a boot's configuration
//! ([`Config`]), off unless the boot passes `--hle`, and a machine's counts
//! ([`Statistics`]). The engine glue is `machine::engine::hle`. None of it
//! feeds the translation store's translator id (`build.rs` hashes neither
//! this module nor the engine glue), so building it in leaves every stored
//! translation valid.
mod config;
mod routine;
mod signature;
mod signatures;
mod statistics;

pub use config::Config;
pub use routine::Routine as Id;
pub use signature::{Ineligible, PREFIX_BYTES, Signature, eligible, find, mask_word, prefix_hash};
pub use statistics::{Refusal, Statistics};

#[cfg(test)]
mod tests;

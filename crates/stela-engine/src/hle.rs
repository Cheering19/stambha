//! High-level emulation (HLE): native x86-64 replacements for hot guest
//! library functions (`memmove`, `memset`, `bzero`, ...), found by their
//! bytes when the engine compiles a function's entry and checked against
//! the translated originals.
//!
//! This module holds the parts that need no machine: the masked function
//! signatures and the phase-1 eligibility of a function ([`Signature`],
//! [`eligible`]), a boot's configuration ([`Config`]), off unless the boot
//! passes `--hle`, and a machine's counts ([`Statistics`]). The engine glue
//! is `machine::engine::hle`. Neither feeds the translation store's
//! translator id (`build.rs` hashes neither), so changing them leaves every
//! stored translation valid. The routine ids ([`Id`]) are the compiler's
//! (`stela_compiler::x86_64::hle::Routine`), beside the composite blocks it
//! emits for them, which its fingerprint covers.
mod config;
mod signature;
mod signatures;
mod statistics;

pub use config::Config;
pub use signature::{Ineligible, PREFIX_BYTES, Signature, eligible, find, mask_word, prefix_hash};
pub use statistics::{Refusal, Statistics};
pub use stela_compiler::x86_64::hle::Routine as Id;

#[cfg(test)]
mod tests;

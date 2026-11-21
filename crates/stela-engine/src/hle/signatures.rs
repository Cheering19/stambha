//! The signatures of the guest functions HLE replaces, generated offline
//! from a macOS build's dyld shared cache, one line per function naming its
//! symbol, image and build. A line holds the function's routine, length,
//! exit offset and hashes ([`Signature::generated`]), never its bytes:
//! Stela contains no Apple software. A function joins the table only once
//! its routine has agreed with it on the real bytes, so the table is empty
//! until then and `--hle` matches nothing.
use super::Signature;

pub static SIGNATURES: &[Signature] = &[];

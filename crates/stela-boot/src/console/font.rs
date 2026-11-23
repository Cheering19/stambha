//! The console's bitmap font: Spleen 2.1.0 8x16, Copyright (c) 2018-2024,
//! Frederic Cambus (BSD-2-Clause, `LICENSES/BSD-2-Clause.txt`), in the Linux
//! console's PSF1 format with its Unicode table, which
//! `stela_boot_core::psf` reads.
pub use stela_boot_core::psf::Font;

/// The one font, drawn at whole multiples of its pixels.
static SPLEEN: &[u8] = include_bytes!("spleen-8x16.psfu");

pub fn console() -> Option<Font> {
    Font::parse(SPLEEN)
}

//! A bitmap font in the Linux console's PSF1 or PSF2 format, as the boot
//! console and the startup pages draw it (`stela-boot`'s `console::font`).
//! Each glyph is `height` rows of `width` bits, MSB first, each row padded
//! to whole bytes. The first 128 glyphs are ASCII in code point order; any
//! other character is found in the font's Unicode table ([`Font::index`]).

/// One font, parsed from its PSF file.
pub struct Font {
    pub width: usize,
    pub height: usize,
    row_bytes: usize,
    glyph_bytes: usize,
    count: usize,
    glyphs: &'static [u8],
    /// The Unicode table after the glyphs, when the file has one.
    table: Option<Table>,
}

#[derive(Clone, Copy)]
enum Table {
    /// PSF1: UCS-2 little-endian values, each glyph's ended by `0xFFFF`,
    /// with `0xFFFE` starting the sequences that follow its characters.
    Psf1(&'static [u8]),
    /// PSF2: UTF-8, each glyph's ended by `0xFF`, with `0xFE` starting its
    /// sequences.
    Psf2(&'static [u8]),
}

const PSF1_MAGIC: [u8; 2] = [0x36, 0x04];
const PSF1_512: u8 = 1;
const PSF1_TABLE: u8 = 2;
const PSF2_MAGIC: [u8; 4] = [0x72, 0xb5, 0x4a, 0x86];
const PSF2_TABLE: usize = 1;
/// The glyphs a font must have: printable ASCII and below.
const GLYPHS: usize = 0x80;

fn word(file: &[u8], offset: usize) -> Option<usize> {
    let bytes = file.get(offset..offset + 4)?;
    Some(u32::from_le_bytes(bytes.try_into().ok()?) as usize)
}

impl Font {
    pub fn parse(file: &'static [u8]) -> Option<Self> {
        let (width, height, header, count, glyph_bytes, table) = if file.starts_with(&PSF1_MAGIC) {
            let mode = *file.get(2)?;
            let height = usize::from(*file.get(3)?);
            let count = if mode & PSF1_512 != 0 { 512 } else { 256 };
            (8, height, 4, count, height, mode & PSF1_TABLE != 0)
        } else if file.starts_with(&PSF2_MAGIC) {
            (
                word(file, 28)?,
                word(file, 24)?,
                word(file, 8)?,
                word(file, 16)?,
                word(file, 20)?,
                word(file, 12)? & PSF2_TABLE != 0,
            )
        } else {
            return None;
        };
        let row_bytes = width.div_ceil(8);
        if width == 0 || height == 0 || glyph_bytes != row_bytes * height || count < GLYPHS {
            return None;
        }
        let end = header.checked_add(count.checked_mul(glyph_bytes)?)?;
        let glyphs = file.get(header..end)?;
        let table = match (table, file.get(end..)) {
            (true, Some(rest)) if file.starts_with(&PSF1_MAGIC) => Some(Table::Psf1(rest)),
            (true, Some(rest)) => Some(Table::Psf2(rest)),
            _ => None,
        };
        Some(Self {
            width,
            height,
            row_bytes,
            glyph_bytes,
            count,
            glyphs,
            table,
        })
    }

    /// Whether pixel (`x`, `y`) of `character`'s glyph is set; anything
    /// outside ASCII draws as `?`.
    pub fn pixel(&self, character: u8, x: usize, y: usize) -> bool {
        let index = if usize::from(character) < GLYPHS {
            character
        } else {
            b'?'
        };
        self.set(usize::from(index), x, y)
    }

    /// The glyph that draws `character`: ASCII by its code point, anything
    /// else by the font's Unicode table, and `?` when the font has none.
    pub fn index(&self, character: char) -> usize {
        let code = u32::from(character);
        if code < GLYPHS as u32 {
            return code as usize;
        }
        self.table
            .and_then(|table| table.find(character))
            .filter(|&index| index < self.count)
            .unwrap_or(usize::from(b'?'))
    }

    /// Whether pixel (`x`, `y`) of glyph `index` ([`Self::index`]) is set.
    pub fn set(&self, index: usize, x: usize, y: usize) -> bool {
        let row = index * self.glyph_bytes + y * self.row_bytes;
        self.glyphs[row + x / 8] & (0x80 >> (x % 8)) != 0
    }
}

impl Table {
    /// The first glyph whose own characters, not its sequences, include
    /// `character`.
    fn find(self, character: char) -> Option<usize> {
        match self {
            Self::Psf1(table) => {
                let wanted = u16::try_from(u32::from(character)).ok()?;
                let mut glyph = 0;
                let mut sequences = false;
                for pair in table.as_chunks::<2>().0 {
                    match u16::from_le_bytes(*pair) {
                        0xffff => {
                            glyph += 1;
                            sequences = false;
                        }
                        0xfffe => sequences = true,
                        value if !sequences && value == wanted => return Some(glyph),
                        _ => {}
                    }
                }
                None
            }
            Self::Psf2(table) => table.split(|&byte| byte == 0xff).position(|entry| {
                let own = entry.split(|&byte| byte == 0xfe).next().unwrap_or(&[]);
                core::str::from_utf8(own).is_ok_and(|own| own.contains(character))
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The boot console's font.
    static SPLEEN: &[u8] = include_bytes!("../../stela-boot/src/console/spleen-8x16.psfu");

    fn font() -> Font {
        Font::parse(SPLEEN).expect("Spleen parses")
    }

    fn ink(font: &Font, index: usize) -> usize {
        (0..font.height)
            .flat_map(|y| (0..font.width).map(move |x| (x, y)))
            .filter(|&(x, y)| font.set(index, x, y))
            .count()
    }

    #[test]
    fn ascii_draws_by_its_code_point() {
        let font = font();
        assert_eq!((font.width, font.height), (8, 16));
        assert_eq!(font.index('A'), usize::from(b'A'));
        for y in 0..16 {
            for x in 0..8 {
                assert_eq!(font.pixel(b'A', x, y), font.set(usize::from(b'A'), x, y));
            }
        }
    }

    #[test]
    fn other_characters_draw_by_the_unicode_table() {
        let font = font();
        // The middle dot: a few pixels in the middle rows.
        let dot = font.index('\u{b7}');
        assert_ne!(dot, usize::from(b'?'));
        assert!((1..=9).contains(&ink(&font, dot)));
        assert!(
            (0..16)
                .filter(|y| !(5..11).contains(y))
                .all(|y| (0..8).all(|x| !font.set(dot, x, y)))
        );
        // An accented letter has a glyph of its own, unlike its bare letter.
        let e = font.index('\u{e9}');
        assert_ne!(e, usize::from(b'?'));
        assert_ne!(e, usize::from(b'e'));
    }

    #[test]
    fn a_character_the_font_lacks_draws_as_a_question_mark() {
        assert_eq!(font().index('\u{1f600}'), usize::from(b'?'));
        // Bytes past ASCII still draw as `?` for the console.
        let font = font();
        for y in 0..16 {
            assert_eq!(font.pixel(0xe9, 3, y), font.pixel(b'?', 3, y));
        }
    }

    #[test]
    fn a_file_that_is_not_a_font_is_refused() {
        assert!(Font::parse(b"not a font").is_none());
        assert!(Font::parse(&SPLEEN[..100]).is_none());
    }
}

//! A host painter for the page's scenes, as `stela-boot`'s `ui::Canvas`
//! paints them at scale 1: its colours, rounded panels, the console font
//! (`psf`) and the header. The tests check every screen paints; with
//! `STELA_WIFI_PAGE_SHOTS=DIR` they also write each screen to `DIR` as a
//! PPM, to look at.
extern crate std;
use super::*;
use std::vec::Vec as StdVec;

/// The console's font (`stela-boot`'s `console::font`).
static SPLEEN: &[u8] = include_bytes!("../../../../stela-boot/src/console/spleen-8x16.psfu");

fn font() -> crate::psf::Font {
    crate::psf::Font::parse(SPLEEN).expect("Spleen parses")
}

/// `ui`'s colours.
fn rgb(tone: Tone) -> u32 {
    match tone {
        Tone::Background => 0x18191c,
        Tone::Card => 0x222428,
        Tone::Raised => 0x2c2e34,
        Tone::Text => 0xe6e7ea,
        Tone::Dim => 0x8b8e96,
        Tone::Orange => 0xf0883a,
        Tone::Blue => 0x3f86e8,
        Tone::Red => 0xe4574d,
    }
}

fn glyph(character: char, x: usize, y: usize) -> bool {
    let font = font();
    font.set(font.index(character), x, y)
}

struct Painter {
    width: usize,
    height: usize,
    pixels: StdVec<u32>,
}

impl Painter {
    fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            pixels: vec![rgb(Tone::Background); width * height],
        }
    }

    fn fill(&mut self, x: usize, y: usize, width: usize, height: usize, colour: u32) {
        let right = (x + width).min(self.width);
        for row in y..(y + height).min(self.height) {
            if x < right {
                self.pixels[row * self.width + x..row * self.width + right].fill(colour);
            }
        }
    }

    /// `Canvas::panel`.
    fn panel(&mut self, x: usize, y: usize, width: usize, height: usize, radius: usize, c: u32) {
        let radius = radius.min(width / 2).min(height / 2) as u64;
        for row in 0..height {
            let from_edge = row.min(height - 1 - row) as u64;
            let inset = match from_edge < radius {
                true => {
                    let dy = radius - from_edge;
                    (radius - (radius * radius - dy * dy).isqrt()) as usize
                }
                false => 0,
            };
            self.fill(x + inset, y + row, width.saturating_sub(2 * inset), 1, c);
        }
    }

    /// `Canvas::text`: one glyph a character.
    fn text(&mut self, x: usize, y: usize, text: &str, size: usize, colour: u32) {
        let font = font();
        for (index, character) in text.chars().enumerate() {
            let left = x + index * 8 * size;
            let glyph = font.index(character);
            for gy in 0..16 {
                for gx in 0..8 {
                    if font.set(glyph, gx, gy) {
                        self.fill(left + gx * size, y + gy * size, size, size, colour);
                    }
                }
            }
        }
    }

    fn centred(&mut self, middle: usize, y: usize, text: &str, colour: u32) {
        let x = middle.saturating_sub(text.chars().count() * 8 / 2);
        self.text(x, y, text, 1, colour);
    }

    /// `Canvas::header`: the logo and the name, centred at the top.
    fn header(&mut self) {
        const LOGO: usize = 32;
        const NAME: &str = "Stambha";
        let left = self
            .width
            .saturating_sub(LOGO + 12 + NAME.chars().count() * 16)
            / 2;
        for py in 0..LOGO {
            for px in 0..LOGO {
                if let Some(colour) = crate::screen::logo_pixel(px as u32, py as u32, LOGO as u32) {
                    self.fill(left + px, 28 + py, 1, 1, colour);
                }
            }
        }
        let ink = |y: usize| NAME.chars().any(|c| (0..8).any(|x| glyph(c, x, y)));
        let first = (0..16).find(|&y| ink(y)).unwrap_or(0);
        let last = (0..16).rev().find(|&y| ink(y)).unwrap_or(15);
        let y = (28 + LOGO / 2).saturating_sub(first + last + 1);
        self.text(left + LOGO + 12, y, NAME, 2, rgb(Tone::Text));
    }

    /// The page as `wifi_menu` shows it: background, header, then the scene.
    fn page(scene: &Scene) -> Self {
        let mut painter = Self::new(SCREEN.width, SCREEN.height);
        painter.header();
        for draw in &scene.draws {
            match draw {
                Draw::Panel {
                    x,
                    y,
                    width,
                    height,
                    radius,
                    tone,
                } => painter.panel(*x, *y, *width, *height, *radius, rgb(*tone)),
                Draw::Text { x, y, text, tone } => painter.text(*x, *y, text, 1, rgb(*tone)),
            }
        }
        painter
    }

    fn ppm(&self) -> StdVec<u8> {
        let mut file = std::format!("P6\n{} {}\n255\n", self.width, self.height).into_bytes();
        for pixel in &self.pixels {
            file.extend_from_slice(&pixel.to_be_bytes()[1..]);
        }
        file
    }

    /// Pixels of `colour` in the band of rows from `top`, `rows` high.
    fn count(&self, top: usize, rows: usize, colour: u32) -> usize {
        self.pixels[top * self.width..(top + rows).min(self.height) * self.width]
            .iter()
            .filter(|&&pixel| pixel == colour)
            .count()
    }
}

/// The startup menu as `choice::Menu::draw` lays it out, with its Wi-Fi
/// line from [`status_line`]: two startup choices, the first counting
/// down, and the firmware's Windows entry under "This PC".
fn main_menu() -> Painter {
    let mut painter = Painter::new(SCREEN.width, SCREEN.height);
    painter.header();
    let (left, column) = (420, 440);
    let mut y = 112;
    for (index, name) in ["macOS Tahoe", "Install macOS", "Windows Boot Manager"]
        .into_iter()
        .enumerate()
    {
        if index == 2 {
            y += 16;
            painter.text(left + 16, y, "This PC", 1, rgb(Tone::Dim));
            y += 28;
        }
        if index == 0 {
            painter.panel(left, y, column, 32, 8, rgb(Tone::Raised));
            painter.panel(left + 6, y + 9, 3, 14, 1, rgb(Tone::Orange));
        }
        painter.text(left + 20, y + 8, name, 1, rgb(Tone::Text));
        y += 36;
    }
    let line = status_line(Some("GarvitRoom_5G"), None, true);
    painter.centred(640, 720 - 24, &line, rgb(Tone::Dim));
    painter.centred(640, 720 - 56, "macOS Tahoe in 5", rgb(Tone::Dim));
    painter
}

/// Each screen, named.
fn screens() -> StdVec<(&'static str, Painter)> {
    let networks = || {
        let mut networks = vec![configured("GarvitRoom_5G"), saved("Mom's phone")];
        networks.push(saved("Library guest"));
        networks.push(Network {
            usable: false,
            ..saved("Old router")
        });
        networks
    };
    let mut list = Page::new(networks(), true, Some("Mom's phone"));
    let list_shot = Painter::page(&list.scene(SCREEN));
    list.press(Key::Down);
    list.press(Key::Delete);
    let confirm = Painter::page(&list.scene(SCREEN));
    let mut add = Page::new(networks(), true, None);
    add.press(Key::End);
    add.press(Key::Enter);
    typed(&mut add, "Cafe 5G");
    add.press(Key::Enter);
    typed(&mut add, "coffee-time");
    let hidden = Painter::page(&add.scene(SCREEN));
    add.press(Key::Tab);
    let shown = Painter::page(&add.scene(SCREEN));
    add.press(Key::Up);
    for _ in 0..7 {
        add.press(Key::Backspace);
    }
    add.press(Key::Enter);
    let error = Painter::page(&add.scene(SCREEN));
    let mut scrolled = many(30);
    scrolled.press(Key::End);
    scrolled.press(Key::Up);
    let empty = Page::new(Vec::new(), false, None);
    StdVec::from([
        ("main-menu", main_menu()),
        ("panel", list_shot),
        ("forget-confirm", confirm),
        ("add-hidden", hidden),
        ("add-shown", shown),
        ("add-empty-name", error),
        ("panel-scrolled", Painter::page(&scrolled.scene(SCREEN))),
        ("panel-empty", Painter::page(&empty.scene(SCREEN))),
    ])
}

#[test]
fn every_screen_paints_under_the_header() {
    for (name, painter) in screens() {
        // The name's ink in the header, and text under it.
        assert!(painter.count(28, 32, rgb(Tone::Text)) > 0, "{name}");
        assert!(painter.count(112, 300, rgb(Tone::Text)) > 0, "{name}");
        if let Some(directory) = std::env::var_os("STELA_WIFI_PAGE_SHOTS") {
            let path = std::path::Path::new(&directory).join(std::format!("{name}.ppm"));
            std::fs::write(&path, painter.ppm()).expect("the screenshot is written");
        }
    }
}

#[test]
fn the_dialog_shows_its_error_in_red() {
    let shots = screens();
    let red = |name: &str| {
        let (_, painter) = shots.iter().find(|(shot, _)| *shot == name).expect(name);
        painter.count(112 + 32 + 212, 16, rgb(Tone::Red))
    };
    assert!(red("add-empty-name") > 0);
    assert_eq!(red("add-hidden"), 0);
}

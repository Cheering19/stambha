//! The page Install macOS opens before its disk picker on a PC whose
//! firmware shows no graphics card Stela can start macOS with
//! (`stela_boot_core::install_caution`): the reason, the question, and Back
//! and Install anyway, Back focused. This reads the keys and paints; the
//! page's state and words are the core's.
use crate::{
    choice::{Press, Wait, key},
    console::{Framebuffer, font},
    output,
    ui::{self, Canvas},
};
use core::sync::atomic::{AtomicBool, Ordering};
use stela_boot_core::install_caution::{ASK, Caution, Focus, Key, Step, TITLE, wrap};

/// Install anyway was chosen this boot: the page asks once.
static CONFIRMED: AtomicBool = AtomicBool::new(false);

/// Whether Install macOS goes on to its disk picker: at once when the
/// firmware shows a card Stela drives (`storage::install_caution`) or Install
/// anyway was chosen earlier this boot, else as the page is answered. Without
/// a framebuffer or keyboard to ask with it goes on, as it did before the
/// page existed.
pub(crate) fn confirmed() -> bool {
    if CONFIRMED.load(Ordering::Relaxed) {
        return true;
    }
    let Some(mut caution) = crate::storage::install_caution() else {
        return true;
    };
    let Some((screen, font)) = Framebuffer::from_firmware().zip(font::console()) else {
        return true;
    };
    let mut canvas = Canvas::new(screen, font);
    loop {
        draw(&mut canvas, &caution);
        canvas.show();
        let key = match key(None) {
            Wait::Key(Press::Left) => Key::Left,
            Wait::Key(Press::Right) => Key::Right,
            Wait::Key(Press::Character('\t')) => Key::Tab,
            Wait::Key(Press::Enter) => Key::Enter,
            Wait::Key(Press::Escape) => Key::Escape,
            Wait::Key(_) => Key::Other,
            Wait::NoInput => return true,
            Wait::Nothing | Wait::Timeout => continue,
        };
        match caution.press(key) {
            Step::Stay => {}
            Step::Back => {
                output::arguments(format_args!(
                    "INSTALL CAUTION answered: back to the menu, nothing started\n"
                ));
                return false;
            }
            Step::Install => {
                output::arguments(format_args!("INSTALL CAUTION answered: install anyway\n"));
                CONFIRMED.store(true, Ordering::Relaxed);
                return true;
            }
        }
    }
}

fn draw(canvas: &mut Canvas, caution: &Caution) {
    const LINE: usize = 26;
    let (width, height) = canvas.size();
    canvas.fill(0, 0, width, height, ui::BACKGROUND);
    let top = canvas.header() + 16;
    let column = width.saturating_sub(32).min(720);
    let left = (width - column) / 2;
    canvas.text(left, top, TITLE, 1, ui::TEXT);
    let glyph = canvas.text_width(" ", 1).max(1);
    let columns = column.saturating_sub(40) / glyph;
    let reason = wrap(caution.reason, columns);
    let ask = wrap(ASK, columns);
    let card = 24 + (reason.len() + 1 + ask.len()) * LINE;
    let card_top = top + 36;
    canvas.panel(left, card_top, column, card, 8, ui::CARD);
    canvas.panel(
        left + 6,
        card_top + 14,
        3,
        card.saturating_sub(28),
        1,
        ui::ORANGE,
    );
    let mut y = card_top + 16;
    for line in reason {
        canvas.text(left + 20, y, line, 1, ui::TEXT);
        y += LINE;
    }
    y += LINE;
    for line in ask {
        canvas.text(left + 20, y, line, 1, ui::DIM);
        y += LINE;
    }
    let buttons = card_top + card + 32;
    let right = left + column;
    let install = "Install anyway";
    let install_width = canvas.text_width(install, 1) + 48;
    button(
        canvas,
        right - install_width,
        buttons,
        install_width,
        install,
        caution.focus() == Focus::Install,
    );
    let back_width = canvas.text_width("Back", 1) + 48;
    button(
        canvas,
        right - install_width - 12 - back_width,
        buttons,
        back_width,
        "Back",
        caution.focus() == Focus::Back,
    );
    let keys = [
        ("Right", "Install anyway"),
        ("Enter", "Choose"),
        ("Esc", "Back"),
    ];
    let gap = 24;
    let total: usize = keys
        .iter()
        .map(|(key, label)| canvas.text_width(key, 1) + 8 + canvas.text_width(label, 1))
        .sum::<usize>()
        + gap * (keys.len() - 1);
    let mut x = width.saturating_sub(total) / 2;
    for (key, label) in keys {
        canvas.text(x, height - 40, key, 1, ui::ORANGE);
        x += canvas.text_width(key, 1) + 8;
        canvas.text(x, height - 40, label, 1, ui::DIM);
        x += canvas.text_width(label, 1) + gap;
    }
}

fn button(canvas: &mut Canvas, x: usize, y: usize, width: usize, text: &str, focused: bool) {
    let (background, foreground) = if focused {
        (ui::ORANGE, ui::BACKGROUND)
    } else {
        (ui::RAISED, ui::TEXT)
    };
    canvas.panel(x, y, width, 36, 8, background);
    canvas.text(x + 24, y + 10, text, 1, foreground);
}

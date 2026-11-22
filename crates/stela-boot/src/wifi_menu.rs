//! The startup menu's Wi-Fi page (`W` at the menu, `choice::Menu`): the
//! networks the USB saves, the `--wifi` one first, then each of
//! `wifi.saved`'s (`options::WIFI_SAVED`). Enter on one has this boot join
//! it first (`--wifi-prefer`); "Add a network" saves a name and a password
//! typed here, and Delete forgets a saved network. A password shows as dots
//! unless Tab shows it; no name or password reaches the log.
use crate::{
    choice::{Press, Wait, key},
    console::{Framebuffer, font},
    output,
    ui::{self, Canvas},
};
use alloc::{format, string::String, vec::Vec};
use stela_boot_core::{options::WIFI_SAVED, wifi_saved};
use stela_drivers::net::wifi::network::{SSID_BYTES, Secret};
use uefi::{
    CString16,
    fs::{FileSystem, PathBuf},
};

const ROW: usize = 36;
const ADD: &str = "Add a network...";

/// The page until a network is chosen, the one `--wifi` names being
/// `configured`: the chosen network's name, or `None` when the page is left
/// with Escape or the console has no input.
pub(crate) fn run(volume: &mut FileSystem, configured: &str) -> Option<String> {
    let (screen, font) = Framebuffer::from_firmware().zip(font::console())?;
    let mut canvas = Canvas::new(screen, font);
    let mut selected = 0;
    let mut notice: Option<String> = None;
    loop {
        let mut names = Vec::from([String::from(configured)]);
        names.extend(
            saved_names(volume)
                .into_iter()
                .filter(|name| name != configured),
        );
        let items = names.len() + 1;
        selected = selected.min(items - 1);
        draw_list(&mut canvas, &names, selected, notice.as_deref());
        canvas.show();
        match key(None) {
            Wait::Key(Press::Up) => selected = (selected + items - 1) % items,
            Wait::Key(Press::Down) => selected = (selected + 1) % items,
            Wait::Key(Press::Escape) | Wait::NoInput => return None,
            Wait::Key(Press::Enter) if selected < names.len() => {
                return Some(names.swap_remove(selected));
            }
            Wait::Key(Press::Enter) => match add(volume, &mut canvas) {
                Ok(Some(name)) => return Some(name),
                Ok(None) => notice = None,
                Err(why) => notice = Some(String::from(why)),
            },
            Wait::Key(Press::Delete) if selected > 0 && selected < names.len() => {
                notice = Some(match forget(volume, names[selected].as_bytes()) {
                    Ok(()) => format!("Forgot {}.", names[selected]),
                    Err(why) => String::from(why),
                });
            }
            Wait::Key(_) | Wait::Nothing | Wait::Timeout => {}
        }
    }
}

/// The networks the saved file names, in order; a name that is not UTF-8
/// cannot be chosen by name and is left out.
fn saved_names(volume: &mut FileSystem) -> Vec<String> {
    let mut file = read(volume);
    let names = wifi_saved::names(&file)
        .into_iter()
        .filter_map(|name| String::from_utf8(name).ok())
        .collect();
    wifi_saved::wipe(&mut file);
    names
}

fn path() -> Option<PathBuf> {
    CString16::try_from(WIFI_SAVED).map(PathBuf::from).ok()
}

/// The saved file's bytes; none when it is absent or unreadable.
fn read(volume: &mut FileSystem) -> Vec<u8> {
    path()
        .and_then(|path| volume.read(&path).ok())
        .unwrap_or_default()
}

fn write(volume: &mut FileSystem, bytes: &[u8]) -> Result<(), &'static str> {
    let path = path().ok_or("The saved networks file has no path.")?;
    volume
        .write(&path, bytes)
        .map_err(|_| "The saved networks could not be written to the USB.")
}

/// Forget the saved network `name`: every line of it leaves the file.
fn forget(volume: &mut FileSystem, name: &[u8]) -> Result<(), &'static str> {
    let mut file = read(volume);
    let mut kept = wifi_saved::without(&file, name);
    let written = write(volume, &kept);
    wifi_saved::wipe(&mut file);
    wifi_saved::wipe(&mut kept);
    output::arguments(format_args!(
        "CHOICE Wi-Fi: a saved network forgotten at the menu ({})\n",
        match written {
            Ok(()) => "written",
            Err(_) => "not written",
        }
    ));
    written
}

/// Ask for a network's name and password and save them: the name, or `None`
/// when either question is left with Escape.
fn add(volume: &mut FileSystem, canvas: &mut Canvas) -> Result<Option<String>, &'static str> {
    let Some(name) = typed(
        canvas,
        "Network name",
        "Type the Wi-Fi network's name exactly.",
        false,
    ) else {
        return Ok(None);
    };
    let name = String::from_utf8(name).map_err(|_| "A network name is plain text.")?;
    if !(1..=SSID_BYTES).contains(&name.len()) {
        return Err("A network name is 1 to 32 bytes.");
    }
    let title = format!("Password for {name}");
    let Some(mut password) = typed(
        canvas,
        &title,
        "8 to 63 characters, or 64 hex digits. Tab shows or hides it.",
        true,
    ) else {
        return Ok(None);
    };
    let usable = Secret::parse(&password).is_ok();
    if !usable {
        wifi_saved::wipe(&mut password);
        return Err("A Wi-Fi password is 8 to 63 characters, or 64 hex digits.");
    }
    let mut file = read(volume);
    let mut updated = wifi_saved::without(&file, name.as_bytes());
    let mut line = wifi_saved::line(name.as_bytes(), &password);
    updated.extend_from_slice(&line);
    let written = write(volume, &updated);
    for bytes in [&mut password, &mut file, &mut updated, &mut line] {
        wifi_saved::wipe(bytes);
    }
    output::arguments(format_args!(
        "CHOICE Wi-Fi: a network saved at the menu ({}; its name and password are not logged)\n",
        match written {
            Ok(()) => "written",
            Err(_) => "not written",
        }
    ));
    written.map(|()| Some(name))
}

/// One line of text typed in, `secret` showing it as dots until Tab: the
/// bytes, or `None` on Escape.
fn typed(canvas: &mut Canvas, title: &str, hint: &str, secret: bool) -> Option<Vec<u8>> {
    let mut text: Vec<u8> = Vec::new();
    let mut shown = !secret;
    loop {
        let field = match shown {
            true => String::from_utf8_lossy(&text).into_owned(),
            false => "\u{2022}".repeat(text.len()),
        };
        draw_field(canvas, title, hint, &field);
        canvas.show();
        match key(None) {
            Wait::Key(Press::Enter) => return Some(text),
            Wait::Key(Press::Escape) | Wait::NoInput => {
                wifi_saved::wipe(&mut text);
                return None;
            }
            Wait::Key(Press::Character('\u{8}')) => {
                if let Some(last) = text.last_mut() {
                    *last = 0;
                }
                text.pop();
            }
            Wait::Key(Press::Character('\t')) if secret => shown = !shown,
            Wait::Key(Press::Character(character)) if !character.is_control() => {
                let mut bytes = [0; 4];
                let encoded = character.encode_utf8(&mut bytes).as_bytes();
                // Room for a PSK's 64 digits; a name stops at 32 bytes.
                let limit = match secret {
                    true => 64,
                    false => SSID_BYTES,
                };
                if text.len() + encoded.len() <= limit {
                    text.extend_from_slice(encoded);
                }
            }
            Wait::Key(_) | Wait::Nothing | Wait::Timeout => {}
        }
    }
}

/// The page's frame: the header, a title and the column the rows fill.
fn frame(canvas: &mut Canvas, title: &str) -> (usize, usize, usize) {
    let (width, height) = canvas.size();
    canvas.fill(0, 0, width, height, ui::BACKGROUND);
    let top = canvas.header() + 24;
    let column = width.saturating_sub(32).min(440);
    let left = (width - column) / 2;
    canvas.text(left + 16, top, title, 1, ui::TEXT);
    (left, top + 40, column)
}

/// The footer: a notice in orange, else the keys, dim.
fn footer(canvas: &mut Canvas, notice: Option<&str>, keys: &str) {
    let (width, height) = canvas.size();
    let (text, colour) = match notice {
        Some(notice) => (notice, ui::ORANGE),
        None => (keys, ui::DIM),
    };
    canvas.text_centred(width / 2, height.saturating_sub(56), text, 1, colour);
}

fn draw_list(canvas: &mut Canvas, names: &[String], selected: usize, notice: Option<&str>) {
    let (left, top, column) = frame(canvas, "Wi-Fi networks");
    let (_, height) = canvas.size();
    let rows: Vec<&str> = names.iter().map(String::as_str).chain([ADD]).collect();
    let fits = (height.saturating_sub(top + 120) / ROW).max(1);
    let first = stela_boot_core::screen::window(selected, rows.len(), fits);
    let mut y = top;
    for index in (first..rows.len()).take(fits) {
        if index == selected {
            canvas.panel(left, y, column, ROW - 4, 8, ui::RAISED);
            canvas.panel(left + 6, y + 9, 3, ROW - 22, 1, ui::ORANGE);
        }
        let colour = match index == rows.len() - 1 {
            true => ui::BLUE,
            false => ui::TEXT,
        };
        canvas.text(left + 20, y + 8, rows[index], 1, colour);
        if index == 0 {
            canvas.text_right(left + column - 16, y + 8, "default", 1, ui::DIM);
        }
        y += ROW;
    }
    footer(
        canvas,
        notice,
        "Enter: join first this boot   Delete: forget   Esc: back",
    );
}

fn draw_field(canvas: &mut Canvas, title: &str, hint: &str, field: &str) {
    let (left, top, column) = frame(canvas, title);
    canvas.panel(left, top, column, ROW, 8, ui::RAISED);
    canvas.text(left + 16, top + 10, field, 1, ui::TEXT);
    canvas.text(left + 16, top + ROW + 16, hint, 1, ui::DIM);
    footer(canvas, None, "Enter: next   Esc: back");
}

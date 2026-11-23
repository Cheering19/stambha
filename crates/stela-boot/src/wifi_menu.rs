//! The startup menu's Wi-Fi page (`W` at the menu, `choice::Menu`): the
//! networks the USB saves, the `--wifi` one first, then each of
//! `wifi.saved`'s (`options::WIFI_SAVED`), as a panel in the menu's look.
//! Enter on one has this boot join it first (`--wifi-prefer`); "Join other
//! network..." saves a name and a password typed in a dialog, and Delete or
//! Backspace forgets a saved network once Enter confirms it. What the page
//! holds, how keys change it and what it draws are
//! `stela_boot_core::wifi_page`'s; this reads and writes the file and paints.
//! A password shows as dots unless Tab shows it; no name or password reaches
//! the log.
use crate::{
    choice::{Press, Wait, key},
    console::{Framebuffer, font},
    output,
    ui::{self, Canvas},
};
use alloc::{string::String, vec::Vec};
use stela_boot_core::{
    options::{WIFI_SAVED, WIFI_SAVED_NEW},
    wifi_hardware,
    wifi_page::{self, Action, Draw, Geometry, Key, Network, Page, Save, Scene, Tone},
    wifi_saved,
};
use stela_drivers::net::wifi::network::Secret;
use uefi::{
    CString16,
    fs::{FileSystem, PathBuf},
};

/// Whether the startup menu offers Wi-Fi on this PC, from the firmware's PCI
/// census before ExitBootServices (`stela_boot_core::wifi_hardware`); one
/// line says which way it decided.
pub(crate) fn offered() -> bool {
    let hardware = match crate::pci_io::capture() {
        Ok(census) => {
            let functions: Vec<_> = census
                .functions
                .iter()
                .map(crate::pci_io::FunctionObservation::identity)
                .collect();
            wifi_hardware::decide(Ok(wifi_hardware::Census {
                functions: &functions,
                unread: census.failures.len(),
            }))
        }
        Err(error) => wifi_hardware::decide(Err(&alloc::format!("{error}"))),
    };
    output::arguments(format_args!("{}", hardware.line()));
    hardware.offered()
}

/// The page until a network is chosen, the one `--wifi` names being
/// `configured` and the one chosen before `prefer`: the chosen network's
/// name, or `None` when the page is left with Escape, the console has no
/// input, or no `--wifi` is set, when a network can be saved or forgotten
/// but not chosen.
pub(crate) fn run(
    volume: &mut FileSystem,
    configured: Option<&str>,
    prefer: Option<&str>,
) -> Option<String> {
    let (screen, font) = Framebuffer::from_firmware().zip(font::console())?;
    let mut canvas = Canvas::new(screen, font);
    let mut page = Page::new(networks(volume, configured), configured.is_some(), prefer);
    loop {
        let scene = page.scene(geometry(&canvas));
        paint(&mut canvas, &scene);
        drop(scene);
        let key = match key(None) {
            Wait::Key(press) => translate(press),
            Wait::NoInput => return None,
            Wait::Nothing | Wait::Timeout => continue,
        };
        match page.press(key) {
            Action::Stay => {}
            Action::Leave => return None,
            Action::Join(name) => return Some(name),
            Action::Forget(name) => {
                let forgot = forget(volume, name.as_bytes());
                page.forgotten(&name, forgot, networks(volume, configured));
            }
            Action::Save(save) => {
                let name = save.name.clone();
                let written = add(volume, save);
                if let Action::Join(name) = page.saved(&name, written, networks(volume, configured))
                {
                    return Some(name);
                }
            }
        }
    }
}

/// Whether the USB holds a saved networks file, so the menu offers the page
/// even when no entry sets `--wifi`.
pub(crate) fn saved(volume: &mut FileSystem) -> bool {
    path().is_some_and(|path| matches!(volume.try_exists(&path), Ok(true)))
}

/// A key as the page reads it: Backspace and Tab come as characters.
fn translate(press: Press) -> Key {
    match press {
        Press::Up => Key::Up,
        Press::Down => Key::Down,
        Press::Home => Key::Home,
        Press::End => Key::End,
        Press::Delete => Key::Delete,
        Press::Escape => Key::Escape,
        Press::Enter => Key::Enter,
        Press::Character('\u{8}') => Key::Backspace,
        Press::Character('\t') => Key::Tab,
        Press::Character(character) => Key::Character(character),
        Press::Left | Press::Right | Press::Other => Key::Other,
    }
}

/// The networks the page lists, for the menu to check its preference
/// against once the page closed (`wifi_page::kept_preference`).
pub(crate) fn listed(volume: &mut FileSystem, configured: Option<&str>) -> Vec<Network> {
    networks(volume, configured)
}

/// The networks to list: `configured`, then the saved file's; the file's
/// bytes are wiped once read.
fn networks(volume: &mut FileSystem, configured: Option<&str>) -> Vec<Network> {
    let mut file = saved_file(volume).unwrap_or_default();
    let listed = wifi_saved::listing(&file);
    wifi_saved::wipe(&mut file);
    wifi_page::networks(configured, listed)
}

fn path() -> Option<PathBuf> {
    CString16::try_from(WIFI_SAVED).map(PathBuf::from).ok()
}

/// The copy a write makes first ([`WIFI_SAVED_NEW`]).
fn spare_path() -> Option<PathBuf> {
    CString16::try_from(WIFI_SAVED_NEW).map(PathBuf::from).ok()
}

/// What `path` holds on `volume`.
fn stored(volume: &mut FileSystem, path: Option<PathBuf>) -> wifi_saved::Stored {
    let Some(path) = path else {
        return wifi_saved::Stored::Unreadable;
    };
    match volume.try_exists(&path) {
        Ok(false) => wifi_saved::Stored::Absent,
        Ok(true) => match volume.read(&path) {
            Ok(bytes) => wifi_saved::Stored::Bytes(bytes),
            Err(_) => wifi_saved::Stored::Unreadable,
        },
        Err(_) => wifi_saved::Stored::Unreadable,
    }
}

/// The saved networks' bytes as the page and the boot read them
/// (`wifi_saved::current`): `None` when the file is there but did not read.
pub(crate) fn saved_file(volume: &mut FileSystem) -> Option<Vec<u8>> {
    let saved = stored(volume, path());
    let spare = stored(volume, spare_path());
    wifi_saved::current(saved, spare)
}

/// Whether `path` reads back as `bytes`; what was read is wiped.
fn reads_back(volume: &mut FileSystem, path: &PathBuf, bytes: &[u8]) -> bool {
    match volume.read(path) {
        Ok(mut read) => {
            let same = read == bytes;
            wifi_saved::wipe(&mut read);
            same
        }
        Err(_) => false,
    }
}

/// Write `bytes` as the saved networks. The firmware file protocol has no
/// truncate and uefi's `write` deletes a file before it writes it again, so
/// the bytes go to [`WIFI_SAVED_NEW`] first and are read back: a write of
/// the file itself that stops after the delete leaves that copy, which
/// [`saved_file`] then reads. The copy goes once the file reads back whole.
fn write(volume: &mut FileSystem, bytes: &[u8]) -> Result<(), &'static str> {
    const FAILED: &str = "The saved networks could not be written to the USB.";
    let path = path().ok_or("The saved networks file has no path.")?;
    let spare = spare_path().ok_or("The saved networks file has no path.")?;
    volume.write(&spare, bytes).map_err(|_| FAILED)?;
    if !reads_back(volume, &spare, bytes) {
        return Err(FAILED);
    }
    volume.write(&path, bytes).map_err(|_| FAILED)?;
    if !reads_back(volume, &path, bytes) {
        return Err(FAILED);
    }
    // The file is whole: the copy is no longer needed, and one left behind
    // is read only while the file is absent.
    let _ = volume.remove_file(&spare);
    Ok(())
}

/// Refused when the saved networks are there but did not read: writing a
/// change over them would erase every network the file holds.
const UNREAD: &str = "The saved networks could not be read from the USB, so nothing was changed.";

/// Forget the saved network `name`: every line of it leaves the file.
fn forget(volume: &mut FileSystem, name: &[u8]) -> Result<(), &'static str> {
    let mut file = saved_file(volume).ok_or(UNREAD)?;
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

/// Save the network the dialog took, in place of any line of the same name;
/// its password is wiped as `save` drops.
fn add(volume: &mut FileSystem, save: Save) -> Result<(), &'static str> {
    // The dialog takes only a key `Secret::parse` takes; checked again, as
    // nothing else stands between it and the file.
    if Secret::parse(&save.password).is_err() {
        return Err(wifi_page::PASSWORD_LENGTH);
    }
    let mut file = saved_file(volume).ok_or(UNREAD)?;
    let mut updated = wifi_saved::without(&file, save.name.as_bytes());
    let mut line = wifi_saved::line(save.name.as_bytes(), &save.password);
    updated.extend_from_slice(&line);
    let written = write(volume, &updated);
    for bytes in [&mut file, &mut updated, &mut line] {
        wifi_saved::wipe(bytes);
    }
    drop(save);
    output::arguments(format_args!(
        "CHOICE Wi-Fi: a network saved at the menu ({}; its name and password are not logged)\n",
        match written {
            Ok(()) => "written",
            Err(_) => "not written",
        }
    ));
    written
}

/// The canvas as the page lays out on it.
fn geometry(canvas: &Canvas) -> Geometry {
    let (width, height) = canvas.size();
    Geometry {
        width,
        height,
        top: ui::HEADER + 24,
        glyph: canvas.text_width(" ", 1),
    }
}

fn colour(tone: Tone) -> u32 {
    match tone {
        Tone::Background => ui::BACKGROUND,
        Tone::Card => ui::CARD,
        Tone::Raised => ui::RAISED,
        Tone::Text => ui::TEXT,
        Tone::Dim => ui::DIM,
        Tone::Orange => ui::ORANGE,
        Tone::Blue => ui::BLUE,
        Tone::Red => ui::RED,
    }
}

/// The page's background and header, then `scene`, shown at once.
fn paint(canvas: &mut Canvas, scene: &Scene) {
    let (width, height) = canvas.size();
    canvas.fill(0, 0, width, height, ui::BACKGROUND);
    canvas.header();
    for draw in &scene.draws {
        match draw {
            Draw::Panel {
                x,
                y,
                width,
                height,
                radius,
                tone,
            } => canvas.panel(*x, *y, *width, *height, *radius, colour(*tone)),
            Draw::Text { x, y, text, tone } => canvas.text(*x, *y, text, 1, colour(*tone)),
        }
    }
    canvas.show();
}

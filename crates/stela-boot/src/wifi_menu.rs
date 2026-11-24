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
    wifi_prompt, wifi_saved,
};
use stela_drivers::net::wifi::network::Secret;
use uefi::{
    CString16,
    fs::{FileSystem, PathBuf},
};

/// What the startup menu makes of this PC's Wi-Fi, from the firmware's PCI
/// census before ExitBootServices (`stela_boot_core::wifi_hardware`): the
/// Wi-Fi page, or a page saying why there is none. One line says which.
pub(crate) fn hardware() -> wifi_hardware::Hardware {
    let hardware = match crate::pci_io::capture() {
        Ok(census) => {
            let functions: Vec<_> = census
                .functions
                .iter()
                .map(crate::pci_io::FunctionObservation::identity)
                .collect();
            let pci = crate::storage::census_pci(&census.functions);
            wifi_hardware::decide(Ok(wifi_hardware::Census {
                functions: &functions,
                unread: census.failures.len(),
                empty_ports: stela_loader::host_firmware::bundle::empty_ports(&pci).count(),
            }))
        }
        Err(error) => wifi_hardware::decide(Err(&alloc::format!("{error}"))),
    };
    output::arguments(format_args!("{}", hardware.line()));
    hardware
}

/// The page `W` opens where the menu offers no Wi-Fi page: `lines`
/// (`wifi_hardware::Hardware::unavailable`) until a key is pressed.
pub(crate) fn unavailable(lines: &[String]) {
    let Some((screen, font)) = Framebuffer::from_firmware().zip(font::console()) else {
        return;
    };
    let mut canvas = Canvas::new(screen, font);
    output::arguments(format_args!(
        "CHOICE Wi-Fi page: no Wi-Fi chip the driver starts, so it says why\n"
    ));
    let scene = wifi_page::unavailable_scene(lines, geometry(&canvas));
    paint(&mut canvas, &scene);
    drop(scene);
    loop {
        match key(None) {
            Wait::Key(_) | Wait::NoInput => return,
            Wait::Nothing | Wait::Timeout => {}
        }
    }
}

/// The page until a network is chosen, the one `--wifi` names being
/// `configured` and the one chosen before `prefer`: the chosen network's
/// name, or `None` when the page is left with Escape or the console has no
/// input. With no `--wifi`, the boot joins the saved networks, and the one
/// chosen goes last in their file, which it joins first ([`chosen`]).
pub(crate) fn run(
    volume: &mut FileSystem,
    configured: Option<&str>,
    prefer: Option<&str>,
) -> Option<String> {
    let page = Page::new(networks(volume, configured), true, prefer);
    drive(volume, configured, None, page)
}

/// The page as the boot opens it by itself (`wifi_prompt::decide`): to
/// choose a network, or on the one the earlier boot could not join, its
/// password ready. `key` is the `--wifi` network's key file, where its
/// password typed again goes; `saved` the usable saved networks' names in
/// the file's order, which a record's `Which::Saved` counts. The chosen
/// network's name, as [`run`] gives it; `None` when the page is left.
pub(crate) fn open(
    volume: &mut FileSystem,
    configured: Option<&str>,
    key: Option<&str>,
    prompt: wifi_prompt::Prompt,
    saved: &[String],
) -> Option<String> {
    let networks = networks(volume, configured);
    let page = match prompt {
        wifi_prompt::Prompt::Choose => {
            Page::auto(networks, true, String::from(wifi_prompt::CHOOSE))
        }
        wifi_prompt::Prompt::Retry(which, cause) => {
            let name = match which {
                wifi_prompt::Which::Configured => String::from(configured?),
                wifi_prompt::Which::Saved(place) => saved.get(place.checked_sub(1)?)?.clone(),
            };
            let notice = wifi_prompt::retry_notice(&name, cause);
            Page::retry(networks, true, &name, notice)
        }
    };
    drive(volume, configured, key, page)
}

/// `page` until it is left or a network chosen: [`run`]'s and [`open`]'s.
fn drive(
    volume: &mut FileSystem,
    configured: Option<&str>,
    key_file: Option<&str>,
    mut page: Page,
) -> Option<String> {
    let (screen, font) = Framebuffer::from_firmware().zip(font::console())?;
    let mut canvas = Canvas::new(screen, font);
    loop {
        let scene = page.scene(geometry(&canvas));
        paint(&mut canvas, &scene);
        drop(scene);
        let pressed = match key(None) {
            Wait::Key(press) => translate(press),
            Wait::NoInput => return None,
            Wait::Nothing | Wait::Timeout => continue,
        };
        match page.press(pressed) {
            Action::Stay => {}
            Action::Leave => return None,
            Action::Join(name) => return chosen(volume, configured, name),
            Action::Forget(name) => {
                let forgot = forget(volume, name.as_bytes());
                page.forgotten(&name, forgot, networks(volume, configured));
            }
            Action::Save(save) => {
                let name = save.name.clone();
                // The `--wifi` network's password is its key file's.
                let written = match (configured, key_file) {
                    (Some(configured), Some(path)) if configured == name => {
                        write_key(volume, path, save)
                    }
                    _ => add(volume, save),
                };
                if let Action::Join(name) = page.saved(&name, written, networks(volume, configured))
                {
                    return chosen(volume, configured, name);
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

/// `name`, chosen on the page: with no `--wifi`, its lines go last in the
/// saved file, so this boot joins it first; with `--wifi`, the menu names
/// it with `--wifi-prefer`.
fn chosen(volume: &mut FileSystem, configured: Option<&str>, name: String) -> Option<String> {
    if configured.is_none()
        && let Some(mut file) = saved_file(volume)
    {
        let mut moved = wifi_saved::last(&file, name.as_bytes());
        if moved != file {
            let written = write(volume, &moved);
            output::arguments(format_args!(
                "CHOICE Wi-Fi: a saved network chosen at the menu joins first ({})\n",
                match written {
                    Ok(()) => "written",
                    Err(_) => "not written",
                }
            ));
        }
        wifi_saved::wipe(&mut file);
        wifi_saved::wipe(&mut moved);
    }
    Some(name)
}

/// Write the `--wifi` network's password typed again into its key file at
/// `path`, as `stambha settings` writes it, and read it back; it is wiped
/// as `save` drops. A failure the earlier boot recorded is the old key's.
fn write_key(volume: &mut FileSystem, path: &str, save: Save) -> Result<(), &'static str> {
    const FAILED: &str = "The network's key could not be written to the USB.";
    if Secret::parse(&save.password).is_err() {
        return Err(wifi_page::PASSWORD_LENGTH);
    }
    let path = CString16::try_from(path)
        .map(PathBuf::from)
        .map_err(|_| FAILED)?;
    let written = match volume.write(&path, &save.password[..]) {
        Ok(()) if reads_back(volume, &path, &save.password) => Ok(()),
        _ => Err(FAILED),
    };
    drop(save);
    output::arguments(format_args!(
        "CHOICE Wi-Fi: the --wifi network's password typed again at the menu ({}; it is not \
         logged)\n",
        match written {
            Ok(()) => "written",
            Err(_) => "not written",
        }
    ));
    if written.is_ok() {
        output::arguments(format_args!("{}", wifi_prompt::Record::Cleared.line()));
    }
    written
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
    if written.is_ok() {
        output::arguments(format_args!("{}", wifi_prompt::Record::Cleared.line()));
    }
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
    if written.is_ok() {
        output::arguments(format_args!("{}", wifi_prompt::Record::Cleared.line()));
    }
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

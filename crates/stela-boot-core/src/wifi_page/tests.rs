use super::*;
use alloc::{string::ToString, vec};

mod render;

/// A 1280 by 720 screen, the startup menu's header above row 112.
const SCREEN: Geometry = Geometry {
    width: 1280,
    height: 720,
    top: 112,
    glyph: 8,
};

fn saved(name: &str) -> Network {
    Network {
        name: name.to_string(),
        configured: false,
        usable: true,
    }
}

fn configured(name: &str) -> Network {
    Network {
        configured: true,
        ..saved(name)
    }
}

/// `--wifi home`, then `cafe` and `lab` saved.
fn page() -> Page {
    Page::new(
        vec![configured("home"), saved("cafe"), saved("lab")],
        true,
        None,
    )
}

fn many(count: usize) -> Page {
    let networks = (0..count).map(|n| saved(&format!("net-{n:02}"))).collect();
    Page::new(networks, true, None)
}

fn typed(page: &mut Page, text: &str) {
    for character in text.chars() {
        assert_eq!(page.press(Key::Character(character)), Action::Stay);
    }
}

fn form(page: &Page) -> &Form {
    match page.mode() {
        Mode::Add(form) => form,
        _ => panic!("the dialog is not open"),
    }
}

/// The page at "Join other network...", its dialog open.
fn adding() -> Page {
    let mut page = page();
    page.press(Key::End);
    assert_eq!(page.press(Key::Enter), Action::Stay);
    assert!(matches!(page.mode(), Mode::Add(_)));
    page
}

fn has_text(scene: &Scene, wanted: &str) -> bool {
    scene.texts().any(|text| text.contains(wanted))
}

// The startup menu's line.

#[test]
fn the_status_line_names_the_network_this_boot_joins_first() {
    assert_eq!(
        status_line(Some("home"), None, false).as_deref(),
        Some("Wi-Fi: home \u{b7} press W to change")
    );
    assert_eq!(
        status_line(Some("home"), Some("cafe"), true).as_deref(),
        Some("Wi-Fi: cafe \u{b7} press W to change")
    );
}

#[test]
fn the_status_line_shows_not_set_up_only_with_saved_networks() {
    assert_eq!(
        status_line(None, None, true).as_deref(),
        Some("Wi-Fi: not set up \u{b7} press W to change")
    );
    // A preference without `--wifi` joins nothing, as `with_prefer` adds none.
    assert_eq!(
        status_line(None, Some("cafe"), true).as_deref(),
        Some("Wi-Fi: not set up \u{b7} press W to change")
    );
    assert_eq!(status_line(None, None, false), None);
}

// The rows.

#[test]
fn the_configured_network_comes_first_and_each_saved_one_once() {
    let listed = vec![
        Listed {
            name: b"cafe".to_vec(),
            usable: true,
        },
        Listed {
            name: b"home".to_vec(),
            usable: false,
        },
        Listed {
            name: vec![0xff, 0xfe],
            usable: true,
        },
        Listed {
            name: b"lab".to_vec(),
            usable: false,
        },
    ];
    assert_eq!(
        networks(Some("home"), listed.clone()),
        [
            configured("home"),
            saved("cafe"),
            Network {
                usable: false,
                ..saved("lab")
            }
        ]
    );
    assert_eq!(networks(None, listed)[1].name, "home");
}

// Navigation.

#[test]
fn up_and_down_wrap_through_the_networks_and_join_other() {
    let mut page = page();
    assert_eq!(page.selected(), Row::Network(0));
    page.press(Key::Down);
    page.press(Key::Down);
    assert_eq!(page.selected(), Row::Network(2));
    page.press(Key::Down);
    assert_eq!(page.selected(), Row::JoinOther);
    page.press(Key::Down);
    assert_eq!(page.selected(), Row::Network(0));
    page.press(Key::Up);
    assert_eq!(page.selected(), Row::JoinOther);
    page.press(Key::Home);
    assert_eq!(page.selected(), Row::Network(0));
    page.press(Key::End);
    assert_eq!(page.selected(), Row::JoinOther);
}

#[test]
fn enter_joins_the_selected_network_first_and_escape_leaves() {
    let mut page = page();
    page.press(Key::Down);
    assert_eq!(page.press(Key::Enter), Action::Join("cafe".to_string()));
    assert_eq!(page.press(Key::Escape), Action::Leave);
}

#[test]
fn a_network_whose_key_a_boot_cannot_use_is_not_joined() {
    let broken = Network {
        usable: false,
        ..saved("cafe")
    };
    let mut page = Page::new(vec![configured("home"), broken], true, None);
    page.press(Key::Down);
    assert_eq!(page.press(Key::Enter), Action::Stay);
    assert!(
        page.notice()
            .is_some_and(|notice| notice.starts_with("cafe's saved key"))
    );
}

#[test]
fn the_menus_preference_names_only_a_usable_saved_network_after_the_page() {
    let networks = vec![
        configured("home"),
        saved("cafe"),
        Network {
            usable: false,
            ..saved("lab")
        },
    ];
    let keep = |name: &str| kept_preference(Some(name.to_string()), &networks);
    assert_eq!(keep("cafe").as_deref(), Some("cafe"));
    // Forgotten on the page, so no longer listed.
    assert_eq!(keep("attic"), None);
    // Listed, but its key is not one a boot can use.
    assert_eq!(keep("lab"), None);
    // `--wifi`'s own network is the default, never a preference.
    assert_eq!(keep("home"), None);
    assert_eq!(kept_preference(None, &networks), None);
}

#[test]
fn the_page_opens_on_the_network_chosen_before() {
    let page = Page::new(vec![configured("home"), saved("cafe")], true, Some("cafe"));
    assert_eq!(page.selected(), Row::Network(1));
    let scene = page.scene(SCREEN);
    assert!(has_text(&scene, "this boot"));
    assert!(has_text(&scene, "default"));
}

#[test]
fn without_wifi_a_network_is_managed_but_not_chosen() {
    let mut page = Page::new(vec![saved("cafe")], false, None);
    assert_eq!(page.press(Key::Enter), Action::Stay);
    assert_eq!(page.notice(), Some(NO_WIFI));
    assert!(!has_text(&page.scene(SCREEN), "Enter: join first"));
    assert_eq!(page.press(Key::Delete), Action::Stay);
    assert!(matches!(page.mode(), Mode::Confirm(0)));
}

#[test]
fn an_empty_page_offers_only_join_other() {
    let mut page = Page::new(Vec::new(), true, None);
    assert_eq!(page.selected(), Row::JoinOther);
    page.press(Key::Down);
    assert_eq!(page.selected(), Row::JoinOther);
    let scene = page.scene(SCREEN);
    assert!(has_text(&scene, "No saved networks yet."));
    assert!(has_text(&scene, "Join other network..."));
    assert_eq!(page.press(Key::Delete), Action::Stay);
    assert_eq!(page.press(Key::Enter), Action::Stay);
    assert!(matches!(page.mode(), Mode::Add(_)));
}

#[test]
fn one_network_lists_with_its_lock_and_marker() {
    let page = Page::new(vec![configured("home")], true, None);
    let scene = page.scene(SCREEN);
    assert!(has_text(&scene, "home"));
    assert!(has_text(&scene, "default"));
    assert!(!has_text(&scene, " of "), "nothing scrolls");
    // The highlight's orange mark, as the startup menu draws it.
    assert!(scene.draws.contains(&Draw::Panel {
        x: 426,
        y: 172 + 9,
        width: 3,
        height: 14,
        radius: 1,
        tone: Tone::Orange
    }));
    assert_eq!(page.visible(SCREEN), 0..2);
}

#[test]
fn an_unusable_saved_key_is_marked() {
    let page = Page::new(
        vec![Network {
            usable: false,
            ..saved("lab")
        }],
        true,
        None,
    );
    assert!(has_text(&page.scene(SCREEN), "key unusable"));
}

#[test]
fn many_networks_scroll_to_keep_the_selection_in_view() {
    let mut page = many(30);
    let fits = page.visible(SCREEN).len();
    assert_eq!(fits, (720 - (172 + 120)) / 36);
    assert_eq!(page.visible(SCREEN), 0..fits);
    for _ in 0..fits {
        page.press(Key::Down);
    }
    assert_eq!(page.visible(SCREEN), 1..fits + 1);
    let scene = page.scene(SCREEN);
    assert!(has_text(&scene, &format!("net-{fits:02}")));
    assert!(!has_text(&scene, "net-00"));
    assert!(has_text(&scene, &format!("{} of 31", fits + 1)));
    page.press(Key::End);
    assert_eq!(page.visible(SCREEN), 31 - fits..31);
    assert!(has_text(&page.scene(SCREEN), "Join other network..."));
    page.press(Key::Down);
    assert_eq!(page.visible(SCREEN), 0..fits);
}

#[test]
fn a_small_screen_still_shows_a_row() {
    let small = Geometry {
        width: 320,
        height: 200,
        ..SCREEN
    };
    assert_eq!(many(5).visible(small).len(), 1);
}

// Forgetting.

#[test]
fn delete_asks_before_a_saved_network_is_forgotten() {
    let mut page = page();
    page.press(Key::Down);
    assert_eq!(page.press(Key::Delete), Action::Stay);
    assert!(matches!(page.mode(), Mode::Confirm(1)));
    let scene = page.scene(SCREEN);
    assert!(has_text(&scene, "Forget cafe?"));
    assert!(has_text(&scene, "Enter to confirm, Esc to cancel"));
    // Other keys keep the question.
    assert_eq!(page.press(Key::Down), Action::Stay);
    assert_eq!(page.press(Key::Character('y')), Action::Stay);
    assert!(matches!(page.mode(), Mode::Confirm(1)));
    assert_eq!(page.press(Key::Enter), Action::Forget("cafe".to_string()));
    assert!(matches!(page.mode(), Mode::List));
    page.forgotten("cafe", Ok(()), vec![configured("home"), saved("lab")]);
    assert_eq!(page.notice(), Some("Forgot cafe."));
    assert_eq!(page.selected(), Row::Network(1));
}

#[test]
fn escape_and_backspace_in_the_question() {
    let mut page = page();
    page.press(Key::End);
    page.press(Key::Up);
    assert_eq!(page.press(Key::Backspace), Action::Stay);
    assert!(matches!(page.mode(), Mode::Confirm(2)));
    assert_eq!(page.press(Key::Escape), Action::Stay);
    assert!(matches!(page.mode(), Mode::List));
    assert_eq!(page.selected(), Row::Network(2));
}

#[test]
fn the_configured_network_is_never_forgotten_here() {
    let mut page = page();
    assert_eq!(page.press(Key::Delete), Action::Stay);
    assert!(matches!(page.mode(), Mode::List));
    assert!(page.notice().is_some_and(|n| n.contains("stela.cfg")));
    assert!(!has_text(&page.scene(SCREEN), "Delete: forget"));
}

#[test]
fn forgetting_the_last_row_keeps_a_row_selected() {
    let mut page = page();
    page.press(Key::End);
    page.press(Key::Up);
    page.press(Key::Delete);
    page.press(Key::Enter);
    page.forgotten("lab", Err("not written"), vec![configured("home")]);
    assert_eq!(page.selected(), Row::JoinOther);
    assert_eq!(page.notice(), Some("not written"));
}

// Adding.

#[test]
fn the_dialog_takes_a_name_then_a_password_and_saves() {
    let mut page = adding();
    typed(&mut page, "Mom's phone");
    assert_eq!(page.press(Key::Enter), Action::Stay);
    assert_eq!(form(&page).field(), Field::Password);
    typed(&mut page, "100% pass word");
    let Action::Save(save) = page.press(Key::Enter) else {
        panic!("a network to save");
    };
    assert_eq!(save.name, "Mom's phone");
    assert_eq!(save.password, b"100% pass word");
    assert!(matches!(page.mode(), Mode::List));
    // The file line it makes reads back as that network.
    let line = wifi_saved::line(save.name.as_bytes(), &save.password);
    assert_eq!(wifi_saved::parse(&line).networks.len(), 1);
    assert!(!format!("{save:?}").contains("pass word"));
}

#[test]
fn a_saved_network_joins_first_or_stays_selected() {
    let mut page = adding();
    let after = vec![
        configured("home"),
        saved("cafe"),
        saved("lab"),
        saved("new"),
    ];
    assert_eq!(
        page.saved("new", Ok(()), after.clone()),
        Action::Join("new".to_string())
    );
    let mut page = Page::new(vec![saved("cafe")], false, None);
    page.press(Key::Down);
    page.press(Key::Enter);
    assert_eq!(page.saved("new", Ok(()), after.clone()), Action::Stay);
    assert_eq!(page.selected(), Row::Network(3));
    assert_eq!(page.notice(), Some("Saved new."));
    assert_eq!(page.saved("new", Err("not written"), after), Action::Stay);
    assert_eq!(page.notice(), Some("not written"));
}

#[test]
fn escape_cancels_the_dialog_from_either_field() {
    let mut page = adding();
    typed(&mut page, "cafe");
    assert_eq!(page.press(Key::Escape), Action::Stay);
    assert!(matches!(page.mode(), Mode::List));
    assert_eq!(page.selected(), Row::JoinOther);
    page.press(Key::Enter);
    assert_eq!(form(&page).name(), "", "a new dialog starts empty");
    typed(&mut page, "cafe");
    page.press(Key::Enter);
    typed(&mut page, "coffee-time");
    assert_eq!(page.press(Key::Escape), Action::Stay);
    assert!(matches!(page.mode(), Mode::List));
}

#[test]
fn an_empty_name_is_refused() {
    let mut page = adding();
    assert_eq!(page.press(Key::Enter), Action::Stay);
    assert_eq!(form(&page).field(), Field::Name);
    assert_eq!(form(&page).error(), Some(EMPTY_NAME));
    assert!(has_text(&page.scene(SCREEN), EMPTY_NAME));
    // Down goes on as Enter does.
    assert_eq!(page.press(Key::Down), Action::Stay);
    assert_eq!(form(&page).error(), Some(EMPTY_NAME));
}

#[test]
fn a_name_stops_at_32_bytes() {
    let mut page = adding();
    typed(&mut page, &"n".repeat(31));
    // A two-byte character no longer fits.
    page.press(Key::Character('\u{e9}'));
    assert_eq!(form(&page).error(), Some(LONG_NAME));
    assert_eq!(form(&page).name().len(), 31);
    page.press(Key::Character('n'));
    assert_eq!(form(&page).error(), None);
    page.press(Key::Character('n'));
    assert_eq!(form(&page).error(), Some(LONG_NAME));
    assert_eq!(form(&page).name().len(), 32);
    page.press(Key::Backspace);
    assert_eq!(form(&page).name().len(), 31);
    // Control characters are not typed.
    page.press(Key::Character('\u{1b}'));
    assert_eq!(form(&page).name().len(), 31);
}

#[test]
fn a_password_must_be_one_secret_parse_takes() {
    let mut page = adding();
    typed(&mut page, "cafe");
    page.press(Key::Enter);
    assert_eq!(page.press(Key::Enter), Action::Stay);
    assert_eq!(form(&page).error(), Some(EMPTY_PASSWORD));
    typed(&mut page, "short");
    assert_eq!(page.press(Key::Enter), Action::Stay);
    assert_eq!(form(&page).error(), Some(PASSWORD_LENGTH));
    for _ in 0..5 {
        page.press(Key::Backspace);
    }
    typed(&mut page, &"g".repeat(64));
    assert_eq!(page.press(Key::Enter), Action::Stay);
    assert_eq!(form(&page).error(), Some(PASSWORD_HEX));
    page.press(Key::Character('a'));
    assert_eq!(form(&page).error(), Some(LONG_PASSWORD));
    assert_eq!(form(&page).password_len(), 64);
    page.press(Key::Character('\u{e9}'));
    assert_eq!(form(&page).error(), Some(PLAIN_PASSWORD));
    for _ in 0..64 {
        page.press(Key::Backspace);
    }
    typed(&mut page, &"ab".repeat(32));
    assert!(matches!(page.press(Key::Enter), Action::Save(_)));
}

#[test]
fn up_returns_to_the_name_keeping_the_password() {
    let mut page = adding();
    typed(&mut page, "caf");
    page.press(Key::Enter);
    typed(&mut page, "coffee-time");
    page.press(Key::Up);
    assert_eq!(form(&page).field(), Field::Name);
    typed(&mut page, "e");
    page.press(Key::Enter);
    let Action::Save(save) = page.press(Key::Enter) else {
        panic!("a network to save");
    };
    assert_eq!(
        (save.name.as_str(), &save.password[..]),
        ("cafe", &b"coffee-time"[..])
    );
}

// Masking.

/// The dialog with `cafe` and `password` typed in the password field.
fn masked() -> Page {
    let mut page = adding();
    typed(&mut page, "cafe");
    page.press(Key::Enter);
    typed(&mut page, "password");
    page
}

/// The dots of a masked field: six unit circles.
fn dots(scene: &Scene) -> usize {
    scene
        .draws
        .iter()
        .filter(|draw| {
            matches!(
                draw,
                Draw::Panel {
                    width: 6,
                    height: 6,
                    radius: 3,
                    ..
                }
            )
        })
        .count()
}

#[test]
fn a_password_is_dots_until_tab_shows_it() {
    let mut page = masked();
    let scene = page.scene(SCREEN);
    assert!(!has_text(&scene, "password"));
    assert!(!has_text(&scene, "pass"));
    assert_eq!(dots(&scene), 8);
    assert!(has_text(&scene, "Tab: show"));
    drop(scene);
    page.press(Key::Tab);
    assert!(form(&page).shown());
    let scene = page.scene(SCREEN);
    assert!(has_text(&scene, "password"));
    assert_eq!(dots(&scene), 0);
    assert!(has_text(&scene, "Tab: hide"));
    drop(scene);
    page.press(Key::Tab);
    assert!(!has_text(&page.scene(SCREEN), "password"));
}

#[test]
fn tab_in_the_name_field_shows_nothing() {
    let mut page = adding();
    page.press(Key::Tab);
    assert!(!form(&page).shown());
    typed(&mut page, "cafe");
    page.press(Key::Enter);
    typed(&mut page, "password");
    page.press(Key::Up);
    // A password shown is shown wherever the focus is; hidden here.
    assert!(!has_text(&page.scene(SCREEN), "password"));
}

#[test]
fn no_other_screen_draws_the_password() {
    let mut page = masked();
    page.press(Key::Tab);
    let Action::Save(save) = page.press(Key::Enter) else {
        panic!("a network to save");
    };
    assert_eq!(save.password, b"password");
    assert!(!has_text(&page.scene(SCREEN), "password"));
    let status = status_line(Some("home"), Some("cafe"), true).unwrap_or_default();
    assert!(!status.contains("password"));
}

#[test]
fn a_long_password_shows_its_end() {
    let mut page = adding();
    typed(&mut page, "lab");
    page.press(Key::Enter);
    let psk = "0123456789abcdef".repeat(4);
    typed(&mut page, &psk);
    // The field is the column less its margins: (440 - 32 - 28) / 8.
    let capacity = 47;
    assert_eq!(dots(&page.scene(SCREEN)), capacity);
    page.press(Key::Tab);
    let scene = page.scene(SCREEN);
    assert!(scene.texts().any(|text| text == &psk[64 - capacity..]));
}

#[test]
fn the_tail_counts_characters() {
    assert_eq!(tail("caf\u{e9} 5G", 4), "\u{e9} 5G");
    assert_eq!(tail("lab", 10), "lab");
    assert_eq!(tail("lab", 0), "");
}

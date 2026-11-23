use super::*;
use alloc::string::ToString;

/// A FADT of `revision` whose Preferred_PM_Profile is `profile`.
fn fadt(revision: u8, profile: u8) -> Vec<u8> {
    let mut table = alloc::vec![0; 276];
    table[..4].copy_from_slice(b"FACP");
    table[8] = revision;
    table[FADT_PM_PROFILE] = profile;
    table
}

/// Garvit's ASUS TUF A16 (FA608PP): its FADT's first 48 bytes as dumped,
/// revision 6 and profile 2 at byte 45, and a battery, a lid and an ELAN
/// precision touchpad.
const TUF_A16_FADT: [u8; 48] = [
    0x46, 0x41, 0x43, 0x50, 0x14, 0x01, 0x00, 0x00, 0x06, 0x5d, 0x5f, 0x41, 0x53, 0x55, 0x53, 0x5f,
    0x4e, 0x6f, 0x74, 0x65, 0x62, 0x6f, 0x6f, 0x6b, 0x09, 0x20, 0x07, 0x01, 0x41, 0x4d, 0x49, 0x20,
    0x13, 0x00, 0x01, 0x00, 0x00, 0x40, 0xb2, 0xb6, 0x00, 0x00, 0xb2, 0xb4, 0x00, 0x02, 0x09, 0x00,
];

fn laptop() -> Facts {
    Facts {
        pm_profile: Some(2),
        battery: true,
        lid: true,
        precision_touchpad: true,
        trackpad_port: true,
    }
}

fn desktop() -> Facts {
    Facts {
        pm_profile: Some(1),
        ..Facts::default()
    }
}

/// The installed macOS's line, as setup writes it: with `--input`.
fn installed(extra: &str) -> System {
    System::parse(&alloc::format!("--system --input{extra}")).unwrap()
}

#[test]
fn the_profile_is_read_from_revision_2_on() {
    assert_eq!(pm_profile(&fadt(6, 2)), Some(2));
    assert_eq!(pm_profile(&fadt(2, 8)), Some(8));
    // ACPI 1.0's byte 45 is reserved.
    assert_eq!(pm_profile(&fadt(1, 2)), None);
    assert_eq!(pm_profile(&fadt(6, 2)[..45]), None);
    assert_eq!(pm_profile(&[]), None);
    assert_eq!(pm_profile(&TUF_A16_FADT), Some(2));
}

#[test]
fn a_laptop_is_a_mobile_class_profile_or_a_battery_and_a_lid() {
    for profile in [2, 6, 8] {
        let facts = Facts {
            pm_profile: Some(profile),
            ..Facts::default()
        };
        assert!(facts.laptop(), "{}", profile_name(profile));
    }
    for profile in [0, 1, 3, 4, 5, 7, 9, 0xff] {
        let facts = Facts {
            pm_profile: Some(profile),
            ..Facts::default()
        };
        assert!(!facts.laptop(), "{}", profile_name(profile));
        // A battery and a lid make one whatever the profile says.
        assert!(
            Facts {
                battery: true,
                lid: true,
                ..facts
            }
            .laptop()
        );
        // One of them alone does not: a UPS battery, a desktop's lid switch.
        assert!(
            !Facts {
                battery: true,
                ..facts
            }
            .laptop()
        );
        assert!(!Facts { lid: true, ..facts }.laptop());
    }
    assert!(!Facts::default().laptop());
    assert!(
        Facts {
            battery: true,
            lid: true,
            ..Facts::default()
        }
        .laptop()
    );
    // A touchpad alone is no laptop: USB touchpads plug into desktops.
    assert!(
        !Facts {
            precision_touchpad: true,
            trackpad_port: true,
            ..desktop()
        }
        .laptop()
    );
}

#[test]
fn a_laptop_takes_every_feature_but_lid_sleep() {
    let mut options = installed("");
    let plan = plan(&options, laptop());
    assert_eq!(
        [plan.battery, plan.lid, plan.brightness_keys, plan.trackpad],
        [Choice::Taken; 4]
    );
    assert!(plan.takes_brightness_keys());
    plan.apply(&mut options);
    assert_eq!(options.battery, Some(Battery::Host));
    assert_eq!(options.lid(), Lid::Method);
    assert!(options.brightness_keys);
    assert_eq!(options.guest_trackpad(), Some(GuestTrackpad::Mt2));
    assert!(!options.lid_sleep);
    // The record stays one the parser accepts.
    assert_eq!(options.check(), Ok(()));
    assert_eq!(
        plan.line(),
        "LAPTOP detected (FADT mobile, battery, lid): battery on, lid method, brightness keys, \
         trackpad mt2\n"
    );
}

#[test]
fn a_desktop_takes_nothing_and_keeps_what_is_given() {
    let mut options = installed("");
    let before = options.clone();
    let plan = plan(&options, desktop());
    plan.apply(&mut options);
    assert_eq!(options, before);
    assert!(!plan.takes_brightness_keys());
    assert_eq!(
        plan.line(),
        "DESKTOP (FADT desktop, no battery, no lid): no laptop features\n"
    );
    let given = installed(" --battery --brightness-keys");
    assert_eq!(
        super::plan(&given, desktop()).line(),
        "DESKTOP (FADT desktop, no battery, no lid): no laptop features but those given: \
         --battery as given, --brightness-keys as given\n"
    );
    let unread = Facts::default();
    assert_eq!(
        super::plan(&installed(""), unread).line(),
        "DESKTOP (no FADT profile, no battery, no lid): no laptop features\n"
    );
}

#[test]
fn laptop_off_takes_nothing_but_the_options_given() {
    let mut options = installed(" --laptop off");
    let before = options.clone();
    let plan = plan(&options, laptop());
    plan.apply(&mut options);
    assert_eq!(options, before);
    assert_eq!(plan.battery, Choice::Skipped("--laptop off"));
    assert_eq!(
        plan.line(),
        "LAPTOP detected (FADT mobile, battery, lid), --laptop off: no laptop features\n"
    );
    let given = installed(" --laptop off --lid method");
    let plan = super::plan(&given, laptop());
    assert_eq!(plan.lid, Choice::Given("--lid method"));
    assert_eq!(
        plan.line(),
        "LAPTOP detected (FADT mobile, battery, lid), --laptop off: only those given: \
         --lid method as given\n"
    );
}

#[test]
fn explicit_options_win() {
    // `--lid ignore` keeps the lid ignored, a test battery and the
    // built-in trackpad ID stand, and `--lid-sleep` with its method too.
    for (extra, check) in [
        (
            " --lid ignore",
            (|options: &System| options.lid() == Lid::Ignore) as fn(&System) -> bool,
        ),
        (" --test-battery 50,charging", |options| {
            options.battery
                == Some(Battery::Test {
                    percent: 50,
                    state: crate::options::TestState::Charging,
                })
        }),
        (" --trackpad builtin", |options| {
            options.trackpad == Some(GuestTrackpad::Builtin)
        }),
        (" --lid method --lid-sleep", |options| {
            options.lid() == Lid::Method && options.lid_sleep
        }),
    ] {
        let mut options = installed(extra);
        let plan = plan(&options, laptop());
        plan.apply(&mut options);
        assert!(check(&options), "{extra}");
        assert_eq!(options.check(), Ok(()), "{extra}");
    }
    let options = installed(" --lid ignore --test-battery 50,charging --trackpad builtin");
    assert_eq!(
        plan(&options, laptop()).line(),
        "LAPTOP detected (FADT mobile, battery, lid): --test-battery as given, --lid ignore as \
         given, brightness keys, --trackpad builtin as given\n"
    );
}

#[test]
fn each_feature_without_its_device_is_skipped_with_its_reason() {
    // A mobile profile alone, with no battery, lid or touchpad.
    let bare = Facts {
        pm_profile: Some(2),
        ..Facts::default()
    };
    let mut options = installed("");
    let plan = plan(&options, bare);
    assert_eq!(plan.battery, Choice::Skipped("no ACPI battery"));
    assert_eq!(plan.lid, Choice::Skipped("no ACPI lid"));
    assert_eq!(plan.brightness_keys, Choice::Taken);
    assert_eq!(plan.trackpad, Choice::Skipped("no precision touchpad"));
    plan.apply(&mut options);
    assert_eq!(options.battery, None);
    assert_eq!(options.lid, None);
    assert_eq!(options.trackpad, None);
    assert_eq!(
        plan.line(),
        "LAPTOP detected (FADT mobile, no battery, no lid): brightness keys; skipped: battery \
         (no ACPI battery), lid (no ACPI lid), trackpad (no precision touchpad)\n"
    );
    // The trackpad needs host input, then a free hub port.
    let no_input = System::parse("--system").unwrap();
    assert_eq!(
        super::plan(&no_input, laptop()).trackpad,
        Choice::Skipped("no --input")
    );
    let full_hub = Facts {
        trackpad_port: false,
        ..laptop()
    };
    let plan = super::plan(&installed(""), full_hub);
    assert_eq!(
        plan.trackpad,
        Choice::Skipped("no free port on the guest's USB hub")
    );
    let mut options = installed("");
    plan.apply(&mut options);
    assert_eq!(options.trackpad, None);
}

#[test]
fn a_boot_that_withholds_the_battery_does_not_take_it() {
    const DISK: &str = "74EXN0LI:11111111-2222-3333-4444-555555555555";
    for (line, boot) in [
        (
            alloc::format!(
                "--system --input --storage {DISK} --nor {DISK} --bundle b --install p.json"
            ),
            "the macOS installer",
        ),
        (
            alloc::format!("--system --input --preboot {DISK} --upgrade"),
            "a staged macOS update",
        ),
        (
            alloc::format!("--system --input --preboot {DISK} --paired-recovery"),
            "recoveryOS",
        ),
        (
            alloc::format!("--system --input --preboot {DISK} --bundle-repair"),
            "the Metal plugin repair",
        ),
    ] {
        let mut options = System::parse(&line).unwrap();
        let plan = plan(&options, laptop());
        assert_eq!(plan.battery, Choice::Withheld(boot), "{line}");
        plan.apply(&mut options);
        assert_eq!(options.battery, None, "{line}");
        assert!(
            plan.line()
                .contains(&alloc::format!("skipped: battery (not given to {boot})")),
            "{}",
            plan.line()
        );
        // The other features stay.
        assert_eq!(options.lid(), Lid::Method, "{line}");
    }
    // The installed macOS takes it, in single-user and safe mode too.
    for flag in ["", " --single-user", " --safe-boot"] {
        let options =
            System::parse(&alloc::format!("--system --input --preboot {DISK}{flag}")).unwrap();
        assert_eq!(plan(&options, laptop()).battery, Choice::Taken, "{flag}");
    }
}

#[test]
fn the_tuf_a16_is_a_laptop_with_every_feature() {
    let facts = Facts {
        pm_profile: pm_profile(&TUF_A16_FADT),
        ..laptop()
    };
    assert_eq!(
        plan(&installed(""), facts).line().to_string(),
        "LAPTOP detected (FADT mobile, battery, lid): battery on, lid method, brightness keys, \
         trackpad mt2\n"
    );
}

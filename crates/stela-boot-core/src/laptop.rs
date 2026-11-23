//! Whether the PC is a laptop, and the laptop features a boot then takes
//! without being asked (`--laptop auto`, the default).
//!
//! Setup writes none of the laptop options and the startup menu offers
//! none, so each was a line to find and hand-write. The boot reads facts it
//! reads anyway instead: the FADT's Preferred_PM_Profile, the ACPI batteries
//! (PNP0C0A, present by `_STA`) and the lid (PNP0C0D) the power probe
//! attached, and the precision touchpads among the host's inputs. A laptop
//! is a mobile-class profile ([`mobile_class`]), or a battery and a lid
//! together, which a laptop whose firmware leaves the profile unspecified
//! or says desktop still has. On a laptop the boot behaves as if `--battery`,
//! `--lid method` and `--brightness-keys` were given, and `--trackpad mt2`
//! too where a precision touchpad and `--input` are there ([`plan`]).
//!
//! What the line gives wins: a `--test-battery`, `--lid ignore`,
//! `--trackpad builtin`. `--laptop off` takes nothing, so each feature stays
//! the opt-in it was ([`crate::options::Laptop`]). `--lid-sleep` is never
//! taken: it puts macOS to sleep each time the lid darkens the panel, which
//! is the user's to ask for. The guards of each option hold as they did: the battery is not
//! taken for a boot that withholds it ([`System::battery_withheld`]), and
//! the battery's own refusal of an empty reading is the engine's, after
//! this. None of this is in the option line, whose meaning setup and
//! `stela.efi` share (`stela_loader::mount_interface::OPTIONS_REVISION`): a
//! line means what it meant, and only a boot's reading of its PC adds to it.
use crate::options::{Battery, GuestTrackpad, Laptop, Lid, System};
use alloc::{format, string::String, vec::Vec};

/// Byte offset of the FADT's Preferred_PM_Profile (ACPI 6.5, 5.2.9).
pub const FADT_PM_PROFILE: usize = 45;

/// The FADT's Preferred_PM_Profile; `None` without one, or before revision
/// 2, where ACPI 1.0 kept that byte reserved.
pub fn pm_profile(fadt: &[u8]) -> Option<u8> {
    match fadt.get(8) {
        Some(&revision) if revision >= 2 => fadt.get(FADT_PM_PROFILE).copied(),
        _ => None,
    }
}

/// A Preferred_PM_Profile's name (ACPI 6.5, table 5.9).
pub fn profile_name(profile: u8) -> &'static str {
    match profile {
        0 => "unspecified",
        1 => "desktop",
        2 => "mobile",
        3 => "workstation",
        4 => "enterprise server",
        5 => "SOHO server",
        6 => "appliance PC",
        7 => "performance server",
        8 => "tablet",
        _ => "reserved",
    }
}

/// Whether a profile names a laptop-class machine: Mobile, Tablet or
/// Appliance PC. Each laptop feature also needs its own device, so a
/// machine that only claims the class gains nothing it lacks.
pub fn mobile_class(profile: u8) -> bool {
    matches!(profile, 2 | 6 | 8)
}

/// What the boot read of the PC.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Facts {
    /// The FADT's Preferred_PM_Profile ([`pm_profile`]).
    pub pm_profile: Option<u8>,
    /// An ACPI battery (PNP0C0A) present by `_STA` attached.
    pub battery: bool,
    /// An ACPI lid (PNP0C0D) attached.
    pub lid: bool,
    /// A host input is a precision touchpad.
    pub precision_touchpad: bool,
    /// The guest's USB hub keeps a port for a trackpad beside the disks
    /// it took.
    pub trackpad_port: bool,
}

impl Facts {
    /// A mobile-class profile, or a battery and a lid.
    pub fn laptop(&self) -> bool {
        self.pm_profile.is_some_and(mobile_class) || (self.battery && self.lid)
    }

    /// `FADT mobile, battery, lid`: what the decision read.
    fn describe(&self) -> String {
        let profile = match self.pm_profile {
            Some(profile) => format!("FADT {}", profile_name(profile)),
            None => String::from("no FADT profile"),
        };
        let have = |present: bool, name: &str| match present {
            true => String::from(name),
            false => format!("no {name}"),
        };
        format!(
            "{profile}, {}, {}",
            have(self.battery, "battery"),
            have(self.lid, "lid")
        )
    }
}

/// What a boot does with one laptop feature.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Choice {
    /// Taken as if its option were given.
    Taken,
    /// Its option, as the line gives it, stands.
    Given(&'static str),
    /// Not taken, for this reason.
    Skipped(&'static str),
    /// Not taken: the boot this names withholds it.
    Withheld(&'static str),
}

/// The laptop features one boot takes ([`plan`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Plan {
    pub facts: Facts,
    /// `--laptop`'s value.
    pub mode: Laptop,
    pub battery: Choice,
    pub lid: Choice,
    pub brightness_keys: Choice,
    pub trackpad: Choice,
}

/// The laptop features `options` and `facts` give this boot. Off a laptop,
/// or with `--laptop off`, each is what the line gives or skipped.
pub fn plan(options: &System, facts: Facts) -> Plan {
    let auto = facts.laptop() && options.laptop() == Laptop::Auto;
    let not_laptop = match options.laptop() {
        Laptop::Off => "--laptop off",
        Laptop::Auto => "not a laptop",
    };
    let choose = |given: Option<&'static str>, skip: Option<Choice>| match given {
        Some(spelling) => Choice::Given(spelling),
        None if !auto => Choice::Skipped(not_laptop),
        None => skip.unwrap_or(Choice::Taken),
    };
    let battery = choose(
        options.battery.map(|battery| match battery {
            Battery::Host => "--battery",
            Battery::Test { .. } => "--test-battery",
        }),
        // A boot that withholds the battery says so first: an install's.
        match options.battery_withheld(false) {
            Some(boot) => Some(Choice::Withheld(boot)),
            None => (!facts.battery).then_some(Choice::Skipped("no ACPI battery")),
        },
    );
    let lid = choose(
        options.lid.map(|lid| match lid {
            Lid::Ignore => "--lid ignore",
            Lid::Method => "--lid method",
        }),
        (!facts.lid).then_some(Choice::Skipped("no ACPI lid")),
    );
    let brightness_keys = choose(options.brightness_keys.then_some("--brightness-keys"), None);
    let trackpad = choose(
        options.trackpad.map(|kind| match kind {
            GuestTrackpad::Mt2 => "--trackpad mt2",
            GuestTrackpad::Builtin => "--trackpad builtin",
        }),
        if !options.input {
            Some(Choice::Skipped("no --input"))
        } else if !facts.precision_touchpad {
            Some(Choice::Skipped("no precision touchpad"))
        } else if !facts.trackpad_port {
            Some(Choice::Skipped("no free port on the guest's USB hub"))
        } else {
            None
        },
    );
    Plan {
        facts,
        mode: options.laptop(),
        battery,
        lid,
        brightness_keys,
        trackpad,
    }
}

impl Plan {
    /// Whether the boot takes `--brightness-keys` itself, after the ACPI
    /// hardware mode that attaches them with the option has started.
    pub fn takes_brightness_keys(&self) -> bool {
        self.brightness_keys == Choice::Taken
    }

    /// Set the features taken in `options`, as their options would.
    pub fn apply(&self, options: &mut System) {
        if self.battery == Choice::Taken {
            options.battery = Some(Battery::Host);
        }
        if self.lid == Choice::Taken {
            options.lid = Some(Lid::Method);
        }
        if self.brightness_keys == Choice::Taken {
            options.brightness_keys = true;
        }
        if self.trackpad == Choice::Taken {
            options.trackpad = Some(GuestTrackpad::Mt2);
        }
    }

    /// The boot's one line: `LAPTOP detected (FADT mobile, battery, lid):
    /// battery on, lid method, brightness keys, trackpad mt2`, each skipped
    /// feature after with its reason, or `DESKTOP (...): no laptop
    /// features`. Off a laptop a feature the line gives is still named.
    pub fn line(&self) -> String {
        let facts = self.facts.describe();
        let features = [
            ("battery", "battery on", self.battery),
            ("lid", "lid method", self.lid),
            ("brightness keys", "brightness keys", self.brightness_keys),
            ("trackpad", "trackpad mt2", self.trackpad),
        ];
        let given: Vec<String> = features
            .iter()
            .filter_map(|&(_, _, choice)| match choice {
                Choice::Given(spelling) => Some(format!("{spelling} as given")),
                _ => None,
            })
            .collect();
        if !self.facts.laptop() {
            return match given.is_empty() {
                true => format!("DESKTOP ({facts}): no laptop features\n"),
                false => format!(
                    "DESKTOP ({facts}): no laptop features but those given: {}\n",
                    given.join(", ")
                ),
            };
        }
        if self.mode == Laptop::Off {
            return match given.is_empty() {
                true => format!("LAPTOP detected ({facts}), --laptop off: no laptop features\n"),
                false => format!(
                    "LAPTOP detected ({facts}), --laptop off: only those given: {}\n",
                    given.join(", ")
                ),
            };
        }
        let mut on = Vec::new();
        let mut skipped = Vec::new();
        for (name, taken, choice) in features {
            match choice {
                Choice::Taken => on.push(String::from(taken)),
                Choice::Given(spelling) => on.push(format!("{spelling} as given")),
                Choice::Skipped(reason) => skipped.push(format!("{name} ({reason})")),
                Choice::Withheld(boot) => skipped.push(format!("{name} (not given to {boot})")),
            }
        }
        let mut line = format!("LAPTOP detected ({facts}): ");
        line.push_str(&match on.is_empty() {
            true => String::from("no laptop features"),
            false => on.join(", "),
        });
        if !skipped.is_empty() {
            line.push_str("; skipped: ");
            line.push_str(&skipped.join(", "));
        }
        line.push('\n');
        line
    }
}

/// The line of a boot that opens no host devices, and so reads no battery,
/// lid or touchpad.
pub const UNREAD: &str = "LAPTOP detection skipped: this boot opens no host devices; only the laptop options given apply\n";

#[cfg(test)]
mod tests;

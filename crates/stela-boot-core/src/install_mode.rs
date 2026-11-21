//! Install mode's options in a boot's chosen option line: `--install-mode
//! verify` and `--install-build BUILD`, which `stela.efi` reads before
//! any other parsing (`payload::run`) to find and verify a restore payload
//! and stop. Every other word goes on to the mount's own parser.
use crate::Result;
use alloc::string::String;
use stela_contracts::refuse;

/// Install mode's option that turns it on.
pub const MODE: &str = "--install-mode";
/// The build of the payload install mode verifies.
pub const BUILD: &str = "--install-build";

/// The install-mode options a line holds.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct InstallMode {
    pub verify: bool,
    pub build: Option<String>,
}

/// Read the install options in `options`: each of [`MODE`] and [`BUILD`],
/// spelled exactly, wherever it stands, takes the word after it. Every
/// other word is the mount's, which its own parser reads and refuses when
/// it does not know it, so no option of the mount's, whatever its name or
/// place in the line (a flag without a value before it among them), is
/// read as one of install mode's.
pub fn parse(options: &str) -> Result<InstallMode> {
    let mut install = InstallMode::default();
    let mut words = options.split_ascii_whitespace();
    while let Some(name) = words.next() {
        if name != MODE && name != BUILD {
            continue;
        }
        let value = words
            .next()
            .ok_or(refuse!("install option without a value"))?;
        match name {
            MODE if value == "verify" => install.verify = true,
            MODE => return Err(refuse!("--install-mode must be verify")),
            _ => install.build = Some(value.into()),
        }
    }
    if !install.verify && install.build.is_some() {
        return Err(refuse!("install options need --install-mode"));
    }
    Ok(install)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::{string::ToString, vec::Vec};

    /// The Install macOS line setup writes (`stela-cli`'s setup tests).
    const INSTALL_LINE: &str = "--system --storage 74EXN0LI:11111111-2222-3333-4444-555555555555 \
        --nor 74EXN0LI:22222222-3333-4444-5555-666666666666 --bundle \\stela\\boot\\22A\\r \
        --ecid 0x0000000000000001 --install \\stela\\install\\22A\\payload.json \
        --install-source 33333333-4444-5555-6666-777777777777 --development-restore";

    /// `options` with `inserted` among its words before word `at`.
    fn with(options: &str, inserted: &str, at: usize) -> String {
        let mut words: Vec<&str> = options.split_ascii_whitespace().collect();
        let mut tail = words.split_off(at);
        words.extend(inserted.split_ascii_whitespace());
        words.append(&mut tail);
        words.join(" ")
    }

    #[test]
    fn install_mode_reads_its_mode_and_build_and_refuses_bad_ones() {
        let verify = parse("--install-mode verify --install-build 22A").unwrap();
        assert_eq!(
            verify,
            InstallMode {
                verify: true,
                build: Some("22A".to_string())
            }
        );
        assert_eq!(
            parse("--system --storage x").unwrap(),
            InstallMode::default()
        );
        for line in [
            "--install-mode other",
            "--install-mode",
            "--install-build 22A",
            "--install-mode verify --install-build",
        ] {
            assert!(parse(line).is_err(), "{line}");
        }
    }

    /// No option of the mount's, whatever its name and wherever it stands
    /// in setup's Install line, is read as an install-mode option: the
    /// line goes on to the mount whole.
    #[test]
    fn no_mount_option_anywhere_in_the_install_line_is_read_as_install_mode() {
        let options = [
            "--medium-read-ahead 0",
            "--install-source 33333333-4444-5555-6666-777777777777",
            "--install \\stela\\install\\22A\\payload.json",
            "--install-future 3",
            "--verify-on-read",
            "--update-restore",
            "--external-disks",
            "--usb-storage A",
        ];
        let words = INSTALL_LINE.split_ascii_whitespace().count();
        assert_eq!(words, 14);
        let read = |line: &str| parse(line).unwrap_or_else(|error| panic!("{line}: {error:?}"));
        for option in options {
            for at in 1..=words {
                let line = with(INSTALL_LINE, option, at);
                assert_eq!(read(&line), InstallMode::default(), "{line}");
            }
        }
        // Install mode's own options are read wherever they stand.
        for at in 1..=words {
            let line = with(
                INSTALL_LINE,
                "--install-mode verify --install-build 22A",
                at,
            );
            assert!(read(&line).verify, "{line}");
        }
    }
}

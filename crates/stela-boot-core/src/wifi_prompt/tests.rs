use super::*;
use alloc::vec::Vec;

/// An earlier boot's log as the USB keeps it: its marker line, stamped
/// lines, and `last` at its end.
fn boot(lines: &[&str]) -> Vec<u8> {
    let mut text = String::from("[   99.000] an even earlier boot's last line\n");
    text.push_str("=== STELA BOOT 12 === version=0.6.1 firmware_clock=2026-11-24T14:54:23\n");
    for (n, line) in lines.iter().enumerate() {
        text.push_str(&format!("[{:>8}.000] {line}", n + 1));
        if !line.ends_with('\n') {
            text.push('\n');
        }
    }
    text.into_bytes()
}

fn after_power_off(records: &[Record]) -> Earlier {
    let mut lines: Vec<String> = records.iter().map(|record| record.line()).collect();
    lines.push(String::from("STELA POWER-OFF code=0x10"));
    let lines: Vec<&str> = lines.iter().map(String::as_str).collect();
    earlier(&boot(&lines))
}

/// A boot that has nothing in its way: a supported chip, a menu, no
/// network, after a boot macOS turned off.
fn open(earlier: Earlier) -> Facts {
    Facts {
        earlier: Some(earlier),
        supported: true,
        held: false,
        no_menu: false,
        network: false,
    }
}

#[test]
fn each_record_reads_back_as_written_and_names_no_network() {
    for record in [
        Record::Joined(Which::Configured),
        Record::Joined(Which::Saved(3)),
        Record::Failed(Which::Configured, Cause::Password),
        Record::Failed(Which::Saved(1), Cause::Range),
        Record::Wired,
        Record::Cleared,
    ] {
        let line = record.line();
        assert!(line.starts_with(RECORD) && line.ends_with('\n'), "{line}");
        let stamped = format!("[  91.855] {line}");
        assert_eq!(Record::parse(&stamped), Some(record));
    }
    assert_eq!(
        Record::parse("WIFI RECORD failed network=saved-0 cause=range"),
        None
    );
    assert_eq!(
        Record::parse("WIFI RECORD failed network=home cause=range"),
        None
    );
}

#[test]
fn the_earlier_boot_is_the_part_after_the_last_marker() {
    assert_eq!(earlier(b"").ending, Ending::First);
    let restart = earlier(&boot(&["GUEST| ...", "STELA RESTART code=0x12"]));
    assert_eq!(restart.ending, Ending::Restart);
    assert_eq!(
        earlier(&boot(&["STELA STOPPED code=0x13; power off"])).ending,
        Ending::Stopped
    );
    assert_eq!(
        earlier(&boot(&["GUEST| still running"])).ending,
        Ending::Unfinished
    );
    // A line of an older boot before the marker is not this one's.
    let mut stream = boot(&["STELA POWER-OFF code=0x10"]);
    stream.extend_from_slice(b"=== STELA BOOT 13 === version=0.6.1\n[ 1.0] HOST ...\n");
    assert_eq!(earlier(&stream).ending, Ending::Unfinished);
    // Bytes with no marker are a boot whose marker the ring wrote over.
    assert_eq!(
        earlier(b"[ 1.0] STELA POWER-OFF code=0x10\n").ending,
        Ending::PowerOff
    );
    let records = after_power_off(&[
        Record::Failed(Which::Saved(2), Cause::Password),
        Record::Wired,
        Record::Joined(Which::Saved(2)),
    ]);
    assert_eq!(records.wifi, Some(Record::Joined(Which::Saved(2))));
    assert!(records.wired);
}

/// The page opens by itself on a normal boot with no network, and after a
/// failed join on that network, with the password field ready (the page's
/// `Page::retry`).
#[test]
fn a_boot_with_no_network_or_a_failed_join_opens_the_page() {
    assert_eq!(decide(&open(after_power_off(&[]))), Ok(Prompt::Choose));
    assert_eq!(
        decide(&open(earlier(b""))),
        Ok(Prompt::Choose),
        "a new USB's first boot"
    );
    let failed = after_power_off(&[Record::Failed(Which::Configured, Cause::Password)]);
    let facts = Facts {
        network: true,
        ..open(failed)
    };
    assert_eq!(
        decide(&facts),
        Ok(Prompt::Retry(Which::Configured, Cause::Password))
    );
    let missing = after_power_off(&[Record::Failed(Which::Saved(2), Cause::Range)]);
    assert_eq!(
        decide(&Facts {
            network: true,
            ..open(missing)
        }),
        Ok(Prompt::Retry(Which::Saved(2), Cause::Range))
    );
    assert!(line(&decide(&facts)).contains("network=configured"));
}

/// Every boot that may run with nobody at the PC goes on with no page.
#[test]
fn a_boot_nobody_may_be_at_never_opens_the_page() {
    let failed = after_power_off(&[Record::Failed(Which::Configured, Cause::Password)]);
    let ended = |ending: Ending| Earlier { ending, ..failed };
    let cases = [
        // The installer's restarts, an update's, any boot an install chains into.
        (open(ended(Ending::Restart)), Closed::Restart),
        // A crash's or a hang's recovery boot.
        (open(ended(Ending::Stopped)), Closed::Stopped),
        (open(ended(Ending::Unfinished)), Closed::Unfinished),
        // A log that did not read says nothing of the earlier boot.
        (
            Facts {
                earlier: None,
                ..open(failed)
            },
            Closed::Unread,
        ),
        (
            Facts {
                held: true,
                ..open(failed)
            },
            Closed::Held,
        ),
        (
            Facts {
                no_menu: true,
                ..open(failed)
            },
            Closed::NoMenu,
        ),
        // A wired network was up: Ethernet does the job.
        (open(after_power_off(&[Record::Wired])), Closed::Wired),
        // Stambha joined.
        (
            Facts {
                network: true,
                ..open(after_power_off(&[Record::Joined(Which::Configured)]))
            },
            Closed::Joined,
        ),
        // A chip the driver does not start, or none: only the menu's line.
        (
            Facts {
                supported: false,
                ..open(failed)
            },
            Closed::Unsupported,
        ),
    ];
    for (facts, closed) in cases {
        assert_eq!(decide(&facts), Err(closed), "{facts:?}");
        assert!(line(&decide(&facts)).starts_with("CHOICE Wi-Fi page: stays closed: "));
    }
    // A restart that follows a failure never opens it either.
    let restarted = Facts {
        network: true,
        ..open(ended(Ending::Restart))
    };
    assert_eq!(decide(&restarted), Err(Closed::Restart));
}

/// Once a join worked, or the page saved or forgot a network, the failure
/// is not asked about again; a network set up with no failure is left be.
#[test]
fn a_join_or_an_edit_clears_the_failure() {
    let joined = after_power_off(&[
        Record::Failed(Which::Configured, Cause::Password),
        Record::Joined(Which::Configured),
    ]);
    let cleared = after_power_off(&[
        Record::Failed(Which::Configured, Cause::Password),
        Record::Cleared,
    ]);
    for (earlier, closed) in [
        (joined, Closed::Joined),
        (cleared, Closed::NetworkSet),
        (after_power_off(&[]), Closed::NetworkSet),
    ] {
        let facts = Facts {
            network: true,
            ..open(earlier)
        };
        assert_eq!(decide(&facts), Err(closed));
    }
}

#[test]
fn the_retry_notice_names_the_network_and_the_way_out() {
    let password = retry_notice("home", Cause::Password);
    assert!(password.starts_with("Couldn't join home: the password may be wrong."));
    assert!(password.contains("press Esc to start without Wi-Fi"));
    assert!(retry_notice("home", Cause::Range).starts_with("home isn't in range."));
}

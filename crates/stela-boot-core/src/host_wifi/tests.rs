use super::*;

/// What an rtw89 radio prints once M3a's handshake ran (M3 spec S14.6),
/// the chip's counts after the `; `.
const HANDSHAKEN: &str = "scans 1 (12 networks, target found ch 6 -48 dBm), auth 1/1 status 0, \
     assoc 1/1 status 0 aid 3, eapol rx 2 tx 2 acked 1, keys -, port closed, \
     last \"keys before M3b\"; tx sent 4 reported 3 acked 3 full 0, rpq skipped 0 stale 0, \
     rx fcs 0 other 0 short 0 overflow 0, held dropped 0";

#[test]
fn mask_addresses_masks_every_sextet() {
    assert_eq!(
        mask_addresses("deauth from 02:03:04:05:06:07 (reason 3) to 02:11:22:33:44:55"),
        "deauth from xx:xx:xx:xx:xx:xx (reason 3) to xx:xx:xx:xx:xx:xx"
    );
    // Either case, at either end, and back to back.
    assert_eq!(
        mask_addresses("0A:bB:cc:DD:ee:FF02:03:04:05:06:07"),
        "xx:xx:xx:xx:xx:xxxx:xx:xx:xx:xx:xx"
    );
    // Not an address: too short, other separators, a non-hex digit.
    for text in [
        "02:03:04:05:06",
        "02-03-04-05-06-07",
        "02:03:04:05:06:0g",
        "2:03:04:05:06:07",
        "",
    ] {
        assert_eq!(mask_addresses(text), text);
    }
    // Text around it is kept, multi-byte characters included.
    assert_eq!(
        mask_addresses("é02:03:04:05:06:07ü \"x\""),
        "éxx:xx:xx:xx:xx:xxü \"x\""
    );
}

#[test]
fn progress_reads_the_eapol_counts_and_a_refusal() {
    let progress = Progress::read(HANDSHAKEN);
    assert_eq!(
        progress,
        Progress {
            eapol: Some(Eapol {
                received: 2,
                sent: 2,
                acked: 1
            }),
            keys_refused: true,
        }
    );
    assert!(progress.handshaken());
    // Nothing reached yet: no EAPOL, no refusal.
    let fresh = "scans 0 (0 networks, target -), auth -, assoc -, eapol -, keys -, \
                 port closed; tx sent 0 reported 0 acked 0 full 0";
    assert_eq!(Progress::read(fresh), Progress::default());
    assert!(!Progress::read(fresh).handshaken());
}

#[test]
fn the_handshake_needs_two_each_way_and_the_radios_refusal() {
    const KEYS: Option<&str> = Some("keys before M3b");
    for (eapol, last, handshaken) in [
        ("eapol rx 2 tx 2 acked 2", KEYS, true),
        ("eapol rx 3 tx 4 acked 0", KEYS, true),
        ("eapol rx 2 tx 1 acked 1", KEYS, false),
        ("eapol rx 1 tx 2 acked 2", KEYS, false),
        ("eapol rx 2 tx 2 acked 2", None, false),
        // Another refusal, or one that only begins as the first key's, is
        // no pass, however many frames went each way.
        ("eapol rx 2 tx 2 acked 2", Some("SAE before M3b"), false),
        ("eapol rx 2 tx 2 acked 2", Some("tx power"), false),
        (
            "eapol rx 2 tx 2 acked 2",
            Some("keys before M3b, again"),
            false,
        ),
        ("eapol rx 2 tx 2 acked 2", Some("no keys before M3b"), false),
        ("eapol rx 2 tx 2 acked 2", Some(""), false),
    ] {
        let line = alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), auth 1/1 status 0, assoc 1/1 \
             status 0 aid 1, {eapol}, keys -, port closed{}; tx sent 2",
            match last {
                Some(last) => alloc::format!(", last \"{last}\""),
                None => String::new(),
            }
        );
        assert_eq!(Progress::read(&line).handshaken(), handshaken, "{line}");
    }
    // The refusal is the station's own, before the chip's counts.
    assert!(
        !Progress::read(
            "eapol rx 2 tx 2 acked 2, keys -, port closed; tx sent 2, last \"keys before M3b\""
        )
        .handshaken()
    );
    // A malformed count reads as no EAPOL at all.
    assert_eq!(
        Progress::read("eapol rx 2 tx x acked 2, last \"a\"").eapol,
        None
    );
}

/// A progress line with `rx`/`tx` EAPOL frames, a refusal when `last`.
fn line(rx: u32, tx: u32, last: bool) -> Progress {
    Progress::read(&alloc::format!(
        "scans 1 (1 networks, target found ch 1 -40 dBm), eapol rx {rx} tx {tx} acked 0, keys -, \
         port closed{}; tx sent 2",
        if last {
            ", last \"keys before M3b\""
        } else {
            ""
        }
    ))
}

#[test]
fn the_watch_polls_on_for_the_flush_after_the_handshake() {
    let mut watch = Watch::new(1_000, None);
    assert_eq!(watch.after(2_000, &line(0, 0, false)), None);
    assert_eq!(watch.after(3_000, &line(2, 2, false)), None);
    // Handshaken at 4 ms: polled on for FLUSH_US more, whatever the line
    // says meanwhile, then passed.
    assert_eq!(watch.after(4_000, &line(2, 2, true)), None);
    assert_eq!(watch.after(4_000 + FLUSH_US - 1, &line(2, 2, true)), None);
    assert_eq!(watch.after(4_000 + FLUSH_US - 1, &line(0, 0, false)), None);
    assert_eq!(
        watch.after(4_000 + FLUSH_US, &line(2, 2, true)),
        Some(End::Handshaken)
    );
}

#[test]
fn the_watch_fails_at_the_budget_without_the_handshake() {
    let mut watch = Watch::new(1_000, None);
    assert_eq!(
        watch.after(1_000 + STATION_BUDGET_US - 1, &line(2, 1, true)),
        None
    );
    assert_eq!(
        watch.after(1_000 + STATION_BUDGET_US, &line(2, 1, true)),
        Some(End::Budget)
    );
    // A handshake just before the budget still gets its flush, then passes.
    let mut watch = Watch::new(0, None);
    assert_eq!(watch.after(STATION_BUDGET_US - 10, &line(2, 2, true)), None);
    assert_eq!(watch.after(STATION_BUDGET_US + 10, &line(2, 2, true)), None);
    assert_eq!(
        watch.after(STATION_BUDGET_US - 10 + FLUSH_US, &line(2, 2, true)),
        Some(End::Handshaken)
    );
    // A clock that runs backwards ends nothing early.
    for deadline in [None, Some(100_000_000)] {
        let mut watch = Watch::new(5_000, deadline);
        assert_eq!(watch.after(0, &line(0, 0, false)), None);
    }
}

#[test]
fn the_watch_stops_a_margin_before_the_deadline() {
    // `--deadline` 30 s after the start: stopped DEADLINE_MARGIN_US short
    // of it, inside the budget.
    let deadline = 1_000 + 30_000_000;
    let stop = deadline - DEADLINE_MARGIN_US;
    let mut watch = Watch::new(1_000, Some(deadline));
    assert_eq!(watch.after(stop - 1, &line(2, 1, true)), None);
    assert_eq!(watch.after(stop, &line(2, 1, true)), Some(End::Deadline));
    // A deadline whose margin starts at or past the budget leaves the
    // budget's stop.
    for deadline in [
        STATION_BUDGET_US + DEADLINE_MARGIN_US,
        STATION_BUDGET_US + DEADLINE_MARGIN_US + 1,
        u64::MAX,
    ] {
        let mut watch = Watch::new(0, Some(deadline));
        assert_eq!(watch.after(STATION_BUDGET_US - 1, &line(0, 0, false)), None);
        assert_eq!(
            watch.after(STATION_BUDGET_US, &line(0, 0, false)),
            Some(End::Budget)
        );
    }
    // A deadline nearer than the margin, or already past: stopped at the
    // first poll.
    for deadline in [0, 1_000, 1_000 + DEADLINE_MARGIN_US - 1, DEADLINE_MARGIN_US] {
        let mut watch = Watch::new(1_000, Some(deadline));
        assert_eq!(
            watch.after(1_000, &line(0, 0, false)),
            Some(End::Deadline),
            "{deadline}"
        );
    }
    // A handshake seen by the stop still gets its flush, which the margin
    // holds (with the deauthentication's).
    let mut watch = Watch::new(0, Some(20_000_000));
    let stop = 20_000_000 - DEADLINE_MARGIN_US;
    assert_eq!(watch.after(stop, &line(2, 2, true)), None);
    assert_eq!(watch.after(stop + FLUSH_US - 1, &line(2, 2, true)), None);
    assert_eq!(
        watch.after(stop + FLUSH_US, &line(2, 2, true)),
        Some(End::Handshaken)
    );
}

#[test]
fn hide_network_names_no_ssid() {
    assert_eq!(
        hide_network("--host-check --wifi-check --wifi my%20home --deadline 600"),
        "--host-check --wifi-check --wifi <ssid> --deadline 600"
    );
    // A line without --wifi is printed as it was written.
    let text = "--host-check  --wifi-check --wifi-key x";
    assert_eq!(hide_network(text), text);
    // --wifi as the last word has no name to hide.
    assert_eq!(
        hide_network("--host-check --wifi-check\t--wifi"),
        "--host-check --wifi-check --wifi"
    );
}

#[test]
fn hide_ssid_hides_every_occurrence_of_the_name() {
    // The connection keeper's failures name the network after the radio.
    assert_eq!(
        hide_ssid("rtw89: Home Net not found", b"Home Net"),
        "rtw89: <ssid> not found"
    );
    assert_eq!(
        hide_ssid(
            "station port closed; keeper rtw89: Home Net offers no supported security: \
             Akm; Home Net",
            b"Home Net"
        ),
        "station port closed; keeper rtw89: <ssid> offers no supported security: Akm; <ssid>"
    );
    // Byte for byte: another case, or part of the name, is not the name.
    for text in ["rtw89: home net not found", "rtw89: Home Ne not found", ""] {
        assert_eq!(hide_ssid(text, b"Home Net"), text);
    }
    // A name of several-byte characters, and one that is not UTF-8, which
    // the keeper writes as from_utf8_lossy does.
    assert_eq!(
        hide_ssid("rtw89: café ü not found", "café ü".as_bytes()),
        "rtw89: <ssid> not found"
    );
    assert_eq!(
        hide_ssid("rtw89: caf\u{fffd} not found", b"caf\xe9"),
        "rtw89: <ssid> not found"
    );
    // An empty name hides nothing.
    assert_eq!(hide_ssid("rtw89:  not found", b""), "rtw89:  not found");
}

#[test]
fn hide_ssid_hides_a_name_inside_other_words_too() {
    // The line gives up legibility, never the name.
    assert_eq!(
        hide_ssid("eapol rx 2; scan rx", b"rx"),
        "eapol <ssid> 2; scan <ssid>"
    );
    assert_eq!(
        hide_ssid("scans 1, assoc", b"s"),
        "<ssid>can<ssid> 1, a<ssid><ssid>oc"
    );
    // Overlapping occurrences are hidden as one, touching ones one each.
    assert_eq!(hide_ssid("x aaa y", b"aa"), "x <ssid> y");
    assert_eq!(hide_ssid("ababa", b"aba"), "<ssid>");
    assert_eq!(hide_ssid("abab", b"ab"), "<ssid><ssid>");
}

#[test]
fn hide_ssid_leaves_a_hidden_name_hidden() {
    // A `<ssid>` already written stays as it is, so hiding twice changes
    // nothing, even for a name that is part of it or is it.
    let text = "keeper rtw89: Home Net not found; <ssid>";
    for name in ["Home Net", "s", "ssid", "id>", "<", "<ssid>"] {
        let once = hide_ssid(text, name.as_bytes());
        assert!(once.ends_with("; <ssid>"), "{name}: {once}");
        assert_eq!(hide_ssid(&once, name.as_bytes()), once, "{name}");
    }
    // A name longer than the placeholder is still hidden over it.
    assert_eq!(hide_ssid("<ssid> <ssid>x", b"<ssid>x"), "<ssid> <ssid>");
}

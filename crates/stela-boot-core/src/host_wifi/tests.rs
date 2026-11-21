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
            last: true,
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
    for (eapol, last, handshaken) in [
        ("eapol rx 2 tx 2 acked 2", true, true),
        ("eapol rx 3 tx 4 acked 0", true, true),
        ("eapol rx 2 tx 1 acked 1", true, false),
        ("eapol rx 1 tx 2 acked 2", true, false),
        ("eapol rx 2 tx 2 acked 2", false, false),
    ] {
        let line = alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), auth 1/1 status 0, assoc 1/1 \
             status 0 aid 1, {eapol}, keys -, port closed{}; tx sent 2",
            if last {
                ", last \"keys before M3b\""
            } else {
                ""
            }
        );
        assert_eq!(Progress::read(&line).handshaken(), handshaken, "{line}");
    }
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
    let mut watch = Watch::new(1_000);
    assert_eq!(watch.after(2_000, &line(0, 0, false)), None);
    assert_eq!(watch.after(3_000, &line(2, 2, false)), None);
    // Handshaken at 4 ms: polled on for FLUSH_US more, whatever the line
    // says meanwhile, then passed.
    assert_eq!(watch.after(4_000, &line(2, 2, true)), None);
    assert_eq!(watch.after(4_000 + FLUSH_US - 1, &line(2, 2, true)), None);
    assert_eq!(watch.after(4_000 + FLUSH_US - 1, &line(0, 0, false)), None);
    assert_eq!(watch.after(4_000 + FLUSH_US, &line(2, 2, true)), Some(true));
}

#[test]
fn the_watch_fails_at_the_budget_without_the_handshake() {
    let mut watch = Watch::new(1_000);
    assert_eq!(
        watch.after(1_000 + STATION_BUDGET_US - 1, &line(2, 1, true)),
        None
    );
    assert_eq!(
        watch.after(1_000 + STATION_BUDGET_US, &line(2, 1, true)),
        Some(false)
    );
    // A handshake just before the budget still gets its flush, then passes.
    let mut watch = Watch::new(0);
    assert_eq!(watch.after(STATION_BUDGET_US - 10, &line(2, 2, true)), None);
    assert_eq!(watch.after(STATION_BUDGET_US + 10, &line(2, 2, true)), None);
    assert_eq!(
        watch.after(STATION_BUDGET_US - 10 + FLUSH_US, &line(2, 2, true)),
        Some(true)
    );
    // A clock that runs backwards ends nothing early.
    let mut watch = Watch::new(5_000);
    assert_eq!(watch.after(0, &line(0, 0, false)), None);
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

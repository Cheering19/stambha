use super::*;
use alloc::string::ToString;
use core::net::Ipv4Addr;

/// What an rtw89 radio prints once M3b's handshake ran (M3 spec S14.6): its
/// keys installed, the port open and the data counts, then the chip's
/// counts after the `; `, Block Ack and PN drops among them.
const KEYED: &str = "scans 1 (12 networks, target found ch 157 -41 dBm), auth 1/1 status 0, \
     assoc 1/1 status 0 aid 4, eapol rx 2 tx 2 acked 2, keys 2, port open, rx data 9 sealed 0 \
     nokey 0 refused 1, tx data 7; tx sent 14 reported 14 acked 14 full 0, rpq skipped 0 \
     stale 0, rx fcs 0 other 40 short 2 overflow 0, held dropped 0, ba started 1 stopped 0, \
     pn dropped 1";

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
fn progress_reads_the_eapol_counts_the_keys_and_the_port() {
    let progress = Progress::read(KEYED);
    assert_eq!(
        progress,
        Progress {
            eapol: Some(Eapol {
                received: 2,
                sent: 2,
                acked: 2
            }),
            keys: Some(2),
            port_open: true,
        }
    );
    assert!(progress.keyed());
    // Nothing reached yet.
    let fresh = "scans 0 (0 networks, target -), auth -, assoc -, eapol -, keys -, \
                 port closed; tx sent 0 reported 0 acked 0 full 0";
    assert_eq!(Progress::read(fresh), Progress::default());
    assert!(!Progress::read(fresh).keyed());
}

#[test]
fn m3b_needs_both_keys_and_the_port_open() {
    for (keys, port, keyed) in [
        ("keys 2", "port open", true),
        // A rekey counts on.
        ("keys 5", "port open", true),
        ("keys 1", "port open", false),
        ("keys 0", "port open", false),
        ("keys -", "port open", false),
        ("keys 2", "port closed", false),
        ("keys x", "port open", false),
    ] {
        let line = alloc::format!(
            "scans 1 (1 networks, target found ch 1 -40 dBm), auth 1/1 status 0, assoc 1/1 \
             status 0 aid 1, eapol rx 2 tx 2 acked 2, {keys}, {port}, rx data 0 sealed 0 \
             nokey 0 refused 0, tx data 0; tx sent 2"
        );
        assert_eq!(Progress::read(&line).keyed(), keyed, "{line}");
    }
}

#[test]
fn an_sae_join_beacon_loss_and_a_refusal_read_the_same() {
    // `sae t/a` stands where `auth t/a status s` does; `beacon lost` and
    // `last` follow the data counts.
    let line = "scans 2 (9 networks, target found ch 36 -60 dBm), sae 2/2, assoc 1/1 status 0 \
                aid 2, eapol rx 4 tx 4 acked 4, keys 4, port open, rx data 3 sealed 1 nokey 0 \
                refused 0, tx data 5, beacon lost 1, last \"a key other than CCMP-128 on an \
                RTL8852C\"; tx sent 3";
    let progress = Progress::read(line);
    assert_eq!(progress.keys, Some(4));
    assert!(progress.keyed());
    assert_eq!(progress.eapol.map(|e| e.received), Some(4));
}

#[test]
fn only_the_stations_own_fields_count() {
    // The chip's part, after the first `; `, and the radio's last refusal
    // are no fields of the station's, whatever they read.
    for line in [
        "eapol rx 2 tx 2 acked 2, keys -, port closed; keys 2, port open",
        "eapol rx 2 tx 2 acked 2, keys 2, port closed; tx sent 2, port open",
        "eapol rx 2 tx 2 acked 2, keys -, port closed, last \"x, keys 2, port open\"; tx",
        "keys 2, port closed, last \"port open\"",
        "keys 2, port closed, last \"a, port open, b\"",
        // A field is read whole.
        "keys 2, port closed, report open",
        "keys 2, port opening",
    ] {
        assert!(!Progress::read(line).keyed(), "{line}");
    }
    // A malformed count reads as no EAPOL at all.
    assert_eq!(
        Progress::read("eapol rx 2 tx x acked 2, last \"a\"").eapol,
        None
    );
}

#[test]
fn the_cut_is_the_budget_without_a_near_deadline() {
    assert_eq!(STATION_BUDGET_US, 90_000_000);
    let cut = Cut::new(1_000, None);
    assert_eq!(cut.end(), End::Budget);
    assert!(!cut.reached(1_000 + STATION_BUDGET_US - 1));
    assert!(cut.reached(1_000 + STATION_BUDGET_US));
    // A clock that runs backwards ends nothing early.
    assert!(!cut.reached(0));
    // A deadline whose margin starts at or past the budget leaves the
    // budget's cut.
    for deadline in [
        STATION_BUDGET_US + DEADLINE_MARGIN_US,
        STATION_BUDGET_US + DEADLINE_MARGIN_US + 1,
        u64::MAX,
    ] {
        let cut = Cut::new(0, Some(deadline));
        assert_eq!(cut.end(), End::Budget, "{deadline}");
        assert!(!cut.reached(STATION_BUDGET_US - 1));
        assert!(cut.reached(STATION_BUDGET_US));
    }
}

#[test]
fn the_cut_is_a_margin_before_a_near_deadline() {
    // `--deadline` 30 s after the start: cut DEADLINE_MARGIN_US short of
    // it, inside the budget.
    let deadline = 1_000 + 30_000_000;
    let cut = Cut::new(1_000, Some(deadline));
    assert_eq!(cut.end(), End::Deadline);
    assert!(!cut.reached(deadline - DEADLINE_MARGIN_US - 1));
    assert!(cut.reached(deadline - DEADLINE_MARGIN_US));
    // A deadline nearer than the margin, or already past: cut at once.
    for deadline in [0, 1_000, 1_000 + DEADLINE_MARGIN_US - 1, DEADLINE_MARGIN_US] {
        let cut = Cut::new(1_000, Some(deadline));
        assert_eq!(cut.end(), End::Deadline, "{deadline}");
        assert!(cut.reached(1_000), "{deadline}");
    }
}

#[test]
fn failures_keep_the_count_the_first_and_the_last() {
    let mut failures = Failures::default();
    assert!(failures.is_empty());
    assert_eq!(alloc::format!("{failures}"), "");
    failures.note(String::from("network link has no carrier"));
    assert!(!failures.is_empty());
    assert_eq!(
        alloc::format!("{failures}"),
        "stack failed once: network link has no carrier"
    );
    failures.note(String::from("network link has no carrier"));
    assert_eq!(
        alloc::format!("{failures}"),
        "stack failed 2 times: network link has no carrier"
    );
    failures.note(String::from(
        "network link: rtw89: the AP ended the association",
    ));
    failures.note(String::from("DHCP: no IPv4 lease"));
    assert_eq!(
        alloc::format!("{failures}"),
        "stack failed 4 times, first: network link has no carrier, last: DHCP: no IPv4 lease"
    );
}

fn lease(router: Option<Ipv4Addr>) -> Outcome {
    Outcome::Leased(Lease {
        address: Ipv4Addr::new(192, 168, 1, 23),
        prefix: 24,
        router,
        after_us: 7_900_000,
    })
}

#[test]
fn a_lease_with_the_keys_and_the_port_open_passes() {
    let progress = Progress::read(KEYED);
    assert_eq!(
        ending(&lease(Some(Ipv4Addr::new(192, 168, 1, 1))), &progress),
        (
            true,
            String::from("; DHCP lease 192.168.1.23/24 router 192.168.1.1 after 7 s")
        )
    );
    assert_eq!(
        ending(&lease(None), &progress).1,
        "; DHCP lease 192.168.1.23/24 router - after 7 s"
    );
}

#[test]
fn every_other_ending_fails_and_says_why() {
    let keyed = Progress::read(KEYED);
    let closed = Progress::read(&KEYED.replace("port open", "port closed"));
    assert_eq!(
        ending(&lease(None), &closed),
        (
            false,
            String::from(
                "; DHCP lease 192.168.1.23/24 router - after 7 s, but not \"keys 2, port open\""
            )
        )
    );
    assert_eq!(
        ending(&Outcome::Cut(End::Budget), &keyed),
        (false, String::from("; no DHCP lease within 90 s"))
    );
    assert_eq!(
        ending(&Outcome::Cut(End::Deadline), &keyed),
        (
            false,
            String::from("; stopped 10 s before --deadline without a DHCP lease")
        )
    );
    assert_eq!(
        ending(
            &Outcome::Failed(String::from("invalid request: interface address capacity")),
            &keyed
        ),
        (
            false,
            String::from("; no DHCP lease: invalid request: interface address capacity")
        )
    );
}

/// The line prints an address only when it names no network beyond the
/// owner's own: a private, link-local or shared (100.64.0.0/10) one.
#[test]
fn only_local_addresses_are_shown() {
    for (octets, local) in [
        ([10, 1, 2, 3], true),
        ([172, 16, 0, 1], true),
        ([172, 31, 255, 254], true),
        ([172, 32, 0, 1], false),
        ([192, 168, 29, 1], true),
        ([192, 169, 0, 1], false),
        ([169, 254, 3, 4], true),
        ([100, 64, 0, 1], true),
        ([100, 127, 255, 254], true),
        ([100, 63, 255, 255], false),
        ([100, 128, 0, 1], false),
        ([203, 0, 113, 5], false),
        ([8, 8, 8, 8], false),
    ] {
        let address = Ipv4Addr::from(octets);
        let shown = match local {
            true => address.to_string(),
            false => String::from("<public>"),
        };
        assert_eq!(Shown(address).to_string(), shown, "{address}");
    }
}

#[test]
fn a_public_lease_and_router_read_as_public() {
    let keyed = Progress::read(KEYED);
    let leased = |address: [u8; 4], prefix, router: [u8; 4]| {
        Outcome::Leased(Lease {
            address: Ipv4Addr::from(address),
            prefix,
            router: Some(Ipv4Addr::from(router)),
            after_us: 7_900_000,
        })
    };
    assert_eq!(
        ending(&leased([203, 0, 113, 5], 24, [203, 0, 113, 1]), &keyed),
        (
            true,
            String::from("; DHCP lease <public>/24 router <public> after 7 s")
        )
    );
    assert_eq!(
        ending(&leased([100, 64, 12, 34], 10, [100, 64, 0, 1]), &keyed).1,
        "; DHCP lease 100.64.12.34/10 router 100.64.0.1 after 7 s"
    );
    assert_eq!(
        ending(&leased([10, 0, 0, 9], 8, [8, 8, 4, 4]), &keyed).1,
        "; DHCP lease 10.0.0.9/8 router <public> after 7 s"
    );
}

/// The network stack's DHCP account (`Stack::dhcp_summary`) names the
/// lease's server and router: each reads as [`Shown`].
#[test]
fn the_dhcp_account_shows_only_a_local_server_and_router() {
    let account = "sent discover=1 request=1; replies offer=1 ack=1 nak=0 other=0 (broadcast=2 \
                   unicast=0) for_other_clients=0; not taken: checksum=0 xid=0 no_server_id=0 \
                   your_ip=0 no_mask=0; broadcast_frames=2 vlan_tagged=0; lease=3600s \
                   server=203.0.113.1 mask=255.255.255.0 router=192.168.1.1";
    assert_eq!(
        hide_public(account),
        account.replace("server=203.0.113.1", "server=<public>")
    );
    assert_eq!(
        hide_public("lease=3600s server=10.0.0.1 mask=255.0.0.0 router=8.8.8.8"),
        "lease=3600s server=10.0.0.1 mask=255.0.0.0 router=<public>"
    );
    for kept in [
        "lease=unseen",
        "lease=none server=none mask=none router=none",
        "server=<public> router=<public>",
        "server=",
        "router=1.2.3",
    ] {
        assert_eq!(hide_public(kept), kept);
    }
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

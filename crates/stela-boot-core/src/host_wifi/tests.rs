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

/// Every IPv4 address in free text reads as [`Shown`] writes it: a public
/// one, the network check's server among them, as `<public>`.
#[test]
fn public_addresses_in_text_read_as_public() {
    assert_eq!(
        hide_public_addresses(
            "http://93.184.215.14/ via 192.168.29.1:80, 10.0.0.2 and 8.8.8.8. (100.64.0.9)"
        ),
        "http://<public>/ via 192.168.29.1:80, 10.0.0.2 and <public>. (100.64.0.9)"
    );
    // At either end, after a letter, and back to back with other text.
    assert_eq!(
        hide_public_addresses("203.0.113.5v1.2.3.4x169.254.1.1"),
        "<public>v<public>x169.254.1.1"
    );
    // No address: too few or too many parts, an octet past 255, a lone
    // number, nothing; and text around it is kept, multi-byte included.
    for text in [
        "after 7 s",
        "1.2.3",
        "1.2.3.4.5",
        "1.2.3.256",
        "...",
        "",
        "é \"x\" 12",
    ] {
        assert_eq!(hide_public_addresses(text), text);
    }
    assert_eq!(hide_public_addresses("é8.8.4.4ü"), "é<public>ü");
}

const URL: &str = "http://example.com/";

fn ended(result: Result<Vec<u8>, NetError>, after_us: u64) -> Post {
    Post::Ended { result, after_us }
}

/// Any HTTP answer proves the path end to end, as the normal boot's
/// `--network-check` takes it: a 2xx body, by its length and SHA-256 as the
/// normal boot prints it, or any other status.
#[test]
fn any_answer_passes_the_network_check() {
    use sha2::Digest;
    let body = b"<!doctype html><title>Example Domain</title>".to_vec();
    let digest = alloc::format!("{:x}", sha2::Sha256::digest(&body));
    assert_eq!(
        network_check(URL, Some(&ended(Ok(body.clone()), 412_900)), b"test"),
        (
            true,
            alloc::format!(
                "POST http://example.com/ answered 2xx in 412 ms: body bytes={} sha256={digest}",
                body.len()
            )
        )
    );
    assert_eq!(
        network_check(URL, Some(&ended(Ok(Vec::new()), 0)), b"test").1,
        "POST http://example.com/ answered 2xx in 0 ms: body bytes=0 \
         sha256=e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    );
    for status in [100, 404, 405, 500, 503] {
        assert_eq!(
            network_check(
                URL,
                Some(&ended(Err(NetError::Status(status)), 380_000)),
                b"test"
            ),
            (
                true,
                alloc::format!(
                    "POST http://example.com/ answered HTTP {status} in 380 ms, which proves the \
                     path"
                )
            )
        );
    }
}

/// Only a transport failure fails it: the stack's own words, which say
/// where it ended (DNS, the connection, TLS, the reply).
#[test]
fn only_a_transport_failure_fails_the_network_check() {
    use stela_runtime::net::TlsUnavailable;
    for error in [
        NetError::Dns(String::from("example.com: no address")),
        NetError::Dns(String::from(
            "no DNS server from DHCPv4 or IPv6 router advertisements",
        )),
        NetError::Timeout("DNS"),
        NetError::Reset,
        NetError::Timeout("TCP transfer"),
        NetError::Timeout("HTTP request"),
        NetError::Http(String::from("response header too large")),
        NetError::Truncated("HTTP response body"),
        NetError::Tls(String::from("InvalidCertificate(Expired)")),
        NetError::TlsUnavailable(TlsUnavailable::CorrelatedClock),
        NetError::ProxyAuthentication,
        NetError::NetworkAuthentication,
        NetError::NoCarrier,
    ] {
        let words = error.to_string();
        assert_eq!(
            network_check(URL, Some(&ended(Err(error), 15_000_999)), b"test"),
            (
                false,
                alloc::format!("POST http://example.com/ failed after 15000 ms: {words}")
            ),
            "{words}"
        );
    }
}

/// The station check's cut ends the POST as it ends the wait for a lease,
/// and without a lease nothing is posted.
#[test]
fn a_cut_or_no_lease_fails_the_network_check() {
    assert_eq!(
        network_check(URL, Some(&Post::Cut(End::Budget)), b"test"),
        (
            false,
            String::from("POST http://example.com/: no answer within the station check's 90 s")
        )
    );
    assert_eq!(
        network_check(URL, Some(&Post::Cut(End::Deadline)), b"test"),
        (
            false,
            String::from(
                "POST http://example.com/: stopped 10 s before --deadline without an answer"
            )
        )
    );
    assert_eq!(
        network_check(URL, None, b"test"),
        (
            false,
            String::from("not run: the station check got no DHCP lease (see CHECK wifi)")
        )
    );
}

/// The line names no network, no station and no public address: a keeper
/// failure that reaches the POST as the link's names the network and the
/// AP, and the server's address may be public.
#[test]
fn the_network_check_names_no_network_station_or_public_address() {
    let error = NetError::Link(String::from(
        "rtw89: test is gone: deauthenticated by 02:03:04:05:06:07 (reason 3)",
    ));
    assert_eq!(
        network_check(
            "http://203.0.113.7/p",
            Some(&ended(Err(error), 2_000)),
            b"test"
        )
        .1,
        "POST http://<public>/p failed after 2 ms: network link: rtw89: <ssid> is gone: \
         deauthenticated by xx:xx:xx:xx:xx:xx (reason 3)"
    );
    // A server in the owner's own network is shown.
    assert_eq!(
        network_check(
            "http://192.168.1.10:8080/",
            Some(&ended(Err(NetError::Status(404)), 3_000)),
            b"test"
        )
        .1,
        "POST http://192.168.1.10:8080/ answered HTTP 404 in 3 ms, which proves the path"
    );
    // A redirect's or a header's words are the server's, hidden the same.
    assert_eq!(
        network_check(
            URL,
            Some(&ended(
                Err(NetError::Http(String::from(
                    "transfer coding \"x 198.51.100.4\""
                ))),
                1_000
            )),
            b"test"
        )
        .1,
        "POST http://example.com/ failed after 1 ms: HTTP: transfer coding \"x <public>\""
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

/// The check hides the name in its own line as a normal boot hides it in a
/// Wi-Fi adapter's (`network_link::hide_ssid`, tested there in full), and
/// waits for a lease as the stack does.
#[test]
fn the_check_hides_the_name_and_waits_as_a_normal_boot() {
    assert_eq!(
        hide_ssid(
            "station port closed; keeper rtw89: Home Net offers no supported security: \
             Akm; Home Net",
            b"Home Net"
        ),
        "station port closed; keeper rtw89: <ssid> offers no supported security: Akm; <ssid>"
    );
    assert_eq!(DHCP_WAIT_US, 60_000_000);
    assert_eq!(STATION_BUDGET_US, WIFI_CARRIER_WAIT_US + DHCP_WAIT_US);
}

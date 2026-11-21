use super::*;

#[test]
fn audio_hpet_handoff_requires_an_audio_diagnostic() {
    assert_eq!(
        System::parse("--system --audio-own-hpet").unwrap_err(),
        Error::Constraint(Constraint::AudioHpetDependency)
    );
}
#[test]
fn native_pci_ioapic_handoff_round_trips_and_refuses_diagnostic_paths() {
    let text = "--system --native-pci-own-ioapic";
    let options = System::parse(text).unwrap();
    assert!(options.native_pci_own_ioapic);
    assert_eq!(System::parse(&options.to_string()), Ok(options));
    assert_eq!(
        System::parse("--system --native-pci-own-ioapic --native-pci-own-ioapic").unwrap_err(),
        Error::DuplicateNativePciIoapic
    );
    assert_eq!(
        System::parse("--system --native-pci-own-ioapic --audio-check duplex").unwrap_err(),
        Error::Constraint(Constraint::NativePciIoapicExclusive)
    );
}

use alloc::{format, string::ToString, vec};

#[test]
fn diagnostic_options_are_typed() {
    for text in ["--system", "--system --wifi --audio-own-hpet"] {
        assert_eq!(System::parse(text).unwrap().diagnostic_option(), None);
    }
    for (text, flag) in [
        (
            "--system --native-cores 2 --native-bootstrap-check",
            "--native-bootstrap-check",
        ),
        (
            "--system --native-cores 2 --native-shared-check",
            "--native-shared-check",
        ),
    ] {
        assert_eq!(System::parse(text).unwrap().diagnostic_option(), Some(flag));
    }
}

#[test]
fn host_check_lines_round_trip_and_refuse_bad_values() {
    let line = "--host-check \
                --nor-file 11111111-2222-3333-4444-555555555555:/stela/nor.bin --deadline 30 \
                --key-seconds 5";
    let options = HostCheck::parse(line).unwrap();
    assert_eq!(options.deadline, Some(30));
    assert_eq!(HostCheck::parse(&options.to_string()), Ok(options));
    assert_eq!(HostCheck::parse("--host-check"), Ok(HostCheck::default()));
    for text in [
        "--system",
        "--host-check --native-dma-check",
        "--host-check --deadline 0",
        "--host-check --nor-file x",
        "--host-check --unknown",
        "--host-check --wifi-check=1",
        "--host-check --keep-lines=1",
        "--host-check --keep-lines 1",
    ] {
        assert!(HostCheck::parse(text).is_err(), "{text}");
    }
}

#[test]
fn host_check_wifi_check_is_off_unless_named_and_round_trips() {
    assert!(!HostCheck::parse("--host-check").unwrap().wifi_check);
    assert!(
        !HostCheck::parse("--host-check --deadline 30")
            .unwrap()
            .wifi_check
    );
    let options = HostCheck::parse("--host-check --wifi-check --deadline 600").unwrap();
    assert!(options.wifi_check);
    assert_eq!(options.deadline, Some(600));
    assert_eq!(
        options.to_string(),
        "--host-check --deadline 600 --wifi-check"
    );
    assert_eq!(HostCheck::parse(&options.to_string()), Ok(options));
    // Repeated, it stays on, as the other flags' repeats overwrite.
    assert!(
        HostCheck::parse("--host-check --wifi-check --wifi-check")
            .unwrap()
            .wifi_check
    );
    assert_eq!(
        HostCheck::parse("--host-check --wifi-check").unwrap(),
        HostCheck {
            wifi_check: true,
            ..HostCheck::default()
        }
    );
    // Without it, the line prints as before.
    assert_eq!(
        HostCheck::parse("--host-check --key-seconds 5")
            .unwrap()
            .to_string(),
        "--host-check --key-seconds 5"
    );
}

#[test]
fn host_check_keep_lines_is_off_unless_named_and_round_trips() {
    assert!(!HostCheck::parse("--host-check").unwrap().keep_lines);
    assert!(
        !HostCheck::parse("--host-check --wifi-check --deadline 30")
            .unwrap()
            .keep_lines
    );
    let options =
        HostCheck::parse("--host-check --keep-lines --wifi-check --deadline 600").unwrap();
    assert!(options.keep_lines);
    assert!(options.wifi_check);
    assert_eq!(options.deadline, Some(600));
    assert_eq!(
        options.to_string(),
        "--host-check --deadline 600 --wifi-check --keep-lines"
    );
    assert_eq!(HostCheck::parse(&options.to_string()), Ok(options));
    assert_eq!(
        HostCheck::parse("--host-check --keep-lines --keep-lines").unwrap(),
        HostCheck {
            keep_lines: true,
            ..HostCheck::default()
        }
    );
}

#[test]
fn host_check_parses_wifi() {
    let options = HostCheck::parse("--host-check --wifi-check --wifi home").unwrap();
    assert!(options.wifi_check);
    assert_eq!(options.ssid.as_deref(), Some("home"));
    assert_eq!(options.wifi_key.as_deref(), Some(DEFAULT_WIFI_KEY));
    assert_eq!(options.wifi_country, None);
    // The default key path is not printed.
    assert_eq!(options.to_string(), "--host-check --wifi-check --wifi home");
    assert_eq!(HostCheck::parse(&options.to_string()), Ok(options));
    // System's word rules: `%XX` escapes, a key path, a country in any case.
    let options = HostCheck::parse(
        "--host-check --wifi-country in --wifi my%20net --keep-lines --wifi-check \
         --wifi-key \\EFI\\k.key --deadline 600",
    )
    .unwrap();
    assert_eq!(options.ssid.as_deref(), Some("my net"));
    assert_eq!(options.wifi_key.as_deref(), Some("\\EFI\\k.key"));
    assert_eq!(options.wifi_country.as_deref(), Some("IN"));
    assert_eq!(
        options.to_string(),
        "--host-check --deadline 600 --wifi-check --wifi my%20net --wifi-key \\EFI\\k.key \
         --wifi-country IN --keep-lines"
    );
    assert_eq!(HostCheck::parse(&options.to_string()), Ok(options));
    // Without --wifi nothing of it is set, as before.
    let options = HostCheck::parse("--host-check --wifi-check").unwrap();
    assert_eq!(
        (options.ssid, options.wifi_key, options.wifi_country),
        (None, None, None)
    );
    for (text, error) in [
        (
            "--host-check --wifi-check --wifi",
            Error::MissingWifiNetwork,
        ),
        ("--host-check --wifi-check --wifi %", Error::WifiNetwork),
        (
            "--host-check --wifi-check --wifi a --wifi b",
            Error::DuplicateWifi,
        ),
        (
            "--host-check --wifi-check --wifi a --wifi-key",
            Error::MissingWifiKey,
        ),
        (
            "--host-check --wifi-check --wifi a --wifi-key k --wifi-key k",
            Error::DuplicateWifiKey,
        ),
        (
            "--host-check --wifi-check --wifi a --wifi-country USA",
            Error::WifiCountry,
        ),
        (
            "--host-check --wifi-check --wifi a --wifi-country U1",
            Error::WifiCountry,
        ),
        (
            "--host-check --wifi-check --wifi a --wifi-country US --wifi-country US",
            Error::DuplicateWifiCountry,
        ),
    ] {
        assert_eq!(HostCheck::parse(text), Err(error), "{text}");
    }
}

#[test]
fn wifi_without_wifi_check_is_refused() {
    for text in [
        "--host-check --wifi home",
        "--host-check --keep-lines --wifi home --wifi-key k",
    ] {
        assert_eq!(
            HostCheck::parse(text),
            Err(Error::Constraint(Constraint::HostWifiDependency)),
            "{text}"
        );
    }
}

#[test]
fn wifi_key_without_wifi_is_refused() {
    for text in [
        "--host-check --wifi-check --wifi-key k",
        "--host-check --wifi-check --wifi-country US",
        "--host-check --wifi-key k",
    ] {
        assert_eq!(
            HostCheck::parse(text),
            Err(Error::Constraint(Constraint::WifiDependency)),
            "{text}"
        );
    }
}

#[test]
fn host_check_network_check_posts_over_the_wifi_network() {
    let options = HostCheck::parse(
        "--host-check --network-check http://example.com/ --wifi-check --keep-lines --wifi home \
         --deadline 600 --wifi-country IN",
    )
    .unwrap();
    assert_eq!(
        options.network_check.as_deref(),
        Some("http://example.com/")
    );
    assert_eq!(
        options.to_string(),
        "--host-check --deadline 600 --wifi-check --wifi home --wifi-country IN \
         --network-check http://example.com/ --keep-lines"
    );
    assert_eq!(HostCheck::parse(&options.to_string()), Ok(options));
    // https too, with the normal boot's URL rules: the value is kept raw.
    let raw = "https://H:443/a/../p%2f?sig=%2B%3d";
    let options = HostCheck::parse(&format!(
        "--host-check --wifi-check --wifi a --network-check {raw}"
    ))
    .unwrap();
    assert_eq!(options.network_check.as_deref(), Some(raw));
    assert_eq!(HostCheck::parse(&options.to_string()), Ok(options));
    // Without it nothing is posted, and the line prints as before.
    let options = HostCheck::parse("--host-check --wifi-check --wifi a").unwrap();
    assert_eq!(options.network_check, None);
    assert_eq!(options.to_string(), "--host-check --wifi-check --wifi a");
    for (text, error) in [
        (
            "--host-check --wifi-check --wifi a --network-check",
            Error::MissingNetworkUrl,
        ),
        (
            "--host-check --wifi-check --wifi a --network-check ftp://h/p",
            Error::UrlScheme,
        ),
        (
            "--host-check --wifi-check --wifi a --network-check http://h",
            Error::UrlPath,
        ),
        (
            "--host-check --wifi-check --wifi a --network-check http:///p",
            Error::UrlHost,
        ),
        (
            "--host-check --wifi-check --wifi a --network-check http://u:p@h/p",
            Error::Url(stela_runtime::net::http::UrlError::Credentials),
        ),
        (
            "--host-check --wifi-check --wifi a --network-check http://h/p \
             --network-check https://h/p",
            Error::DuplicateNetworkCheck,
        ),
    ] {
        assert_eq!(HostCheck::parse(text), Err(error), "{text}");
    }
}

#[test]
fn network_check_without_wifi_is_refused() {
    for text in [
        "--host-check --network-check http://h/p",
        "--host-check --wifi-check --network-check http://h/p",
        "--host-check --wifi-check --keep-lines --network-check https://h/p --deadline 600",
    ] {
        assert_eq!(
            HostCheck::parse(text),
            Err(Error::Constraint(Constraint::HostNetworkCheckDependency)),
            "{text}"
        );
    }
    // --wifi without --wifi-check is refused as before, first.
    assert_eq!(
        HostCheck::parse("--host-check --wifi a --network-check http://h/p"),
        Err(Error::Constraint(Constraint::HostWifiDependency))
    );
    assert_eq!(
        Constraint::HostNetworkCheckDependency.message(),
        "host check: --network-check posts over the network of --wifi"
    );
}

const DISK: &str = "74EXN0LI:11111111-2222-3333-4444-555555555555";

fn round_trip(text: &str) -> System {
    let options = System::parse(text).unwrap();
    let line = options.to_string();
    assert_eq!(System::parse(&line), Ok(options.clone()), "{line}");
    options
}

#[test]
fn single_user_needs_a_preboot_selection() {
    let selected = round_trip(&format!("--system --preboot {DISK} {SINGLE_USER}"));
    assert!(selected.single_user);
    assert!(System::parse(&format!("--system {SINGLE_USER}")).is_err());
}

#[test]
fn a_bundle_repair_is_a_preboot_macos_boot() {
    use stela_loader::boot_selection::BUNDLE_REPAIR_OPTION;
    let repair = round_trip(&format!("--system --preboot {DISK} {BUNDLE_REPAIR_OPTION}"));
    assert!(repair.bundle_repair);
    assert!(!round_trip(&format!("--system --preboot {DISK}")).bundle_repair);
    assert_eq!(
        System::parse(&format!("--system {BUNDLE_REPAIR_OPTION}")),
        Err(Error::Constraint(Constraint::PrebootSelection))
    );
    assert_eq!(
        System::parse(&format!(
            "--system --preboot {DISK} {BUNDLE_REPAIR_OPTION} --upgrade"
        )),
        Err(Error::Constraint(Constraint::BundleRepairSelection))
    );
}

#[test]
fn no_boot_that_starts_metal_clients_sees_the_agx_model_and_restore_ramdisks_keep_it() {
    use stela_loader::boot_selection::{
        BUNDLE_REPAIR_OPTION, PAIRED_RECOVERY_OPTION, SYSTEM_RECOVERY_OPTION, UPGRADE_OPTION,
    };
    // The installed macOS, and a recoveryOS, whose WindowServer composites
    // in software without a Metal device: no Metal device rather than the
    // model, whose work never completes.
    for flag in [
        "",
        SINGLE_USER,
        BUNDLE_REPAIR_OPTION,
        PAIRED_RECOVERY_OPTION,
        SYSTEM_RECOVERY_OPTION,
    ] {
        let system = round_trip(&format!("--system --preboot {DISK} {flag}"));
        assert!(system.starts_metal_clients(), "{flag}");
        assert!(system.hides_agx(false), "{flag}");
        assert!(system.hides_agx(true), "{flag}");
    }
    // The installer's restore and a staged update run the restore ramdisk,
    // open no host GPU and keep the model.
    for line in [
        String::from("--system"),
        format!("--system --preboot {DISK} {UPGRADE_OPTION}"),
    ] {
        let ramdisk = round_trip(&line);
        assert!(!ramdisk.starts_metal_clients(), "{line}");
        assert!(!ramdisk.hides_agx(false), "{line}");
        assert!(ramdisk.hides_agx(true), "{line}");
    }
}

#[test]
fn scanout_mirror_round_trips() {
    assert!(!round_trip("--system").scanout_mirror);
    assert!(round_trip(&format!("--system {SCANOUT_MIRROR}")).scanout_mirror);
}

#[test]
fn maintenance_writes_require_an_explicit_preboot_selection() {
    let ordinary = round_trip(&format!("--system --preboot {DISK}"));
    assert!(!ordinary.maintenance_write);
    assert!(!ordinary.to_string().contains(MAINTENANCE_WRITE));
    let maintenance = round_trip(&format!("--system --preboot {DISK} {MAINTENANCE_WRITE}"));
    assert!(maintenance.maintenance_write);
    assert_eq!(
        System::parse(&format!("--system {MAINTENANCE_WRITE}")),
        Err(Error::Constraint(Constraint::PrebootSelection))
    );
}

#[test]
fn setup_lines_parse_and_serialize_identically() {
    let preboot = format!(
        "--system --storage {DISK} --nor {DISK} --preboot {DISK} --ecid 0x0000000000001234 \
         --external-disks --guest-network"
    );
    let options = round_trip(&preboot);
    assert!(options.bundle && options.external_disks);
    assert_eq!(options.ecid, Some(0x1234));
    assert_eq!(options.to_string(), preboot);
    let bundle = format!("--system --storage {DISK} --nor {DISK} --bundle \\EFI\\stela\\r");
    assert_eq!(round_trip(&bundle).to_string(), bundle);
}

#[test]
fn chosen_disk_round_trips_and_the_mount_refuses_it_unreplaced() {
    let line = "--system --storage @disk --nor @disk --preboot @disk";
    let options = round_trip(line);
    assert!(options.storage.as_ref().is_some_and(is_chosen));
    assert_eq!(options.to_string(), line);
    assert!(System::parse_resolved(line).is_err());
    let resolved = line.replace("@disk", DISK);
    assert!(System::parse_resolved(&resolved).is_ok());
}

#[test]
fn every_option_survives_a_round_trip() {
    let options = round_trip(&format!(
        "--system --storage {DISK} --nor {DISK} --bundle b --install p.json \
         --install-source 11111111-2222-3333-4444-555555555555 --verify-on-read \
         --usb-storage A --usb-storage B --input --idle-skip --sep-observe --native-cores 3 \
         --native-artifacts --serialized --profile --monitor-trace --display-trace \
         --network-check http://h/p --audio-check transport-duplex --audio-own-hpet --dram-base 0x10000000000 \
         --load 0x10000004000 --entry 0x10000008000 --x0 7 --budget 99 --break 0x10 \
         --trace 0x20 --watch 0x30,0x40 --dump x21-0x8,0x10 --dump 0x50,0x60"
    ));
    assert_eq!(options.usb_storage, ["A", "B"]);
    assert_eq!(options.audio_check, Some(AudioCheck::TransportDuplex));
    assert!(options.audio_own_hpet);
    assert!(options.display_trace);
    assert!(!System::default().display_trace);
    assert!(!options.parallel);
    assert_eq!(options.dumps[0], (Base::Register(21, -8), 0x10));
    let restore = round_trip(&format!(
        "--system --bundle b --storage {DISK} --restore-study --restore-source {DISK} \
         --restore-manifest m.plist --restore-identity 2"
    ));
    assert_eq!(restore.restore_manifest_path.as_deref(), Some("m.plist"));
}

#[test]
fn usb4_approvals_repeat_and_keep_their_canonical_spelling() {
    let first = "446ab8f0-5aff-47c9-8962-5870641e4d61";
    let second = "e254fadb-773c-43c3-b00d-4170323c36f8";
    let options = round_trip(&format!(
        "--system --usb4-approve {} --usb4-approve {second}",
        first.to_uppercase()
    ));
    assert_eq!(options.usb4_approved.len(), 2);
    assert_eq!(
        options.to_string(),
        format!("--system --usb4-approve {first} --usb4-approve {second}")
    );
    for (text, error) in [
        ("--system --usb4-approve", Error::Usb4Approve),
        ("--system --usb4-approve nonsense", Error::Usb4Approve),
    ] {
        assert_eq!(System::parse(text).err(), Some(error), "{text}");
    }
    assert_eq!(
        System::parse(&format!(
            "--system --usb4-approve {first} --usb4-approve {first}"
        ))
        .err(),
        Some(Error::DuplicateUsb4Approve)
    );
}

#[test]
fn wifi_names_escape_spaces_and_the_key_path_defaults() {
    let options = round_trip("--system --wifi Home%20Net%25 --wifi-country gb");
    assert_eq!(options.ssid.as_deref(), Some("Home Net%"));
    assert_eq!(options.wifi_key.as_deref(), Some(DEFAULT_WIFI_KEY));
    assert_eq!(options.wifi_country.as_deref(), Some("GB"));
    assert_eq!(
        options.to_string(),
        "--system --wifi Home%20Net%25 --wifi-country GB"
    );
    let keyed = round_trip("--system --wifi caf\u{e9} --wifi-key \\EFI\\k");
    assert_eq!(keyed.ssid.as_deref(), Some("caf\u{e9}"));
    assert_eq!(keyed.wifi_key.as_deref(), Some("\\EFI\\k"));
    for text in [
        "--system --wifi-key \\k",
        "--system --wifi-country GB",
        "--system --wifi %",
        "--system --wifi %4",
        "--system --wifi %FF",
        "--system --wifi 0123456789abcdef0123456789abcdef0",
        "--system --wifi a --wifi b",
        "--system --wifi a --wifi-country GBR",
        "--system --wifi a --wifi-country 12",
    ] {
        assert!(System::parse(text).is_err(), "{text}");
    }
}

#[test]
fn static_ipv4_and_dns_round_trip_and_stay_in_their_subnet() {
    let text = "--system --ipv4 192.168.1.20/24 --ipv4-gateway 192.168.1.1 --dns 1.1.1.1,2606:4700:4700::1111";
    let options = round_trip(text);
    assert_eq!(
        options.ipv4,
        Some(StaticIpv4 {
            address: core::net::Ipv4Addr::new(192, 168, 1, 20),
            prefix: 24
        })
    );
    assert_eq!(options.dns.len(), 2);
    assert_eq!(options.to_string(), text);
    assert_eq!(
        round_trip("--system --ipv4 10.0.0.1/31").to_string(),
        "--system --ipv4 10.0.0.1/31"
    );
    for text in [
        "--system --ipv4 192.168.1.20",
        "--system --ipv4 192.168.1.20/33",
        "--system --ipv4 192.168.1.0/24",
        "--system --ipv4 192.168.1.255/24",
        "--system --ipv4 224.0.0.1/4",
        "--system --ipv4 10.0.0.2/8 --ipv4 10.0.0.3/8",
        "--system --ipv4-gateway 192.168.1.1",
        "--system --ipv4 192.168.1.20/24 --ipv4-gateway 192.168.2.1",
        "--system --ipv4 192.168.1.20/24 --ipv4-gateway 192.168.1.20",
        "--system --dns 1.1.1.1,1.1.1.1",
        "--system --dns 1.1.1.1,1.0.0.1,9.9.9.9,8.8.8.8",
        "--system --dns 0.0.0.0",
        "--system --dns 1.1.1.1 --dns 9.9.9.9",
    ] {
        assert!(System::parse(text).is_err(), "{text}");
    }
}

#[test]
fn native_checks_imply_serialized_execution_without_writing_it() {
    let options = round_trip("--system --native-cores 2 --native-rounds");
    assert!(!options.parallel);
    assert!(!options.to_string().contains("--serialized"));
    assert!(System::parse("--system").unwrap().parallel);
}

#[test]
fn native_core_counts_refuse_duplicates_zero_and_over_capacity() {
    for text in [
        "--system --native-cores 1 --native-cores 1",
        "--system --native-cores 0",
        "--system --native-cores 65",
        "--system --native-cores x",
        "--system --native-cores",
    ] {
        assert!(System::parse(text).is_err(), "{text}");
    }
    assert_eq!(
        System::parse("--system --native-cores 64")
            .unwrap()
            .native_cores,
        Some(64)
    );
}

#[test]
fn guest_pes_bound_the_executors_that_run_them() {
    for text in [
        "--system --guest-pes",
        "--system --guest-pes 0",
        "--system --guest-pes 8 --guest-pes 8",
    ] {
        assert!(System::parse(text).is_err(), "{text}");
    }
    let options = System::parse(&format!("--system --preboot {DISK} --guest-pes 8")).unwrap();
    assert_eq!(options.guest_pes, Some(8));
    assert!(options.to_string().contains("--guest-pes 8"));
    assert_eq!(options.assigned_native_cores(64, None), Ok(8));
    assert_eq!(options.assigned_native_cores(2, None), Ok(2));
    assert_eq!(options.assigned_native_cores(64, Some(4)), Ok(4));
}

#[test]
fn preboot_checks_the_live_count_before_starting_aps() {
    let preboot = format!("--system --preboot {DISK} --guest-pes 8");
    let options = System::parse(&preboot).unwrap();
    assert_eq!(options.validate_native_cores(64, 8, 8), Ok(()));
    assert_eq!(options.validate_native_cores(2, 2, 8), Ok(()));
    assert!(options.validate_native_cores(64, 8, 6).is_err());
    assert!(options.validate_native_cores(64, 8, 10).is_err());
    assert!(options.validate_native_cores(2, 2, 10).is_err());
    let options = System::parse(&format!("{preboot} --native-cores 4")).unwrap();
    assert_eq!(options.validate_native_cores(64, 4, 8), Ok(()));
    assert!(options.validate_native_cores(64, 4, 2).is_err());
    let options = System::parse(&format!("{preboot} --serialized")).unwrap();
    assert_eq!(options.validate_native_cores(64, 1, 10), Ok(()));
}

#[test]
fn exact_roster_distinguishes_verified_disk_authored_and_diagnostic_sources() {
    let assigned = |options: &str, available, prepared| {
        System::parse(options)
            .unwrap()
            .assigned_native_cores(available, prepared)
    };
    let preboot = format!("--system --preboot {DISK}");
    assert!(assigned(&preboot, 64, None).is_err());
    assert_eq!(
        assigned(&format!("{preboot} --native-cores 3"), 64, None),
        Ok(3)
    );
    assert_eq!(
        assigned(&format!("{preboot} --serialized"), 64, None),
        Ok(1)
    );
    assert_eq!(assigned("--system", 64, None), Ok(1));
    assert_eq!(assigned("--system", 64, Some(8)), Ok(8));
    assert_eq!(assigned("--system --native-cores 3", 64, Some(8)), Ok(3));
    assert!(assigned("--system --native-cores 9", 64, Some(8)).is_err());
    assert!(assigned("--system --native-cores 3", 2, Some(8)).is_err());
    assert!(assigned("--system --native-cores 3", 64, None).is_err());
    assert_eq!(assigned("--system --serialized", 64, Some(8)), Ok(1));
    assert!(assigned("--system --serialized --native-cores 4", 64, Some(8)).is_err());
    assert_eq!(
        assigned("--system --native-shared-check --native-cores 4", 64, None),
        Ok(4)
    );
    assert!(assigned("--system --native-bootstrap-check", 64, None).is_err());
}

#[test]
fn invalid_lines_refuse() {
    for text in [
        "--storage x",
        "--system --unknown 1",
        "--system --install p.json",
        "--system --ecid 0",
        "--system --safe-boot",
        "--system --network-check ftp://h/p",
        "--system --dram-base 0x1000",
        "--system --storage-ram 1 --storage-ram 2",
    ] {
        assert!(System::parse(text).is_err(), "{text}");
    }
}

#[test]
fn numbers_take_decimal_and_separated_hex() {
    assert_eq!(number("0x1_0000"), Ok(0x10000));
    assert_eq!(number("42"), Ok(42));
    assert!(number("0xg").is_err());
    assert_eq!(
        vec![Base::parse("x3+0x10").unwrap()],
        [Base::Register(3, 16)]
    );
}

mod constraints;
mod grammar;

#[test]
fn host_check_parses_throughput() {
    // M4 spec S3.6: a GET of --throughput-bytes over the --wifi network,
    // optionally a POST of as many to --upload-check.
    let options = HostCheck::parse(
        "--host-check --wifi-check --wifi home --throughput-check http://x/f \
         --throughput-bytes 1048576",
    )
    .unwrap();
    assert_eq!(options.throughput_check.as_deref(), Some("http://x/f"));
    assert_eq!(options.throughput_bytes, 1_048_576);
    assert_eq!(options.upload_check, None);
    assert_eq!(
        options.to_string(),
        "--host-check --wifi-check --wifi home --throughput-check http://x/f \
         --throughput-bytes 1048576"
    );
    assert_eq!(HostCheck::parse(&options.to_string()), Ok(options));
    // 50 MiB by default, which the line leaves out; an upload URL, in any
    // order, with the normal boot's URL rules.
    let options = HostCheck::parse(
        "--host-check --upload-check https://u/up --wifi-check --throughput-check \
         https://d/down --wifi a --keep-lines",
    )
    .unwrap();
    assert_eq!(options.throughput_bytes, DEFAULT_THROUGHPUT_BYTES);
    assert_eq!(DEFAULT_THROUGHPUT_BYTES, 52_428_800);
    assert_eq!(options.upload_check.as_deref(), Some("https://u/up"));
    assert_eq!(
        options.to_string(),
        "--host-check --wifi-check --wifi a --throughput-check https://d/down \
         --upload-check https://u/up --keep-lines"
    );
    assert_eq!(HostCheck::parse(&options.to_string()), Ok(options));
    // The largest size is 2^32 bytes.
    let options = HostCheck::parse(
        "--host-check --wifi-check --wifi a --throughput-check http://x/f \
         --throughput-bytes 4294967296",
    )
    .unwrap();
    assert_eq!(options.throughput_bytes, 1 << 32);
    let base = "--host-check --wifi-check --wifi a --throughput-check http://x/f";
    for (extra, error) in [
        (
            " --throughput-check http://y/g",
            Error::DuplicateThroughputCheck,
        ),
        (
            " --throughput-bytes 1 --throughput-bytes 2",
            Error::DuplicateThroughputBytes,
        ),
        (
            " --upload-check http://u/p --upload-check http://u/q",
            Error::DuplicateUploadCheck,
        ),
        (" --throughput-bytes", Error::ThroughputBytes),
        (" --throughput-bytes 4294967297", Error::ThroughputBytes),
        (" --throughput-bytes -1", Error::ThroughputBytes),
        (" --throughput-bytes 1k", Error::ThroughputBytes),
        (" --upload-check", Error::MissingUploadUrl),
        (" --upload-check ftp://u/p", Error::UrlScheme),
    ] {
        let text = format!("{base}{extra}");
        assert_eq!(HostCheck::parse(&text), Err(error), "{text}");
    }
    assert_eq!(
        HostCheck::parse("--host-check --wifi-check --wifi a --throughput-check"),
        Err(Error::MissingThroughputUrl)
    );
    assert_eq!(
        HostCheck::parse("--host-check --wifi-check --wifi a --throughput-check http://h"),
        Err(Error::UrlPath)
    );
}

#[test]
fn throughput_without_wifi_is_refused() {
    for text in [
        "--host-check --throughput-check http://h/p",
        "--host-check --wifi-check --throughput-check http://h/p",
        "--host-check --wifi-check --throughput-check http://h/p --upload-check http://h/u",
    ] {
        assert_eq!(
            HostCheck::parse(text),
            Err(Error::Constraint(Constraint::HostThroughputDependency)),
            "{text}"
        );
    }
    // The size and the upload measure the --throughput-check GET's network.
    for text in [
        "--host-check --wifi-check --wifi a --throughput-bytes 10",
        "--host-check --wifi-check --wifi a --upload-check http://h/u",
    ] {
        assert_eq!(
            HostCheck::parse(text),
            Err(Error::Constraint(Constraint::HostThroughputDependency)),
            "{text}"
        );
    }
    assert_eq!(
        Constraint::HostThroughputDependency.message(),
        "host check: --throughput-check, its --throughput-bytes and --upload-check run over the network of --wifi"
    );
}

#[test]
fn throughput_bytes_zero_is_refused() {
    assert_eq!(
        HostCheck::parse(
            "--host-check --wifi-check --wifi a --throughput-check http://x/f --throughput-bytes 0"
        ),
        Err(Error::ThroughputBytes)
    );
    assert_eq!(
        Error::ThroughputBytes.message(),
        "--throughput-bytes takes a size from 1 to 4294967296 bytes"
    );
}

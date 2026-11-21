use super::*;
use alloc::{boxed::Box, string::ToString, vec, vec::Vec};
use stela_crypto::{Sha256, digest::Digest};
use stela_drivers::{
    firmware::{Catalog, ErrorKind, Provenance, Resource, Target},
    usb::bluetooth::{
        FirmwareReceipt,
        realtek::{DownloadResponse, Error, UsbIdentity},
    },
};

/// The controller's BD_ADDR as Read BD_ADDR returns it, least significant
/// byte first: f6:e5:d4:c3:b2:a1 as written.
const ADDRESS: [u8; 6] = [0xa1, 0xb2, 0xc3, 0xd4, 0xe5, 0xf6];
const AT: &str = "xhci 0000:05:00.4 port 2.3";

fn stock() -> LocalVersion {
    LocalVersion {
        raw: [0, 0x0c, 0x0c, 0, 0x0c, 0x5d, 0, 0x52, 0x88],
    }
}
/// The readings of a running patch: lmp_subver 0x0d7b.
fn patched() -> LocalVersion {
    LocalVersion {
        raw: [0, 0x0c, 0x0c, 0, 0x0c, 0x5d, 0, 0x7b, 0x0d],
    }
}
fn identity() -> Identity {
    Identity {
        usb: UsbIdentity {
            vendor: 0x13d3,
            product: 0x3612,
            revision: 0,
        },
        version: stock(),
        register_subversion: 0x8852,
        register_revision: None,
        rom_version: 0,
        security: [0, 0],
        project: Project::Rtl8852Cu,
    }
}
fn receipt(name: &str, length: usize, member: &str) -> FirmwareReceipt {
    FirmwareReceipt {
        name: name.into(),
        length,
        sha256: [0x11; 32],
        source: "https://gitlab.com/kernel-firmware/linux-firmware".into(),
        revision: "59f9fc2ef8d1e312d2a613d4e1ffad2ff31a0a01".into(),
        member: member.into(),
        license: "LICENSES/LICENCE.rtlwifi_firmware.txt".into(),
        target_driver: "realtek-bluetooth-usb-epatch-v2".into(),
        target_hardware: "usb:13d3:3612:0000".into(),
        target_revision: identity().revision_label(),
        target_board: None,
    }
}
fn config() -> FirmwareReceipt {
    receipt(
        "rtl_bt/rtl8852cu_config.bin",
        6,
        "rtl_bt/rtl8761bu_config.bin",
    )
}
fn missing(name: &str) -> firmware::Error {
    firmware::Error {
        name: name.into(),
        kind: ErrorKind::Missing,
    }
}
fn download() -> Vec<DownloadResponse> {
    [0, 1, 0x82]
        .into_iter()
        .map(|sent_index| DownloadResponse {
            sent_index,
            returned_index: sent_index,
        })
        .collect()
}
fn report() -> OwnedReport {
    OwnedReport {
        dropped: None,
        before: identity(),
        after: patched(),
        address: ADDRESS,
        fallback: None,
        firmware: Box::new(receipt(
            "rtl_bt/rtl8852cu_fw_v2.bin",
            112953,
            "rtl_bt/rtl8852cu_fw_v2.bin",
        )),
        configuration: ConfigurationEvidence::Supplied(config()),
        container_version: ContainerVersion::V2([0x6e, 0xb1, 0x34, 1, 0x49, 0x8c, 1, 0]),
        v2: None,
        download: download(),
        credits: 1,
    }
}
fn heard(devices: usize) -> ScanReport {
    ScanReport {
        reports: 340,
        devices,
        saturated: false,
        best_rssi: Some(-48),
        malformed: 0,
        other_events: 0,
        window_us: 8_000_000,
    }
}
fn controller<'a>(
    initialization: Option<Result<&'a InitializationReport, &'a InitializationError>>,
) -> Controller<'a> {
    Controller {
        location: AT,
        vendor: 0x13d3,
        product: 0x3612,
        revision: 0,
        initialization,
    }
}
fn text(lines: &[Line]) -> Vec<(&'static str, Verdict, &str)> {
    lines
        .iter()
        .map(|line| (line.name, line.verdict, line.detail.as_str()))
        .collect()
}

#[test]
fn a_booted_rtl8852cu_that_hears_advertisers_passes_every_step() {
    let booted = InitializationReport::Realtek(report());
    let got = lines(&[controller(Some(Ok(&booted)))], Some(&Ok(heard(12))));
    assert_eq!(
        text(&got),
        [
            (
                "bluetooth-found",
                Verdict::Pass,
                "13d3:3612 revision 0000 at xhci 0000:05:00.4 port 2.3: Realtek setup"
            ),
            (
                "bluetooth-rom",
                Verdict::Pass,
                "RTL8852CU stock ROM: hci_ver=0c hci_rev=000c lmp_ver=0c lmp_subver=8852 \
                 manufacturer=005d register=8852 rom_version=0 key_id=0 (security 00-00)"
            ),
            (
                "bluetooth-firmware",
                Verdict::Pass,
                "rtl_bt/rtl8852cu_fw_v2.bin 112953 bytes sha256 1111111111111111… EPATCH v2 \
                 6eb13401498c0100; config rtl_bt/rtl8852cu_config.bin 6 bytes \
                 (rtl_bt/rtl8761bu_config.bin)"
            ),
            (
                "bluetooth-download",
                Verdict::Pass,
                "3 fragments acknowledged, the last sent as index 0x82"
            ),
            (
                "bluetooth-version",
                Verdict::Pass,
                "fw version 0x000c0d7b: hci_ver=0c hci_rev=000c lmp_ver=0c lmp_subver=0d7b \
                 manufacturer=005d; address f6:e5:d4:..:..:a1"
            ),
            (
                "bluetooth-scan",
                Verdict::Pass,
                "passive LE scan for 8 s: 12 advertisers in 340 reports, best RSSI -48 dBm"
            ),
            (
                "bluetooth",
                Verdict::Pass,
                "13d3:3612 runs its patch and its radio hears advertisers"
            ),
        ]
    );
    assert_eq!(
        got.iter().map(|line| line.name).collect::<Vec<_>>()[..6],
        STEPS
    );
}

#[test]
fn a_dropped_patch_and_a_v1_fallback_are_named_where_they_happened() {
    let mut booted = report();
    booted.dropped = Some(Box::new(FirmwareDrop {
        register_subversion: 0x8852,
        register_revision: None,
        version: patched(),
        events: vec![vec![0x0e, 4, 0, 0x66, 0xfc, 0]],
        event_count: 1,
    }));
    booted.fallback = Some(Box::new(missing("rtl_bt/rtl8852cu_fw_v2.bin")));
    booted.firmware = Box::new(receipt(
        "rtl_bt/rtl8852cu_fw.bin",
        113336,
        "rtl_bt/rtl8852cu_fw.bin",
    ));
    booted.container_version = ContainerVersion::V1(0xd7b8_fabf);
    let booted = InitializationReport::Realtek(booted);
    let got = lines(&[controller(Some(Ok(&booted)))], Some(&Ok(heard(3))));
    assert!(got[1].detail.ends_with(
        "; a running patch was dropped first with 0xfc66 (it read hci_ver=0c hci_rev=000c \
         lmp_ver=0c lmp_subver=0d7b manufacturer=005d, register=8852; 1 events in the 200 ms \
         after)"
    ));
    assert!(
        got[2]
            .detail
            .starts_with("rtl_bt/rtl8852cu_fw.bin 113336 bytes sha256 1111111111111111… EPATCH v1 0xd7b8fabf (after rtl_bt/rtl8852cu_fw_v2.bin not supplied);"),
        "{}",
        got[2].detail
    );
    assert!(got.iter().all(|line| line.verdict == Verdict::Pass));
}

#[test]
fn a_failed_attempt_passes_what_it_reached_and_fails_where_it_stopped() {
    // Both images missing: identified, no image staged.
    let attempt = Attempt {
        version: Some(stock()),
        register_subversion: Some(0x8852),
        rom_version: Some(0),
        security: Some([0, 0]),
        identity: Some(identity()),
        fallback: Some(missing("rtl_bt/rtl8852cu_fw_v2.bin")),
        configuration: Some(Ok(config())),
        ..Attempt::default()
    };
    let failed = InitializationError::Realtek(Error::Attempt {
        cause: Box::new(Error::Resource(missing("rtl_bt/rtl8852cu_fw.bin"))),
        observation: Box::new(attempt),
    });
    let got = lines(&[controller(Some(Err(&failed)))], None);
    assert_eq!(
        text(&got)[1..],
        [
            (
                "bluetooth-rom",
                Verdict::Pass,
                "RTL8852CU stock ROM: hci_ver=0c hci_rev=000c lmp_ver=0c lmp_subver=8852 \
                 manufacturer=005d register=8852 rom_version=0 key_id=0 (security 00-00)"
            ),
            (
                "bluetooth-firmware",
                Verdict::Fail,
                "rtl_bt/rtl8852cu_fw.bin not supplied (rtl_bt/rtl8852cu_fw_v2.bin not \
                 supplied first); config rtl_bt/rtl8852cu_config.bin 6 bytes \
                 (rtl_bt/rtl8761bu_config.bin)"
            ),
            ("bluetooth-download", Verdict::Skip, "no image was staged"),
            (
                "bluetooth-version",
                Verdict::Skip,
                "no image was downloaded"
            ),
            (
                "bluetooth-scan",
                Verdict::Skip,
                "the controller did not come up, so it was not scanned"
            ),
            ("bluetooth", Verdict::Fail, "failed at bluetooth-firmware"),
        ]
    );
    // Readings no btrtl IC matches, even after the drop: the ROM step
    // fails with both readings, and nothing after it runs.
    let unknown = Attempt {
        dropped: Some(FirmwareDrop {
            register_subversion: 0x8852,
            register_revision: None,
            version: patched(),
            events: Vec::new(),
            event_count: 0,
        }),
        version: Some(patched()),
        register_subversion: Some(0x8852),
        ..Attempt::default()
    };
    let failed = InitializationError::Realtek(Error::Attempt {
        cause: Box::new(Error::Unsupported(
            "readings match no btrtl IC, also after its running patch was dropped",
        )),
        observation: Box::new(unknown),
    });
    let got = lines(&[controller(Some(Err(&failed)))], None);
    assert_eq!(got[1].verdict, Verdict::Fail);
    assert!(got[1].detail.starts_with(
        "hci_ver=0c hci_rev=000c lmp_ver=0c lmp_subver=0d7b manufacturer=005d register=8852; a \
         running patch was dropped first"
    ));
    assert!(
        got[1]
            .detail
            .ends_with(": readings match no btrtl IC, also after its running patch was dropped")
    );
    assert!(got[2..6].iter().all(|line| line.verdict == Verdict::Skip));
}

#[test]
fn no_address_is_printed_whole_in_any_line() {
    let booted = InitializationReport::Realtek(report());
    let mut all = lines(&[controller(Some(Ok(&booted)))], Some(&Ok(heard(1))));
    // A refused address after the upload: its response holds the address.
    let attempt = Attempt {
        version: Some(stock()),
        register_subversion: Some(0x8852),
        rom_version: Some(0),
        security: Some([0, 0]),
        identity: Some(identity()),
        firmware: Some(receipt(
            "rtl_bt/rtl8852cu_fw_v2.bin",
            112953,
            "rtl_bt/rtl8852cu_fw_v2.bin",
        )),
        configuration: Some(Ok(config())),
        container_version: Some(ContainerVersion::V2([0; 8])),
        download: download(),
        after: Some(patched()),
        ..Attempt::default()
    };
    let mut observed = vec![0];
    observed.extend_from_slice(&ADDRESS);
    let refused = InitializationError::Realtek(Error::Attempt {
        cause: Box::new(Error::Protocol {
            requirement: "invalid controller address",
            observed,
        }),
        observation: Box::new(attempt),
    });
    let failed = lines(&[controller(Some(Err(&refused)))], None);
    assert_eq!(failed[4].verdict, Verdict::Fail);
    assert!(
        failed[4]
            .detail
            .ends_with(": invalid controller address (7 bytes)"),
        "{}",
        failed[4].detail
    );
    all.extend(failed);
    let text: String = all
        .iter()
        .map(|line| line.detail.to_ascii_lowercase())
        .collect::<Vec<_>>()
        .join("\n");
    assert!(text.contains("address f6:e5:d4:..:..:a1"));
    // The hidden middle bytes, c3 and b2, in no rendering.
    for shown in [
        "c3:b2", "b2:c3", "c3b2", "b2c3", "c3-b2", "b2-c3", "0xc3", "0xb2", "195, 178", "178, 195",
        "d4:c3", "b2:a1",
    ] {
        assert!(!text.contains(shown), "{shown} in {text}");
    }
}

#[test]
fn no_realtek_controller_fails_the_check_naming_what_was_found() {
    let lines_of = |controllers: &[Controller<'_>]| lines(controllers, None);
    let none = lines_of(&[]);
    assert_eq!(
        text(&none)[0],
        (
            "bluetooth-found",
            Verdict::Fail,
            "no USB Bluetooth controller a Realtek setup takes; found none"
        )
    );
    let mediatek = Controller {
        location: "xhci 0000:00:14.0 port 10",
        vendor: 0x0e8d,
        product: 0x7922,
        revision: 0x100,
        initialization: None,
    };
    let other = lines_of(&[mediatek]);
    assert_eq!(
        other[0].detail,
        "no USB Bluetooth controller a Realtek setup takes; found 0e8d:7922 at xhci \
         0000:00:14.0 port 10"
    );
    for got in [none, other] {
        assert!(got[1..6].iter().all(|line| line.verdict == Verdict::Skip));
        assert_eq!(
            text(&got)[6],
            ("bluetooth", Verdict::Fail, "failed at bluetooth-found")
        );
    }
}

#[test]
fn a_scan_that_hears_nothing_fails_the_check() {
    let booted = InitializationReport::Realtek(report());
    let got = lines(&[controller(Some(Ok(&booted)))], Some(&Ok(heard(0))));
    assert_eq!(got[5].verdict, Verdict::Fail);
    assert!(got[5].detail.starts_with(
        "passive LE scan for 8 s heard no advertiser (340 reports): nothing advertised nearby"
    ));
    assert_eq!(got[6].detail, "failed at bluetooth-scan");
    let refused = Error::Controller {
        opcode: 0x200b,
        status: 0x0c,
        parameters: vec![0x0c],
    };
    let got = lines(&[controller(Some(Ok(&booted)))], Some(&Err(refused)));
    assert_eq!(
        text(&got)[5],
        (
            "bluetooth-scan",
            Verdict::Fail,
            "the scan failed: command 0x200b answered status 0x0c"
        )
    );
}

#[test]
fn the_check_takes_the_first_realtek_controller_and_scans_only_a_booted_one() {
    let booted = InitializationReport::Realtek(report());
    let failed = InitializationError::Realtek(Error::Unsupported("USB identity"));
    let mediatek = Controller {
        location: "xhci 0000:00:14.0 port 10",
        vendor: 0x0e8d,
        product: 0x7922,
        revision: 0x100,
        initialization: None,
    };
    let controllers = [mediatek, controller(Some(Ok(&booted)))];
    assert_eq!(chosen(&controllers), Some(1));
    assert!(scannable(&controllers[1]));
    assert!(!scannable(&controller(Some(Err(&failed)))));
    assert!(!scannable(&controller(None)));
    assert_eq!(Verdict::Skip.to_string(), "SKIP");
}

fn catalog(name: &'static str, driver: &'static str, bytes: &[u8]) -> Catalog {
    Catalog::from_handoff(vec![Resource {
        name,
        target: Target {
            driver,
            hardware: "usb:13d3:3612:0000",
            revision: "r",
            board: None,
        },
        provenance: Provenance {
            source: "synthetic",
            revision: "v1",
            member: name,
            license: "MIT",
        },
        expected_length: bytes.len(),
        expected_sha256: Sha256::digest(bytes).into(),
        bytes: bytes.to_vec(),
    }])
    .unwrap()
}

#[test]
fn only_usb_bluetooth_requests_reach_the_bundle() {
    let bundle = catalog("bt.bin", "realtek-bluetooth-usb-epatch-v2", b"bt");
    let rest = catalog("nic.bin", "realtek", b"nic");
    let provider = BluetoothOnly {
        bluetooth: &bundle,
        rest: &rest,
    };
    let request = |name, driver| Requirement {
        name,
        target: Target {
            driver,
            hardware: "usb:13d3:3612:0000",
            revision: "r",
            board: None,
        },
        minimum_bytes: 1,
    };
    assert_eq!(
        provider
            .load(request("bt.bin", "realtek-bluetooth-usb-epatch-v2"))
            .unwrap()
            .bytes(),
        b"bt"
    );
    assert_eq!(
        provider
            .load(request("nic.bin", "realtek"))
            .unwrap()
            .bytes(),
        b"nic"
    );
    // Neither is asked for the other's files.
    assert_eq!(
        provider
            .load(request("nic.bin", "realtek-bluetooth-usb-epatch-v2"))
            .unwrap_err()
            .kind,
        ErrorKind::Missing
    );
    assert_eq!(
        provider
            .load(request("bt.bin", "realtek"))
            .unwrap_err()
            .kind,
        ErrorKind::Missing
    );
}

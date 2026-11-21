use super::*;
use crate::{
    block::BlockError,
    firmware::{Catalog, Image, Provenance, Provider, Requirement, Resource, Target},
    usb::{
        bluetooth::{
            Binding, FirmwareStatus, InitializationError, InitializationReport, Pipe, State,
            intel::PlatformInputs,
        },
        xhci::{Device, FULL_SPEED},
    },
};
use alloc::{collections::VecDeque, rc::Rc, string::String};
use core::cell::{Cell, RefCell};
use sha2::{Digest, Sha256};
/// The RTL8852CU's stock-ROM label for its first image, `_v2.bin`.
const RTL8852CU_V2: &str =
    "epatch-v2:hci-0c-000c:lmp-0c-8852:manufacturer-005d:register-8852-none:rom-00:security-00-00";
/// The complete btusb 8852CE group (btusb.c:575-591).
const RTL8852CE: [(u16, u16); 8] = [
    (0x04ca, 0x4007),
    (0x04c5, 0x1675),
    (0x0cb8, 0xc558),
    (0x13d3, 0x3587),
    (0x13d3, 0x3586),
    (0x13d3, 0x3592),
    (0x13d3, 0x3612),
    (0x0489, 0xe122),
];
pub(super) fn initialized_report(project: Project) -> OwnedReport {
    let (mut hci, catalog, _) = fixture(project, true);
    initialize(&mut hci, &catalog).unwrap()
}
struct Step {
    parameters: Vec<u8>,
    response: Vec<u8>,
}
#[derive(Default)]
struct Trace {
    commands: usize,
    closed: usize,
}
struct Model {
    steps: VecDeque<Step>,
    events: VecDeque<Vec<u8>>,
    time: Cell<u64>,
    trace: Rc<RefCell<Trace>>,
    cleanup_error: bool,
    fail_command: Option<usize>,
}
impl Io for Model {
    fn validate(&self) -> core::result::Result<(), BlockError> {
        Ok(()) // This in-memory model has no physical device-generation owner.
    }
    fn control(&mut self, setup: [u8; 8], p: &[u8]) -> core::result::Result<(), BlockError> {
        assert_eq!(setup[..6], [0x20, 0, 0, 0, 0, 0]);
        assert_eq!(self.trace.borrow().closed, 0);
        let step = self.steps.pop_front().expect("unexpected command");
        assert_eq!(p, step.parameters);
        self.trace.borrow_mut().commands += 1;
        if self.fail_command == Some(self.trace.borrow().commands) {
            return Err(BlockError::Device { status: 0x55 });
        }
        // Exercise framing across two transfer completions.
        if !step.response.is_empty() {
            self.events.push_back(step.response[..1].to_vec());
            self.events.push_back(step.response[1..].to_vec());
        }
        Ok(())
    }
    fn receive(&mut self, pipe: Pipe) -> core::result::Result<Option<Vec<u8>>, BlockError> {
        Ok(if pipe == Pipe::Event {
            self.events.pop_front()
        } else {
            None
        })
    }
    fn write_acl(&mut self, _: &[u8]) -> core::result::Result<(), BlockError> {
        panic!("firmware commands must use EP0")
    }
    fn quiesce_bulk(&mut self) -> core::result::Result<(), BlockError> {
        panic!("Realtek must not enter Intel bulk boot mode")
    }
    fn close(&mut self) -> core::result::Result<(), BlockError> {
        self.trace.borrow_mut().closed += 1;
        if self.cleanup_error {
            Err(BlockError::Timeout)
        } else {
            Ok(())
        }
    }
    fn now_us(&self) -> u64 {
        let t = self.time.get();
        self.time.set(t + 100_000);
        t
    }
}
fn command(op: u16, p: &[u8]) -> Vec<u8> {
    let mut b = op.to_le_bytes().to_vec();
    b.push(p.len() as u8);
    b.extend_from_slice(p);
    b
}
fn cc(op: u16, p: &[u8]) -> Vec<u8> {
    let mut b = vec![0x0e, (p.len() + 3) as u8, 1];
    b.extend_from_slice(&op.to_le_bytes());
    b.extend_from_slice(p);
    b
}
/// The projects whose first image is an RTBTCore (EPATCH v2) container.
fn v2(project: Project) -> bool {
    matches!(project, Project::Rtl8852Bu | Project::Rtl8852Cu)
}
fn version(project: Project, after: bool) -> Vec<u8> {
    let (hci, revision, lmp) = match project {
        Project::Rtl8761Bu => (10, 11, 0x8761u16),
        Project::Rtl8822B => (7, 11, 0x8822),
        Project::Rtl8852Bu => (11, 11, 0x8852),
        // btrtl.c:305-306: IC_INFO(RTL_ROM_LMP_8852A, 0xc, 0xc, HCI_USB).
        Project::Rtl8852Cu => (12, 12, 0x8852),
    };
    let mut b = vec![0, hci, revision, 0, hci, 0x5d, 0];
    b.extend_from_slice(&(if after { 0x5678 } else { lmp }).to_le_bytes());
    b
}
fn firmware(project: Project) -> Vec<u8> {
    if v2(project) {
        let mut b = b"RTBTCore".to_vec();
        b.extend_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        b.extend_from_slice(&2u32.to_le_bytes());
        b.extend_from_slice(&1u32.to_le_bytes());
        b.extend_from_slice(&16u32.to_le_bytes());
        b.extend_from_slice(&[1, 0, 0, 0, 1, 15, 0, 0, 4, 0, 0, 0, 0xde, 0xad, 0xbe, 0xef]);
        b.extend_from_slice(&8u32.to_le_bytes());
        b.extend_from_slice(&2u32.to_le_bytes());
        b.extend_from_slice(&[0x99, 0x98]);
        b.extend_from_slice(&[0xff, 2, 1, 1, project.id(), 1, 0, 0x51, 4, 0xfd, 0x77]);
        return b;
    }
    v1_image(project)
}
/// A Realtech (EPATCH v1) container for `project`: one ROM 0 patch.
fn v1_image(project: Project) -> Vec<u8> {
    let mut b = vec![0; 36];
    b[..8].copy_from_slice(b"Realtech");
    b[8..12].copy_from_slice(&0x12345678u32.to_le_bytes());
    b[12..14].copy_from_slice(&1u16.to_le_bytes());
    b[14..16].copy_from_slice(&1u16.to_le_bytes());
    b[16..18].copy_from_slice(&4u16.to_le_bytes());
    b[18..22].copy_from_slice(&24u32.to_le_bytes());
    b[28..].copy_from_slice(&[0xff, project.id(), 1, 0, 0x51, 4, 0xfd, 0x77]);
    b
}
fn fixture(project: Project, config: bool) -> (Hci<Model>, Catalog, Rc<RefCell<Trace>>) {
    let (vid, pid, sub, hci) = match project {
        Project::Rtl8761Bu => (0x0bdau16, 0x8771u16, 0x8761u16, 10),
        Project::Rtl8822B => (0x13d3, 0x3526, 0x8822, 7),
        Project::Rtl8852Bu => (0x0bda, 0xb85b, 0x8852, 11),
        Project::Rtl8852Cu => (0x13d3, 0x3612, 0x8852, 12),
    };
    let mut descriptor = [18, 1, 0, 2, 0xe0, 1, 1, 64, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1];
    descriptor[8..10].copy_from_slice(&vid.to_le_bytes());
    descriptor[10..12].copy_from_slice(&pid.to_le_bytes());
    let configuration = vec![
        9, 2, 39, 0, 1, 1, 0, 0x80, 50, 9, 4, 0, 0, 3, 0xe0, 1, 1, 0, 7, 5, 0x81, 3, 16, 0, 1, 7,
        5, 0x82, 2, 64, 0, 0, 7, 5, 2, 2, 64, 0, 0,
    ];
    let device = Device {
        id: crate::usb::xhci::SlotId::fixture(1),
        slot: 1,
        speed: FULL_SPEED,
        location: "Realtek stateful HCI model".into(),
        descriptor,
        configurations: vec![configuration.clone()],
        configuration,
        manufacturer: None,
        product: None,
        serial: None,
        language: None,
        vendor_ethernet_revision: None,
        vendor_ethernet_tcr: None,
    };
    let hardware = match project {
        Project::Rtl8761Bu => "usb:0bda:8771:0100",
        Project::Rtl8822B => "usb:13d3:3526:0100",
        Project::Rtl8852Bu => "usb:0bda:b85b:0100",
        Project::Rtl8852Cu => "usb:13d3:3612:0100",
    };
    let revision = match project {
        Project::Rtl8761Bu => {
            "epatch-v1:hci-0a-000b:lmp-0a-8761:manufacturer-005d:register-8761-none:rom-00:security-00-00"
        }
        Project::Rtl8822B => {
            "epatch-v1:hci-07-000b:lmp-07-8822:manufacturer-005d:register-8822-000b:rom-00:security-00-00"
        }
        Project::Rtl8852Bu => {
            "epatch-v2:hci-0b-000b:lmp-0b-8852:manufacturer-005d:register-8852-none:rom-00:security-00-00"
        }
        Project::Rtl8852Cu => RTL8852CU_V2,
    };
    let target = Target {
        driver: if v2(project) {
            "realtek-bluetooth-usb-epatch-v2"
        } else {
            "realtek-bluetooth-usb-epatch-v1"
        },
        hardware,
        revision,
        board: None,
    };
    let name = match project {
        Project::Rtl8761Bu => "rtl_bt/rtl8761bu_fw.bin",
        Project::Rtl8822B => "rtl_bt/rtl8822b_fw.bin",
        Project::Rtl8852Bu => "rtl_bt/rtl8852bu_fw.bin",
        Project::Rtl8852Cu => "rtl_bt/rtl8852cu_fw_v2.bin",
    };
    let cfgname = match project {
        Project::Rtl8761Bu => "rtl_bt/rtl8761bu_config.bin",
        Project::Rtl8822B => "rtl_bt/rtl8822b_config.bin",
        Project::Rtl8852Bu => "rtl_bt/rtl8852bu_config.bin",
        Project::Rtl8852Cu => "rtl_bt/rtl8852cu_config.bin",
    };
    let cfg = vec![0x55, 0xab, 0x23, 0x87, 0, 0];
    let fw = firmware(project);
    let resource = |name, bytes: Vec<u8>| Resource {
        name,
        target,
        provenance: Provenance {
            source: "original stateful fixture",
            revision: "model-v1",
            // linux-firmware WHENCE links both v2 profiles' configs to it.
            member: if v2(project) && name == cfgname {
                "rtl_bt/rtl8761bu_config.bin"
            } else {
                name
            },
            license: "MIT",
        },
        expected_length: bytes.len(),
        expected_sha256: Sha256::digest(&bytes).into(),
        bytes,
    };
    let mut resources = vec![resource(name, fw)];
    if config {
        resources.push(resource(cfgname, cfg.clone()));
    }
    let catalog = Catalog::from_handoff(resources).unwrap();
    let mut steps = VecDeque::new();
    let mut add = |op, p: &[u8], r: &[u8]| {
        steps.push_back(Step {
            parameters: command(op, p),
            response: cc(op, r),
        })
    };
    add(
        0xfc61,
        &[0x10, 0x38, 4, 0x28, 0x80],
        &[0, sub as u8, (sub >> 8) as u8],
    );
    if project == Project::Rtl8822B {
        add(0xfc61, &[0x10, 0x3a, 4, 0x28, 0x80], &[0, 11, 0]);
    }
    add(0x1001, &[], &version(project, false));
    add(0xfc6d, &[], &[0, 0]);
    add(0xfc61, &[0x10, 0xa4, 0xad, 0, 0xb0], &[0, 0, 0]);
    let mut data = if v2(project) {
        vec![0x80, 0xde, 0xad, 0xbe, 0xef]
    } else {
        vec![0x80, 0x78, 0x56, 0x34, 0x12]
    };
    if config {
        data.extend_from_slice(&cfg);
    }
    add(0xfc20, &data, &[0, 0x42]);
    add(0x1001, &[], &version(project, true));
    add(0x1009, &[], &[0, 1, 2, 3, 4, 5, 6]);
    let trace = Rc::new(RefCell::new(Trace::default()));
    let binding = Binding::new(
        &device,
        &crate::usb::descriptor::interfaces(&device.configuration)[0],
    )
    .unwrap();
    assert_eq!(hci, version(project, false)[1]);
    (
        Hci::from_io(
            binding,
            Model {
                steps,
                events: VecDeque::new(),
                time: Cell::new(0),
                trace: trace.clone(),
                cleanup_error: false,
                fail_command: None,
            },
        ),
        catalog,
        trace,
    )
}
struct Recording<'a> {
    catalog: &'a Catalog,
    requests: RefCell<Vec<(String, String, String, String)>>,
}
impl Provider for Recording<'_> {
    fn load(&self, r: Requirement<'_>) -> core::result::Result<Image<'_>, crate::firmware::Error> {
        self.requests.borrow_mut().push((
            r.name.into(),
            r.target.hardware.into(),
            r.target.revision.into(),
            r.target.driver.into(),
        ));
        self.catalog.load(r)
    }
}
fn attempt(e: &Error) -> (&Error, &Attempt) {
    match e {
        Error::Attempt { cause, observation } => (cause, observation),
        Error::Cleanup { cause, .. } => attempt(cause),
        _ => panic!("missing partial attempt {e:?}"),
    }
}
#[test]
fn native_report_keeps_identity_receipts_and_indices_after_provider_drop() {
    for project in [Project::Rtl8761Bu, Project::Rtl8822B, Project::Rtl8852Bu] {
        let (mut hci, catalog, trace) = fixture(project, true);
        {
            let provider = Recording {
                catalog: &catalog,
                requests: RefCell::new(vec![]),
            };
            let InitializationReport::Realtek(r) = hci
                .initialize_with_provider(&provider, PlatformInputs::default())
                .unwrap()
            else {
                panic!("Realtek report expected")
            };
            assert_eq!(r.before.project, project);
            assert_eq!(r.before.security, [0, 0]);
            assert_eq!(r.after.raw, version(project, true)[..]);
            assert_eq!(r.address, [1, 2, 3, 4, 5, 6]);
            if project == Project::Rtl8852Bu {
                assert_eq!(
                    r.container_version,
                    ContainerVersion::V2([1, 2, 3, 4, 5, 6, 7, 8])
                );
                let meta = r.v2.as_ref().unwrap();
                assert_eq!(meta.project, Project::Rtl8852Bu);
                assert_eq!(meta.sections[1].opcode, 8);
                assert!(meta.sections[1].skipped);
                let ConfigurationEvidence::Supplied(config) = &r.configuration else {
                    panic!("supplied config expected")
                };
                assert_eq!(config.name, "rtl_bt/rtl8852bu_config.bin");
                assert_eq!(config.member, "rtl_bt/rtl8761bu_config.bin");
            } else {
                assert_eq!(r.container_version, ContainerVersion::V1(0x12345678));
                assert!(r.v2.is_none());
            }
            assert_eq!(
                r.download,
                vec![DownloadResponse {
                    sent_index: 0x80,
                    returned_index: 0x42
                }]
            );
            assert_eq!(provider.requests.borrow().len(), 2);
            assert_eq!(r.firmware.target_revision, r.before.revision_label());
            assert!(r.firmware.target_revision.len() < 256);
            assert_eq!(r.firmware.target_hardware, r.before.usb.hardware_label());
        }
        drop(catalog);
        let commands = trace.borrow().commands;
        let empty = Catalog::from_handoff(vec![]).unwrap();
        assert!(
            hci.initialize_with_provider(&empty, PlatformInputs::default())
                .is_ok()
        );
        assert_eq!(trace.borrow().commands, commands);
        assert_eq!(hci.state(), State::Open);
        assert_eq!(hci.firmware_status(), FirmwareStatus::Unverified);
        assert!(hci.io.steps.is_empty());
    }
}
#[test]
fn missing_firmware_records_both_exact_companions_and_closes_without_upload() {
    let (mut hci, _, trace) = fixture(Project::Rtl8761Bu, true);
    let empty = Catalog::from_handoff(vec![]).unwrap();
    let provider = Recording {
        catalog: &empty,
        requests: RefCell::new(vec![]),
    };
    let InitializationError::Realtek(e) = hci
        .initialize_with_provider(&provider, PlatformInputs::default())
        .unwrap_err()
    else {
        panic!("Realtek error expected")
    };
    let (cause, a) = attempt(e);
    assert!(matches!(cause,Error::Resource(e)if e.name=="rtl_bt/rtl8761bu_fw.bin"));
    assert!(a.download.is_empty());
    assert_eq!(a.identity.as_ref().unwrap().rom_version, 0);
    assert!(a.configuration.as_ref().unwrap().is_err());
    let calls = provider.requests.borrow();
    assert_eq!(calls.len(), 2);
    assert_eq!(calls[0].0, "rtl_bt/rtl8761bu_fw.bin");
    assert_eq!(calls[1].0, "rtl_bt/rtl8761bu_config.bin");
    assert_eq!(calls[0].1, "usb:0bda:8771:0100");
    assert_eq!(calls[0].2, calls[1].2);
    drop(calls);
    assert_eq!(trace.borrow().commands, 4);
    assert_eq!(hci.state(), State::Closed);
    let before = trace.borrow().commands;
    assert!(
        hci.initialize_with_provider(&provider, PlatformInputs::default())
            .is_err()
    );
    assert_eq!(trace.borrow().commands, before);
    assert_eq!(provider.requests.borrow().len(), 2);
    assert!(hci.submit_command(&[3, 0x0c, 0]).is_err());
    assert!(hci.submit_acl(&[1, 0, 0, 0]).is_err());
}
#[test]
fn absent_config_is_kept_only_for_the_exact_optional_profile() {
    let (mut hci, catalog, _) = fixture(Project::Rtl8761Bu, false);
    let r = initialize(&mut hci, &catalog).unwrap();
    assert!(
        matches!(r.configuration,ConfigurationEvidence::OptionalMissing(e)if e.name=="rtl_bt/rtl8761bu_config.bin")
    );
    let (mut hci, catalog, trace) = fixture(Project::Rtl8822B, false);
    let e = initialize(&mut hci, &catalog).unwrap_err();
    assert!(matches!(attempt(&e).0,Error::Resource(e)if e.name=="rtl_bt/rtl8822b_config.bin"));
    assert_eq!(trace.borrow().commands, 5);
    assert_eq!(hci.state(), State::Closed);
}
#[test]
fn completion_status_credits_shape_and_timeout_are_required() {
    for fault in 0..7 {
        let (mut hci, catalog, trace) = fixture(Project::Rtl8761Bu, true);
        let step = &mut hci.io.steps[4];
        match fault {
            0 => step.response[5] = 0x0c,
            1 => step.response[2] = 0,
            2 => step.response[3] = 0x21,
            3 => step.response = cc(0xfc20, &[0]),
            4 => step.response.clear(),
            5 => step.response = vec![0x0f, 4, 0, 1, 0x20, 0xfc], // delivery/Status without CC
            _ => {
                step.response = vec![0x0f, 4, 0, 1, 0x20, 0xfc];
                step.response.extend_from_slice(&cc(0xfc20, &[0, 0x42]));
            }
        }
        let result = initialize(&mut hci, &catalog);
        if fault == 6 {
            assert!(result.is_ok());
            continue;
        }
        let e = result.unwrap_err();
        let (error, a) = attempt(&e);
        if fault == 1 {
            // A completion may grant no credits; without a later grant
            // the next command never goes out.
            assert!(matches!(
                error,
                Error::Timeout("genuine HCI command credits")
            ));
            assert_eq!(trace.borrow().commands, 5);
            assert_eq!(hci.state(), State::Closed);
            continue;
        }
        assert!(a.firmware.is_some());
        assert!(a.download.is_empty());
        assert_eq!(trace.borrow().commands, 5);
        assert_eq!(hci.state(), State::Closed);
    }
}
#[test]
fn rejected_identity_security_rom_and_postboot_observations_fail_closed() {
    for fault in 0..6 {
        let (mut hci, catalog, trace) = fixture(Project::Rtl8761Bu, true);
        match fault {
            0 => hci.io.steps[1].response[10] = 0,
            1 => hci.io.steps[2].response[6] = 7,
            2 => hci.io.steps[3].response[6] = 3,
            3 => hci.io.steps[5].response = cc(0x1001, &version(Project::Rtl8761Bu, false)),
            4 => hci.io.steps[5].response[10] = 0,
            _ => hci.io.steps[6].response = cc(0x1009, &[0, 0, 0, 0, 0, 0, 0]),
        }
        let e = initialize(&mut hci, &catalog).unwrap_err();
        let (_, a) = attempt(&e);
        if fault >= 3 {
            assert_eq!(a.download.len(), 1);
            assert!(a.after.is_some());
        } else {
            assert!(a.download.is_empty());
        }
        assert_eq!(hci.state(), State::Closed);
        assert_eq!(trace.borrow().closed, 1);
    }
}
#[test]
fn cleanup_failure_is_separate_and_cannot_reopen_candidate() {
    let (mut hci, catalog, _) = fixture(Project::Rtl8761Bu, true);
    hci.io.steps[4].response[5] = 1;
    hci.io.cleanup_error = true;
    let e = initialize(&mut hci, &catalog).unwrap_err();
    assert!(matches!(e, Error::Cleanup { .. }));
    assert!(matches!(
        attempt(&e).0,
        Error::Controller {
            opcode: 0xfc20,
            status: 1,
            ..
        }
    ));
    assert_eq!(hci.state(), State::Failed);
    assert!(hci.submit_command(&[3, 0x0c, 0]).is_err());
}
#[test]
fn optional_config_integrity_or_target_errors_never_become_absence() {
    struct RejectConfig<'a> {
        catalog: &'a Catalog,
        kind: crate::firmware::ErrorKind,
    }
    impl Provider for RejectConfig<'_> {
        fn load(
            &self,
            r: Requirement<'_>,
        ) -> core::result::Result<Image<'_>, crate::firmware::Error> {
            if r.name.ends_with("_config.bin") {
                Err(crate::firmware::Error {
                    name: r.name.into(),
                    kind: self.kind.clone(),
                })
            } else {
                self.catalog.load(r)
            }
        }
    }
    for project in [Project::Rtl8761Bu, Project::Rtl8852Bu] {
        for kind in [
            crate::firmware::ErrorKind::WrongTarget,
            crate::firmware::ErrorKind::DigestMismatch {
                expected: [1; 32],
                actual: [2; 32],
            },
        ] {
            let (mut hci, catalog, trace) = fixture(project, true);
            let provider = RejectConfig {
                catalog: &catalog,
                kind: kind.clone(),
            };
            let e = initialize(&mut hci, &provider).unwrap_err();
            assert!(matches!(attempt(&e).0,Error::Resource(e)if e.kind==kind));
            assert!(attempt(&e).1.download.is_empty());
            assert_eq!(trace.borrow().commands, 4);
            assert_eq!(hci.state(), State::Closed);
        }
    }
}
#[test]
fn usb_failures_keep_typed_error_and_native_attempt_never_reopens() {
    for command in 1..=7 {
        let (mut hci, catalog, trace) = fixture(Project::Rtl8761Bu, true);
        hci.io.fail_command = Some(command);
        let InitializationError::Realtek(e) = hci
            .initialize_with_provider(&catalog, PlatformInputs::default())
            .unwrap_err()
        else {
            panic!("Realtek error expected")
        };
        assert!(
            matches!(attempt(e).0,Error::Transport(e)if matches!(e.kind,crate::usb::bluetooth::ErrorKind::Usb(BlockError::Device{status:0x55})))
        );
        assert_eq!(hci.state(), State::Failed);
        let sent = trace.borrow().commands;
        assert!(
            hci.initialize_with_provider(&catalog, PlatformInputs::default())
                .is_err()
        );
        assert_eq!(trace.borrow().commands, sent);
        assert_eq!(trace.borrow().closed, 1);
    }
}
#[test]
fn v2_requires_exact_usb_stock_hci_register_rom_and_key_profile() {
    for (vendor, product) in [
        (0x0cb8, 0xc559),
        (0x0bda, 0x4853),
        (0x0bda, 0x887b),
        (0x0bda, 0xb85b),
        (0x13d3, 0x3570),
        (0x13d3, 0x3571),
        (0x13d3, 0x3572),
        (0x13d3, 0x3591),
        (0x0489, 0xe123),
        (0x0489, 0xe125),
    ] {
        let usb = UsbIdentity {
            vendor,
            product,
            revision: 0x1234,
        };
        let v = LocalVersion::parse(&version(Project::Rtl8852Bu, false)).unwrap();
        let identity = Identity::select(usb, v.clone(), 0x8852, None, 1, [0, 0x42]).unwrap();
        assert_eq!(identity.project, Project::Rtl8852Bu);
        assert!(identity.revision_label().ends_with("rom-01:security-00-42"));
        assert!(identity.usb.hardware_label().ends_with(":1234"));
        for wrong in [Project::Rtl8761Bu, Project::Rtl8822B] {
            assert!(
                Identity::select(
                    usb,
                    LocalVersion::parse(&version(wrong, false)).unwrap(),
                    0x8852,
                    None,
                    1,
                    [0, 0]
                )
                .is_err()
            );
        }
        assert!(Identity::select(usb, v.clone(), 0x8851, None, 1, [0, 0]).is_err());
        assert!(Identity::select(usb, v, 0x8852, None, 1, [1, 0]).is_err());
    }
    // 0cb8:c558 left this list: btusb.c:580 lists it as an 8852CE, which
    // the RTL8852CU test below takes.
    for (vendor, product) in [(0x0bda, 0x8520), (0x13d3, 0x3573), (0x1209, 0xb85b)] {
        assert!(
            UsbIdentity {
                vendor,
                product,
                revision: 0x100
            }
            .project()
            .is_none()
        );
    }
}
#[test]
fn v2_missing_companions_are_exact_and_only_named_optional_config_absence_is_permitted() {
    let (mut hci, _, trace) = fixture(Project::Rtl8852Bu, true);
    let empty = Catalog::from_handoff(vec![]).unwrap();
    let provider = Recording {
        catalog: &empty,
        requests: RefCell::new(vec![]),
    };
    let e = initialize(&mut hci, &provider).unwrap_err();
    let (cause, a) = attempt(&e);
    assert!(matches!(cause,Error::Resource(e)if e.name=="rtl_bt/rtl8852bu_fw.bin"));
    assert!(a.download.is_empty());
    assert!(a.v2.is_none());
    let requests = provider.requests.borrow();
    assert_eq!(requests.len(), 2);
    assert_eq!(requests[0].0, "rtl_bt/rtl8852bu_fw.bin");
    assert_eq!(requests[1].0, "rtl_bt/rtl8852bu_config.bin");
    for r in requests.iter() {
        assert_eq!(r.1, "usb:0bda:b85b:0100");
        assert_eq!(
            r.2,
            "epatch-v2:hci-0b-000b:lmp-0b-8852:manufacturer-005d:register-8852-none:rom-00:security-00-00"
        );
        assert_eq!(r.3, "realtek-bluetooth-usb-epatch-v2");
    }
    assert_eq!(trace.borrow().commands, 4);
    assert_eq!(hci.state(), State::Closed);
    let (mut hci, catalog, _) = fixture(Project::Rtl8852Bu, false);
    let r = initialize(&mut hci, &catalog).unwrap();
    assert!(
        matches!(r.configuration,ConfigurationEvidence::OptionalMissing(e)if e.name=="rtl_bt/rtl8852bu_config.bin")
    );
}
#[test]
fn v2_validated_metadata_and_alias_receipt_survive_rejected_upload_or_boot() {
    for boot in [false, true] {
        let (mut hci, catalog, trace) = fixture(Project::Rtl8852Bu, true);
        if boot {
            hci.io.steps[5].response = cc(0x1001, &version(Project::Rtl8852Bu, false));
        } else {
            hci.io.steps[4].response = cc(0xfc20, &[0x0c, 0]);
        }
        let InitializationError::Realtek(e) = hci
            .initialize_with_provider(&catalog, PlatformInputs::default())
            .unwrap_err()
        else {
            panic!("Realtek error expected")
        };
        let (_, a) = attempt(e);
        assert_eq!(
            a.container_version,
            Some(ContainerVersion::V2([1, 2, 3, 4, 5, 6, 7, 8]))
        );
        let metadata = a.v2.as_ref().unwrap();
        assert_eq!(metadata.sections[0].subsections[0].control, [0, 0]);
        assert!(metadata.sections[1].skipped);
        assert_eq!(metadata.sections[1].length, 2);
        assert_eq!(
            metadata.sections[1].sha256,
            <[u8; 32]>::from(Sha256::digest([0x99, 0x98]))
        );
        let config = a.configuration.as_ref().unwrap().as_ref().unwrap();
        assert_eq!(config.name, "rtl_bt/rtl8852bu_config.bin");
        assert_eq!(config.member, "rtl_bt/rtl8761bu_config.bin");
        assert_eq!(a.download.len(), usize::from(boot));
        assert_eq!(a.after.is_some(), boot);
        assert_eq!(hci.state(), State::Closed);
        let commands = trace.borrow().commands;
        assert!(
            hci.initialize_with_provider(&catalog, PlatformInputs::default())
                .is_err()
        );
        assert_eq!(trace.borrow().commands, commands);
        assert!(hci.submit_command(&[3, 0x0c, 0]).is_err());
    }
}
#[test]
fn rtl8852cu_requires_the_btusb_8852ce_group_and_its_usb_stock_profile() {
    // CHIP_ID_8852C (btrtl.c:71, :670).
    assert_eq!(Project::Rtl8852Cu.id(), 25);
    for (vendor, product) in RTL8852CE {
        let usb = UsbIdentity {
            vendor,
            product,
            revision: 0,
        };
        assert_eq!(
            usb.project(),
            Some(Project::Rtl8852Cu),
            "{vendor:04x}:{product:04x}"
        );
        assert_eq!(
            crate::usb::bluetooth::Setup::of(vendor, product, false),
            Some(crate::usb::bluetooth::Setup::Realtek)
        );
        let stock = LocalVersion::parse(&version(Project::Rtl8852Cu, false)).unwrap();
        let identity = Identity::select(usb, stock.clone(), 0x8852, None, 1, [0, 0x42]).unwrap();
        assert_eq!(identity.project, Project::Rtl8852Cu);
        assert!(
            identity
                .revision_label()
                .ends_with(":rom-01:security-00-42")
        );
        // An 8852B's or 8761B's stock readings, another register
        // subversion and a nonzero key are refused under an 8852CE ID.
        for wrong in [Project::Rtl8852Bu, Project::Rtl8761Bu] {
            let other = LocalVersion::parse(&version(wrong, false)).unwrap();
            assert!(Identity::select(usb, other, 0x8852, None, 0, [0, 0]).is_err());
        }
        assert!(Identity::select(usb, stock.clone(), 0x8851, None, 0, [0, 0]).is_err());
        assert!(Identity::select(usb, stock, 0x8852, None, 0, [1, 0]).is_err());
    }
    // An 8852C's readings under an 8852BE ID select nothing.
    let be = UsbIdentity {
        vendor: 0x0bda,
        product: 0xb85b,
        revision: 0,
    };
    let stock = LocalVersion::parse(&version(Project::Rtl8852Cu, false)).unwrap();
    assert!(Identity::select(be, stock, 0x8852, None, 0, [0, 0]).is_err());
}
#[test]
fn rtl8852cu_names_btrtls_v2_image_then_its_v1_image_and_an_optional_config() {
    let usb = UsbIdentity {
        vendor: 0x13d3,
        product: 0x3612,
        revision: 0,
    };
    let stock = LocalVersion::parse(&version(Project::Rtl8852Cu, false)).unwrap();
    let identity = Identity::select(usb, stock, 0x8852, None, 0, [0, 0]).unwrap();
    // btrtl.c:1196-1209, :310-311.
    assert_eq!(
        identity.firmware_images(),
        [
            ("rtl_bt/rtl8852cu_fw_v2.bin", Container::V2),
            ("rtl_bt/rtl8852cu_fw.bin", Container::V1)
        ]
    );
    assert_eq!(identity.firmware_name(), "rtl_bt/rtl8852cu_fw_v2.bin");
    assert_eq!(identity.configuration_name(), "rtl_bt/rtl8852cu_config.bin");
    assert!(!identity.configuration_required());
    assert_eq!(identity.revision_label(), RTL8852CU_V2);
    assert_eq!(
        identity.revision_label_for(Container::V1),
        RTL8852CU_V2.replacen("epatch-v2", "epatch-v1", 1)
    );
    // The other profiles name their one image, in the container they had.
    for (project, image) in [
        (
            Project::Rtl8761Bu,
            ("rtl_bt/rtl8761bu_fw.bin", Container::V1),
        ),
        (Project::Rtl8822B, ("rtl_bt/rtl8822b_fw.bin", Container::V1)),
        (
            Project::Rtl8852Bu,
            ("rtl_bt/rtl8852bu_fw.bin", Container::V2),
        ),
    ] {
        let (mut hci, catalog, _) = fixture(project, true);
        let report = initialize(&mut hci, &catalog).unwrap();
        assert_eq!(report.before.firmware_images(), [image]);
    }
}
#[test]
fn rtl8852cu_boots_its_v2_image_with_the_linked_empty_config() {
    let (mut hci, catalog, trace) = fixture(Project::Rtl8852Cu, true);
    let provider = Recording {
        catalog: &catalog,
        requests: RefCell::new(vec![]),
    };
    let InitializationReport::Realtek(r) = hci
        .initialize_with_provider(&provider, PlatformInputs::default())
        .unwrap()
    else {
        panic!("Realtek report expected")
    };
    assert_eq!(r.before.project, Project::Rtl8852Cu);
    assert_eq!(r.before.usb.hardware_label(), "usb:13d3:3612:0100");
    assert_eq!(r.after.raw, version(Project::Rtl8852Cu, true)[..]);
    assert_eq!(
        r.container_version,
        ContainerVersion::V2([1, 2, 3, 4, 5, 6, 7, 8])
    );
    let meta = r.v2.as_ref().unwrap();
    assert_eq!(meta.project, Project::Rtl8852Cu);
    assert_eq!(meta.extensions.last().unwrap().value, 25);
    assert_eq!(r.firmware.name, "rtl_bt/rtl8852cu_fw_v2.bin");
    assert_eq!(r.firmware.target_driver, "realtek-bluetooth-usb-epatch-v2");
    assert_eq!(r.firmware.target_revision, RTL8852CU_V2);
    // linux-firmware links the 8852C config to the 8761BU's six bytes,
    // which btrtl appends to the patch (btrtl.c:936-950).
    let ConfigurationEvidence::Supplied(config) = &r.configuration else {
        panic!("supplied config expected")
    };
    assert_eq!(config.name, "rtl_bt/rtl8852cu_config.bin");
    assert_eq!(config.member, "rtl_bt/rtl8761bu_config.bin");
    assert_eq!(
        r.download,
        [DownloadResponse {
            sent_index: 0x80,
            returned_index: 0x42
        }]
    );
    let requests = provider.requests.borrow();
    assert_eq!(
        requests
            .iter()
            .map(|r| (r.0.as_str(), r.3.as_str()))
            .collect::<Vec<_>>(),
        [
            (
                "rtl_bt/rtl8852cu_fw_v2.bin",
                "realtek-bluetooth-usb-epatch-v2"
            ),
            (
                "rtl_bt/rtl8852cu_config.bin",
                "realtek-bluetooth-usb-epatch-v2"
            )
        ]
    );
    drop(requests);
    assert!(hci.io.steps.is_empty());
    assert_eq!(trace.borrow().closed, 0);
}
/// The RTL8852CU's stock-ROM label for its second image, `.bin`.
const RTL8852CU_V1: &str =
    "epatch-v1:hci-0c-000c:lmp-0c-8852:manufacturer-005d:register-8852-none:rom-00:security-00-00";
fn rtl8852cu_v1_target() -> Target<'static> {
    Target {
        driver: "realtek-bluetooth-usb-epatch-v1",
        hardware: "usb:13d3:3612:0100",
        revision: RTL8852CU_V1,
        board: None,
    }
}
/// A catalog resource from the original stateful fixture.
fn resource(
    name: &'static str,
    bytes: Vec<u8>,
    target: Target<'static>,
    member: &'static str,
) -> Resource<'static> {
    Resource {
        name,
        target,
        provenance: Provenance {
            source: "original stateful fixture",
            revision: "model-v1",
            member,
            license: "MIT",
        },
        expected_length: bytes.len(),
        expected_sha256: Sha256::digest(&bytes).into(),
        bytes,
    }
}
#[test]
fn rtl8852cu_falls_back_to_its_v1_image_only_when_its_v2_image_is_missing() {
    let (mut hci, _, _) = fixture(Project::Rtl8852Cu, true);
    let cfg = vec![0x55, 0xab, 0x23, 0x87, 0, 0];
    let catalog = Catalog::from_handoff(vec![
        resource(
            "rtl_bt/rtl8852cu_fw.bin",
            v1_image(Project::Rtl8852Cu),
            rtl8852cu_v1_target(),
            "rtl_bt/rtl8852cu_fw.bin",
        ),
        resource(
            "rtl_bt/rtl8852cu_config.bin",
            cfg.clone(),
            rtl8852cu_v1_target(),
            "rtl_bt/rtl8761bu_config.bin",
        ),
    ])
    .unwrap();
    // The v1 patch ends in its container version, then the config follows.
    let mut data = vec![0x80, 0x78, 0x56, 0x34, 0x12];
    data.extend_from_slice(&cfg);
    hci.io.steps[4].parameters = command(0xfc20, &data);
    let provider = Recording {
        catalog: &catalog,
        requests: RefCell::new(vec![]),
    };
    let r = initialize(&mut hci, &provider).unwrap();
    assert_eq!(r.firmware.name, "rtl_bt/rtl8852cu_fw.bin");
    assert_eq!(r.firmware.target_driver, "realtek-bluetooth-usb-epatch-v1");
    assert_eq!(r.firmware.target_revision, RTL8852CU_V1);
    assert_eq!(r.before.revision_label_for(Container::V1), RTL8852CU_V1);
    assert_eq!(r.container_version, ContainerVersion::V1(0x12345678));
    assert!(r.v2.is_none());
    let fallback = r.fallback.as_ref().unwrap();
    assert_eq!(fallback.name, "rtl_bt/rtl8852cu_fw_v2.bin");
    assert_eq!(fallback.kind, crate::firmware::ErrorKind::Missing);
    let requests = provider.requests.borrow();
    assert_eq!(
        requests
            .iter()
            .map(|r| (r.0.as_str(), r.2.as_str(), r.3.as_str()))
            .collect::<Vec<_>>(),
        [
            (
                "rtl_bt/rtl8852cu_fw_v2.bin",
                RTL8852CU_V2,
                "realtek-bluetooth-usb-epatch-v2"
            ),
            (
                "rtl_bt/rtl8852cu_fw.bin",
                RTL8852CU_V1,
                "realtek-bluetooth-usb-epatch-v1"
            ),
            (
                "rtl_bt/rtl8852cu_config.bin",
                RTL8852CU_V1,
                "realtek-bluetooth-usb-epatch-v1"
            )
        ]
    );
    drop(requests);
    assert!(hci.io.steps.is_empty());
}
#[test]
fn a_present_but_refused_rtl8852cu_v2_image_is_never_replaced_by_its_v1_image() {
    struct Refuse<'a> {
        catalog: &'a Catalog,
        kind: crate::firmware::ErrorKind,
        requests: RefCell<Vec<String>>,
    }
    impl Provider for Refuse<'_> {
        fn load(
            &self,
            r: Requirement<'_>,
        ) -> core::result::Result<Image<'_>, crate::firmware::Error> {
            self.requests.borrow_mut().push(r.name.into());
            if r.name == "rtl_bt/rtl8852cu_fw_v2.bin" {
                return Err(crate::firmware::Error {
                    name: r.name.into(),
                    kind: self.kind.clone(),
                });
            }
            self.catalog.load(r)
        }
    }
    for kind in [
        crate::firmware::ErrorKind::WrongTarget,
        crate::firmware::ErrorKind::DigestMismatch {
            expected: [1; 32],
            actual: [2; 32],
        },
    ] {
        let (mut hci, catalog, trace) = fixture(Project::Rtl8852Cu, true);
        let provider = Refuse {
            catalog: &catalog,
            kind: kind.clone(),
            requests: RefCell::new(vec![]),
        };
        let e = initialize(&mut hci, &provider).unwrap_err();
        let (cause, a) = attempt(&e);
        assert!(
            matches!(cause, Error::Resource(e) if e.name == "rtl_bt/rtl8852cu_fw_v2.bin" && e.kind == kind)
        );
        assert!(a.fallback.is_none());
        assert!(a.download.is_empty());
        assert_eq!(
            *provider.requests.borrow(),
            ["rtl_bt/rtl8852cu_fw_v2.bin", "rtl_bt/rtl8852cu_config.bin"]
        );
        assert_eq!(trace.borrow().commands, 4);
        assert_eq!(hci.state(), State::Closed);
    }
}
#[test]
fn both_missing_rtl8852cu_images_fail_on_the_v1_name_with_the_v2_absence_kept() {
    let (mut hci, _, trace) = fixture(Project::Rtl8852Cu, true);
    let empty = Catalog::from_handoff(vec![]).unwrap();
    let provider = Recording {
        catalog: &empty,
        requests: RefCell::new(vec![]),
    };
    let e = initialize(&mut hci, &provider).unwrap_err();
    let (cause, a) = attempt(&e);
    assert!(matches!(
        cause,
        Error::Resource(e)
            if e.name == "rtl_bt/rtl8852cu_fw.bin" && e.kind == crate::firmware::ErrorKind::Missing
    ));
    assert!(
        a.fallback
            .as_ref()
            .is_some_and(|e| e.name == "rtl_bt/rtl8852cu_fw_v2.bin")
    );
    assert!(a.download.is_empty() && a.v2.is_none());
    let requests = provider.requests.borrow();
    assert_eq!(
        requests
            .iter()
            .map(|r| (r.0.as_str(), r.3.as_str()))
            .collect::<Vec<_>>(),
        [
            (
                "rtl_bt/rtl8852cu_fw_v2.bin",
                "realtek-bluetooth-usb-epatch-v2"
            ),
            ("rtl_bt/rtl8852cu_fw.bin", "realtek-bluetooth-usb-epatch-v1"),
            (
                "rtl_bt/rtl8852cu_config.bin",
                "realtek-bluetooth-usb-epatch-v1"
            )
        ]
    );
    drop(requests);
    assert_eq!(trace.borrow().commands, 4);
    assert_eq!(hci.state(), State::Closed);
}
/// An RTL8852CU a warm reboot left running its patch, as far as its first
/// identification: its register still reads the ROM's subversion and Read
/// Local Version the patch's (lmp_subver 0x5678, no btrtl IC). Then 0xfc66,
/// answered with `events`, and the stock fixture's identification and upload.
fn patched_fixture(events: &[&[u8]]) -> (Hci<Model>, Catalog, Rc<RefCell<Trace>>) {
    let (mut hci, catalog, trace) = fixture(Project::Rtl8852Cu, true);
    let first = [
        Step {
            parameters: command(0xfc61, &[0x10, 0x38, 4, 0x28, 0x80]),
            response: cc(0xfc61, &[0, 0x52, 0x88]),
        },
        Step {
            parameters: command(0x1001, &[]),
            response: cc(0x1001, &version(Project::Rtl8852Cu, true)),
        },
        Step {
            parameters: command(0xfc66, &[]),
            response: events.concat(),
        },
    ];
    for step in first.into_iter().rev() {
        hci.io.steps.push_front(step);
    }
    (hci, catalog, trace)
}
#[test]
fn a_running_patch_is_dropped_once_with_fc66_and_the_stock_rom_then_boots() {
    // The controller is silent after 0xfc66; or it completes it granting no
    // credit, which Linux takes for nothing (btrtl sent it outside the
    // command flow), and sends a vendor event.
    let completion = [0x0e, 4, 0, 0x66, 0xfc, 0];
    let vendor = [0xff, 2, 0x01, 0x02];
    for events in [&[][..], &[&completion[..], &vendor[..]][..]] {
        let (mut hci, catalog, trace) = patched_fixture(events);
        let r = initialize(&mut hci, &catalog).unwrap();
        assert!(hci.io.steps.is_empty(), "every command in order");
        assert_eq!(trace.borrow().commands, 10);
        assert_eq!(r.before.project, Project::Rtl8852Cu);
        assert_eq!(r.before.version.raw, version(Project::Rtl8852Cu, false)[..]);
        let dropped = r.dropped.as_ref().unwrap();
        assert_eq!(dropped.register_subversion, 0x8852);
        assert_eq!(dropped.register_revision, None);
        assert_eq!(dropped.version.raw, version(Project::Rtl8852Cu, true)[..]);
        assert_eq!(dropped.event_count, events.len());
        assert_eq!(
            dropped.events,
            events.iter().map(|e| e.to_vec()).collect::<Vec<_>>()
        );
        assert_eq!(r.download.len(), 1);
    }
}
#[test]
fn readings_matching_no_btrtl_ic_after_the_drop_fail_closed_with_both_kept() {
    let (mut hci, catalog, trace) = patched_fixture(&[]);
    // The same patched readings again, then nothing more is asked.
    hci.io.steps.truncate(4);
    hci.io.steps.push_back(Step {
        parameters: command(0x1001, &[]),
        response: cc(0x1001, &version(Project::Rtl8852Cu, true)),
    });
    let e = initialize(&mut hci, &catalog).unwrap_err();
    let (cause, a) = attempt(&e);
    assert!(matches!(cause, Error::Unsupported(_)));
    assert!(hci.io.steps.is_empty());
    assert_eq!(trace.borrow().commands, 5);
    assert_eq!(
        a.dropped.as_ref().unwrap().version.raw,
        version(Project::Rtl8852Cu, true)[..]
    );
    assert_eq!(
        a.version.as_ref().unwrap().raw,
        version(Project::Rtl8852Cu, true)[..]
    );
    assert!(a.rom_version.is_none() && a.security.is_none() && a.identity.is_none());
    assert_eq!(hci.state(), State::Closed);
}
#[test]
fn readings_of_another_btrtl_ic_are_never_dropped() {
    // An 8852B's stock readings under an 8852CE ID match btrtl's 8852B
    // entry: no 0xfc66, and the USB-pinned profile then refuses them.
    let (mut hci, catalog, trace) = fixture(Project::Rtl8852Cu, true);
    hci.io.steps[1].response = cc(0x1001, &version(Project::Rtl8852Bu, false));
    let e = initialize(&mut hci, &catalog).unwrap_err();
    let (cause, a) = attempt(&e);
    assert!(matches!(cause, Error::Unsupported(_)));
    assert!(a.dropped.is_none());
    assert_eq!(trace.borrow().commands, 4);
}
#[test]
fn btrtls_usb_ic_table_decides_a_drop() {
    let read = |hci: u8, revision: u16, lmp: u16| {
        let mut b = vec![0, hci];
        b.extend_from_slice(&revision.to_le_bytes());
        b.extend_from_slice(&[hci, 0x5d, 0]);
        b.extend_from_slice(&lmp.to_le_bytes());
        LocalVersion::parse(&b).unwrap()
    };
    for (hci, revision, lmp, known) in [
        (0xc, 0xc, 0x8852, true),
        (0xb, 0xb, 0x8852, true),
        (0xc, 0x87, 0x8852, true),
        (0xa, 0xb, 0x8761, true),
        // 8761CU's entry takes any HCI version (btrtl.c:235, :355-357).
        (0x33, 0xe, 0x8761, true),
        (0x6, 0xb, 0x1200, true),
        (0xb, 0xc, 0x8852, false),
        (0xc, 0xc, 0x5678, false),
        // 8822C's USB entry is 0xc/0xa; 0xc/0x8 is UART only.
        (0x8, 0xc, 0x8822, false),
    ] {
        assert_eq!(
            identity::btrtl_ic(lmp, None, &read(hci, revision, lmp)),
            known,
            "{lmp:04x} {revision:04x} {hci:02x}"
        );
    }
    // An 8822E is matched on its registers alone, as hci_ver 0x0c, and no
    // USB entry has it (btrtl.c:1094-1108).
    assert!(!identity::btrtl_ic(
        0x8822,
        Some(0x000e),
        &read(0xc, 0xb, 0x8822)
    ));
    assert!(identity::btrtl_ic(
        0x8822,
        Some(0x000b),
        &read(0x7, 0xb, 0x8822)
    ));
}
#[test]
fn an_event_no_command_awaits_fails_an_undropped_identification_closed() {
    let (mut hci, catalog, trace) = fixture(Project::Rtl8761Bu, true);
    let mut response = vec![0xff, 2, 0x01, 0x02];
    response.extend_from_slice(&hci.io.steps[2].response);
    hci.io.steps[2].response = response;
    let e = initialize(&mut hci, &catalog).unwrap_err();
    assert!(matches!(
        attempt(&e).0,
        Error::Protocol {
            requirement: "unexpected initialization event",
            ..
        }
    ));
    assert!(attempt(&e).1.dropped.is_none());
    assert_eq!(trace.borrow().commands, 3);
    assert_eq!(hci.state(), State::Closed);
}

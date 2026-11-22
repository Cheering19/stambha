//! The replay harness: the guest's function (`Hci::bridged`), the owner's
//! [`Link`] and the scripted controller ([`Chip`]), driven turn by turn as
//! the engine's service turns and the guest's xHCI drive them, with
//! bluetoothd 26B123's recorded start-up and the procedures System Settings
//! runs to list nearby devices.
use super::{
    Cause,
    chip::{self, Chip, Responder},
    link::{Link, Packet},
    policy,
};
use crate::{
    counter::FREQUENCY_HZ,
    usb::{
        BusPeer, Function, Reply, Token,
        control::{
            SET_ADDRESS, SET_CONFIGURATION,
            tests::{setup, transfer},
        },
        hci::{
            EVENTS, Hci,
            capture::{self, Direction, Kind},
            model::Model,
        },
    },
};
use alloc::{string::ToString, vec, vec::Vec};

const ADDRESS: [u8; 6] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x02];
/// A service turn every millisecond.
const TURN: u64 = FREQUENCY_HZ / 1000;
const BRING_UP: &str = include_str!("fixtures/26B123-bring-up.hci");
/// The general inquiry access code, for 2.56 s.
const INQUIRY: [u8; 8] = [0x01, 0x04, 5, 0x33, 0x8b, 0x9e, 2, 0];

/// The commands the guest sent in a capture.
fn commands(text: &str) -> Vec<Vec<u8>> {
    capture::read(text)
        .expect("a capture")
        .into_iter()
        .filter(|p| p.direction == Direction::FromGuest && p.kind == Kind::Command)
        .map(|p| p.bytes)
        .collect()
}

fn opcode(packet: &[u8]) -> u16 {
    u16::from_le_bytes([packet[0], packet[1]])
}

fn packet(opcode: u16, parameters: &[u8]) -> Vec<u8> {
    let mut packet = opcode.to_le_bytes().to_vec();
    packet.push(parameters.len() as u8);
    packet.extend_from_slice(parameters);
    packet
}

/// Whether `event` answers `opcode`.
fn answers(event: &[u8], opcode: u16) -> bool {
    match *event {
        [0x0e, _, _, low, high, ..] | [0x0f, _, _, _, low, high, ..] => {
            u16::from_le_bytes([low, high]) == opcode
        }
        _ => false,
    }
}

/// An answer's event, status, return length and command credits.
fn summary(event: &[u8]) -> (u8, u8, usize, u8) {
    match event[0] {
        0x0e => (0x0e, event[5], event.len() - 6, event[2]),
        _ => (event[0], event[2], 0, event[3]),
    }
}

struct Rig {
    hci: Hci,
    link: Link,
    chip: Chip,
    now: u64,
    failed: bool,
}

impl Rig {
    fn new(chip: Chip) -> Self {
        let mut hci = Hci::bridged(ADDRESS);
        transfer(&mut hci, setup(0, SET_ADDRESS, 1, 0, 0), &[]).unwrap();
        transfer(&mut hci, setup(0, SET_CONFIGURATION, 1, 0, 0), &[]).unwrap();
        Self {
            hci,
            link: Link::default(),
            chip,
            now: 0,
            failed: false,
        }
    }

    /// A rig whose guest ran bluetoothd's start-up.
    fn started(chip: Chip) -> Self {
        let mut rig = Self::new(chip);
        for command in commands(BRING_UP) {
            rig.command(&command);
        }
        assert_eq!(rig.hci.fallback(), None);
        rig
    }

    /// One service turn, as the engine's: what the function queued goes to
    /// the controller, and what the controller has comes back as far as the
    /// guest has room. A failure is told to the function once, and the
    /// controller is left alone after it.
    fn turn(&mut self) {
        self.now += TURN;
        self.chip.now = self.now;
        self.hci.set_time(self.now);
        let outbound: Vec<_> = core::iter::from_fn(|| self.hci.take()).collect();
        if self.failed {
            return;
        }
        self.link.queue(outbound);
        let room = self.hci.room_left();
        let received = self
            .link
            .send(&mut self.chip)
            .and_then(|()| self.link.receive(&mut self.chip, room));
        match received {
            Ok(packets) => {
                for packet in packets {
                    match packet {
                        Packet::Event(event) => self.hci.deliver_event(&event),
                        Packet::Acl(data) => self.hci.deliver_acl(&data),
                        Packet::Diagnostic(_) => {}
                    }
                }
            }
            Err(_) => {
                self.failed = true;
                self.link.clear();
                self.hci.host_failed();
            }
        }
    }

    /// The events the guest reads now, each in its USB packets.
    fn read(&mut self) -> Vec<Vec<u8>> {
        let (mut events, mut event) = (Vec::new(), Vec::new());
        loop {
            let mut buffer = [0; 16];
            let Reply::Ack(n) = self.hci.packet(EVENTS, Token::In, &mut buffer).unwrap() else {
                return events;
            };
            event.extend_from_slice(&buffer[..n]);
            if n < buffer.len() {
                events.push(core::mem::take(&mut event));
            }
        }
    }

    /// bluetoothd sends `command` and the turns run until it is answered:
    /// every event the guest read meanwhile.
    fn command(&mut self, command: &[u8]) -> Vec<Vec<u8>> {
        let op = opcode(command);
        let request = setup(0x20, 0, 0, 0, command.len() as u16);
        transfer(&mut self.hci, request, command).expect("the function takes it");
        let mut events = self.read();
        for _ in 0..3000 {
            if events.iter().any(|e| answers(e, op)) {
                return events;
            }
            self.turn();
            events.extend(self.read());
        }
        panic!("{op:#06x} was never answered");
    }

    /// The events the guest reads over `turns` turns.
    fn run(&mut self, turns: usize) -> Vec<Vec<u8>> {
        let mut events = Vec::new();
        for _ in 0..turns {
            self.turn();
            events.extend(self.read());
        }
        events
    }
}

/// With `--bluetooth-lend` and a controller that answers: bluetoothd's
/// start-up completes, every answer the model's status and length with one
/// credit, Apple's identity kept. Every opcode of the start-up off the list,
/// each vendor one above all, is answered without reaching the controller;
/// the controller's vendor event and credits after its reset reach nobody.
#[test]
fn macoss_start_up_through_a_lent_controller_completes_as_apples_module() {
    let mut chip = Chip::new();
    chip.noise = true;
    let mut rig = Rig::new(chip);
    rig.hci.capture();
    let mut model = Model::new(ADDRESS);
    let start_up = commands(BRING_UP);
    let mut local = 0;
    for command in &start_up {
        let op = opcode(command);
        let sent = rig.chip.received.len();
        let events = rig.command(command);
        let answer = events.iter().find(|e| answers(e, op)).unwrap();
        let expected = model.command(command, 0);
        let (code, status, returns, _) = summary(&expected[0]);
        assert_eq!(summary(answer), (code, status, returns, 1), "{op:#06x}");
        let forwarded = policy::forward(command).is_some();
        assert!(!forwarded || !policy::vendor(op));
        assert_eq!(
            rig.chip.received.len(),
            sent + usize::from(forwarded),
            "{op:#06x}"
        );
        if !forwarded {
            local += 1;
        }
        if [0x1001, 0xfc79].contains(&op) {
            assert_eq!(answer, &expected[0], "the module's identity");
        }
    }
    assert_eq!((local, rig.chip.received.len()), (28, 18));
    assert!(rig.chip.opcodes().iter().all(|&op| !policy::vendor(op)));
    assert_eq!(rig.hci.fallback(), None);
    let counters = rig.hci.bridge_counters().unwrap();
    assert_eq!((counters.forwarded, counters.dropped), (18, 2));
    // The capture holds both sides, and reads back as it was written.
    let captured = rig.hci.take_captured();
    for direction in [Direction::ToChip, Direction::FromChip] {
        assert!(captured.iter().any(|p| p.direction == direction));
    }
    assert!(
        captured
            .iter()
            .filter(|p| p.direction == Direction::ToChip)
            .all(|p| !policy::vendor(opcode(&p.bytes)))
    );
    let lines: Vec<_> = captured.iter().map(ToString::to_string).collect();
    assert_eq!(capture::read(&lines.join("\n")), Ok(captured));
}

/// System Settings' inquiry reaches the controller, and each device's
/// result reaches macOS in the inquiry mode bluetoothd set, in order, then
/// Inquiry Complete.
#[test]
fn an_inquiry_reaches_the_controller_and_each_kind_of_result_reaches_macos() {
    let air = vec![
        Responder::new(1, "Headphones", true),
        Responder::new(2, "Old phone", false),
        Responder::new(3, "Keyboard", true),
    ];
    let mut chip = Chip::new();
    chip.air = air.clone();
    let mut rig = Rig::started(chip);
    // The start-up set Extended Inquiry Result mode.
    for (mode, kinds) in [(2, [0x2f, 0x22, 0x2f]), (1, [0x22; 3]), (0, [0x02; 3])] {
        if mode != 2 {
            rig.command(&packet(0x0c45, &[mode]));
        }
        let mut events = rig.command(&INQUIRY);
        assert_eq!(events[0], [0x0f, 4, 0, 1, 0x01, 0x04], "status, one credit");
        events.extend(rig.run(3000));
        let mut expected: Vec<_> = air.iter().map(|r| r.report(mode)).collect();
        expected.push(vec![0x01, 1, 0]);
        assert_eq!(events[1..], expected, "mode {mode}");
        let codes: Vec<u8> = events[1..4].iter().map(|e| e[0]).collect();
        assert_eq!(codes, kinds);
        // The device's own name is in its extended result.
        if mode == 2 {
            assert_eq!(&events[1][17..29], b"\x0b\x09Headphones");
        }
    }
    assert_eq!(rig.hci.fallback(), None);
    // A cancelled inquiry ends with no more results.
    rig.command(&INQUIRY);
    let cancelled = rig.command(&packet(0x0402, &[]));
    assert!(answers(cancelled.last().unwrap(), 0x0402));
    assert!(rig.run(3000).is_empty());
}

/// A name request reaches the controller, and the device's name, or Page
/// Timeout for no device, reaches macOS.
#[test]
fn a_remote_name_request_reaches_the_controller_and_the_name_reaches_macos() {
    let headphones = Responder::new(1, "Headphones", true);
    let mut chip = Chip::new();
    chip.air = vec![headphones.clone()];
    let mut rig = Rig::started(chip);
    let mut request = headphones.address.to_vec();
    request.extend_from_slice(&[1, 0, 0, 0]);
    let mut events = rig.command(&packet(0x0419, &request));
    assert_eq!(events[0], [0x0f, 4, 0, 1, 0x19, 0x04]);
    events.extend(rig.run(100));
    assert_eq!(events[1..], [headphones.named()]);
    assert_eq!(&events[1][9..19], b"Headphones");
    // No device has this address: Page Timeout after bluetoothd's page
    // timeout, 5.12 s.
    let nobody = [9, 9, 9, 9, 9, 0x02];
    let mut request = nobody.to_vec();
    request.extend_from_slice(&[1, 0, 0, 0]);
    rig.command(&packet(0x0419, &request));
    assert_eq!(rig.run(6000), [chip::name_complete(0x04, nobody, &[])]);
    assert_eq!(rig.hci.fallback(), None);
}

/// The fallback at each step: whichever forwarded command of the start-up
/// the controller leaves unanswered, refuses or fails at, the guest gets the
/// model's answer to it and to everything after, so bluetoothd's start-up
/// completes as on the model alone, and nothing reaches the controller
/// again.
#[test]
fn the_bridge_falls_back_at_any_start_up_step_and_the_start_up_still_completes() {
    let start_up = commands(BRING_UP);
    let mut steps = Vec::new();
    for (index, command) in start_up.iter().enumerate() {
        if policy::forward(command).is_some() {
            let op = opcode(command);
            let nth = start_up[..=index]
                .iter()
                .filter(|c| opcode(c) == op)
                .count();
            steps.push((op, nth));
        }
    }
    assert_eq!(steps.len(), 18);
    for (sent, &(op, nth)) in steps.iter().enumerate() {
        for cause in [Cause::Timeout, Cause::Refused(0x1f), Cause::HostFailed] {
            let mut chip = Chip::new();
            match cause {
                Cause::Timeout => chip.stall = Some((op, nth)),
                Cause::Refused(status) => chip.refuse = Some((op, nth, status)),
                _ => chip.fail = Some((op, nth)),
            }
            let mut rig = Rig::new(chip);
            let mut model = Model::new(ADDRESS);
            for command in &start_up {
                let answer = rig.command(command);
                let answer = answer.iter().find(|e| answers(e, opcode(command))).unwrap();
                let (code, status, returns, _) = summary(&model.command(command, 0)[0]);
                assert_eq!(
                    summary(answer),
                    (code, status, returns, 1),
                    "{op:#06x} {cause:?}: {:#06x}",
                    opcode(command)
                );
            }
            let fallback = rig.hci.fallback().expect("fell back");
            assert_eq!((fallback.opcode, fallback.cause), (Some(op), cause));
            assert_eq!(rig.chip.received.len(), sent + 1, "{op:#06x} {cause:?}");
        }
    }
}

/// Bounded delivery: a poll that gives more results than the guest's queue
/// holds leaves the rest with the owner, and every one reaches the guest, in
/// order, none overflowed.
#[test]
fn a_burst_beyond_the_guests_room_is_held_and_none_is_lost() {
    let air: Vec<_> = (0..100).map(|n| Responder::new(n, "Phone", true)).collect();
    let mut chip = Chip::new();
    chip.air = air.clone();
    chip.response = 0;
    let mut rig = Rig::started(chip);
    let mut events = rig.command(&INQUIRY);
    events.extend(rig.run(3000));
    assert_eq!(events.len(), 1 + 100 + 1);
    for (event, responder) in events[1..].iter().zip(&air) {
        assert_eq!(event, &responder.report(2));
    }
    assert_eq!(rig.hci.counters().overflowed, 0);
}

/// A guest bus reset mid-inquiry: the old inquiry's results stay out until
/// the guest's next HCI Reset is answered, and the controller is never
/// reset by USB.
#[test]
fn a_bus_reset_mid_inquiry_keeps_the_old_results_out() {
    let mut chip = Chip::new();
    chip.air = vec![Responder::new(1, "Headphones", true)];
    chip.response = FREQUENCY_HZ / 2;
    let mut rig = Rig::started(chip);
    rig.command(&INQUIRY);
    rig.hci.reset().unwrap();
    transfer(&mut rig.hci, setup(0, SET_ADDRESS, 1, 0, 0), &[]).unwrap();
    transfer(&mut rig.hci, setup(0, SET_CONFIGURATION, 1, 0, 0), &[]).unwrap();
    assert!(rig.run(3000).is_empty(), "the old inquiry's events");
    assert!(rig.command(&packet(0x0c03, &[])).len() == 1);
    assert_eq!(rig.hci.fallback(), None);
    assert_eq!(rig.chip.opcodes().last(), Some(&0x0c03));
}

/// bluetoothd that keeps restarting its start-up, as it does when it rejects
/// what it was answered: the first restart still reaches the controller,
/// the second is the model's, and the start-up completes on it.
#[test]
fn bluetoothd_restarting_its_start_up_twice_falls_back_to_the_model() {
    let mut rig = Rig::started(Chip::new());
    let mut model = Model::new(ADDRESS);
    for restart in 1..=2 {
        for command in commands(BRING_UP) {
            let answer = rig.command(&command);
            let answer = answer
                .iter()
                .find(|e| answers(e, opcode(&command)))
                .unwrap();
            let (code, status, returns, _) = summary(&model.command(&command, 0)[0]);
            assert_eq!(
                summary(answer),
                (code, status, returns, 1),
                "restart {restart}"
            );
        }
    }
    let fallback = rig.hci.fallback().expect("fell back");
    assert_eq!(
        (fallback.opcode, fallback.cause),
        (Some(0x0c03), Cause::Restarted)
    );
    assert_eq!(
        rig.chip.received.len(),
        2 * 18,
        "the second restart is the model's"
    );
}

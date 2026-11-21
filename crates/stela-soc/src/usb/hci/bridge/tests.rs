use super::super::model::{self, Model};
use super::chip::{self, Chip};
use super::*;

const ADDRESS: [u8; 6] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x02];
/// The host controller's own address, as a test gives it.
const CHIP_ADDRESS: [u8; 6] = [0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0x02];

fn packet(opcode: u16, parameters: &[u8]) -> Vec<u8> {
    let mut packet = opcode.to_le_bytes().to_vec();
    packet.push(parameters.len() as u8);
    packet.extend_from_slice(parameters);
    packet
}

/// A Command Complete with `credits`, `status` and `returns`.
fn complete(opcode: u16, credits: u8, status: u8, returns: &[u8]) -> Vec<u8> {
    let [low, high] = opcode.to_le_bytes();
    let mut event = vec![
        COMMAND_COMPLETE,
        4 + returns.len() as u8,
        credits,
        low,
        high,
        status,
    ];
    event.extend_from_slice(returns);
    event
}

fn status(opcode: u16, credits: u8, status: u8) -> Vec<u8> {
    let [low, high] = opcode.to_le_bytes();
    vec![COMMAND_STATUS, 4, status, credits, low, high]
}

/// The events of `steps`, which must send nothing to the host's controller.
fn events(steps: Vec<Step>) -> Vec<Vec<u8>> {
    steps
        .into_iter()
        .map(|step| match step {
            Step::Guest(event) => event,
            Step::Chip(command) => panic!("{:#06x} went to the chip", command.opcode()),
        })
        .collect()
}

/// The command of `steps`, which must be exactly one for the host's
/// controller.
fn chip(steps: Vec<Step>) -> Vec<u8> {
    match &steps[..] {
        [Step::Chip(command)] => command.bytes().to_vec(),
        _ => panic!("one command for the chip: {steps:02x?}"),
    }
}

#[test]
fn a_forwarded_answer_wins_over_the_shadows_and_has_one_credit() {
    let mut bridge = Bridge::new(ADDRESS);
    let read_address = packet(0x1009, &[]);
    assert_eq!(chip(bridge.command(&read_address, 0)), read_address);
    // The host controller's credits are its own.
    let answer = events(bridge.chip_event(&complete(0x1009, 5, 0, &CHIP_ADDRESS), 10));
    assert_eq!(answer, [complete(0x1009, 1, 0, &CHIP_ADDRESS)]);
    // A procedure answers with Command Status, also with one credit.
    let inquiry = packet(0x0401, &[0x33, 0x8b, 0x9e, 8, 0]);
    assert_eq!(chip(bridge.command(&inquiry, 20)), inquiry);
    assert_eq!(
        events(bridge.chip_event(&status(0x0401, 2, 0), 30)),
        [status(0x0401, 1, 0)]
    );
    assert_eq!(bridge.fallen(), None);
    assert_eq!(bridge.counters().forwarded, 2);
}

#[test]
fn vendor_and_unlisted_commands_are_the_shadows() {
    let mut bridge = Bridge::new(ADDRESS);
    // Read Local Version Information is the module's: Broadcom, Core 4.0.
    let version = events(bridge.command(&packet(0x1001, &[]), 0));
    assert_eq!(version[0][5..], [0, 6, 0, 0, 6, 15, 0, 0x0e, 0x41]);
    // Write BD_ADDR is the module's vendor command.
    let written = events(bridge.command(&packet(0xfc01, &[9, 8, 7, 6, 5, 0x42]), 0));
    assert_eq!(written, [complete(0xfc01, 1, 0, &[])]);
    assert_eq!(bridge.address(), [9, 8, 7, 6, 5, 0x42]);
    // Every vendor opcode, of every parameter length the guest might send.
    for ocf in 0..0x400u16 {
        let opcode = 0x3f << 10 | ocf;
        for length in [0, 1, 9] {
            assert!(policy::forward(&packet(opcode, &vec![0; length])).is_none());
            for step in bridge.command(&packet(opcode, &vec![0; length]), 0) {
                assert!(
                    matches!(step, Step::Guest(_)),
                    "{opcode:#06x} reached the chip"
                );
            }
        }
    }
    // A standard command not on the list.
    let scan = events(bridge.command(&packet(0x0c1a, &[3]), 0));
    assert_eq!(scan, [complete(0x0c1a, 1, 0, &[])]);
    assert_eq!(bridge.counters().forwarded, 0);
    assert_eq!(bridge.fallen(), None);
}

/// The list holds standard commands the model implements and lists, each
/// with the bit bluetoothd reads for it, once.
#[test]
fn the_forwarded_commands_are_standard_ones_the_model_lists() {
    let bitmap = super::super::model::supported_commands();
    for (index, &(opcode, octet, bit, _)) in policy::FORWARDED.iter().enumerate() {
        assert!(!policy::vendor(opcode));
        assert!(super::super::model::implements(opcode), "{opcode:#06x}");
        assert_ne!(bitmap[usize::from(octet)] & 1 << bit, 0, "{opcode:#06x}");
        for &(other, other_octet, other_bit, _) in &policy::FORWARDED[index + 1..] {
            assert_ne!(other, opcode);
            assert_ne!((other_octet, other_bit), (octet, bit), "{opcode:#06x}");
        }
    }
}

#[test]
fn one_command_is_in_flight_and_the_next_waits_its_turn() {
    let mut bridge = Bridge::new(ADDRESS);
    let reset = packet(0x0c03, &[]);
    assert_eq!(chip(bridge.command(&reset, 0)), reset);
    // A local command and a forwarded one wait behind it.
    assert!(bridge.command(&packet(0x1001, &[]), 1).is_empty());
    let mask = packet(0x0c01, &[0xff; 8]);
    assert!(bridge.command(&mask, 2).is_empty());
    let steps = bridge.chip_event(&complete(0x0c03, 1, 0, &[]), 3);
    assert_eq!(steps.len(), 3);
    assert_eq!(steps[0], Step::Guest(complete(0x0c03, 1, 0, &[])));
    assert!(matches!(&steps[1], Step::Guest(e) if e[3..5] == [0x01, 0x10]));
    assert!(matches!(&steps[2], Step::Chip(c) if c.bytes() == mask));
    assert!(bridge.has_room());
}

#[test]
fn credits_alone_and_stale_answers_are_dropped() {
    let mut bridge = Bridge::new(ADDRESS);
    // Credits alone (opcode 0, no status), before and during a command.
    assert!(
        bridge
            .chip_event(&[COMMAND_COMPLETE, 3, 1, 0, 0], 0)
            .is_empty()
    );
    let local_name = packet(0x0c14, &[]);
    chip(bridge.command(&local_name, 0));
    assert!(
        bridge
            .chip_event(&[COMMAND_COMPLETE, 3, 1, 0, 0], 0)
            .is_empty()
    );
    assert!(bridge.chip_event(&status(0x0000, 1, 0), 0).is_empty());
    // An answer to another command than the one in flight.
    assert!(
        bridge
            .chip_event(&complete(0x0c03, 1, 0, &[]), 0)
            .is_empty()
    );
    let answer = events(bridge.chip_event(&complete(0x0c14, 1, 0, &[0; 248]), 0));
    assert_eq!(answer.len(), 1);
    assert_eq!(bridge.counters().dropped, 4);
    assert_eq!(bridge.fallen(), None);
}

#[test]
fn a_deadline_gives_the_shadows_answer_and_falls_back_for_good() {
    let mut bridge = Bridge::new(ADDRESS);
    chip(bridge.command(&packet(0x0c03, &[]), 100));
    assert_eq!(bridge.deadline(), Some(100 + DEADLINE));
    assert!(bridge.advance(100 + DEADLINE - 1).is_empty());
    let answer = events(bridge.advance(100 + DEADLINE));
    assert_eq!(answer, [complete(0x0c03, 1, 0, &[])]);
    assert_eq!(
        bridge.fallen(),
        Some(Fallback {
            opcode: Some(0x0c03),
            cause: Cause::Timeout,
        })
    );
    assert_eq!(
        bridge.fallen().unwrap().to_string(),
        "0x0c03 no answer in 1 s"
    );
    // The late answer, and every event after, is dropped.
    assert!(
        bridge
            .chip_event(&complete(0x0c03, 1, 0, &[]), 0)
            .is_empty()
    );
    assert!(bridge.chip_event(&[0x01, 1, 0], 0).is_empty());
    // Nothing reaches the host's controller again; the shadow answers.
    let address = events(bridge.command(&packet(0x1009, &[]), 0));
    assert_eq!(address, [complete(0x1009, 1, 0, &ADDRESS)]);
    // The shadow's procedures end with their events now.
    let inquiry = events(bridge.command(&packet(0x0401, &[0x33, 0x8b, 0x9e, 1, 0]), 0));
    assert_eq!(inquiry, [status(0x0401, 1, 0)]);
    let end = bridge.deadline().expect("the inquiry ends");
    assert_eq!(events(bridge.advance(end)), [vec![0x01, 1, 0]]);
}

#[test]
fn an_error_the_model_would_not_give_falls_back_and_a_procedures_does_not() {
    let mut bridge = Bridge::new(ADDRESS);
    // Inquiry Cancel with no inquiry: the radio's outcome, passed on.
    chip(bridge.command(&packet(0x0402, &[]), 0));
    let answer = events(bridge.chip_event(&complete(0x0402, 1, 0x0c, &[]), 0));
    assert_eq!(answer, [complete(0x0402, 1, 0x0c, &[])]);
    // An error both give is the controller's.
    let bad_mode = packet(0x0c45, &[7]);
    chip(bridge.command(&bad_mode, 0));
    let answer = events(bridge.chip_event(&complete(0x0c45, 1, 0x12, &[]), 0));
    assert_eq!(answer, [complete(0x0c45, 1, 0x12, &[])]);
    assert_eq!(bridge.fallen(), None);
    // Set Event Mask refused where the model takes it.
    let mask = packet(0x0c01, &[0xff; 8]);
    chip(bridge.command(&mask, 0));
    let answer = events(bridge.chip_event(&complete(0x0c01, 1, 0x1f, &[]), 0));
    assert_eq!(answer, [complete(0x0c01, 1, 0, &[])]);
    assert_eq!(
        bridge.fallen(),
        Some(Fallback {
            opcode: Some(0x0c01),
            cause: Cause::Refused(0x1f),
        })
    );
}

#[test]
fn an_answer_of_another_kind_or_length_falls_back() {
    for wrong in [
        // Read BD_ADDR with five bytes.
        complete(0x1009, 1, 0, &[1, 2, 3, 4, 5]),
        // Command Status for a command that completes.
        status(0x1009, 1, 0),
    ] {
        let mut bridge = Bridge::new(ADDRESS);
        chip(bridge.command(&packet(0x1009, &[]), 0));
        let answer = events(bridge.chip_event(&wrong, 0));
        assert_eq!(answer, [complete(0x1009, 1, 0, &ADDRESS)]);
        assert_eq!(bridge.fallen().unwrap().cause, Cause::Malformed);
    }
}

#[test]
fn a_failed_host_controller_falls_back_with_the_answer_in_flight() {
    let mut bridge = Bridge::new(ADDRESS);
    chip(bridge.command(&packet(0x0c14, &[]), 0));
    assert!(bridge.command(&packet(0x0c03, &[]), 0).is_empty());
    let answers = events(bridge.chip_failed(0));
    assert_eq!(
        answers.len(),
        2,
        "the command in flight and the one waiting"
    );
    assert_eq!(answers[0][2..6], [1, 0x14, 0x0c, 0]);
    assert_eq!(answers[1], complete(0x0c03, 1, 0, &[]));
    assert_eq!(bridge.fallen().unwrap().cause, Cause::HostFailed);
    assert!(bridge.chip_failed(0).is_empty());
    // Between commands it falls back all the same.
    let mut bridge = Bridge::new(ADDRESS);
    assert!(bridge.chip_failed(0).is_empty());
    assert_eq!(
        bridge.fallen().unwrap().to_string(),
        "between commands: the host controller failed"
    );
    events(bridge.command(&packet(0x0c03, &[]), 0));
}

/// bluetoothd 26B123's start-up, the guest's side.
const BRING_UP: &str = include_str!("fixtures/26B123-bring-up.hci");

/// The commands the guest sent in a capture.
fn commands(capture: &str) -> Vec<Vec<u8>> {
    use super::super::capture::{Direction, Kind, read};
    read(capture)
        .expect("a capture")
        .into_iter()
        .filter(|p| p.direction == Direction::FromGuest && p.kind == Kind::Command)
        .map(|p| p.bytes)
        .collect()
}

/// One guest command through `bridge`, `chip` answering at once what
/// reaches it, until nothing is left to carry: the guest's events.
fn exchange(bridge: &mut Bridge, chip: &mut Chip, command: &[u8]) -> Vec<Vec<u8>> {
    let mut steps: VecDeque<Step> = bridge.command(command, 0).into();
    let mut events = Vec::new();
    while let Some(step) = steps.pop_front() {
        match step {
            Step::Guest(event) => events.push(event),
            Step::Chip(command) => {
                for event in chip.command(command.bytes()) {
                    steps.extend(bridge.chip_event(&event, 0));
                }
            }
        }
    }
    events
}

/// The return parameters of the one successful Command Complete of `events`.
fn returns(events: &[Vec<u8>]) -> Vec<u8> {
    assert_eq!(events.len(), 1, "{events:02x?}");
    let completion = Completion::of(&events[0]).expect("a completion");
    assert_eq!((completion.code, completion.status), (COMMAND_COMPLETE, 0));
    completion.returns.to_vec()
}

/// The differential replay: bluetoothd's whole start-up through the bridge
/// and a controller that has everything the model lists gets, at every
/// step, the pure model's event, status and return length, and the same
/// returns but for the controller's own address and ACL buffers. Only the
/// listed standard commands reach the controller, in order.
#[test]
fn bluetoothds_start_up_through_the_bridge_answers_as_the_model_does() {
    let mut model = Model::new(ADDRESS);
    let mut bridge = Bridge::new(ADDRESS);
    let mut chip = Chip::new();
    let start_up = commands(BRING_UP);
    assert_eq!(start_up.len(), 46);
    for command in &start_up {
        let op = opcode(command);
        let expected = model.command(command, 0);
        let got = exchange(&mut bridge, &mut chip, command);
        assert_eq!(got.len(), expected.len(), "{op:#06x}");
        for (got, expected) in got.iter().zip(&expected) {
            let (g, e) = (
                Completion::of(got).unwrap(),
                Completion::of(expected).unwrap(),
            );
            assert_eq!(
                (g.code, g.opcode, g.status, g.returns.len()),
                (e.code, e.opcode, e.status, e.returns.len()),
                "{op:#06x}"
            );
            assert_eq!(got[2 + usize::from(g.code == COMMAND_STATUS)], 1);
            match op {
                0x1009 => assert_eq!(g.returns, chip::ADDRESS),
                0x1005 => assert_eq!(g.returns, [0xfd, 0x03, 0, 6, 0, 0, 0]),
                _ => assert_eq!(got, expected, "{op:#06x}"),
            }
        }
    }
    assert_eq!(bridge.fallen(), None);
    assert_eq!(
        chip.opcodes(),
        [
            0x0c03, 0x1002, 0x1009, 0x1003, 0x2003, 0x0c01, 0x2001, 0x0c56, 0x0c6d, 0x1005, 0x0c5b,
            0x0c24, 0x0c23, 0x0c13, 0x0c14, 0x0c18, 0x0c45, 0x0c52,
        ]
    );
    // The module's identity stays: Core 4.0 Broadcom, chip id 73, its IDs.
    let version = returns(&exchange(&mut bridge, &mut chip, &packet(0x1001, &[])));
    assert_eq!(version, [6, 0, 0, 6, 15, 0, 0x0e, 0x41]);
    let verbose = returns(&exchange(&mut bridge, &mut chip, &packet(0xfc79, &[])));
    assert_eq!(verbose[0], 73);
    let product = returns(&exchange(&mut bridge, &mut chip, &packet(0xfc5a, &[])));
    assert_eq!(product, [0xac, 0x05, 0x8f, 0x82]);
    assert_eq!(chip.opcodes().len(), 18);
}

/// The capabilities the guest reads are the model's, less what the
/// controller lacks of the commands it is sent, and never more.
#[test]
fn the_merge_keeps_the_modules_capabilities_less_what_the_controller_lacks() {
    let mut chip = Chip::new();
    // No Write Default Erroneous Data Reporting (octet 18 bit 3) or Write
    // Inquiry Mode (12, 7), both forwarded; no Write Scan Enable (7, 7) or
    // Host Number Of Completed Packets (10, 7), both the shadow's.
    for (octet, bit) in [(18, 3), (12, 7), (7, 7), (10, 7)] {
        chip.commands[octet] &= !(1 << bit);
    }
    // No extended inquiry response (feature bit 48).
    chip.features[6] &= !1;
    // ACL longer than the transport carries.
    chip.buffers[..2].copy_from_slice(&0x2000u16.to_le_bytes());
    let mut bridge = Bridge::new(ADDRESS);
    let model = model::supported_commands();
    let bitmap = returns(&exchange(&mut bridge, &mut chip, &packet(0x1002, &[])));
    for octet in 0..64 {
        let lacking = match octet {
            18 => 1 << 3,
            12 => 1 << 7,
            _ => 0,
        };
        assert_eq!(bitmap[octet], model[octet] & !lacking, "octet {octet}");
    }
    // Features: the model's, less what the controller lacks.
    let features = returns(&exchange(&mut bridge, &mut chip, &packet(0x1003, &[])));
    let expected = returns(&Model::new(ADDRESS).command(&packet(0x1003, &[]), 0));
    assert_ne!(expected[6] & 1, 0, "the model has it");
    assert_eq!(features[6], expected[6] & !1);
    for ((feature, model), chip) in features.iter().zip(&expected).zip(chip.features) {
        assert_eq!(*feature, model & chip);
    }
    // Extended features: the model's page numbers, maximum 1; page 1 as
    // both have it; page 2 the module's, empty, whatever the controller has.
    for page in [0, 1] {
        let extended = returns(&exchange(&mut bridge, &mut chip, &packet(0x1004, &[page])));
        assert_eq!(extended[..2], [page, 1]);
    }
    let page_2 = returns(&exchange(&mut bridge, &mut chip, &packet(0x1004, &[2])));
    assert_eq!(page_2, [2, 1, 0, 0, 0, 0, 0, 0, 0, 0]);
    assert!(
        chip.received
            .iter()
            .all(|c| opcode(c) != 0x1004 || c[3] < 2),
        "later pages stay the module's"
    );
    // Buffers: the controller's ACL buffers, as long as the transport
    // carries at most; no synchronous ones.
    let buffers = returns(&exchange(&mut bridge, &mut chip, &packet(0x1005, &[])));
    assert_eq!(buffers, [0x00, 0x10, 0, 6, 0, 0, 0]);
    // LE features: none in milestone 1, whatever the controller has.
    let le = returns(&exchange(&mut bridge, &mut chip, &packet(0x2003, &[])));
    assert_eq!(le, [0; 8]);
    assert_eq!(bridge.fallen(), None);
}

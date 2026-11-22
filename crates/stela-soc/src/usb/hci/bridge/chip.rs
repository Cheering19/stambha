//! A scripted stand-in for the host's controller: a standard Bluetooth
//! controller with an RTL8852C's identity after its patch, as Linux
//! `drivers/bluetooth/btrtl.c` and the Core specification describe one. The
//! bitmap, feature pages and buffer sizes are typical values, not ones read
//! from Garvit's chip (UNVERIFIED until a host check prints them). It
//! answers every standard command it lists with its specified return
//! lengths, records every command it is sent, and answers a vendor command,
//! which the bridge must never send, Unknown HCI Command.
//!
//! Its air holds the devices in range ([`Responder`]): an inquiry reports
//! each in the inquiry mode the host set, then completes; a remote name
//! request gives a device's name, or Page Timeout for an address no device
//! has. It can also stall at a command, refuse one, fail as a transport, and
//! send what Realtek firmware may after a reset (a vendor event and credits
//! alone). It implements [`HostController`], with the test's clock.
use super::link::{HostController, Packet};
use crate::{counter::FREQUENCY_HZ, usb::hci::Outbound};
use alloc::{string::String, vec, vec::Vec};

const COMMAND_COMPLETE: u8 = 0x0e;
const COMMAND_STATUS: u8 = 0x0f;
const INQUIRY_COMPLETE: u8 = 0x01;
const INQUIRY_RESULT: u8 = 0x02;
const REMOTE_NAME_REQUEST_COMPLETE: u8 = 0x07;
const INQUIRY_RESULT_WITH_RSSI: u8 = 0x22;
const EXTENDED_INQUIRY_RESULT: u8 = 0x2f;
const SUCCESS: u8 = 0x00;
const UNKNOWN_COMMAND: u8 = 0x01;
const PAGE_TIMEOUT: u8 = 0x04;
/// System counts in a baseband slot, 625 us, and an inquiry length unit.
const SLOT: u64 = FREQUENCY_HZ * 625 / 1_000_000;
const INQUIRY_UNIT: u64 = 2048 * SLOT;
/// How long after an inquiry starts each device answers, in turn.
pub const RESPONSE: u64 = FREQUENCY_HZ / 100;

/// The controller's address in the tests.
pub const ADDRESS: [u8; 6] = [0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0x02];
/// Read Local Version Information: Core 5.3 (12), Realtek (93).
pub const VERSION: [u8; 8] = [0x0c, 0x0c, 0x00, 0x0c, 0x5d, 0x00, 0x52, 0x88];
/// LMP features page 0 of a Realtek combo controller: everything the model
/// lists and more (encryption, EDR, eSCO, sniff, role switch).
pub const FEATURES: [u8; 8] = [0xff, 0xff, 0xff, 0xfe, 0xdb, 0xfd, 0x7b, 0x87];
/// Page 2: Secure Connections and more, which the bridge hides.
pub const FEATURES_2: [u8; 8] = [0x0f, 0x03, 0, 0, 0, 0, 0, 0];
/// LE features: encryption, extended advertising and more.
pub const LE_FEATURES: [u8; 8] = [0xff, 0x59, 0x01, 0, 0, 0, 0, 0];
/// Read Buffer Size: ACL 1021 x 6, SCO 255 x 12.
pub const BUFFERS: [u8; 7] = [0xfd, 0x03, 0xff, 0x06, 0x00, 0x0c, 0x00];
/// What Realtek firmware may send after a reset: a vendor event and a
/// Command Complete that only gives credits.
pub const VENDOR_EVENT: [u8; 4] = [0xff, 2, 0x34, 0x01];
pub const CREDITS_ALONE: [u8; 5] = [COMMAND_COMPLETE, 3, 1, 0, 0];

/// Commands that read what a write stored: (read, write, power-on value).
const SETTINGS: &[(u16, u16, &[u8])] = &[
    (0x080e, 0x080f, &[0, 0]),
    (0x0c15, 0x0c16, &[0x40, 0x1f]),
    (0x0c17, 0x0c18, &[0x00, 0x20]),
    (0x0c19, 0x0c1a, &[0]),
    (0x0c1b, 0x0c1c, &[0x00, 0x08, 0x12, 0x00]),
    (0x0c1d, 0x0c1e, &[0x00, 0x10, 0x12, 0x00]),
    (0x0c1f, 0x0c20, &[0]),
    (0x0c23, 0x0c24, &[0, 0, 0]),
    (0x0c25, 0x0c26, &[0x60, 0x00]),
    (0x0c42, 0x0c43, &[0]),
    (0x0c44, 0x0c45, &[0]),
    (0x0c46, 0x0c47, &[0]),
    (0x0c55, 0x0c56, &[0]),
    (0x0c5a, 0x0c5b, &[0]),
    (0x0c6c, 0x0c6d, &[0, 0]),
];

/// A device in range that answers inquiries and name requests.
#[derive(Debug, Clone)]
pub struct Responder {
    pub address: [u8; 6],
    pub class: [u8; 3],
    pub clock_offset: u16,
    pub rssi: i8,
    /// Its extended inquiry response, 240 bytes; none for a pre-2.1 device.
    pub eir: Option<Vec<u8>>,
    pub name: Vec<u8>,
}

impl Responder {
    /// A device whose EIR names it `name`, or with none.
    pub fn new(last: u8, name: &str, eir: bool) -> Self {
        let mut eir_data = vec![name.len() as u8 + 1, 0x09];
        eir_data.extend_from_slice(name.as_bytes());
        eir_data.resize(240, 0);
        Self {
            address: [last, 0x11, 0x22, 0x33, 0x44, 0x02],
            class: [0x04, 0x04, 0x24],
            clock_offset: 0x1234 + u16::from(last),
            rssi: -40 - (last % 50) as i8,
            eir: eir.then_some(eir_data),
            name: name.as_bytes().to_vec(),
        }
    }

    /// Its report in inquiry mode `mode` (Vol 4 Part E 7.7.2, 7.7.33,
    /// 7.7.38).
    pub fn report(&self, mode: u8) -> Vec<u8> {
        let [clock_low, clock_high] = self.clock_offset.to_le_bytes();
        let mut p = vec![1];
        p.extend_from_slice(&self.address);
        match (mode, &self.eir) {
            (0, _) => {
                p.extend_from_slice(&[1, 0, 0]);
                p.extend_from_slice(&self.class);
                p.extend_from_slice(&[clock_low, clock_high]);
                event(INQUIRY_RESULT, &p)
            }
            (_, eir) => {
                p.extend_from_slice(&[1, 0]);
                p.extend_from_slice(&self.class);
                p.extend_from_slice(&[clock_low, clock_high, self.rssi as u8]);
                match eir {
                    Some(eir) if mode == 2 => {
                        p.extend_from_slice(eir);
                        event(EXTENDED_INQUIRY_RESULT, &p)
                    }
                    _ => event(INQUIRY_RESULT_WITH_RSSI, &p),
                }
            }
        }
    }

    /// Its Remote Name Request Complete (7.7.7).
    pub fn named(&self) -> Vec<u8> {
        name_complete(SUCCESS, self.address, &self.name)
    }
}

fn event(code: u8, parameters: &[u8]) -> Vec<u8> {
    let mut event = vec![code, parameters.len() as u8];
    event.extend_from_slice(parameters);
    event
}

pub fn name_complete(status: u8, address: [u8; 6], name: &[u8]) -> Vec<u8> {
    let mut p = vec![status];
    p.extend_from_slice(&address);
    p.extend_from_slice(name);
    p.resize(1 + 6 + 248, 0);
    event(REMOTE_NAME_REQUEST_COMPLETE, &p)
}

/// What a scheduled event belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tag {
    Answer,
    Inquiry,
    Name,
}

/// The stand-in controller.
#[derive(Debug, Clone)]
pub struct Chip {
    /// Every command it was sent, in order, and its ACL data.
    pub received: Vec<Vec<u8>>,
    pub acl: Vec<Vec<u8>>,
    /// Supported Commands.
    pub commands: [u8; 64],
    /// The credits each answer gives.
    pub credits: u8,
    /// LMP features page 0, and Read Buffer Size's return parameters.
    pub features: [u8; 8],
    pub buffers: [u8; 7],
    /// The devices in range, and how long after each other they answer an
    /// inquiry.
    pub air: Vec<Responder>,
    pub response: u64,
    /// The system count now, which the test sets.
    pub now: u64,
    /// A command it never answers, from the given time it is sent (1: the
    /// first).
    pub stall: Option<(u16, usize)>,
    /// A command it answers with this error status, its return parameters
    /// kept.
    pub refuse: Option<(u16, usize, u8)>,
    /// A command at which its transport fails.
    pub fail: Option<(u16, usize)>,
    failed: bool,
    /// Whether it sends [`VENDOR_EVENT`] and [`CREDITS_ALONE`] after each
    /// reset.
    pub noise: bool,
    scheduled: Vec<(u64, Tag, Vec<u8>)>,
    /// Its settings since Reset: (write opcode, parameters).
    written: Vec<(u16, Vec<u8>)>,
    local_name: Vec<u8>,
    eir: Vec<u8>,
}

impl Chip {
    pub fn new() -> Self {
        // Every command of Core 5.3's first 42 octets.
        let mut commands = [0; 64];
        commands[..42].fill(0xff);
        Self {
            received: Vec::new(),
            acl: Vec::new(),
            commands,
            credits: 2,
            features: FEATURES,
            buffers: BUFFERS,
            air: Vec::new(),
            response: RESPONSE,
            now: 0,
            stall: None,
            refuse: None,
            fail: None,
            failed: false,
            noise: false,
            scheduled: Vec::new(),
            written: Vec::new(),
            local_name: vec![0; 248],
            eir: vec![0; 241],
        }
    }

    /// Opcodes of the commands it was sent.
    pub fn opcodes(&self) -> Vec<u16> {
        self.received
            .iter()
            .map(|c| u16::from_le_bytes([c[0], c[1]]))
            .collect()
    }

    /// How many times it was sent `opcode`, this one included.
    fn times(&self, opcode: u16) -> usize {
        self.opcodes().into_iter().filter(|&o| o == opcode).count()
    }

    fn complete(&self, opcode: u16, status: u8, returns: &[u8]) -> Vec<u8> {
        let [low, high] = opcode.to_le_bytes();
        let mut event = vec![
            COMMAND_COMPLETE,
            4 + returns.len() as u8,
            self.credits,
            low,
            high,
        ];
        event.push(status);
        event.extend_from_slice(returns);
        event
    }

    fn status(&self, opcode: u16, status: u8) -> Vec<u8> {
        let [low, high] = opcode.to_le_bytes();
        vec![COMMAND_STATUS, 4, status, self.credits, low, high]
    }

    fn stored(&self, write: u16, default: &[u8]) -> Vec<u8> {
        self.written
            .iter()
            .rev()
            .find(|(opcode, _)| *opcode == write)
            .map_or_else(|| default.to_vec(), |(_, p)| p.clone())
    }

    fn ok(&self, opcode: u16, returns: &[u8]) -> Vec<Vec<u8>> {
        vec![self.complete(opcode, SUCCESS, returns)]
    }

    fn schedule(&mut self, after: u64, tag: Tag, event: Vec<u8>) {
        self.scheduled.push((self.now + after, tag, event));
    }

    /// One command; its answer at once, its procedure's events later.
    pub fn command(&mut self, packet: &[u8]) -> Vec<Vec<u8>> {
        self.received.push(packet.to_vec());
        let opcode = u16::from_le_bytes([packet[0], packet[1]]);
        let times = self.times(opcode);
        if self.stall == Some((opcode, times)) {
            return Vec::new();
        }
        let scheduled = self.scheduled.len();
        let mut answer = self.answer(opcode, &packet[3..]);
        if let Some((_, _, status)) = self.refuse.filter(|r| (r.0, r.1) == (opcode, times)) {
            match answer[0][0] {
                COMMAND_STATUS => answer[0][2] = status,
                _ => answer[0][5] = status,
            }
            // A refused procedure does not run.
            self.scheduled.truncate(scheduled);
        }
        answer
    }

    fn answer(&mut self, opcode: u16, p: &[u8]) -> Vec<Vec<u8>> {
        let p = p.to_vec();
        if let Some(&(_, write, default)) = SETTINGS.iter().find(|s| s.0 == opcode) {
            return self.ok(opcode, &self.stored(write, default));
        }
        if SETTINGS.iter().any(|s| s.1 == opcode) {
            self.written.push((opcode, p));
            return self.ok(opcode, &[]);
        }
        match opcode {
            0x0401 => {
                let mode = self.stored(0x0c45, &[0])[0];
                let reports: Vec<_> = self.air.iter().map(|r| r.report(mode)).collect();
                for (at, report) in (1..).zip(reports) {
                    self.schedule(at * self.response, Tag::Inquiry, report);
                }
                let end = u64::from(p[3]) * INQUIRY_UNIT;
                self.schedule(end, Tag::Inquiry, vec![INQUIRY_COMPLETE, 1, SUCCESS]);
                vec![self.status(opcode, SUCCESS)]
            }
            0x0402 => {
                self.scheduled.retain(|s| s.1 != Tag::Inquiry);
                self.ok(opcode, &[])
            }
            0x0419 => {
                let peer: [u8; 6] = p[..6].try_into().expect("an address");
                let named = self
                    .air
                    .iter()
                    .find(|r| r.address == peer)
                    .map(|r| r.named());
                match named {
                    Some(named) => self.schedule(2 * RESPONSE, Tag::Name, named),
                    None => {
                        let timeout = u16::from_le_bytes(
                            self.stored(0x0c18, &[0, 0x20])[..2]
                                .try_into()
                                .expect("two bytes"),
                        );
                        let complete = name_complete(PAGE_TIMEOUT, peer, &[]);
                        self.schedule(u64::from(timeout) * SLOT, Tag::Name, complete);
                    }
                }
                vec![self.status(opcode, SUCCESS)]
            }
            0x041a => {
                self.scheduled.retain(|s| s.1 != Tag::Name);
                self.ok(opcode, &p[..6])
            }
            0x0c01 | 0x0c63 | 0x2001 => self.ok(opcode, &[]),
            0x0c03 => {
                self.written.clear();
                self.scheduled.clear();
                self.local_name = vec![0; 248];
                self.eir = vec![0; 241];
                let mut events = self.ok(opcode, &[]);
                if self.noise {
                    events.extend([VENDOR_EVENT.to_vec(), CREDITS_ALONE.to_vec()]);
                }
                events
            }
            0x0c13 => {
                self.local_name = p;
                self.ok(opcode, &[])
            }
            0x0c14 => self.ok(opcode, &self.local_name),
            0x0c51 => self.ok(opcode, &self.eir),
            0x0c52 => {
                self.eir = p;
                self.ok(opcode, &[])
            }
            0x0c58 => self.ok(opcode, &[0x04]),
            0x1001 => self.ok(opcode, &VERSION),
            0x1002 => self.ok(opcode, &self.commands),
            0x1003 => self.ok(opcode, &self.features),
            0x1004 => {
                let features = match p[0] {
                    0 => self.features,
                    1 => {
                        let ssp = self.stored(0x0c56, &[0])[0];
                        let le = self.stored(0x0c6d, &[0, 0])[0];
                        [ssp | le << 1, 0, 0, 0, 0, 0, 0, 0]
                    }
                    2 => FEATURES_2,
                    _ => [0; 8],
                };
                let mut r = vec![p[0], 2];
                r.extend_from_slice(&features);
                self.ok(opcode, &r)
            }
            0x1005 => self.ok(opcode, &self.buffers),
            0x1009 => self.ok(opcode, &ADDRESS),
            0x2002 => self.ok(opcode, &[0xfb, 0x00, 0x08]),
            0x2003 => self.ok(opcode, &LE_FEATURES),
            _ => vec![self.complete(opcode, UNKNOWN_COMMAND, &[])],
        }
    }
}

impl HostController for Chip {
    fn send(&mut self, packet: &Outbound) -> Result<bool, String> {
        if self.failed {
            return Err(String::from("the scripted controller failed"));
        }
        match packet {
            Outbound::Command(bytes) => {
                let opcode = u16::from_le_bytes([bytes[0], bytes[1]]);
                if self.fail == Some((opcode, self.times(opcode) + 1)) {
                    self.received.push(bytes.clone());
                    self.failed = true;
                    return Err(String::from("the scripted controller failed"));
                }
                for event in self.command(bytes) {
                    self.schedule(0, Tag::Answer, event);
                }
            }
            Outbound::Acl(bytes) => self.acl.push(bytes.clone()),
        }
        Ok(true)
    }

    fn poll(&mut self) -> Result<Vec<Packet>, String> {
        if self.failed {
            return Err(String::from("the scripted controller failed"));
        }
        // In time order, a command's answer before its procedure's events
        // at the same time, other ties as scheduled.
        self.scheduled.sort_by_key(|s| (s.0, s.1 != Tag::Answer));
        let due = self
            .scheduled
            .iter()
            .take_while(|s| s.0 <= self.now)
            .count();
        Ok(self
            .scheduled
            .drain(..due)
            .map(|(_, _, event)| Packet::Event(event))
            .collect())
    }
}

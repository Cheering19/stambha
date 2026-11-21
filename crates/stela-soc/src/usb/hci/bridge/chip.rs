//! A scripted stand-in for the host's controller: a standard Bluetooth
//! controller with an RTL8852C's identity after its patch, as Linux
//! `drivers/bluetooth/btrtl.c` and the Core specification describe one. The
//! bitmap, feature pages and buffer sizes are typical values, not ones read
//! from Garvit's chip (UNVERIFIED until a host check prints them). It
//! answers every standard command it lists with its specified return
//! lengths, records every command it is sent, and answers a vendor command,
//! which the bridge must never send, Unknown HCI Command.
use alloc::{vec, vec::Vec};

const COMMAND_COMPLETE: u8 = 0x0e;
const COMMAND_STATUS: u8 = 0x0f;
const SUCCESS: u8 = 0x00;
const UNKNOWN_COMMAND: u8 = 0x01;

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

/// The stand-in controller.
#[derive(Debug, Clone)]
pub struct Chip {
    /// Every command it was sent, in order.
    pub received: Vec<Vec<u8>>,
    /// Supported Commands.
    pub commands: [u8; 64],
    /// The credits each answer gives.
    pub credits: u8,
    /// LMP features page 0, and Read Buffer Size's return parameters.
    pub features: [u8; 8],
    pub buffers: [u8; 7],
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
            commands,
            credits: 2,
            features: FEATURES,
            buffers: BUFFERS,
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

    /// One command; its events, in order.
    pub fn command(&mut self, packet: &[u8]) -> Vec<Vec<u8>> {
        self.received.push(packet.to_vec());
        let opcode = u16::from_le_bytes([packet[0], packet[1]]);
        let p = packet[3..].to_vec();
        if let Some(&(_, write, default)) = SETTINGS.iter().find(|s| s.0 == opcode) {
            return self.ok(opcode, &self.stored(write, default));
        }
        if SETTINGS.iter().any(|s| s.1 == opcode) {
            self.written.push((opcode, p));
            return self.ok(opcode, &[]);
        }
        match opcode {
            0x0401 | 0x0419 => vec![self.status(opcode, SUCCESS)],
            0x041a => self.ok(opcode, &p[..6]),
            0x0402 | 0x0c01 | 0x0c63 | 0x2001 => self.ok(opcode, &[]),
            0x0c03 => {
                self.written.clear();
                self.local_name = vec![0; 248];
                self.eir = vec![0; 241];
                self.ok(opcode, &[])
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

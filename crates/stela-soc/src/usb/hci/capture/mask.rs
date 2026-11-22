//! What the capture hides of a packet before it holds it: everything but
//! what a rule below shows. Headers always show (opcodes, lengths, event
//! codes, LE subevents, statuses, command credits, ACL handles and L2CAP
//! lengths and channels), so lines still diff and replay; so do the
//! parameters of the commands and returns that say nothing of anyone. An
//! address shows its vendor half and its last byte, as
//! `stela_contracts::masked::Mac` prints one (in HCI order, bytes 0 and 3 to
//! 5); its two middle bytes are hidden. Hidden whatever their place are:
//!
//! - names: the Mac's (Write Local Name, Read Local Name's return), a remote
//!   device's, and the name types of extended inquiry and advertising data;
//! - opaque payloads: manufacturer and service data and URIs in that data,
//!   LE advertising and scan response data, every vendor command's
//!   parameters and every vendor return past its status, vendor events,
//!   and ACL data past its HCI and L2CAP headers (so SMP's keys on channels
//!   6 and 7, and a keyboard's reports, never show);
//! - keys and secrets: Link Key and PIN Code Request Replies, User Passkey
//!   Request Replies, Remote OOB (Extended) Data Request Replies, Write
//!   Stored Link Key, LE Encrypt and its result, LE Enable Encryption, LE
//!   Long Term Key Request Reply, LE Add Device To Resolving List, the local
//!   OOB data reads' returns, Return Link Keys, Link Key Notification and LE
//!   Generate DHKey Complete;
//! - anything of a command, return or event no rule names, and every byte
//!   past the length its header declares;
//! - of an event with several inquiry results or advertising reports,
//!   everything past the count: their arrays may come device by device or
//!   parameter by parameter;
//! - extended inquiry or advertising data from a structure that runs past
//!   the data's end.
//!
//! Evidence: Bluetooth Core v5.4 Vol 4 Part E 5.4 (packet formats), 7.1-7.8
//! (parameters) and 7.7 (events); Vol 3 Part C 11 and the Assigned Numbers'
//! AD types; Vol 3 Part A 3 (L2CAP basic frames).
use alloc::{vec, vec::Vec};
use core::ops::Range;

/// Commands whose parameters say nothing of anyone.
const PLAIN_COMMANDS: &[u16] = &[
    0x0401, 0x0402, 0x0406, 0x080f, 0x0c01, 0x0c03, 0x0c16, 0x0c18, 0x0c1a, 0x0c1c, 0x0c1e, 0x0c20,
    0x0c24, 0x0c26, 0x0c31, 0x0c33, 0x0c35, 0x0c43, 0x0c45, 0x0c47, 0x0c56, 0x0c5b, 0x0c63, 0x0c6d,
    0x1004, 0x2001, 0x200a, 0x200b, 0x200c, 0x200e, 0x2010,
];

/// Commands whose return parameters, past the status, say nothing of anyone.
const PLAIN_RETURNS: &[u16] = &[
    0x080e, 0x0c15, 0x0c17, 0x0c19, 0x0c1b, 0x0c1d, 0x0c1f, 0x0c23, 0x0c25, 0x0c42, 0x0c44, 0x0c46,
    0x0c55, 0x0c58, 0x0c5a, 0x0c6c, 0x1001, 0x1002, 0x1003, 0x1004, 0x1005, 0x2002, 0x2003, 0x2007,
    0x200f, 0x201c,
];

/// Events whose parameters say nothing of anyone: Inquiry Complete,
/// Disconnection Complete, Encryption Change, Read Remote Supported Features
/// and Version Complete, Hardware Error, Number Of Completed Packets, Data
/// Buffer Overflow, Max Slots Change.
const PLAIN_EVENTS: &[u8] = &[0x01, 0x05, 0x08, 0x0b, 0x0c, 0x10, 0x13, 0x1a, 0x1b];

/// AD types whose data say nothing of anyone: flags, service UUID lists,
/// TX power, class of device, OOB flags, connection interval range,
/// solicitation UUIDs, appearance, LE role.
const PLAIN_AD: &[u8] = &[
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x0a, 0x0d, 0x11, 0x12, 0x14, 0x15, 0x19, 0x1c, 0x1f,
];

const VENDOR_GROUP: u16 = 0x3f;

/// Which bytes of a whole `packet` are hidden.
struct Mask<'a> {
    packet: &'a [u8],
    hidden: Vec<bool>,
    /// Where the packet's header says it ends, or its own end if that is
    /// sooner: no rule shows a byte past it, whatever trails there.
    end: usize,
}

/// Where `packet`'s header says it ends: its header and the parameter or
/// data length the header gives (Vol 4 Part E 5.4).
fn declared_end(kind: super::Kind, packet: &[u8]) -> usize {
    let declared = match kind {
        super::Kind::Command => packet.get(2).map(|&length| 3 + usize::from(length)),
        super::Kind::Event => packet.get(1).map(|&length| 2 + usize::from(length)),
        super::Kind::Acl => packet
            .get(2..4)
            .map(|length| 4 + usize::from(u16::from_le_bytes([length[0], length[1]]))),
    };
    declared.unwrap_or(packet.len()).min(packet.len())
}

impl Mask<'_> {
    fn show(&mut self, range: Range<usize>) {
        let end = range.end.min(self.end);
        if range.start < end {
            self.hidden[range.start..end].fill(false);
        }
    }

    /// The parameters from `from` to the declared end.
    fn rest(&mut self, from: usize) {
        self.show(from..self.end);
    }

    /// An address at `at`: its last byte and vendor half.
    fn address(&mut self, at: usize) {
        self.show(at..at + 1);
        self.show(at + 3..at + 6);
    }

    /// Extended inquiry or advertising data in `range`: each structure's
    /// length and type, and the data of the plain types, up to the first
    /// structure of length 0, which ends the significant part (Vol 3 Part C
    /// 11). A structure that runs past the data's end is malformed: it and
    /// everything after it stay hidden, since no length read from there on
    /// can be trusted to fall on a structure's start.
    fn data(&mut self, range: Range<usize>) {
        let end = range.end.min(self.end);
        let mut at = range.start;
        while at < end {
            let length = usize::from(self.packet[at]);
            if length == 0 {
                self.show(at..at + 1);
                return;
            }
            if at + 1 + length > end {
                return;
            }
            self.show(at..at + 2);
            if PLAIN_AD.contains(&self.packet[at + 1]) {
                self.show(at + 2..at + 1 + length);
            }
            at += 1 + length;
        }
    }

    fn command(&mut self) {
        self.show(0..3);
        let Some(&[low, high]) = self.packet.first_chunk() else {
            return;
        };
        match u16::from_le_bytes([low, high]) {
            // An address first, the rest plain: Create Connection, its
            // Cancel, Remote Name Request and its Cancel, LE Set Random
            // Address, the module's Write BD_ADDR.
            0x0405 | 0x0408 | 0x0419 | 0x041a | 0x2005 | 0xfc01 => {
                self.address(3);
                self.rest(9);
            }
            // LE Add or Remove Device From Filter Accept List.
            0x2011 | 0x2012 => {
                self.show(3..4);
                self.address(4);
            }
            // LE Set Advertising Parameters: the peer at 7.
            0x2006 => {
                self.show(3..10);
                self.address(10);
                self.rest(16);
            }
            // LE Create Connection: the peer at 6.
            0x200d => {
                self.show(3..9);
                self.address(9);
                self.rest(15);
            }
            // Write Extended Inquiry Response.
            0x0c52 => {
                self.show(3..4);
                self.data(4..self.packet.len());
            }
            // LE Set Advertising Data, LE Set Scan Response Data: the length
            // alone.
            0x2008 | 0x2009 => self.show(3..4),
            opcode if PLAIN_COMMANDS.contains(&opcode) => self.rest(3),
            _ => {}
        }
    }

    fn event(&mut self) {
        self.show(0..2);
        let Some(&code) = self.packet.first() else {
            return;
        };
        match code {
            0x0e => {
                // Credits, opcode and status.
                self.show(2..6);
                let Some(&[low, high]) = self.packet.get(3..5).and_then(|o| o.first_chunk()) else {
                    return;
                };
                let opcode = u16::from_le_bytes([low, high]);
                match opcode {
                    0x0408 | 0x041a | 0x1009 => self.address(6),
                    0x0c51 => {
                        self.show(6..7);
                        self.data(7..self.packet.len());
                    }
                    _ if opcode >> 10 == VENDOR_GROUP => {}
                    _ if PLAIN_RETURNS.contains(&opcode) => self.rest(6),
                    _ => {}
                }
            }
            0x0f => self.show(2..6),
            // Inquiry Result (14 bytes a device), with RSSI (14 as well).
            // Their parameters are arrays, which a controller may send
            // device by device (as Linux's `inquiry_info` reads them) or
            // parameter by parameter: one response reads the same either
            // way, but of several only the count shows, since a rule for one
            // layout would show the other's address bytes.
            0x02 | 0x22 => {
                self.show(2..3);
                if self.packet.get(2) == Some(&1) {
                    self.address(3);
                    self.show(9..17);
                }
            }
            // Extended Inquiry Result: one device, then its EIR.
            0x2f => {
                self.show(2..3);
                self.address(3);
                self.show(9..17);
                self.data(17..self.packet.len());
            }
            // Remote Name Request Complete: the name stays hidden.
            0x07 => {
                self.show(2..3);
                self.address(3);
            }
            // Connection Complete, Connection Request, Remote Host Supported
            // Features Notification.
            0x03 => {
                self.show(2..5);
                self.address(5);
                self.rest(11);
            }
            0x04 | 0x3d => {
                self.address(2);
                self.rest(8);
            }
            0x3e => self.le_meta(),
            code if PLAIN_EVENTS.contains(&code) => self.rest(2),
            _ => {}
        }
    }

    fn le_meta(&mut self) {
        self.show(2..3);
        match self.packet.get(2) {
            // LE Connection Complete: status, handle, role, peer type, then
            // the peer, then plain timing.
            Some(0x01) => {
                self.show(3..8);
                self.address(8);
                self.rest(14);
            }
            // LE Advertising Report: the event and address types, the
            // address, the data's length, hidden data, the RSSI. Its
            // parameters are arrays too, so as for an inquiry result only
            // one report shows past the count.
            Some(0x02) => {
                self.show(3..4);
                if self.packet.get(3) == Some(&1) {
                    self.show(4..6);
                    self.address(6);
                    self.show(12..13);
                    let length = usize::from(self.packet.get(12).copied().unwrap_or(0));
                    self.show(13 + length..14 + length);
                }
            }
            // Others show their status alone, as LE Generate DHKey Complete.
            _ => self.show(3..4),
        }
    }

    fn acl(&mut self) {
        self.show(0..4);
        // A start fragment (packet boundary 00 or 10) begins with the L2CAP
        // header: length and channel.
        if self
            .packet
            .get(1)
            .is_some_and(|&high| high >> 4 & 0b01 == 0)
        {
            self.show(4..8);
        }
    }
}

/// Which bytes of `packet`, of `kind`, the capture hides.
pub(super) fn hidden(kind: super::Kind, packet: &[u8]) -> Vec<bool> {
    let mut mask = Mask {
        packet,
        hidden: vec![true; packet.len()],
        end: declared_end(kind, packet),
    };
    match kind {
        super::Kind::Command => mask.command(),
        super::Kind::Event => mask.event(),
        super::Kind::Acl => mask.acl(),
    }
    mask.hidden
}

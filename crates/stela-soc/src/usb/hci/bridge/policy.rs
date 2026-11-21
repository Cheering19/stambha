//! Which of the guest's commands go to the host's controller: only the
//! standard commands on [`FORWARDED`]. Everything else, every vendor command
//! (OGF 0x3F) above all, is the shadow model's. Vendor opcodes mean other
//! things to other vendors' firmware: on a Realtek controller 0xFC20
//! downloads firmware, 0xFC61 reads a register and 0xFC66 drops the
//! patch (Linux `drivers/bluetooth/btrtl.c`), so an Apple or Broadcom opcode
//! sent there could undo the host driver's setup. [`forward`] is the only
//! maker of a [`ChipCommand`], and it refuses any vendor opcode whatever the
//! list says, which a compile-time check also holds the list to.
//!
//! The list is milestone 1's, which lists nearby classic devices: the
//! controller resets and takes the guest's configuration (event masks,
//! inquiry mode, page timeout, names, class), and runs inquiries and remote
//! name requests. It keeps local:
//!
//! - the identity the module gives (Read Local Version Information and the
//!   Broadcom vendor commands), so bluetoothd stays on its BCM20702B0 path;
//! - scan enable: with no command to accept a connection or answer pairing
//!   in the model's Supported Commands, a connectable host controller would
//!   hand bluetoothd requests it cannot answer;
//! - host flow control: controller-to-host flow control stays off in the
//!   host's controller, as Linux leaves it, and the USB queues push back;
//! - connections and everything LE, the model's alone until later
//!   milestones list them.
use super::{Cause, Completion};
use alloc::vec::Vec;

const VENDOR_GROUP: u16 = 0x3f;
const SUCCESS: u8 = 0x00;

/// How the host controller's answer to a forwarded command becomes the
/// guest's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Forward {
    /// Configuration, reset and resources: the controller's answer, unless
    /// it fails where the model succeeds.
    Strict,
    /// A radio procedure, whose error is the radio's outcome: the
    /// controller's answer whatever its status.
    Procedure,
}

/// The standard commands milestone 1 forwards (opcode, Supported Commands
/// octet and bit of Bluetooth Core v5.4 Vol 4 Part E 6.27, kind). Each is
/// one the model implements and lists.
pub const FORWARDED: &[(u16, u8, u8, Forward)] = &[
    // Link control: inquiry and remote names.
    (0x0401, 0, 0, Forward::Procedure), // Inquiry
    (0x0402, 0, 1, Forward::Procedure), // Inquiry Cancel
    (0x0419, 2, 3, Forward::Procedure), // Remote Name Request
    (0x041a, 2, 4, Forward::Procedure), // Remote Name Request Cancel
    // Link policy.
    (0x080e, 5, 3, Forward::Strict), // Read Default Link Policy Settings
    (0x080f, 5, 4, Forward::Strict), // Write Default Link Policy Settings
    // Controller and baseband.
    (0x0c01, 5, 6, Forward::Strict),  // Set Event Mask
    (0x0c03, 5, 7, Forward::Strict),  // Reset
    (0x0c13, 7, 0, Forward::Strict),  // Write Local Name
    (0x0c14, 7, 1, Forward::Strict),  // Read Local Name
    (0x0c15, 7, 2, Forward::Strict),  // Read Connection Accept Timeout
    (0x0c16, 7, 3, Forward::Strict),  // Write Connection Accept Timeout
    (0x0c17, 7, 4, Forward::Strict),  // Read Page Timeout
    (0x0c18, 7, 5, Forward::Strict),  // Write Page Timeout
    (0x0c1b, 8, 0, Forward::Strict),  // Read Page Scan Activity
    (0x0c1c, 8, 1, Forward::Strict),  // Write Page Scan Activity
    (0x0c1d, 8, 2, Forward::Strict),  // Read Inquiry Scan Activity
    (0x0c1e, 8, 3, Forward::Strict),  // Write Inquiry Scan Activity
    (0x0c1f, 8, 4, Forward::Strict),  // Read Authentication Enable
    (0x0c20, 8, 5, Forward::Strict),  // Write Authentication Enable
    (0x0c23, 9, 0, Forward::Strict),  // Read Class Of Device
    (0x0c24, 9, 1, Forward::Strict),  // Write Class Of Device
    (0x0c25, 9, 2, Forward::Strict),  // Read Voice Setting
    (0x0c26, 9, 3, Forward::Strict),  // Write Voice Setting
    (0x0c42, 12, 4, Forward::Strict), // Read Inquiry Scan Type
    (0x0c43, 12, 5, Forward::Strict), // Write Inquiry Scan Type
    (0x0c44, 12, 6, Forward::Strict), // Read Inquiry Mode
    (0x0c45, 12, 7, Forward::Strict), // Write Inquiry Mode
    (0x0c46, 13, 0, Forward::Strict), // Read Page Scan Type
    (0x0c47, 13, 1, Forward::Strict), // Write Page Scan Type
    (0x0c51, 17, 0, Forward::Strict), // Read Extended Inquiry Response
    (0x0c52, 17, 1, Forward::Strict), // Write Extended Inquiry Response
    (0x0c55, 17, 5, Forward::Strict), // Read Simple Pairing Mode
    (0x0c56, 17, 6, Forward::Strict), // Write Simple Pairing Mode
    (0x0c58, 18, 0, Forward::Strict), // Read Inquiry Response Transmit Power Level
    (0x0c5a, 18, 2, Forward::Strict), // Read Default Erroneous Data Reporting
    (0x0c5b, 18, 3, Forward::Strict), // Write Default Erroneous Data Reporting
    (0x0c63, 22, 2, Forward::Strict), // Set Event Mask Page 2
    (0x0c6c, 24, 5, Forward::Strict), // Read LE Host Support
    (0x0c6d, 24, 6, Forward::Strict), // Write LE Host Support
    // Informational: the controller's own address.
    (0x1009, 15, 1, Forward::Strict), // Read BD_ADDR
    // LE: the event mask only.
    (0x2001, 25, 0, Forward::Strict), // LE Set Event Mask
];

// No vendor opcode is ever on the list.
const _: () = {
    let mut i = 0;
    while i < FORWARDED.len() {
        assert!(FORWARDED[i].0 >> 10 != VENDOR_GROUP);
        i += 1;
    }
};

/// A command the host's controller may be sent: a standard command on
/// [`FORWARDED`], never a vendor one. [`forward`] is its only maker.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChipCommand(Vec<u8>);

impl ChipCommand {
    pub fn opcode(&self) -> u16 {
        u16::from_le_bytes([self.0[0], self.0[1]])
    }

    pub fn bytes(&self) -> &[u8] {
        &self.0
    }

    pub fn into_bytes(self) -> Vec<u8> {
        self.0
    }
}

/// Whether `opcode` is a vendor command's.
pub fn vendor(opcode: u16) -> bool {
    opcode >> 10 == VENDOR_GROUP
}

/// `packet`, a whole command, as one for the host's controller, and how its
/// answer becomes the guest's; `None` for a command the shadow answers.
pub fn forward(packet: &[u8]) -> Option<(ChipCommand, Forward)> {
    let opcode = u16::from_le_bytes([*packet.first()?, *packet.get(1)?]);
    if vendor(opcode) {
        return None;
    }
    let &(_, _, _, kind) = FORWARDED.iter().find(|entry| entry.0 == opcode)?;
    Some((ChipCommand(packet.to_vec()), kind))
}

/// The return parameters of the guest's answer to a forwarded command,
/// from the host controller's answer `chip` and the shadow's, or why the
/// bridge falls back instead.
pub(super) fn answer(
    kind: Forward,
    shadow: Option<&Completion<'_>>,
    chip: &Completion<'_>,
) -> Result<Vec<u8>, Cause> {
    let Some(shadow) = shadow else {
        return Ok(chip.returns.to_vec());
    };
    if chip.code != shadow.code || chip.returns.len() != shadow.returns.len() {
        return Err(Cause::Malformed);
    }
    let refused = chip.status != SUCCESS && shadow.status == SUCCESS;
    if refused && kind != Forward::Procedure {
        return Err(Cause::Refused(chip.status));
    }
    Ok(chip.returns.to_vec())
}

//! The function's full capture (`--bluetooth-trace`): every HCI packet that
//! crosses it after each configuration, both ways, and for a lent controller
//! (`super::bridge`) each one to and from the host's, for the owner's log.
//!
//! A line holds one packet: where it went (`>` from the guest, `<` to it,
//! `chip>` to the host's controller, `chip<` from it), its kind (`cmd`,
//! `evt`, `acl`), its first [`CAPTURED_BYTES`] in hex, then `+N` for the N
//! bytes cut, which [`Captured::parse`] reads back. Packets carry no H4
//! indicator: the USB transport has none (Bluetooth Core v5.4 Vol 4 Part
//! B 2), so the kind names it. The capture holds at most
//! [`CAPTURED_PACKETS`] packets per configuration, of the boot's first
//! [`CAPTURED_CONFIGURATIONS`], and none unless the owner asks for it.
//!
//! A packet is masked as it is captured ([`Captured::new`], `mask`), so no
//! raw byte leaves the function: its headers and the parameters that say
//! nothing of anyone show; the middle bytes of addresses, names, opaque
//! payloads, keys and secrets are held as 0 and printed `xx`, keeping their
//! length, and [`Captured::parse`] reads `xx` back as a hidden 0, so a
//! masked log is itself a fixture. Even so the log names nearby devices'
//! classes, signal strengths and address ends: keep it private until it
//! has been checked.
use alloc::vec::Vec;
use core::fmt;

mod mask;

/// Packets captured after each configuration.
pub const CAPTURED_PACKETS: u16 = 1024;
/// Bytes a captured packet shows: every command and event whole (at most
/// 258 and 257 bytes), and an ACL packet's first ones.
pub const CAPTURED_BYTES: usize = 300;

/// Where a captured packet went.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// From the guest to the function.
    FromGuest,
    /// From the function to the guest.
    ToGuest,
    /// From the function to the host's controller.
    ToChip,
    /// From the host's controller to the function.
    FromChip,
}

impl Direction {
    pub const ALL: [Self; 4] = [Self::FromGuest, Self::ToGuest, Self::ToChip, Self::FromChip];

    fn token(self) -> &'static str {
        match self {
            Self::FromGuest => ">",
            Self::ToGuest => "<",
            Self::ToChip => "chip>",
            Self::FromChip => "chip<",
        }
    }
}

/// What a captured packet is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Command,
    Event,
    Acl,
}

impl Kind {
    pub const ALL: [Self; 3] = [Self::Command, Self::Event, Self::Acl];

    fn token(self) -> &'static str {
        match self {
            Self::Command => "cmd",
            Self::Event => "evt",
            Self::Acl => "acl",
        }
    }
}

/// One captured packet.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Captured {
    pub direction: Direction,
    pub kind: Kind,
    /// Its first [`CAPTURED_BYTES`], each hidden one as 0.
    pub bytes: Vec<u8>,
    /// Which of those are hidden.
    pub hidden: Vec<bool>,
    /// How many bytes followed those.
    pub cut: usize,
}

impl Captured {
    /// `packet`, masked: what is hidden of it is never held.
    pub fn new(direction: Direction, kind: Kind, packet: &[u8]) -> Self {
        let shown = packet.len().min(CAPTURED_BYTES);
        let mut hidden = mask::hidden(kind, packet);
        hidden.truncate(shown);
        let bytes = packet[..shown]
            .iter()
            .zip(&hidden)
            .map(|(&byte, &hidden)| if hidden { 0 } else { byte })
            .collect();
        Self {
            direction,
            kind,
            bytes,
            hidden,
            cut: packet.len() - shown,
        }
    }

    /// The packet a line as [`fmt::Display`] writes it shows; `None` for any
    /// other text.
    pub fn parse(line: &str) -> Option<Self> {
        let mut words = line.split_ascii_whitespace();
        let direction = words.next()?;
        let direction = *Direction::ALL.iter().find(|d| d.token() == direction)?;
        let kind = words.next()?;
        let kind = *Kind::ALL.iter().find(|k| k.token() == kind)?;
        let (mut bytes, mut hidden) = (Vec::new(), Vec::new());
        let mut cut = 0;
        for word in words {
            if cut != 0 {
                return None;
            }
            if let Some(count) = word.strip_prefix('+') {
                cut = count.parse().ok().filter(|&n| n != 0)?;
                continue;
            }
            if word.len() != 2 {
                return None;
            }
            let masked = word == "xx";
            bytes.push(if masked {
                0
            } else {
                u8::from_str_radix(word, 16).ok()?
            });
            hidden.push(masked);
        }
        (!bytes.is_empty() && (cut == 0 || bytes.len() == CAPTURED_BYTES)).then_some(Self {
            direction,
            kind,
            bytes,
            hidden,
            cut,
        })
    }
}

impl fmt::Display for Captured {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} {}", self.direction.token(), self.kind.token())?;
        for (byte, &hidden) in self.bytes.iter().zip(&self.hidden) {
            if hidden {
                f.write_str(" xx")?;
            } else {
                write!(f, " {byte:02x}")?;
            }
        }
        if self.cut != 0 {
            write!(f, " +{}", self.cut)?;
        }
        Ok(())
    }
}

/// What starts each captured packet's line in the boot log.
pub const LOG_PREFIX: &str = "GUEST BLUETOOTH RAW ";

/// The packets of a capture: each line [`Captured::parse`] reads, on its own
/// or after [`LOG_PREFIX`] as the boot log writes it, blank lines and `#`
/// comments skipped. `Err` names the first other line, counted from 1.
pub fn read(text: &str) -> Result<Vec<Captured>, usize> {
    text.lines()
        .enumerate()
        .filter_map(|(index, line)| {
            let line = line
                .find(LOG_PREFIX)
                .map_or(line, |at| &line[at + LOG_PREFIX.len()..])
                .trim();
            (!line.is_empty() && !line.starts_with('#'))
                .then(|| Captured::parse(line).ok_or(index + 1))
        })
        .collect()
}

/// Configurations captured in a boot. Each refills the budget of
/// [`CAPTURED_PACKETS`]; a guest that configures the function again and
/// again (a daemon restarting, a reset loop) would otherwise fill the boot
/// log without end.
pub const CAPTURED_CONFIGURATIONS: u8 = 4;

/// Where the capture stopped, for the owner's log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum End {
    /// This configuration's packets were taken; the next configuration
    /// starts again.
    Configuration,
    /// The boot's last configuration to be captured ended, or one came
    /// after it.
    Boot,
}

/// What the function captures, and what it holds until the owner takes it.
#[derive(Debug, Clone, Default)]
pub(super) struct Capture {
    /// Whether the owner asked for the capture.
    enabled: bool,
    /// Packets still captured in this configuration.
    left: u16,
    /// Configurations captured so far.
    configurations: u8,
    /// Whether the boot's capture is over and the log was told.
    over: bool,
    held: Vec<Captured>,
}

impl Capture {
    /// The owner asks for the capture; it starts at once on a configured
    /// function.
    pub(super) fn enable(&mut self, configured: bool) {
        if !self.enabled {
            self.enabled = true;
            if configured {
                self.start();
            }
        }
    }

    /// A configuration's budget, while the boot has one left.
    fn start(&mut self) -> Option<End> {
        if self.configurations < CAPTURED_CONFIGURATIONS {
            self.configurations += 1;
            self.left = CAPTURED_PACKETS;
            return None;
        }
        self.left = 0;
        self.over().then_some(End::Boot)
    }

    /// Whether the boot's capture is over just now, said once.
    fn over(&mut self) -> bool {
        !core::mem::replace(&mut self.over, true)
    }

    /// The guest set the configuration to `value` (0: unconfigured); `Some`
    /// when the capture is over for the boot, once.
    pub(super) fn configured(&mut self, value: u8) -> Option<End> {
        if !self.enabled || value == 0 {
            self.left = 0;
            return None;
        }
        self.start()
    }

    /// Captures `packet`; `Some` when it was this configuration's last.
    pub(super) fn record(
        &mut self,
        direction: Direction,
        kind: Kind,
        packet: &[u8],
    ) -> Option<End> {
        if self.left == 0 {
            return None;
        }
        self.left -= 1;
        if self.held.len() < usize::from(CAPTURED_PACKETS) {
            self.held.push(Captured::new(direction, kind, packet));
        }
        if self.left != 0 {
            return None;
        }
        if self.configurations < CAPTURED_CONFIGURATIONS {
            return Some(End::Configuration);
        }
        self.over().then_some(End::Boot)
    }

    pub(super) fn take(&mut self) -> Vec<Captured> {
        core::mem::take(&mut self.held)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::{string::ToString, vec};

    /// The `--bluetooth-trace` line format round-trips every packet.
    #[test]
    fn a_captured_line_reads_back_as_the_packet_it_shows() {
        for direction in Direction::ALL {
            for kind in Kind::ALL {
                let packet = Captured::new(direction, kind, &[0x03, 0x0c, 0]);
                assert_eq!(Captured::parse(&packet.to_string()), Some(packet));
            }
        }
        assert_eq!(
            Captured::new(Direction::FromGuest, Kind::Command, &[0x03, 0x0c, 0]).to_string(),
            "> cmd 03 0c 00"
        );
        assert_eq!(
            Captured::new(Direction::ToGuest, Kind::Event, &[0x0e, 4, 1, 3, 0x0c, 0]).to_string(),
            "< evt 0e 04 01 03 0c 00"
        );
        let long: Vec<u8> = (0..=255).cycle().take(1025).collect();
        let cut = Captured::new(Direction::FromChip, Kind::Acl, &long);
        assert_eq!((cut.bytes.len(), cut.cut), (CAPTURED_BYTES, 725));
        let line = cut.to_string();
        // The HCI and L2CAP headers show, the payload is hidden.
        assert!(
            line.starts_with("chip< acl 00 01 02 03 04 05 06 07 xx "),
            "{line}"
        );
        assert!(line.ends_with(" xx +725"), "{line}");
        assert_eq!(Captured::parse(&line), Some(cut));
        // Extra spaces between words read the same.
        assert_eq!(
            Captured::parse("  chip>   cmd 01 04 05 33 8b 9e 08 00 "),
            Some(Captured::new(
                Direction::ToChip,
                Kind::Command,
                &[1, 4, 5, 0x33, 0x8b, 0x9e, 8, 0]
            ))
        );
    }

    /// A neighbour's address 32:31:30:b8:a7:10, in HCI order.
    const NEIGHBOUR: [u8; 6] = [0x10, 0xa7, 0xb8, 0x30, 0x31, 0x32];
    const NAME: &[u8] = b"Garvits Mac";
    const KEY: [u8; 16] = [0x5a; 16];

    /// Whether `secret` is anywhere in what the capture holds of a packet.
    fn holds(captured: &Captured, secret: &[u8]) -> bool {
        captured.bytes.windows(secret.len()).any(|w| w == secret)
    }

    /// An extended inquiry result, a Write Local Name and a Link Key Request
    /// Reply as the capture holds them: no address's middle bytes, no name,
    /// no key and no manufacturer data leave the function, while the headers
    /// stay, so lines still diff and replay.
    #[test]
    fn the_capture_never_holds_an_addresss_middle_a_name_or_a_key() {
        let mut eir = vec![NAME.len() as u8 + 1, 0x09];
        eir.extend_from_slice(NAME);
        eir.extend_from_slice(&[5, 0xff, 0x4c, 0x00, 0x99, 0x98, 3, 0x03, 0x0b, 0x11]);
        eir.resize(240, 0);
        let mut result = vec![0x2f, 255, 1];
        result.extend_from_slice(&NEIGHBOUR);
        result.extend_from_slice(&[1, 0, 0x04, 0x04, 0x24, 0x34, 0x12, 0xd0]);
        result.extend_from_slice(&eir);
        let mut local_name = vec![0x13, 0x0c, 248];
        local_name.extend_from_slice(NAME);
        local_name.resize(3 + 248, 0);
        let mut reply = vec![0x0b, 0x04, 22];
        reply.extend_from_slice(&NEIGHBOUR);
        reply.extend_from_slice(&KEY);
        for (direction, kind, packet) in [
            (Direction::FromChip, Kind::Event, &result),
            (Direction::FromGuest, Kind::Command, &local_name),
            (Direction::ToChip, Kind::Command, &reply),
        ] {
            let captured = Captured::new(direction, kind, packet);
            for secret in [&NEIGHBOUR[1..3], NAME, &KEY[..], &[0x99, 0x98][..]] {
                assert!(!holds(&captured, secret), "{captured}");
            }
            let line = captured.to_string();
            assert!(!line.contains("a7 b8"), "{line}");
            assert_eq!(captured.bytes[..3], packet[..3], "the header");
            assert!(!captured.hidden[..3].contains(&true));
            // A masked line reads back as the packet it shows, hidden
            // bytes as 00, so a masked log is itself a fixture.
            assert_eq!(Captured::parse(&line), Some(captured));
        }
        let line = Captured::new(Direction::FromChip, Kind::Event, &result).to_string();
        let name = " xx".repeat(NAME.len());
        assert!(
            line.starts_with(&alloc::format!(
                "chip< evt 2f ff 01 10 xx xx 30 31 32 01 00 04 04 24 34 12 d0 0c 09{name} \
                 05 ff xx xx xx xx 03 03 0b 11 00 xx"
            )),
            "{line}"
        );
        let line = Captured::new(Direction::ToChip, Kind::Command, &reply).to_string();
        assert!(line.starts_with("chip> cmd 0b 04 16 xx xx xx"), "{line}");
    }

    /// Keys and secrets are never held, wherever they travel: the commands
    /// that carry them, their returns, the events and the ACL channels.
    #[test]
    fn keys_and_secrets_are_never_held() {
        const SECRET: u8 = 0xa5;
        let none_held = |captured: Captured, from: usize| {
            assert!(captured.bytes[from..].iter().all(|&b| b == 0), "{captured}");
            assert!(captured.hidden[from..].iter().all(|&h| h), "{captured}");
            assert!(!captured.hidden[..from].contains(&true), "{captured}");
        };
        let with = |head: &[u8], length: usize| {
            let mut packet = head.to_vec();
            packet.resize(head.len() + length, SECRET);
            packet
        };
        // Link key, PIN, passkey and OOB replies, stored link keys, LE
        // encryption, LTK replies and resolving list IRKs.
        for opcode in [
            0x040b, 0x040d, 0x042e, 0x0430, 0x0445, 0x0c11, 0x2017, 0x2019, 0x201a, 0x2027,
        ] {
            let [low, high] = u16::to_le_bytes(opcode);
            let command = with(&[low, high, 32], 32);
            none_held(
                Captured::new(Direction::FromGuest, Kind::Command, &command),
                3,
            );
        }
        // LE Encrypt's result, the local OOB data, and every vendor
        // command's returns past the status.
        for opcode in [0x2017, 0x0c57, 0x0c7d, 0xfc79] {
            let [low, high] = u16::to_le_bytes(opcode);
            let complete = with(&[0x0e, 36, 1, low, high, 0], 32);
            none_held(
                Captured::new(Direction::FromChip, Kind::Event, &complete),
                6,
            );
        }
        // Return Link Keys, Link Key Notification, a vendor event, and LE
        // Generate DHKey Complete past its status.
        for (head, from) in [
            (&[0x15, 23][..], 2),
            (&[0x18, 23][..], 2),
            (&[0xff, 23][..], 2),
            (&[0x3e, 34, 0x09, 0x00][..], 4),
        ] {
            let event = with(head, usize::from(head[1]) + 2 - head.len());
            none_held(Captured::new(Direction::ToGuest, Kind::Event, &event), from);
        }
        // SMP (channels 6 and 7) and every other channel's payload: the HCI
        // and L2CAP headers stay; a continuation has no L2CAP header.
        for cid in [6u8, 7, 0x40] {
            let acl = with(&[0x01, 0x20, 24, 0, 20, 0, cid, 0], 20);
            none_held(Captured::new(Direction::FromChip, Kind::Acl, &acl), 8);
        }
        let continuation = with(&[0x01, 0x10, 20, 0], 20);
        none_held(
            Captured::new(Direction::FromGuest, Kind::Acl, &continuation),
            4,
        );
        // Vendor parameters, the Mac's name and LE advertising data.
        for (head, from) in [
            (&[0x4c, 0xfc, 9][..], 3),
            (&[0x13, 0x0c, 248][..], 3),
            (&[0x08, 0x20, 32, 31][..], 4),
            (&[0x09, 0x20, 32, 31][..], 4),
        ] {
            let command = with(head, usize::from(head[2]) + 3 - head.len());
            none_held(
                Captured::new(Direction::FromGuest, Kind::Command, &command),
                from,
            );
        }
        // Read Local Name's return and a remote name past the address.
        let read_name = with(&[0x0e, 252, 1, 0x14, 0x0c, 0], 248);
        none_held(
            Captured::new(Direction::ToGuest, Kind::Event, &read_name),
            6,
        );
        let mut named = vec![0x07, 255, 0];
        named.extend_from_slice(&NEIGHBOUR);
        let named = with(&named, 248);
        let captured = Captured::new(Direction::ToGuest, Kind::Event, &named);
        assert_eq!(captured.bytes[3..9], [0x10, 0, 0, 0x30, 0x31, 0x32]);
        none_held(
            Captured {
                bytes: captured.bytes[9..].to_vec(),
                hidden: captured.hidden[9..].to_vec(),
                ..captured
            },
            0,
        );
    }

    /// Commands whose parameters say nothing of anyone stay whole, and the
    /// addresses in them keep their vendor half and last byte.
    #[test]
    fn plain_commands_stay_whole_and_addresses_keep_their_ends() {
        for plain in [
            &[0x01, 0x04, 5, 0x33, 0x8b, 0x9e, 8, 0][..],
            &[
                0x01, 0x0c, 8, 0xff, 0xff, 0xfb, 0xff, 0x07, 0xf8, 0xbf, 0x3d,
            ][..],
            &[0x45, 0x0c, 1, 2][..],
            &[0x0e, 12, 1, 0x01, 0x10, 0, 6, 0, 0, 6, 15, 0, 0x0e, 0x41][..],
        ] {
            let kind = if plain[0] == 0x0e {
                Kind::Event
            } else {
                Kind::Command
            };
            let captured = Captured::new(Direction::FromGuest, kind, plain);
            assert_eq!(captured.bytes, plain);
            assert!(!captured.hidden.contains(&true), "{captured}");
        }
        let mut request = vec![0x19, 0x04, 10];
        request.extend_from_slice(&NEIGHBOUR);
        request.extend_from_slice(&[1, 0, 0x34, 0x12]);
        assert_eq!(
            Captured::new(Direction::ToChip, Kind::Command, &request).to_string(),
            "chip> cmd 19 04 0a 10 xx xx 30 31 32 01 00 34 12"
        );
        let mut address = vec![0x0e, 10, 1, 0x09, 0x10, 0];
        address.extend_from_slice(&NEIGHBOUR);
        assert_eq!(
            Captured::new(Direction::FromChip, Kind::Event, &address).to_string(),
            "chip< evt 0e 0a 01 09 10 00 10 xx xx 30 31 32"
        );
    }

    #[test]
    fn other_text_is_no_captured_packet() {
        for bad in [
            "",
            ">",
            "> cmd",
            "> sco 01",
            "? cmd 01",
            "> cmd 1",
            "> cmd 001",
            "> cmd 0g",
            "> cmd 01 +",
            "> cmd 01 +x",
            "> cmd 01 +0",
            "> cmd +3",
            "> cmd 01 +3",
            "> cmd 01 +3 02",
        ] {
            assert_eq!(Captured::parse(bad), None, "{bad:?}");
        }
    }

    /// A fixture's lines, or the boot log's raw lines as grep gives them.
    #[test]
    fn a_capture_reads_from_its_own_lines_or_the_boot_logs() {
        let text = "# a comment\n\n> cmd 03 0c 00\n\
                    [  12.5] GUEST BLUETOOTH RAW < evt 0e 04 01 03 0c 00\n";
        assert_eq!(
            read(text),
            Ok(vec![
                Captured::new(Direction::FromGuest, Kind::Command, &[3, 0x0c, 0]),
                Captured::new(Direction::ToGuest, Kind::Event, &[0x0e, 4, 1, 3, 0x0c, 0]),
            ])
        );
        assert_eq!(read("> cmd 03 0c 00\nGUEST BLUETOOTH up\n"), Err(2));
        assert_eq!(read(""), Ok(vec![]));
    }

    #[test]
    fn the_capture_holds_its_bound_per_configuration_and_only_once_asked() {
        let mut capture = Capture::default();
        assert_eq!(capture.configured(1), None);
        assert_eq!(
            capture.record(Direction::FromGuest, Kind::Command, &[3, 0x0c, 0]),
            None
        );
        assert!(capture.take().is_empty());
        capture.enable(true);
        for _ in 1..CAPTURED_PACKETS {
            assert_eq!(
                capture.record(Direction::ToGuest, Kind::Event, &[0x0e, 0]),
                None
            );
        }
        assert_eq!(
            capture.record(Direction::ToGuest, Kind::Event, &[0x0e, 0]),
            Some(End::Configuration)
        );
        assert_eq!(
            capture.record(Direction::ToGuest, Kind::Event, &[0x0e, 0]),
            None
        );
        assert_eq!(capture.take().len(), usize::from(CAPTURED_PACKETS));
        assert_eq!(capture.configured(0), None);
        assert_eq!(
            capture.record(Direction::FromGuest, Kind::Command, &[3, 0x0c, 0]),
            None
        );
        capture.configured(1);
        capture.record(Direction::FromGuest, Kind::Command, &[3, 0x0c, 0]);
        assert_eq!(
            capture.take(),
            vec![Captured::new(
                Direction::FromGuest,
                Kind::Command,
                &[3, 0x0c, 0]
            )]
        );
    }

    /// A guest that configures the function again and again gets its first
    /// [`CAPTURED_CONFIGURATIONS`] captured, then none: the budget refills
    /// per configuration, but not without end, and the end is said once.
    #[test]
    fn the_capture_refills_for_a_bounded_number_of_configurations() {
        let mut capture = Capture::default();
        capture.enable(true);
        let command = [3, 0x0c, 0];
        for _ in 1..CAPTURED_CONFIGURATIONS {
            capture.record(Direction::FromGuest, Kind::Command, &command);
            assert_eq!(capture.configured(1), None);
        }
        // The last configuration captured: its budget's end is the boot's.
        for _ in 1..CAPTURED_PACKETS {
            capture.record(Direction::ToGuest, Kind::Event, &[0x0e, 0]);
        }
        assert_eq!(
            capture.record(Direction::ToGuest, Kind::Event, &[0x0e, 0]),
            Some(End::Boot)
        );
        assert_eq!(capture.configured(1), None, "said once");
        assert_eq!(
            capture.record(Direction::FromGuest, Kind::Command, &command),
            None
        );
        let held = capture.take();
        assert_eq!(held.len(), usize::from(CAPTURED_PACKETS));
        // Reconfigured before the last budget ran out: the next
        // configuration says the end.
        let mut capture = Capture::default();
        capture.enable(true);
        for _ in 1..CAPTURED_CONFIGURATIONS {
            assert_eq!(capture.configured(1), None);
        }
        assert_eq!(capture.configured(1), Some(End::Boot));
        assert_eq!(capture.configured(1), None);
        assert_eq!(
            capture.record(Direction::FromGuest, Kind::Command, &command),
            None
        );
        assert!(capture.take().is_empty());
    }
}

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
//! [`CAPTURED_PACKETS`] packets per configuration, and none unless the owner
//! asks for it.
use alloc::vec::Vec;
use core::fmt;

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
    /// Its first [`CAPTURED_BYTES`].
    pub bytes: Vec<u8>,
    /// How many bytes followed those.
    pub cut: usize,
}

impl Captured {
    pub fn new(direction: Direction, kind: Kind, packet: &[u8]) -> Self {
        let shown = packet.len().min(CAPTURED_BYTES);
        Self {
            direction,
            kind,
            bytes: packet[..shown].to_vec(),
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
        let mut bytes = Vec::new();
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
            bytes.push(u8::from_str_radix(word, 16).ok()?);
        }
        (!bytes.is_empty() && (cut == 0 || bytes.len() == CAPTURED_BYTES)).then_some(Self {
            direction,
            kind,
            bytes,
            cut,
        })
    }
}

impl fmt::Display for Captured {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} {}", self.direction.token(), self.kind.token())?;
        for byte in &self.bytes {
            write!(f, " {byte:02x}")?;
        }
        if self.cut != 0 {
            write!(f, " +{}", self.cut)?;
        }
        Ok(())
    }
}

/// What the function captures, and what it holds until the owner takes it.
#[derive(Debug, Clone, Default)]
pub(super) struct Capture {
    /// Whether the owner asked for the capture.
    enabled: bool,
    /// Packets still captured in this configuration.
    left: u16,
    held: Vec<Captured>,
}

impl Capture {
    /// The owner asks for the capture; it starts at once on a configured
    /// function.
    pub(super) fn enable(&mut self, configured: bool) {
        if !self.enabled {
            self.enabled = true;
            if configured {
                self.left = CAPTURED_PACKETS;
            }
        }
    }

    /// The guest set the configuration to `value` (0: unconfigured).
    pub(super) fn configured(&mut self, value: u8) {
        self.left = if self.enabled && value != 0 {
            CAPTURED_PACKETS
        } else {
            0
        };
    }

    /// Captures `packet`; true when it was this configuration's last.
    pub(super) fn record(&mut self, direction: Direction, kind: Kind, packet: &[u8]) -> bool {
        if self.left == 0 {
            return false;
        }
        self.left -= 1;
        if self.held.len() < usize::from(CAPTURED_PACKETS) {
            self.held.push(Captured::new(direction, kind, packet));
        }
        self.left == 0
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
        assert!(line.starts_with("chip< acl 00 01 02 ") && line.ends_with(" 2b +725"));
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

    #[test]
    fn the_capture_holds_its_bound_per_configuration_and_only_once_asked() {
        let mut capture = Capture::default();
        capture.configured(1);
        assert!(!capture.record(Direction::FromGuest, Kind::Command, &[3, 0x0c, 0]));
        assert!(capture.take().is_empty());
        capture.enable(true);
        for _ in 1..CAPTURED_PACKETS {
            assert!(!capture.record(Direction::ToGuest, Kind::Event, &[0x0e, 0]));
        }
        assert!(capture.record(Direction::ToGuest, Kind::Event, &[0x0e, 0]));
        assert!(!capture.record(Direction::ToGuest, Kind::Event, &[0x0e, 0]));
        assert_eq!(capture.take().len(), usize::from(CAPTURED_PACKETS));
        capture.configured(0);
        assert!(!capture.record(Direction::FromGuest, Kind::Command, &[3, 0x0c, 0]));
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
}

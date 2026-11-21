//! Whole multi-contact touch frames ([`Frame`]): every contact a
//! precision touchpad lists in one scan, in micrometres from its top-left
//! corner, with its buttons and the scan's time. A consumer that models a
//! multitouch device (the guest's Magic Trackpad 2) needs the contacts
//! themselves; the mouse path ([`super::touchpad`]) flattens them to one
//! point. Positions in micrometres let a consumer place them knowing only
//! the pad's [`Surface`], not its units.
//!
//! A frame lists the contacts of one scan only: a contact left out of the
//! next frame lifted, and one listed with its tip up is lifting.

/// The contacts one frame holds at most.
pub const MAX_FINGERS: usize = 10;

/// One contact of a frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Finger {
    /// The pad's contact identifier: the same contact keeps it from
    /// landing to lifting.
    pub id: u8,
    /// From the pad's top-left corner, y downward.
    pub x_um: u32,
    pub y_um: u32,
    /// Touching the surface (HID Tip Switch).
    pub tip: bool,
    /// The pad takes the contact for a finger; `false` for a palm or
    /// another large contact (HID Confidence).
    pub confident: bool,
}

/// One scan's contacts, buttons and time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    /// When the pad scanned it, in microseconds of a clock that only runs
    /// forward.
    pub time_us: u64,
    /// The pad's buttons held, button 1 in bit 0.
    pub buttons: u8,
    count: u8,
    fingers: [Finger; MAX_FINGERS],
}

impl Frame {
    /// A frame of `fingers`' first [`MAX_FINGERS`], in their order.
    pub fn new(time_us: u64, buttons: u8, fingers: &[Finger]) -> Self {
        let count = fingers.len().min(MAX_FINGERS);
        let mut held = [Finger::default(); MAX_FINGERS];
        held[..count].copy_from_slice(&fingers[..count]);
        Self {
            time_us,
            buttons,
            count: count as u8,
            fingers: held,
        }
    }

    /// Nothing on the pad and no button held: what a device that stopped
    /// leaves.
    pub fn lifted(time_us: u64) -> Self {
        Self::new(time_us, 0, &[])
    }

    pub fn fingers(&self) -> &[Finger] {
        &self.fingers[..usize::from(self.count)]
    }

    /// Whether the frame holds anything down: a contact touching, or a
    /// button.
    pub fn holds(&self) -> bool {
        self.buttons != 0 || self.fingers().iter().any(|finger| finger.tip)
    }
}

/// The size of a pad's surface: the extent its contacts' positions span.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Surface {
    pub width_um: u32,
    pub height_um: u32,
}

#[cfg(test)]
mod tests;

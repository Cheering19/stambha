//! The firmware's picture while GPU claims hold it.
//!
//! A claim of a GPU that may show the firmware's screen stops the boot's
//! writes to that framebuffer before it changes the GPU (its BAR sizing turns
//! memory decoding off for a moment). The picture itself lasts until a step
//! ends it: a reset of the GPU, by its claim or by its driver, or the start
//! of the driver's display engine, which blanks what the firmware lit. A
//! driver bound to the GPU drives that screen from then on. A claim that
//! lets go with none of those, as a driver whose probe failed before them,
//! leaves the picture as it was, and the boot draws on it again: its text,
//! a stop's reason, or macOS's frames copied from another GPU.
use alloc::vec::Vec;

/// The claims holding the firmware's picture, by PCI bus and devfn, and
/// whether a step ended it.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Held {
    holders: Vec<(u8, u8)>,
    ended: bool,
}

impl Held {
    pub const fn new() -> Self {
        Self {
            holders: Vec::new(),
            ended: false,
        }
    }

    /// The claim of the GPU at `bus` and `devfn` stopped the boot's writes
    /// to the picture.
    pub fn hold(&mut self, bus: u8, devfn: u8) {
        if !self.holders.contains(&(bus, devfn)) {
            self.holders.push((bus, devfn));
        }
    }

    /// The GPU at `bus` and `devfn` ended the picture: it was reset, its
    /// driver started its display engine, or its driver bound it.
    pub fn end(&mut self, bus: u8, devfn: u8) {
        self.holders.retain(|&holder| holder != (bus, devfn));
        self.ended = true;
    }

    /// The claim of the GPU at `bus` and `devfn` let go without ending the
    /// picture. Whether the boot may draw on it again now: no claim holds
    /// it any more, and none ended it.
    pub fn release(&mut self, bus: u8, devfn: u8) -> bool {
        self.holders.retain(|&holder| holder != (bus, devfn));
        self.holders.is_empty() && !self.ended
    }

    /// At a stop: whether the picture still shows, so the stop's reason may
    /// be drawn on it. No step ended it, and `ended_in_probe` says for each
    /// claim still holding it whether its driver ended it in a probe that
    /// never returned, the probe the stop may come from.
    pub fn shows_at_stop(&self, ended_in_probe: impl Fn(u8, u8) -> bool) -> bool {
        !self.ended
            && !self
                .holders
                .iter()
                .any(|&(bus, devfn)| ended_in_probe(bus, devfn))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const IGPU: (u8, u8) = (1, 0);
    const CARD: (u8, u8) = (3, 0);

    #[test]
    fn a_driver_that_fails_before_any_step_leaves_the_picture() {
        let mut held = Held::new();
        held.hold(IGPU.0, IGPU.1);
        // During the probe, a stop may draw on it.
        assert!(held.shows_at_stop(|_, _| false));
        assert!(held.release(IGPU.0, IGPU.1));
        assert!(held.shows_at_stop(|_, _| false));
    }

    #[test]
    fn a_reset_or_a_bound_driver_ends_the_picture_for_good() {
        let mut held = Held::new();
        held.hold(IGPU.0, IGPU.1);
        held.end(IGPU.0, IGPU.1);
        assert!(!held.shows_at_stop(|_, _| false));
        // A later claim that lets go does not bring it back.
        held.hold(CARD.0, CARD.1);
        assert!(!held.release(CARD.0, CARD.1));
    }

    #[test]
    fn a_driver_ending_the_picture_in_a_probe_that_stopped_is_seen_at_the_stop() {
        let mut held = Held::new();
        held.hold(IGPU.0, IGPU.1);
        assert!(!held.shows_at_stop(|bus, devfn| (bus, devfn) == IGPU));
        assert!(held.shows_at_stop(|bus, devfn| (bus, devfn) == CARD));
    }

    #[test]
    fn the_picture_comes_back_only_after_the_last_claim_lets_go() {
        // Where the screen's function is unknown, every GPU that may show
        // it holds it.
        let mut held = Held::new();
        held.hold(IGPU.0, IGPU.1);
        held.hold(CARD.0, CARD.1);
        held.hold(CARD.0, CARD.1);
        assert!(!held.release(IGPU.0, IGPU.1));
        assert!(held.release(CARD.0, CARD.1));
        // One that ends it after another let go keeps it ended.
        let mut held = Held::new();
        held.hold(IGPU.0, IGPU.1);
        held.hold(CARD.0, CARD.1);
        assert!(!held.release(IGPU.0, IGPU.1));
        held.end(CARD.0, CARD.1);
        assert!(!held.shows_at_stop(|_, _| false));
    }
}

//! A laptop's lid and its built-in panel, with `--lid method`.
//!
//! Linux's ACPI button driver only reports the lid; what follows is its
//! userspace's (logind waits after a boot and ignores a closed lid while
//! an external display is connected), and a wrong `_LID` is common enough
//! that Linux keeps DMI quirks for it (`button.c`'s lid quirks: machines
//! that read closed at boot, or always). So the reading a boot starts with
//! counts for nothing here ([`LidRequest`]): only a lid notification after
//! which `_LID` reads closed darkens the panel, and the next lid
//! notification, whatever `_LID` then reads, or any input of the built-in
//! keyboard or touchpad lights it again, so a wrong reading never leaves
//! the screen dark. A display that an external monitor shows as well stays
//! lit ([`PanelPower`]): its outputs go dark together.

/// The lid's request of a built-in panel: dark or not.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LidRequest {
    dark: bool,
}

impl LidRequest {
    /// The request `dark` already is.
    pub const fn with(dark: bool) -> Self {
        Self { dark }
    }

    pub fn dark(self) -> bool {
        self.dark
    }

    /// A lid notification, after which `_LID` read `open`: the request it
    /// changed to, where it changed. A dark panel lights at any.
    pub fn notified(&mut self, open: bool) -> Option<bool> {
        let dark = !self.dark && !open;
        self.change(dark)
    }

    /// Input of the built-in keyboard or touchpad: a dark panel lights.
    pub fn input(&mut self) -> Option<bool> {
        self.change(false)
    }

    fn change(&mut self, dark: bool) -> Option<bool> {
        (dark != self.dark).then(|| {
            self.dark = dark;
            dark
        })
    }
}

/// What a lid request does to the guest's display.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LidPower {
    /// Its outputs go dark.
    Off,
    /// Its outputs light again.
    On,
    /// The lid no longer asks it dark, but the guest's display sleeps.
    StaysOff,
    /// An external monitor shows it too: it stays as the guest has it.
    KeptLit,
}

/// A guest display's power as its display sleep and the lid ask.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PanelPower {
    /// The guest's display is on.
    guest_on: bool,
    /// The lid's request last followed.
    request: bool,
    /// The display is dark for the lid.
    lid_dark: bool,
}

impl Default for PanelPower {
    fn default() -> Self {
        Self::new()
    }
}

impl PanelPower {
    pub const fn new() -> Self {
        Self {
            guest_on: true,
            request: false,
            lid_dark: false,
        }
    }

    /// The lid's request last followed.
    pub fn request(&self) -> bool {
        self.request
    }

    /// Whether the outputs are lit.
    pub fn lit(&self) -> bool {
        self.guest_on && !self.lid_dark
    }

    /// The guest's display power `on`: whether the outputs are lit now.
    pub fn guest(&mut self, on: bool) -> bool {
        self.guest_on = on;
        self.lit()
    }

    /// The lid's request `dark` ([`LidRequest`]), for a display shown on
    /// built-in panels alone where `panels_only`: what it does, `None` for
    /// a request already followed, or one to light a display the lid never
    /// darkened.
    pub fn lid(&mut self, dark: bool, panels_only: bool) -> Option<LidPower> {
        if dark == self.request {
            return None;
        }
        self.request = dark;
        if dark {
            if !panels_only {
                return Some(LidPower::KeptLit);
            }
            self.lid_dark = true;
            return Some(LidPower::Off);
        }
        if !self.lid_dark {
            return None;
        }
        self.lid_dark = false;
        Some(if self.guest_on {
            LidPower::On
        } else {
            LidPower::StaysOff
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_a_notified_close_darkens_and_any_notification_or_input_lights() {
        let mut request = LidRequest::default();
        // An open reading asks nothing; a closed one after a notification
        // darkens.
        assert_eq!(request.notified(true), None);
        assert_eq!(request.notified(false), Some(true));
        // The next notification lights, whatever _LID reads: a lid that
        // reads closed for good does not keep the screen dark.
        assert_eq!(request.notified(false), Some(false));
        assert_eq!(request.notified(false), Some(true));
        assert_eq!(request.notified(true), Some(false));
        // Built-in input lights a dark panel, and leaves a lit one.
        assert_eq!(request.input(), None);
        assert_eq!(request.notified(false), Some(true));
        assert_eq!(request.input(), Some(false));
        assert!(!request.dark());
        assert!(LidRequest::with(true).dark());
    }

    #[test]
    fn a_closed_lid_darkens_only_a_display_on_built_in_panels_alone() {
        let mut panel = PanelPower::new();
        assert!(panel.lit());
        assert_eq!(panel.lid(true, true), Some(LidPower::Off));
        assert!(!panel.lit());
        // A wake of the guest's display stays dark under the lid.
        assert!(!panel.guest(true));
        assert_eq!(panel.lid(true, true), None);
        assert_eq!(panel.lid(false, true), Some(LidPower::On));
        assert!(panel.lit());
        // Lit again while the guest's display sleeps: it stays off.
        assert_eq!(panel.lid(true, true), Some(LidPower::Off));
        assert!(!panel.guest(false));
        assert_eq!(panel.lid(false, true), Some(LidPower::StaysOff));
        assert!(panel.guest(true));
        // An external monitor showing it too keeps it as the guest has it.
        assert_eq!(panel.lid(true, false), Some(LidPower::KeptLit));
        assert!(panel.lit());
        assert_eq!(panel.lid(false, false), None);
        assert!(panel.lit());
    }
}

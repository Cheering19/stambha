//! A built-in panel's power while the host's lid is shut. The guest's
//! display sleep turns its display's outputs off and on; a closed lid
//! turns them off as well where they are all built-in panels, and an open
//! lid gives the guest its say back. An external monitor that shows the
//! display too keeps it lit: the display's outputs go dark together.

/// Whether the guest's display is lit: while the guest's display is on,
/// unless the lid is closed and each of its outputs is a built-in panel.
pub fn lit(guest_on: bool, lid_closed: bool, panels_only: bool) -> bool {
    guest_on && !(lid_closed && panels_only)
}

#[cfg(test)]
mod tests {
    use super::lit;

    #[test]
    fn a_closed_lid_darkens_only_a_display_on_built_in_panels_alone() {
        // The lid open: the guest's display sleep decides.
        assert!(lit(true, false, true));
        assert!(!lit(false, false, true));
        // Closed, on the panel alone: dark, however the guest's display is.
        assert!(!lit(true, true, true));
        assert!(!lit(false, true, true));
        // Closed, with an external monitor showing it too: the guest decides.
        assert!(lit(true, true, false));
        assert!(!lit(false, true, false));
    }
}

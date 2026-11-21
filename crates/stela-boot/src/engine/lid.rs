//! The host's lid as ACPI last read it (`aml`), for the scanout: while it
//! is closed, a guest display shown on built-in panels alone stays dark
//! (`gpu::display`, [`stela_boot_core::lid_panel`]).
use core::sync::atomic::{AtomicBool, Ordering};

static CLOSED: AtomicBool = AtomicBool::new(false);

/// The lid was read open, or closed.
pub(in crate::engine) fn set(open: bool) {
    CLOSED.store(!open, Ordering::Release);
}

/// Whether the lid was last read closed; open until a reading says not.
#[cfg_attr(not(feature = "nvk"), allow(dead_code))]
pub(in crate::engine) fn closed() -> bool {
    CLOSED.load(Ordering::Acquire)
}

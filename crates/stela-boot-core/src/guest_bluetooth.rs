//! The boot log's one line about the guest's Bluetooth. A boot with host
//! input gives macOS a model of Apple's internal USB Bluetooth module
//! (`stela_soc::usb::hci::model`): BlueTool's chip boot matches it and
//! bluetoothd brings it up, so the daemon stays up and Setup's first screen
//! (Language Chooser), which asks it for the paired devices before it
//! draws, gets an empty list at once.
//!
//! The model has no radio. It is powered, has a stable locally administered
//! address, accepts scans and connection attempts and finds nothing, and has
//! no paired devices. That is a degraded answer, not Bluetooth, and the line
//! says so once. A host controller is not lent to the guest unless the boot
//! asks for it (`--bluetooth-lend`): it then sits behind the same module,
//! which falls back to the model when the controller fails or stalls, and
//! [`lend_line`] names it instead.
use alloc::{format, string::String};

/// The line for a boot that logs the guest's Bluetooth packets
/// (`--bluetooth-trace`): what is hidden of them, and what is not.
pub const TRACE_LINE: &str = "BLUETOOTH TRACE the guest's Bluetooth packets are logged masked: \
     addresses keep their vendor half and last byte, and names, payloads and keys are hidden; \
     the log still names nearby devices' classes and signal strengths, so keep it private \
     until it has been checked\n";

/// Why a boot that asks to lend the host's controller (`--bluetooth-lend`)
/// does not, as its log line; `None` when nothing stops it, so the flag is
/// never ignored without a word. A boot without `--input` gives the guest
/// no Bluetooth controller at all; a host with no controller that started
/// has none to lend; and under `--idle-skip` the guest's clock leaps over
/// waits to the next deadline, and the bridge's wait for the controller's
/// answer is one in guest time, so every forwarded command would fall back
/// at once.
pub fn lend_refusal(input: bool, host_controller: bool, idle_skip: bool) -> Option<&'static str> {
    if !input {
        return Some(
            "BLUETOOTH LEND not lent: the boot has no --input, so the guest has no Bluetooth \
             controller\n",
        );
    }
    if !host_controller {
        return Some(
            "BLUETOOTH LEND not lent: no host Bluetooth controller started, so the model \
             answers alone\n",
        );
    }
    idle_skip.then_some(
        "BLUETOOTH LEND not lent: --idle-skip leaps the guest's clock over the bridge's wait \
         for the controller's answers, so the model answers alone\n",
    )
}

/// The line for a boot that lends the host's controller at `location` to
/// the guest behind Apple's module.
pub fn lend_line(location: &str) -> String {
    format!(
        "BLUETOOTH LEND the guest's controller is {location} behind Apple's module; \
         it falls back to the model on failure\n"
    )
}

/// The line for a boot that gives macOS host input, by whether the host
/// has a Bluetooth controller; `None` for a boot without host input.
pub fn model_line(host_controller: bool, input: bool) -> Option<&'static str> {
    if !input {
        return None;
    }
    Some(if host_controller {
        "BLUETOOTH MODEL the guest's controller is a model with no radio: it is powered, \
         finds no devices and has none paired; this computer's controller is not lent to it\n"
    } else {
        "BLUETOOTH MODEL the guest's controller is a model with no radio: it is powered, \
         finds no devices and has none paired\n"
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_boot_with_input_says_once_that_the_controller_has_no_radio() {
        for host_controller in [false, true] {
            let line = model_line(host_controller, true).expect("reported");
            assert!(line.starts_with("BLUETOOTH MODEL the guest's controller is a model "));
            assert!(line.contains("no radio"));
            assert!(line.contains("finds no devices and has none paired"));
            assert!(line.ends_with('\n') && line.matches('\n').count() == 1);
        }
    }

    /// Under `--idle-skip` the guest's clock leaps to the next deadline, the
    /// bridge's wait for the controller among them, so a lent controller
    /// would fall back at once: it is not lent, and the log says why.
    #[test]
    fn idle_skip_keeps_the_controller_from_being_lent_and_says_why() {
        assert_eq!(lend_refusal(true, true, false), None);
        let line = lend_refusal(true, true, true).expect("refused");
        assert!(line.starts_with("BLUETOOTH LEND not lent: --idle-skip "));
        assert!(line.contains("model"));
        assert!(line.ends_with('\n') && line.matches('\n').count() == 1);
    }

    /// `--bluetooth-lend` is never dropped without a word: a boot without
    /// host input, or a host with no controller that started, says why
    /// nothing is lent, before `--idle-skip` is weighed.
    #[test]
    fn a_lend_with_no_input_or_no_controller_says_why_it_is_not_lent() {
        for idle_skip in [false, true] {
            let line = lend_refusal(false, true, idle_skip).expect("refused");
            assert!(line.starts_with("BLUETOOTH LEND not lent: the boot has no --input"));
            assert_eq!(lend_refusal(false, false, idle_skip), Some(line));
            let line = lend_refusal(true, false, idle_skip).expect("refused");
            assert!(line.starts_with("BLUETOOTH LEND not lent: no host Bluetooth controller"));
            assert!(line.contains("model"));
        }
        for input in [false, true] {
            for host_controller in [false, true] {
                for idle_skip in [false, true] {
                    let line = lend_refusal(input, host_controller, idle_skip);
                    assert_eq!(line.is_none(), input && host_controller && !idle_skip);
                    assert!(line.is_none_or(|l| l.ends_with('\n') && l.matches('\n').count() == 1));
                }
            }
        }
    }

    /// `--bluetooth-trace` says once what its lines hide and that the log
    /// still needs checking before it is posted.
    #[test]
    fn the_capture_says_what_it_hides_and_that_the_log_stays_private() {
        assert!(TRACE_LINE.starts_with("BLUETOOTH TRACE "));
        for said in ["masked", "names", "keys", "hidden", "private"] {
            assert!(TRACE_LINE.contains(said), "{said}");
        }
        assert!(TRACE_LINE.ends_with('\n') && TRACE_LINE.matches('\n').count() == 1);
    }

    /// `--bluetooth-lend`: the line names the controller lent, the module it
    /// sits behind, and the fallback.
    #[test]
    fn a_lent_controller_is_named_with_its_fallback() {
        let line = lend_line("xhci 0000:05:00.4 port 2.3");
        assert_eq!(
            line,
            "BLUETOOTH LEND the guest's controller is xhci 0000:05:00.4 port 2.3 behind \
             Apple's module; it falls back to the model on failure\n"
        );
        assert_eq!(line.matches('\n').count(), 1);
    }

    #[test]
    fn a_host_controller_is_named_as_not_lent_and_a_boot_without_input_says_nothing() {
        let line = model_line(true, true).expect("reported");
        assert!(line.contains("controller is not lent"));
        assert!(!model_line(false, true).unwrap().contains("lent"));
        assert_eq!(model_line(true, false), None);
        assert_eq!(model_line(false, false), None);
    }
}

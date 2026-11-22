extern crate std;

use super::*;
use crate::platform::power_snapshot::TestState;

fn reading(percent: u8) -> PowerSnapshot {
    PowerSnapshot::synthetic(percent, TestState::Discharging)
}

#[test]
fn latest_counts_writes_and_hands_each_newer_value_on_once() {
    let latest = Latest::<u32>::new();
    let seen = AtomicU32::new(0);
    assert_eq!(latest.load(), (None, 0));
    assert_eq!(latest.take_newer(&seen), None);
    latest.store(7);
    latest.store(8);
    assert_eq!(latest.load(), (Some(8), 2));
    assert_eq!(latest.take_newer(&seen), Some(8));
    assert_eq!(seen.load(Ordering::Relaxed), 2);
    assert_eq!(latest.take_newer(&seen), None);
    latest.store(9);
    assert_eq!(latest.take_newer(&seen), Some(9));
}

#[test]
fn a_reader_never_waits_for_a_writer_and_takes_the_value_later() {
    let latest = Latest::<u32>::new();
    let seen = AtomicU32::new(0);
    latest.store(1);
    // A writer holds the flag: the reader leaves it for a later call.
    latest.held.store(true, Ordering::Release);
    assert_eq!(latest.take_newer(&seen), None);
    assert_eq!(seen.load(Ordering::Relaxed), 0);
    latest.held.store(false, Ordering::Release);
    assert_eq!(latest.take_newer(&seen), Some(1));
}

#[test]
fn values_cross_processors_whole() {
    // Each value is a pair that must arrive as written.
    let latest = std::sync::Arc::new(Latest::<(u64, u64)>::new());
    let writer = {
        let latest = latest.clone();
        std::thread::spawn(move || {
            for n in 1..=20_000u64 {
                latest.store((n, n.wrapping_mul(0x9e37_79b9_7f4a_7c15)));
            }
        })
    };
    let seen = AtomicU32::new(0);
    let mut last = 0;
    while last < 20_000 {
        if let Some((n, check)) = latest.take_newer(&seen) {
            assert_eq!(check, n.wrapping_mul(0x9e37_79b9_7f4a_7c15));
            assert!(n >= last);
            last = n;
        }
    }
    writer.join().unwrap();
}

#[test]
fn a_turn_refreshes_every_30_s_passes_a_newer_reading_once_and_flushes_when_due() {
    let relay = Relay::new();
    // The first turn asks for a reading at once; the next only 30 s later.
    let first = relay.turn(1_000);
    assert_eq!(
        first,
        Turn {
            refresh: true,
            reading: None,
            flush: false,
            expired: false,
            keys: false,
        }
    );
    assert!(!relay.turn(1_000 + REFRESH_MILLIS - 1).refresh);
    assert!(relay.turn(1_000 + REFRESH_MILLIS).refresh);
    // A published reading goes on once.
    relay.publish(reading(70), Some(0));
    assert_eq!(relay.turn(40_000).reading, Some(reading(70)));
    assert_eq!(relay.turn(40_001).reading, None);
    // A notification the SMC holds is flushed once due, and not again.
    relay.delivered(Some(45_000));
    assert!(!relay.turn(44_999).flush);
    assert!(relay.turn(45_000).flush);
    relay.delivered(None);
    assert!(!relay.turn(90_000).flush);
    // With a newer reading a turn passes it on rather than flush.
    relay.delivered(Some(50_000));
    relay.publish(reading(69), Some(0));
    let turn = relay.turn(60_000);
    assert_eq!((turn.reading, turn.flush), (Some(reading(69)), false));
}

#[test]
fn a_relay_waiting_for_a_complete_reading_asks_every_second_then_every_30_once_attached() {
    let relay = Relay::new();
    assert_eq!(relay.mode(), Mode::Off);
    relay.wait(10_000);
    assert_eq!(relay.mode(), Mode::Waiting);
    assert!(relay.turn(10_000).refresh);
    assert!(!relay.turn(10_000 + WAITING_REFRESH_MILLIS - 1).refresh);
    assert!(relay.turn(10_000 + WAITING_REFRESH_MILLIS).refresh);
    relay.attached();
    assert_eq!(relay.mode(), Mode::Attached);
    assert!(!relay.turn(12_000).refresh);
    assert!(relay.turn(11_000 + REFRESH_MILLIS).refresh);
    // Attached, the wait's end means nothing.
    assert!(!relay.turn(10_000 + WAIT_MILLIS).expired);
    relay.give_up();
    assert_eq!(relay.mode(), Mode::Off);
}

#[test]
fn a_wait_with_no_complete_reading_ends_once() {
    let relay = Relay::new();
    relay.wait(5_000);
    assert!(!relay.turn(5_000 + WAIT_MILLIS - 1).expired);
    let ended = relay.turn(5_000 + WAIT_MILLIS);
    assert!(ended.expired && !ended.refresh);
    assert_eq!(relay.mode(), Mode::Off);
    assert!(!relay.turn(5_000 + WAIT_MILLIS + 1).expired);
}

#[test]
fn a_failed_or_slow_reading_doubles_the_period_up_to_5_minutes_and_says_so_once() {
    let relay = Relay::new();
    relay.attached();
    assert!(relay.turn(0).refresh);
    // Failing: the period doubles each time, from 30 s to 5 min at most,
    // and only the first failure is said.
    let first = relay.polled(true, 2);
    assert_eq!(
        first,
        Polled {
            failed: true,
            slow: false,
            recovered: false,
            period_ms: 2 * REFRESH_MILLIS
        }
    );
    let mut period = 2 * REFRESH_MILLIS;
    for _ in 0..6 {
        let again = relay.polled(true, 2);
        period = (period * 2).min(MAX_REFRESH_MILLIS);
        assert_eq!(again.period_ms, period);
        assert!(!again.failed);
    }
    assert_eq!(period, MAX_REFRESH_MILLIS);
    // The turns follow the longer period.
    assert!(!relay.turn(MAX_REFRESH_MILLIS - 1).refresh);
    assert!(relay.turn(MAX_REFRESH_MILLIS).refresh);
    // A success says it recovered and goes back to 30 s.
    let back = relay.polled(false, 2);
    assert!(back.recovered && !back.failed);
    assert_eq!(back.period_ms, REFRESH_MILLIS);
    assert!(!relay.polled(false, 2).recovered);
    // A reading that took past the bound backs off as a failure does, and
    // is said once.
    let slow = relay.polled(false, SLOW_REFRESH_MILLIS);
    assert!(slow.slow && !slow.failed);
    assert_eq!(slow.period_ms, 2 * REFRESH_MILLIS);
    assert!(!relay.polled(false, SLOW_REFRESH_MILLIS).slow);
    assert_eq!(relay.polled(false, 1).period_ms, REFRESH_MILLIS);
    // Waiting, the period starts from a second.
    let waiting = Relay::new();
    waiting.wait(0);
    assert_eq!(
        waiting.polled(true, 1).period_ms,
        2 * WAITING_REFRESH_MILLIS
    );
}

#[test]
fn an_attached_relay_drains_the_smcs_requests_every_2_s() {
    let relay = Relay::new();
    relay.wait(0);
    assert!(!relay.turn(0).keys);
    relay.attached();
    assert!(relay.turn(100).keys);
    assert!(!relay.turn(100 + KEYS_MILLIS - 1).keys);
    assert!(relay.turn(100 + KEYS_MILLIS).keys);
}

#[test]
fn the_attach_line_says_where_the_battery_comes_from_and_what_follows_it() {
    use alloc::{format, string::ToString};
    let snapshot = PowerSnapshot::synthetic(80, TestState::Charging);
    let line = |source| {
        Attached {
            snapshot: &snapshot,
            source,
        }
        .to_string()
    };
    let start = format!("{snapshot} warning=480 mAh low=144 mAh (the SMC's battery from ");
    assert_eq!(
        line(Source::Test),
        format!("{start}--test-battery: one pack, this reading for the whole boot)")
    );
    let host = "the host's ACPI battery: one pack for the whole boot, which keeps its last \
                reading if the battery is removed; ";
    assert_eq!(
        line(Source::Host { follows: true }),
        format!("{start}{host}read again every 30 s and at each notification)")
    );
    assert_eq!(
        line(Source::Host { follows: false }),
        format!("{start}{host}ACPI hardware mode is off, so no reading follows this one)")
    );
}

#[test]
fn a_change_line_with_no_battery_says_the_smc_keeps_the_last_reading() {
    use alloc::string::ToString;
    let gone = PowerSnapshot {
        ac_online: Some(true),
        battery: None,
    };
    assert_eq!(
        Change(&gone).to_string(),
        "battery none ac=1 (the SMC keeps its last reading and its one pack)"
    );
    let reading = reading(70);
    assert_eq!(Change(&reading).to_string(), reading.to_string());
}

#[test]
fn a_held_reading_says_the_smc_shows_it_only_once_the_next_confirms_it() {
    use alloc::{format, string::ToString};
    let empty = PowerSnapshot::synthetic(0, TestState::Charging);
    assert_eq!(
        Held(&empty).to_string(),
        format!(
            "{empty} (held back: a fall to 0 without the firmware's critical bit, which the \
             SMC shows once the next reading confirms it)"
        )
    );
}

#[test]
fn the_reading_at_attach_counts_as_the_guests() {
    let relay = Relay::new();
    assert_eq!(relay.take(), None);
    relay.publish(reading(70), Some(0));
    assert_eq!(relay.take(), Some(reading(70)));
    assert_eq!(relay.turn(1).reading, None);
}

#[test]
fn a_reading_is_of_the_first_battery_with_one_until_the_guest_takes_one_then_of_that_one_alone() {
    // Before the guest's SMC has a battery: the first with a reading.
    assert_eq!(pick(None, [None, Some('b'), Some('c')]), Some((1, 'b')));
    assert_eq!(pick(None, [Some('a'), Some('b')]), Some((0, 'a')));
    assert_eq!(pick::<char>(None, [None, None]), None);
    // After, the battery it took alone: one whose `_BST` failed gives no
    // reading, not another pack's, which the SMC would show as its one.
    assert_eq!(pick(Some(1), [Some('a'), Some('b')]), Some((1, 'b')));
    assert_eq!(pick(Some(0), [None, Some('b')]), None);
    assert_eq!(pick(Some(2), [Some('a'), Some('b')]), None);
}

#[test]
fn the_guest_keeps_the_battery_its_smc_took_and_another_packs_reading_brings_only_the_ac_adapter() {
    // Taken before the guest starts: the battery of the reading at attach.
    let relay = Relay::new();
    relay.publish(reading(70), Some(1));
    assert_eq!(relay.pinned(), None);
    assert_eq!(relay.take(), Some(reading(70)));
    relay.attached();
    assert_eq!(relay.pinned(), Some(1));
    // A reading of another pack, published before the firmware's processor
    // saw the pin, carries the AC adapter alone.
    relay.publish(reading(40), Some(0));
    assert_eq!(
        relay.turn(1_000).reading,
        Some(PowerSnapshot {
            battery: None,
            ..reading(40)
        })
    );
    relay.publish(reading(69), Some(1));
    assert_eq!(relay.turn(2_000).reading, Some(reading(69)));
    // Taken while the guest waited: the battery of the last reading a turn
    // gave, whichever pack the ones before were of.
    let waiting = Relay::new();
    waiting.wait(0);
    waiting.publish(reading(70), Some(0));
    assert_eq!(waiting.turn(0).reading, Some(reading(70)));
    waiting.publish(reading(71), Some(2));
    assert_eq!(waiting.turn(1).reading, Some(reading(71)));
    waiting.attached();
    assert_eq!(waiting.pinned(), Some(2));
    // A battery the SMC refused pins nothing.
    let refused = Relay::new();
    refused.publish(reading(70), Some(0));
    refused.take();
    refused.give_up();
    assert_eq!(refused.pinned(), None);
}

#[test]
fn a_reading_is_logged_when_what_macos_acts_on_changes() {
    let relay = Relay::new();
    assert!(relay.changed(&reading(70)));
    assert!(!relay.changed(&reading(70)));
    let drift = PowerSnapshot {
        battery: reading(70).battery.map(|battery| BatteryState {
            voltage_mv: Some(11_000),
            ..battery
        }),
        ..reading(70)
    };
    assert!(!relay.changed(&drift));
    assert!(relay.changed(&reading(69)));
    assert!(relay.changed(&PowerSnapshot::synthetic(69, TestState::Charging)));
}

use crate::platform::power_snapshot::BatteryState;

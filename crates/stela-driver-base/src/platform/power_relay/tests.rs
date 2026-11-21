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
            flush: false
        }
    );
    assert!(!relay.turn(1_000 + REFRESH_MILLIS - 1).refresh);
    assert!(relay.turn(1_000 + REFRESH_MILLIS).refresh);
    // A published reading goes on once.
    relay.publish(reading(70));
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
    relay.publish(reading(69));
    let turn = relay.turn(60_000);
    assert_eq!((turn.reading, turn.flush), (Some(reading(69)), false));
}

#[test]
fn the_reading_at_attach_counts_as_the_guests() {
    let relay = Relay::new();
    assert_eq!(relay.take(), None);
    relay.publish(reading(70));
    assert_eq!(relay.take(), Some(reading(70)));
    assert_eq!(relay.turn(1).reading, None);
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

extern crate std;
use super::*;
use core::cell::Cell;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

#[test]
fn the_plan_reserves_one_more_only_when_asked_parallel_and_room_is_left() {
    assert_eq!(plan(false, true, 8, 16), Plan::Off);
    assert_eq!(plan(true, true, 8, 16), Plan::Reserve { executors: 9 });
    // The FA608PP: 8 PEs of 16 threads leaves seven spare.
    assert_eq!(Plan::Reserve { executors: 9 }.executors(8), 9);
    // Exactly one spare is enough; none is not.
    assert_eq!(plan(true, true, 15, 16), Plan::Reserve { executors: 16 });
    assert!(matches!(plan(true, true, 16, 16), Plan::Refused(_)));
    assert!(matches!(plan(true, true, 64, 64), Plan::Refused(_)));
    assert!(matches!(
        plan(true, true, usize::MAX, usize::MAX),
        Plan::Refused(_)
    ));
    // Serialized: one processor runs every PE, so there is nothing to add.
    assert!(matches!(plan(true, false, 1, 16), Plan::Refused(_)));
    assert!(matches!(plan(true, true, 0, 16), Plan::Refused(_)));
    // Off or refused, the boot reserves its PEs alone: fail closed.
    for plan in [Plan::Off, plan(true, true, 16, 16), plan(true, false, 1, 4)] {
        assert_eq!(plan.executors(16), 16);
    }
}

#[test]
fn the_host_core_exists_only_when_every_planned_processor_started() {
    assert_eq!(confirm(9, 9, 9), Ok(()));
    // The host core's own slot, the last, failed: the PEs are whole.
    assert!(confirm(9, 9, 8).is_err());
    // An earlier slot failed and took the rest with it.
    assert!(confirm(9, 9, 5).is_err());
    // No startup page: the reservation fell back to one processor.
    assert!(confirm(9, 1, 1).is_err());
    assert!(confirm(9, 8, 8).is_err());
}

/// A counter that moves one tick a look, for the boot processor's waits.
fn ticking() -> impl FnMut() -> u64 {
    let now = Cell::new(0u64);
    move || {
        now.set(now.get() + 1);
        now.get()
    }
}

#[test]
fn a_ping_answered_in_time_has_the_host_core_serve() {
    let channel = Channel::new();
    assert_eq!(channel.state(), State::Off);
    // Nothing waits and nothing is served before the check.
    assert_eq!(channel.take(), None);
    assert_eq!(channel.turn(0), Turn::Idle);
    // The host core answers on the boot processor's third look.
    let mut looks = 0;
    let took = channel.self_check(ticking(), 100, || {
        looks += 1;
        if looks == 3 {
            assert_eq!(channel.state(), State::Checking);
            assert_eq!(channel.turn(5), Turn::Idle);
        }
    });
    assert!(took.is_ok(), "{took:?}");
    assert_eq!(channel.state(), State::Serving);
    // Answered once: the next pass finds nothing to answer, and serves.
    assert_eq!(channel.take(), None);
    assert_eq!(channel.turn(10), Turn::Serve);
}

#[test]
fn an_unanswered_ping_falls_back_and_a_late_answer_changes_nothing() {
    let channel = Channel::new();
    let took = channel.self_check(ticking(), 50, || {});
    assert!(took.is_err());
    assert_eq!(channel.state(), State::Fallback);
    // The host core wakes late: it answers, but never serves.
    assert_eq!(channel.turn(1_000), Turn::Idle);
    assert_eq!(channel.state(), State::Fallback);
    assert!(!channel.covers(1_000, 100));
    assert_eq!(channel.turn(2_000), Turn::Idle);
}

#[test]
fn pe0_leaves_the_pump_only_to_a_serving_host_core_with_a_fresh_beat() {
    let channel = Channel::new();
    assert!(!channel.covers(0, 100));
    channel
        .self_check(ticking(), 100, || {
            channel.turn(0);
        })
        .unwrap();
    assert_eq!(channel.turn(1_000), Turn::Serve);
    assert!(channel.covers(1_050, 100));
    // Another core's counter a little behind the beat is still fresh.
    assert!(channel.covers(990, 100));
    // A host core that stopped beating gives the work back at once.
    assert!(!channel.covers(1_100, 100));
    assert!(!channel.covers(50_000, 100));
    assert_eq!(channel.turn(50_000), Turn::Serve);
    assert!(channel.covers(50_001, 100));
}

#[test]
fn the_stop_is_acknowledged_and_nothing_serves_after_it() {
    let channel = Channel::new();
    // No host core: nothing to stop.
    assert!(channel.stop(ticking(), 10, || {}));
    assert_eq!(channel.state(), State::Off);
    channel
        .self_check(ticking(), 100, || {
            channel.turn(0);
        })
        .unwrap();
    let mut left = false;
    let acknowledged = channel.stop(ticking(), 100, || {
        // From the stop's first word nothing serves, even before the
        // host core reads it.
        assert!(!channel.covers(0, u64::MAX >> 2));
        if !left {
            left = channel.turn(0) == Turn::Leave;
        }
    });
    assert!(acknowledged);
    assert!(left);
    assert_eq!(channel.state(), State::Stopped);
    assert_eq!(channel.turn(0), Turn::Leave);
}

#[test]
fn a_host_core_that_never_answers_its_stop_is_given_up_after_the_limit() {
    let channel = Channel::new();
    channel
        .self_check(ticking(), 100, || {
            channel.turn(0);
        })
        .unwrap();
    assert!(!channel.stop(ticking(), 20, || {}));
    // Given up: it serves nothing from then on, and leaves when it looks.
    assert_eq!(channel.state(), State::Stopped);
    assert_eq!(channel.turn(0), Turn::Leave);
    assert!(!channel.covers(0, 100));
}

#[test]
fn requests_are_answered_in_order_and_once() {
    let channel = Channel::new();
    let first = channel.post(Request::Ping);
    assert_eq!(channel.take(), Some((first, Request::Ping)));
    assert!(!channel.answered(first));
    channel.turn(0);
    assert!(channel.answered(first));
    let second = channel.post(Request::Ping);
    assert!(second > first);
    assert!(!channel.answered(second));
    assert_eq!(channel.take(), Some((second, Request::Ping)));
    channel.turn(0);
    assert!(channel.answered(second));
    let stop = channel.post(Request::Stop);
    assert_eq!(channel.take(), Some((stop, Request::Stop)));
    assert_eq!(channel.turn(0), Turn::Leave);
    assert!(channel.answered(stop));
}

/// The handoff across two threads, as across the boot processor and the
/// host core: the check passes, the host core beats while serving, and the
/// stop ends its loop.
#[test]
fn the_handoff_between_two_processors_checks_serves_and_stops() {
    let channel = Arc::new(Channel::new());
    let served = Arc::new(AtomicU64::new(0));
    let running = Arc::new(AtomicBool::new(true));
    let host = {
        let (channel, served, running) = (channel.clone(), served.clone(), running.clone());
        std::thread::spawn(move || {
            let mut now = 0u64;
            loop {
                now += 1;
                match channel.turn(now) {
                    Turn::Serve => {
                        served.fetch_add(1, Ordering::Relaxed);
                    }
                    Turn::Idle => std::thread::yield_now(),
                    Turn::Leave => break,
                }
            }
            running.store(false, Ordering::Release);
        })
    };
    let clock = std::time::Instant::now();
    let micros = || clock.elapsed().as_micros() as u64;
    // Two seconds is far past any answer; the loop answers in microseconds.
    let took = channel.self_check(micros, 2_000_000, std::thread::yield_now);
    assert!(took.is_ok(), "{took:?}");
    while served.load(Ordering::Relaxed) < 100 {
        std::thread::yield_now();
    }
    assert!(channel.stop(micros, 2_000_000, std::thread::yield_now));
    host.join().unwrap();
    assert!(!running.load(Ordering::Acquire));
    let after = served.load(Ordering::Relaxed);
    assert_eq!(channel.turn(0), Turn::Leave);
    assert_eq!(served.load(Ordering::Relaxed), after);
}

#[test]
fn shares_split_the_ticks_and_clear() {
    let shares = Shares::new();
    shares.add(Taker::Boot, 25);
    shares.add(Taker::HostCore, 70);
    shares.add(Taker::Waiting, 5);
    let (ticks, total) = shares.take();
    assert_eq!(ticks, [5, 25, 70]);
    assert_eq!(total, 100);
    assert_eq!(percent(ticks[Taker::Boot as usize], total), 25);
    assert_eq!(shares.take(), ([0; 3], 0));
    assert_eq!(percent(1, 0), 0);
    assert_eq!(percent(u64::MAX, u64::MAX), 100);
}

#[test]
fn every_state_has_a_name() {
    for state in State::ALL {
        assert!(!state.name().is_empty());
    }
}

use super::*;

/// Garvit's ASUS TUF A16 (FA608PP) FADT as dumped (2026-11-18), with only
/// the fields this module reads: SCI_INT 9, SMI_CMD 0xb2, ACPI_ENABLE 0xa0,
/// PM1a_CNT 0x804 and flags 0x23c5b5 (no HW_REDUCED_ACPI).
fn tuf_a16() -> alloc::vec::Vec<u8> {
    let mut fadt = alloc::vec![0; 276];
    fadt[..4].copy_from_slice(b"FACP");
    fadt[46..48].copy_from_slice(&9u16.to_le_bytes());
    fadt[48..52].copy_from_slice(&0xb2u32.to_le_bytes());
    fadt[52] = 0xa0;
    fadt[53] = 0xa1;
    fadt[64..68].copy_from_slice(&0x804u32.to_le_bytes());
    fadt[112..116].copy_from_slice(&0x23c5b5u32.to_le_bytes());
    fadt
}

#[test]
fn reads_the_tuf_a16_acpi_mode_registers() {
    let registers = Registers::read(&tuf_a16()).unwrap();
    assert_eq!(
        registers,
        Registers {
            sci_interrupt: 9,
            smi_command: 0xb2,
            acpi_enable: 0xa0,
            pm1a_control: 0x804,
            reduced: false,
        }
    );
    assert_eq!(registers.control_port(), Some(0x804));
    assert!(Registers::read(&tuf_a16()[..100]).is_none());
    let mut reduced = tuf_a16();
    reduced[112..116].copy_from_slice(&(1u32 << 20).to_le_bytes());
    assert!(Registers::read(&reduced).unwrap().reduced);
}

#[test]
fn names_what_entering_acpi_mode_takes() {
    let registers = Registers::read(&tuf_a16()).unwrap();
    assert_eq!(
        registers.entry(Some(1)),
        "the firmware left SCI_EN set, so no SMI_CMD write enters ACPI mode"
    );
    assert_eq!(
        registers.entry(Some(0)),
        "SCI_EN is clear, so ACPI mode is entered with SMI_CMD 0xb2 <- 0xa0"
    );
    assert!(registers.entry(None).starts_with("PM1a_CNT unread"));
    let none = Registers {
        smi_command: 0,
        ..registers
    };
    assert_eq!(
        none.entry(Some(0)),
        "SCI_EN is clear and the FADT has no SMI_CMD or ACPI_ENABLE"
    );
    let line = starting_line(&registers, "GSI 9 low level", Some(1));
    assert_eq!(
        line,
        "ACPI hardware mode starting (--acpi-hardware): SCI_INT 9 on GSI 9 low level; the \
         firmware left SCI_EN set, so no SMI_CMD write enters ACPI mode\n"
    );
}

#[test]
fn the_off_lines_name_the_option_and_what_stays_off() {
    let off = off_line();
    assert!(off.starts_with("UNSUPPORTED ACPI hardware mode: off unless --acpi-hardware"));
    assert!(off.contains(WITHOUT) && off.ends_with('\n'));
    let refused = refused_line("the SCI's route was not captured");
    assert!(refused.contains("--acpi-hardware is given, but the SCI's route was not captured"));
    assert!(refused.contains(WITHOUT));
}

/// A turn every 4 ms (250 a second) for `ms`, each interrupted when
/// `interrupted` says so; the turns that dispatched and rearmed, and the
/// changes.
fn run(
    governor: &mut SciGovernor,
    from: u64,
    ms: u64,
    interrupted: impl Fn(u64) -> bool,
) -> (u32, u32, alloc::vec::Vec<(u64, Change)>) {
    let (mut dispatched, mut rearmed, mut changes) = (0, 0, alloc::vec::Vec::new());
    let mut now = from;
    while now < from + ms {
        let turn = governor.turn(now, interrupted(now));
        dispatched += u32::from(turn.dispatch);
        rearmed += u32::from(turn.rearm);
        if let Some(change) = turn.change {
            changes.push((now, change));
        }
        now += 4;
    }
    (dispatched, rearmed, changes)
}

#[test]
fn a_laptops_few_events_stay_interrupts() {
    let mut governor = SciGovernor::new(0);
    // Ten events a second for a minute: a held brightness key is fewer.
    let (dispatched, rearmed, changes) = run(&mut governor, 0, 60_000, |now| now % 100 == 0);
    assert_eq!(dispatched, 600);
    assert_eq!(rearmed, 600);
    assert!(changes.is_empty());
    assert!(!governor.polled());
    // No interrupt, no handler and no rearm.
    let turn = governor.turn(60_004, false);
    assert!(!turn.dispatch && !turn.rearm && turn.change.is_none());
}

#[test]
fn a_burst_shorter_than_the_storm_windows_stays_interrupts() {
    let mut governor = SciGovernor::new(0);
    // Every turn for two seconds, then quiet: two windows, not three.
    let (_, _, changes) = run(&mut governor, 0, 2_000, |_| true);
    assert!(changes.is_empty());
    let (_, _, changes) = run(&mut governor, 2_000, 10_000, |_| false);
    assert!(changes.is_empty());
    assert!(!governor.polled());
}

#[test]
fn a_held_sci_is_masked_polled_retried_and_then_polled_for_good() {
    let mut governor = SciGovernor::new(0);
    let (_, _, changes) = run(&mut governor, 0, 3_004, |_| true);
    assert_eq!(
        changes,
        [(
            3_000,
            Change::Storm {
                per_window: 250,
                lasting: false
            }
        )]
    );
    assert!(governor.polled());
    assert!(
        changes[0]
            .1
            .line()
            .contains("then interrupts are tried again")
    );
    // Masked: the handler runs every POLL_MS and the line is never rearmed,
    // even if an interrupt arrived before the mask.
    let (dispatched, rearmed, changes) = run(&mut governor, 3_004, 10_000, |_| false);
    assert_eq!(rearmed, 0);
    assert!(changes.is_empty());
    assert!((99..=101).contains(&dispatched), "{dispatched}");
    let (_, rearmed, _) = run(&mut governor, 13_004, 4, |_| true);
    assert_eq!(rearmed, 0);
    // After RETRY_MS, interrupts again, rearmed at once.
    let (_, _, changes) = run(&mut governor, 13_008, 20_000, |_| false);
    assert_eq!(changes, [(33_000, Change::Retry)]);
    assert!(!governor.polled());
    // A second storm keeps the polling for the rest of the boot.
    let (_, _, changes) = run(&mut governor, 33_008, 3_100, |_| true);
    assert_eq!(changes.len(), 1);
    assert_eq!(
        changes[0].1,
        Change::Storm {
            per_window: 250,
            lasting: true
        }
    );
    assert!(changes[0].1.line().contains("for the rest of the boot"));
    let (dispatched, rearmed, changes) = run(&mut governor, 36_108, 600_000, |_| false);
    assert!(changes.is_empty());
    assert_eq!(rearmed, 0);
    assert!(dispatched >= 5_990);
    assert!(governor.polled());
    assert_eq!(governor.storms(), 2);
}

#[test]
fn the_retry_turn_runs_the_handler_and_rearms_with_no_interrupt_pending() {
    let mut governor = SciGovernor::new(0);
    run(&mut governor, 0, 3_004, |_| true);
    assert!(governor.polled());
    // The line was masked, so no interrupt arrives; the turn that tries
    // interrupts again must still dispatch and rearm, or it stays masked.
    let turn = governor.turn(3_000 + RETRY_MS, false);
    assert_eq!(
        turn,
        Turn {
            dispatch: true,
            rearm: true,
            change: Some(Change::Retry)
        }
    );
    assert!(!governor.polled());
}

#[test]
fn a_clock_that_goes_backwards_starts_no_storm() {
    let mut governor = SciGovernor::new(5_000);
    for now in (0..5_000).rev().step_by(4) {
        assert!(governor.turn(now, true).change.is_none());
    }
    assert!(!governor.polled());
}

#[test]
fn the_ec_takes_a_windows_budget_of_queries_and_reports_its_first_refusal() {
    let mut ec = EcGovernor::new(0);
    for _ in 0..QUERIES_PER_WINDOW {
        assert_eq!(ec.admit(10), Admit::Take);
    }
    assert_eq!(ec.admit(10), Admit::Wait { first: true });
    assert_eq!(ec.admit(500), Admit::Wait { first: false });
    // The next window starts the budget again.
    assert_eq!(ec.admit(1_000), Admit::Take);
}

#[test]
fn failed_transactions_in_a_row_stop_the_ecs_events_once() {
    let mut ec = EcGovernor::new(0);
    assert!(!ec.failed());
    assert!(!ec.failed());
    ec.succeeded();
    assert!(!ec.failed());
    assert!(!ec.failed());
    assert!(ec.failed());
    assert!(ec.stopped());
    assert!(!ec.failed(), "the stop is reported once");
    assert_eq!(ec.admit(5_000), Admit::Stopped);
}

#[test]
fn each_query_number_is_named_once() {
    let mut ec = EcGovernor::new(0);
    assert!(ec.first(0x2f));
    assert!(!ec.first(0x2f));
    assert!(ec.first(0x17));
    assert!(ec.first(0xff));
    assert!(ec.first(0));
    assert!(!ec.first(0xff));
}

use super::*;
use alloc::string::ToString;

/// A 52 Wh pack at a 15.4 V design voltage, three quarters full,
/// discharging at 15.4 W: the shape of an mWh laptop firmware.
fn energy() -> Readings {
    Readings {
        unit: Unit::Energy,
        design_capacity: Some(52_000),
        full_charge_capacity: Some(48_000),
        design_voltage_mv: Some(15_400),
        warning_capacity: Some(4_800),
        low_capacity: Some(1_540),
        cycle_count: Some(42),
        state: Some(DISCHARGING),
        rate: Some(15_400),
        remaining_capacity: Some(36_000),
        voltage_mv: Some(16_000),
        model: Name::new(b"A32-K55"),
        serial: Name::new(b"0123"),
    }
}

/// A 5000 mAh pack a quarter full, charging at 2.4 A: an mAh firmware.
fn charge() -> Readings {
    Readings {
        unit: Unit::Charge,
        design_capacity: Some(5_000),
        full_charge_capacity: Some(4_800),
        design_voltage_mv: Some(11_550),
        warning_capacity: Some(480),
        low_capacity: Some(150),
        cycle_count: None,
        state: Some(CHARGING),
        rate: Some(2_400),
        remaining_capacity: Some(1_200),
        voltage_mv: Some(12_000),
        model: Name::new(b"Pack"),
        serial: Name::default(),
    }
}

#[test]
fn energy_firmware_converts_every_capacity_and_the_rate_through_the_design_voltage() {
    let battery = BatteryState::from_readings(&energy(), Some(false));
    assert_eq!(battery.status, Status::Discharging);
    // mWh * 1000 / mV, rounded to the nearest mAh.
    assert_eq!(battery.design_mah, Some(3_377));
    assert_eq!(battery.full_mah, Some(3_117));
    assert_eq!(battery.remaining_mah, Some(2_338));
    assert_eq!(battery.warning_mah, Some(312));
    assert_eq!(battery.low_mah, Some(100));
    // 15.4 W at the 15.4 V the capacities were converted with, not 16 V.
    assert_eq!(battery.current_ma, Some(-1_000));
    assert_eq!(battery.power_mw, Some(15_400));
    assert_eq!(battery.voltage_mv, Some(16_000));
    // The percentage and times come from the firmware's own units.
    assert_eq!(battery.percent, Some(75));
    assert_eq!(battery.time_to_empty_min, Some(140));
    assert_eq!(battery.time_to_full_min, None);
    assert_eq!(battery.cycle_count, Some(42));
    assert!(!battery.critical);
    assert_eq!(battery.model.text(), b"A32-K55");
}

#[test]
fn energy_firmware_without_a_design_voltage_converts_through_the_present_one() {
    let readings = Readings {
        design_voltage_mv: None,
        ..energy()
    };
    let battery = BatteryState::from_readings(&readings, Some(false));
    assert_eq!(battery.full_mah, Some(3_000));
    assert_eq!(battery.remaining_mah, Some(2_250));
    assert_eq!(battery.current_ma, Some(-963));
    // A zero design voltage is no voltage either.
    let zero = Readings {
        design_voltage_mv: Some(0),
        ..energy()
    };
    assert_eq!(
        BatteryState::from_readings(&zero, Some(false)).full_mah,
        Some(3_000)
    );
}

#[test]
fn charge_firmware_keeps_its_units_and_derives_power_from_the_present_voltage() {
    let battery = BatteryState::from_readings(&charge(), Some(true));
    assert_eq!(battery.status, Status::Charging);
    assert_eq!(
        (battery.design_mah, battery.full_mah, battery.remaining_mah),
        (Some(5_000), Some(4_800), Some(1_200))
    );
    assert_eq!(battery.current_ma, Some(2_400));
    assert_eq!(battery.power_mw, Some(28_800));
    assert_eq!(battery.percent, Some(25));
    assert_eq!(battery.time_to_full_min, Some(90));
    assert_eq!(battery.time_to_empty_min, None);
    assert_eq!(battery.cycle_count, None);
}

#[test]
fn unknown_values_stay_unknown_and_no_voltage_converts_no_energy() {
    let readings = Readings {
        design_voltage_mv: None,
        voltage_mv: None,
        rate: None,
        cycle_count: None,
        ..energy()
    };
    let battery = BatteryState::from_readings(&readings, Some(false));
    assert_eq!(
        (battery.design_mah, battery.full_mah, battery.remaining_mah),
        (None, None, None)
    );
    assert_eq!((battery.current_ma, battery.power_mw), (None, None));
    assert_eq!(battery.voltage_mv, None);
    // The percentage needs no voltage.
    assert_eq!(battery.percent, Some(75));
    // An unknown rate discharges, as Linux's nonzero test counts it, with
    // no time estimate.
    assert_eq!(battery.status, Status::Discharging);
    assert_eq!(battery.time_to_empty_min, None);

    let unknown = Readings {
        remaining_capacity: None,
        full_charge_capacity: None,
        design_capacity: None,
        state: None,
        ..charge()
    };
    let on_battery = BatteryState::from_readings(&unknown, Some(false));
    assert_eq!(on_battery.percent, None);
    assert_eq!(on_battery.time_to_full_min, None);
    // Unknown state bits: discharging off AC, not charging on it.
    assert_eq!(on_battery.status, Status::Discharging);
    assert_eq!(
        BatteryState::from_readings(&unknown, Some(true)).status,
        Status::NotCharging
    );
    assert_eq!(
        BatteryState::from_readings(&unknown, None).status,
        Status::NotCharging
    );
}

#[test]
fn the_percentage_rounds_to_nearest_falls_back_to_the_design_capacity_and_holds_at_100() {
    let at = |remaining, full, design| {
        let readings = Readings {
            remaining_capacity: remaining,
            full_charge_capacity: full,
            design_capacity: design,
            ..charge()
        };
        BatteryState::from_readings(&readings, Some(true)).percent
    };
    assert_eq!(at(Some(1), Some(3), None), Some(33));
    assert_eq!(at(Some(2), Some(3), None), Some(67));
    assert_eq!(at(Some(0), Some(3), None), Some(0));
    assert_eq!(at(Some(4_900), Some(4_800), None), Some(100));
    // Linux's ACPI_BATTERY_CAPACITY_VALID: zero is no full charge capacity.
    assert_eq!(at(Some(2_500), Some(0), Some(5_000)), Some(50));
    assert_eq!(at(Some(2_500), None, Some(5_000)), Some(50));
    assert_eq!(at(Some(2_500), None, None), None);
    assert_eq!(at(Some(2_500), Some(0), Some(0)), None);
}

#[test]
fn status_follows_linux_for_a_full_a_charge_limited_and_an_idle_battery() {
    let with = |state, rate, remaining, full, design, ac| {
        let readings = Readings {
            state,
            rate,
            remaining_capacity: remaining,
            full_charge_capacity: full,
            design_capacity: design,
            ..charge()
        };
        BatteryState::from_readings(&readings, ac)
    };
    // No bit and full: full, with no current and no time.
    let full = with(Some(0), Some(0), Some(4_800), Some(4_800), None, Some(true));
    assert_eq!(full.status, Status::Full);
    assert_eq!(full.current_ma, Some(0));
    assert_eq!(
        (full.time_to_full_min, full.time_to_empty_min),
        (None, None)
    );
    assert_eq!(full.percent, Some(100));
    // Full by the design capacity when the full charge capacity is unknown.
    let design = with(Some(0), Some(0), Some(5_000), None, Some(5_000), Some(true));
    assert_eq!(design.status, Status::Full);
    // No bit, not full: not charging; nor is an empty one full.
    assert_eq!(
        with(Some(0), Some(0), Some(4_000), Some(4_800), None, Some(true)).status,
        Status::NotCharging
    );
    assert_eq!(
        with(Some(0), Some(0), Some(0), Some(0), Some(0), Some(true)).status,
        Status::NotCharging
    );
    // Charging bit at full with no rate: not charging; with a rate, charging.
    assert_eq!(
        with(
            Some(CHARGING),
            Some(0),
            Some(4_800),
            Some(4_800),
            None,
            Some(true)
        )
        .status,
        Status::NotCharging
    );
    assert_eq!(
        with(
            Some(CHARGING),
            Some(5),
            Some(4_800),
            Some(4_800),
            None,
            Some(true)
        )
        .status,
        Status::Charging
    );
    // The charge-limiting bit: not charging.
    let limited = with(
        Some(CHARGE_LIMITING),
        Some(0),
        Some(3_840),
        Some(4_800),
        None,
        Some(true),
    );
    assert_eq!(limited.status, Status::NotCharging);
    assert_eq!(limited.current_ma, Some(0));
    // Discharging bit with no rate: idle on AC, discharging off it.
    assert_eq!(
        with(
            Some(DISCHARGING),
            Some(0),
            Some(3_000),
            Some(4_800),
            None,
            Some(true)
        )
        .status,
        Status::NotCharging
    );
    let idle = with(
        Some(DISCHARGING),
        Some(0),
        Some(3_000),
        Some(4_800),
        None,
        Some(false),
    );
    assert_eq!(idle.status, Status::Discharging);
    assert_eq!(idle.current_ma, Some(0));
    assert_eq!(idle.time_to_empty_min, None);
    // Both bits: discharging wins, as in Linux.
    assert_eq!(
        with(
            Some(DISCHARGING | CHARGING),
            Some(100),
            Some(3_000),
            Some(4_800),
            None,
            Some(true)
        )
        .status,
        Status::Discharging
    );
}

#[test]
fn critical_is_the_firmware_bit_or_the_low_capacity_while_discharging() {
    let with = |state, remaining| {
        let readings = Readings {
            state: Some(state),
            remaining_capacity: Some(remaining),
            rate: Some(800),
            ..charge()
        };
        BatteryState::from_readings(&readings, Some(state & DISCHARGING == 0))
    };
    assert!(with(DISCHARGING | CRITICAL, 1_000).critical);
    assert!(with(CRITICAL, 1_000).critical);
    assert!(with(DISCHARGING, 150).critical);
    assert!(!with(DISCHARGING, 151).critical);
    // At the low capacity on AC, charging: not critical.
    assert!(!with(CHARGING, 150).critical);
    // No low capacity, or a zero one, takes nothing from the bit.
    let unset = Readings {
        state: Some(DISCHARGING),
        low_capacity: Some(0),
        remaining_capacity: Some(0),
        ..charge()
    };
    assert!(!BatteryState::from_readings(&unset, Some(false)).critical);
}

#[test]
fn names_keep_31_bytes_up_to_the_first_nul() {
    assert_eq!(Name::new(b"serial\0ignored").text(), b"serial");
    let long = [b'x'; 40];
    assert_eq!(Name::new(&long).text(), &long[..31]);
    assert_eq!(Name::new(&long).padded()[31], 0);
    assert_eq!(Name::new(b"").text(), b"");
    let mut padded = [0u8; 32];
    padded[..4].copy_from_slice(b"Pack");
    assert_eq!(Name::new(b"Pack").padded(), &padded);
}

#[test]
fn summaries_differ_where_macos_would_act_and_nowhere_else() {
    let snapshot = |readings: Readings, ac| PowerSnapshot {
        ac_online: ac,
        battery: Some(BatteryState::from_readings(&readings, ac)),
    };
    let base = snapshot(energy(), Some(false));
    // Voltage and rate jitter change no summary.
    let jitter = snapshot(
        Readings {
            voltage_mv: Some(15_900),
            rate: Some(15_000),
            ..energy()
        },
        Some(false),
    );
    assert_eq!(base.summary(), jitter.summary());
    let step = snapshot(
        Readings {
            remaining_capacity: Some(35_500),
            ..energy()
        },
        Some(false),
    );
    let plugged = snapshot(energy(), Some(true));
    let none = PowerSnapshot {
        ac_online: Some(false),
        battery: None,
    };
    let unknown = PowerSnapshot {
        ac_online: None,
        battery: None,
    };
    let packs = [base, step, plugged, none, unknown].map(|s| s.summary().pack());
    for (index, pack) in packs.iter().enumerate() {
        assert_ne!(*pack, 0);
        assert!(!packs[..index].contains(pack), "{index}");
    }
    assert_eq!(
        base.summary(),
        Summary {
            ac_online: Some(false),
            status: Some(Status::Discharging),
            percent: Some(75),
            critical: false,
        }
    );
}

#[test]
fn the_log_line_names_percent_status_power_and_charge() {
    let snapshot = PowerSnapshot {
        ac_online: Some(false),
        battery: Some(BatteryState::from_readings(&energy(), Some(false))),
    };
    assert_eq!(
        snapshot.to_string(),
        "battery 75% discharging 15.4 W ac=0 charge=2338/3117 mAh design=3377 mAh \
         voltage=16000 mV current=-1000 mA cycles=42 empty_in=140 min"
    );
    let unknown = Readings {
        design_voltage_mv: None,
        voltage_mv: None,
        rate: None,
        cycle_count: None,
        remaining_capacity: None,
        state: Some(DISCHARGING | CRITICAL),
        ..energy()
    };
    let critical = PowerSnapshot {
        ac_online: None,
        battery: Some(BatteryState::from_readings(&unknown, None)),
    };
    assert_eq!(
        critical.to_string(),
        "battery ?% discharging ? W ac=? charge=?/? mAh design=? mAh \
         voltage=? mV current=? mA cycles=? critical"
    );
    let charging = PowerSnapshot {
        ac_online: Some(true),
        battery: Some(BatteryState::from_readings(&charge(), Some(true))),
    };
    assert_eq!(
        charging.to_string(),
        "battery 25% charging 28.8 W ac=1 charge=1200/4800 mAh design=5000 mAh \
         voltage=12000 mV current=2400 mA cycles=? full_in=90 min"
    );
    let none = PowerSnapshot {
        ac_online: Some(true),
        battery: None,
    };
    assert_eq!(none.to_string(), "battery none ac=1");
}

#[cfg(feature = "native-acpi")]
#[test]
fn a_control_method_batterys_snapshot_carries_its_units_unknowns_and_cycle_count() {
    use super::super::battery::{self, Extended, Information, Snapshot, State};
    let information = Information {
        unit: battery::Unit::Energy,
        design_capacity: Some(52_000),
        full_charge_capacity: None,
        rechargeable: Some(true),
        design_voltage_mv: Some(15_400),
        warning_capacity: Some(4_800),
        low_capacity: None,
        low_granularity: None,
        high_granularity: None,
        model: b"A32-K55".to_vec(),
        serial: b"0123".to_vec(),
        chemistry: b"LION".to_vec(),
        oem: b"ASUS".to_vec(),
        extended: Some(Extended {
            revision: 1,
            cycle_count: Some(42),
            accuracy: None,
            maximum_sampling_ms: None,
            minimum_sampling_ms: None,
            maximum_averaging_ms: None,
            minimum_averaging_ms: None,
            swapping_capability: None,
        }),
    };
    let snapshot = Snapshot {
        status: 0x1f,
        information: information.clone(),
        state: State {
            flags: DISCHARGING,
            rate: Some(15_400),
            remaining_capacity: Some(36_000),
            voltage_mv: None,
        },
    };
    let readings = Readings::acpi(&snapshot);
    assert_eq!(readings.unit, Unit::Energy);
    assert_eq!(readings.design_capacity, Some(52_000));
    assert_eq!(readings.full_charge_capacity, None);
    assert_eq!(readings.design_voltage_mv, Some(15_400));
    assert_eq!(readings.warning_capacity, Some(4_800));
    assert_eq!(readings.low_capacity, None);
    assert_eq!(readings.cycle_count, Some(42));
    assert_eq!(readings.state, Some(DISCHARGING));
    assert_eq!(
        (
            readings.rate,
            readings.remaining_capacity,
            readings.voltage_mv
        ),
        (Some(15_400), Some(36_000), None)
    );
    assert_eq!(readings.model.text(), b"A32-K55");
    assert_eq!(readings.serial.text(), b"0123");
    // `_BIF` has no cycle count; unknown state bits stay unknown.
    let older = Snapshot {
        information: Information {
            extended: None,
            unit: battery::Unit::Charge,
            ..information
        },
        state: State {
            flags: u32::MAX,
            ..snapshot.state
        },
        ..snapshot
    };
    let readings = Readings::acpi(&older);
    assert_eq!(readings.unit, Unit::Charge);
    assert_eq!((readings.cycle_count, readings.state), (None, None));
}

#[test]
fn the_test_battery_goes_through_the_same_conversion() {
    let charging = PowerSnapshot::synthetic(80, TestState::Charging);
    assert_eq!(charging.ac_online, Some(true));
    let battery = charging.battery.unwrap();
    assert_eq!(battery.status, Status::Charging);
    assert_eq!(battery.percent, Some(80));
    assert_eq!(
        (battery.remaining_mah, battery.full_mah, battery.design_mah),
        (Some(3_840), Some(4_800), Some(5_000))
    );
    assert_eq!(battery.current_ma, Some(1_600));
    assert_eq!(battery.time_to_full_min, Some(36));
    assert_eq!(battery.model.text(), b"Stela test battery");

    let discharging = PowerSnapshot::synthetic(5, TestState::Discharging);
    assert_eq!(discharging.ac_online, Some(false));
    let battery = discharging.battery.unwrap();
    assert_eq!(battery.status, Status::Discharging);
    assert_eq!(battery.current_ma, Some(-800));
    assert_eq!(battery.percent, Some(5));
    assert_eq!(battery.time_to_empty_min, Some(18));
    assert!(!battery.critical);

    let full = PowerSnapshot::synthetic(100, TestState::Full);
    let battery = full.battery.unwrap();
    assert_eq!((battery.status, battery.percent), (Status::Full, Some(100)));
    assert_eq!(battery.current_ma, Some(0));
    // Past 100 the percentage holds.
    assert_eq!(
        PowerSnapshot::synthetic(150, TestState::Charging)
            .battery
            .unwrap()
            .percent,
        Some(100)
    );
}

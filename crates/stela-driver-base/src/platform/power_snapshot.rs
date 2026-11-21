// SPDX-License-Identifier: GPL-2.0-or-later
// Battery status and capacity, following Linux drivers/acpi/battery.c.
// Copyright (C) 2007 Alexey Starikovskiy
// Copyright (C) 2004-2007 Vladimir Lebedev
// Copyright (C) 2001, 2002 Andy Grover and Paul Diefenbaugh
//! The host's battery and AC adapter as one reading in the units an Apple
//! SMC reports them in: charge in mAh, voltage in mV, current in mA with its
//! sign, time in minutes.
//!
//! ACPI's control-method battery (`battery`) keeps its firmware's units:
//! `_BIF`/`_BIX` name a power unit, mWh and mW or mAh and mA, and any field
//! may be the unknown 0xFFFFFFFF, which it keeps as `None`. [`Readings`]
//! holds those values as they are; [`BatteryState::from_readings`] converts
//! them. Energy becomes charge through one voltage, the design voltage
//! (`_BIF` Design Voltage), or the present one (`_BST`) when the design
//! voltage is unknown, for every capacity and for the rate, so remaining
//! charge over current is the firmware's remaining energy over power. A value
//! no known voltage converts stays unknown.
//!
//! The status follows Linux's `acpi_battery_get_property`: the discharging
//! bit, unless the rate is zero with the AC adapter online; then the
//! charging bit, unless the battery is full with no rate; then the
//! charge-limiting bit; then full when the battery reports no bit and is
//! full by `acpi_battery_is_full`. The percentage is Linux's
//! `DIV_ROUND_CLOSEST(remaining * 100, full)` over the full charge capacity,
//! or the design capacity when that is not valid, held at 100. Linux takes
//! critical from the `_BST` critical bit; this reading also takes it from a
//! remaining capacity at or below the firmware's Design Capacity of Low while
//! the battery discharges, the level ACPI names for an emergency shutdown.
//! The times are what the rate gives: the remaining capacity while it
//! discharges, the capacity still to charge while it charges.
use core::fmt;

/// `_BST` battery state bits (ACPI 6.5 section 10.2.2.11).
const DISCHARGING: u32 = 1 << 0;
const CHARGING: u32 = 1 << 1;
const CRITICAL: u32 = 1 << 2;
const CHARGE_LIMITING: u32 = 1 << 3;

/// The firmware's power unit (`_BIF`/`_BIX` Power Unit).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Unit {
    /// Capacity in mWh, rate in mW.
    #[default]
    Energy,
    /// Capacity in mAh, rate in mA.
    Charge,
}

/// A battery identity string as Linux keeps it: at most 31 bytes, ended
/// by the first NUL, held NUL-padded in 32. Firmware strings are raw bytes;
/// each that is not printable ASCII (0x20 to 0x7e) is held as `?`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Name([u8; Name::BYTES]);

impl Name {
    pub const BYTES: usize = 32;

    pub fn new(text: &[u8]) -> Self {
        let length = text
            .iter()
            .position(|&byte| byte == 0)
            .unwrap_or(text.len())
            .min(Self::BYTES - 1);
        let mut bytes = [0; Self::BYTES];
        for (to, &from) in bytes.iter_mut().zip(&text[..length]) {
            *to = if (0x20..=0x7e).contains(&from) {
                from
            } else {
                b'?'
            };
        }
        Self(bytes)
    }

    /// The text, without its padding.
    pub fn text(&self) -> &[u8] {
        let length = self.0.iter().position(|&byte| byte == 0).unwrap_or(0);
        &self.0[..length]
    }

    /// All 32 bytes, NUL-padded.
    pub fn padded(&self) -> &[u8; Self::BYTES] {
        &self.0
    }
}

/// One battery's ACPI readings in its firmware's units, unknowns `None`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Readings {
    pub unit: Unit,
    pub design_capacity: Option<u32>,
    pub full_charge_capacity: Option<u32>,
    pub design_voltage_mv: Option<u32>,
    /// Design Capacity of Warning and of Low.
    pub warning_capacity: Option<u32>,
    pub low_capacity: Option<u32>,
    /// `_BIX` only.
    pub cycle_count: Option<u32>,
    /// `_BST` Battery State.
    pub state: Option<u32>,
    /// `_BST` Present Rate, a magnitude.
    pub rate: Option<u32>,
    pub remaining_capacity: Option<u32>,
    pub voltage_mv: Option<u32>,
    pub model: Name,
    pub serial: Name,
}

#[cfg(feature = "native-acpi")]
impl Readings {
    /// A control-method battery's last snapshot.
    pub fn acpi(snapshot: &super::battery::Snapshot) -> Self {
        let information = &snapshot.information;
        Self {
            unit: match information.unit {
                super::battery::Unit::Energy => Unit::Energy,
                super::battery::Unit::Charge => Unit::Charge,
            },
            design_capacity: information.design_capacity,
            full_charge_capacity: information.full_charge_capacity,
            design_voltage_mv: information.design_voltage_mv,
            warning_capacity: information.warning_capacity,
            low_capacity: information.low_capacity,
            cycle_count: information
                .extended
                .as_ref()
                .and_then(|extended| extended.cycle_count),
            state: (snapshot.state.flags != u32::MAX).then_some(snapshot.state.flags),
            rate: snapshot.state.rate,
            remaining_capacity: snapshot.state.remaining_capacity,
            voltage_mv: snapshot.state.voltage_mv,
            model: Name::new(&information.model),
            serial: Name::new(&information.serial),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Charging,
    Discharging,
    Full,
    /// On external power and neither charging nor full, or unknown.
    NotCharging,
}

impl Status {
    pub fn name(self) -> &'static str {
        match self {
            Self::Charging => "charging",
            Self::Discharging => "discharging",
            Self::Full => "full",
            Self::NotCharging => "not-charging",
        }
    }
}

/// One battery in the SMC's units.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BatteryState {
    pub status: Status,
    pub percent: Option<u8>,
    pub remaining_mah: Option<u32>,
    pub full_mah: Option<u32>,
    pub design_mah: Option<u32>,
    pub warning_mah: Option<u32>,
    pub low_mah: Option<u32>,
    pub voltage_mv: Option<u32>,
    /// Positive while charging, negative while discharging, zero otherwise.
    pub current_ma: Option<i32>,
    pub power_mw: Option<u32>,
    pub cycle_count: Option<u32>,
    pub time_to_empty_min: Option<u32>,
    pub time_to_full_min: Option<u32>,
    pub critical: bool,
    pub model: Name,
    pub serial: Name,
}

impl BatteryState {
    /// `readings` converted, with `ac_online` as `_PSR` read it.
    pub fn from_readings(readings: &Readings, ac_online: Option<bool>) -> Self {
        let r = readings;
        // Linux's ACPI_BATTERY_CAPACITY_VALID; a zero voltage is none.
        let valid = |value: Option<u32>| value.filter(|&value| value != 0);
        let full_capacity = valid(r.full_charge_capacity).or(valid(r.design_capacity));
        let percent = r
            .remaining_capacity
            .zip(full_capacity)
            .map(|(remaining, full)| {
                closest(u64::from(remaining) * 100, u64::from(full)).min(100) as u8
            });
        // Linux's acpi_battery_is_full.
        let is_full = r.remaining_capacity.is_some_and(|remaining| {
            remaining != 0
                && (r.full_charge_capacity == Some(remaining)
                    || r.design_capacity.is_some_and(|design| design <= remaining))
        });
        let status = match r.state {
            // No bits to go on: Linux's assumption that a machine with no
            // known supply is on mains.
            None if ac_online == Some(false) => Status::Discharging,
            None => Status::NotCharging,
            Some(state) if state & DISCHARGING != 0 => {
                if r.rate != Some(0) || ac_online == Some(false) {
                    Status::Discharging
                } else {
                    Status::NotCharging
                }
            }
            Some(state) if state & CHARGING != 0 => {
                if !is_full || r.rate.is_some_and(|rate| rate > 0) {
                    Status::Charging
                } else {
                    Status::NotCharging
                }
            }
            Some(state) if state & CHARGE_LIMITING != 0 => Status::NotCharging,
            Some(0) if is_full => Status::Full,
            Some(_) => Status::NotCharging,
        };
        let volts = valid(r.design_voltage_mv).or(valid(r.voltage_mv));
        let charge = |value: Option<u32>| match r.unit {
            Unit::Charge => value,
            Unit::Energy => Some(saturate(closest(
                u64::from(value?) * 1000,
                u64::from(volts?),
            ))),
        };
        let flowing = |magnitude: Option<u32>| match status {
            Status::Charging => magnitude.map(|value| i32::try_from(value).unwrap_or(i32::MAX)),
            Status::Discharging => magnitude.map(|value| -i32::try_from(value).unwrap_or(i32::MAX)),
            Status::Full | Status::NotCharging => Some(0),
        };
        let power = match r.unit {
            Unit::Energy => r.rate,
            Unit::Charge => r
                .rate
                .zip(valid(r.voltage_mv).or(valid(r.design_voltage_mv)))
                .map(|(rate, millivolts)| {
                    saturate(closest(u64::from(rate) * u64::from(millivolts), 1000))
                }),
        };
        let rate = r.rate.filter(|&rate| rate > 0).map(u64::from);
        let minutes = |capacity: u32, rate: u64| saturate(closest(u64::from(capacity) * 60, rate));
        let time_to_empty_min = match status {
            Status::Discharging => r
                .remaining_capacity
                .zip(rate)
                .map(|(remaining, rate)| minutes(remaining, rate)),
            _ => None,
        };
        let time_to_full_min = match status {
            Status::Charging => full_capacity
                .zip(r.remaining_capacity)
                .zip(rate)
                .map(|((full, remaining), rate)| minutes(full.saturating_sub(remaining), rate)),
            _ => None,
        };
        let low = valid(r.low_capacity)
            .zip(r.remaining_capacity)
            .is_some_and(|(low, remaining)| remaining <= low);
        Self {
            status,
            percent,
            remaining_mah: charge(r.remaining_capacity),
            full_mah: charge(r.full_charge_capacity),
            design_mah: charge(r.design_capacity),
            warning_mah: charge(r.warning_capacity),
            low_mah: charge(r.low_capacity),
            voltage_mv: r.voltage_mv,
            current_ma: flowing(charge(r.rate)),
            power_mw: flowing(power).map(i32::unsigned_abs),
            cycle_count: r.cycle_count,
            time_to_empty_min,
            time_to_full_min,
            critical: r.state.is_some_and(|state| state & CRITICAL != 0)
                || (status == Status::Discharging && low),
            model: r.model,
            serial: r.serial,
        }
    }
}

/// `numerator / denominator` rounded to the nearest, as Linux's
/// `DIV_ROUND_CLOSEST_ULL`.
fn closest(numerator: u64, denominator: u64) -> u64 {
    (numerator + denominator / 2) / denominator
}

fn saturate(value: u64) -> u32 {
    u32::try_from(value).unwrap_or(u32::MAX)
}

/// The host's power: its first battery and its AC adapter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PowerSnapshot {
    pub ac_online: Option<bool>,
    pub battery: Option<BatteryState>,
}

/// The parts of a snapshot a person or macOS acts on: whether the AC adapter
/// is online, the battery's status and percentage, and whether it is
/// critical. A change in it is a change worth a notification or a log line;
/// voltage, current and the times move without one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Summary {
    pub ac_online: Option<bool>,
    pub status: Option<Status>,
    pub percent: Option<u8>,
    pub critical: bool,
}

impl Summary {
    /// The summary in 32 bits, distinct for distinct summaries and never 0.
    pub fn pack(self) -> u32 {
        let ac = match self.ac_online {
            None => 0,
            Some(false) => 1,
            Some(true) => 2,
        };
        let status = match self.status {
            None => 0,
            Some(Status::Charging) => 1,
            Some(Status::Discharging) => 2,
            Some(Status::Full) => 3,
            Some(Status::NotCharging) => 4,
        };
        1 << 31
            | ac << 16
            | status << 12
            | u32::from(self.critical) << 8
            | self.percent.map_or(0xff, u32::from)
    }
}

/// The battery a `--test-battery` boot presents in place of the host's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TestState {
    Charging,
    Discharging,
    /// On AC and fully charged.
    Full,
}

impl PowerSnapshot {
    pub fn summary(&self) -> Summary {
        Summary {
            ac_online: self.ac_online,
            status: self.battery.map(|battery| battery.status),
            percent: self.battery.and_then(|battery| battery.percent),
            critical: self.battery.is_some_and(|battery| battery.critical),
        }
    }

    /// A synthetic 4800 mAh battery at `percent`, through the same
    /// conversion as a host's (a 5000 mAh design at 11.55 V, 12 V now,
    /// charging at 1600 mA or discharging at 800 mA), its AC adapter online
    /// unless it discharges; for a machine with no battery, such as a virtual
    /// one.
    pub fn synthetic(percent: u8, state: TestState) -> Self {
        const FULL: u32 = 4_800;
        let (bits, rate, ac) = match state {
            TestState::Charging => (CHARGING, 1_600, true),
            TestState::Discharging => (DISCHARGING, 800, false),
            TestState::Full => (0, 0, true),
        };
        let readings = Readings {
            unit: Unit::Charge,
            design_capacity: Some(5_000),
            full_charge_capacity: Some(FULL),
            design_voltage_mv: Some(11_550),
            warning_capacity: Some(480),
            low_capacity: Some(144),
            cycle_count: Some(1),
            state: Some(bits),
            rate: Some(rate),
            remaining_capacity: Some(FULL * u32::from(percent) / 100),
            voltage_mv: Some(12_000),
            model: Name::new(b"Stela test battery"),
            serial: Name::new(b"STELA-TEST"),
        };
        Self {
            ac_online: Some(ac),
            battery: Some(BatteryState::from_readings(&readings, Some(ac))),
        }
    }
}

/// A value, or `?` when it is unknown.
struct Known<T>(Option<T>);

impl<T: fmt::Display> fmt::Display for Known<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.0 {
            Some(value) => value.fmt(f),
            None => f.write_str("?"),
        }
    }
}

/// The `POWER` log line's text: the percentage, status, power and AC first,
/// then the charge in mAh, the voltage, the current and what the rate gives.
impl fmt::Display for PowerSnapshot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let ac = match self.ac_online {
            Some(true) => "1",
            Some(false) => "0",
            None => "?",
        };
        let Some(battery) = &self.battery else {
            return write!(f, "battery none ac={ac}");
        };
        write!(
            f,
            "battery {}% {} ",
            Known(battery.percent),
            battery.status.name()
        )?;
        match battery.power_mw {
            Some(milliwatts) => write!(f, "{}.{} W", milliwatts / 1000, milliwatts % 1000 / 100)?,
            None => f.write_str("? W")?,
        }
        write!(
            f,
            " ac={ac} charge={}/{} mAh design={} mAh voltage={} mV current={} mA cycles={}",
            Known(battery.remaining_mah),
            Known(battery.full_mah),
            Known(battery.design_mah),
            Known(battery.voltage_mv),
            Known(battery.current_ma),
            Known(battery.cycle_count),
        )?;
        if let Some(minutes) = battery.time_to_empty_min {
            write!(f, " empty_in={minutes} min")?;
        }
        if let Some(minutes) = battery.time_to_full_min {
            write!(f, " full_in={minutes} min")?;
        }
        if battery.critical {
            f.write_str(" critical")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;

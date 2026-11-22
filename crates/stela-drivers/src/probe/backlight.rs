//! A laptop panel's brightness through ACPI video, as Linux's ACPI video
//! driver sets it when it registers a panel's backlight
//! (`acpi_video_init_brightness`), which nouveau asks of it for a panel it
//! drives without a backlight of its own (`nouveau_acpi_video_register_backlight`).
//!
//! Each display output device with `_BCL` and `_BCM` keeps the level `_BQC`
//! (or `_BCQ`) reads where `_BCL` lists it and it is not zero; else, as
//! when the firmware or a GPU reset left an uninitialized or zero level,
//! it is set to the highest level `_BCL` lists. `_BQC` is first checked as
//! Linux checks it (`acpi_video_bqc_quirk`): a test level is set and read
//! back, and a `_BQC` that answers an index of `_BCL` instead of a level is
//! read as one, while one that answers neither is not trusted.
//!
//! The device's brightness notifications, which a laptop's brightness keys
//! send, step it to the next of its levels through `_BCM`, as Linux's
//! `acpi_video_switch_brightness` does ([`Control`]).
use crate::platform::acpi::{Argument, Error, Handle, Session, Value};
use alloc::{string::String, vec::Vec};

/// `_BCL`'s first two entries are the levels for AC power and for battery;
/// the levels a panel runs follow them (`ACPI_VIDEO_FIRST_LEVEL`).
const FIRST_LEVEL: usize = 2;

/// A `_BCL` list as Linux reads it (`acpi_video_get_levels`): its integer
/// entries as 32-bit levels, repeats after the first two dropped where
/// they follow each other; where the first two entries do not appear among
/// the rest, they are added to them; the levels sorted ascending where the
/// first of them is the highest.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Levels {
    /// The AC and battery levels, then the levels the panel runs.
    levels: Vec<u64>,
    /// The highest level of all.
    max: u64,
    /// The panel's levels were listed highest first.
    reversed: bool,
}

impl Levels {
    /// `_BCL`'s entries, `None` where one is not an integer; `None` with
    /// fewer than two entries, which Linux refuses.
    pub fn parse(listed: &[Option<u64>]) -> Option<Self> {
        if listed.len() < FIRST_LEVEL {
            return None;
        }
        let mut levels: Vec<u64> = Vec::new();
        let mut max = 0;
        for &value in listed.iter().flatten() {
            let value = value & u64::from(u32::MAX);
            if levels.len() > FIRST_LEVEL && levels.last() == Some(&value) {
                continue;
            }
            levels.push(value);
            max = max.max(value);
        }
        if levels.len() < FIRST_LEVEL {
            return None;
        }
        let found = levels[FIRST_LEVEL..]
            .iter()
            .map(|&level| usize::from(level == levels[0]) + usize::from(level == levels[1]))
            .sum::<usize>();
        if found < FIRST_LEVEL {
            // Linux moves the levels up by the missing count, so the last
            // of the first two entries (or both) also lead the levels.
            let missing = FIRST_LEVEL - found;
            let added: Vec<u64> = levels[FIRST_LEVEL - missing..FIRST_LEVEL].to_vec();
            levels.splice(FIRST_LEVEL..FIRST_LEVEL, added);
        }
        let reversed = levels.get(FIRST_LEVEL) == Some(&max);
        if reversed {
            levels[FIRST_LEVEL..].sort_unstable();
        }
        Some(Self {
            levels,
            max,
            reversed,
        })
    }

    /// The levels the panel runs.
    fn steps(&self) -> &[u64] {
        &self.levels[FIRST_LEVEL..]
    }

    /// The highest level.
    pub fn max(&self) -> u64 {
        self.max
    }

    /// How many levels the panel runs.
    pub fn count(&self) -> usize {
        self.steps().len()
    }

    /// The level halfway through the panel's levels.
    fn middle(&self) -> u64 {
        let steps = self.steps();
        steps.get(steps.len() / 2).copied().unwrap_or(self.max)
    }

    /// The level `_BQC` names by index of the panel's levels, as listed.
    fn indexed(&self, index: u64) -> Option<u64> {
        let steps = self.steps().len() as u64;
        let index = match self.reversed {
            true => steps.checked_sub(1)?.checked_sub(index)?,
            false => index,
        };
        self.steps().get(usize::try_from(index).ok()?).copied()
    }
}

/// A display output device's brightness methods.
pub trait Panel {
    /// What `_BQC` (or `_BCQ`) reads; `None` where the device has neither
    /// or its evaluation failed, which Linux then no longer asks.
    fn current(&mut self) -> Option<u64>;
    /// `_BCM` with `level`.
    fn set(&mut self, level: u64) -> Result<(), Error>;
}

/// What [`initialize`] set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Brightness {
    pub level: u64,
    pub max: u64,
    /// The level read before, where `_BQC` was trusted.
    pub read: Option<u64>,
}

/// Set `panel` to the level read, else the highest of `levels`, as Linux
/// does when its ACPI video driver registers the panel's backlight.
pub fn initialize(levels: &Levels, panel: &mut impl Panel) -> Result<Brightness, Error> {
    let max = levels.max;
    let read = match panel.current() {
        Some(current) => trusted(levels, panel, current)?,
        None => None,
    };
    let level = read
        .filter(|&level| level != 0 && levels.steps().contains(&level))
        .unwrap_or(max);
    panel.set(level)?;
    Ok(Brightness { level, max, read })
}

/// The level `current`, which `_BQC` read first, names, once a test level
/// set and read back shows whether `_BQC` answers levels, indices or
/// neither (`acpi_video_bqc_quirk`); `None` for neither.
fn trusted(levels: &Levels, panel: &mut impl Panel, current: u64) -> Result<Option<u64>, Error> {
    let test = match current == levels.max {
        true => levels.steps().get(1).copied().unwrap_or(levels.max),
        false => levels.max,
    };
    panel.set(test)?;
    // A failed read gives the level last set, as Linux's does.
    let answered = panel.current().unwrap_or(test);
    if answered == test {
        return Ok(Some(current));
    }
    let count = levels.levels.len() as u64;
    let by_index = answered < count && levels.indexed(answered) == Some(test);
    Ok(by_index.then(|| levels.indexed(current)).flatten())
}

/// A display output device's brightness notifications
/// (`ACPI_VIDEO_NOTIFY_*`), for each of which Linux's
/// `acpi_video_device_notify` switches the brightness
/// (`brightness_switch_enabled`, its default).
pub const NOTIFY_CYCLE: u32 = 0x85;
pub const NOTIFY_INC: u32 = 0x86;
pub const NOTIFY_DEC: u32 = 0x87;
pub const NOTIFY_ZERO: u32 = 0x88;
pub const NOTIFY_OFF: u32 = 0x89;

/// Whether a display output device's notification `event` switches its
/// brightness.
pub fn switches(event: u32) -> bool {
    (NOTIFY_CYCLE..=NOTIFY_OFF).contains(&event)
}

/// The level after `current` for brightness `event`, as Linux's
/// `acpi_video_get_next_level` picks it among the panel's levels: `current`
/// taken to the closest of them (the first listed of two as close), then
/// for INC the next above it, for DEC the next below, for CYCLE the next
/// above or the lowest after the highest, and 0 for ZERO and OFF; INC at
/// the highest and DEC at the lowest stay, and any other event leaves the
/// closest. Linux seeds its search with 255 where this takes the levels'
/// own bounds, which is the same for levels up to 255, as ACPI's
/// percentages are.
pub fn next_level(levels: &Levels, current: u64, event: u32) -> u64 {
    let steps = levels.steps();
    let closest = steps
        .iter()
        .copied()
        .min_by_key(|level| level.abs_diff(current))
        .unwrap_or(current);
    let min = steps.iter().copied().min().unwrap_or(closest);
    let max = steps.iter().copied().max().unwrap_or(closest);
    let above = || steps.iter().copied().filter(|&level| level > closest).min();
    let below = || steps.iter().copied().filter(|&level| level < closest).max();
    match event {
        NOTIFY_CYCLE if closest < max => above().unwrap_or(max),
        NOTIFY_CYCLE => min,
        NOTIFY_INC => above().unwrap_or(max),
        NOTIFY_DEC => below().unwrap_or(min),
        NOTIFY_ZERO | NOTIFY_OFF => 0,
        _ => closest,
    }
}

/// What [`switch`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Switch {
    pub from: u64,
    pub to: u64,
    pub max: u64,
    /// `from` is what `_BQC` read, not the level last set.
    pub read: bool,
}

/// Linux's `acpi_video_switch_brightness` for brightness `event`: the
/// panel's level now, as `acpi_video_device_lcd_get_level_current` takes
/// it (what `_BQC` reads where it is one of the panel's levels, else
/// `last`, the level last set), then the next level for `event`
/// ([`next_level`]) set by `_BCM`. With no level known, the middle of the
/// levels stands for it, so a first press goes no further than half the
/// range the wrong way. A `_BQC` that answers an index of `_BCL`
/// (`acpi_video_bqc_quirk`) names no level here, so the level last set is
/// taken.
pub fn switch(
    levels: &Levels,
    panel: &mut impl Panel,
    last: Option<u64>,
    event: u32,
) -> Result<Switch, Error> {
    let read = panel
        .current()
        .filter(|level| levels.steps().contains(level));
    let from = read.or(last).unwrap_or_else(|| levels.middle());
    let to = next_level(levels, from, event);
    panel.set(to)?;
    Ok(Switch {
        from,
        to,
        max: levels.max,
        read: read.is_some(),
    })
}

/// A display output device with `_BCL` and `_BCM`, kept for its
/// brightness notifications ([`Self::switch`]), as Linux's ACPI video
/// driver keeps each panel's backlight.
pub struct Control<'a> {
    pub path: String,
    pub levels: Levels,
    device: Device<'a>,
    /// The level last set (`brightness->curr`).
    last: Option<u64>,
}

impl<'a> Control<'a> {
    /// The device, whose notifications its handler takes.
    pub fn handle(&self) -> Handle<'a> {
        self.device.handle
    }

    /// [`initialize`], as Linux's ACPI video driver sets a backlight's
    /// level when it registers it (`acpi_video_init_brightness`), the
    /// level set kept as the one a press without `_BQC` starts from.
    pub fn initialize(&mut self) -> Result<Brightness, Error> {
        let set = initialize(&self.levels, &mut self.device)?;
        self.last = Some(set.level);
        Ok(set)
    }

    /// [`switch`] for brightness `event`, from the level last set.
    pub fn switch(&mut self, event: u32) -> Result<Switch, Error> {
        let switched = switch(&self.levels, &mut self.device, self.last, event)?;
        self.last = Some(switched.to);
        Ok(switched)
    }
}

/// What [`controls`] found.
pub struct Controls<'a> {
    pub controls: Vec<Control<'a>>,
    /// The devices with `_BCL` and `_BCM` whose `_BCL` lists no levels.
    pub unlisted: Vec<String>,
    /// Those without `_ADR`, or that their video bus's `_DOD` leaves out.
    pub undeclared: Vec<String>,
}

/// The video buses' outputs past which Linux trusts no `_DOD`
/// (`acpi_video_device_in_dod`).
const DOD_OUTPUTS: usize = 8;

/// Each present display output device with `_BCL` and `_BCM` in
/// `session`'s namespace whose `_BCL` lists levels, for its brightness
/// notifications: as Linux's ACPI video driver registers a backlight only
/// for an output with `_ADR` (`acpi_video_bus_get_one_device`) that its
/// video bus's `_DOD` lists, by the low twelve bits of `_ADR`, unless the
/// bus has no `_DOD` or more than eight outputs (`acpi_video_device_in_dod`).
pub fn controls(session: &Session) -> Result<Controls<'_>, Error> {
    let mut found = Controls {
        controls: Vec::new(),
        unlisted: Vec::new(),
        undeclared: Vec::new(),
    };
    let addressed: Vec<String> = session
        .all_devices()?
        .into_iter()
        .filter(|device| device.child("_ADR").is_ok())
        .filter_map(|device| device.path().ok())
        .collect();
    for output in outputs(session)? {
        if !in_dod(session, &output, &addressed) {
            found.undeclared.push(output.path);
            continue;
        }
        match output.levels {
            Some(levels) => found.controls.push(Control {
                path: output.path,
                levels,
                device: output.device,
                last: None,
            }),
            None => found.unlisted.push(output.path),
        }
    }
    Ok(found)
}

/// Whether `output` has an `_ADR` its video bus's `_DOD` lists, or the bus
/// has no `_DOD` to go by; `addressed` are the paths of the devices with
/// `_ADR`.
fn in_dod(session: &Session, output: &Output<'_>, addressed: &[String]) -> bool {
    let address = match output.device.handle.evaluate(Some("_ADR"), &[]) {
        Ok(Some(Value::Integer(address))) => address,
        _ => return false,
    };
    let Some((bus, _)) = output.path.rsplit_once('.') else {
        return true;
    };
    let listed: Vec<u64> = match session
        .handle(bus)
        .and_then(|bus| bus.evaluate(Some("_DOD"), &[]))
    {
        Ok(Some(Value::Package(values))) => values
            .iter()
            .filter_map(|value| match value {
                Value::Integer(id) => Some(*id),
                _ => None,
            })
            .collect(),
        _ => Vec::new(),
    };
    let outputs = addressed
        .iter()
        .filter(|path| {
            path.rsplit_once('.')
                .is_some_and(|(parent, _)| parent == bus)
        })
        .count();
    listed.is_empty()
        || outputs > DOD_OUTPUTS
        || listed.iter().any(|id| id & 0xfff == address & 0xfff)
}

/// A present display output device with `_BCL` and `_BCM`.
struct Output<'a> {
    path: String,
    /// What `_BCL` lists; `None` where it is unreadable or lists too few.
    levels: Option<Levels>,
    device: Device<'a>,
}

/// Every present display output device with `_BCL` and `_BCM`.
fn outputs(session: &Session) -> Result<Vec<Output<'_>>, Error> {
    let mut outputs = Vec::new();
    for handle in session.all_devices()? {
        let has = |name: &str| handle.child(name).is_ok();
        if !has("_BCL") || !has("_BCM") {
            continue;
        }
        let path = handle.path().unwrap_or_default();
        let listed = match handle.evaluate(Some("_BCL"), &[]) {
            Ok(Some(Value::Package(values))) => values
                .iter()
                .map(|value| match value {
                    Value::Integer(level) => Some(*level),
                    _ => None,
                })
                .collect::<Vec<_>>(),
            _ => Vec::new(),
        };
        let query = ["_BQC", "_BCQ"].into_iter().find(|name| has(name));
        outputs.push(Output {
            path,
            levels: Levels::parse(&listed),
            device: Device { handle, query },
        });
    }
    Ok(outputs)
}

/// ACPI's methods on one display output device.
struct Device<'a> {
    handle: Handle<'a>,
    /// `_BQC`, else `_BCQ`, while it evaluates.
    query: Option<&'static str>,
}

impl Panel for Device<'_> {
    fn current(&mut self) -> Option<u64> {
        let name = self.query?;
        match self.handle.evaluate(Some(name), &[]) {
            Ok(Some(Value::Integer(level))) => Some(level & u64::from(u32::MAX)),
            _ => {
                self.query = None;
                None
            }
        }
    }

    fn set(&mut self, level: u64) -> Result<(), Error> {
        self.handle
            .invoke(Some("_BCM"), &[Argument::Integer(level)])
    }
}

/// What [`initialize_all`] did for one display output device.
#[derive(Debug)]
pub struct Outcome {
    pub path: String,
    /// The level set, or why none was: `_BCL` unreadable (`None`), or a
    /// refused `_BCM`.
    pub result: Result<Brightness, Option<Error>>,
}

/// Set each present display output device with `_BCL` and `_BCM` in
/// `session`'s namespace ([`initialize`]).
pub fn initialize_all(session: &Session) -> Result<Vec<Outcome>, Error> {
    let mut outcomes = Vec::new();
    for mut output in outputs(session)? {
        let result = match &output.levels {
            Some(levels) => initialize(levels, &mut output.device).map_err(Some),
            None => Err(None),
        };
        outcomes.push(Outcome {
            path: output.path,
            result,
        });
    }
    Ok(outcomes)
}

#[cfg(test)]
mod tests;

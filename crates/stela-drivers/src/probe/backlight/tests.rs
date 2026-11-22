use super::*;
use alloc::{vec, vec::Vec};

/// A display output device whose `_BQC` answers `level` (as an index of
/// `steps` where `indexed`), or nothing; every `_BCM` is recorded.
struct Fake {
    query: bool,
    level: u64,
    indexed: Option<Vec<u64>>,
    /// `_BQC` answers this until the first `_BCM`, as firmware that
    /// leaves its variable uninitialized.
    first: Option<u64>,
    sets: Vec<u64>,
}

impl Fake {
    fn new(level: u64) -> Self {
        Self {
            query: true,
            level,
            indexed: None,
            first: None,
            sets: Vec::new(),
        }
    }
}

impl Panel for Fake {
    fn current(&mut self) -> Option<u64> {
        if !self.query {
            return None;
        }
        if self.sets.is_empty()
            && let Some(first) = self.first
        {
            return Some(first);
        }
        Some(match &self.indexed {
            Some(steps) => steps.iter().position(|&step| step == self.level)? as u64,
            None => self.level,
        })
    }

    fn set(&mut self, level: u64) -> Result<(), Error> {
        self.sets.push(level);
        self.level = level;
        Ok(())
    }
}

fn parsed(listed: &[u64]) -> Levels {
    Levels::parse(&listed.iter().copied().map(Some).collect::<Vec<_>>()).unwrap()
}

#[test]
fn bcl_is_read_as_linux_reads_it() {
    // The usual list: AC and battery levels, then the steps ascending.
    let plain = parsed(&[100, 40, 10, 40, 70, 100]);
    assert_eq!(
        (plain.steps(), plain.max, plain.reversed),
        (&[10, 40, 70, 100][..], 100, false)
    );
    // Repeats after the first two are dropped where they follow each other.
    assert_eq!(
        parsed(&[100, 40, 10, 10, 40, 100, 100]).steps(),
        [10, 40, 100]
    );
    // Steps listed highest first are sorted.
    let reversed = parsed(&[100, 40, 100, 70, 40, 10]);
    assert_eq!(
        (reversed.steps(), reversed.reversed),
        (&[10, 40, 70, 100][..], true)
    );
    // First two entries missing from the steps lead them.
    assert_eq!(parsed(&[50, 30, 10, 60, 80]).steps(), [50, 30, 10, 60, 80]);
    assert_eq!(parsed(&[90, 50, 10, 50, 80]).steps(), [50, 10, 50, 80]);
    // Levels are 32-bit; entries that are not integers are skipped.
    let wide = Levels::parse(&[Some(1 << 32 | 100), Some(40), None, Some(40), Some(100)]);
    assert_eq!(wide.unwrap().steps(), [40, 100]);
    assert_eq!(Levels::parse(&[Some(100)]), None);
    assert_eq!(Levels::parse(&[None, None, Some(5)]), None);
}

#[test]
fn a_listed_level_read_back_is_kept() {
    let levels = parsed(&[100, 40, 10, 40, 70, 100]);
    let mut panel = Fake::new(70);
    let set = initialize(&levels, &mut panel).unwrap();
    assert_eq!(
        set,
        Brightness {
            level: 70,
            max: 100,
            read: Some(70)
        }
    );
    // The test level, then the level read.
    assert_eq!(panel.sets, [100, 70]);
    // At the maximum, the test is the second step.
    let mut full = Fake::new(100);
    assert_eq!(initialize(&levels, &mut full).unwrap().level, 100);
    assert_eq!(full.sets, [40, 100]);
}

/// A GPU reset or firmware that leaves the level at zero, or one `_BCL`
/// does not list, gets the highest level: the backlight comes on.
#[test]
fn a_zero_or_unlisted_level_becomes_the_highest() {
    let levels = parsed(&[100, 40, 0, 10, 40, 70, 100]);
    for read in [0, 55] {
        let mut panel = Fake::new(read);
        let set = initialize(&levels, &mut panel).unwrap();
        assert_eq!((set.level, set.read), (100, Some(read)));
        assert_eq!(panel.sets.last(), Some(&100));
    }
    // No `_BQC`: the highest at once.
    let mut blind = Fake::new(0);
    blind.query = false;
    assert_eq!(initialize(&levels, &mut blind).unwrap().read, None);
    assert_eq!(blind.sets, [100]);
}

#[test]
fn a_bqc_that_answers_indices_is_read_as_one() {
    let listed = [100, 40, 10, 40, 70, 100];
    let levels = parsed(&listed);
    let mut panel = Fake::new(40);
    panel.indexed = Some(levels.steps().to_vec());
    // It reads index 1, which is 40 of the steps.
    let set = initialize(&levels, &mut panel).unwrap();
    assert_eq!((set.level, set.read), (40, Some(40)));
    // Highest first, an index counts from the top as listed.
    let reversed = parsed(&[100, 40, 100, 70, 40, 10]);
    assert_eq!(reversed.indexed(0), Some(100));
    assert_eq!(reversed.indexed(3), Some(10));
    assert_eq!(reversed.indexed(4), None);
}

#[test]
fn a_bqc_that_answers_neither_is_not_trusted() {
    let levels = parsed(&[100, 40, 10, 40, 70, 100]);
    // Uninitialized at first, then always 7: neither a level nor an index.
    let mut panel = Fake::new(7);
    panel.first = Some(70);
    panel.indexed = None;
    let mut stuck = StuckAt(7, vec![]);
    let set = initialize(&levels, &mut stuck).unwrap();
    assert_eq!((set.level, set.read), (100, None));
    assert_eq!(stuck.1, [100, 100]);
    // Where the test level reads back, the first read is kept when it is
    // listed, as Linux keeps it.
    assert_eq!(initialize(&levels, &mut panel).unwrap().level, 70);
}

/// `_BQC` always answers the same value.
struct StuckAt(u64, Vec<u64>);

impl Panel for StuckAt {
    fn current(&mut self) -> Option<u64> {
        Some(self.0)
    }

    fn set(&mut self, level: u64) -> Result<(), Error> {
        self.1.push(level);
        Ok(())
    }
}

#[test]
fn a_refused_bcm_is_answered() {
    struct Refusing;
    impl Panel for Refusing {
        fn current(&mut self) -> Option<u64> {
            None
        }
        fn set(&mut self, _: u64) -> Result<(), Error> {
            Err(Error(5))
        }
    }
    let levels = parsed(&[100, 40, 10, 100]);
    assert_eq!(initialize(&levels, &mut Refusing), Err(Error(5)));
}

/// The FA608PP's LCD levels: AC 90 and battery 60, then 2 to 100 by 2.
fn every_second() -> Levels {
    let mut listed = vec![90, 60];
    listed.extend((1..=50).map(|step| step * 2));
    parsed(&listed)
}

#[test]
fn brightness_keys_step_as_linux_steps_them() {
    let levels = every_second();
    for (current, event, next) in [
        (60, NOTIFY_INC, 62),
        (60, NOTIFY_DEC, 58),
        // The highest and lowest stay.
        (100, NOTIFY_INC, 100),
        (2, NOTIFY_DEC, 2),
        // An unlisted level goes to the closest first (the lower of two).
        (61, NOTIFY_INC, 62),
        (61, NOTIFY_DEC, 58),
        (0, NOTIFY_INC, 4),
        (255, NOTIFY_DEC, 98),
        // Cycling wraps after the highest.
        (98, NOTIFY_CYCLE, 100),
        (100, NOTIFY_CYCLE, 2),
        (60, NOTIFY_ZERO, 0),
        (60, NOTIFY_OFF, 0),
        // Another event leaves the closest level.
        (61, 0x80, 60),
    ] {
        assert_eq!(
            next_level(&levels, current, event),
            next,
            "{current} {event:#x}"
        );
    }
    assert!((NOTIFY_CYCLE..=NOTIFY_OFF).all(switches));
    assert!(!switches(0x80) && !switches(0x8a) && !switches(0x84));
}

#[test]
fn a_switch_starts_from_bqc_else_the_level_last_set() {
    let levels = every_second();
    // A listed level read back.
    let mut panel = Fake::new(40);
    assert_eq!(
        switch(&levels, &mut panel, Some(70), NOTIFY_INC),
        Ok(Switch {
            from: 40,
            to: 42,
            max: 100,
            read: true
        })
    );
    assert_eq!(panel.sets, [42]);
    // No _BQC, or one naming no level: the level last set, else the middle
    // of the levels, so a first press goes no further than half the range
    // the wrong way.
    let mut unread = Fake::new(41);
    unread.query = false;
    let switched = switch(&levels, &mut unread, Some(70), NOTIFY_DEC).unwrap();
    assert_eq!((switched.from, switched.to, switched.read), (70, 68, false));
    let mut unlisted = Fake::new(41);
    let switched = switch(&levels, &mut unlisted, None, NOTIFY_DEC).unwrap();
    assert_eq!((switched.from, switched.to, switched.read), (52, 50, false));
}

mod namespace {
    extern crate std;
    use super::*;
    use crate::platform::acpi::{
        Mode, Session,
        test_support::{self, TEST_SERIAL, aml},
    };
    use core::sync::atomic::{AtomicU32, Ordering};

    unsafe extern "C" {
        fn acpi_os_wait_events_complete();
    }

    /// A panel with `_BCL`, `_BCM` and `_BQC`, one whose `_BCL` lists
    /// nothing, and one without `_BCM`.
    fn outputs() -> Vec<u8> {
        let levels: Vec<Vec<u8>> = [100, 40, 10, 40, 70, 100]
            .into_iter()
            .map(aml::integer)
            .collect();
        let device = |name: &str, body: &[Vec<u8>]| aml::device(name, &body.concat());
        aml::scope(
            "\\_SB_",
            &[
                device(
                    "LCD0",
                    &[
                        aml::named("_ADR", &aml::integer(0x110)),
                        aml::named("BLVL", &aml::integer(40)),
                        aml::named("_BCL", &aml::package_of(&levels)),
                        aml::method("_BCM", 1, &aml::store(&[aml::ARG0], &aml::name("BLVL"))),
                        aml::method("_BQC", 0, &aml::ret(&aml::name("BLVL"))),
                    ],
                ),
                device(
                    "LCD1",
                    &[
                        aml::named("_ADR", &aml::integer(0x111)),
                        aml::named("_BCL", &aml::package_of(&[])),
                        aml::method("_BCM", 1, &[]),
                    ],
                ),
                device(
                    "LCD2",
                    &[
                        aml::named("_ADR", &aml::integer(0x112)),
                        aml::named("_BCL", &aml::package_of(&levels)),
                    ],
                ),
            ]
            .concat(),
        )
    }

    #[test]
    fn controls_are_the_outputs_with_bcl_levels_and_bcm() {
        let _serial = TEST_SERIAL.lock().unwrap();
        // SAFETY: the fixture owns every ACPICA service; serialized singleton.
        let session = unsafe {
            Session::start(
                test_support::dsdt_fixture(&outputs()),
                Mode::InterpreterOnly,
            )
        }
        .unwrap();
        let mut found = controls(&session).unwrap();
        assert_eq!(found.unlisted, ["\\_SB_.LCD1"]);
        let [control] = &mut found.controls[..] else {
            panic!("{:?}", found.controls.len());
        };
        assert_eq!(control.path, "\\_SB_.LCD0");
        assert_eq!((control.levels.max(), control.levels.count()), (100, 4));
        let up = control.switch(NOTIFY_INC).unwrap();
        assert_eq!((up.from, up.to, up.read), (40, 70, true));
        let level = session
            .handle("\\_SB.LCD0.BLVL")
            .unwrap()
            .evaluate(None, &[])
            .unwrap();
        assert_eq!(level, Some(Value::Integer(70)));
        let down = control.switch(NOTIFY_DEC).unwrap();
        assert_eq!((down.from, down.to), (70, 40));
        drop(found);
        session.shutdown().unwrap();
    }

    /// Two video buses: one whose `_DOD` lists its first output alone, and
    /// one without `_DOD`; and a panel without `_BQC`.
    fn buses() -> Vec<u8> {
        let levels: Vec<Vec<u8>> = [100, 40, 10, 40, 70, 100]
            .into_iter()
            .map(aml::integer)
            .collect();
        let output = |name: &str, address: u64, extra: &[u8]| {
            aml::device(
                name,
                &[
                    aml::named("_ADR", &aml::integer(address)),
                    aml::named("BLVL", &aml::integer(40)),
                    aml::named("_BCL", &aml::package_of(&levels)),
                    aml::method("_BCM", 1, &aml::store(&[aml::ARG0], &aml::name("BLVL"))),
                    extra.to_vec(),
                ]
                .concat(),
            )
        };
        let query = aml::method("_BQC", 0, &aml::ret(&aml::name("BLVL")));
        aml::scope(
            "\\_SB_",
            &[
                aml::device(
                    "GFX0",
                    &[
                        aml::named("_ADR", &aml::integer(0x0002_0000)),
                        aml::named("_DOD", &aml::package_of(&[aml::integer(0x8001_0110)])),
                        output("LCD0", 0x110, &query),
                        output("LCD1", 0x111, &query),
                        // _ADR is compared by its low twelve bits.
                        output("LCD2", 0x1110, &query),
                    ]
                    .concat(),
                ),
                aml::device(
                    "GFX1",
                    &[
                        aml::named("_ADR", &aml::integer(0x0003_0000)),
                        output("LCD3", 0x400, &[]),
                    ]
                    .concat(),
                ),
            ]
            .concat(),
        )
    }

    #[test]
    fn an_output_its_buses_dod_leaves_out_is_no_control() {
        let _serial = TEST_SERIAL.lock().unwrap();
        // SAFETY: the fixture owns every ACPICA service; serialized singleton.
        let session =
            unsafe { Session::start(test_support::dsdt_fixture(&buses()), Mode::InterpreterOnly) }
                .unwrap();
        let mut found = controls(&session).unwrap();
        let paths: Vec<_> = found.controls.iter().map(|control| &control.path).collect();
        assert_eq!(
            paths,
            ["\\_SB_.GFX0.LCD0", "\\_SB_.GFX0.LCD2", "\\_SB_.GFX1.LCD3"]
        );
        assert_eq!(found.undeclared, ["\\_SB_.GFX0.LCD1"]);
        // Without _BQC the first press starts from the level the
        // registration set, the highest where nothing was read.
        let lcd3 = &mut found.controls[2];
        assert_eq!(
            lcd3.initialize(),
            Ok(Brightness {
                level: 100,
                max: 100,
                read: None
            })
        );
        let down = lcd3.switch(NOTIFY_DEC).unwrap();
        assert_eq!((down.from, down.to, down.read), (100, 70, false));
        drop(found);
        session.shutdown().unwrap();
    }

    /// `STELA_TEST_DSDT` names the ASUS TUF A16 FA608PP's DSDT; skipped
    /// without one. Fn+F8's EC query `_Q17` notifies the iGPU's panel
    /// output `LCD` with 0x86 (`BRTN`), and `LCD`, which `VGA`'s `_DOD`
    /// lists, is its one output with `_BCL` and `_BCM`. The DSDT declares
    /// the dGPU's `EDP1` and `LCD`'s `AFN7` (the ATIF call its `_BCM`
    /// makes) External; which table defines them is unverified, as the
    /// SSDT dump at hand holds one table in all of its 21 files. Here
    /// `_BCM` ends at the missing `AFN7`.
    #[test]
    fn the_fa608pps_brightness_key_reaches_its_lcd_control() {
        static NOTIFIED: AtomicU32 = AtomicU32::new(0);
        let Some(services) = test_support::named_dsdt_fixture() else {
            return;
        };
        let _serial = TEST_SERIAL.lock().unwrap();
        // SAFETY: the fixture owns every ACPICA service; serialized singleton.
        let session = unsafe { Session::start(services, Mode::InterpreterOnly) }.unwrap();
        let mut found = controls(&session).unwrap();
        let paths: Vec<_> = found.controls.iter().map(|control| &control.path).collect();
        assert_eq!(paths, ["\\_SB_.PCI0.GP17.VGA_.LCD_"]);
        let control = &mut found.controls[0];
        assert_eq!((control.levels.max(), control.levels.count()), (100, 50));
        let notification = control
            .handle()
            .notify(|value| NOTIFIED.store(value, Ordering::Release))
            .unwrap();
        // BRTN notifies EDP1 next, which this namespace lacks.
        let _ = session
            .handle("\\_SB.PCI0.SBRG.EC0._Q17")
            .unwrap()
            .invoke(None, &[]);
        // SAFETY: serialized test process context outside an AML frame.
        unsafe { acpi_os_wait_events_complete() };
        let event = NOTIFIED.load(Ordering::Acquire);
        assert_eq!(event, NOTIFY_INC);
        assert_eq!(control.switch(event), Err(crate::platform::acpi::NOT_FOUND));
        drop(notification);
        drop(found);
        session.shutdown().unwrap();
    }
}

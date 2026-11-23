//! The Wi-Fi claim's reset decisions over a model configuration space: which
//! reset each chip gets, that a chip firmware left on is reset like any
//! other, and that every unprovable case keeps the seal untouched.
extern crate std;
use super::*;
use crate::bus::pci::{Address, ConfigError, power};
use alloc::{boxed::Box, rc::Rc, vec::Vec};
use core::cell::RefCell;

const FLR: u32 = 1 << 28;

#[derive(Default)]
struct State {
    events: Vec<&'static str>,
    waited_us: u64,
    /// A Secondary Bus Reset is offered, and whether it fails.
    bus_reset: bool,
    fail_bus_reset: bool,
    /// What the function's identity reads after a reset, when it changes.
    identity_after_reset: Option<u32>,
    /// The Command register the bus reset's restore leaves.
    command_after_bus_reset: Option<u32>,
}

struct Config {
    words: Box<[u32; 128]>,
    state: Rc<RefCell<State>>,
}

impl Config {
    fn reset(&mut self) {
        if let Some(identity) = self.state.borrow().identity_after_reset {
            self.words[0] = identity;
        }
    }
}

impl ConfigSpace for Config {
    fn read8(&self, offset: u16) -> Result<u8, ConfigError> {
        Ok((self.read32(offset & !3)? >> (8 * (offset & 3))) as u8)
    }
    fn read16(&self, offset: u16) -> Result<u16, ConfigError> {
        Ok((self.read32(offset & !3)? >> (8 * (offset & 3))) as u16)
    }
    fn write8(&mut self, offset: u16, value: u8) -> Result<(), ConfigError> {
        let shift = 8 * (offset & 3);
        let word = self.read32(offset & !3)?;
        self.write32(
            offset & !3,
            (word & !(0xff << shift)) | (u32::from(value) << shift),
        )
    }
    fn write16(&mut self, offset: u16, value: u16) -> Result<(), ConfigError> {
        let shift = 8 * (offset & 3);
        let word = self.read32(offset & !3)?;
        self.write32(
            offset & !3,
            (word & !(0xffff << shift)) | (u32::from(value) << shift),
        )
    }
    fn read32(&self, offset: u16) -> Result<u32, ConfigError> {
        Ok(self.words[usize::from(offset / 4)])
    }
    fn write32(&mut self, offset: u16, value: u32) -> Result<(), ConfigError> {
        let before = self.words[usize::from(offset / 4)];
        self.words[usize::from(offset / 4)] = value;
        if offset == 4 && value & u32::from(regs::COMMAND_MASTER) != 0 {
            self.state.borrow_mut().events.push("bus-master");
        }
        if offset == 0x84 && before & 3 == 3 && value & 3 == 0 {
            self.state.borrow_mut().events.push("d0");
        }
        if offset == 0x48 && value & 0x8000 != 0 {
            self.state.borrow_mut().events.push("flr");
            self.words[0x48 / 4] = 0;
            self.reset();
        }
        Ok(())
    }
}

/// A cold requester over the model: its function and the bus reset the
/// mount would offer.
struct Peer {
    function: Function<Config>,
    state: Rc<RefCell<State>>,
}

impl Cold for Peer {
    type Config = Config;
    fn function_mut(&mut self) -> &mut Function<Config> {
        &mut self.function
    }
    fn bus_reset_available(&self) -> bool {
        self.state.borrow().bus_reset
    }
    fn reset_bus(&mut self, timer: &dyn Timer) -> Result<(), BlockError> {
        timer.wait_us(102_000).map_err(BlockError::from)?;
        self.state.borrow_mut().events.push("bus-reset");
        if self.state.borrow().fail_bus_reset {
            return Err(BlockError::Unsupported("the bridge did not come back"));
        }
        self.function.config().reset();
        if let Some(command) = self.state.borrow().command_after_bus_reset {
            self.function.config().words[1] = command;
        }
        Ok(())
    }
}

struct ModelTimer(Rc<RefCell<State>>);

// SAFETY: the model's time advances exactly by each requested minimum; it
// owns no native timing source and reaches no hardware.
unsafe impl Timer for ModelTimer {
    fn check(&self) -> Result<(), power::Error> {
        Ok(())
    }
    fn wait_us(&self, minimum: u64) -> Result<(), power::Error> {
        self.0.borrow_mut().waited_us += minimum;
        Ok(())
    }
}

/// A parked function `vendor:device`: memory decoding and INTx Disable on,
/// Bus Master clear, a PCI Express capability at 0x40 whose Device
/// Capabilities hold `devcap`, and a power management capability at 0x80
/// in D0.
fn model(vendor: u16, device: u16, devcap: u32) -> (Peer, Rc<RefCell<State>>) {
    let state = Rc::new(RefCell::new(State::default()));
    let mut words = [0; 128];
    words[0] = u32::from(vendor) | u32::from(device) << 16;
    words[1] = 0x0010_0402;
    words[2] = 0x0280_0000;
    words[4] = 0xfe00_0000;
    words[0x34 / 4] = 0x40;
    words[0x40 / 4] = 0x0002_8010;
    words[0x44 / 4] = devcap;
    words[0x80 / 4] = 0x0003_0001;
    words[0x100 / 4] = u32::MAX;
    let function = Function::new(
        Address {
            segment: 0,
            bus: 2,
            device: 0,
            function: 0,
        },
        Config {
            words: Box::new(words),
            state: state.clone(),
        },
    )
    .unwrap();
    (
        Peer {
            function,
            state: state.clone(),
        },
        state,
    )
}

fn run(peer: &mut Peer, chip: Chip) -> Result<Quiesced, Error> {
    let timer = ModelTimer(peer.state.clone());
    quiesce(peer, chip, &timer)
}

const AX210: (u16, u16) = (0x8086, 0x2725);
const MT7925: (u16, u16) = (0x14c3, 0x7925);
const RTL8822CE: (u16, u16) = (0x10ec, 0xc822);

#[test]
fn plan_takes_flr_first_then_the_bus_and_otherwise_nothing() {
    assert_eq!(plan(Ok(()), false), Ok(Reset::FunctionLevel));
    assert_eq!(plan(Ok(()), true), Ok(Reset::FunctionLevel));
    for why in [Flr::Absent, Flr::Broken] {
        assert_eq!(plan(Err(why), true), Ok(Reset::SecondaryBus));
        assert_eq!(plan(Err(why), false), Err(Error::NoReset(why)));
    }
}

/// An Intel radio, an RTL8822CE or an RTL8821CE that advertises FLR gets one, and no bus
/// reset even when one is offered; it answers again as the same stopped
/// function, and Bus Master is never written.
#[test]
fn a_radio_with_flr_is_reset_function_level() {
    for ((vendor, device), chip) in [
        (AX210, Chip::Intel),
        (RTL8822CE, Chip::Rtl8822c),
        ((0x10ec, 0xc821), Chip::Rtl8821c),
    ] {
        let (mut peer, state) = model(vendor, device, FLR);
        state.borrow_mut().bus_reset = true;
        let quiesced = run(&mut peer, chip).unwrap();
        assert_eq!(quiesced.reset, Reset::FunctionLevel);
        assert_eq!(quiesced.chip, chip);
        assert_eq!(state.borrow().events, ["flr"]);
        assert!(state.borrow().waited_us >= 100_000);
    }
}

/// Without FLR, an Intel radio or an RTL8822CE gets the bus reset the mount
/// offers.
#[test]
fn a_radio_without_flr_gets_the_bus_reset() {
    for ((vendor, device), chip) in [
        (AX210, Chip::Intel),
        (RTL8822CE, Chip::Rtl8822c),
        ((0x10ec, 0xc821), Chip::Rtl8821c),
    ] {
        let (mut peer, state) = model(vendor, device, 0);
        state.borrow_mut().bus_reset = true;
        let quiesced = run(&mut peer, chip).unwrap();
        assert_eq!(quiesced.reset, Reset::SecondaryBus);
        assert_eq!(state.borrow().events, ["bus-reset"]);
    }
}

/// An MT7925 is never given its FLR, which Linux's quirk_no_flr refuses,
/// even when it advertises one, under either of its IDs: it gets the bus
/// reset, or stays sealed.
#[test]
fn an_mt7925_never_gets_its_flr() {
    for device in [0x7925, 0x0717] {
        let (mut peer, state) = model(0x14c3, device, FLR);
        assert_eq!(
            flr(&mut peer.function, Chip::Mt7925),
            Ok(Err(Flr::Broken)),
            "{device:04x}"
        );
        state.borrow_mut().bus_reset = true;
        let quiesced = run(&mut peer, Chip::Mt7925).unwrap();
        assert_eq!(quiesced.reset, Reset::SecondaryBus);
        assert_eq!(state.borrow().events, ["bus-reset"]);

        let (mut peer, state) = model(0x14c3, device, FLR);
        assert_eq!(
            run(&mut peer, Chip::Mt7925),
            Err(Error::NoReset(Flr::Broken))
        );
        assert!(state.borrow().events.is_empty());
    }
    // The quirk names the IDs whatever the claim calls the chip.
    let (mut peer, _) = model(MT7925.0, MT7925.1, FLR);
    assert_eq!(flr(&mut peer.function, Chip::Intel), Ok(Err(Flr::Broken)));
    let (mut peer, _) = model(0x14c3, 0x0616, FLR);
    assert_eq!(flr(&mut peer.function, Chip::Intel), Ok(Err(Flr::Broken)));
}

/// An MT7922 is denied its FLR under each of its IDs, as the MT7925 is,
/// Linux's quirk_no_flr naming 14c3:0616; an MT7921, which the quirk does
/// not name, gets the FLR it advertises.
#[test]
fn an_mt7922_never_gets_its_flr_and_an_mt7921_does() {
    for (vendor, device) in [(0x14c3, 0x7922), (0x14c3, 0x0616), (0x0b48, 0x7922)] {
        let (mut peer, state) = model(vendor, device, FLR);
        assert_eq!(
            flr(&mut peer.function, Chip::Mt7922),
            Ok(Err(Flr::Broken)),
            "{vendor:04x}:{device:04x}"
        );
        state.borrow_mut().bus_reset = true;
        let quiesced = run(&mut peer, Chip::Mt7922).unwrap();
        assert_eq!(quiesced.reset, Reset::SecondaryBus);
        assert_eq!(state.borrow().events, ["bus-reset"]);
    }
    for device in [0x7961, 0x0608] {
        let (mut peer, state) = model(0x14c3, device, FLR);
        state.borrow_mut().bus_reset = true;
        let quiesced = run(&mut peer, Chip::Mt7921).unwrap();
        assert_eq!(quiesced.reset, Reset::FunctionLevel);
        assert_eq!(quiesced.chip, Chip::Mt7921);
        assert_eq!(state.borrow().events, ["flr"]);
    }
    assert_eq!(Chip::Mt7921.name(), "MT7921");
    assert_eq!(Chip::Mt7922.name(), "MT7922");
    use crate::net::wifi::mediatek::identity::Chip as Mediatek;
    for (mediatek, chip) in [
        (Mediatek::Mt7921, Chip::Mt7921),
        (Mediatek::Mt7922, Chip::Mt7922),
        (Mediatek::Mt7925, Chip::Mt7925),
    ] {
        assert_eq!(Chip::mediatek(mediatek), chip);
    }
}

/// A radio neither reset reaches stays sealed: nothing is written, and the
/// log says why.
#[test]
fn a_radio_no_reset_reaches_stays_sealed() {
    for ((vendor, device), chip) in [
        (AX210, Chip::Intel),
        (RTL8822CE, Chip::Rtl8822c),
        (MT7925, Chip::Mt7925),
    ] {
        let (mut peer, state) = model(vendor, device, 0);
        let before = peer.function.config().words.clone();
        let error = run(&mut peer, chip).unwrap_err();
        let why = if chip == Chip::Mt7925 {
            Flr::Broken
        } else {
            Flr::Absent
        };
        assert_eq!(error, Error::NoReset(why));
        assert!(state.borrow().events.is_empty());
        assert_eq!(peer.function.config().words, before);
    }
    assert_eq!(
        alloc::format!("{}", Error::NoReset(Flr::Absent)),
        "kept sealed, Bus Master stays off: nothing proves its engines stopped: it offers no \
         Function Level Reset, and no Secondary Bus Reset of its bus is offered (it sits on a \
         root bus, or another device or a bridge shares its bus)"
    );
    assert_eq!(
        alloc::format!("{}", Error::NoReset(Flr::Broken)),
        "kept sealed, Bus Master stays off: nothing proves its engines stopped: its Function \
         Level Reset is broken (Linux quirk_no_flr), and no Secondary Bus Reset of its bus is \
         offered (it sits on a root bus, or another device or a bridge shares its bus)"
    );
}

/// A radio firmware left on, here in D3hot with its memory decoding on, is
/// brought to D0 and then reset like any other: the reset reads nothing of
/// the chip's own state. The log names how it was found.
#[test]
fn a_radio_firmware_left_on_is_reset_after_d0() {
    let (mut peer, state) = model(AX210.0, AX210.1, FLR);
    peer.function.config().words[0x84 / 4] = 3;
    let quiesced = run(&mut peer, Chip::Intel).unwrap();
    assert_eq!(
        quiesced.found,
        Found {
            power_state: 3,
            memory: true
        }
    );
    assert_eq!(state.borrow().events, ["d0", "flr"]);
    assert_eq!(
        alloc::format!("{quiesced}"),
        "Wi-Fi claim: Intel Wi-Fi quiesced by a Function Level Reset before Bus Master (found \
         in D3, memory decoding on)"
    );
    let (mut peer, _) = model(MT7925.0, MT7925.1, 0);
    peer.function.config().words[1] = 0x0010_0400;
    peer.state.borrow_mut().bus_reset = true;
    let quiesced = run(&mut peer, Chip::Mt7925).unwrap();
    assert_eq!(
        alloc::format!("{quiesced}"),
        "Wi-Fi claim: MT7925 quiesced by a Secondary Bus Reset before Bus Master (found in D0, \
         memory decoding off)"
    );
}

/// A function found mastering was not parked as the mount promises: it is
/// neither powered up nor reset.
#[test]
fn a_radio_found_mastering_is_refused_untouched() {
    let (mut peer, state) = model(AX210.0, AX210.1, FLR);
    peer.function.config().words[1] |= u32::from(regs::COMMAND_MASTER);
    peer.function.config().words[0x84 / 4] = 3;
    assert!(matches!(
        run(&mut peer, Chip::Intel),
        Err(Error::NotStopped(ClaimError::Active { .. }))
    ));
    assert!(state.borrow().events.is_empty());
    assert_eq!(peer.function.config().words[0x84 / 4], 3);
}

/// A failed bus reset keeps the seal and names the failure.
#[test]
fn a_failed_bus_reset_keeps_the_seal() {
    let (mut peer, state) = model(MT7925.0, MT7925.1, 0);
    state.borrow_mut().bus_reset = true;
    state.borrow_mut().fail_bus_reset = true;
    let error = run(&mut peer, Chip::Mt7925).unwrap_err();
    assert_eq!(
        error,
        Error::Reset(
            Reset::SecondaryBus,
            BlockError::Unsupported("the bridge did not come back")
        )
    );
    assert_eq!(
        alloc::format!("{error}"),
        "kept sealed, Bus Master stays off: its Secondary Bus Reset failed: unsupported: the \
         bridge did not come back"
    );
}

/// A function that comes back from its reset as another function, or
/// mastering, is refused: the reset proves nothing of it.
#[test]
fn a_radio_not_itself_after_its_reset_is_refused() {
    let (mut peer, state) = model(AX210.0, AX210.1, FLR);
    state.borrow_mut().identity_after_reset = Some(0x1234_8086);
    assert_eq!(
        run(&mut peer, Chip::Intel),
        Err(Error::AfterReset(
            Reset::FunctionLevel,
            ClaimError::IdentityChanged
        ))
    );
    let (mut peer, state) = model(MT7925.0, MT7925.1, 0);
    state.borrow_mut().bus_reset = true;
    state.borrow_mut().command_after_bus_reset = Some(0x0010_0406);
    assert!(matches!(
        run(&mut peer, Chip::Mt7925),
        Err(Error::AfterReset(
            Reset::SecondaryBus,
            ClaimError::Active { .. }
        ))
    ));
}

/// A function whose Device Capabilities read all-ones is refused before any
/// reset.
#[test]
fn an_unanswering_radio_is_refused_before_any_reset() {
    let (mut peer, state) = model(AX210.0, AX210.1, u32::MAX);
    state.borrow_mut().bus_reset = true;
    assert_eq!(
        run(&mut peer, Chip::Intel),
        Err(Error::Unreadable("its Device Capabilities read all-ones"))
    );
    assert!(state.borrow().events.is_empty());
}

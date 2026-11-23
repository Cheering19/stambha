//! The cold claim of a Wi-Fi function whose driver sets Bus Master before it
//! resets its engines: an Intel radio (iwlwifi), a MediaTek MT7925 (mt7925e)
//! or a Realtek RTL8822CE (rtw88). Their claim used to keep the
//! preparation's seal for good, so the Bus Master each driver sets was
//! refused and none became a station (issues #224 and #18).
//!
//! The mount parks every function with Bus Master clear, drains its
//! outstanding requests and seals the bit. Here, before the seal may be
//! released, the function is reset back to its power-on state by a PCI
//! Express reset, as Linux `pci_reset_function` would reset it, and only a
//! completed reset is acknowledged as its quiescence:
//! - a Function Level Reset, when its Device Capabilities advertise one and
//!   Linux's `quirk_no_flr` does not name it;
//! - else a Secondary Bus Reset of its own bus, when the mount offers one:
//!   only through the forwarding bridge directly above it, with every other
//!   function on that bus a parked, unclaimed function of the same device.
//!
//! Either reset returns every engine of the function, its embedded firmware
//! included, to its initial state (PCI Express Base Specification, 6.6), so
//! nothing the firmware or an earlier driver left running survives it. A
//! chip left on, in a low power state its own firmware manages, or in any
//! undefined state is reset with the rest, since the reset reads nothing of
//! the chip's own state; and with Bus Master clear since the park, nothing
//! the chip did before it reached memory. That is why, unlike the RTL8852CE
//! (whose claim has no reset and so refuses LPS and the undefined state,
//! `realtek::ax::quiesce`), these chips need no per-state proof. After the
//! reset the function must answer again as the same parked function, with
//! Bus Master, MSI, MSI-X and INTx off (`requester::validate_stopped`).
//!
//! A function neither reset reaches, or whose reset fails, stays sealed: its
//! preparation is kept as it is, its driver never attaches, and the boot log
//! says why ([`Error`]). The D3hot to D0 soft reset `pci_reset_function` also
//! tries is not used: only the function's own No_Soft_Reset claim vouches
//! for it.
use crate::{
    block::BlockError,
    bus::pci::{
        CAPABILITY_EXPRESS, CAPABILITY_POWER, CapabilityError, ConfigError, ConfigSpace, Function,
        power::Timer,
        regs,
        requester::{self, ClaimError, preparation::PreparingRequester},
    },
};
use core::fmt;

/// The Wi-Fi driver a function is claimed for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chip {
    /// An Intel radio `iwl_pci_probe` identifies (`intel::identity::device`).
    Intel,
    /// A MediaTek MT7925 (`mediatek::identity::Chip::Mt7925`).
    Mt7925,
    /// A Realtek RTL8822CE (`realtek::identity::RTL8822C_DEVICES`).
    Rtl8822c,
}

impl Chip {
    /// The chip as the boot log names it.
    pub fn name(self) -> &'static str {
        match self {
            Self::Intel => "Intel Wi-Fi",
            Self::Mt7925 => "MT7925",
            Self::Rtl8822c => "RTL8822CE",
        }
    }
}

/// The reset that quiesced a function.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reset {
    FunctionLevel,
    SecondaryBus,
}

impl fmt::Display for Reset {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::FunctionLevel => "Function Level Reset",
            Self::SecondaryBus => "Secondary Bus Reset",
        })
    }
}

/// Why a function has no Function Level Reset to quiesce it with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Flr {
    /// Its Device Capabilities advertise none, or it has no PCI Express
    /// capability.
    Absent,
    /// It advertises one Linux's `quirk_no_flr` refuses: a MediaTek MT7922
    /// or MT7925, whose configuration reads stay all-ones after it. Every
    /// MT7925 ID is treated so.
    Broken,
}

/// The function as firmware left it, before the reset: its power state (0
/// for D0, 3 for D3hot) and whether its memory decoding was on, which a
/// firmware driver that used the chip leaves on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Found {
    pub power_state: u8,
    pub memory: bool,
}

/// A function reset to its power-on state, ready for its receipt.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Quiesced {
    pub chip: Chip,
    pub reset: Reset,
    pub found: Found,
}

impl fmt::Display for Quiesced {
    /// The boot log's warning for a function whose claim went through.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Wi-Fi claim: {} quiesced by a {} before Bus Master (found in D{}, memory decoding {})",
            self.chip.name(),
            self.reset,
            self.found.power_state,
            if self.found.memory { "on" } else { "off" },
        )
    }
}

/// Why a function stays sealed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
    /// It was not parked stopped as the mount promised.
    NotStopped(ClaimError),
    /// Its capabilities or configuration could not be read.
    Unreadable(&'static str),
    /// Its power-up to D0 failed.
    Power(BlockError),
    /// Neither reset can reach it: why it has no FLR, and the mount offers
    /// no Secondary Bus Reset of its bus.
    NoReset(Flr),
    /// The reset failed or did not happen.
    Reset(Reset, BlockError),
    /// After the reset it did not answer as the same stopped function.
    AfterReset(Reset, ClaimError),
}

impl From<CapabilityError> for Error {
    fn from(_: CapabilityError) -> Self {
        Self::Unreadable("its capability list is malformed or unreadable")
    }
}

impl From<ConfigError> for Error {
    fn from(_: ConfigError) -> Self {
        Self::Unreadable("its configuration space could not be read")
    }
}

/// Why no Secondary Bus Reset is offered, as far as the probe can see.
const NO_BUS_RESET: &str = "no Secondary Bus Reset of its bus is offered (it sits on a root bus, \
                            or another device or a bridge shares its bus)";

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("kept sealed, Bus Master stays off: ")?;
        match self {
            Self::NotStopped(error) => write!(f, "it was not parked stopped: {error}"),
            Self::Unreadable(why) => f.write_str(why),
            Self::Power(error) => write!(f, "its power-up to D0 failed: {error}"),
            Self::NoReset(Flr::Absent) => write!(
                f,
                "nothing proves its engines stopped: it offers no Function Level Reset, and \
                 {NO_BUS_RESET}"
            ),
            Self::NoReset(Flr::Broken) => write!(
                f,
                "nothing proves its engines stopped: its Function Level Reset is broken (Linux \
                 quirk_no_flr), and {NO_BUS_RESET}"
            ),
            Self::Reset(reset, error) => write!(f, "its {reset} failed: {error}"),
            Self::AfterReset(reset, error) => write!(
                f,
                "after its {reset} it did not answer as the same stopped function: {error}"
            ),
        }
    }
}

/// What the claim needs of a cold requester: its function, and the
/// Secondary Bus Reset the mount offers for it.
pub trait Cold {
    type Config: ConfigSpace;
    fn function_mut(&mut self) -> &mut Function<Self::Config>;
    /// Whether a Secondary Bus Reset is offered and would find a bus it may
    /// reset now. Reads only.
    fn bus_reset_available(&self) -> bool;
    /// Reset the function's bus, its state saved and restored around it.
    fn reset_bus(&mut self, timer: &dyn Timer) -> Result<(), BlockError>;
}

impl Cold for PreparingRequester {
    type Config = crate::bus::pci::owned::OwnedConfig;
    fn function_mut(&mut self) -> &mut Function<Self::Config> {
        PreparingRequester::function_mut(self)
    }
    fn bus_reset_available(&self) -> bool {
        PreparingRequester::bus_reset_available(self)
    }
    fn reset_bus(&mut self, timer: &dyn Timer) -> Result<(), BlockError> {
        PreparingRequester::reset_bus(self, timer)
    }
}

/// Linux's `quirk_no_flr` among Wi-Fi functions (`drivers/pci/quirks.c`):
/// MediaTek MT7922 (14c3:0616) and MT7925 (14c3:7925).
fn flr_quirk(vendor: u16, device: u16) -> bool {
    vendor == 0x14c3 && matches!(device, 0x0616 | 0x7925)
}

/// Whether `function` offers a Function Level Reset `chip`'s claim may use.
/// Reads only.
pub fn flr<C: ConfigSpace>(
    function: &mut Function<C>,
    chip: Chip,
) -> Result<Result<(), Flr>, Error> {
    if chip == Chip::Mt7925 || flr_quirk(function.vendor, function.device) {
        return Ok(Err(Flr::Broken));
    }
    let Some(express) = function
        .capabilities()?
        .into_iter()
        .find(|&(_, id)| id == CAPABILITY_EXPRESS)
        .map(|(offset, _)| u16::from(offset))
    else {
        return Ok(Err(Flr::Absent));
    };
    let capabilities = function.config().read32(express + 4)?;
    if capabilities == u32::MAX {
        return Err(Error::Unreadable("its Device Capabilities read all-ones"));
    }
    Ok(if capabilities & (1 << 28) != 0 {
        Ok(())
    } else {
        Err(Flr::Absent)
    })
}

/// The reset that quiesces a function with `flr` and, when `bus_reset`, a
/// Secondary Bus Reset of its bus on offer: FLR first, as Linux
/// `pci_reset_function` orders them, then the bus.
pub fn plan(flr: Result<(), Flr>, bus_reset: bool) -> Result<Reset, Error> {
    match (flr, bus_reset) {
        (Ok(()), _) => Ok(Reset::FunctionLevel),
        (Err(_), true) => Ok(Reset::SecondaryBus),
        (Err(why), false) => Err(Error::NoReset(why)),
    }
}

/// `function` as firmware left it. Reads only.
fn found<C: ConfigSpace>(function: &mut Function<C>) -> Result<Found, Error> {
    let memory = function.config().read16(regs::COMMAND)? & regs::COMMAND_MEMORY != 0;
    let mut power_state = 0;
    if let Some((offset, _)) = function
        .capabilities()?
        .into_iter()
        .find(|&(_, id)| id == CAPABILITY_POWER)
    {
        let control = function.config().read16(u16::from(offset) + 4)?;
        if control == u16::MAX {
            return Err(Error::Unreadable(
                "its power management control reads all-ones",
            ));
        }
        power_state = (control & 3) as u8;
    }
    Ok(Found {
        power_state,
        memory,
    })
}

/// Reset the parked function `cold` holds for `chip` back to its power-on
/// state, with Bus Master sealed clear throughout: check it is parked
/// stopped, bring it to D0, reset it ([`plan`]), then check it answers as
/// the same stopped function. A function this refuses stays sealed, and the
/// caller keeps its preparation.
pub fn quiesce<P: Cold + ?Sized>(
    cold: &mut P,
    chip: Chip,
    timer: &dyn Timer,
) -> Result<Quiesced, Error> {
    requester::validate_stopped(cold.function_mut()).map_err(Error::NotStopped)?;
    let found = found(cold.function_mut())?;
    cold.function_mut().power_up(timer).map_err(Error::Power)?;
    let reset = plan(flr(cold.function_mut(), chip)?, cold.bus_reset_available())?;
    match reset {
        Reset::FunctionLevel => {
            let reset_done = cold
                .function_mut()
                .function_level_reset(timer)
                .map_err(|error| Error::Reset(reset, error))?;
            if !reset_done {
                return Err(Error::Reset(
                    reset,
                    BlockError::Unsupported("the function no longer offers it"),
                ));
            }
        }
        Reset::SecondaryBus => cold
            .reset_bus(timer)
            .map_err(|error| Error::Reset(reset, error))?,
    }
    requester::validate_stopped(cold.function_mut())
        .map_err(|error| Error::AfterReset(reset, error))?;
    Ok(Quiesced { chip, reset, found })
}

#[cfg(test)]
mod claim_tests;
#[cfg(test)]
mod tests;

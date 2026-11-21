// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! One RTL8852CE taken from its register window to a kept station (M3 spec
//! S16.1), as rtw88's `realtek/attach.rs` takes an RTL8822C: the regulatory
//! rules of the network's country, then `rtw89_ops_start`
//! (`mac80211.c:57`), whose `rtw89_core_start` (`core.c:6658`) and idle exit
//! [`live::start`] runs, then mac80211's first station on the chip, the
//! [`Rtl8852c`] radio, kept connected by the shared connection keeper. What
//! `rtw89_pci_probe` (`pci.c:4769`) does before, staging the firmware and
//! taking the function over, is the caller's.
//!
//! Two countries are chosen here, and they can differ:
//! - the channel rules (which channels the station scans and joins on, and
//!   the power cfg80211 allows there) are regulatory.db's for the network's
//!   country, or for the world domain `00` that cfg80211 starts from when
//!   the network names none, as rtw88's attach takes them;
//! - the TX power (Realtek's per-regd byrate and limit tables) follows the
//!   regulatory index of the efuse country when the firmware's map has it,
//!   else the network's, else the world's ([`super::regd::select`], after
//!   `rtw89_regd_init_hint`, `regd.c:738-768`).
//!
//! Linux takes both from the efuse country of a programmed chip: it hints
//! cfg80211 with it under `REGULATORY_STRICT_REG` and ignores the APs'
//! country elements (`regd.c:747-757`), so its channel rules follow the
//! efuse too. Stela keeps the configured network's country for the channel
//! rules, so a network configured for another country than the efuse's
//! scans and joins under that country's rules with the efuse country's TX
//! power limits.
//!
//! [`attach_reporting`] hands back the bring-up's report beside the
//! station, for the normal boot's probe (M3 spec S16.2, `stela-drivers`'
//! `probe::native::rtw89_keep`), which also refuses a missing regulatory
//! database ([`AttachError::NoRegulatoryDatabase`]) before it claims the
//! chip; the host check's station check (`--wifi-check --wifi`) runs
//! [`attach`].
//!
//! This grants what [`live::start`] and [`Rtl8852c`] grant and nothing
//! more: no interrupt, and no teardown on drop. A kept station that is
//! dropped leaves its chip running with its rings and pools quarantined, as
//! [`Live`] documents; the caller stops it through
//! [`crate::wifi::connection::keeper::Keeper::into_radio`],
//! [`Rtl8852c::into_chip`] and [`Live::stop`].
use super::bringup::{BringUpError, FirmwareFile, Host, Running};
use super::io::AxIo;
use super::live::{self, Live, StartError};
use super::pci::Rings;
use super::station::{self, Rtl8852c, Wifi};
use crate::bus::dma::{mapping::DmaDomain, memory::PageSource};
use crate::wifi::{
    network::Network,
    regulatory::database::{self, Database},
    scan::Scanner,
    station::Station,
};
use crate::{Clock, Mac};
use alloc::sync::Arc;
use core::fmt;
use stela_driver_base::entropy::Entropy;

/// What a station needs of the machine around it, besides the chip.
pub struct Inputs {
    pub clock: Clock,
    /// regulatory.db, whose country the network names, or the world domain
    /// `00` cfg80211 starts from when it names none.
    pub database: Database,
    pub network: Network,
    /// The source of the station's handshake nonces.
    pub entropy: Arc<dyn Entropy>,
}

/// Why the chip did not reach a station.
#[derive(Debug)]
pub enum AttachError {
    /// No regulatory database loaded, so the radio may not transmit: the
    /// probe's refusal, as rtw88's (`realtek::attach`), before the chip is
    /// claimed. [`attach`] itself is always given one.
    NoRegulatoryDatabase,
    /// regulatory.db holds no usable rules for the network's country (or
    /// `00`); refused before any register access.
    Country([u8; 2], database::Error),
    /// [`live::start`] failed at a step, and powered the chip off.
    Start(BringUpError),
    /// The efuse address cannot be a station's; the chip was stopped.
    Station(station::Error),
}

impl fmt::Display for AttachError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoRegulatoryDatabase => f.write_str("RTL8852CE: no regulatory database"),
            Self::Country(alpha2, error) => write!(
                f,
                "RTL8852CE: regulatory.db country {}: {error:?}",
                alpha2.escape_ascii()
            ),
            // "RTL8852CE bring-up: <step>: …".
            Self::Start(error) => write!(f, "{error}"),
            Self::Station(error) => write!(f, "RTL8852CE station: {error}"),
        }
    }
}

/// The register window handed back with an [`AttachError`], and the rings
/// once [`live::start`] installed them, which must be kept: nothing proves
/// the DMA stopped. `None` only if the chip went with a radio that could not
/// be made, which the address check before it rules out.
pub type Returned<I> = Option<(I, Option<Rings>)>;

/// A kept station and what its bring-up reported ([`attach_reporting`]).
pub type Reported<I> = (Wifi<Live<I>>, Running);

/// The address [`Rtl8852c::new`] would refuse, checked while the chip can
/// still be stopped: `Station::new`'s and `Scanner::new`'s check.
fn station_address(address: Mac) -> Result<(), station::Error> {
    Station::new(address).map_err(station::Error::Station)?;
    Scanner::new(address).map_err(station::Error::Scan)?;
    Ok(())
}

/// Bring the RTL8852CE behind `io` up on `firmware` for `host` and keep it
/// connected to `inputs.network`: the network's country is selected in
/// `inputs.database` (else `00`) for the channel rules, before any register
/// access; [`live::start`] runs with the network's country as the TX power's
/// fallback after the efuse's; then the radio at the efuse address keeps the
/// connection, its handshake nonces from `inputs.entropy`.
///
/// A failure hands the window back, with the rings once they were
/// installed: an unknown country before anything ran, a failed start after
/// it powered the chip off, an address no station can have after the chip
/// was stopped ([`Live::stop`]).
///
/// # Safety
/// [`live::start`]'s contract, for as long as the station is used: `io` is
/// the exclusive register window of exactly the RTL8852C PCI requester that
/// `domain` isolates; `source` yields real coherent WB DMA RAM for that
/// requester, with NoSnoop and ATS disabled; one worker owns the device and
/// its queues, and no other engine, IRQ or reset worker touches them.
#[expect(
    clippy::result_large_err,
    reason = "A failure hands the window and the rings back by value: the rings are never freed, as nothing proves the DMA stopped."
)]
pub unsafe fn attach<I: AxIo>(
    io: I,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    cut: u8,
    firmware: FirmwareFile<'_>,
    host: Host,
    inputs: Inputs,
) -> Result<Wifi<Live<I>>, (AttachError, Returned<I>)> {
    // SAFETY: the caller's contract is `attach_reporting`'s, word for word.
    unsafe { attach_reporting(io, source, domain, cut, firmware, host, inputs) }
        .map(|(wifi, _)| wifi)
}

/// [`attach`], with what the bring-up reported ([`Live::running`]) beside
/// the station: the keeper keeps its radio, and with it the live chip, to
/// itself, so a caller that reports the bring-up, as a normal boot's probe
/// does, takes the report here.
///
/// # Safety
/// As for [`attach`].
#[expect(
    clippy::result_large_err,
    reason = "A failure hands the window and the rings back by value: the rings are never freed, as nothing proves the DMA stopped."
)]
pub unsafe fn attach_reporting<I: AxIo>(
    io: I,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    cut: u8,
    firmware: FirmwareFile<'_>,
    host: Host,
    inputs: Inputs,
) -> Result<Reported<I>, (AttachError, Returned<I>)> {
    let Inputs {
        clock,
        database,
        network,
        entropy,
    } = inputs;
    let alpha2 = network.country.unwrap_or(*b"00");
    let country = match database.select(alpha2) {
        Ok(country) => country,
        Err(error) => return Err((AttachError::Country(alpha2, error), Some((io, None)))),
    };
    // SAFETY: the caller's contract is `start`'s, word for word.
    let started = unsafe { live::start(io, source, domain, cut, firmware, host, network.country) };
    let live = match started {
        Ok(live) => live,
        Err(StartError { error, io, rings }) => {
            return Err((AttachError::Start(error), Some((io, rings))));
        }
    };
    let address = Mac(live.mac());
    if let Err(error) = station_address(address) {
        // SAFETY: `start`'s contract, still holding for this chip.
        let (io, rings, _pools) = unsafe { live.stop() };
        return Err((AttachError::Station(error), Some((io, Some(rings)))));
    }
    let running = *live.running();
    match Rtl8852c::new(live, address, country, clock) {
        Ok(radio) => Ok((radio.keep(network, entropy), running)),
        Err(error) => Err((AttachError::Station(error), None)),
    }
}

#[cfg(test)]
mod tests;

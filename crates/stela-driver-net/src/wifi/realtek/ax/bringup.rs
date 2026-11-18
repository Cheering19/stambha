// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE bring-up to running firmware, in the order and with the failure
//! handling of `rtw89_chip_info_setup` (`core.c:7383`): `rtw89_mac_pwr_on`
//! (`mac.c:1588`) with the tail of `rtw89_mac_power_switch(true)`
//! (`mac.c:1521`: `efuse_read_fw_secure`, then the `MAC_AX_NOTIFY_TP_MAJOR`
//! scoreboard), then `rtw89_chip_efuse_info_setup` (`core.c:7274`), i.e.
//! `rtw89_mac_partial_init` (`mac.c:4307`: MAC pre-init, PCI pre-init,
//! `rtw89_fw_download` with its 5-try loop, `fw.c:2043`) and
//! `rtw89_parse_efuse_map_ax` (`efuse.c:281`) for the MAC address, and
//! finally `rtw89_mac_pwr_off` (`mac.c:1603`), which Linux runs on success
//! and on every failure after power-on.
//!
//! This module does not grant a usable radio: the chip is powered off again
//! before `bring_up` returns, as Linux's probe leaves it, and nothing past
//! the firmware's ready status (phycap, BB/RF, H2C, C2H, scan) is ported.
//! It does not support secure-boot chips, and it does not claim the MAC
//! address read from the efuse is valid.
use super::efuse::{self, SecureBoot};
use super::io::{AxError, AxIo};
use super::pci::Rings;
use super::{dmac, fwdl, pci, power};
use crate::bus::dma::{mapping::DmaDomain, memory::PageSource};
use crate::wifi::realtek::firmware::AxImage;
use alloc::{sync::Arc, vec};
use core::fmt;

/// `R_AX_SCOREBOARD` (`reg.h:156`), the 8852C `btc_sb` register
/// (`rtw8852c.c:3287`); `rtw89_mac_update_scoreboard` writes byte 3.
const R_AX_SCOREBOARD: u32 = 0x00AC;
/// `MAC_AX_NOTIFY_TP_MAJOR` / `MAC_AX_NOTIFY_PWR_MAJOR` (`reg.h:161-162`).
const MAC_AX_NOTIFY_TP_MAJOR: u8 = 0x81;
const MAC_AX_NOTIFY_PWR_MAJOR: u8 = 0x80;
/// `rtw89_fw_download`: `for (retry = 0; retry < 5; retry++)` (`fw.c:2048`).
const FWDL_TRIES: usize = 5;

/// An RTL8852CE whose firmware reported `RTW89_FWDL_WCPU_FW_INIT_RDY`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Running {
    /// The chip cut (`enum rtw89_cv`, 0 = CAV) the bring-up was given.
    pub cut: u8,
    /// The downloaded image's version, major.minor.sub.index.
    pub firmware: (u8, u8, u8, u8),
    /// The efuse MAC address (logical map 0x400).
    pub mac: [u8; 6],
    /// Always false: a secure-boot chip is refused.
    pub secure_boot: bool,
}

impl fmt::Display for Running {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (a, b, c, d) = self.firmware;
        let m = self.mac;
        write!(
            f,
            "RTL8852CE firmware running: cut {}, firmware {a}.{b}.{c}.{d}, \
             MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, secure boot {}",
            self.cut,
            m[0],
            m[1],
            m[2],
            m[3],
            m[4],
            m[5],
            if self.secure_boot { "on" } else { "off" },
        )
    }
}

/// The step that failed and why.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BringUpError {
    /// One of `"power on"`, `"secure boot"`, `"dma rings"`,
    /// `"mac pre-init"`, `"pci pre-init"`, `"firmware download"`, `"efuse"`.
    pub step: &'static str,
    pub error: AxError,
}

impl fmt::Display for BringUpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "RTL8852CE bring-up: {}: {}", self.step, self.error)
    }
}

fn at(step: &'static str) -> impl FnOnce(AxError) -> BringUpError {
    move |error| BringUpError { step, error }
}

/// Bring an RTL8852CE of chip cut `cut` up to running firmware `image`, and
/// read its MAC address, then power it off again. Steps, in Linux's order:
///
/// 1. `"power on"`: `power::power_on`, with Linux's own power-off and
///    single retry (`rtw89_mac_pwr_on`). A failure here returns at once.
/// 2. `"secure boot"`: the efuse secure-boot bytes, then the
///    `MAC_AX_NOTIFY_TP_MAJOR` scoreboard write that closes
///    `rtw89_mac_power_switch(true)`. A secure-boot chip is refused with
///    `SecureBootUnsupported`.
/// 3. `"dma rings"`: `Rings::install`.
/// 4. `"mac pre-init"`: `dmac::pre_init`.
/// 5. `"pci pre-init"`: `pci::pre_init`.
/// 6. `"firmware download"`: up to five `fwdl::download` attempts
///    (`fw.c:2043`), the last error returned. An attempt whose CH12 submit
///    failed leaves CH12 stopped; the next attempt then runs `pci::pre_init`
///    again first (a failure there is reported as `"pci pre-init"`).
/// 7. `"efuse"`: the whole physical map, its logical map and the MAC.
///
/// After `"power on"` succeeded, success and every failure end as
/// `rtw89_chip_info_setup` does, in `rtw89_mac_pwr_off`: `power::power_off`,
/// then, only if it succeeded, the `MAC_AX_NOTIFY_PWR_MAJOR` scoreboard
/// write. Linux ignores the result of that power-off, and so does this.
///
/// On success the firmware result is reported with the `Rings` that DMA was
/// started on. The caller must keep them (dropping them leaves every mapping
/// quarantined, never returned): the power-off does not prove the engine
/// stopped reading them. On a failure after the rings were programmed, they
/// are dropped after the power-off and stay quarantined the same way.
///
/// # Safety
/// The contracts of `Rings::install`, `pci::pre_init` and `Ch12::submit`:
/// `io` is the exclusive register window of exactly the RTL8852C PCI
/// requester that `domain` isolates; `source` yields real coherent WB DMA
/// RAM for that requester, with NoSnoop and ATS disabled; one worker owns
/// the device and its queues, and no other engine, IRQ or reset worker
/// touches them.
pub unsafe fn bring_up<I: AxIo + ?Sized>(
    io: &mut I,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    cut: u8,
    image: &AxImage<'_>,
) -> Result<(Running, Rings), BringUpError> {
    power::power_on(io).map_err(at("power on"))?;
    // SAFETY: the caller's `Rings::install` contract, forwarded unchanged.
    let mut rings = match unsafe { prepare(io, source, domain) } {
        Ok(rings) => rings,
        Err(error) => {
            power_down(io);
            return Err(error);
        }
    };
    // SAFETY: the caller's `pci::pre_init` and `Ch12::submit` contracts;
    // `rings` were installed on `domain` for the requester behind `io`.
    let result = unsafe { start(io, cut, &mut rings, image) };
    power_down(io);
    // On a failure `rings` drop here, after the power-off.
    Ok((result?, rings))
}

/// `rtw89_mac_pwr_off` (`mac.c:1603`): `rtw89_mac_power_switch(false)`,
/// whose result Linux ignores. `rtw89_mac_set_vcore_reset` does nothing
/// until thermal protection set a voltage level, which nothing here does.
fn power_down<I: AxIo + ?Sized>(io: &mut I) {
    if power::power_off(io).is_ok() {
        // mac.c:1580, only after pwr_off_func succeeded.
        let _ = io.write8(R_AX_SCOREBOARD + 3, MAC_AX_NOTIFY_PWR_MAJOR);
    }
}

/// The tail of `rtw89_mac_power_switch(true)` (`mac.c:1557-1570`), then the
/// ring memory.
///
/// # Safety
/// The `Rings::install` contract.
unsafe fn prepare<I: AxIo + ?Sized>(
    io: &mut I,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
) -> Result<Rings, BringUpError> {
    // efuse_read_fw_secure (mac.c:1560), then rtw89_mac_update_scoreboard(
    // MAC_AX_NOTIFY_TP_MAJOR) (mac.c:1569). Linux only warns on an
    // unreadable secure-boot efuse and fails later, in
    // rtw89_parse_efuse_map_ax; here an unread state is not taken for "off".
    let secure = efuse::secure_boot(io).map_err(at("secure boot"))?;
    io.write8(R_AX_SCOREBOARD + 3, MAC_AX_NOTIFY_TP_MAJOR)
        .map_err(at("secure boot"))?;
    // The non-secure download path is the only one ported (fwdl.rs).
    if let SecureBoot::On { .. } = secure {
        return Err(at("secure boot")(AxError::SecureBootUnsupported));
    }
    // SAFETY: the caller's `Rings::install` contract.
    unsafe { Rings::install(source, domain) }.map_err(at("dma rings"))
}

/// `rtw89_chip_efuse_info_setup` (`core.c:7274`) up to the MAC address:
/// `rtw89_mac_partial_init` (`mac.c:4307`), then the efuse map.
///
/// # Safety
/// The `pci::pre_init` and `Ch12::submit` contracts for `io` and `rings`.
unsafe fn start<I: AxIo + ?Sized>(
    io: &mut I,
    cut: u8,
    rings: &mut Rings,
    image: &AxImage<'_>,
) -> Result<Running, BringUpError> {
    // rtw89_mac_ctrl_hci_dma_trx and rtw89_mac_dmac_pre_init, then the
    // HCI's mac_pre_init. rtw89_mac_fwdl_preconfig is NULL for AX
    // (mac.c:7516).
    dmac::pre_init(io).map_err(at("mac pre-init"))?;
    // SAFETY: the caller's `pci::pre_init` contract for these rings.
    unsafe { pci::pre_init(io, cut, rings) }.map_err(at("pci pre-init"))?;
    // SAFETY: the caller's contracts; `pci::pre_init` just programmed
    // `rings` through this `io`.
    unsafe { download(io, cut, rings, image) }?;

    // rtw89_parse_efuse_map_ax (efuse.c:281): the DDV physical map from 0,
    // its logical map, then rtw8852c_read_efuse's MAC (rtw8852c.c:648). The
    // R_AX_SYS_WL_EFUSE_CTRL autoload check only warns and is not ported.
    let mut physical = vec![0u8; efuse::PHYSICAL_EFUSE_SIZE];
    efuse::read_physical(io, 0, &mut physical).map_err(at("efuse"))?;
    let logical = efuse::logical_map(&physical).map_err(at("efuse"))?;
    Ok(Running {
        cut,
        firmware: image.version(),
        mac: efuse::mac_address(&logical),
        secure_boot: false,
    })
}

/// `rtw89_fw_download(RTW89_FW_NORMAL)` (`fw.c:2043`): up to five attempts
/// of `__rtw89_fw_download`, the last error returned. A failed CH12 submit
/// stops the queue (Linux's queue has no such state), so the next attempt
/// restarts it with `pci::pre_init` first.
///
/// # Safety
/// The `pci::pre_init` and `Ch12::submit` contracts for `io` and `rings`.
unsafe fn download<I: AxIo + ?Sized>(
    io: &mut I,
    cut: u8,
    rings: &mut Rings,
    image: &AxImage<'_>,
) -> Result<(), BringUpError> {
    let mut last = AxError::Firmware("not attempted");
    for attempt in 0..FWDL_TRIES {
        if attempt > 0 && rings.ch12_stopped() {
            // SAFETY: the caller's `pci::pre_init` contract for these rings.
            unsafe { pci::pre_init(io, cut, rings) }.map_err(at("pci pre-init"))?;
        }
        // SAFETY: the caller's `Ch12::submit` contract; `pci::pre_init`
        // programmed `rings` through this `io`.
        match unsafe { fwdl::download(io, &mut rings.ch12(), image) } {
            Ok(()) => return Ok(()),
            Err(error) => last = error,
        }
    }
    Err(at("firmware download")(last))
}

#[cfg(test)]
mod tests;

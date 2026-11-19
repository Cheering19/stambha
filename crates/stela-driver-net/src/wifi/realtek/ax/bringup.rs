// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE bring-up to running firmware, in the order and with the failure
//! handling of `rtw89_chip_info_setup` (`core.c:7383`): `rtw89_mac_pwr_on`
//! (`mac.c:1588`) with the tail of `rtw89_mac_power_switch(true)`
//! (`mac.c:1521`: `efuse_read_fw_secure`, then the `MAC_AX_NOTIFY_TP_MAJOR`
//! scoreboard), then `rtw89_chip_efuse_info_setup` (`core.c:7274`), i.e.
//! `rtw89_mac_partial_init` (`mac.c:4307`: MAC pre-init, PCI pre-init,
//! `rtw89_fw_download` with its 5-try loop, `fw.c:2043`) and
//! `rtw89_parse_efuse_map_ax` (`efuse.c:281`) for the MAC address, RFE
//! type and crystal cap, then `rtw89_parse_phycap_map_ax` (`efuse.c:350`)
//! for the trims. From there it continues, on the same power-on and
//! download (spec S1.1), with the rest of `rtw89_mac_init` (`mac.c:4360`):
//! `rtw89_chip_enable_bb_rf` (`rtw8852c_mac_enable_bb_rf`,
//! `rtw8852c.c:2999`), `sys_init_ax` (`mac.c:1701`), `trx_init_ax`
//! (`mac.c:4029`) and `rtw89_pci_ops_mac_post_init_ax` (`pci.c:3238`); then
//! it sends one H2C, `rtw89_fw_h2c_set_ofld_cfg` (`fw.c:5320`), and waits for
//! the firmware's acknowledgement on RXQ. Then `rtw89_core_start` goes on
//! (`core.c:6687-6695`) with the radio steps of [`phy::init_radio`]: the
//! BB/RF reset, the firmware's BB, BB gain and radio tables with the RF_REG
//! H2Cs, and the coex init's Wi-Fi grant. Once the post-init has run, the
//! HCI is torn down as `rtw89_core_stop` does (`core.c:6765-6767`):
//! `rtw89_pci_ops_deinit` (`pci.c:3053`), then the power-off. Last comes
//! `rtw89_mac_pwr_off` (`mac.c:1603`), which Linux runs on success and on
//! every failure after power-on.
//!
//! This module does not grant a usable radio: the chip is powered off again
//! before `bring_up` returns, and nothing past the coex init is ported: no
//! `rtw89_phy_dm_init`, no calibration, no channel, station, scan or
//! interrupts. It does not support secure-boot chips, and it does not claim
//! the MAC address read from the efuse is valid.
use super::command::{self, CommandError, H2cAck};
use super::efuse::{self, PhyEfuse, SecureBoot};
use super::elements::PhyTables;
use super::h2c::H2c;
use super::io::{AxError, AxIo};
use super::pci::Rings;
use super::phy::{self, PhyReport};
use super::radio::RfAt;
use super::{dmac, fwdl, pci, power, sys, trx};
use crate::bus::dma::{mapping::DmaDomain, memory::PageSource};
use crate::wifi::realtek::firmware::AxImage;
use alloc::{sync::Arc, vec};
use core::fmt;

/// `R_AX_SCOREBOARD` (`reg.h:156`), the 8852C `btc_sb` register
/// (`rtw8852c.c:3287`); `rtw89_mac_update_scoreboard` writes byte 3.
const R_AX_SCOREBOARD: u32 = 0x00AC;
/// `MAC_AX_NOTIFY_TP_MAJOR` (`reg.h:161`); `power::power_switch_off` writes
/// `MAC_AX_NOTIFY_PWR_MAJOR`.
const MAC_AX_NOTIFY_TP_MAJOR: u8 = 0x81;
/// `rtw89_fw_download`: `for (retry = 0; retry < 5; retry++)` (`fw.c:2048`).
const FWDL_TRIES: u8 = 5;
/// `rtw89_mac_pwr_on` (`mac.c:1588`): one power switch and one retry.
const POWER_ON_TRIES: u8 = 2;

/// What the bring-up needs to know of the PCIe host it runs under.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Host {
    /// The function's PCIe DEVCTL2 `LTR_EN` (bit 10 of capability + 0x28),
    /// which decides whether `rtw89_pci_ltr_set_v1` programs the chip's LTR
    /// (`rtw89_pci_dev_ltr_enabled`, `pci.c:3042`). Linux reads it at
    /// post-init; the probe reads it while preparing the function, and
    /// nothing in Stela writes DEVCTL2 between the two (spec S1.8). Stela
    /// reads what firmware left there. Linux's PCI core may itself set
    /// `LTR_EN` at enumeration (`pci_configure_ltr`), so Linux can see true
    /// on a machine where Stela reads false.
    pub ltr: bool,
}

/// An RTL8852CE whose firmware reported `RTW89_FWDL_WCPU_FW_INIT_RDY`, whose
/// MAC was initialized, and whose firmware acknowledged an H2C.
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
    /// The MAC init (BB/RF enable through PCI post-init) completed. Always
    /// true from [`bring_up`], kept for the line it prints.
    pub mac_init: bool,
    /// The post-init programmed the chip's LTR ([`Host::ltr`]).
    pub ltr: bool,
    /// The CMAC's BA CAM reset (`rst_bacam`, `mac.c:2904`) never completed;
    /// Linux only warns of it ("failed to reset BA CAM") and carries on.
    pub bacam_reset_timed_out: bool,
    /// What the firmware answered to OFLD_CFG: from [`bring_up`], a REC_ACK,
    /// a DONE_ACK with return 0, or both.
    pub h2c: H2cAck,
    /// What the radio steps after OFLD_CFG applied and read back: always
    /// `Some` from [`bring_up`].
    pub phy: Option<PhyReport>,
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
        )?;
        // No H2C is sent before the MAC init, so without it there is no
        // answer to report.
        if !self.mac_init {
            return Ok(());
        }
        write!(
            f,
            ", MAC init done, LTR {}, H2C OFLD_CFG ",
            if self.ltr { "on" } else { "off" }
        )?;
        f.write_str(match (self.h2c.rec, self.h2c.done) {
            (true, true) => "acknowledged (receive and done)",
            (true, false) => "acknowledged (receive only)",
            (false, true) => "acknowledged (done only)",
            (false, false) => "not acknowledged",
        })?;
        // What else RXQ delivered, named only when something did.
        for (count, what) in [
            (self.h2c.other_c2h, "other C2H"),
            (self.h2c.unmatched_ack, "unmatched acks"),
            (self.h2c.skipped, "skipped"),
        ] {
            if count != 0 {
                write!(f, ", {what} {count}")?;
            }
        }
        if let Some(log) = &self.h2c.log {
            write!(f, ", firmware log \"{}\"", log.as_str())?;
        }
        if self.bacam_reset_timed_out {
            f.write_str(", BA CAM reset timed out")?;
        }
        if let Some(phy) = &self.phy {
            write!(f, "{phy}")?;
        }
        Ok(())
    }
}

/// The step that failed and why.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BringUpError {
    /// One of `"phy tables"`, `"power on"`, `"secure boot"`, `"dma rings"`,
    /// `"mac pre-init"`, `"pci pre-init"`, `"firmware download"`, `"efuse"`,
    /// `"phycap"`, `"bb rf enable"`, `"sys init"`, `"dmac init"`,
    /// `"cmac init"`, `"imr"`, `"pci post-init"`, `"h2c"`, `"c2h"`,
    /// `"bb rf reset"`, `"bb init"`, `"rf init"`, `"rf h2c"`, `"coex init"`,
    /// `"phy report"`; the probe that opens the register window, stages the
    /// firmware, checks its PHY tables (`"phy tables"`) and runs [`quiesce`]
    /// before [`bring_up`] adds `"register window"`, `"identity"`,
    /// `"firmware"` and `"quiesce"`.
    pub step: &'static str,
    /// The step's last failure.
    pub error: AxError,
    /// What the step's retries ran into before `error`, kept because the
    /// first failure is the likelier root cause.
    pub earlier: Earlier,
    /// For `"rf init"` and `"rf h2c"`, the RF path, and for a page its
    /// index, the step failed on; printed between the step and the error.
    pub rf: Option<RfAt>,
}

/// What a retried bring-up step ran into before the error it reports.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Earlier {
    /// The step was not retried, or its first attempt failed the same way.
    #[default]
    Nothing,
    /// The first of `tries` attempts failed with `error`, which the last
    /// attempt's error differs from.
    First { tries: u8, error: AxError },
    /// A `"pci pre-init"` re-run inside the firmware download: download
    /// attempt `attempt` (from 1) failed with `stopped`, which left CH12
    /// stopped, so the next attempt re-ran `pci::pre_init` first. `first`
    /// is attempt 1's error, when that was an earlier, different one.
    Ch12Stopped {
        attempt: u8,
        stopped: AxError,
        first: Option<AxError>,
    },
}

impl Earlier {
    /// `First`, when `first` differs from `last`.
    fn first_of(tries: u8, first: AxError, last: AxError) -> Self {
        if first == last {
            Self::Nothing
        } else {
            Self::First {
                tries,
                error: first,
            }
        }
    }
}

impl BringUpError {
    /// `step` failed with `error`, with nothing earlier.
    pub const fn new(step: &'static str, error: AxError) -> Self {
        Self {
            step,
            error,
            earlier: Earlier::Nothing,
            rf: None,
        }
    }
}

impl fmt::Display for BringUpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "RTL8852CE bring-up: {}: ", self.step)?;
        if let Some(rf) = self.rf {
            write!(f, "{rf}: ")?;
        }
        write!(f, "{}", self.error)?;
        match self.earlier {
            Earlier::Nothing => Ok(()),
            Earlier::First { tries, error } => write!(f, " (first of {tries} attempts: {error})"),
            Earlier::Ch12Stopped {
                attempt,
                stopped,
                first,
            } => {
                write!(
                    f,
                    " (re-run after firmware download attempt {attempt} of {FWDL_TRIES} failed \
                     and stopped CH12: {stopped}"
                )?;
                if let Some(first) = first {
                    write!(f, "; first of {FWDL_TRIES} attempts: {first}")?;
                }
                f.write_str(")")
            }
        }
    }
}

fn at(step: &'static str) -> impl FnOnce(AxError) -> BringUpError {
    move |error| BringUpError::new(step, error)
}

/// Bring an RTL8852CE of chip cut `cut` up to running firmware `image`, read
/// its MAC address, initialize its MAC for `host`, have its firmware
/// acknowledge one H2C, configure its BB and RF from `tables` and grant its
/// RF to Wi-Fi, then power it off again.
///
/// First `tables` are checked ([`phy::check_tables`]); a file without the
/// tables the radio steps apply is refused as `"phy tables"` before any I/O.
/// Then, in Linux's order:
///
/// 1. `"power on"`: `power::power_on`, with Linux's own power-off and
///    single retry (`rtw89_mac_pwr_on`). When the retry fails too, the error
///    keeps both tries' failures.
/// 2. `"secure boot"`: the efuse secure-boot bytes, then the
///    `MAC_AX_NOTIFY_TP_MAJOR` scoreboard write that closes
///    `rtw89_mac_power_switch(true)`. A secure-boot chip is refused with
///    `SecureBootUnsupported`.
/// 3. `"dma rings"`: `Rings::install`.
/// 4. `"mac pre-init"`: `dmac::pre_init`.
/// 5. `"pci pre-init"`: `pci::pre_init`.
/// 6. `"firmware download"`: up to five `fwdl::download` attempts
///    (`fw.c:2043`), the last error returned, with the first when they
///    differ. An attempt whose CH12 submit failed leaves CH12 stopped; the
///    next attempt then runs `pci::pre_init` again first (a failure there is
///    reported as `"pci pre-init"`, with the download error that stopped
///    CH12 and, when earlier and different, the first one).
/// 7. `"efuse"`: the whole physical map, its logical map and the MAC.
///    `"phycap"`: the physical phycap window (`efuse::phycap`), which the
///    1216-byte map does not reach; with the logical map it gives the RFE
///    type, crystal cap and trims (`efuse::phy_efuse`, `"efuse"` if a map
///    is short).
///
/// Then, on the same power-on and download (spec S1.1), the rest of
/// `rtw89_mac_init` and the first H2C:
///
/// 8. `"bb rf enable"`: `sys::enable_bb_rf`.
/// 9. `"sys init"`: `sys::sys_init`.
/// 10. `"dmac init"`, `"cmac init"`, `"imr"`: `trx::init`, which names the
///     step that failed.
/// 11. `"pci post-init"`: `pci::post_init`, with LTR only when `host.ltr`.
/// 12. `"h2c"`, `"c2h"`: `command::ofld_cfg` at the sequence state a
///     download leaves (seq 0): a failed submit or a DONE_ACK with a
///     non-zero return is `"h2c"`; no matching ack within 100 ms, or a
///     malformed RXQ, is `"c2h"`. Neither failure has a Linux counterpart:
///     Linux ignores `rtw89_fw_h2c_set_ofld_cfg`'s result (`mac.c:4394`)
///     and never waits for its answer (spec S1.9).
///
/// Then the rest of `rtw89_core_start` through the coex init
/// (`core.c:6687-6695`), [`phy::init_radio`] at RFE type and crystal cap
/// from step 7 and cut `cut`, with the H2C sequence state OFLD_CFG left
/// (seq 1), so the RF_REG pages carry seq 1 and 2 and ask for no ack:
///
/// 13. `"bb rf reset"`, `"bb init"`, `"rf init"` / `"rf h2c"`,
///     `"coex init"`, `"phy report"`, in that order; an RF init failure
///     names its path and page ([`BringUpError::rf`]).
///
/// Once step 11 has succeeded (on success, and on every failure from step
/// 12 on) the HCI is torn down as `rtw89_core_stop` does between
/// `rtw89_hci_stop` and `rtw89_mac_pwr_off` (`core.c:6765-6767`):
/// `pci::deinit`, Linux's `rtw89_pci_ops_deinit` (`pci.c:3053`), which
/// clears PCI wake, turns LTR off when `host.ltr`, stops the AXI master and
/// HCI DMA, and clears every ring index. Its result is ignored, as Linux
/// ignores it. A failure at or before step 11 skips it, as Linux's
/// `rtw89_mac_init` failure path does (`mac.c:4397`).
///
/// A failure from step 4 through step 11 first runs `pci::stop_dma`:
/// `B_AX_STOP_WPDMA`, the AXI master stopped and the TX/RX HCI DMA
/// disabled, the start of `rtw89_pci_ops_mac_pre_init_ax`. Step 4 already
/// enables the HCI DMA (`rtw89_mac_dmac_func_pre_en_ax`, `mac.c:4222`, sets
/// `TXHCI_EN`/`RXHCI_EN` and clears `STOP_AXI_MST` and the channel stops),
/// so this keeps it from running across the power-off. It is Stela's own
/// (Linux powers off straight away), and its result is ignored. A failure in
/// steps 1-3 has started no DMA and powers off alone.
///
/// Success and every failure then end as `rtw89_chip_info_setup` does after
/// a power-on, and as `rtw89_mac_init`'s failure path does, in
/// `rtw89_mac_pwr_off`: `power::power_off`, then, only if it succeeded, the
/// `MAC_AX_NOTIFY_PWR_MAJOR` scoreboard write. Linux ignores the result of
/// that power-off, and so does this. A failed `"power on"` ends the same
/// way, so the chip is left off; Linux returns from it without, but its
/// retry ran the same power-off between the two tries.
///
/// On success the result is reported with the `Rings` that DMA was started
/// on. The caller must keep them (dropping them leaves every mapping
/// quarantined, never returned): the power-off does not prove the engine
/// stopped reading or writing them (the deinit stops the HCI DMA but waits
/// for nothing, as Linux does not). On a failure after the rings were
/// programmed, they are dropped after the power-off and stay quarantined the
/// same way.
///
/// # Safety
/// The contracts of `Rings::install`, `pci::pre_init`, `Ch12::submit` and
/// `Rxq::poll`: `io` is the exclusive register window of exactly the
/// RTL8852C PCI requester that `domain` isolates; `source` yields real
/// coherent WB DMA RAM for that requester, with NoSnoop and ATS disabled;
/// one worker owns the device and its queues, and no other engine, IRQ or
/// reset worker touches them. `host` carries no safety obligation: a wrong
/// `ltr` only programs, or skips, the chip's own LTR registers.
pub unsafe fn bring_up<I: AxIo + ?Sized>(
    io: &mut I,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    cut: u8,
    image: &AxImage<'_>,
    host: Host,
    tables: &PhyTables<'_>,
) -> Result<(Running, Rings), BringUpError> {
    phy::check_tables(tables).map_err(at("phy tables"))?;
    if let Err(error) = power::power_on(io) {
        power_down(io);
        return Err(BringUpError {
            step: "power on",
            error: error.retry,
            earlier: Earlier::first_of(POWER_ON_TRIES, error.first, error.retry),
            rf: None,
        });
    }
    // SAFETY: the caller's `Rings::install` contract, forwarded unchanged.
    let mut rings = match unsafe { prepare(io, source, domain) } {
        Ok(rings) => rings,
        Err(error) => {
            power_down(io);
            return Err(error);
        }
    };
    // SAFETY: the caller's `pci::pre_init`, `Ch12::submit` and `Rxq::poll`
    // contracts; `rings` were installed on `domain` for the requester behind
    // `io`.
    let result = match unsafe { start(io, cut, &mut rings, image, host) } {
        Ok(started) => {
            // SAFETY: the caller's `Ch12::submit` and `Rxq::poll` contracts;
            // `start` programmed `rings` through this `io` with
            // `pci::pre_init`, and nothing since re-programmed or stopped
            // them.
            let acked = unsafe { acknowledge(io, &mut rings, started, host, tables) };
            // rtw89_core_stop (core.c:6765-6767): rtw89_hci_deinit between
            // rtw89_hci_stop (interrupts and NAPI, neither ported) and
            // rtw89_mac_pwr_off. Linux ignores its result, and so does this.
            let _ = pci::deinit(io, &rings, host.ltr);
            acked
        }
        Err(error) => {
            // From "mac pre-init" on the HCI DMA is enabled
            // (rtw89_mac_dmac_func_pre_en_ax): stop it before the power-off.
            // Stela's own; its result is ignored like the power-off's.
            let _ = pci::stop_dma(io);
            Err(error)
        }
    };
    power_down(io);
    // On a failure `rings` drop here, after the power-off.
    Ok((result?, rings))
}

/// `rtw89_mac_pwr_off` (`mac.c:1603`): `rtw89_mac_power_switch(false)`,
/// whose result Linux ignores. `rtw89_mac_set_vcore_reset` does nothing
/// until thermal protection set a voltage level, which nothing here does.
fn power_down<I: AxIo + ?Sized>(io: &mut I) {
    // The scoreboard write (mac.c:1580) follows only a successful
    // pwr_off_func.
    let _ = power::power_switch_off(io);
}

/// Stop an RTL8852CE that firmware or an earlier driver left running,
/// before its requester is taken over, by its MAC power state
/// (`R_AX_IC_PWR_STATE[9:8]`, `reg.h:332-334`):
/// - 0, `MAC_AX_MAC_OFF`: nothing runs; nothing is written.
/// - 1, `MAC_AX_MAC_ON`: the PCIe WPDMA is stopped and the AXI master and
///   TX/RX HCI DMA disabled ([`pci::stop_dma`], the start of
///   `rtw89_pci_ops_mac_pre_init_ax`), then `rtw89_mac_power_switch(false)`
///   runs `rtw8852c_pwr_off_func` (XTAL SI power-down commands, then
///   `B_AX_APFM_OFFMAC` set and polled clear, then the PCIe low-power
///   option), then writes the `MAC_AX_NOTIFY_PWR_MAJOR` scoreboard.
/// - 2, `MAC_AX_MAC_LPS`, firmware-managed power save with the WCPU alive
///   and its HCI DMA path live, and 3, undefined: refused with
///   `MacPowerState`, nothing written. Leaving LPS is the firmware's
///   handshake, which is not ported, so the chip is never taken as stopped.
///
/// Returns whether the MAC was on. This proves nothing about DMA the device
/// issued before, which the caller excludes (Bus Master clear).
pub fn quiesce<I: AxIo + ?Sized>(io: &mut I) -> Result<bool, AxError> {
    match power::mac_state(io)? {
        power::MAC_AX_MAC_OFF => Ok(false),
        power::MAC_AX_MAC_ON_STATE => {
            pci::stop_dma(io)?;
            power::power_switch_off(io)?;
            Ok(true)
        }
        state => {
            debug_assert!(state == power::MAC_AX_MAC_LPS || state == 3);
            Err(AxError::MacPowerState(state))
        }
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

/// What [`start`] leaves for [`acknowledge`].
struct Started {
    cut: u8,
    firmware: (u8, u8, u8, u8),
    mac: [u8; 6],
    /// `cmac::init`'s `rst_bacam` timed out.
    bacam_reset_timed_out: bool,
    /// The H2C sequence state after the download.
    h2c: H2c,
    /// The efuse RFE type, crystal cap and trims.
    efuse: PhyEfuse,
}

/// `rtw89_chip_efuse_info_setup` (`core.c:7274`) through the phycap map:
/// `rtw89_mac_partial_init` (`mac.c:4307`), then the efuse map, then the
/// phycap window; then the rest of `rtw89_mac_init` (`mac.c:4360`) through
/// the PCIe post-init.
///
/// # Safety
/// The `pci::pre_init`, `Ch12::submit` and `Rxq::poll` contracts for `io`
/// and `rings`.
unsafe fn start<I: AxIo + ?Sized>(
    io: &mut I,
    cut: u8,
    rings: &mut Rings,
    image: &AxImage<'_>,
    host: Host,
) -> Result<Started, BringUpError> {
    // rtw89_mac_ctrl_hci_dma_trx and rtw89_mac_dmac_pre_init, then the
    // HCI's mac_pre_init. rtw89_mac_fwdl_preconfig is NULL for AX
    // (mac.c:7516).
    dmac::pre_init(io).map_err(at("mac pre-init"))?;
    // SAFETY: the caller's `pci::pre_init` contract for these rings.
    unsafe { pci::pre_init(io, cut, rings) }.map_err(at("pci pre-init"))?;
    // SAFETY: the caller's contracts; `pci::pre_init` just programmed
    // `rings` through this `io`.
    unsafe { download(io, cut, rings, image) }?;
    // `h2c_seq` is zeroed by a download that reached the sections
    // (fw.c:2021), and the FWDL header does not advance it (fw.c:1657).
    let h2c = H2c::after_download();

    // rtw89_parse_efuse_map_ax (efuse.c:281): the DDV physical map from 0,
    // its logical map, then rtw8852c_read_efuse's MAC (rtw8852c.c:648). The
    // R_AX_SYS_WL_EFUSE_CTRL autoload check only warns and is not ported.
    let mut physical = vec![0u8; efuse::PHYSICAL_EFUSE_SIZE];
    efuse::read_physical(io, 0, &mut physical).map_err(at("efuse"))?;
    let logical = efuse::logical_map(&physical).map_err(at("efuse"))?;
    // rtw89_parse_phycap_map_ax (efuse.c:350-381), which fails the setup as
    // the efuse map does (core.c:7297-7299): its own DDV dump of physical
    // 0x590..0x5F0, then rtw8852c_read_phycap's trims; rtw8852c_read_efuse's
    // RFE type and crystal cap come from the logical map.
    let window = efuse::phycap(io).map_err(at("phycap"))?;
    let phy_efuse = efuse::phy_efuse(&logical, &window).map_err(at("efuse"))?;

    // rtw89_mac_init (mac.c:4360) after rtw89_mac_partial_init:
    // rtw89_chip_enable_bb_rf, sys_init_ax, trx_init_ax, then the HCI's
    // mac_post_init. rtw89_mac_feat_init and rtw89_fw_send_all_early_h2c do
    // nothing for an RTL8852C on PCIe (spec S1.2-S1.3).
    sys::enable_bb_rf(io).map_err(at("bb rf enable"))?;
    sys::sys_init(io).map_err(at("sys init"))?;
    let bacam_reset_timed_out =
        trx::init(io).map_err(|error| BringUpError::new(error.step, error.error))?;
    pci::post_init(io, host.ltr).map_err(at("pci post-init"))?;
    Ok(Started {
        cut,
        firmware: image.version(),
        mac: efuse::mac_address(&logical),
        bacam_reset_timed_out,
        h2c,
        efuse: phy_efuse,
    })
}

/// `rtw89_fw_h2c_set_ofld_cfg` (`mac.c:4394`), the last step of
/// `rtw89_mac_init`, with Stela's own wait for its acknowledgement; then
/// the radio steps of `rtw89_core_start` through the coex init
/// (`core.c:6687-6695`, [`phy::init_radio`]) on the same H2C sequence.
///
/// # Safety
/// The `Ch12::submit` and `Rxq::poll` contracts for `io` and `rings`.
unsafe fn acknowledge<I: AxIo + ?Sized>(
    io: &mut I,
    rings: &mut Rings,
    started: Started,
    host: Host,
    tables: &PhyTables<'_>,
) -> Result<Running, BringUpError> {
    let mut h2c = started.h2c;
    // SAFETY: the caller's `Ch12::submit` and `Rxq::poll` contracts.
    let ack = unsafe { command::ofld_cfg(io, rings, &mut h2c) }.map_err(|error| match error {
        CommandError::Submit(error) => BringUpError::new("h2c", error),
        CommandError::Wait(error) => BringUpError::new("c2h", error),
    })?;
    // `h2c` is now past OFLD_CFG (seq 1), the state init_rf_reg expects:
    // its pages go out at seq 1 and 2, where no REC_ACK is forced.
    // SAFETY: the caller's `Ch12::submit` and `Rxq::poll` contracts, which
    // are `init_radio`'s; the OFLD_CFG wait just polled the same rings.
    let (phy, _gain) =
        unsafe { phy::init_radio(io, rings, &mut h2c, tables, &started.efuse, started.cut) }
            .map_err(|error| BringUpError {
                rf: error.rf,
                ..BringUpError::new(error.step, error.error)
            })?;
    // The BB gain table is memory only on AX (bb.rs), and nothing after the
    // coex init that reads it is ported yet.
    Ok(Running {
        cut: started.cut,
        firmware: started.firmware,
        mac: started.mac,
        secure_boot: false,
        mac_init: true,
        ltr: host.ltr,
        bacam_reset_timed_out: started.bacam_reset_timed_out,
        h2c: ack,
        phy: Some(phy),
    })
}

/// `rtw89_fw_download(RTW89_FW_NORMAL)` (`fw.c:2043`): up to five attempts
/// of `__rtw89_fw_download`, the last error returned, with the first when it
/// differs. A failed CH12 submit stops the queue (Linux's queue has no such
/// state), so the next attempt restarts it with `pci::pre_init` first; if
/// that fails, it is reported with the download error that stopped CH12.
///
/// # Safety
/// The `pci::pre_init` and `Ch12::submit` contracts for `io` and `rings`.
unsafe fn download<I: AxIo + ?Sized>(
    io: &mut I,
    cut: u8,
    rings: &mut Rings,
    image: &AxImage<'_>,
) -> Result<(), BringUpError> {
    // SAFETY: the caller's `Ch12::submit` contract; `pci::pre_init`
    // programmed `rings` through this `io`.
    let first = match unsafe { fwdl::download(io, &mut rings.ch12(), image) } {
        Ok(()) => return Ok(()),
        Err(error) => error,
    };
    let mut last = first;
    for attempt in 2..=FWDL_TRIES {
        if rings.ch12_stopped() {
            // SAFETY: the caller's `pci::pre_init` contract for these rings.
            if let Err(error) = unsafe { pci::pre_init(io, cut, rings) } {
                return Err(BringUpError {
                    step: "pci pre-init",
                    error,
                    earlier: Earlier::Ch12Stopped {
                        attempt: attempt - 1,
                        stopped: last,
                        first: (attempt > 2 && first != last).then_some(first),
                    },
                    rf: None,
                });
            }
        }
        // SAFETY: as for the first attempt; a stopped CH12 was restarted by
        // `pci::pre_init` on these rings just above.
        match unsafe { fwdl::download(io, &mut rings.ch12(), image) } {
            Ok(()) => return Ok(()),
            Err(error) => last = error,
        }
    }
    Err(BringUpError {
        step: "firmware download",
        error: last,
        earlier: Earlier::first_of(FWDL_TRIES, first, last),
        rf: None,
    })
}

#[cfg(test)]
mod tests;

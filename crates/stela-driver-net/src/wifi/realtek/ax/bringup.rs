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
//! for the trims, then `rtw89_fw_recognize_elements` (`core.c:7413`) for the
//! firmware's TX power tables of that RFE type, with the regulatory index
//! of `rtw89_regd_setup`. From there it continues, on the same power-on and
//! download (spec S1.1), with the rest of `rtw89_mac_init` (`mac.c:4360`):
//! `rtw89_chip_enable_bb_rf` (`rtw8852c_mac_enable_bb_rf`,
//! `rtw8852c.c:2999`), `sys_init_ax` (`mac.c:1701`), `trx_init_ax`
//! (`mac.c:4029`) and `rtw89_pci_ops_mac_post_init_ax` (`pci.c:3238`); then
//! it sends one H2C, `rtw89_fw_h2c_set_ofld_cfg` (`fw.c:5320`), and waits for
//! the firmware's acknowledgement on RXQ. Then `rtw89_core_start` goes on
//! (`core.c:6687-6697`) with the radio steps of [`phy::init_radio`]: the
//! BB/RF reset, the firmware's BB, BB gain and radio tables with the RF_REG
//! H2Cs, and the coex init's Wi-Fi grant; then `rtw89_phy_dm_init` with
//! NCTL, RCK, DACK and RX DCK ([`phy::calibrate`]). [`power_up`] is these
//! steps, which the live chip ([`live::start`]) continues from with
//! `rtw89_leave_ips` (the rest of `rtw89_core_start`, `rtw89_set_channel`
//! to 2.4 GHz channel 1 with its TX power, `rtw89_mac_vif_init`). Then, as
//! `rtw89_ops_hw_scan` runs from idle (`mac80211.c:1279-1320`),
//! [`bring_up`] runs the first passive scan ([`scan::run_once`], M2c spec
//! S2): the firmware's scan offload over 38 channels, whose networks the
//! result reports. After the coex init the coex half of `rtw89_core_stop`
//! (`core.c:6738, 6760`) hands the antenna back; once the post-init has run,
//! the HCI is torn down as `rtw89_core_stop` does (`core.c:6765-6767`):
//! `rtw89_pci_ops_deinit` (`pci.c:3053`), then the power-off
//! ([`shut_down`]). Last comes `rtw89_mac_pwr_off` (`mac.c:1603`), which
//! Linux runs on success and on every failure after power-on.
//!
//! [`bring_up`] does not grant a usable radio: the chip is powered off again
//! before it returns. It sets no channel but the scan's boot channel and
//! runs no IQK, DPK, TSSI, association, transmission request or
//! interrupts; the scan is passive, and `rtw89_hw_scan_complete`'s restores
//! are left to the power-off. This module does not support secure-boot
//! chips, and it does not claim the MAC address read from the efuse is
//! valid.
use super::bb::BbGain;
use super::command::{self, CommandError, H2cAck};
use super::efuse::{self, PhyEfuse, RxGain, SecureBoot};
use super::elements::{self, PhyTables, RfPath};
use super::h2c::H2c;
use super::io::{AxError, AxIo};
use super::live::{self, StartError};
use super::pci::Rings;
use super::phy::{self, PhyReport, RfkReport};
use super::radio::RfAt;
use super::regd::{self, Regds};
use super::rf::{RFREG_MASK, read_rf};
use super::scan::{self, ScanReport};
use super::txpwr_tables::{self, TxPowerTables};
use super::{coex, dmac, fwdl, pci, power, sys, trx};
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
/// RF 0x18, `RR_CFGCH`, which `rtw89_phy_read_rf_v1` reads over SWSI (the
/// DAV register), and `RR_CFGCH_V1` (address bit 16 set), which it reads
/// through the path's direct BB window (the DDV register, MMIO 0x1E060 for
/// path A and 0x1F060 for B) (`reg.h:8581-8582`, `phy.c:983-997`).
const RR_CFGCH: u32 = 0x18;
const RR_CFGCH_V1: u32 = 0x10018;

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

/// The firmware an RTL8852CE is brought up with: `image`, the normal
/// firmware the download sends, and `file`, the whole
/// `rtw89/rtw8852c_fw-2.bin` it came from, whose elements after the MFW
/// container carry the PHY tables ([`elements::phy_tables`]) and the TX
/// power and regulatory tables ([`txpwr_tables::tx_power_tables`]), both
/// read from it as `rtw89_fw_recognize_elements` reads them (`fw.c:1560`).
#[derive(Clone, Copy)]
pub struct FirmwareFile<'a> {
    pub image: &'a AxImage<'a>,
    pub file: &'a [u8],
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
    /// What `rtw89_phy_dm_init` calibrated and read back, and which of its
    /// polls timed out: always `Some` from [`bring_up`].
    pub rfk: Option<RfkReport>,
    /// RF 0x18 of each path through both of its windows, read right after
    /// the radio's report: always `Some` from [`bring_up`]. It is printed on
    /// lines of its own ([`Rf18::lines`]), not on this one.
    pub rf18: Option<Rf18>,
    /// What the first passive scan heard: always `Some` from [`bring_up`],
    /// `None` from a live chip's [`live::Live::running`], which has not
    /// scanned. Its networks print on lines of their own
    /// ([`ScanReport::listed`]).
    pub scan: Option<ScanReport>,
}

/// RF 0x18 (`RR_CFGCH`: channel, band and bandwidth) of path A, then B,
/// read through both of its windows, which are separate registers that
/// `rtw8852c_set_channel_rf` reads and rewrites each from its own value
/// (`_ch_setting`, `_bw_setting`, `is_dav`, `rtw8852c_rfk.c:3822-3933`).
/// Stela's own reads, after the radio's report and before the calibration:
/// M2b's line printed only the SWSI one, and on hardware its path B may
/// have read 0x01000 where 0x01001 was expected (the photo was unclear).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rf18 {
    /// `read_rf(path, 0x18)`: over SWSI (DAV).
    pub dav: [u32; 2],
    /// `read_rf(path, 0x10018)`: the direct window (DDV).
    pub ddv: [u32; 2],
}

/// One path of an [`Rf18`], printed as `WIFI RF18 <path>: DAV 0x18 over
/// SWSI 0x<v>, DDV 0x10018 direct 0x<v>`, each value as five lower-case hex
/// digits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rf18Line {
    pub path: char,
    pub dav: u32,
    pub ddv: u32,
}

impl Rf18 {
    /// Path A's line, then path B's.
    pub fn lines(self) -> impl Iterator<Item = Rf18Line> {
        let Self { dav, ddv } = self;
        [('A', 0), ('B', 1)]
            .into_iter()
            .map(move |(path, i)| Rf18Line {
                path,
                dav: dav[i],
                ddv: ddv[i],
            })
    }
}

impl fmt::Display for Rf18Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "WIFI RF18 {}: DAV 0x18 over SWSI 0x{:05x}, DDV 0x10018 direct 0x{:05x}",
            self.path, self.dav, self.ddv
        )
    }
}

/// [`Rf18`]: path A's DAV then DDV, then path B's. The DAV reads repeat the
/// SWSI reads of [`PhyReport::rf18`] on purpose, a few register accesses
/// later: each window is paired with a reading of the other taken back to
/// back, and two SWSI reads that disagree show the serial interface itself
/// misreading.
fn read_rf18<I: AxIo + ?Sized>(io: &mut I) -> Result<Rf18, AxError> {
    let mut rf18 = Rf18 {
        dav: [0; 2],
        ddv: [0; 2],
    };
    for (i, path) in [RfPath::A, RfPath::B].into_iter().enumerate() {
        rf18.dav[i] = read_rf(io, path, RR_CFGCH, RFREG_MASK)?;
        rf18.ddv[i] = read_rf(io, path, RR_CFGCH_V1, RFREG_MASK)?;
    }
    Ok(rf18)
}

impl Running {
    /// The `WIFI RF18` lines of [`Running::rf18`], path A then B.
    pub fn rf18_lines(&self) -> impl Iterator<Item = Rf18Line> {
        self.rf18.into_iter().flat_map(|rf18| rf18.lines())
    }
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
        if let Some(rfk) = &self.rfk {
            write!(f, "{rfk}")?;
        }
        if let Some(scan) = &self.scan {
            write!(f, "{scan}")?;
        }
        Ok(())
    }
}

/// The step that failed and why.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BringUpError {
    /// One of `"phy tables"`, `"power on"`, `"secure boot"`, `"dma rings"`,
    /// `"mac pre-init"`, `"pci pre-init"`, `"firmware download"`, `"efuse"`,
    /// `"phycap"`, `"tx power"` (the file's TX power tables),
    /// `"bb rf enable"`, `"sys init"`, `"dmac init"`, `"cmac init"`,
    /// `"imr"`, `"pci post-init"`, `"h2c"`, `"c2h"`, `"bb rf reset"`,
    /// `"bb init"`, `"rf init"`, `"rf h2c"`, `"coex init"`, `"phy report"`,
    /// `"dm init"`, `"nctl init"`, `"rfk init"`, `"dm tail"`,
    /// `"core start"`, `"set channel"`, `"tx power"` (its writes),
    /// `"vif init"`, `"tx rings"`, `"scan start"`, `"scan"`; the probe that
    /// opens the register window,
    /// stages the firmware, checks its PHY tables (`"phy tables"`) and runs
    /// [`quiesce`] before [`bring_up`] adds `"register window"`,
    /// `"identity"`, `"firmware"` and `"quiesce"`.
    pub step: &'static str,
    /// The step's last failure.
    pub error: AxError,
    /// What the step's retries ran into before `error`, kept because the
    /// first failure is the likelier root cause.
    pub earlier: Earlier,
    /// For `"rf init"` and `"rf h2c"`, the RF path, and for a page its
    /// index, the step failed on; printed between the step and the error.
    pub rf: Option<RfAt>,
    /// RF 0x18 through both windows, when the failure came after it was read
    /// (any calibration or scan step): as [`Running::rf18`], printed on
    /// lines of its own ([`BringUpError::rf18_lines`]), not on this one.
    pub rf18: Option<Rf18>,
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
            rf18: None,
        }
    }

    /// The `WIFI RF18` lines of [`BringUpError::rf18`], path A then B; none
    /// when the failure came before RF 0x18 was read.
    pub fn rf18_lines(&self) -> impl Iterator<Item = Rf18Line> {
        self.rf18.into_iter().flat_map(|rf18| rf18.lines())
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

/// Bring an RTL8852CE of chip cut `cut` up to running `firmware.image`, read
/// its MAC address, initialize its MAC for `host`, have its firmware
/// acknowledge one H2C, configure its BB and RF from `firmware.file`'s PHY
/// tables, grant its RF to Wi-Fi, calibrate it, set its boot channel with
/// the file's TX power and run its firmware's passive scan, then power it
/// off again: [`live::start`], then [`scan::run_once`], then
/// [`live::Live::stop`] (M3 spec S3.3).
///
/// First the file's PHY tables are found ([`elements::phy_tables`]) and
/// checked ([`phy::check_tables`]); a file whose elements cannot be walked,
/// or which lacks a table the radio steps apply, is refused as
/// `"phy tables"` before any I/O. Then, in Linux's order:
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
///    `"tx power"`: the file's TX power and REGD tables for that RFE type
///    ([`txpwr_tables::tx_power_tables`]), as `rtw89_chip_info_setup`
///    recognizes the firmware elements right after the efuse
///    (`core.c:7409-7417`). A file without them fails here, before the MAC
///    init and before any of its PHY tables reach the chip: Stela has no
///    built-in tables to fall back on (M3 spec S1.8), and Linux takes no
///    element once one fails (`fw.c:1604-1605`), so neither set is used
///    without the other. The regulatory index is chosen from the efuse
///    country, else the network's, else the world's (M3 spec S4.2;
///    [`regd::select`]); [`bring_up`] has no network.
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
///     names its path and page ([`BringUpError::rf`]). Stela's own reads of
///     RF 0x18 through both windows ([`Running::rf18`]) close
///     `"phy report"`.
///
/// Then `rtw89_phy_dm_init` (`core.c:6697`), [`phy::calibrate`] with the
/// same tables, efuse fields and cut:
///
/// 14. `"dm init"`, `"nctl init"`, `"rfk init"`, `"dm tail"`, in that
///     order. A calibration poll that times out does not fail its step: it
///     is reported in [`Running::rfk`], as Linux only logs it.
///
/// Then `rtw89_leave_ips` ([`live::leave_ips`], M3 spec S2), with the efuse
/// MAC, as channel inputs the BB gain table [`phy::init_radio`] decoded, the
/// efuse RX gain offsets of the logical map from step 7
/// ([`efuse::rx_gain`]), PHY 0's RPL bias from [`phy::calibrate`] and cut
/// `cut`, and as TX power the tables and regulatory index of step 7; its
/// H2Cs go on from the sequence state the RF pages left (seq 3). Then the
/// TX pools, and the first passive scan ([`scan::run_once`], M2c spec S2)
/// with the image's version:
///
/// 15. `"core start"`, `"set channel"` with `"tx power"` between its RF
///     part and its help's exit (`core.c:584-588`), `"vif init"`,
///     `"tx rings"`, `"scan start"`, `"scan"`, in that order. A scan that
///     never ends is stopped and reported in [`Running::scan`], not failed.
///
/// Once `"coex init"` has succeeded (on success, and on a failure of
/// `"phy report"` or of any step of 14 or 15, a DACK or RX DCK I/O failure
/// included; spec S1.16), the coex half of `rtw89_core_stop` runs first
/// ([`coex::radio_off`], `core.c:6738, 6760`): the grant handed to BT and
/// the scoreboard's driver bits cleared (the WLRFK bit a failed DACK leaves
/// set with them), so WL is not left marked on for BT across the power-off.
/// Its result is ignored, as Linux ignores it.
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
/// same way. The TX pools are dropped after the power-off: no TXBD ever
/// named them, so the chip never had their addresses.
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
    firmware: FirmwareFile<'_>,
    host: Host,
) -> Result<(Running, Rings), BringUpError> {
    // SAFETY: the caller's contract is `start`'s, word for word, for the
    // same window borrowed.
    let mut live = match unsafe { live::start(&mut *io, source, domain, cut, firmware, host, None) }
    {
        Ok(live) => live,
        // `start` powered the chip off; the rings it hands back drop here.
        Err(StartError { error, .. }) => return Err(error),
    };
    // SAFETY: as above; `start` returned the live chip on this window.
    let scanned = unsafe { scan::run_once(&mut live) };
    let mut running = *live.running();
    // SAFETY: as above.
    let (_, rings, pools) = unsafe { live.stop() };
    drop(pools);
    match scanned {
        Ok(report) => {
            running.scan = Some(report);
            Ok((running, rings))
        }
        // The rings drop after the power-off.
        Err(error) => Err(BringUpError {
            rf18: running.rf18,
            ..BringUpError::new(error.step, error.error)
        }),
    }
}

/// What the bring-up through `rtw89_phy_dm_init` leaves for the live chip:
/// M2b2's report (no scan), the H2C sequence state, the inputs of every
/// channel switch, and the TX power tables with their regulatory index and
/// the alpha2 it came from.
pub(super) struct Calibrated {
    pub running: Running,
    pub h2c: H2c,
    pub gain: BbGain,
    pub rx_gain: RxGain,
    pub offset_base: i8,
    pub tables: TxPowerTables,
    pub regds: (Regds, [u8; 2]),
}

/// [`bring_up`]'s steps before `rtw89_leave_ips` (its "phy tables" check
/// and steps 1-14), with the teardown each failure needs: the chip is
/// powered off on every error, and the rings, once installed, come back
/// with it for the caller to keep.
///
/// # Safety
/// [`bring_up`]'s contract.
#[expect(
    clippy::result_large_err,
    reason = "A failure hands the rings back by value: they are never freed, as nothing proves the DMA stopped."
)]
pub(super) unsafe fn power_up<I: AxIo + ?Sized>(
    io: &mut I,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    cut: u8,
    firmware: FirmwareFile<'_>,
    host: Host,
    network_country: Option<[u8; 2]>,
) -> Result<(Calibrated, Rings), (BringUpError, Option<Rings>)> {
    let tables = elements::phy_tables(firmware.file, 0).map_err(|_| {
        let unreadable = AxError::FirmwareFile("firmware elements are unreadable");
        (BringUpError::new("phy tables", unreadable), None)
    })?;
    phy::check_tables(&tables).map_err(|error| (BringUpError::new("phy tables", error), None))?;
    if let Err(error) = power::power_on(io) {
        power_down(io);
        let failed = BringUpError {
            step: "power on",
            error: error.retry,
            earlier: Earlier::first_of(POWER_ON_TRIES, error.first, error.retry),
            rf: None,
            rf18: None,
        };
        return Err((failed, None));
    }
    // SAFETY: the caller's `Rings::install` contract, forwarded unchanged.
    let mut rings = match unsafe { prepare(io, source, domain) } {
        Ok(rings) => rings,
        Err(error) => {
            power_down(io);
            return Err((error, None));
        }
    };
    // SAFETY: the caller's `pci::pre_init`, `Ch12::submit` and `Rxq::poll`
    // contracts; `rings` were installed on `domain` for the requester behind
    // `io`.
    let started = match unsafe { init_mac(io, cut, &mut rings, firmware, host, network_country) } {
        Ok(started) => started,
        Err(error) => {
            // From "mac pre-init" on the HCI DMA is enabled
            // (rtw89_mac_dmac_func_pre_en_ax): stop it before the power-off.
            // Stela's own; its result is ignored like the power-off's.
            let _ = pci::stop_dma(io);
            power_down(io);
            return Err((error, Some(rings)));
        }
    };
    // SAFETY: the caller's `Ch12::submit` and `Rxq::poll` contracts;
    // `init_mac` programmed `rings` through this `io` with `pci::pre_init`,
    // and nothing since re-programmed or stopped them.
    match unsafe { acknowledge(io, &mut rings, started, host, &tables) } {
        (Ok(calibrated), _) => Ok((calibrated, rings)),
        (Err(error), coex_up) => {
            // As `shut_down`, the coex half only once the coex init ran.
            if coex_up {
                let _ = coex::radio_off(io);
            }
            let _ = pci::deinit(io, &rings, host.ltr);
            power_down(io);
            Err((error, Some(rings)))
        }
    }
}

/// `rtw89_core_stop` (`core.c:6727-6767`) of a chip whose coex init and
/// PCIe post-init ran: with the coex set up, `rtw89_btc_ntfy_radio_state
/// (WL_OFF)` and `rtw89_btc_ntfy_poweroff` (`core.c:6738, 6760`) hand the
/// antenna to BT and clear the driver's scoreboard bits before the HCI is
/// touched ([`coex::radio_off`]); then `rtw89_hci_deinit` between
/// `rtw89_hci_stop` (interrupts and NAPI, neither ported) and
/// `rtw89_mac_pwr_off` (`core.c:6765-6767`, [`pci::deinit`]); then the
/// power-off. Linux ignores every result there, and so does this.
pub(super) fn shut_down<I: AxIo + ?Sized>(io: &mut I, rings: &Rings, host: Host) {
    let _ = coex::radio_off(io);
    let _ = pci::deinit(io, rings, host.ltr);
    power_down(io);
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

/// What [`init_mac`] leaves for [`acknowledge`].
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
    /// The efuse RX gain offsets of the logical map (spec S5).
    rx_gain: RxGain,
    /// The file's TX power tables for the efuse RFE type.
    tables: TxPowerTables,
    /// Their regulatory index, and the alpha2 it was found by.
    regds: (Regds, [u8; 2]),
}

/// `rtw89_chip_efuse_info_setup` (`core.c:7274`) through the phycap map:
/// `rtw89_mac_partial_init` (`mac.c:4307`), then the efuse map, then the
/// phycap window; then `rtw89_fw_recognize_elements`' TX power tables for
/// the efuse RFE type (`core.c:7413`) and the regulatory index; then the
/// rest of `rtw89_mac_init` (`mac.c:4360`) through the PCIe post-init.
///
/// # Safety
/// The `pci::pre_init`, `Ch12::submit` and `Rxq::poll` contracts for `io`
/// and `rings`.
unsafe fn init_mac<I: AxIo + ?Sized>(
    io: &mut I,
    cut: u8,
    rings: &mut Rings,
    firmware: FirmwareFile<'_>,
    host: Host,
    network_country: Option<[u8; 2]>,
) -> Result<Started, BringUpError> {
    let image = firmware.image;
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
    // rtw89_fw_recognize_elements (core.c:7413), which rtw89_chip_info_setup
    // runs right after the efuse because the TX power elements are chosen
    // by its RFE type (fw.c:1175-1221); AX chips have aid 0. Then the
    // regulatory index: the efuse country first (rtw8852c.c:641-642,
    // regd.c:738-768), else the network's, else the world's.
    let tables = txpwr_tables::tx_power_tables(firmware.file, 0, phy_efuse.rfe_type)
        .map_err(at("tx power"))?;
    let regds = regd::select(
        tables.regd_map(),
        efuse::country_code(&logical),
        network_country,
    );

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
        // rtw8852c_efuse_parsing_gain_offset (rtw8852c.c:575-622), which
        // rtw8852c_read_efuse runs on the same logical map.
        rx_gain: efuse::rx_gain(&logical),
        tables,
        regds,
    })
}

/// `rtw89_fw_h2c_set_ofld_cfg` (`mac.c:4394`), the last step of
/// `rtw89_mac_init`, with Stela's own wait for its acknowledgement; then
/// the radio steps of `rtw89_core_start` through the coex init
/// (`core.c:6687-6695`, [`phy::init_radio`]) on the same H2C sequence and
/// `rtw89_phy_dm_init` (`core.c:6697`, [`phy::calibrate`]). Also returns
/// whether the coex init completed, which the teardown must undo
/// ([`coex::radio_off`]): on success, and on any failure from the coex
/// init's report on.
///
/// # Safety
/// The `Ch12::submit` and `Rxq::poll` contracts for `io` and `rings`, which
/// [`init_mac`] programmed through this `io` with a `pci::pre_init` that
/// succeeded.
unsafe fn acknowledge<I: AxIo + ?Sized>(
    io: &mut I,
    rings: &mut Rings,
    started: Started,
    host: Host,
    tables: &PhyTables<'_>,
) -> (Result<Calibrated, BringUpError>, bool) {
    let mut h2c = started.h2c;
    // SAFETY: the caller's `Ch12::submit` and `Rxq::poll` contracts.
    let ack = match unsafe { command::ofld_cfg(io, rings, &mut h2c) } {
        Ok(ack) => ack,
        Err(CommandError::Submit(error)) => return (Err(BringUpError::new("h2c", error)), false),
        Err(CommandError::Wait(error)) => return (Err(BringUpError::new("c2h", error)), false),
    };
    // `h2c` is now past OFLD_CFG (seq 1), the state init_rf_reg expects:
    // its pages go out at seq 1 and 2, where no REC_ACK is forced.
    // SAFETY: the caller's `Ch12::submit` and `Rxq::poll` contracts, which
    // are `init_radio`'s; the OFLD_CFG wait just polled the same rings.
    let (phy, gain) = match unsafe {
        phy::init_radio(io, rings, &mut h2c, tables, &started.efuse, started.cut)
    } {
        Ok(done) => done,
        Err(error) => {
            let failed = BringUpError {
                rf: error.rf,
                ..BringUpError::new(error.step, error.error)
            };
            return (Err(failed), error.coex_up);
        }
    };
    // Every failure from here on follows the coex init, so the teardown
    // undoes it. Stela's own: RF 0x18 through both windows, at the point
    // M2b's report read it over SWSI alone, as part of that report.
    let rf18 = match read_rf18(io) {
        Ok(rf18) => rf18,
        Err(error) => return (Err(BringUpError::new("phy report", error)), true),
    };
    // rtw89_phy_dm_init (core.c:6697). A failure after the reads keeps
    // them: a failed run is the one most likely to need them.
    let (rfk, offset_base) = match phy::calibrate(io, tables, &started.efuse, started.cut) {
        Ok(done) => done,
        Err(error) => {
            let failed = BringUpError {
                rf18: Some(rf18),
                ..BringUpError::new(error.step, error.error)
            };
            return (Err(failed), true);
        }
    };
    let running = Running {
        cut: started.cut,
        firmware: started.firmware,
        mac: started.mac,
        secure_boot: false,
        mac_init: true,
        ltr: host.ltr,
        bacam_reset_timed_out: started.bacam_reset_timed_out,
        h2c: ack,
        phy: Some(phy),
        rfk: Some(rfk),
        rf18: Some(rf18),
        scan: None,
    };
    // The BB gain table is memory only on AX (bb.rs); it, the efuse RX gain
    // offsets, PHY 0's RPL bias (`efuse_gain.offset_base[RTW89_PHY_0]`, which
    // bb_sethw set) and the cut are every `__rtw89_set_channel`'s.
    let calibrated = Calibrated {
        running,
        h2c,
        gain,
        rx_gain: started.rx_gain,
        offset_base: offset_base[0],
        tables: started.tables,
        regds: started.regds,
    };
    (Ok(calibrated), true)
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
                    rf18: None,
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
        rf18: None,
    })
}

#[cfg(test)]
pub(crate) mod tests;

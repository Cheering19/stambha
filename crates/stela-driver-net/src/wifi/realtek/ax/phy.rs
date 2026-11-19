// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! The radio half of `rtw89_core_start` (pinned rtw89/core.c:6658) that
//! follows `rtw89_mac_init`, as far as the Wi-Fi grant, for an RTL8852C on
//! PHY 0 (core.c:6687-6695): `rtw89_chip_reset_bb_rf`
//! ([`sys::reset_bb_rf`]), `rtw89_phy_init_bb_reg` ([`bb::init_bb_reg`],
//! phy.c:1885), `rtw89_phy_init_rf_reg(false)` with its RF_REG H2Cs
//! ([`radio::init_rf_reg`], phy.c:2005) and the register end state of
//! `rtw89_btc_ntfy_init(BTC_MODE_NORMAL)` ([`coex::init`], coex.c:8723).
//! `rtw89_btc_ntfy_poweron` only counts and `bb_postinit` is NULL for the
//! RTL8852C (rtw8852c.c:3065), so neither is ported. [`init_radio`] runs
//! them in that order with the firmware's own tables, which [`check_tables`]
//! requires (spec S1.1, S3), then reads back what Stela reports of them
//! ([`PhyReport`], spec S11): RF 0x18 on both paths, the grant registers
//! (as Linux's coex dump reads them, coex.c:11931-11932) and the C2Hs that
//! arrived since the OFLD_CFG wait.
//!
//! After it, [`calibrate`] is the `rtw89_phy_dm_init` that follows the coex
//! init (core.c:6697, phy.c:8235-8261), in its order: the [`dm`] calls of
//! its first part, `rtw89_phy_init_rf_nctl` ([`bb::init_rf_nctl`]),
//! `rtw8852c_rfk_init` (rtw8852c.c:1985-1997: [`rfk::rck`], [`rfk::dack`],
//! [`rfk::rx_dck`]), then the [`dm`] tail; it returns what they read back
//! and which polls timed out ([`RfkReport`], spec S16).
//!
//! Where Linux carries on with a half-configured radio (an RFE type no
//! radio branch takes, an RF write SWSI dropped, an XTAL SI command that
//! timed out), Stela stops at the step that failed (spec S1.2-S1.3, S12.7);
//! a calibration poll that times out is only recorded, as Linux only logs
//! it. This module grants no channel, no IQK, DPK, TSSI or LCK, no tracking
//! loop, no coex H2C and no acknowledgement wait: the bring-up powers the
//! chip off after it.
use super::bb::{self, BbGain};
use super::coex;
use super::command;
use super::efuse::PhyEfuse;
use super::elements::{PhyTables, Radio, Reg2, RfPath};
use super::h2c::H2c;
use super::io::{AxError, AxIo};
use super::pci::Rings;
use super::radio::{self, RfAt};
use super::rf::{self, RFREG_MASK};
use super::{dm, rfk, sys};
use core::fmt;

/// `RR_CFGCH` (reg.h:8581), the RF register the radio tables write last on
/// both paths for every supported RFE type (spec S11.1).
const RR_CFGCH: u32 = 0x18;
/// `GNT_WL_RFC_S0`/`S1` and `GNT_BT_RFC_S0`/`S1` of `R_AX_GNT_VAL_V1`, bits
/// 1-4 (reg.h:3737-3741).
const GNT_RFC_MASK: u32 = 0x1E;

/// What the M2b1 radio steps applied and read back.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhyReport {
    /// The efuse RFE type the tables were selected by (logical 0x2CA).
    pub rfe: u8,
    /// The efuse crystal cap (logical 0x2B9), raw.
    pub xtal: u8,
    /// BB table entries written.
    pub bb: u16,
    /// BB gain table entries decoded.
    pub bb_gain: u16,
    /// Radio table entries applied, by slot: path B, then path A.
    pub rf: [u16; 2],
    /// RF register words stored and sent in RF_REG pages, by slot.
    pub rf_words: [u16; 2],
    /// RF 0x18 read back, path A then path B.
    pub rf18: [u32; 2],
    /// `R_AX_GNT_VAL_V1` (0xDA4C) after the coex init.
    pub gnt: u32,
    /// `R_AX_GNT_SW_CTRL` (0xDA48) read back after the coex init.
    pub gnt_sw: u32,
    /// `R_AX_GNT_VAL_V1` sampled before the coex init.
    pub gnt_before: u32,
    /// C2H frames RXQ held after the RF pages (Stela's own count).
    pub c2h_after: u8,
}

impl fmt::Display for PhyReport {
    /// The spec's M2b1 line (its "Done means"), then the grant reads in its style (`GNT RFC`
    /// is `GNT & 0x1E`), then the late C2Hs when there were any.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            ", PHY rfe {} xtal {:#04x}: BB {}, BB gain {}, RF B {} A {}, RF pages {}+{}, RF18 A \
             0x{:05x} B 0x{:05x}, GNT 0x{:08x}, GNT RFC 0x{:02x}, GNT SW 0x{:08x}, GNT before \
             coex 0x{:08x}",
            self.rfe,
            self.xtal,
            self.bb,
            self.bb_gain,
            self.rf[0],
            self.rf[1],
            self.rf_words[0],
            self.rf_words[1],
            self.rf18[0],
            self.rf18[1],
            self.gnt,
            self.gnt & GNT_RFC_MASK,
            self.gnt_sw,
            self.gnt_before,
        )?;
        if self.c2h_after != 0 {
            write!(f, ", C2H after RF pages {}", self.c2h_after)?;
        }
        Ok(())
    }
}

/// What `rtw89_phy_dm_init` read back and which of its polls timed out
/// (spec S16). Index 0 is path A (S0), 1 is path B (S1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RfkReport {
    /// The thermal meters as `rtw89_phy_stat_init` read them
    /// ([`dm::stat_init`]).
    pub thermal: [u8; 2],
    /// The crystal cap read back from XTAL SI XO, then XI ([`dm::cfo_init`]).
    pub xtal: (u8, u8),
    /// Each path's RCK code ([`rfk::rck`]).
    pub rck: [u8; 2],
    /// The DRCK code ([`rfk::Dack::drck`]).
    pub drck: u8,
    /// Each side's ADDCK `(A0, A1)` result ([`rfk::Dack::addck`]).
    pub addck: [(u16, u16); 2],
    /// How many RX DC offsets each path's recovery rewrote ([`rfk::rx_dck`]).
    pub rx_dck_rewrites: [u8; 2],
    /// The NCTL block poll timed out ([`bb::init_rf_nctl`]); Linux logs
    /// "failed to poll nctl block" and loads the table anyway.
    pub nctl_timed_out: bool,
    pub rck_timed_out: [bool; 2],
    pub drck_timed_out: bool,
    pub addck_timed_out: [bool; 2],
    pub dack_timed_out: [bool; 2],
    pub rx_dck_timed_out: [bool; 2],
}

impl fmt::Display for RfkReport {
    /// The spec's M2b2 line (its "Done means"), then `, timed out: ` and
    /// each timed-out poll in S16's order, only when one did.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [(ia, qa), (ib, qb)] = self.addck;
        write!(
            f,
            ", calibrated: thermal A {} B {}, xtal {:#04x}/{:#04x}, RCK A {:#04x} B {:#04x}, \
             DRCK {:#04x}, ADDCK A 0x{ia:03x}/0x{qa:03x} B 0x{ib:03x}/0x{qb:03x}, RX DCK \
             rewrites A {} B {}",
            self.thermal[0],
            self.thermal[1],
            self.xtal.0,
            self.xtal.1,
            self.rck[0],
            self.rck[1],
            self.drck,
            self.rx_dck_rewrites[0],
            self.rx_dck_rewrites[1],
        )?;
        let polls = [
            (self.nctl_timed_out, "NCTL"),
            (self.rck_timed_out[0], "RCK A"),
            (self.rck_timed_out[1], "RCK B"),
            (self.drck_timed_out, "DRCK"),
            (self.addck_timed_out[0], "ADDCK A"),
            (self.addck_timed_out[1], "ADDCK B"),
            (self.dack_timed_out[0], "DACK A"),
            (self.dack_timed_out[1], "DACK B"),
            (self.rx_dck_timed_out[0], "RX DCK A"),
            (self.rx_dck_timed_out[1], "RX DCK B"),
        ];
        let mut sep = ", timed out: ";
        for (_, name) in polls.into_iter().filter(|(timed_out, _)| *timed_out) {
            write!(f, "{sep}{name}")?;
            sep = ", ";
        }
        Ok(())
    }
}

/// The radio step that failed and why; an RF init failure also names its
/// path and page.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhyError {
    /// One of `"phy tables"`, `"bb rf reset"`, `"bb init"`, `"rf init"`,
    /// `"rf h2c"`, `"coex init"`, `"phy report"` ([`init_radio`]), or
    /// `"dm init"`, `"nctl init"`, `"rfk init"`, `"dm tail"`
    /// ([`calibrate`]).
    pub step: &'static str,
    /// What failed in that step: an I/O or poll failure, a table that
    /// selects nothing (`PhyPackage`, `PhyTable`), a missing table (`Phy`)
    /// or a CH12 submit failure. A calibration poll that times out is not
    /// one: it is recorded in [`RfkReport`].
    pub error: AxError,
    /// For `"rf init"` and `"rf h2c"`, where.
    pub rf: Option<RfAt>,
    /// [`coex::init`] had completed, so the scoreboard marks Wi-Fi on for
    /// BT and the teardown must run [`coex::radio_off`] first.
    pub coex_up: bool,
}

fn at(step: &'static str) -> impl FnOnce(AxError) -> PhyError {
    move |error| PhyError {
        step,
        error,
        rf: None,
        coex_up: false,
    }
}

/// The tables [`check_tables`] requires, unwrapped.
struct Required<'a> {
    bb: Reg2<'a>,
    bb_gain: Reg2<'a>,
    nctl: Reg2<'a>,
    radio: [Radio<'a>; 2],
}

fn required<'a>(t: &PhyTables<'a>) -> Result<Required<'a>, AxError> {
    let bb = t.bb.ok_or(AxError::Phy("firmware carries no BB table"))?;
    let bb_gain = t
        .bb_gain
        .ok_or(AxError::Phy("firmware carries no BB gain table"))?;
    let nctl = t
        .nctl
        .ok_or(AxError::Phy("firmware carries no NCTL table"))?;
    let b = t.radio[0].ok_or(AxError::Phy("firmware carries no radio table for slot 0"))?;
    let a = t.radio[1].ok_or(AxError::Phy("firmware carries no radio table for slot 1"))?;
    if b.path != RfPath::B || a.path != RfPath::A {
        return Err(AxError::Phy("firmware radio slots are not B then A"));
    }
    if t.radio[2..].iter().any(Option::is_some) {
        return Err(AxError::Phy(
            "firmware carries a radio table for a slot past 1",
        ));
    }
    Ok(Required {
        bb,
        bb_gain,
        nctl,
        radio: [b, a],
    })
}

/// Spec S3: the file carries BB, BB gain, NCTL and two radio tables, slot 0
/// for path B and slot 1 for path A, and no other radio slot. Linux would
/// fall back to `rtw8852c_table.c` for a missing one, which Stela does not
/// have (spec S1.1). A walk that stopped early is not refused when every
/// table was found, as Linux keeps what it found.
pub fn check_tables(t: &PhyTables<'_>) -> Result<(), AxError> {
    required(t).map(|_| ())
}

/// The radio steps of `rtw89_core_start` after `rtw89_mac_init`, through the
/// coex init, for RFE type `efuse.rfe_type` at cut `cv`, then Stela's
/// report reads. Steps, in Linux's order (spec S17), each stopping
/// everything after it:
///
/// 1. `"phy tables"`: [`check_tables`], before any I/O.
/// 2. `"bb rf reset"`: [`sys::reset_bb_rf`].
/// 3. `"bb init"`: [`bb::init_bb_reg`]: the BB table, the TX power unit,
///    the BB gain table, the BB reset.
/// 4. `"rf init"` / `"rf h2c"`: [`radio::init_rf_reg`], path B then A, each
///    table's stored words sent as RF_REG pages on CH12; the error names
///    the path and page.
/// 5. `"coex init"`: `R_AX_GNT_VAL_V1` sampled (Stela's own read), then
///    [`coex::init`].
/// 6. `"phy report"`: RF 0x18 of path A, then B (SWSI reads), the GNT value
///    and the GNT SW readback, then [`command::count_c2h`].
///
/// `h2c` must be the state OFLD_CFG left (seq 1): the RF pages then carry
/// seq 1 and 2 and ask for no acknowledgement ([`radio::init_rf_reg`]). On
/// success the decoded BB gain table is returned with the report; on AX it
/// is memory only, for later steps. Once [`coex::init`] has succeeded the
/// scoreboard marks Wi-Fi on for BT, so whoever stops the chip runs
/// [`coex::radio_off`] first, as `rtw89_core_stop` does: after success, and
/// after a failure whose [`PhyError::coex_up`] is set.
///
/// # Safety
/// The contracts of `Ch12::submit` and `Rxq::poll`: `io` is the exclusive
/// register window of the same RTL8852C requester that `pci::pre_init`
/// programmed with `rings`, and the `pre_init` contract still holds: its DMA
/// domain isolates that requester, the rings are coherent WB RAM with
/// NoSnoop and ATS disabled, and one worker owns the device and its queues.
pub unsafe fn init_radio<I: AxIo + ?Sized>(
    io: &mut I,
    rings: &mut Rings,
    h2c: &mut H2c,
    tables: &PhyTables<'_>,
    efuse: &PhyEfuse,
    cv: u8,
) -> Result<(PhyReport, BbGain), PhyError> {
    let t = required(tables).map_err(at("phy tables"))?;
    let rfe = efuse.rfe_type;
    sys::reset_bb_rf(io).map_err(at("bb rf reset"))?;
    let bb = bb::init_bb_reg(io, &t.bb, &t.bb_gain, rfe, cv).map_err(at("bb init"))?;
    // SAFETY: the caller's contract is `init_rf_reg`'s (`Ch12::submit`'s),
    // word for word.
    let rf =
        unsafe { radio::init_rf_reg(io, rings, h2c, t.radio, rfe, cv) }.map_err(|e| PhyError {
            step: e.step(),
            error: e.error(),
            rf: Some(e.at()),
            coex_up: false,
        })?;
    // Stela's own sample of the grant before the coex init sets it; it is
    // part of that step.
    let gnt_before = coex::gnt(io).map_err(at("coex init"))?;
    coex::init(io, rfe).map_err(at("coex init"))?;
    let report = |io: &mut I, rings: &mut Rings| -> Result<PhyReport, AxError> {
        let rf18 = [
            rf::read_rf(io, RfPath::A, RR_CFGCH, RFREG_MASK)?,
            rf::read_rf(io, RfPath::B, RR_CFGCH, RFREG_MASK)?,
        ];
        let gnt = coex::gnt(io)?;
        let gnt_sw = coex::gnt_sw(io)?;
        // SAFETY: the caller's contract is `count_c2h`'s (`Rxq::poll`'s),
        // and the OFLD_CFG wait before this polled the same RXQ, which a
        // successful `pre_init` of these rings allowed.
        let c2h_after = unsafe { command::count_c2h(io, rings) }?;
        Ok(PhyReport {
            rfe,
            xtal: efuse.xtal_cap,
            bb: bb.bb,
            bb_gain: bb.bb_gain,
            rf: rf.applied,
            rf_words: rf.words,
            rf18,
            gnt,
            gnt_sw,
            gnt_before,
            c2h_after,
        })
    };
    let report = report(io, rings).map_err(after_coex("phy report"))?;
    Ok((report, bb.gain))
}

/// A failure of a step that runs after [`coex::init`] succeeded.
fn after_coex(step: &'static str) -> impl FnOnce(AxError) -> PhyError {
    move |error| PhyError {
        coex_up: true,
        ..at(step)(error)
    }
}

/// `rtw89_phy_dm_init` (phy.c:8235-8261) for an RTL8852C's PHY 0 at RFE type
/// `efuse.rfe_type` and cut `cv`, after [`init_radio`] succeeded (it runs
/// after the coex init, core.c:6695-6697). Steps, in Linux's order (spec S2,
/// S17), each stopping everything after it:
///
/// 1. `"dm init"`: [`dm::stat_init`] with the efuse thermal trims,
///    [`dm::bb_sethw`], [`dm::env_monitor_init`],
///    [`dm::physts_parsing_init`], [`dm::dig_init`], [`dm::cfo_init`] with
///    the efuse crystal cap, [`dm::edcca_init`] (spec S12; the calls
///    between them are memory only on this chip, S12.1).
/// 2. `"nctl init"`: [`bb::init_rf_nctl`] with the firmware's NCTL table
///    (`rtw89_chip_rfk_hw_init` before it is NULL, rtw8852c.c:3077).
/// 3. `"rfk init"`: `rtw8852c_rfk_init` (rtw8852c.c:1985-1997), whose
///    memory-only set-up is not ported: [`rfk::rck`], [`rfk::dack`] with its
///    coex RFK notify, [`rfk::rx_dck`].
/// 4. `"dm tail"`: [`dm::set_txpwr_ctrl`], [`dm::power_trim`] with the
///    efuse trims, [`dm::cfg_txrx_path`].
///
/// A calibration poll that times out (NCTL, RCK, DRCK, ADDCK, DACK, RX DCK)
/// is recorded in the report and the steps carry on, as Linux only logs it
/// (spec S1.3). An I/O failure, an RF access SWSI never serves, an XTAL SI
/// command that never completes (spec S12.7) or a table that selects
/// nothing fails the step. Every failure has [`PhyError::coex_up`] set:
/// the coex init ran, so the teardown must run [`coex::radio_off`] first,
/// which also clears the scoreboard's WLRFK bit a failed DACK leaves set. A
/// `tables` without an NCTL table, which [`check_tables`] refuses before any
/// I/O, fails as `"phy tables"`.
pub fn calibrate<I: AxIo + ?Sized>(
    io: &mut I,
    tables: &PhyTables<'_>,
    efuse: &PhyEfuse,
    cv: u8,
) -> Result<RfkReport, PhyError> {
    let t = required(tables).map_err(after_coex("phy tables"))?;
    let (thermal, xtal) = dm_init(io, efuse).map_err(after_coex("dm init"))?;
    let nctl_timed_out =
        bb::init_rf_nctl(io, &t.nctl, efuse.rfe_type, cv).map_err(after_coex("nctl init"))?;
    let ((rck, rck_timed_out), dack, (rx_dck_timed_out, rx_dck_rewrites)) =
        rfk_init(io).map_err(after_coex("rfk init"))?;
    dm_tail(io, efuse).map_err(after_coex("dm tail"))?;
    Ok(RfkReport {
        thermal,
        xtal,
        rck,
        drck: dack.drck,
        addck: dack.addck,
        rx_dck_rewrites,
        nctl_timed_out,
        rck_timed_out,
        drck_timed_out: dack.drck_timed_out,
        addck_timed_out: dack.addck_timed_out,
        dack_timed_out: dack.dack_timed_out,
        rx_dck_timed_out,
    })
}

/// The first part of `rtw89_phy_dm_init` (phy.c:8237-8253). Returns the
/// thermals and the crystal cap read back.
fn dm_init<I: AxIo + ?Sized>(io: &mut I, efuse: &PhyEfuse) -> Result<([u8; 2], (u8, u8)), AxError> {
    let thermal = dm::stat_init(io, efuse.thermal_trim)?;
    // The RPL bias Linux keeps in `efuse_gain.offset_base` is for the
    // channel set-up, which is not ported.
    dm::bb_sethw(io)?;
    dm::env_monitor_init(io)?;
    dm::physts_parsing_init(io)?;
    dm::dig_init(io)?;
    let xtal = dm::cfo_init(io, efuse.xtal_cap)?;
    dm::edcca_init(io)?;
    Ok((thermal, xtal))
}

/// What `rtw8852c_rfk_init`'s three calibrations return, in their order.
type RfkInit = (([u8; 2], [bool; 2]), rfk::Dack, ([bool; 2], [u8; 2]));

/// The calibrations of `rtw8852c_rfk_init` (rtw8852c.c:1995-1997).
fn rfk_init<I: AxIo + ?Sized>(io: &mut I) -> Result<RfkInit, AxError> {
    let rck = rfk::rck(io)?;
    let dack = rfk::dack(io)?;
    let rx_dck = rfk::rx_dck(io)?;
    Ok((rck, dack, rx_dck))
}

/// The tail of `rtw89_phy_dm_init` (phy.c:8258-8260).
fn dm_tail<I: AxIo + ?Sized>(io: &mut I, efuse: &PhyEfuse) -> Result<(), AxError> {
    dm::set_txpwr_ctrl(io)?;
    dm::power_trim(io, efuse)?;
    dm::cfg_txrx_path(io)
}

#[cfg(test)]
mod tests;

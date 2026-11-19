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
//! Where Linux carries on with a half-configured radio (an RFE type no
//! radio branch takes, an RF write SWSI dropped), Stela stops at the step
//! that failed (spec S1.2-S1.3). This module grants no `rtw89_phy_dm_init`,
//! no NCTL table, no calibration, no channel, no coex H2C and no
//! acknowledgement wait: the bring-up powers the chip off after it.
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
use super::sys;
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

/// The radio step that failed and why; an RF init failure also names its
/// path and page.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PhyError {
    /// One of `"phy tables"`, `"bb rf reset"`, `"bb init"`, `"rf init"`,
    /// `"rf h2c"`, `"coex init"`, `"phy report"`.
    pub step: &'static str,
    pub error: AxError,
    /// For `"rf init"` and `"rf h2c"`, where.
    pub rf: Option<RfAt>,
}

fn at(step: &'static str) -> impl FnOnce(AxError) -> PhyError {
    move |error| PhyError {
        step,
        error,
        rf: None,
    }
}

/// The tables [`check_tables`] requires, unwrapped.
struct Required<'a> {
    bb: Reg2<'a>,
    bb_gain: Reg2<'a>,
    radio: [Radio<'a>; 2],
}

fn required<'a>(t: &PhyTables<'a>) -> Result<Required<'a>, AxError> {
    let bb = t.bb.ok_or(AxError::Phy("firmware carries no BB table"))?;
    let bb_gain = t
        .bb_gain
        .ok_or(AxError::Phy("firmware carries no BB gain table"))?;
    t.nctl
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
/// 5. `"phy report"`: `R_AX_GNT_VAL_V1` sampled before the coex init.
/// 6. `"coex init"`: [`coex::init`].
/// 7. `"phy report"`: RF 0x18 of path A, then B (SWSI reads), the GNT value
///    and the GNT SW readback, then [`command::count_c2h`].
///
/// `h2c` must be the state OFLD_CFG left (seq 1): the RF pages then carry
/// seq 1 and 2 and ask for no acknowledgement ([`radio::init_rf_reg`]). On
/// success the decoded BB gain table is returned with the report; on AX it
/// is memory only, for later steps.
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
        })?;
    let gnt_before = coex::gnt(io).map_err(at("phy report"))?;
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
    let report = report(io, rings).map_err(at("phy report"))?;
    Ok((report, bb.gain))
}

#[cfg(test)]
mod tests;

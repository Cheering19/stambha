// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! `rtw89_phy_init_rf_reg(noio = false)` (pinned rtw89/phy.c:2005-2039) for
//! an RTL8852C: each firmware radio table, slot 0 first, is walked through
//! [`phy_table::apply`] with `rtw89_phy_config_rf_reg_v1`
//! (phy.c:1713-1731) as its `config`, writing to the **table's** path, not
//! the slot's (phy.c:2023). In the pinned file slot 0 is RADIO_B, so path B
//! is configured first. Each written register from 0x100 up is stored as
//! `rtw89_phy_cofig_rf_reg_store` does (phy.c:1623-1641), and once a table is
//! done its stored words are sent to the firmware in pages of 500 as
//! `rtw89_phy_config_rf_reg_fw` (phy.c:1643-1669) and `rtw89_fw_h2c_rf_reg`
//! (fw.c:8090-8119) do, on CH12 through `Ch12::submit`.
//!
//! `rtw89_io_pack`/`unpack` do nothing on PCIe (fw.c:12682-12692) and are
//! not ported. Linux only warns when a page cannot be sent
//! (phy.c:2034-2036) and stops at nothing in the walk; Stela fails: a table
//! that selects nothing, or an RF access that fails, is
//! [`RfInitError::Table`] (step `"rf init"`), and a CH12 submit failure is
//! [`RfInitError::Submit`] (step `"rf h2c"`). Nothing waits for the
//! firmware: no acknowledgement is asked for at the sequence numbers the
//! bring-up uses, and none is read.
//!
//! This module grants no NCTL table, no channel, no calibration, no `noio`
//! variant and no access beyond `rf::write_rf`'s and the contract of
//! `Ch12::submit`.
use super::elements::{Radio, RfPath};
use super::h2c::H2c;
use super::io::{AxError, AxIo};
use super::pci::Rings;
use super::phy_table::{self, RADIO_A, RADIO_B};
use super::rf::{self, RFREG_MASK};
use alloc::vec::Vec;
use core::fmt;

/// `RTW89_H2C_RF_PAGE_SIZE` (fw.h:4663): words in one RF_REG page.
pub const RF_PAGE_WORDS: usize = 500;
/// `RTW89_H2C_RF_PAGE_NUM` (fw.h:4664): pages a path may store.
pub const RF_PAGES: usize = 3;
/// The words a path may store; later ones are dropped (phy.c:1631-1635).
const RF_WORDS: usize = RF_PAGE_WORDS * RF_PAGES;

/// `rtw89_phy_config_rf_reg_v1`'s one delay address: 50 ms
/// (`rtw89_io_mdelay`, plain `mdelay` on PCIe).
const DELAY_ADDR: u32 = 0xFE;
const DELAY_US: u64 = 50_000;
/// Registers below this are written but never stored (phy.c:1727-1728).
const STORED_FROM: u32 = 0x100;

/// The stored word for RF register `addr` set to `data`
/// (`(reg->addr << 20) | reg->data`, phy.c:1638-1639): the shift drops
/// addr bits from 12 up, so in practice bits [27:20] carry addr [7:0] and
/// [19:0] the data.
pub fn rf_reg_word(addr: u32, data: u32) -> u32 {
    addr.wrapping_shl(20) | data
}

/// What [`init_rf_reg`] did, by slot (0 is path B in the pinned file): the
/// entries each table's branch handed on, and the words stored and sent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RfInit {
    pub applied: [u16; 2],
    pub words: [u16; 2],
}

/// Why [`init_rf_reg`] failed, split as the bring-up names its steps, with
/// the path (and for a page, the page) it failed on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RfInitError {
    /// `path`'s radio table selected nothing or reached an unselected ELSE,
    /// or one of its RF writes failed: step `"rf init"`.
    Table { path: RfPath, error: AxError },
    /// `path`'s RF_REG page `page` (its H2C func) was not sent on CH12:
    /// step `"rf h2c"`.
    Submit {
        path: RfPath,
        page: u8,
        error: AxError,
    },
}

impl RfInitError {
    /// The bring-up step this failure names.
    pub fn step(&self) -> &'static str {
        match self {
            Self::Table { .. } => "rf init",
            Self::Submit { .. } => "rf h2c",
        }
    }

    /// The underlying error.
    pub fn error(&self) -> AxError {
        match *self {
            Self::Table { error, .. } | Self::Submit { error, .. } => error,
        }
    }

    /// Where it failed.
    pub fn at(&self) -> RfAt {
        match *self {
            Self::Table { path, .. } => RfAt { path, page: None },
            Self::Submit { path, page, .. } => RfAt {
                path,
                page: Some(page),
            },
        }
    }
}

/// The RF path, and for an RF_REG H2C the page, an RF init failure
/// happened on. Linux logs neither (it only warns of a failed page,
/// phy.c:2034-2036); a bring-up line names them, as `"path B"` or
/// `"path A page 1"`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RfAt {
    pub path: RfPath,
    pub page: Option<u8>,
}

impl fmt::Display for RfAt {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let path = match self.path {
            RfPath::A => "A",
            RfPath::B => "B",
            RfPath::C => "C",
            RfPath::D => "D",
        };
        write!(f, "path {path}")?;
        if let Some(page) = self.page {
            write!(f, " page {page}")?;
        }
        Ok(())
    }
}

/// The element name a radio table for `path` carries (fw.h:4387-4390).
fn table_name(path: RfPath) -> &'static str {
    match path {
        RfPath::A => RADIO_A,
        RfPath::B => RADIO_B,
        RfPath::C => "RADIO_C",
        RfPath::D => "RADIO_D",
    }
}

/// `rtw89_phy_config_rf_reg_v1` for one entry of `path`'s table: 0xFE is a
/// 50 ms delay; anything else is a full-mask RF write, then stored in
/// `words` if `addr` is 0x100 or more and fewer than 1500 are stored. Only
/// 0xFE is a delay, and no BYPASS data is skipped.
fn config_rf_reg_v1<I: AxIo + ?Sized>(
    io: &mut I,
    path: RfPath,
    addr: u32,
    data: u32,
    words: &mut Vec<u32>,
) -> Result<(), AxError> {
    if addr == DELAY_ADDR {
        io.delay_us(DELAY_US);
        return Ok(());
    }
    rf::write_rf(io, path, addr, RFREG_MASK, data)?;
    // rtw89_phy_cofig_rf_reg_store: a word past the last page is warned
    // about and dropped, after the write.
    if addr >= STORED_FROM && words.len() < RF_WORDS {
        words.push(rf_reg_word(addr, data));
    }
    Ok(())
}

/// `rtw89_phy_init_rf_reg(noio = false)` over the two radio slots at RFE
/// type `rfe` and cut `cv`, slot 0 first. For each slot, its table is
/// applied to the table's own path through `rtw89_phy_config_rf_reg_v1`,
/// then its stored words are sent as `ceil(words / 500)` RF_REG pages
/// (func 0, 1, 2) with `h2c`'s next sequence numbers, each submitted on
/// CH12; a path that stored nothing sends nothing. A submit returns once
/// CH12 has consumed the packet, and nothing waits for the firmware.
///
/// A failure stops everything after it: no later page, no later slot. It
/// names the path, and for a page its index ([`RfInitError::at`]).
///
/// `h2c` must be the state the OFLD_CFG H2C leaves (seq 1), as it is in
/// Linux, where `rtw89_fw_h2c_set_ofld_cfg` is the last H2C of
/// `rtw89_mac_init` (mac.c:4394) and this runs next (core.c:6676-6693). The
/// pinned file's tables then send one page per path, at seq 1 (path B) and
/// 2 (path A), and neither falls on the forced-REC_ACK rule (`seq % 4 ==
/// 0`, fw.c:1641), so the firmware is asked for no acknowledgement. Called
/// at another state, a page at a multiple of 4 would ask for a REC_ACK that
/// nothing here reads; it would stay in RXQ for the next poll. This is not
/// checked: the bring-up threads the one `H2c` from OFLD_CFG through.
///
/// # Safety
/// The contract of `Ch12::submit`: `io` is the exclusive register window of
/// the same RTL8852C requester that `pci::pre_init` programmed with
/// `rings`, and the `pre_init` contract still holds: its DMA domain
/// isolates that requester, the rings are coherent WB RAM with NoSnoop and
/// ATS disabled, and one worker owns the device and its queues.
pub unsafe fn init_rf_reg<I: AxIo + ?Sized>(
    io: &mut I,
    rings: &mut Rings,
    h2c: &mut H2c,
    radio: [Radio<'_>; 2],
    rfe: u8,
    cv: u8,
) -> Result<RfInit, RfInitError> {
    let mut done = RfInit {
        applied: [0; 2],
        words: [0; 2],
    };
    // `rtw89_fw_h2c_rf_reg_info`, reused per path: `curr_idx` is reset
    // after each send (phy.c:1666).
    let mut words = Vec::with_capacity(RF_WORDS);
    for (slot, Radio { path, table }) in radio.into_iter().enumerate() {
        words.clear();
        let applied = phy_table::apply(&table, table_name(path), rfe, cv, &mut |addr, data| {
            config_rf_reg_v1(io, path, addr, data, &mut words)
        })
        .map_err(|error| RfInitError::Table { path, error })?;
        done.applied[slot] = applied;
        // At most RF_WORDS (1500) are stored, which fits.
        done.words[slot] = words.len() as u16;
        // rtw89_phy_config_rf_reg_fw: `len = min(remain, 500)` per page,
        // for at most RF_PAGES pages, while words remain.
        for (page, chunk) in words.chunks(RF_PAGE_WORDS).enumerate() {
            let (_, packet) = h2c.rf_reg(path, page as u8, chunk);
            // SAFETY: the caller's contract is `submit`'s, word for word.
            unsafe { rings.ch12().submit(io, &packet) }.map_err(|error| RfInitError::Submit {
                path,
                page: page as u8,
                error,
            })?;
        }
    }
    Ok(done)
}

#[cfg(test)]
mod tests;

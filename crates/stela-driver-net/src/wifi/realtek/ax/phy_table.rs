// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! The condition-header engine rtw89 runs every BB, BB gain, radio and NCTL
//! table through: `rtw89_phy_sel_headline` (pinned rtw89/phy.c:1733-1813)
//! picks the table's branch by RFE type and cut, and `rtw89_phy_init_reg`
//! (phy.c:1815-1882) walks its IF/ELIF/ELSE/END bodies, handing each entry
//! of the taken branch to a `config` callback. The word layout is phy.h's
//! (phy.h:13-29).
//!
//! Where Linux logs `"invalid PHY package"` or `"failed to load CR"` and
//! carries on with the table half applied, [`apply`] returns
//! [`AxError::PhyPackage`] or [`AxError::PhyTable`]. Nothing here touches
//! a register: what an entry means (a BB write, a delay, an RF write) is
//! the callback's, and this module grants no BB or RF setup by itself.
use super::{AxError, elements::Reg2};

/// Table names for errors, as the element IDs name them (fw.h:4385-4391).
pub const BB_REG: &str = "BB_REG";
pub const BB_GAIN: &str = "BB_GAIN";
pub const RADIO_A: &str = "RADIO_A";
pub const RADIO_B: &str = "RADIO_B";
pub const RF_NCTL: &str = "RF_NCTL";

/// `PHY_HEADLINE_VALID` and the `PHY_COND_*` words in [31:28] (phy.h:14,
/// 24-28).
const HEADLINE: u32 = 0xf;
const BRANCH_IF: u32 = 0x8;
const BRANCH_ELIF: u32 = 0x9;
const BRANCH_ELSE: u32 = 0xa;
const BRANCH_END: u32 = 0xb;
const CHECK: u32 = 0x4;
/// `PHY_COND_DONT_CARE` (phy.h:29).
const DONT_CARE: u8 = 0xff;

/// `get_phy_headline` / `get_phy_cond`: bits [31:28].
fn cond(addr: u32) -> u32 {
    addr >> 28
}

/// `get_phy_target`: bits [27:0].
fn target(addr: u32) -> u32 {
    addr & 0x0fff_ffff
}

/// `get_phy_compare`: `rfe` in [23:16], `cv` in [7:0].
fn compare(rfe: u8, cv: u8) -> u32 {
    (u32::from(rfe) << 16) | u32::from(cv)
}

/// `get_phy_cond_rfe`: bits [23:16].
fn cond_rfe(addr: u32) -> u8 {
    (addr >> 16) as u8
}

/// `get_phy_cond_cv`: bits [7:0].
fn cond_cv(addr: u32) -> u8 {
    addr as u8
}

/// `rtw89_phy_sel_headline`: the length of the leading headline run and
/// the index of the selected headline, or `None` where Linux returns
/// `-EINVAL`. With no headline the index is 0.
fn select(table: &Reg2<'_>, rfe: u8, cv: u8) -> Option<(usize, usize)> {
    // phy.c:1746-1752: only the leading run of 0xF words.
    let size = table
        .iter()
        .take_while(|&(addr, _)| cond(addr) == HEADLINE)
        .count();
    if size == 0 {
        return Some((0, 0));
    }
    let headlines = || table.iter().take(size).map(|(addr, _)| addr);
    // Case 1, RFE and cv match; case 2, RFE match and cv don't care: the
    // first headline whose whole target equals (phy.c:1755-1776).
    for want in [compare(rfe, cv), compare(rfe, DONT_CARE)] {
        if let Some(index) = headlines().position(|addr| target(addr) == want) {
            return Some((size, index));
        }
    }
    // Case 3, RFE match; case 4, RFE don't care: the largest cv, ties to
    // the last (`>=`, phy.c:1778-1810). Linux shares `cv_max` between the
    // two, but case 4 runs only when case 3 set nothing, so it is still 0.
    for want in [rfe, DONT_CARE] {
        let mut cv_max = 0;
        let mut selected = None;
        for (index, addr) in headlines().enumerate() {
            if cond_rfe(addr) == want && cond_cv(addr) >= cv_max {
                cv_max = cond_cv(addr);
                selected = Some(index);
            }
        }
        if let Some(index) = selected {
            return Some((size, index));
        }
    }
    None
}

/// `rtw89_phy_init_reg` over `table` (named `name` in errors) at RFE type
/// `rfe` and cut `cv`: selects the headline, then passes every entry of
/// the taken branches to `config` in table order. Returns how many entries
/// `config` was given.
///
/// - No headline for `rfe`/`cv`: [`AxError::PhyPackage`], before any entry.
/// - An ELSE no branch reached: [`AxError::PhyTable`] at its index, after
///   the entries before it.
/// - A `config` error stops the walk and is returned.
/// - A table of more than `u16::MAX` entries is refused with
///   [`AxError::Phy`] before any entry, so the count and index fit. The
///   pinned file's longest table has 17604.
pub fn apply(
    table: &Reg2<'_>,
    name: &'static str,
    rfe: u8,
    cv: u8,
    config: &mut dyn FnMut(u32, u32) -> Result<(), AxError>,
) -> Result<u16, AxError> {
    if table.len() > usize::from(u16::MAX) {
        return Err(AxError::Phy("PHY table has more than 65535 entries"));
    }
    let (size, selected) = select(table, rfe, cv).ok_or(AxError::PhyPackage {
        table: name,
        rfe,
        cv,
    })?;
    // phy.c:1842: regs[headline_idx], which is entry 0 when there is no
    // headline. An empty table has none, and its walk below is empty.
    let cfg_target = table.get(selected).map_or(0, |(addr, _)| target(addr));
    let mut branch = 0;
    let mut matched = true;
    let mut found = false;
    let mut applied: u16 = 0;
    for (index, (addr, data)) in table.iter().enumerate().skip(size) {
        match cond(addr) {
            BRANCH_IF | BRANCH_ELIF => branch = target(addr),
            BRANCH_ELSE => {
                matched = false;
                if !found {
                    return Err(AxError::PhyTable {
                        table: name,
                        index: u16::try_from(index).unwrap_or(u16::MAX),
                    });
                }
            }
            BRANCH_END => {
                matched = true;
                found = false;
            }
            CHECK => {
                if found {
                    matched = false;
                } else {
                    matched = branch == cfg_target;
                    found = matched;
                }
            }
            _ => {
                if matched {
                    config(addr, data)?;
                    applied += 1;
                }
            }
        }
    }
    Ok(applied)
}

#[cfg(test)]
mod tests;

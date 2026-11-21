// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! The TX power regulatory index of an RTL8852C, chosen as pinned rtw89
//! chooses it: the map is the firmware's REGD element (Linux's
//! `rtw89_regd_setup` takes it over the static `rtw89_regd_map`,
//! regd.c:716-722), a country is looked up as
//! `rtw89_regd_find_reg_by_name` does (regd.c:307-320, a miss being
//! `rtw89_ww_regd`, regd.c:24-25), the efuse country wins when the map has
//! it as in `rtw89_regd_init_hint` (regd.c:738-768), and otherwise the
//! country the stack would hint (here the network's) is looked up as
//! `rtw89_regd_notifier_apply` does (regd.c:882-896). Each band's index is
//! read as `rtw89_regd_get` reads it (core.h:8923-8933), with
//! `txpwr_uk_follow_etsi` true as `rtw89_regd_setup_reg_rules` leaves it on
//! PCIe without ACPI (regd.c:681-707). The index values are
//! `enum rtw89_regulation_type` (core.h:774-792).
//!
//! The country the stack would hint is the configured network's alone.
//! Without an efuse country Linux leaves cfg80211 free to follow the
//! associated AP's country element (`regulatory_hint_country_ie`, which
//! `REGULATORY_STRICT_REG` stops only for a programmed chip,
//! regd.c:747-757), and its notifier then moves the index to that country;
//! Stela follows no AP's country element, so on a chip without an efuse
//! country the index of a network configured without one stays the
//! world's, `WW`, whatever the AP announces.
//!
//! This module does not grant any channel, the channel rules a country
//! allows (Stela's regulatory database owns those), UNII-4 or 6 GHz policy,
//! TAS or antenna-gain policy, or any register write; it only names the
//! index the TX power tables are read at.
use super::{chan::HwBand, txpwr_tables::RegdEntry};

/// `enum rtw89_regulation_type` (core.h:774-792).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Regd {
    Ww = 0,
    Etsi = 1,
    Fcc = 2,
    Mkk = 3,
    Na = 4,
    Ic = 5,
    Kcc = 6,
    Acma = 7,
    Ncc = 8,
    Mexico = 9,
    Chile = 10,
    Ukraine = 11,
    Cn = 12,
    Qatar = 13,
    Uk = 14,
    Thailand = 15,
}

impl Regd {
    /// A REGD entry's rule byte; anything past `RTW89_THAILAND` is
    /// `RTW89_NA`, as `rtw89_regd_entcpy` stores it (fw.c:1323-1344).
    pub const fn from_rule(rule: u8) -> Self {
        match rule {
            0 => Self::Ww,
            1 => Self::Etsi,
            2 => Self::Fcc,
            3 => Self::Mkk,
            5 => Self::Ic,
            6 => Self::Kcc,
            7 => Self::Acma,
            8 => Self::Ncc,
            9 => Self::Mexico,
            10 => Self::Chile,
            11 => Self::Ukraine,
            12 => Self::Cn,
            13 => Self::Qatar,
            14 => Self::Uk,
            15 => Self::Thailand,
            _ => Self::Na,
        }
    }
}

/// `regd->txpwr_regd` for the bands Stela runs, as the map gives them
/// (UK is kept; [`Regds::txpwr`] applies the ETSI rule).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Regds {
    pub g2: Regd,
    pub g5: Regd,
}

impl Regds {
    /// `rtw89_ww_regd`: WW on every band.
    const WW: Self = Self {
        g2: Regd::Ww,
        g5: Regd::Ww,
    };

    /// `rtw89_regd_get` (core.h:8923-8933) with `txpwr_uk_follow_etsi`
    /// set: UK reads as ETSI. 6 GHz is not supported, so no rule applies
    /// there and it reads NA.
    pub fn txpwr(&self, band: HwBand) -> Regd {
        let regd = match band {
            HwBand::G2 => self.g2,
            HwBand::G5 => self.g5,
            HwBand::G6 => return Regd::Na,
        };
        if regd == Regd::Uk { Regd::Etsi } else { regd }
    }
}

/// `rtw89_ww_regd.alpha2` (regd.c:24-25).
const WW_ALPHA2: [u8; 2] = *b"00";

/// `rtw89_regd_find_reg_by_name` (regd.c:307-320): the first entry with
/// this alpha2, or `None` for the world regd.
fn find(map: &[RegdEntry], alpha2: [u8; 2]) -> Option<&RegdEntry> {
    map.iter().find(|entry| entry.alpha2 == alpha2)
}

/// The regulatory index for `map` (the firmware's REGD entries), the
/// efuse country `efuse` and the network's country `network`, with the
/// alpha2 of the entry used (`00` for the world regd). The efuse country
/// wins when the map has it (`rtwdev->regulatory.programmed`); otherwise
/// the network's country is looked up; a miss, or no country at all, is
/// WW on every band.
pub fn select(
    map: &[RegdEntry],
    efuse: Option<[u8; 2]>,
    network: Option<[u8; 2]>,
) -> (Regds, [u8; 2]) {
    let entry = efuse
        .and_then(|alpha2| find(map, alpha2))
        .or_else(|| find(map, network.unwrap_or(WW_ALPHA2)));
    match entry {
        Some(entry) => (
            Regds {
                g2: Regd::from_rule(entry.rule[HwBand::G2 as usize]),
                g5: Regd::from_rule(entry.rule[HwBand::G5 as usize]),
            },
            entry.alpha2,
        ),
        None => (Regds::WW, WW_ALPHA2),
    }
}

#[cfg(test)]
mod tests;

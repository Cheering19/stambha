// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
extern crate std;
use super::*;
use alloc::vec::Vec;

/// One REGD entry with the rules for 2.4, 5 and 6 GHz as given.
fn entry(alpha2: &[u8; 2], rule: [u8; 3]) -> RegdEntry {
    RegdEntry {
        alpha2: *alpha2,
        rule,
        fmap: 0,
    }
}

/// Three rows of Linux's static `rtw89_regd_map` (regd.c:43, :56, :118)
/// and its UK row (regd.c:77).
fn map() -> Vec<RegdEntry> {
    alloc::vec![
        entry(b"US", [2, 2, 2]),    // FCC
        entry(b"DE", [1, 1, 1]),    // ETSI
        entry(b"IN", [1, 1, 4]),    // ETSI, 6 GHz NA
        entry(b"GB", [14, 14, 14]), // UK
    ]
}

const WW: Regds = Regds {
    g2: Regd::Ww,
    g5: Regd::Ww,
};

#[test]
fn efuse_country_wins_when_mapped() {
    let (regds, alpha2) = select(&map(), Some(*b"US"), Some(*b"DE"));
    assert_eq!(
        regds,
        Regds {
            g2: Regd::Fcc,
            g5: Regd::Fcc
        }
    );
    assert_eq!(&alpha2, b"US");
    // With no network country the efuse still wins.
    assert_eq!(select(&map(), Some(*b"IN"), None).1, *b"IN");
}

#[test]
fn unmapped_efuse_falls_to_the_network_country() {
    // "ZZ" is in no map: regd.c:747-748 finds the world regd, so the stack's
    // country (the network's here) decides.
    let (regds, alpha2) = select(&map(), Some(*b"ZZ"), Some(*b"DE"));
    assert_eq!(
        regds,
        Regds {
            g2: Regd::Etsi,
            g5: Regd::Etsi
        }
    );
    assert_eq!(&alpha2, b"DE");
    // No efuse country at all.
    let (regds, alpha2) = select(&map(), None, Some(*b"IN"));
    assert_eq!(
        (regds.g2, regds.g5, &alpha2),
        (Regd::Etsi, Regd::Etsi, b"IN")
    );
}

#[test]
fn nothing_known_is_world() {
    // regd.c:24-25, :307-320: a miss is rtw89_ww_regd, "00", WW everywhere.
    assert_eq!(select(&map(), None, None), (WW, *b"00"));
    assert_eq!(select(&map(), None, Some(*b"00")), (WW, *b"00"));
    assert_eq!(select(&map(), Some(*b"ZZ"), Some(*b"ZZ")), (WW, *b"00"));
    // An empty map knows nothing, not even the efuse's country.
    assert_eq!(select(&[], Some(*b"US"), Some(*b"US")), (WW, *b"00"));
}

#[test]
fn first_entry_of_a_country_wins() {
    // rtw89_regd_find_reg_by_name returns the first match (regd.c:314-316).
    let m = [entry(b"US", [2, 2, 2]), entry(b"US", [3, 3, 3])];
    assert_eq!(select(&m, Some(*b"US"), None).0.g2, Regd::Fcc);
}

#[test]
fn rule_16_or_more_is_na() {
    // Task 2 already stores NA for a rule past THAILAND (fw.c:1323-1344);
    // an entry built by hand gets the same answer here.
    let m = [entry(b"US", [16, 255, 2])];
    assert_eq!(
        select(&m, Some(*b"US"), None).0,
        Regds {
            g2: Regd::Na,
            g5: Regd::Na
        }
    );
    // Every defined value keeps its number (core.h:774-792).
    for v in 0..16u8 {
        assert_eq!(Regd::from_rule(v) as u8, v);
    }
    assert_eq!(Regd::from_rule(16), Regd::Na);
    assert_eq!(Regd::from_rule(u8::MAX), Regd::Na);
}

#[test]
fn uk_follows_etsi_for_tx_power() {
    // txpwr_uk_follow_etsi is true on PCIe without ACPI (regd.c:692), so
    // rtw89_regd_get (core.h:8923-8933) reads UK as ETSI.
    let (regds, alpha2) = select(&map(), Some(*b"GB"), None);
    assert_eq!(&alpha2, b"GB");
    assert_eq!((regds.g2, regds.g5), (Regd::Uk, Regd::Uk));
    assert_eq!(regds.txpwr(HwBand::G2), Regd::Etsi);
    assert_eq!(regds.txpwr(HwBand::G5), Regd::Etsi);
    // Only UK is changed.
    let us = select(&map(), Some(*b"US"), None).0;
    assert_eq!(us.txpwr(HwBand::G2), Regd::Fcc);
    let split = Regds {
        g2: Regd::Mkk,
        g5: Regd::Thailand,
    };
    assert_eq!(split.txpwr(HwBand::G2), Regd::Mkk);
    assert_eq!(split.txpwr(HwBand::G5), Regd::Thailand);
    // 6 GHz is not supported: no rule applies.
    assert_eq!(us.txpwr(HwBand::G6), Regd::Na);
}

#[test]
#[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES"]
fn real_8852c_fw_2_regd_map_selects_as_linux_does() {
    use crate::wifi::realtek::ax::txpwr_tables::tx_power_tables;
    let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
    let b = std::fs::read(root.join("rtw89/rtw8852c_fw-2.bin")).unwrap();
    assert_eq!(b.len(), 2_644_326);
    let t = tx_power_tables(&b, 0, 1).unwrap();
    let map = t.regd_map();
    // Every alpha2 the file names is two upper-case letters, so the efuse
    // bytes `country_code` refuses could match nothing here in Linux either,
    // and "00" is not listed: a miss is the world regd.
    assert!(
        map.iter()
            .all(|e| e.alpha2.iter().all(u8::is_ascii_uppercase))
    );
    let pick = |efuse: Option<[u8; 2]>, network: Option<[u8; 2]>| select(map, efuse, network);
    let fcc = Regds {
        g2: Regd::Fcc,
        g5: Regd::Fcc,
    };
    let etsi = Regds {
        g2: Regd::Etsi,
        g5: Regd::Etsi,
    };
    // The file's rows agree with Linux's static map (regd.c:43, :56, :118).
    assert_eq!(pick(Some(*b"US"), Some(*b"DE")), (fcc, *b"US"));
    assert_eq!(pick(None, Some(*b"DE")), (etsi, *b"DE"));
    assert_eq!(pick(Some(*b"ZZ"), Some(*b"IN")), (etsi, *b"IN"));
    assert_eq!(pick(Some(*b"ZZ"), Some(*b"ZZ")), (WW, *b"00"));
    assert_eq!(pick(None, None), (WW, *b"00"));
    let (gb, _) = pick(None, Some(*b"GB"));
    assert_eq!((gb.g2, gb.txpwr(HwBand::G2)), (Regd::Uk, Regd::Etsi));
}

/// `rtw89_regd_setup_unii4` without ACPI (`regd.c:359-409`): the DSM
/// evaluation fails, `val` becomes `RTW89_ACPI_CONF_UNII4_US` alone, so only
/// US is cleared from `block_unii4`; CA needs the DSM's bit, and the world
/// regd has no index (`rtw89_regd_get_index`, `regd.c:327-338`), so both
/// stay blocked (`rtw89_regd_apply_policy_unii4`, `regd.c:770-797`).
#[test]
fn unii4_is_us_only_without_acpi() {
    assert!(unii4_allowed(*b"US"));
    for alpha2 in [*b"CA", *b"IN", *b"00", *b"DE", *b"us"] {
        assert!(!unii4_allowed(alpha2), "{alpha2:?}");
    }
}

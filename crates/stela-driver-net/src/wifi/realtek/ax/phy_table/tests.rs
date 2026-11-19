// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
extern crate std;
use super::*;
use crate::wifi::realtek::ax::elements::{
    ID_BB_REG, RfPath, phy_tables,
    tests::{container, element, pairs},
};
use alloc::{format, string::String, vec::Vec};

/// A firmware file whose one BB_REG element holds `regs`. A TXPWR element
/// follows it, so an empty table is still read (fw.c:1584 needs a byte past
/// a header).
fn file(regs: &[(u32, u32)]) -> Vec<u8> {
    let mut b = container(32, 16);
    element(&mut b, ID_BB_REG, [0; 4], 0, 0, &pairs(regs));
    element(&mut b, 9, [0; 4], 0, 0, &[0; 16]);
    b
}

/// `apply` over `regs` at `rfe`/`cv`, with the entries `config` saw.
fn run(
    regs: &[(u32, u32)],
    name: &'static str,
    rfe: u8,
    cv: u8,
) -> (Result<u16, AxError>, Vec<(u32, u32)>) {
    let b = file(regs);
    let table = phy_tables(&b, 0).unwrap().bb.unwrap();
    let mut seen = Vec::new();
    let result = apply(&table, name, rfe, cv, &mut |addr, data| {
        seen.push((addr, data));
        Ok(())
    });
    (result, seen)
}

const IF: u32 = 0x8000_0000;
const ELIF: u32 = 0x9000_0000;
const ELSE: (u32, u32) = (0xa000_0000, 0);
const END: (u32, u32) = (0xb000_0000, 0);
const CHECK: (u32, u32) = (0x4000_0000, 0);

#[test]
fn exact_headline_runs_the_matching_branch_and_never_an_else() {
    let regs = [
        (0xf001_0001, 0),
        (0xf001_00ff, 0),
        (0x8001_0001, 0),
        (0x4000_0000, 0),
        (0x100, 1),
        (0xa000_0000, 0),
        (0x200, 2),
        (0xb000_0000, 0),
        (0x300, 3),
    ];
    let (result, seen) = run(&regs, BB_REG, 1, 1);
    assert_eq!(seen, [(0x100, 1), (0x300, 3)]);
    assert_eq!(result, Ok(2));
}

#[test]
fn cv_dont_care_headline_is_the_second_choice() {
    let regs = [
        (0xf001_0000, 0),
        (0xf001_00ff, 0),
        (IF | 0x0001_0000, 0),
        CHECK,
        (0x100, 1),
        END,
        (IF | 0x0001_00ff, 0),
        CHECK,
        (0x200, 2),
        END,
    ];
    let (result, seen) = run(&regs, BB_REG, 1, 1);
    assert_eq!(seen, [(0x200, 2)]);
    assert_eq!(result, Ok(1));

    // Case 2 takes the first headline equal to compare(rfe, 0xFF), before
    // case 3 could take the last one with cv 0xFF (phy.c:1767-1776).
    let regs = [
        (0xf001_00ff, 0),
        (0xf001_01ff, 0),
        (IF | 0x0001_01ff, 0),
        CHECK,
        (0x100, 1),
        END,
        (IF | 0x0001_00ff, 0),
        CHECK,
        (0x200, 2),
        END,
    ];
    assert_eq!(run(&regs, BB_REG, 1, 1).1, [(0x200, 2)]);
}

#[test]
fn same_rfe_highest_cv_wins_with_ties_to_the_last() {
    let regs = [
        (0xf001_0000, 0),
        (0xf001_0002, 0),
        (0xf001_0002, 0),
        (IF | 0x0001_0000, 0),
        CHECK,
        (0x100, 1),
        END,
        (IF | 0x0001_0002, 0),
        CHECK,
        (0x200, 2),
        END,
    ];
    let (result, seen) = run(&regs, BB_REG, 1, 1);
    assert_eq!(seen, [(0x200, 2)]);
    assert_eq!(result, Ok(1));

    // The two cv 2 headlines differ in their package byte, which selection
    // ignores, so only `>=` takes index 2 over index 1 (phy.c:1779-1789).
    let regs = [
        (0xf001_0000, 0),
        (0xf001_0002, 0),
        (0xf001_0102, 0),
        (IF | 0x0001_0002, 0),
        CHECK,
        (0x100, 1),
        END,
        (IF | 0x0001_0102, 0),
        CHECK,
        (0x200, 2),
        END,
    ];
    assert_eq!(run(&regs, BB_REG, 1, 1).1, [(0x200, 2)]);
}

#[test]
fn dont_care_rfe_is_the_last_resort() {
    let regs = [
        (0xf0ff_0000, 0),
        (IF | 0x00ff_0000, 0),
        CHECK,
        (0x100, 1),
        END,
    ];
    let (result, seen) = run(&regs, BB_REG, 7, 1);
    assert_eq!(seen, [(0x100, 1)]);
    assert_eq!(result, Ok(1));

    // A headline for the RFE type itself, whatever its cv, comes first.
    let regs = [
        (0xf0ff_0005, 0),
        (0xf007_0000, 0),
        (IF | 0x00ff_0005, 0),
        CHECK,
        (0x100, 1),
        END,
        (IF | 0x0007_0000, 0),
        CHECK,
        (0x200, 2),
        END,
    ];
    assert_eq!(run(&regs, BB_REG, 7, 1).1, [(0x200, 2)]);
}

#[test]
fn no_matching_headline_is_refused() {
    let regs = [(0xf002_0001, 0), (0x100, 1), (0x200, 2)];
    let (result, seen) = run(&regs, RADIO_B, 1, 1);
    assert_eq!(
        result,
        Err(AxError::PhyPackage {
            table: "RADIO_B",
            rfe: 1,
            cv: 1
        })
    );
    assert!(seen.is_empty());
}

#[test]
fn no_headlines_apply_every_entry() {
    let (result, seen) = run(&[(0x8008, 0), (0x8000, 8)], RF_NCTL, 1, 1);
    assert_eq!(seen, [(0x8008, 0), (0x8000, 8)]);
    assert_eq!(result, Ok(2));
}

#[test]
fn with_no_headlines_the_first_entry_is_the_target() {
    // phy.c:1842 takes regs[headline_idx = 0] even when it is no headline,
    // so a branch on the first entry's own target is taken.
    let regs = [
        (IF | 0x0002_0001, 0),
        CHECK,
        (0x100, 1),
        ELSE,
        (0x200, 2),
        END,
        (0x300, 3),
    ];
    let (result, seen) = run(&regs, RF_NCTL, 1, 1);
    assert_eq!(seen, [(0x100, 1), (0x300, 3)]);
    assert_eq!(result, Ok(2));
}

#[test]
fn an_empty_table_applies_nothing() {
    assert_eq!(run(&[], RF_NCTL, 1, 1), (Ok(0), Vec::new()));
}

#[test]
fn a_headline_word_past_the_leading_run_is_a_register_entry() {
    // Only the leading run counts (phy.c:1746-1752); later words of any
    // other nibble reach config (phy.c:1877-1880).
    let regs = [(0xf001_0001, 0), (0x100, 1), (0xf001_0001, 2)];
    let (result, seen) = run(&regs, BB_REG, 1, 1);
    assert_eq!(seen, [(0x100, 1), (0xf001_0001, 2)]);
    assert_eq!(result, Ok(2));
}

#[test]
fn else_reached_without_a_branch_stops_the_table() {
    let regs = [
        (0xf001_0001, 0),
        (IF | 0x0002_0001, 0),
        CHECK,
        (0x100, 1),
        ELSE,
        (0x200, 2),
        END,
        (0x300, 3),
    ];
    let (result, seen) = run(&regs, RADIO_A, 1, 1);
    assert_eq!(
        result,
        Err(AxError::PhyTable {
            table: "RADIO_A",
            index: 4
        })
    );
    assert!(seen.is_empty());
}

#[test]
fn a_check_after_a_taken_branch_skips_the_rest() {
    let regs = [
        (0xf001_0001, 0),
        (IF | 0x0001_0001, 0),
        CHECK,
        (0x100, 1),
        (ELIF | 0x0001_0001, 0),
        CHECK,
        (0x200, 2),
        ELSE,
        (0x300, 3),
        END,
        (0x400, 4),
    ];
    let (result, seen) = run(&regs, BB_REG, 1, 1);
    assert_eq!(seen, [(0x100, 1), (0x400, 4)]);
    assert_eq!(result, Ok(2));
}

#[test]
fn config_error_stops_the_walk() {
    let b = file(&[(1, 1), (2, 2), (3, 3)]);
    let table = phy_tables(&b, 0).unwrap().bb.unwrap();
    let failure = AxError::Timeout {
        register: 0x1174c,
        mask: 1,
    };
    let mut calls = 0;
    let result = apply(&table, BB_REG, 1, 1, &mut |addr, _| {
        calls += 1;
        if addr == 2 { Err(failure) } else { Ok(()) }
    });
    assert_eq!(result, Err(failure));
    assert_eq!(calls, 2);
}

#[test]
fn a_table_too_long_to_count_is_refused_before_any_entry() {
    let regs: Vec<(u32, u32)> = (0..=u32::from(u16::MAX)).map(|i| (0x100, i)).collect();
    let (result, seen) = run(&regs, BB_REG, 1, 1);
    assert_eq!(
        result,
        Err(AxError::Phy("PHY table has more than 65535 entries"))
    );
    assert!(seen.is_empty());
    let (result, seen) = run(&regs[1..], BB_REG, 1, 1);
    assert_eq!(result, Ok(u16::MAX));
    assert_eq!(seen.len(), 65535);
}

#[test]
fn phy_errors_display() {
    assert_eq!(format!("{}", AxError::Phy("phy says")), "phy says");
    assert_eq!(
        format!(
            "{}",
            AxError::PhyPackage {
                table: RADIO_B,
                rfe: 7,
                cv: 1
            }
        ),
        "no RADIO_B branch for RFE type 7 at cut 1"
    );
    assert_eq!(
        format!(
            "{}",
            AxError::PhyTable {
                table: BB_REG,
                index: 4
            }
        ),
        "BB_REG entry 4 is an ELSE no branch reached"
    );
}

/// The applied entries and the SHA-256 of them as LE (addr, data) pairs.
fn applied(
    table: &Reg2<'_>,
    name: &'static str,
    rfe: u8,
) -> Result<(Vec<(u32, u32)>, String), AxError> {
    use sha2::{Digest, Sha256};
    let mut seen = Vec::new();
    let count = apply(table, name, rfe, 1, &mut |addr, data| {
        seen.push((addr, data));
        Ok(())
    })?;
    assert_eq!(usize::from(count), seen.len());
    let mut hash = Sha256::new();
    for (addr, data) in &seen {
        hash.update(addr.to_le_bytes());
        hash.update(data.to_le_bytes());
    }
    let hex = hash.finalize().iter().map(|b| format!("{b:02x}")).collect();
    Ok((seen, hex))
}

#[test]
#[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES"]
fn real_8852c_fw_2_tables_select_as_linux_does() {
    let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
    let b = std::fs::read(root.join("rtw89/rtw8852c_fw-2.bin")).unwrap();
    let t = phy_tables(&b, 0).unwrap();
    let (bb, gain, nctl) = (t.bb.unwrap(), t.bb_gain.unwrap(), t.nctl.unwrap());
    let (slot0, slot1) = (t.radio[0].unwrap(), t.radio[1].unwrap());
    assert_eq!((slot0.path, slot1.path), (RfPath::B, RfPath::A));
    let sha = |table, name, rfe| applied(table, name, rfe).map(|(seen, hex)| (seen.len(), hex));

    // Spec S5.4, cv 1.
    let s = |hex: &str| String::from(hex);
    assert_eq!(
        sha(&bb, BB_REG, 1),
        Ok((
            1426,
            s("8b8d51888011a74317d9bac62e6af0304593cb8d07c89c7ca303cc1ffd843589")
        ))
    );
    assert_eq!(
        sha(&gain, BB_GAIN, 1),
        Ok((
            336,
            s("fb58551204573a91512657139e87cb15ce55ff92a0e488237017e867a9619d8d")
        ))
    );
    assert_eq!(
        sha(&slot0.table, RADIO_B, 1),
        Ok((
            2013,
            s("1b551bf1750c6ca50c3803fce5f390e0fee501ac5f95bf0604b6ff83ee5c1740")
        ))
    );
    assert_eq!(
        sha(&slot1.table, RADIO_A, 1),
        Ok((
            2046,
            s("6308cb90f70bb6aa7fa5fb261ae6fd060222340af6cf7680e86e484bb5c8ae88")
        ))
    );
    let nctl_sha = s("fec1c1dcc061316be04e931aecde57a42eea8f4af0112147502c34c53cbf2df5");
    assert_eq!(sha(&nctl, RF_NCTL, 1), Ok((1774, nctl_sha.clone())));
    assert_eq!(
        sha(&bb, BB_REG, 51),
        Ok((
            1426,
            s("a88da834bd28a7c60ac3b288e942f9741669e52994e6d4c69699e109ae0ecce6")
        ))
    );
    assert_eq!(
        sha(&gain, BB_GAIN, 51),
        Ok((
            336,
            s("5b6abc466d8a84d9e8d392010233509049cb8753224bac9556873e50271954b5")
        ))
    );
    assert_eq!(
        sha(&slot0.table, RADIO_B, 51),
        Ok((
            2013,
            s("84efe5358bc1fa44de5cc6b95901e258807d808f45184e54d94345ae58e5fdae")
        ))
    );
    assert_eq!(
        sha(&slot1.table, RADIO_A, 51),
        Ok((
            2046,
            s("829e4f7039f756917890ddbe9bc47952616db75ae02488dd8cdad3d76b94cb94")
        ))
    );
    // Spec S19.6: RFE 53 and 54 select different headlines but apply one
    // body.
    for rfe in [53, 54] {
        assert!(
            sha(&bb, BB_REG, rfe).unwrap().1.starts_with("65be9a2c"),
            "rfe {rfe}"
        );
    }

    // RFE 0 and 0xFF have BB but no radio branch.
    for rfe in [0, 0xff] {
        assert_eq!(
            sha(&slot0.table, RADIO_B, rfe),
            Err(AxError::PhyPackage {
                table: RADIO_B,
                rfe,
                cv: 1
            })
        );
        assert_eq!(sha(&bb, BB_REG, rfe).map(|(n, _)| n), Ok(1426));
    }

    // Spec S1.2: the counts hold for every RFE type the radio accepts, and
    // only those are accepted; NCTL is the same for all.
    let supported = [1, 2, 3, 4, 5, 7, 21, 22, 50, 51, 52, 53, 54, 63, 64];
    for rfe in 0..=u8::MAX {
        assert_eq!(sha(&bb, BB_REG, rfe).map(|(n, _)| n), Ok(1426), "rfe {rfe}");
        assert_eq!(
            sha(&gain, BB_GAIN, rfe).map(|(n, _)| n),
            Ok(336),
            "rfe {rfe}"
        );
        assert_eq!(
            sha(&nctl, RF_NCTL, rfe),
            Ok((1774, nctl_sha.clone())),
            "rfe {rfe}"
        );
        for (radio, name, count) in [(&slot0.table, RADIO_B, 2013), (&slot1.table, RADIO_A, 2046)] {
            match applied(radio, name, rfe) {
                Ok((seen, _)) => {
                    assert!(supported.contains(&rfe), "rfe {rfe} {name}");
                    assert_eq!(seen.len(), count, "rfe {rfe} {name}");
                    // Each radio body ends with exactly one {0xFE, 0x63}.
                    assert_eq!(seen.last(), Some(&(0xfe, 0x63)));
                    assert_eq!(seen.iter().filter(|&&e| e == (0xfe, 0x63)).count(), 1);
                }
                Err(e) => {
                    assert!(!supported.contains(&rfe), "rfe {rfe} {name}: {e}");
                    assert_eq!(
                        e,
                        AxError::PhyPackage {
                            table: name,
                            rfe,
                            cv: 1
                        }
                    );
                }
            }
        }
    }

    // The first and last BB entries applied.
    let (seen, _) = applied(&bb, BB_REG, 1).unwrap();
    assert_eq!(seen[..2], [(0x70c, 0x20), (0x704, 0x601e_0100)]);
    assert_eq!(seen[seen.len() - 2..], [(0xf4, 0x28), (0xf8, 0x2022_0610)]);
}

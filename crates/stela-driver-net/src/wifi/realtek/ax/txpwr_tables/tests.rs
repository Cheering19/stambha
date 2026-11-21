// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
extern crate std;
use super::*;
use crate::wifi::realtek::ax::elements::tests::{container, element_with_head};
use alloc::{string::ToString, vec::Vec};

/// Append a TX power (or REGD) element with the head of fw.h:4475-4491:
/// rsvd0, rsvd1, rfe_type (rsvd2 for REGD), ent_sz, num_ents le32.
fn txpwr(b: &mut Vec<u8>, id: u32, aid: u16, rfe: u8, ent_sz: u8, entries: &[&[u8]]) {
    let num = entries.len() as u32;
    let mut head = [0, 0, rfe, ent_sz, 0, 0, 0, 0];
    head[4..].copy_from_slice(&num.to_le_bytes());
    let body: Vec<u8> = entries.iter().flat_map(|e| e.iter().copied()).collect();
    element_with_head(b, id, [0, 0x82, 0, 0], aid, head, &body);
}

/// A file with an empty rfe 0 element for every needed ID but `without`,
/// then whatever `add` appends, then a DIAG_MAC element: the walk never
/// reaches a last element whose header ends the file (fw.c:1584).
fn file_without(without: Option<u32>, add: impl FnOnce(&mut Vec<u8>)) -> Vec<u8> {
    let mut b = container(32, 16);
    for id in NEEDED.into_iter().filter(|&id| Some(id) != without) {
        let ent_sz = if id == ID_REGD { 9 } else { 7 };
        txpwr(&mut b, id, 0, 0, ent_sz, &[]);
    }
    add(&mut b);
    element_with_head(&mut b, 28, [0; 4], 0, [0; 8], &[0; 16]);
    b
}

/// A TX power or REGD element whose head claims `num_ents` entries of
/// `ent_sz` bytes over `body`.
fn claiming(b: &mut Vec<u8>, id: u32, ent_sz: u8, num_ents: u32, body: &[u8]) {
    let mut head = [0, 0, 0, ent_sz, 0, 0, 0, 0];
    head[4..].copy_from_slice(&num_ents.to_le_bytes());
    element_with_head(b, id, [0; 4], 0, head, body);
}

fn file(add: impl FnOnce(&mut Vec<u8>)) -> Vec<u8> {
    file_without(None, add)
}

/// A `rtw89_fw_txpwr_lmt_{2,5}ghz_entry` (fw.h:5843-5862).
fn lmt(bw: u8, nt: u8, rs: u8, bf: u8, regd: u8, ch_idx: u8, v: i8) -> [u8; 7] {
    [bw, nt, rs, bf, regd, ch_idx, v as u8]
}

/// A `rtw89_fw_txpwr_byrate_entry` (fw.h:5832-5841).
#[allow(clippy::too_many_arguments)]
fn byr(band: u8, nss: u8, rs: u8, shf: u8, len: u8, data: u32, bw: u8, ofdma: u8) -> [u8; 11] {
    let d = data.to_le_bytes();
    [band, nss, rs, shf, len, d[0], d[1], d[2], d[3], bw, ofdma]
}

/// A `rtw89_fw_regd_entry` (fw.h:5822-5829).
fn regd(alpha2: &[u8; 2], rules: [u8; 3], fmap: u32) -> [u8; 9] {
    let f = fmap.to_le_bytes();
    [
        alpha2[0], alpha2[1], rules[0], rules[1], rules[2], f[0], f[1], f[2], f[3],
    ]
}

#[test]
fn rfe_match_wins_over_default_and_last_match_wins() {
    // fw.c:1199-1206: a matching rfe is always taken, so the last match
    // wins; rfe 0 is taken only while nothing or another rfe 0 is held.
    let b = file(|b| {
        for (rfe, v) in [(0, 3), (1, 5), (1, 7), (0, 9)] {
            txpwr(b, ID_TXPWR_LMT_2G, 0, rfe, 7, &[&lmt(0, 0, 0, 0, 2, 0, v)]);
        }
    });
    let at_rfe = |rfe| {
        tx_power_tables(&b, 0, rfe)
            .unwrap()
            .lmt(HwBand::G2, 0, 0, 0, 0, 2, 0)
    };
    assert_eq!(
        at_rfe(1),
        7,
        "the last rfe 1 element, not the rfe 0 after it"
    );
    assert_eq!(at_rfe(2), 9, "no match: the last default element");
}

#[test]
fn foreign_rfe_and_aid_are_skipped() {
    // fw.c:1186-1187: a nonzero aid must be the chip's; fw.c:1208-1210: any
    // other rfe is skipped.
    let b = file(|b| {
        txpwr(b, ID_TXPWR_LMT_5G, 0, 1, 7, &[&lmt(0, 0, 1, 0, 1, 0, 5)]);
        txpwr(
            b,
            ID_TXPWR_LMT_5G,
            0x7102,
            1,
            7,
            &[&lmt(0, 0, 1, 0, 1, 0, 11)],
        );
        txpwr(b, ID_TXPWR_LMT_5G, 0, 3, 7, &[&lmt(0, 0, 1, 0, 1, 0, 13)]);
    });
    let t = tx_power_tables(&b, 0, 1).unwrap();
    assert_eq!(t.lmt(HwBand::G5, 0, 0, 1, 0, 1, 0), 5);
    let t = tx_power_tables(&b, 0x7102, 1).unwrap();
    assert_eq!(t.lmt(HwBand::G5, 0, 0, 1, 0, 1, 0), 11);
}

#[test]
fn lmt_entry_lands_at_its_indices() {
    let b = file(|b| {
        txpwr(b, ID_TXPWR_LMT_2G, 0, 0, 7, &[&lmt(0, 1, 2, 0, 2, 5, 60)]);
        txpwr(b, ID_TXPWR_LMT_5G, 0, 0, 7, &[&lmt(3, 1, 2, 1, 15, 52, -4)]);
        // ru, nt, regd, ch_idx, v (fw.h:5876-5891).
        txpwr(b, ID_TXPWR_LMT_RU_2G, 0, 0, 5, &[&[7, 1, 3, 13, 40]]);
        txpwr(b, ID_TXPWR_LMT_RU_5G, 0, 0, 5, &[&[2, 0, 14, 38, 30]]);
        // band, tx_shape_rs, regd, v, reg6_pwr (fw.h:5906-5912).
        txpwr(b, ID_TX_SHAPE_LMT, 0, 0, 5, &[&[1, 1, 2, 3, 0]]);
    });
    let t = tx_power_tables(&b, 0, 1).unwrap();
    assert_eq!(t.lmt(HwBand::G2, 0, 1, 2, 0, 2, 5), 60);
    assert_eq!(t.lmt_2ghz[0][1][2][0][2][5], 60);
    assert_eq!(t.lmt(HwBand::G2, 0, 1, 2, 0, 2, 4), 0);
    assert_eq!(t.lmt(HwBand::G2, 0, 0, 2, 0, 2, 5), 0);
    assert_eq!(t.lmt(HwBand::G5, 3, 1, 2, 1, 15, 52), -4);
    assert_eq!(t.lmt_ru(HwBand::G2, 7, 1, 3, 13), 40);
    assert_eq!(t.lmt_ru(HwBand::G5, 2, 0, 14, 38), 30);
    assert_eq!(t.tx_shape(HwBand::G5, 1, 2), 3);
    assert_eq!(t.tx_shape(HwBand::G2, 1, 2), 0);
    // core.h:8946-8947: a rate section past OFDM reads OFDM's.
    assert_eq!(t.tx_shape(HwBand::G5, 2, 2), 3);
    // Past the tables: no value.
    assert_eq!(t.lmt(HwBand::G2, 2, 0, 0, 0, 0, 0), 0);
    assert_eq!(t.lmt(HwBand::G6, 0, 0, 0, 0, 0, 0), 0);
    assert_eq!(t.lmt(HwBand::G5, 0, 0, 0, 0, 0, 53), 0);
    assert_eq!(t.lmt_ru(HwBand::G2, 8, 0, 0, 0), 0);
}

#[test]
fn out_of_range_entries_are_skipped() {
    // fw.c:12143-12164 and its siblings: each index is checked against its
    // dimension.
    let b = file(|b| {
        txpwr(
            b,
            ID_TXPWR_LMT_2G,
            0,
            0,
            7,
            &[
                &lmt(2, 0, 0, 0, 0, 0, 1),  // bw 40 is the last 2G bw
                &lmt(0, 2, 0, 0, 0, 0, 1),  // nt
                &lmt(0, 0, 3, 0, 0, 0, 1),  // rs
                &lmt(0, 0, 0, 2, 0, 0, 1),  // bf
                &lmt(0, 0, 0, 0, 16, 0, 1), // regd
                &lmt(0, 0, 0, 0, 0, 14, 1), // ch_idx
                &lmt(1, 1, 2, 1, 15, 13, 9),
            ],
        );
        txpwr(
            b,
            ID_TXPWR_LMT_RU_5G,
            0,
            0,
            5,
            &[&[8, 0, 0, 0, 1], &[0, 0, 0, 53, 1]],
        );
        txpwr(
            b,
            ID_TX_SHAPE_LMT,
            0,
            0,
            5,
            &[
                &[3, 0, 0, 1, 0],
                &[0, 2, 0, 1, 0],
                &[0, 0, 16, 1, 0],
                &[0, 0, 0, 1, 3],
            ],
        );
    });
    let t = tx_power_tables(&b, 0, 0).unwrap();
    assert_eq!(t.lmt(HwBand::G2, 1, 1, 2, 1, 15, 13), 9);
    let set = t
        .lmt_2ghz
        .iter()
        .flatten()
        .flatten()
        .flatten()
        .flatten()
        .flatten()
        .filter(|&&v| v != 0)
        .count();
    assert_eq!(set, 1);
    assert!(
        t.lmt_ru_5ghz
            .iter()
            .flatten()
            .flatten()
            .flatten()
            .all(|&v| v == 0)
    );
    assert!(
        t.tx_shape_lmt
            .iter()
            .flatten()
            .flatten()
            .flatten()
            .all(|&v| v == 0)
    );
}

#[test]
fn short_entry_is_zero_extended_and_long_entry_with_garbage_is_skipped() {
    // core.h:5364-5379 copies min(sizeof, ent_sz) into a zeroed entry;
    // fw.c:12050-12067 accepts a longer entry only if its extra bytes are 0.
    let b = file(|b| {
        // The pinned file's TX_SHAPE_LMT entries are 4 bytes: no reg6_pwr.
        txpwr(b, ID_TX_SHAPE_LMT, 0, 0, 4, &[&[0, 1, 2, 3]]);
        txpwr(
            b,
            ID_TXPWR_LMT_2G,
            0,
            0,
            8,
            &[&[0, 0, 1, 0, 2, 0, 21, 0], &[0, 0, 1, 0, 2, 1, 22, 0x5a]],
        );
        // A 9-byte by-rate entry: bw and ofdma read as 0.
        txpwr(
            b,
            ID_TXPWR_BYRATE,
            0,
            0,
            9,
            &[&byr(1, 0, RS_OFDM, 0, 1, 0x22, 0, 0)[..9]],
        );
    });
    let t = tx_power_tables(&b, 0, 0).unwrap();
    assert_eq!(t.tx_shape(HwBand::G2, 1, 2), 3);
    assert_eq!(t.tx_shape_lmt[0][1][2][0], 3);
    assert_eq!(t.lmt(HwBand::G2, 0, 0, 1, 0, 2, 0), 21);
    assert_eq!(
        t.lmt(HwBand::G2, 0, 0, 1, 0, 2, 1),
        0,
        "garbage past the entry"
    );
    assert_eq!(t.byrate(HwBand::G5).ofdm[0], 0x22);
}

#[test]
fn byrate_bytes_go_to_shf_plus_i() {
    // fw.c:12125-12140: byte i of data (LE) lands at index shf + i.
    let b = file(|b| {
        txpwr(
            b,
            ID_TXPWR_BYRATE,
            0,
            0,
            11,
            &[
                &byr(0, 0, RS_OFDM, 4, 2, 0x0000_3c38, 0, 0),
                &byr(1, 1, RS_MCS, 10, 6, 0x0403_0201, 0, 1),
                &byr(0, 1, RS_HEDCM, 0, 4, 0xfcfd_feff, 0, 0),
                &byr(0, 0, RS_CCK, 0, 4, 0x4443_4241, 1, 0),
                &byr(0, 0, RS_OFFSET, 0, 5, 0x0403_0201, 0, 0),
                // Past the section: skipped (fw.c:12083-12086).
                &byr(0, 0, RS_OFDM, 7, 2, 0x7777, 0, 0),
                &byr(0, 0, RS_CCK, 0, 1, 0x55, 5, 0),
                &byr(3, 0, RS_CCK, 0, 1, 0x55, 0, 0),
                &byr(0, 2, RS_HEDCM, 0, 1, 0x55, 0, 0),
                &byr(0, 0, 5, 0, 1, 0x55, 0, 0),
            ],
        );
    });
    let t = tx_power_tables(&b, 0, 0).unwrap();
    let g2 = t.byrate(HwBand::G2);
    assert_eq!(g2.ofdm, [0, 0, 0, 0, 0x38, 0x3c, 0, 0]);
    // Bytes past the four in data are 0 (data >>= 8 per byte).
    assert_eq!(t.byrate(HwBand::G5).mcs[1][1][10..16], [1, 2, 3, 4, 0, 0]);
    assert_eq!(g2.hedcm[0][1], [-1, -2, -3, -4]);
    assert_eq!(g2.cck, [0; 4], "bw 1 is not the 20 MHz table");
    assert_eq!(t.byr[0][1].cck, [0x41, 0x42, 0x43, 0x44]);
    assert_eq!(g2.offset, [1, 2, 3, 4, 0, 0, 0, 0]);
    assert_eq!(t.byr[0][1].ofdm, [0; 8]);
    assert!(t.byr[2].iter().all(|r| *r == ByRate::default()));
}

#[test]
fn regd_entries_are_listed() {
    // fw.c:1319-1366: rules past THAILAND become NA, a short entry takes NA
    // rules and an empty fmap, and fmap keeps TAS and DAG only.
    let b = file(|b| {
        txpwr(b, ID_REGD, 0, 0, 9, &[&regd(b"JP", [3, 3, 3], 0)]);
        txpwr(
            b,
            ID_REGD,
            0,
            0,
            9,
            &[
                &regd(b"US", [2, 2, 2], 0x1),
                &regd(b"XX", [16, 0xff, 1], 0xffff_fffe),
            ],
        );
    });
    let t = tx_power_tables(&b, 0, 1).unwrap();
    assert_eq!(
        t.regd_map(),
        [
            RegdEntry {
                alpha2: *b"US",
                rule: [2, 2, 2],
                fmap: 1
            },
            RegdEntry {
                alpha2: *b"XX",
                rule: [REGD_NA, REGD_NA, 1],
                fmap: 2
            },
        ],
        "the later element replaces the earlier"
    );
    let short = file(|b| txpwr(b, ID_REGD, 0, 0, 3, &[&[b'D', b'E', 1]]));
    assert_eq!(
        tx_power_tables(&short, 0, 1).unwrap().regd_map(),
        [RegdEntry {
            alpha2: *b"DE",
            rule: [1, REGD_NA, REGD_NA],
            fmap: 0
        }]
    );
    let long = file(|b| {
        let mut e = regd(b"FR", [1, 1, 1], 0).to_vec();
        e.push(0x5a);
        txpwr(b, ID_REGD, 0, 0, 10, &[&e]);
    });
    assert_eq!(
        tx_power_tables(&long, 0, 1).unwrap().regd_map()[0].alpha2,
        *b"FR",
        "REGD entries take no zero-extension check"
    );
}

#[test]
fn a_regd_element_over_255_entries_is_ignored() {
    // fw.c:1378-1385: the element is ignored and an earlier one kept.
    let b = file(|b| {
        txpwr(b, ID_REGD, 0, 0, 9, &[&regd(b"US", [2, 2, 2], 0)]);
        claiming(b, ID_REGD, 0, 256, &[]);
    });
    assert_eq!(
        tx_power_tables(&b, 0, 1).unwrap().regd_map()[0].alpha2,
        *b"US"
    );
}

#[test]
fn entries_running_past_their_element_are_refused() {
    // Linux reads num_ents * ent_sz bytes without checking the element's
    // size; Stela refuses the selected element instead.
    let b = file(|b| claiming(b, ID_TXPWR_LMT_2G, 7, 2, &[0; 7]));
    assert_eq!(
        tx_power_tables(&b, 0, 1).err(),
        Some(AxError::TxPower(
            "TXPWR_LMT_2GHZ entries run past their element"
        ))
    );
    let b = file(|b| claiming(b, ID_REGD, 9, 2, &[0; 9]));
    assert_eq!(
        tx_power_tables(&b, 0, 1).err(),
        Some(AxError::TxPower("REGD entries run past their element"))
    );
    // A selected element is checked when its table is read, so a default
    // replaced by a match is never refused.
    let b = file(|b| {
        claiming(b, ID_TXPWR_LMT_2G, 7, 2, &[0; 7]);
        txpwr(b, ID_TXPWR_LMT_2G, 0, 1, 7, &[]);
    });
    assert!(tx_power_tables(&b, 0, 1).is_ok());
    // fw.c:1400-1405: a zero entry size leaves every entry unread.
    let b = file(|b| claiming(b, ID_REGD, 0, 1, &[]));
    assert_eq!(
        tx_power_tables(&b, 0, 1).err(),
        Some(AxError::TxPower("REGD element has invalid entries"))
    );
}

#[test]
fn a_zero_entry_size_leaves_a_table_empty() {
    // core.h:5375-5379: the cursor never advances past data.
    let b = file(|b| {
        txpwr(b, ID_TXPWR_LMT_2G, 0, 0, 7, &[&lmt(0, 0, 0, 0, 0, 0, 1)]);
        claiming(b, ID_TXPWR_LMT_2G, 0, 5, &[]);
    });
    let t = tx_power_tables(&b, 0, 1).unwrap();
    assert!(
        t.lmt_2ghz
            .iter()
            .flatten()
            .flatten()
            .flatten()
            .flatten()
            .flatten()
            .all(|&v| v == 0)
    );
}

#[test]
fn missing_element_is_named() {
    let names = [
        "firmware lacks TXPWR_BYRATE",
        "firmware lacks TXPWR_LMT_2GHZ",
        "firmware lacks TXPWR_LMT_5GHZ",
        "firmware lacks TXPWR_LMT_RU_2GHZ",
        "firmware lacks TXPWR_LMT_RU_5GHZ",
        "firmware lacks TX_SHAPE_LMT",
        "firmware lacks REGD",
    ];
    for (id, name) in NEEDED.into_iter().zip(names) {
        let b = file_without(Some(id), |_| {});
        assert_eq!(
            tx_power_tables(&b, 0, 1).err(),
            Some(AxError::TxPower(name))
        );
    }
    // An element only for another rfe is as good as none.
    let b = file_without(Some(ID_TXPWR_LMT_5G), |b| {
        txpwr(b, ID_TXPWR_LMT_5G, 0, 5, 7, &[]);
    });
    assert_eq!(
        tx_power_tables(&b, 0, 1).err(),
        Some(AxError::TxPower("firmware lacks TXPWR_LMT_5GHZ"))
    );
    assert_eq!(
        tx_power_tables(&[0u8; 64], 0, 1).err(),
        Some(AxError::TxPower("firmware has no element container"))
    );
}

#[test]
fn tx_power_error_names_its_subsystem() {
    assert_eq!(
        AxError::TxPower("firmware lacks REGD").to_string(),
        "tx power: firmware lacks REGD"
    );
}

#[test]
fn taken_names_each_selected_element() {
    let b = file(|b| txpwr(b, ID_TXPWR_LMT_RU_5G, 0, 1, 5, &[&[0, 0, 0, 0, 1]]));
    let t = tx_power_tables(&b, 0, 1).unwrap();
    let ids: Vec<u32> = t.taken.iter().map(|e| e.unwrap().id).collect();
    assert_eq!(ids, NEEDED);
    let ru5 = t.taken[4].unwrap();
    assert_eq!((ru5.rfe_type, ru5.size), (1, 5));
    assert_eq!(
        &b[ru5.offset..ru5.offset + 4],
        &ID_TXPWR_LMT_RU_5G.to_le_bytes()
    );
}

#[test]
#[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES"]
fn real_8852c_fw_2_carries_every_tx_power_table() {
    use sha2::{Digest, Sha256};
    let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
    let b = std::fs::read(root.join("rtw89/rtw8852c_fw-2.bin")).unwrap();
    assert_eq!(b.len(), 2_644_326);
    // The file carries rfe 0 and rfe 5 elements only, so rfe 1 (the owner's
    // chip) takes every default.
    let t = tx_power_tables(&b, 0, 1).unwrap();
    let digests: Vec<(u32, usize, u8, std::string::String)> = t
        .taken
        .iter()
        .map(|e| {
            let e = e.unwrap();
            let bytes = &b[e.offset..e.offset + 32 + e.size];
            let hex = Sha256::digest(bytes)
                .iter()
                .map(|x| std::format!("{x:02x}"))
                .collect();
            (e.id, e.offset, e.rfe_type, hex)
        })
        .collect();
    let want: [(u32, usize, &str); 7] = [
        (
            9,
            2_225_968,
            "59c46c280a48222bd79e86ab96608c3827f246fef192eaca93cbede120113f6c",
        ),
        (
            10,
            2_259_200,
            "ca94e02ee21e84b1fd4f71a533d3d399fe4623d569deaaa834e85e90b89b3748",
        ),
        (
            11,
            2_275_696,
            "617c97db33eeeab95c0027a4f611bbe7db3f12b4bea2d4aa09904a34d5c952f9",
        ),
        (
            13,
            2_390_240,
            "dd4ba55f5843aab8d151ff64e06f3875d42936288ad53390880e270faac21e8d",
        ),
        (
            14,
            2_396_160,
            "9ec4faebaf1a61e9d1c5960e405f03d25f482e66eb70fa902f7824aafa39c6ca",
        ),
        (
            16,
            2_632_400,
            "68532463e4a3b001573dd478eea2daf883fa6f06a8d96efbe5d755cf186edc7e",
        ),
        (
            20,
            2_634_272,
            "cd9a0310c638c4d1cdb483aebbe5fbd6a62583404f0d94b8fa67f2d8a9611886",
        ),
    ];
    for ((id, offset, rfe, hex), (wid, woffset, whex)) in digests.iter().zip(want) {
        assert_eq!((*id, *offset, *rfe, hex.as_str()), (wid, woffset, 0, whex));
    }
    assert_eq!(t.regd_map().len(), 242);
    let us = t
        .regd_map()
        .iter()
        .find(|e| e.alpha2 == *b"US")
        .expect("US in the REGD map");
    assert_eq!(us.rule[0], 2, "US is FCC on 2.4 GHz");
    // 2G, 20 MHz, 1TX, OFDM, non-BF, FCC, channel 1.
    assert!(t.lmt(HwBand::G2, 0, 0, RS_OFDM, 0, 2, 0) > 0);
    assert!(t.byrate(HwBand::G2).cck.iter().any(|&v| v != 0));
}

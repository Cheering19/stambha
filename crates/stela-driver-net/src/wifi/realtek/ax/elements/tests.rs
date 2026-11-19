// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
extern crate std;
use super::*;
use alloc::vec::Vec;

/// An MFW container with one entry at `shift` of `size` bytes, padded to
/// where its elements begin (ALIGN(shift + size, 16), fw.c:1579-1580).
pub(crate) fn container(shift: u32, size: u32) -> Vec<u8> {
    let mut b = alloc::vec![0u8; 32];
    b[0] = 0xff;
    b[1] = 1;
    b[16 + 4..16 + 8].copy_from_slice(&shift.to_le_bytes());
    b[16 + 8..16 + 12].copy_from_slice(&size.to_le_bytes());
    let end = (shift + size) as usize;
    b.resize(end.next_multiple_of(16), 0xee);
    b
}

/// Append one element: id, size, ver, aid, the union's 8-byte head
/// (`idx` first for reg2), then `body`, padded to the next element.
pub(crate) fn element(b: &mut Vec<u8>, id: u32, ver: [u8; 4], aid: u16, idx: u8, body: &[u8]) {
    assert_eq!(b.len() % 16, 0);
    b.extend_from_slice(&id.to_le_bytes());
    b.extend_from_slice(&(body.len() as u32).to_le_bytes());
    b.extend_from_slice(&ver);
    b.extend_from_slice(&aid.to_le_bytes());
    b.extend_from_slice(&[0; 2 + 4 + 4]);
    b.push(idx);
    b.extend_from_slice(&[0; 7]);
    b.extend_from_slice(body);
    let padded = b.len().next_multiple_of(16);
    b.resize(padded, 0);
}

pub(crate) fn pairs(regs: &[(u32, u32)]) -> Vec<u8> {
    regs.iter()
        .flat_map(|(a, d)| a.to_le_bytes().into_iter().chain(d.to_le_bytes()))
        .collect()
}

fn listed(table: &Reg2<'_>) -> Vec<(u32, u32)> {
    table.iter().collect()
}

#[test]
fn tables_are_found_after_the_last_container_entry_and_borrowed() {
    let mut b = container(32, 5);
    assert_eq!(b.len(), 48);
    element(
        &mut b,
        ID_BB_REG,
        [0, 0x28, 0, 0],
        0,
        0,
        &pairs(&[(0x800, 1), (0x804, 2)]),
    );
    element(&mut b, 9, [0; 4], 0, 0, &[1, 2, 3]); // TXPWR_BYRATE: walked past
    element(&mut b, ID_BB_GAIN, [1; 4], 0, 0, &pairs(&[(0x4a00, 3)]));
    element(&mut b, ID_RF_NCTL, [2; 4], 0, 0, &pairs(&[(0x8008, 0)]));
    let t = phy_tables(&b, 0).unwrap();
    let bb = t.bb.unwrap();
    assert_eq!(bb.version, [0, 0x28, 0, 0]);
    assert_eq!(listed(&bb), [(0x800, 1), (0x804, 2)]);
    assert_eq!(bb.len(), 2);
    assert_eq!(listed(&t.bb_gain.unwrap()), [(0x4a00, 3)]);
    assert_eq!(listed(&t.nctl.unwrap()), [(0x8008, 0)]);
    assert!(t.radio.iter().all(Option::is_none));
    assert_eq!(t.stopped, None);
}

#[test]
fn a_radio_takes_its_slot_from_idx_and_its_path_from_its_id() {
    // fw.c:1120-1128 and 1159-1160: the 8852C file puts RADIO_A at idx 1
    // and RADIO_B at idx 0, so slot 0 (configured first) is path B.
    let mut b = container(32, 16);
    element(&mut b, ID_RADIO_A, [0; 4], 0, 1, &pairs(&[(0xa, 0xa)]));
    element(&mut b, ID_RADIO_B, [0; 4], 0, 0, &pairs(&[(0xb, 0xb)]));
    let t = phy_tables(&b, 0).unwrap();
    let slot0 = t.radio[0].unwrap();
    let slot1 = t.radio[1].unwrap();
    assert_eq!(
        (slot0.path, listed(&slot0.table)),
        (RfPath::B, alloc::vec![(0xb, 0xb)])
    );
    assert_eq!(
        (slot1.path, listed(&slot1.table)),
        (RfPath::A, alloc::vec![(0xa, 0xa)])
    );
    assert!(t.radio[2].is_none() && t.radio[3].is_none());
}

#[test]
fn the_first_element_of_a_table_wins() {
    // fw.c:1139-1140: an element whose table is already set is ignored.
    let mut b = container(32, 16);
    element(&mut b, ID_BB_REG, [1; 4], 0, 0, &pairs(&[(1, 1)]));
    element(&mut b, ID_BB_REG, [2; 4], 0, 0, &pairs(&[(2, 2)]));
    element(&mut b, ID_RADIO_A, [1; 4], 0, 0, &pairs(&[(3, 3)]));
    element(&mut b, ID_RADIO_B, [2; 4], 0, 0, &pairs(&[(4, 4)]));
    let t = phy_tables(&b, 0).unwrap();
    assert_eq!(t.bb.unwrap().version, [1; 4]);
    let slot0 = t.radio[0].unwrap();
    assert_eq!((slot0.path, slot0.table.version), (RfPath::A, [1; 4]));
}

#[test]
fn an_element_for_another_chip_aid_is_ignored() {
    // fw.c:1136-1138: a nonzero aid must equal the chip's; AX chips keep
    // aid 0 (core.c:7151-7162 sets it on BE only).
    let mut b = container(32, 16);
    element(&mut b, ID_BB_REG, [7; 4], 0x7102, 0, &pairs(&[(7, 7)]));
    element(&mut b, ID_BB_REG, [0; 4], 0, 0, &pairs(&[(1, 1)]));
    assert_eq!(phy_tables(&b, 0).unwrap().bb.unwrap().version, [0; 4]);
    assert_eq!(phy_tables(&b, 0x7102).unwrap().bb.unwrap().version, [7; 4]);
}

#[test]
fn a_size_not_a_multiple_of_a_pair_drops_the_partial_pair() {
    // fw.c:1146: n_regs = size / sizeof(regs[0]).
    let mut b = container(32, 16);
    let mut body = pairs(&[(5, 6)]);
    body.extend_from_slice(&[0xaa; 5]);
    element(&mut b, ID_RF_NCTL, [0; 4], 0, 0, &body);
    assert_eq!(listed(&phy_tables(&b, 0).unwrap().nctl.unwrap()), [(5, 6)]);
}

#[test]
fn a_radio_slot_past_the_last_path_is_refused() {
    // Linux would index rf_radio[idx] out of bounds (fw.c:1125).
    let mut b = container(32, 16);
    element(&mut b, ID_RADIO_C, [0; 4], 0, 4, &pairs(&[(1, 1)]));
    assert_eq!(
        phy_tables(&b, 0).err(),
        Some(ElementError::RadioSlot { offset: 48, idx: 4 })
    );
}

#[test]
fn an_element_running_past_the_file_stops_the_walk_and_keeps_what_was_found() {
    // fw.c:1587-1591 stops at an oversized element. Stela also stops when
    // the header-inclusive end passes the file, so no byte past it is read.
    let mut b = container(32, 16);
    element(&mut b, ID_BB_REG, [0; 4], 0, 0, &pairs(&[(1, 1)]));
    let at = b.len();
    element(&mut b, ID_BB_GAIN, [0; 4], 0, 0, &pairs(&[(2, 2), (3, 3)]));
    b.truncate(at + 32 + 8);
    let t = phy_tables(&b, 0).unwrap();
    assert!(t.bb.is_some());
    assert!(t.bb_gain.is_none());
    assert_eq!(t.stopped, Some(at));
}

#[test]
fn a_file_with_no_container_has_no_tables() {
    assert_eq!(
        phy_tables(&[0u8; 64], 0).err(),
        Some(ElementError::NotContainer)
    );
    assert_eq!(
        phy_tables(&[0xff; 8], 0).err(),
        Some(ElementError::NotContainer)
    );
    let mut empty = container(32, 16);
    empty[1] = 0;
    assert_eq!(
        phy_tables(&empty, 0).err(),
        Some(ElementError::EmptyContainer)
    );
    let mut short = alloc::vec![0u8; 16];
    short[0] = 0xff;
    short[1] = 2;
    assert_eq!(
        phy_tables(&short, 0).err(),
        Some(ElementError::Truncated {
            offset: 16,
            bytes: 32
        })
    );
}

#[test]
fn an_element_ending_exactly_at_the_end_of_the_file_is_kept() {
    let mut b = container(32, 16);
    element(&mut b, ID_BB_REG, [0; 4], 0, 0, &pairs(&[(1, 1), (2, 2)]));
    assert_eq!(b.len() % 16, 0, "the body already ends on the alignment");
    let t = phy_tables(&b, 0).unwrap();
    assert_eq!(listed(&t.bb.unwrap()), [(1, 1), (2, 2)]);
    assert_eq!(t.stopped, None);
}

#[test]
fn elements_follow_the_last_entry_not_the_furthest_one() {
    // rtw89_mfw_get_size (fw.c:687-688) uses info[fw_nr - 1].
    let mut b = alloc::vec![0u8; 48];
    b[0] = 0xff;
    b[1] = 2;
    for (entry, (shift, size)) in [(48u32, 100u32), (48, 16)].into_iter().enumerate() {
        let at = 16 + 16 * entry;
        b[at + 4..at + 8].copy_from_slice(&shift.to_le_bytes());
        b[at + 8..at + 12].copy_from_slice(&size.to_le_bytes());
    }
    b.resize(64, 0);
    element(&mut b, ID_RF_NCTL, [0; 4], 0, 0, &pairs(&[(9, 9)]));
    assert_eq!(listed(&phy_tables(&b, 0).unwrap().nctl.unwrap()), [(9, 9)]);
}

#[test]
fn a_container_end_that_wraps_to_zero_is_refused() {
    // Linux aligns in u32: 0xffff_fff0 + 8 rounds to 0, -EINVAL.
    let mut b = container(32, 16);
    b[20..24].copy_from_slice(&0xffff_fff0u32.to_le_bytes());
    b[24..28].copy_from_slice(&8u32.to_le_bytes());
    assert_eq!(phy_tables(&b, 0).err(), Some(ElementError::EmptyContainer));
}

#[test]
fn a_pair_index_past_the_table_is_none() {
    let mut b = container(32, 16);
    element(&mut b, ID_BB_REG, [0; 4], 0, 0, &pairs(&[(1, 1)]));
    let bb = phy_tables(&b, 0).unwrap().bb.unwrap();
    assert_eq!(bb.get(1), None);
    assert_eq!(bb.get(usize::MAX / 8), None);
    assert_eq!(bb.get(usize::MAX), None);
}

#[test]
fn a_container_with_no_elements_has_no_tables() {
    let b = container(32, 16);
    let t = phy_tables(&b, 0).unwrap();
    assert!(t.bb.is_none() && t.bb_gain.is_none() && t.nctl.is_none());
    assert!(t.radio.iter().all(Option::is_none));
    assert_eq!(t.stopped, None);
}

#[test]
#[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES"]
fn real_8852c_fw_2_carries_every_phy_table() {
    let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
    let b = std::fs::read(root.join("rtw89/rtw8852c_fw-2.bin")).unwrap();
    assert_eq!(b.len(), 2_644_326);
    let t = phy_tables(&b, 0).unwrap();
    assert_eq!(t.stopped, None);
    let ends = |r: &Reg2<'_>| (r.len(), r.get(0), r.get(r.len() - 1), r.version);
    assert_eq!(
        ends(&t.bb.unwrap()),
        (
            2061,
            Some((0xf0ff_0000, 0)),
            Some((0xf8, 0x2022_0610)),
            [0, 0x28, 0, 0]
        )
    );
    assert_eq!(
        ends(&t.bb_gain.unwrap()),
        (
            662,
            Some((0xf0ff_0000, 0)),
            Some((0x0107_003d, 0)),
            [0, 0x28, 0, 0]
        )
    );
    let slot0 = t.radio[0].unwrap();
    let slot1 = t.radio[1].unwrap();
    assert_eq!(slot0.path, RfPath::B);
    assert_eq!(
        ends(&slot0.table),
        (
            15713,
            Some((0xf001_0000, 0)),
            Some((0xfe, 0x63)),
            [0, 0x63, 0, 0]
        )
    );
    assert_eq!(slot1.path, RfPath::A);
    assert_eq!(
        ends(&slot1.table),
        (
            17604,
            Some((0xf001_0000, 0)),
            Some((0xfe, 0x63)),
            [0, 0x63, 0, 0]
        )
    );
    assert!(t.radio[2].is_none() && t.radio[3].is_none());
    assert_eq!(
        ends(&t.nctl.unwrap()),
        (1774, Some((0x8008, 0)), Some((0x8088, 0)), [0, 0x11, 0, 0])
    );
}

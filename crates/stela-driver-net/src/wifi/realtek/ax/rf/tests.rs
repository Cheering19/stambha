// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::{boxed::Box, vec::Vec};

/// A model answering SWSI from its RF file.
fn rf_model() -> RegisterModel {
    let mut m = RegisterModel::new();
    m.emulate_rf();
    m
}

/// The register-level trace: everything but the decoded `RfW`/`RfR`.
fn registers(m: &RegisterModel) -> Vec<Op> {
    m.trace
        .iter()
        .copied()
        .filter(|op| !matches!(op, Op::RfW(..) | Op::RfR(..)))
        .collect()
}

fn reads_of(m: &RegisterModel, reg: u32) -> usize {
    m.trace.iter().filter(|o| **o == Op::R32(reg)).count()
}

fn has_writes(m: &RegisterModel) -> bool {
    m.trace
        .iter()
        .any(|op| matches!(op, Op::W8(..) | Op::W16(..) | Op::W32(..)))
}

#[test]
fn swsi_full_write_matches_linux_sequence() {
    let mut m = rf_model();
    m.set_rf(1, 0x05, 0xABCDE);
    write_rf(&mut m, RfPath::B, 0x05, 0xFFFFF, 0).unwrap();
    assert_eq!(
        registers(&m),
        [
            Op::R32(0x1174C),
            Op::R32(0x1174C),
            Op::R32(0x10370),
            Op::W32(0x10370, 0x1050_0000),
        ]
    );
    assert_eq!(m.rf_ops(), [Op::RfW(1, 0x05, 0xFFFFF, 0)]);
    assert_eq!(m.get_rf(1, 0x05), 0);
}

#[test]
fn swsi_masked_write_sets_the_bit_mask_first() {
    let mut m = rf_model();
    m.set_rf(0, 0x42, 0x0_1234);
    write_rf(&mut m, RfPath::A, 0x42, 0x80000, 1).unwrap();
    assert_eq!(
        registers(&m),
        [
            Op::R32(0x1174C),
            Op::R32(0x1174C),
            Op::R32(0x10374),
            Op::W32(0x10374, 0x0008_0000),
            Op::R32(0x10370),
            Op::W32(0x10370, 0x8428_0000),
        ]
    );
    // The value is shifted to the mask, as SWSI carries it.
    assert_eq!(m.rf_ops(), [Op::RfW(0, 0x42, 0x80000, 0x80000)]);
    assert_eq!(m.get_rf(0, 0x42), 0x8_1234);
}

#[test]
fn swsi_masked_write_is_not_cut_to_the_mask() {
    // phy.c:1172-1177: data is shifted and cut to 20 bits, never ANDed with
    // the mask; the chip applies the mask.
    let mut m = rf_model();
    write_rf(&mut m, RfPath::A, 0x8F, 0xE00, 0x1F).unwrap();
    assert_eq!(m.trace.last(), Some(&Op::RfW(0, 0x8F, 0xE00, 0x3E00)));
    assert!(m.trace.contains(&Op::W32(0x10370, 0x88F0_3E00)));
    assert_eq!(m.get_rf(0, 0x8F), 0xE00);
}

#[test]
fn swsi_read_matches_linux_sequence() {
    let mut m = rf_model();
    m.set_rf(0, 0x18, 0x01001);
    assert_eq!(read_rf(&mut m, RfPath::A, 0x18, 0xFFFFF), Ok(0x01001));
    assert_eq!(
        registers(&m),
        [
            Op::R32(0x1174C),
            Op::R32(0x1174C),
            Op::R32(0x10378),
            Op::W32(0x10378, 0x018),
            Op::Delay(2),
            Op::R32(0x1174C),
            Op::R32(0x1174C),
        ]
    );
    assert_eq!(m.rf_ops(), [Op::RfR(0, 0x18), Op::Delay(2)]);

    let mut m = rf_model();
    m.set_rf(1, 0x18, 0x01001);
    assert_eq!(read_rf(&mut m, RfPath::B, 0x18, 0xFFFFF), Ok(0x01001));
    assert!(m.trace.contains(&Op::W32(0x10378, 0x118)));
    assert_eq!(m.rf_ops(), [Op::RfR(1, 0x18), Op::Delay(2)]);
}

#[test]
fn swsi_read_returns_the_masked_field() {
    let mut m = rf_model();
    m.set_rf(1, 0x00, 0x337E1);
    assert_eq!(read_rf(&mut m, RfPath::B, 0x00, 0xF0000), Ok(0x3));
    // A hook answers when the read is latched.
    m.on_rf_read(1, 0x1C, Box::new(|_: &mut RegisterModel| 0x8));
    assert_eq!(read_rf(&mut m, RfPath::B, 0x1C, 0x8), Ok(1));
}

#[test]
fn direct_write_is_masked_and_followed_by_a_microsecond() {
    let mut m = rf_model();
    m.set32(0x1F0C0, 0xFFF0_0000);
    write_rf(&mut m, RfPath::B, 0x10030, 0xFFFFF, 0x7F003).unwrap();
    assert_eq!(
        m.trace,
        [
            Op::R32(0x1F0C0),
            Op::W32(0x1F0C0, 0xFFF7_F003),
            Op::Delay(1),
        ]
    );
}

#[test]
fn direct_read_has_no_delay() {
    let mut m = rf_model();
    m.set32(0x1E060, 0x0001_1124);
    assert_eq!(read_rf(&mut m, RfPath::A, 0x10018, 0xFFFFF), Ok(0x11124));
    assert_eq!(m.trace, [Op::R32(0x1E060)]);
}

#[test]
fn swsi_stuck_busy_fails_without_writing() {
    let busy = Err(AxError::Timeout {
        register: 0x1174C,
        mask: 0x0300_0000,
    });
    let mut m = rf_model();
    m.swsi_status = 0x0100_0000;
    assert_eq!(
        write_rf(&mut m, RfPath::A, 0x05, 0xFFFFF, 0),
        busy.map(|_: u32| ())
    );
    // atomic(1, 30): 30_000 / 1001 + 2 samples, each one read because b24
    // short-circuits the b25 read.
    assert_eq!(reads_of(&m, 0x1174C), 31);
    assert!(!has_writes(&m));

    let mut m = rf_model();
    m.swsi_status = 0x0100_0000;
    assert_eq!(read_rf(&mut m, RfPath::B, 0x18, 0xFFFFF), busy);
    assert_eq!(reads_of(&m, 0x1174C), 31);
    assert!(!has_writes(&m));
}

#[test]
fn swsi_read_busy_checks_both_bits() {
    // b24 clear, so each sample also reads b25 (phy.c:922-926).
    let mut m = rf_model();
    m.swsi_status = 0x0200_0000;
    assert!(read_rf(&mut m, RfPath::A, 0x18, 0xFFFFF).is_err());
    assert_eq!(reads_of(&m, 0x1174C), 62);
    assert!(!has_writes(&m));
}

#[test]
fn swsi_read_done_never_set_fails() {
    let mut m = rf_model();
    // SWSI idle, but R_DATA_DONE (b26) never sets after the latch.
    m.on_read(0x1174C, Box::new(|_: &mut RegisterModel| 0));
    assert_eq!(
        read_rf(&mut m, RfPath::A, 0x18, 0xFFFFF),
        Err(AxError::Timeout {
            register: 0x1174C,
            mask: 0x0400_0000
        })
    );
    // Two idle reads, then the 31-sample read-done wait.
    assert_eq!(reads_of(&m, 0x1174C), 2 + 31);
    assert!(m.trace.contains(&Op::RfR(0, 0x18)));
}

#[test]
fn path_c_is_refused_untouched() {
    for path in [RfPath::C, RfPath::D] {
        let mut m = rf_model();
        assert!(matches!(
            write_rf(&mut m, path, 0x05, 0xFFFFF, 0),
            Err(AxError::Phy(_))
        ));
        assert!(matches!(
            read_rf(&mut m, path, 0x10005, 0xFFFFF),
            Err(AxError::Phy(_))
        ));
        assert_eq!(m.trace, Vec::new());
    }
    assert_eq!(
        write_rf(&mut rf_model(), RfPath::C, 0, 0, 0),
        Err(AxError::Phy("RF path past the chip's two"))
    );
}

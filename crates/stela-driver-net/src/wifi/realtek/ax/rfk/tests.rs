// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::{boxed::Box, vec, vec::Vec};

const F: u32 = 0xFFFFF;

/// A model whose RF 0x05 holds 0x12345 on both paths, whose RF 0x1C has b3
/// (RCK done) set when `done`, and whose RF 0x1B reads `0x3C00 | stored`, so
/// its [14:10] field is 0x0F after the 0x240 trigger.
fn rck_model(done: bool) -> RegisterModel {
    let mut m = RegisterModel::new();
    m.emulate_rf();
    for p in 0..2u8 {
        m.set_rf(p, 0x05, 0x12345);
        m.set_rf(p, 0x1C, if done { 0x8 } else { 0 });
        m.on_rf_read(p, 0x1B, Box::new(move |m| 0x3C00 | m.get_rf(p, 0x1B)));
    }
    m
}

/// Only the decoded SWSI accesses.
fn rf_only(m: &RegisterModel) -> Vec<Op> {
    m.rf_ops()
        .into_iter()
        .filter(|op| matches!(op, Op::RfW(..) | Op::RfR(..)))
        .collect()
}

/// `_rck` for path `p` at the RF level when the first poll sample is done,
/// with each SWSI read's own 2 us settle (phy.c:970).
fn rck_ops(p: u8) -> Vec<Op> {
    vec![
        Op::RfR(p, 0x05),
        Op::Delay(2),
        Op::RfW(p, 0x05, 0x1, 0),
        Op::RfW(p, 0x00, 0xF0000, 0x30000),
        Op::RfR(p, 0x00),
        Op::Delay(2),
        Op::RfW(p, 0x1B, F, 0x240),
        Op::RfR(p, 0x1C),
        Op::Delay(2),
        Op::RfR(p, 0x1B),
        Op::Delay(2),
        Op::RfW(p, 0x1B, F, 0x0F),
        Op::RfW(p, 0x05, F, 0x12345),
        Op::RfR(p, 0x1B),
        Op::Delay(2),
        Op::RfR(p, 0x1C),
        Op::Delay(2),
    ]
}

#[test]
fn rck_matches_linux_sequence() {
    let mut m = rck_model(true);
    assert_eq!(rck(&mut m), Ok(([0x0F, 0x0F], [false, false])));
    let mut want = rck_ops(0);
    want.extend(rck_ops(1));
    assert_eq!(m.rf_ops(), want);
    // The brief's SWSI-only view, path A first.
    let mut swsi = Vec::new();
    for p in 0..2u8 {
        swsi.extend([
            Op::RfR(p, 0x05),
            Op::RfW(p, 0x05, 0x1, 0),
            Op::RfW(p, 0x00, 0xF0000, 0x30000),
            Op::RfR(p, 0x00),
            Op::RfW(p, 0x1B, F, 0x240),
            Op::RfR(p, 0x1C),
            Op::RfR(p, 0x1B),
            Op::RfW(p, 0x1B, F, 0x0F),
            Op::RfW(p, 0x05, F, 0x12345),
            Op::RfR(p, 0x1B),
            Op::RfR(p, 0x1C),
        ]);
    }
    assert_eq!(rf_only(&m), swsi);
    // RF 0x05 is restored whole. (RF 0x1B's unshifted write of the code is
    // pinned by the trace above; the model's read hook stores 0x3C00 | it
    // again on the closing debug read.)
    for p in 0..2u8 {
        assert_eq!(m.get_rf(p, 0x05), 0x12345);
        assert_eq!(m.get_rf(p, 0x1B), 0x3C0F);
        assert_eq!(m.get_rf(p, 0x00) & 0xF0000, 0x30000);
    }
}

#[test]
fn rck_timeout_is_recorded_and_writes_back() {
    let mut m = rck_model(false);
    assert_eq!(rck(&mut m), Ok(([0x0F, 0x0F], [true, true])));
    let ops = rf_only(&m);
    for p in 0..2u8 {
        let trigger = ops
            .iter()
            .position(|op| *op == Op::RfW(p, 0x1B, F, 0x240))
            .unwrap();
        let result = trigger
            + 1
            + ops[trigger + 1..]
                .iter()
                .position(|op| *op == Op::RfR(p, 0x1B))
                .unwrap();
        // read_poll_timeout_atomic(.., 2, 20, ..): 11 samples.
        assert_eq!(result - trigger - 1, 11);
        assert!(
            ops[trigger + 1..result]
                .iter()
                .all(|op| *op == Op::RfR(p, 0x1C))
        );
        // Then the write-back, the restore and the two debug reads anyway.
        assert_eq!(
            ops[result..result + 5],
            [
                Op::RfR(p, 0x1B),
                Op::RfW(p, 0x1B, F, 0x0F),
                Op::RfW(p, 0x05, F, 0x12345),
                Op::RfR(p, 0x1B),
                Op::RfR(p, 0x1C),
            ]
        );
        // 11 poll samples plus the closing debug read.
        assert_eq!(ops.iter().filter(|op| **op == Op::RfR(p, 0x1C)).count(), 12);
        assert_eq!(m.get_rf(p, 0x05), 0x12345);
    }
    // Ten 2 us waits between the samples of each path.
    let polls = m.rf_ops();
    let a = polls
        .iter()
        .position(|op| *op == Op::RfW(0, 0x1B, F, 0x240))
        .unwrap();
    let delays: Vec<u64> = polls[a + 1..]
        .iter()
        .take_while(|op| **op != Op::RfR(0, 0x1B))
        .filter_map(|op| match op {
            Op::Delay(us) => Some(*us),
            _ => None,
        })
        .collect();
    // Each sample's own 2 us SWSI settle, and a 2 us poll wait between them.
    assert_eq!(delays, vec![2; 11 + 10]);
}

#[test]
fn rck_swsi_stuck_fails() {
    let mut m = rck_model(true);
    m.swsi_status = 1 << 24;
    assert!(matches!(
        rck(&mut m),
        Err(AxError::Timeout {
            register: 0x1174C,
            ..
        })
    ));
    assert!(rf_only(&m).is_empty());
}

/// DRCK's MMIO registers: `R_DRCK` 0xC0C4, `R_DRCK_RES` 0xC0C8 and
/// `R_DRCK_FH` 0xC094 behind the BB base.
const DRCK: u32 = 0x1C0C4;
const DRCK_RES: u32 = 0x1C0C8;
const DRCK_FH: u32 = 0x1C094;

#[test]
fn drck_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    m.set32(DRCK_RES, 0x0006_8008);
    assert_eq!(drck(&mut m), Ok((0x0D, false)));
    assert_eq!(
        m.writes(),
        [
            Op::W32(DRCK, 0x40),
            Op::W32(DRCK, 0),
            Op::W32(DRCK_FH, 0x200),
            Op::Delay(1),
            Op::W32(DRCK_FH, 0),
            Op::W32(DRCK, 0),
            Op::W32(DRCK, 0x0D),
        ]
    );
    assert_eq!(
        m.trace,
        [
            Op::R32(DRCK),
            Op::W32(DRCK, 0x40),
            // The poll's one sample.
            Op::R32(DRCK_RES),
            Op::R32(DRCK),
            Op::W32(DRCK, 0),
            Op::R32(DRCK_FH),
            Op::W32(DRCK_FH, 0x200),
            Op::Delay(1),
            Op::R32(DRCK_FH),
            Op::W32(DRCK_FH, 0),
            Op::R32(DRCK_RES),
            Op::R32(DRCK),
            Op::W32(DRCK, 0),
            Op::R32(DRCK),
            Op::W32(DRCK, 0x0D),
            // The debug read of the whole register.
            Op::R32(DRCK),
        ]
    );
}

#[test]
fn drck_keeps_bits_outside_its_fields() {
    let mut m = RegisterModel::new();
    m.set32(DRCK, 0xFFFF_FFFF);
    m.set32(DRCK_FH, 0x8000_0001);
    m.set32(DRCK_RES, 0x000F_8008);
    assert_eq!(drck(&mut m), Ok((0x1F, false)));
    // b6 and b9 cleared, [4:0] the code.
    assert_eq!(m.get32(DRCK), 0xFFFF_FDBF);
    assert_eq!(m.get32(DRCK_FH), 0x8000_0001);
}

#[test]
fn drck_timeout_is_recorded() {
    let mut m = RegisterModel::new();
    m.set32(DRCK_RES, 0x0006_8000);
    assert_eq!(drck(&mut m), Ok((0x0D, true)));
    let samples = m
        .trace
        .iter()
        .take_while(|op| **op != Op::W32(DRCK, 0))
        .filter(|op| **op == Op::R32(DRCK_RES))
        .count();
    // read_poll_timeout_atomic(.., 1, 10000, ..): 9992 samples, 9991 waits.
    assert_eq!(samples, 9992);
    let waits = m
        .trace
        .iter()
        .take_while(|op| **op != Op::W32(DRCK, 0))
        .filter(|op| **op == Op::Delay(1))
        .count();
    assert_eq!(waits, 9991);
    // The rest of the sequence still runs after the enable and the waits.
    let writes = m.writes();
    assert_eq!(writes[0], Op::W32(DRCK, 0x40));
    assert_eq!(
        writes[1 + 9991..],
        [
            Op::W32(DRCK, 0),
            Op::W32(DRCK_FH, 0x200),
            Op::Delay(1),
            Op::W32(DRCK_FH, 0),
            Op::W32(DRCK, 0),
            Op::W32(DRCK, 0x0D),
        ]
    );
    assert_eq!(m.trace.last(), Some(&Op::R32(DRCK)));
}

/// ADDCK's MMIO registers: `R_ADDCK0`/`R_ADDCK1` 0xC0F4/0xC1F4, their
/// results `R_ADDCKR0`/`R_ADDCKR1` 0xC0FC/0xC1FC and reloads
/// `R_ADDCK0_RL`/`R_ADDCK1_RL` 0xC0F8/0xC1F8, behind the BB base.
const ADDCK: [u32; 2] = [0x1C0F4, 0x1C1F4];
const ADDCKR: [u32; 2] = [0x1C0FC, 0x1C1FC];
const ADDCK_RL: [u32; 2] = [0x1C0F8, 0x1C1F8];

/// One path of `_addck` from zero, its one poll sample met.
fn addck_ops(s: usize) -> Vec<Op> {
    let (r, res) = (ADDCK[s], ADDCKR[s]);
    vec![
        Op::R32(r),
        Op::W32(r, 0x4),
        Op::R32(r),
        Op::W32(r, 0x14),
        Op::R32(r),
        Op::W32(r, 0x4),
        Op::Delay(1),
        Op::R32(r),
        Op::W32(r, 0x104),
        Op::R32(res),
        Op::R32(r),
        Op::W32(r, 0x100),
    ]
}

#[test]
fn addck_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    m.set32(ADDCKR[0], 1);
    m.set32(ADDCKR[1], 1);
    assert_eq!(addck(&mut m), Ok([false, false]));
    let mut want = addck_ops(0);
    want.extend(addck_ops(1));
    assert_eq!(m.trace, want);
}

#[test]
fn addck_s1_timeout_is_recorded_on_path_b() {
    let mut m = RegisterModel::new();
    m.set32(ADDCKR[0], 1);
    assert_eq!(addck(&mut m), Ok([false, true]));
    let samples = m
        .trace
        .iter()
        .filter(|op| **op == Op::R32(ADDCKR[1]))
        .count();
    assert_eq!(samples, 9992);
    // S1's reset bit is still cleared after the timeout.
    assert_eq!(m.trace.last(), Some(&Op::W32(ADDCK[1], 0x100)));
}

#[test]
fn addck_s0_timeout_does_not_skip_s1() {
    let mut m = RegisterModel::new();
    m.set32(ADDCKR[1], 1);
    assert_eq!(addck(&mut m), Ok([true, false]));
    assert!(m.trace.ends_with(&addck_ops(1)));
}

#[test]
fn addck_reload_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    m.set32(ADDCKR[0], 0x0008_0601);
    m.set32(ADDCKR[1], 0x0008_0601);
    assert_eq!(addck_reload(&mut m), Ok([(0x201, 0x201); 2]));
    assert_eq!(
        m.writes(),
        [
            Op::W32(ADDCK[0], 0),
            Op::W32(ADDCK[1], 0),
            Op::W32(ADDCK_RL[0], 0x0804_0000),
            Op::W32(ADDCK_RL[0], 0x0806_0100),
            Op::W32(ADDCK_RL[0], 0x3806_0100),
            Op::W32(ADDCK_RL[1], 0x0804_0000),
            Op::W32(ADDCK_RL[1], 0x0806_0100),
            Op::W32(ADDCK_RL[1], 0x3806_0100),
        ]
    );
    // Backup: each path's trigger field cleared before its two result
    // reads.
    assert_eq!(
        m.trace[..10],
        [
            Op::R32(ADDCK[0]),
            Op::W32(ADDCK[0], 0),
            Op::R32(ADDCKR[0]),
            Op::R32(ADDCKR[0]),
            Op::R32(ADDCK[1]),
            Op::W32(ADDCK[1], 0),
            Op::R32(ADDCKR[1]),
            Op::R32(ADDCKR[1]),
            Op::R32(ADDCK_RL[0]),
            Op::W32(ADDCK_RL[0], 0x0804_0000),
        ]
    );
}

#[test]
fn addck_reload_splits_each_path_and_keeps_other_bits() {
    let mut m = RegisterModel::new();
    m.set32(ADDCK[0], 0xFFFF_FFFF);
    m.set32(ADDCKR[0], 0xFFF0_0000 | (0x155 << 10) | 0x2AA);
    m.set32(ADDCKR[1], (0x3FF << 10) | 0x001);
    m.set32(ADDCK_RL[1], 0xC000_00FF);
    assert_eq!(addck_reload(&mut m), Ok([(0x155, 0x2AA), (0x3FF, 0x001)]));
    // [9:8] cleared only.
    assert_eq!(m.get32(ADDCK[0]), 0xFFFF_FCFF);
    assert_eq!(
        m.get32(ADDCK_RL[0]),
        (3 << 28) | (0x155 << 18) | (0x2AA << 8)
    );
    assert_eq!(
        m.get32(ADDCK_RL[1]),
        0xC000_00FF | (3 << 28) | (0x3FF << 18) | (0x001 << 8)
    );
}

/// A DACK model: SWSI answered, the CMAC enabled and the scoreboard's
/// firmware byte as milestone 1 leaves it (for the coex notify), DRCK's and
/// both ADDCK polls met, and every DACK done bit set except S1's 0xC140 b31
/// when `s1_done` is false. S0's MSBK register 0xC05C reads the 0xC000
/// [4:1] selector in [31:24] (with its done bit b2), so `msbk[0][0][i] = i`;
/// S0's bias 0xC048 reads 0x554 ([11:2] = 0x155) and its DADCK 0xC060 reads
/// 0x7A in [31:24]. Every other DACK result reads 0.
fn dack_model(s1_done: bool) -> RegisterModel {
    let mut m = RegisterModel::new();
    m.emulate_rf();
    m.set32(0xC000, 0x4000_0000);
    m.set32(0x00AC, 0x8100_0000);
    m.set32(DRCK_RES, 0x8);
    m.set32(ADDCKR[0], 1);
    m.set32(ADDCKR[1], 1);
    m.set32(0x1C040, 1 << 31);
    m.set32(0x1C064, 1 << 31);
    m.set32(0x1C080, 1 << 2);
    m.set32(0x1C140, if s1_done { 1 << 31 } else { 0 });
    m.set32(0x1C164, 1 << 31);
    m.set32(0x1C15C, 1 << 2);
    m.set32(0x1C180, 1 << 2);
    m.on_read(
        0x1C05C,
        Box::new(|m| (((m.get32(0x1C000) >> 1) & 0xF) << 24) | (1 << 2)),
    );
    m.set32(0x1C048, 0x554);
    m.set32(0x1C060, 0x7A00_0000);
    m
}

/// The index of the first `op` in `trace`.
fn first(trace: &[Op], op: Op) -> usize {
    trace
        .iter()
        .position(|o| *o == op)
        .unwrap_or_else(|| panic!("{op:?} not in the trace"))
}

/// The index of the first 32-bit write of `reg` in `trace`.
fn first_w32(trace: &[Op], reg: u32) -> usize {
    trace
        .iter()
        .position(|o| matches!(o, Op::W32(r, _) if *r == reg))
        .unwrap_or_else(|| panic!("no write of {reg:#x}"))
}

/// `_dack_reload_by_path`'s four MSBK words from `base` (0x1C200 plus the
/// index and path offsets), each followed by its debug read-back.
fn msbk_words(base: u32, words: [u32; 4]) -> Vec<Op> {
    words
        .iter()
        .zip((0..).map(|k: u32| base + 4 * k))
        .flat_map(|(&v, reg)| [Op::W32(reg, v), Op::R32(reg)])
        .collect()
}

/// The reload's bias/DADCK word `v` at `reg`, then its b0 set.
fn bias_word(reg: u32, v: u32) -> [Op; 3] {
    [Op::W32(reg, v), Op::R32(reg), Op::W32(reg, v | 1)]
}

#[test]
fn dack_s0_reloads_packed_msbk_words() {
    let mut m = dack_model(true);
    assert_eq!(dack(&mut m).unwrap().dack_timed_out, [false, false]);
    let base = 0x1C200;
    let at = first(&m.trace, Op::W32(base, 0x0F0E_0D0C));
    let mut want = msbk_words(base, [0x0F0E_0D0C, 0x0B0A_0908, 0x0706_0504, 0x0302_0100]);
    want.extend(bias_word(0x1C210, 0x555E_8000));
    assert_eq!(m.trace[at..at + want.len()], want[..]);
    // Index 1 (the 0xC020/0xC080 side, whose MSBK, bias and DADCK read 0
    // here), then path B's two indexes, each at its own offset.
    for off in [0x14, 0x28, 0x3C] {
        let b = base + off;
        let at = first(&m.trace, Op::W32(b, 0));
        let mut want = msbk_words(b, [0; 4]);
        want.extend(bias_word(b + 0x10, 0));
        assert_eq!(m.trace[at..at + want.len()], want[..]);
    }
    // Each reload starts with dack_reload_defs: b17 of 0xC004, 0xC024,
    // 0xC104 and 0xC124.
    let w = m.writes();
    let at = first(&w, Op::W32(base, 0x0F0E_0D0C));
    assert_eq!(
        w[at - 4..at],
        [
            Op::W32(0x1C004, 0x0302_0000),
            Op::W32(0x1C024, 0x0302_0000),
            Op::W32(0x1C104, 0x2_0000),
            Op::W32(0x1C124, 0x2_0000),
        ]
    );
}

#[test]
fn dack_runs_notify_drck_addck_s0_s1_notify_in_order() {
    let mut m = dack_model(true);
    dack(&mut m).unwrap();
    let t = &m.trace;
    let order = [
        first(t, Op::W32(0x00AC, 0x8100_4803)),
        first(t, Op::W32(0x1C0C4, 0x40)),
        first(t, Op::RfW(0, 0x00, F, 0x337E1)),
        first_w32(t, 0x1C0F4),
        first(t, Op::RfW(0, 0x01, F, 0)),
        first_w32(t, 0x112A0),
        first_w32(t, 0x132A0),
        first(t, Op::W32(0x00AC, 0x8100_4003)),
    ];
    assert!(order.windows(2).all(|w| w[0] < w[1]), "{order:?}");
    // The coex START comes first, with its grant, and the STOP's 1 ms ends
    // the trace.
    assert_eq!(t[0], Op::W32(0xDA48, 0x5_7DDF));
    assert_eq!(t.last(), Some(&Op::Delay(1000)));
}

#[test]
fn dack_returns_drck_and_addck_results() {
    let mut m = dack_model(true);
    m.set32(DRCK_RES, 0x0006_8008);
    m.set32(ADDCKR[0], (0x155 << 10) | 0x2AB);
    m.set32(ADDCKR[1], (0x3FF << 10) | 0x001);
    assert_eq!(
        dack(&mut m),
        Ok(Dack {
            drck: 0x0D,
            drck_timed_out: false,
            addck: [(0x155, 0x2AB), (0x3FF, 0x001)],
            addck_timed_out: [false, false],
            dack_timed_out: [false, false],
        })
    );
    // The ADDCK results reloaded into 0xC0F8/0xC1F8.
    assert_eq!(
        m.get32(ADDCK_RL[0]),
        (3 << 28) | (0x155 << 18) | (0x2AB << 8)
    );
    assert_eq!(
        m.get32(ADDCK_RL[1]),
        (3 << 28) | (0x3FF << 18) | (0x001 << 8)
    );
}

#[test]
fn dack_restores_rf_mode_and_reg5() {
    let mut m = dack_model(true);
    m.set_rf(0, 0x00, 0x33C01);
    m.set_rf(1, 0x00, 0x33C01);
    dack(&mut m).unwrap();
    let rf = rf_only(&m);
    // The whole SWSI view of _dac_cal: both RR_MOD reads, after DRCK the
    // reg5 clear and the 0x337E1 mode, after ADDCK RR_MODOPT cleared, and
    // after S1 the mode and reg5 restored.
    assert_eq!(
        rf,
        [
            Op::RfR(0, 0x00),
            Op::RfR(1, 0x00),
            Op::RfW(0, 0x05, 1, 0),
            Op::RfW(1, 0x05, 1, 0),
            Op::RfW(0, 0x00, F, 0x337E1),
            Op::RfW(1, 0x00, F, 0x337E1),
            Op::RfW(0, 0x01, F, 0),
            Op::RfW(1, 0x01, F, 0),
            Op::RfW(0, 0x00, F, 0x33C01),
            Op::RfW(1, 0x00, F, 0x33C01),
            Op::RfW(0, 0x05, 1, 1),
            Op::RfW(1, 0x05, 1, 1),
        ]
    );
    for p in 0..2u8 {
        assert_eq!(m.get_rf(p, 0x00), 0x33C01);
        assert_eq!(m.get_rf(p, 0x05) & 1, 1);
    }
}

#[test]
fn dack_s0_matches_linux_sequence() {
    let mut m = dack_model(true);
    dack(&mut m).unwrap();
    let w = m.writes();
    let start = first_w32(&w, 0x112A0);
    assert_eq!(
        w[start..start + 16],
        [
            // txck_force(A, true, DAC_160M).
            Op::W32(0x112A0, 0),
            Op::W32(0x112A0, 0x3000),
            Op::W32(0x112A0, 0xB000),
            // dack_defs_s0.
            Op::W32(0x112B8, 0x4000_0000),
            Op::W32(0x1030C, 0x1000_0000),
            Op::W32(0x1032C, 0),
            Op::W32(0x1C004, 0x0300_0000),
            Op::W32(0x1C024, 0x0300_0000),
            // _dack_reset(A).
            Op::W32(0x1C000, 0),
            Op::W32(0x1C000, 0x2_0000),
            // The trigger, its one met sample (no wait), the trigger off.
            Op::W32(0x1C004, 0x0300_0001),
            Op::W32(0x1C004, 0x0300_0000),
            // txck_force(A, false, ..).
            Op::W32(0x112A0, 0x3000),
            // _dack_backup_s0: the debug mode, then the first selectors.
            Op::W32(0x112B8, 0x4000_0000),
            Op::W32(0x1C000, 0x2_0000),
            Op::W32(0x1C020, 0),
        ]
    );
    // Sixteen selector pairs, then dack_reload_defs, before the first word.
    for i in 0..16u32 {
        let k = start + 14 + 2 * i as usize;
        assert_eq!(w[k], Op::W32(0x1C000, 0x2_0000 | (i << 1)));
        assert_eq!(w[k + 1], Op::W32(0x1C020, i << 1));
    }
    assert_eq!(
        first(&w, Op::W32(0x1C200, 0x0F0E_0D0C)),
        start + 14 + 32 + 4
    );
    // The done chain: one sample reads the four registers in order.
    let trig = first(&m.trace, Op::W32(0x1C004, 0x0300_0001));
    assert_eq!(
        m.trace[trig + 1..trig + 5],
        [
            Op::R32(0x1C040),
            Op::R32(0x1C064),
            Op::R32(0x1C05C),
            Op::R32(0x1C080),
        ]
    );
    // The backup's last selector pair and its reads, then bias and DADCK of
    // both sides, then the first reload.
    let bk = first(&m.trace, Op::W32(0x1C000, 0x2_001E));
    assert_eq!(
        m.trace[bk + 1..bk + 12],
        [
            Op::R32(0x1C05C),
            Op::R32(0x1C020),
            Op::W32(0x1C020, 0x1E),
            Op::R32(0x1C080),
            Op::R32(0x1C048),
            Op::R32(0x1C06C),
            Op::R32(0x1C060),
            Op::R32(0x1C084),
            Op::R32(0x1C004),
            Op::W32(0x1C004, 0x0302_0000),
            Op::R32(0x1C024),
        ]
    );
    // After both reloads, 0x12B8 b30 is cleared and S1 starts.
    let s1 = first_w32(&w, 0x132A0);
    assert_eq!(w[s1 - 1], Op::W32(0x112B8, 0));
    assert_eq!(
        w[s1 - 3..s1 - 1],
        [Op::W32(0x1C224, 0), Op::W32(0x1C224, 1)]
    );
}

#[test]
fn dack_s1_matches_linux_sequence() {
    let mut m = dack_model(true);
    dack(&mut m).unwrap();
    let w = m.writes();
    let start = first_w32(&w, 0x132A0);
    assert_eq!(
        w[start..start + 16],
        [
            Op::W32(0x132A0, 0),
            Op::W32(0x132A0, 0x3000),
            Op::W32(0x132A0, 0xB000),
            // dack_defs_s1 (0x030C and 0x032C are already set by S0's).
            Op::W32(0x132B8, 0x4000_0000),
            Op::W32(0x1030C, 0x1000_0000),
            Op::W32(0x1032C, 0),
            // b17 is left set by S0's reloads.
            Op::W32(0x1C104, 0x0302_0000),
            Op::W32(0x1C124, 0x0302_0000),
            Op::W32(0x1C100, 0),
            Op::W32(0x1C100, 0x2_0000),
            Op::W32(0x1C104, 0x0302_0001),
            Op::W32(0x1C104, 0x0302_0000),
            Op::W32(0x132A0, 0x3000),
            Op::W32(0x132B8, 0x4000_0000),
            Op::W32(0x1C100, 0x2_0000),
            Op::W32(0x1C120, 0),
        ]
    );
    let trig = first(&m.trace, Op::W32(0x1C104, 0x0302_0001));
    assert_eq!(
        m.trace[trig + 1..trig + 5],
        [
            Op::R32(0x1C140),
            Op::R32(0x1C164),
            Op::R32(0x1C15C),
            Op::R32(0x1C180),
        ]
    );
    let bk = first(&m.trace, Op::W32(0x1C100, 0x2_001E));
    assert_eq!(
        m.trace[bk + 1..bk + 9],
        [
            Op::R32(0x1C15C),
            Op::R32(0x1C120),
            Op::W32(0x1C120, 0x1E),
            Op::R32(0x1C180),
            Op::R32(0x1C148),
            Op::R32(0x1C16C),
            Op::R32(0x1C160),
            Op::R32(0x1C184),
        ]
    );
    // Path B's second reload, then 0x32B8 b30 cleared, then the restore.
    let end = first(&w, Op::W32(0x132B8, 0));
    assert_eq!(w[end - 2..end], [Op::W32(0x1C24C, 0), Op::W32(0x1C24C, 1)]);
}

#[test]
fn dack_s1_timeout_is_recorded_on_path_b() {
    let mut m = dack_model(false);
    assert_eq!(dack(&mut m).unwrap().dack_timed_out, [false, true]);
    let t = &m.trace;
    let trig = first(t, Op::W32(0x1C104, 0x0302_0001));
    let stop = trig + first(&t[trig..], Op::W32(0x1C104, 0x0302_0000));
    // read_poll_timeout_atomic(.., 1, 10000, ..): 9992 samples, 9991 waits,
    // each sample stopping at 0xC140.
    let poll = &t[trig..stop];
    assert_eq!(
        poll.iter().filter(|o| **o == Op::R32(0x1C140)).count(),
        9992
    );
    assert_eq!(poll.iter().filter(|o| **o == Op::Delay(1)).count(), 9991);
    assert!(
        !poll
            .iter()
            .any(|o| matches!(o, Op::R32(0x1C164 | 0x1C15C | 0x1C180)))
    );
    // S1's backup and both reloads still run, then the restore and STOP.
    let bk = first(t, Op::W32(0x1C100, 0x2_001E));
    assert!(bk > stop);
    for b in [0x1C228, 0x1C23C] {
        assert!(first(t, Op::W32(b + 0x10, 0)) > bk);
        assert!(t.contains(&Op::W32(b + 0x10, 1)));
    }
    assert!(first(t, Op::W32(0x132B8, 0)) > bk);
    assert_eq!(t.last(), Some(&Op::Delay(1000)));
}

#[test]
fn dack_s0_timeout_is_recorded_on_path_a_and_s1_runs() {
    let mut m = dack_model(true);
    m.set32(0x1C064, 0);
    assert_eq!(dack(&mut m).unwrap().dack_timed_out, [true, false]);
    assert!(m.trace.contains(&Op::W32(0x1C104, 0x0302_0001)));
}

#[test]
fn dack_swsi_stuck_fails_after_the_coex_start() {
    let mut m = dack_model(true);
    m.swsi_status = 1 << 24;
    assert!(matches!(
        dack(&mut m),
        Err(AxError::Timeout {
            register: 0x1174C,
            ..
        })
    ));
    assert!(m.trace.contains(&Op::W32(0x00AC, 0x8100_4803)));
    assert!(
        rf_only(&m)
            .iter()
            .all(|o| !matches!(o, Op::RfW(..) | Op::RfR(..)))
    );
}

/// The `(i, q)` TIA offsets an RX DCK model answers, by RF 0x00 [14:10].
type Tia = [(u32, u32); 32];

/// An RX DCK model: SWSI answered, RF 0x05 0x12345 on both paths, RF 0x93
/// b5 (done) set on path `p` when `done[p]`, RF 0x95 b3 (check) set when
/// `check`, and RF 0x92 [7:5] = `res`. Reads of RF 0x92 and 0x93 answer
/// their [15:9] from `tia` keyed on RF 0x00 [14:10], other bits as stored.
fn rx_dck_model(done: [bool; 2], check: bool, res: u32, tia: Tia) -> RegisterModel {
    let mut m = RegisterModel::new();
    m.emulate_rf();
    for p in 0..2u8 {
        m.set_rf(p, 0x05, 0x12345);
        m.set_rf(p, 0x92, res << 5);
        m.set_rf(p, 0x93, if done[p as usize] { 1 << 5 } else { 0 });
        m.set_rf(p, 0x95, if check { 1 << 3 } else { 0 });
        let index = move |m: &RegisterModel| ((m.get_rf(p, 0x00) >> 10) & 0x1F) as usize;
        m.on_rf_read(
            p,
            0x92,
            Box::new(move |m| (m.get_rf(p, 0x92) & !0xFE00) | (tia[index(m)].0 << 9)),
        );
        m.on_rf_read(
            p,
            0x93,
            Box::new(move |m| (m.get_rf(p, 0x93) & !0xFE00) | (tia[index(m)].1 << 9)),
        );
    }
    m
}

/// One `_rx_dck_toggle` whose first poll sample is done.
fn rx_dck_toggle_ops(p: u8) -> [Op; 4] {
    [
        Op::RfW(p, 0x92, 1, 0),
        Op::RfW(p, 0x92, 1, 1),
        Op::RfR(p, 0x93),
        Op::RfW(p, 0x92, 1, 0),
    ]
}

/// The recover's select of `index` in RF 0x00 [14:10] and its two reads.
fn rx_dck_read_ops(p: u8, index: u32) -> [Op; 3] {
    [
        Op::RfW(p, 0x00, 0x7C00, index << 10),
        Op::RfR(p, 0x92),
        Op::RfR(p, 0x93),
    ]
}

/// `_rx_dck_value_rewrite` of `v` to LUT `addr`.
fn rx_dck_rewrite_ops(p: u8, addr: u32, v: u32) -> [Op; 9] {
    [
        Op::RfW(p, 0xEC, 0x80000, 0x80000),
        Op::RfW(p, 0xF0, 0x10000, 0x10000),
        Op::RfW(p, 0x92, 2, 2),
        Op::RfW(p, 0x33, 0xFF, addr),
        Op::RfW(p, 0x3F, F, v),
        Op::RfW(p, 0x3F, F, v),
        Op::RfW(p, 0x92, 2, 0),
        Op::RfW(p, 0xF0, 0x10000, 0),
        Op::RfW(p, 0xEC, 0x80000, 0),
    ]
}

/// The recover's eight index reads, in Linux order.
const RX_DCK_INDEXES: [u32; 8] = [0x00, 0x01, 0x0C, 0x0D, 0x10, 0x11, 0x1C, 0x1D];

#[test]
fn rx_dck_single_toggle_when_check_is_clear() {
    let mut m = rx_dck_model([true; 2], false, 0, [(0, 0); 32]);
    assert_eq!(rx_dck(&mut m), Ok(([false, false], [0, 0])));
    let mut want = Vec::new();
    for p in 0..2u8 {
        want.extend([
            Op::RfR(p, 0x05),
            Op::RfW(p, 0x05, 1, 0),
            Op::RfW(p, 0x00, 0xF0000, 0x30000),
            Op::RfW(p, 0x00, 2, 0),
            Op::RfW(p, 0x93, 0xF, 0),
        ]);
        want.extend(rx_dck_toggle_ops(p));
        want.push(Op::RfR(p, 0x95));
        for index in RX_DCK_INDEXES {
            want.extend(rx_dck_read_ops(p, index));
        }
        want.push(Op::RfW(p, 0x05, F, 0x12345));
    }
    assert_eq!(rf_only(&m), want);
    for p in 0..2u8 {
        // RF 0x05 restored whole; RF 0x00 left in RX mode at the last index.
        assert_eq!(m.get_rf(p, 0x05), 0x12345);
        assert_eq!(m.get_rf(p, 0x00), 0x30000 | (0x1D << 10));
    }
}

#[test]
fn rx_dck_second_toggle_when_res_above_one() {
    let mut m = rx_dck_model([true; 2], true, 3, [(0, 0); 32]);
    assert_eq!(rx_dck(&mut m), Ok(([false, false], [0, 0])));
    let ops = rf_only(&m);
    for p in 0..2u8 {
        let at = first(&ops, Op::RfW(p, 0x93, 0xF, 0)) + 1;
        let mut want = Vec::from(rx_dck_toggle_ops(p));
        want.extend([
            Op::RfR(p, 0x95),
            Op::RfR(p, 0x92),
            Op::RfW(p, 0x8F, 0xE00, 0x600),
        ]);
        want.extend(rx_dck_toggle_ops(p));
        want.push(Op::RfW(p, 0x8F, 0xE00, 0x200));
        want.extend(rx_dck_read_ops(p, 0x00));
        assert_eq!(ops[at..at + want.len()], want[..]);
        assert_eq!(m.get_rf(p, 0x8F), 0x200);
    }
}

#[test]
fn rx_dck_res_of_one_skips_the_second_toggle() {
    let mut m = rx_dck_model([true; 2], true, 1, [(0, 0); 32]);
    assert_eq!(rx_dck(&mut m), Ok(([false, false], [0, 0])));
    let ops = rf_only(&m);
    let at = first(&ops, Op::RfR(0, 0x95));
    assert_eq!(
        ops[at..at + 3],
        [
            Op::RfR(0, 0x95),
            Op::RfR(0, 0x92),
            Op::RfW(0, 0x00, 0x7C00, 0)
        ]
    );
    assert!(!ops.iter().any(|op| matches!(op, Op::RfW(_, 0x8F, ..))));
}

#[test]
fn rx_dck_recover_rewrites_an_out_of_range_offset() {
    let mut tia = [(0, 0); 32];
    tia[0x00] = (0x40, 0x40);
    tia[0x0C] = (0x60, 0x45);
    let mut m = rx_dck_model([true; 2], false, 0, tia);
    assert_eq!(rx_dck(&mut m), Ok(([false, false], [1, 1])));
    let ops = rf_only(&m);
    for p in 0..2u8 {
        // i is 0x20 off and takes the base's 0x40; q is 5 off and is kept:
        // ((0x45 >> 1) << 6) | (0x40 >> 1) = 0x8A0.
        let at = first(&ops, Op::RfW(p, 0x00, 0x7C00, 0x0C << 10));
        let mut want = Vec::from(rx_dck_read_ops(p, 0x0C));
        want.extend(rx_dck_rewrite_ops(p, 0x0C, 0x8A0));
        want.extend(rx_dck_read_ops(p, 0x0D));
        assert_eq!(ops[at..at + want.len()], want[..]);
        assert_eq!(m.get_rf(p, 0x3F), 0x8A0);
        assert_eq!(m.get_rf(p, 0x33), 0x0C);
        // Every enable is cleared again.
        assert_eq!(m.get_rf(p, 0xEC) & 0x80000, 0);
        assert_eq!(m.get_rf(p, 0xF0) & 0x10000, 0);
        assert_eq!(m.get_rf(p, 0x92) & 2, 0);
    }
    assert_eq!(
        ops.iter()
            .filter(|op| matches!(op, Op::RfW(_, 0x3F, ..)))
            .count(),
        4
    );
}

#[test]
fn rx_dck_threshold_is_asymmetric() {
    // A delta of 9 on i and 10 on q passes.
    let mut tia = [(0, 0); 32];
    tia[0x10] = (0x20, 0x20);
    tia[0x1C] = (0x29, 0x2A);
    let mut m = rx_dck_model([true; 2], false, 0, tia);
    assert_eq!(rx_dck(&mut m), Ok(([false, false], [0, 0])));
    assert!(
        !rf_only(&m)
            .iter()
            .any(|op| matches!(op, Op::RfW(_, 0x3F, ..)))
    );
    // 10 on both fails the pass check, but neither is over the threshold,
    // so the target's own values are rewritten:
    // ((0x2A >> 1) << 6) | (0x2A >> 1) = 0x555.
    tia[0x1C] = (0x2A, 0x2A);
    let mut m = rx_dck_model([true; 2], false, 0, tia);
    assert_eq!(rx_dck(&mut m), Ok(([false, false], [1, 1])));
    let ops = rf_only(&m);
    for p in 0..2u8 {
        let at = first(&ops, Op::RfW(p, 0x00, 0x7C00, 0x1C << 10)) + 3;
        assert_eq!(ops[at..at + 9], rx_dck_rewrite_ops(p, 0x1C, 0x555));
    }
}

#[test]
fn rx_dck_toggle_timeout_is_recorded() {
    let mut m = rx_dck_model([false, true], false, 0, [(0, 0); 32]);
    assert_eq!(rx_dck(&mut m), Ok(([true, false], [0, 0])));
    let ops = m.rf_ops();
    let on = first(&ops, Op::RfW(0, 0x92, 1, 1));
    let off = on + first(&ops[on..], Op::RfW(0, 0x92, 1, 0));
    let poll = &ops[on + 1..off];
    // read_poll_timeout_atomic(.., 2, 2000, false, ..): 1001 samples, each
    // with its 2 us SWSI settle, and 1000 2 us waits between them.
    assert_eq!(
        poll.iter().filter(|op| **op == Op::RfR(0, 0x93)).count(),
        1001
    );
    assert_eq!(poll.iter().filter(|op| **op == Op::Delay(2)).count(), 2001);
    assert_eq!(poll.len(), 1001 + 2001);
    // Path A still checks, recovers and restores, and path B runs.
    let rest = rf_only(&m);
    // The pulse, its 1001 samples, then the pulse cleared.
    let after = first(&rest, Op::RfW(0, 0x92, 1, 1)) + 1 + 1001 + 1;
    assert_eq!(rest[after - 1], Op::RfW(0, 0x92, 1, 0));
    assert_eq!(rest[after], Op::RfR(0, 0x95));
    assert!(rest.contains(&Op::RfW(0, 0x05, F, 0x12345)));
    assert_eq!(rest.last(), Some(&Op::RfW(1, 0x05, F, 0x12345)));
}

#[test]
fn rx_dck_swsi_stuck_fails() {
    let mut m = rx_dck_model([true; 2], false, 0, [(0, 0); 32]);
    m.swsi_status = 1 << 24;
    assert!(matches!(
        rx_dck(&mut m),
        Err(AxError::Timeout {
            register: 0x1174C,
            ..
        })
    ));
    assert!(rf_only(&m).is_empty());
}

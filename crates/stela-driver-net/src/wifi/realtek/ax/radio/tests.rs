// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
extern crate std;
use super::*;
use crate::wifi::realtek::ax::elements::{
    ID_RADIO_A, ID_RADIO_B, phy_tables,
    tests::{container, element, pairs},
};
use crate::wifi::realtek::ax::pci::tests::{FWCMD, device_read, reclaim_at_once, running};
use crate::wifi::realtek::ax::pci::{FWCMD_SLOT_BYTES, Rings};
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use crate::wifi::realtek::beacon::tests::Model;
use alloc::{
    boxed::Box,
    format,
    string::{String, ToString},
    sync::Arc,
    vec::Vec,
};
use std::sync::Mutex;

/// A firmware file with RADIO_B in slot 0 holding `b` and RADIO_A in slot 1
/// holding `a`, as the pinned file orders them. A TXPWR element follows, so
/// the last radio element is still read (fw.c:1584).
fn file(b: &[(u32, u32)], a: &[(u32, u32)]) -> Vec<u8> {
    let mut f = container(32, 16);
    element(&mut f, ID_RADIO_B, [0; 4], 0, 0, &pairs(b));
    element(&mut f, ID_RADIO_A, [0; 4], 0, 1, &pairs(a));
    element(&mut f, 9, [0; 4], 0, 0, &[0; 16]);
    f
}

fn slots(f: &[u8]) -> [Radio<'_>; 2] {
    let t = phy_tables(f, 0).unwrap();
    [t.radio[0].unwrap(), t.radio[1].unwrap()]
}

struct Ran {
    got: Result<RfInit, RfInitError>,
    m: Arc<Mutex<Model>>,
    io: RegisterModel,
}

/// `init_rf_reg` over `radio` on running rings whose CH12 consumes each
/// packet at once (or never, without `reclaim`), with the H2C sequence
/// after OFLD_CFG (seq 1).
fn run(radio: [Radio<'_>; 2], rfe: u8, cv: u8, reclaim: bool) -> Ran {
    let (m, mut rings, mut io) = running();
    io.emulate_rf();
    if reclaim {
        reclaim_at_once(&mut io);
    }
    let mut h2c = H2c::after_download();
    h2c.ofld_cfg();
    let got = init(&mut io, &mut rings, &mut h2c, radio, rfe, cv);
    Ran { got, m, io }
}

fn init(
    io: &mut RegisterModel,
    rings: &mut Rings,
    h2c: &mut H2c,
    radio: [Radio<'_>; 2],
    rfe: u8,
    cv: u8,
) -> Result<RfInit, RfInitError> {
    // SAFETY: `io` is the modeled requester that `running` programmed with
    // these rings through `pre_init`; the model domain lets no real device
    // reach the published IOVAs, and this test is the only worker.
    unsafe { init_rf_reg(io, rings, h2c, radio, rfe, cv) }
}

/// CH12 slot `slot`'s first `len` bytes, as the device would read them.
fn packet(m: &Arc<Mutex<Model>>, slot: usize, len: usize) -> Vec<u8> {
    let state = m.lock().unwrap();
    device_read(
        &state,
        u64::from(FWCMD) + (slot * FWCMD_SLOT_BYTES) as u64,
        len,
    )
}

fn le32(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(b[at..at + 4].try_into().unwrap())
}

/// The CH12 kicks (writes of the host index, 0x1080) in order.
fn kicks(io: &RegisterModel) -> Vec<usize> {
    io.trace
        .iter()
        .enumerate()
        .filter(|(_, op)| matches!(op, Op::W16(0x1080, _)))
        .map(|(i, _)| i)
        .collect()
}

fn first(io: &RegisterModel, want: impl Fn(&Op) -> bool) -> usize {
    io.trace.iter().position(want).unwrap()
}

fn direct_writes(io: &RegisterModel, base: u32) -> usize {
    io.trace
        .iter()
        .filter(|op| matches!(op, Op::W32(a, _) if (base..base + 0x400).contains(a)))
        .count()
}

/// Every entry of a radio table, as `n` direct writes of RF 0x05 on the
/// D-die (bit 16 set), each with its own data.
fn direct_entries(n: u32) -> Vec<(u32, u32)> {
    (0..n).map(|i| (0x10005, i)).collect()
}

#[test]
fn rf_reg_word_keeps_addr_bits_7_to_0() {
    assert_eq!(rf_reg_word(0x10030, 0x7F003), 0x0307_F003);
    assert_eq!(rf_reg_word(0x10005, 0), 0x0050_0000);
    assert_eq!(rf_reg_word(0x10018, 0x11124), 0x0181_1124);
}

#[test]
fn page_constants_are_linuxs() {
    assert_eq!((RF_PAGE_WORDS, RF_PAGES), (500, 3));
}

#[test]
fn config_rf_reg_v1_delays_writes_and_stores() {
    let f = file(&[(0xFE, 0x63), (0x05, 0), (0x10005, 0x1)], &[]);
    let r = run(slots(&f), 1, 1, true);
    let got = r.got.unwrap();
    // 0xFE is a delay, not stored; RF 0x05 (below 0x100) is written over
    // SWSI and not stored; 0x10005 is a direct write, stored.
    assert_eq!(got.applied, [3, 0]);
    assert_eq!(got.words, [1, 0]);
    let kick = kicks(&r.io)[0];
    let before: Vec<Op> = {
        let mut io = RegisterModel::new();
        io.trace = r.io.trace[..kick].to_vec();
        io.rf_ops()
    };
    assert_eq!(
        before,
        [
            Op::Delay(50_000),
            Op::RfW(1, 0x05, 0xFFFFF, 0),
            Op::W32(0x1F014, 0x1),
            Op::Delay(1),
        ]
    );
    // One page for path B, none for an empty path A.
    assert_eq!(kicks(&r.io).len(), 1);
    let p = packet(&r.m, 0, 28);
    assert_eq!(le32(&p, 16), 0x0100_0026);
    assert_eq!(le32(&p, 24), 0x0050_0001);
}

#[test]
fn init_rf_reg_configures_slot_0_as_path_b_first() {
    let f = file(&[(0x10005, 1)], &[(0x10005, 2)]);
    let r = run(slots(&f), 1, 1, true);
    assert_eq!(
        r.got,
        Ok(RfInit {
            applied: [1, 1],
            words: [1, 1],
        })
    );
    let k = kicks(&r.io);
    assert_eq!(k.len(), 2);
    let b = first(&r.io, |op| matches!(op, Op::W32(0x1F014, _)));
    let a = first(&r.io, |op| matches!(op, Op::W32(0x1E014, _)));
    assert!(
        b < k[0] && k[0] < a && a < k[1],
        "{b} {} {a} {}",
        k[0],
        k[1]
    );
    let p0 = packet(&r.m, 0, 28);
    let p1 = packet(&r.m, 1, 28);
    assert_eq!(le32(&p0, 16), 0x0100_0026);
    assert_eq!(le32(&p1, 16), 0x0200_0022);
    // hdr1: 8 + 4 bytes, no REC_ACK or DONE_ACK at seq 1 and 2.
    assert_eq!(le32(&p0, 20), 12);
    assert_eq!(le32(&p1, 20), 12);
    assert_eq!(le32(&p0, 24), 0x0050_0001);
    assert_eq!(le32(&p1, 24), 0x0050_0002);
}

/// Check one RF_REG_B page in CH12 slot `slot`: func `page`, `words`
/// payload words starting at the table's entry `from`.
fn assert_page(m: &Arc<Mutex<Model>>, slot: usize, page: u8, from: u32, words: usize) {
    let len = 24 + 4 * words;
    let p = packet(m, slot, len);
    // The descriptor's pkt_size and the header's length agree.
    assert_eq!(le32(&p, 0) & 0xFFFF, (8 + 4 * words) as u32);
    let hdr0 = le32(&p, 16);
    assert_eq!(hdr0 & 0xFF, 0x26, "slot {slot}");
    assert_eq!((hdr0 >> 8) & 0xFF, u32::from(page), "slot {slot}");
    assert_eq!(le32(&p, 20) & 0x3FFF, (8 + 4 * words) as u32);
    for i in 0..words {
        assert_eq!(le32(&p, 24 + 4 * i), 0x0050_0000 | (from + i as u32));
    }
}

#[test]
fn pages_split_every_500_words() {
    let f = file(&direct_entries(1001), &[]);
    let r = run(slots(&f), 1, 1, true);
    assert_eq!(
        r.got,
        Ok(RfInit {
            applied: [1001, 0],
            words: [1001, 0],
        })
    );
    assert_eq!(kicks(&r.io).len(), 3);
    assert_page(&r.m, 0, 0, 0, 500);
    assert_page(&r.m, 1, 1, 500, 500);
    assert_page(&r.m, 2, 2, 1000, 1);

    // The 1501st word is written to the chip but dropped from the pages.
    let f = file(&direct_entries(1501), &[]);
    let r = run(slots(&f), 1, 1, true);
    assert_eq!(
        r.got,
        Ok(RfInit {
            applied: [1501, 0],
            words: [1500, 0],
        })
    );
    assert_eq!(direct_writes(&r.io, 0x1F000), 1501);
    assert_eq!(kicks(&r.io).len(), 3);
    assert_page(&r.m, 0, 0, 0, 500);
    assert_page(&r.m, 1, 1, 500, 500);
    assert_page(&r.m, 2, 2, 1000, 500);
}

#[test]
fn no_radio_branch_fails_rf_init_before_any_write() {
    // One headline, for RFE 1 at cut 1 only.
    let f = file(
        &[(0xF001_0001, 0), (0x05, 0), (0x10005, 1)],
        &[(0x10005, 2)],
    );
    let r = run(slots(&f), 0, 1, true);
    assert_eq!(
        r.got,
        Err(RfInitError::Table {
            path: RfPath::B,
            error: AxError::PhyPackage {
                table: "RADIO_B",
                rfe: 0,
                cv: 1,
            },
        })
    );
    assert_eq!(r.got.unwrap_err().step(), "rf init");
    assert!(r.io.rf_ops().is_empty(), "{:?}", r.io.rf_ops());
    assert!(kicks(&r.io).is_empty());
}

#[test]
fn rf_write_failure_is_rf_init_and_sends_nothing() {
    let f = file(&[(0x05, 0)], &[]);
    let (_m, mut rings, mut io) = running();
    io.emulate_rf();
    reclaim_at_once(&mut io);
    // SWSI stays busy.
    io.swsi_status = 1 << 24;
    let mut h2c = H2c::after_download();
    let got = init(&mut io, &mut rings, &mut h2c, slots(&f), 1, 1);
    let err = got.unwrap_err();
    assert_eq!(
        err,
        RfInitError::Table {
            path: RfPath::B,
            error: AxError::Timeout {
                register: 0x1174C,
                mask: 0x0300_0000,
            },
        }
    );
    assert_eq!(err.step(), "rf init");
    assert!(kicks(&io).is_empty());
}

#[test]
fn submit_failure_is_rf_h2c() {
    let f = file(&[(0x10005, 1)], &[(0x10005, 2)]);
    let r = run(slots(&f), 1, 1, false);
    let err = r.got.unwrap_err();
    assert!(
        matches!(
            err,
            RfInitError::Submit {
                path: RfPath::B,
                page: 0,
                error: AxError::Timeout { .. },
            }
        ),
        "{err:?}"
    );
    assert_eq!(err.step(), "rf h2c");
    // Path A was never configured after B's page failed.
    assert_eq!(kicks(&r.io).len(), 1);
    assert_eq!(direct_writes(&r.io, 0x1E000), 0);
}

#[test]
fn errors_name_their_steps_and_where_they_happened() {
    let e = AxError::Phy("x");
    let table = RfInitError::Table {
        path: RfPath::A,
        error: e,
    };
    let submit = RfInitError::Submit {
        path: RfPath::B,
        page: 2,
        error: e,
    };
    assert_eq!(table.step(), "rf init");
    assert_eq!(submit.step(), "rf h2c");
    assert_eq!(table.error(), e);
    assert_eq!(submit.error(), e);
    assert_eq!(
        table.at(),
        RfAt {
            path: RfPath::A,
            page: None
        }
    );
    assert_eq!(
        submit.at(),
        RfAt {
            path: RfPath::B,
            page: Some(2)
        }
    );
    assert_eq!(table.at().to_string(), "path A");
    assert_eq!(submit.at().to_string(), "path B page 2");
}

#[test]
fn a_failure_on_path_a_names_path_a() {
    // Path B succeeds; path A's only entry, an SWSI write, finds SWSI busy.
    let f = file(&[(0x10005, 1)], &[(0x05, 0)]);
    let (_m, mut rings, mut io) = running();
    io.emulate_rf();
    reclaim_at_once(&mut io);
    io.on_read(
        0x1174C,
        Box::new(
            |m: &mut RegisterModel| {
                if m.get32(0x1F014) != 0 { 1 << 24 } else { 0 }
            },
        ),
    );
    let mut h2c = H2c::after_download();
    h2c.ofld_cfg();
    let err = init(&mut io, &mut rings, &mut h2c, slots(&f), 1, 1).unwrap_err();
    assert_eq!(
        err,
        RfInitError::Table {
            path: RfPath::A,
            error: AxError::Timeout {
                register: 0x1174C,
                mask: 0x0300_0000,
            },
        }
    );
    // Path B's page went out first.
    assert_eq!(kicks(&io).len(), 1);
}

fn sha256(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

#[test]
#[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES"]
fn real_8852c_fw_2_rf_init_sends_linux_pages() {
    let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
    let f = std::fs::read(root.join("rtw89/rtw8852c_fw-2.bin")).unwrap();
    let r = run(slots(&f), 1, 1, true);
    assert_eq!(
        r.got,
        Ok(RfInit {
            applied: [2013, 2046],
            words: [392, 392],
        })
    );
    assert_eq!(kicks(&r.io).len(), 2);
    let b = packet(&r.m, 0, 1592);
    let a = packet(&r.m, 1, 1592);
    assert_eq!(
        b[..16],
        [0x28, 0x06, 0x00, 0x0D, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    );
    assert_eq!(b[16..24], [0x26, 0x00, 0x00, 0x01, 0x28, 0x06, 0x00, 0x00]);
    assert_eq!(a[16..24], [0x22, 0x00, 0x00, 0x02, 0x28, 0x06, 0x00, 0x00]);
    assert_eq!(
        b[24..36],
        [
            0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x03, 0x00, 0x24, 0x11, 0x81, 0x01
        ]
    );
    assert_eq!(
        sha256(&b[24..]),
        "a7edc647d3703c7b66a1b50f73a6b9319012d67064812b2697060241ed8e375b"
    );
    assert_eq!(
        sha256(&a[24..]),
        "a8d77d2119a004ef1c0684f4e05874ae75981d10b64310aa5d3bf6d50e5a9d3c"
    );
    let rfw = |path| {
        r.io.trace
            .iter()
            .filter(|op| matches!(op, Op::RfW(p, ..) if *p == path))
            .count()
    };
    assert_eq!((rfw(1), rfw(0)), (1620, 1653));
    assert_eq!(
        (direct_writes(&r.io, 0x1F000), direct_writes(&r.io, 0x1E000)),
        (392, 392)
    );
    // Path B, every write of it, before path A's first.
    let last_b =
        r.io.trace
            .iter()
            .rposition(|op| matches!(op, Op::RfW(1, ..)))
            .unwrap();
    let first_a = first(&r.io, |op| matches!(op, Op::RfW(0, ..)));
    assert!(last_b < kicks(&r.io)[0] && kicks(&r.io)[0] < first_a);
}

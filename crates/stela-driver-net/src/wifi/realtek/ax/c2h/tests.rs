extern crate std;
use super::*;

use alloc::vec::Vec;
fn c2h(w0: u32, w2: u32) -> Vec<u8> {
    let mut v = Vec::new();
    v.extend_from_slice(&w0.to_le_bytes());
    v.extend_from_slice(&12u32.to_le_bytes());
    v.extend_from_slice(&w2.to_le_bytes());
    v
}

#[test]
fn c2h_parses_rec_and_done_acks() {
    assert_eq!(
        parse(&c2h(0x0000_0001, 0x1425)),
        Some(C2h::RecAck {
            cat: 1,
            class: 9,
            func: 0x14,
            seq: 0
        })
    );
    assert_eq!(
        parse(&c2h(0x0000_0101, 0x1425)),
        Some(C2h::DoneAck {
            cat: 1,
            class: 9,
            func: 0x14,
            ret: 0,
            seq: 0
        })
    );
    assert_eq!(
        parse(&c2h(0x0000_0101, 0x0705_1425)),
        Some(C2h::DoneAck {
            cat: 1,
            class: 9,
            func: 0x14,
            ret: 5,
            seq: 7
        })
    );
    // REC_ACK seq is [23:16].
    assert_eq!(
        parse(&c2h(0x0000_0001, 0x0003_1425)),
        Some(C2h::RecAck {
            cat: 1,
            class: 9,
            func: 0x14,
            seq: 3
        })
    );
    // An ack shorter than 12 bytes is skipped.
    assert_eq!(parse(&c2h(0x0000_0001, 0)[..11]), None);
}

#[test]
fn c2h_log_plain_and_formatted() {
    let mut p = c2h(0x0000_0201, 0)[..8].to_vec();
    p.extend_from_slice(b"hello");
    assert_eq!(parse(&p), Some(C2h::Log(b"hello")));
    let mut p = c2h(0x0000_0201, 0)[..8].to_vec();
    p.extend_from_slice(&[0xA5, 0xA5]);
    p.extend_from_slice(&[0; 9]);
    assert_eq!(parse(&p), Some(C2h::FormattedLog));
    // 10 body bytes starting A5 A5 is still plain.
    let mut p = c2h(0x0000_0201, 0)[..8].to_vec();
    p.extend_from_slice(&[0xA5, 0xA5, 0, 0, 0, 0, 0, 0, 0, 0]);
    assert!(matches!(parse(&p), Some(C2h::Log(_))));
}

#[test]
fn c2h_other_and_short() {
    assert_eq!(
        parse(&c2h(0x0000_0102 | 0x4, 0)),
        Some(C2h::Other {
            cat: 2,
            class: 1,
            func: 1
        })
    );
    assert_eq!(parse(&[0; 7]), None);
}

fn scanofld(w2: u32, w5: u32) -> Vec<u8> {
    let mut v = Vec::new();
    for w in [0x0000_0905u32, 36, w2, 0, 0, w5, 0, 0, 0] {
        v.extend_from_slice(&w.to_le_bytes());
    }
    v
}

#[test]
fn scanofld_rsp_decodes_w2_and_w5() {
    assert_eq!(
        parse(&scanofld(0x6913_0006, 0x0100_0002)),
        Some(C2h::ScanOfld(ScanOfld {
            pri_ch: 6,
            reason: 3,
            status: 1,
            period: 105,
            tx_fail: 2,
            air_density: 0,
            band: 1,
            mac_idx: false,
        }))
    );
    assert_eq!(
        parse(&scanofld(0, 0x0500_00F5)[..24]),
        Some(C2h::ScanOfld(ScanOfld {
            pri_ch: 0,
            reason: 0,
            status: 0,
            period: 0,
            tx_fail: 5,
            air_density: 15,
            band: 1,
            mac_idx: true,
        }))
    );
    assert_eq!(
        (
            SCAN_ENTER_CH,
            SCAN_LEAVE_CH,
            SCAN_END,
            SCAN_ENTER_OP,
            SCAN_LEAVE_OP
        ),
        (3, 4, 5, 8, 9)
    );
    assert_eq!(SCAN_STATUS_SUCCESS, 1);
}

#[test]
fn short_scanofld_rsp_is_none() {
    let v = scanofld(0x6913_0006, 0x0100_0002);
    assert_eq!(parse(&v[..20]), None);
    for n in 0..24 {
        assert_eq!(parse(&v[..n]), None);
    }
    // Other class-1 functions stay Other.
    let mut o = v.clone();
    o[1] = 0x0A;
    assert!(matches!(parse(&o), Some(C2h::Other { .. })));
}

/// A BCNFLTR_RPT (cat 1, class 1, func 0xD) carrying `w2`, 12 bytes.
fn bcnfltr(w2: u32) -> Vec<u8> {
    c2h(0x0000_0D05, w2)
}

#[test]
fn bcnfltr_rpt_decodes_w2() {
    assert_eq!(
        parse(&bcnfltr(0x0028_0000)),
        Some(C2h::BeaconFilter(BcnFltr {
            macid: 0,
            kind: 0,
            event: 0,
            ma: 0x28
        }))
    );
    assert_eq!(BCNFLTR_BEACON_LOSS, 0);
    // MACID [7:0], TYPE [9:8], EVENT [11:10], MA [23:16] (fw.h:4011-4014).
    assert_eq!(
        parse(&bcnfltr(0x0050_0605)),
        Some(C2h::BeaconFilter(BcnFltr {
            macid: 5,
            kind: 2,
            event: 1,
            ma: 0x50
        }))
    );
    assert_eq!(
        parse(&bcnfltr(0xFFFF_FFFF)),
        Some(C2h::BeaconFilter(BcnFltr {
            macid: 0xFF,
            kind: 3,
            event: 3,
            ma: 0xFF
        }))
    );
    // Bits outside the fields change nothing.
    assert_eq!(
        parse(&bcnfltr(0xFF00_F000)),
        Some(C2h::BeaconFilter(BcnFltr {
            macid: 0,
            kind: 0,
            event: 0,
            ma: 0
        }))
    );
}

#[test]
fn short_bcnfltr_rpt_is_none_and_other_categories_stay_other() {
    let v = bcnfltr(0x0028_0000);
    for n in 0..12 {
        assert_eq!(parse(&v[..n]), None, "{n} bytes");
    }
    // A longer report reads only w2.
    let mut long = v.clone();
    long.extend_from_slice(&[0xFF; 8]);
    assert_eq!(parse(&long), parse(&v));
    // cat 2 (not MAC) with class 1 func 0xD is not a beacon filter report.
    assert_eq!(
        parse(&c2h(0x0000_0D06, 0x0028_0000)),
        Some(C2h::Other {
            cat: 2,
            class: 1,
            func: 0xD
        })
    );
}

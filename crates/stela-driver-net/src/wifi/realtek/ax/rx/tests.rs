extern crate std;
use super::*;

use alloc::{vec, vec::Vec};
fn buf(info: u32, d0: u32, total: usize) -> Vec<u8> {
    let mut b = vec![0u8; total];
    b[..4].copy_from_slice(&info.to_le_bytes());
    b[4..8].copy_from_slice(&d0.to_le_bytes());
    b
}

const FS_LS: u32 = 0xC000;

#[test]
fn rx_parse_short_descriptor_c2h() {
    let mut b = buf(FS_LS | 32, 0x0A00_000C, 32);
    for (i, x) in b[20..32].iter_mut().enumerate() {
        *x = i as u8 + 1;
    }
    let want = b[20..32].to_vec();
    let Ok(RxBuffer::Frame(f)) = parse(&b) else {
        panic!()
    };
    assert_eq!((f.pkt_type, f.payload), (10, &want[..]));
    assert_eq!(f.desc.pkt_type, 10);
}

#[test]
fn rx_parse_long_descriptor_with_drv_info() {
    // LONG b31, DRV_INFO 2 (bits 30:28), type 10, RPKT_LEN 6.
    let d0 = 1 << 31 | 2 << 28 | 10 << 24 | 6;
    let mut b = buf(FS_LS | 60, d0, 60);
    b[52..58].copy_from_slice(&[9, 8, 7, 6, 5, 4]);
    let Ok(RxBuffer::Frame(f)) = parse(&b) else {
        panic!()
    };
    assert_eq!((f.pkt_type, f.payload), (10, &[9u8, 8, 7, 6, 5, 4][..]));
}

#[test]
fn rx_parse_caps_payload_at_rpkt_len() {
    let b = buf(FS_LS | 40, 0x0A00_000C, 40);
    let Ok(RxBuffer::Frame(f)) = parse(&b) else {
        panic!()
    };
    assert_eq!(f.payload.len(), 12);
    assert_eq!(f.payload.as_ptr(), b[20..].as_ptr());
}

#[test]
fn rx_parse_reports_segment() {
    let b = buf(0x4000 | 32, 0x0A00_000C, 32);
    assert_eq!(parse(&b), Ok(RxBuffer::Segment));
    let b = buf(0x8000 | 32, 0x0A00_000C, 32);
    assert_eq!(parse(&b), Ok(RxBuffer::Segment));
}

#[test]
fn rx_buffer_shorter_than_its_descriptor_is_refused() {
    let err = Err(AxError::Rx("RX buffer shorter than its descriptor"));
    assert_eq!(parse(&buf(FS_LS | 10, 0x0A00_000C, 32)), err);
    // WRITE_SIZE claims 32 but the slice is shorter.
    assert_eq!(parse(&buf(FS_LS | 32, 0x0A00_000C, 32)[..30]), err);
    assert_eq!(parse(&[0; 3]), err);
}

fn put(b: &mut [u8], at: usize, v: u32) {
    b[at..at + 4].copy_from_slice(&v.to_le_bytes());
}

#[test]
fn long_descriptor_fields_decode() {
    let mut b = buf(FS_LS | 80, 0x8000_0040, 80);
    put(&mut b, 8, 0x240B_0053);
    put(&mut b, 12, 0x1234_5678);
    put(&mut b, 16, 0x0000_0201);
    put(&mut b, 20, 0);
    b[36] = 0xAB;
    let Ok(RxBuffer::Frame(f)) = parse(&b) else {
        panic!()
    };
    assert_eq!(
        f.desc,
        RxDesc {
            pkt_type: 0,
            pkt_len: 0x40,
            long: true,
            mac_info_valid: false,
            ppdu_type: 3,
            ppdu_cnt: 5,
            data_rate: 0x0B,
            gi_ltf: 2,
            bw: 1,
            free_run: 0x1234_5678,
            a1_match: true,
            sw_dec: false,
            hw_dec: false,
            ampdu: false,
            crc32_err: true,
            icv_err: false,
            frame_type: Some(0),
        }
    );
    assert_eq!(f.payload.as_ptr(), b[36..].as_ptr());
    assert_eq!(f.payload[0], 0xAB);
}

/// A long descriptor with d0-d3 all ones and the given d4. d0's DRV_INFO 7
/// puts the payload at 4 + 32 + 56 = 92.
fn all_ones_long(d4: u32) -> Vec<u8> {
    let mut b = buf(FS_LS | 100, 0xFFFF_FFFF, 100);
    for at in [8, 12, 16] {
        put(&mut b, at, 0xFFFF_FFFF);
    }
    put(&mut b, 20, d4);
    b
}

#[test]
fn all_ones_long_descriptor_reads_every_field_at_its_maximum() {
    let b = all_ones_long(0xFFFF_FFFF);
    let Ok(RxBuffer::Frame(f)) = parse(&b) else {
        panic!()
    };
    assert_eq!(
        f.desc,
        RxDesc {
            pkt_type: 15,
            pkt_len: 0x3FFF,
            long: true,
            mac_info_valid: true,
            ppdu_type: 15,
            ppdu_cnt: 7,
            data_rate: 0x1FF,
            gi_ltf: 7,
            bw: 7,
            free_run: 0xFFFF_FFFF,
            a1_match: true,
            sw_dec: true,
            hw_dec: true,
            ampdu: true,
            crc32_err: true,
            icv_err: true,
            frame_type: Some(3),
        }
    );
    assert_eq!(f.pkt_type, 15);
    assert_eq!(f.payload.as_ptr(), b[92..].as_ptr());
    assert_eq!(f.payload.len(), 8);
}

#[test]
fn long_descriptor_frame_type_is_d4_bits_1_0() {
    let b = all_ones_long(0xFFFF_FFFE);
    let Ok(RxBuffer::Frame(f)) = parse(&b) else {
        panic!()
    };
    assert_eq!(f.desc.frame_type, Some(2));
    assert_eq!(f.desc.frame_type, Some(FRAME_DATA));
}

#[test]
fn short_descriptor_has_no_frame_type() {
    let mut b = buf(FS_LS | 40, 0x0080_0010 | 1 << 24, 40);
    put(&mut b, 8, 0xE000_0000);
    put(&mut b, 16, 0x0000_0402);
    // A long descriptor's d4 position (buffer byte 4 + 16) is this short
    // descriptor's payload and must not be read as a frame type.
    put(&mut b, 20, 2);
    let Ok(RxBuffer::Frame(f)) = parse(&b) else {
        panic!()
    };
    assert_eq!(f.desc.frame_type, None);
    assert_eq!(f.payload[0], 2);
    assert!(!f.desc.long && f.desc.mac_info_valid);
    assert_eq!((f.desc.pkt_type, f.desc.bw), (1, 7));
    assert!(f.desc.sw_dec && !f.desc.hw_dec && f.desc.icv_err);
    assert_eq!(f.payload.as_ptr(), b[20..].as_ptr());
    assert_eq!(RX_TYPE_WIFI, 0);
    assert_eq!((RX_TYPE_PPDU_STAT, RX_TYPE_C2H), (1, 10));
    assert_eq!((FRAME_MGNT, FRAME_DATA), (0, 2));
}

#[test]
fn every_truncation_and_every_descriptor_never_panics() {
    let mut b = buf(FS_LS | 70, 0xFFFF_FFFF, 70);
    for x in b.iter_mut().skip(8) {
        *x = 0xFF;
    }
    for n in 0..=b.len() {
        let _ = parse(&b[..n]);
    }
    for d0 in [0u32, 0x8000_0000, 0x7000_0000, 0xFFFF_FFFF, 0x8000_3FFF] {
        for w in [0u16, 4, 20, 36, 44, 60, 0x3FFF] {
            let _ = parse(&buf(FS_LS | u32::from(w), d0, 70));
            let _ = parse(&buf(FS_LS | u32::from(w), d0, 20));
        }
    }
}

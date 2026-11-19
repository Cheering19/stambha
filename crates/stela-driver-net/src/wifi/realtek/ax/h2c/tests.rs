use super::*;
use crate::wifi::realtek::ax::elements::RfPath;
use alloc::vec;

const S8_PACKET: [u8; 32] = [
    0x10, 0x00, 0x00, 0x0D, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x25, 0x14, 0x00, 0x00, 0x10, 0xC0,
    0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x5E, 0x00, 0x00, 0x00,
];

#[test]
fn ofld_cfg_packet_is_byte_exact() {
    assert_eq!(H2c::after_download().ofld_cfg(), (0, S8_PACKET.to_vec()));
}

#[test]
fn rec_ack_forced_every_fourth_seq() {
    let mut h = H2c::after_download();
    for want in 0u8..5 {
        let (seq, p) = h.packet(1, 9, 0x14, false, &[]);
        assert_eq!(seq, want);
        assert_eq!(p.len(), 24);
        let hdr0 = u32::from_le_bytes(p[16..20].try_into().unwrap());
        let hdr1 = u32::from_le_bytes(p[20..24].try_into().unwrap());
        assert_eq!(hdr0 >> 24, u32::from(want));
        assert_eq!(hdr1 & 0x3FFF, 8);
        assert_eq!(hdr1 & (1 << 14) != 0, want % 4 == 0, "seq {want}");
        assert_eq!(hdr1 & (1 << 15), 0);
    }
    // 5 calls made so far; 251 more reach 256 and wrap to 0.
    for _ in 0..251 {
        h.packet(1, 9, 0x14, false, &[]);
    }
    assert_eq!(h.packet(1, 9, 0x14, false, &[]).0, 0);
}

#[test]
fn explicit_rec_ack_is_never_cleared_and_payload_is_carried() {
    let mut h = H2c::after_download();
    h.packet(0, 0, 0, false, &[]);
    let (seq, p) = h.packet(1, 9, 0x14, true, &[1, 2, 3]);
    assert_eq!(seq, 1);
    assert_eq!(&p[24..], &[1, 2, 3]);
    let hdr1 = u32::from_le_bytes(p[20..24].try_into().unwrap());
    assert_eq!(hdr1, 11 | 1 << 15);
    // descriptor pkt_size = 8 + payload
    assert_eq!(u32::from_le_bytes(p[..4].try_into().unwrap()), 0x0D00_000B);
}

#[test]
fn rf_reg_packet_is_byte_exact() {
    let mut h = H2c::after_download();
    assert_eq!(h.ofld_cfg().0, 0);
    let mut want = vec![0x10, 0x00, 0x00, 0x0D];
    want.extend_from_slice(&[0; 12]);
    want.extend_from_slice(&[0x26, 0x00, 0x00, 0x01, 0x10, 0x00, 0x00, 0x00]);
    want.extend_from_slice(&[0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x03, 0x00]);
    assert_eq!(
        h.rf_reg(RfPath::B, 0, &[0x0050_0000, 0x0003_0000]),
        (1, want)
    );
    // Path A is class 8.
    let (seq, p) = h.rf_reg(RfPath::A, 0, &[0x0050_0000]);
    assert_eq!(seq, 2);
    assert_eq!(&p[16..20], &[0x22, 0x00, 0x00, 0x02]);
    // func carries the page.
    let (seq, p) = h.rf_reg(RfPath::B, 2, &[0x0050_0000]);
    assert_eq!(seq, 3);
    assert_eq!(&p[16..20], &[0x26, 0x02, 0x00, 0x03]);
    // Seq 4 falls on the forced REC_ACK; DONE_ACK is never asked for.
    let (seq, p) = h.rf_reg(RfPath::B, 0, &[0x0050_0000]);
    assert_eq!(seq, 4);
    let hdr1 = u32::from_le_bytes(p[20..24].try_into().unwrap());
    assert_eq!(hdr1, 12 | 1 << 14);
}

#[test]
fn rf_reg_classes_and_category_are_linuxs() {
    assert_eq!(H2C_CAT_OUTSRC, 2);
    assert_eq!(H2C_CL_OUTSRC_RF_REG_A, 8);
    assert_eq!(H2C_CL_OUTSRC_RF_REG_B, 9);
}

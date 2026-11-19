use super::*;
use crate::wifi::realtek::ax::chan::HwBand;
use crate::wifi::realtek::ax::elements::RfPath;
use crate::wifi::realtek::ax::io::AxError;
use alloc::format;
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

// ---- M2c: the H2Cs of `rtw89_core_start`, `rtw89_mac_vif_init` and the
// scan offload (spec S3, S7.3, S7.4, S9.2, S9.3). ----

/// Splits a CH12 packet into its descriptor's `pkt_size` and the bytes
/// after the 16-byte descriptor, checking that the descriptor carries only
/// the length and the H2C RX type.
fn body(p: &[u8]) -> (u32, &[u8]) {
    let dword0 = u32::from_le_bytes(p[..4].try_into().unwrap());
    assert_eq!(dword0 & !0x3FFF, 0x0D00_0000, "descriptor type");
    assert_eq!(&p[4..16], &[0; 12], "descriptor tail");
    assert_eq!(dword0 & 0x3FFF, (p.len() - 16) as u32, "pkt_size");
    (dword0 & 0x3FFF, &p[16..])
}

fn sha256(bytes: &[u8]) -> [u8; 32] {
    use sha2::{Digest, Sha256};
    Sha256::digest(bytes).into()
}

fn hex32(s: &str) -> [u8; 32] {
    let mut out = [0; 32];
    for (i, b) in out.iter_mut().enumerate() {
        *b = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap();
    }
    out
}

/// Bytes from a spec vector: hex pairs separated by spaces and `|`.
fn hex(s: &str) -> Vec<u8> {
    s.split([' ', '|'])
        .filter(|t| !t.is_empty())
        .map(|t| u8::from_str_radix(t, 16).unwrap())
        .collect()
}

fn concat(parts: &[&[u8]]) -> Vec<u8> {
    parts.iter().flat_map(|p| p.iter().copied()).collect()
}

fn word(b: &[u8], i: usize) -> u32 {
    u32::from_le_bytes(b[4 * i..4 * i + 4].try_into().unwrap())
}

#[test]
fn fw_log_packet_is_byte_exact() {
    let (seq, p) = H2c::at(3).fw_log();
    assert_eq!(seq, 3);
    let (len, b) = body(&p);
    assert_eq!(len, 20);
    assert_eq!(
        b,
        &[
            0x01, 0x00, 0x00, 0x03, 0x14, 0x00, 0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        ]
    );
}

#[test]
fn ba_cam_packets_are_byte_exact() {
    let mut h = H2c::at(4);
    let packets: Vec<(u8, Vec<u8>)> = (0..8u8).map(|i| h.ba_cam_init(8 + i, i)).collect();
    let want: [(u8, [u8; 16]); 4] = [
        (
            4,
            [
                0x31, 0x00, 0x00, 0x04, 0x10, 0xC0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x80,
            ],
        ),
        (
            5,
            [
                0x31, 0x00, 0x00, 0x05, 0x10, 0x80, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,
                0x00, 0x90,
            ],
        ),
        // Seq 8 is the other forced REC_ACK.
        (
            8,
            [
                0x31, 0x00, 0x00, 0x08, 0x10, 0xC0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00,
                0x00, 0xC0,
            ],
        ),
        (
            11,
            [
                0x31, 0x00, 0x00, 0x0B, 0x10, 0x80, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x07, 0x00,
                0x00, 0xF0,
            ],
        ),
    ];
    for (seq, bytes) in want {
        let (got_seq, p) = &packets[usize::from(seq - 4)];
        assert_eq!(*got_seq, seq);
        let (len, b) = body(p);
        assert_eq!(len, 16, "seq {seq}");
        assert_eq!(b, &bytes, "seq {seq}");
    }
}

#[test]
fn macid_pause_unpause_is_byte_exact() {
    let (seq, p) = H2c::at(12).macid_pause(0, false);
    assert_eq!(seq, 12);
    let (len, b) = body(&p);
    assert_eq!(len, 40);
    let want = concat(&[
        &[0x25, 0x08, 0x00, 0x0C, 0x28, 0x40, 0x00, 0x00],
        &[0; 16],
        &[0x01, 0x00, 0x00, 0x00],
        &[0; 12],
    ]);
    assert_eq!(b, &want[..]);
}

#[test]
fn macid_pause_sets_the_group_and_bit_of_the_macid() {
    // macid 37: group 1, bit 5; pausing sets the pause word as well as the
    // mask (fw.c:5216-5220).
    let (_, p) = H2c::at(1).macid_pause(37, true);
    let b = body(&p).1;
    let words: Vec<u32> = (2..10).map(|i| word(b, i)).collect();
    assert_eq!(words, [0, 1 << 5, 0, 0, 0, 1 << 5, 0, 0]);
    // REC_ACK is asked for on every macid pause (fw.c:5224), not only on
    // seq % 4 == 0.
    assert_eq!(word(b, 1), 40 | 1 << 14);
    // macid 127: group 3, bit 31.
    let (_, p) = H2c::at(1).macid_pause(127, false);
    let b = body(&p).1;
    assert_eq!(word(b, 2 + 7), 1 << 31);
}

#[test]
fn role_and_join_are_byte_exact() {
    let mut h = H2c::at(13);
    let (seq, p) = h.role_maintain(0, 0, 0);
    assert_eq!(seq, 13);
    let (len, b) = body(&p);
    assert_eq!(len, 12);
    assert_eq!(
        b,
        &[
            0x21, 0x04, 0x00, 0x0D, 0x0C, 0x80, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00
        ]
    );
    let (seq, p) = h.join_info_disconnect(0, 0);
    assert_eq!(seq, 14);
    let (len, b) = body(&p);
    assert_eq!(len, 12);
    assert_eq!(
        b,
        &[
            0x21, 0x00, 0x00, 0x0E, 0x0C, 0x80, 0x00, 0x00, 0x00, 0x01, 0x00, 0x04
        ]
    );
}

#[test]
fn role_and_join_place_macid_port_and_update_mode() {
    // ROLE_MAINTAIN: macid [7:0], upd_mode [12:10], wifi_role 1 [16:13],
    // port [21:19] (fw.h:1813-1819).
    let (_, p) = H2c::at(1).role_maintain(5, 3, 3);
    assert_eq!(word(body(&p).1, 2), 5 | 3 << 10 | 1 << 13 | 3 << 19);
    // JOININFO: macid [7:0], op b8, port [23:21], wifi_role 1 [29:26]
    // (fw.h:1837-1849).
    let (_, p) = H2c::at(1).join_info_disconnect(5, 3);
    assert_eq!(word(body(&p).1, 2), 5 | 1 << 8 | 3 << 21 | 1 << 26);
}

#[test]
fn addr_cam_is_byte_exact_for_the_test_mac() {
    let (seq, p) = H2c::at(15).addr_cam([0x02, 0x11, 0x22, 0x33, 0x44, 0x55], 0, 0);
    assert_eq!(seq, 15);
    let (len, b) = body(&p);
    assert_eq!(len, 68);
    // Spec S7.4's vector, as printed there.
    let want = hex(concat!(
        "19 00 00 0F 44 80 00 00 | 00 00 00 00 00 00 20 00 01 00 13 00 00 00 00 00 02 11 22 33 ",
        "44 55 00 00 ",
        "00 00 00 00 00 00 00 00 00 00 00 00 00 00 02 00 00 00 00 00 00 00 00 00 00 00 08 00 ",
        "FD 00 00 00 00 00 00 00"
    ));
    assert_eq!(want.len(), 68);
    assert_eq!(b, &want[..]);
}

#[test]
fn addr_cam_places_port_and_macid_in_w8() {
    // w8 (v0): macid [7:0], PORT_INT [10:8], TSF_SYNC [13:11] (cam.h:73-75).
    let (_, p) = H2c::at(1).addr_cam([0; 6], 9, 2);
    let b = body(&p).1;
    assert_eq!(word(b, 2 + 8), 9 | 2 << 8 | 2 << 11);
    // An all-zero MAC hashes to 0, leaving only VALID in w2.
    assert_eq!(word(b, 2 + 2), 1);
}

#[test]
fn default_cmac_tbl_matches_its_hash() {
    let (seq, p) = H2c::at(16).default_cmac_tbl(0);
    assert_eq!(seq, 16);
    let (len, b) = body(&p);
    assert_eq!(len, 76);
    assert_eq!(&b[..8], &[0x15, 0x0A, 0x00, 0x10, 0x4C, 0xC0, 0x00, 0x00]);
    let payload = &b[8..];
    assert_eq!(payload.len(), 68);
    assert_eq!(
        sha256(payload),
        hex32("3bd688b8d3ed50b27c2099eb83420e33eca08b299ee939a7e3b6ee0e63dd9051")
    );
    // The words the hash stands for (spec S7.3).
    let words: Vec<u32> = (0..17).map(|i| word(payload, i)).collect();
    let mut want = [0u32; 17];
    want[0] = 0x80;
    want[9] = 0x0020_0000;
    want[15] = 0x0F0C_0000;
    assert_eq!(words, want);
}

#[test]
fn default_cmac_tbl_carries_a_seven_bit_macid() {
    // SET_CTRL_INFO_MACID is GENMASK(6, 0) (fw.h:734-737).
    let (_, p) = H2c::at(1).default_cmac_tbl(0x85);
    assert_eq!(word(body(&p).1, 2), 0x05 | 0x80);
}

/// Spec S1.4: 2.4 GHz 1-13, then 5 GHz 36-64, 100-144 and 149-165 in steps
/// of 4, each period 105, dwell 20, DFS.
fn spec_list() -> Vec<ScanChannel> {
    let g2 = (1..=13).map(|c| (HwBand::G2, c));
    let g5 = (36..=64)
        .step_by(4)
        .chain((100..=144).step_by(4))
        .chain((149..=165).step_by(4))
        .map(|c| (HwBand::G5, c));
    g2.chain(g5)
        .map(|(band, channel)| ScanChannel {
            band,
            channel,
            period: 105,
            dwell: 20,
            dfs: true,
        })
        .collect()
}

#[test]
fn add_scanofld_ch_matches_the_spec_list() {
    let list = spec_list();
    assert_eq!(list.len(), 38);
    let (seq, p) = H2c::at(18).add_scanofld_ch(&list).unwrap();
    assert_eq!(seq, 18);
    assert_eq!(p.len(), 1092);
    let head = concat(&[
        &[0x34, 0x04, 0x00, 0x0D],
        &[0; 12],
        &[0x25, 0x16, 0x00, 0x12, 0x34, 0xC4, 0x00, 0x00],
    ]);
    assert_eq!(&p[..24], &head[..]);
    assert_eq!(body(&p).0, 1076);
    let payload = &p[24..];
    assert_eq!(payload.len(), 1068);
    assert_eq!(&payload[..4], &[0x26, 0x07, 0x00, 0x00]);
    assert_eq!(
        sha256(payload),
        hex32("03fc97f3c0505402e7e6c2de291051726ac7cca6855253467afd321bf2d9ef53")
    );
    let entry = |i: usize| &payload[4 + 28 * i..4 + 28 * (i + 1)];
    let e0 = concat(&[&[0x69, 0x14, 0x01, 0x01, 0xF8, 0x30, 0xFF, 0x01], &[0; 20]]);
    let e13 = concat(&[&[0x69, 0x14, 0x24, 0x24, 0xF8, 0x70, 0xFF, 0x01], &[0; 20]]);
    assert_eq!(entry(0), &e0[..]);
    assert_eq!(entry(13), &e13[..]);
}

#[test]
fn scan_channel_words_follow_the_chinfo_bits() {
    // fw.h:2955-2969: a non-DFS entry still pauses data (an ACTIVE channel
    // sets pause_data too, fw.c:9705-9707) and asks for TX; the band takes
    // [15:14].
    let c = ScanChannel {
        band: HwBand::G6,
        channel: 37,
        period: 45,
        dwell: 0,
        dfs: false,
    };
    assert_eq!(
        c.words(),
        [
            45 | 37 << 16 | 37 << 24,
            0x1F << 3 | 1 << 12 | 1 << 13 | 2 << 14 | 0xFF << 16,
            0,
            0,
            0,
            0,
            0
        ]
    );
    let d = ScanChannel { dfs: true, ..c };
    assert_eq!(d.words()[1], c.words()[1] | 1 << 24);
}

#[test]
fn add_scanofld_ch_refuses_70_entries_without_advancing_seq() {
    let one = spec_list()[0];
    let mut h = H2c::at(18);
    assert_eq!(SCAN_LIST_LIMIT, 69);
    assert_eq!(
        h.add_scanofld_ch(&[one; 70]),
        Err(AxError::Scan("scan list past 69 channels"))
    );
    assert_eq!(h, H2c::at(18));
    // 69 is the AX batch limit (fw.h:349) and is accepted.
    let (seq, p) = h.add_scanofld_ch(&[one; 69]).unwrap();
    assert_eq!(seq, 18);
    assert_eq!(p.len(), 16 + 8 + 4 + 28 * 69);
    assert_eq!(p[24], 69);
    assert_eq!(h, H2c::at(19));
}

#[test]
fn scan_error_display_names_the_scan() {
    assert_eq!(
        format!("{}", AxError::Scan("scan list past 69 channels")),
        "scan: scan list past 69 channels"
    );
}

#[test]
fn scanofld_start_and_stop_are_byte_exact() {
    let mut h = H2c::at(19);
    let (seq, p) = h.scanofld(0, 0, true);
    assert_eq!(seq, 19);
    let mut want = concat(&[
        &[0x24, 0x00, 0x00, 0x0D],
        &[0; 12],
        &[0x25, 0x17, 0x00, 0x13, 0x24, 0xC0, 0x00, 0x00],
        &[0x00, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00],
        &[0; 20],
    ]);
    assert_eq!(p, want);
    // Stop: w0 = 0, otherwise the same.
    let (seq, p) = h.scanofld(0, 0, false);
    assert_eq!(seq, 20);
    want[16 + 3] = 0x14;
    want[24 + 2] = 0x00;
    assert_eq!(p, want);
}

#[test]
fn scanofld_places_macid_and_port() {
    // w0: macid [7:0], port [18:16], operation [21:20] (fw.h:3047-3051).
    let (_, p) = H2c::at(1).scanofld(7, 4, true);
    assert_eq!(word(body(&p).1, 2), 7 | 4 << 16 | 1 << 20);
}

#[test]
fn packet_with_rack_sets_rec_ack_off_the_fourth_seq() {
    let (_, p) = H2c::at(1).packet_with(1, 9, 0x16, true, false, &[]);
    assert_eq!(word(&p, 5), 8 | 1 << 14);
    let (_, p) = H2c::at(1).packet_with(1, 9, 0x16, false, true, &[]);
    assert_eq!(word(&p, 5), 8 | 1 << 15);
}

#[test]
fn m2c_class_and_func_constants_are_linuxs() {
    // fw.h:4694-4811.
    assert_eq!((H2C_CL_FW_INFO, H2C_FUNC_LOG_CFG), (0, 0));
    assert_eq!((H2C_CL_BA_CAM, H2C_FUNC_MAC_BA_CAM), (0xC, 0));
    assert_eq!((H2C_CL_MAC_FR_EXCHG, H2C_FUNC_MAC_CCTLINFO_UD_V1), (5, 0xA));
    assert_eq!(
        (H2C_CL_MAC_ADDR_CAM_UPDATE, H2C_FUNC_MAC_ADDR_CAM_UPD),
        (6, 0)
    );
    assert_eq!(
        (
            H2C_CL_MAC_MEDIA_RPT,
            H2C_FUNC_MAC_JOININFO,
            H2C_FUNC_MAC_FWROLE_MAINTAIN
        ),
        (8, 0, 4)
    );
    assert_eq!(
        (
            H2C_CL_MAC_FW_OFLD,
            H2C_FUNC_MAC_MACID_PAUSE,
            H2C_FUNC_ADD_SCANOFLD_CH,
            H2C_FUNC_SCANOFLD
        ),
        (9, 8, 0x16, 0x17)
    );
}

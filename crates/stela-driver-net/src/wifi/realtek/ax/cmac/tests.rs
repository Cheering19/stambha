extern crate std;
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::{boxed::Box, vec, vec::Vec};

/// CMAC enabled; the ADDR_CAM clear bit (0xCE34 bit 8, 16-bit view) and the
/// BA CAM reset field (0xCE3C [1:0]) self-clear, as the hardware's do.
fn settled() -> RegisterModel {
    let mut m = RegisterModel::new();
    m.set32(0xC000, 0x4000_0000);
    m.set32(0x9D00, 0);
    m.on_read(
        0xCE34,
        Box::new(|m: &mut RegisterModel| m.get32(0xCE34) & !0x100),
    );
    m.on_read(
        0xCE3C,
        Box::new(|m: &mut RegisterModel| m.get32(0xCE3C) & !0x3),
    );
    m
}

#[test]
fn cmac_init_matches_linux_sequence() {
    let mut m = settled();
    // The BA CAM reset self-clears, so it did not time out.
    assert_eq!(init(&mut m), Ok(false));
    let want: Vec<Op> = vec![
        // scheduler_init_ax, mac.c:2591
        Op::W32(0xC33C, 0x41),
        Op::W32(0xC340, 0),
        Op::W32(0xC338, 0x18),
        // addr_cam_init_ax, mac.c:2564
        Op::W32(0xCE34, 0x007F_0101),
        // rx_fltr_init_ax, mac.c:2692
        Op::W32(0xCE28, 0x5555_5555),
        Op::W32(0xCE24, 0x5555_5555),
        Op::W32(0xCE2C, 0x5555_5555),
        Op::W32(0xCE20, 0x0300_44BE),
        Op::W16(0xCE04, 0x007F),
        // cca_ctrl_init_ax, mac.c:2758, with _patch_dis_resp_chk
        Op::W32(0xC390, 0x7121_00FF),
        Op::W32(0xCC00, 0x0038_0000),
        Op::W32(0xCC04, 0x1F80_0000),
        // nav_ctrl_init_ax, mac.c:2794
        Op::W32(0xCC80, 0x0403_0000),
        Op::W32(0xCC80, 0x0403_C400),
        // spatial_reuse_init_ax, mac.c:2808
        Op::W8(0xCE4A, 0),
        Op::W8(0xCE4B, 0x01),
        // tmac_init_ax, mac.c:2830
        Op::W32(0xCC20, 0),
        Op::W32(0xCA00, 0x0006_0000),
        Op::W32(0xCA1C, 0x7000),
        Op::W32(0xCA1C, 0x7700),
        // trxptcl_init_ax, mac.c:2856
        Op::W32(0xCC04, 0x1F80_110A),
        Op::W32(0xCCB0, 0x0010_0000),
        Op::W32(0xCC08, 0),
        Op::W32(0xC090, 0x80),
        // rmac_init_ax, mac.c:2919: rst_bacam reads 0 back after its poll.
        Op::W32(0xCE3C, 0x2),
        Op::W8(0xCE3C, 0x04),
        Op::W16(0xCE02, 0x20F0),
        Op::W8(0xCE00, 0x01),
        Op::W32(0xCE20, 0x0317_44BE),
        Op::W8(0xCE04, 0x6F),
        // cmac_com_init_ax, mac.c:2983
        Op::W32(0xC088, 0),
        // ptcl_init_ax, mac.c:3021
        Op::W32(0xC624, 0x0405_0000),
        Op::W32(0xC6E8, 0x3F),
        Op::W8(0xC600, 0x03),
        Op::W8(0xC600, 0x03),
        Op::W8(0xC660, 0x10),
    ];
    assert_eq!(m.writes(), want);
}

#[test]
fn scheduler_skips_prebkf_when_tx_partial_mode_set() {
    let mut m = settled();
    m.set32(0x9D00, 0x800);
    init(&mut m).unwrap();
    assert!(!m.trace.iter().any(|op| matches!(op, Op::W32(0xC338, _))));
    assert!(m.trace.contains(&Op::W32(0xC33C, 0x41)));
}

#[test]
fn addr_cam_reset_timeout() {
    let mut m = settled();
    m.on_read(
        0xCE34,
        Box::new(|m: &mut RegisterModel| m.get32(0xCE34) | 0x100),
    );
    assert_eq!(
        init(&mut m),
        Err(AxError::Timeout {
            register: 0xCE34,
            mask: 0x100
        })
    );
    assert!(m.trace.contains(&Op::R16(0xCE34)));
    // Nothing of the receive filter ran after it.
    assert!(!m.trace.iter().any(|op| matches!(op, Op::W32(0xCE28, _))));
}

#[test]
fn bacam_reset_timeout_only_warns() {
    let mut m = settled();
    m.on_read(0xCE3C, Box::new(|_: &mut RegisterModel| 2));
    // The timeout is reported, not an error, and the init carries on.
    assert_eq!(init(&mut m), Ok(true));
    assert!(m.trace.iter().any(|op| matches!(op, Op::W8(0xCE3C, _))));
}

#[test]
fn cmac_not_enabled_refused() {
    let mut m = settled();
    m.set32(0xC000, 0);
    assert_eq!(
        init(&mut m),
        Err(AxError::Mac("CMAC function is not enabled"))
    );
    assert!(m.writes().is_empty());
}

#[test]
fn dead_cmac_reads_refused() {
    for dead in [0xEAEA_EAEA, 0xDEAD_BEEF] {
        let mut m = settled();
        m.on_read(0xC000, Box::new(move |_: &mut RegisterModel| dead));
        assert!(matches!(check_cmac_en(&mut m), Err(AxError::Mac(_))));
    }
}

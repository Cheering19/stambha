// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
extern crate std;
use super::*;
use crate::wifi::realtek::ax::elements::{
    ID_BB_GAIN, ID_BB_REG, ID_RADIO_A, ID_RADIO_B, ID_RADIO_C, ID_RF_NCTL, phy_tables,
    tests::{container, element, pairs},
};
use alloc::{string::ToString, vec::Vec};

/// The elements of a file: (id, slot) in file order.
fn file(elements: &[(u32, u8)]) -> Vec<u8> {
    let mut b = container(32, 5);
    for &(id, idx) in elements {
        element(&mut b, id, [0; 4], 0, idx, &pairs(&[(0x05, 0)]));
    }
    b
}

/// Every table, radio B in slot 0 and A in slot 1, as the pinned file has
/// them.
const ALL: [(u32, u8); 5] = [
    (ID_BB_REG, 0),
    (ID_BB_GAIN, 0),
    (ID_RADIO_A, 1),
    (ID_RADIO_B, 0),
    (ID_RF_NCTL, 0),
];

fn checked(elements: &[(u32, u8)]) -> Result<(), AxError> {
    let b = file(elements);
    check_tables(&phy_tables(&b, 0).unwrap())
}

#[test]
fn check_tables_accepts_every_table_with_radio_b_then_a() {
    assert_eq!(checked(&ALL), Ok(()));
}

#[test]
fn check_tables_refuses_a_missing_or_swapped_table() {
    for (dropped, message) in [
        (0, "firmware carries no BB table"),
        (1, "firmware carries no BB gain table"),
        (2, "firmware carries no radio table for slot 1"),
        (3, "firmware carries no radio table for slot 0"),
        (4, "firmware carries no NCTL table"),
    ] {
        let mut elements = ALL.to_vec();
        elements.remove(dropped);
        assert_eq!(checked(&elements), Err(AxError::Phy(message)), "{message}");
    }
    // Radio A in slot 0 and B in slot 1.
    assert_eq!(
        checked(&[
            (ID_BB_REG, 0),
            (ID_BB_GAIN, 0),
            (ID_RADIO_A, 0),
            (ID_RADIO_B, 1),
            (ID_RF_NCTL, 0),
        ]),
        Err(AxError::Phy("firmware radio slots are not B then A"))
    );
    // A third radio path, which the RTL8852C does not have.
    let mut elements = ALL.to_vec();
    elements.push((ID_RADIO_C, 2));
    assert_eq!(
        checked(&elements),
        Err(AxError::Phy(
            "firmware carries a radio table for a slot past 1"
        ))
    );
}

#[test]
fn check_tables_names_the_first_missing_table_in_spec_order() {
    // Nothing at all: the BB table is named first.
    assert_eq!(
        check_tables(&PhyTables::default()),
        Err(AxError::Phy("firmware carries no BB table"))
    );
    // Only the radio tables missing: slot 0 before slot 1.
    assert_eq!(
        checked(&[(ID_BB_REG, 0), (ID_BB_GAIN, 0), (ID_RF_NCTL, 0)]),
        Err(AxError::Phy("firmware carries no radio table for slot 0"))
    );
}

/// The brief's report, with the extra grant reads at their expected values.
fn report() -> PhyReport {
    PhyReport {
        rfe: 1,
        xtal: 0x2F,
        bb: 1426,
        bb_gain: 336,
        rf: [2013, 2046],
        rf_words: [392, 392],
        rf18: [0x01001, 0x01001],
        gnt: 0x6,
        gnt_sw: 0x0005_7DDF,
        gnt_before: 0x18,
        c2h_after: 0,
    }
}

#[test]
fn phy_report_display() {
    assert_eq!(
        report().to_string(),
        ", PHY rfe 1 xtal 0x2f: BB 1426, BB gain 336, RF B 2013 A 2046, RF pages 392+392, \
         RF18 A 0x01001 B 0x01001, GNT 0x00000006, GNT RFC 0x06, GNT SW 0x00057ddf, \
         GNT before coex 0x00000018"
    );
    let late = PhyReport {
        c2h_after: 2,
        ..report()
    };
    assert_eq!(
        late.to_string(),
        alloc::format!("{}, C2H after RF pages 2", report())
    );
}

#[test]
fn phy_report_rfc_bits_are_gnt_bits_4_to_1() {
    // BT granted on both paths and a status bit outside [4:1] set.
    let bt = PhyReport {
        gnt: 0x0000_0119,
        ..report()
    };
    assert!(
        bt.to_string().contains("GNT 0x00000119, GNT RFC 0x18,"),
        "{bt}"
    );
}

#[test]
#[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES"]
fn real_8852c_fw_2_passes_check_tables() {
    let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
    let f = std::fs::read(root.join("rtw89/rtw8852c_fw-2.bin")).unwrap();
    let t = phy_tables(&f, 0).unwrap();
    // The probe refuses nothing in the pinned file: radio B in slot 0, A in
    // slot 1, as spec S3 lists them.
    assert_eq!(check_tables(&t), Ok(()));
    assert_eq!(t.radio[0].map(|r| r.path), Some(RfPath::B));
    assert_eq!(t.radio[1].map(|r| r.path), Some(RfPath::A));
}

/// The brief's all-clear calibration report.
fn rfk() -> RfkReport {
    RfkReport {
        thermal: [30, 31],
        xtal: (0x2F, 0x2F),
        rck: [0x0F, 0x10],
        drck: 0x0D,
        addck: [(0x201, 0x1FF), (0x200, 0x200)],
        rx_dck_rewrites: [0, 1],
        nctl_timed_out: false,
        rck_timed_out: [false; 2],
        drck_timed_out: false,
        addck_timed_out: [false; 2],
        dack_timed_out: [false; 2],
        rx_dck_timed_out: [false; 2],
    }
}

/// Spec S16's M2b2 suffix.
const CALIBRATED: &str = ", calibrated: thermal A 30 B 31, xtal 0x2f/0x2f, RCK A 0x0f B 0x10, \
                          DRCK 0x0d, ADDCK A 0x201/0x1ff B 0x200/0x200, RX DCK rewrites A 0 B 1";

#[test]
fn rfk_report_display() {
    assert_eq!(rfk().to_string(), CALIBRATED);
    let mut dack_b = [false; 2];
    dack_b[1] = true;
    let late = RfkReport {
        nctl_timed_out: true,
        dack_timed_out: dack_b,
        ..rfk()
    };
    assert_eq!(
        late.to_string(),
        alloc::format!("{CALIBRATED}, timed out: NCTL, DACK B")
    );
}

#[test]
fn rfk_report_names_every_timeout_in_spec_order() {
    let all = RfkReport {
        nctl_timed_out: true,
        rck_timed_out: [true; 2],
        drck_timed_out: true,
        addck_timed_out: [true; 2],
        dack_timed_out: [true; 2],
        rx_dck_timed_out: [true; 2],
        ..rfk()
    };
    assert_eq!(
        all.to_string(),
        alloc::format!(
            "{CALIBRATED}, timed out: NCTL, RCK A, RCK B, DRCK, ADDCK A, ADDCK B, DACK A, DACK B, \
             RX DCK A, RX DCK B"
        )
    );
    // Each field's values are zero-padded to their width.
    let small = RfkReport {
        rck: [0x01, 0x1F],
        drck: 0x02,
        addck: [(0x003, 0x00F), (0, 0x3FF)],
        ..rfk()
    };
    assert!(
        small
            .to_string()
            .contains("RCK A 0x01 B 0x1f, DRCK 0x02, ADDCK A 0x003/0x00f B 0x000/0x3ff,"),
        "{small}"
    );
}

#[test]
fn calibrate_returns_the_rpl_bias_bb_sethw_read() {
    use crate::wifi::realtek::ax::tests::model::RegisterModel;
    let b = file(&ALL);
    let t = phy_tables(&b, 0).unwrap();
    let efuse = PhyEfuse {
        rfe_type: 1,
        xtal_cap: 0x2F,
        thermal_trim: None,
        pa_bias_trim: None,
    };
    let mut m = RegisterModel::new();
    m.emulate_rf();
    // CMAC on, for the DACK's coex RFK notify.
    m.set32(0xC000, 0x4000_0000);
    // XTAL SI: every command is done at its first read.
    m.on_read(
        0x0270,
        alloc::boxed::Box::new(|m: &mut RegisterModel| m.get32(0x0270) & 0x7FFF_FFFF),
    );
    // R_RPL_BIAS_COMP [7:0] and R_RPL_BIAS_COMP1 [7:0], each an s8.
    m.set32(0x14DF0, 0x0808_06F0);
    m.set32(0x16DF0, 0xFFFF_FF05);
    // The calibration polls time out, which is recorded, not a failure.
    let (report, offset_base) = calibrate(&mut m, &t, &efuse, 1).unwrap();
    assert_eq!(offset_base, [-16, 5]);
    assert!(report.rck_timed_out[0]);
}

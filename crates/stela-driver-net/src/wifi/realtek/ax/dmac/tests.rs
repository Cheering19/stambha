extern crate std;
use super::*;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::{boxed::Box, vec::Vec};

/// Both DLE managers report ready, as the hardware's do.
fn ready() -> RegisterModel {
    let mut io = RegisterModel::new();
    io.on_read(0x8D00, Box::new(|_: &mut RegisterModel| 3));
    io.on_read(0x9100, Box::new(|_: &mut RegisterModel| 3));
    io
}

#[test]
fn dmac_pre_init_matches_linux_sequence() {
    let mut m = ready();
    // Non-zero starting values, so every read-modify-write is visible.
    m.set32(0x7880, 0x0000_0004);
    m.set32(0x1000, 0x000E_0001);
    m.set32(0x1010, 0x000F_FFFF);
    m.set32(0x11C0, 0x0000_000F);
    m.set32(0x0088, 0x0000_0002);
    m.set32(0x8C08, 0xFFFF_FFFF);
    m.set32(0x9008, 0x8000_0000);
    m.set32(0x1700, 0x0000_0FFF);

    pre_init(&mut m).unwrap();

    let mut want: Vec<Op> = Vec::new();
    // rtw89_mac_ctrl_hci_dma_trx(true), mac.h:1642: set TXDMA|RXDMA in
    // R_AX_HCI_FUNC_EN_V1 (0x7880).
    want.extend([Op::R32(0x7880), Op::W32(0x7880, 0x0000_0007)]);
    // rtw89_mac_hci_func_en_ax, mac.c:4208 (8852C): a plain write of
    // MAC|DMAC|DISPATCHER|PKT_BUF|H_AXIDMA.
    want.push(Op::W32(0x8400, 0x6044_4000));
    // rtw89_mac_dmac_func_pre_en_ax, mac.c:4222: a plain write of CLK_EN.
    want.push(Op::W32(0x8404, 0x0004_0000));
    // R_AX_HAXI_INIT_CFG1: clear DMA_MODE [19:18] and STOP_AXI_MST (17), set
    // TXHCI_EN_V1 (7), RXHCI_EN_V1 (15), DMA mode PCIE_1B = 0.
    want.extend([Op::R32(0x1000), Op::W32(0x1000, 0x0000_8081)]);
    // R_AX_HAXI_DMA_STOP1: clear ACH0-7, CH8, CH9, CH12 (0x7FF00).
    want.extend([Op::R32(0x1010), Op::W32(0x1010, 0x0008_00FF)]);
    // R_AX_HAXI_DMA_STOP2: clear CH10, CH11.
    want.extend([Op::R32(0x11C0), Op::W32(0x11C0, 0x0000_000C)]);
    // R_AX_PLATFORM_ENABLE: set AXIDMA_EN (bit 3).
    want.extend([Op::R32(0x0088), Op::W32(0x0088, 0x0000_000A)]);
    // rtw89_mac_dle_init(DLFW), mac.c:2274: check_mac_en_ax reads DMAC_FUNC_EN.
    want.push(Op::R32(0x8400));
    // dle_func_en_ax(false), mac.c:2069: clear DLE_WDE_EN (26) | DLE_PLE_EN
    // (23), which the plain write above left clear.
    want.extend([Op::R32(0x8400), Op::W32(0x8400, 0x6044_4000)]);
    // dle_clk_en_ax(true), mac.c:2079: set WDE_CLK_EN (26) | PLE_CLK_EN (23).
    want.extend([Op::R32(0x8404), Op::W32(0x8404, 0x0484_0000)]);
    // dle_mix_cfg_ax, mac.c:2092. WDE: page 64 B (sel 0), start bound 0,
    // free (linked) pages 0, over an all-ones start.
    want.extend([Op::R32(0x8C08), Op::W32(0x8C08, 0xE000_C0FC)]);
    // PLE: page 128 B (sel 1), bound = 2048 * 64 / 8192 = 16, free 2544.
    want.extend([Op::R32(0x9008), Op::W32(0x9008, 0x89F0_1001)]);
    // wde_quota_cfg_ax, mac.c:2178: QTA0 hif, QTA1 wcpu (min = SCC's 60),
    // QTA3 pkt_in, QTA4 cpu_io. QTA2 is skipped.
    want.extend([
        Op::W32(0x8C40, 0),
        Op::W32(0x8C44, 60),
        Op::W32(0x8C4C, 0),
        Op::W32(0x8C50, 0),
    ]);
    // ple_quota_cfg_ax, mac.c:2193: QTA0..QTA11 = min | max << 16 from
    // ple_qt44 (min, mac.c:1857) and ple_qt45 (max, mac.c:1860).
    want.extend([
        Op::W32(0x9040, 0),
        Op::W32(0x9044, 0),
        Op::W32(0x9048, 16 | 32 << 16),   // c2h
        Op::W32(0x904C, 256 | 256 << 16), // h2c
        Op::W32(0x9050, 0),
        Op::W32(0x9054, 0),
        Op::W32(0x9058, 0),
        Op::W32(0x905C, 0),
        Op::W32(0x9060, 0),
        Op::W32(0x9064, 0),
        Op::W32(0x9068, 0),
        Op::W32(0x906C, 0), // tx_rpt, written for 8852C
    ]);
    // dle_func_en_ax(true): set WDE_EN | PLE_EN.
    want.extend([Op::R32(0x8400), Op::W32(0x8400, 0x64C4_4000)]);
    // chk_dle_rdy_ax, mac.c:2149: WDE (0x8D00) then PLE (0x9100), both ready
    // on the first sample.
    want.extend([Op::R32(0x8D00), Op::R32(0x9100)]);
    // rtw89_mac_hfc_init(true, false, true), mac.c:1194: check_mac_en_ax.
    want.push(Op::R32(0x8400));
    // hfc_func_en(false, false): clear FC_EN (0) and FC_CH12_EN (3).
    want.extend([Op::R32(0x1700), Op::W32(0x1700, 0x0000_0FF6)]);
    // hfc_h2c_cfg_ax, mac.c:1127: R_AX_CH_PAGE_CTRL_V1 = h2c_prec 40 << 16 ...
    want.push(Op::W32(0x1704, 40 << 16));
    // ... and CH12_FULL_COND [11:10] = h2c_full_cond 0.
    want.extend([Op::R32(0x1700), Op::W32(0x1700, 0x0000_03F6)]);
    // hfc_func_en(false, true): set CH12_EN (3), FC_EN stays clear.
    want.extend([Op::R32(0x1700), Op::W32(0x1700, 0x0000_03FE)]);

    assert_eq!(m.trace, want);
    // No delay: both polls succeed on their first sample.
    assert!(!m.trace.iter().any(|op| matches!(op, Op::Delay(_))));
}

#[test]
fn dle_ready_timeout() {
    let mut m = RegisterModel::new();
    // Bit 1 of 0x8D00 never rises: only the buffer manager is ready.
    m.on_read(0x8D00, Box::new(|_: &mut RegisterModel| 1));
    m.on_read(0x9100, Box::new(|_: &mut RegisterModel| 3));
    assert_eq!(
        pre_init(&mut m),
        Err(AxError::Timeout {
            register: 0x8D00,
            mask: 0x3
        })
    );
    // Nothing of HFC ran after the failed DLE.
    assert!(!m.trace.iter().any(|op| matches!(op, Op::W32(0x1704, _))));
}

#[test]
fn ple_ready_timeout_names_ple_register() {
    let mut m = RegisterModel::new();
    m.on_read(0x8D00, Box::new(|_: &mut RegisterModel| 3));
    m.on_read(0x9100, Box::new(|_: &mut RegisterModel| 2));
    assert_eq!(
        pre_init(&mut m),
        Err(AxError::Timeout {
            register: 0x9100,
            mask: 0x3
        })
    );
}

#[test]
fn dmac_not_enabled_is_refused_before_dle_writes() {
    // `rtw89_mac_check_mac_en_ax` reads MAC_FUNC_EN|DMAC_FUNC_EN back clear.
    let mut m = ready();
    m.on_read(0x8400, Box::new(|_: &mut RegisterModel| 0));
    assert!(matches!(pre_init(&mut m), Err(AxError::Dma(_))));
    assert!(!m.trace.iter().any(|op| matches!(op, Op::W32(0x8C08, _))));
}

#[test]
fn dmac_dead_read_is_refused() {
    let mut m = ready();
    m.on_read(0x8400, Box::new(|_: &mut RegisterModel| 0xDEAD_BEEF));
    assert!(matches!(pre_init(&mut m), Err(AxError::Dma(_))));
}

#[test]
fn dle_init_scc_matches_linux_sequence() {
    let mut m = ready();
    m.set32(0x8C08, 0xFFFF_FFFF);
    m.set32(0x9008, 0);
    m.set32(0x8400, 0x6000_0000 | 0x0480_0000);

    dle_init(&mut m, &SCC, None).unwrap();

    // mac.c:2274 with rtw8852c_dle_mem_pcie[RTW89_QTA_SCC]: wde_size19
    // {64, 3328, 0}, ple_size19 {128, 1904, 16}, wde_qt18, ple_qt46/47.
    let want = [
        Op::W32(0x8400, 0x6000_0000),
        Op::W32(0x8404, 0x0480_0000),
        Op::W32(0x8C08, 0xED00_C0FC),
        Op::W32(0x9008, 0x0770_1A01),
        Op::W32(0x8C40, 0x0C9C_0C9C),
        Op::W32(0x8C44, 0x003C_003C),
        Op::W32(0x8C4C, 0),
        Op::W32(0x8C50, 0x0028_0028),
        Op::W32(0x9040, 0x020D_020D),
        Op::W32(0x9044, 0),
        Op::W32(0x9048, 0x0020_0010),
        Op::W32(0x904C, 0x0014_0014),
        Op::W32(0x9050, 0x040A_000D),
        Op::W32(0x9054, 0x000D_000D),
        Op::W32(0x9058, 0x04AF_00B2),
        Op::W32(0x905C, 0),
        Op::W32(0x9060, 0x041D_0020),
        Op::W32(0x9064, 0x003E_003E),
        Op::W32(0x9068, 0x00A0_0008),
        Op::W32(0x906C, 0x040D_0010),
        Op::W32(0x8400, 0x6480_0000),
    ];
    assert_eq!(m.writes(), want);
}

#[test]
fn dle_init_scc_ready_timeout() {
    let mut m = RegisterModel::new();
    m.on_read(0x8D00, Box::new(|_: &mut RegisterModel| 3));
    m.on_read(0x9100, Box::new(|_: &mut RegisterModel| 1));
    m.set32(0x8400, 0x6000_0000);
    assert_eq!(
        dle_init(&mut m, &SCC, None),
        Err(AxError::Timeout {
            register: 0x9100,
            mask: 0x3
        })
    );
}

#[test]
fn preload_init_matches_linux_sequence() {
    let mut m = RegisterModel::new();
    preload_init(&mut m).unwrap();
    // preload_init_set_ax(0), mac.c:2345: USEMAXSZ = 10 * 52 = 520, B0_PRELD_FEN,
    // NXT_TXENDWIN = 1, NXT_RSVMINSZ = 52.
    assert_eq!(
        m.writes(),
        [
            Op::W32(0x9F48, 0x0208_0000),
            Op::W32(0x9F48, 0x8208_0000),
            Op::W32(0x9F4C, 0x0100),
            Op::W32(0x9F4C, 0x0134),
        ]
    );
}

fn hfc_model() -> RegisterModel {
    let mut m = RegisterModel::new();
    m.set32(0x8400, 0x6000_0000);
    m.set32(0x1700, 0x0000_0C39);
    m.set32(0x179C, 0x0C9C);
    m.set32(0x1798, 0);
    m
}

#[test]
fn hfc_init_matches_linux_sequence() {
    let mut m = hfc_model();
    hfc_init(&mut m).unwrap();

    // mac.c:1194 with en = h2c_en = true; rtw8852c_hfc_param_ini_pcie[SCC].
    let mut want = Vec::new();
    want.push(Op::W32(0x1700, 0x0000_0C30));
    for ch in 0..12u32 {
        let grp1 = matches!(ch, 4..=7 | 10 | 11);
        let val = if grp1 { 0x864E_000D } else { 0x064E_000D };
        want.push(Op::W32(0x1710 + 4 * ch, val));
    }
    want.extend([
        Op::W32(0x1790, 0x064E_064E),
        Op::W32(0x17A4, 0),
        Op::W32(0x1704, 0x0028_0002),
        Op::W32(0x1794, 0x0C9C),
        Op::W32(0x17A0, 0),
        Op::W32(0x1700, 0x0000_0010),
        Op::W32(0x1700, 0x0000_0019),
        Op::Delay(10),
    ]);
    assert_eq!(m.writes(), want);

    // hfc_get_mix_info_ax, mac.c:1053, is the last thing that runs.
    let reads: Vec<u32> = m
        .trace
        .iter()
        .filter_map(|op| match op {
            Op::R32(r) => Some(*r),
            _ => None,
        })
        .collect();
    assert_eq!(
        reads[reads.len() - 10..],
        [
            0x1798, 0x178C, 0x179C, 0x17A8, 0x1700, 0x1704, 0x1794, 0x17A0, 0x17A4, 0x1790
        ]
    );
}

#[test]
fn hfc_init_checks_dmac_before_each_step() {
    let mut m = hfc_model();
    hfc_init(&mut m).unwrap();
    // 1 start + 12 hfc_ch_ctrl + hfc_pub_ctrl + 12 hfc_upd_ch_info + mix info.
    let n = m.trace.iter().filter(|op| **op == Op::R32(0x8400)).count();
    assert_eq!(n, 1 + 12 + 1 + 12 + 1);
}

#[test]
fn hfc_init_refuses_inconsistent_public_pages() {
    let mut m = hfc_model();
    m.set32(0x179C, 0x0C9B);
    assert_eq!(
        hfc_init(&mut m),
        Err(AxError::Mac("HCI flow control public pages do not add up"))
    );
}

/// DMAC enabled, and the STA scheduler reporting `SS_INIT_DONE_1` once
/// `SS_EN` has been written.
fn sta_model() -> RegisterModel {
    let mut m = RegisterModel::new();
    m.set32(0x8400, 0x6000_0000);
    m.set32(0x9D00, 0);
    m.set32(0x9D04, 0);
    m.set32(0x9D1C, 0);
    m.on_read(
        0x9E10,
        Box::new(|m: &mut RegisterModel| m.get32(0x9E10) | 0x8000_0000),
    );
    m
}

#[test]
fn sta_sch_mpdu_sec_match_linux_sequence() {
    let mut m = sta_model();
    sta_sch_init(&mut m).unwrap();
    mpdu_proc_init(&mut m).unwrap();
    sec_eng_init(&mut m).unwrap();
    // sta_sch_init_ax (mac.c:2425), mpdu_proc_init_ax (:2458), sec_eng_init_ax
    // (:2479). The second 0x9E10 write is the NONEMPTY clear, on a value that
    // already has bit 28 clear.
    assert_eq!(
        m.writes(),
        [
            Op::W8(0x9E10, 0x01),
            Op::W32(0x9E10, 0xA000_0001),
            Op::W32(0x9E10, 0xA000_0001),
            Op::W32(0x9E50, 0x0A00),
            Op::W32(0x9C04, 0x02A9_5A95),
            Op::W32(0x9C14, 0x0000_AA55),
            Op::W32(0x9C00, 0x3),
            Op::W32(0x9C40, 0x010E_05F0),
            Op::W32(0x9D00, 0x71F),
            Op::W32(0x9D04, 0x3),
            Op::W32(0x9D1C, 0x8000_0000),
        ]
    );
}

/// `B_AX_TX_PARTIAL_MODE`, `R_AX_SEC_ENG_CTRL` bit 11 (`reg.h`).
const B_AX_TX_PARTIAL_MODE: u32 = 1 << 11;

#[test]
fn sec_eng_keeps_tx_partial_mode() {
    let mut m = sta_model();
    m.set32(0x9D00, B_AX_TX_PARTIAL_MODE);
    sec_eng_init(&mut m).unwrap();
    assert_eq!(m.writes()[0], Op::W32(0x9D00, 0xF1F));
}

#[test]
fn sta_sch_init_timeout() {
    let mut m = sta_model();
    m.on_read(0x9E10, Box::new(|m: &mut RegisterModel| m.get32(0x9E10)));
    assert_eq!(
        sta_sch_init(&mut m),
        Err(AxError::Timeout {
            register: 0x9E10,
            mask: 0x8000_0000
        })
    );
}

#[test]
fn sta_sch_mpdu_sec_refuse_a_disabled_dmac() {
    for step in [sta_sch_init, mpdu_proc_init, sec_eng_init] {
        let mut m = sta_model();
        m.set32(0x8400, 0);
        assert!(matches!(step(&mut m), Err(AxError::Dma(_))));
        assert!(m.writes().is_empty());
    }
}

#[test]
fn dmac_init_runs_in_linux_order() {
    let mut m = ready();
    m.set32(0x8400, 0x6000_0000);
    m.set32(0x1700, 0x0000_0C39);
    m.set32(0x179C, 0x0C9C);
    m.on_read(
        0x9E10,
        Box::new(|m: &mut RegisterModel| m.get32(0x9E10) | 0x8000_0000),
    );
    init(&mut m).unwrap();
    let pos = |want: fn(&Op) -> bool| m.trace.iter().position(want).unwrap();
    let dle = pos(|op| matches!(op, Op::W32(0x8C40, _)));
    let preload = pos(|op| matches!(op, Op::W32(0x9F48, _)));
    let hfc = pos(|op| matches!(op, Op::W32(0x1790, _)));
    let sta = pos(|op| matches!(op, Op::W8(0x9E10, _)));
    let mpdu = pos(|op| matches!(op, Op::W32(0x9C04, _)));
    let sec = pos(|op| matches!(op, Op::W32(0x9D1C, _)));
    assert!(dle < preload && preload < hfc && hfc < sta && sta < mpdu && mpdu < sec);
}

extern crate std;
use super::*;
use crate::wifi::realtek::ax::chan::HwBand;
use crate::wifi::realtek::ax::command::Refused;
use crate::wifi::realtek::ax::h2c::H2c;
use crate::wifi::realtek::ax::pci::FWCMD_SLOT_BYTES;
use crate::wifi::realtek::ax::pci::tests::{FWCMD, device_read, running};
use crate::wifi::realtek::ax::receive::Tag;
use crate::wifi::realtek::ax::tests::firmware::Firmware;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use alloc::vec::Vec;

const G2_1: Tag = Tag {
    band: HwBand::G2,
    channel: 1,
};
const TEST_MAC: [u8; 6] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];

/// Spec S7.1 from a zeroed register file, writes and delays only: every
/// port 0 register op of `rtw89_mac_port_update` for a NO_LINK station
/// whose port function was off.
fn port_update_from_zero() -> Vec<Op> {
    let mut want = Vec::from([Op::W32(0xC400, 0); 7]);
    want.extend_from_slice(&[
        Op::W32(0xC414, 0x64),
        Op::W8(0xC590, 0),
        Op::W8(0xCA08, 3),
        Op::W16(0xC426, 0),
        Op::W32(0xC63C, 0),
        Op::W32(0xC404, 2),
        Op::W32(0xC404, 0x00C8_0002),
        Op::W32(0xC408, 0),
        Op::W16(0xC40E, 5),
        Op::W16(0xC412, 0x100),
        Op::W32(0xC6A0, 0),
        Op::W32(0xC568, 0),
        Op::W32(0xC400, 4),
        Op::Delay(20),
        Op::W32(0xC40C, 0x0005_00A0),
        Op::W32(0xCE84, 0),
    ]);
    want
}

#[test]
fn port_update_from_reset_matches_linux_sequence() {
    let mut io = RegisterModel::new();
    port_update(&mut io).unwrap();
    assert_eq!(io.trace[0], Op::R32(0xC400));
    assert_eq!(io.writes(), port_update_from_zero());
    assert!(!io.trace.contains(&Op::Delay(1000)));
}

#[test]
fn port_update_reads_each_register_at_linuxs_width() {
    let mut io = RegisterModel::new();
    port_update(&mut io).unwrap();
    let reads: Vec<Op> = io
        .trace
        .iter()
        .copied()
        .filter(|o| matches!(o, Op::R8(_) | Op::R16(_) | Op::R32(_)))
        .collect();
    // func_sw's PORT_FUNC_EN test, then one read per read-modify-write;
    // hiq_win is a plain write8 and has none.
    let mut want = Vec::from([Op::R32(0xC400); 8]);
    want.extend_from_slice(&[
        Op::R32(0xC414),
        Op::R8(0xCA08),
        Op::R16(0xC426),
        Op::R32(0xC63C),
        Op::R32(0xC404),
        Op::R32(0xC404),
        Op::R32(0xC408),
        Op::R16(0xC40E),
        Op::R16(0xC412),
        Op::R32(0xC6A0),
        Op::R32(0xC568),
        Op::R32(0xC400),
        Op::R32(0xC40C),
        Op::R32(0xCE84),
    ]);
    assert_eq!(reads, want);
}

#[test]
fn port_update_with_port_on_runs_func_sw() {
    let mut io = RegisterModel::new();
    io.set32(0xC400, 0x0001_0004);
    port_update(&mut io).unwrap();
    // `msleep(beacon_int + 1)` with beacon_int 0, then PORT_FUNC_EN and
    // BRK_SETUP off, TSFTR_RST on and the beacon counter timer cleared.
    assert_eq!(
        io.trace[..5],
        [
            Op::R32(0xC400),
            Op::Delay(1000),
            Op::R32(0xC400),
            Op::W32(0xC400, 0),
            Op::R32(0xC400),
        ]
    );
    let writes = io.writes();
    assert_eq!(
        writes[..4],
        [
            Op::Delay(1000),
            Op::W32(0xC400, 0),
            Op::W32(0xC400, 0x20),
            Op::W32(0xC434, 0),
        ]
    );
    // The rest is the reset sequence with TSFTR_RST kept in 0xC400.
    let rest: Vec<Op> = port_update_from_zero()
        .into_iter()
        .map(|o| match o {
            Op::W32(0xC400, v) => Op::W32(0xC400, v | 0x20),
            o => o,
        })
        .collect();
    assert_eq!(writes[4..], rest[..]);
    assert_eq!(io.get32(0xC400), 0x24);
}

#[test]
fn port_update_clears_only_the_port_cfg_bits_linux_names() {
    // Every bit 0xC400 owns that `port_update` does not touch survives:
    // TBTT_UPD_SHIFT_SEL b15, BCN_DROP_ALLOW b14, BCN_FORCETX_EN b9,
    // TXBCN_BTCCA_EN b8, BCNERR_CNT_EN b7, BCN_AGRES b6 and above b16.
    let kept = 0xFFFE_C3C0;
    let mut io = RegisterModel::new();
    io.set32(0xC400, kept | 0x0001_3C1B);
    port_update(&mut io).unwrap();
    // PORT_FUNC_EN was off, so func_sw left TSFTR_RST alone.
    assert_eq!(io.get32(0xC400), kept | 0x4);
}

#[test]
fn port_update_keeps_the_neighbouring_fields() {
    let mut io = RegisterModel::new();
    for reg in [
        0xC404, 0xC408, 0xC40C, 0xC410, 0xC414, 0xC424, 0xC6A0, 0xCE84,
    ] {
        io.set32(reg, 0xFFFF_FFFF);
    }
    io.set32(0xC568, 0xFFFF_FFFF);
    port_update(&mut io).unwrap();
    // TBTT_SETUP [7:0] = 2 and TBTT_HOLD [27:16] = 200.
    assert_eq!(io.get32(0xC404), 0xF0C8_FF02);
    // BCN_MSK_AREA [27:16] = 0.
    assert_eq!(io.get32(0xC408), 0xF000_FFFF);
    // BCNERLY [11:0] = 160 and TBTTERLY (0xC40E) [11:0] = 5.
    assert_eq!(io.get32(0xC40C), 0xF005_F0A0);
    // TBTT_AGG_NUM (0xC412) [15:8] = 1.
    assert_eq!(io.get32(0xC410), 0x01FF_FFFF);
    // BCN_SPACE [15:0] = 100.
    assert_eq!(io.get32(0xC414), 0xFFFF_0064);
    // DTIM_NUM (0xC426) [15:8] = 0.
    assert_eq!(io.get32(0xC424), 0x00FF_FFFF);
    // BSS color of port 0 [5:0] = 0.
    assert_eq!(io.get32(0xC6A0), 0xFFFF_FFC0);
    // P0MB_ALL [23:1] cleared.
    assert_eq!(io.get32(0xC568), 0xFF00_0001);
    // BCAID_P0 [10:0] = 0.
    assert_eq!(io.get32(0xCE84), 0xFFFF_F800);
}

#[test]
fn hiq_drop_clears_port0_bits_only() {
    let mut io = RegisterModel::new();
    io.set32(0xC63C, 0x001F_0003);
    port_update(&mut io).unwrap();
    assert!(io.trace.contains(&Op::W32(0xC63C, 0x001E_0002)));
}

#[test]
fn hiq_dtim_sets_the_update_bits_and_keeps_the_rest() {
    let mut io = RegisterModel::new();
    io.set32(0xCA08, 0x0000_00F4);
    port_update(&mut io).unwrap();
    assert!(io.trace.contains(&Op::W8(0xCA08, 0xF7)));
}

#[test]
fn tbl_init_writes_dmac_then_cmac() {
    let mut io = RegisterModel::new();
    tbl_init(&mut io, 0).unwrap();
    let mut want = Vec::new();
    for i in 0..4 {
        want.push(Op::W32(0xC04, 0x1880_0000 + 4 * i));
        want.push(Op::W32(0x40000, 0));
    }
    want.extend_from_slice(&[
        Op::W32(0xC04, 0x1884_0000),
        Op::W32(0x40000, 0x4),
        Op::W32(0x40004, 0x400A_0004),
        Op::W32(0x40008, 0),
        Op::W32(0x4000C, 0),
        Op::W32(0x40010, 0),
        Op::W32(0x40014, 0x0E43_000B),
        Op::W32(0x40018, 0),
        Op::W32(0x4001C, 0x000B_8109),
    ]);
    // Plain writes: nothing is read.
    assert_eq!(io.trace, want);
}

#[test]
fn tbl_init_addresses_the_macids_entries() {
    // DMAC_TBL_BASE_ADDR + (macid << 4) + (i << 2) and CMAC_TBL_BASE_ADDR +
    // macid * CCTL_INFO_SIZE (32) (mac.c:4403-4435, mac.h:310-353).
    let mut io = RegisterModel::new();
    tbl_init(&mut io, 5).unwrap();
    let selects: Vec<Op> = io
        .trace
        .iter()
        .copied()
        .filter(|o| matches!(o, Op::W32(0xC04, _)))
        .collect();
    assert_eq!(
        selects,
        [
            Op::W32(0xC04, 0x1880_0050),
            Op::W32(0xC04, 0x1880_0054),
            Op::W32(0xC04, 0x1880_0058),
            Op::W32(0xC04, 0x1880_005C),
            Op::W32(0xC04, 0x1884_00A0),
        ]
    );
}

fn first(trace: &[Op], want: impl Fn(&Op) -> bool) -> usize {
    trace
        .iter()
        .position(want)
        .unwrap_or_else(|| panic!("not in the trace"))
}

fn kicks(trace: &[Op]) -> Vec<usize> {
    trace
        .iter()
        .enumerate()
        .filter(|(_, o)| matches!(o, Op::W16(0x1080, _)))
        .map(|(i, _)| i)
        .collect()
}

#[test]
fn vif_init_sends_five_h2cs_after_the_tables() {
    let (m, mut rings, mut io) = running();
    io.set32(0xC000, 1 << 30);
    Firmware::new(&m).install(&mut io);
    // Spec S7.3: core_tail left the sequence at 12.
    let mut h2c = H2c::at(12);
    let (got, ledger) = {
        let mut session = Session::new(&mut rings, &mut h2c, G2_1);
        // SAFETY: `io` is the modeled requester that `running` programmed
        // with these rings through `pre_init`; the model domain lets no real
        // device reach the published IOVAs, and this test is the only worker.
        let got = unsafe { init(&mut io, &mut session, TEST_MAC) };
        (got, session.ledger)
    };
    assert_eq!(got, Ok(()));
    let t = &io.trace;
    // Port update first, the tables next, the CMAC check, then the H2Cs.
    let port = first(t, |o| *o == Op::R32(0xC400));
    let tables = first(t, |o| matches!(o, Op::W32(0xC04, _)));
    let last_table = t
        .iter()
        .rposition(|o| matches!(o, Op::W32(0x4001C, _)))
        .unwrap();
    let cmac = first(t, |o| *o == Op::R32(0xC000));
    let k = kicks(t);
    assert_eq!(k.len(), 5);
    assert!(port < tables, "{port} {tables}");
    assert!(first(t, |o| *o == Op::W32(0xCE84, 0)) < tables);
    assert!(
        last_table < cmac && cmac < k[0],
        "{last_table} {cmac} {}",
        k[0]
    );
    // The CMAC check is the only read of 0xC000.
    assert_eq!(t.iter().filter(|o| **o == Op::R32(0xC000)).count(), 1);
    let slot = |n: usize, len: usize| {
        let state = m.lock().unwrap();
        device_read(
            &state,
            u64::from(FWCMD) + (n * FWCMD_SLOT_BYTES) as u64,
            len,
        )
    };
    // Task 8's packets at seq 12-16, in Linux's order.
    let mut b = H2c::at(12);
    let want = [
        b.macid_pause(MAC_ID, false),
        b.role_maintain(MAC_ID, PORT, 0),
        b.join_info_disconnect(MAC_ID, PORT),
        b.addr_cam(TEST_MAC, MAC_ID, PORT),
        b.default_cmac_tbl(MAC_ID),
    ];
    for (n, (seq, packet)) in want.iter().enumerate() {
        assert_eq!(usize::from(*seq), 12 + n);
        assert_eq!(slot(n, packet.len()), *packet, "seq {seq}");
    }
    // The H2C headers as spec S7.3 and the Task 8 vectors give them: cat,
    // class and func, seq, length and the ack bits.
    let header = |n: usize| slot(n, 24)[16..].to_vec();
    assert_eq!(header(0), [0x25, 0x08, 0x00, 0x0C, 0x28, 0x40, 0x00, 0x00]);
    assert_eq!(header(1), [0x21, 0x04, 0x00, 0x0D, 0x0C, 0x80, 0x00, 0x00]);
    assert_eq!(header(2), [0x21, 0x00, 0x00, 0x0E, 0x0C, 0x80, 0x00, 0x00]);
    assert_eq!(header(3), [0x19, 0x00, 0x00, 0x0F, 0x44, 0x80, 0x00, 0x00]);
    assert_eq!(header(4), [0x15, 0x0A, 0x00, 0x10, 0x4C, 0xC0, 0x00, 0x00]);
    // REC_ACKs: the macid pause asks for one, and seq 16 by the
    // every-fourth rule; DONE_ACKs: the other four.
    assert_eq!((ledger.rec_acks, ledger.done_acks), (2, 4));
    assert_eq!(ledger.refused, None);
    assert_eq!(h2c, H2c::at(17));
}

#[test]
fn vif_init_refuses_cmac_off_before_any_h2c() {
    let (m, mut rings, mut io) = running();
    Firmware::new(&m).install(&mut io);
    let mut h2c = H2c::at(12);
    let mut session = Session::new(&mut rings, &mut h2c, G2_1);
    // SAFETY: as in `vif_init_sends_five_h2cs_after_the_tables`.
    let got = unsafe { init(&mut io, &mut session, TEST_MAC) };
    assert!(matches!(got, Err(AxError::Mac(_))), "{got:?}");
    // The port and the tables were set up, as in Linux, before the check.
    let t = &io.trace;
    assert!(t.contains(&Op::W32(0x4001C, 0x000B_8109)));
    assert!(t.contains(&Op::R32(0xC000)));
    assert!(kicks(t).is_empty());
    assert_eq!(*session.h2c, H2c::at(12));
}

#[test]
fn vif_init_refuses_a_dead_cmac_read() {
    // RTW89_R32_DEAD and RTW89_R32_EA read as a disabled CMAC even with b30.
    for dead in [0xDEAD_BEEF, 0xEAEA_EAEA] {
        let (m, mut rings, mut io) = running();
        io.set32(0xC000, dead);
        Firmware::new(&m).install(&mut io);
        let mut h2c = H2c::at(12);
        let mut session = Session::new(&mut rings, &mut h2c, G2_1);
        // SAFETY: as in `vif_init_sends_five_h2cs_after_the_tables`.
        let got = unsafe { init(&mut io, &mut session, TEST_MAC) };
        assert!(matches!(got, Err(AxError::Mac(_))), "{dead:#x}: {got:?}");
        assert!(kicks(&io.trace).is_empty());
    }
}

#[test]
fn vif_init_records_a_refused_cam_and_goes_on() {
    let (m, mut rings, mut io) = running();
    io.set32(0xC000, 1 << 30);
    let mut fw = Firmware::new(&m);
    fw.refuse(6, 0, 2);
    fw.install(&mut io);
    let mut h2c = H2c::at(12);
    let mut session = Session::new(&mut rings, &mut h2c, G2_1);
    // SAFETY: as in `vif_init_sends_five_h2cs_after_the_tables`.
    let got = unsafe { init(&mut io, &mut session, TEST_MAC) };
    // Linux ignores the CAM's DONE_ACK: the refusal is recorded, and the
    // CMAC table H2C still goes out.
    assert_eq!(got, Ok(()));
    assert_eq!(kicks(&io.trace).len(), 5);
    assert_eq!(
        session.ledger.refused,
        Some(Refused {
            class: 6,
            func: 0,
            seq: 15,
            ret: 2
        })
    );
}

#[test]
fn vif_init_stops_at_the_first_failed_submit() {
    // No firmware reclaims CH12: the macid pause is the only kick.
    let (_m, mut rings, mut io) = running();
    io.set32(0xC000, 1 << 30);
    let mut h2c = H2c::at(12);
    let mut session = Session::new(&mut rings, &mut h2c, G2_1);
    // SAFETY: as in `vif_init_sends_five_h2cs_after_the_tables`.
    let got = unsafe { init(&mut io, &mut session, TEST_MAC) };
    assert_eq!(
        got,
        Err(AxError::Timeout {
            register: 0x1080,
            mask: 0x0FFF_0000
        })
    );
    assert_eq!(kicks(&io.trace).len(), 1);
    // Only the macid pause took a seq: the later H2Cs were never built.
    assert_eq!(*session.h2c, H2c::at(13));
}

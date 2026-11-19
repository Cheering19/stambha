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

/// Spec S3's vectors: the H2C header and payload after the 16-byte
/// descriptor.
const FW_LOG_SEQ3: [u8; 20] = [
    0x01, 0x00, 0x00, 0x03, 0x14, 0x00, 0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
];
const BA_CAM_SEQ4: [u8; 16] = [
    0x31, 0x00, 0x00, 0x04, 0x10, 0xC0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
];
const BA_CAM_SEQ11: [u8; 16] = [
    0x31, 0x00, 0x00, 0x0B, 0x10, 0x80, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0xF0,
];

#[test]
fn rts_threshold_matches_linux_sequence() {
    let mut io = RegisterModel::new();
    update_rts_threshold(&mut io).unwrap();
    assert_eq!(
        io.trace,
        [
            Op::R16(0xC614),
            Op::W16(0xC614, 0x0200),
            Op::R16(0xC614),
            Op::W16(0xC614, 0x02FF)
        ]
    );
}

#[test]
fn rts_threshold_keeps_the_other_bits() {
    let mut io = RegisterModel::new();
    io.set32(0xC614, 0xABCD_1234);
    update_rts_threshold(&mut io).unwrap();
    assert_eq!(io.get32(0xC614), 0xABCD_02FF);
}

fn first(trace: &[Op], want: impl Fn(&Op) -> bool) -> usize {
    trace
        .iter()
        .position(want)
        .unwrap_or_else(|| panic!("not in the trace"))
}

#[test]
fn core_tail_runs_ppdu_rts_then_nine_h2cs() {
    let (m, mut rings, mut io) = running();
    io.set32(0xC000, 1 << 30);
    Firmware::new(&m).install(&mut io);
    // M2b's RF pages left the sequence at 3 (spec S3).
    let mut h2c = H2c::at(3);
    let (got, ledger) = {
        let mut session = Session::new(&mut rings, &mut h2c, G2_1);
        // SAFETY: `io` is the modeled requester that `running` programmed
        // with these rings through `pre_init`; the model domain lets no real
        // device reach the published IOVAs, and this test is the only worker.
        let got = unsafe { run(&mut io, &mut session) };
        (got, session.ledger)
    };
    assert_eq!(got, Ok(()));
    let t = &io.trace;
    let ppdu = first(t, |o| *o == Op::W32(0xCE40, 0x2B));
    let rts = first(t, |o| matches!(o, Op::W16(0xC614, _)));
    let kicks: Vec<usize> = t
        .iter()
        .enumerate()
        .filter(|(_, o)| matches!(o, Op::W16(0x1080, _)))
        .map(|(i, _)| i)
        .collect();
    assert_eq!(kicks.len(), 9);
    assert!(first(t, |o| *o == Op::R32(0xC000)) < ppdu);
    assert!(ppdu < rts && rts < kicks[0], "{ppdu} {rts} {}", kicks[0]);
    assert!(t.contains(&Op::W32(0x9C18, 1)));
    let slot = |n: usize, len: usize| {
        let state = m.lock().unwrap();
        device_read(
            &state,
            u64::from(FWCMD) + (n * FWCMD_SLOT_BYTES) as u64 + 16,
            len,
        )
    };
    assert_eq!(slot(0, 20), FW_LOG_SEQ3);
    assert_eq!(slot(1, 16), BA_CAM_SEQ4);
    assert_eq!(slot(8, 16), BA_CAM_SEQ11);
    // The eight BA CAM H2Cs ask for a DONE_ACK; seq 4 and 8 also for a
    // REC_ACK. The firmware log asks for neither.
    assert_eq!((ledger.rec_acks, ledger.done_acks), (2, 8));
    assert_eq!(ledger.refused, None);
    assert_eq!(h2c, H2c::at(12));
}

#[test]
fn core_tail_refuses_a_disabled_cmac_before_any_write() {
    let (m, mut rings, mut io) = running();
    Firmware::new(&m).install(&mut io);
    let mut h2c = H2c::at(3);
    let mut session = Session::new(&mut rings, &mut h2c, G2_1);
    // SAFETY: as in `core_tail_runs_ppdu_rts_then_nine_h2cs`.
    let got = unsafe { run(&mut io, &mut session) };
    assert_eq!(got, Err(AxError::Mac("CMAC function is not enabled")));
    assert!(io.writes().is_empty(), "{:x?}", io.writes());
}

#[test]
fn core_tail_records_a_refused_ba_cam() {
    let (m, mut rings, mut io) = running();
    io.set32(0xC000, 1 << 30);
    let mut fw = Firmware::new(&m);
    fw.refuse(0xC, 0, 4);
    fw.install(&mut io);
    let mut h2c = H2c::at(3);
    let mut session = Session::new(&mut rings, &mut h2c, G2_1);
    // SAFETY: as in `core_tail_runs_ppdu_rts_then_nine_h2cs`.
    let got = unsafe { run(&mut io, &mut session) };
    // Linux ignores the BA CAM acks: the first refusal is recorded only.
    assert_eq!(got, Ok(()));
    assert_eq!(
        session.ledger.refused,
        Some(Refused {
            class: 0xC,
            func: 0,
            seq: 4,
            ret: 4
        })
    );
}

#[test]
fn core_tail_fails_when_ch12_never_reclaims() {
    let (_m, mut rings, mut io) = running();
    io.set32(0xC000, 1 << 30);
    let mut h2c = H2c::at(3);
    let mut session = Session::new(&mut rings, &mut h2c, G2_1);
    // SAFETY: as in `core_tail_runs_ppdu_rts_then_nine_h2cs`.
    let got = unsafe { run(&mut io, &mut session) };
    assert_eq!(
        got,
        Err(AxError::Timeout {
            register: 0x1080,
            mask: 0x0FFF_0000
        })
    );
    // The first kick was the last.
    assert_eq!(
        io.trace
            .iter()
            .filter(|o| matches!(o, Op::W16(0x1080, _)))
            .count(),
        1
    );
}

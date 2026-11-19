extern crate std;
use super::*;
use crate::wifi::realtek::ax::c2h::{SCAN_END, SCAN_ENTER_CH, ScanOfld};
use crate::wifi::realtek::ax::chan::HwBand;
use crate::wifi::realtek::ax::h2c::ScanChannel;
use crate::wifi::realtek::ax::pci::{
    encode_txbd,
    tests::{FWCMD, TX_POOL, device_read, device_write, reclaim_at_once, running, rxq_buffer},
};
use crate::wifi::realtek::ax::receive::Received;
use crate::wifi::realtek::ax::tests::firmware::{self, Firmware};
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use crate::wifi::realtek::beacon::tests::Model;
use alloc::{boxed::Box, sync::Arc, vec, vec::Vec};
use std::sync::Mutex;

/// The whole OFLD_CFG CH12 packet at seq 0 (spec S8).
const S8_PACKET: [u8; 32] = [
    0x10, 0x00, 0x00, 0x0D, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x25, 0x14, 0x00, 0x00, 0x10, 0xC0,
    0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x5E, 0x00, 0x00, 0x00,
];

/// An RXQ buffer as the device writes it: the RX BD info (FS | LS |
/// WRITE_SIZE), a short RX descriptor (RPKT_LEN, RPKT_TYPE) and `payload`.
fn rx_buffer(pkt_type: u8, payload: &[u8]) -> Vec<u8> {
    let write = 4 + 16 + payload.len();
    let mut b = vec![0u8; write];
    b[..4].copy_from_slice(&(0xC000 | write as u32).to_le_bytes());
    b[4..8].copy_from_slice(&(payload.len() as u32 | u32::from(pkt_type) << 24).to_le_bytes());
    b[20..].copy_from_slice(payload);
    b
}

/// A C2H: header w0 (`cat`, `class`, `func`), w1 (length), then `body`.
fn c2h(cat: u8, class: u8, func: u8, body: &[u8]) -> Vec<u8> {
    let w0 = u32::from(cat) | u32::from(class) << 2 | u32::from(func) << 8;
    let mut p = Vec::new();
    p.extend_from_slice(&w0.to_le_bytes());
    p.extend_from_slice(&(8 + body.len() as u32).to_le_bytes());
    p.extend_from_slice(body);
    rx_buffer(10, &p)
}

/// A class 0 ack (func 0 REC_ACK, 1 DONE_ACK) carrying w2.
fn ack(func: u8, w2: u32) -> Vec<u8> {
    c2h(1, 0, func, &w2.to_le_bytes())
}

/// REC_ACK w2: cat 1, class 9, func 0x14, seq 0 [23:16].
fn rec_ack() -> Vec<u8> {
    ack(0, 0x0000_1425)
}

/// DONE_ACK w2: cat 1, class 9, func 0x14, return [23:16], seq 0 [31:24].
fn done_ack(ret: u8) -> Vec<u8> {
    ack(1, 0x1425 | u32::from(ret) << 16)
}

/// Once the CH12 host index has advanced (OFLD_CFG kicked), the device
/// fills RXQ entries 0.. with `buffers` and its RXQ index reports them.
fn script(io: &mut RegisterModel, m: &Arc<Mutex<Model>>, buffers: Vec<Vec<u8>>) {
    let m = m.clone();
    let mut sent = false;
    io.on_read(
        0x1218,
        Box::new(move |r: &mut RegisterModel| {
            if !sent && r.get32(0x1080) & 0xFFF != 0 {
                let state = m.lock().unwrap();
                for (entry, buffer) in buffers.iter().enumerate() {
                    device_write(&state, rxq_buffer(entry), buffer);
                }
                sent = true;
            }
            if sent {
                (buffers.len() as u32) << 16
            } else {
                0
            }
        }),
    );
}

struct Ran {
    got: Result<H2cAck, CommandError>,
    elapsed: u64,
    m: Arc<Mutex<Model>>,
    io: RegisterModel,
    h2c: H2c,
}

/// Send OFLD_CFG at seq 0 to a running queue whose device answers with
/// `buffers`; `reclaim` makes CH12 consume the packet at once.
fn ran_with(buffers: Vec<Vec<u8>>, reclaim: bool) -> Ran {
    let (m, mut rings, mut io) = running();
    if reclaim {
        reclaim_at_once(&mut io);
    }
    script(&mut io, &m, buffers);
    let mut h2c = H2c::after_download();
    let began = io.now;
    // SAFETY: `io` is the modeled requester that `running` programmed with
    // these rings through `pre_init`; the model domain lets no real device
    // reach the published IOVAs, and this test is the only worker.
    let got = unsafe { ofld_cfg(&mut io, &mut rings, &mut h2c) };
    Ran {
        got,
        elapsed: io.now - began,
        m,
        io,
        h2c,
    }
}

fn ran(buffers: Vec<Vec<u8>>) -> Ran {
    ran_with(buffers, true)
}

fn acked(rec: bool, done: bool) -> H2cAck {
    H2cAck {
        rec,
        done,
        other_c2h: 0,
        unmatched_ack: 0,
        skipped: 0,
        log: None,
    }
}

#[test]
fn ofld_cfg_sends_the_exact_packet() {
    let mut r = ran(vec![done_ack(0)]);
    assert!(r.got.is_ok());
    let (packet, txbd) = {
        let state = r.m.lock().unwrap();
        (
            device_read(&state, u64::from(FWCMD), 32),
            device_read(&state, u64::from(TX_POOL + 12 * 16 * 8), 8),
        )
    };
    assert_eq!(packet, S8_PACKET);
    assert_eq!(txbd, encode_txbd(32, u64::from(FWCMD)));
    // Kicked once on CH12; the sequence advanced past 0.
    assert_eq!(r.io.writes()[0], Op::W16(0x1080, 1));
    assert_eq!(r.h2c.ofld_cfg().0, 1);
}

#[test]
fn rec_then_done_ack_succeeds() {
    let r = ran(vec![rec_ack(), done_ack(0)]);
    assert_eq!(r.got, Ok(acked(true, true)));
    assert!(r.elapsed < 100_000, "elapsed {}", r.elapsed);
    // Both entries were consumed and the host index written back once.
    assert_eq!(
        r.io.writes()
            .iter()
            .filter(|op| matches!(op, Op::W16(0x1218, _)))
            .copied()
            .collect::<Vec<_>>(),
        [Op::W16(0x1218, 2)]
    );
}

#[test]
fn done_ack_alone_succeeds_at_once() {
    let r = ran(vec![done_ack(0)]);
    assert_eq!(r.got, Ok(acked(false, true)));
    assert_eq!(r.elapsed, 0);
}

#[test]
fn rec_ack_alone_succeeds_after_the_deadline() {
    let r = ran(vec![rec_ack()]);
    assert_eq!(r.got, Ok(acked(true, false)));
    assert!(r.elapsed >= 100_000, "elapsed {}", r.elapsed);
    // Sampled every 50 us.
    assert!(r.io.writes().contains(&Op::Delay(50)));
}

#[test]
fn ofld_cfg_without_ack_fails_after_100ms() {
    let r = ran(vec![]);
    assert_eq!(
        r.got,
        Err(CommandError::Wait(AxError::NoAck {
            class: 9,
            func: 0x14,
            seq: 0,
            other_c2h: 0,
            unmatched_ack: 0,
            skipped: 0,
            rxq_hw: 0,
        }))
    );
    assert!(r.elapsed >= 100_000, "elapsed {}", r.elapsed);
}

#[test]
fn done_ack_with_nonzero_return_is_rejected() {
    let r = ran(vec![done_ack(3)]);
    assert_eq!(
        r.got,
        Err(CommandError::Submit(AxError::H2cRejected {
            class: 9,
            func: 0x14,
            seq: 0,
            ret: 3
        }))
    );
}

#[test]
fn unrelated_c2h_and_frames_are_counted_not_matched() {
    let r = ran(vec![
        // RTW89_CORE_RX_TYPE_WIFI (0).
        rx_buffer(0, &[0xAA; 24]),
        // A class 1 MAC C2H.
        c2h(1, 1, 0, &[0; 4]),
        // A REC_ACK for func 0x15.
        ack(0, 0x0000_1525),
        done_ack(0),
    ]);
    assert_eq!(
        r.got,
        Ok(H2cAck {
            rec: false,
            done: true,
            other_c2h: 1,
            unmatched_ack: 1,
            skipped: 1,
            log: None
        })
    );
}

#[test]
fn acks_for_another_seq_or_category_are_not_matched() {
    let r = ran(vec![
        // seq 1: REC_ACK [23:16], DONE_ACK [31:24].
        ack(0, 0x0001_1425),
        ack(1, 0x0100_1425),
        // H2C cat 0 instead of H2C_CAT_MAC.
        ack(0, 0x0000_1424),
        ack(1, 0x0000_1424),
    ]);
    // Four acks, none for OFLD_CFG at seq 0, in RXQ entries 0-3.
    assert_eq!(
        r.got,
        Err(CommandError::Wait(AxError::NoAck {
            class: 9,
            func: 0x14,
            seq: 0,
            other_c2h: 0,
            unmatched_ack: 4,
            skipped: 0,
            rxq_hw: 4,
        }))
    );
}

#[test]
fn plain_log_is_kept() {
    let r = ran(vec![c2h(1, 0, 2, b"fw: up\x01"), done_ack(0)]);
    let ack = r.got.unwrap();
    assert!(ack.done);
    assert_eq!(ack.log.map(|log| log.as_str() == "fw: up"), Some(true));
}

#[test]
fn only_the_first_plain_log_is_kept_and_segments_are_skipped() {
    let mut segment = rec_ack();
    segment[1] &= !0x40; // LS cleared: not a whole frame.
    let mut formatted = vec![0xA5, 0xA5];
    formatted.extend_from_slice(&[0; 9]);
    let r = ran(vec![
        c2h(1, 0, 2, b"first"),
        c2h(1, 0, 2, b"second"),
        c2h(1, 0, 2, &formatted),
        segment,
        done_ack(0),
    ]);
    let ack = r.got.unwrap();
    assert_eq!(ack.log.unwrap().as_str(), "first");
    assert_eq!((ack.other_c2h, ack.skipped, ack.rec), (2, 1, false));
}

#[test]
fn malformed_rx_buffer_fails_the_wait() {
    let mut short = done_ack(0);
    short[..4].copy_from_slice(&(0xC000u32 | 10).to_le_bytes());
    let r = ran(vec![short]);
    assert_eq!(
        r.got,
        Err(CommandError::Wait(AxError::Rx(
            "RX buffer shorter than its descriptor"
        )))
    );
}

#[test]
fn a_malformed_buffer_after_the_done_ack_in_one_batch_still_fails_the_wait() {
    let mut short = rec_ack();
    short[..4].copy_from_slice(&(0xC000u32 | 10).to_le_bytes());
    let r = ran(vec![done_ack(0), short]);
    assert_eq!(
        r.got,
        Err(CommandError::Wait(AxError::Rx(
            "RX buffer shorter than its descriptor"
        )))
    );
}

#[test]
fn unreclaimed_ofld_cfg_fails_the_submit() {
    let r = ran_with(vec![done_ack(0)], false);
    assert_eq!(
        r.got,
        Err(CommandError::Submit(AxError::Timeout {
            register: 0x1080,
            mask: 0x0FFF_0000
        }))
    );
    // RXQ was never read.
    assert!(!r.io.trace.contains(&Op::R32(0x1218)));
}

#[test]
fn log_text_keeps_48_printable_ascii_bytes() {
    let mut raw = vec![0u8, b'\n', 0x7F, 0xC3];
    raw.extend((0..60).map(|i| b'a' + (i % 26) as u8));
    let text = LogText::new(&raw);
    assert_eq!(text.as_str().len(), 48);
    assert!(text.as_str().starts_with("abcdefghijklmnopqrstuvwxyz"));
    assert_eq!(LogText::new(b"").as_str(), "");
}

/// RXQ entries 0.. hold `buffers` and its index reports them from the first
/// read; then [`count_c2h`] runs once.
fn counted(buffers: Vec<Vec<u8>>) -> (Result<u8, AxError>, RegisterModel) {
    let (m, mut rings, mut io) = running();
    {
        let state = m.lock().unwrap();
        for (entry, buffer) in buffers.iter().enumerate() {
            device_write(&state, rxq_buffer(entry), buffer);
        }
    }
    io.set32(0x1218, (buffers.len() as u32) << 16);
    let began = io.now;
    // SAFETY: as in `ran_with`: the modeled requester `running` programmed
    // with these rings, and no real device behind the model domain.
    let got = unsafe { count_c2h(&mut io, &mut rings) };
    // It never waits.
    assert_eq!(io.now, began);
    (got, io)
}

#[test]
fn count_c2h_counts_c2h_frames_in_one_poll_without_waiting() {
    // Two C2Hs (an ack and a log) and a frame of another type.
    let (got, io) = counted(vec![
        rec_ack(),
        c2h(1, 0, 2, b"late"),
        rx_buffer(0, &[0; 8]),
    ]);
    assert_eq!(got, Ok(2));
    // One index read, then the host index written back once for all three.
    assert_eq!(
        io.trace.iter().filter(|o| **o == Op::R32(0x1218)).count(),
        1
    );
    assert_eq!(
        io.trace
            .iter()
            .filter(|o| matches!(o, Op::W16(0x1218, _)))
            .count(),
        1
    );
    assert!(io.trace.contains(&Op::W16(0x1218, 3)));
}

#[test]
fn count_c2h_of_an_empty_rxq_is_zero_and_writes_nothing() {
    let (got, io) = counted(vec![]);
    assert_eq!(got, Ok(0));
    assert!(io.writes().is_empty(), "{:x?}", io.writes());
}

#[test]
fn count_c2h_fails_on_a_malformed_buffer() {
    let mut short = rec_ack();
    short[..4].copy_from_slice(&(0xC000u32 | 10).to_le_bytes());
    let (got, _) = counted(vec![rec_ack(), short]);
    assert_eq!(
        got,
        Err(AxError::Rx("RX buffer shorter than its descriptor"))
    );
}

// The session (spec S1.13-S1.14, S9.1).

const G2_1: Tag = Tag {
    band: HwBand::G2,
    channel: 1,
};
const BSSID: [u8; 6] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];

/// What reached a session's sink, owned.
#[derive(Debug, PartialEq, Eq)]
enum Sunk {
    Scan(ScanOfld),
    Frame(Received),
    OtherC2h,
}

fn sunk(event: Event<'_>) -> Sunk {
    match event {
        Event::C2h(C2h::ScanOfld(s)) => Sunk::Scan(s),
        Event::C2h(_) => Sunk::OtherC2h,
        Event::Frame(f) => Sunk::Frame(f),
    }
}

fn is_end(e: &Event<'_>) -> bool {
    matches!(e, Event::C2h(C2h::ScanOfld(s)) if s.reason == SCAN_END)
}

struct InSession<T> {
    got: T,
    ledger: Ledger,
    sunk: Vec<Sunk>,
    elapsed: u64,
    io: RegisterModel,
}

type Body<'a, T> =
    &'a mut dyn FnMut(&mut Session<'_>, &mut RegisterModel, &mut dyn FnMut(Event<'_>)) -> T;

/// Run `body` in a session whose next H2C carries `seq`, on running queues
/// that CH12 reclaims at once; `device` sets the device up (a `script` of
/// RXQ buffers, or a `Firmware`).
fn in_session<T>(
    seq: u8,
    device: impl FnOnce(&mut RegisterModel, &Arc<Mutex<Model>>),
    body: Body<'_, T>,
) -> InSession<T> {
    let (m, mut rings, mut io) = running();
    reclaim_at_once(&mut io);
    device(&mut io, &m);
    let mut h2c = H2c::at(seq);
    let mut out = Vec::new();
    let began = io.now;
    let (got, ledger) = {
        let mut session = Session::new(&mut rings, &mut h2c, G2_1);
        let got = body(&mut session, &mut io, &mut |e| out.push(sunk(e)));
        (got, session.ledger)
    };
    InSession {
        got,
        ledger,
        sunk: out,
        elapsed: io.now - began,
        io,
    }
}

fn scripted(buffers: Vec<Vec<u8>>) -> impl FnOnce(&mut RegisterModel, &Arc<Mutex<Model>>) {
    move |io, m| script(io, m, buffers)
}

/// An H2C cat 1 / class 9 / `func` with REC_ACK and DONE_ACK.
fn ofld(s: &mut Session<'_>, func: u8) -> (u8, Vec<u8>) {
    s.h2c.packet_with(1, 9, func, true, true, &[0; 4])
}

#[test]
fn send_drains_rxq_after_the_submit() {
    let r = in_session(0, scripted(vec![done_ack(0)]), &mut |s, io, sink| {
        let packet = s.h2c.ofld_cfg();
        // SAFETY: `io` is the modeled requester that `running` programmed
        // with these rings through `pre_init`; the model domain lets no real
        // device reach the published IOVAs, and this test is the only worker.
        unsafe { s.send(io, packet, sink) }
    });
    assert_eq!(r.got, Ok(0));
    assert_eq!(r.ledger.done_acks, 1);
    assert_eq!(r.ledger.refused, None);
    // One drain, no wait: the index was read once and written back once.
    assert_eq!(r.elapsed, 0);
    assert_eq!(
        r.io.trace.iter().filter(|o| **o == Op::R32(0x1218)).count(),
        1
    );
    assert!(r.io.trace.contains(&Op::W16(0x1218, 1)));
}

#[test]
fn refused_done_ack_is_recorded_once() {
    let r = in_session(
        13,
        scripted(vec![
            firmware::done_ack(8, 4, 13, 2),
            firmware::done_ack(8, 4, 13, 3),
        ]),
        &mut |s, io, sink| {
            let packet = s.h2c.role_maintain(0, 0, 0);
            // SAFETY: as in `send_drains_rxq_after_the_submit`.
            unsafe { s.send(io, packet, sink) }
        },
    );
    assert_eq!(r.got, Ok(13));
    assert_eq!(
        r.ledger,
        Ledger {
            done_acks: 2,
            refused: Some(Refused {
                class: 8,
                func: 4,
                seq: 13,
                ret: 2
            }),
            ..Ledger::default()
        }
    );
    // Acks are the ledger's, not the sink's.
    assert!(r.sunk.is_empty(), "{:?}", r.sunk);
}

#[test]
fn ledger_counts_rec_acks_logs_and_other_c2hs() {
    let r = in_session(
        4,
        scripted(vec![
            firmware::rec_ack(0xC, 0, 4),
            c2h(1, 0, 2, b"log"),
            c2h(1, 3, 1, &[0; 4]),
            firmware::done_ack(0xC, 0, 4, 0),
        ]),
        &mut |s, io, sink| {
            let packet = s.h2c.ba_cam_init(8, 0);
            // SAFETY: as in `send_drains_rxq_after_the_submit`.
            unsafe { s.send(io, packet, sink) }
        },
    );
    assert_eq!(r.got, Ok(4));
    assert_eq!(
        r.ledger,
        Ledger {
            rec_acks: 1,
            done_acks: 1,
            logs: 1,
            other_c2h: 1,
            refused: None,
        }
    );
}

#[test]
fn send_and_wait_matches_class_and_func_not_seq() {
    let r = in_session(
        18,
        scripted(vec![firmware::done_ack(9, 0x16, 7, 0)]),
        &mut |s, io, sink| {
            let packet = ofld(s, 0x16);
            // SAFETY: as in `send_drains_rxq_after_the_submit`.
            unsafe { s.send_and_wait(io, packet, 0x16, 0x3F, sink) }
        },
    );
    assert_eq!(r.got, Ok(18));
    assert_eq!(r.ledger.done_acks, 1);
    assert!(r.elapsed < 100_000, "elapsed {}", r.elapsed);
}

fn add_ch_returning(ret: u8) -> InSession<Result<u8, CommandError>> {
    in_session(
        18,
        scripted(vec![firmware::done_ack(9, 0x16, 18, ret)]),
        &mut |s, io, sink| {
            let packet = ofld(s, 0x16);
            // SAFETY: as in `send_drains_rxq_after_the_submit`.
            unsafe { s.send_and_wait(io, packet, 0x16, 0x3F, sink) }
        },
    )
}

#[test]
fn send_and_wait_add_ch_masks_the_return() {
    // RTW89_C2H_SCAN_DONE_ACK_RETURN (mac.c:5537-5539): b6 is not an error.
    let ok = add_ch_returning(0x40);
    assert_eq!(ok.got, Ok(18));
    assert_eq!(ok.ledger.refused, None);
    let refused = add_ch_returning(0x01);
    assert_eq!(
        refused.got,
        Err(CommandError::Submit(AxError::H2cRejected {
            class: 9,
            func: 0x16,
            seq: 18,
            ret: 1
        }))
    );
    // The wait consumed it: it is the error, not the ledger's refusal.
    assert_eq!(refused.ledger.refused, None);
}

#[test]
fn send_and_wait_times_out_after_100ms() {
    let r = in_session(19, scripted(vec![]), &mut |s, io, sink| {
        let packet = ofld(s, 0x17);
        // SAFETY: as in `send_drains_rxq_after_the_submit`.
        unsafe { s.send_and_wait(io, packet, 0x17, 0xFF, sink) }
    });
    assert_eq!(
        r.got,
        Err(CommandError::Wait(AxError::NoAck {
            class: 9,
            func: 0x17,
            seq: 19,
            other_c2h: 0,
            unmatched_ack: 0,
            skipped: 0,
            rxq_hw: 0,
        }))
    );
    assert!(r.elapsed >= 100_000, "elapsed {}", r.elapsed);
    assert!(r.io.writes().contains(&Op::Delay(50)));
}

#[test]
fn send_and_wait_counts_what_else_arrived() {
    let r = in_session(
        19,
        scripted(vec![
            // Another func's DONE_ACK with a return: unmatched, and refused.
            firmware::done_ack(9, 0x16, 18, 5),
            firmware::rec_ack(9, 0x17, 19),
            c2h(1, 0, 2, b"log"),
            firmware::scanofld(SCAN_ENTER_CH, 0, 0, 1, 105),
        ]),
        &mut |s, io, sink| {
            let packet = ofld(s, 0x17);
            // SAFETY: as in `send_drains_rxq_after_the_submit`.
            unsafe { s.send_and_wait(io, packet, 0x17, 0xFF, sink) }
        },
    );
    assert_eq!(
        r.got,
        Err(CommandError::Wait(AxError::NoAck {
            class: 9,
            func: 0x17,
            seq: 19,
            other_c2h: 2,
            unmatched_ack: 1,
            skipped: 0,
            rxq_hw: 4,
        }))
    );
    assert_eq!(
        r.ledger.refused,
        Some(Refused {
            class: 9,
            func: 0x16,
            seq: 18,
            ret: 5
        })
    );
    // The scan report still reached the sink.
    assert!(
        matches!(
            r.sunk[..],
            [Sunk::Scan(ScanOfld {
                reason: SCAN_ENTER_CH,
                ..
            })]
        ),
        "{:?}",
        r.sunk
    );
}

#[test]
fn frames_during_a_wait_reach_the_sink() {
    let [frame, status] =
        firmware::frame_with_status(0x80, [0xFF; 6], BSSID, b"Home", Some(1), 1, 0x70);
    let r = in_session(
        19,
        scripted(vec![frame, status, firmware::done_ack(9, 0x17, 19, 0)]),
        &mut |s, io, sink| {
            let packet = ofld(s, 0x17);
            // SAFETY: as in `send_drains_rxq_after_the_submit`.
            unsafe { s.send_and_wait(io, packet, 0x17, 0xFF, sink) }
        },
    );
    assert_eq!(r.got, Ok(19));
    let [Sunk::Frame(f)] = &r.sunk[..] else {
        panic!("{:?}", r.sunk);
    };
    assert_eq!(f.signal, Some(-54));
    assert_eq!(f.tag, G2_1);
    assert_eq!(&f.mpdu[16..22], &BSSID);
}

#[test]
fn a_failed_submit_leaves_rxq_untouched() {
    let (m, mut rings, mut io) = running();
    script(&mut io, &m, vec![done_ack(0)]);
    let mut h2c = H2c::at(18);
    let mut session = Session::new(&mut rings, &mut h2c, G2_1);
    let packet = ofld(&mut session, 0x16);
    // SAFETY: as in `send_drains_rxq_after_the_submit`; CH12 never reclaims.
    let got = unsafe { session.send_and_wait(&mut io, packet, 0x16, 0x3F, &mut |_| {}) };
    assert!(
        matches!(got, Err(CommandError::Submit(AxError::Timeout { .. }))),
        "{got:?}"
    );
    assert!(!io.trace.contains(&Op::R32(0x1218)));
}

#[test]
fn wait_drains_until_done_or_the_budget() {
    let r = in_session(
        0,
        scripted(vec![done_ack(0), firmware::scanofld(SCAN_END, 1, 0, 0, 0)]),
        &mut |s, io, sink| {
            let packet = s.h2c.ofld_cfg();
            // SAFETY: as in `send_drains_rxq_after_the_submit`.
            let first = unsafe { s.send(io, packet, sink) };
            // Everything arrived in `send`'s drain, so this wait finds
            // nothing and its budget passes.
            // SAFETY: as above.
            let expired = unsafe { s.wait(io, 1_000, sink, &mut is_end) };
            (first, expired)
        },
    );
    assert_eq!(r.got, (Ok(0), Ok(false)));
    assert!(r.elapsed >= 1_000, "elapsed {}", r.elapsed);
    // `send`'s drain delivered the END_SCAN to the sink.
    assert!(matches!(
        r.sunk[..],
        [Sunk::Scan(ScanOfld {
            reason: SCAN_END,
            ..
        })]
    ));
}

#[test]
fn scripted_scan_crosses_the_rxq_wrap_and_wait_ends_at_end_scan() {
    let r = in_session(
        18,
        |io, m| {
            let mut fw = Firmware::new(m);
            fw.beacon(1, BSSID, b"Home", 0x70);
            fw.install(io);
        },
        &mut |s, io, sink| {
            let list: Vec<ScanChannel> = (1..=13)
                .map(|channel| ScanChannel {
                    band: HwBand::G2,
                    channel,
                    period: 105,
                    dwell: 20,
                    dfs: true,
                })
                .collect();
            let add = s.h2c.add_scanofld_ch(&list).unwrap();
            // SAFETY: as in `send_drains_rxq_after_the_submit`.
            let added = unsafe { s.send_and_wait(io, add, 0x16, 0x3F, sink) };
            let start = s.h2c.scanofld(0, 0, true);
            // SAFETY: as above.
            let started = unsafe { s.send_and_wait(io, start, 0x17, 0xFF, sink) };
            // SAFETY: as above.
            let ended = unsafe { s.wait(io, 1_000_000, sink, &mut is_end) };
            (added, started, ended)
        },
    );
    assert_eq!(r.got, (Ok(18), Ok(19), Ok(true)));
    // 13 ENTER_CH and LEAVE_CH, the beacon and its status, END_SCAN and the
    // four acks: past RXQ's 32-entry wrap, in order.
    let entered: Vec<u8> = r
        .sunk
        .iter()
        .filter_map(|s| match s {
            Sunk::Scan(s) if s.reason == SCAN_ENTER_CH => Some(s.pri_ch),
            _ => None,
        })
        .collect();
    assert_eq!(entered, (1..=13).collect::<Vec<u8>>());
    assert!(matches!(
        r.sunk.last(),
        Some(Sunk::Scan(ScanOfld {
            reason: SCAN_END,
            status: 1,
            ..
        }))
    ));
    let frames: Vec<&Received> = r
        .sunk
        .iter()
        .filter_map(|s| match s {
            Sunk::Frame(f) => Some(f),
            _ => None,
        })
        .collect();
    let [beacon] = frames[..] else {
        panic!("{:?}", r.sunk);
    };
    assert_eq!((beacon.tag.channel, beacon.signal), (2, Some(-54)));
    assert!(!r.sunk.contains(&Sunk::OtherC2h));
    assert_eq!((r.ledger.rec_acks, r.ledger.done_acks), (2, 2));
    assert!(
        r.io.trace.contains(&Op::W16(0x1218, 0)),
        "RXQ never wrapped"
    );
}

fn scan_list(list: &[(HwBand, u8)]) -> Vec<ScanChannel> {
    list.iter()
        .map(|&(band, channel)| ScanChannel {
            band,
            channel,
            period: 105,
            dwell: 20,
            dfs: true,
        })
        .collect()
}

#[test]
fn firmware_silent_and_refused_h2cs() {
    let r = in_session(
        18,
        |io, m| {
            let mut fw = Firmware::new(m);
            fw.refuse(9, 0x16, 1);
            fw.silent(9, 0x17);
            fw.install(io);
        },
        &mut |s, io, sink| {
            let add = s
                .h2c
                .add_scanofld_ch(&scan_list(&[(HwBand::G2, 1)]))
                .unwrap();
            // SAFETY: as in `send_drains_rxq_after_the_submit`.
            let added = unsafe { s.send_and_wait(io, add, 0x16, 0x3F, sink) };
            let start = s.h2c.scanofld(0, 0, true);
            // SAFETY: as above.
            let started = unsafe { s.send_and_wait(io, start, 0x17, 0xFF, sink) };
            (added, started)
        },
    );
    assert_eq!(
        r.got.0,
        Err(CommandError::Submit(AxError::H2cRejected {
            class: 9,
            func: 0x16,
            seq: 18,
            ret: 1
        }))
    );
    assert!(
        matches!(
            r.got.1,
            Err(CommandError::Wait(AxError::NoAck {
                func: 0x17,
                seq: 19,
                ..
            }))
        ),
        "{:?}",
        r.got.1
    );
    // A silent SCANOFLD start scans nothing.
    assert!(r.sunk.is_empty(), "{:?}", r.sunk);
}

#[test]
fn firmware_probe_response_goes_to_the_cam_mac_and_a_burst_fills_rxq() {
    const MAC: [u8; 6] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];
    let other = [0x02, 0x66, 0, 0, 0, 7];
    let mut full = 0;
    let r = in_session(
        15,
        |io, m| {
            let mut fw = Firmware::new(m);
            fw.probe_response(1, other, b"Lab", 0x50);
            fw.burst(31);
            fw.install(io);
        },
        &mut |s, io, sink| {
            let cam = s.h2c.addr_cam(MAC, 0, 0);
            // SAFETY: as in `send_drains_rxq_after_the_submit`.
            let cammed = unsafe { s.send(io, cam, sink) };
            let list = scan_list(&[(HwBand::G2, 1), (HwBand::G5, 36)]);
            let add = s.h2c.add_scanofld_ch(&list).unwrap();
            // SAFETY: as above.
            let added = unsafe { s.send_and_wait(io, add, 0x16, 0x3F, sink) };
            let start = s.h2c.scanofld(0, 0, true);
            // SAFETY: as above.
            let started = unsafe { s.send_and_wait(io, start, 0x17, 0xFF, sink) };
            // SAFETY: as above.
            let ended = unsafe { s.wait(io, 100_000, sink, &mut is_end) };
            full = s.rings().rxq_full_batches();
            (cammed, added, started, ended)
        },
    );
    assert_eq!(r.got, (Ok(15), Ok(16), Ok(17), Ok(true)));
    let frames: Vec<&Received> = r
        .sunk
        .iter()
        .filter_map(|s| match s {
            Sunk::Frame(f) => Some(f),
            _ => None,
        })
        .collect();
    let [probe] = frames[..] else {
        panic!("{:?}", r.sunk);
    };
    assert_eq!(probe.mpdu[0], 0x50);
    assert_eq!(&probe.mpdu[4..10], &MAC);
    assert_eq!(&probe.mpdu[16..22], &other);
    assert_eq!(
        probe.tag,
        Tag {
            band: HwBand::G5,
            channel: 36
        }
    );
    assert_eq!(probe.signal, Some(-70));
    // The 31 data frames filled every entry the device may fill, once.
    assert_eq!(full, 1);
}

#[test]
fn firmware_without_end_answers_the_stop() {
    let r = in_session(
        18,
        |io, m| {
            let mut fw = Firmware::new(m);
            fw.no_end();
            fw.end_on_stop();
            fw.end_status(2);
            fw.install(io);
        },
        &mut |s, io, sink| {
            let add = s
                .h2c
                .add_scanofld_ch(&scan_list(&[(HwBand::G2, 6)]))
                .unwrap();
            // SAFETY: as in `send_drains_rxq_after_the_submit`.
            let added = unsafe { s.send_and_wait(io, add, 0x16, 0x3F, sink) };
            let start = s.h2c.scanofld(0, 0, true);
            // SAFETY: as above.
            let started = unsafe { s.send_and_wait(io, start, 0x17, 0xFF, sink) };
            // SAFETY: as above.
            let unended = unsafe { s.wait(io, 10_000, sink, &mut is_end) };
            let stop = s.h2c.scanofld(0, 0, false);
            // SAFETY: as above.
            let stopped = unsafe { s.send_and_wait(io, stop, 0x17, 0xFF, sink) };
            // SAFETY: as above.
            let ended = unsafe { s.wait(io, 100_000, sink, &mut is_end) };
            (added, started, unended, stopped, ended)
        },
    );
    assert_eq!(r.got, (Ok(18), Ok(19), Ok(false), Ok(20), Ok(true)));
    assert!(matches!(
        r.sunk.last(),
        Some(Sunk::Scan(ScanOfld {
            reason: SCAN_END,
            status: 2,
            ..
        }))
    ));
}

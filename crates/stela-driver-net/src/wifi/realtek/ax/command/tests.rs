extern crate std;
use super::*;
use crate::wifi::realtek::ax::pci::{
    encode_txbd,
    tests::{FWCMD, TX_POOL, device_read, device_write, reclaim_at_once, running, rxq_buffer},
};
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

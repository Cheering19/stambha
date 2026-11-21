//! A scripted RTL8852C firmware for session, scan and bring-up tests. It
//! sits behind the two register reads the driver polls:
//!
//! - CH12's index (0x1080): each host-index advance hands it the packet in
//!   that FWCMD slot. It consumes the entry at once, and answers the H2C as
//!   its header asks: a REC_ACK when hdr1 b14 is set, then a DONE_ACK when
//!   hdr1 b15 is set (return 0 unless [`Firmware::refuse`] names the H2C;
//!   nothing at all for one [`Firmware::silent`] names). ADD_SCANOFLD_CH
//!   stores the channel list, the address CAM the local MAC, and SCANOFLD
//!   start queues the scan: per listed channel ENTER_CH, that channel's
//!   scripted frames each followed by its PPDU_STAT, its raw
//!   [`Firmware::buffers`] batches, LEAVE_CH; then END_SCAN and the
//!   [`Firmware::after_end`] batches. [`Firmware::after`] adds a batch after
//!   one H2C's acks.
//! - RXQ's index (0x1218): each read releases at most one queued batch (an
//!   H2C's acks, one scan report, or one frame with its status) into the
//!   free entries, as many buffers as fit while one entry stays empty, and
//!   reports the hardware index past them, wrapping at 32.
//!
//! [`Air`] is its TX side, behind the index reads of the data and
//! management channels (ACH0-3, CH8) and of RPQ (0x121C).
extern crate std;
use crate::wifi::realtek::ax::pci::tests::{
    FWCMD, TX_POOL, device_read, device_write, rpp, rpq_buffer, rpq_buffer_iova, rxq_buffer,
};
use crate::wifi::realtek::ax::pci::{
    FWCMD_SLOT_BYTES, RX_ENTRIES, TX_ENTRIES, TxChannel, WD_HEAD_BYTES,
};
use crate::wifi::realtek::ax::tests::model::RegisterModel;
use crate::wifi::realtek::beacon::tests::Model;
use alloc::{boxed::Box, collections::VecDeque, rc::Rc, sync::Arc, vec, vec::Vec};
use core::cell::RefCell;
use std::sync::Mutex;

const CH12_IDX: u32 = 0x1080;
const RXQ_IDX: u32 = 0x1218;
const DESC_LEN: usize = 16;
const HDR_LEN: usize = 8;
const H2C_REC_ACK: u32 = 1 << 14;
const H2C_DONE_ACK: u32 = 1 << 15;
const H2C_LEN: u32 = 0x3FFF;
const CL_FW_OFLD: u8 = 9;
const FUNC_ADD_SCANOFLD_CH: u8 = 0x16;
const FUNC_SCANOFLD: u8 = 0x17;
const CL_ADDR_CAM: u8 = 6;
/// SCANOFLD_RSP reasons and status (`fw.h:228-245`).
const ENTER_CH: u8 = 3;
const LEAVE_CH: u8 = 4;
const END_SCAN: u8 = 5;
const STATUS_SUCCESS: u8 = 1;
/// The descriptor rate every scripted frame carries: legacy OFDM 6M, 3.2 us
/// GI-LTF code 0, 20 MHz.
const RATE_OFDM6: u32 = 4;
const FCS: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

/// One RXQ buffer as the device writes it: the RX BD info (FS | LS |
/// WRITE_SIZE), a short (16-byte) or long (32-byte) descriptor with
/// `pkt_type`, `ppdu_cnt` and the OFDM 6M rate (long: frame type `frame_type`
/// in d4), then `payload`.
pub fn rx_buffer(
    pkt_type: u8,
    long: bool,
    frame_type: u8,
    ppdu_cnt: u8,
    payload: &[u8],
) -> Vec<u8> {
    let rxd = if long { 32 } else { 16 };
    let write = 4 + rxd + payload.len();
    let mut b = vec![0u8; write];
    b[..4].copy_from_slice(&(0xC000 | write as u32).to_le_bytes());
    let d0 = payload.len() as u32 | u32::from(pkt_type) << 24 | if long { 1 << 31 } else { 0 };
    let d1 = u32::from(ppdu_cnt & 0x7) << 4 | RATE_OFDM6 << 16;
    b[4..8].copy_from_slice(&d0.to_le_bytes());
    b[8..12].copy_from_slice(&d1.to_le_bytes());
    if long {
        b[20..24].copy_from_slice(&u32::from(frame_type).to_le_bytes());
    }
    b[4 + rxd..].copy_from_slice(payload);
    b
}

/// A C2H buffer: header w0 (`cat`, `class`, `func`), w1 (length), `body`.
pub fn c2h(cat: u8, class: u8, func: u8, body: &[u8]) -> Vec<u8> {
    let w0 = u32::from(cat) | u32::from(class) << 2 | u32::from(func) << 8;
    let mut p = Vec::new();
    p.extend_from_slice(&w0.to_le_bytes());
    p.extend_from_slice(&(8 + body.len() as u32).to_le_bytes());
    p.extend_from_slice(body);
    rx_buffer(10, false, 0, 0, &p)
}

/// A REC_ACK for H2C cat 1 / `class` / `func` / `seq` (`fw.h:3965-3974`).
pub fn rec_ack(class: u8, func: u8, seq: u8) -> Vec<u8> {
    let w2 = 1 | u32::from(class) << 2 | u32::from(func) << 8 | u32::from(seq) << 16;
    c2h(1, 0, 0, &w2.to_le_bytes())
}

/// A DONE_ACK for H2C cat 1 / `class` / `func` / `seq` returning `ret`
/// (`fw.h:3952-3963`).
pub fn done_ack(class: u8, func: u8, seq: u8, ret: u8) -> Vec<u8> {
    let w2 = 1
        | u32::from(class) << 2
        | u32::from(func) << 8
        | u32::from(ret) << 16
        | u32::from(seq) << 24;
    c2h(1, 0, 1, &w2.to_le_bytes())
}

/// A SCANOFLD_RSP (cat 1, class 1, func 9) with `reason`, `status`,
/// `pri_ch`, `period` and `band` (`fw.h:4093-4118`).
pub fn scanofld(reason: u8, status: u8, band: u8, pri_ch: u8, period: u8) -> Vec<u8> {
    let w2 = u32::from(pri_ch)
        | u32::from(reason) << 16
        | u32::from(status) << 20
        | u32::from(period) << 24;
    let body: Vec<u8> = [w2, 0, 0, u32::from(band) << 24]
        .iter()
        .flat_map(|w| w.to_le_bytes())
        .collect();
    c2h(1, 1, 9, &body)
}

/// A beacon (`fc0` 0x80) or probe response (0x50) to `da` from `bssid`
/// naming `ssid`, with the DS parameter set when `ds` is `Some`, as a long
/// MGNT WIFI buffer at `ppdu_cnt`, followed by its PPDU status carrying
/// `rssi` on both paths and no IE01 channel.
pub fn frame_with_status(
    fc0: u8,
    da: [u8; 6],
    bssid: [u8; 6],
    ssid: &[u8],
    ds: Option<u8>,
    ppdu_cnt: u8,
    rssi: u8,
) -> [Vec<u8>; 2] {
    [
        mgnt_frame(fc0, da, bssid, ssid, ds, ppdu_cnt),
        ppdu_status(ppdu_cnt, rssi, 0),
    ]
}

/// The WIFI buffer of [`frame_with_status`]: a long MGNT descriptor at
/// `ppdu_cnt` and the frame with its FCS.
pub fn mgnt_frame(
    fc0: u8,
    da: [u8; 6],
    bssid: [u8; 6],
    ssid: &[u8],
    ds: Option<u8>,
    ppdu_cnt: u8,
) -> Vec<u8> {
    let mut f = vec![fc0, 0, 0, 0];
    f.extend_from_slice(&da);
    f.extend_from_slice(&bssid);
    f.extend_from_slice(&bssid);
    f.extend_from_slice(&[0x10, 0]);
    // Timestamp, beacon interval 100 TU, capability ESS.
    f.extend_from_slice(&[0; 8]);
    f.extend_from_slice(&[0x64, 0, 0x01, 0]);
    f.extend_from_slice(&[0, ssid.len() as u8]);
    f.extend_from_slice(ssid);
    f.extend_from_slice(&[1, 8, 0x8C, 0x12, 0x98, 0x24, 0xB0, 0x48, 0x60, 0x6C]);
    if let Some(ch) = ds {
        f.extend_from_slice(&[3, 1, ch]);
    }
    f.extend_from_slice(&FCS);
    rx_buffer(0, true, 0, ppdu_cnt, &f)
}

/// A PPDU status buffer at `ppdu_cnt`: the header (VALID, LEN 5 so 40
/// bytes, IE map 12), `rssi` on paths A and B, then a 32-byte IE01 whose
/// channel index (w0 [23:16]) is `chan_idx`, 0 meaning none.
pub fn ppdu_status(ppdu_cnt: u8, rssi: u8, chan_idx: u8) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&0x8000_058Cu32.to_le_bytes());
    p.extend_from_slice(&(u32::from(rssi) << 8 | u32::from(rssi)).to_le_bytes());
    let mut ie01 = vec![0u8; 32];
    ie01[0] = 1;
    ie01[2] = chan_idx;
    p.extend_from_slice(&ie01);
    rx_buffer(1, false, 0, ppdu_cnt, &p)
}

#[derive(Clone)]
struct Scripted {
    channel_index: usize,
    fc0: u8,
    bssid: [u8; 6],
    ssid: Vec<u8>,
    rssi: u8,
}

/// The firmware's script, built before [`Firmware::install`].
pub struct Firmware {
    m: Arc<Mutex<Model>>,
    refused: Vec<(u8, u8, u8)>,
    silent: Vec<(u8, u8)>,
    frames: Vec<Scripted>,
    /// END_SCAN's status.
    end_status: u8,
    /// No END_SCAN on the scan's own end.
    no_end: bool,
    end_on_stop: bool,
    burst: usize,
    /// Batches released after the acks of H2C cat 1 / class / func.
    after: Vec<(u8, u8, Vec<Vec<u8>>)>,
    /// Raw batches released after a listed channel's ENTER_CH.
    raw: Vec<(usize, Vec<Vec<u8>>)>,
    /// Batches released after END_SCAN.
    after_end: Vec<Vec<Vec<u8>>>,
}

impl Firmware {
    /// A firmware that acks every H2C with return 0, hears nothing, and ends
    /// a scan with END_SCAN status SUCCESS.
    pub(in crate::wifi::realtek) fn new(m: &Arc<Mutex<Model>>) -> Self {
        Self {
            m: m.clone(),
            refused: Vec::new(),
            silent: Vec::new(),
            frames: Vec::new(),
            end_status: STATUS_SUCCESS,
            no_end: false,
            end_on_stop: false,
            burst: 0,
            after: Vec::new(),
            raw: Vec::new(),
            after_end: Vec::new(),
        }
    }

    /// Each time H2C cat 1 / `class` / `func` is answered, `batch` follows
    /// its acks as one more batch.
    pub fn after(&mut self, class: u8, func: u8, batch: Vec<Vec<u8>>) {
        self.after.push((class, func, batch));
    }

    /// `batch`, raw RXQ buffers, arrives as one batch on listed channel
    /// `channel_index`, after its scripted frames and before its LEAVE_CH.
    /// The buffers carry their own PPDU counts.
    pub fn buffers(&mut self, channel_index: usize, batch: Vec<Vec<u8>>) {
        self.raw.push((channel_index, batch));
    }

    /// `batch` arrives as one batch after END_SCAN, whether the scan or a
    /// stop ended it.
    pub fn after_end(&mut self, batch: Vec<Vec<u8>>) {
        self.after_end.push(batch);
    }

    /// DONE_ACKs for H2C cat 1 / `class` / `func` return `ret`.
    pub fn refuse(&mut self, class: u8, func: u8, ret: u8) {
        self.refused.push((class, func, ret));
    }

    /// H2C cat 1 / `class` / `func` gets no REC_ACK and no DONE_ACK.
    pub fn silent(&mut self, class: u8, func: u8) {
        self.silent.push((class, func));
    }

    /// A beacon from `bssid` naming `ssid` on listed channel `channel_index`
    /// (0-based, in ADD_SCANOFLD_CH order), heard at raw RSSI `rssi`. A 2.4
    /// GHz beacon carries its channel in the DS parameter set.
    pub fn beacon(&mut self, channel_index: usize, bssid: [u8; 6], ssid: &[u8], rssi: u8) {
        self.script(0x80, channel_index, bssid, ssid, rssi);
    }

    /// As [`Firmware::beacon`], a probe response addressed to the MAC the
    /// address CAM H2C carried (broadcast before one arrives).
    pub fn probe_response(&mut self, channel_index: usize, bssid: [u8; 6], ssid: &[u8], rssi: u8) {
        self.script(0x50, channel_index, bssid, ssid, rssi);
    }

    /// The scan ends with no END_SCAN.
    pub fn no_end(&mut self) {
        self.no_end = true;
    }

    /// A SCANOFLD stop is answered, after its acks, with END_SCAN (status
    /// SUCCESS, or [`Firmware::end_status`]'s).
    pub fn end_on_stop(&mut self) {
        self.end_on_stop = true;
    }

    /// END_SCAN carries `status`, at the scan's end or a stop's.
    pub fn end_status(&mut self, status: u8) {
        self.end_status = status;
    }

    /// After the first listed channel's ENTER_CH, `buffers` foreign data
    /// frames (no PPDU status) arrive in one batch.
    pub fn burst(&mut self, buffers: usize) {
        self.burst = buffers;
    }

    fn script(&mut self, fc0: u8, channel_index: usize, bssid: [u8; 6], ssid: &[u8], rssi: u8) {
        self.frames.push(Scripted {
            channel_index,
            fc0,
            bssid,
            ssid: ssid.to_vec(),
            rssi,
        });
    }

    /// Answer CH12 and RXQ index reads on `io`, replacing any earlier hooks
    /// of 0x1080 and 0x1218.
    pub fn install(self, io: &mut RegisterModel) {
        let state = self.answering();
        let ch12 = state.clone();
        io.on_read(
            CH12_IDX,
            Box::new(move |r: &mut RegisterModel| ch12.ch12(r)),
        );
        io.on_read(RXQ_IDX, Box::new(move |r: &mut RegisterModel| state.rxq(r)));
    }

    /// The firmware as [`Answering`], for hooks of 0x1080 and 0x1218 that
    /// hand it over only from some point on, as the bring-up's do once
    /// OFLD_CFG's scripted acks are out.
    pub fn answering(self) -> Answering {
        Answering(Rc::new(RefCell::new(State {
            fw: self,
            ch12: 0,
            channels: Vec::new(),
            local: [0xFF; 6],
            batches: VecDeque::new(),
            ppdu_cnt: 0,
            hw: 0,
        })))
    }
}

/// A [`Firmware`] behind index reads its caller forwards.
#[derive(Clone)]
pub struct Answering(Rc<RefCell<State>>);

impl Answering {
    /// A CH12 index read (0x1080), as [`Firmware::install`] answers it.
    pub fn ch12(&self, r: &mut RegisterModel) -> u32 {
        self.0.borrow_mut().ch12(r)
    }

    /// An RXQ index read (0x1218), as [`Firmware::install`] answers it.
    pub fn rxq(&self, r: &mut RegisterModel) -> u32 {
        self.0.borrow_mut().rxq(r)
    }

    /// From now on the next H2C is read from FWCMD slot `ch12` and the next
    /// RXQ buffer written to entry `hw`: the slots and entries before them
    /// were someone else's.
    pub fn resume_at(&self, ch12: usize, hw: usize) {
        let mut state = self.0.borrow_mut();
        state.ch12 = ch12 % TX_ENTRIES;
        state.hw = hw % RX_ENTRIES;
    }
}

struct State {
    fw: Firmware,
    /// The next FWCMD slot the firmware reads.
    ch12: usize,
    /// ADD_SCANOFLD_CH's list: (band, primary channel, period).
    channels: Vec<(u8, u8, u8)>,
    local: [u8; 6],
    batches: VecDeque<Vec<Vec<u8>>>,
    ppdu_cnt: u8,
    /// RXQ's hardware index.
    hw: usize,
}

impl State {
    fn ch12(&mut self, r: &mut RegisterModel) -> u32 {
        let host = (r.get32(CH12_IDX) & 0xFFF) as usize % TX_ENTRIES;
        while self.ch12 != host {
            let at = u64::from(FWCMD) + (self.ch12 * FWCMD_SLOT_BYTES) as u64;
            let packet = {
                let m = self.fw.m.lock().unwrap();
                let head = device_read(&m, at, DESC_LEN + HDR_LEN);
                let hdr1 = u32::from_le_bytes(head[20..24].try_into().unwrap());
                device_read(&m, at, DESC_LEN + (hdr1 & H2C_LEN) as usize)
            };
            self.h2c(&packet[DESC_LEN..]);
            self.ch12 = (self.ch12 + 1) % TX_ENTRIES;
        }
        (host | host << 16) as u32
    }

    fn rxq(&mut self, r: &mut RegisterModel) -> u32 {
        let host = (r.get32(RXQ_IDX) & 0xFFF) as usize % RX_ENTRIES;
        let mut free = (host + RX_ENTRIES - self.hw - 1) % RX_ENTRIES;
        if let Some(mut batch) = self.batches.pop_front() {
            let m = self.fw.m.lock().unwrap();
            let mut written = 0;
            while written < batch.len() && free > 0 {
                device_write(&m, rxq_buffer(self.hw), &batch[written]);
                self.hw = (self.hw + 1) % RX_ENTRIES;
                written += 1;
                free -= 1;
            }
            batch.drain(..written);
            if !batch.is_empty() {
                self.batches.push_front(batch);
            }
        }
        (host | self.hw << 16) as u32
    }

    /// One H2C: its 8-byte header and payload.
    fn h2c(&mut self, h2c: &[u8]) {
        let hdr0 = u32::from_le_bytes(h2c[..4].try_into().unwrap());
        let hdr1 = u32::from_le_bytes(h2c[4..8].try_into().unwrap());
        let (cat, class, func, seq) = (
            (hdr0 & 0x3) as u8,
            ((hdr0 >> 2) & 0x3F) as u8,
            (hdr0 >> 8) as u8,
            (hdr0 >> 24) as u8,
        );
        let payload = &h2c[HDR_LEN..];
        if cat != 1 || self.fw.silent.contains(&(class, func)) {
            return;
        }
        let mut acks = Vec::new();
        if hdr1 & H2C_REC_ACK != 0 {
            acks.push(rec_ack(class, func, seq));
        }
        if hdr1 & H2C_DONE_ACK != 0 {
            let ret = self
                .fw
                .refused
                .iter()
                .find(|r| (r.0, r.1) == (class, func))
                .map_or(0, |r| r.2);
            acks.push(done_ack(class, func, seq, ret));
        }
        if !acks.is_empty() {
            self.batches.push_back(acks);
        }
        for (_, _, batch) in self.fw.after.iter().filter(|a| (a.0, a.1) == (class, func)) {
            self.batches.push_back(batch.clone());
        }
        match (class, func) {
            (CL_ADDR_CAM, 0) => self.local.copy_from_slice(&payload[16..22]),
            (CL_FW_OFLD, FUNC_ADD_SCANOFLD_CH) => {
                let n = usize::from(payload[0]);
                self.channels = (0..n)
                    .map(|i| {
                        let e = &payload[4 + 28 * i..];
                        let w1 = u32::from_le_bytes(e[4..8].try_into().unwrap());
                        (((w1 >> 14) & 0x3) as u8, e[3], e[0])
                    })
                    .collect();
            }
            (CL_FW_OFLD, FUNC_SCANOFLD) => {
                let w0 = u32::from_le_bytes(payload[..4].try_into().unwrap());
                if (w0 >> 20) & 0x3 != 0 {
                    self.scan();
                } else if self.fw.end_on_stop {
                    self.end_scan();
                }
            }
            _ => {}
        }
    }

    fn next_ppdu(&mut self) -> u8 {
        self.ppdu_cnt = (self.ppdu_cnt + 1) % 8;
        self.ppdu_cnt
    }

    fn scan(&mut self) {
        for (i, (band, ch, period)) in self.channels.clone().into_iter().enumerate() {
            self.batches
                .push_back(vec![scanofld(ENTER_CH, 0, band, ch, period)]);
            if i == 0 && self.fw.burst > 0 {
                let mut data = vec![0x08, 0, 0, 0];
                data.extend_from_slice(&self.local);
                data.extend_from_slice(&[0x02, 0x99, 0, 0, 0, 1]);
                data.extend_from_slice(&[0x02, 0x99, 0, 0, 0, 1]);
                data.extend_from_slice(&[0, 0]);
                data.extend_from_slice(&FCS);
                let burst = (0..self.fw.burst)
                    .map(|_| {
                        let cnt = self.next_ppdu();
                        rx_buffer(0, false, 0, cnt, &data)
                    })
                    .collect();
                self.batches.push_back(burst);
            }
            for f in self.fw.frames.clone() {
                if f.channel_index != i {
                    continue;
                }
                let da = if f.fc0 == 0x50 { self.local } else { [0xFF; 6] };
                let ds = (band == 0).then_some(ch);
                let cnt = self.next_ppdu();
                let pair = frame_with_status(f.fc0, da, f.bssid, &f.ssid, ds, cnt, f.rssi);
                self.batches.push_back(pair.to_vec());
            }
            for (_, batch) in self.fw.raw.iter().filter(|r| r.0 == i) {
                self.batches.push_back(batch.clone());
            }
            self.batches
                .push_back(vec![scanofld(LEAVE_CH, 0, band, ch, period)]);
        }
        if !self.fw.no_end {
            self.end_scan();
        }
    }

    /// END_SCAN with the scripted status, then the after-end batches.
    fn end_scan(&mut self) {
        self.batches
            .push_back(vec![scanofld(END_SCAN, self.fw.end_status, 0, 0, 0)]);
        self.batches.extend(self.fw.after_end.iter().cloned());
    }
}

/// The data and management channels [`Air`] serves: their channel, TX ring
/// and IDX register (`pci.h:556-559`).
const TX_RINGS: [(TxChannel, usize, u32); 5] = [
    (TxChannel::Ach0, 0, 0x1058),
    (TxChannel::Ach1, 1, 0x105C),
    (TxChannel::Ach2, 2, 0x1060),
    (TxChannel::Ach3, 3, 0x1064),
    (TxChannel::Ch8, 8, 0x1078),
];
const RPQ_IDX: u32 = 0x121C;
/// `RTW89_TX_RETRY_LIMIT` (`core.h:4677`), and the RPP's TX_STATUS field.
const TX_RETRY_LIMIT: u32 = 1;
const RPP_TX_STATUS: u32 = 0x7 << 13;
/// The TXBD length of every frame: WD head, wp_info and one addr_info.
const WD_BYTES: usize = 70;

/// One frame the chip sent: the channel whose ring held its TXBD, its WD
/// head (body and info), the WD page wp_info names, and its bytes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sent {
    pub ch: TxChannel,
    pub head: [u8; WD_HEAD_BYTES],
    pub page: u8,
    pub bytes: Vec<u8>,
}

/// The TX side of a scripted RTL8852C: the DMA engine of the data and
/// management channels and the MAC's release reports. A read of any of
/// their IDX registers, or of RPQ's, first takes every TXBD kicked on every
/// one of them, as the device sends on its own: it reads the TXBD (length
/// 70), the WD page it names and the payload its addr_info names, records
/// the frame ([`Air::sent`]), and queues its RPP (the page's seq, the head's
/// qsel, status DONE unless [`Air::drop_tx`]). A TX IDX read then reports
/// that ring's hardware index past what it took; an RPQ IDX read writes
/// every queued RPP into one RPQ buffer, unless [`Air::silent_rpq`] holds
/// them, and reports RPQ's hardware index past it.
#[derive(Clone)]
pub struct Air(Rc<RefCell<AirState>>);

struct AirState {
    m: Arc<Mutex<Model>>,
    /// Each served ring's hardware index: the next TXBD it reads.
    hw: [usize; 5],
    sent: Vec<Sent>,
    /// The next frames whose report says RETRY_LIMIT.
    drops: usize,
    silent: bool,
    /// RPP dwords not yet written to RPQ.
    reports: Vec<u32>,
    /// RPQ's hardware index.
    rpq_hw: usize,
}

impl Air {
    /// Serve the TX and RPQ index reads of `io`, whose DMA `m` models,
    /// replacing any earlier hooks of them.
    pub(in crate::wifi::realtek) fn install(io: &mut RegisterModel, m: &Arc<Mutex<Model>>) -> Self {
        let air = Self(Rc::new(RefCell::new(AirState {
            m: m.clone(),
            hw: [0; 5],
            sent: Vec::new(),
            drops: 0,
            silent: false,
            reports: Vec::new(),
            rpq_hw: 0,
        })));
        for (n, &(_, _, idx)) in TX_RINGS.iter().enumerate() {
            let state = air.0.clone();
            io.on_read(
                idx,
                Box::new(move |r: &mut RegisterModel| {
                    let mut state = state.borrow_mut();
                    state.take(r);
                    (r.get32(idx) & 0xFFF) | (state.hw[n] as u32) << 16
                }),
            );
        }
        let state = air.0.clone();
        io.on_read(
            RPQ_IDX,
            Box::new(move |r: &mut RegisterModel| state.borrow_mut().rpq(r)),
        );
        air
    }

    /// Every frame sent so far, in the order the device took them.
    pub fn sent(&self) -> Vec<Sent> {
        self.0.borrow().sent.clone()
    }

    /// RPQ speaks again: every report it held goes out acknowledged, as
    /// does every later one.
    pub fn ack_all(&mut self) {
        let mut state = self.0.borrow_mut();
        state.silent = false;
        state.drops = 0;
        for report in &mut state.reports {
            *report &= !RPP_TX_STATUS;
        }
    }

    /// The next `n` frames sent are reported unacknowledged (RETRY_LIMIT).
    pub fn drop_tx(&mut self, n: usize) {
        self.0.borrow_mut().drops += n;
    }

    /// From now on RPQ writes nothing: the reports wait for
    /// [`Air::ack_all`]. The TXBDs are still taken.
    pub fn silent_rpq(&mut self) {
        self.0.borrow_mut().silent = true;
    }

    /// Queue `rpp`, a raw RPP dword, after the reports already queued.
    pub fn report(&mut self, rpp: u32) {
        self.0.borrow_mut().reports.push(rpp);
    }
}

impl AirState {
    /// Take every TXBD kicked on a served ring, oldest first per ring.
    fn take(&mut self, r: &RegisterModel) {
        for (n, &(ch, ring, idx)) in TX_RINGS.iter().enumerate() {
            let host = (r.get32(idx) & 0xFFF) as usize % TX_ENTRIES;
            while self.hw[n] != host {
                let entry = self.hw[n];
                let frame = {
                    let m = self.m.lock().unwrap();
                    let bd_at = u64::from(TX_POOL) + ((ring * TX_ENTRIES + entry) * 8) as u64;
                    let bd = device_read(&m, bd_at, 8);
                    assert_eq!(u16::from_le_bytes([bd[0], bd[1]]), WD_BYTES as u16);
                    let wd_at = u64::from(u32::from_le_bytes(bd[4..8].try_into().unwrap()));
                    let wd = device_read(&m, wd_at, WD_BYTES);
                    let seq0 = u16::from_le_bytes([wd[56], wd[57]]);
                    assert_ne!(seq0 & 0x8000, 0, "wp_info without VALID");
                    let length_opt = u16::from_le_bytes([wd[64], wd[65]]);
                    assert_ne!(length_opt & 0x8000, 0, "addr_info without LS");
                    let low = u16::from_le_bytes([wd[66], wd[67]]);
                    let high = u16::from_le_bytes([wd[68], wd[69]]);
                    let at = u64::from(u32::from(high) << 16 | u32::from(low));
                    Sent {
                        ch,
                        head: wd[..WD_HEAD_BYTES].try_into().unwrap(),
                        page: (seq0 & 0x7FFF) as u8,
                        bytes: device_read(&m, at, usize::from(length_opt & 0x7FF)),
                    }
                };
                let body2 = u32::from_le_bytes(frame.head[8..12].try_into().unwrap());
                let status = if self.drops > 0 {
                    self.drops -= 1;
                    TX_RETRY_LIMIT
                } else {
                    0
                };
                self.reports
                    .push(rpp(u32::from(frame.page), status, (body2 >> 17) & 0x3F));
                self.sent.push(frame);
                self.hw[n] = (entry + 1) % TX_ENTRIES;
            }
        }
    }

    /// An RPQ index read.
    fn rpq(&mut self, r: &mut RegisterModel) -> u32 {
        self.take(r);
        let host = (r.get32(RPQ_IDX) & 0xFFF) as usize % RX_ENTRIES;
        let free = (host + RX_ENTRIES - self.rpq_hw - 1) % RX_ENTRIES;
        if !self.silent && !self.reports.is_empty() && free > 0 {
            let buffer = rpq_buffer(0, &core::mem::take(&mut self.reports));
            device_write(
                &self.m.lock().unwrap(),
                rpq_buffer_iova(self.rpq_hw),
                &buffer,
            );
            self.rpq_hw = (self.rpq_hw + 1) % RX_ENTRIES;
        }
        (host | self.rpq_hw << 16) as u32
    }
}

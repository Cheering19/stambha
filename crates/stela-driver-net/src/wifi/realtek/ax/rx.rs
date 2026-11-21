// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2020  Realtek Corporation
//! RX buffer parser: the RX BD info word (`pci.h:1536`), the AX RX
//! descriptor fields (`txrx.h:345-352`), the offset math of
//! `rtw89_core_query_rxdesc` (`core.c:4107`, `:4140`) and the copy length of
//! `rtw89_skb_put_rx_data` (`pci.c:278-303`). Linux copies `WRITE_SIZE` minus
//! the offset; capping the payload at `RPKT_LEN` as well is a deliberate
//! narrowing of Linux, so a buffer whose descriptor claims fewer bytes than
//! the BD wrote yields only those. The descriptor fields are those of
//! `rtw89_core_query_rxdesc` (`core.c:4107-4160`, `txrx.h:345-424`) for the
//! RTL8852C (`BW` is the `_v1` mask, d1 [31:29]); of a long descriptor's
//! d5 and d7 only the MAC ID [23:16] and the security type [20:17], which
//! `rtw89_core_skb_pn_valid` reads; the fields Stela does not use are not
//! read. Only FS && LS single-segment buffers are framed. It reads no
//! register and grants no DMA.
use super::io::AxError;

const RXBD_FS: u32 = 1 << 15;
const RXBD_LS: u32 = 1 << 14;
const RXBD_WRITE_SIZE: u32 = 0x3FFF;
const RXD_RPKT_LEN: u32 = 0x3FFF;
const RXD_RPKT_TYPE_SHIFT: u32 = 24;
const RXD_DRV_INFO_SHIFT: u32 = 28;
const RXD_LONG: u32 = 1 << 31;
const RXD_MAC_INFO_VLD: u32 = 1 << 23;
/// `RTW89_CORE_RX_TYPE_WIFI`, `_PPDU_STAT` and `_C2H` (`core.h:225-241`).
pub const RX_TYPE_WIFI: u8 = 0;
pub const RX_TYPE_PPDU_STAT: u8 = 1;
pub const RX_TYPE_C2H: u8 = 10;
/// `RTW89_RX_TYPE_MGNT` and `_DATA` (`core.h:4475-4479`).
pub const FRAME_MGNT: u8 = 0;
pub const FRAME_DATA: u8 = 2;
const BD_INFO_BYTES: usize = 4;
const SHORT_RXD: usize = 16;
const LONG_RXD: usize = 32;
const TOO_SHORT: AxError = AxError::Rx("RX buffer shorter than its descriptor");

/// The fields `rtw89_core_query_rxdesc` reads that Stela uses (spec S11.1).
/// `frame_type` is `Some` only for a long descriptor; `mac_id` and
/// `sec_type` are 0 for a short one, as Linux's zeroed `desc_info` leaves
/// them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RxDesc {
    pub pkt_type: u8,
    pub pkt_len: u16,
    pub long: bool,
    pub mac_info_valid: bool,
    pub ppdu_type: u8,
    pub ppdu_cnt: u8,
    pub data_rate: u16,
    pub gi_ltf: u8,
    pub bw: u8,
    pub free_run: u32,
    pub a1_match: bool,
    pub sw_dec: bool,
    pub hw_dec: bool,
    pub ampdu: bool,
    pub crc32_err: bool,
    pub icv_err: bool,
    pub frame_type: Option<u8>,
    /// `AX_RXD_MAC_ID_MASK`.
    pub mac_id: u8,
    /// `AX_RXD_SEC_TYPE_MASK`, an `enum rtw89_sec_key_type`.
    pub sec_type: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RxFrame<'a> {
    pub pkt_type: u8,
    pub desc: RxDesc,
    pub payload: &'a [u8],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RxBuffer<'a> {
    Frame(RxFrame<'a>),
    /// Not both FS and LS: a multi-buffer frame, which is not assembled.
    Segment,
}

fn le32(b: &[u8], at: usize) -> Option<u32> {
    Some(u32::from_le_bytes(b.get(at..at + 4)?.try_into().ok()?))
}

/// Parse one RXQ buffer: offset = 4 + 16|32 + drv_info * 8, payload =
/// `WRITE_SIZE - offset` bytes, capped at `RPKT_LEN`.
pub fn parse(buffer: &[u8]) -> Result<RxBuffer<'_>, AxError> {
    let info = le32(buffer, 0).ok_or(TOO_SHORT)?;
    if info & RXBD_FS == 0 || info & RXBD_LS == 0 {
        return Ok(RxBuffer::Segment);
    }
    let write_size = (info & RXBD_WRITE_SIZE) as usize;
    let d0 = le32(buffer, BD_INFO_BYTES).ok_or(TOO_SHORT)?;
    let long = d0 & RXD_LONG != 0;
    let rxd_len = if long { LONG_RXD } else { SHORT_RXD };
    let word = |n: usize| le32(buffer, BD_INFO_BYTES + n * 4).ok_or(TOO_SHORT);
    let (d1, d2, d3) = (word(1)?, word(2)?, word(3)?);
    let (frame_type, mac_id, sec_type) = if long {
        let (d4, d5, d7) = (word(4)?, word(5)?, word(7)?);
        (
            Some((d4 & 0x3) as u8),
            ((d5 >> 16) & 0xFF) as u8,
            ((d7 >> 17) & 0xF) as u8,
        )
    } else {
        (None, 0, 0)
    };
    let pkt_type = ((d0 >> RXD_RPKT_TYPE_SHIFT) & 0xF) as u8;
    let desc = RxDesc {
        pkt_type,
        pkt_len: (d0 & RXD_RPKT_LEN) as u16,
        long,
        mac_info_valid: d0 & RXD_MAC_INFO_VLD != 0,
        ppdu_type: (d1 & 0xF) as u8,
        ppdu_cnt: ((d1 >> 4) & 0x7) as u8,
        data_rate: ((d1 >> 16) & 0x1FF) as u16,
        gi_ltf: ((d1 >> 25) & 0x7) as u8,
        bw: (d1 >> 29) as u8,
        free_run: d2,
        a1_match: d3 & 1 != 0,
        sw_dec: d3 & (1 << 1) != 0,
        hw_dec: d3 & (1 << 2) != 0,
        ampdu: d3 & (1 << 3) != 0,
        crc32_err: d3 & (1 << 9) != 0,
        icv_err: d3 & (1 << 10) != 0,
        frame_type,
        mac_id,
        sec_type,
    };
    let drv_info = ((d0 >> RXD_DRV_INFO_SHIFT) & 0x7) as usize * 8;
    let offset = BD_INFO_BYTES + rxd_len + drv_info;
    let copy_len = write_size.checked_sub(offset).ok_or(TOO_SHORT)?;
    let copy_len = copy_len.min((d0 & RXD_RPKT_LEN) as usize);
    let payload = buffer.get(offset..offset + copy_len).ok_or(TOO_SHORT)?;
    Ok(RxBuffer::Frame(RxFrame {
        pkt_type,
        desc,
        payload,
    }))
}

#[cfg(test)]
mod tests;

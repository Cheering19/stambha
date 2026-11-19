// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2020  Realtek Corporation
//! RX buffer parser: the RX BD info word (`pci.h:1536`), the AX RX
//! descriptor fields (`txrx.h:345-352`), the offset math of
//! `rtw89_core_query_rxdesc` (`core.c:4107`, `:4140`) and the copy length of
//! `rtw89_skb_put_rx_data` (`pci.c:278-303`). Linux copies `WRITE_SIZE` minus
//! the offset; capping the payload at `RPKT_LEN` as well is a deliberate
//! narrowing of Linux, so a buffer whose descriptor claims fewer bytes than
//! the BD wrote yields only those. Only FS && LS single-segment buffers are
//! framed. It reads no register and grants no DMA.
use super::io::AxError;

const RXBD_FS: u32 = 1 << 15;
const RXBD_LS: u32 = 1 << 14;
const RXBD_WRITE_SIZE: u32 = 0x3FFF;
const RXD_RPKT_LEN: u32 = 0x3FFF;
const RXD_RPKT_TYPE_SHIFT: u32 = 24;
const RXD_DRV_INFO_SHIFT: u32 = 28;
const RXD_LONG: u32 = 1 << 31;
const BD_INFO_BYTES: usize = 4;
const SHORT_RXD: usize = 16;
const LONG_RXD: usize = 32;
const TOO_SHORT: AxError = AxError::Rx("RX buffer shorter than its descriptor");

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RxFrame<'a> {
    pub pkt_type: u8,
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
    let rxd_len = if d0 & RXD_LONG != 0 {
        LONG_RXD
    } else {
        SHORT_RXD
    };
    let drv_info = ((d0 >> RXD_DRV_INFO_SHIFT) & 0x7) as usize * 8;
    let offset = BD_INFO_BYTES + rxd_len + drv_info;
    let copy_len = write_size.checked_sub(offset).ok_or(TOO_SHORT)?;
    let copy_len = copy_len.min((d0 & RXD_RPKT_LEN) as usize);
    let payload = buffer.get(offset..offset + copy_len).ok_or(TOO_SHORT)?;
    Ok(RxBuffer::Frame(RxFrame {
        pkt_type: ((d0 >> RXD_RPKT_TYPE_SHIFT) & 0xF) as u8,
        payload,
    }))
}

#[cfg(test)]
mod tests;

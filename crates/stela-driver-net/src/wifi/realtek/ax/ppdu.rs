// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
//! An RTL8852C's PPDU status and the rule that pairs it with the frames of
//! its PPDU. [`parse`] reads a PPDU_STAT payload as
//! `rtw89_core_rx_process_ppdu_sts` (`core.c:4059-4085`) does: the MAC info
//! of `rtw89_core_rx_process_mac_ppdu` (`core.c:1939-2014`, `txrx.h:430-458`),
//! the PHY status header of `rtw89_core_rx_process_phy_ppdu` and
//! `rtw89_core_update_phy_ppdu` (`core.c:2268-2307`, `txrx.h:460-487`), and
//! the IE walk of `rtw89_core_rx_parse_phy_sts` (`core.c:2310-2357`) with the
//! AX IE lengths of `rtw89_phy_gen_ax` (`phy.c:9045-9047`) and the IE01
//! channel index of `rtw89_core_parse_phy_status_ie01` (`core.c:2124`).
//! [`frame_key`], [`ppdu_key`] and [`matches()`] are the rate, GI and
//! bandwidth of `rtw89_core_update_rx_status` (`core.c:4408-4445`) and
//! `rtw89_core_rx_ppdu_match` (`core.c:2437-2471`), with
//! `rtw89_rxdesc_to_nl_he_gi` (`core.c:2374-2393`) and
//! `rtw89_hw_to_rate_info_bw` (`core.h:8456-8466`). [`rssi_dbm`] is
//! `RTW89_RSSI_RAW_TO_DBM` (`core.h:59-61`), which `rtw8852c_query_ppdu`
//! (`rtw8852c.c:2980-2996`) applies.
//!
//! Every read of the device-written payload is bounds-checked: a payload
//! too short for the word Linux would read stops the parse where Linux's
//! own length checks would (spec S11.3). The per-user MAC info words, the
//! IE00/IE09/IE10 contents, CFO, EVM and antenna diversity are not read;
//! monitor mode is never on. This module reads no register and grants no
//! DMA, no queueing of frames and no channel tag: pairing and delivery
//! belong to the scan receiver.
use super::rx::RxDesc;

/// `RTW89_MAX_PPDU_CNT` (`core.h:843`).
pub const MAX_PPDU_CNT: u8 = 8;
/// The GI and bandwidth values below are interface constants only: the
/// numeric values of `enum nl80211_he_gi` (`include/uapi/linux/nl80211.h`)
/// and `enum rate_info_bw` (`include/net/cfg80211.h`) that Linux stores in
/// `rx_status`; no code is taken from those headers.
///
/// `enum nl80211_he_gi`, as Linux stores it in `rx_status->he_gi`, and the
/// `U8_MAX` of an invalid PPDU-side GI.
pub const GI_0_8: u8 = 0;
pub const GI_1_6: u8 = 1;
pub const GI_3_2: u8 = 2;
pub const GI_NONE: u8 = 0xFF;
/// `enum rate_info_bw`, as Linux stores it in `rx_status->bw`.
pub const BW_20: u8 = 0;
pub const BW_40: u8 = 3;
pub const BW_80: u8 = 4;
pub const BW_160: u8 = 5;

/// `RTW89_PPDU_MAC_INFO_SIZE`, `_USR_SIZE` and `_RX_CNT_SIZE`
/// (`core.h:4541-4543`), and the RTL8852C's `ppdu_max_usr`
/// (`rtw8852c.c:3225`).
const MAC_INFO_SIZE: usize = 8;
const MAC_INFO_USR_SIZE: usize = 4;
const MAC_RX_CNT_SIZE: usize = 96;
const PPDU_MAX_USR: usize = 8;
/// `RTW89_RXINFO_W0_USR_NUM`, `_RX_CNT_VLD` and `RTW89_RXINFO_W1_PLCP_LEN`
/// (`txrx.h:448-458`).
const RXINFO_W0_USR_NUM: u32 = 0xF;
const RXINFO_W0_RX_CNT_VLD: u32 = 1 << 29;
const RXINFO_W1_PLCP_LEN_SHIFT: u32 = 16;
/// `PHY_STS_HDR_LEN` (`core.h:841`) and the `rtw89_phy_sts_hdr` fields
/// (`txrx.h:465-473`).
const PHY_STS_HDR_LEN: usize = 8;
const HDR_W0_IE_MAP: u32 = 0x1F;
const HDR_W0_HDR_2_EN: u32 = 1 << 5;
const HDR_W0_VALID: u32 = 1 << 7;
const HDR_W0_LEN_SHIFT: u32 = 8;
const HDR_W0_RSSI_AVG_SHIFT: u32 = 24;
/// `RTW89_PHY_STS_IEHDR_TYPE` and `_LEN` (`txrx.h:486-487`).
const IEHDR_TYPE: u32 = 0x1F;
const IEHDR_LEN_SHIFT: u32 = 5;
const IEHDR_LEN: u32 = 0x7F;
/// `RTW89_PHY_STS_IE01_W0_CH_IDX` (`txrx.h:629`).
const IE01_W0_CH_IDX_SHIFT: u32 = 16;
/// `RTW89_CCK_PKT` (`phy.h:286`), `RTW89_PHYSTS_IE01_CMN_OFDM` and
/// `_IE09_FTR_0` (`phy.h:236-245`).
const CCK_PKT: u32 = 11;
const IE01_CMN_OFDM: u8 = 1;
const IE09_FTR_0: u8 = 9;
/// `VAR_LEN` and `VAR_LEN_UNIT` (`phy.h:157-158`).
const VAR_LEN: u8 = 0xFF;
const VAR_LEN_UNIT: usize = 8;
/// `rtw89_phy_gen_ax.physt_ie_len` (`phy.c:9045-9047`).
const IE_LEN: [u8; 32] = [
    16, 32, 24, 24, 8, 8, 8, 8, VAR_LEN, 8, VAR_LEN, 176, VAR_LEN, VAR_LEN, VAR_LEN, VAR_LEN,
    VAR_LEN, VAR_LEN, 16, 24, VAR_LEN, VAR_LEN, VAR_LEN, 0, 24, 24, 24, 24, 32, 32, 32, 32,
];
/// `DATA_RATE_MODE_CTRL_MASK`, the AX index masks and modes
/// (`txrx.h:10-24`, spec S11.2).
const RATE_MODE_SHIFT: u16 = 7;
const RATE_MODE_NON_HT: u16 = 0;
const RATE_MODE_HT: u16 = 1;
const RATE_NOT_HT_IDX: u16 = 0xF;
const RATE_HT_IDX: u16 = 0x1F;
const RATE_VHT_HE_IDX: u16 = 0xF;

/// The parts of `struct rtw89_rx_phy_ppdu` the scan uses. `header` says the
/// PHY status header passed its VALID and length checks, so `rssi` and
/// `rssi_avg` were read; `valid` is Linux's `phy_ppdu.valid`, a completed IE
/// walk. `chan_idx` is the last IE01's index, kept like Linux's even when a
/// later IE fails the walk; only a `valid` status may use it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PhyPpdu {
    pub header: bool,
    pub valid: bool,
    /// Raw RSSI of path A and path B.
    pub rssi: [u8; 2],
    pub rssi_avg: u8,
    pub chan_idx: u8,
}

/// The fields `rtw89_core_rx_ppdu_match` compares: `rate_idx` is the raw
/// hardware index (legacy `[3:0]`, HT `[4:0]`, VHT/HE `[3:0]`), `gi` a [`GI_0_8`]
/// .. [`GI_NONE`] value and `bw` a [`BW_20`] .. [`BW_160`] value. `legacy` is
/// `RX_ENC_LEGACY`, which the match does not compare.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RateKey {
    pub legacy: bool,
    pub rate_idx: u8,
    pub gi: u8,
    pub bw: u8,
}

fn le32(b: &[u8], at: usize) -> Option<u32> {
    let end = at.checked_add(4)?;
    Some(u32::from_le_bytes(b.get(at..end)?.try_into().ok()?))
}

/// The PHY status offset after the MAC info, or `None` where
/// `rtw89_core_rx_process_mac_ppdu` returns `-EINVAL`. Linux reads w0 and
/// w1 without a length check, but any payload shorter than 8 bytes puts the
/// status past the end, so refusing it changes nothing.
fn mac_info_end(payload: &[u8]) -> Option<usize> {
    let w0 = le32(payload, 0)?;
    let w1 = le32(payload, 4)?;
    let usr_num = (w0 & RXINFO_W0_USR_NUM) as usize;
    if usr_num > PPDU_MAX_USR {
        return None;
    }
    let mut at = MAC_INFO_SIZE + usr_num * MAC_INFO_USR_SIZE;
    // 8-byte alignment.
    if usr_num & 1 != 0 {
        at += MAC_INFO_USR_SIZE;
    }
    if w0 & RXINFO_W0_RX_CNT_VLD != 0 {
        at += MAC_RX_CNT_SIZE;
    }
    // Linux's `plcp_size` is a `u8` assigned `PLCP_LEN << 3` (`core.c:1950`,
    // `:1968`), so the offset is truncated to a byte: PLCP_LEN 0x21 adds 8.
    let plcp_size = ((w1 >> RXINFO_W1_PLCP_LEN_SHIFT) as u8) << 3;
    at += usize::from(plcp_size);
    (at <= payload.len()).then_some(at)
}

/// `rtw89_core_rx_parse_phy_sts`'s walk over `sts` (the whole PHY status,
/// header included). Returns whether it completed; IE01 sets `chan_idx` on
/// the way. Every IE length is a multiple of 8 and so is `sts.len()`, so the
/// header read at `pos < end` always has 4 bytes; a short read fails all
/// the same.
fn walk(sts: &[u8], hdr_2_en: bool, to_self: bool, chan_idx: &mut u8) -> bool {
    let end = sts.len();
    let mut pos = PHY_STS_HDR_LEN;
    if hdr_2_en {
        pos += PHY_STS_HDR_LEN;
    }
    // `phy_ppdu->ie09`: an IE09 that `rtw89_core_process_phy_status_ie`
    // accepted, i.e. of an A1-matched status (`core.c:2234-2236`, `:2334`).
    let mut ie09 = false;
    while pos < end {
        let Some(w0) = le32(sts, pos) else {
            return false;
        };
        if ie09 && end - pos == 8 && w0 == 0 {
            break;
        }
        let ie = (w0 & IEHDR_TYPE) as u8;
        let ie_len = match IE_LEN[usize::from(ie)] {
            VAR_LEN => ((w0 >> IEHDR_LEN_SHIFT) & IEHDR_LEN) as usize * VAR_LEN_UNIT,
            n => usize::from(n),
        };
        pos += ie_len;
        if pos > end || ie_len == 0 {
            return false;
        }
        if ie == IE01_CMN_OFDM {
            *chan_idx = (w0 >> IE01_W0_CH_IDX_SHIFT) as u8;
        } else if ie == IE09_FTR_0 && to_self {
            ie09 = true;
        }
    }
    true
}

/// Parse a PPDU_STAT `payload` (after the descriptor and drv_info) as
/// `rtw89_core_rx_process_ppdu_sts` does (spec S11.3). The pairing that
/// follows in Linux whatever the outcome (`core.c:4083-4085`) is the
/// caller's.
pub fn parse(desc: &RxDesc, payload: &[u8]) -> PhyPpdu {
    let mut ppdu = PhyPpdu::default();
    let start = if desc.mac_info_valid {
        match mac_info_end(payload) {
            Some(at) => at,
            None => return ppdu,
        }
    } else {
        0
    };
    let Some(sts) = payload.get(start..) else {
        return ppdu;
    };
    // Linux reads the header unchecked; a status shorter than it can never
    // equal `LEN * 8` except when empty, where Linux reads past the buffer.
    let (Some(w0), Some(w1)) = (le32(sts, 0), le32(sts, 4)) else {
        return ppdu;
    };
    let len = ((w0 >> HDR_W0_LEN_SHIFT) & 0xFF) as usize * 8;
    if w0 & HDR_W0_VALID == 0 || len != sts.len() {
        return ppdu;
    }
    ppdu.header = true;
    ppdu.rssi_avg = (w0 >> HDR_W0_RSSI_AVG_SHIFT) as u8;
    ppdu.rssi = [w1 as u8, (w1 >> 8) as u8];
    // Below CCK_PKT the status is a failure report: no walk
    // (`core.c:2318-2320`).
    if w0 & HDR_W0_IE_MAP >= CCK_PKT {
        let hdr_2_en = w0 & HDR_W0_HDR_2_EN != 0;
        ppdu.valid = walk(sts, hdr_2_en, desc.a1_match, &mut ppdu.chan_idx);
    }
    ppdu
}

fn key(desc: &RxDesc, rx_status: bool) -> RateKey {
    let rate = desc.data_rate;
    let mode = (rate >> RATE_MODE_SHIFT) & 0x3;
    let rate_idx = match mode {
        RATE_MODE_NON_HT => rate & RATE_NOT_HT_IDX,
        RATE_MODE_HT => rate & RATE_HT_IDX,
        _ => rate & RATE_VHT_HE_IDX,
    } as u8;
    let gi = match desc.gi_ltf {
        // RTW89_GILTF_SGI_4XHE08, _2XHE08, _1XHE08 (`core.h:4466-4471`).
        1 | 3 | 5 => GI_0_8,
        // RTW89_GILTF_2XHE16, _1XHE16.
        2 | 4 => GI_1_6,
        // RTW89_GILTF_LGI_4XHE32.
        0 => GI_3_2,
        _ if rx_status => GI_3_2,
        _ => GI_NONE,
    };
    let bw = match desc.bw {
        // RTW89_CHANNEL_WIDTH_40, _80, _160 (`core.h:948-952`).
        1 => BW_40,
        2 => BW_80,
        3 => BW_160,
        _ => BW_20,
    };
    RateKey {
        legacy: mode == RATE_MODE_NON_HT,
        rate_idx,
        gi,
        bw,
    }
}

/// A WIFI frame's key, as `rtw89_core_update_rx_status` fills
/// `rx_status` (`rx_status = true`: an invalid GI-LTF reads as 3.2 us).
pub fn frame_key(desc: &RxDesc) -> RateKey {
    key(desc, true)
}

/// A PPDU_STAT descriptor's key, as `rtw89_core_rx_ppdu_match` computes it
/// (`rx_status = false`: an invalid GI-LTF reads as [`GI_NONE`]).
pub fn ppdu_key(desc: &RxDesc) -> RateKey {
    key(desc, false)
}

/// `rtw89_core_rx_ppdu_match`: `curr` (`curr_rx_ppdu_cnt`) equals the
/// PPDU_STAT's `PPDU_CNT` and the frame's raw rate index, GI and bandwidth
/// equal the PPDU's. No MAC ID, A1, encoding or validity condition.
pub fn matches(curr: u8, ppdu: &RxDesc, frame: &RateKey) -> bool {
    let p = ppdu_key(ppdu);
    curr == ppdu.ppdu_cnt && frame.rate_idx == p.rate_idx && frame.gi == p.gi && frame.bw == p.bw
}

/// `RTW89_RSSI_RAW_TO_DBM`: `(s8)(raw >> 1) - 110`, from -110 to 17 dBm.
pub const fn rssi_dbm(raw: u8) -> i8 {
    (raw >> 1) as i8 - 110
}

#[cfg(test)]
mod tests;

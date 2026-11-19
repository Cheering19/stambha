// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2020-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! The channel state of an RTL8852C's 20 MHz 2.4 or 5 GHz channel, as
//! `rtw89_chan_create` (`chan.c:131-155`) fills `struct rtw89_chan` with
//! `rtw89_get_subband_type` (`chan.c:19-60`); the BB gain band and OFDM
//! gain-offset index of `rtw89_subband_to_bb_gain_band` and
//! `rtw89_subband_to_gain_offset_band_of_ofdm` (`phy.h:801-858`); the
//! channel index of `rtw89_encode_chan_idx` / `rtw89_decode_chan_idx`
//! (`phy.c:8569-8638`); the SCO value of `rtw8852c_ctrl_ch`
//! (`rtw8852c.c:1173-1174`); and cfg80211's `ieee80211_channel_to_frequency`,
//! which `rtw89_chan_create` and `rtw8852c_fill_freq_with_ppdu` call.
//!
//! At 20 MHz the primary channel is the centre channel and `pri_ch_idx` is
//! `RTW89_SC_DONT_CARE`, so neither is stored. `tx_comp_band`, `pri_sb_idx`
//! and `rfsi_band` are not ported. This module writes no register: it grants
//! no channel switch, no 6 GHz channel and no 40, 80 or 160 MHz channel.
use super::io::AxError;

/// `enum rtw89_band` (`core.h:337-341`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HwBand {
    G2 = 0,
    G5 = 1,
    G6 = 2,
}

/// The 2.4 and 5 GHz part of `enum rtw89_subband` (`core.h:108-126`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Subband {
    G2 = 0,
    G5Band1 = 1,
    G5Band3 = 3,
    G5Band4 = 4,
}

/// The fields of `struct rtw89_chan` a 20 MHz channel needs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Chan {
    /// `channel` and `primary_channel`.
    pub channel: u8,
    /// `band_type`.
    pub band: HwBand,
    /// `freq`, in MHz.
    pub freq: u16,
    /// `subband_type`.
    pub subband: Subband,
}

impl Chan {
    /// `rtw89_chan_create(chan, ch, ch, band, RTW89_CHANNEL_WIDTH_20)`,
    /// accepting only the channels `_ctrl_ch` programs
    /// (`rtw8852c_rfk.c:3941-3944`) in 2G 1-14 and 5G.
    pub fn new20(band: HwBand, channel: u8) -> Result<Self, AxError> {
        let subband = match (band, channel) {
            (HwBand::G2, 1..=14) => Subband::G2,
            (HwBand::G5, 36..=64) => Subband::G5Band1,
            (HwBand::G5, 100..=144) => Subband::G5Band3,
            (HwBand::G5, 149..=177) => Subband::G5Band4,
            _ => return Err(AxError::Phy("not a 20 MHz 2.4 or 5 GHz channel")),
        };
        Ok(Self {
            channel,
            band,
            freq: frequency(band, channel),
            subband,
        })
    }

    /// `rtw89_subband_to_bb_gain_band` (`phy.h:833-858`):
    /// `RTW89_BB_GAIN_BAND_{2G, 5G_L, 5G_M, 5G_H}` (`core.h:7197-7208`).
    pub const fn bb_gain_band(&self) -> usize {
        match self.subband {
            Subband::G2 => 0,
            Subband::G5Band1 => 1,
            Subband::G5Band3 => 2,
            Subband::G5Band4 => 3,
        }
    }

    /// `rtw89_subband_to_gain_offset_band_of_ofdm` (`phy.h:801-830`):
    /// `RTW89_GAIN_OFFSET_{2G_OFDM, 5G_LOW, 5G_MID, 5G_HIGH}`
    /// (`core.h:138-154`), an index into `RxGain::offset`.
    pub const fn ofdm_gain_offset(&self) -> usize {
        match self.subband {
            Subband::G2 => 1,
            Subband::G5Band1 => 2,
            Subband::G5Band3 => 3,
            Subband::G5Band4 => 4,
        }
    }

    /// `rtw89_encode_chan_idx(rtwdev, chan->primary_channel, band)`
    /// (`phy.c:8582-8619`, called at `rtw8852c.c:1238`).
    pub fn chan_idx(&self) -> u8 {
        encode_chan_idx(self.band, self.channel)
    }

    /// `DIV_ROUND_CLOSEST(1 << 18, central_freq)` (`rtw8852c.c:1174`), the
    /// value of BB `R_FC0_BW` `B_FC0_BW_INV`.
    pub fn sco(&self) -> u8 {
        let freq = u32::from(self.freq);
        ((262_144 + freq / 2) / freq) as u8
    }
}

/// `rtw89_ch_base_table` (`phy.c:8569-8572`).
const CH_BASE: [u8; 16] = [
    1, 0xff, 36, 100, 132, 149, 0xff, 1, 33, 65, 97, 129, 161, 193, 225, 0xff,
];
/// `RTW89_CH_BASE_IDX_*` (`phy.c:8573-8577`).
const CH_BASE_IDX_5G_FIRST: usize = 2;
const CH_BASE_IDX_5G_LAST: usize = 5;
const CH_BASE_IDX_6G_FIRST: usize = 7;
const CH_BASE_IDX_6G_LAST: usize = 14;

/// `rtw89_encode_chan_idx` (`phy.c:8582-8619`): the base index in bits
/// [7:4] and the offset in [3:0]. An unknown channel is 0, as Linux returns
/// after its warning.
fn encode_chan_idx(band: HwBand, ch: u8) -> u8 {
    let (first, last) = match band {
        HwBand::G2 => return ch & 0x0F,
        HwBand::G5 => (CH_BASE_IDX_5G_FIRST, CH_BASE_IDX_5G_LAST),
        HwBand::G6 => (CH_BASE_IDX_6G_FIRST, CH_BASE_IDX_6G_LAST),
    };
    match (first..=last).rev().find(|&idx| ch >= CH_BASE[idx]) {
        Some(idx) => ((idx as u8) << 4) | (((ch - CH_BASE[idx]) >> 1) & 0x0F),
        None => 0,
    }
}

/// `rtw89_decode_chan_idx` (`phy.c:8621-8638`): base index 0 is 2.4 GHz
/// with the offset as the channel, up to 5 is 5 GHz and above is 6 GHz, at
/// `base + 2 * offset`. The unused bases (0xff) wrap as Linux's `u8` does.
pub fn decode_chan_idx(idx: u8) -> (HwBand, u8) {
    let base = usize::from(idx >> 4);
    let offset = idx & 0x0F;
    if base == 0 {
        return (HwBand::G2, offset);
    }
    let band = if base <= CH_BASE_IDX_5G_LAST {
        HwBand::G5
    } else {
        HwBand::G6
    };
    (band, CH_BASE[base].wrapping_add(offset << 1))
}

/// `ieee80211_channel_to_frequency` (cfg80211, `net/wireless/util.c`) in
/// MHz: 2.4 GHz `2407 + 5c` for 1-13 and 2484 for 14; 5 GHz `5000 + 5c`,
/// except `4000 + 5c` for 182-196; 6 GHz 5935 for channel 2 and `5950 + 5c`
/// up to 233. A channel with no frequency is 0, as cfg80211 returns.
pub const fn frequency(band: HwBand, channel: u8) -> u16 {
    let c = channel as u16;
    match band {
        _ if channel == 0 => 0,
        HwBand::G2 if channel == 14 => 2484,
        HwBand::G2 if channel < 14 => 2407 + 5 * c,
        HwBand::G2 => 0,
        HwBand::G5 if channel >= 182 && channel <= 196 => 4000 + 5 * c,
        HwBand::G5 => 5000 + 5 * c,
        HwBand::G6 if channel == 2 => 5935,
        HwBand::G6 if channel <= 233 => 5950 + 5 * c,
        HwBand::G6 => 0,
    }
}

#[cfg(test)]
mod tests;

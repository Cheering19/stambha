// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2020-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! The channel state of an RTL8852C's 2.4 or 5 GHz channel of 20 to 160 MHz,
//! as `rtw89_get_channel_params` (`core.c:402-461`) takes it from a channel
//! definition and `rtw89_chan_create` (`chan.c:131-155`) fills `struct
//! rtw89_chan`: the centre's subband (`rtw89_get_subband_type`,
//! `chan.c:19-60`), the primary's index (`rtw89_get_primary_chan_idx`,
//! `chan.c:85-115`) and the TX subcarriers `rtw8852c_set_channel_mac`
//! computes with `rtw89_phy_get_txsc` (`rtw8852c.c:829-848`,
//! `phy.c:805-873`); the BB gain band and OFDM gain-offset index of
//! `rtw89_subband_to_bb_gain_band` and
//! `rtw89_subband_to_gain_offset_band_of_ofdm` (`phy.h:801-858`); the
//! channel index of `rtw89_encode_chan_idx` / `rtw89_decode_chan_idx`
//! (`phy.c:8569-8638`); the SCO value of `rtw8852c_ctrl_ch`
//! (`rtw8852c.c:1173-1174`); and cfg80211's `ieee80211_channel_to_frequency`,
//! which `rtw89_chan_create` and `rtw8852c_fill_freq_with_ppdu` call.
//!
//! A channel is built only where `_ctrl_ch` programs its centre
//! (`rtw8852c_rfk.c:3941-3944`), 2.4 GHz kept to 1-14 and at most 40 MHz,
//! and only where its primary lies inside its width at a 20 MHz step, as
//! `cfg80211_chandef_valid` admits a definition. The 6 GHz subbands are
//! described, but no 6 GHz channel is built until M4f. `tx_comp_band`,
//! `pri_sb_idx` and `rfsi_band` are not ported. This module writes no
//! register: it grants no channel switch, and no 40, 80 or 160 MHz channel
//! by itself.
use super::io::AxError;
use crate::wifi::Band;
use crate::wifi::station::operations::ChannelDefinition;
use crate::wifi::station::phy::Width;

/// `enum rtw89_band` (`core.h:337-341`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HwBand {
    G2 = 0,
    G5 = 1,
    G6 = 2,
}

/// `enum rtw89_subband` (`core.h:108-126`); `RTW89_CH_5G_BAND_2` is unused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Subband {
    G2 = 0,
    G5Band1 = 1,
    G5Band3 = 3,
    G5Band4 = 4,
    G6Idx0 = 5,
    G6Idx1 = 6,
    G6Idx2 = 7,
    G6Idx3 = 8,
    G6Idx4 = 9,
    G6Idx5 = 10,
    G6Idx6 = 11,
    G6Idx7 = 12,
}

/// `enum rtw89_bandwidth`'s ordinary widths (`core.h`), in its order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Bw {
    Mhz20 = 0,
    Mhz40 = 1,
    Mhz80 = 2,
    Mhz160 = 3,
}

impl Bw {
    /// The width in MHz.
    pub const fn mhz(self) -> u16 {
        20 << self as u16
    }
}

/// `enum rtw89_sc_offset` (`core.h:1010-1022`): the primary's place.
const SC_DONT_CARE: u8 = 0;
const SC_20_UPPER: u8 = 1;
const SC_20_LOWER: u8 = 2;

/// What [`Chan::new20`] refuses.
const NOT_20: AxError = AxError::Phy("not a 20 MHz 2.4 or 5 GHz channel");
/// What [`Chan::new`] refuses.
const NOT_A_CHANNEL: AxError = AxError::Phy("not a 2.4 or 5 GHz channel an RTL8852C tunes");

/// `struct rtw89_chan`'s fields an RTL8852C channel needs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Chan {
    /// `channel`: the centre channel.
    pub channel: u8,
    /// `primary_channel`.
    pub primary: u8,
    /// `band_type`.
    pub band: HwBand,
    /// `band_width`.
    pub width: Bw,
    /// `freq`: the centre's, in MHz.
    pub freq: u16,
    /// `subband_type`: the centre's.
    pub subband: Subband,
    /// `pri_ch_idx`, an `enum rtw89_sc_offset`.
    pub pri_ch_idx: u8,
    /// The 20, 40 and 80 MHz TX subcarriers `rtw8852c_set_channel_mac`
    /// takes (`rtw8852c.c:829-848`): [`Chan::txsc`] of each width below the
    /// channel's, 0 for the others.
    pub txsc: [u8; 3],
}

impl Chan {
    /// `rtw89_chan_create(chan, centre, primary, band, width)`: refused
    /// where `_ctrl_ch` would program nothing (a centre in 2G 1-14 or in 5G
    /// 36-64, 100-144 or 149-177), for 2.4 GHz past 40 MHz, for a primary
    /// outside the width or off its 20 MHz grid (`cfg80211_chandef_valid`),
    /// and for 6 GHz, which M4f brings.
    pub fn new(band: HwBand, centre: u8, primary: u8, width: Bw) -> Result<Self, AxError> {
        let programs = match band {
            HwBand::G2 => matches!(centre, 1..=14) && width <= Bw::Mhz40,
            HwBand::G5 => matches!(centre, 36..=64 | 100..=144 | 149..=177),
            HwBand::G6 => false,
        };
        let freq = frequency(band, centre);
        let primary_freq = frequency(band, primary);
        if !programs || freq == 0 || primary_freq == 0 {
            return Err(NOT_A_CHANNEL);
        }
        let apart = freq.abs_diff(primary_freq);
        let inside = match width {
            Bw::Mhz20 => centre == primary,
            _ => apart >= 10 && (apart - 10).is_multiple_of(20) && apart <= width.mhz() / 2 - 10,
        };
        if !inside {
            return Err(NOT_A_CHANNEL);
        }
        let mut chan = Self {
            channel: centre,
            primary,
            band,
            width,
            freq,
            subband: subband_type(band, centre),
            pri_ch_idx: primary_chan_idx(width, freq, primary_freq),
            txsc: [0; 3],
        };
        for (slot, dbw) in chan.txsc.iter_mut().zip([Bw::Mhz20, Bw::Mhz40, Bw::Mhz80]) {
            if dbw < width {
                *slot = txsc_of(width, primary, centre, dbw);
            }
        }
        Ok(chan)
    }

    /// [`Chan::new`] at 20 MHz, centre and primary one.
    pub fn new20(band: HwBand, channel: u8) -> Result<Self, AxError> {
        Self::new(band, channel, channel, Bw::Mhz20).map_err(|_| NOT_20)
    }

    /// `rtw89_get_channel_params` (`core.c:402-461`): the centre channel
    /// from the control channel and `center_freq1`, 2 away at 40 MHz and
    /// `2 + 4 * offset` at 80 and 160, then [`Chan::new`]. Refused before
    /// any I/O: 80+80 and 320 MHz, a control channel whose frequency is not
    /// its number's, and a centre whose frequency is not `center_freq1`.
    pub fn from_definition(definition: &ChannelDefinition) -> Result<Self, AxError> {
        let width = match definition.width {
            Width::Mhz20 => Bw::Mhz20,
            Width::Mhz40 => Bw::Mhz40,
            Width::Mhz80 => Bw::Mhz80,
            Width::Mhz160 => Bw::Mhz160,
            Width::Mhz80P80 | Width::Mhz320 => {
                return Err(AxError::Phy("80+80 or 320 MHz on an RTL8852C"));
            }
        };
        let band = match definition.control.band {
            Band::Ghz2 => HwBand::G2,
            Band::Ghz5 => HwBand::G5,
            Band::Ghz6 => HwBand::G6,
        };
        let primary = u8::try_from(definition.control.number).map_err(|_| NOT_A_CHANNEL)?;
        let primary_freq = u32::from(definition.control.frequency_mhz);
        if u32::from(frequency(band, primary)) != primary_freq {
            return Err(NOT_A_CHANNEL);
        }
        let center_freq = definition.center1_mhz;
        let above = primary_freq > center_freq;
        let step = match width {
            Bw::Mhz20 => 0,
            Bw::Mhz40 => 2,
            Bw::Mhz80 | Bw::Mhz160 => {
                let offset = primary_freq.abs_diff(center_freq).saturating_sub(10) / 20;
                u8::try_from(2 + offset * 4).map_err(|_| NOT_A_CHANNEL)?
            }
        };
        let centre = if above {
            primary.checked_sub(step)
        } else {
            primary.checked_add(step)
        }
        .ok_or(NOT_A_CHANNEL)?;
        let chan = Self::new(band, centre, primary, width)?;
        if width != Bw::Mhz20 && u32::from(chan.freq) != center_freq {
            return Err(NOT_A_CHANNEL);
        }
        Ok(chan)
    }

    /// `rtw89_phy_get_txsc(rtwdev, chan, dbw)` (`phy.c:805-873`) literally,
    /// `0xff` for a 160 MHz channel's 40 MHz subcarrier it cannot place.
    pub fn txsc(&self, dbw: Bw) -> u8 {
        txsc_of(self.width, self.primary, self.channel, dbw)
    }

    /// `rtw89_subband_to_bb_gain_band` (`phy.h:833-858`):
    /// `RTW89_BB_GAIN_BAND_{2G, 5G_L, 5G_M, 5G_H, 6G_L, 6G_M, 6G_H, 6G_UH}`
    /// (`core.h:7197-7208`).
    pub const fn bb_gain_band(&self) -> usize {
        match self.subband {
            Subband::G2 => 0,
            Subband::G5Band1 => 1,
            Subband::G5Band3 => 2,
            Subband::G5Band4 => 3,
            Subband::G6Idx0 | Subband::G6Idx1 => 4,
            Subband::G6Idx2 | Subband::G6Idx3 => 5,
            Subband::G6Idx4 | Subband::G6Idx5 => 6,
            Subband::G6Idx6 | Subband::G6Idx7 => 7,
        }
    }

    /// `rtw89_subband_to_gain_offset_band_of_ofdm` (`phy.h:801-830`):
    /// `RTW89_GAIN_OFFSET_{2G_OFDM, 5G_LOW, 5G_MID, 5G_HIGH, 6G_L0 .. 6G_UH1}`
    /// (`core.h:138-154`), an index into `RxGain::offset`.
    pub const fn ofdm_gain_offset(&self) -> usize {
        match self.subband {
            Subband::G2 => 1,
            Subband::G5Band1 => 2,
            Subband::G5Band3 => 3,
            Subband::G5Band4 => 4,
            Subband::G6Idx0 => 5,
            Subband::G6Idx1 => 6,
            Subband::G6Idx2 => 7,
            Subband::G6Idx3 => 8,
            Subband::G6Idx4 => 9,
            Subband::G6Idx5 => 10,
            Subband::G6Idx6 => 11,
            Subband::G6Idx7 => 12,
        }
    }

    /// `rtw89_encode_chan_idx(rtwdev, chan->primary_channel, band)`
    /// (`phy.c:8582-8619`, called at `rtw8852c.c:1238`).
    pub fn chan_idx(&self) -> u8 {
        encode_chan_idx(self.band, self.primary)
    }

    /// `DIV_ROUND_CLOSEST(1 << 18, central_freq)` (`rtw8852c.c:1174`), the
    /// value of BB `R_FC0_BW` `B_FC0_BW_INV`.
    pub fn sco(&self) -> u8 {
        let freq = u32::from(self.freq);
        ((262_144 + freq / 2) / freq) as u8
    }
}

/// `rtw89_get_subband_type` (`chan.c:19-60`) literally, its `default`s
/// included: a 6 GHz centre between two ranges is `IDX0`'s.
pub fn subband_type(band: HwBand, centre: u8) -> Subband {
    match band {
        HwBand::G2 => Subband::G2,
        HwBand::G5 => match centre {
            100..=144 => Subband::G5Band3,
            149..=177 => Subband::G5Band4,
            _ => Subband::G5Band1,
        },
        HwBand::G6 => match centre {
            33..=61 => Subband::G6Idx1,
            65..=93 => Subband::G6Idx2,
            97..=125 => Subband::G6Idx3,
            129..=157 => Subband::G6Idx4,
            161..=189 => Subband::G6Idx5,
            193..=221 => Subband::G6Idx6,
            225..=253 => Subband::G6Idx7,
            _ => Subband::G6Idx0,
        },
    }
}

/// `rtw89_get_primary_chan_idx` (`chan.c:85-115`): `DONT_CARE` at 20 MHz;
/// `UPPER` or `LOWER` at 40; at 80 and 160 `UPPER + 2o` or `LOWER + 2o`
/// with `o = (|f_p - f_c| - 10) / 20`.
fn primary_chan_idx(width: Bw, center_freq: u16, primary_freq: u16) -> u8 {
    let above = primary_freq > center_freq;
    match width {
        Bw::Mhz20 => SC_DONT_CARE,
        Bw::Mhz40 if above => SC_20_UPPER,
        Bw::Mhz40 => SC_20_LOWER,
        Bw::Mhz80 | Bw::Mhz160 => {
            let offset = ((primary_freq.abs_diff(center_freq) - 10) / 20) as u8;
            let base = if above { SC_20_UPPER } else { SC_20_LOWER };
            base + offset * 2
        }
    }
}

/// `rtw89_phy_get_txsc` (`phy.c:805-873`) of a channel of width `cbw`
/// around `central` with primary `pri`, for the subcarrier of `dbw`.
fn txsc_of(cbw: Bw, pri: u8, central: u8, dbw: Bw) -> u8 {
    if cbw == dbw || cbw == Bw::Mhz20 {
        return 0;
    }
    let tmp = if pri > central {
        (pri - central) >> 1
    } else {
        ((central - pri) >> 1) + 1
    };
    match cbw {
        Bw::Mhz20 => 0,
        Bw::Mhz40 if pri > central => 1,
        Bw::Mhz40 => 2,
        Bw::Mhz80 if dbw == Bw::Mhz20 => tmp,
        Bw::Mhz80 if pri > central => 9,
        Bw::Mhz80 => 10,
        Bw::Mhz160 => match dbw {
            Bw::Mhz20 => tmp,
            Bw::Mhz40 => match tmp {
                1 | 3 => 9,
                5 | 7 => 11,
                2 | 4 => 10,
                6 | 8 => 12,
                _ => 0xff,
            },
            _ if pri > central => 13,
            _ => 14,
        },
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

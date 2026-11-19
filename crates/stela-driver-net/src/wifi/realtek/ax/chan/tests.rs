// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2020-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
use super::*;
use alloc::vec::Vec;

/// The 38 channels of the scan list (spec S1.4): 2.4 GHz 1-13, then 5 GHz
/// 36-64, 100-144 and 149-165 in steps of 4.
fn scan_channels() -> Vec<(HwBand, u8)> {
    let mut list: Vec<(HwBand, u8)> = (1..=13).map(|c| (HwBand::G2, c)).collect();
    for range in [36..=64, 100..=144, 149..=165] {
        list.extend(range.step_by(4).map(|c| (HwBand::G5, c)));
    }
    list
}

/// band, channel, freq, sco, chan_idx, subband, BB gain band, OFDM index.
type Row = (HwBand, u8, u16, u8, u8, Subband, usize, usize);

#[test]
fn chan_state_matches_linux_for_the_spec_channels() {
    // Spec S4's table: band, channel, freq, sco (0x4974[6:0]), chan_idx,
    // subband, BB gain band, OFDM gain-offset index.
    let rows: [Row; 9] = [
        (HwBand::G2, 1, 0x96C, 0x6D, 0x01, Subband::G2, 0, 1),
        (HwBand::G2, 6, 0x985, 0x6C, 0x06, Subband::G2, 0, 1),
        (HwBand::G2, 13, 0x9A8, 0x6A, 0x0D, Subband::G2, 0, 1),
        (HwBand::G2, 14, 0x9B4, 0x6A, 0x0E, Subband::G2, 0, 1),
        (HwBand::G5, 36, 0x143C, 0x33, 0x20, Subband::G5Band1, 1, 2),
        (HwBand::G5, 100, 0x157C, 0x30, 0x30, Subband::G5Band3, 2, 3),
        (HwBand::G5, 149, 0x1671, 0x2E, 0x50, Subband::G5Band4, 3, 4),
        (HwBand::G5, 153, 0x1685, 0x2D, 0x52, Subband::G5Band4, 3, 4),
        (HwBand::G5, 165, 0x16C1, 0x2D, 0x58, Subband::G5Band4, 3, 4),
    ];
    for (band, ch, freq, sco, idx, subband, gain_band, ofdm) in rows {
        let c = Chan::new20(band, ch).unwrap();
        let what = (band, ch);
        assert_eq!(
            c,
            Chan {
                channel: ch,
                band,
                freq,
                subband
            },
            "{what:?}"
        );
        assert_eq!(c.sco(), sco, "{what:?}");
        assert_eq!(c.chan_idx(), idx, "{what:?}");
        assert_eq!(c.bb_gain_band(), gain_band, "{what:?}");
        assert_eq!(c.ofdm_gain_offset(), ofdm, "{what:?}");
        assert_eq!(frequency(band, ch), freq, "{what:?}");
    }
    // The enum discriminants are Linux's (core.h:108-126, :337-341).
    assert_eq!(
        [HwBand::G2 as u8, HwBand::G5 as u8, HwBand::G6 as u8],
        [0, 1, 2]
    );
    assert_eq!(
        [
            Subband::G2 as u8,
            Subband::G5Band1 as u8,
            Subband::G5Band3 as u8,
            Subband::G5Band4 as u8
        ],
        [0, 1, 3, 4]
    );
}

#[test]
fn subband_edges_follow_rtw89_get_subband_type() {
    // chan.c:30-38: 36-64 BAND_1, 100-144 BAND_3, 149-177 BAND_4.
    for (ch, sb) in [
        (64, Subband::G5Band1),
        (100, Subband::G5Band3),
        (144, Subband::G5Band3),
        (149, Subband::G5Band4),
        (177, Subband::G5Band4),
    ] {
        assert_eq!(Chan::new20(HwBand::G5, ch).unwrap().subband, sb, "{ch}");
    }
}

#[test]
fn channels_ctrl_ch_skips_are_refused() {
    // rtw8852c_rfk.c:3941-3944, with 2G kept to 1-14 and 6G out of scope.
    for (band, ch) in [
        (HwBand::G2, 0),
        (HwBand::G2, 15),
        (HwBand::G5, 32),
        (HwBand::G5, 68),
        (HwBand::G5, 146),
        (HwBand::G5, 178),
        (HwBand::G6, 1),
        (HwBand::G6, 37),
        (HwBand::G6, 233),
    ] {
        assert!(
            matches!(
                Chan::new20(band, ch),
                Err(AxError::Phy("not a 20 MHz 2.4 or 5 GHz channel"))
            ),
            "{band:?} {ch}"
        );
    }
    for (band, ch) in [
        (HwBand::G2, 1),
        (HwBand::G2, 14),
        (HwBand::G5, 36),
        (HwBand::G5, 64),
        (HwBand::G5, 100),
        (HwBand::G5, 144),
        (HwBand::G5, 149),
        (HwBand::G5, 177),
    ] {
        assert!(Chan::new20(band, ch).is_ok(), "{band:?} {ch}");
    }
}

#[test]
fn chan_idx_round_trips() {
    let list = scan_channels();
    assert_eq!(list.len(), 38);
    for (band, ch) in list {
        let c = Chan::new20(band, ch).unwrap();
        assert_eq!(decode_chan_idx(c.chan_idx()), (band, ch), "{band:?} {ch}");
    }
    assert_eq!(decode_chan_idx(0x70), (HwBand::G6, 1));
    assert_eq!(decode_chan_idx(0x52), (HwBand::G5, 153));
}

#[test]
fn frequency_follows_ieee80211_channel_to_frequency() {
    assert_eq!(frequency(HwBand::G2, 1), 2412);
    assert_eq!(frequency(HwBand::G2, 13), 2472);
    assert_eq!(frequency(HwBand::G2, 14), 2484);
    assert_eq!(frequency(HwBand::G5, 36), 5180);
    assert_eq!(frequency(HwBand::G5, 177), 5885);
    assert_eq!(frequency(HwBand::G6, 1), 5955);
    assert_eq!(frequency(HwBand::G6, 233), 7115);
    // cfg80211's exceptions (net/wireless/util.c, outside the donor tree):
    // 6 GHz channel 2 is 5935, 5 GHz 182-196 is the 4.9 GHz Japan band, and
    // a number with no frequency is 0.
    assert_eq!(frequency(HwBand::G6, 2), 5935);
    assert_eq!(frequency(HwBand::G5, 184), 4920);
    assert_eq!(frequency(HwBand::G5, 196), 4980);
    assert_eq!(frequency(HwBand::G5, 197), 5985);
    for (band, ch) in [
        (HwBand::G2, 0),
        (HwBand::G2, 15),
        (HwBand::G5, 0),
        (HwBand::G6, 0),
        (HwBand::G6, 234),
    ] {
        assert_eq!(frequency(band, ch), 0, "{band:?} {ch}");
    }
    // A chan_idx with offset 15 in the 2 GHz slot decodes to channel 15,
    // which has no frequency, as Linux's status->freq would be 0.
    assert_eq!(decode_chan_idx(0x0F), (HwBand::G2, 15));
    assert_eq!(frequency(HwBand::G2, 15), 0);
}

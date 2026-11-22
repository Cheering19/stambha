// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2020-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
use super::*;
use crate::wifi::Channel;
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
                primary: ch,
                band,
                width: Bw::Mhz20,
                freq,
                subband,
                pri_ch_idx: 0,
                txsc: [0; 3],
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

// Channels of any width (M4 Task 10, spec S4).

/// The definition of a channel of `width` on `control` around `center1`
/// MHz, as mac80211's chandef carries it.
fn definition(band: Band, control: u16, width: Width, center1_mhz: u32) -> ChannelDefinition {
    let hw = match band {
        Band::Ghz2 => HwBand::G2,
        Band::Ghz5 => HwBand::G5,
        Band::Ghz6 => HwBand::G6,
    };
    ChannelDefinition {
        control: Channel {
            band,
            number: control,
            frequency_mhz: frequency(hw, control as u8),
        },
        width,
        center1_mhz,
        center2_mhz: None,
        punctured: 0,
    }
}

/// centre, freq, subband, pri_ch_idx, txsc 20/40/80, chan_idx, sco.
type Wide = (u8, u16, Subband, u8, [u8; 3], u8, u8);

fn wide(c: &Chan) -> Wide {
    (
        c.channel,
        c.freq,
        c.subband,
        c.pri_ch_idx,
        c.txsc,
        c.chan_idx(),
        c.sco(),
    )
}

#[test]
fn chan_matches_rtw89_get_channel_params() {
    // Spec S4.4's rows, then rtw89_get_primary_chan_idx's edges of an 80
    // MHz channel (chan.c:85-115).
    let rows: [(ChannelDefinition, Wide); 7] = [
        (
            definition(Band::Ghz5, 157, Width::Mhz80, 5775),
            (155, 5775, Subband::G5Band4, 1, [1, 9, 0], 0x54, 45),
        ),
        (
            definition(Band::Ghz5, 157, Width::Mhz40, 5775),
            (155, 5775, Subband::G5Band4, 1, [1, 0, 0], 0x54, 45),
        ),
        (
            definition(Band::Ghz2, 6, Width::Mhz40, 2447),
            (8, 2447, Subband::G2, 2, [2, 0, 0], 0x06, 107),
        ),
        (
            definition(Band::Ghz5, 100, Width::Mhz160, 5570),
            (114, 5570, Subband::G5Band3, 8, [8, 12, 14], 0x30, 47),
        ),
        // HT40+ on channel 1: LOWER.
        (
            definition(Band::Ghz2, 1, Width::Mhz40, 2422),
            (3, 2422, Subband::G2, 2, [2, 0, 0], 0x01, 108),
        ),
        // VHT80 on 36: (5210 - 5180 - 10) / 20 = 1, LOWER + 2 = LOWEST.
        (
            definition(Band::Ghz5, 36, Width::Mhz80, 5210),
            (42, 5210, Subband::G5Band1, 4, [4, 10, 0], 0x20, 50),
        ),
        // VHT80 on 48: UPPER + 2 = UPMOST.
        (
            definition(Band::Ghz5, 48, Width::Mhz80, 5210),
            (42, 5210, Subband::G5Band1, 3, [3, 9, 0], 0x26, 50),
        ),
    ];
    for (def, want) in rows {
        let c = Chan::from_definition(&def).unwrap();
        assert_eq!(wide(&c), want, "{def:?}");
        assert_eq!(c.primary, def.control.number as u8);
    }
    let c = Chan::from_definition(&definition(Band::Ghz5, 157, Width::Mhz80, 5775)).unwrap();
    assert_eq!((c.width, c.width as u8, c.width.mhz()), (Bw::Mhz80, 2, 80));
    // rtw89_phy_get_txsc itself, past what the MAC asks: the channel's own
    // width is 0, and a 40 MHz channel answers its UPPER/LOWER 1 or 2 for
    // any other width, as Linux's switch does.
    assert_eq!(
        (c.txsc(Bw::Mhz20), c.txsc(Bw::Mhz40), c.txsc(Bw::Mhz80)),
        (1, 9, 0)
    );
    let ht40 = Chan::new(HwBand::G5, 155, 153, Bw::Mhz40).unwrap();
    assert_eq!((ht40.txsc(Bw::Mhz20), ht40.txsc(Bw::Mhz80)), (2, 2));
    assert_eq!(ht40.pri_ch_idx, 2);
}

#[test]
fn txsc_160_follows_linux_per_primary() {
    // phy.c:833-856 for the eight primaries of 100-128 around 114: 20 MHz
    // `tmp`, 40 MHz 9-12 by `tmp`, 80 MHz 13 above the centre, 14 below.
    for (primary, txsc, pri_ch_idx) in [
        (100, [8, 12, 14], 8),
        (104, [6, 12, 14], 6),
        (108, [4, 10, 14], 4),
        (112, [2, 10, 14], 2),
        (116, [1, 9, 13], 1),
        (120, [3, 9, 13], 3),
        (124, [5, 11, 13], 5),
        (128, [7, 11, 13], 7),
    ] {
        let c = Chan::new(HwBand::G5, 114, primary, Bw::Mhz160).unwrap();
        assert_eq!((c.txsc, c.pri_ch_idx), (txsc, pri_ch_idx), "{primary}");
    }
}

#[test]
fn new20_is_new_at_20() {
    let mut list = scan_channels();
    list.extend([
        (HwBand::G2, 14),
        (HwBand::G5, 169),
        (HwBand::G5, 173),
        (HwBand::G5, 177),
    ]);
    for (band, ch) in list {
        let c = Chan::new20(band, ch).unwrap();
        assert_eq!(Chan::new(band, ch, ch, Bw::Mhz20), Ok(c), "{band:?} {ch}");
        assert_eq!(
            (c.primary, c.width, c.pri_ch_idx, c.txsc),
            (ch, Bw::Mhz20, 0, [0; 3]),
            "{band:?} {ch}"
        );
        let band_of = match band {
            HwBand::G2 => Band::Ghz2,
            HwBand::G5 => Band::Ghz5,
            HwBand::G6 => Band::Ghz6,
        };
        let def = definition(band_of, u16::from(ch), Width::Mhz20, u32::from(c.freq));
        assert_eq!(Chan::from_definition(&def), Ok(c), "{band:?} {ch}");
    }
}

#[test]
fn wide_definitions_outside_the_tables_are_refused() {
    // 2.4 GHz past 40 MHz; 80+80 and 320 MHz; a centre in a gap _ctrl_ch
    // skips (64 with 68 above it: centre 66); a centre other than
    // center_freq1; 6 GHz until M4f.
    for def in [
        definition(Band::Ghz2, 6, Width::Mhz80, 2457),
        definition(Band::Ghz5, 64, Width::Mhz40, 5330),
        definition(Band::Ghz5, 157, Width::Mhz80, 5765),
        definition(Band::Ghz6, 1, Width::Mhz20, 5955),
        definition(Band::Ghz6, 1, Width::Mhz80, 5985),
    ] {
        assert_eq!(
            Chan::from_definition(&def),
            Err(AxError::Phy("not a 2.4 or 5 GHz channel an RTL8852C tunes")),
            "{def:?}"
        );
    }
    for width in [Width::Mhz80P80, Width::Mhz320] {
        assert_eq!(
            Chan::from_definition(&definition(Band::Ghz5, 36, width, 5210)),
            Err(AxError::Phy("80+80 or 320 MHz on an RTL8852C"))
        );
    }
    // A primary outside the width, off its 20 MHz grid, or apart at 20.
    for (centre, primary, width) in [
        (42, 52, Bw::Mhz80),
        (42, 34, Bw::Mhz80),
        (38, 38, Bw::Mhz40),
        (155, 157, Bw::Mhz20),
        (114, 132, Bw::Mhz160),
    ] {
        assert!(
            Chan::new(HwBand::G5, centre, primary, width).is_err(),
            "{centre} {primary} {width:?}"
        );
    }
    // _ctrl_ch takes any centre in 36-64, so a centre of 50 at 80 MHz with
    // its primary 10 MHz off is built, as Linux would program it.
    assert!(Chan::new(HwBand::G5, 50, 52, Bw::Mhz80).is_ok());
}

#[test]
fn six_ghz_subbands_follow_rtw89_get_subband_type() {
    // chan.c:41-58, the gaps falling to IDX0 as its `default`; then the BB
    // gain band (phy.h:842-857) and OFDM gain offset (phy.h:813-828) of each.
    let rows = [
        (1, Subband::G6Idx0, 4, 5),
        (31, Subband::G6Idx0, 4, 5),
        (33, Subband::G6Idx1, 4, 6),
        (65, Subband::G6Idx2, 5, 7),
        (125, Subband::G6Idx3, 5, 8),
        (129, Subband::G6Idx4, 6, 9),
        (189, Subband::G6Idx5, 6, 10),
        (193, Subband::G6Idx6, 7, 11),
        (253, Subband::G6Idx7, 7, 12),
    ];
    let mut c = Chan::new20(HwBand::G5, 36).unwrap();
    for (centre, subband, gain_band, ofdm) in rows {
        assert_eq!(subband_type(HwBand::G6, centre), subband, "{centre}");
        c.subband = subband;
        assert_eq!((c.bb_gain_band(), c.ofdm_gain_offset()), (gain_band, ofdm));
    }
    assert_eq!([Subband::G6Idx0 as u8, Subband::G6Idx7 as u8], [5, 12]);
    // 5 GHz's default is BAND_1, 2.4 GHz has one.
    assert_eq!(subband_type(HwBand::G5, 32), Subband::G5Band1);
    assert_eq!(subband_type(HwBand::G2, 99), Subband::G2);
}

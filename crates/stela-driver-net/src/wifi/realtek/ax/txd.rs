// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
// Copyright(c) 2020  Realtek Corporation
// Copyright(c) 2019-2022  Realtek Corporation
//! The TX WD body (v1) and info of an RTL8852C's management and data
//! frames, and the CCMP strip that hands the hardware a host-reserved
//! packet number.
//!
//! A frame's `struct rtw89_tx_desc_info` (`core.h:1253-1299`) is filled as
//! `rtw89_core_tx_update_desc_info` (`core.c:1265-1327`) fills it for a
//! station's frame: the type of `rtw89_core_get_tx_type` (`core.c:624-634`),
//! then `rtw89_core_tx_update_mgmt_info` (`core.c:902-939`) with the rate of
//! `rtw89_core_get_mgmt_rate` (`core.c:748-786`), or
//! `rtw89_core_tx_update_data_info` (`core.c:1123-1155`) with the retry
//! floor of `rtw89_core_get_data_rate` (`core.c:1080-1120`), the key of
//! `rtw89_core_tx_update_sec_key` (`core.c:680-746`, `hw_sec_hdr` set,
//! `rtw8852c.c:3211`), the EAPOL BK of `rtw89_core_tx_update_ampdu_info`
//! (`core.c:637-677`) and the header length of
//! `rtw89_core_tx_update_llc_hdr` (`core.c:808-817`). The qsel and its
//! channel are `rtw89_core_get_qsel`, `rtw89_core_get_qsel_mgmt` and
//! `rtw89_core_get_tid_indicate` (`txrx.h:833-890`) with
//! `rtw89_core_get_ch_dma` (`core.c:819-854`, PCIe's entry of the 8852C's
//! `get_ch_dma`, `rtw8852c.c:3105`). It is laid out as
//! `rtw89_core_fill_txdesc_v1` (`core.c:1661-1687`, `rtw8852c.c:3103`) lays
//! it out with its `rtw89_build_txwd_*` dwords (`core.c:1494-1635`) and the
//! `RTW89_TXWD_BODY*`/`INFO*` masks (`txrx.h:68-151`): 32 bytes of body,
//! then 24 of info (`txwd_body_size`, `txwd_info_size`,
//! `rtw8852c.c:3263-3264`), little-endian. The frame-control and header
//! rules are IEEE 802.11's as Linux's `include/linux/ieee80211.h` and
//! cfg80211's `ieee80211_hdrlen` read them.
//!
//! The CCMP strip turns a station frame that carries its host-reserved
//! 8-byte CCMP header (`Protection::HostIv`) into the frame Linux hands an
//! `hw_sec_hdr` chip, which carries none (`IEEE80211_KEY_FLAG_GENERATE_IV`
//! is set only without `hw_sec_hdr`, `cam.c:523-524`): the header must be
//! exactly the one the reservation's PN and key ID give, or nothing changes.
//! The PN then travels in BODY4/5 with `wp_offset` 1, and the hardware
//! inserts the header and appends the MIC (`TX_PARTIAL_MODE` kept and
//! `APPEND_MIC` set by `sec_eng_init`, [`super::dmac`]).
//!
//! This module touches no register, ring or page, and does not grant a
//! queued frame, a free WD page, a key in the security CAM (the caller names
//! a CAM entry it installed), a fresh PN (the caller's `KeySlot` reserves
//! it), a frame on the air or an acknowledgement. It builds no H2C
//! descriptor (`fill_txdesc_fwcmd_v1`), no HE HT Control
//! (`rtw89_core_tx_update_he_qos_htc` adds none and so sets no BK: no HE
//! association, `core.c:1057-1078`), no TX A-MPDU but the WD fields of one
//! the caller names ([`Aggregate`]: the agreement itself is the station's),
//! no injected rate (`core.c:1226-1262`), no high queue (`hiq`, so `port` stays 0), no TX
//! report (`report`, `sn`, `tx_cnt_lmt*`: `rtw89_tx_rpt_init` returns
//! without `tx_rpt_enabled`, which PCIe never sets, `mac.h:1835-1849`), no
//! ER SU (`er_cap`), no MLO, and no hardware-encrypted management frame (a
//! robust management frame is sealed on the host and goes without a key, so
//! `hw_mgmt_tx_encrypt`, `core.c:930-933`, never applies). Its only cipher
//! is CCMP-128.
use super::{
    chan::HwBand,
    io::AxError,
    pci::{MAX_TX_FRAME, TxChannel, WD_HEAD_BYTES},
};
use alloc::vec::Vec;

/// A data frame's hardware key: `rtw89_core_tx_update_sec_key`'s inputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxSec {
    /// `key->keyidx` (0-3): BODY1 SEC_KEYID.
    pub keyid: u8,
    /// The security CAM entry the key is installed in (`sec_cam_idx`, below
    /// `scam_num` 128, `rtw8852c.c:3220`): INFO2 SEC_CAM_IDX.
    pub cam: u8,
    /// The host-reserved PN (`atomic64_inc_return(&key->tx_pn)`), 1 to
    /// 2^48 - 1: BODY4/5. It must be the PN [`strip_ccmp`] checked.
    pub pn: u64,
}

/// A data frame's A-MPDU, `rtw89_core_tx_update_ampdu_info`'s inputs
/// (`core.c:636-675`) for a frame mac80211 marks `IEEE80211_TX_CTL_AMPDU`, one
/// of a TID whose TX Block Ack agreement is operational: the agreement's
/// window (`ampdu_params[tid].agg_num`, 0 until TX_OPERATIONAL set it), and
/// the peer's HT A-MPDU length exponent (`ht_cap.ampdu_factor`) and minimum
/// MPDU start spacing (`ht_cap.ampdu_density`, 0-7).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Aggregate {
    pub agg_num: u16,
    pub factor: u8,
    pub density: u8,
}

/// The peer's RA capabilities a data frame's info carries
/// (`rtwsta_link->ra.stbc_cap`, `ra.ldpc_cap`, `core.c:1145-1146`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RateCaps {
    pub stbc: bool,
    pub ldpc: bool,
}

/// `RTW89_TX_QSEL_*` (`txrx.h:798-831`): the band-0 queues used here.
const QSEL_BE_0: u8 = 0x00;
const QSEL_BK_0: u8 = 0x01;
const QSEL_VI_0: u8 = 0x02;
const QSEL_VO_0: u8 = 0x03;
const QSEL_B0_MGMT: u8 = 0x12;

/// `RTW89_HW_RATE_CCK1`, `RTW89_HW_RATE_OFDM6` (`core.h:345-349`).
const HW_RATE_CCK1: u8 = 0x0;
const HW_RATE_OFDM6: u8 = 0x4;
/// `RTW89_SEC_KEY_TYPE_CCMP128` (`core.h:321`).
const SEC_KEY_TYPE_CCMP128: u8 = 6;
/// `chip->scam_num` (`rtw8852c.c:3220`).
const SCAM_NUM: u8 = 128;
/// Every frame here, as the vif and its AP peer, is macid 0.
const MAC_ID: u8 = 0;
/// `IEEE80211_QOS_CTL_TAG1D_MASK`: `skb->priority`'s TID bits.
const QOS_CTL_TAG1D_MASK: u8 = 0x7;
/// `IEEE80211_QOS_CTL_TID_MASK`: the QoS control's TID field.
const QOS_CTL_TID_MASK: u8 = 0xF;
/// `IEEE80211_CCMP_HDR_LEN`.
const CCMP_HDR_LEN: usize = 8;
/// CCMP's 48-bit PN.
const PN_LIMIT: u64 = 1 << 48;

// The frame-control and sequence-control fields (`IEEE80211_FCTL_*`,
// `IEEE80211_FTYPE_*`, `IEEE80211_STYPE_*`, `IEEE80211_SCTL_SEQ`).
const FCTL_FTYPE: u16 = 0x000C;
const FCTL_STYPE: u16 = 0x00F0;
const FCTL_TODS: u16 = 0x0100;
const FCTL_FROMDS: u16 = 0x0200;
const FCTL_PROTECTED: u16 = 0x4000;
const FCTL_ORDER: u16 = 0x8000;
const FTYPE_MGMT: u16 = 0x0000;
const FTYPE_DATA: u16 = 0x0008;
const STYPE_NULLFUNC: u16 = 0x0040;
const STYPE_QOS_DATA: u16 = 0x0080;
const STYPE_QOS_NULLFUNC: u16 = 0x00C0;
const SCTL_SEQ: u16 = 0xFFF0;
/// `hdr->seq_ctrl` and `hdr->addr1`.
const SEQ_CTRL: usize = 22;
const ADDR1: usize = 4;
/// `ieee80211_hdrlen`'s three-address header, and its two additions.
const HDR_LEN_3ADDR: usize = 24;
const HDR_LEN_4ADDR: usize = 30;
const QOS_CTL_LEN: usize = 2;
const HT_CTL_LEN: usize = 4;

// `RTW89_TXWD_BODY*` and `RTW89_TXWD_INFO*` (`txrx.h:68-151`), the fields
// `rtw89_core_fill_txdesc_v1` fills.
const BODY0_WP_OFFSET_V1: u32 = 0x1F << 24;
const BODY0_WD_INFO_EN: u32 = 1 << 22;
const BODY0_CHANNEL_DMA: u32 = 0xF << 16;
const BODY0_HDR_LLC_LEN: u32 = 0x1F << 11;
const BODY0_WD_PAGE: u32 = 1 << 7;
const BODY1_ADDR_INFO_NUM: u32 = 0x3F << 26;
const BODY1_SEC_KEYID: u32 = 0x3 << 4;
const BODY1_SEC_TYPE: u32 = 0xF;
const BODY2_MACID: u32 = 0x7F << 24;
const BODY2_TID_INDICATE: u32 = 1 << 23;
const BODY2_QSEL: u32 = 0x3F << 17;
const BODY2_TXPKT_SIZE: u32 = 0x3FFF;
const BODY3_BK: u32 = 1 << 13;
const BODY3_AGG_EN: u32 = 1 << 12;
const BODY3_SW_SEQ: u32 = 0xFFF;
const BODY4_SEC_IV_L1: u32 = 0xFF << 24;
const BODY4_SEC_IV_L0: u32 = 0xFF << 16;
const BODY5_SEC_IV_H5: u32 = 0xFF << 24;
const BODY5_SEC_IV_H4: u32 = 0xFF << 16;
const BODY5_SEC_IV_H3: u32 = 0xFF << 8;
const BODY5_SEC_IV_H2: u32 = 0xFF;
const BODY7_USE_RATE_V1: u32 = 1 << 31;
const BODY7_DATA_RATE: u32 = 0x1FF << 16;
const INFO0_DATA_STBC: u32 = 1 << 12;
const INFO0_DATA_LDPC: u32 = 1 << 11;
const INFO0_DISDATAFB: u32 = 1 << 10;
const INFO1_DATA_RTY_LOWEST_RATE: u32 = 0x1FF << 16;
const INFO1_MAX_AGGNUM: u32 = 0xFF;
const INFO2_AMPDU_DENSITY: u32 = 0x7 << 18;
const INFO2_FORCE_KEY_EN: u32 = 1 << 8;
const INFO2_SEC_CAM_IDX: u32 = 0xFF;
const INFO4_HW_RTS_EN: u32 = 1 << 31;
const INFO4_RTS_EN: u32 = 1 << 27;
const BODY_BYTES: usize = 32;
const _: () = assert!(WD_HEAD_BYTES == BODY_BYTES + 24);

/// `rtw89_core_get_qsel` (`txrx.h:833-852`): TIDs 0 and 3 are BE_0, 1 and
/// 2 BK_0, 4 and 5 VI_0, 6 and 7 VO_0. Any other value is BE_0, where Linux
/// also warns.
pub const fn qsel_of_tid(tid: u8) -> u8 {
    match tid {
        1 | 2 => QSEL_BK_0,
        4 | 5 => QSEL_VI_0,
        6 | 7 => QSEL_VO_0,
        _ => QSEL_BE_0,
    }
}

/// `rtw89_core_get_tid_indicate` (`txrx.h:873-890`): TIDs 2, 3, 5 and 7,
/// the second TID of each access category. Any other value is `false`.
pub const fn tid_indicate(tid: u8) -> bool {
    matches!(tid, 2 | 3 | 5 | 7)
}

/// The rate `rtw89_core_get_mgmt_rate` (`core.c:748-786`) gives a station's
/// management frame: CCK1 (0x0) on 2.4 GHz and OFDM6 (0x4) otherwise, plus
/// the index of the lowest basic rate when `basic` names one. `basic` is the
/// joined BSS's basic-rate bitmap in mac80211's band order (on 2.4 GHz CCK
/// 1, 2, 5.5, 11 then OFDM 6-54 Mb/s, elsewhere OFDM only, rtw89_bitrates,
/// `core.c:153-166`); it is 0 for a frame Linux would send with no station
/// (`!rtwsta_link`). mac80211's `IEEE80211_TX_CTL_NO_CCK_RATE` and P2P
/// are not followed.
pub fn management_rate(band: HwBand, basic: u16) -> u8 {
    lowest_of(band, basic)
}

/// The retry floor `rtw89_core_get_data_rate` (`core.c:1080-1120`) gives a
/// station's data frame: [`management_rate`]'s rule over the peer's
/// supported rates (`link_sta->supp_rates[band]`), with no rate pattern and
/// no P2P.
pub fn lowest_data_rate(band: HwBand, supported: u16) -> u8 {
    lowest_of(band, supported)
}

/// `lowest_rate`, plus `__ffs(rates)` when `rates` is not empty.
fn lowest_of(band: HwBand, rates: u16) -> u8 {
    let lowest = match band {
        HwBand::G2 => HW_RATE_CCK1,
        HwBand::G5 | HwBand::G6 => HW_RATE_OFDM6,
    };
    if rates == 0 {
        lowest
    } else {
        lowest + rates.trailing_zeros() as u8
    }
}

/// The WD head of a management frame (`bytes`, its MAC header first, no
/// FCS) on band 0, as Linux builds it for a station without a hardware
/// key: CH8 (`B0_MGMT`), the WD info enabled, the fixed rate of
/// [`management_rate`] with fallback disabled, macid 0, RTS unless A1 is a
/// group address, and no key or header length. A null data or QoS null
/// frame is management here as in `rtw89_core_get_tx_type`. Any other
/// frame, and one shorter than its 802.11 header or longer than
/// [`MAX_TX_FRAME`], is refused. A robust management frame goes as sealed by the host, with
/// its Protected bit and without a key (M3 spec S1.2).
pub fn management(
    bytes: &[u8],
    band: HwBand,
    basic: u16,
) -> Result<(TxChannel, [u8; WD_HEAD_BYTES]), AxError> {
    let fc = frame_control(bytes)?;
    if !(is_mgmt(fc) || is_any_nullfunc(fc)) {
        return Err(AxError::Tx("not a management or null data frame"));
    }
    let frame = Frame::read(bytes, fc)?;
    let qsel = QSEL_B0_MGMT;
    let ch = MGMT_CHANNEL;
    let info = DescInfo {
        pkt_size: frame.len,
        seq: frame.seq,
        is_bmc: frame.is_bmc,
        wd_page: true,
        qsel,
        ch_dma: ch.ring() as u8,
        mac_id: MAC_ID,
        // fixed data rate for mgmt frames
        en_wd_info: true,
        use_rate: true,
        dis_data_fb: true,
        data_rate: u16::from(management_rate(band, basic)),
        ..DescInfo::default()
    };
    Ok((ch, fill_txdesc_v1(&info)))
}

/// The WD head of a data frame (`bytes`, its MAC header first, no FCS and,
/// with a key, no CCMP header: see [`strip_ccmp`]) on band 0, as Linux
/// builds it for a station: the queue of `tid` (masked to 0-7 as
/// `skb->priority` is), the WD info enabled, macid 0, the firmware's rate
/// (`use_rate` 0) with the retry floor of [`lowest_data_rate`], the peer's
/// STBC and LDPC, BK for an EAPOL frame (`eapol`), RTS unless A1 is a group
/// address, and the 802.11 header length. With `sec`, the frame is
/// encrypted as CCMP-128 by the hardware under CAM entry `sec.cam` with
/// `sec.pn` in BODY4/5 and `wp_offset` 1.
/// With `agg`, a frame that is not EAPOL goes in an A-MPDU: AGG_EN,
/// MAX_AGGNUM and AMPDU_DENSITY as `rtw89_core_tx_update_ampdu_info` sets
/// them (`core.c:636-675`); an EAPOL frame never does, whatever `agg` says.
///
/// `tid` is `skb->priority`, whose TID mac80211 also writes into a QoS
/// frame's QoS control (`ieee80211_set_qos_hdr`, `net/mac80211/wme.c:193-232`)
/// and which it sets to 0 for a peer without WMM, whose frames carry no QoS
/// control (`__ieee80211_select_queue`, `wme.c:167-170`).
///
/// Refused: a frame that is not data or is null data (management, see
/// [`management`]), one shorter than its 802.11 header or longer than
/// [`MAX_TX_FRAME`], a `tid` whose TID is not the frame's own (its QoS
/// control's four TID bits, 0 without one), a Protected frame without `sec`
/// (no data frame is sealed on the host, M3 spec S1.2, so it would leave
/// unencrypted), `sec` for a frame without the Protected bit (mac80211
/// hands a hardware key only with a Protected frame), and a key the
/// descriptor cannot carry (key ID past 3, PN 0 or past 48 bits, CAM entry
/// past 127).
#[expect(
    clippy::too_many_arguments,
    reason = "rtw89_core_tx_update_desc_info's inputs for one data frame, each a field of its own."
)]
pub fn data(
    bytes: &[u8],
    tid: u8,
    eapol: bool,
    band: HwBand,
    supported: u16,
    caps: RateCaps,
    sec: Option<TxSec>,
    agg: Option<Aggregate>,
) -> Result<(TxChannel, [u8; WD_HEAD_BYTES]), AxError> {
    let fc = frame_control(bytes)?;
    if !is_data(fc) || is_any_nullfunc(fc) {
        return Err(AxError::Tx("not a data frame with a body"));
    }
    let frame = Frame::read(bytes, fc)?;
    match (fc & FCTL_PROTECTED != 0, sec.is_some()) {
        (true, false) => return Err(AxError::Tx("a protected data frame without its key")),
        (false, true) => return Err(AxError::Tx("a key for an unprotected data frame")),
        _ => {}
    }
    // rtw89_core_tx_update_data_info
    let tid = tid & QOS_CTL_TAG1D_MASK;
    if frame.tid != tid {
        return Err(AxError::Tx("a TID other than the frame's own"));
    }
    let ch = TID_CHANNEL[usize::from(tid)];
    let mut info = DescInfo {
        pkt_size: frame.len,
        seq: frame.seq,
        is_bmc: frame.is_bmc,
        wd_page: true,
        ch_dma: ch.ring() as u8,
        tid_indicate: tid_indicate(tid),
        qsel: qsel_of_tid(tid),
        mac_id: MAC_ID,
        stbc: caps.stbc,
        ldpc: caps.ldpc,
        // enable wd_info for AMPDU
        en_wd_info: true,
        ..DescInfo::default()
    };
    if let Some(sec) = sec {
        update_sec_key(&mut info, sec)?;
    }
    info.data_retry_lowest_rate = u16::from(lowest_data_rate(band, supported));
    // rtw89_core_tx_update_ampdu_info: an EAPOL frame goes with BK and
    // returns before any aggregation (core.c:649-651); an aggregated frame
    // takes its agreement's size, or 4 << factor without one, less one as a
    // byte (core.c:668-674).
    info.bk = eapol;
    if let Some(agg) = agg.filter(|_| !eapol) {
        let num = if agg.agg_num != 0 {
            agg.agg_num
        } else {
            // The HT exponent is two bits (`IEEE80211_HT_AMPDU_PARM_FACTOR`).
            4 << (agg.factor & 0x3)
        };
        info.agg_en = true;
        info.ampdu_num = num.wrapping_sub(1) as u8;
        info.ampdu_density = agg.density;
    }
    // rtw89_core_tx_update_llc_hdr: in units of 2 bytes.
    info.hdr_llc_len = (frame.header >> 1) as u8;
    Ok((ch, fill_txdesc_v1(&info)))
}

/// Remove the 8-byte CCMP header a station's `Protection::HostIv` frame
/// carries after its MAC header (24 bytes, 30 with four addresses, plus 2
/// for QoS and 4 more for QoS with Order, cfg80211's `ieee80211_hdrlen`),
/// once it is exactly `[pn0, pn1, 0, 0x20 | keyid << 6, pn2, pn3, pn4,
/// pn5]`, the header the reservation gave (`HostTxNonce::into_header`).
/// The frame keeps its Protected bit; its new length is the `L` of
/// [`data`], whose `sec.pn` must be this `pn`.
///
/// Anything else changes nothing in `mpdu`: a frame that is not data, one
/// without the Protected bit (the 8 bytes after its header are plaintext,
/// whatever they hold), one shorter than its MAC header and 8 bytes, a key
/// ID past 3, a PN of 0 or past 48 bits, and a header that is not the
/// reservation's (`Tx("CCMP header does not match its reservation")`), so
/// no frame is queued under a PN other than the one reserved for it.
pub fn strip_ccmp(mpdu: &mut Vec<u8>, pn: u64, keyid: u8) -> Result<(), AxError> {
    let fc = frame_control(mpdu)?;
    if !is_data(fc) {
        return Err(AxError::Tx("not a data frame"));
    }
    if fc & FCTL_PROTECTED == 0 {
        return Err(AxError::Tx("not a protected frame"));
    }
    let header = hdrlen(fc);
    if mpdu.len() < header {
        return Err(AxError::Tx("frame shorter than its 802.11 header"));
    }
    check_ccmp(keyid, pn)?;
    let reserved = ccmp_header(pn, keyid);
    let at = header..header + CCMP_HDR_LEN;
    let Some(found) = mpdu.get(at.clone()) else {
        return Err(AxError::Tx("frame shorter than its CCMP header"));
    };
    if found != reserved {
        return Err(AxError::Tx("CCMP header does not match its reservation"));
    }
    mpdu.drain(at);
    Ok(())
}

/// mac80211's `ccmp_pn2hdr` (Ext IV b5 of byte 3), as `security::tx`
/// builds a host reservation's header.
fn ccmp_header(pn: u64, keyid: u8) -> [u8; CCMP_HDR_LEN] {
    let [p0, p1, p2, p3, p4, p5, _, _] = pn.to_le_bytes();
    [p0, p1, 0, 0x20 | keyid << 6, p2, p3, p4, p5]
}

/// A key ID the header and SEC_KEYID [5:4] carry, and a PN of 48 bits that
/// a reservation can hold (they start at 1).
fn check_ccmp(keyid: u8, pn: u64) -> Result<(), AxError> {
    if keyid > 3 {
        return Err(AxError::Tx("a CCMP key ID past 3"));
    }
    if pn == 0 || pn >= PN_LIMIT {
        return Err(AxError::Tx("a CCMP packet number of 0 or past 48 bits"));
    }
    Ok(())
}

/// `rtw89_core_tx_update_sec_key` (`core.c:680-746`) for a CCMP-128 key on
/// an `hw_sec_hdr` chip: the PN's six bytes and `wp_offset` 1 ("in unit of
/// 8 bytes for security header").
fn update_sec_key(info: &mut DescInfo, sec: TxSec) -> Result<(), AxError> {
    check_ccmp(sec.keyid, sec.pn)?;
    if sec.cam >= SCAM_NUM {
        return Err(AxError::Tx("a security CAM index past 127"));
    }
    info.sec_en = true;
    info.sec_keyid = sec.keyid;
    info.sec_type = SEC_KEY_TYPE_CCMP128;
    info.sec_cam_idx = sec.cam;
    let [p0, p1, p2, p3, p4, p5, _, _] = sec.pn.to_le_bytes();
    info.sec_seq = [p0, p1, p2, p3, p4, p5];
    info.wp_offset = 1;
    Ok(())
}

/// `rtw89_core_get_ch_dma` (`core.c:819-854`) of a qsel queued here. It is
/// only evaluated in the constants below, so a qsel without a channel
/// fails the build.
const fn ch_dma(qsel: u8) -> TxChannel {
    match TxChannel::of_qsel(qsel) {
        Some(ch) => ch,
        None => panic!("a qsel without a TX channel"),
    }
}

const MGMT_CHANNEL: TxChannel = ch_dma(QSEL_B0_MGMT);
/// The channel of each TID's qsel.
const TID_CHANNEL: [TxChannel; 8] = {
    let mut table = [TxChannel::Ach0; 8];
    let mut tid = 0;
    while tid < table.len() {
        table[tid] = ch_dma(qsel_of_tid(tid as u8));
        tid += 1;
    }
    table
};

fn frame_control(bytes: &[u8]) -> Result<u16, AxError> {
    match bytes {
        [a, b, ..] => Ok(u16::from_le_bytes([*a, *b])),
        _ => Err(AxError::Tx("frame shorter than its 802.11 header")),
    }
}

/// `ieee80211_is_mgmt`.
const fn is_mgmt(fc: u16) -> bool {
    fc & FCTL_FTYPE == FTYPE_MGMT
}

/// `ieee80211_is_data`.
const fn is_data(fc: u16) -> bool {
    fc & FCTL_FTYPE == FTYPE_DATA
}

/// `ieee80211_is_any_nullfunc`.
const fn is_any_nullfunc(fc: u16) -> bool {
    is_data(fc) && matches!(fc & FCTL_STYPE, STYPE_NULLFUNC | STYPE_QOS_NULLFUNC)
}

/// cfg80211's `ieee80211_hdrlen` for a management or data frame.
const fn hdrlen(fc: u16) -> usize {
    if is_data(fc) {
        let mut len = if fc & (FCTL_TODS | FCTL_FROMDS) == FCTL_TODS | FCTL_FROMDS {
            HDR_LEN_4ADDR
        } else {
            HDR_LEN_3ADDR
        };
        if fc & STYPE_QOS_DATA != 0 {
            len += QOS_CTL_LEN;
            if fc & FCTL_ORDER != 0 {
                len += HT_CTL_LEN;
            }
        }
        len
    } else if fc & FCTL_ORDER != 0 {
        HDR_LEN_3ADDR + HT_CTL_LEN
    } else {
        HDR_LEN_3ADDR
    }
}

/// What `rtw89_core_tx_update_desc_info` reads from every frame.
struct Frame {
    /// `skb->len`: TXPKT_SIZE.
    len: u16,
    /// `ieee80211_hdrlen`.
    header: usize,
    /// `(seq_ctrl & IEEE80211_SCTL_SEQ) >> 4`.
    seq: u16,
    /// `is_broadcast_ether_addr(addr1) || is_multicast_ether_addr(addr1)`.
    is_bmc: bool,
    /// A data frame's QoS control TID (`IEEE80211_QOS_CTL_TID_MASK`), 0
    /// without one, as for a management frame.
    tid: u8,
}

impl Frame {
    /// `bytes` with frame control `fc`, a management or data frame.
    fn read(bytes: &[u8], fc: u16) -> Result<Self, AxError> {
        let header = hdrlen(fc);
        if bytes.len() < header {
            return Err(AxError::Tx("frame shorter than its 802.11 header"));
        }
        if bytes.len() > MAX_TX_FRAME {
            return Err(AxError::Tx("frame longer than one TX address entry"));
        }
        let seq_ctrl = u16::from_le_bytes([bytes[SEQ_CTRL], bytes[SEQ_CTRL + 1]]);
        // `ieee80211_get_qos_ctl`: right after the three or four addresses.
        let tid = if is_data(fc) && fc & STYPE_QOS_DATA != 0 {
            let at = if fc & (FCTL_TODS | FCTL_FROMDS) == FCTL_TODS | FCTL_FROMDS {
                HDR_LEN_4ADDR
            } else {
                HDR_LEN_3ADDR
            };
            bytes[at] & QOS_CTL_TID_MASK
        } else {
            0
        };
        Ok(Self {
            len: bytes.len() as u16,
            header,
            seq: (seq_ctrl & SCTL_SEQ) >> 4,
            is_bmc: bytes[ADDR1] & 1 != 0,
            tid,
        })
    }
}

/// The fields of `struct rtw89_tx_desc_info` (`core.h:1253-1299`) a frame
/// here sets; the module doc names the ones that stay 0, as they do for
/// these frames in Linux. `addr_info_nr` is always 1: a frame of at most
/// [`MAX_TX_FRAME`] takes one TX address entry
/// (`rtw89_pci_fill_txaddr_info_v1`, `pci.c:1457-1491`).
#[derive(Default)]
struct DescInfo {
    pkt_size: u16,
    wp_offset: u8,
    mac_id: u8,
    qsel: u8,
    ch_dma: u8,
    hdr_llc_len: u8,
    is_bmc: bool,
    en_wd_info: bool,
    wd_page: bool,
    use_rate: bool,
    dis_data_fb: bool,
    tid_indicate: bool,
    bk: bool,
    agg_en: bool,
    ampdu_num: u8,
    ampdu_density: u8,
    sec_en: bool,
    sec_keyid: u8,
    sec_type: u8,
    sec_cam_idx: u8,
    sec_seq: [u8; 6],
    data_rate: u16,
    data_retry_lowest_rate: u16,
    seq: u16,
    stbc: bool,
    ldpc: bool,
}

/// `FIELD_PREP(mask, value)`.
const fn prep(mask: u32, value: u32) -> u32 {
    (value << mask.trailing_zeros()) & mask
}

/// `rtw89_core_fill_txdesc_v1` (`core.c:1661-1687`) with its body v1 and
/// info builders (`core.c:1494-1634`): BODY4/5 only with a key, the info
/// only with `en_wd_info`.
fn fill_txdesc_v1(d: &DescInfo) -> [u8; WD_HEAD_BYTES] {
    let mut body = [0u32; 8];
    body[0] = prep(BODY0_WP_OFFSET_V1, d.wp_offset.into())
        | prep(BODY0_WD_INFO_EN, d.en_wd_info.into())
        | prep(BODY0_CHANNEL_DMA, d.ch_dma.into())
        | prep(BODY0_HDR_LLC_LEN, d.hdr_llc_len.into())
        | prep(BODY0_WD_PAGE, d.wd_page.into());
    body[1] = prep(BODY1_ADDR_INFO_NUM, 1)
        | prep(BODY1_SEC_KEYID, d.sec_keyid.into())
        | prep(BODY1_SEC_TYPE, d.sec_type.into());
    body[2] = prep(BODY2_TID_INDICATE, d.tid_indicate.into())
        | prep(BODY2_QSEL, d.qsel.into())
        | prep(BODY2_TXPKT_SIZE, d.pkt_size.into())
        | prep(BODY2_MACID, d.mac_id.into());
    body[3] = prep(BODY3_SW_SEQ, d.seq.into())
        | prep(BODY3_AGG_EN, d.agg_en.into())
        | prep(BODY3_BK, d.bk.into());
    if d.sec_en {
        let s = d.sec_seq.map(u32::from);
        body[4] = prep(BODY4_SEC_IV_L0, s[0]) | prep(BODY4_SEC_IV_L1, s[1]);
        body[5] = prep(BODY5_SEC_IV_H2, s[2])
            | prep(BODY5_SEC_IV_H3, s[3])
            | prep(BODY5_SEC_IV_H4, s[4])
            | prep(BODY5_SEC_IV_H5, s[5]);
    }
    body[7] =
        prep(BODY7_USE_RATE_V1, d.use_rate.into()) | prep(BODY7_DATA_RATE, d.data_rate.into());
    let mut info = [0u32; 6];
    if d.en_wd_info {
        info[0] = prep(INFO0_DATA_STBC, d.stbc.into())
            | prep(INFO0_DATA_LDPC, d.ldpc.into())
            | prep(INFO0_DISDATAFB, d.dis_data_fb.into());
        info[1] = prep(INFO1_MAX_AGGNUM, d.ampdu_num.into())
            | prep(INFO1_DATA_RTY_LOWEST_RATE, d.data_retry_lowest_rate.into());
        info[2] = prep(INFO2_AMPDU_DENSITY, d.ampdu_density.into())
            | prep(INFO2_FORCE_KEY_EN, d.sec_en.into())
            | prep(INFO2_SEC_CAM_IDX, d.sec_cam_idx.into());
        info[4] = prep(INFO4_RTS_EN, (!d.is_bmc).into()) | prep(INFO4_HW_RTS_EN, 1);
    }
    let mut head = [0u8; WD_HEAD_BYTES];
    for (i, word) in body.iter().chain(&info).enumerate() {
        head[4 * i..4 * i + 4].copy_from_slice(&word.to_le_bytes());
    }
    head
}

#[cfg(test)]
mod tests;

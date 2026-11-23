// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) AND GPL-2.0-only
// Copyright(c) 2019-2020  Realtek Corporation
// The QoS defaults and the WMM parameter checks are adapted from pinned Linux
// net/mac80211/util.c (ieee80211_set_wmm_default) and mlme.c
// (_ieee80211_sta_wmm_params): Copyright 2002-2005, Instant802 Networks, Inc.;
// 2003-2008, Jouni Malinen; 2005-2006, Devicescape Software, Inc.; 2006-2007
// Jiri Benc; 2007 Johannes Berg; 2007 Michael Wu; 2013-2014 Intel Mobile
// Communications GmbH; 2015-2017 Intel Deutschland GmbH; 2018-2026 Intel
// Corporation. See NOTICE.
//! A station's join, association and leaving on an RTL8852C (M3 spec S9-S11),
//! as Linux's mac80211 drives rtw89 through them, each step the H2Cs and
//! register accesses of the rtw89 callback mac80211 calls there, in that
//! order:
//!
//! - [`set_bssid`], before authentication: `BSS_CHANGED_BSSID`
//!   (`rtw89_ops_link_info_changed`, `mac80211.c:771-776`: the address CAM
//!   with the BSSID, `cam.c:734-739`), then the AP's station added
//!   (`rtw89_core_sta_link_add`, `core.c:5735-5797`, whose only chip access
//!   on this chip is `rtw89_phy_dig_suspend`), the order mac80211's
//!   `ieee80211_prep_connection` gives them (`mlme.c:9773-9789`).
//! - [`associate`], at the association response: mac80211's
//!   `ieee80211_assoc_config_link` programs the AP's WMM parameters first,
//!   or 802.11's QoS defaults without valid ones (`mlme.c:6308-6323`)
//!   (`rtw89_ops_conf_tx`, `mac80211.c:912-937`, for VO, VI, BE and BK,
//!   `mlme.c:4094-4114`, `:6310`) with the long slot, which every
//!   disassociation resets (`ieee80211_reset_erp_info`, `main.c:430-438`);
//!   then `BSS_CHANGED_ASSOC` (`mlme.c:4392-4461`):
//!   `rtw89_station_mode_sta_assoc` (`mac80211.c:661-676`) with
//!   `rtw89_vif_type_mapping` to INFRA and `rtw89_core_sta_link_assoc`
//!   (`core.c:5878-5979`: the CMAC table, join info, the address CAM with
//!   the AID, `rtw89_phy_ra_assoc` (`phy.c:731-776`), the beacon filter and
//!   `rtw89_phy_dig_resume(false)`), then `__rtw89_ops_bss_link_assoc`
//!   (`mac80211.c:678-686`: [`vif::port_update`] for INFRA); then, on the
//!   same notify, `BSS_CHANGED_ERP_SLOT` when the slot turned short
//!   (`mlme.c:4323-4326`, always on 5 GHz): `rtw89_conf_tx` again
//!   (`mac80211.c:781-782`).
//! - [`leave`], from an association: mac80211's `ieee80211_set_disassoc`
//!   (`mlme.c:4506-4718`) moves the AP's station down (`sta_info.c:
//!   1572-1606`): ASSOC to AUTH, `rtw89_core_sta_link_disassoc`
//!   (`core.c:5799-5816`: the beacon filter off); AUTH to NONE,
//!   `rtw89_core_sta_link_disconnect` (`core.c:5818-5859`: the beamformee
//!   off, `rtw89_vif_type_mapping` to NO_LINK, the CMAC table, join info
//!   disconnected, the address CAM without the link); NONE to NOTEXIST,
//!   `rtw89_core_sta_link_remove` (`core.c:5981-6006`), which touches
//!   nothing on this chip. Then `BSS_CHANGED_BSSID` with the BSSID cleared
//!   (the address CAM of [`clear_bssid`]), `BSS_CHANGED_ERP_SLOT`
//!   (`rtw89_conf_tx` with the AP's parameters and the long slot) and
//!   `BSS_CHANGED_MU_GROUPS` (`rtw89_mac_bf_set_gid_table`, `mac.c:
//!   6914-6953`, the cleared GID table), in `rtw89_ops_link_info_changed`'s
//!   order; then `ieee80211_set_wmm_default` without QoS (`util.c:
//!   1025-1135`). Linux runs neither `rtw89_fw_h2c_role_maintain` nor a port
//!   update here: 0xC400 stays as the association set it (spec S11.6).
//! - [`clear_bssid`], a join that never associated: the BSSID cleared
//!   (`ieee80211_destroy_auth_data`'s and `ieee80211_destroy_assoc_data`'s
//!   `BSS_CHANGED_BSSID`); DIG stays suspended, as Linux resumes it only at
//!   association or after a scan. When the AP had authenticated the
//!   station, [`disconnect`] comes first: mac80211 destroys the AP's
//!   station from AUTH (`sta_info_destroy_addr`), through AUTH to NONE's
//!   `rtw89_core_sta_link_disconnect`, before it clears the BSSID.
//!
//! Every H2C goes through the command session with no wait, as
//! `rtw89_h2c_tx` sends them (spec S10); their acks are recorded, not waited
//! on, and frames drained meanwhile go to the caller's sink. The first
//! failure is returned and nothing after it runs. The pure parts are
//! [`ra_info`] (`rtw89_phy_ra_sta_update`, `phy.c:322-503`, with
//! `rtw89_phy_ra_assoc`'s RSSI level) and [`edca_param`]
//! (`____rtw89_conf_tx_edca`, `mac80211.c:393-431`).
//!
//! The station is HT 20 MHz or legacy toward its AP (spec S1.7), so every
//! HE, VHT, EHT and beamforming branch of these callbacks does nothing:
//! `rtw89_phy_set_bss_color`, `rtw89_chip_cfg_txpwr_ul_tb_offset`,
//! `rtw89_mac_set_he_obss_narrow_bw_ru` and `rtw89_mac_bf_assoc` return
//! before any access without HE support or a VHT or HE beamformer;
//! `rtw89_mac_set_he_tb` and `rtw89_fw_h2c_init_trx_protect` are BE-only;
//! `rtw89_phy_ul_tb_assoc` needs `ul_tb_waveform_ctrl`, false on 8852C;
//! `rtw89_queue_chanctx_work` does nothing for one channel context; rtw89
//! has no `mgd_prepare_tx`. `rtw89_phy_ra_assoc` runs before
//! `rtw89_sta_link_can_er` sets `er_cap` (`core.c:5935-5947`), so the RA
//! H2C carries the zeroed new station's ER bit, 0.
//!
//! The rtw89 references are to the donor tree (`a804c5742bff`), and the
//! mac80211 ones (`mlme.c`, `main.c`, `util.c`, `sta_info.c`, all under
//! `net/mac80211/`) to the same Linux commit's `net/mac80211/`.
//!
//! This module does not grant (spec S1.15): `rtw89_fw_h2c_general_pkt` and
//! its templates, the BTC role notifies, `rtw89_chip_rfk_channel`, LPS, the
//! watchdog, keys (the deletes that come before [`leave`]'s beacon filter,
//! `sta_info.c:1577-1578`, are [`super::keys::delete`]'s; [`leave`] carries
//! the key fields they leave), RX block ack teardown, TX flushes, and any
//! channel switch. Nor `ieee80211_regulatory_limit_wmm_params` on the
//! defaults [`associate`] and [`leave`] program: the station layer limits
//! the AP's parameters, which [`associate`] takes as given.
use super::chan::HwBand;
use super::command::Session;
use super::dm::Dig;
use super::io::{AxError, AxIo, clear32};
use super::receive::Event;
use super::vif::{self, MAC_ID, NET_TYPE_INFRA, NET_TYPE_NO_LINK, PORT};
use crate::wifi::station::phy::AccessCategory;

pub use super::vif::PortConfig;

/// The address CAM's security fields (`cam.c:906-927`): the key IDs of
/// w9 `[31:18]` in place, and w10 and w11 whole, as [`super::keys::KeyCam`]
/// holds them; 0 before any key.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CamKeys {
    /// `SEC_ENT0_KEYID` to `SEC_ENT6_KEYID`, w9 `[31:18]`; other bits are
    /// ignored.
    pub keyids: u32,
    /// w10: `SEC_ENT_VALID` `[7:0]`, `SEC_ENT0` to `SEC_ENT2`.
    pub valid_and_ents: u32,
    /// w11: `SEC_ENT3` to `SEC_ENT6`.
    pub ents_hi: u32,
}

impl CamKeys {
    const NONE: Self = Self {
        keyids: 0,
        valid_and_ents: 0,
        ents_hi: 0,
    };
}

/// What the station's address CAM H2C carries (`rtw89_cam_fill_addr_cam_info`
/// and `rtw89_cam_fill_bssid_cam_info`, `cam.c:762-927`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AddrCam {
    /// The SMA, the vif's own address.
    pub mac: [u8; 6],
    /// The TMA and the BSSID CAM's BSSID: the AP's address, which is its
    /// BSSID, or zero without a BSS.
    pub bssid: [u8; 6],
    /// `rtwvif_link->net_type`.
    pub net_type: u8,
    /// `vif->cfg.aid`, carried when the net type is INFRA.
    pub aid: u16,
    pub keys: CamKeys,
}

impl AddrCam {
    /// The vif with no BSS: vif init, the scan's start and restore, and a
    /// cleared BSSID (spec S7.4).
    pub const fn unassociated(mac: [u8; 6]) -> Self {
        Self::with_bssid(mac, [0; 6])
    }

    /// NO_LINK with `bssid` as TMA and BSSID: the BSSID set before
    /// authentication (spec S9.2, no station: TMA is the vif's BSSID), and
    /// the AP's station after its disconnection (spec S11.5, TMA its
    /// address), which build the same bytes.
    pub const fn with_bssid(mac: [u8; 6], bssid: [u8; 6]) -> Self {
        Self {
            mac,
            bssid,
            net_type: NET_TYPE_NO_LINK,
            aid: 0,
            keys: CamKeys::NONE,
        }
    }

    /// The associated station (spec S10.3): INFRA with its AID.
    pub const fn connected(mac: [u8; 6], bssid: [u8; 6], aid: u16) -> Self {
        Self {
            mac,
            bssid,
            net_type: NET_TYPE_INFRA,
            aid,
            keys: CamKeys::NONE,
        }
    }
}

/// The AP's negotiated HT capabilities toward the station
/// (`link_sta->ht_cap` after `ieee80211_ht_cap_ie_to_sta_ht_cap`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HtPeer {
    /// `mcs.rx_mask[0..4]`.
    pub rx_mask: [u8; 4],
    /// `IEEE80211_HT_CAP_LDPC_CODING`.
    pub ldpc: bool,
    /// `IEEE80211_HT_CAP_RX_STBC` non-zero.
    pub rx_stbc: bool,
    /// `IEEE80211_HT_CAP_SGI_20`.
    pub sgi20: bool,
}

/// What `rtw89_phy_ra_assoc` reads of the AP's station.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RaPeer {
    /// `link_sta->supp_rates[band]`: on 2.4 GHz CCK 1-11 in bits 0-3 and
    /// OFDM 6-54 in bits 4-11, elsewhere OFDM 6-54 in bits 0-7 (rtw89's
    /// bands, `core.c:153-166`, `:293-312`).
    pub legacy: u16,
    /// HT, or `None` for a legacy peer.
    pub ht: Option<HtPeer>,
    /// `ewma_rssi_read(&rtwsta_link->avg_rssi)`: the average of the PPDU
    /// RSSI (110 + dBm, in half dB) of the AP's frames to this station
    /// (`core.c:2036-2061`), 0 while none was averaged, which is what the
    /// receiver reports today (spec S1.9: no RSSI floor, level 0).
    pub rssi: u8,
}

/// `struct rtw89_ra_info` as `rtw89_phy_ra_assoc` fills it for an HT or
/// legacy peer at 20 MHz: no DCM, no ER, no GI-LTF, no CSI.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RaInfo {
    /// `mode_ctrl`: `RTW89_RA_MODE_CCK` 1 | `_OFDM` 2 | `_HT` 4.
    pub mode: u8,
    pub ra_mask: u64,
    /// `en_sgi`.
    pub sgi: bool,
    /// `ldpc_cap`.
    pub ldpc: bool,
    /// `stbc_cap`.
    pub stbc: bool,
    pub ss_num: u8,
    pub init_rate_lv: u8,
}

/// `RTW89_RA_MODE_*` (`core.h:4441-4446`).
const RA_MODE_CCK: u8 = 1 << 0;
const RA_MODE_OFDM: u8 = 1 << 1;
const RA_MODE_HT: u8 = 1 << 2;
/// `RA_MASK_*` (`phy.h:31-38`).
const RA_MASK_CCK_RATES: u64 = 0xF;
const RA_MASK_OFDM_RATES: u64 = 0xFF0;
const RA_MASK_SUBCCK_RATES: u64 = 0x5;
const RA_MASK_SUBOFDM_RATES: u64 = 0x10;
const RA_MASK_HT_RATES: [u64; 4] = [0xFF << 12, 0xFF << 24, 0xFF << 36, 0xFF << 48];
/// `hal.tx_nss` for the RTL8852C (spec, "Every branch").
const TX_NSS: u8 = 2;
/// `rtw89_phy_ra_mask_rssi`'s `rssi_lv_t` with `RA_FLOOR_UP_GAP` (3) added
/// to every level, as `ratr_state` 0 asks (`phy.c:156-171`).
const RSSI_LEVELS: [u8; 7] = [33, 47, 51, 55, 59, 63, 103];
/// The masks of levels 1-5 and above (`phy.c:172-188`).
const RSSI_FLOORS: [u64; 5] = [
    0xFFFF_FFFF_FFFF_FFF0,
    0xFFFF_FFFF_FFFF_EFE0,
    0xFFFF_FFFF_FFFF_CFC0,
    0xFFFF_FFFF_FFFF_8F80,
    0xFFFF_FFFF_FFFF_0F00,
];

/// `rtw89_phy_ra_mask_rssi(rssi, 0)`: the average halved, its level the
/// first threshold above it, else 0.
fn rssi_floor(rssi: u8) -> u64 {
    let rssi = rssi >> 1;
    match RSSI_LEVELS.iter().position(|&t| rssi < t).unwrap_or(0) {
        0 => u64::MAX,
        lv => RSSI_FLOORS[(lv - 1).min(RSSI_FLOORS.len() - 1)],
    }
}

/// `rtw89_phy_ra_mask_recover` (`phy.c:190-199`).
fn recover(mut ra_mask: u64, bak: u64) -> u64 {
    let legacy = RA_MASK_CCK_RATES | RA_MASK_OFDM_RATES;
    if ra_mask & !legacy == 0 {
        ra_mask |= bak & !legacy;
    }
    if ra_mask == 0 {
        ra_mask |= bak & legacy;
    }
    ra_mask
}

/// `rtw89_phy_ra_sta_update(csi = false)` and `rtw89_phy_ra_assoc`'s RSSI
/// level (M3 spec S10.4) for `peer` on `band`, at 20 MHz with no bitrate
/// mask and no rate pattern:
///
/// - HT: mode HT, the RX MCS bytes at bits 12, 24, 36 and 48, STBC, LDPC and
///   SGI-20 as negotiated; the stream count is mac80211's, the number of
///   non-zero RX mask bytes (`sta_info.c:3570-3579`), at least 1.
/// - 2.4 GHz adds the legacy rates, CCK mode for any of bits 0-3, OFDM for
///   any of 4-11; elsewhere OFDM mode, the rates shifted up by 4.
/// - HT keeps the first `tx_nss` (2) streams' rates, SUBOFDM with OFDM and
///   SUBCCK with CCK; legacy with OFDM keeps OFDM and SUBCCK.
/// - Unless CCK alone, the RSSI floor of `peer.rssi`, then the recover.
/// - `ss_num = min(rx_nss, 2) - 1`; the initial rate level 1 above 40, 2
///   above 20, 3 above 1, else 0, of the average halved.
pub fn ra_info(peer: &RaPeer, band: HwBand) -> RaInfo {
    let mut mode = 0;
    let mut ra_mask = 0u64;
    let mut info = RaInfo {
        mode: 0,
        ra_mask: 0,
        sgi: false,
        ldpc: false,
        stbc: false,
        ss_num: 0,
        init_rate_lv: 0,
    };
    let mut rx_nss = 0u8;
    if let Some(ht) = &peer.ht {
        mode |= RA_MODE_HT;
        ra_mask |= u64::from(ht.rx_mask[3]) << 48
            | u64::from(ht.rx_mask[2]) << 36
            | u64::from(ht.rx_mask[1]) << 24
            | u64::from(ht.rx_mask[0]) << 12;
        info.stbc = ht.rx_stbc;
        info.ldpc = ht.ldpc;
        info.sgi = ht.sgi20;
        rx_nss = ht.rx_mask.iter().filter(|&&m| m != 0).count() as u8;
    }
    if band == HwBand::G2 {
        let supp = u64::from(peer.legacy) & (RA_MASK_CCK_RATES | RA_MASK_OFDM_RATES);
        ra_mask |= supp;
        if supp & RA_MASK_CCK_RATES != 0 {
            mode |= RA_MODE_CCK;
        }
        if supp & RA_MASK_OFDM_RATES != 0 {
            mode |= RA_MODE_OFDM;
        }
    } else {
        ra_mask |= (u64::from(peer.legacy) & 0xFF) << 4;
        mode |= RA_MODE_OFDM;
    }
    let bak = ra_mask;
    if mode >= RA_MODE_HT {
        let mut mask = RA_MASK_HT_RATES[..usize::from(TX_NSS)]
            .iter()
            .fold(0, |m, r| m | r);
        if mode & RA_MODE_OFDM != 0 {
            mask |= RA_MASK_SUBOFDM_RATES;
        }
        if mode & RA_MODE_CCK != 0 {
            mask |= RA_MASK_SUBCCK_RATES;
        }
        ra_mask &= mask;
    } else if mode & RA_MODE_OFDM != 0 {
        ra_mask &= RA_MASK_OFDM_RATES | RA_MASK_SUBCCK_RATES;
    }
    if mode != RA_MODE_CCK {
        ra_mask &= rssi_floor(peer.rssi);
    }
    info.mode = mode;
    info.ra_mask = recover(ra_mask, bak);
    info.ss_num = rx_nss.clamp(1, TX_NSS) - 1;
    // `RSSI_FACTOR` (`core.h:60`).
    let rssi = peer.rssi >> 1;
    info.init_rate_lv = if rssi > 40 {
        1
    } else if rssi > 20 {
        2
    } else if rssi > 1 {
        3
    } else {
        0
    };
    info
}

/// `____rtw89_conf_tx_edca`'s word for one access category (M3 spec S10.8):
/// TXOPLMT `[26:16]` = `txop_32us`, CWMAX `[15:12]` and CWMIN `[11:8]` the
/// exponents, AIFS `[7:0]` = `aifsn * slot + sifs` with slot 9 (short) or 20
/// and SIFS 10 on 2.4 GHz, else 16 (`rtw89_aifsn_to_aifs`,
/// `mac80211.c:393-412`, which returns it as a u8). mac80211 hands rtw89
/// `cw = 2^ecw - 1`, of which rtw89 takes `ilog2(cw + 1)`, the exponent
/// again.
pub fn edca_param(ac: &AccessCategory, short_slot: bool, band: HwBand) -> u32 {
    let slot: u32 = if short_slot { 9 } else { 20 };
    let sifs: u32 = if band == HwBand::G2 { 10 } else { 16 };
    let aifs = (u32::from(ac.aifsn) * slot + sifs) & 0xFF;
    (u32::from(ac.txop_32us) & 0x7FF) << 16
        | (u32::from(ac.ecw_max) & 0xF) << 12
        | (u32::from(ac.ecw_min) & 0xF) << 8
        | aifs
}

/// The firmware features the association reads (`fw.c:880-905` for the
/// RTL8852C).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Features {
    /// `BEACON_FILTER`, from 0.27.56.10: without it
    /// `rtw89_fw_h2c_set_bcn_fltr_cfg` sends nothing (`fw.c:5409-5410`).
    pub beacon_filter: bool,
    /// `BEACON_LOSS_COUNT_V1`, from 0.27.128.0: the loss count's cap is 127
    /// instead of 15.
    pub loss_count_v1: bool,
}

impl Features {
    /// The features of firmware version (major, minor, sub, idx).
    pub fn of(firmware: (u8, u8, u8, u8)) -> Self {
        Self {
            beacon_filter: firmware >= (0, 27, 56, 10),
            loss_count_v1: firmware >= (0, 27, 128, 0),
        }
    }
}

/// What the association programs (M3 spec S10), as mac80211 holds it at
/// `BSS_CHANGED_ASSOC`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AssocInputs {
    /// The vif's own address.
    pub mac: [u8; 6],
    /// The AP's address, its BSSID.
    pub bssid: [u8; 6],
    /// The AID of the association response.
    pub aid: u16,
    /// The band of the channel context: the channel the chip is tuned to.
    pub band: HwBand,
    /// `bss_conf->beacon_int`, in TU.
    pub beacon_int: u16,
    /// `bss_conf->dtim_period`, at least 1 (`mlme.c:7095`).
    pub dtim: u8,
    pub peer: RaPeer,
    /// The AP's EDCA parameters in ACI order (BE, BK, VI, VO), as mac80211
    /// hands them to `conf_tx`. `None`, the AP without WMM, and a set
    /// mac80211 finds invalid both program 802.11's QoS defaults, as
    /// `ieee80211_assoc_config_link` does (`mlme.c:6308-6323`,
    /// [`qos_defaults`]; see [`AssocInputs::params`]).
    pub edca: Option<[AccessCategory; 4]>,
    /// `bss_conf->use_short_slot` from the AP's capability; 5 GHz is always
    /// short (`mlme.c:4308-4311`).
    pub short_slot: bool,
}

impl AssocInputs {
    /// mac80211's `use_short_slot` after `ieee80211_handle_bss_capability`.
    fn short_slot(&self) -> bool {
        self.short_slot || self.band != HwBand::G2
    }

    /// `use_11b` of `ieee80211_set_wmm_default` (`util.c:1047-1049`): a 2.4
    /// GHz BSS that is not `operating_11g_mode`, which has no rate above 11
    /// Mbps (`mlme.c:6405-6406`).
    fn use_11b(&self) -> bool {
        self.band == HwBand::G2 && self.peer.legacy & RATES_ABOVE_11M == 0
    }

    /// The EDCA parameters `conf_tx` gets at the association, in ACI order:
    /// the AP's, each AIFSN under 2 taken as 2, as
    /// `_ieee80211_sta_wmm_params` takes them (`mlme.c:4214-4226`); or, with
    /// no WMM or a set it finds invalid (an AC missing, here one out of ACI
    /// order, CWmin 0 or CWmin above CWmax, `mlme.c:4228-4245`), the QoS
    /// defaults ([`qos_defaults`]).
    pub fn params(&self) -> [AccessCategory; 4] {
        let valid = |edca: &[AccessCategory; 4]| {
            edca.iter()
                .enumerate()
                .all(|(i, c)| usize::from(c.aci) == i && c.ecw_min != 0 && c.ecw_min <= c.ecw_max)
        };
        match self.edca {
            Some(mut edca) if valid(&edca) => {
                for c in &mut edca {
                    c.aifsn = c.aifsn.max(2);
                }
                edca
            }
            _ => qos_defaults(self.use_11b()),
        }
    }
}

/// The 2.4 GHz rates above 11 Mbps, OFDM 12-54 (bits 6-11 of the 12-bit
/// order).
const RATES_ABOVE_11M: u16 = 0xFC0;

/// mac80211's AC order, VO, VI, BE, BK (`IEEE80211_AC_VO` 0 to `_BK` 3),
/// as the ACIs `ac_to_fw_idx` maps them to (`mac80211.c:386-391`).
const MAC80211_AC_ORDER: [u8; 4] = [3, 2, 0, 1];

/// `R_AX_BFMEE_RESP_OPTION` and its NDPA enables, HT b0, VHT b1, HE b2
/// (`reg.h:3252-3264`): 8852C's `bfee_ctrl` (`mac.c:7468-7472`).
const R_AX_BFMEE_RESP_OPTION: u32 = 0xCD80;
const B_AX_BFMEE_NDPA_EN: u32 = 0x7;
/// `rtw89_mac_mu_gid_addr_ax` (`mac.c:4489-4493`, `reg.h:2178-2189`): the
/// two position enables, then the four positions.
const GID_TABLE: [u32; 6] = [0xC080, 0xC084, 0xC070, 0xC074, 0xC078, 0xC07C];

/// `rtw89_conf_tx` (`mac80211.c:479-486`), each AC in mac80211's order:
/// one EDCA H2C for `params` with the slot `short_slot` gives on `band`.
///
/// # Safety
/// [`associate`]'s contract.
unsafe fn conf_tx<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    params: &[AccessCategory; 4],
    short_slot: bool,
    band: HwBand,
    sink: &mut dyn FnMut(Event<'_>),
) -> Result<(), AxError> {
    for aci in MAC80211_AC_ORDER {
        let param = edca_param(&params[usize::from(aci)], short_slot, band);
        let packet = session.h2c.edca(aci, param);
        // SAFETY: the caller's contract is `send`'s, word for word.
        unsafe { session.send(io, packet, sink) }?;
    }
    Ok(())
}

/// `ieee80211_set_wmm_default(enable_qos = true)` (`util.c:1054-1117`,
/// 802.11-2007 Table 7-37), in ACI order: aCWmin 31 when `use_11b`, else 15, and
/// aCWmax 1023; BE AIFSN 3 and BK 7, each CWmin aCWmin and CWmax aCWmax, no
/// TXOP; VI AIFSN 2, CW `(aCWmin + 1) / 2 - 1` to aCWmin, TXOP 3008 us
/// (6016 with `use_11b`); VO AIFSN 2, CW `(aCWmin + 1) / 4 - 1` to
/// `(aCWmin + 1) / 2 - 1`, TXOP 1504 us (3264). As exponents: aCWmin 15 is
/// 4 and 31 is 5.
pub fn qos_defaults(use_11b: bool) -> [AccessCategory; 4] {
    let cw = if use_11b { 5 } else { 4 };
    let ac = |aci: u8, aifsn: u8, ecw_min: u8, ecw_max: u8, txop_32us: u16| AccessCategory {
        aci,
        admission_control: false,
        aifsn,
        ecw_min,
        ecw_max,
        txop_32us,
    };
    let (vi_txop, vo_txop) = if use_11b {
        (6016 / 32, 3264 / 32)
    } else {
        (3008 / 32, 1504 / 32)
    };
    [
        ac(0, 3, cw, 10, 0),
        ac(1, 7, cw, 10, 0),
        ac(2, 2, cw - 1, cw, vi_txop),
        ac(3, 2, cw - 2, cw - 1, vo_txop),
    ]
}

/// `ieee80211_set_wmm_default(enable_qos = false)` (`util.c:1025-1135`):
/// every AC AIFSN 2, CWmax 1023, CWmin 31 when `use_11b` (a 2.4 GHz BSS
/// without a rate above 11 Mbps, `mlme.c:6405-6406`), else 15, no TXOP.
fn wmm_defaults(use_11b: bool) -> [AccessCategory; 4] {
    let ecw_min = if use_11b { 5 } else { 4 };
    core::array::from_fn(|aci| AccessCategory {
        aci: aci as u8,
        admission_control: false,
        aifsn: 2,
        ecw_min,
        ecw_max: 10,
        txop_32us: 0,
    })
}

/// The BSSID set before authentication (M3 spec S9.2-S9.3): the address CAM
/// with `bssid`, NO_LINK, through `session` with what it drains handed to
/// `sink`; then DIG suspended, unless it already is (`dig`).
///
/// # Safety
/// The contract of `Ch12::submit` and `Rxq::poll` for `io` and the
/// session's rings: `io` is the exclusive register window of the same
/// RTL8852C requester that `pci::pre_init` programmed with them, `pre_init`
/// succeeded, and its contract still holds.
pub unsafe fn set_bssid<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    mac: [u8; 6],
    bssid: [u8; 6],
    dig: &mut Dig,
    sink: &mut dyn FnMut(Event<'_>),
) -> Result<(), AxError> {
    let packet = session
        .h2c
        .addr_cam_with(&AddrCam::with_bssid(mac, bssid), MAC_ID, PORT);
    // SAFETY: the caller's contract is `send`'s, word for word.
    unsafe { session.send(io, packet, sink) }?;
    dig.suspend(io)
}

/// The association (M3 spec S10; see the module documentation), in Linux's
/// order: the EDCA parameters of [`AssocInputs::params`] with the long
/// slot; the CMAC table, join info connected, the address CAM with the AID,
/// the RA, the beacon filter when the firmware has it (`features`), DIG
/// resumed when paused with spatial reuse off (`total_sta_assoc` counts
/// this station only after `rtw89_core_sta_link_assoc` returns,
/// `mac80211.c:569-575`); the INFRA port update; with the short slot, the
/// EDCA parameters again. The port update's `beacon_int + 1` ms wait
/// ([`vif::quiet_us`]) is a busy wait here; [`associate_begin`] and
/// [`associate_end`] are the same steps with that wait left to the caller.
///
/// # Safety
/// As [`set_bssid`].
pub unsafe fn associate<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    assoc: &AssocInputs,
    features: Features,
    dig: &mut Dig,
    sink: &mut dyn FnMut(Event<'_>),
) -> Result<(), AxError> {
    // SAFETY: the caller's contract is `associate_begin`'s, word for word.
    let on = unsafe { associate_begin(io, session, assoc, features, dig, sink) }?;
    if on {
        io.delay_us(vif::quiet_us(assoc.beacon_int));
    }
    // SAFETY: as above, `associate_end`'s.
    unsafe { associate_end(io, session, assoc, on, sink) }
}

/// [`associate`] up to the INFRA port update's check: every step before
/// it, then whether the port function is on ([`vif::port_function_on`]),
/// in which case [`vif::quiet_us`] of `assoc`'s beacon interval must pass,
/// with nothing of the port changed, before [`associate_end`].
///
/// # Safety
/// As [`set_bssid`].
pub unsafe fn associate_begin<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    assoc: &AssocInputs,
    features: Features,
    dig: &mut Dig,
    sink: &mut dyn FnMut(Event<'_>),
) -> Result<bool, AxError> {
    let band = assoc.band;
    let edca = assoc.params();
    // SAFETY: the caller's contract is `conf_tx`'s, word for word.
    unsafe { conf_tx(io, session, &edca, false, band, sink) }?;
    // Each H2C takes its seq right before its send, as
    // `rtw89_h2c_pkt_set_hdr` does: a failed send leaves the later ones
    // unnumbered.
    let packet = session.h2c.assoc_cmac_tbl(MAC_ID, PORT, band);
    // SAFETY: the caller's contract is `send`'s, word for word.
    unsafe { session.send(io, packet, sink) }?;
    let packet = session.h2c.join_info(MAC_ID, PORT, true);
    // SAFETY: as above.
    unsafe { session.send(io, packet, sink) }?;
    let cam = AddrCam::connected(assoc.mac, assoc.bssid, assoc.aid);
    let packet = session.h2c.addr_cam_with(&cam, MAC_ID, PORT);
    // SAFETY: as above.
    unsafe { session.send(io, packet, sink) }?;
    let packet = session.h2c.ra(MAC_ID, &ra_info(&assoc.peer, band));
    // SAFETY: as above.
    unsafe { session.send(io, packet, sink) }?;
    if features.beacon_filter {
        let packet = session.h2c.bcn_fltr(MAC_ID, true, features.loss_count_v1);
        // SAFETY: as above.
        unsafe { session.send(io, packet, sink) }?;
    }
    dig.resume(io, false)?;
    vif::port_function_on(io)
}

/// [`associate`] after [`associate_begin`], which found the port function
/// `on`, and its wait: the INFRA port update past its check
/// ([`vif::port_update_after`]), then, with the short slot, the EDCA
/// parameters again.
///
/// # Safety
/// As [`set_bssid`].
pub unsafe fn associate_end<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    assoc: &AssocInputs,
    on: bool,
    sink: &mut dyn FnMut(Event<'_>),
) -> Result<(), AxError> {
    vif::port_update_after(io, &PortConfig::infra(assoc.beacon_int, assoc.dtim), on)?;
    if assoc.short_slot() {
        // SAFETY: the caller's contract is `conf_tx`'s, word for word.
        unsafe { conf_tx(io, session, &assoc.params(), true, assoc.band, sink) }?;
    }
    Ok(())
}

/// Leaving the association `assoc` made (M3 spec S11; see the module
/// documentation), in Linux's order, after the keys were deleted
/// ([`super::keys::delete`], which mac80211 runs first,
/// `sta_info.c:1577-1578`): the beacon filter off when the firmware has
/// it; the beamformee's NDPA off on MAC 0; the CMAC table, join info
/// disconnected, the address CAM with NO_LINK; the address CAM with the
/// BSSID cleared, both CAMs with the key fields `keys` the deletes left
/// (`rtw89_cam_fill_addr_cam_info` reads the vif's entry, `cam.c:906-927`);
/// the association's EDCA parameters ([`AssocInputs::params`]) with the
/// long slot (`BSS_CHANGED_ERP_SLOT`, which `ieee80211_reset_erp_info`
/// always raises), the GID table cleared, then `wmm_defaults` with the
/// long slot. Nothing of the port or DIG.
///
/// # Safety
/// As [`set_bssid`].
pub unsafe fn leave<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    assoc: &AssocInputs,
    features: Features,
    keys: CamKeys,
    sink: &mut dyn FnMut(Event<'_>),
) -> Result<(), AxError> {
    let band = assoc.band;
    if features.beacon_filter {
        let packet = session.h2c.bcn_fltr(MAC_ID, false, features.loss_count_v1);
        // SAFETY: the caller's contract is `send`'s, word for word.
        unsafe { session.send(io, packet, sink) }?;
    }
    // SAFETY: as above, `disconnect`'s.
    unsafe { disconnect(io, session, assoc.mac, assoc.bssid, band, keys, sink) }?;
    // SAFETY: as above, `clear_bssid`'s.
    unsafe { clear_bssid(io, session, assoc.mac, keys, sink) }?;
    // SAFETY: as above, `conf_tx`'s.
    unsafe { conf_tx(io, session, &assoc.params(), false, band, sink) }?;
    // The membership and position tables, zeroed before the notify
    // (`mlme.c:4652-4655`), as plain writes.
    for reg in GID_TABLE {
        io.write32(reg, 0)?;
    }
    let defaults = wmm_defaults(assoc.use_11b());
    // SAFETY: as above, `conf_tx`'s.
    unsafe { conf_tx(io, session, &defaults, false, band, sink) }?;
    Ok(())
}

/// The AP's station at `bssid` moved from AUTH to NONE,
/// `rtw89_core_sta_link_disconnect` (`core.c:5818-5859`), on `band`: the
/// beamformee's NDPA off on MAC 0 (`rtw89_mac_bf_disassoc`), the CMAC
/// table, join info disconnected, and the address CAM of `mac` with the
/// BSSID, NO_LINK and the key fields `keys`. [`leave`] runs it after the
/// beacon filter; a join whose station authenticated and never associated
/// runs it alone, before [`clear_bssid`], as `ieee80211_destroy_assoc_data`
/// destroys that station before clearing the BSSID.
///
/// # Safety
/// As [`set_bssid`].
pub unsafe fn disconnect<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    mac: [u8; 6],
    bssid: [u8; 6],
    band: HwBand,
    keys: CamKeys,
    sink: &mut dyn FnMut(Event<'_>),
) -> Result<(), AxError> {
    // `rtw89_mac_bf_disassoc`: `rtw89_mac_bfee_ctrl(false)`.
    clear32(io, R_AX_BFMEE_RESP_OPTION, B_AX_BFMEE_NDPA_EN)?;
    let packet = session.h2c.assoc_cmac_tbl(MAC_ID, PORT, band);
    // SAFETY: the caller's contract is `send`'s, word for word.
    unsafe { session.send(io, packet, sink) }?;
    let packet = session.h2c.join_info(MAC_ID, PORT, false);
    // SAFETY: as above.
    unsafe { session.send(io, packet, sink) }?;
    let cam = AddrCam {
        keys,
        ..AddrCam::with_bssid(mac, bssid)
    };
    let packet = session.h2c.addr_cam_with(&cam, MAC_ID, PORT);
    // SAFETY: as above.
    unsafe { session.send(io, packet, sink) }.map(|_| ())
}

/// The BSSID cleared on a station that is not associated: the address CAM
/// of [`AddrCam::unassociated`] with the key fields `keys`, which without a
/// key ever installed are vif init's bytes.
///
/// # Safety
/// As [`set_bssid`].
pub unsafe fn clear_bssid<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    mac: [u8; 6],
    keys: CamKeys,
    sink: &mut dyn FnMut(Event<'_>),
) -> Result<(), AxError> {
    let cam = AddrCam {
        keys,
        ..AddrCam::unassociated(mac)
    };
    let packet = session.h2c.addr_cam_with(&cam, MAC_ID, PORT);
    // SAFETY: the caller's contract is `send`'s, word for word.
    unsafe { session.send(io, packet, sink) }.map(|_| ())
}

#[cfg(test)]
mod tests;

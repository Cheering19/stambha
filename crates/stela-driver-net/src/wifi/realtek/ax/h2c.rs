// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! H2C packet builder: `rtw89_h2c_pkt_set_hdr` (`fw.c:1632`) with its AX
//! REC_ACK-every-fourth-sequence rule (`fw.c:1641`), the FWCMD TX descriptor
//! of `rtw89_core_fill_txdesc_fwcmd_v1` (`core.c:1909`, reused from the
//! firmware download), `rtw89_fw_h2c_set_ofld_cfg` (`fw.c:5320`),
//! `rtw89_fw_h2c_rf_reg` (`fw.c:8090-8119`), and the encoders the scan
//! needs: `rtw89_fw_h2c_fw_log` (`fw.c:2827-2862`),
//! `rtw89_fw_h2c_init_ba_cam_v0_ext` (`fw.c:2654-2692`),
//! `rtw89_fw_h2c_macid_pause` (`fw.c:5180-5235`),
//! `rtw89_fw_h2c_role_maintain` (`fw.c:4949-5003`),
//! `rtw89_fw_h2c_join_info` (`fw.c:5045-5143`, AX format),
//! `rtw89_fw_h2c_cam` (`fw.c:2290-2335`, v0) with
//! `rtw89_cam_fill_addr_cam_info` and `rtw89_cam_fill_bssid_cam_info`
//! (`cam.c:762-927`), `rtw89_fw_h2c_default_cmac_tbl` (`fw.c:3612-3661`),
//! `rtw89_fw_h2c_scan_list_offload_ax` (`fw.c:7418-7488`) and
//! `rtw89_fw_h2c_scan_offload_ax` (`fw.c:7609-7684`).
//!
//! Each of those is fixed to the one unassociated station interface of
//! milestone 2c's spec: wifi role STATION, self role CLIENT, net type
//! NO_LINK, MAC 0, PHY 0, zero BSSID, address CAM 0 and BSSID CAM 0, no
//! extra-op, no delay and no target channel. It only builds bytes: it submits
//! nothing, waits for no acknowledgement and grants no register access.
use super::chan::HwBand;
use super::elements::RfPath;
use super::fwdl::{RX_TYPE_H2C, descriptor};
use super::io::AxError;
use alloc::vec::Vec;

/// `H2C_HDR_*` (`fw.h:4673-4683`).
const H2C_HEADER_LEN: usize = 8;
const H2C_HDR_REC_ACK: u32 = 1 << 14;
const H2C_HDR_DONE_ACK: u32 = 1 << 15;
const H2C_HDR_TOTAL_LEN: u32 = 0x3FFF;
pub(super) const H2C_CAT_MAC: u8 = 1;
pub(super) const H2C_CL_MAC_FW_OFLD: u8 = 9;
pub(super) const H2C_FUNC_OFLD_CFG: u8 = 0x14;
/// `H2C_CL_FW_INFO` / `H2C_FUNC_LOG_CFG` (`fw.h:4694-4695`).
pub(super) const H2C_CL_FW_INFO: u8 = 0;
pub(super) const H2C_FUNC_LOG_CFG: u8 = 0;
/// `H2C_CL_MAC_FR_EXCHG` and 8852C's `h2c_cctl_func_id`,
/// `H2C_FUNC_MAC_CCTLINFO_UD_V1` (`fw.h:4744-4748`, `rtw8852c.c:3260`).
pub(super) const H2C_CL_MAC_FR_EXCHG: u8 = 5;
pub(super) const H2C_FUNC_MAC_CCTLINFO_UD_V1: u8 = 0xA;
/// `H2C_CL_MAC_ADDR_CAM_UPDATE` / `H2C_FUNC_MAC_ADDR_CAM_UPD`
/// (`fw.h:4755-4756`).
pub(super) const H2C_CL_MAC_ADDR_CAM_UPDATE: u8 = 6;
pub(super) const H2C_FUNC_MAC_ADDR_CAM_UPD: u8 = 0;
/// `H2C_CL_MAC_MEDIA_RPT` and its JOININFO and FWROLE_MAINTAIN funcs
/// (`fw.h:4759-4761`).
pub(super) const H2C_CL_MAC_MEDIA_RPT: u8 = 8;
pub(super) const H2C_FUNC_MAC_JOININFO: u8 = 0;
pub(super) const H2C_FUNC_MAC_FWROLE_MAINTAIN: u8 = 4;
/// The FW_OFLD funcs of `enum rtw89_fw_ofld_h2c_func` the scan sends
/// (`fw.h:4768-4774`).
pub(super) const H2C_FUNC_MAC_MACID_PAUSE: u8 = 0x8;
pub(super) const H2C_FUNC_ADD_SCANOFLD_CH: u8 = 0x16;
pub(super) const H2C_FUNC_SCANOFLD: u8 = 0x17;
/// `H2C_CL_BA_CAM` / `H2C_FUNC_MAC_BA_CAM` (`fw.h:4808-4809`).
pub(super) const H2C_CL_BA_CAM: u8 = 0xC;
pub(super) const H2C_FUNC_MAC_BA_CAM: u8 = 0;
/// `H2C_CAT_OUTSRC` (`fw.h:4877`).
pub const H2C_CAT_OUTSRC: u8 = 2;
/// `H2C_CL_OUTSRC_RF_REG_A` and `_B` (`fw.h:4890-4891`).
pub const H2C_CL_OUTSRC_RF_REG_A: u8 = 8;
pub const H2C_CL_OUTSRC_RF_REG_B: u8 = 9;

/// `fw.c:5321`: the OFLD_CFG payload.
pub const OFLD_CFG: [u8; 8] = [0x09, 0, 0, 0, 0x5E, 0, 0, 0];

/// `RTW89_SCAN_LIST_LIMIT_AX` (`fw.h:345-349`): `2048 / 28 - 4` channels in
/// one ADD_SCANOFLD_CH.
pub const SCAN_LIST_LIMIT: usize = 69;

/// `enum rtw89_wifi_role` STATION, `RTW89_SELF_ROLE_CLIENT` and
/// `RTW89_NET_TYPE_NO_LINK` (`core.h:250-283`): the one vif M2c creates.
const WIFI_ROLE_STATION: u32 = 1;
const SELF_ROLE_CLIENT: u32 = 0;
const NET_TYPE_NO_LINK: u32 = 0;

/// `FIELD_PREP(GENMASK(hi, lo), v)` as `le32_encode_bits` does it: `v` is
/// masked to the field's width before it is shifted in.
const fn field(v: u32, hi: u32, lo: u32) -> u32 {
    let width = hi - lo + 1;
    let mask = if width == 32 {
        u32::MAX
    } else {
        (1 << width) - 1
    };
    (v & mask) << lo
}

fn le_words(words: &[u32]) -> Vec<u8> {
    words.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// One entry of the scan offload channel list: the fields of
/// `struct rtw89_mac_chinfo_ax` that `rtw89_hw_scan_prep_chan_list_ax` and
/// `rtw89_hw_scan_add_chan_ax` (`fw.c:9628-9724`, `:9913-9960`) vary for a
/// 20 MHz scan with no SSIDs and no operating channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScanChannel {
    /// `ch_band`.
    pub band: HwBand,
    /// `central_ch` and `pri_ch` (`hw_value`).
    pub channel: u8,
    /// `period`.
    pub period: u8,
    /// `dwell_time`.
    pub dwell: u8,
    /// `dfs_ch`: the channel carries `RADAR` or `NO_IR` (`fw.c:9955-9959`).
    pub dfs: bool,
}

impl ScanChannel {
    /// `struct rtw89_h2c_chinfo_elem` (`fw.c:7444-7473`, bits
    /// `fw.h:2945-2975`).
    ///
    /// - w0: period [7:0], dwell [15:8], center channel [23:16], primary
    ///   channel [31:24].
    /// - w1: bw 0 (`RTW89_SCAN_WIDTH`) [2:0], notify action 0x1F
    ///   (`RTW89_SCANOFLD_DEBUG_MASK`) [7:3], num_pkt 0 [11:8] (no SSIDs),
    ///   tx b12 = 1, pause_data b13 = 1 (both the DFS and the ACTIVE branch
    ///   set it, `fw.c:9699-9707`), band [15:14], probe id 0xFF
    ///   (`RTW89_SCANOFLD_PKT_NONE`) [23:16], dfs b24, tx_null b25 = 0
    ///   (not an operating channel), random b26 = 0 (no
    ///   `NL80211_SCAN_FLAG_RANDOM_SN`); MACID_TX is written only with an
    ///   extra op.
    /// - w2, w3: packet ids 0 (`kzalloc`ed, no SSIDs); w4–w6: not written.
    pub fn words(&self) -> [u32; 7] {
        let ch = u32::from(self.channel);
        let w0 = field(u32::from(self.period), 7, 0)
            | field(u32::from(self.dwell), 15, 8)
            | field(ch, 23, 16)
            | field(ch, 31, 24);
        let w1 = field(0, 2, 0)
            | field(0x1F, 7, 3)
            | field(0, 11, 8)
            | field(1, 12, 12)
            | field(1, 13, 13)
            | field(self.band as u32, 15, 14)
            | field(0xFF, 23, 16)
            | field(u32::from(self.dfs), 24, 24);
        [w0, w1, 0, 0, 0, 0, 0]
    }
}

/// `rtwdev->fw.h2c_seq`, zeroed after a download that reached the sections
/// (`fw.c:2021`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct H2c {
    seq: u8,
}

impl H2c {
    /// The state after a download: the next H2C carries seq 0.
    pub fn after_download() -> Self {
        Self { seq: 0 }
    }

    /// The state whose next H2C carries `seq`.
    #[cfg(test)]
    pub fn at(seq: u8) -> Self {
        Self { seq }
    }

    /// One CH12 packet with DONE_ACK as `dack` asks and no REC_ACK beyond
    /// the every-fourth-sequence rule: `packet_with(.., false, dack, ..)`.
    pub fn packet(
        &mut self,
        cat: u8,
        class: u8,
        func: u8,
        dack: bool,
        payload: &[u8],
    ) -> (u8, Vec<u8>) {
        self.packet_with(cat, class, func, false, dack, payload)
    }

    /// One CH12 packet: the 16-byte descriptor, the 8-byte FWCMD header and
    /// `payload`. Returns the sequence number it carries. REC_ACK is set
    /// when `rack` asks for it or `seq % 4 == 0`; the sequence then
    /// advances as a u8. The header's DEL_TYPE is `FWCMD_TYPE_H2C` (0).
    pub fn packet_with(
        &mut self,
        cat: u8,
        class: u8,
        func: u8,
        rack: bool,
        dack: bool,
        payload: &[u8],
    ) -> (u8, Vec<u8>) {
        let seq = self.seq;
        let rack = rack || seq.is_multiple_of(4);
        let hdr0 = u32::from(cat & 0x3)
            | u32::from(class & 0x3F) << 2
            | u32::from(func) << 8
            | u32::from(seq) << 24;
        let total = (payload.len() + H2C_HEADER_LEN) as u32;
        let hdr1 = (total & H2C_HDR_TOTAL_LEN)
            | if rack { H2C_HDR_REC_ACK } else { 0 }
            | if dack { H2C_HDR_DONE_ACK } else { 0 };
        let mut p = Vec::with_capacity(16 + H2C_HEADER_LEN + payload.len());
        p.extend_from_slice(&descriptor(H2C_HEADER_LEN + payload.len(), RX_TYPE_H2C));
        p.extend_from_slice(&hdr0.to_le_bytes());
        p.extend_from_slice(&hdr1.to_le_bytes());
        p.extend_from_slice(payload);
        self.seq = self.seq.wrapping_add(1);
        (seq, p)
    }

    /// `rtw89_fw_h2c_set_ofld_cfg`: MAC / FW_OFLD / OFLD_CFG, rack 0, dack 1.
    pub fn ofld_cfg(&mut self) -> (u8, Vec<u8>) {
        self.packet(
            H2C_CAT_MAC,
            H2C_CL_MAC_FW_OFLD,
            H2C_FUNC_OFLD_CFG,
            true,
            &OFLD_CFG,
        )
    }

    /// `rtw89_fw_h2c_rf_reg`: one page of stored RF register words for
    /// `path`, as OUTSRC / RF_REG_A (path A) or RF_REG_B (any other path,
    /// as Linux's ternary has it) with func `page`, rack 0 and dack 0. The
    /// payload is `words` as little-endian u32s; only the every-fourth
    /// sequence rule can ask for a REC_ACK.
    pub fn rf_reg(&mut self, path: RfPath, page: u8, words: &[u32]) -> (u8, Vec<u8>) {
        let class = if path == RfPath::A {
            H2C_CL_OUTSRC_RF_REG_A
        } else {
            H2C_CL_OUTSRC_RF_REG_B
        };
        let payload = le_words(words);
        self.packet(H2C_CAT_OUTSRC, class, page, false, &payload)
    }

    /// `rtw89_fw_h2c_fw_log(false)`: MAC / FW_INFO / LOG_CFG, rack 0,
    /// dack 0, 12 bytes. w0 = level `RTW89_FW_LOG_LEVEL_LOUD` (4) [7:0] |
    /// path `BIT(RTW89_FW_LOG_LEVEL_C2H)` (2) [15:8]; w1 = comp 0 (logging
    /// is disabled); w2 = comp_ext 0 (`fw.h:1900-1918`).
    pub fn fw_log(&mut self) -> (u8, Vec<u8>) {
        let w0 = field(4, 7, 0) | field(1 << 1, 15, 8);
        self.packet(
            H2C_CAT_MAC,
            H2C_CL_FW_INFO,
            H2C_FUNC_LOG_CFG,
            false,
            &le_words(&[w0, 0, 0]),
        )
    }

    /// `rtw89_fw_h2c_init_ba_cam_v0_ext(entry, uid)`: MAC / BA_CAM /
    /// BA_CAM, rack 0, dack 1, 8 bytes. w0 = VALID b0; w1 = uid [7:0],
    /// std_en b8 = 0, band b9 = 0, entry [31:28] (`fw.h:1925-1935`).
    pub fn ba_cam_init(&mut self, entry: u8, uid: u8) -> (u8, Vec<u8>) {
        let w0 = field(1, 0, 0);
        let w1 = field(u32::from(entry), 31, 28) | field(u32::from(uid), 7, 0);
        self.packet(
            H2C_CAT_MAC,
            H2C_CL_BA_CAM,
            H2C_FUNC_MAC_BA_CAM,
            true,
            &le_words(&[w0, w1]),
        )
    }

    /// `rtw89_mac_set_macid_pause`'s H2C, `rtw89_fw_h2c_macid_pause(sh,
    /// grp, pause)` without `MACID_PAUSE_SLEEP`: MAC / FW_OFLD /
    /// MACID_PAUSE, rack 1, dack 0, `struct rtw89_fw_macid_pause_grp`
    /// (32 bytes, `fw.h:315-318`). `sh = macid[4:0]`, `grp = macid >> 5`
    /// (`mac.c:4439-4440`): `mask_grp[grp] = BIT(sh)`, and `pause_grp[grp]`
    /// too when pausing. A macid of 128 or more, past
    /// `RTW89_MAX_MAC_ID_NUM`, would index past Linux's four groups; it
    /// gives an all-zero payload here instead.
    pub fn macid_pause(&mut self, macid: u8, pause: bool) -> (u8, Vec<u8>) {
        let sh = macid & 0x1F;
        let grp = usize::from(macid >> 5);
        let mut words = [0u32; 8];
        if grp < 4 {
            words[4 + grp] = 1 << sh;
            if pause {
                words[grp] = 1 << sh;
            }
        }
        self.packet_with(
            H2C_CAT_MAC,
            H2C_CL_MAC_FW_OFLD,
            H2C_FUNC_MAC_MACID_PAUSE,
            true,
            false,
            &le_words(&words),
        )
    }

    /// `rtw89_fw_h2c_role_maintain(vif, NULL, upd_mode)` for M2c's station
    /// vif: MAC / MEDIA_RPT / FWROLE_MAINTAIN, rack 0, dack 1, 4 bytes.
    /// w0 = macid [7:0] | self_role CLIENT 0 [9:8] | upd_mode [12:10] |
    /// wifi_role STATION 1 [16:13] | band (`mac_idx` 0) [18:17] | port
    /// [21:19] (`fw.h:1813-1819`).
    pub fn role_maintain(&mut self, macid: u8, port: u8, upd_mode: u8) -> (u8, Vec<u8>) {
        let w0 = field(u32::from(macid), 7, 0)
            | field(SELF_ROLE_CLIENT, 9, 8)
            | field(u32::from(upd_mode), 12, 10)
            | field(WIFI_ROLE_STATION, 16, 13)
            | field(0, 18, 17)
            | field(u32::from(port), 21, 19);
        self.packet(
            H2C_CAT_MAC,
            H2C_CL_MAC_MEDIA_RPT,
            H2C_FUNC_MAC_FWROLE_MAINTAIN,
            true,
            &le_words(&[w0]),
        )
    }

    /// `rtw89_fw_h2c_join_info(vif, NULL, dis_conn = true)` for M2c's
    /// station vif in the AX format (`struct rtw89_h2c_join`, 4 bytes):
    /// MAC / MEDIA_RPT / JOININFO, rack 0, dack 1. w0 = macid [7:0] | OP
    /// b8 = 1 | band b9 = 0 | wmm 0 [11:10] | trigger b12 = 0 | the HE and
    /// DL fields 0 | port [23:21] | net_type NO_LINK 0 [25:24] | wifi_role
    /// STATION 1 [29:26] | self_role CLIENT 0 [31:30] (`fw.h:1837-1849`).
    pub fn join_info_disconnect(&mut self, macid: u8, port: u8) -> (u8, Vec<u8>) {
        let w0 = field(u32::from(macid), 7, 0)
            | field(1, 8, 8)
            | field(u32::from(port), 23, 21)
            | field(NET_TYPE_NO_LINK, 25, 24)
            | field(WIFI_ROLE_STATION, 29, 26)
            | field(SELF_ROLE_CLIENT, 31, 30);
        self.packet(
            H2C_CAT_MAC,
            H2C_CL_MAC_MEDIA_RPT,
            H2C_FUNC_MAC_JOININFO,
            true,
            &le_words(&[w0]),
        )
    }

    /// `rtw89_fw_h2c_cam(vif, NULL, NULL, upd_mode)` in the v0 layout AX
    /// always uses (`struct rtw89_h2c_addr_cam_v0`, 60 bytes,
    /// `fw.c:2302-2305`; bits `cam.h:38-116`): MAC / ADDR_CAM_UPDATE /
    /// ADDR_CAM_UPD, rack 0, dack 1. v0 carries no `upd_mode`, so CREATE
    /// and INFO_CHANGE build the same bytes.
    ///
    /// The vif's address CAM 0 (offset 0, len `ADDR_CAM_ENT_SIZE` 0x20,
    /// valid, no address mask, `sec_ent_mode` NORMAL 2) and BSSID CAM 0
    /// (len `BSSID_CAM_ENT_SIZE` 8, valid, PHY 0) are those
    /// `rtw89_cam_init` picks for the first vif (`cam.c:631-760`); SMA is
    /// `mac`, TMA the zero BSSID, BSS color 0, and the BSSID mask
    /// `RTW89_BSSID_MATCH_ALL` (0x3F). No security entry, AID or WoWLAN
    /// bit is set for a NO_LINK vif.
    pub fn addr_cam(&mut self, mac: [u8; 6], macid: u8, port: u8) -> (u8, Vec<u8>) {
        let m = mac.map(u32::from);
        let sma_hash = mac.iter().fold(0u8, |h, b| h ^ b);
        let tma_hash = 0u32;
        let port = u32::from(port);
        let mut w = [0u32; 15];
        w[1] = field(0, 7, 0) | field(0, 15, 8) | field(0x20, 23, 16);
        w[2] = field(1, 0, 0)
            | field(NET_TYPE_NO_LINK, 2, 1)
            | field(u32::from(sma_hash), 23, 16)
            | field(tma_hash, 31, 24);
        w[3] = field(0, 5, 0);
        w[4] = field(m[0], 7, 0) | field(m[1], 15, 8) | field(m[2], 23, 16) | field(m[3], 31, 24);
        w[5] = field(m[4], 7, 0) | field(m[5], 15, 8);
        w[8] = field(u32::from(macid), 7, 0) | field(port, 10, 8) | field(port, 13, 11);
        w[9] = field(2, 17, 16);
        w[12] = field(0, 7, 0) | field(0, 15, 8) | field(8, 23, 16);
        w[13] = field(1, 0, 0) | field(0x3F, 7, 2);
        self.packet(
            H2C_CAT_MAC,
            H2C_CL_MAC_ADDR_CAM_UPDATE,
            H2C_FUNC_MAC_ADDR_CAM_UPD,
            true,
            &le_words(&w),
        )
    }

    /// `rtw89_fw_h2c_default_cmac_tbl(vif, NULL)` for 8852C, whose
    /// `h2c_cctl_func_id` is CCTLINFO_UD_V1, so the TX power mode, TX path
    /// and antenna select words are not written: MAC / FR_EXCHG /
    /// CCTLINFO_UD_V1, rack 0, dack 1, 68 bytes (`H2C_CMC_TBL_LEN`).
    /// w0 = macid [6:0] | operation b7; MGQ_RPT_EN w1 b21 =
    /// `tx_rpt_enabled` (false on PCIe; set only by `usb.c:1228`) with its
    /// mask w9 b21; DOPPLER_CTRL w7 [19:18] and TXPWR_TOLERENCE w7 [27:24]
    /// = 0 with their masks w15 [19:18] and [27:24] (`fw.h:734-741`,
    /// `:792-797`, `:1277-1303`). `DATA_DCM` is AP-only.
    pub fn default_cmac_tbl(&mut self, macid: u8) -> (u8, Vec<u8>) {
        let mut w = [0u32; 17];
        w[0] = field(u32::from(macid), 6, 0) | field(1, 7, 7);
        w[9] = field(1, 21, 21);
        w[15] = field(0x3, 19, 18) | field(0xF, 27, 24);
        self.packet(
            H2C_CAT_MAC,
            H2C_CL_MAC_FR_EXCHG,
            H2C_FUNC_MAC_CCTLINFO_UD_V1,
            true,
            &le_words(&w),
        )
    }

    /// `rtw89_fw_h2c_scan_list_offload_ax(list.len(), list)`: MAC /
    /// FW_OFLD / ADD_SCANOFLD_CH, rack 1, dack 1. The payload is
    /// `struct rtw89_h2c_chinfo`: `ch_num`, `elem_size` 7 (28 bytes in
    /// words), `arg` 0, 0, then each entry's [`ScanChannel::words`].
    ///
    /// More than [`SCAN_LIST_LIMIT`] entries do not fit one H2C (Linux
    /// splits its list there, `fw.c:10020-10037`); they are refused with
    /// `Scan` before a sequence number is taken.
    pub fn add_scanofld_ch(&mut self, list: &[ScanChannel]) -> Result<(u8, Vec<u8>), AxError> {
        if list.len() > SCAN_LIST_LIMIT {
            return Err(AxError::Scan("scan list past 69 channels"));
        }
        let mut payload = Vec::with_capacity(4 + 28 * list.len());
        payload.extend_from_slice(&[list.len() as u8, 7, 0, 0]);
        for c in list {
            payload.extend_from_slice(&le_words(&c.words()));
        }
        Ok(self.packet_with(
            H2C_CAT_MAC,
            H2C_CL_MAC_FW_OFLD,
            H2C_FUNC_ADD_SCANOFLD_CH,
            true,
            true,
            &payload,
        ))
    }

    /// `rtw89_fw_h2c_scan_offload_ax(opt, vif, false)` with `opt` as
    /// `rtw89_hw_scan_offload` fills it unconnected (`fw.c:10535-10547`:
    /// `target_ch_mode` 0, `delay` 0, `repeat`, `norm_pd` and `slow_pd` 0):
    /// MAC / FW_OFLD / SCANOFLD, rack 1, dack 1, 28 bytes
    /// (`struct rtw89_h2c_scanofld`, bits `fw.h:3037-3065`).
    /// w0 = macid [7:0] | port [18:16] | band (`mac_idx` 0) b19 | operation
    /// `enable` [21:20]; w1 = notify_end b0 = 1, start mode IMMEDIATE 0;
    /// w2, the TSF words, w5 and w6 (no extra op) = 0.
    pub fn scanofld(&mut self, macid: u8, port: u8, enable: bool) -> (u8, Vec<u8>) {
        let w0 = field(u32::from(macid), 7, 0)
            | field(u32::from(port), 18, 16)
            | field(0, 19, 19)
            | field(u32::from(enable), 21, 20);
        let w1 = field(1, 0, 0) | field(0, 1, 1) | field(0, 2, 2) | field(0, 4, 3);
        self.packet_with(
            H2C_CAT_MAC,
            H2C_CL_MAC_FW_OFLD,
            H2C_FUNC_SCANOFLD,
            true,
            true,
            &le_words(&[w0, w1, 0, 0, 0, 0, 0]),
        )
    }
}

#[cfg(test)]
mod tests;

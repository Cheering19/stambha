// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! H2C packet builder: `rtw89_h2c_pkt_set_hdr` (`fw.c:1632`) with its AX
//! REC_ACK-every-fourth-sequence rule (`fw.c:1641`), the FWCMD TX descriptor
//! of `rtw89_core_fill_txdesc_fwcmd_v1` (`core.c:1909`, reused from the
//! firmware download), `rtw89_fw_h2c_set_ofld_cfg` (`fw.c:5320`) and
//! `rtw89_fw_h2c_rf_reg` (`fw.c:8090-8119`). It only builds bytes: it
//! submits nothing and grants no register access.
use super::elements::RfPath;
use super::fwdl::{RX_TYPE_H2C, descriptor};
use alloc::vec::Vec;

/// `H2C_HDR_*` (`fw.h:4673-4683`).
const H2C_HEADER_LEN: usize = 8;
const H2C_HDR_REC_ACK: u32 = 1 << 14;
const H2C_HDR_DONE_ACK: u32 = 1 << 15;
const H2C_HDR_TOTAL_LEN: u32 = 0x3FFF;
pub(super) const H2C_CAT_MAC: u8 = 1;
pub(super) const H2C_CL_MAC_FW_OFLD: u8 = 9;
pub(super) const H2C_FUNC_OFLD_CFG: u8 = 0x14;
/// `H2C_CAT_OUTSRC` (`fw.h:4877`).
pub const H2C_CAT_OUTSRC: u8 = 2;
/// `H2C_CL_OUTSRC_RF_REG_A` and `_B` (`fw.h:4890-4891`).
pub const H2C_CL_OUTSRC_RF_REG_A: u8 = 8;
pub const H2C_CL_OUTSRC_RF_REG_B: u8 = 9;

/// `fw.c:5321`: the OFLD_CFG payload.
pub const OFLD_CFG: [u8; 8] = [0x09, 0, 0, 0, 0x5E, 0, 0, 0];

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

    /// One CH12 packet: the 16-byte descriptor, the 8-byte FWCMD header and
    /// `payload`. Returns the sequence number it carries. REC_ACK is forced
    /// when `seq % 4 == 0`; the sequence then advances as a u8.
    pub fn packet(
        &mut self,
        cat: u8,
        class: u8,
        func: u8,
        dack: bool,
        payload: &[u8],
    ) -> (u8, Vec<u8>) {
        let seq = self.seq;
        let rack = seq.is_multiple_of(4);
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
        let payload: Vec<u8> = words.iter().flat_map(|w| w.to_le_bytes()).collect();
        self.packet(H2C_CAT_OUTSRC, class, page, false, &payload)
    }
}

#[cfg(test)]
mod tests;

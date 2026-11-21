// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! C2H parser: the header of `rtw89_fw_c2h_parse_attr` (`fw.c:9043`,
//! `fw.h:3926`), the class 0 REC_ACK, DONE_ACK and log handlers
//! (`mac.c:5459`, `:5495`, `:5559`) and the formatted-log test of
//! `rtw89_fw_log_dump` (`fw.c:2249`), the class 1 SCANOFLD_RSP fields
//! `rtw89_mac_c2h_scanofld_rsp` reads (`mac.c:5283`, `fw.h:4093`), and the
//! class 1 BCNFLTR_RPT fields `rtw89_mac_bcn_fltr_rpt` reads (`mac.c:5390-5405`,
//! `fw.h:4005-4014`), and the fields of the OUTSRC (2) RA (1) STS_RPT (0)
//! report that `__rtw89_phy_c2h_ra_rpt_iter` reads on AX (`phy.c:3247-3254`,
//! `fw.h:4016-4030`, `phy.h:160-162`, `:201-205`), with [`ra_rate`] its
//! `txrate` (`phy.c:3274-3330`). Formatted logs are not rendered (the
//! firmware's format table is not ported), and nothing is acted on here: a
//! beacon filter report is decoded raw, and the live chip matches it to its
//! vif and turns a loss into a beacon loss (`Live::service`); an RA report
//! is decoded raw, and the live chip hands its own MAC ID's on. It grants no
//! I/O.
use super::rx::{Rate, RateMode};

const C2H_HEADER_LEN: usize = 8;
const ACK_LEN: usize = 12;
const C2H_CAT_MAC: u8 = 1;
const C2H_CLASS_INFO: u8 = 0;
const FUNC_REC_ACK: u8 = 0;
const FUNC_DONE_ACK: u8 = 1;
const FUNC_LOG: u8 = 2;
const C2H_CLASS_OFLD: u8 = 1;
const FUNC_SCANOFLD_RSP: u8 = 9;
/// `RTW89_MAC_C2H_FUNC_BCNFLTR_RPT` (`mac.h:415`).
const FUNC_BCNFLTR_RPT: u8 = 0xD;
/// Bytes through `w5`, the last word read.
const SCANOFLD_LEN: usize = 24;
/// `struct rtw89_c2h_mac_bcnfltr_rpt`: w0, w1 and w2 (`fw.h:4005-4009`).
const BCNFLTR_LEN: usize = 12;
/// `RTW89_BCN_FLTR_BEACON_LOSS`, the first `enum rtw89_bcn_fltr_type`
/// (`fw.h:273-277`); RSSI is 1 and NOTIFY 2.
pub const BCNFLTR_BEACON_LOSS: u8 = 0;
/// `enum rtw89_scanofld_notify_reason` and `rtw89_scanofld_status`
/// (`fw.h:228-245`).
pub const SCAN_ENTER_CH: u8 = 3;
pub const SCAN_LEAVE_CH: u8 = 4;
pub const SCAN_END: u8 = 5;
pub const SCAN_ENTER_OP: u8 = 8;
pub const SCAN_LEAVE_OP: u8 = 9;
pub const SCAN_STATUS_SUCCESS: u8 = 1;
/// `RTW89_C2H_FW_LOG_SIGNATURE` and `RTW89_C2H_FW_FORMATTED_LOG_MIN_LEN`
/// (`fw.h:3993`, `:3990`).
const LOG_SIGNATURE: [u8; 2] = [0xA5, 0xA5];
const FORMATTED_LOG_MIN_LEN: usize = 11;

/// The fields of `struct rtw89_c2h_scanofld` (`fw.h:4093-4118`) that
/// `rtw89_mac_c2h_scanofld_rsp` reads on AX.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScanOfld {
    pub pri_ch: u8,
    pub reason: u8,
    pub status: u8,
    pub period: u8,
    pub tx_fail: u8,
    pub air_density: u8,
    pub band: u8,
    pub mac_idx: bool,
}

/// The w2 fields of `struct rtw89_c2h_mac_bcnfltr_rpt` (`fw.h:4011-4014`),
/// raw: `macid` `[7:0]`, `kind` (`TYPE`, `[9:8]`: [`BCNFLTR_BEACON_LOSS`],
/// RSSI 1 or NOTIFY 2), `event` `[11:10]` and `ma` `[23:16]`, the moving average
/// whose signal Linux takes as `ma - MAX_RSSI` (`mac.c:5403`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BcnFltr {
    pub macid: u8,
    pub kind: u8,
    pub event: u8,
    pub ma: u8,
}

/// `RTW89_C2H_CAT_OUTSRC`, `RTW89_PHY_C2H_CLASS_RA` and
/// `RTW89_PHY_C2H_FUNC_STS_RPT` (`fw.h:174`, `phy.h:201-205`, `:160-162`).
const C2H_CAT_OUTSRC: u8 = 2;
const C2H_CLASS_RA: u8 = 1;
const FUNC_STS_RPT: u8 = 0;
/// `struct rtw89_c2h_ra_rpt`: the header, w2 and w3 (`fw.h:4016-4020`).
const RA_RPT_LEN: usize = 16;

/// The fields of `struct rtw89_c2h_ra_rpt` (`fw.h:4022-4030`) that
/// `__rtw89_phy_c2h_ra_rpt_iter` reads on AX, raw: `macid` (w2 `[15:0]`), and
/// of w3 `rate` (MCSNSS `[6:0]`), `mode` (MD_SEL `[9:8]`,
/// `enum rtw89_ra_report_mode`, `core.h:4449-4455`), `giltf` (`[12:10]`) and
/// `bw` (`[14:13]`). RETRY_RATIO, MCSNSS_B7, MD_SEL_B2 and BW_B2 are BE's
/// (`format_v1`, `phy.c:3256-3264`) and are not read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RaReport {
    pub macid: u16,
    pub mode: u8,
    pub rate: u8,
    pub giltf: u8,
    pub bw: u8,
}

impl RaReport {
    /// Whether `__rtw89_phy_c2h_ra_rpt_iter` keeps it: a legacy rate must be
    /// one of `rtw89_bitrates`' twelve (`rtw89_legacy_rate_to_bitrate`,
    /// `core.c:277-291`; `phy.c:3266-3270`).
    pub fn valid(&self) -> bool {
        self.mode != 0 || self.rate < 12
    }
}

/// `report`'s `txrate` as `__rtw89_phy_c2h_ra_rpt_iter` fills it on AX
/// (`phy.c:3274-3330`): a legacy rate's index; HT's MCS
/// `RTW89_RA_RATE_MASK_HT_MCS` `[4:0]` (8852C's firmware has no
/// `OLD_HT_RA_FORMAT`, `fw.c:865`), its streams from it; VHT's and HE's
/// streams `RTW89_RA_RATE_MASK_NSS` `[6:4]` + 1 and MCS `[3:0]`
/// (`fw.h:4078-4082`). HT and VHT take any GILTF but 0 as short GI; HE's
/// 0.8 us GI is `RTW89_GILTF_2XHE08` or `_1XHE08` only (`phy.c:3310-3315`).
pub fn ra_rate(report: &RaReport) -> Rate {
    let rate = report.rate;
    let (mode, nss, mcs, sgi) = match report.mode {
        0 => (RateMode::Legacy, 1, rate, false),
        1 => {
            let mcs = rate & 0x1F;
            (RateMode::Ht, mcs / 8 + 1, mcs, report.giltf != 0)
        }
        2 => (
            RateMode::Vht,
            ((rate >> 4) & 0x7) + 1,
            rate & 0xF,
            report.giltf != 0,
        ),
        _ => (
            RateMode::He,
            ((rate >> 4) & 0x7) + 1,
            rate & 0xF,
            matches!(report.giltf, 3 | 5),
        ),
    };
    Rate {
        mode,
        nss,
        mcs,
        bw: report.bw,
        sgi,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum C2h<'a> {
    RecAck {
        cat: u8,
        class: u8,
        func: u8,
        seq: u8,
    },
    DoneAck {
        cat: u8,
        class: u8,
        func: u8,
        ret: u8,
        seq: u8,
    },
    Log(&'a [u8]),
    FormattedLog,
    ScanOfld(ScanOfld),
    BeaconFilter(BcnFltr),
    RaReport(RaReport),
    Other {
        cat: u8,
        class: u8,
        func: u8,
    },
}

/// `None` when the payload is shorter than the header, than an ack's or a
/// beacon filter report's 12 bytes, than an RA report's 16, or than a scan
/// report's 24.
pub fn parse(payload: &[u8]) -> Option<C2h<'_>> {
    let w0 = u32::from_le_bytes(payload.get(..4)?.try_into().ok()?);
    payload.get(4..C2H_HEADER_LEN)?;
    let cat = (w0 & 0x3) as u8;
    let class = ((w0 >> 2) & 0x3F) as u8;
    let func = ((w0 >> 8) & 0xFF) as u8;
    if cat == C2H_CAT_OUTSRC && class == C2H_CLASS_RA && func == FUNC_STS_RPT {
        let words = payload.get(8..RA_RPT_LEN)?;
        let w2 = u32::from_le_bytes(words[..4].try_into().ok()?);
        let w3 = u32::from_le_bytes(words[4..].try_into().ok()?);
        return Some(C2h::RaReport(RaReport {
            macid: w2 as u16,
            mode: ((w3 >> 8) & 0x3) as u8,
            rate: (w3 & 0x7F) as u8,
            giltf: ((w3 >> 10) & 0x7) as u8,
            bw: ((w3 >> 13) & 0x3) as u8,
        }));
    }
    if cat == C2H_CAT_MAC && class == C2H_CLASS_OFLD && func == FUNC_BCNFLTR_RPT {
        let w2 = u32::from_le_bytes(payload.get(8..BCNFLTR_LEN)?.try_into().ok()?);
        return Some(C2h::BeaconFilter(BcnFltr {
            macid: w2 as u8,
            kind: ((w2 >> 8) & 0x3) as u8,
            event: ((w2 >> 10) & 0x3) as u8,
            ma: (w2 >> 16) as u8,
        }));
    }
    if cat == C2H_CAT_MAC && class == C2H_CLASS_OFLD && func == FUNC_SCANOFLD_RSP {
        let word = |n: usize| -> Option<u32> {
            let at = n * 4;
            Some(u32::from_le_bytes(
                payload.get(at..at + 4)?.try_into().ok()?,
            ))
        };
        let _ = payload.get(..SCANOFLD_LEN)?;
        let (w2, w5) = (word(2)?, word(5)?);
        return Some(C2h::ScanOfld(ScanOfld {
            pri_ch: w2 as u8,
            reason: ((w2 >> 16) & 0xF) as u8,
            status: ((w2 >> 20) & 0xF) as u8,
            period: (w2 >> 24) as u8,
            tx_fail: (w5 & 0xF) as u8,
            air_density: ((w5 >> 4) & 0xF) as u8,
            band: ((w5 >> 24) & 0x3) as u8,
            mac_idx: w5 & (1 << 26) != 0,
        }));
    }
    if cat != C2H_CAT_MAC || class != C2H_CLASS_INFO {
        return Some(C2h::Other { cat, class, func });
    }
    match func {
        FUNC_REC_ACK | FUNC_DONE_ACK => {
            let w2 = u32::from_le_bytes(payload.get(8..ACK_LEN)?.try_into().ok()?);
            let (cat, class, f) = (
                (w2 & 0x3) as u8,
                ((w2 >> 2) & 0x3F) as u8,
                ((w2 >> 8) & 0xFF) as u8,
            );
            Some(if func == FUNC_REC_ACK {
                // fw.h:3965-3974: H2C_SEQ is [23:16].
                C2h::RecAck {
                    cat,
                    class,
                    func: f,
                    seq: (w2 >> 16) as u8,
                }
            } else {
                // fw.h:3952-3963: H2C_RETURN [23:16], H2C_SEQ [31:24].
                C2h::DoneAck {
                    cat,
                    class,
                    func: f,
                    ret: (w2 >> 16) as u8,
                    seq: (w2 >> 24) as u8,
                }
            })
        }
        FUNC_LOG => {
            let body = &payload[C2H_HEADER_LEN..];
            if body.len() >= FORMATTED_LOG_MIN_LEN && body[..2] == LOG_SIGNATURE {
                Some(C2h::FormattedLog)
            } else {
                Some(C2h::Log(body))
            }
        }
        _ => Some(C2h::Other { cat, class, func }),
    }
}

#[cfg(test)]
mod tests;

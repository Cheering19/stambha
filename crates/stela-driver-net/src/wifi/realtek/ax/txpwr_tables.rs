// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! The TX power and regulatory tables an rtw89 firmware file carries as
//! elements, read through the walk of [`super::elements`] and copied into
//! owned tables. Each TX power element is picked for the chip's RFE type as
//! `rtw89_fw_recognize_txpwr_from_elm` (pinned rtw89/fw.c:1176-1218) picks
//! it, the REGD element is listed as `rtw89_recognize_regd_from_elm`
//! (fw.c:1368-1410) and `rtw89_regd_entcpy` (fw.c:1319-1356) list it, and
//! the picked elements' entries are range-checked and stored as
//! `rtw89_load_rfe_data_from_fw` (fw.c:12470-12563) and its
//! `rtw89_fw_load_*` loaders (fw.c:12050-12447) store them. The entry
//! layouts are fw.h:5822-5912 and the table dimensions core.h:677-791,
//! 825-831, 985-1007 and 5386-5436.
//!
//! Only the 2.4 and 5 GHz by-rate, limit, RU limit and TX shape tables and
//! the REGD map are read. The 6 GHz, diversity-antenna (DA), RU shape and
//! tracking elements are walked past, and a missing needed element fails
//! where Linux would fall back to the chip's built-in tables (spec S1.8).
//! Nothing here chooses a regulatory index, reads a limit as `set_txpwr`
//! does, or writes a register.
use super::{
    chan::HwBand,
    elements::{self, Element, ElementError},
    io::AxError,
};
use alloc::{boxed::Box, vec::Vec};
use core::{fmt, slice::Chunks};

/// `RTW89_FW_ELEMENT_ID_TXPWR_BYRATE` (fw.h:4392).
pub const ID_TXPWR_BYRATE: u32 = 9;
/// `RTW89_FW_ELEMENT_ID_TXPWR_LMT_2GHZ`.
pub const ID_TXPWR_LMT_2G: u32 = 10;
/// `RTW89_FW_ELEMENT_ID_TXPWR_LMT_5GHZ`.
pub const ID_TXPWR_LMT_5G: u32 = 11;
/// `RTW89_FW_ELEMENT_ID_TXPWR_LMT_RU_2GHZ`.
pub const ID_TXPWR_LMT_RU_2G: u32 = 13;
/// `RTW89_FW_ELEMENT_ID_TXPWR_LMT_RU_5GHZ`.
pub const ID_TXPWR_LMT_RU_5G: u32 = 14;
/// `RTW89_FW_ELEMENT_ID_TX_SHAPE_LMT`.
pub const ID_TX_SHAPE_LMT: u32 = 16;
/// `RTW89_FW_ELEMENT_ID_REGD`.
pub const ID_REGD: u32 = 20;

/// `RTW89_BAND_NUM` (core.h:337-342): 2.4, 5 and 6 GHz.
pub const BAND_NUM: usize = 3;
/// `RTW89_BYR_BW_NUM` (core.h:988): 20 MHz to 320 MHz.
pub const BYR_BW_NUM: usize = 5;
/// `RTW89_2G_BW_NUM` (core.h:985): 20 and 40 MHz.
pub const BW_NUM_2G: usize = 2;
/// `RTW89_5G_BW_NUM` (core.h:986): 20 to 160 MHz.
pub const BW_NUM_5G: usize = 4;
/// `RTW89_NTX_NUM`.
pub const NTX_NUM: usize = 2;
/// `RTW89_RS_LMT_NUM`: CCK, OFDM and MCS.
pub const RS_LMT_NUM: usize = 3;
/// `RTW89_RS_TX_SHAPE_NUM`: CCK and OFDM.
pub const RS_TX_SHAPE_NUM: usize = 2;
/// `RTW89_BF_NUM`.
pub const BF_NUM: usize = 2;
/// `RTW89_REGD_NUM` (core.h:774-792).
pub const REGD_NUM: usize = 16;
/// `RTW89_2G_CH_NUM`: channels 1-14.
pub const CH_NUM_2G: usize = 14;
/// `RTW89_5G_CH_NUM`: 36-64, 100-144 and 149-177 in steps of 2.
pub const CH_NUM_5G: usize = 53;
/// `RTW89_RU_NUM` (core.h:998-1008).
pub const RU_NUM: usize = 8;
/// `NUM_OF_RTW89_REG_6GHZ_POWER`.
pub const REG_6GHZ_POWER_NUM: usize = 3;
/// `RTW89_REG_6GHZ_POWER_DFLT`: the only slot 2.4 and 5 GHz read
/// (core.h:8949-8950).
pub const REG_6GHZ_POWER_DFLT: usize = 0;

/// `enum rtw89_rate_section` (core.h:701-710).
pub const RS_CCK: u8 = 0;
pub const RS_OFDM: u8 = 1;
pub const RS_MCS: u8 = 2;
pub const RS_HEDCM: u8 = 3;
pub const RS_OFFSET: u8 = 4;

/// `RTW89_RATE_CCK_NUM` and its siblings (core.h:727-736), with
/// `__RTW89_RATE_MCS_NUM` and `__RTW89_RATE_OFFSET_NUM`.
pub const RATE_CCK_NUM: usize = 4;
pub const RATE_OFDM_NUM: usize = 8;
pub const RATE_MCS_NUM: usize = 16;
pub const RATE_HEDCM_NUM: usize = 4;
pub const RATE_OFFSET_NUM: usize = 8;
/// `RTW89_NSS_NUM` and `RTW89_NSS_HEDCM_NUM`.
pub const NSS_NUM: usize = 4;
pub const NSS_HEDCM_NUM: usize = 2;
/// `RTW89_OFDMA_NUM`.
pub const OFDMA_NUM: usize = 2;

/// `RTW89_NA`: the rule a REGD entry takes for a band it does not name or
/// names past `RTW89_THAILAND` (fw.c:1323-1344).
pub const REGD_NA: u8 = 4;
/// `RTW89_REGD_MAX_COUNTRY_NUM` (core.h:6990).
pub const REGD_MAX_COUNTRY_NUM: u32 = 255;
/// `NUM_OF_RTW89_REGD_FUNC` (core.h:6967-6972): TAS and DAG.
const REGD_FUNC_MASK: u32 = (1 << 2) - 1;
/// `RTW89_TXPWR_CONF_DFLT_RFE_TYPE` (core.h:5355).
const DFLT_RFE_TYPE: u8 = 0;

/// `rtw89_txpwr_lmt_{2,5}ghz_data.v`: `[bw][nt][rs][bf][regd][ch_idx]`.
pub type Lmt<const BW: usize, const CH: usize> =
    [[[[[[i8; CH]; REGD_NUM]; BF_NUM]; RS_LMT_NUM]; NTX_NUM]; BW];
pub type Lmt2g = Lmt<BW_NUM_2G, CH_NUM_2G>;
pub type Lmt5g = Lmt<BW_NUM_5G, CH_NUM_5G>;
/// `rtw89_txpwr_lmt_ru_{2,5}ghz_data.v`: `[ru][nt][regd][ch_idx]`.
pub type LmtRu<const CH: usize> = [[[[i8; CH]; REGD_NUM]; NTX_NUM]; RU_NUM];
pub type LmtRu2g = LmtRu<CH_NUM_2G>;
pub type LmtRu5g = LmtRu<CH_NUM_5G>;
/// `rtw89_tx_shape_lmt_data.v`: `[band][tx_shape_rs][regd][reg6_pwr]`.
pub type TxShapeLmt = [[[[u8; REG_6GHZ_POWER_NUM]; REGD_NUM]; RS_TX_SHAPE_NUM]; BAND_NUM];

/// `struct rtw89_txpwr_byrate` (core.h:825-832) without its trap byte: one
/// band and bandwidth's by-rate powers, in 0.25 dB.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ByRate {
    pub cck: [i8; RATE_CCK_NUM],
    pub ofdm: [i8; RATE_OFDM_NUM],
    /// `[ofdma][nss][idx]`.
    pub mcs: [[[i8; RATE_MCS_NUM]; NSS_NUM]; OFDMA_NUM],
    /// `[ofdma][nss][idx]`, MCS 0/1/3/4.
    pub hedcm: [[[i8; RATE_HEDCM_NUM]; NSS_HEDCM_NUM]; OFDMA_NUM],
    pub offset: [i8; RATE_OFFSET_NUM],
}

/// One country of the REGD element, as `rtw89_regd_entcpy` keeps it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RegdEntry {
    pub alpha2: [u8; 2],
    /// The 2.4, 5 and 6 GHz rules, `RTW89_NA` when past `RTW89_THAILAND`
    /// or not in a short entry.
    pub rule: [u8; 3],
    /// The `rtw89_regd_func` bits: TAS (bit 0) and DAG (bit 1) only.
    pub fmap: u32,
}

/// Which element a table came from, as Linux logs it
/// ("Firmware element ... version", fw.c:1607-1609).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Taken {
    pub id: u32,
    /// Where its header starts in the file.
    pub offset: usize,
    /// Its `size`: the bytes after the 32-byte header.
    pub size: usize,
    pub version: [u8; 4],
    /// The element's RFE type; REGD has none and reads 0.
    pub rfe_type: u8,
}

/// The tables `rtw89_load_rfe_data_from_fw` builds for one RFE type, each
/// starting zeroed and boxed. Not `Clone`: a copy would be one 40 KB value
/// on the stack in a debug build.
pub struct TxPowerTables {
    /// `rtwdev->byr[band][bw]`.
    pub byr: Box<[[ByRate; BYR_BW_NUM]; BAND_NUM]>,
    pub lmt_2ghz: Box<Lmt2g>,
    pub lmt_5ghz: Box<Lmt5g>,
    pub lmt_ru_2ghz: Box<LmtRu2g>,
    pub lmt_ru_5ghz: Box<LmtRu5g>,
    /// `tx_shape.lmt`.
    pub tx_shape_lmt: Box<TxShapeLmt>,
    /// `elm_info->regd->map`.
    pub regd: Vec<RegdEntry>,
    /// The elements read, in the order of [`NEEDED`]; `None` in tables not
    /// read from a file.
    pub taken: [Option<Taken>; 7],
}

/// The elements read, in ID order.
pub const NEEDED: [u32; 7] = [
    ID_TXPWR_BYRATE,
    ID_TXPWR_LMT_2G,
    ID_TXPWR_LMT_5G,
    ID_TXPWR_LMT_RU_2G,
    ID_TXPWR_LMT_RU_5G,
    ID_TX_SHAPE_LMT,
    ID_REGD,
];

impl Default for TxPowerTables {
    /// Every table zeroed and an empty REGD map.
    fn default() -> Self {
        Self {
            byr: Box::default(),
            lmt_2ghz: boxed([[[[[0; CH_NUM_2G]; REGD_NUM]; BF_NUM]; RS_LMT_NUM]; NTX_NUM]),
            lmt_5ghz: boxed([[[[[0; CH_NUM_5G]; REGD_NUM]; BF_NUM]; RS_LMT_NUM]; NTX_NUM]),
            lmt_ru_2ghz: boxed([[[0; CH_NUM_2G]; REGD_NUM]; NTX_NUM]),
            lmt_ru_5ghz: boxed([[[0; CH_NUM_5G]; REGD_NUM]; NTX_NUM]),
            tx_shape_lmt: Box::default(),
            regd: Vec::new(),
            taken: [None; 7],
        }
    }
}

impl fmt::Debug for TxPowerTables {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("TxPowerTables")
            .field("taken", &self.taken)
            .field("regd", &self.regd.len())
            .finish_non_exhaustive()
    }
}

impl TxPowerTables {
    /// The 20 MHz by-rate table of `band`, the only bandwidth the AX by-rate
    /// writes read (`rtw89_phy_set_txpwr_byrate_ax`, phy.c:3104-3106).
    pub fn byrate(&self, band: HwBand) -> &ByRate {
        &self.byr[band as usize][0]
    }

    /// `lmt[bw][nt][rs][bf][regd][ch_idx]` of `band`'s limit table, raw in
    /// 0.25 dB; 0 past the table and on 6 GHz.
    #[allow(clippy::too_many_arguments)] // One argument per table index.
    pub fn lmt(&self, band: HwBand, bw: u8, nt: u8, rs: u8, bf: u8, regd: u8, ch_idx: usize) -> i8 {
        fn at<const BW: usize, const CH: usize>(
            v: &Lmt<BW, CH>,
            [bw, nt, rs, bf, regd]: [u8; 5],
            ch_idx: usize,
        ) -> Option<i8> {
            v.get(usize::from(bw))?
                .get(usize::from(nt))?
                .get(usize::from(rs))?
                .get(usize::from(bf))?
                .get(usize::from(regd))?
                .get(ch_idx)
                .copied()
        }
        let index = [bw, nt, rs, bf, regd];
        match band {
            HwBand::G2 => at(&self.lmt_2ghz, index, ch_idx),
            HwBand::G5 => at(&self.lmt_5ghz, index, ch_idx),
            HwBand::G6 => None,
        }
        .unwrap_or(0)
    }

    /// `lmt_ru[ru][nt][regd][ch_idx]` of `band`'s RU limit table, raw in
    /// 0.25 dB; 0 past the table and on 6 GHz.
    pub fn lmt_ru(&self, band: HwBand, ru: u8, nt: u8, regd: u8, ch_idx: usize) -> i8 {
        fn at<const CH: usize>(v: &LmtRu<CH>, [ru, nt, regd]: [u8; 3], ch: usize) -> Option<i8> {
            v.get(usize::from(ru))?
                .get(usize::from(nt))?
                .get(usize::from(regd))?
                .get(ch)
                .copied()
        }
        let index = [ru, nt, regd];
        match band {
            HwBand::G2 => at(&self.lmt_ru_2ghz, index, ch_idx),
            HwBand::G5 => at(&self.lmt_ru_5ghz, index, ch_idx),
            HwBand::G6 => None,
        }
        .unwrap_or(0)
    }

    /// `rtw89_get_tx_shape_idx` (core.h:8935-8957) for a regulatory index
    /// already chosen: a rate section past OFDM reads OFDM's, and the 6 GHz
    /// power type is the default (6 GHz is not supported); 0 past the table.
    pub fn tx_shape(&self, band: HwBand, rs: u8, regd: u8) -> u8 {
        let rs = if usize::from(rs) >= RS_TX_SHAPE_NUM {
            RS_OFDM
        } else {
            rs
        };
        self.tx_shape_lmt[band as usize][usize::from(rs)]
            .get(usize::from(regd))
            .map_or(0, |by_power| by_power[REG_6GHZ_POWER_DFLT])
    }

    /// The REGD element's countries, in file order.
    pub fn regd_map(&self) -> &[RegdEntry] {
        &self.regd
    }
}

/// The TX power and REGD tables of `firmware` for a chip whose aid is
/// `chip_aid` (0 on every AX chip) and whose efuse RFE type is `rfe`.
pub fn tx_power_tables(firmware: &[u8], chip_aid: u16, rfe: u8) -> Result<TxPowerTables, AxError> {
    let mut confs: [Option<Conf<'_>>; KINDS.len()] = [None; KINDS.len()];
    let mut regd: Option<(Element<'_>, Vec<RegdEntry>)> = None;
    let mut failed = None;
    elements::walk(firmware, &mut |element| {
        if failed.is_some() {
            return;
        }
        if element.id == ID_REGD {
            match recognize_regd(&element) {
                Some(Ok(map)) => regd = Some((element, map)),
                Some(Err(why)) => failed = Some(why),
                None => {}
            }
        } else if let Some(at) = KINDS.iter().position(|kind| kind.id == element.id) {
            recognize_txpwr(&mut confs[at], element, chip_aid, rfe);
        }
    })
    .map_err(|error| {
        AxError::TxPower(match error {
            ElementError::NotContainer => "firmware has no element container",
            ElementError::EmptyContainer => "firmware's element container is empty",
            ElementError::Truncated { .. } => "firmware's element container runs past the file",
            // Only `phy_tables` refuses a radio slot.
            ElementError::RadioSlot { .. } => "firmware has an unusable element",
        })
    })?;
    if let Some(why) = failed {
        return Err(AxError::TxPower(why));
    }
    // fw.c:1617-1627: a needed element no handler took.
    for (conf, kind) in confs.iter().zip(&KINDS) {
        if conf.is_none() {
            return Err(AxError::TxPower(kind.lacks));
        }
    }
    let Some((regd_element, map)) = regd else {
        return Err(AxError::TxPower("firmware lacks REGD"));
    };

    let mut t = TxPowerTables::default();
    for (at, (conf, kind)) in confs.iter().zip(&KINDS).enumerate() {
        let Some(conf) = conf else { continue };
        let entries = conf.entries(kind.overruns)?;
        match kind.id {
            ID_TXPWR_BYRATE => load_byrate(&mut t.byr, entries),
            ID_TXPWR_LMT_2G => load_lmt(&mut t.lmt_2ghz, entries),
            ID_TXPWR_LMT_5G => load_lmt(&mut t.lmt_5ghz, entries),
            ID_TXPWR_LMT_RU_2G => load_lmt_ru(&mut t.lmt_ru_2ghz, entries),
            ID_TXPWR_LMT_RU_5G => load_lmt_ru(&mut t.lmt_ru_5ghz, entries),
            _ => load_tx_shape(&mut t.tx_shape_lmt, entries),
        }
        t.taken[at] = Some(taken(&conf.element, conf.rfe_type));
    }
    t.regd = map;
    t.taken[KINDS.len()] = Some(taken(&regd_element, 0));
    Ok(t)
}

/// A needed TX power element and the messages naming it.
struct Kind {
    id: u32,
    lacks: &'static str,
    overruns: &'static str,
}

/// The TX power elements of [`NEEDED`], REGD aside.
const KINDS: [Kind; 6] = [
    Kind {
        id: ID_TXPWR_BYRATE,
        lacks: "firmware lacks TXPWR_BYRATE",
        overruns: "TXPWR_BYRATE entries run past their element",
    },
    Kind {
        id: ID_TXPWR_LMT_2G,
        lacks: "firmware lacks TXPWR_LMT_2GHZ",
        overruns: "TXPWR_LMT_2GHZ entries run past their element",
    },
    Kind {
        id: ID_TXPWR_LMT_5G,
        lacks: "firmware lacks TXPWR_LMT_5GHZ",
        overruns: "TXPWR_LMT_5GHZ entries run past their element",
    },
    Kind {
        id: ID_TXPWR_LMT_RU_2G,
        lacks: "firmware lacks TXPWR_LMT_RU_2GHZ",
        overruns: "TXPWR_LMT_RU_2GHZ entries run past their element",
    },
    Kind {
        id: ID_TXPWR_LMT_RU_5G,
        lacks: "firmware lacks TXPWR_LMT_RU_5GHZ",
        overruns: "TXPWR_LMT_RU_5GHZ entries run past their element",
    },
    Kind {
        id: ID_TX_SHAPE_LMT,
        lacks: "firmware lacks TX_SHAPE_LMT",
        overruns: "TX_SHAPE_LMT entries run past their element",
    },
];

/// `struct rtw89_txpwr_conf` (core.h:5357-5362): the element held for one
/// table.
#[derive(Clone, Copy)]
struct Conf<'a> {
    element: Element<'a>,
    rfe_type: u8,
    ent_sz: u8,
    num_ents: u32,
}

impl<'a> Conf<'a> {
    /// `rtw89_for_each_in_txpwr_conf` (core.h:5375-5379): `num_ents`
    /// entries of `ent_sz` bytes. Linux reads them without checking the
    /// element's size; Stela refuses entries that run past it.
    fn entries(&self, overruns: &'static str) -> Result<Chunks<'a, u8>, AxError> {
        let bytes = entries_len(self.num_ents, self.ent_sz);
        let content = self
            .element
            .contents
            .get(..bytes)
            .ok_or(AxError::TxPower(overruns))?;
        // A zero entry size reads nothing: the cursor never passes data.
        Ok(content.chunks(usize::from(self.ent_sz).max(1)))
    }
}

fn entries_len(num_ents: u32, ent_sz: u8) -> usize {
    (num_ents as usize).saturating_mul(usize::from(ent_sz))
}

/// `rtw89_fw_recognize_txpwr_from_elm` (fw.c:1176-1218) for the table held
/// in `held`.
fn recognize_txpwr<'a>(held: &mut Option<Conf<'a>>, element: Element<'a>, chip_aid: u16, rfe: u8) {
    // fw.c:1186-1187.
    if element.aid != 0 && element.aid != chip_aid {
        return;
    }
    // __rtw89_fw_txpwr_element (fw.h:4475-4482): rsvd0, rsvd1, rfe_type,
    // ent_sz, num_ents.
    let [_, _, rfe_type, ent_sz, n0, n1, n2, n3] = element.head;
    // fw.c:1199-1206: a match is always taken, so the last one wins; the
    // default only while nothing or another default is held.
    let take = rfe_type == rfe
        || (rfe_type == DFLT_RFE_TYPE && held.is_none_or(|conf| conf.rfe_type == DFLT_RFE_TYPE));
    if take {
        *held = Some(Conf {
            element,
            rfe_type,
            ent_sz,
            num_ents: u32::from_le_bytes([n0, n1, n2, n3]),
        });
    }
}

/// `rtw89_recognize_regd_from_elm` (fw.c:1368-1410): `None` when the element
/// is ignored, else its map or why it is refused. Linux checks neither aid
/// nor RFE type here, and a later element replaces an earlier one.
fn recognize_regd(element: &Element<'_>) -> Option<Result<Vec<RegdEntry>, &'static str>> {
    // __rtw89_fw_regd_element (fw.h:4484-4491): rsvd0-2, ent_sz, num_ents.
    let [_, _, _, ent_sz, n0, n1, n2, n3] = element.head;
    let num_ents = u32::from_le_bytes([n0, n1, n2, n3]);
    // fw.c:1378-1385.
    if num_ents > REGD_MAX_COUNTRY_NUM {
        return None;
    }
    let Some(content) = element.contents.get(..entries_len(num_ents, ent_sz)) else {
        return Some(Err("REGD entries run past their element"));
    };
    let map: Vec<RegdEntry> = if ent_sz == 0 {
        Vec::new()
    } else {
        content
            .chunks(usize::from(ent_sz))
            .map(regd_entry)
            .collect()
    };
    // fw.c:1400-1405: a zero entry size lists none of them.
    if map.len() != num_ents as usize {
        return Some(Err("REGD element has invalid entries"));
    }
    Some(Ok(map))
}

/// `rtw89_regd_entcpy` (fw.c:1319-1356).
fn regd_entry(raw: &[u8]) -> RegdEntry {
    // struct rtw89_fw_regd_entry (fw.h:5822-5829), with its defaults.
    let mut e = [0, 0, REGD_NA, REGD_NA, REGD_NA, 0, 0, 0, 0];
    let valid = raw.len().min(e.len());
    e[..valid].copy_from_slice(&raw[..valid]);
    let rule = |r: u8| {
        if usize::from(r) < REGD_NUM {
            r
        } else {
            REGD_NA
        }
    };
    RegdEntry {
        alpha2: [e[0], e[1]],
        rule: [rule(e[2]), rule(e[3]), rule(e[4])],
        fmap: u32::from_le_bytes([e[5], e[6], e[7], e[8]]) & REGD_FUNC_MASK,
    }
}

/// `rtw89_txpwr_entcpy` (core.h:5364-5371) with
/// `__fw_txpwr_entry_acceptable` (fw.c:12050-12067): the first
/// `min(N, ent_sz)` bytes over a zeroed entry, or `None` when a byte past
/// the `N` this driver knows is not zero.
fn entry<const N: usize>(raw: &[u8]) -> Option<[u8; N]> {
    let (known, extension) = raw.split_at(raw.len().min(N));
    if extension.iter().any(|&byte| byte != 0) {
        return None;
    }
    let mut e = [0; N];
    e[..known.len()].copy_from_slice(known);
    Some(e)
}

/// `fw_txpwr_byrate_entry_valid` (fw.c:12070-12110) past the size check.
fn byrate_valid([band, nss, rs, shf, len, bw, ofdma]: [u8; 7]) -> bool {
    if usize::from(band) >= BAND_NUM || usize::from(bw) >= BYR_BW_NUM {
        return false;
    }
    let end = usize::from(shf) + usize::from(len);
    let nss = usize::from(nss);
    let ofdma = usize::from(ofdma);
    match rs {
        RS_CCK => end <= RATE_CCK_NUM,
        RS_OFDM => end <= RATE_OFDM_NUM,
        RS_MCS => end <= RATE_MCS_NUM && nss < NSS_NUM && ofdma < OFDMA_NUM,
        RS_HEDCM => end <= RATE_HEDCM_NUM && nss < NSS_HEDCM_NUM && ofdma < OFDMA_NUM,
        RS_OFFSET => end <= RATE_OFFSET_NUM,
        _ => false,
    }
}

/// `rtw89_fw_load_txpwr_byrate` (fw.c:12113-12141) with
/// `rtw89_phy_raw_byr_seek` (phy.c:2425-2444).
fn load_byrate(byr: &mut [[ByRate; BYR_BW_NUM]; BAND_NUM], entries: Chunks<'_, u8>) {
    for raw in entries {
        // struct rtw89_fw_txpwr_byrate_entry (fw.h:5832-5841).
        let Some([band, nss, rs, shf, len, d0, d1, d2, d3, bw, ofdma]) = entry::<11>(raw) else {
            continue;
        };
        if !byrate_valid([band, nss, rs, shf, len, bw, ofdma]) {
            continue;
        }
        let head = &mut byr[usize::from(band)][usize::from(bw)];
        let (nss, ofdma) = (usize::from(nss), usize::from(ofdma));
        let mut data = u32::from_le_bytes([d0, d1, d2, d3]);
        for i in 0..usize::from(len) {
            let idx = usize::from(shf) + i;
            let slot = match rs {
                RS_CCK => &mut head.cck[idx],
                RS_OFDM => &mut head.ofdm[idx],
                RS_MCS => &mut head.mcs[ofdma][nss][idx],
                RS_HEDCM => &mut head.hedcm[ofdma][nss][idx],
                _ => &mut head.offset[idx],
            };
            *slot = (data & 0xff) as u8 as i8;
            data >>= 8;
        }
    }
}

/// `rtw89_fw_load_txpwr_lmt_{2,5}ghz` (fw.c:12143-12222).
fn load_lmt<const BW: usize, const CH: usize>(v: &mut Lmt<BW, CH>, entries: Chunks<'_, u8>) {
    for raw in entries {
        // struct rtw89_fw_txpwr_lmt_{2,5}ghz_entry (fw.h:5843-5862).
        let Some([bw, nt, rs, bf, regd, ch_idx, value]) = entry::<7>(raw) else {
            continue;
        };
        let [bw, nt, rs, bf, regd, ch_idx] = [bw, nt, rs, bf, regd, ch_idx].map(usize::from);
        if bw >= BW
            || nt >= NTX_NUM
            || rs >= RS_LMT_NUM
            || bf >= BF_NUM
            || regd >= REGD_NUM
            || ch_idx >= CH
        {
            continue;
        }
        v[bw][nt][rs][bf][regd][ch_idx] = value as i8;
    }
}

/// `rtw89_fw_load_txpwr_lmt_ru_{2,5}ghz` (fw.c:12265-12334).
fn load_lmt_ru<const CH: usize>(v: &mut LmtRu<CH>, entries: Chunks<'_, u8>) {
    for raw in entries {
        // struct rtw89_fw_txpwr_lmt_ru_{2,5}ghz_entry (fw.h:5876-5891).
        let Some([ru, nt, regd, ch_idx, value]) = entry::<5>(raw) else {
            continue;
        };
        let [ru, nt, regd, ch_idx] = [ru, nt, regd, ch_idx].map(usize::from);
        if ru >= RU_NUM || nt >= NTX_NUM || regd >= REGD_NUM || ch_idx >= CH {
            continue;
        }
        v[ru][nt][regd][ch_idx] = value as i8;
    }
}

/// `rtw89_fw_load_tx_shape_lmt` (fw.c:12373-12411).
fn load_tx_shape(v: &mut TxShapeLmt, entries: Chunks<'_, u8>) {
    for raw in entries {
        // struct rtw89_fw_tx_shape_lmt_entry (fw.h:5906-5912).
        let Some([band, rs, regd, value, reg6_pwr]) = entry::<5>(raw) else {
            continue;
        };
        let [band, rs, regd, reg6_pwr] = [band, rs, regd, reg6_pwr].map(usize::from);
        if band >= BAND_NUM || rs >= RS_TX_SHAPE_NUM || regd >= REGD_NUM {
            continue;
        }
        if reg6_pwr >= REG_6GHZ_POWER_NUM {
            continue;
        }
        v[band][rs][regd][reg6_pwr] = value;
    }
}

fn taken(element: &Element<'_>, rfe_type: u8) -> Taken {
    Taken {
        id: element.id,
        offset: element.offset,
        size: element.contents.len(),
        version: element.version,
        rfe_type,
    }
}

/// `N` copies of `row` on the heap, without building the whole table on
/// the stack.
fn boxed<T: Clone, const N: usize>(row: T) -> Box<[T; N]> {
    let rows: Box<[T]> = alloc::vec![row; N].into_boxed_slice();
    match rows.try_into() {
        Ok(table) => table,
        Err(_) => unreachable!("vec![row; N] holds N rows"),
    }
}

#[cfg(test)]
mod tests;

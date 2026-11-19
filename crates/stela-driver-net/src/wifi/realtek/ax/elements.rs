// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! The PHY register tables an rtw89 firmware file carries after its
//! multi-firmware container, found as `rtw89_fw_recognize_elements`
//! (pinned rtw89/fw.c:1560) walks the elements and borrowed as
//! `rtw89_build_phy_tbl_from_elm` (fw.c:1099) builds them: BB, BB gain, each
//! radio slot and NCTL. Linux prefers these to the chip's built-in tables
//! (phy.c:1892, 2021-2022, 2073). Nothing here writes a register or grants
//! BB/RF setup, and TX power, regulatory and every other element are walked
//! past unread.

/// `RTW89_FW_ELEMENT_ID_BB_REG` (fw.h:4385).
pub const ID_BB_REG: u32 = 2;
/// `RTW89_FW_ELEMENT_ID_BB_GAIN`.
pub const ID_BB_GAIN: u32 = 3;
/// `RTW89_FW_ELEMENT_ID_RADIO_A`; B, C and D follow it.
pub const ID_RADIO_A: u32 = 4;
pub const ID_RADIO_B: u32 = 5;
pub const ID_RADIO_C: u32 = 6;
pub const ID_RADIO_D: u32 = 7;
/// `RTW89_FW_ELEMENT_ID_RF_NCTL`.
pub const ID_RF_NCTL: u32 = 8;
/// `RF_PATH_MAX` (core.h:842): the radio slots.
pub const RF_PATH_MAX: usize = 4;

/// An RF path, as `enum rtw89_rf_path`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RfPath {
    A,
    B,
    C,
    D,
}

/// One `rtw89_reg2_def` table borrowed from the file: little-endian
/// address/data pairs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Reg2<'a> {
    /// The element's `ver` bytes, as Linux logs them.
    pub version: [u8; 4],
    regs: &'a [u8],
}

impl<'a> Reg2<'a> {
    /// `n_regs`: whole pairs only (fw.c:1146).
    pub fn len(&self) -> usize {
        self.regs.len() / PAIR
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Pair `index` as (addr, data).
    pub fn get(&self, index: usize) -> Option<(u32, u32)> {
        let at = index.checked_mul(PAIR)?;
        self.regs.get(at..at.checked_add(PAIR)?).map(pair)
    }

    /// Every pair in file order.
    pub fn iter(&self) -> impl Iterator<Item = (u32, u32)> + use<'a> {
        self.regs
            .as_chunks::<PAIR>()
            .0
            .iter()
            .map(|bytes| pair(bytes))
    }
}

fn pair(bytes: &[u8]) -> (u32, u32) {
    (le32(bytes, 0), le32(bytes, 4))
}

fn le32(bytes: &[u8], at: usize) -> u32 {
    stela_driver_core::bytes::le32(bytes, at)
}

/// A radio table and the path its writes go to.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Radio<'a> {
    pub path: RfPath,
    pub table: Reg2<'a>,
}

/// The tables a file carries, as `rtw89_fw_elm_info` keeps them.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PhyTables<'a> {
    pub bb: Option<Reg2<'a>>,
    pub bb_gain: Option<Reg2<'a>>,
    /// By slot (`elm->u.reg2.idx`), not by path.
    pub radio: [Option<Radio<'a>>; RF_PATH_MAX],
    pub nctl: Option<Reg2<'a>>,
    /// The offset of an element that ran past the file, where the walk
    /// stopped as Linux's does; the tables before it are kept.
    pub stopped: Option<usize>,
}

/// Why a file's elements could not be walked.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ElementError {
    /// No MFW container (`rtw89_mfw_get_size` returns 0, fw.c:677-681).
    NotContainer,
    /// The container lists no image, or its last one ends at 0.
    EmptyContainer,
    /// The container's entry table runs past the file.
    Truncated { offset: usize, bytes: usize },
    /// A radio element's slot is past the last path.
    RadioSlot { offset: usize, idx: u8 },
}

/// The PHY tables in `firmware`, keeping only elements whose aid is 0 or
/// `chip_aid` (0 on every AX chip).
pub fn phy_tables(firmware: &[u8], chip_aid: u16) -> Result<PhyTables<'_>, ElementError> {
    let len = firmware.len();
    let mut offset = elements_start(firmware)?;
    let mut tables = PhyTables::default();
    // fw.c:1584: while (offset + sizeof(*hdr) < firmware->size).
    while offset + HEADER < len {
        let id = le32(firmware, offset);
        let size = le32(firmware, offset + 4) as usize;
        // fw.c:1587-1591 stops when offset + size reaches the end; Stela
        // also stops when the header-inclusive end passes it.
        let Some(end) = (offset + HEADER)
            .checked_add(size)
            .filter(|&end| offset + size < len && end <= len)
        else {
            tables.stopped = Some(offset);
            break;
        };
        tables.take(firmware, offset, id, end, chip_aid)?;
        // fw.c:1611-1612.
        offset = end.next_multiple_of(ELEMENT_ALIGN);
    }
    Ok(tables)
}

impl<'a> PhyTables<'a> {
    /// `rtw89_build_phy_tbl_from_elm` for the element at `offset`, whose
    /// contents end at `end`; any other id is walked past.
    fn take(
        &mut self,
        firmware: &'a [u8],
        offset: usize,
        id: u32,
        end: usize,
        chip_aid: u16,
    ) -> Result<(), ElementError> {
        let radio = match id {
            ID_RADIO_A => Some(RfPath::A),
            ID_RADIO_B => Some(RfPath::B),
            ID_RADIO_C => Some(RfPath::C),
            ID_RADIO_D => Some(RfPath::D),
            ID_BB_REG | ID_BB_GAIN | ID_RF_NCTL => None,
            _ => return Ok(()),
        };
        let table = Reg2 {
            version: [
                firmware[offset + 8],
                firmware[offset + 9],
                firmware[offset + 10],
                firmware[offset + 11],
            ],
            regs: &firmware[offset + HEADER..end],
        };
        // fw.c:1136-1138: ignore an element for another aid.
        let aid = u16::from_le_bytes([firmware[offset + 12], firmware[offset + 13]]);
        if aid != 0 && aid != chip_aid {
            return Ok(());
        }
        if let Some(path) = radio {
            let idx = firmware[offset + REG2_IDX];
            let slot = self
                .radio
                .get_mut(usize::from(idx))
                .ok_or(ElementError::RadioSlot { offset, idx })?;
            // fw.c:1139-1140: the first element of a slot wins.
            slot.get_or_insert(Radio { path, table });
            return Ok(());
        }
        let slot = match id {
            ID_BB_REG => &mut self.bb,
            ID_BB_GAIN => &mut self.bb_gain,
            _ => &mut self.nctl,
        };
        slot.get_or_insert(table);
        Ok(())
    }
}

/// Where the elements begin: ALIGN(last entry's shift + size, 16)
/// (`rtw89_mfw_get_size`, fw.c:668-691, and fw.c:1579-1582).
fn elements_start(firmware: &[u8]) -> Result<usize, ElementError> {
    if firmware.len() < MFW_HEADER || firmware[0] != MFW_SIG {
        return Err(ElementError::NotContainer);
    }
    let count = usize::from(firmware[1]);
    if count == 0 {
        return Err(ElementError::EmptyContainer);
    }
    let table = count * MFW_ENTRY;
    if MFW_HEADER + table > firmware.len() {
        return Err(ElementError::Truncated {
            offset: MFW_HEADER,
            bytes: table,
        });
    }
    let last = MFW_HEADER + (count - 1) * MFW_ENTRY;
    // Linux sums and aligns in u32, so an end that wraps to 0 is refused.
    let end = le32(firmware, last + 4).wrapping_add(le32(firmware, last + 8));
    match end.wrapping_add(ELEMENT_ALIGN as u32 - 1) & !(ELEMENT_ALIGN as u32 - 1) {
        0 => Err(ElementError::EmptyContainer),
        start => Ok(start as usize),
    }
}

const MFW_SIG: u8 = 0xff;
const MFW_HEADER: usize = 16;
const MFW_ENTRY: usize = 16;
/// `RTW89_FW_ELEMENT_ALIGN` (fw.h:4380).
const ELEMENT_ALIGN: usize = 16;
/// `sizeof(struct rtw89_fw_element_hdr)` (fw.h:4570-4630): 24 fixed bytes
/// and the union's 8-byte head; `size` excludes it.
const HEADER: usize = 32;
/// `u.reg2.idx`, the first byte of the union.
const REG2_IDX: usize = 24;
/// `sizeof(struct rtw89_reg2_def)`.
const PAIR: usize = 8;

#[cfg(test)]
pub(crate) mod tests;

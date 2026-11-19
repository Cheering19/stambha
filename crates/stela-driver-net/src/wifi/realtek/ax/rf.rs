// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! RF register access for an RTL8852C, as its chip ops `read_rf` and
//! `write_rf` reach it (rtw8852c.c:3068-3069; PCIe takes `rtw89_raw_io`,
//! fw.c:12682-12692, 12900-12901): `rtw89_phy_read_rf_v1` and
//! `rtw89_phy_write_rf_v1` (pinned rtw89/phy.c:984-997, 1190-1203).
//! Address bit 16 (`RTW89_RF_ADDR_ADSEL_MASK`, phy.h:11) chooses the path:
//! - set: the direct BB window at `rf_base_addr[path] + 4 * (addr & 0xFF)`
//!   (`rtw89_phy_read_rf`/`rtw89_phy_write_rf`, phy.c:928-947, 1128-1150;
//!   `rf_base_addr = {0xE000, 0xF000}`, rtw8852c.c:3172);
//! - clear: the SWSI serial interface (`rtw89_phy_read_rf_a`/
//!   `rtw89_phy_write_rf_a`, phy.c:950-982, 1153-1188), after
//!   `rtw89_phy_check_swsi_busy` (phy.c:922-926) finds it idle. Registers
//!   are reg.h:8858-8868 and 9138-9141.
//!
//! Where Linux logs and drops the write, or returns `INV_RF_DATA`, because
//! SWSI stayed busy or a read never completed, this returns
//! [`AxError::Timeout`] on 0x1174C, as the spec's ruling on RF serial
//! failures requires; a path past the chip's two (`rf_path_num = 2`,
//! rtw8852c.c:3215) is [`AxError::Phy`] before any I/O. This module grants
//! no RF table, calibration or other radio setup: it only reaches the
//! registers.
use super::{
    bb::{self, CR_BASE},
    elements::RfPath,
    io::{AxError, AxIo, Polled, poll_sample_atomic},
};

/// `RFREG_MASK` (core.h:48): an RF register's 20 bits.
pub const RFREG_MASK: u32 = 0xFFFFF;

/// `RTW89_RF_ADDR_ADSEL_MASK` (phy.h:11): the direct window, not SWSI.
const ADDR_ADSEL: u32 = 1 << 16;
/// `rtw8852c_chip_info.rf_base_addr` (rtw8852c.c:3172), BB offsets.
const RF_BASE: [u32; 2] = [0xE000, 0xF000];

/// `R_SWSI_DATA_V1` and its `B_SWSI_DATA_BIT_MASK_EN_V1` (reg.h:8858-8862).
const R_SWSI_DATA_V1: u32 = 0x0370;
const B_SWSI_DATA_BIT_MASK_EN_V1: u32 = 1 << 31;
/// `R_SWSI_BIT_MASK_V1` (reg.h:8863-8864); its field is `RFREG_MASK`.
const R_SWSI_BIT_MASK_V1: u32 = 0x0374;
/// `R_SWSI_READ_ADDR_V1` and `B_SWSI_READ_ADDR_V1` [10:0] (reg.h:8865-8868).
const R_SWSI_READ_ADDR_V1: u32 = 0x0378;
const B_SWSI_READ_ADDR_V1: u32 = 0x7FF;
/// `R_SWSI_V1` and its busy and done bits (reg.h:9138-9141).
const R_SWSI_V1: u32 = 0x174C;
const B_SWSI_W_BUSY_V1: u32 = 1 << 24;
const B_SWSI_R_BUSY_V1: u32 = 1 << 25;
const B_SWSI_R_DATA_DONE_V1: u32 = 1 << 26;

/// The path's index, refused past the chip's two (phy.c:989-992,
/// 1195-1198).
fn index(path: RfPath) -> Result<u32, AxError> {
    match path {
        RfPath::A => Ok(0),
        RfPath::B => Ok(1),
        RfPath::C | RfPath::D => Err(AxError::Phy("RF path past the chip's two")),
    }
}

/// `rtw89_phy_read_rf_v1`: RF `addr` of `path`, the `mask` field of it
/// shifted down.
pub fn read_rf<I: AxIo + ?Sized>(
    io: &mut I,
    path: RfPath,
    addr: u32,
    mask: u32,
) -> Result<u32, AxError> {
    let path = index(path)?;
    if addr & ADDR_ADSEL != 0 {
        bb::read32_mask(io, direct(path, addr), mask & RFREG_MASK)
    } else {
        read_swsi(io, path, addr, mask)
    }
}

/// `rtw89_phy_write_rf_v1`: replace the `mask` field of RF `addr` of `path`
/// with `data`, shifted to the mask's lowest bit.
pub fn write_rf<I: AxIo + ?Sized>(
    io: &mut I,
    path: RfPath,
    addr: u32,
    mask: u32,
    data: u32,
) -> Result<(), AxError> {
    let path = index(path)?;
    if addr & ADDR_ADSEL != 0 {
        bb::field32(io, direct(path, addr), mask & RFREG_MASK, data)?;
        // "delay to ensure writing properly" (phy.c:1146-1147).
        io.delay_us(1);
        Ok(())
    } else {
        write_swsi(io, path, addr, mask, data)
    }
}

/// The direct window's BB offset for `addr` (phy.c:940-941, 1140-1141).
fn direct(path: u32, addr: u32) -> u32 {
    RF_BASE[path as usize] + ((addr & 0xFF) << 2)
}

/// `rtw89_phy_check_swsi_busy`: W_BUSY, and only if that is clear a second
/// read for R_BUSY (`||` short-circuits).
fn swsi_busy<I: AxIo + ?Sized>(io: &mut I) -> Result<bool, AxError> {
    Ok(bb::read32_mask(io, R_SWSI_V1, B_SWSI_W_BUSY_V1)? != 0
        || bb::read32_mask(io, R_SWSI_V1, B_SWSI_R_BUSY_V1)? != 0)
}

/// `read_poll_timeout_atomic(rtw89_phy_check_swsi_busy, busy, !busy, 1, 30,
/// false, ..)`: 31 samples at most. A timeout fails the access with nothing
/// written.
fn wait_swsi_idle<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    match poll_sample_atomic(io, 1, 30, false, swsi_busy, |busy| !busy)? {
        Polled::Met(_) => Ok(()),
        Polled::Expired(_) => Err(AxError::Timeout {
            register: CR_BASE + R_SWSI_V1,
            mask: B_SWSI_W_BUSY_V1 | B_SWSI_R_BUSY_V1,
        }),
    }
}

/// `rtw89_phy_write_rf_a` (phy.c:1153-1188). The data is cut to 20 bits and,
/// for a partial mask, shifted to it, but never ANDed with the mask: the
/// chip applies the mask from `R_SWSI_BIT_MASK_V1`. Nothing waits for the
/// write to finish; the next access's idle wait covers it.
fn write_swsi<I: AxIo + ?Sized>(
    io: &mut I,
    path: u32,
    addr: u32,
    mask: u32,
    data: u32,
) -> Result<(), AxError> {
    wait_swsi_idle(io)?;
    let mut data = data & RFREG_MASK;
    let mask = mask & RFREG_MASK;
    let mut mask_en = 0;
    if mask != RFREG_MASK {
        mask_en = B_SWSI_DATA_BIT_MASK_EN_V1;
        bb::field32(io, R_SWSI_BIT_MASK_V1, RFREG_MASK, mask)?;
        // `__ffs(mask)`; an empty mask, undefined in Linux, shifts by 0 as
        // `io::field32` does.
        data = (data << (mask.trailing_zeros() % u32::BITS)) & RFREG_MASK;
    }
    // FIELD_PREP of BIT_MASK_EN b31, PATH [30:28], ADDR [27:20], VAL [19:0].
    let val = mask_en | (path << 28) | ((addr & 0xFF) << 20) | data;
    // `rtw89_phy_write32_mask(.., MASKDWORD, val)`: still a read first.
    bb::field32(io, R_SWSI_DATA_V1, u32::MAX, val)
}

/// `rtw89_phy_read_rf_a` (phy.c:950-982).
fn read_swsi<I: AxIo + ?Sized>(
    io: &mut I,
    path: u32,
    addr: u32,
    mask: u32,
) -> Result<u32, AxError> {
    wait_swsi_idle(io)?;
    let mask = mask & RFREG_MASK;
    // FIELD_PREP of PATH [10:8] and ADDR [7:0].
    let val = (path << 8) | (addr & 0xFF);
    bb::field32(io, R_SWSI_READ_ADDR_V1, B_SWSI_READ_ADDR_V1, val)?;
    io.delay_us(2);
    let done = |io: &mut I| bb::read32_mask(io, R_SWSI_V1, B_SWSI_R_DATA_DONE_V1);
    match poll_sample_atomic(io, 1, 30, false, done, |done| done != 0)? {
        Polled::Met(_) => bb::read32_mask(io, R_SWSI_V1, mask),
        Polled::Expired(_) => Err(AxError::Timeout {
            register: CR_BASE + R_SWSI_V1,
            mask: B_SWSI_R_DATA_DONE_V1,
        }),
    }
}

#[cfg(test)]
mod tests;

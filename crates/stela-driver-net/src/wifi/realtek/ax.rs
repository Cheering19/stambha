// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE (rtw89, the "AX" generation) register access. The masked and
//! polling helpers mirror rtw89's `rtw89_read*`/`rtw89_write*`,
//! `rtw89_write*_set`/`_clr`/`_mask` and `read_poll_timeout` uses from
//! pinned rtw89/core.h. This module grants no power, firmware or DMA
//! readiness; it only reads and writes a mapped window and waits on it.
pub mod efuse;
pub mod io;

pub mod power;
#[cfg(test)]
pub(crate) mod tests;

pub use io::{
    AxError, AxIo, NativeAxIo, clear8, clear16, clear32, field8, field16, field32, poll32,
    poll32_value, set8, set16, set32,
};
pub use power::{power_off, power_on, xtal_si_read, xtal_si_write};

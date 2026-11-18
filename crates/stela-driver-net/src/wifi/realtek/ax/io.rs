// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! Register access for rtw89 chips over a PCI BAR window, with the masked
//! read-modify-write and polling helpers that rtw89 builds on
//! `rtw89_read*`/`rtw89_write*` and `read_poll_timeout`. Unlike rtw88's
//! `Io`, nothing here is tied to one chip.
use crate::{Clock, bus::mmio::Mmio};
use alloc::sync::Arc;
use core::{any::Any, fmt, ptr::NonNull};

/// Smallest window: the S8 indirect watchdog registers sit at 0x40000.
pub const MIN_WINDOW_BYTES: usize = 0x4_0004;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxError {
    /// `mask` bits of `register` never reached the wanted value in time.
    Timeout {
        register: u32,
        mask: u32,
    },
    /// The register read all ones: the device is gone or not responding.
    AllOnes {
        register: u32,
    },
    /// Unaligned, past the window, or the window is unusable or retired.
    OutOfWindow {
        register: u32,
    },
    /// Uncalibrated, retired or non-advancing time source.
    Clock,
    Firmware(&'static str),
    Dma(&'static str),
    SecureBootUnsupported,
    /// `rtw89_mac_reset_pwr_state_ax`: the MAC is already powered on (-EBUSY).
    AlreadyPowered,
}

impl fmt::Display for AxError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Timeout { register, mask } => {
                write!(
                    f,
                    "timeout waiting on register {register:#x} mask {mask:#x}"
                )
            }
            Self::AllOnes { register } => write!(f, "register {register:#x} read all ones"),
            Self::OutOfWindow { register } => {
                write!(f, "register {register:#x} is outside the usable window")
            }
            Self::Clock => f.write_str("time source unavailable or not advancing"),
            Self::Firmware(what) => write!(f, "firmware: {what}"),
            Self::Dma(what) => write!(f, "dma: {what}"),
            Self::SecureBootUnsupported => f.write_str("secure-boot firmware is unsupported"),
            Self::AlreadyPowered => f.write_str("MAC has already powered on"),
        }
    }
}

/// Exact register access plus delays and a monotonic clock. Reads are
/// volatile observations and stores complete in program order; `delay_us`
/// waits at least the requested time.
pub trait AxIo {
    fn read8(&mut self, reg: u32) -> Result<u8, AxError>;
    fn read16(&mut self, reg: u32) -> Result<u16, AxError>;
    fn read32(&mut self, reg: u32) -> Result<u32, AxError>;
    fn write8(&mut self, reg: u32, v: u8) -> Result<(), AxError>;
    fn write16(&mut self, reg: u32, v: u16) -> Result<(), AxError>;
    fn write32(&mut self, reg: u32, v: u32) -> Result<(), AxError>;
    fn delay_us(&mut self, us: u64);
    /// Monotonic microseconds from the same clock `delay_us` waits on.
    fn now_us(&mut self) -> Result<u64, AxError>;
}

macro_rules! masked {
    ($t:ty, $read:ident, $write:ident, $set:ident, $clear:ident, $field:ident) => {
        /// `rtw89_write*_set`: set `bits`, keeping the rest.
        pub fn $set<I: AxIo + ?Sized>(io: &mut I, reg: u32, bits: $t) -> Result<(), AxError> {
            let old = io.$read(reg)?;
            io.$write(reg, old | bits)
        }
        /// `rtw89_write*_clr`: clear `bits`, keeping the rest.
        pub fn $clear<I: AxIo + ?Sized>(io: &mut I, reg: u32, bits: $t) -> Result<(), AxError> {
            let old = io.$read(reg)?;
            io.$write(reg, old & !bits)
        }
        /// `rtw89_write*_mask`: replace the `mask` field with `value`, which
        /// is shifted up to the mask's lowest set bit and cut to the mask.
        pub fn $field<I: AxIo + ?Sized>(
            io: &mut I,
            reg: u32,
            mask: $t,
            value: $t,
        ) -> Result<(), AxError> {
            let shift = mask.trailing_zeros() % <$t>::BITS;
            let old = io.$read(reg)?;
            io.$write(reg, (old & !mask) | ((value << shift) & mask))
        }
    };
}
masked!(u8, read8, write8, set8, clear8, field8);
masked!(u16, read16, write16, set16, clear16, field16);
masked!(u32, read32, write32, set32, clear32, field32);

/// Poll until `reg & mask == want`, sampling every `interval_us` for up to
/// `timeout_us` of wall time (`read_poll_timeout`: one last sample after the
/// deadline). A register reading all ones is refused at once. An unchecked
/// stalled clock is stopped by an independent sample-count guard.
pub fn poll32<I: AxIo + ?Sized>(
    io: &mut I,
    reg: u32,
    mask: u32,
    want: u32,
    interval_us: u64,
    timeout_us: u64,
) -> Result<(), AxError> {
    let start = io.now_us()?;
    let end = start.checked_add(timeout_us).ok_or(AxError::Clock)?;
    let wants_all_ones = mask == u32::MAX && want == u32::MAX;
    let guard = (timeout_us / interval_us.max(1))
        .saturating_add(3)
        .saturating_mul(2);
    let mut last = start;
    for _ in 0..guard {
        let now = io.now_us()?;
        if now < last {
            return Err(AxError::Clock);
        }
        last = now;
        let value = io.read32(reg)?;
        if value == u32::MAX && !wants_all_ones {
            return Err(AxError::AllOnes { register: reg });
        }
        if value & mask == want {
            return Ok(());
        }
        if now > end {
            return Err(AxError::Timeout {
                register: reg,
                mask,
            });
        }
        io.delay_us(interval_us);
        if io.now_us()? <= now {
            return Err(AxError::Clock);
        }
    }
    Err(AxError::Clock)
}

/// Real volatile MMIO over the device's BAR. The owner is kept for good on
/// quarantine: the device may still reach the mapping.
pub struct NativeAxIo {
    registers: Mmio,
    clock: Clock,
    bytes: usize,
    owned: Option<Arc<dyn Any>>,
}

impl NativeAxIo {
    /// # Safety
    /// `window` is the exclusive uncached BAR of the owned rtw89 requester,
    /// mapped for at least `bytes`, and `owner` keeps that claim and mapping
    /// alive. No other worker touches the device through another path, and
    /// `clock` is calibrated for the CPU that drives it.
    pub unsafe fn new(
        window: NonNull<u8>,
        bytes: usize,
        owner: Arc<dyn Any>,
        clock: Clock,
    ) -> Result<Self, AxError> {
        if bytes < MIN_WINDOW_BYTES || window.as_ptr().addr() & 3 != 0 {
            return Err(AxError::OutOfWindow { register: 0 });
        }
        if clock.ticks_per_microsecond == 0 {
            return Err(AxError::Clock);
        }
        Ok(Self {
            // SAFETY: the constructor's exclusive UC window contract.
            registers: unsafe { Mmio::new(window.as_ptr() as u64, bytes) },
            clock,
            bytes,
            owned: Some(owner),
        })
    }

    /// Refuse every further access and keep the owner (BAR claim and
    /// mapping) alive indefinitely.
    pub fn quarantine(&mut self) {
        if let Some(owner) = self.owned.take() {
            core::mem::forget(owner);
        }
    }

    fn check(&self, reg: u32, width: usize) -> Result<usize, AxError> {
        let at = reg as usize;
        if self.owned.is_none()
            || !at.is_multiple_of(width)
            || at.checked_add(width).is_none_or(|end| end > self.bytes)
        {
            return Err(AxError::OutOfWindow { register: reg });
        }
        Ok(at)
    }
}

impl AxIo for NativeAxIo {
    fn read8(&mut self, reg: u32) -> Result<u8, AxError> {
        Ok(self.registers.read8(self.check(reg, 1)?))
    }
    fn read16(&mut self, reg: u32) -> Result<u16, AxError> {
        Ok(self.registers.read16(self.check(reg, 2)?))
    }
    fn read32(&mut self, reg: u32) -> Result<u32, AxError> {
        Ok(self.registers.read32(self.check(reg, 4)?))
    }
    fn write8(&mut self, reg: u32, v: u8) -> Result<(), AxError> {
        self.registers.write8(self.check(reg, 1)?, v);
        Ok(())
    }
    fn write16(&mut self, reg: u32, v: u16) -> Result<(), AxError> {
        self.registers.write16(self.check(reg, 2)?, v);
        Ok(())
    }
    fn write32(&mut self, reg: u32, v: u32) -> Result<(), AxError> {
        self.registers.write32(self.check(reg, 4)?, v);
        Ok(())
    }
    fn delay_us(&mut self, us: u64) {
        self.clock.delay(us);
    }
    fn now_us(&mut self) -> Result<u64, AxError> {
        self.owned.as_ref().ok_or(AxError::Clock)?;
        Ok(self.clock.now_us())
    }
}

//! Fake rtw89 register file for tests: byte-addressed values, a full
//! operation trace, per-register read hooks and a virtual clock that only
//! `delay_us` advances, so polls are deterministic and instant. With
//! [`RegisterModel::emulate_rf`] it also answers the RTL8852C's SWSI serial
//! interface (BB 0x0370/0x0374/0x0378/0x174C) from an RF register file.
use crate::wifi::realtek::ax::{AxError, AxIo};
use alloc::{boxed::Box, collections::BTreeMap, vec::Vec};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    R8(u32),
    R16(u32),
    R32(u32),
    W8(u32, u8),
    W16(u32, u16),
    W32(u32, u32),
    Delay(u64),
    /// An SWSI RF write as the chip decodes it: path, address, the bit mask
    /// (0xFFFFF unless MASK_EN), and the 20-bit value already shifted to the
    /// mask, as SWSI carries it.
    RfW(u8, u8, u32, u32),
    /// An SWSI RF read latched by a write of 0x10378: path, address.
    RfR(u8, u8),
}

/// MMIO of `R_SWSI_DATA_V1`, `R_SWSI_BIT_MASK_V1`, `R_SWSI_READ_ADDR_V1`
/// and `R_SWSI_V1` (BB 0x0370, 0x0374, 0x0378, 0x174C; reg.h:8858-8868,
/// 9138-9141).
const SWSI_DATA: u32 = 0x10370;
const SWSI_BIT_MASK: u32 = 0x10374;
const SWSI_READ_ADDR: u32 = 0x10378;
const SWSI: u32 = 0x1174C;
/// `B_SWSI_R_DATA_DONE_V1`.
const SWSI_R_DATA_DONE: u32 = 0x0400_0000;
/// `RFREG_MASK` (core.h:48).
const RF_MASK: u32 = 0xF_FFFF;

pub type ReadHook = Box<dyn FnMut(&mut RegisterModel) -> u32>;

pub struct RegisterModel {
    regs: BTreeMap<u32, u8>,
    pub trace: Vec<Op>,
    hooks: BTreeMap<u32, ReadHook>,
    pub now: u64,
    /// Keep `now_us` constant across delays, as a stalled clock would.
    pub freeze_clock: bool,
    /// Microseconds each register read adds to the clock, as PCIe latency.
    pub read_cost_us: u64,
    /// Registers whose reads fail with [`READ_FAILED`].
    failing_reads: Vec<u32>,
    /// Registers whose 16-bit writes fail with [`WRITE_FAILED`], unstored.
    failing_writes: Vec<u32>,
    /// Status bits ORed into every emulated read of 0x1174C: set b24
    /// (`W_BUSY`) or b25 (`R_BUSY`) to keep SWSI busy.
    pub swsi_status: u32,
    /// Whether [`RegisterModel::emulate_rf`] was called.
    rf_emulated: bool,
    /// The RF register file, by (path, address), 20 bits each.
    rf: BTreeMap<(u8, u8), u32>,
    rf_hooks: BTreeMap<(u8, u8), ReadHook>,
    /// The (path, address) the last write of 0x10378 latched.
    rf_latched: (u8, u8),
}

/// What a read of a register named by [`RegisterModel::fail_reads_of`]
/// returns, as a register window that errors would.
pub const READ_FAILED: AxError = AxError::Mac("injected read failure");
pub const WRITE_FAILED: AxError = AxError::Mac("injected write failure");

impl RegisterModel {
    pub fn new() -> Self {
        Self {
            regs: BTreeMap::new(),
            trace: Vec::new(),
            hooks: BTreeMap::new(),
            now: 1_000,
            freeze_clock: false,
            read_cost_us: 0,
            failing_reads: Vec::new(),
            failing_writes: Vec::new(),
            swsi_status: 0,
            rf_emulated: false,
            rf: BTreeMap::new(),
            rf_hooks: BTreeMap::new(),
            rf_latched: (0, 0),
        }
    }
    /// Answer the SWSI serial interface from the RF register file:
    /// - a write32 of 0x10370 decodes PATH [30:28], ADDR [27:20], VAL [19:0]
    ///   and MASK_EN b31, takes the mask from 0x10374 [19:0] when MASK_EN is
    ///   set (else 0xFFFFF), stores `(old & !mask) | (val & mask)` and traces
    ///   [`Op::RfW`];
    /// - a write32 of 0x10378 latches ADDR [7:0] and PATH [10:8], runs the
    ///   [`RegisterModel::on_rf_read`] hook of that RF register if any, and
    ///   traces [`Op::RfR`];
    /// - a read32 of 0x1174C returns `swsi_status | R_DATA_DONE |
    ///   rf[latched]`, unless an [`RegisterModel::on_read`] hook of 0x1174C
    ///   replaces it.
    ///
    /// The SWSI registers still store what is written. The direct window
    /// (0x1E000/0x1F000) is plain registers, not aliased to the RF file.
    pub fn emulate_rf(&mut self) {
        self.rf_emulated = true;
    }
    /// Preset an RF register (cut to 20 bits) without tracing.
    pub fn set_rf(&mut self, path: u8, addr: u8, v: u32) {
        self.rf.insert((path, addr), v & RF_MASK);
    }
    /// Current value of an RF register without tracing.
    pub fn get_rf(&self, path: u8, addr: u8) -> u32 {
        *self.rf.get(&(path, addr)).unwrap_or(&0)
    }
    /// Replace the value an SWSI read of RF `path`/`addr` returns: the hook
    /// runs each time that read is latched, and what it returns (cut to 20
    /// bits) is stored in the RF file.
    pub fn on_rf_read(&mut self, path: u8, addr: u8, hook: ReadHook) {
        self.rf_hooks.insert((path, addr), hook);
    }
    /// The RF-level view: [`Op::RfW`], [`Op::RfR`], delays and every write
    /// that is not to an SWSI register (0x10370, 0x10374, 0x10378), in order.
    pub fn rf_ops(&self) -> Vec<Op> {
        self.trace
            .iter()
            .copied()
            .filter(|op| match op {
                Op::RfW(..) | Op::RfR(..) | Op::Delay(_) => true,
                Op::W8(reg, _) | Op::W16(reg, _) | Op::W32(reg, _) => {
                    !matches!(*reg, SWSI_DATA | SWSI_BIT_MASK | SWSI_READ_ADDR)
                }
                Op::R8(_) | Op::R16(_) | Op::R32(_) => false,
            })
            .collect()
    }
    /// The emulated effect of a write32 of `reg`, after it is stored.
    fn rf_write(&mut self, reg: u32, v: u32) {
        match reg {
            SWSI_DATA => {
                let path = ((v >> 28) & 0x7) as u8;
                let addr = (v >> 20) as u8;
                let val = v & RF_MASK;
                let mask = if v & 0x8000_0000 != 0 {
                    self.get32(SWSI_BIT_MASK) & RF_MASK
                } else {
                    RF_MASK
                };
                let old = self.get_rf(path, addr);
                self.rf.insert((path, addr), (old & !mask) | (val & mask));
                self.trace.push(Op::RfW(path, addr, mask, val));
            }
            SWSI_READ_ADDR => {
                let path = ((v >> 8) & 0x7) as u8;
                let addr = v as u8;
                self.rf_latched = (path, addr);
                self.trace.push(Op::RfR(path, addr));
                if let Some(mut hook) = self.rf_hooks.remove(&(path, addr)) {
                    let value = hook(self);
                    self.rf.insert((path, addr), value & RF_MASK);
                    self.rf_hooks.entry((path, addr)).or_insert(hook);
                }
            }
            _ => {}
        }
    }
    /// Make every read of `reg` (at any width) fail with [`READ_FAILED`].
    pub fn fail_reads_of(&mut self, reg: u32) {
        self.failing_reads.push(reg);
    }
    /// Make every 16-bit write of `reg` fail with [`WRITE_FAILED`], traced
    /// but not stored, until [`RegisterModel::allow_writes_of`].
    pub fn fail_writes_of(&mut self, reg: u32) {
        self.failing_writes.push(reg);
    }
    pub fn allow_writes_of(&mut self, reg: u32) {
        self.failing_writes.retain(|&failing| failing != reg);
    }
    fn get(&self, reg: u32, width: u32) -> u32 {
        (0..width).fold(0, |v, i| {
            v | (*self.regs.get(&(reg + i)).unwrap_or(&0) as u32) << (8 * i)
        })
    }
    fn put(&mut self, reg: u32, width: u32, value: u32) {
        for i in 0..width {
            self.regs.insert(reg + i, (value >> (8 * i)) as u8);
        }
    }
    /// Preset a register without tracing.
    pub fn set32(&mut self, reg: u32, value: u32) {
        self.put(reg, 4, value);
    }
    /// Current value of a register without tracing.
    pub fn get32(&self, reg: u32) -> u32 {
        self.get(reg, 4)
    }
    /// Replace the value every read of `reg` returns; the hook may change
    /// the model first. The returned value is also stored.
    pub fn on_read(&mut self, reg: u32, hook: ReadHook) {
        self.hooks.insert(reg, hook);
    }
    /// Writes and delays only, in order.
    pub fn writes(&self) -> Vec<Op> {
        self.trace
            .iter()
            .copied()
            .filter(|op| !matches!(op, Op::R8(_) | Op::R16(_) | Op::R32(_)))
            .collect()
    }
    fn read(&mut self, reg: u32, width: u32) -> u32 {
        if !self.freeze_clock {
            self.now += self.read_cost_us;
        }
        if let Some(mut hook) = self.hooks.remove(&reg) {
            let v = hook(self);
            self.put(reg, width, v);
            self.hooks.entry(reg).or_insert(hook);
        } else if self.rf_emulated && reg == SWSI && width == 4 {
            let (path, addr) = self.rf_latched;
            let v = self.swsi_status | SWSI_R_DATA_DONE | self.get_rf(path, addr);
            self.put(reg, width, v);
        }
        self.get(reg, width)
    }
}

impl AxIo for RegisterModel {
    fn read8(&mut self, reg: u32) -> Result<u8, AxError> {
        self.trace.push(Op::R8(reg));
        if self.failing_reads.contains(&reg) {
            return Err(READ_FAILED);
        }
        Ok(self.read(reg, 1) as u8)
    }
    fn read16(&mut self, reg: u32) -> Result<u16, AxError> {
        self.trace.push(Op::R16(reg));
        if self.failing_reads.contains(&reg) {
            return Err(READ_FAILED);
        }
        Ok(self.read(reg, 2) as u16)
    }
    fn read32(&mut self, reg: u32) -> Result<u32, AxError> {
        self.trace.push(Op::R32(reg));
        if self.failing_reads.contains(&reg) {
            return Err(READ_FAILED);
        }
        Ok(self.read(reg, 4))
    }
    fn write8(&mut self, reg: u32, v: u8) -> Result<(), AxError> {
        self.trace.push(Op::W8(reg, v));
        self.put(reg, 1, v as u32);
        Ok(())
    }
    fn write16(&mut self, reg: u32, v: u16) -> Result<(), AxError> {
        self.trace.push(Op::W16(reg, v));
        if self.failing_writes.contains(&reg) {
            return Err(WRITE_FAILED);
        }
        self.put(reg, 2, v as u32);
        Ok(())
    }
    fn write32(&mut self, reg: u32, v: u32) -> Result<(), AxError> {
        self.trace.push(Op::W32(reg, v));
        self.put(reg, 4, v);
        if self.rf_emulated {
            self.rf_write(reg, v);
        }
        Ok(())
    }
    fn delay_us(&mut self, us: u64) {
        self.trace.push(Op::Delay(us));
        if !self.freeze_clock {
            self.now += us;
        }
    }
    fn now_us(&mut self) -> Result<u64, AxError> {
        Ok(self.now)
    }
}

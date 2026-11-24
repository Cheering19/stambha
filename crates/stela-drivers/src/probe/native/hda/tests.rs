//! An HD Audio controller firmware left powered down or not decoding is
//! brought to D0 with memory decode on before its cold preflight, as Linux
//! `pci_enable_device` does, with Bus Master left as it was.
use super::*;
use crate::bus::pci::{Address, ConfigError, ConfigSpace, Function, power, regs};
use alloc::boxed::Box;

const PM: u16 = 0x50;

struct Config {
    words: Box<[u32; 64]>,
}

impl ConfigSpace for Config {
    fn read8(&self, offset: u16) -> Result<u8, ConfigError> {
        Ok((self.read32(offset & !3)? >> (8 * (offset & 3))) as u8)
    }
    fn read16(&self, offset: u16) -> Result<u16, ConfigError> {
        Ok((self.read32(offset & !3)? >> (8 * (offset & 3))) as u16)
    }
    fn write8(&mut self, offset: u16, value: u8) -> Result<(), ConfigError> {
        let shift = 8 * (offset & 3);
        let word = self.read32(offset & !3)?;
        self.write32(
            offset & !3,
            (word & !(0xff << shift)) | (u32::from(value) << shift),
        )
    }
    fn write16(&mut self, offset: u16, value: u16) -> Result<(), ConfigError> {
        let shift = 8 * (offset & 3);
        let word = self.read32(offset & !3)?;
        self.write32(
            offset & !3,
            (word & !(0xffff << shift)) | (u32::from(value) << shift),
        )
    }
    fn read32(&self, offset: u16) -> Result<u32, ConfigError> {
        Ok(self.words[usize::from(offset / 4)])
    }
    fn write32(&mut self, offset: u16, value: u32) -> Result<(), ConfigError> {
        self.words[usize::from(offset / 4)] = value;
        Ok(())
    }
}

struct Waits;
// SAFETY: a host test's software timer; no device depends on real time.
unsafe impl power::Timer for Waits {
    fn check(&self) -> Result<(), power::Error> {
        Ok(())
    }
    fn wait_us(&self, _: u64) -> Result<(), power::Error> {
        Ok(())
    }
}

/// AMD's 1022:15e3 with its power management capability, in `state` with
/// Command `command`.
fn controller(state: u32, command: u16) -> Function<Config> {
    let mut words = [0; 64];
    words[0] = 0x15e3_1022;
    words[1] = 0x0010_0000 | u32::from(command);
    words[2] = 0x0403_0000;
    words[4] = 0xfe80_0000;
    words[0x34 / 4] = u32::from(PM);
    words[usize::from(PM / 4)] = 0x0003_0001; // PM, version 3, end of list
    words[usize::from(PM / 4) + 1] = state | 1 << 3; // No_Soft_Reset
    Function::new(
        Address {
            segment: 0,
            bus: 0xc1,
            device: 0,
            function: 6,
        },
        Config {
            words: Box::new(words),
        },
    )
    .unwrap()
}

#[test]
fn a_controller_left_in_d3hot_without_decode_reaches_d0_decoding() {
    let mut function = controller(3, 0);
    enable_memory(&mut function, &Waits).unwrap();
    let config = function.config();
    assert_eq!(config.read16(PM + 4).unwrap() & 3, 0);
    let command = config.read16(regs::COMMAND).unwrap();
    assert_ne!(command & regs::COMMAND_MEMORY, 0);
    // Bus Master stays off until the cold stage has stopped old engines.
    assert_eq!(command & regs::COMMAND_MASTER, 0);
}

#[test]
fn a_decoding_controller_in_d0_is_left_as_it_is() {
    let command = regs::COMMAND_MEMORY | regs::COMMAND_INTX_DISABLE;
    let mut function = controller(0, command);
    enable_memory(&mut function, &Waits).unwrap();
    assert_eq!(function.config().read16(regs::COMMAND).unwrap(), command);
}

#[test]
fn a_controller_that_does_not_answer_is_refused() {
    let mut function = controller(0, 0);
    function.config().words[1] = u32::MAX;
    assert!(enable_memory(&mut function, &Waits).is_err());
}

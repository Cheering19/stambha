// SPDX-License-Identifier: GPL-2.0-only
// Copyright(c) 1999 - 2018 Intel Corporation.
// Copyright (C) 2007 Intel
// Copyright (C) Zhang Yanmin (yanmin.zhang@intel.com)
// Copyright (C) Shaohua Li (shaohua.li@intel.com)
// Copyright (c) 2026 Stela contributors
//! PCIe ASPM errata handling, shared by physical-device drivers.
//!
//! The 82574 workaround follows Linux `e1000e/netdev.c`'s
//! `__e1000e_disable_aspm` at commit
//! `a804c5742bff224d4cb4d25dd383554d598be32b`: disable L0s/L1 on the
//! downstream component before its upstream bridge. Register/topology access
//! and validation here are Stela's own implementation.
pub mod native;

use super::{ConfigSpace, EcamWindow, Function};
use crate::{Handoff, block::BlockError};
use alloc::vec::Vec;
use core::fmt;

const CAPABILITY_PCIE: u8 = 0x10;
const PCIE_FLAGS: u16 = 0x02;
const PCIE_LINK_CONTROL: u16 = 0x10;
const PCIE_ASPM: u16 = 3;
const BRIDGE_BUSES: u16 = 0x18;
const EXT_CAPABILITY_L1SS: u16 = 0x1e;
const L1SS_CONTROL1: u16 = 8;
const ASPM_L1_2: u32 = 1 << 2;

/// Disable only ASPM L1.2 at the downstream/upstream link components.
/// PCI-PM L1.2, L1.1, timing parameters and conventional Link Control remain
/// unchanged. Both capability chains and control snapshots are validated before
/// the downstream-first writes. An error may leave the downstream disabled;
/// it never rolls back by re-enabling an unsafe state or enables the device.
///
/// # Safety
/// The `disable_aspm_link` ownership/mapping requirements apply. In addition,
/// the host must own firmware/ACPI ASPM policy for this link; no firmware/AML
/// or other driver may restore L1.2 while this policy is in force.
pub unsafe fn disable_aspm_l1_2_link(
    function: &mut Function,
    handoff: &Handoff,
) -> Result<(), BlockError> {
    let windows = segment_windows(function, handoff)?;
    let mut functions = Vec::new();
    for window in windows {
        // SAFETY: checked exclusive segment ownership and complete ECAM
        // mappings; no second PCI or firmware owner may race this operation.
        match unsafe { Function::enumerate(&window) } {
            Ok(found) => functions.extend(found),
            Err(failure) => {
                let error = match failure.error {
                    super::EnumerationError::Config { error, .. } => BlockError::Config(error),
                    super::EnumerationError::Capability { error, .. } => {
                        BlockError::PciCapability(error)
                    }
                };
                // Retain every failed observation and original owner; a reason
                // return does not authorize config/root retirement.
                core::mem::forget(failure);
                return Err(error);
            }
        }
    }
    disable_l1_2_with(function, &mut functions)
}

fn l1ss_control<C: ConfigSpace>(function: &Function<C>) -> Result<Option<(u16, u32)>, BlockError> {
    l1ss_control_of(&function.config)
}

fn l1ss_control_of<C: ConfigSpace + ?Sized>(config: &C) -> Result<Option<(u16, u32)>, BlockError> {
    // Validate a real conventional PCIe function before accessing its extended
    // configuration space. Keep malformed chains distinct from absent L1SS.
    link_control_of(config)?;
    let first = config.read32(0x100)?;
    if first == u32::MAX {
        return Err(BlockError::Device { status: first });
    }
    let mut found = None;
    for entry in super::capability::extended_capabilities(config) {
        let entry = entry?;
        if entry.id != EXT_CAPABILITY_L1SS {
            continue;
        }
        if found.is_some() || entry.version != 1 {
            return Err(BlockError::Unsupported(
                "invalid or duplicate PCI L1SS capability",
            ));
        }
        let capability = config.read32(entry.offset + 4)?;
        let control = config.read32(entry.offset + L1SS_CONTROL1)?;
        if capability == u32::MAX || control == u32::MAX {
            return Err(BlockError::Device { status: u32::MAX });
        }
        if capability & (1 << 4) == 0 || (control & ASPM_L1_2 != 0 && capability & ASPM_L1_2 == 0) {
            return Err(BlockError::Unsupported(
                "inconsistent PCI L1SS support/control",
            ));
        }
        found = Some((entry.offset + L1SS_CONTROL1, control));
    }
    Ok(found)
}

fn clear_l1_2<C: ConfigSpace>(
    function: &mut Function<C>,
    snapshot: (u16, u32),
) -> Result<(), BlockError> {
    clear_l1ss(&mut function.config, snapshot, ASPM_L1_2)
}

/// Clear `bits` of L1 PM Substates Control 1 from its `snapshot`, read back.
fn clear_l1ss<C: ConfigSpace + ?Sized>(
    config: &mut C,
    snapshot: (u16, u32),
    bits: u32,
) -> Result<(), BlockError> {
    let (offset, original) = snapshot;
    let wanted = original & !bits;
    if original != wanted {
        // Linux notes some devices require DWORD access to L1SS. CTL1 is a
        // full control register, unlike Link Control's adjacent W1C status.
        config.write32(offset, wanted)?;
    }
    let observed = config.read32(offset)?;
    if observed != wanted {
        return Err(BlockError::Device { status: observed });
    }
    Ok(())
}

fn disable_l1_2_with<C: ConfigSpace>(
    function: &mut Function<C>,
    functions: &mut [Function<C>],
) -> Result<(), BlockError> {
    let parent = upstream(function, functions)?;
    let child_control = l1ss_control(function)?;
    let parent_control = parent
        .map(|index| l1ss_control(&functions[index]))
        .transpose()?
        .flatten();
    // An older peer without L1SS cannot form L1.2. Do not refuse a working
    // endpoint or invent a missing control; disable every advertised side.
    if let Some(child) = child_control {
        clear_l1_2(function, child)?;
    }
    if let Some((parent, control)) = parent.zip(parent_control) {
        clear_l1_2(&mut functions[parent], control)?;
    }
    Ok(())
}

/// Disable both ASPM states on a PCIe function and its immediate upstream
/// bridge, before enabling the device's MMIO or bus mastering. Both ends
/// are read back; a refused setting is an error. Bus topology comes from
/// type-1 headers, not from bus-number arithmetic. An endpoint directly on
/// a root bus has no parent bridge and only its own control is changed.
///
/// # Safety
///
/// `handoff` must describe valid identity-mapped ECAM windows after
/// ExitBootServices. The caller must exclusively own PCI configuration
/// access throughout the function's segment for this operation; no other
/// firmware, thread or driver may access those configuration registers.
/// `function` must be the unique active owner of its addressed function.
pub unsafe fn disable_aspm_link(
    function: &mut Function,
    handoff: &Handoff,
) -> Result<(), BlockError> {
    let windows = segment_windows(function, handoff)?;
    let mut functions = Vec::new();
    for window in windows {
        // SAFETY: the caller owns configuration access; segment_windows
        // checked every window's entire mapping and disjoint bus numbers.
        match unsafe { Function::enumerate(&window) } {
            Ok(found) => functions.extend(found),
            Err(failure) => {
                let error = match failure.error {
                    super::EnumerationError::Config { error, .. } => BlockError::Config(error),
                    super::EnumerationError::Capability { error, .. } => {
                        BlockError::PciCapability(error)
                    }
                };
                // Retain every failed observation and original owner; a reason
                // return does not authorize config/root retirement.
                core::mem::forget(failure);
                return Err(error);
            }
        }
    }
    disable_aspm_with(function, &mut functions)
}

fn segment_windows<C: ConfigSpace>(
    function: &Function<C>,
    handoff: &Handoff,
) -> Result<Vec<EcamWindow>, BlockError> {
    let windows: Vec<_> = handoff
        .ecam
        .iter()
        .filter(|window| window.segment == function.address.segment)
        .copied()
        .collect();
    let mapped = windows
        .iter()
        .all(|window| window.start_bus <= window.end_bus && handoff.is_mapped(&window.range()));
    let contains = windows
        .iter()
        .any(|window| (window.start_bus..=window.end_bus).contains(&function.address.bus));
    if !mapped || !contains {
        // Skipping an unmapped range could hide the upstream bridge.
        return Err(BlockError::Unsupported(
            "unmapped PCI configuration for ASPM",
        ));
    }
    for (index, window) in windows.iter().enumerate() {
        if windows[index + 1..]
            .iter()
            .any(|other| window.start_bus <= other.end_bus && other.start_bus <= window.end_bus)
        {
            return Err(BlockError::Unsupported(
                "overlapping PCI configuration windows",
            ));
        }
    }
    Ok(windows)
}

fn link_control_optional<C: ConfigSpace>(
    function: &Function<C>,
) -> Result<Option<u16>, BlockError> {
    link_control_optional_of(&function.config)
}

fn link_control_optional_of<C: ConfigSpace + ?Sized>(
    config: &C,
) -> Result<Option<u16>, BlockError> {
    let mut express = None;
    for entry in super::capability::capabilities(config) {
        let entry = entry?;
        if entry.id == CAPABILITY_PCIE && express.replace(entry.offset).is_some() {
            return Err(BlockError::Unsupported("multiple PCIe capabilities"));
        }
    }
    let Some(capability) = express else {
        return Ok(None);
    };
    if capability + PCIE_LINK_CONTROL + 2 > 0x100 {
        return Err(BlockError::Unsupported(
            "PCIe link control outside capability space",
        ));
    }
    let flags = config.read16(capability + PCIE_FLAGS)?;
    let port_type = (flags >> 4) & 15;
    if matches!(port_type, 9 | 10) {
        return Ok(None);
    }
    if flags & 15 == 0 || !matches!(port_type, 0 | 1 | 4 | 5 | 6 | 7 | 8) {
        return Err(BlockError::Unsupported("PCIe function has no link control"));
    }
    Ok(Some(capability + PCIE_LINK_CONTROL))
}
fn link_control<C: ConfigSpace>(function: &Function<C>) -> Result<u16, BlockError> {
    link_control_of(&function.config)
}
fn link_control_of<C: ConfigSpace + ?Sized>(config: &C) -> Result<u16, BlockError> {
    link_control_optional_of(config)?
        .ok_or(BlockError::Unsupported("ASPM requires a PCIe capability"))
}
pub fn aspm_control<C: ConfigSpace>(function: &Function<C>) -> Result<u16, BlockError> {
    Ok(link_control_word(function)? & PCIE_ASPM)
}
pub fn link_control_word<C: ConfigSpace>(function: &Function<C>) -> Result<u16, BlockError> {
    match link_control_optional(function)? {
        Some(at) => Ok(function.config.read16(at)?),
        None => Ok(0),
    }
}
pub fn link_speed<C: ConfigSpace>(function: &Function<C>) -> Result<u8, BlockError> {
    match link_control_optional(function)? {
        Some(at) => Ok((function.config.read16(at + 2)? & 15) as u8),
        None => Ok(0),
    }
}
fn upstream<C: ConfigSpace>(
    function: &Function<C>,
    functions: &[Function<C>],
) -> Result<Option<usize>, BlockError> {
    let mut found = None;
    for (index, candidate) in functions.iter().enumerate() {
        if candidate.address.segment != function.address.segment
            || candidate.address == function.address
            || candidate.class[..2] != [6, 4]
            || candidate.header_type()? != 1
        {
            continue;
        }
        let buses = candidate.config.read32(BRIDGE_BUSES)?;
        let primary = buses as u8;
        let secondary = (buses >> 8) as u8;
        let subordinate = (buses >> 16) as u8;
        // Firmware leaves unused bridges with secondary bus zero. They do
        // not make every root-bus endpoint their downstream child.
        if secondary == 0 || secondary != function.address.bus {
            continue;
        }
        if primary != candidate.address.bus || secondary == primary || subordinate < secondary {
            return Err(BlockError::Unsupported(
                "invalid upstream PCI bridge routing",
            ));
        }
        if found.replace(index).is_some() {
            return Err(BlockError::Unsupported("ambiguous upstream PCI bridge"));
        }
    }
    Ok(found)
}

fn clear_aspm<C: ConfigSpace>(function: &mut Function<C>, control: u16) -> Result<(), BlockError> {
    clear_aspm_of(&mut function.config, control)
}

fn clear_aspm_of<C: ConfigSpace + ?Sized>(config: &mut C, control: u16) -> Result<(), BlockError> {
    let value = config.read16(control)?;
    if value & PCIE_ASPM != 0 {
        // Narrow access is essential: Link Status shares this DWORD and
        // includes W1C fields that must not be acknowledged by this write.
        config.write16(control, value & !PCIE_ASPM)?;
    }
    let status = config.read16(control)?;
    if status & PCIE_ASPM != 0 {
        return Err(BlockError::Device {
            status: u32::from(status),
        });
    }
    Ok(())
}

/// L1 PM Substates Control 1's four enables: ASPM L1.1 and L1.2, PCI-PM
/// L1.1 and L1.2 (Linux `PCI_L1SS_CTL1_L1SS_MASK`).
const L1SS_ENABLES: u32 = 0xf;

/// One end of a link as [`disable_link_power`] found it: Link Control's
/// ASPM field (bit 0 L0s, bit 1 L1) and L1 PM Substates Control 1's four
/// enables, `None` where that end has no L1 PM Substates capability.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct EndPower {
    pub aspm: u16,
    pub substates: Option<u32>,
}

impl fmt::Display for EndPower {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(["off", "L0s", "L1", "L0s+L1"][usize::from(self.aspm & PCIE_ASPM)])?;
        match self.substates {
            None => f.write_str(" with no L1SS"),
            Some(0) => f.write_str(" with L1SS none"),
            Some(enables) => write!(f, " with L1SS enables {enables:x}"),
        }
    }
}

/// What [`disable_link_power`] found at the device's end and at the port's
/// before it turned their power management off; `port` is `None` where
/// only the device's end was given.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LinkPowerOff {
    pub device: EndPower,
    pub port: Option<EndPower>,
}

/// Turn a link's power management off as Linux's ASPM core does when its
/// policy is `performance` (`pcie_config_aspm_link(link, 0)`): every L1 PM
/// Substate off at both ends, the device's first, then ASPM L0s and L1 off
/// in Link Control at the device and then at the port, which is the order
/// PCIe asks for disabling L1. `port` is the bridge directly above
/// `device`; without it only the device's end changes, which still stops
/// L1 entry, as L1 needs both ends to enable it. Both ends' capabilities
/// are validated before the first write and every write is read back. An
/// error may leave the device's end off; nothing is turned back on.
pub fn disable_link_power<P: ConfigSpace + ?Sized, C: ConfigSpace + ?Sized>(
    device: &mut C,
    port: Option<&mut P>,
) -> Result<LinkPowerOff, BlockError> {
    let mut port = port;
    let device_control = link_control_of(device)?;
    let device_l1ss = l1ss_control_of(device)?;
    let port_controls = match port.as_deref() {
        Some(port) => Some((link_control_of(port)?, l1ss_control_of(port)?)),
        None => None,
    };
    let found = LinkPowerOff {
        device: EndPower {
            aspm: device.read16(device_control)? & PCIE_ASPM,
            substates: device_l1ss.map(|(_, enables)| enables & L1SS_ENABLES),
        },
        port: match (port.as_deref(), port_controls) {
            (Some(port), Some((control, l1ss))) => Some(EndPower {
                aspm: port.read16(control)? & PCIE_ASPM,
                substates: l1ss.map(|(_, enables)| enables & L1SS_ENABLES),
            }),
            _ => None,
        },
    };
    if let Some(snapshot) = device_l1ss {
        clear_l1ss(device, snapshot, L1SS_ENABLES)?;
    }
    if let (Some(port), Some((_, Some(snapshot)))) = (port.as_deref_mut(), port_controls) {
        clear_l1ss(port, snapshot, L1SS_ENABLES)?;
    }
    clear_aspm_of(device, device_control)?;
    if let (Some(port), Some((control, _))) = (port, port_controls) {
        clear_aspm_of(port, control)?;
    }
    Ok(found)
}

fn disable_aspm_with<C: ConfigSpace>(
    function: &mut Function<C>,
    functions: &mut [Function<C>],
) -> Result<(), BlockError> {
    let upstream = upstream(function, functions)?;
    let downstream_control = link_control(function)?;
    // Validate both capabilities before changing either end of the link.
    let upstream_control = upstream
        .map(|index| link_control(&functions[index]))
        .transpose()?;
    clear_aspm(function, downstream_control)?;
    if let Some((index, control)) = upstream.zip(upstream_control) {
        clear_aspm(&mut functions[index], control)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests;

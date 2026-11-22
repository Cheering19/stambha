//! The routine blocks call for AMX register operations through their PE's
//! cell when some operations are left to the reference model
//! (`--amx-native off`): [`routine`] completes those on the model alone,
//! which never takes a host kernel, copies and constants included, and the
//! rest through the compiler's routine (`x86_64::amx_routine::routine`,
//! host kernels where `amx::native` covers an operand, else the model).
//! Under step timing the counting routine (`forms::routine`) wraps it.
//!
//! Each completes an operation as the compiler's routine does: whole or
//! not at all, counted in the cell by number and path, every [`SAMPLE`]th
//! call of an operation timed. The code blocks hold is the same whichever
//! routine the cell names, and so is the translator id.
use super::forms::AmxCell;
use stela_compiler::{
    amx::{
        AmxOperation, AmxRegisters,
        model::{self, AmxMemory, MemoryFault},
    },
    x86_64::{
        amx_routine,
        amx_transfer::{Cell, KINDS, SAMPLE},
    },
};

/// The register operation each number (bits 9:5 of a word) names, where
/// blocks call the routine for it.
pub(super) const OPERATIONS: [Option<AmxOperation>; KINDS] = {
    let mut table = [None; KINDS];
    let mut index = 0;
    while index < AmxOperation::ALL.len() {
        let operation = AmxOperation::ALL[index];
        let number = operation.encoding().0 as usize;
        if number < KINDS && amx_routine::calls(operation) {
            table[number] = Some(operation);
        }
        index += 1;
    }
    table
};

/// The bit of operation number `number` in a mask of operations left to
/// the model ([`AmxCell::model`]); none past the mask.
pub(super) const fn bit(number: u32) -> u32 {
    match 1u32.checked_shl(number) {
        Some(bit) => bit,
        None => 0,
    }
}

/// The time stamp counter, or 0 where there is none.
pub(super) fn ticks() -> u64 {
    #[cfg(target_arch = "x86_64")]
    {
        // SAFETY: RDTSC is unprivileged on every host Stela runs on.
        unsafe { core::arch::x86_64::_rdtsc() }
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        0
    }
}

/// What a register operation reaches of memory: nothing.
struct NoMemory;

impl AmxMemory for NoMemory {
    fn read(&mut self, address: u64, _: &mut [u8]) -> Result<(), MemoryFault> {
        Err(MemoryFault {
            address,
            write: false,
        })
    }
    fn write(&mut self, address: u64, _: &[u8]) -> Result<(), MemoryFault> {
        Err(MemoryFault {
            address,
            write: true,
        })
    }
}

/// Complete the operation numbered `operation` with `operand` on the
/// model alone: 1, counted on the model's path as the compiler's routine
/// counts, or 0 with nothing changed.
///
/// # Safety
/// `cell` is the PE's cell with a live register file at `registers`, which
/// nothing else accesses during the call.
unsafe fn modelled(cell: *mut Cell, operation: u32, operand: u64) -> u64 {
    // SAFETY: the caller's contract.
    let cell = unsafe { &mut *cell };
    let number = operation as usize;
    let (Some(generation), Some(Some(operation))) = (cell.generation, OPERATIONS.get(number))
    else {
        return 0;
    };
    if cell.registers == 0 {
        return 0;
    }
    // SAFETY: live and exclusive (the caller's contract).
    let registers = unsafe { &mut *(cell.registers as *mut AmxRegisters) };
    let counts = cell.kernels[number];
    let started = (counts[0] + counts[1]).is_multiple_of(SAMPLE).then(ticks);
    if model::execute_for(generation, registers, *operation, operand, &mut NoMemory).is_err() {
        // No register operation fails; were one to, the service reports it.
        return 0;
    }
    if let Some(started) = started {
        let sampled = &mut cell.sampled[number][1];
        sampled[0] += ticks().wrapping_sub(started);
        sampled[1] += 1;
    }
    cell.kernels[number][1] += 1;
    cell.calls += 1;
    1
}

/// Complete the operation numbered `operation` on the model alone where
/// the cell's [`AmxCell::model`] names it, else through the compiler's
/// routine: 1, or 0 with nothing changed.
///
/// # Safety
/// As `amx_routine::routine`, and `cell` is the `cell` of an [`AmxCell`].
pub(super) unsafe fn complete(cell: *mut Cell, operation: u32, operand: u64) -> u64 {
    // SAFETY: the caller's contract: `cell` is an `AmxCell`'s first field,
    // which nothing else accesses during the call.
    let model = unsafe { (*cell.cast::<AmxCell>()).model };
    if model & bit(operation) != 0 {
        // SAFETY: the caller's contract.
        unsafe { modelled(cell, operation, operand) }
    } else {
        // SAFETY: the caller's contract.
        unsafe { amx_routine::routine(cell, operation, operand) }
    }
}

/// The routine blocks call when operations are left to the model
/// (`Machine::enter_amx_cell`): [`complete`].
///
/// # Safety
/// As `amx_routine::routine`, and `cell` is the `cell` of an [`AmxCell`].
#[cfg(target_arch = "x86_64")]
pub unsafe extern "sysv64" fn routine(cell: *mut Cell, operation: u32, operand: u64) -> u64 {
    // SAFETY: the caller's contract.
    unsafe { complete(cell, operation, operand) }
}

/// Only x86-64 runs translated code; ARM hosts compile the routine alone.
///
/// # Safety
/// As on x86-64.
#[cfg(not(target_arch = "x86_64"))]
pub unsafe extern "C" fn routine(cell: *mut Cell, operation: u32, operand: u64) -> u64 {
    // SAFETY: the caller's contract.
    unsafe { complete(cell, operation, operand) }
}

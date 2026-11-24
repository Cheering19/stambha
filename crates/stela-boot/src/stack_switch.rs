//! Running code on another stack: the GPU's services and Mesa's threads on
//! stacks of their own, and the boot's first stop on the one kept for it
//! (`output::fatal`).
use core::{arch::naked_asm, ffi::c_void};

/// This code's stack pointer.
pub(crate) fn pointer() -> u64 {
    let pointer: u64;
    // SAFETY: reads a register.
    unsafe { core::arch::asm!("mov {}, rsp", out(reg) pointer, options(nomem, nostack)) };
    pointer
}

#[unsafe(naked)]
unsafe extern "win64" fn switch(
    body: unsafe extern "win64" fn(*mut c_void),
    argument: *mut c_void,
    top: u64,
) {
    naked_asm!(
        "push rbp",
        "mov rbp, rsp",
        "lea rsp, [r8 - 32]",
        "mov rax, rcx",
        "mov rcx, rdx",
        "call rax",
        "mov rsp, rbp",
        "pop rbp",
        "ret",
    );
}

unsafe extern "win64" fn call(context: *mut c_void) {
    // SAFETY: `on_stack` keeps its body through `switch`.
    unsafe { (*context.cast::<&mut dyn FnMut()>())() };
}

/// Runs `body` on the stack ending at `top`, 16-byte aligned.
///
/// # Safety
/// Nothing else uses the stack below `top` until this returns.
pub(crate) unsafe fn on_stack(top: u64, mut body: &mut dyn FnMut()) {
    // SAFETY: the caller's contract, and the body outlives the call.
    unsafe { switch(call, (&raw mut body).cast(), top) };
}

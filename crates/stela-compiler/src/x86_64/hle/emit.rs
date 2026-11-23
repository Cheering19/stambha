//! What every native routine's fast path is built from: the bail to the
//! original's first instruction, the range check of a guest buffer with the
//! clamp that keeps a mispredicted check from reaching past it, and the
//! success exit to the original's own `ret`.
//!
//! A fast path reads X0 in R15 and X1 in RBP ([`super::super::pinned`]) and
//! the frame through RDI, and takes scratch from RAX, RCX, RDX, R8-R11 and
//! XMM0-XMM15 only. It writes no guest register and no frame field before
//! its commit point, so a bail or a fault leaves the guest state of the
//! function's entry.
use super::super::asm::{Alu, Code, Cond, Gpr, Patch, Size};
use super::super::chain::{self, DirectExit};
use crate::{GuestAddress, block::ExitKind};
use alloc::vec::Vec;

/// A fast path's jumps to the bail, which the composite points at the
/// original's first instruction once the fast path's length is known.
#[derive(Default)]
pub(super) struct Bails(Vec<Patch>);

impl Bails {
    /// Bail now: `jmp rel32`.
    pub(super) fn jump(&mut self, code: &mut Code) {
        let patch = code.asm().jmp();
        self.0.push(patch);
    }

    /// Bail when `cond` holds: `jcc rel32`.
    pub(super) fn when(&mut self, code: &mut Code, cond: Cond) {
        let patch = code.asm().jcc(cond);
        self.0.push(patch);
    }

    /// Point every bail at `target`, the original's first instruction.
    pub(super) fn bind(self, code: &mut Code, target: usize) {
        for patch in self.0 {
            super::super::asm::resolve(code, patch, target);
        }
    }
}

/// Check the guest buffer of `n` bytes at the address in `base` (R15 or
/// RBP) against the frame's `lower_limit`, bailing unless
/// `base < lower_limit && lower_limit - base >= n`, a test that subtracts
/// and never adds, so it cannot wrap; and leave in `into` (R8-R11) the
/// address to dereference. Past each of the two branches a CMOV keyed to
/// the comparison it took makes `into` and `n` 0 where the branch should
/// have bailed, so a mispredicted check reaches no address past the range
/// (as `RangeCheck`'s CMOVA does). `n` is a scratch register; RAX and RDX
/// and the flags are clobbered.
pub(super) fn checked(code: &mut Code, base: Gpr, n: Gpr, into: Gpr, bails: &mut Bails) {
    debug_assert!(matches!(base, Gpr::R15 | Gpr::Rbp));
    debug_assert!(matches!(into, Gpr::R8 | Gpr::R9 | Gpr::R10 | Gpr::R11));
    debug_assert!(!matches!(n, Gpr::Rax | Gpr::Rdx) && n != into);
    {
        let mut asm = code.asm();
        // The clamp's zero, before the comparisons set the flags.
        asm.alu(Alu::Xor, Size::Dword, Gpr::Rdx, Gpr::Rdx);
        asm.mov_load(Size::Qword, Gpr::Rax, super::super::lower_limit());
        asm.mov(Size::Qword, into, base);
        // RAX = lower_limit - base: CF or ZF when base >= lower_limit.
        asm.alu(Alu::Sub, Size::Qword, Gpr::Rax, base);
    }
    bails.when(code, Cond::Be);
    {
        let mut asm = code.asm();
        asm.cmov(Cond::Be, Size::Qword, into, Gpr::Rdx);
        asm.cmov(Cond::Be, Size::Qword, n, Gpr::Rdx);
        // The bytes left below the limit against `n`: CF when too few.
        asm.alu(Alu::Cmp, Size::Qword, Gpr::Rax, n);
    }
    bails.when(code, Cond::B);
    let mut asm = code.asm();
    asm.cmov(Cond::B, Size::Qword, into, Gpr::Rdx);
    asm.cmov(Cond::B, Size::Qword, n, Gpr::Rdx);
}

/// The fast path's success: a direct exit to `exit`, the original's `ret`
/// in the same page and past the entry, after the forward link slot a
/// chain links it into the `ret` block's code through (unpolled, as every
/// forward link is), and the exit it is.
pub(super) fn success(code: &mut Code, exit: GuestAddress) -> DirectExit {
    let mut asm = code.asm();
    let slot = chain::link_slot(asm.linked_block(), exit, ExitKind::Direct);
    let offset = asm.len() + slot;
    super::super::block::direct_exit_with(&mut asm, exit, ExitKind::Direct);
    DirectExit {
        offset,
        target: exit,
        kind: ExitKind::Direct,
        skipped: 0,
    }
}

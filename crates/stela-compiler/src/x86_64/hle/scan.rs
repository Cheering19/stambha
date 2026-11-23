//! strlen: `X0 = s`, returning in X0 the bytes before the first NUL.
//!
//! The string must lie at least `max_bytes` plus a vector below the
//! range's limit, or the call bails. The scan loads aligned 16-byte (or,
//! with AVX2, 32-byte) vectors from `s & !(width - 1)`, masking off the
//! first one's bytes before `s`: an aligned load never crosses a page, so
//! it touches no page the byte-wise original does not. No NUL in the
//! vectors that start below `s + max_bytes` bails, so the scan reads at most
//! `max_bytes` plus two vectors.
use super::super::asm::{Alu, Code, Cond, Gpr, ShiftOp, Size};
use super::Request;
use super::emit::{self, Bails, Isa, Label, at};
use crate::x86_64::chain::DirectExit;
use alloc::{vec, vec::Vec};

pub(super) fn strlen(code: &mut Code, request: Request, bails: &mut Bails) -> Vec<DirectExit> {
    let isa = Isa::of(request.target, Isa::Avx2);
    let width = isa.widest();
    let w = width as i32;
    let s = Gpr::R8;
    // The range [s, s + max_bytes + width) below the limit.
    code.asm().mov_imm32(Gpr::Rcx, request.max_bytes + width);
    emit::checked(code, Gpr::R15, Gpr::Rcx, s, bails);
    isa.zero(code, 0);
    {
        let mut asm = code.asm();
        asm.mov(Size::Qword, Gpr::Rax, s);
        asm.alu_imm32(Alu::And, Size::Qword, Gpr::Rax, -w);
        asm.mov(Size::Dword, Gpr::Rcx, s);
        asm.alu_imm32(Alu::And, Size::Dword, Gpr::Rcx, w - 1);
    }
    nuls(code, isa, width);
    {
        let mut asm = code.asm();
        // The first vector's bytes from s on.
        asm.shift_cl(ShiftOp::Shr, Size::Dword, Gpr::Rdx);
        asm.test(Size::Dword, Gpr::Rdx, Gpr::Rdx);
    }
    let first = code.asm().jcc(Cond::Ne);
    code.asm()
        .lea(Size::Qword, Gpr::R10, at(s, None, request.max_bytes as i32));
    let top = code.asm().len();
    {
        let mut asm = code.asm();
        asm.alu_imm32(Alu::Add, Size::Qword, Gpr::Rax, w);
        asm.alu(Alu::Cmp, Size::Qword, Gpr::Rax, Gpr::R10);
    }
    let mut long = Label::default();
    long.when(code, Cond::Ae);
    nuls(code, isa, width);
    {
        let mut asm = code.asm();
        asm.test(Size::Dword, Gpr::Rdx, Gpr::Rdx);
        let back = asm.jcc(Cond::E);
        asm.bind_back(back, top);
        // The NUL at RAX + its bit, past s.
        asm.bsf(Size::Dword, Gpr::Rdx, Gpr::Rdx);
        asm.alu(Alu::Add, Size::Qword, Gpr::Rax, Gpr::Rdx);
        asm.alu(Alu::Sub, Size::Qword, Gpr::Rax, s);
        // The commit: X0.
        asm.mov(Size::Qword, Gpr::R15, Gpr::Rax);
    }
    let mut done = Label::default();
    done.jump(code);
    code.asm().bind(first);
    {
        let mut asm = code.asm();
        asm.bsf(Size::Dword, Gpr::Rdx, Gpr::Rdx);
        // The commit: X0.
        asm.mov(Size::Qword, Gpr::R15, Gpr::Rdx);
    }
    done.jump(code);
    // No NUL within max_bytes: the original scans on.
    long.bind(code);
    isa.clean(code);
    bails.jump(code);
    done.bind(code);
    isa.clean(code);
    vec![emit::success(code, request.exit)]
}

/// EDX = a bit per NUL of the aligned vector at RAX (vector 0 is zero).
fn nuls(code: &mut Code, isa: Isa, width: u32) {
    isa.load(code, width, 1, at(Gpr::Rax, None, 0));
    isa.equal_bytes(code, width, 1, 0, Gpr::Rdx);
}

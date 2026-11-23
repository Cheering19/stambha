//! bzero (`X0 = dst`, `X1 = n`, no result) and memset (`X0 = dst`,
//! `X1 = c`, `X2 = n`, returning `dst` in X0, which the fast path never
//! writes): `n` bytes at `dst` set to 0, or to `c`'s low byte.
//!
//! The same size classes as memmove's ([`super::copy`]), stores only: a
//! call of at most 256 bytes probes its last byte where it crosses a host
//! page, and a larger one probes every page it covers before the first
//! store, so a fault leaves memory as the call found it. The byte is in
//! every byte of RAX and of vector 0, to the widest vector the host has.
use super::super::asm::{Alu, Code, Cond, Gpr, Size, Unary};
use super::copy::{self, SMALL};
use super::emit::{self, Bails, Isa, Label, at};
use super::{Request, x2};
use crate::x86_64::chain::DirectExit;
use alloc::{vec, vec::Vec};

/// Which fill a fast path is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Fill {
    /// `bzero(dst, n)`.
    Bzero,
    /// `memset(dst, c, n)`.
    Memset,
}

pub(super) fn fill(
    code: &mut Code,
    request: Request,
    fill: Fill,
    bails: &mut Bails,
) -> Vec<DirectExit> {
    let isa = Isa::of(request.target, Isa::Avx512);
    let mut done = Label::default();
    let mut cleaned = Label::default();
    {
        let mut asm = code.asm();
        match fill {
            Fill::Bzero => asm.mov(Size::Qword, Gpr::Rcx, Gpr::Rbp),
            Fill::Memset => asm.mov_load(Size::Qword, Gpr::Rcx, x2()),
        }
        asm.test(Size::Qword, Gpr::Rcx, Gpr::Rcx);
    }
    done.when(code, Cond::E);
    code.asm()
        .alu_imm32(Alu::Cmp, Size::Qword, Gpr::Rcx, request.max_bytes as i32);
    bails.when(code, Cond::A);
    emit::checked(code, Gpr::R15, Gpr::Rcx, Gpr::R8, bails);
    code.asm()
        .alu_imm32(Alu::Cmp, Size::Qword, Gpr::Rcx, SMALL as i32);
    let large = code.asm().jcc(Cond::A);
    emit::probe_crossing(code, Gpr::R8, Gpr::Rcx);
    pattern(code, isa, fill);
    copy::small(code, isa, None, &mut cleaned);
    code.asm().bind(large);
    emit::probe_dst(code, Gpr::R8, Gpr::Rcx);
    pattern(code, isa, fill);
    large_fill(code, isa);
    cleaned.bind(code);
    isa.clean(code);
    done.bind(code);
    vec![emit::success(code, request.exit)]
}

/// The fill's byte in every byte of RAX and of vector 0: 0 for bzero, W1's
/// low byte (RBP's) times `0x0101_0101_0101_0101` for memset. RDX is
/// clobbered.
fn pattern(code: &mut Code, isa: Isa, fill: Fill) {
    {
        let mut asm = code.asm();
        match fill {
            Fill::Bzero => asm.alu(Alu::Xor, Size::Dword, Gpr::Rax, Gpr::Rax),
            Fill::Memset => {
                asm.movzx(Size::Dword, Gpr::Rax, Size::Byte, Gpr::Rbp);
                asm.mov_imm64(Gpr::Rdx, 0x0101_0101_0101_0101);
                asm.imul(Size::Qword, Gpr::Rax, Gpr::Rdx);
            }
        }
    }
    isa.broadcast(code);
}

/// A fill of `n` (RCX, more than 256) bytes at R8 with vector 0, its pages
/// probed: the head and tail vectors, then the destination-aligned ones
/// between.
fn large_fill(code: &mut Code, isa: Isa) {
    let (dst, n) = (Gpr::R8, Gpr::Rcx);
    let width = isa.widest();
    let w = width as i32;
    isa.store(code, width, at(dst, None, 0), 0);
    isa.store(code, width, at(dst, Some(n), -w), 0);
    {
        let mut asm = code.asm();
        asm.mov(Size::Qword, Gpr::Rdx, dst);
        asm.unary(Unary::Neg, Size::Qword, Gpr::Rdx);
        asm.alu_imm32(Alu::And, Size::Qword, Gpr::Rdx, w - 1);
        asm.lea(Size::Qword, Gpr::R10, at(n, None, -w));
        asm.lea(Size::Qword, Gpr::R11, at(n, None, -4 * w));
    }
    copy::forward_loop(code, isa, None);
}

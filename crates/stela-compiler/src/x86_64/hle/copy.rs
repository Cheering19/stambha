//! memmove and memcpy: `X0 = dst`, `X1 = src`, `X2 = n`, returning `dst`
//! in X0, which the fast path never writes. A copy of the same `n` bytes
//! with the same overlap as C's `memmove`, which memcpy's contract allows.
//!
//! - `n == 0` copies nothing and touches no memory.
//! - `n > max_bytes` bails, as does a range check of either buffer
//!   ([`emit::checked`]).
//! - `n <= 256`: a size class of `n` in `[half, 2 * half]` loads the head
//!   and tail halves (`[0, half)` and `[n - half, n)`, which overlap) into
//!   GPRs or up to sixteen vectors, and only then stores them, so any
//!   overlap copies exactly. Where the destination crosses a host page, its
//!   last byte is probed first ([`emit::probe_crossing`]).
//! - `n > 256`: every page of the source is read and every page of the
//!   destination probed ([`emit::probe_src`], [`emit::probe_dst`]) before
//!   the first store, so a fault leaves memory as the call found it. The
//!   head and tail vectors are loaded, then a loop over destination-aligned
//!   vectors runs backward when the destination starts inside the source
//!   (`dst - src < n`, unsigned) and forward otherwise, so it never reads a
//!   source byte it already overwrote; the head and tail are stored last.
use super::super::asm::{Alu, Code, Cond, Gpr, Patch, Size, Unary};
use super::emit::{self, Bails, Isa, Label, at, load_gpr};
use super::{Request, x2};
use crate::x86_64::chain::DirectExit;
use alloc::{vec, vec::Vec};

/// The largest call the unrolled size classes copy.
pub(super) const SMALL: u32 = 256;

pub(super) fn memmove(code: &mut Code, request: Request, bails: &mut Bails) -> Vec<DirectExit> {
    let isa = Isa::of(request.target, Isa::Avx512);
    let mut done = Label::default();
    let mut cleaned = Label::default();
    {
        let mut asm = code.asm();
        asm.mov_load(Size::Qword, Gpr::Rcx, x2());
        asm.test(Size::Qword, Gpr::Rcx, Gpr::Rcx);
    }
    done.when(code, Cond::E);
    code.asm()
        .alu_imm32(Alu::Cmp, Size::Qword, Gpr::Rcx, request.max_bytes as i32);
    bails.when(code, Cond::A);
    emit::checked(code, Gpr::R15, Gpr::Rcx, Gpr::R8, bails);
    emit::checked(code, Gpr::Rbp, Gpr::Rcx, Gpr::R9, bails);
    code.asm()
        .alu_imm32(Alu::Cmp, Size::Qword, Gpr::Rcx, SMALL as i32);
    let large = code.asm().jcc(Cond::A);
    emit::probe_crossing(code, Gpr::R8, Gpr::Rcx);
    small(code, isa, Some(Gpr::R9), &mut cleaned);
    code.asm().bind(large);
    large_copy(code, isa, &mut cleaned);
    cleaned.bind(code);
    isa.clean(code);
    done.bind(code);
    vec![emit::success(code, request.exit)]
}

/// The size classes of a call of `n` (RCX, 1 to 256) bytes to R8: from
/// `source` (R9) for a copy, or of RAX's byte and vector 0's (a fill) when
/// `None`. Each class ends with a jump to `done`.
pub(super) fn small(code: &mut Code, isa: Isa, source: Option<Gpr>, done: &mut Label) {
    // A binary search over the classes' lower bounds, three compares deep.
    let mut classes = Vec::new();
    code.asm().alu_imm32(Alu::Cmp, Size::Qword, Gpr::Rcx, 16);
    let upper = code.asm().jcc(Cond::Ae);
    // n < 16: [1], [2, 4), [4, 8), [8, 16).
    branch(code, 4, &mut classes);
    code.asm().bind(upper);
    // n >= 16: [16, 32), [32, 64), [64, 128), [128, 256].
    branch(code, 64, &mut classes);
    for (patch, half) in classes {
        code.asm().bind(patch);
        class(code, isa, half, source);
        done.jump(code);
    }
}

/// The jumps to the four classes of `n` (RCX) in `[mid / 4, mid * 4)`:
/// `cmp rcx, mid`, then `mid / 2` or `mid * 2` on either side. Each class
/// is named by its `half`, its lowest `n`, and pushed with its jump.
fn branch(code: &mut Code, mid: u32, classes: &mut Vec<(Patch, u32)>) {
    let (below, above) = (mid / 2, mid * 2);
    let mut asm = code.asm();
    asm.alu_imm32(Alu::Cmp, Size::Qword, Gpr::Rcx, mid as i32);
    let high = asm.jcc(Cond::Ae);
    asm.alu_imm32(Alu::Cmp, Size::Qword, Gpr::Rcx, below as i32);
    classes.push((asm.jcc(Cond::Ae), below));
    classes.push((asm.jmp(), below / 2));
    asm.bind(high);
    asm.alu_imm32(Alu::Cmp, Size::Qword, Gpr::Rcx, above as i32);
    classes.push((asm.jcc(Cond::Ae), above));
    classes.push((asm.jmp(), mid));
}

/// The class of `n` (RCX) in `[half, 2 * half]`: the head `[0, half)` and
/// the tail `[n - half, n)`, in GPRs (RAX and RDX) up to 8 bytes and in
/// vectors past that, every load before any store.
fn class(code: &mut Code, isa: Isa, half: u32, source: Option<Gpr>) {
    let tail = -(half as i32);
    let dst = Gpr::R8;
    if half <= 8 {
        let size = emit::gpr_size(half);
        match source {
            Some(src) => {
                load_gpr(code, half, Gpr::Rax, at(src, None, 0));
                load_gpr(code, half, Gpr::Rdx, at(src, Some(Gpr::Rcx), tail));
                let mut asm = code.asm();
                asm.mov_store(size, at(dst, None, 0), Gpr::Rax);
                asm.mov_store(size, at(dst, Some(Gpr::Rcx), tail), Gpr::Rdx);
            }
            None => {
                let mut asm = code.asm();
                asm.mov_store(size, at(dst, None, 0), Gpr::Rax);
                asm.mov_store(size, at(dst, Some(Gpr::Rcx), tail), Gpr::Rax);
            }
        }
        return;
    }
    let width = isa.widest().min(half);
    let count = half / width;
    let chunks: Vec<(i32, Option<Gpr>)> = (0..count)
        .map(|index| ((index * width) as i32, None))
        .chain((0..count).map(|index| (tail + (index * width) as i32, Some(Gpr::Rcx))))
        .collect();
    match source {
        Some(src) => {
            for (register, &(offset, index)) in chunks.iter().enumerate() {
                isa.load(code, width, register as u8, at(src, index, offset));
            }
            for (register, &(offset, index)) in chunks.iter().enumerate() {
                isa.store(code, width, at(dst, index, offset), register as u8);
            }
        }
        None => {
            for &(offset, index) in &chunks {
                isa.store(code, width, at(dst, index, offset), 0);
            }
        }
    }
}

/// A copy of `n` (RCX, more than 256) bytes from R9 to R8: the probes, the
/// head and tail vectors (registers 0 and 1), then the loop in the
/// direction the overlap needs, and the head and tail stored last.
fn large_copy(code: &mut Code, isa: Isa, done: &mut Label) {
    let (dst, src, n) = (Gpr::R8, Gpr::R9, Gpr::Rcx);
    let width = isa.widest();
    let w = width as i32;
    emit::probe_src(code, src, n);
    emit::probe_dst(code, dst, n);
    isa.load(code, width, 0, at(src, None, 0));
    isa.load(code, width, 1, at(src, Some(n), -w));
    {
        let mut asm = code.asm();
        asm.mov(Size::Qword, Gpr::Rax, dst);
        asm.alu(Alu::Sub, Size::Qword, Gpr::Rax, src);
        asm.alu(Alu::Cmp, Size::Qword, Gpr::Rax, n);
    }
    let backward = code.asm().jcc(Cond::B);
    // Forward, from the first destination-aligned vector past dst:
    // RDX = -dst mod width, while RDX < n - width.
    {
        let mut asm = code.asm();
        asm.mov(Size::Qword, Gpr::Rdx, dst);
        asm.unary(Unary::Neg, Size::Qword, Gpr::Rdx);
        asm.alu_imm32(Alu::And, Size::Qword, Gpr::Rdx, w - 1);
        asm.lea(Size::Qword, Gpr::R10, at(n, None, -w));
        asm.lea(Size::Qword, Gpr::R11, at(n, None, -4 * w));
    }
    forward_loop(code, isa, Some(src));
    isa.store(code, width, at(dst, None, 0), 0);
    isa.store(code, width, at(dst, Some(n), -w), 1);
    done.jump(code);
    code.asm().bind(backward);
    // Backward, from the last destination-aligned vector that ends by
    // dst + n: RDX = ((dst + n) & -width) - dst - width, while RDX > 0.
    {
        let mut asm = code.asm();
        asm.lea(Size::Qword, Gpr::Rdx, at(dst, Some(n), 0));
        asm.alu_imm32(Alu::And, Size::Qword, Gpr::Rdx, -w);
        asm.alu(Alu::Sub, Size::Qword, Gpr::Rdx, dst);
        asm.alu_imm32(Alu::Sub, Size::Qword, Gpr::Rdx, w);
    }
    // Four vectors at RDX, RDX - w, RDX - 2w and RDX - 3w while the lowest
    // is past 0 (RDX > 3w), then one at a time while RDX > 0.
    let top = code.asm().len();
    code.asm().alu_imm32(Alu::Cmp, Size::Qword, Gpr::Rdx, 3 * w);
    let single = code.asm().jcc(Cond::Le);
    for index in 0..4u8 {
        let offset = -(index as i32) * w;
        isa.load(code, width, 2 + index, at(src, Some(Gpr::Rdx), offset));
    }
    for index in 0..4u8 {
        let offset = -(index as i32) * w;
        isa.store(code, width, at(dst, Some(Gpr::Rdx), offset), 2 + index);
    }
    {
        let mut asm = code.asm();
        asm.alu_imm32(Alu::Sub, Size::Qword, Gpr::Rdx, 4 * w);
        let back = asm.jmp();
        asm.bind_back(back, top);
        asm.bind(single);
    }
    let one = code.asm().len();
    code.asm().test(Size::Qword, Gpr::Rdx, Gpr::Rdx);
    let finished = code.asm().jcc(Cond::Le);
    isa.load(code, width, 2, at(src, Some(Gpr::Rdx), 0));
    isa.store(code, width, at(dst, Some(Gpr::Rdx), 0), 2);
    {
        let mut asm = code.asm();
        asm.alu_imm32(Alu::Sub, Size::Qword, Gpr::Rdx, w);
        let back = asm.jmp();
        asm.bind_back(back, one);
        asm.bind(finished);
    }
    isa.store(code, width, at(dst, Some(n), -w), 1);
    isa.store(code, width, at(dst, None, 0), 0);
    done.jump(code);
}

/// The forward loop of a large copy (`source` R9) or fill (`None`, vector
/// 0) to R8: four vectors at RDX while RDX < R11 (`n - 4w`), then one while
/// RDX < R10 (`n - w`), signed, each copy's loads before its stores.
pub(super) fn forward_loop(code: &mut Code, isa: Isa, source: Option<Gpr>) {
    let dst = Gpr::R8;
    let width = isa.widest();
    let w = width as i32;
    let top = code.asm().len();
    code.asm().alu(Alu::Cmp, Size::Qword, Gpr::Rdx, Gpr::R11);
    let single = code.asm().jcc(Cond::Ge);
    match source {
        Some(src) => {
            for index in 0..4u8 {
                let offset = index as i32 * w;
                isa.load(code, width, 2 + index, at(src, Some(Gpr::Rdx), offset));
            }
            for index in 0..4u8 {
                let offset = index as i32 * w;
                isa.store(code, width, at(dst, Some(Gpr::Rdx), offset), 2 + index);
            }
        }
        None => {
            for index in 0..4 {
                isa.store(code, width, at(dst, Some(Gpr::Rdx), index * w), 0);
            }
        }
    }
    {
        let mut asm = code.asm();
        asm.alu_imm32(Alu::Add, Size::Qword, Gpr::Rdx, 4 * w);
        let back = asm.jmp();
        asm.bind_back(back, top);
        asm.bind(single);
    }
    let one = code.asm().len();
    code.asm().alu(Alu::Cmp, Size::Qword, Gpr::Rdx, Gpr::R10);
    let finished = code.asm().jcc(Cond::Ge);
    match source {
        Some(src) => {
            isa.load(code, width, 2, at(src, Some(Gpr::Rdx), 0));
            isa.store(code, width, at(dst, Some(Gpr::Rdx), 0), 2);
        }
        None => isa.store(code, width, at(dst, Some(Gpr::Rdx), 0), 0),
    }
    let mut asm = code.asm();
    asm.alu_imm32(Alu::Add, Size::Qword, Gpr::Rdx, w);
    let back = asm.jmp();
    asm.bind_back(back, one);
    asm.bind(finished);
}

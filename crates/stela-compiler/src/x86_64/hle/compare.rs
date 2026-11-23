//! memcmp: `X0 = a`, `X1 = b`, `X2 = n`, returning in X0 the difference of
//! the first bytes that differ, `u32(a[i]) - u32(b[i])` zero-extended (the
//! stand-in's form, `sub w0, w3, w4`), or 0.
//!
//! Loads only, all inside `[a, a + n)` and `[b, b + n)`. Below 16 bytes, a
//! size class's head and tail GPR loads (which overlap) are XORed; from 16,
//! 16-byte (or, with AVX2, 32-byte) vectors at increasing offsets, then one
//! that ends at `n`, are compared with `pcmpeqb`/`pmovmskb`. The lowest
//! differing bit of the first chunk that differs names the byte, which is
//! read again from both buffers for the result. As the original stops at
//! the first difference and these loads may run past it, a load the
//! original would not make may fault: the fault bypasses and the original
//! runs, with the result it gives.
use super::super::asm::{Alu, Code, Cond, Gpr, Size};
use super::emit::{self, Bails, Isa, Label, at, load_gpr};
use super::{Request, x2};
use crate::x86_64::chain::DirectExit;
use alloc::{vec, vec::Vec};

pub(super) fn memcmp(code: &mut Code, request: Request, bails: &mut Bails) -> Vec<DirectExit> {
    let isa = Isa::of(request.target, Isa::Avx2);
    let (a, b, n) = (Gpr::R8, Gpr::R9, Gpr::Rcx);
    let mut equal = Label::default();
    let mut found = Label::default();
    let mut differ = Label::default();
    {
        let mut asm = code.asm();
        asm.mov_load(Size::Qword, n, x2());
        asm.test(Size::Qword, n, n);
    }
    equal.when(code, Cond::E);
    code.asm()
        .alu_imm32(Alu::Cmp, Size::Qword, n, request.max_bytes as i32);
    bails.when(code, Cond::A);
    emit::checked(code, Gpr::R15, n, a, bails);
    emit::checked(code, Gpr::Rbp, n, b, bails);
    code.asm().alu_imm32(Alu::Cmp, Size::Qword, n, 16);
    let vectors = code.asm().jcc(Cond::Ae);
    // n of 1 to 15: [8, 16), [4, 8), [2, 4), [1].
    let mut classes = Vec::new();
    for half in [8, 4, 2] {
        code.asm().alu_imm32(Alu::Cmp, Size::Qword, n, half);
        classes.push((code.asm().jcc(Cond::Ae), half as u32));
    }
    classes.push((code.asm().jmp(), 1));
    for (patch, half) in classes {
        code.asm().bind(patch);
        // The head, at offset 0, then the tail, at n - half: RDX holds the
        // chunk's offset, RAX the XOR of its bytes.
        code.asm().alu(Alu::Xor, Size::Dword, Gpr::Rdx, Gpr::Rdx);
        word(code, half, None);
        differ.when(code, Cond::Ne);
        code.asm()
            .lea(Size::Qword, Gpr::Rdx, at(n, None, -(half as i32)));
        word(code, half, Some(n));
        differ.when(code, Cond::Ne);
        equal.jump(code);
    }
    // A differing GPR chunk: its lowest set bit's byte past RDX.
    differ.bind(code);
    {
        let mut asm = code.asm();
        asm.bsf(Size::Qword, Gpr::Rax, Gpr::Rax);
        asm.shift_imm(super::super::asm::ShiftOp::Shr, Size::Dword, Gpr::Rax, 3);
        asm.alu(Alu::Add, Size::Qword, Gpr::Rdx, Gpr::Rax);
    }
    found.jump(code);
    code.asm().bind(vectors);
    let mut mismatch = Label::default();
    let narrow = if isa >= Isa::Avx2 {
        code.asm().alu_imm32(Alu::Cmp, Size::Qword, n, 32);
        let narrow = code.asm().jcc(Cond::B);
        chunks(code, isa, 32, &mut mismatch, &mut equal);
        Some(narrow)
    } else {
        None
    };
    if let Some(narrow) = narrow {
        code.asm().bind(narrow);
    }
    chunks(code, isa, 16, &mut mismatch, &mut equal);
    // A differing vector chunk at RDX: EAX has a bit per differing byte.
    mismatch.bind(code);
    {
        let mut asm = code.asm();
        asm.bsf(Size::Dword, Gpr::Rax, Gpr::Rax);
        asm.alu(Alu::Add, Size::Qword, Gpr::Rdx, Gpr::Rax);
    }
    found.bind(code);
    {
        let mut asm = code.asm();
        asm.movzx(Size::Dword, Gpr::Rax, Size::Byte, at(a, Some(Gpr::Rdx), 0));
        asm.movzx(Size::Dword, Gpr::R10, Size::Byte, at(b, Some(Gpr::Rdx), 0));
        asm.alu(Alu::Sub, Size::Dword, Gpr::Rax, Gpr::R10);
        // The commit: X0, zero-extended from its low word.
        asm.mov(Size::Dword, Gpr::R15, Gpr::Rax);
    }
    let mut done = Label::default();
    done.jump(code);
    equal.bind(code);
    code.asm().alu(Alu::Xor, Size::Dword, Gpr::R15, Gpr::R15);
    done.bind(code);
    isa.clean(code);
    vec![emit::success(code, request.exit)]
}

/// RAX = the `half` bytes of `a` XOR those of `b` at RDX (`n - half` for
/// the tail, `index` RCX with a displacement, else 0): ZF when they agree.
fn word(code: &mut Code, half: u32, index: Option<Gpr>) {
    let offset = if index.is_some() { -(half as i32) } else { 0 };
    load_gpr(code, half, Gpr::Rax, at(Gpr::R8, index, offset));
    load_gpr(code, half, Gpr::R10, at(Gpr::R9, index, offset));
    let size = if half == 8 { Size::Qword } else { Size::Dword };
    code.asm().alu(Alu::Xor, size, Gpr::Rax, Gpr::R10);
}

/// Compare `bytes`-wide chunks (n >= bytes) at RDX = 0, `bytes`, ... while
/// the chunk ends before `n`, then the one ending at `n`, jumping to
/// `mismatch` with RDX at a chunk that differs and EAX a bit per differing
/// byte, or to `equal`.
fn chunks(code: &mut Code, isa: Isa, bytes: u32, mismatch: &mut Label, equal: &mut Label) {
    let all = if bytes == 32 { -1 } else { 0xffff };
    {
        let mut asm = code.asm();
        asm.alu(Alu::Xor, Size::Dword, Gpr::Rdx, Gpr::Rdx);
        asm.lea(Size::Qword, Gpr::R11, at(Gpr::Rcx, None, -(bytes as i32)));
    }
    let top = code.asm().len();
    code.asm().alu(Alu::Cmp, Size::Qword, Gpr::Rdx, Gpr::R11);
    let last = code.asm().jcc(Cond::Ae);
    chunk(code, isa, bytes, all);
    mismatch.when(code, Cond::Ne);
    {
        let mut asm = code.asm();
        asm.alu_imm32(Alu::Add, Size::Qword, Gpr::Rdx, bytes as i32);
        let back = asm.jmp();
        asm.bind_back(back, top);
        asm.bind(last);
        asm.mov(Size::Qword, Gpr::Rdx, Gpr::R11);
    }
    chunk(code, isa, bytes, all);
    mismatch.when(code, Cond::Ne);
    equal.jump(code);
}

/// EAX = a bit per byte of the chunk at RDX that differs: ZF when none.
fn chunk(code: &mut Code, isa: Isa, bytes: u32, all: i32) {
    isa.load(code, bytes, 0, at(Gpr::R8, Some(Gpr::Rdx), 0));
    isa.load(code, bytes, 1, at(Gpr::R9, Some(Gpr::Rdx), 0));
    isa.equal_bytes(code, bytes, 0, 1, Gpr::Rax);
    code.asm().alu_imm32(Alu::Xor, Size::Dword, Gpr::Rax, all);
}

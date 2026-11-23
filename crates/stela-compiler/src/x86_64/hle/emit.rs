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
use super::super::asm::{Alu, Code, Cond, Evex, Gpr, Map, Mem, Patch, Pp, Rm, Size, Vex};
use super::super::chain::{self, DirectExit};
use crate::{GuestAddress, TargetCpu, block::ExitKind};
use alloc::vec::Vec;

/// The bytes of a host page, the granule a probe covers: no guest page is
/// smaller, and the shadow maps and protects the host's.
pub(super) const HOST_PAGE: i32 = 4096;

/// The vector encodings a fast path uses, by what the host has: legacy SSE2
/// (16-byte XMM), AVX2 (VEX, up to 32-byte YMM) or AVX-512F (EVEX, up to
/// 64-byte ZMM). With AVX2 or AVX-512 every vector instruction is VEX or
/// EVEX encoded, 16-byte ones included, so no legacy SSE form runs with
/// the upper halves dirty, and the fast path ends each way out of it that
/// follows a vector instruction with `vzeroupper` ([`Self::clean`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum Isa {
    Sse2,
    Avx2,
    Avx512,
}

impl Isa {
    /// The widest set `target` has, no wider than `cap`.
    pub(super) fn of(target: TargetCpu, cap: Isa) -> Self {
        let isa = if target.supports(TargetCpu::Avx512f) {
            Self::Avx512
        } else if target.supports(TargetCpu::Avx2) {
            Self::Avx2
        } else {
            Self::Sse2
        };
        isa.min(cap)
    }

    /// The bytes of its widest vector.
    pub(super) const fn widest(self) -> u32 {
        match self {
            Self::Sse2 => 16,
            Self::Avx2 => 32,
            Self::Avx512 => 64,
        }
    }

    /// `vzeroupper` where a vector instruction may have dirtied the upper
    /// halves, before a way out of the fast path.
    pub(super) fn clean(self, code: &mut Code) {
        if self != Self::Sse2 {
            code.asm().vzeroupper();
        }
    }

    const fn vex(self, l: bool, map: Map, pp: Pp) -> Vex {
        Vex {
            map,
            pp,
            w: false,
            l,
        }
    }

    const fn evex512(map: Map, pp: Pp, w: bool, scale: u8) -> Evex {
        Evex {
            map,
            pp,
            w,
            length: 2,
            mask: 0,
            zeroing: false,
            broadcast: false,
            scale,
        }
    }

    /// `movdqu`/`vmovdqu`/`vmovdqu64` of `bytes` (16, 32 or 64) from
    /// `memory` into vector `register`.
    pub(super) fn load(self, code: &mut Code, bytes: u32, register: u8, memory: Mem) {
        self.move_vector(code, bytes, register, memory, 0x6f);
    }

    /// The same, from vector `register` to `memory`.
    pub(super) fn store(self, code: &mut Code, bytes: u32, memory: Mem, register: u8) {
        self.move_vector(code, bytes, register, memory, 0x7f);
    }

    fn move_vector(self, code: &mut Code, bytes: u32, register: u8, memory: Mem, opcode: u8) {
        debug_assert!(bytes <= self.widest() && register < 16);
        let mut asm = code.asm();
        match (bytes, self) {
            (16, Self::Sse2) if opcode == 0x6f => asm.movdqu_load(register, memory),
            (16, Self::Sse2) => asm.movdqu_store(memory, register),
            (16 | 32, _) => asm.vex(
                self.vex(bytes == 32, Map::M0f, Pp::Pf3),
                opcode,
                register,
                0,
                Rm::Mem(memory),
            ),
            (64, Self::Avx512) => asm.evex(
                Self::evex512(Map::M0f, Pp::Pf3, true, 64),
                opcode,
                register,
                0,
                Rm::Mem(memory),
            ),
            _ => unreachable!("a {bytes}-byte vector of {self:?}"),
        }
    }

    /// Every byte of vector 0, to its widest, the low byte of RAX, which
    /// holds that byte in each of its own.
    pub(super) fn broadcast(self, code: &mut Code) {
        let mut asm = code.asm();
        match self {
            Self::Sse2 => {
                asm.movq_to_xmm(0, Gpr::Rax);
                asm.punpcklqdq(0, Rm::Xmm(0));
            }
            Self::Avx2 => {
                // vmovq xmm0, rax (VEX.128.66.0F.W1 6E), then
                // vpbroadcastq ymm0, xmm0 (VEX.256.66.0F38.W0 59).
                asm.vex(
                    Vex {
                        w: true,
                        ..self.vex(false, Map::M0f, Pp::P66)
                    },
                    0x6e,
                    0,
                    0,
                    Rm::Reg(Gpr::Rax),
                );
                asm.vex(self.vex(true, Map::M0f38, Pp::P66), 0x59, 0, 0, Rm::Xmm(0));
            }
            Self::Avx512 => {
                asm.vex(
                    Vex {
                        w: true,
                        ..self.vex(false, Map::M0f, Pp::P66)
                    },
                    0x6e,
                    0,
                    0,
                    Rm::Reg(Gpr::Rax),
                );
                // vpbroadcastq zmm0, xmm0 (EVEX.512.66.0F38.W1 59).
                asm.evex(
                    Self::evex512(Map::M0f38, Pp::P66, true, 8),
                    0x59,
                    0,
                    0,
                    Rm::Xmm(0),
                );
            }
        }
    }

    /// Vector `register` zeroed to its widest (`pxor`, or VEX.128 `vpxor`,
    /// which zeroes the bits above).
    pub(super) fn zero(self, code: &mut Code, register: u8) {
        let mut asm = code.asm();
        match self {
            Self::Sse2 => asm.sse(Some(0x66), &[0x0f, 0xef], register, Rm::Xmm(register)),
            _ => asm.vex(
                self.vex(false, Map::M0f, Pp::P66),
                0xef,
                register,
                register,
                Rm::Xmm(register),
            ),
        }
    }

    /// `pcmpeqb` of vectors `left` and `right` into `left`, then its byte
    /// sign mask into `mask`: a bit per equal byte, for `bytes` (16 or
    /// 32) of them.
    pub(super) fn equal_bytes(self, code: &mut Code, bytes: u32, left: u8, right: u8, mask: Gpr) {
        debug_assert!(bytes == 16 || (bytes == 32 && self >= Self::Avx2));
        let mut asm = code.asm();
        match self {
            Self::Sse2 => {
                asm.sse(Some(0x66), &[0x0f, 0x74], left, Rm::Xmm(right));
                asm.sse(Some(0x66), &[0x0f, 0xd7], mask.number(), Rm::Xmm(left));
            }
            _ => {
                let l = bytes == 32;
                asm.vex(
                    self.vex(l, Map::M0f, Pp::P66),
                    0x74,
                    left,
                    left,
                    Rm::Xmm(right),
                );
                asm.vex(
                    self.vex(l, Map::M0f, Pp::P66),
                    0xd7,
                    mask.number(),
                    0,
                    Rm::Xmm(left),
                );
            }
        }
    }
}

/// `[base + index + displacement]`.
pub(super) fn at(base: Gpr, index: Option<Gpr>, displacement: i32) -> Mem {
    match index {
        Some(index) => Mem::indexed(base, index, 1, displacement),
        None => Mem::base(base, displacement),
    }
}

/// The GPR access of `bytes` (1, 2, 4 or 8).
pub(super) fn gpr_size(bytes: u32) -> Size {
    match bytes {
        1 => Size::Byte,
        2 => Size::Word,
        4 => Size::Dword,
        8 => Size::Qword,
        _ => unreachable!("a {bytes}-byte GPR access"),
    }
}

/// Load `bytes` (1, 2, 4 or 8) at `memory` into `into`, zero-extended.
pub(super) fn load_gpr(code: &mut Code, bytes: u32, into: Gpr, memory: Mem) {
    let mut asm = code.asm();
    match bytes {
        1 | 2 => asm.movzx(Size::Dword, into, gpr_size(bytes), memory),
        _ => asm.mov_load(gpr_size(bytes), into, memory),
    }
}

/// Read one byte of each host page the `n` bytes at `base` (R8-R11, a
/// checked buffer) touch, from the buffer's first, before anything is
/// stored: the byte at `base` and each page boundary inside the buffer,
/// none past its end. A page that cannot be read faults here, with guest
/// memory as the call found it. RAX, RDX and R10 are clobbered.
pub(super) fn probe_src(code: &mut Code, base: Gpr, n: Gpr) {
    probe(code, base, n, false);
}

/// The same for a buffer the call stores to: a value-preserving `lock or
/// byte [page byte], 0` on each page, which faults where the page cannot be
/// written (or read) and changes nothing where it can, atomically, so no
/// other PE's store between a load and a store back is lost.
pub(super) fn probe_dst(code: &mut Code, base: Gpr, n: Gpr) {
    probe(code, base, n, true);
}

fn probe(code: &mut Code, base: Gpr, n: Gpr, write: bool) {
    let top = {
        let mut asm = code.asm();
        if write {
            asm.alu(Alu::Xor, Size::Dword, Gpr::Rdx, Gpr::Rdx);
        }
        asm.mov(Size::Qword, Gpr::Rax, base);
        asm.lea(Size::Qword, Gpr::R10, at(base, Some(n), 0));
        asm.len()
    };
    let mut asm = code.asm();
    if write {
        asm.lock();
        asm.alu(Alu::Or, Size::Byte, Mem::base(Gpr::Rax, 0), Gpr::Rdx);
    } else {
        asm.movzx(Size::Dword, Gpr::Rdx, Size::Byte, Mem::base(Gpr::Rax, 0));
    }
    // The next page's first byte.
    asm.alu_imm32(Alu::And, Size::Qword, Gpr::Rax, -HOST_PAGE);
    asm.alu_imm32(Alu::Add, Size::Qword, Gpr::Rax, HOST_PAGE);
    asm.alu(Alu::Cmp, Size::Qword, Gpr::Rax, Gpr::R10);
    let back = asm.jcc(Cond::B);
    asm.bind_back(back, top);
}

/// Where the `n` bytes at `base` (n of 1 to 256, a checked buffer) cross a
/// host page boundary, probe the last of them as [`probe_dst`] does, so
/// that a store to either page faults before any byte changes: a buffer of
/// at most 256 bytes touches at most two pages, and the first store of a
/// fast path is to the first. RAX and RDX are clobbered.
pub(super) fn probe_crossing(code: &mut Code, base: Gpr, n: Gpr) {
    let last = at(base, Some(n), -1);
    let mut asm = code.asm();
    asm.lea(Size::Qword, Gpr::Rax, last);
    asm.alu(Alu::Xor, Size::Qword, Gpr::Rax, base);
    asm.alu_imm32(Alu::Cmp, Size::Qword, Gpr::Rax, HOST_PAGE);
    let within = asm.jcc(Cond::B);
    asm.alu(Alu::Xor, Size::Dword, Gpr::Rdx, Gpr::Rdx);
    asm.lock();
    asm.alu(Alu::Or, Size::Byte, last, Gpr::Rdx);
    asm.bind(within);
}

/// Forward jumps to one label.
#[derive(Default)]
pub(super) struct Label(Vec<Patch>);

impl Label {
    /// `jmp` to the label.
    pub(super) fn jump(&mut self, code: &mut Code) {
        let patch = code.asm().jmp();
        self.0.push(patch);
    }

    /// `jcc` to the label.
    pub(super) fn when(&mut self, code: &mut Code, cond: Cond) {
        let patch = code.asm().jcc(cond);
        self.0.push(patch);
    }

    /// The label is here.
    pub(super) fn bind(self, code: &mut Code) {
        let mut asm = code.asm();
        for patch in self.0 {
            asm.bind(patch);
        }
    }
}

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

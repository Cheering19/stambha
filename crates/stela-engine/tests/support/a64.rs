//! A64 encoders for the HLE stand-ins (Arm ARM C4.1): one function per
//! form they use. Registers are numbers, 31 naming XZR/WZR; branch offsets
//! are in words from the branch.

pub const NOP: u32 = 0xd503_201f;
pub const RET: u32 = 0xd65f_03c0;
pub const BTI_C: u32 = 0xd503_245f;
pub const BLR_X16: u32 = 0xd63f_0200;
pub const ZR: u32 = 31;

/// Condition codes.
pub const EQ: u32 = 0b0000;
pub const NE: u32 = 0b0001;
pub const HS: u32 = 0b0010;
pub const LO: u32 = 0b0011;

fn imm9(offset: i32) -> u32 {
    assert!((-256..256).contains(&offset), "imm9 {offset}");
    (offset as u32 & 0x1ff) << 12
}

fn words19(words: i32) -> u32 {
    assert!((-(1 << 18)..1 << 18).contains(&words), "imm19 {words}");
    (words as u32 & 0x7ffff) << 5
}

/// `ldrb wT,[xN]`.
pub fn ldrb(rt: u32, rn: u32) -> u32 {
    0x3940_0000 | rn << 5 | rt
}
/// `ldrb wT,[xN],#offset` (post-index).
pub fn ldrb_post(rt: u32, rn: u32, offset: i32) -> u32 {
    0x3840_0400 | imm9(offset) | rn << 5 | rt
}
/// `ldrb wT,[xN,#offset]!` (pre-index).
pub fn ldrb_pre(rt: u32, rn: u32, offset: i32) -> u32 {
    0x3840_0c00 | imm9(offset) | rn << 5 | rt
}
/// `strb wT,[xN],#offset` (post-index).
pub fn strb_post(rt: u32, rn: u32, offset: i32) -> u32 {
    0x3800_0400 | imm9(offset) | rn << 5 | rt
}
/// `strb wT,[xN,#offset]!` (pre-index).
pub fn strb_pre(rt: u32, rn: u32, offset: i32) -> u32 {
    0x3800_0c00 | imm9(offset) | rn << 5 | rt
}
/// `subs xD, xN, #immediate`.
pub fn subs_imm(rd: u32, rn: u32, immediate: u32) -> u32 {
    assert!(immediate < 1 << 12);
    0xf100_0000 | immediate << 10 | rn << 5 | rd
}
/// `subs wD, wN, wM`.
pub fn subs_w(rd: u32, rn: u32, rm: u32) -> u32 {
    0x6b00_0000 | rm << 16 | rn << 5 | rd
}
/// `sub xD, xN, xM`.
pub fn sub(rd: u32, rn: u32, rm: u32) -> u32 {
    0xcb00_0000 | rm << 16 | rn << 5 | rd
}
/// `add xD, xN, xM`.
pub fn add(rd: u32, rn: u32, rm: u32) -> u32 {
    0x8b00_0000 | rm << 16 | rn << 5 | rd
}
/// `cmp xN, xM` (`subs xzr, xN, xM`).
pub fn cmp(rn: u32, rm: u32) -> u32 {
    0xeb00_0000 | rm << 16 | rn << 5 | ZR
}
/// `mov xD, xM` (`orr xD, xzr, xM`).
pub fn mov(rd: u32, rm: u32) -> u32 {
    0xaa00_03e0 | rm << 16 | rd
}
/// `mov wD, wM` (`orr wD, wzr, wM`), which zero-extends into xD.
pub fn mov_w(rd: u32, rm: u32) -> u32 {
    0x2a00_03e0 | rm << 16 | rd
}
/// `mov wD, #immediate` (`movz`).
pub fn movz_w(rd: u32, immediate: u32) -> u32 {
    assert!(immediate < 1 << 16);
    0x5280_0000 | immediate << 5 | rd
}
/// `cbz xT` by `words`.
pub fn cbz(rt: u32, words: i32) -> u32 {
    0xb400_0000 | words19(words) | rt
}
/// `cbz wT` by `words`.
pub fn cbz_w(rt: u32, words: i32) -> u32 {
    0x3400_0000 | words19(words) | rt
}
/// `cbnz wT` by `words`.
pub fn cbnz_w(rt: u32, words: i32) -> u32 {
    0x3500_0000 | words19(words) | rt
}
/// `b.cond` by `words`.
pub fn b_cond(condition: u32, words: i32) -> u32 {
    0x5400_0000 | words19(words) | condition
}
/// `b` by `words`.
pub fn b(words: i32) -> u32 {
    assert!((-(1 << 25)..1 << 25).contains(&words), "imm26 {words}");
    0x1400_0000 | (words as u32 & 0x03ff_ffff)
}

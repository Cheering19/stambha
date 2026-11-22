//! Host-native paths for AMX operations, exact against [`super::model`].
//!
//! The reference model computes every lane with Stela's software Arm
//! evaluators, which costs microseconds for one matrix `fma32`. These paths
//! compute the same lanes with host vector instructions and return `false`
//! for any operand they do not cover, leaving the registers unchanged, so the
//! caller then runs the model: the model stays the definition and the
//! fallback for every rare mode.
//!
//! Covered, chosen once from the [`TargetCpu`] (`Native::select`):
//!
//! - `fma32`, `fms32`, `fma64` and `fms64` in matrix and vector shape,
//!   accumulating, product-only or add-only (bits 29:27 = 0, 1, or 2 and 4:
//!   `z ± x` and `z ± y`, a multiply-add by 1.0, which is exact, so the sum
//!   rounds once, signed zeros included), every lane-enable mode, and
//!   `fma32`'s even-f16 widening of X and Y (bits 61/60). Tiers:
//!   `Avx512` (EVEX.512, one ZMM per row, opmask lane enables; AVX-512F with
//!   OS-enabled opmask and ZMM state), `Fma` (VEX.256 with FMA3, two YMM
//!   halves per row, F16C for the widening, without which widening operands
//!   use the model), and below FMA3 the model: a multiply-add rounded once
//!   cannot be built exactly from SSE multiplies and adds.
//! - `vecfp` on IEEE types, one Z row or from the M2 the multi-row form
//!   (bit 31) row by row, its operands staged by the model's own shuffles,
//!   indexed loads and the M4's aligned offsets, a
//!   broadcast Y lane or any lane-enable mode, the
//!   zero-operand and zero-writing lane values: modes 0/1 (`z ± x*y`) and
//!   from the M2 10-12 (`x*y`, `z + x`, `z + y`) on f16 lanes with F16C,
//!   on f32 and f64 lanes with the FMA row kernels, on f16 into f32 (type
//!   3) with F16C; modes 4 (`x <= 0 ? 0 : y`), 5 and 7 (FMIN and FMAX with
//!   the default NaN, -0 below +0) as integer bit selections on every tier.
//!   The f16 rows (VEX.128, four lanes per step; on the `Avx512` tier
//!   EVEX.512, sixteen lanes per step with opmask enables): the product of
//!   two f16 values is exact in f32, the f32 sum is rounded to odd from its
//!   exact TwoSum error, and the conversion to f16 then rounds once
//!   correctly (24 >= 11 + 2 bits).
//! - `fma16` and `fms16` with F16C, in vector, f16 matrix and bit-62
//!   widened (f32 row pairs) matrix shape, accumulating, product-only or
//!   add-only (bits 29:27 = 0, 1, 2 or 4), every lane-enable mode: the same
//!   once-rounded f16 sum (the product alone is added to -0, `z ± x` and
//!   `z ± y` multiply by 1.0), and in the widened shape one f32 addition of
//!   the exact product.
//! - `mac16` on the `Avx512` tier, every operand: vector, matrix and
//!   bit-62 widened (i32 row pairs) shape, int8 inputs (bits 61/60), each
//!   selection of bits 29:28, the shift of bits 59:55, with or without Z
//!   (bit 27), every lane-enable mode. Integer lanes in i32 with Z's wrap,
//!   so exact.
//! - `matfp` on IEEE types on the multiply-add tiers: f32 and f64 (types 4
//!   and 7) through the FMA kernels, f16 (and its widening into f32, type
//!   3) through the f16 and widened rows with F16C, modes 0/1 (`z ± x*y`);
//!   mode 4 (`x <= 0 ? 0 : y`, IEEE order on the bits) and the lane value
//!   that writes zeros as bit selections; every lane mode; operands staged
//!   by the model's own shuffles and indexed loads. Not bf16, the no-op
//!   modes or bits 56:54.
//! - On every tier, the model's too, the fused operations' copies and
//!   constants (bits 29:27 = 3, 5, 6, 7), which need no arithmetic, except
//!   copies of f16-widened halves, whose NaN payloads the model rewrites,
//!   and `vecfp`'s bit selections.
//!
//! The kernels are inline assembly, so they run in the boot, whose UEFI
//! target is soft-float (vector intrinsics would become software calls).
//!
//! Arithmetic runs under MXCSR 0x1F80 (round to nearest even, no DAZ/FTZ,
//! exceptions masked) and restores the caller's MXCSR, so host status flags
//! never escape. x86 and Arm agree on every non-NaN fused, product and sign
//! result in that mode, and a result is NaN on one exactly when it is on the
//! other; NaN results become the Arm default NaN, as the AMX environment
//! (FPCR.DN) gives.
use super::{AmxOperation, AmxRegisters, Generation};
#[cfg(target_arch = "x86_64")]
use super::{model::write_mask, state::ROW_BYTES};
use crate::TargetCpu;

#[cfg(target_arch = "x86_64")]
const VECTOR: u64 = 1 << 63;
/// Fused-op modes (bits 29:27) that copy an operand or write a constant:
/// x or -x (3), y or -y (5), z unchanged (6), +0 or -0 (7).
#[cfg(target_arch = "x86_64")]
const fn copies(mode: u64) -> bool {
    matches!(mode, 3 | 5 | 6 | 7)
}

/// `fma16`/`fms16` bit 62: f32 accumulation into interleaved row pairs.
#[cfg(target_arch = "x86_64")]
const WIDEN_16_32: u64 = 1 << 62;
#[cfg(target_arch = "x86_64")]
const X_HALF: u64 = 1 << 61;
#[cfg(target_arch = "x86_64")]
const Y_HALF: u64 = 1 << 60;

/// The widest host path that computes the covered operations exactly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Tier {
    Model,
    Fma,
    Avx512,
}

impl Tier {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Model => "model",
            Self::Fma => "fma",
            Self::Avx512 => "avx512f",
        }
    }
}

/// The host paths one boot uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Native {
    /// The fused-multiply tier.
    tier: Tier,
    /// VEX F16C: `fma32`'s f16 widening on the `Fma` tier, and `vecfp` f16.
    f16c: bool,
}

impl Native {
    /// The model tier: no host arithmetic, so the model computes every
    /// operation except the fused operations' copies and constants and
    /// `vecfp`'s bit selections, which this tier still completes on the
    /// host (and, with F16C, the f16 rows). To leave every operation to
    /// the model, do not call [`Self::execute`] (the engine's
    /// `--amx-native off`).
    pub const MODEL: Self = Self {
        tier: Tier::Model,
        f16c: false,
    };

    /// The best paths `target` admits. AVX-512F has its own EVEX f16
    /// conversions; the VEX paths need F16C.
    pub const fn select(target: TargetCpu) -> Self {
        if !cfg!(target_arch = "x86_64") {
            return Self::MODEL;
        }
        let tier = if target.supports(TargetCpu::Avx512f) {
            Tier::Avx512
        } else if target.supports(TargetCpu::Fma) {
            Tier::Fma
        } else {
            Tier::Model
        };
        Self {
            tier,
            f16c: target.supports(TargetCpu::F16c),
        }
    }

    pub const fn tier(self) -> Tier {
        self.tier
    }

    /// Complete `operation` natively when a path covers `operand`, exactly as
    /// `model::execute_for(generation, ..)` would; `false` leaves the
    /// registers untouched for the model.
    pub fn execute(
        self,
        generation: Generation,
        registers: &mut AmxRegisters,
        operation: AmxOperation,
        operand: u64,
    ) -> bool {
        #[cfg(not(target_arch = "x86_64"))]
        let _ = (generation, registers, operand);
        match operation {
            #[cfg(target_arch = "x86_64")]
            AmxOperation::Fma32
            | AmxOperation::Fms32
            | AmxOperation::Fma64
            | AmxOperation::Fms64 => {
                // Copies and constants (bits 29:27 of 3, 5, 6, 7) need no FMA.
                (self.tier != Tier::Model || copies((operand >> 27) & 7))
                    && x86::fused(self, registers, operation, operand)
            }
            #[cfg(target_arch = "x86_64")]
            AmxOperation::Fma16 | AmxOperation::Fms16 => {
                (self.f16c || copies((operand >> 27) & 7))
                    && x86::fused_half(self.tier, registers, operation, operand)
            }
            #[cfg(target_arch = "x86_64")]
            AmxOperation::VectorFloat => x86::vector_float(self, generation, registers, operand),
            #[cfg(target_arch = "x86_64")]
            AmxOperation::Mac16 => self.tier == Tier::Avx512 && x86::mac16(registers, operand),
            #[cfg(target_arch = "x86_64")]
            AmxOperation::MatrixFloat => x86::matrix_float(self, generation, registers, operand),
            _ => false,
        }
    }
}

/// The decoded form of a covered fused operation.
#[cfg_attr(not(target_arch = "x86_64"), allow(dead_code))]
struct Fused {
    vector: bool,
    z_row: usize,
    /// Per-byte lane enables (`write_mask`): lane i is on when bit
    /// `i * element bytes` is.
    x_enable: u64,
    y_enable: u64,
}

#[cfg(target_arch = "x86_64")]
mod x86 {
    //! The kernels are inline assembly with explicit registers: the UEFI
    //! target is soft-float, where vector intrinsics would become software
    //! calls. Each block is self-contained (no vector value lives across
    //! blocks) and declares the SysV clobbers, which on the soft-float
    //! target name registers its own code never uses.
    use super::{Fused, Native, ROW_BYTES, Tier, VECTOR, X_HALF, Y_HALF, write_mask};
    use crate::amx::{AmxOperation, AmxRegisters, Generation, model};
    use core::arch::asm;

    type Row = [u8; ROW_BYTES];

    static AMX_MXCSR: u32 = 0x1f80;
    static NAN_32: u32 = 0x7fc0_0000;
    static NAN_64: u64 = 0x7ff8_0000_0000_0000;
    static NEGATIVE_ZERO_32: u32 = 0x8000_0000;
    static NEGATIVE_ZERO_64: u64 = 0x8000_0000_0000_0000;

    /// A row of `bytes`-byte lanes each holding `value`.
    const fn splat(value: u64, bytes: usize) -> Row {
        // One eight-byte word of the lanes, stored eight times.
        let word = match bytes {
            2 => (value & 0xffff) * 0x0001_0001_0001_0001,
            4 => (value & 0xffff_ffff) * 0x0000_0001_0000_0001,
            _ => value,
        }
        .to_le_bytes();
        let mut row = [0u8; ROW_BYTES];
        let mut at = 0;
        while at < ROW_BYTES {
            row[at] = word[0];
            row[at + 1] = word[1];
            row[at + 2] = word[2];
            row[at + 3] = word[3];
            row[at + 4] = word[4];
            row[at + 5] = word[5];
            row[at + 6] = word[6];
            row[at + 7] = word[7];
            at += 8;
        }
        row
    }

    /// 1.0 in every f32 and every f64 lane.
    const ONES_32: Row = splat(0x3f80_0000, 4);
    const ONES_64: Row = splat(0x3ff0_0000_0000_0000, 8);

    /// The four fused forms.
    #[derive(Clone, Copy)]
    enum Form {
        /// `z + x*y`.
        Add,
        /// `z - x*y`.
        Subtract,
        /// `x*y`.
        Product,
        /// `-0 - x*y`: the negated product.
        NegatedProduct,
    }

    pub(super) fn fused(
        native: Native,
        registers: &mut AmxRegisters,
        operation: AmxOperation,
        operand: u64,
    ) -> bool {
        let (double, subtract) = match operation {
            AmxOperation::Fma32 => (false, false),
            AmxOperation::Fms32 => (false, true),
            AmxOperation::Fma64 => (true, false),
            AmxOperation::Fms64 => (true, true),
            _ => return false,
        };
        let mode = (operand >> 27) & 7;
        let product_only = mode == 1;
        // Only fma32/fms32 read bits 61/60; f64 ignores them.
        let x_half = !double && operand & X_HALF != 0;
        let y_half = !double && operand & Y_HALF != 0;
        let bytes = if double { 8 } else { 4 };
        if super::copies(mode) {
            // Copies and constants: no arithmetic. Widened copies keep
            // NaN payloads the model negates for fms32: left to it.
            let copies_half = match mode {
                3 => x_half,
                5 => y_half,
                _ => false,
            };
            return !copies_half && skip(registers, operand, bytes, mode, subtract, Shape::Plain);
        }
        if (x_half || y_half) && native.tier == Tier::Fma && !native.f16c {
            return false;
        }
        let fused = Fused {
            vector: operand & VECTOR != 0,
            z_row: ((operand >> 20) & 63) as usize,
            x_enable: write_mask(operand >> 41, bytes, 7),
            y_enable: write_mask(operand >> 32, bytes, 7),
        };
        let form = match (product_only, subtract) {
            (false, false) => Form::Add,
            (false, true) => Form::Subtract,
            (true, false) => Form::Product,
            (true, true) => Form::NegatedProduct,
        };
        let mut x = registers.x_operand((operand >> 10) & 0x1ff);
        let mut y = registers.y_operand(operand & 0x1ff);
        let bytes = bytes as usize;
        let lanes = 64 / bytes;
        // Per-lane all-ones masks (VEX) or the opmask (EVEX) of X's lanes;
        // with every lane on, as most operands have, no loop.
        let (mask, opmask) = lane_masks(fused.x_enable, bytes);
        // SAFETY: `select` chose the tier from a target whose features the
        // host admits, and F16C is checked above for VEX widening. Every
        // pointer is to a live 64-byte row; MXCSR is restored after.
        unsafe {
            let saved = enter();
            if x_half {
                x = widen(native.tier, &x);
            }
            if y_half {
                y = widen(native.tier, &y);
            }
            // z ± x (bits 29:27 = 2) and z ± y (4): a multiply-add by 1.0,
            // whose product is exact, so the sum rounds once to the model's
            // add or subtract, signed zeros and default NaNs included.
            let one = if double { ONES_64 } else { ONES_32 };
            match mode {
                2 => y = one,
                4 => x = one,
                _ => {}
            }
            let mut row = |index: usize, y: &Row| {
                let z = registers.z[index].as_mut_ptr();
                match native.tier {
                    Tier::Avx512 => row_avx512(double, form, z, &x, y, opmask),
                    _ => row_fma(double, form, z, &x, y, &mask),
                }
            };
            if fused.vector {
                row(fused.z_row, &y);
            } else {
                // Y lane j into Z row j*g + z_row%g, g = 64/lanes, in one
                // kernel loop over the enabled Y lanes.
                let g = 64 / lanes;
                let rows = lane_bits(fused.y_enable, bytes);
                let matrix = Matrix {
                    z: registers.z[fused.z_row & (g - 1)].as_mut_ptr(),
                    stride: g * ROW_BYTES,
                    rows,
                    lanes,
                };
                match native.tier {
                    Tier::Avx512 => matrix_avx512(double, form, matrix, &x, &y, opmask),
                    _ => matrix_fma(double, form, matrix, &x, &y, &mask),
                }
            }
            leave(saved);
        }
        true
    }

    /// Where a fused operation's lanes go.
    #[derive(Clone, Copy, PartialEq, Eq)]
    enum Shape {
        /// Vector or matrix of the operation's lanes (`fma.c`).
        Plain,
        /// `fma16` bit 62: f32 lane i/2 of row 2j + i%2.
        Widened,
    }

    /// The fused forms with no arithmetic: bits 29:27 = 3 (x, or -x for
    /// fms), 5 (y, -y), 6 (z unchanged) and 7 (+0, -0 for fms), written to
    /// the enabled lanes as the model does. Copies of widened halves are
    /// the caller's to refuse.
    fn skip(
        registers: &mut AmxRegisters,
        operand: u64,
        bytes: u32,
        mode: u64,
        subtract: bool,
        shape: Shape,
    ) -> bool {
        if mode == 6 {
            return true;
        }
        if shape == Shape::Widened && mode != 7 {
            return false;
        }
        let bytes = bytes as usize;
        let lanes = 64 / bytes;
        let x = registers.x_operand((operand >> 10) & 0x1ff);
        let y = registers.y_operand(operand & 0x1ff);
        let x_enable = write_mask(operand >> 41, bytes as u32, 7);
        let y_enable = write_mask(operand >> 32, bytes as u32, 7);
        let z_row = ((operand >> 20) & 63) as usize;
        let (out_bytes, sign) = match (shape, bytes) {
            (Shape::Widened, _) | (_, 4) => (4, 0x8000_0000u64),
            (_, 2) => (2, 0x8000),
            _ => (8, 1 << 63),
        };
        // Rows of `out_bytes` lanes: each lane's value, and all-ones where
        // it is written, merged into Z eight bytes at a time (stored whole
        // when every lane is on, as most operands have).
        let negated = |row: &Row| -> Row {
            let mut row = *row;
            if subtract {
                for (byte, flip) in row.iter_mut().zip(splat(sign, out_bytes)) {
                    *byte ^= flip;
                }
            }
            row
        };
        let every_lane = x_enable == u64::MAX;
        let store = |z: &mut Row, values: &Row, mask: &Row| {
            if every_lane {
                *z = *values;
                return;
            }
            merge(z, values, mask);
        };
        let (x_mask, parity_masks) = if every_lane {
            ([0xff; ROW_BYTES], [[0xff; ROW_BYTES]; 2])
        } else {
            let mut x_mask = [0u8; ROW_BYTES];
            let mut parity_masks = [[0u8; ROW_BYTES]; 2];
            for i in (0..lanes).filter(|&i| on(x_enable, i, bytes)) {
                x_mask[i * bytes..i * bytes + bytes].fill(0xff);
                parity_masks[i & 1][(i >> 1) * 4..(i >> 1) * 4 + 4].fill(0xff);
            }
            (x_mask, parity_masks)
        };
        let constant = splat(if subtract { sign } else { 0 }, out_bytes);
        let lane_values = |j: usize| -> Row {
            match mode {
                3 => negated(&x),
                5 => negated(&splat(lane(&y, bytes, j), bytes)),
                _ => constant,
            }
        };
        if operand & VECTOR != 0 {
            let values = match mode {
                3 => negated(&x),
                5 => negated(&y),
                _ => constant,
            };
            store(&mut registers.z[z_row], &values, &x_mask);
            return true;
        }
        for j in (0..lanes).filter(|&j| on(y_enable, j, bytes)) {
            match shape {
                Shape::Widened => {
                    for (parity, mask) in parity_masks.iter().enumerate() {
                        store(&mut registers.z[2 * j + parity], &constant, mask);
                    }
                }
                Shape::Plain => {
                    let g = 64 / lanes;
                    let values = lane_values(j);
                    store(
                        &mut registers.z[j * g + (z_row & (g - 1))],
                        &values,
                        &x_mask,
                    );
                }
            }
        }
        true
    }

    /// Save MXCSR and load the AMX environment. A conditional switch that
    /// skips LDMXCSR when MXCSR already holds it measured slower here.
    ///
    /// # Safety
    /// Pair with [`leave`].
    unsafe fn enter() -> u32 {
        let mut saved = 0u32;
        // SAFETY: stmxcsr/ldmxcsr of valid 4-byte locations.
        unsafe {
            asm!(
                "stmxcsr [{saved}]",
                "ldmxcsr [{amx}]",
                saved = in(reg) &raw mut saved,
                amx = in(reg) &raw const AMX_MXCSR,
                options(nostack, preserves_flags),
            );
        }
        saved
    }

    /// # Safety
    /// `saved` came from [`enter`].
    unsafe fn leave(saved: u32) {
        // SAFETY: ldmxcsr of the value stmxcsr produced.
        unsafe {
            asm!(
                "ldmxcsr [{saved}]",
                saved = in(reg) &raw const saved,
                options(nostack, preserves_flags, readonly),
            );
        }
    }

    /// Lane `lane` of `bytes`-byte lanes is on in a per-byte enable.
    fn on(enable: u64, lane: usize, bytes: usize) -> bool {
        (enable >> (lane * bytes)) & 1 != 0
    }

    /// Bit i for each lane i of `bytes`-byte lanes a per-byte enable turns
    /// on; every bit, with no loop, when all are on (most operands).
    fn lane_bits(enable: u64, bytes: usize) -> u32 {
        if enable == u64::MAX {
            return u32::MAX;
        }
        (0..64 / bytes).fold(0, |bits, lane| {
            bits | u32::from(on(enable, lane, bytes)) << lane
        })
    }

    /// The fused kernels' lane masks of a per-byte enable: all ones in each
    /// enabled lane's bytes (VEX) and [`lane_bits`] (EVEX); no loop when
    /// every lane is on.
    fn lane_masks(enable: u64, bytes: usize) -> (Row, u32) {
        if enable == u64::MAX {
            return ([0xff; ROW_BYTES], u32::MAX);
        }
        let mut mask = [0u8; ROW_BYTES];
        let mut opmask = 0u32;
        for lane in (0..64 / bytes).filter(|&lane| on(enable, lane, bytes)) {
            mask[lane * bytes..lane * bytes + bytes].fill(0xff);
            opmask |= 1 << lane;
        }
        (mask, opmask)
    }

    /// The even f16 lanes of a row widened to f32, exactly; NaNs stay NaN
    /// (every covered result from a NaN input is the default NaN).
    ///
    /// # Safety
    /// The tier's instructions exist: F16C for `Fma`, AVX-512F for `Avx512`.
    unsafe fn widen(tier: Tier, row: &Row) -> Row {
        let mut out = [0u8; ROW_BYTES];
        let (source, destination) = (row.as_ptr(), out.as_mut_ptr());
        // SAFETY: 64-byte arrays, unaligned forms; the tier's instructions
        // exist as the caller guarantees.
        unsafe {
            if tier == Tier::Avx512 {
                asm!(
                    "vmovdqu32 zmm0, [{source}]",
                    "vpmovdw ymm0, zmm0",
                    "vcvtph2ps zmm0, ymm0",
                    "vmovups [{destination}], zmm0",
                    "vzeroupper",
                    source = in(reg) source,
                    destination = in(reg) destination,
                    clobber_abi("sysv64"),
                    options(nostack, preserves_flags),
                );
            } else {
                asm!(
                    "vpcmpeqd xmm4, xmm4, xmm4",
                    "vpsrld xmm4, xmm4, 16",
                    "vpand xmm0, xmm4, [{source}]",
                    "vpand xmm1, xmm4, [{source} + 16]",
                    "vpand xmm2, xmm4, [{source} + 32]",
                    "vpand xmm3, xmm4, [{source} + 48]",
                    "vpackusdw xmm0, xmm0, xmm1",
                    "vpackusdw xmm2, xmm2, xmm3",
                    "vcvtph2ps ymm0, xmm0",
                    "vcvtph2ps ymm2, xmm2",
                    "vmovups [{destination}], ymm0",
                    "vmovups [{destination} + 32], ymm2",
                    "vzeroupper",
                    source = in(reg) source,
                    destination = in(reg) destination,
                    clobber_abi("sysv64"),
                    options(nostack, preserves_flags),
                );
            }
        }
        out
    }

    /// One half-row (32 bytes at `$offset`) of a VEX.256 fused row:
    /// ymm3 = old Z, ymm2 = result, NaN results become the default NaN,
    /// disabled lanes keep the old Z.
    macro_rules! vex_half {
        ($s:literal, $offset:literal, $init:literal, $op:literal) => {
            concat!(
                "vmovups ymm0, ymmword ptr [{x} + ",
                $offset,
                "]\n",
                "vmovups ymm1, ymmword ptr [{y} + ",
                $offset,
                "]\n",
                "vmovups ymm3, ymmword ptr [{z} + ",
                $offset,
                "]\n",
                $init,
                "\n",
                $op,
                $s,
                " ymm2, ymm0, ymm1\n",
                "vcmpunord",
                $s,
                " ymm4, ymm2, ymm2\n",
                "vblendv",
                $s,
                " ymm2, ymm2, ymm7, ymm4\n",
                "vmovups ymm5, ymmword ptr [{mask} + ",
                $offset,
                "]\n",
                "vblendv",
                $s,
                " ymm2, ymm3, ymm2, ymm5\n",
                "vmovups ymmword ptr [{z} + ",
                $offset,
                "], ymm2\n",
            )
        };
    }

    /// A VEX.256 fused row: ymm6 = -0 and ymm7 = the default NaN in every
    /// lane, then both halves.
    macro_rules! vex_row {
        ($s:literal, $broadcast:literal, $init:literal, $op:literal, $z:expr, $x:expr, $y:expr, $mask:expr, $nan:expr, $zero:expr) => {
            asm!(
                concat!($broadcast, " ymm6, [{zero}]"),
                concat!($broadcast, " ymm7, [{nan}]"),
                vex_half!($s, "0", $init, $op),
                vex_half!($s, "32", $init, $op),
                "vzeroupper",
                x = in(reg) $x,
                y = in(reg) $y,
                z = in(reg) $z,
                mask = in(reg) $mask,
                nan = in(reg) $nan,
                zero = in(reg) $zero,
                clobber_abi("sysv64"),
                options(nostack, preserves_flags),
            )
        };
    }

    /// One Z row on the FMA3 tier.
    ///
    /// # Safety
    /// FMA3 and AVX exist; `z` is a 64-byte row.
    unsafe fn row_fma(double: bool, form: Form, z: *mut u8, x: &Row, y: &Row, mask: &Row) {
        let (x, y, mask) = (x.as_ptr(), y.as_ptr(), mask.as_ptr());
        // SAFETY: FMA3 exists as the caller guarantees; every pointer is to
        // 64 live bytes (or a constant), unaligned forms.
        unsafe {
            if double {
                let (nan, zero) = (&raw const NAN_64, &raw const NEGATIVE_ZERO_64);
                match form {
                    Form::Add => vex_row!(
                        "pd",
                        "vbroadcastsd",
                        "vmovapd ymm2, ymm3",
                        "vfmadd231",
                        z,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                    Form::Subtract => vex_row!(
                        "pd",
                        "vbroadcastsd",
                        "vmovapd ymm2, ymm3",
                        "vfnmadd231",
                        z,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                    Form::Product => {
                        vex_row!("pd", "vbroadcastsd", "", "vmul", z, x, y, mask, nan, zero)
                    }
                    Form::NegatedProduct => vex_row!(
                        "pd",
                        "vbroadcastsd",
                        "vmovapd ymm2, ymm6",
                        "vfnmadd231",
                        z,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                }
            } else {
                let (nan, zero) = (&raw const NAN_32, &raw const NEGATIVE_ZERO_32);
                match form {
                    Form::Add => vex_row!(
                        "ps",
                        "vbroadcastss",
                        "vmovaps ymm2, ymm3",
                        "vfmadd231",
                        z,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                    Form::Subtract => vex_row!(
                        "ps",
                        "vbroadcastss",
                        "vmovaps ymm2, ymm3",
                        "vfnmadd231",
                        z,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                    Form::Product => {
                        vex_row!("ps", "vbroadcastss", "", "vmul", z, x, y, mask, nan, zero)
                    }
                    Form::NegatedProduct => vex_row!(
                        "ps",
                        "vbroadcastss",
                        "vmovaps ymm2, ymm6",
                        "vfnmadd231",
                        z,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                }
            }
        }
    }

    macro_rules! evex_row {
        ($s:literal, $broadcast:literal, $init:literal, $op:literal, $z:expr, $x:expr, $y:expr, $opmask:expr, $nan:expr, $zero:expr) => {
            asm!(
                concat!($broadcast, " zmm6, [{zero}]"),
                "vmovups zmm0, zmmword ptr [{x}]",
                "vmovups zmm1, zmmword ptr [{y}]",
                "vmovups zmm3, zmmword ptr [{z}]",
                $init,
                concat!($op, $s, " zmm2, zmm0, zmm1"),
                concat!("vcmpunord", $s, " k1, zmm2, zmm2"),
                concat!($broadcast, " zmm2 {{k1}}, [{nan}]"),
                "kmovw k2, {opmask:e}",
                // The element width decides which lanes the opmask writes.
                concat!("vmovu", $s, " zmmword ptr [{z}] {{k2}}, zmm2"),
                "vzeroupper",
                x = in(reg) $x,
                y = in(reg) $y,
                z = in(reg) $z,
                opmask = in(reg) $opmask,
                nan = in(reg) $nan,
                zero = in(reg) $zero,
                clobber_abi("sysv64"),
                options(nostack, preserves_flags),
            )
        };
    }

    /// One Z row on the AVX-512F tier.
    ///
    /// # Safety
    /// AVX-512F exists with OS-enabled state; `z` is a 64-byte row.
    unsafe fn row_avx512(double: bool, form: Form, z: *mut u8, x: &Row, y: &Row, opmask: u32) {
        let (x, y) = (x.as_ptr(), y.as_ptr());
        // SAFETY: AVX-512F exists as the caller guarantees; every pointer is
        // to 64 live bytes (or a constant), unaligned forms.
        unsafe {
            if double {
                let (nan, zero) = (&raw const NAN_64, &raw const NEGATIVE_ZERO_64);
                match form {
                    Form::Add => evex_row!(
                        "pd",
                        "vbroadcastsd",
                        "vmovapd zmm2, zmm3",
                        "vfmadd231",
                        z,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                    Form::Subtract => evex_row!(
                        "pd",
                        "vbroadcastsd",
                        "vmovapd zmm2, zmm3",
                        "vfnmadd231",
                        z,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                    Form::Product => {
                        evex_row!("pd", "vbroadcastsd", "", "vmul", z, x, y, opmask, nan, zero)
                    }
                    Form::NegatedProduct => evex_row!(
                        "pd",
                        "vbroadcastsd",
                        "vmovapd zmm2, zmm6",
                        "vfnmadd231",
                        z,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                }
            } else {
                let (nan, zero) = (&raw const NAN_32, &raw const NEGATIVE_ZERO_32);
                match form {
                    Form::Add => evex_row!(
                        "ps",
                        "vbroadcastss",
                        "vmovaps zmm2, zmm3",
                        "vfmadd231",
                        z,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                    Form::Subtract => evex_row!(
                        "ps",
                        "vbroadcastss",
                        "vmovaps zmm2, zmm3",
                        "vfnmadd231",
                        z,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                    Form::Product => {
                        evex_row!("ps", "vbroadcastss", "", "vmul", z, x, y, opmask, nan, zero)
                    }
                    Form::NegatedProduct => evex_row!(
                        "ps",
                        "vbroadcastss",
                        "vmovaps zmm2, zmm6",
                        "vfnmadd231",
                        z,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                }
            }
        }
    }

    /// A matrix product's Z rows: the first, the step between them, the Y
    /// lanes whose rows are written (bit j) and the lane count.
    struct Matrix {
        z: *mut u8,
        stride: usize,
        rows: u32,
        lanes: usize,
    }

    /// One half of a VEX.256 matrix row: X in `$x`, its lane masks in
    /// `$mask`, the broadcast Y lane in ymm1.
    macro_rules! vex_matrix_half {
        ($s:literal, $offset:literal, $init:literal, $op:literal, $x:literal, $mask:literal) => {
            concat!(
                "vmovups ymm3, ymmword ptr [rdi + ",
                $offset,
                "]\n",
                $init,
                "\n",
                $op,
                $s,
                " ymm2, ",
                $x,
                ", ymm1\n",
                "vcmpunord",
                $s,
                " ymm4, ymm2, ymm2\n",
                "vblendv",
                $s,
                " ymm2, ymm2, ymm7, ymm4\n",
                "vblendv",
                $s,
                " ymm2, ymm3, ymm2, ",
                $mask,
                "\n",
                "vmovups ymmword ptr [rdi + ",
                $offset,
                "], ymm2\n",
            )
        };
    }

    /// A VEX.256 matrix product: ymm8/9 = X, ymm10/11 = its lane masks,
    /// ymm6 = -0, ymm7 = the default NaN; then each enabled Y lane.
    macro_rules! vex_matrix {
        ($s:literal, $broadcast:literal, $size:literal, $step:literal, $init:literal, $op:literal, $m:expr, $x:expr, $y:expr, $mask:expr, $nan:expr, $zero:expr) => {
            asm!(
                concat!($broadcast, " ymm6, [{zero}]"),
                concat!($broadcast, " ymm7, [{nan}]"),
                "vmovups ymm8, ymmword ptr [{x}]",
                "vmovups ymm9, ymmword ptr [{x} + 32]",
                "vmovups ymm10, ymmword ptr [{mask}]",
                "vmovups ymm11, ymmword ptr [{mask} + 32]",
                "2:",
                "test ecx, 1",
                "jz 3f",
                concat!($broadcast, " ymm1, ", $size, " ptr [rsi]"),
                vex_matrix_half!($s, "0", $init, $op, "ymm8", "ymm10"),
                vex_matrix_half!($s, "32", $init, $op, "ymm9", "ymm11"),
                "3:",
                concat!("add rsi, ", $step),
                "add rdi, {stride}",
                "shr ecx, 1",
                "dec rdx",
                "jnz 2b",
                "vzeroupper",
                inout("rsi") $y => _,
                inout("rdi") $m.z => _,
                inout("ecx") $m.rows => _,
                inout("rdx") $m.lanes => _,
                stride = in(reg) $m.stride,
                x = in(reg) $x,
                mask = in(reg) $mask,
                nan = in(reg) $nan,
                zero = in(reg) $zero,
                clobber_abi("sysv64"),
                options(nostack),
            )
        };
    }

    /// A matrix product on the FMA3 tier.
    ///
    /// # Safety
    /// FMA3 and AVX exist; `matrix` names rows of a live Z grid.
    unsafe fn matrix_fma(double: bool, form: Form, matrix: Matrix, x: &Row, y: &Row, mask: &Row) {
        let (x, y, mask) = (x.as_ptr(), y.as_ptr(), mask.as_ptr());
        // SAFETY: FMA3 exists as the caller guarantees; every row and
        // constant pointer is live, unaligned forms.
        unsafe {
            if double {
                let (nan, zero) = (&raw const NAN_64, &raw const NEGATIVE_ZERO_64);
                match form {
                    Form::Add => vex_matrix!(
                        "pd",
                        "vbroadcastsd",
                        "qword",
                        "8",
                        "vmovapd ymm2, ymm3",
                        "vfmadd231",
                        matrix,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                    Form::Subtract => vex_matrix!(
                        "pd",
                        "vbroadcastsd",
                        "qword",
                        "8",
                        "vmovapd ymm2, ymm3",
                        "vfnmadd231",
                        matrix,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                    Form::Product => vex_matrix!(
                        "pd",
                        "vbroadcastsd",
                        "qword",
                        "8",
                        "",
                        "vmul",
                        matrix,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                    Form::NegatedProduct => vex_matrix!(
                        "pd",
                        "vbroadcastsd",
                        "qword",
                        "8",
                        "vmovapd ymm2, ymm6",
                        "vfnmadd231",
                        matrix,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                }
            } else {
                let (nan, zero) = (&raw const NAN_32, &raw const NEGATIVE_ZERO_32);
                match form {
                    Form::Add => vex_matrix!(
                        "ps",
                        "vbroadcastss",
                        "dword",
                        "4",
                        "vmovaps ymm2, ymm3",
                        "vfmadd231",
                        matrix,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                    Form::Subtract => vex_matrix!(
                        "ps",
                        "vbroadcastss",
                        "dword",
                        "4",
                        "vmovaps ymm2, ymm3",
                        "vfnmadd231",
                        matrix,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                    Form::Product => vex_matrix!(
                        "ps",
                        "vbroadcastss",
                        "dword",
                        "4",
                        "",
                        "vmul",
                        matrix,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                    Form::NegatedProduct => vex_matrix!(
                        "ps",
                        "vbroadcastss",
                        "dword",
                        "4",
                        "vmovaps ymm2, ymm6",
                        "vfnmadd231",
                        matrix,
                        x,
                        y,
                        mask,
                        nan,
                        zero
                    ),
                }
            }
        }
    }

    /// An EVEX.512 matrix product: zmm8 = X, k2 = its lanes, zmm6 = -0;
    /// then each enabled Y lane.
    macro_rules! evex_matrix {
        ($s:literal, $broadcast:literal, $size:literal, $step:literal, $init:literal, $op:literal, $m:expr, $x:expr, $y:expr, $opmask:expr, $nan:expr, $zero:expr) => {
            asm!(
                concat!($broadcast, " zmm6, [{zero}]"),
                "vmovups zmm8, zmmword ptr [{x}]",
                "kmovw k2, {opmask:e}",
                "2:",
                "test ecx, 1",
                "jz 3f",
                concat!($broadcast, " zmm1, ", $size, " ptr [rsi]"),
                "vmovups zmm3, zmmword ptr [rdi]",
                $init,
                concat!($op, $s, " zmm2, zmm8, zmm1"),
                concat!("vcmpunord", $s, " k1, zmm2, zmm2"),
                concat!($broadcast, " zmm2 {{k1}}, [{nan}]"),
                concat!("vmovu", $s, " zmmword ptr [rdi] {{k2}}, zmm2"),
                "3:",
                concat!("add rsi, ", $step),
                "add rdi, {stride}",
                "shr ecx, 1",
                "dec rdx",
                "jnz 2b",
                "vzeroupper",
                inout("rsi") $y => _,
                inout("rdi") $m.z => _,
                inout("ecx") $m.rows => _,
                inout("rdx") $m.lanes => _,
                stride = in(reg) $m.stride,
                x = in(reg) $x,
                opmask = in(reg) $opmask,
                nan = in(reg) $nan,
                zero = in(reg) $zero,
                clobber_abi("sysv64"),
                options(nostack),
            )
        };
    }

    /// A matrix product on the AVX-512F tier.
    ///
    /// # Safety
    /// AVX-512F exists with OS-enabled state; `matrix` names rows of a live
    /// Z grid.
    unsafe fn matrix_avx512(
        double: bool,
        form: Form,
        matrix: Matrix,
        x: &Row,
        y: &Row,
        opmask: u32,
    ) {
        let (x, y) = (x.as_ptr(), y.as_ptr());
        // SAFETY: AVX-512F exists as the caller guarantees; every row and
        // constant pointer is live, unaligned forms.
        unsafe {
            if double {
                let (nan, zero) = (&raw const NAN_64, &raw const NEGATIVE_ZERO_64);
                match form {
                    Form::Add => evex_matrix!(
                        "pd",
                        "vbroadcastsd",
                        "qword",
                        "8",
                        "vmovapd zmm2, zmm3",
                        "vfmadd231",
                        matrix,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                    Form::Subtract => evex_matrix!(
                        "pd",
                        "vbroadcastsd",
                        "qword",
                        "8",
                        "vmovapd zmm2, zmm3",
                        "vfnmadd231",
                        matrix,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                    Form::Product => evex_matrix!(
                        "pd",
                        "vbroadcastsd",
                        "qword",
                        "8",
                        "",
                        "vmul",
                        matrix,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                    Form::NegatedProduct => evex_matrix!(
                        "pd",
                        "vbroadcastsd",
                        "qword",
                        "8",
                        "vmovapd zmm2, zmm6",
                        "vfnmadd231",
                        matrix,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                }
            } else {
                let (nan, zero) = (&raw const NAN_32, &raw const NEGATIVE_ZERO_32);
                match form {
                    Form::Add => evex_matrix!(
                        "ps",
                        "vbroadcastss",
                        "dword",
                        "4",
                        "vmovaps zmm2, zmm3",
                        "vfmadd231",
                        matrix,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                    Form::Subtract => evex_matrix!(
                        "ps",
                        "vbroadcastss",
                        "dword",
                        "4",
                        "vmovaps zmm2, zmm3",
                        "vfnmadd231",
                        matrix,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                    Form::Product => evex_matrix!(
                        "ps",
                        "vbroadcastss",
                        "dword",
                        "4",
                        "",
                        "vmul",
                        matrix,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                    Form::NegatedProduct => evex_matrix!(
                        "ps",
                        "vbroadcastss",
                        "dword",
                        "4",
                        "vmovaps zmm2, zmm6",
                        "vfnmadd231",
                        matrix,
                        x,
                        y,
                        opmask,
                        nan,
                        zero
                    ),
                }
            }
        }
    }

    const VECFP_MULTIPLE_ROWS: u64 = 1 << 31;
    /// With an indexed load (bit 53): the indices index Y's row (47), four
    /// bits each (48), else two.
    const FLOAT_INDEXED_Y: u64 = 1 << 47;
    const FLOAT_INDEXED_4BIT: u64 = 1 << 48;

    /// The ALU mode of a `vecfp`/`matfp` operand: bits 52:47, or 0 under an
    /// indexed load, which takes bits 48:47.
    const fn float_mode(operand: u64) -> u64 {
        if operand & FLOAT_INDEXED_LOAD != 0 {
            0
        } else {
            (operand >> 47) & 0x3f
        }
    }

    /// X and Y of a `vecfp` row or a `matfp` operand, staged as the model
    /// stages them with its own byte movement: the pool rows at the offsets
    /// (bits 18:10 and 8:0 of a one-row operand), an indexed load (bit 53:
    /// packed lane indices in the pool row expanded from X or Y row bits
    /// 51:49), then the shuffles (bits 30:29 of X, 28:27 of Y).
    fn stage(
        registers: &AmxRegisters,
        operand: u64,
        xy_bits: usize,
        x_offset: u64,
        y_offset: u64,
    ) -> (Row, Row) {
        let mut x = registers.x_operand(x_offset & 0x1ff);
        let mut y = registers.y_operand(y_offset & 0x1ff);
        if operand & (FLOAT_INDEXED_LOAD | 0xf << 27) == 0 {
            return (x, y);
        }
        if operand & FLOAT_INDEXED_LOAD != 0 {
            let source = ((operand >> 49) & 7) as usize;
            let index_bits = if operand & FLOAT_INDEXED_4BIT != 0 {
                4
            } else {
                2
            };
            if operand & FLOAT_INDEXED_Y != 0 {
                model::load_indexed(&mut y, &registers.y[source], index_bits, xy_bits as u32);
            } else {
                model::load_indexed(&mut x, &registers.x[source], index_bits, xy_bits as u32);
            }
        }
        model::shuffle(&mut x, (operand >> 29) & 3, xy_bits / 8);
        model::shuffle(&mut y, (operand >> 27) & 3, xy_bits / 8);
        (x, y)
    }

    /// `vecfp` (`vecfp.c`) on IEEE types: f16 (types 2, 5, 6, 8-15, and
    /// 0/1 on M1) through the f16 rows with F16C, f32 and f64 (types 4, 7)
    /// through the FMA row kernels, f16 into f32 (type 3) through the
    /// widened vector row with F16C, in modes 0 (`z + x*y`), 1 (`z - x*y`)
    /// and from the M2 10 (`x*y`), 11 (`z + x`) and 12 (`z + y`), the adds
    /// as products with 1.0 and the product alone added to -0, each rounded
    /// once; modes 4 (`x <= 0 ? 0 : y`), 5 and 7 (Arm FMIN and FMAX with the
    /// default NaN) and the lane value that writes zeros as integer bit
    /// selections on every tier; a broadcast Y lane, the zero-operand
    /// values, every lane mode, shuffles and indexed loads ([`stage`]); and
    /// from the M2 the multi-row form (bit 31: every 32nd or 16th Z row,
    /// with its broadcasts of bits 34:32 and the M4's aligned offsets),
    /// row by row as the model loops. Refused, before any write: bits 56:54,
    /// the modes the model leaves as no-ops, bf16, and arithmetic a tier
    /// cannot do exactly; nothing in a row decides it, so a multi-row
    /// operation completes whole or not at all.
    pub(super) fn vector_float(
        native: Native,
        generation: Generation,
        registers: &mut AmxRegisters,
        operand: u64,
    ) -> bool {
        // The model ignores bit 37, which would otherwise join the lane field.
        let mut operand = operand & !(1 << 37);
        let mode = float_mode(operand);
        let m2 = generation.m2_extensions();
        let (xy_bits, z_bits, bf16) = float_types(generation, operand);
        let widened = z_bits != xy_bits;
        let arithmetic = matches!(mode, 0 | 1 | 10..=12);
        if (operand >> 54) & 7 != 0
            || !(matches!(mode, 0 | 1 | 4 | 5 | 7) || m2 && matches!(mode, 10..=12))
            || bf16
        {
            return false;
        }
        let bytes = xy_bits / 8;
        let indexed = operand & FLOAT_INDEXED_LOAD != 0;
        let indexed_y = indexed && operand & FLOAT_INDEXED_Y != 0;
        let index_bits = if operand & FLOAT_INDEXED_4BIT != 0 {
            4
        } else {
            2
        };
        // The rows, as `vector_float::vector` steps through them.
        let mut z_row = operand >> 20;
        let mut z_step = 64;
        let (mut x_step, mut y_step) = (64u64, 64u64);
        let mut x_lane_zero = false;
        let mut x_offset = operand >> 10;
        let mut y_offset = operand;
        if m2 && operand & VECFP_MULTIPLE_ROWS != 0 {
            let broadcast = (operand >> 32) & 7;
            operand &= !(0x1ff << 32);
            match broadcast {
                1 => operand |= 3 << 32,
                2 => x_step = 0,
                3 => y_step = 0,
                4 => operand |= 4 << 32,
                5 => operand |= 5 << 32,
                6 => {
                    x_step = 0;
                    x_lane_zero = true;
                }
                7 => {
                    y_step = 0;
                    operand |= 1 << 38;
                }
                _ => {}
            }
            z_step = if z_row & 32 != 0 { 16 } else { 32 };
            if generation.aligned_multiple_offsets() {
                x_offset = model::aligned_multiple_offset(
                    x_offset,
                    bytes,
                    z_step,
                    index_bits,
                    indexed && !indexed_y,
                    broadcast == 6,
                );
                y_offset = model::aligned_multiple_offset(
                    y_offset,
                    bytes,
                    z_step,
                    index_bits,
                    indexed_y,
                    broadcast == 7,
                );
            }
        }
        if indexed {
            if indexed_y {
                y_step = y_step * u64::from(index_bits) / xy_bits as u64;
            } else {
                x_step = x_step * u64::from(index_bits) / xy_bits as u64;
            }
        }
        z_row &= z_step - 1;
        let field = (operand >> 38) & 7;
        let write = !(field == 0 && (operand >> 32) & 0x3f == 3);
        if write && arithmetic {
            let able = match (xy_bits, widened) {
                (32 | 64, _) => native.tier != Tier::Model,
                _ => native.f16c,
            };
            if !able {
                return false;
            }
        }
        // SAFETY: the tier's features (F16C for f16 lanes, FMA3 or
        // AVX-512F for f32 and f64) as checked above; MXCSR is restored
        // after.
        unsafe {
            let saved = enter();
            while z_row <= 63 {
                let (mut x, y) = stage(registers, operand, xy_bits, x_offset, y_offset);
                x_offset = x_offset.wrapping_add(x_step);
                y_offset = y_offset.wrapping_add(y_step);
                if x_lane_zero {
                    // Every lane reads X's lane 0.
                    x = splat(lane(&x, bytes, 0), bytes);
                }
                let shape = VectorRow {
                    xy_bits,
                    widened,
                    mode,
                    write,
                    z_row: z_row as usize,
                };
                vector_row(native, registers, operand, x, y, shape);
                z_row += z_step;
            }
            leave(saved);
        }
        true
    }

    /// One `vecfp` row's form: its lanes, mode, whether it writes results
    /// (else zeros) and its Z row.
    #[derive(Clone, Copy)]
    struct VectorRow {
        xy_bits: usize,
        widened: bool,
        mode: u64,
        write: bool,
        z_row: usize,
    }

    /// One `vecfp` row on staged X and Y, as `vector_float` decided it: the
    /// lane field of `operand` (a broadcast Y lane, the zero-operand
    /// values, the lane enables), then the arithmetic or the selections.
    ///
    /// # Safety
    /// As `vector_float` checked: F16C for f16 arithmetic, FMA3 or AVX-512F
    /// for f32/f64 arithmetic; MXCSR holds the AMX environment.
    unsafe fn vector_row(
        native: Native,
        registers: &mut AmxRegisters,
        operand: u64,
        mut x: Row,
        mut y: Row,
        shape: VectorRow,
    ) {
        let VectorRow {
            xy_bits,
            widened,
            mode,
            write,
            z_row,
        } = shape;
        let bytes = xy_bits / 8;
        let lanes = 64 / bytes;
        let field = (operand >> 38) & 7;
        let enable = if field == 1 {
            // Y lane (bits 36:32, within the lanes) in every lane, every
            // lane on.
            let j = ((operand >> 32) as usize) & (lanes - 1);
            y = splat(lane(&y, bytes, j), bytes);
            u64::MAX
        } else {
            if field == 0 {
                match (operand >> 32) & 0x3f {
                    4 => x = [0; ROW_BYTES],
                    5 => y = [0; ROW_BYTES],
                    _ => {}
                }
            }
            write_mask(operand >> 32, bytes as u32, 9)
        };
        if !write || !matches!(mode, 0 | 1 | 10..=12) {
            select_vector(
                registers, bytes, widened, mode, write, &x, &y, enable, z_row,
            );
            return;
        }
        // z + x (11) and z + y (12) multiply by 1.0; x*y (10) adds to -0.
        let one = match xy_bits {
            16 => HALF_ONES,
            32 => ONES_32,
            _ => ONES_64,
        };
        match mode {
            11 => y = one,
            12 => x = one,
            _ => {}
        }
        let subtract = mode == 1;
        let product = mode == 10;
        // SAFETY: the caller's contract; every pointer is to a live row.
        unsafe {
            match (xy_bits, widened) {
                (32 | 64, _) => {
                    let double = xy_bits == 64;
                    let form = match (subtract, product) {
                        (true, _) => Form::Subtract,
                        (_, true) => Form::Product,
                        _ => Form::Add,
                    };
                    let (mask, opmask) = lane_masks(enable, bytes);
                    let z = registers.z[z_row].as_mut_ptr();
                    match native.tier {
                        Tier::Avx512 => row_avx512(double, form, z, &x, &y, opmask),
                        _ => row_fma(double, form, z, &x, &y, &mask),
                    }
                }
                (_, false) => {
                    let z = &mut registers.z[z_row];
                    let lanes = HalfLanes::of(enable);
                    half_row(native.tier, z, &x, &y, &lanes, subtract, product);
                }
                (_, true) => {
                    widened_vector(registers, &x, &y, enable, z_row, subtract, product);
                }
            }
        }
    }

    /// `half_to_single` exactly in integers, a NaN becoming the default NaN
    /// (FCVT with FPCR.DN).
    const fn half_to_single(half: u16) -> u32 {
        let sign = (half as u32 & 0x8000) << 16;
        let exponent = (half >> 10) & 0x1f;
        let fraction = half as u32 & 0x3ff;
        if exponent == 0x1f {
            if fraction == 0 {
                sign | 0x7f80_0000
            } else {
                0x7fc0_0000
            }
        } else if exponent == 0 {
            if fraction == 0 {
                sign
            } else {
                // fraction * 2^-24 = 2^(top - 24) * 1.f.
                let top = 31 - fraction.leading_zeros();
                sign | (top + 103) << 23 | (fraction << (23 - top)) & 0x7f_ffff
            }
        } else {
            sign | (exponent as u32 + 112) << 23 | fraction << 13
        }
    }

    /// The default NaN of `bytes`-byte lanes.
    const fn default_nan(bytes: usize) -> u64 {
        match bytes {
            2 => 0x7e00,
            4 => 0x7fc0_0000,
            _ => 0x7ff8_0000_0000_0000,
        }
    }

    const fn is_nan(bits: u64, bytes: usize) -> bool {
        let sign = 1u64 << (8 * bytes - 1);
        let infinity = match bytes {
            2 => 0x7c00,
            4 => 0x7f80_0000,
            _ => 0x7ff0_0000_0000_0000,
        };
        bits & (sign - 1) > infinity
    }

    /// A total order key of non-NaN bits in which -0 is below +0, as Arm
    /// FMIN and FMAX order them.
    const fn order_key(bits: u64, bytes: usize) -> u64 {
        let sign = 1u64 << (8 * bytes - 1);
        if bits & sign != 0 {
            !bits & (sign | (sign - 1))
        } else {
            bits | sign
        }
    }

    /// One `vecfp` lane that needs no arithmetic, of `bytes`-byte operands
    /// (widened to f32 already for type 3): mode 4 `x <= 0 ? 0 : y`, 5 and
    /// 7 the minimum and the maximum of X and Z (any NaN giving the default
    /// NaN, -0 below +0).
    const fn select_lane(mode: u64, x: u64, y: u64, z: u64, bytes: usize) -> u64 {
        match mode {
            4 => {
                if not_positive(x, bytes) {
                    0
                } else {
                    y
                }
            }
            _ if is_nan(x, bytes) || is_nan(z, bytes) => default_nan(bytes),
            5 => {
                if order_key(x, bytes) < order_key(z, bytes) {
                    x
                } else {
                    z
                }
            }
            _ => {
                if order_key(x, bytes) > order_key(z, bytes) {
                    x
                } else {
                    z
                }
            }
        }
    }

    /// `vecfp` lanes without arithmetic: the selections of [`select_lane`],
    /// or zeros where `write` is false, in every enabled X lane i: Z lane i
    /// of `z_row`, or for type 3 f32 lane i/2 of row `z_row & !1 | i & 1`.
    #[allow(clippy::too_many_arguments)]
    fn select_vector(
        registers: &mut AmxRegisters,
        bytes: usize,
        widened: bool,
        mode: u64,
        write: bool,
        x: &Row,
        y: &Row,
        enable: u64,
        z_row: usize,
    ) {
        for i in (0..64 / bytes).filter(|&i| on(enable, i, bytes)) {
            let (row, z_lane, z_bytes) = if widened {
                ((z_row & !1) | (i & 1), i >> 1, 4)
            } else {
                (z_row, i, bytes)
            };
            let result = if !write {
                0
            } else if widened {
                let widen = |row: &Row| u64::from(half_to_single(lane(row, 2, i) as u16));
                let z = lane(&registers.z[row], 4, z_lane);
                select_lane(mode, widen(x), widen(y), z, 4)
            } else {
                let z = lane(&registers.z[row], bytes, z_lane);
                select_lane(mode, lane(x, bytes, i), lane(y, bytes, i), z, bytes)
            };
            set_lane(&mut registers.z[row], z_bytes, z_lane, result);
        }
    }

    /// The type-3 `vecfp` row: f16 lanes i of X and Y widened to f32,
    /// `z ± x*y` (or `-0 ± x*y` for the `product` alone) rounded once in
    /// f32 (the product is exact) into f32 lane i/2 of row
    /// `z_row & !1 | i & 1`, where `enable` (per byte) is on.
    ///
    /// # Safety
    /// F16C and AVX exist; MXCSR holds the AMX environment.
    unsafe fn widened_vector(
        registers: &mut AmxRegisters,
        x: &Row,
        y: &Row,
        enable: u64,
        z_row: usize,
        subtract: bool,
        product: bool,
    ) {
        // Each parity's X and Y lanes, sixteen halves in order, widened.
        let mut halves = [[[0u8; 32]; 2]; 2];
        for lane in 0..32 {
            let at = (lane >> 1) * 2..(lane >> 1) * 2 + 2;
            halves[0][lane & 1][at.clone()].copy_from_slice(&x[2 * lane..2 * lane + 2]);
            halves[1][lane & 1][at].copy_from_slice(&y[2 * lane..2 * lane + 2]);
        }
        let mut wide = [[[0u8; ROW_BYTES]; 2]; 2];
        let mut masks = [[0u8; ROW_BYTES]; 2];
        for lane in (0..32).filter(|&lane| on(enable, lane, 2)) {
            masks[lane & 1][(lane >> 1) * 4..(lane >> 1) * 4 + 4].fill(0xff);
        }
        for (half, wide) in halves.iter().flatten().zip(wide.iter_mut().flatten()) {
            // SAFETY: F16C; 32 bytes of halves into 64 of singles.
            unsafe {
                asm!(
                    "vcvtph2ps ymm0, xmmword ptr [{h}]",
                    "vcvtph2ps ymm1, xmmword ptr [{h} + 16]",
                    "vmovups ymmword ptr [{w}], ymm0",
                    "vmovups ymmword ptr [{w} + 32], ymm1",
                    "vzeroupper",
                    h = in(reg) half.as_ptr(),
                    w = in(reg) wide.as_mut_ptr(),
                    clobber_abi("sysv64"),
                    options(nostack),
                );
            }
        }
        for parity in 0..2 {
            let z = registers.z[(z_row & !1) | parity].as_mut_ptr();
            // SAFETY: live 64-byte rows; AVX; the caller's MXCSR.
            unsafe {
                asm!(
                    "vbroadcastss xmm7, dword ptr [{nan}]",
                    "vbroadcastss xmm6, dword ptr [{zero}]",
                    "xor eax, eax",
                    "2:",
                    "vmovups xmm1, xmmword ptr [{y} + rax]",
                    "vmulps xmm0, xmm1, xmmword ptr [{x} + rax]",
                    "vmovups xmm2, xmmword ptr [{z} + rax]",
                    // The addend: Z, or -0 for the product alone.
                    "vmovaps xmm8, xmm2",
                    "test {product:e}, {product:e}",
                    "jz 5f",
                    "vmovaps xmm8, xmm6",
                    "5:",
                    "test {subtract:e}, {subtract:e}",
                    "jz 3f",
                    "vsubps xmm3, xmm8, xmm0",
                    "jmp 4f",
                    "3:",
                    "vaddps xmm3, xmm8, xmm0",
                    "4:",
                    // NaN results become the default NaN.
                    "vcmpps xmm4, xmm3, xmm3, 3",
                    "vblendvps xmm3, xmm3, xmm7, xmm4",
                    // Disabled lanes keep Z.
                    "vmovups xmm5, xmmword ptr [{m} + rax]",
                    "vblendvps xmm3, xmm2, xmm3, xmm5",
                    "vmovups xmmword ptr [{z} + rax], xmm3",
                    "add eax, 16",
                    "cmp eax, 64",
                    "jb 2b",
                    x = in(reg) wide[0][parity].as_ptr(),
                    y = in(reg) wide[1][parity].as_ptr(),
                    z = in(reg) z,
                    m = in(reg) masks[parity].as_ptr(),
                    nan = in(reg) &raw const NAN_32,
                    zero = in(reg) &raw const NEGATIVE_ZERO_32,
                    subtract = in(reg) u32::from(subtract),
                    product = in(reg) u32::from(product),
                    out("rax") _,
                    clobber_abi("sysv64"),
                    options(nostack),
                );
            }
        }
    }

    /// 1.0 and -0 in every f16 lane.
    const HALF_ONES: Row = splat(0x3c00, 2);
    const HALF_NEGATIVE_ZEROS: Row = splat(0x8000, 2);

    /// `fma16`/`fms16` (`fma.c`) in the arithmetic forms (bits 29:27 of 0,
    /// 1, 2 and 4): the vector shape and the f16 matrix row by row through
    /// [`fused_half_row`], the bit-62 matrix into interleaved f32 row pairs
    /// through [`widened_rows`]. Every lane's f16 product is exact in f32.
    /// `z ± x` (2) and `z ± y` (4) multiply by 1.0, whose product is exact,
    /// so the sum rounds once to the model's add or subtract; the product
    /// alone (1) adds it to -0, which leaves every product unchanged (and
    /// `-0 - p` is `-p`, both zeros included), so it too rounds once.
    pub(super) fn fused_half(
        tier: Tier,
        registers: &mut AmxRegisters,
        operation: AmxOperation,
        operand: u64,
    ) -> bool {
        let subtract = operation == AmxOperation::Fms16;
        let mode = (operand >> 27) & 7;
        if super::copies(mode) {
            let shape = if operand & VECTOR == 0 && operand & super::WIDEN_16_32 != 0 {
                Shape::Widened
            } else {
                Shape::Plain
            };
            return skip(registers, operand, 2, mode, subtract, shape);
        }
        let mut x = registers.x_operand((operand >> 10) & 0x1ff);
        let mut y = registers.y_operand(operand & 0x1ff);
        match mode {
            2 => y = HALF_ONES,
            4 => x = HALF_ONES,
            _ => {}
        }
        let product = mode == 1;
        let x_enable = write_mask(operand >> 41, 2, 7);
        let y_enable = write_mask(operand >> 32, 2, 7);
        let z_row = ((operand >> 20) & 63) as usize;
        // SAFETY: the caller checked F16C (with OS-enabled AVX state); every
        // pointer is to a live row; MXCSR is restored after.
        unsafe {
            let saved = enter();
            if operand & VECTOR != 0 {
                let z = &mut registers.z[z_row];
                half_row(tier, z, &x, &y, &HalfLanes::of(x_enable), subtract, product);
            } else if operand & super::WIDEN_16_32 == 0 {
                let lanes = HalfLanes::of(x_enable);
                for j in (0..32).filter(|&j| on(y_enable, j, 2)) {
                    // Y lane j in every lane.
                    let half = u64::from(u16::from_le_bytes([y[2 * j], y[2 * j + 1]]));
                    let word = (half * 0x0001_0001_0001_0001).to_le_bytes();
                    let mut lane = [0u8; ROW_BYTES];
                    for chunk in lane.as_chunks_mut::<8>().0 {
                        *chunk = word;
                    }
                    let z = &mut registers.z[j * 2 + (z_row & 1)];
                    half_row(tier, z, &x, &lane, &lanes, subtract, product);
                }
            } else {
                widened_rows(registers, &x, &y, x_enable, y_enable, subtract, product);
            }
            leave(saved);
        }
        true
    }

    /// The f16 lanes an operation writes, as each row kernel takes them.
    struct HalfLanes {
        /// Per byte: all ones in each written lane ([`fused_half_row`]).
        mask: Row,
        /// Bit i: lane i written ([`fused_half_row_avx512`]).
        lanes: u32,
    }

    impl HalfLanes {
        /// The lanes a per-byte enable (`write_mask`) turns on.
        fn of(enable: u64) -> Self {
            if enable == u64::MAX {
                return Self {
                    mask: ALL_LANES,
                    lanes: u32::MAX,
                };
            }
            let mut lanes = Self {
                mask: [0; ROW_BYTES],
                lanes: 0,
            };
            for lane in (0..32).filter(|&lane| on(enable, lane, 2)) {
                lanes.mask[2 * lane..2 * lane + 2].fill(0xff);
                lanes.lanes |= 1 << lane;
            }
            lanes
        }
    }

    /// One f16 row: `z ± x*y`, or `-0 ± x*y` for the `product` alone, lane
    /// by lane where `lanes` are on: sixteen lanes per step with opmask
    /// enables on the AVX-512F tier, four with byte blends on F16C.
    ///
    /// # Safety
    /// The tier's features exist (AVX-512F with its OS-enabled state for
    /// `Avx512`, else F16C and AVX); MXCSR holds the AMX environment.
    unsafe fn half_row(
        tier: Tier,
        z: &mut Row,
        x: &Row,
        y: &Row,
        lanes: &HalfLanes,
        subtract: bool,
        product: bool,
    ) {
        let addend = product.then_some(&HALF_NEGATIVE_ZEROS);
        // SAFETY: the caller's contract.
        unsafe {
            if tier == Tier::Avx512 {
                fused_half_row_avx512(x, y, addend, z, lanes.lanes, subtract);
            } else {
                fused_half_row(x, y, addend, z, &lanes.mask, subtract);
            }
        }
    }

    /// The bit-62 matrix: X lane i times Y lane j, widened to f32, into f32
    /// lane i/2 of Z row 2j + i%2. The product is exact in f32, so the
    /// fused result is one rounded f32 addition (or subtraction) to Z, or
    /// for the `product` alone to -0.
    ///
    /// # Safety
    /// F16C and AVX exist; MXCSR holds the AMX environment.
    unsafe fn widened_rows(
        registers: &mut AmxRegisters,
        x: &Row,
        y: &Row,
        x_enable: u64,
        y_enable: u64,
        subtract: bool,
        product: bool,
    ) {
        // The even and the odd X lanes, each sixteen f16 in order.
        let mut halves = [[0u8; 32]; 2];
        for lane in 0..32 {
            halves[lane & 1][(lane >> 1) * 2..(lane >> 1) * 2 + 2]
                .copy_from_slice(&x[2 * lane..2 * lane + 2]);
        }
        let mut widened = [[0u8; ROW_BYTES]; 2];
        // All 32 Y lanes widened: 128 bytes.
        let mut y_wide = [0u8; 2 * ROW_BYTES];
        // SAFETY: F16C; 32-byte X halves into 64 bytes, 64 Y bytes into 128.
        unsafe {
            for (half, wide) in halves.iter().zip(widened.iter_mut()) {
                asm!(
                    "vcvtph2ps ymm0, xmmword ptr [{h}]",
                    "vcvtph2ps ymm1, xmmword ptr [{h} + 16]",
                    "vmovups ymmword ptr [{w}], ymm0",
                    "vmovups ymmword ptr [{w} + 32], ymm1",
                    "vzeroupper",
                    h = in(reg) half.as_ptr(),
                    w = in(reg) wide.as_mut_ptr(),
                    clobber_abi("sysv64"),
                    options(nostack),
                );
            }
            asm!(
                "vcvtph2ps ymm0, xmmword ptr [{h}]",
                "vcvtph2ps ymm1, xmmword ptr [{h} + 16]",
                "vcvtph2ps ymm2, xmmword ptr [{h} + 32]",
                "vcvtph2ps ymm3, xmmword ptr [{h} + 48]",
                "vmovups ymmword ptr [{w}], ymm0",
                "vmovups ymmword ptr [{w} + 32], ymm1",
                "vmovups ymmword ptr [{w} + 64], ymm2",
                "vmovups ymmword ptr [{w} + 96], ymm3",
                "vzeroupper",
                h = in(reg) y.as_ptr(),
                w = in(reg) y_wide.as_mut_ptr(),
                clobber_abi("sysv64"),
                options(nostack),
            );
        }
        // Per-lane all-ones masks of each parity's enabled X lanes.
        let mut masks = [[0u8; ROW_BYTES]; 2];
        for lane in (0..32).filter(|&lane| on(x_enable, lane, 2)) {
            masks[lane & 1][(lane >> 1) * 4..(lane >> 1) * 4 + 4].fill(0xff);
        }
        for j in (0..32).filter(|&j| on(y_enable, j, 2)) {
            for parity in 0..2 {
                let z = registers.z[2 * j + parity].as_mut_ptr();
                // SAFETY: live 64-byte rows; AVX; the caller's MXCSR.
                unsafe {
                    asm!(
                        "vbroadcastss xmm1, dword ptr [{y}]",
                        "vbroadcastss xmm7, dword ptr [{nan}]",
                        "vbroadcastss xmm6, dword ptr [{zero}]",
                        "xor eax, eax",
                        "2:",
                        "vmulps xmm0, xmm1, xmmword ptr [{x} + rax]",
                        "vmovups xmm2, xmmword ptr [{z} + rax]",
                        // The addend: Z, or -0 for the product alone.
                        "vmovaps xmm8, xmm2",
                        "test {product:e}, {product:e}",
                        "jz 5f",
                        "vmovaps xmm8, xmm6",
                        "5:",
                        "test {subtract:e}, {subtract:e}",
                        "jz 3f",
                        "vsubps xmm3, xmm8, xmm0",
                        "jmp 4f",
                        "3:",
                        "vaddps xmm3, xmm8, xmm0",
                        "4:",
                        // NaN sums become the default NaN.
                        "vcmpps xmm4, xmm3, xmm3, 3",
                        "vblendvps xmm3, xmm3, xmm7, xmm4",
                        // Disabled lanes keep Z.
                        "vmovups xmm5, xmmword ptr [{m} + rax]",
                        "vblendvps xmm3, xmm2, xmm3, xmm5",
                        "vmovups xmmword ptr [{z} + rax], xmm3",
                        "add eax, 16",
                        "cmp eax, 64",
                        "jb 2b",
                        y = in(reg) y_wide.as_ptr().add(4 * j),
                        x = in(reg) widened[parity].as_ptr(),
                        z = in(reg) z,
                        m = in(reg) masks[parity].as_ptr(),
                        nan = in(reg) &raw const NAN_32,
                        zero = in(reg) &raw const NEGATIVE_ZERO_32,
                        subtract = in(reg) u32::from(subtract),
                        product = in(reg) u32::from(product),
                        out("rax") _,
                        clobber_abi("sysv64"),
                        options(nostack),
                    );
                }
            }
        }
    }

    /// Every byte set: all 32 f16 lanes written.
    const ALL_LANES: Row = [0xff; ROW_BYTES];

    /// One f16 row in one kernel, four lanes per step: `a ± x*y` in f16
    /// lanes, rounded once (f32 product exact, f32 sum rounded to odd,
    /// then to f16 to nearest even), default NaN, written to `z` where
    /// `mask` (per byte) is set. The addend `a` is `addend`, or `z` itself.
    ///
    /// # Safety
    /// F16C and AVX exist; MXCSR holds the AMX environment.
    unsafe fn fused_half_row(
        x: &Row,
        y: &Row,
        addend: Option<&Row>,
        z: &mut Row,
        mask: &Row,
        subtract: bool,
    ) {
        let z = z.as_mut_ptr();
        let addend = addend.map_or(z.cast_const(), |row| row.as_ptr());
        // SAFETY: five live 64-byte rows (the addend may be Z, which each
        // step reads before it writes); the caller's contract.
        unsafe {
            asm!(
                "vpcmpeqd xmm10, xmm10, xmm10",
                "vpslld xmm11, xmm10, 31",
                "vpsrld xmm10, xmm10, 31",
                "vbroadcastss xmm12, dword ptr [{nan}]",
                "vpxor xmm13, xmm13, xmm13",
                "test {subtract:e}, {subtract:e}",
                "jz 2f",
                "vmovdqa xmm13, xmm11",
                "2:",
                "xor eax, eax",
                "3:",
                "vcvtph2ps xmm0, qword ptr [{x} + rax]",
                "vcvtph2ps xmm1, qword ptr [{y} + rax]",
                "vcvtph2ps xmm2, qword ptr [{a} + rax]",
                // p = x*y, exact; negated for z - x*y.
                "vmulps xmm3, xmm0, xmm1",
                "vxorps xmm3, xmm3, xmm13",
                // s = p + z and its TwoSum error e.
                "vaddps xmm4, xmm3, xmm2",
                "vsubps xmm5, xmm4, xmm3",
                "vsubps xmm6, xmm4, xmm5",
                "vsubps xmm6, xmm3, xmm6",
                "vsubps xmm7, xmm2, xmm5",
                "vaddps xmm6, xmm6, xmm7",
                // Inexact (ordered, nonzero e) and even: step toward s + e.
                "vxorps xmm7, xmm7, xmm7",
                "vcmpps xmm7, xmm6, xmm7, 12",
                "vpand xmm8, xmm4, xmm10",
                "vpxor xmm9, xmm9, xmm9",
                "vpcmpeqd xmm8, xmm8, xmm9",
                "vpand xmm7, xmm7, xmm8",
                "vpxor xmm9, xmm4, xmm6",
                "vpsrad xmm9, xmm9, 31",
                "vpor xmm9, xmm9, xmm10",
                "vpand xmm9, xmm9, xmm7",
                "vpaddd xmm4, xmm4, xmm9",
                // NaN sums become the default NaN.
                "vcmpps xmm7, xmm4, xmm4, 3",
                "vblendvps xmm4, xmm4, xmm12, xmm7",
                "vcvtps2ph xmm4, xmm4, 0",
                // Disabled lanes keep Z.
                "vmovq xmm5, qword ptr [{z} + rax]",
                "vmovq xmm6, qword ptr [{m} + rax]",
                "vpblendvb xmm4, xmm5, xmm4, xmm6",
                "vmovq qword ptr [{z} + rax], xmm4",
                "add eax, 8",
                "cmp eax, 64",
                "jb 3b",
                x = in(reg) x.as_ptr(),
                y = in(reg) y.as_ptr(),
                a = in(reg) addend,
                z = in(reg) z,
                m = in(reg) mask.as_ptr(),
                nan = in(reg) &raw const NAN_32,
                subtract = in(reg) u32::from(subtract),
                out("rax") _,
                clobber_abi("sysv64"),
                options(nostack),
            );
        }
    }

    /// [`fused_half_row`] on EVEX.512, sixteen lanes per step: the same
    /// once-rounded `a ± x*y` (f32 product exact, f32 sum rounded to odd
    /// from its TwoSum error, then to f16 to nearest even) and default NaN,
    /// written to `z` where `lanes` (bit i: f16 lane i) is set. A step's
    /// sixteen results and old Z lanes merge as dwords under the opmask
    /// and narrow back to halves, so disabled lanes keep their bits exactly
    /// with AVX-512F alone.
    ///
    /// # Safety
    /// AVX-512F exists with OS-enabled opmask and ZMM state; MXCSR holds the
    /// AMX environment.
    unsafe fn fused_half_row_avx512(
        x: &Row,
        y: &Row,
        addend: Option<&Row>,
        z: &mut Row,
        lanes: u32,
        subtract: bool,
    ) {
        let z = z.as_mut_ptr();
        let addend = addend.map_or(z.cast_const(), |row| row.as_ptr());
        // SAFETY: five live 64-byte rows (the addend may be Z, which each
        // step reads before it writes); the caller's contract.
        unsafe {
            asm!(
                // zmm10 = 1, zmm11 = the sign bit, zmm12 = the default NaN,
                // zmm13 = the product's sign flip, zmm14 = 0, in each lane.
                "vpternlogd zmm10, zmm10, zmm10, 0xff",
                "vpsrld zmm10, zmm10, 31",
                "vpslld zmm11, zmm10, 31",
                "vpbroadcastd zmm12, dword ptr [{nan}]",
                "vpxord zmm13, zmm13, zmm13",
                "vpxord zmm14, zmm14, zmm14",
                "test {subtract:e}, {subtract:e}",
                "jz 2f",
                "vmovdqa32 zmm13, zmm11",
                "2:",
                "xor eax, eax",
                "3:",
                "kmovw k1, ecx",
                "shr ecx, 16",
                "vcvtph2ps zmm0, ymmword ptr [{x} + rax]",
                "vcvtph2ps zmm1, ymmword ptr [{y} + rax]",
                "vcvtph2ps zmm2, ymmword ptr [{a} + rax]",
                // p = x*y, exact; negated for a - x*y.
                "vmulps zmm3, zmm0, zmm1",
                "vpxord zmm3, zmm3, zmm13",
                // s = p + a and its TwoSum error e.
                "vaddps zmm4, zmm3, zmm2",
                "vsubps zmm5, zmm4, zmm3",
                "vsubps zmm6, zmm4, zmm5",
                "vsubps zmm6, zmm3, zmm6",
                "vsubps zmm7, zmm2, zmm5",
                "vaddps zmm6, zmm6, zmm7",
                // Inexact (ordered, nonzero e) and even: step toward s + e.
                "vcmpps k2, zmm6, zmm14, 12",
                "vptestnmd k3, zmm4, zmm10",
                "kandw k2, k2, k3",
                "vpxord zmm9, zmm4, zmm6",
                "vpsrad zmm9, zmm9, 31",
                "vpord zmm9, zmm9, zmm10",
                "vpaddd zmm4 {{k2}}, zmm4, zmm9",
                // NaN sums become the default NaN.
                "vcmpps k2, zmm4, zmm4, 3",
                "vmovaps zmm4 {{k2}}, zmm12",
                "vcvtps2ph ymm4, zmm4, 0",
                // Disabled lanes keep Z: merge as dwords, then narrow.
                "vpmovzxwd zmm4, ymm4",
                "vpmovzxwd zmm5, ymmword ptr [{z} + rax]",
                "vmovdqa32 zmm5 {{k1}}, zmm4",
                "vpmovdw ymmword ptr [{z} + rax], zmm5",
                "add eax, 32",
                "cmp eax, 64",
                "jb 3b",
                "vzeroupper",
                x = in(reg) x.as_ptr(),
                y = in(reg) y.as_ptr(),
                a = in(reg) addend,
                z = in(reg) z,
                nan = in(reg) &raw const NAN_32,
                subtract = in(reg) u32::from(subtract),
                inout("ecx") lanes => _,
                out("rax") _,
                clobber_abi("sysv64"),
                options(nostack),
            );
        }
    }

    /// `mac16` (bits 61/60: int8 X/Y from each lane's low byte; 29:28: the
    /// product, X, Y or 0; 59:55: an arithmetic right shift; 27: no Z
    /// input), in every shape: the vector's and the matrix's 16-bit Z lanes
    /// and the bit-62 matrix's 32-bit ones. Integer lanes, so exact: each
    /// lane's value in i32 (16-bit products fit), shifted, then added to Z
    /// with the wrap of Z's width. AVX-512F alone: the selection becomes a
    /// product with 1 or 0, lane enables opmasks.
    pub(super) fn mac16(registers: &mut AmxRegisters, operand: u64) -> bool {
        let mut x = registers.x_operand((operand >> 10) & 0x1ff);
        let mut y = registers.y_operand(operand & 0x1ff);
        // X alone is X times 1, Y alone 1 times Y, nothing 0 times Y; 1 and
        // 0 stay themselves as int8.
        match (operand >> 28) & 3 {
            1 => y = splat(1, 2),
            2 => x = splat(1, 2),
            3 => x = [0; ROW_BYTES],
            _ => {}
        }
        let int8 = |bit: u64| if operand & bit != 0 { 24u64 } else { 16 };
        let parameters = Mac16 {
            x_extend: int8(MAC16_X_INT8),
            y_extend: int8(MAC16_Y_INT8),
            shift: (operand >> 55) & 0x1f,
            z_input: if operand & (1 << 27) == 0 {
                u32::MAX
            } else {
                0
            },
        };
        let x_enable = write_mask(operand >> 41, 2, 7);
        let y_enable = write_mask(operand >> 32, 2, 7);
        let z_row = ((operand >> 20) & 63) as usize;
        let lanes = HalfLanes::of(x_enable).lanes;
        // SAFETY: the caller checked AVX-512F (with its OS-enabled state);
        // every pointer is to a live 64-byte row. No floating point, so
        // MXCSR is not touched.
        unsafe {
            if operand & VECTOR != 0 {
                mac16_row(&x, &y, registers.z[z_row].as_mut_ptr(), lanes, &parameters);
            } else if operand & super::WIDEN_16_32 == 0 {
                for j in (0..32).filter(|&j| on(y_enable, j, 2)) {
                    let half = u64::from(u16::from_le_bytes([y[2 * j], y[2 * j + 1]]));
                    let lane = splat(half, 2);
                    let z = registers.z[j * 2 + (z_row & 1)].as_mut_ptr();
                    mac16_row(&x, &lane, z, lanes, &parameters);
                }
            } else {
                // Even and odd X lanes into rows 2j and 2j + 1, lane i/2.
                let (mut even, mut odd) = (0u32, 0u32);
                for lane in (0..32).filter(|&lane| on(x_enable, lane, 2)) {
                    if lane & 1 == 0 {
                        even |= 1 << (lane >> 1);
                    } else {
                        odd |= 1 << (lane >> 1);
                    }
                }
                for j in (0..32).filter(|&j| on(y_enable, j, 2)) {
                    let half = i32::from(i16::from_le_bytes([y[2 * j], y[2 * j + 1]]));
                    let y_lane = if parameters.y_extend == 24 {
                        i32::from(half as i8)
                    } else {
                        half
                    };
                    let rows = registers.z[2 * j..2 * j + 2].as_mut_ptr().cast::<u8>();
                    mac16_widened(&x, y_lane, rows, even | odd << 16, &parameters);
                }
            }
        }
        true
    }

    const MAC16_X_INT8: u64 = 1 << 61;
    const MAC16_Y_INT8: u64 = 1 << 60;

    /// A `mac16` operation's lane arithmetic, as its kernels take it: the
    /// left-then-arithmetic-right shift that sign-extends X's and Y's lanes
    /// in i32 from 16 bits (16) or their low byte (24), the result's shift,
    /// and all ones to add Z (none for bit 27).
    struct Mac16 {
        x_extend: u64,
        y_extend: u64,
        shift: u64,
        z_input: u32,
    }

    /// One row of 16-bit `mac16` lanes: `((x' * y') >> shift) + z` where
    /// `lanes` (bit i: lane i) are on, sixteen lanes per step in i32, the
    /// old lanes merged as dwords and the row narrowed back to 16 bits.
    ///
    /// # Safety
    /// AVX-512F exists with OS-enabled opmask and ZMM state; `z` is a live
    /// 64-byte row.
    unsafe fn mac16_row(x: &Row, y: &Row, z: *mut u8, lanes: u32, parameters: &Mac16) {
        // SAFETY: three live 64-byte rows; the caller's contract.
        unsafe {
            asm!(
                "vmovq xmm15, {x_extend}",
                "vmovq xmm14, {y_extend}",
                "vmovq xmm13, {shift}",
                "vpbroadcastd zmm12, {z_input:e}",
                "xor eax, eax",
                "2:",
                "kmovw k1, ecx",
                "shr ecx, 16",
                "vpmovsxwd zmm0, ymmword ptr [{x} + rax]",
                "vpslld zmm0, zmm0, xmm15",
                "vpsrad zmm0, zmm0, xmm15",
                "vpmovsxwd zmm1, ymmword ptr [{y} + rax]",
                "vpslld zmm1, zmm1, xmm14",
                "vpsrad zmm1, zmm1, xmm14",
                "vpmulld zmm0, zmm0, zmm1",
                "vpsrad zmm0, zmm0, xmm13",
                "vpmovsxwd zmm2, ymmword ptr [{z} + rax]",
                "vpandd zmm3, zmm2, zmm12",
                "vpaddd zmm0, zmm0, zmm3",
                "vmovdqa32 zmm2 {{k1}}, zmm0",
                "vpmovdw ymmword ptr [{z} + rax], zmm2",
                "add eax, 32",
                "cmp eax, 64",
                "jb 2b",
                "vzeroupper",
                x = in(reg) x.as_ptr(),
                y = in(reg) y.as_ptr(),
                z = in(reg) z,
                x_extend = in(reg) parameters.x_extend,
                y_extend = in(reg) parameters.y_extend,
                shift = in(reg) parameters.shift,
                z_input = in(reg) parameters.z_input,
                inout("ecx") lanes => _,
                out("rax") _,
                clobber_abi("sysv64"),
                options(nostack),
            );
        }
    }

    /// The bit-62 `mac16` matrix for one Y lane: X's even lanes into the
    /// 32-bit lanes of the first of `rows` (two consecutive Z rows), its
    /// odd lanes into the second's, each `((x' * y_lane) >> shift) + z`
    /// where `lanes` (bits 15:0 the even rows', 31:16 the odd's) are on.
    /// `y_lane` is already sign-extended from its width.
    ///
    /// # Safety
    /// AVX-512F exists with OS-enabled opmask and ZMM state; `rows` is two
    /// live 64-byte rows.
    unsafe fn mac16_widened(x: &Row, y_lane: i32, rows: *mut u8, lanes: u32, parameters: &Mac16) {
        // Even lanes are each dword's low half (or low byte), odd lanes its
        // high half (or bits 23:16).
        let (even_left, odd_left, right) = if parameters.x_extend == 24 {
            (24u64, 8u64, 24u64)
        } else {
            (16, 0, 16)
        };
        // SAFETY: a live X row and two Z rows; the caller's contract.
        unsafe {
            asm!(
                "vmovq xmm15, {even_left}",
                "vmovq xmm14, {odd_left}",
                "vmovq xmm11, {right}",
                "vmovq xmm13, {shift}",
                "vpbroadcastd zmm12, {z_input:e}",
                "vpbroadcastd zmm3, {y:e}",
                "kmovw k1, ecx",
                "shr ecx, 16",
                "kmovw k2, ecx",
                "vmovdqu32 zmm0, zmmword ptr [{x}]",
                "vpslld zmm1, zmm0, xmm15",
                "vpsrad zmm1, zmm1, xmm11",
                "vpslld zmm2, zmm0, xmm14",
                "vpsrad zmm2, zmm2, xmm11",
                "vpmulld zmm1, zmm1, zmm3",
                "vpsrad zmm1, zmm1, xmm13",
                "vpmulld zmm2, zmm2, zmm3",
                "vpsrad zmm2, zmm2, xmm13",
                "vmovdqu32 zmm4, zmmword ptr [{z}]",
                "vpandd zmm4, zmm4, zmm12",
                "vpaddd zmm1, zmm1, zmm4",
                "vmovdqu32 zmmword ptr [{z}] {{k1}}, zmm1",
                "vmovdqu32 zmm5, zmmword ptr [{z} + 64]",
                "vpandd zmm5, zmm5, zmm12",
                "vpaddd zmm2, zmm2, zmm5",
                "vmovdqu32 zmmword ptr [{z} + 64] {{k2}}, zmm2",
                "vzeroupper",
                x = in(reg) x.as_ptr(),
                z = in(reg) rows,
                y = in(reg) y_lane,
                even_left = in(reg) even_left,
                odd_left = in(reg) odd_left,
                right = in(reg) right,
                shift = in(reg) parameters.shift,
                z_input = in(reg) parameters.z_input,
                inout("ecx") lanes => _,
                clobber_abi("sysv64"),
                options(nostack),
            );
        }
    }

    const FLOAT_INDEXED_LOAD: u64 = 1 << 53;

    /// `vecfp`/`matfp`'s element types (bits 45:42) as the model reads
    /// them (`vector_float::types`): X/Y bits, Z bits, bf16.
    fn float_types(generation: Generation, operand: u64) -> (usize, usize, bool) {
        match (operand >> 42) & 0xf {
            0 if generation.m2_extensions() => (16, 16, true),
            1 if generation.m2_extensions() => (16, 32, true),
            3 => (16, 32, false),
            4 => (32, 32, false),
            7 => (64, 64, false),
            _ => (16, 16, false),
        }
    }

    /// IEEE `x <= 0` on a lane's bits, false for NaN (`float::not_positive`):
    /// a zero of either sign, or a negative value that is not NaN.
    const fn not_positive(bits: u64, bytes: usize) -> bool {
        let sign = 1u64 << (8 * bytes - 1);
        let magnitude = bits & (sign - 1);
        let infinity = match bytes {
            2 => 0x7c00,
            4 => 0x7f80_0000,
            _ => 0x7ff0_0000_0000_0000,
        };
        magnitude == 0 || bits & sign != 0 && magnitude <= infinity
    }

    /// Lane `lane` of `bytes`-byte (2, 4 or 8) lanes of a row, with
    /// fixed-size copies.
    fn lane(row: &Row, bytes: usize, lane: usize) -> u64 {
        let at = lane * bytes;
        match bytes {
            2 => u64::from(u16::from_le_bytes([row[at], row[at + 1]])),
            4 => u64::from(u32::from_le_bytes([
                row[at],
                row[at + 1],
                row[at + 2],
                row[at + 3],
            ])),
            _ => {
                let mut value = [0u8; 8];
                value.copy_from_slice(&row[at..at + 8]);
                u64::from_le_bytes(value)
            }
        }
    }

    /// Set lane `lane` of `bytes`-byte (2, 4 or 8) lanes of a row to the
    /// low bytes of `value`.
    fn set_lane(row: &mut Row, bytes: usize, lane: usize, value: u64) {
        let at = lane * bytes;
        match bytes {
            2 => row[at..at + 2].copy_from_slice(&(value as u16).to_le_bytes()),
            4 => row[at..at + 4].copy_from_slice(&(value as u32).to_le_bytes()),
            _ => row[at..at + 8].copy_from_slice(&value.to_le_bytes()),
        }
    }

    /// Z's bytes where `mask` is set become `values`'.
    fn merge(z: &mut Row, values: &Row, mask: &Row) {
        let z = z.as_chunks_mut::<8>().0;
        let values = values.as_chunks::<8>().0;
        let mask = mask.as_chunks::<8>().0;
        for ((z, value), mask) in z.iter_mut().zip(values).zip(mask) {
            let (old, value, mask) = (
                u64::from_le_bytes(*z),
                u64::from_le_bytes(*value),
                u64::from_le_bytes(*mask),
            );
            *z = (old & !mask | value & mask).to_le_bytes();
        }
    }

    /// `matfp` (`matfp.c`) on IEEE types: f32 (type 4) and f64 (7) with the
    /// FMA kernels, f16 (types 2, 5, 6, 8-15, and 0/1 on M1) through the
    /// f16 rows (F16C), f16 into f32 (type 3) through the widened rows
    /// (F16C), modes 0 (`z + x*y`) and 1 (`z - x*y`); mode 4 (`x <= 0 ? 0 :
    /// y`) and the lane-enable value 3 (zeros) as bit selections; the
    /// values 4 and 5 (a zero X or Y) feeding a zero row. Every lane mode,
    /// shuffles and indexed loads ([`stage`]). Refused, before anything is
    /// written: bits 56:54 (the model's no-op), other modes (no-ops), bf16
    /// (types 0/1 from the M2), and the model tier.
    pub(super) fn matrix_float(
        native: Native,
        generation: Generation,
        registers: &mut AmxRegisters,
        operand: u64,
    ) -> bool {
        // The model ignores bits 37 and 63.
        let operand = operand & !(1 << 37 | 1 << 63);
        let mode = float_mode(operand);
        let (xy_bits, z_bits, bf16) = float_types(generation, operand);
        if native.tier == Tier::Model
            || (operand >> 54) & 7 != 0
            || !matches!(mode, 0 | 1 | 4)
            || bf16
        {
            return false;
        }
        let bytes = xy_bits / 8;
        let widened = z_bits != xy_bits;
        let x_enable = write_mask(operand >> 32, bytes as u32, 9);
        let y_enable = write_mask(((operand >> 23) & 7) << 6 | operand >> 58, bytes as u32, 9);
        let (mut x, mut y) = stage(registers, operand, xy_bits, operand >> 10, operand);
        let mut write = true;
        if (operand >> 38) & 7 == 0 {
            match (operand >> 32) & 0x3f {
                3 => write = false,
                4 | 5 => x = [0; ROW_BYTES],
                _ => {}
            }
        }
        if (operand >> 23) & 7 == 0 {
            match (operand >> 58) & 0x3f {
                3 => write = false,
                4 | 5 => y = [0; ROW_BYTES],
                _ => {}
            }
        }
        let z_row = ((operand >> 20) & 7) as usize;
        let lanes = 64 / bytes;
        if !write || mode == 4 {
            // Bit selections: Y lane j (widened from f16 for type 3) where
            // X lane i is positive or NaN, else 0; nothing kept writes zeros.
            if widened && write && !native.f16c {
                return false;
            }
            let keep = if write {
                (0..lanes).fold(0u32, |keep, i| {
                    keep | u32::from(!not_positive(lane(&x, bytes, i), bytes)) << i
                })
            } else {
                0
            };
            let values = if widened && write {
                // SAFETY: F16C exists as checked; MXCSR is restored after.
                let wide = unsafe {
                    let saved = enter();
                    let wide = widen_halves(&y);
                    leave(saved);
                    wide
                };
                // FCVT's default NaN for a NaN.
                wide.map(|value| {
                    if value & 0x7fff_ffff > 0x7f80_0000 {
                        0x7fc0_0000
                    } else {
                        u64::from(value)
                    }
                })
            } else {
                core::array::from_fn(|j| if j < lanes { lane(&y, bytes, j) } else { 0 })
            };
            select_matrix(
                registers, bytes, widened, keep, &values, x_enable, y_enable, z_row,
            );
            return true;
        }
        let subtract = mode == 1;
        match (xy_bits, widened) {
            (32 | 64, _) => {
                let double = xy_bits == 64;
                let form = if subtract { Form::Subtract } else { Form::Add };
                let (mask, opmask) = lane_masks(x_enable, bytes);
                let rows = lane_bits(y_enable, bytes);
                // Y lane j into Z row j*g + z_row%g, g = 64/lanes.
                let g = 64 / lanes;
                let matrix = Matrix {
                    z: registers.z[z_row & (g - 1)].as_mut_ptr(),
                    stride: g * ROW_BYTES,
                    rows,
                    lanes,
                };
                // SAFETY: `select` chose the tier from a target whose
                // features the host admits; every pointer is to a live row;
                // MXCSR is restored after.
                unsafe {
                    let saved = enter();
                    match native.tier {
                        Tier::Avx512 => matrix_avx512(double, form, matrix, &x, &y, opmask),
                        _ => matrix_fma(double, form, matrix, &x, &y, &mask),
                    }
                    leave(saved);
                }
            }
            _ if !native.f16c => return false,
            (_, false) => {
                let lanes = HalfLanes::of(x_enable);
                // SAFETY: F16C as checked, and the tier's own features; every
                // pointer is to a live row; MXCSR is restored after.
                unsafe {
                    let saved = enter();
                    for j in (0..32).filter(|&j| on(y_enable, j, 2)) {
                        let half = u64::from(u16::from_le_bytes([y[2 * j], y[2 * j + 1]]));
                        let lane = splat(half, 2);
                        let z = &mut registers.z[(j * 2) | (z_row & 1)];
                        half_row(native.tier, z, &x, &lane, &lanes, subtract, false);
                    }
                    leave(saved);
                }
            }
            (_, true) => {
                // SAFETY: F16C as checked; MXCSR is restored after.
                unsafe {
                    let saved = enter();
                    widened_rows(registers, &x, &y, x_enable, y_enable, subtract, false);
                    leave(saved);
                }
            }
        }
        true
    }

    /// The 32 f16 lanes of a row widened to f32 exactly (NaNs stay NaN).
    ///
    /// # Safety
    /// F16C exists.
    unsafe fn widen_halves(row: &Row) -> [u32; 32] {
        let mut packed = [0u8; 2 * ROW_BYTES];
        // SAFETY: F16C; 64 bytes of halves into 128 of singles.
        unsafe {
            asm!(
                "vcvtph2ps ymm0, xmmword ptr [{h}]",
                "vcvtph2ps ymm1, xmmword ptr [{h} + 16]",
                "vcvtph2ps ymm2, xmmword ptr [{h} + 32]",
                "vcvtph2ps ymm3, xmmword ptr [{h} + 48]",
                "vmovups ymmword ptr [{w}], ymm0",
                "vmovups ymmword ptr [{w} + 32], ymm1",
                "vmovups ymmword ptr [{w} + 64], ymm2",
                "vmovups ymmword ptr [{w} + 96], ymm3",
                "vzeroupper",
                h = in(reg) row.as_ptr(),
                w = in(reg) packed.as_mut_ptr(),
                clobber_abi("sysv64"),
                options(nostack),
            );
        }
        let singles = packed.as_chunks::<4>().0;
        core::array::from_fn(|j| u32::from_le_bytes(singles[j]))
    }

    /// `matfp` lanes that need no arithmetic: for every enabled X lane i
    /// and Y lane j, Z lane (i, j) of the row map becomes `values[j]` where
    /// bit i of `keep` is set, else 0. The row map is `matfp`'s: row
    /// `j*g | z_row % g` (g the lane bytes), lane i; or for the widened
    /// type row `2j | i % 2`, f32 lane i/2.
    #[allow(clippy::too_many_arguments)]
    fn select_matrix(
        registers: &mut AmxRegisters,
        bytes: usize,
        widened: bool,
        keep: u32,
        values: &[u64; 32],
        x_enable: u64,
        y_enable: u64,
        z_row: usize,
    ) {
        let lanes = 64 / bytes;
        if !widened {
            let mut kept = [0u8; ROW_BYTES];
            let mut written = [0u8; ROW_BYTES];
            for i in (0..lanes).filter(|&i| on(x_enable, i, bytes)) {
                written[i * bytes..i * bytes + bytes].fill(0xff);
                if keep >> i & 1 != 0 {
                    kept[i * bytes..i * bytes + bytes].fill(0xff);
                }
            }
            for j in (0..lanes).filter(|&j| on(y_enable, j, bytes)) {
                let mut row = splat(values[j], bytes);
                for (byte, kept) in row.iter_mut().zip(kept) {
                    *byte &= kept;
                }
                merge(
                    &mut registers.z[(j * bytes) | (z_row & (bytes - 1))],
                    &row,
                    &written,
                );
            }
            return;
        }
        // X lane i into f32 lane i/2 of the row of its parity.
        let mut kept = [[0u8; ROW_BYTES]; 2];
        let mut written = [[0u8; ROW_BYTES]; 2];
        for i in (0..32).filter(|&i| on(x_enable, i, 2)) {
            let at = (i >> 1) * 4..(i >> 1) * 4 + 4;
            written[i & 1][at.clone()].fill(0xff);
            if keep >> i & 1 != 0 {
                kept[i & 1][at].fill(0xff);
            }
        }
        for j in (0..32).filter(|&j| on(y_enable, j, 2)) {
            for parity in 0..2 {
                let mut row = splat(values[j], 4);
                for (byte, kept) in row.iter_mut().zip(kept[parity]) {
                    *byte &= kept;
                }
                merge(&mut registers.z[(2 * j) | parity], &row, &written[parity]);
            }
        }
    }
}

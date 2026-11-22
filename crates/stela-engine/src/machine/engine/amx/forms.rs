//! AMX register operations by form, which step timing (`--profile`) counts
//! with their path and sampled cost, so that a boot's profile ranks the
//! forms worth host kernels.
//!
//! A form is an operation with the fields that choose its work: precision
//! or element type, vector or matrix shape, the fused mode's class (fused,
//! product only, add only, a copy or constant: bits 29:27 of 0, 1, 2/4 and
//! 3/5/6/7), fma32's f16 widening (bits 61/60), the bit-62 widened matrix
//! of `fma16` and `mac16`, and for `vecfp` and `matfp` the element kind,
//! whether `vecfp`'s mode is the fused 0/1 or another, and whether the
//! operation stages its operands (a shuffle, an indexed load, `vecfp`'s
//! multiple rows from the M2) or does nothing (bits 56:54). `fma` and `fms`
//! share forms; offsets, rows and lane enables are no part of one. Every
//! key below [`FORMS`] is one some operand takes.
//!
//! The counting is outside generated code: under step timing the engine
//! gives blocks [`routine`] instead of the compiler's own, which it wraps
//! (through `routines::complete`, which leaves the operations
//! `--amx-native off` names to the model), so the code blocks hold is the
//! same either way.
use super::routines::{self, OPERATIONS, ticks};
use alloc::{format, string::String, vec::Vec};
use core::{fmt, ops::AddAssign};
use stela_compiler::{
    amx::{AmxOperation, Generation},
    x86_64::amx_transfer::{Cell, SAMPLE},
};

/// The forms; each key below is one some operand takes.
pub const FORMS: usize = 64;

const VECTOR: u64 = 1 << 63;
/// `fma16`/`mac16` bit 62: a matrix into interleaved 32-bit row pairs.
const WIDEN_16_32: u64 = 1 << 62;
/// `fma32` bits 61/60: X and Y read as even f16 lanes.
const HALVES: u64 = 3 << 60;
const INDEXED_LOAD: u64 = 1 << 53;
const MULTIPLE_ROWS: u64 = 1 << 31;

/// The first key of each operation's forms.
const FMA32: usize = 0;
const FMA64: usize = FMA32 + 2 * CLASSES.len() * 2;
const FMA16: usize = FMA64 + 2 * CLASSES.len();
const MAC16: usize = FMA16 + SHAPES.len() * CLASSES.len();
const VECFP: usize = MAC16 + SHAPES.len();
const VECFP_STAGED: usize = VECFP + KINDS_OF_FLOAT.len() * 2;
const MATFP: usize = VECFP_STAGED + 1;
const MATFP_STAGED: usize = MATFP + KINDS_OF_FLOAT.len();
const VECINT: usize = MATFP_STAGED + 1;
const MATINT: usize = VECINT + 1;
const EXTRX: usize = MATINT + 1;
const EXTRY: usize = EXTRX + 1;
const GENLUT: usize = EXTRY + 1;
const _: () = assert!(GENLUT + 1 == FORMS);
// Every PE's cell, statistics and published counts hold each form's counts:
// a bound on their size.
const _: () = assert!(FORMS <= 64);

/// The fused modes by class.
const CLASSES: [&str; 4] = ["fused", "product", "add", "copy"];
/// `fma16` and `mac16` shapes.
const SHAPES: [&str; 3] = ["vector", "matrix", "widened"];
/// `vecfp`/`matfp` element kinds: X/Y and Z both f16, f16 into f32, f32,
/// f64, both bf16, bf16 into f32.
const KINDS_OF_FLOAT: [&str; 6] = ["f16", "f16-f32", "f32", "f64", "bf16", "bf16-f32"];

/// Bits 29:27 of a fused operation by class ([`CLASSES`]).
const fn class(operand: u64) -> usize {
    match (operand >> 27) & 7 {
        0 => 0,
        1 => 1,
        2 | 4 => 2,
        _ => 3,
    }
}

/// `vecfp`/`matfp`'s type field (bits 45:42) by kind ([`KINDS_OF_FLOAT`]),
/// as the model reads it: types 0 and 1 are bf16 from the M2, f16 before.
const fn kind(generation: Generation, operand: u64) -> usize {
    let m2 = !matches!(generation, Generation::M1);
    match (operand >> 42) & 0xf {
        0 if m2 => 4,
        1 if m2 => 5,
        3 => 1,
        4 => 2,
        7 => 3,
        _ => 0,
    }
}

/// The form of `operation` with `operand` in `generation`, below
/// [`FORMS`]; `None` for an operation that is not a register operation.
pub fn key(operation: AmxOperation, operand: u64, generation: Generation) -> Option<usize> {
    use AmxOperation::*;
    let vector = operand & VECTOR != 0;
    let shape = if vector {
        0
    } else if operand & WIDEN_16_32 != 0 {
        2
    } else {
        1
    };
    Some(match operation {
        Fma32 | Fms32 => {
            FMA32
                + usize::from(vector) * CLASSES.len() * 2
                + class(operand) * 2
                + usize::from(operand & HALVES != 0)
        }
        Fma64 | Fms64 => FMA64 + usize::from(vector) * CLASSES.len() + class(operand),
        Fma16 | Fms16 => FMA16 + shape * CLASSES.len() + class(operand),
        Mac16 => MAC16 + shape,
        VectorFloat => {
            let multiple = !matches!(generation, Generation::M1) && operand & MULTIPLE_ROWS != 0;
            if (operand >> 54) & 7 != 0
                || operand & INDEXED_LOAD != 0
                || (operand >> 27) & 0xf != 0
                || multiple
            {
                VECFP_STAGED
            } else {
                VECFP + kind(generation, operand) * 2 + usize::from((operand >> 47) & 0x3f > 1)
            }
        }
        MatrixFloat => {
            if (operand >> 54) & 7 != 0 || operand & INDEXED_LOAD != 0 || (operand >> 27) & 0xf != 0
            {
                MATFP_STAGED
            } else {
                MATFP + kind(generation, operand)
            }
        }
        VectorInteger => VECINT,
        MatrixInteger => MATINT,
        ExtractX => EXTRX,
        ExtractY => EXTRY,
        GenerateLookup => GENLUT,
        _ => return None,
    })
}

/// A form's name as the profile prints it, such as `fma32/matrix/fused/f16`
/// or `vecfp/bf16/alu` (`alu`: any mode but the fused 0 and 1, those the
/// model leaves as no-ops included).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FormName(pub usize);

impl fmt::Display for FormName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let key = self.0;
        let classes = CLASSES.len();
        match key {
            FMA32..FMA64 => {
                let index = key - FMA32;
                let half = if index % 2 == 1 { "/f16" } else { "" };
                let shape = ["matrix", "vector"][index / (classes * 2)];
                write!(f, "fma32/{shape}/{}{half}", CLASSES[index / 2 % classes])
            }
            FMA64..FMA16 => {
                let index = key - FMA64;
                let shape = ["matrix", "vector"][index / classes];
                write!(f, "fma64/{shape}/{}", CLASSES[index % classes])
            }
            FMA16..MAC16 => {
                let index = key - FMA16;
                write!(
                    f,
                    "fma16/{}/{}",
                    SHAPES[index / classes],
                    CLASSES[index % classes]
                )
            }
            MAC16..VECFP => write!(f, "mac16/{}", SHAPES[key - MAC16]),
            VECFP..VECFP_STAGED => {
                let index = key - VECFP;
                let mode = ["fma", "alu"][index % 2];
                write!(f, "vecfp/{}/{mode}", KINDS_OF_FLOAT[index / 2])
            }
            VECFP_STAGED => f.write_str("vecfp/staged"),
            MATFP..MATFP_STAGED => write!(f, "matfp/{}", KINDS_OF_FLOAT[key - MATFP]),
            MATFP_STAGED => f.write_str("matfp/staged"),
            VECINT => f.write_str("vecint"),
            MATINT => f.write_str("matint"),
            EXTRX => f.write_str("extrx"),
            EXTRY => f.write_str("extry"),
            GENLUT => f.write_str("genlut"),
            _ => write!(f, "form{key}"),
        }
    }
}

/// Register operations by form and path (host kernel, model), and the TSC
/// ticks and count of the [`SAMPLE`]th calls of each form in blocks, by
/// path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AmxForms {
    pub counts: [[u64; 2]; FORMS],
    pub sampled: [[[u64; 2]; 2]; FORMS],
}

impl AmxForms {
    pub const ZERO: Self = Self {
        counts: [[0; 2]; FORMS],
        sampled: [[[0; 2]; 2]; FORMS],
    };
}

impl Default for AmxForms {
    fn default() -> Self {
        Self::ZERO
    }
}

impl AddAssign for AmxForms {
    fn add_assign(&mut self, other: Self) {
        for (total, count) in self
            .counts
            .as_flattened_mut()
            .iter_mut()
            .zip(other.counts.as_flattened())
        {
            *total += count;
        }
        for (total, count) in self
            .sampled
            .as_flattened_mut()
            .as_flattened_mut()
            .iter_mut()
            .zip(other.sampled.as_flattened().as_flattened())
        {
            *total += count;
        }
    }
}

#[cfg(all(feature = "std", feature = "native-profile"))]
impl serde::Serialize for AmxForms {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let mut fields = serializer.serialize_struct("AmxForms", 2)?;
        fields.serialize_field("counts", &self.counts[..])?;
        fields.serialize_field("sampled", &self.sampled[..])?;
        fields.end()
    }
}

/// What native code finds through `Anchor::Amx`: the compiler's cell,
/// which the anchor's address names, then the forms [`routine`] counts
/// there until the engine folds them into its statistics, and the
/// register operations the engine's routines leave to the model.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AmxCell {
    pub cell: Cell,
    pub forms: AmxForms,
    /// Bit n: the register operation numbered n (bits 9:5 of its word)
    /// runs on the model alone in blocks (`routines::complete`), as the
    /// engine set it at native entry (`Machine::set_amx_native`).
    pub model: u32,
}

const _: () = assert!(core::mem::offset_of!(AmxCell, cell) == 0);

/// `routines::complete`, with what it completes counted by form.
///
/// # Safety
/// As [`routine`].
unsafe fn counted(cell: *mut Cell, operation: u32, operand: u64) -> u64 {
    let number = operation as usize;
    // SAFETY: the caller's contract; nothing borrows the cell meanwhile.
    let (generation, before) =
        unsafe { ((*cell).generation, (*cell).kernels.get(number).copied()) };
    let form = OPERATIONS
        .get(number)
        .copied()
        .flatten()
        .zip(generation)
        .and_then(|(operation, generation)| key(operation, operand, generation));
    let (Some(key), Some(before)) = (form, before) else {
        // SAFETY: the caller's contract.
        return unsafe { routines::complete(cell, operation, operand) };
    };
    let amx_cell = cell.cast::<AmxCell>();
    // SAFETY: the caller's contract: `cell` is an `AmxCell`'s first field.
    let calls = unsafe { (*amx_cell).forms.counts[key] };
    let started = (calls[0] + calls[1]).is_multiple_of(SAMPLE).then(ticks);
    // SAFETY: the caller's contract.
    let done = unsafe { routines::complete(cell, operation, operand) };
    if done != 0 {
        let ended = started.map(|started| ticks().wrapping_sub(started));
        // The routine counts each operation it completes once, by path: a
        // native count unchanged is the model's.
        // SAFETY: the caller's contract; the routine has returned.
        let path = usize::from(unsafe { (*cell).kernels[number][0] } == before[0]);
        // SAFETY: as above; the forms are no part of the compiler's cell.
        let forms = unsafe { &mut (*amx_cell).forms };
        forms.counts[key][path] += 1;
        if let Some(ticks) = ended {
            let sampled = &mut forms.sampled[key][path];
            sampled[0] += ticks;
            sampled[1] += 1;
        }
    }
    done
}

/// The routine blocks call under step timing (`Machine::enter_amx_cell`):
/// `amx_routine::routine`, or the model for the operations the cell leaves
/// to it (`routines::complete`), with each register operation completed
/// counted by form and path and every [`SAMPLE`]th call of a form timed.
///
/// # Safety
/// As `amx_routine::routine`, and `cell` is the `cell` of an [`AmxCell`].
#[cfg(target_arch = "x86_64")]
pub unsafe extern "sysv64" fn routine(cell: *mut Cell, operation: u32, operand: u64) -> u64 {
    // SAFETY: the caller's contract.
    unsafe { counted(cell, operation, operand) }
}

/// Only x86-64 runs translated code; ARM hosts compile the routine alone.
///
/// # Safety
/// As on x86-64.
#[cfg(not(target_arch = "x86_64"))]
pub unsafe extern "C" fn routine(cell: *mut Cell, operation: u32, operand: u64) -> u64 {
    // SAFETY: the caller's contract.
    unsafe { counted(cell, operation, operand) }
}

/// What a PE's AMX unit did, for the profile's lines: moves and register
/// operations completed in blocks and by the engine's service, and every
/// register operation by form, which step timing counts ([`AmxForms`]).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AmxCounts {
    pub moves: [u64; 2],
    pub operations: [u64; 2],
    pub forms: AmxForms,
}

impl AddAssign for AmxCounts {
    fn add_assign(&mut self, other: Self) {
        for (total, count) in self
            .moves
            .iter_mut()
            .chain(&mut self.operations)
            .zip(other.moves.iter().chain(&other.operations))
        {
            *total += count;
        }
        self.forms += other.forms;
    }
}

impl AmxCounts {
    /// The profile line of what was done after `before` until `self`:
    /// `ops=B/S moves=B/S native=P% forms=N`, the register operations and
    /// moves completed in blocks and by the service, the share of the
    /// counted operations a host kernel ran and how many forms ran; then
    /// the `shown` forms that ran most, as `name:native/model/ns/ns`, their
    /// operations by path and the mean nanoseconds of their sampled calls
    /// in blocks by path at `tsc_hz` (`-` where none was sampled).
    pub fn line(&self, before: &Self, tsc_hz: u64, shown: usize) -> String {
        let since = |now: u64, then: u64| now.saturating_sub(then);
        let mut line = format!(
            "ops={}/{} moves={}/{}",
            since(self.operations[0], before.operations[0]),
            since(self.operations[1], before.operations[1]),
            since(self.moves[0], before.moves[0]),
            since(self.moves[1], before.moves[1]),
        );
        let counts = |key: usize| -> [u64; 2] {
            core::array::from_fn(|path| {
                since(self.forms.counts[key][path], before.forms.counts[key][path])
            })
        };
        let mut ran: Vec<(u64, usize)> = (0..FORMS)
            .map(|key| (counts(key).iter().sum(), key))
            .filter(|&(operations, _)| operations != 0)
            .collect();
        let native: u64 = (0..FORMS).map(|key| counts(key)[0]).sum();
        let all: u64 = ran.iter().map(|&(operations, _)| operations).sum();
        match (native * 100).checked_div(all) {
            Some(share) => line.push_str(&format!(" native={share}%")),
            None => line.push_str(" native=-"),
        }
        line.push_str(&format!(" forms={}", ran.len()));
        ran.sort_unstable_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
        for &(_, key) in ran.iter().take(shown) {
            let [native, model] = counts(key);
            line.push_str(&format!(" {}:{native}/{model}", FormName(key)));
            for path in 0..2 {
                let [ticks, calls] = core::array::from_fn(|part| {
                    since(
                        self.forms.sampled[key][path][part],
                        before.forms.sampled[key][path][part],
                    )
                });
                match (u128::from(ticks) * 1_000_000_000)
                    .checked_div(u128::from(tsc_hz) * u128::from(calls))
                {
                    Some(nanoseconds) => line.push_str(&format!("/{nanoseconds}")),
                    None => line.push_str("/-"),
                }
            }
        }
        line
    }
}

#[cfg(test)]
mod tests;

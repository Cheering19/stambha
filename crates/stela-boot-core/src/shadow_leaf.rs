//! The bits of a shadow page-table leaf (`stela-boot`'s `engine::host`
//! faults): the x86 entry a demand fill, a prefill and fault-around write
//! for a guest page the walk translated.
//!
//! Every leaf is written with Accessed set, and a writable one with Dirty
//! set too. Nothing reads them back: the host keeps no dirty or access
//! tracking in its shadow tables (guest dirty and access state is the
//! guest's own descriptors', which the engine's walker never updates, no
//! FEAT_HAFDBS). Left clear, the processor sets each with a locked update
//! of the entry at the first access, and Dirty at the first write, which
//! costs every freshly written entry (tens of millions a minute after the
//! whole-TLB flushes a guest issues) one or two atomic page-table writes,
//! and may keep a core from merging the four 4 KiB entries of one 16 KiB
//! guest page into one TLB entry.

/// x86 page-table entry bits (Intel SDM Vol. 3A, 4.5, table 4-20).
pub const PRESENT: u64 = 1;
pub const WRITABLE: u64 = 1 << 1;
pub const USER: u64 = 1 << 2;
pub const ACCESSED: u64 = 1 << 5;
pub const DIRTY: u64 = 1 << 6;
pub const NX: u64 = 1 << 63;

/// The rights of a shadow leaf: present, never executable (translated code
/// runs from the code window, not guest pages), writable as the guest's
/// translation is, a user page where SMAP must guard it (`guarded`), and
/// Accessed, with Dirty on a writable leaf.
pub const fn rights(writable: bool, guarded: bool) -> u64 {
    PRESENT
        | NX
        | ACCESSED
        | if writable { WRITABLE | DIRTY } else { 0 }
        | if guarded { USER } else { 0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_leaf_is_accessed_and_a_writable_one_dirty() {
        for writable in [false, true] {
            for guarded in [false, true] {
                let leaf = rights(writable, guarded);
                assert_eq!(leaf & (PRESENT | NX | ACCESSED), PRESENT | NX | ACCESSED);
                assert_eq!(leaf & WRITABLE != 0, writable);
                // Dirty only where a write may set it: a read-only leaf
                // stays clean, as the processor would leave it.
                assert_eq!(leaf & DIRTY != 0, writable);
                assert_eq!(leaf & USER != 0, guarded);
                // No other bit: no address, caching or global bits.
                let known = PRESENT | WRITABLE | USER | ACCESSED | DIRTY | NX;
                assert_eq!(leaf & !known, 0);
            }
        }
        assert_eq!(rights(false, false), 0x8000_0000_0000_0021);
        assert_eq!(rights(true, false), 0x8000_0000_0000_0063);
        assert_eq!(rights(true, true), 0x8000_0000_0000_0067);
    }
}

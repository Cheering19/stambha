//! Lines a GPU's boot log carries for whoever reads it after a failure on a
//! PC nobody here can test: a one-time summary of what the GPU and its
//! driver are, and a bound on lines that repeat.
use alloc::{collections::BTreeMap, format, string::String};

/// Counts a keyed line's repeats and says which to print: the first, then
/// each power of two, so a line that repeats every frame costs a log a few
/// lines that still show how often it came.
#[derive(Default)]
pub struct Repeats {
    counts: BTreeMap<String, u64>,
}

/// Keys a [`Repeats`] tracks; a key past them is printed at the powers of
/// two of the untracked count.
const KEYS: usize = 256;

impl Repeats {
    pub const fn new() -> Self {
        Self {
            counts: BTreeMap::new(),
        }
    }

    /// Count one more `key`: the suffix to print it with, `Some("")` the
    /// first time and `Some(" (seen N times)")` at each power of two after,
    /// `None` when it is not printed.
    pub fn count(&mut self, key: &str) -> Option<String> {
        let tracked = self.counts.len() < KEYS || self.counts.contains_key(key);
        let key = if tracked { key } else { "" };
        let count = self.counts.entry(String::from(key)).or_insert(0);
        *count += 1;
        let count = *count;
        match (count.is_power_of_two(), tracked) {
            (false, _) => None,
            (true, true) if count == 1 => Some(String::new()),
            (true, true) => Some(format!(" (seen {count} times)")),
            (true, false) => Some(format!(" ({count} past the log's table)")),
        }
    }
}

/// What a triager needs first of a GPU that opened, or failed to open, the
/// Metal service: printed once per GPU as `GPU SUMMARY …`.
pub struct Summary<'a> {
    /// The GPU's PCI address.
    pub address: &'a str,
    pub vendor: u16,
    pub device: u16,
    pub revision: u8,
    /// The kernel driver's name.
    pub driver: &'a str,
    /// What the kernel driver says of itself and the chip: its version, the
    /// chip's family, the firmware versions and the memory it reports.
    pub facts: &'a str,
    /// Whether the GPU is built into the processor, sharing system memory.
    pub integrated: bool,
    /// The memory Metal tells the guest it works in.
    pub working_set: Option<u64>,
    /// The Vulkan driver's heaps, `size/kind` each.
    pub heaps: &'a str,
    /// Stela's heap: bytes in use and its size.
    pub heap_live: usize,
    pub heap_total: usize,
}

impl core::fmt::Display for Summary<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mib = |bytes: u64| bytes >> 20;
        write!(
            f,
            "GPU SUMMARY {} {:04x}:{:04x} rev {:#04x} driver {}",
            self.address, self.vendor, self.device, self.revision, self.driver
        )?;
        if !self.facts.is_empty() {
            write!(f, " {}", self.facts)?;
        }
        if self.integrated {
            write!(
                f,
                "; integrated: its memory is system memory from Stela's heap of {}M ({}M in use)",
                mib(self.heap_total as u64),
                mib(self.heap_live as u64)
            )?;
        } else {
            f.write_str("; discrete")?;
        }
        match self.working_set {
            Some(bytes) => write!(f, "; Metal works in {}M", mib(bytes))?,
            None => f.write_str("; the driver gave no memory size")?,
        }
        if self.heaps.is_empty() {
            f.write_str("; no Vulkan device")
        } else {
            write!(f, "; Vulkan heaps{}", self.heaps)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    #[test]
    fn a_repeated_line_prints_at_its_first_and_each_power_of_two() {
        let mut repeats = Repeats::new();
        let printed: Vec<(u64, String)> = (1..=17)
            .filter_map(|n| repeats.count("amdgpu_irq.c:670").map(|suffix| (n, suffix)))
            .collect();
        assert_eq!(
            printed,
            [
                (1, String::new()),
                (2, String::from(" (seen 2 times)")),
                (4, String::from(" (seen 4 times)")),
                (8, String::from(" (seen 8 times)")),
                (16, String::from(" (seen 16 times)")),
            ]
        );
        // Another line is counted apart.
        assert_eq!(repeats.count("dma-mapping.h:668"), Some(String::new()));
    }

    #[test]
    fn keys_past_the_table_share_one_count() {
        let mut repeats = Repeats::new();
        for key in 0..KEYS {
            assert_eq!(repeats.count(&format!("{key}")), Some(String::new()));
        }
        assert_eq!(
            repeats.count("new"),
            Some(String::from(" (1 past the log's table)"))
        );
        assert_eq!(
            repeats.count("other"),
            Some(String::from(" (2 past the log's table)"))
        );
        assert_eq!(repeats.count("third"), None);
        // A tracked key still counts on its own.
        assert_eq!(repeats.count("0"), Some(String::from(" (seen 2 times)")));
    }

    #[test]
    fn the_summary_of_an_integrated_gpu_names_the_heap_its_memory_comes_from() {
        let summary = Summary {
            address: "0000:01:00.0",
            vendor: 0x1002,
            device: 0x13c0,
            revision: 0xcb,
            driver: "amdgpu",
            facts: "3.64.0 family GC_10_3_6",
            integrated: true,
            working_set: Some(1893 << 20),
            heaps: " 635M/host 1270M/local",
            heap_live: 512 << 20,
            heap_total: 3300 << 20,
        };
        assert_eq!(
            format!("{summary}"),
            "GPU SUMMARY 0000:01:00.0 1002:13c0 rev 0xcb driver amdgpu 3.64.0 family \
             GC_10_3_6; integrated: its memory is system memory from Stela's heap of 3300M \
             (512M in use); Metal works in 1893M; Vulkan heaps 635M/host 1270M/local"
        );
        let summary = Summary {
            integrated: false,
            working_set: None,
            heaps: "",
            facts: "",
            ..summary
        };
        assert_eq!(
            format!("{summary}"),
            "GPU SUMMARY 0000:01:00.0 1002:13c0 rev 0xcb driver amdgpu; discrete; the driver \
             gave no memory size; no Vulkan device"
        );
    }
}

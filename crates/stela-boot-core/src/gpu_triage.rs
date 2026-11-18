//! Lines a GPU's boot log carries for whoever reads it after a failure on a
//! PC nobody here can test: a bound on lines that repeat.
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
}

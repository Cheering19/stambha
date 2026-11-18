//! Thinning of a log's identical lines in a row: the first is written, each
//! power of two of its repeats leaves one count line, and the next different
//! line is preceded by the total, so a line Linux prints once per retry (an
//! amdgpu submission refused for memory, which RADV retries every
//! millisecond for a second) costs a few lines, not thousands, and the count
//! is in the log even if the boot stops while it repeats.
//!
//! Lines are compared by a hash of their text, which needs no buffer: the
//! sink may run where nothing can be allocated.
use core::fmt;

/// What to write for a line.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Verdict {
    /// Write it, after a line saying the one before came `repeated` more
    /// times than the count already written, if any.
    Write { repeated: Option<u64> },
    /// The same as the one before: write nothing, or a line saying it has
    /// come `count` more times so far.
    Repeat { count: Option<u64> },
}

/// The last line's hash and its repeats.
#[derive(Debug, Default)]
pub struct Repeats {
    last: Option<u64>,
    count: u64,
    written: u64,
}

impl Repeats {
    /// What to write for the line `text`.
    pub fn line(&mut self, text: fmt::Arguments<'_>) -> Verdict {
        self.hashed(hash(text))
    }

    fn hashed(&mut self, hash: u64) -> Verdict {
        if self.last == Some(hash) {
            self.count += 1;
            if self.count.is_power_of_two() {
                self.written = self.count;
                return Verdict::Repeat {
                    count: Some(self.count),
                };
            }
            return Verdict::Repeat { count: None };
        }
        let repeated = (self.count > self.written).then_some(self.count);
        *self = Self {
            last: Some(hash),
            count: 0,
            written: 0,
        };
        Verdict::Write { repeated }
    }
}

/// The line that counts repeats of the line Linux last wrote: `count` more
/// so far, or in all once a different line follows (`ended`). It names that
/// line as Linux's, as the boot's own lines may come between them.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Counted {
    pub count: u64,
    pub ended: bool,
}

impl fmt::Display for Counted {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let when = if self.ended { "in all" } else { "so far" };
        write!(
            f,
            "the line Linux last wrote came {} more times {when}",
            self.count
        )
    }
}

/// FNV-1a over the formatted text.
fn hash(text: fmt::Arguments<'_>) -> u64 {
    struct Fnv(u64);
    impl fmt::Write for Fnv {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            for byte in s.bytes() {
                self.0 = (self.0 ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3);
            }
            Ok(())
        }
    }
    let mut fnv = Fnv(0xcbf2_9ce4_8422_2325);
    let _ = fmt::write(&mut fnv, text);
    fnv.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    #[test]
    fn identical_lines_are_counted_at_powers_of_two_and_totalled() {
        let mut repeats = Repeats::default();
        let refused = format_args!("<3> amdgpu: Not enough memory for command submission!");
        assert_eq!(repeats.line(refused), Verdict::Write { repeated: None });
        let counts: Vec<_> = (0..4265)
            .filter_map(|_| match repeats.line(refused) {
                Verdict::Repeat { count } => count,
                Verdict::Write { .. } => panic!("a repeat written"),
            })
            .collect();
        assert_eq!(
            counts,
            [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096]
        );
        assert_eq!(
            repeats.line(format_args!("<6> other")),
            Verdict::Write {
                repeated: Some(4265)
            }
        );
        assert_eq!(
            repeats.line(format_args!("<6> {}", "other")),
            Verdict::Repeat { count: Some(1) }
        );
        // A total already written is not written again.
        assert_eq!(
            repeats.line(format_args!("<6> third")),
            Verdict::Write { repeated: None }
        );
    }

    /// A count names the line it counts as Linux's last, not the line above
    /// it in the log, which may be the boot's own.
    #[test]
    fn a_count_names_the_line_linux_last_wrote() {
        use alloc::format;
        assert_eq!(
            format!(
                "{}",
                Counted {
                    count: 4,
                    ended: false
                }
            ),
            "the line Linux last wrote came 4 more times so far"
        );
        assert_eq!(
            format!(
                "{}",
                Counted {
                    count: 4265,
                    ended: true
                }
            ),
            "the line Linux last wrote came 4265 more times in all"
        );
    }

    #[test]
    fn different_lines_are_all_written() {
        let mut repeats = Repeats::default();
        for line in ["a", "b", "a", "b"] {
            assert_eq!(
                repeats.line(format_args!("{line}")),
                Verdict::Write { repeated: None }
            );
        }
    }
}

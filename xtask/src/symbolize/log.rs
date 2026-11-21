//! The `PES PC` lines of a boot log (`step_timing::PcSamples::lines`): each
//! beat's group and, every 16 beats, a `PES PC TOTAL` group over every beat
//! since the first. A group is a header line, then a line per level with
//! samples that lists its blocks with the most cycles.
use anyhow::{Context, Result, bail};
use std::collections::{BTreeMap, HashSet};
use stela_engine::machine::engine::step_timing::{Level, pc_samples::level_name};

/// The levels in the order the lines print them.
pub(super) const LEVELS: [Level; 3] = [Level::User, Level::Kernel, Level::Guarded];

/// One block a level line lists: its PC as printed (`0x...`, `v+0x...`,
/// `v-0x...`), its page key's first 8 bytes and its offset there, and its
/// samples and cycles in millions.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Entry {
    pub level: Level,
    pub pc: String,
    pub page: Option<u64>,
    pub offset: u16,
    pub samples: u64,
    pub cycles: u64,
}

/// One group's header and level lines.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct Group {
    total: bool,
    samples: u64,
    cycles: u64,
    gone: [u64; 2],
    other: [u64; 2],
    /// Samples and cycles by level.
    levels: [[u64; 2]; 3],
    entries: Vec<Entry>,
}

/// Which lines a [`Profile`] sums.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Source {
    /// The `taken`th of `of` total groups, the last.
    Total { taken: usize, of: usize },
    /// Every beat group, `of` of them.
    Beats { of: usize },
}

/// The samples a log's lines hold: the counts (cycles in millions) and
/// the listed blocks, each once, most cycles first.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Profile {
    pub source: Source,
    pub samples: u64,
    pub cycles: u64,
    pub gone: [u64; 2],
    pub other: [u64; 2],
    pub levels: [[u64; 2]; 3],
    pub entries: Vec<Entry>,
}

/// An entry's key while groups are summed: level, page, offset, PC.
type Key = (usize, Option<u64>, u16, String);

impl Profile {
    /// The last total group of `text`, or with `beats` (or without a
    /// total) the sum of its beat groups.
    pub(super) fn parse(text: &str, beats: bool) -> Result<Self> {
        let groups = groups(text)?;
        let totals: Vec<&Group> = groups.iter().filter(|group| group.total).collect();
        let beat_groups: Vec<&Group> = groups.iter().filter(|group| !group.total).collect();
        let (source, chosen) = match totals.last() {
            Some(&last) if !beats => (
                Source::Total {
                    taken: totals.len(),
                    of: totals.len(),
                },
                vec![last],
            ),
            _ if !beat_groups.is_empty() => (
                Source::Beats {
                    of: beat_groups.len(),
                },
                beat_groups,
            ),
            _ => bail!(
                "the log has no `PES PC` lines: boot with `--profile` on a host whose \
                 counters sample (its log has `PES WHERE` lines)"
            ),
        };
        let mut profile = Profile {
            source,
            samples: 0,
            cycles: 0,
            gone: [0; 2],
            other: [0; 2],
            levels: [[0; 2]; 3],
            entries: Vec::new(),
        };
        let mut entries: BTreeMap<Key, (u64, u64)> = BTreeMap::new();
        for group in chosen {
            profile.samples += group.samples;
            profile.cycles += group.cycles;
            for index in 0..2 {
                profile.gone[index] += group.gone[index];
                profile.other[index] += group.other[index];
            }
            for (level, counts) in profile.levels.iter_mut().enumerate() {
                counts[0] += group.levels[level][0];
                counts[1] += group.levels[level][1];
            }
            for entry in &group.entries {
                let key = (
                    level_index(entry.level),
                    entry.page,
                    entry.offset,
                    entry.pc.clone(),
                );
                let counts = entries.entry(key).or_default();
                counts.0 += entry.samples;
                counts.1 += entry.cycles;
            }
        }
        profile.entries = entries
            .into_iter()
            .map(|((level, page, offset, pc), (samples, cycles))| Entry {
                level: LEVELS[level],
                pc,
                page,
                offset,
                samples,
                cycles,
            })
            .collect();
        // Stable: equal entries keep the key's order, level first.
        profile
            .entries
            .sort_by(|a, b| b.cycles.cmp(&a.cycles).then(b.samples.cmp(&a.samples)));
        Ok(profile)
    }

    /// The page keys of the listed blocks.
    pub(super) fn pages(&self) -> HashSet<u64> {
        self.entries.iter().filter_map(|entry| entry.page).collect()
    }
}

/// A level's index in [`LEVELS`].
pub(super) fn level_index(level: Level) -> usize {
    LEVELS.iter().position(|&l| l == level).unwrap_or(0)
}

/// The `PES PC` groups of `text`, in order.
fn groups(text: &str) -> Result<Vec<Group>> {
    const PREFIX: &str = "PES PC ";
    let mut groups: Vec<Group> = Vec::new();
    for (number, line) in text.lines().enumerate() {
        let Some(at) = line.find(PREFIX) else {
            continue;
        };
        let rest = &line[at + PREFIX.len()..];
        let (total, rest) = match rest.strip_prefix("TOTAL ") {
            Some(rest) => (true, rest),
            None => (false, rest),
        };
        group_line(&mut groups, total, rest)
            .with_context(|| format!("line {}: {}", number + 1, line.trim()))?;
    }
    Ok(groups)
}

/// Add one line's words, `rest` after its prefix, to `groups`: a header
/// starts a group, a level line fills the last.
fn group_line(groups: &mut Vec<Group>, total: bool, rest: &str) -> Result<()> {
    let mut words = rest.split_ascii_whitespace();
    let first = words.next().context("an empty line")?;
    if first.starts_with("samples=") {
        let mut group = Group {
            total,
            ..Group::default()
        };
        for word in std::iter::once(first).chain(words) {
            let (name, value) = word.split_once('=').context("a `name=value` word")?;
            match name {
                "samples" => group.samples = number(value)?,
                "cycles" => group.cycles = millions(value)?,
                "gone" => group.gone = pair(value)?,
                "other" => group.other = pair(value)?,
                _ => {}
            }
        }
        groups.push(group);
        return Ok(());
    }
    let level = LEVELS
        .into_iter()
        .find(|&level| level_name(level) == first)
        .with_context(|| format!("an unknown level `{first}`"))?;
    let group = groups
        .last_mut()
        .filter(|group| group.total == total)
        .context("a level line before its group's header")?;
    let index = level_index(level);
    for word in words {
        if word.contains('@') {
            group.entries.push(entry(level, word)?);
            continue;
        }
        let (name, value) = word.split_once('=').context("a `name=value` word")?;
        match name {
            "samples" => group.levels[index][0] = number(value)?,
            "cycles" => group.levels[index][1] = millions(value)?,
            _ => {}
        }
    }
    Ok(())
}

/// A block word, `{pc}:{samples}:{cyclesM}@{page}+{offset}`.
fn entry(level: Level, word: &str) -> Result<Entry> {
    let (counts, place) = word.split_once('@').context("a block's `@`")?;
    let (page, offset) = place.split_once('+').context("a block's `+offset`")?;
    let mut fields = counts.rsplitn(3, ':');
    let (Some(cycles), Some(samples), Some(pc)) = (fields.next(), fields.next(), fields.next())
    else {
        bail!("a block's `pc:samples:cycles` in `{word}`");
    };
    let page = match page {
        "?" => None,
        page => Some(u64::from_str_radix(page, 16).with_context(|| format!("a page `{page}`"))?),
    };
    Ok(Entry {
        level,
        pc: pc.into(),
        page,
        offset: u16::from_str_radix(offset, 16).with_context(|| format!("an offset `{offset}`"))?,
        samples: number(samples)?,
        cycles: number(cycles)?,
    })
}

fn number(text: &str) -> Result<u64> {
    text.parse().with_context(|| format!("a number `{text}`"))
}

/// `NM`: N millions.
fn millions(text: &str) -> Result<u64> {
    number(text.strip_suffix('M').unwrap_or(text))
}

/// `N/CM`: samples and their cycles in millions.
fn pair(text: &str) -> Result<[u64; 2]> {
    let (samples, cycles) = text.split_once('/').context("a `samples/cyclesM` pair")?;
    Ok([number(samples)?, millions(cycles)?])
}

#[cfg(test)]
mod tests {
    use super::*;

    const LOG: &str = "\
[   10.000] PES WHERE all: ins=1M cyc=2M
[   20.000] PES PC samples=3 cycles=50M gone=1/16M other=0/0M
[   20.000] PES PC user samples=2 cycles=33M blocks=2 0x18a2b3c40:1:17@1f2e3d4c5b6a7980+c40 0x100:1:16@?+100
[   20.000] PES PC kernel samples=1 cycles=16M blocks=1 v+0x2c4d0:1:16@00000000000000ab+4d0
[   30.000] PES PC samples=2 cycles=34M gone=0/0M other=1/1M
[   30.000] PES PC user samples=2 cycles=34M blocks=1 0x18a2b3c40:2:34@1f2e3d4c5b6a7980+c40
[   30.000] PES PC TOTAL samples=5 cycles=84M gone=1/16M other=1/1M
[   30.000] PES PC TOTAL user samples=4 cycles=67M blocks=2 0x18a2b3c40:3:51@1f2e3d4c5b6a7980+c40 0x100:1:16@?+100
[   30.000] PES PC TOTAL kernel samples=1 cycles=16M blocks=1 v+0x2c4d0:1:16@00000000000000ab+4d0
[   40.000] PES PC samples=1 cycles=16M gone=0/0M other=0/0M
[   40.000] PES PC guarded samples=1 cycles=16M blocks=1 v-0x40:1:16@00000000000000cd+fc0
";

    fn entry(
        level: Level,
        pc: &str,
        page: Option<u64>,
        offset: u16,
        samples: u64,
        cycles: u64,
    ) -> Entry {
        Entry {
            level,
            pc: pc.into(),
            page,
            offset,
            samples,
            cycles,
        }
    }

    #[test]
    fn the_last_total_is_read_by_default() {
        let profile = Profile::parse(LOG, false).unwrap();
        assert_eq!(profile.source, Source::Total { taken: 1, of: 1 });
        assert_eq!((profile.samples, profile.cycles), (5, 84));
        assert_eq!(profile.gone, [1, 16]);
        assert_eq!(profile.other, [1, 1]);
        assert_eq!(profile.levels, [[4, 67], [1, 16], [0, 0]]);
        assert_eq!(
            profile.entries,
            [
                entry(
                    Level::User,
                    "0x18a2b3c40",
                    Some(0x1f2e_3d4c_5b6a_7980),
                    0xc40,
                    3,
                    51
                ),
                entry(Level::User, "0x100", None, 0x100, 1, 16),
                entry(Level::Kernel, "v+0x2c4d0", Some(0xab), 0x4d0, 1, 16),
            ]
        );
        assert_eq!(
            profile.pages(),
            HashSet::from([0x1f2e_3d4c_5b6a_7980, 0xab])
        );
    }

    #[test]
    fn beats_sum_every_beat_group() {
        let profile = Profile::parse(LOG, true).unwrap();
        assert_eq!(profile.source, Source::Beats { of: 3 });
        assert_eq!((profile.samples, profile.cycles), (6, 100));
        assert_eq!(profile.gone, [1, 16]);
        assert_eq!(profile.other, [1, 1]);
        assert_eq!(profile.levels, [[4, 67], [1, 16], [1, 16]]);
        assert_eq!(
            profile.entries,
            [
                entry(
                    Level::User,
                    "0x18a2b3c40",
                    Some(0x1f2e_3d4c_5b6a_7980),
                    0xc40,
                    3,
                    51
                ),
                entry(Level::User, "0x100", None, 0x100, 1, 16),
                entry(Level::Kernel, "v+0x2c4d0", Some(0xab), 0x4d0, 1, 16),
                entry(Level::Guarded, "v-0x40", Some(0xcd), 0xfc0, 1, 16),
            ]
        );
    }

    #[test]
    fn a_log_without_a_total_sums_its_beats() {
        let beats: String = LOG
            .lines()
            .filter(|line| !line.contains("TOTAL"))
            .map(|line| format!("{line}\n"))
            .collect();
        let profile = Profile::parse(&beats, false).unwrap();
        assert_eq!(profile.source, Source::Beats { of: 3 });
        assert_eq!(profile.cycles, 100);
    }

    #[test]
    fn a_log_without_lines_or_with_a_broken_entry_is_refused() {
        let none = Profile::parse("PES WHERE all: ins=1M\nPES HOT user samples=3\n", false);
        assert!(none.unwrap_err().to_string().contains("no `PES PC` lines"));
        let broken = "PES PC samples=1 cycles=16M gone=0/0M other=0/0M\n\
                      PES PC user samples=1 cycles=16M blocks=1 0x100:1@?+100\n";
        let error = Profile::parse(broken, false).unwrap_err();
        assert!(format!("{error:#}").contains("line 2"), "{error:#}");
        // A level line before any header.
        let orphan = "PES PC user samples=1 cycles=16M blocks=1 0x100:1:16@?+100\n";
        assert!(Profile::parse(orphan, false).is_err());
    }

    /// The lines the engine prints parse back into the blocks it held.
    #[test]
    fn the_engines_lines_parse_back() {
        use stela_engine::machine::engine::step_timing::{PcHit, PcSamples};
        let mut samples = PcSamples::default();
        let hits = [
            (
                0x1_8a2b_3c40u64,
                Level::User,
                Some(0x1f2e_3d4c_5b6a_7980u64),
                0xc40u16,
                33_554_432u64,
            ),
            (
                0x1_8a2b_3c40,
                Level::User,
                Some(0x1f2e_3d4c_5b6a_7980),
                0xc40,
                16_777_216,
            ),
            (0x2c4d0, Level::Kernel, None, 0x4d0, 20_000_000),
            (
                0u64.wrapping_sub(0x40),
                Level::Guarded,
                Some(7),
                0xfc0,
                16_777_216,
            ),
        ];
        for (pc, level, page, offset, cycles) in hits {
            samples.add(PcHit {
                pc,
                level,
                page,
                offset,
                cycles,
            });
        }
        samples.add_gone(1, 16_777_216);
        let text: String = samples
            .lines("PES PC TOTAL", 8)
            .iter()
            .map(|line| format!("[   1.000] {line}\n"))
            .collect();
        let profile = Profile::parse(&text, false).unwrap();
        assert_eq!(profile.source, Source::Total { taken: 1, of: 1 });
        assert_eq!(profile.samples, 4);
        assert_eq!(profile.gone, [1, 16]);
        assert_eq!(
            profile.entries,
            [
                entry(
                    Level::User,
                    "0x18a2b3c40",
                    Some(0x1f2e_3d4c_5b6a_7980),
                    0xc40,
                    2,
                    50
                ),
                entry(Level::Kernel, "v+0x2c4d0", None, 0x4d0, 1, 20),
                entry(Level::Guarded, "v-0x40", Some(7), 0xfc0, 1, 16),
            ]
        );
    }
}

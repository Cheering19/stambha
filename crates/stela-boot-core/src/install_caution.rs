//! The page Install macOS opens before its disk picker on a PC whose
//! firmware shows, before Stela starts, no graphics card Stela can start
//! macOS with (`stela-boot`'s `choice::menu`). The install draws through no
//! GPU and would finish, an hour or more later, and the installed macOS then
//! stops at its first start: four reporters met exactly that. The page says
//! why and asks; it refuses nothing, since a card the firmware keeps powered
//! off answers only after Stela has started (`NvidiaNotice::NotYet`).
//!
//! Focus starts on Back, so Enter pressed out of habit goes back to the
//! menu; Right or Tab reaches Install anyway, as the disk picker's
//! confirmation does. What the page holds and how keys change it are here,
//! where the host tests reach them; `stela-boot` paints it.
use alloc::vec::Vec;
use stela_loader::host_firmware::bundle::NvidiaNotice;

/// Said on a PC with no NVIDIA graphics card: the install itself draws
/// through no GPU and would finish, and the installed macOS then could not
/// start.
pub const NO_NVIDIA_GPU: &str = "No NVIDIA graphics card found: macOS can be installed, but it \
    starts only with a GeForce GTX 16 or RTX 20, 30, 40 or 50 series card.";

/// Said on a PC whose NVIDIA cards are all of chips Stela has no driver
/// for, older or newer than the series it names.
pub const NO_DRIVEN_NVIDIA_GPU: &str = "The NVIDIA graphics card here is not one Stela drives: \
    macOS can be installed, but it starts only with a GeForce GTX 16 or RTX 20, 30, 40 or 50 \
    series card.";

/// Said when no NVIDIA card Stela drives answers yet but a PCI port shows
/// nothing below it: a card the firmware left powered off there, as on some
/// laptops, answers only after Stela powers the port on.
pub const NO_NVIDIA_GPU_YET: &str = "No NVIDIA graphics card Stela drives found yet: a card the \
    PC keeps powered off, as some laptops do, is found only after Stela has started. macOS can \
    be installed, but it starts only with a GeForce GTX 16 or RTX 20, 30, 40 or 50 series card.";

/// The page's title.
pub const TITLE: &str = "Install macOS on this PC?";

/// The page's question, under the reason.
pub const ASK: &str = "Installing takes an hour or more. Choose Install anyway only if you \
    know this PC has one of those cards.";

/// The words for the firmware census's `notice`; none when a card Stela
/// drives answers.
pub fn gpu_words(notice: NvidiaNotice) -> Option<&'static str> {
    match notice {
        NvidiaNotice::Driven => None,
        NvidiaNotice::Absent => Some(NO_NVIDIA_GPU),
        NvidiaNotice::NotDriven => Some(NO_DRIVEN_NVIDIA_GPU),
        NvidiaNotice::NotYet => Some(NO_NVIDIA_GPU_YET),
    }
}

/// A key as the page reads it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Key {
    Left,
    Right,
    Tab,
    Enter,
    Escape,
    Other,
}

/// The page's buttons, left to right.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Focus {
    Back,
    Install,
}

/// What a key press leads to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    Stay,
    /// Back to the startup menu, nothing started.
    Back,
    /// On to the disk picker.
    Install,
}

/// The page: its reason and the button with the focus.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Caution {
    pub reason: &'static str,
    focus: Focus,
}

impl Caution {
    /// The page for the census's `notice`; `None` when a card Stela drives
    /// answers, or when another vendor's card this image drives does
    /// (`other_drawn`), and Install macOS goes on to the picker unasked.
    pub fn new(notice: NvidiaNotice, other_drawn: bool) -> Option<Self> {
        if other_drawn {
            return None;
        }
        Some(Self {
            reason: gpu_words(notice)?,
            focus: Focus::Back,
        })
    }

    pub fn focus(&self) -> Focus {
        self.focus
    }

    pub fn press(&mut self, key: Key) -> Step {
        match key {
            Key::Escape => Step::Back,
            Key::Left => {
                self.focus = Focus::Back;
                Step::Stay
            }
            Key::Right => {
                self.focus = Focus::Install;
                Step::Stay
            }
            Key::Tab => {
                self.focus = match self.focus {
                    Focus::Back => Focus::Install,
                    Focus::Install => Focus::Back,
                };
                Step::Stay
            }
            Key::Enter => match self.focus {
                Focus::Back => Step::Back,
                Focus::Install => Step::Install,
            },
            Key::Other => Step::Stay,
        }
    }
}

/// `text` in lines of at most `columns` characters, broken between words;
/// a word longer than a line has a line of its own.
pub fn wrap(text: &str, columns: usize) -> Vec<&str> {
    let columns = columns.max(1);
    let mut lines = Vec::new();
    let mut start: Option<usize> = None;
    let mut end = 0;
    for (at, word) in text
        .split_ascii_whitespace()
        .map(|word| (word.as_ptr() as usize - text.as_ptr() as usize, word))
    {
        let word_end = at + word.len();
        match start {
            Some(first) if word_end - first <= columns => end = word_end,
            Some(first) => {
                lines.push(&text[first..end]);
                (start, end) = (Some(at), word_end);
            }
            None => (start, end) = (Some(at), word_end),
        }
    }
    if let Some(first) = start {
        lines.push(&text[first..end]);
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_a_pc_without_a_card_stela_drives_is_asked() {
        assert_eq!(Caution::new(NvidiaNotice::Driven, false), None);
        for notice in [
            NvidiaNotice::Absent,
            NvidiaNotice::NotDriven,
            NvidiaNotice::NotYet,
        ] {
            let caution = Caution::new(notice, false).expect("asked");
            assert_eq!(Some(caution.reason), gpu_words(notice));
            assert_eq!(caution.focus(), Focus::Back);
            // Another vendor's card this image draws with is macOS's GPU.
            assert_eq!(Caution::new(notice, true), None);
        }
        assert_eq!(gpu_words(NvidiaNotice::Absent), Some(NO_NVIDIA_GPU));
        assert_eq!(
            gpu_words(NvidiaNotice::NotDriven),
            Some(NO_DRIVEN_NVIDIA_GPU)
        );
        assert_eq!(gpu_words(NvidiaNotice::NotYet), Some(NO_NVIDIA_GPU_YET));
    }

    #[test]
    fn enter_out_of_habit_goes_back() {
        let mut caution = Caution::new(NvidiaNotice::Absent, false).unwrap();
        assert_eq!(caution.press(Key::Enter), Step::Back);
        assert_eq!(caution.press(Key::Other), Step::Stay);
        assert_eq!(caution.press(Key::Escape), Step::Back);
    }

    #[test]
    fn install_anyway_takes_a_move_then_enter() {
        let mut caution = Caution::new(NvidiaNotice::NotYet, false).unwrap();
        assert_eq!(caution.press(Key::Right), Step::Stay);
        assert_eq!(caution.focus(), Focus::Install);
        assert_eq!(caution.press(Key::Enter), Step::Install);

        let mut caution = Caution::new(NvidiaNotice::NotYet, false).unwrap();
        assert_eq!(caution.press(Key::Tab), Step::Stay);
        assert_eq!(caution.focus(), Focus::Install);
        assert_eq!(caution.press(Key::Tab), Step::Stay);
        assert_eq!(caution.focus(), Focus::Back);
        caution.press(Key::Right);
        caution.press(Key::Left);
        assert_eq!(caution.press(Key::Enter), Step::Back);
        // Escape leaves even with Install focused.
        caution.press(Key::Right);
        assert_eq!(caution.press(Key::Escape), Step::Back);
    }

    #[test]
    fn wrap_breaks_between_words_within_the_width() {
        assert_eq!(wrap("", 10), Vec::<&str>::new());
        assert_eq!(wrap("one two three", 7), ["one two", "three"]);
        assert_eq!(wrap("  one   two  ", 20), ["one   two"]);
        assert_eq!(wrap("a verylongword b", 4), ["a", "verylongword", "b"]);
        for line in wrap(NO_NVIDIA_GPU_YET, 50) {
            assert!(line.len() <= 50, "{line:?}");
        }
        let joined: Vec<&str> = wrap(ASK, 30)
            .into_iter()
            .flat_map(str::split_ascii_whitespace)
            .collect();
        assert_eq!(joined, ASK.split_ascii_whitespace().collect::<Vec<_>>());
    }
}

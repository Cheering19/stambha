//! Which of a PC's display controllers Stela starts, in which order, and why
//! the others are left alone, from what each reports: whether a stack of
//! this image drives it, whether that stack draws (its Vulkan driver is in
//! the image), whether its driver's table calls it integrated, discrete or
//! leaves that to its probe, and whether the firmware's picture may be on
//! it. Nothing here knows a PC, a GPU model or a maker beyond that.
//!
//! The rule, a working picture first and the faster GPU second:
//!
//! - Every GPU whose stack draws may give the Metal device; a discrete GPU
//!   draws before an integrated one ([`crate::stack::metal_devices`]), so on
//!   a laptop the discrete GPU draws whether or not the panel is wired to it.
//! - Any display shows any drawing GPU's frames: the DCP model copies each
//!   frame from guest memory into the buffer of whatever shows it, a bound
//!   GPU's output or the firmware's framebuffer, at the same cost. So the
//!   GPU that draws need not drive the display.
//! - A GPU that would only show the frames another draws (an integrated GPU
//!   beside a drawing discrete one, or one whose stack does not draw) is
//!   started only where the firmware's picture may be on it: then its own
//!   driver lights that display (a laptop's panel on its integrated GPU).
//!   Elsewhere it is left with the firmware, untouched: no display needs it,
//!   and its start would cost time and could stop the boot.
//! - A GPU whose kind only its probe tells may be a second discrete GPU and
//!   is started, after the GPUs that surely draw, so its probe never delays
//!   or ends theirs.
//! - A GPU no stack drives is left with the firmware; its picture, if the
//!   firmware's is on it, keeps showing macOS by copy.
//! - When a GPU's claim is refused, the next one takes its place: a GPU left
//!   out only beside it is started instead.
use crate::stack::{drawing_kind, may_only_show_frames_of, only_shows_frames_of};
use alloc::{format, string::String, vec::Vec};

/// A display controller of the census, as the selection sees it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Found {
    /// What its stack says of it, or why no stack of this image drives it.
    pub stack: Result<Driven, &'static str>,
    /// The firmware's picture may be on it.
    pub firmware_screen: bool,
}

/// What a display controller's stack says of it before its start.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Driven {
    /// The stack draws: the image holds its Vulkan driver.
    pub draws: bool,
    /// Its driver's table calls it integrated, discrete, or leaves that to
    /// its probe ([`crate::stack::Kernel::table_integrated`]).
    pub integrated: Option<bool>,
}

/// What became of one display controller.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Choice {
    /// No stack of this image drives it.
    Unused(&'static str),
    /// Started in the first turn: it may give the Metal device.
    First,
    /// Started after the first turn: it may only show the frames the GPU
    /// `beside` (an index of the census) draws.
    Later { beside: usize },
    /// Left with the firmware: it would only show the frames `beside` draws,
    /// and the firmware's picture is not on it.
    Left { beside: usize },
    /// Its claim was refused; the claim says why.
    Refused,
}

/// The selection: each controller's choice, by its index in the census,
/// and the order the started ones are claimed in.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Selection {
    pub choices: Vec<Choice>,
    /// The claimed controllers, in the order they were claimed: drawing
    /// discrete ones first, then those whose kind their probe tells, then
    /// drawing integrated ones, then those that only show, each group in the
    /// firmware's order. Their DRM minors follow it.
    pub claimed: Vec<usize>,
}

/// Selects among `found`, in the firmware's order. `claim` claims the
/// controller at an index, in the claim order, and answers whether it was
/// claimed.
pub fn select(found: &[Found], mut claim: impl FnMut(usize) -> bool) -> Selection {
    let mut choices: Vec<Choice> = found
        .iter()
        .map(|found| match found.stack {
            Ok(_) => Choice::Refused,
            Err(reason) => Choice::Unused(reason),
        })
        .collect();
    let mut order: Vec<(usize, Driven)> = found
        .iter()
        .enumerate()
        .filter_map(|(index, found)| found.stack.ok().map(|driven| (index, driven)))
        .collect();
    order.sort_by_key(|(_, driven)| crate::stack::claim_rank(driven.draws, driven.integrated));
    // The claimed drawing GPUs, each as it counts against the next.
    let mut drawing: Vec<(usize, bool)> = Vec::new();
    let mut claimed = Vec::new();
    for &(index, driven) in &order {
        let kinds: Vec<bool> = drawing.iter().map(|&(_, kind)| kind).collect();
        if let Some(beside) = only_shows_frames_of(&kinds, driven.draws, driven.integrated)
            && !found[index].firmware_screen
        {
            choices[index] = Choice::Left {
                beside: drawing[beside].0,
            };
            continue;
        }
        if !claim(index) {
            continue;
        }
        if driven.draws {
            drawing.push((index, drawing_kind(driven.integrated)));
        }
        claimed.push(index);
    }
    // The probe turns, in the claim order: a GPU that may only show another's
    // frames is probed after the GPUs that may give the Metal device.
    let mut first: Vec<(usize, bool)> = Vec::new();
    for &index in &claimed {
        let Ok(driven) = found[index].stack else {
            continue;
        };
        let kinds: Vec<bool> = first.iter().map(|&(_, kind)| kind).collect();
        choices[index] = match may_only_show_frames_of(&kinds, driven.draws, driven.integrated) {
            Some(beside) => Choice::Later {
                beside: first[beside].0,
            },
            None => {
                if driven.draws {
                    first.push((index, drawing_kind(driven.integrated)));
                }
                Choice::First
            }
        };
    }
    Selection { choices, claimed }
}

impl Driven {
    /// Its kind and whether it draws, in words.
    pub fn describe(self) -> String {
        let kind = match self.integrated {
            Some(true) => "integrated",
            Some(false) => "discrete",
            None => "integrated or discrete, as its probe tells",
        };
        let draws = match self.draws {
            true => "draws",
            false => "shows only: this image has no Vulkan driver for it",
        };
        format!("{kind}, {draws}")
    }
}

impl Choice {
    /// The choice in words; `name` names a controller of the census.
    pub fn describe(self, name: &dyn Fn(usize) -> String) -> String {
        match self {
            Self::Unused(reason) => format!("not used: {reason}"),
            Self::First => String::from("started first"),
            Self::Later { beside } => format!(
                "started after {}: it may only show the frames that GPU draws",
                name(beside)
            ),
            Self::Left { beside } => format!(
                "left with the firmware: it would only show the frames {} draws, and the \
                 firmware's picture is not on it",
                name(beside)
            ),
            Self::Refused => String::from("not started: its claim was refused"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    const DISCRETE: Driven = Driven {
        draws: true,
        integrated: Some(false),
    };
    /// An integrated GPU its table names (an Intel GPU, an older AMD APU).
    const INTEGRATED: Driven = Driven {
        draws: true,
        integrated: Some(true),
    };
    /// An AMD GPU amdgpu matches by class: its probe tells its kind.
    const PROBED: Driven = Driven {
        draws: true,
        integrated: None,
    };
    /// A stack without its Vulkan driver in the image (xe today).
    const SHOWS: Driven = Driven {
        draws: false,
        integrated: Some(true),
    };

    fn gpu(driven: Driven, firmware_screen: bool) -> Found {
        Found {
            stack: Ok(driven),
            firmware_screen,
        }
    }

    fn unsupported(firmware_screen: bool) -> Found {
        Found {
            stack: Err("no Stela GPU backend for this PCI vendor"),
            firmware_screen,
        }
    }

    /// The selection when every claim succeeds.
    fn all(found: &[Found]) -> Selection {
        select(found, |_| true)
    }

    #[test]
    fn a_desktop_with_one_card_starts_it() {
        for screen in [true, false] {
            let chosen = all(&[gpu(DISCRETE, screen)]);
            assert_eq!(chosen.choices, [Choice::First]);
            assert_eq!(chosen.claimed, [0]);
        }
    }

    #[test]
    fn a_desktop_with_a_card_and_an_integrated_gpu() {
        // The monitor on the card: the integrated GPU (bus 0, first in the
        // firmware's order) would only show the card's frames.
        let chosen = all(&[gpu(INTEGRATED, false), gpu(DISCRETE, true)]);
        assert_eq!(chosen.choices, [Choice::Left { beside: 1 }, Choice::First]);
        assert_eq!(chosen.claimed, [1]);
        // The monitor on the motherboard: the integrated GPU lights it with
        // the card's frames, probed after the card.
        let chosen = all(&[gpu(INTEGRATED, true), gpu(DISCRETE, false)]);
        assert_eq!(chosen.choices, [Choice::Later { beside: 1 }, Choice::First]);
        assert_eq!(chosen.claimed, [1, 0]);
        // An AMD iGPU of Ryzen 7000 or later, whose probe tells its kind: it
        // may be a second card, so it starts either way, after the card.
        for screen in [true, false] {
            let chosen = all(&[gpu(PROBED, screen), gpu(DISCRETE, !screen)]);
            assert_eq!(chosen.choices, [Choice::Later { beside: 1 }, Choice::First]);
            assert_eq!(chosen.claimed, [1, 0]);
        }
    }

    #[test]
    fn a_muxless_laptop_draws_on_the_discrete_gpu_and_shows_on_the_panels() {
        // The panel on the integrated GPU, the discrete GPU with no outputs
        // or only external ports: the discrete GPU draws, the integrated one
        // shows the panel.
        let laptop = [gpu(INTEGRATED, true), gpu(DISCRETE, false)];
        assert_eq!(
            all(&laptop).choices,
            [Choice::Later { beside: 1 }, Choice::First]
        );
        // With an integrated GPU whose stack only shows, the same.
        let laptop = [gpu(SHOWS, true), gpu(DISCRETE, false)];
        assert_eq!(
            all(&laptop).choices,
            [Choice::Later { beside: 1 }, Choice::First]
        );
        // With an integrated GPU no stack drives, the firmware's picture on
        // it stays and shows the discrete GPU's frames by copy.
        let laptop = [unsupported(true), gpu(DISCRETE, false)];
        let chosen = all(&laptop);
        assert!(matches!(chosen.choices[0], Choice::Unused(_)));
        assert_eq!(chosen.choices[1], Choice::First);
        assert_eq!(chosen.claimed, [1]);
    }

    #[test]
    fn a_laptop_with_a_mux_follows_the_firmwares_setting() {
        // Set to the discrete GPU: the panel is on it, and the integrated
        // GPU, if the firmware left it on, is not needed.
        let chosen = all(&[gpu(INTEGRATED, false), gpu(DISCRETE, true)]);
        assert_eq!(chosen.choices, [Choice::Left { beside: 1 }, Choice::First]);
        // Set to hybrid: as a muxless laptop.
        let chosen = all(&[gpu(INTEGRATED, true), gpu(DISCRETE, false)]);
        assert_eq!(chosen.choices, [Choice::Later { beside: 1 }, Choice::First]);
    }

    #[test]
    fn two_cards_both_start_first_in_the_firmwares_order() {
        let chosen = all(&[gpu(DISCRETE, false), gpu(DISCRETE, true)]);
        assert_eq!(chosen.choices, [Choice::First, Choice::First]);
        assert_eq!(chosen.claimed, [0, 1]);
    }

    #[test]
    fn a_gpu_no_stack_drives_stays_beside_one_that_starts() {
        // ASPEED's BMC display showing the firmware's picture beside a card.
        let chosen = all(&[unsupported(true), gpu(DISCRETE, false)]);
        assert_eq!(
            chosen.choices[0],
            Choice::Unused("no Stela GPU backend for this PCI vendor")
        );
        assert_eq!(chosen.choices[1], Choice::First);
        // A display-only stack alone still starts, to show the picture.
        assert_eq!(all(&[gpu(SHOWS, true)]).choices, [Choice::First]);
        assert_eq!(all(&[gpu(SHOWS, false)]).choices, [Choice::First]);
    }

    #[test]
    fn a_refused_preferred_gpu_lets_the_next_one_start() {
        // The card's claim is refused: the integrated GPU, which would only
        // have shown its frames, starts instead and draws.
        let found = [gpu(INTEGRATED, false), gpu(DISCRETE, true)];
        let mut asked = Vec::new();
        let chosen = select(&found, |index| {
            asked.push(index);
            index != 1
        });
        assert_eq!(asked, [1, 0]);
        assert_eq!(chosen.choices, [Choice::First, Choice::Refused]);
        assert_eq!(chosen.claimed, [0]);
        // Of two cards, the second starts alone.
        let chosen = select(&[gpu(DISCRETE, true), gpu(DISCRETE, false)], |index| {
            index == 1
        });
        assert_eq!(chosen.choices, [Choice::Refused, Choice::First]);
    }

    #[test]
    fn the_claim_order_puts_drawing_discrete_gpus_first() {
        let found = [
            gpu(SHOWS, true),
            gpu(INTEGRATED, true),
            gpu(PROBED, false),
            gpu(DISCRETE, false),
        ];
        let mut asked = Vec::new();
        let chosen = select(&found, |index| {
            asked.push(index);
            true
        });
        assert_eq!(asked, [3, 2, 1, 0]);
        assert_eq!(chosen.claimed, [3, 2, 1, 0]);
        assert_eq!(
            chosen.choices,
            [
                Choice::Later { beside: 3 },
                Choice::Later { beside: 3 },
                Choice::Later { beside: 3 },
                Choice::First
            ]
        );
    }

    #[test]
    fn choices_are_said_in_plain_words() {
        let name = |index: usize| format!("GPU {index}");
        assert_eq!(
            Choice::Left { beside: 1 }.describe(&name),
            "left with the firmware: it would only show the frames GPU 1 draws, and the \
             firmware's picture is not on it"
        );
        assert_eq!(
            Choice::Later { beside: 0 }.describe(&name),
            "started after GPU 0: it may only show the frames that GPU draws"
        );
        assert_eq!(
            PROBED.describe(),
            "integrated or discrete, as its probe tells, draws"
        );
        assert_eq!(
            vec![DISCRETE.describe(), SHOWS.describe()],
            [
                "discrete, draws",
                "integrated, shows only: this image has no Vulkan driver for it"
            ]
        );
    }
}

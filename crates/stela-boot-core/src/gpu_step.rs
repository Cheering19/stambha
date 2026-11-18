//! The boot screen's GPU step and the log's `GPU SELECT` lines, from what
//! the selection chose ([`stela_drivers::gpu::selection`]), which claims
//! were refused and which GPUs their drivers bound. The engine prints what
//! these answer; they read nothing of the machine, so a test gives every
//! kind of PC its whole text (`gpu_step/tests.rs`).
//!
//! The screen names the GPU that draws macOS by its market name, says which
//! other GPU shows the picture where one does, and says why a GPU that would
//! have drawn did not. A display adapter that needs nothing of the user, as
//! one left with the firmware beside the GPU that draws, is named in the log
//! alone.
use crate::gpu_status::{self, Reason};
use alloc::{format, string::String, vec::Vec};
use stela_drivers::gpu::{
    Adapter, Vendor,
    selection::{Choice, Found, Selection},
    stack::{Candidate, metal_devices},
};

/// The step's title.
pub const TITLE: &str = "Starting the GPU";

/// How a line of the step shows on the screen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mark {
    /// The step ended well.
    Ok,
    /// The step ended without a GPU, or a line of its own warns.
    Warn,
    /// A line of its own that tells.
    Info,
}

/// A display controller with a GPU stack whose claim was refused.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Refused {
    pub vendor: Vendor,
    /// Its market name, or its maker alone.
    pub name: String,
    /// Its stack draws, so it would have drawn macOS.
    pub draws: bool,
    pub address: String,
    pub device: u16,
    pub reason: Reason,
    /// The refusal's own words, for the screen as for the log.
    pub why: String,
}

impl Refused {
    /// `adapter`, whose stack draws or not, refused for `reason` in the
    /// refusal's words `why`.
    pub fn new(adapter: &Adapter, draws: bool, reason: Reason, why: &str) -> Self {
        Self {
            vendor: adapter.vendor,
            name: adapter.name(),
            draws,
            address: format!("{}", adapter.address),
            device: adapter.device,
            reason,
            why: String::from(why),
        }
    }

    /// Its screen line ([`gpu_status::refused`]).
    pub fn line(&self) -> String {
        gpu_status::refused(
            self.vendor,
            &self.name,
            &self.address,
            self.device,
            self.reason,
            &self.why,
        )
    }
}

/// The step's lines when no GPU was claimed: the first ends the step. A
/// refused GPU says why, the first as the step's end; without one, the step
/// names the GPUs this PC has and what Stela needs ([`gpu_status::none`]),
/// `ports` being the makers whose GPUs this build drives. The display
/// adapters no stack drives are not named again: the log's `GPU SELECT`
/// lines say why each was not used.
pub fn not_started(
    found: &[Adapter],
    refused: &[Refused],
    ports: &[Vendor],
) -> Vec<(Mark, String)> {
    let Some((first, rest)) = refused.split_first() else {
        return alloc::vec![(Mark::Warn, gpu_status::none(found, ports))];
    };
    let mut lines = alloc::vec![(Mark::Warn, first.line())];
    lines.extend(rest.iter().map(|gpu| (Mark::Info, gpu.line())));
    lines
}

/// A GPU its driver bound, as the step's end sees it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bound<'a> {
    /// Its market name, or its maker alone.
    pub market: &'a str,
    /// Its stack draws: the image holds its Vulkan driver.
    pub draws: bool,
    /// Its driver says it is built into the processor.
    pub integrated: bool,
    /// The firmware's picture may be on it: a screen hangs on its outputs.
    pub shows_screen: bool,
}

/// The step's lines once GPUs bound, `bound` in the claims' order: the
/// first ends the step with the name of the GPU that is to draw macOS, as
/// [`metal_devices`] chooses it before any display is taken, and the
/// drawing GPUs that did not start in whose place it draws (`refused`, and
/// `unbound`, the names of drawing GPUs no driver bound). A refused GPU then
/// says why. A GPU that only shows the picture says so where a screen may
/// hang on it; elsewhere it needs nothing of the user, and the log has it.
pub fn started(bound: &[Bound], unbound: &[&str], refused: &[Refused]) -> Vec<(Mark, String)> {
    let candidates: Vec<Candidate> = bound
        .iter()
        .map(|gpu| Candidate {
            draws: gpu.draws,
            integrated: gpu.integrated,
            drives_display: false,
        })
        .collect();
    let chosen = metal_devices(&candidates);
    let drawer = chosen.serve.first().map(|&index| bound[index].market);
    let failed: Vec<String> = refused
        .iter()
        .filter(|gpu| gpu.draws)
        .map(|gpu| gpu.name.clone())
        .chain(unbound.iter().map(|&name| String::from(name)))
        .collect();
    let mut lines = alloc::vec![(
        Mark::Ok,
        gpu_status::started(drawer, &failed, bound.first().map(|gpu| gpu.market)),
    )];
    lines.extend(refused.iter().map(|gpu| (Mark::Warn, gpu.line())));
    // Without a GPU that draws, the first one is the step's own.
    lines.extend(
        bound
            .iter()
            .enumerate()
            .filter(|&(index, gpu)| {
                !chosen.serve.contains(&index)
                    && gpu.shows_screen
                    && (drawer.is_some() || index != 0)
            })
            .map(|(_, gpu)| (Mark::Info, gpu_status::shows_only(gpu.market, drawer))),
    );
    lines
}

/// The log's `GPU SELECT` lines, without their prefix: one for each display
/// controller of `adapters` with its address, IDs, name, kind and what was
/// chosen for it and why, `refused` saying why a claim was refused; then
/// the order the started ones were claimed in, which the display and the
/// Metal device follow, or that none started.
pub fn selection_lines(
    adapters: &[Adapter],
    found: &[Found],
    chosen: &Selection,
    refused: &[Refused],
) -> Vec<String> {
    let named = |index: usize| {
        let adapter = &adapters[index];
        format!("{} at {}", adapter.name(), adapter.address)
    };
    let mut lines: Vec<String> = adapters
        .iter()
        .zip(found)
        .enumerate()
        .map(|(index, (adapter, found))| {
            let kind = match found.stack {
                Ok(driven) => format!("{}; ", driven.describe()),
                Err(_) => String::new(),
            };
            let address = format!("{}", adapter.address);
            let choice = match (
                chosen.choices[index],
                refused.iter().find(|gpu| gpu.address == address),
            ) {
                (Choice::Refused, Some(gpu)) if !gpu.why.is_empty() => {
                    format!("not started: its claim was refused: {}", gpu.why)
                }
                (choice, _) => choice.describe(&named),
            };
            format!(
                "{address} {:04x}:{:04x} rev {:02x} {}: {kind}{choice}",
                adapter.vendor.pci_id(),
                adapter.device,
                adapter.revision,
                adapter.name(),
            )
        })
        .collect();
    lines.push(match chosen.claimed.is_empty() {
        true => String::from("started none"),
        false => {
            let order: Vec<String> = chosen.claimed.iter().map(|&index| named(index)).collect();
            format!(
                "started in this order: {}. A discrete GPU that draws gives the Metal device \
                 before an integrated one, and the next one does where it fails; the display is \
                 the first of them with a connected display, else the firmware's screen",
                order.join(", ")
            )
        }
    });
    lines
}

#[cfg(test)]
mod tests;

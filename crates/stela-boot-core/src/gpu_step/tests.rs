//! The whole GPU step of the boot screen, and the log's `GPU SELECT` lines,
//! of each kind of PC Stela meets, through the functions the boot uses: the
//! selection ([`select`]), the claims' refusals, the step's lines
//! ([`not_started`], [`started`]) and the screen's step line
//! ([`finished_step`]). Each PC is booted by two builds: a release, whose
//! image drives NVIDIA GPUs alone, and a build with every port and every
//! Vulkan driver in its image.
//!
//! What a PC is made of is what its drivers' tables say of each GPU before
//! the probe (`table`), and what the probe, the claim and the screen then
//! find. When the AMD and Intel ports are in this test binary (a workspace
//! run with every feature), each table answer is checked against them.
use super::*;
use crate::gpu_status::{Picture, shown_by_copy, unbound};
use crate::screen::finished_step;
use alloc::{string::ToString, vec};
use stela_drivers::bus::pci::Address;
use stela_drivers::gpu::selection::{Driven, select};
use stela_drivers::gpu::stack::GpuStack;

/// Which image boots.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Build {
    /// A release: `nvk` alone, so the AMD and Intel ports are not in it.
    Release,
    /// `nvk`, `amdgpu` and `intel`, with RADV and ANV in the image.
    Full,
}

/// One display controller of a PC.
#[derive(Clone, Copy)]
struct Gpu {
    bus: u8,
    slot: u8,
    vendor: u16,
    device: u16,
    revision: u8,
    /// What its driver's table says of it before the probe in the full
    /// build: integrated, discrete or left to the probe; `Err` with the
    /// port's words where the port does not drive it.
    table: Result<Option<bool>, &'static str>,
    /// What its driver says once bound: built into the processor.
    integrated: bool,
    /// A screen hangs on it, so the firmware's picture may be on it.
    screen: bool,
    /// Its claim is refused, for this reason in these words.
    refused: Option<(Reason, &'static str)>,
    /// Its driver's probe fails.
    unbound: bool,
}

const fn gpu(bus: u8, slot: u8, vendor: u16, device: u16, revision: u8) -> Gpu {
    Gpu {
        bus,
        slot,
        vendor,
        device,
        revision,
        table: Ok(Some(false)),
        integrated: false,
        screen: false,
        refused: None,
        unbound: false,
    }
}

impl Gpu {
    const fn screen(self) -> Self {
        Self {
            screen: true,
            ..self
        }
    }

    /// Integrated, by its table (`Some(true)`) or only by its probe (`None`).
    const fn integrated(self, table: Option<bool>) -> Self {
        Self {
            table: Ok(table),
            integrated: true,
            ..self
        }
    }

    /// Discrete, told only by its probe.
    const fn probed(self) -> Self {
        Self {
            table: Ok(None),
            ..self
        }
    }

    const fn refused(self, reason: Reason, why: &'static str) -> Self {
        Self {
            refused: Some((reason, why)),
            ..self
        }
    }

    const fn unbound(self) -> Self {
        Self {
            unbound: true,
            ..self
        }
    }

    const fn not_driven(self, why: &'static str) -> Self {
        Self {
            table: Err(why),
            ..self
        }
    }

    fn adapter(&self) -> Adapter {
        Adapter {
            address: Address {
                segment: 0,
                bus: self.bus,
                device: self.slot,
                function: 0,
            },
            vendor: Vendor::from_pci(self.vendor),
            device: self.device,
            class: [3, 0, 0],
            revision: self.revision,
        }
    }

    /// What its stack says of it in `build`, as [`selection::driven`] tells
    /// the boot: every stack of the full build draws.
    fn found(&self, build: Build) -> Found {
        let adapter = self.adapter();
        let stack = match (build, adapter.vendor) {
            (_, Vendor::Nvidia) => Ok(Driven {
                draws: true,
                integrated: Some(false),
            }),
            (Build::Full, Vendor::Amd | Vendor::Intel) => self.table.map(|integrated| Driven {
                draws: true,
                integrated,
            }),
            _ => Err(adapter.unavailable_reason()),
        };
        // The ports' own tables, where this binary holds them.
        if build == Build::Full
            && let Ok(port) = GpuStack::select(&adapter)
            && port.vendor != Vendor::Nvidia
        {
            assert_eq!(
                Ok(port.kernel.table_integrated(&adapter)),
                self.table,
                "{}",
                adapter.name()
            );
        }
        Found {
            stack,
            firmware_screen: self.screen,
        }
    }

    fn driver(&self) -> &'static str {
        match Vendor::from_pci(self.vendor) {
            Vendor::Nvidia => "nouveau",
            Vendor::Amd => "amdgpu",
            _ => "i915",
        }
    }
}

const NVIDIA: u16 = 0x10de;
const AMD: u16 = 0x1002;
const INTEL: u16 = 0x8086;
const ASPEED: u16 = 0x1a03;

/// A desktop's GeForce RTX 5080 and a laptop's GeForce RTX 4060.
const RTX_5080: Gpu = gpu(1, 0, NVIDIA, 0x2c02, 0xa1);
const RTX_4060_LAPTOP: Gpu = gpu(1, 0, NVIDIA, 0x28e0, 0xa1);
/// The card of a server, beside its BMC display.
const RTX_5080_SERVER: Gpu = gpu(0x41, 0, NVIDIA, 0x2c02, 0xa1);
/// A Radeon RX 7900 XTX and a laptop's Radeon RX 7600M XT, which amdgpu
/// takes by class: their probe tells their kind.
const RX_7900_XTX: Gpu = gpu(3, 0, AMD, 0x744c, 0xc8).probed();
const RX_7600M_XT: Gpu = gpu(3, 0, AMD, 0x7480, 0xc7).probed();
/// A Radeon RX 580, whose generation the port does not drive.
const RX_580: Gpu =
    gpu(3, 0, AMD, 0x67df, 0xe7).not_driven(stela_driver_core::amd_metal::NO_APPLE_FAMILY);
/// An Arc A770, which i915's table marks discrete.
const ARC_A770: Gpu = gpu(3, 0, INTEL, 0x56a0, 0x08);
/// A Ryzen 7040's Radeon 780M, which amdgpu takes by class.
const RADEON_780M: Gpu = gpu(0x65, 0, AMD, 0x15bf, 0xc1).integrated(None);
/// Tiger Lake's and Raptor Lake-P's Iris Xe Graphics and Raptor Lake-S's
/// UHD Graphics 770, which i915's table marks integrated.
const IRIS_XE: Gpu = gpu(0, 2, INTEL, 0x9a49, 0x01).integrated(Some(true));
const IRIS_XE_RPL: Gpu = gpu(0, 2, INTEL, 0xa7a0, 0x04).integrated(Some(true));
const UHD_770: Gpu = gpu(0, 2, INTEL, 0xa780, 0x04).integrated(Some(true));
/// A server's BMC display, which no stack drives.
const BMC: Gpu = gpu(2, 0, ASPEED, 0x2000, 0x52).not_driven("");

/// The NVIDIA claim's refusal when the USB holds no firmware for its chip.
const NO_FIRMWARE: &str = "no pinned firmware for the observed GPU chipset was loaded";

/// The boot screen's lines from the GPU step on, as `status` draws them,
/// and the log's `GPU SELECT` lines, of `pc` booted by `build`.
fn boot(pc: &[Gpu], build: Build) -> (Vec<String>, Vec<String>) {
    let adapters: Vec<Adapter> = pc.iter().map(Gpu::adapter).collect();
    let found: Vec<Found> = pc.iter().map(|gpu| gpu.found(build)).collect();
    let mut refused = Vec::new();
    let chosen = select(&found, |index| match pc[index].refused {
        Some((reason, why)) => {
            let draws = found[index].stack.is_ok_and(|driven| driven.draws);
            refused.push(Refused::new(&adapters[index], draws, reason, why));
            false
        }
        None => true,
    });
    let log = selection_lines(&adapters, &found, &chosen, &refused)
        .into_iter()
        .map(|line| format!("GPU SELECT {line}"))
        .collect();
    let ports: &[Vendor] = match build {
        Build::Release => &[Vendor::Nvidia],
        Build::Full => &[Vendor::Nvidia, Vendor::Amd, Vendor::Intel],
    };
    let mut screen: Vec<(Mark, String)> = Vec::new();
    let mut bound = Vec::new();
    let mut failed_names = Vec::new();
    let names: Vec<String> = adapters.iter().map(Adapter::name).collect();
    let step = if chosen.claimed.is_empty() {
        not_started(&adapters, &refused, ports)
    } else {
        // Each claimed GPU's probe, in the claims' order; a driver that does
        // not bind its GPU says so as it happens.
        for &index in &chosen.claimed {
            let gpu = &pc[index];
            if gpu.unbound {
                let picture = match gpu.screen {
                    true => Picture::ResetForClaim,
                    false => Picture::Elsewhere,
                };
                let address = adapters[index].address.to_string();
                screen.push((
                    Mark::Warn,
                    unbound(&names[index], &address, gpu.driver(), picture),
                ));
                failed_names.push(names[index].as_str());
            } else {
                bound.push(Bound {
                    market: &names[index],
                    draws: true,
                    integrated: gpu.integrated,
                    shows_screen: gpu.screen,
                });
            }
        }
        match bound.is_empty() {
            // The engine's `Error::NoDevices`: each GPU said why above.
            true => vec![(Mark::Warn, String::from(crate::gpu_status::NONE_BOUND))],
            false => started(&bound, &failed_names, &refused),
        }
    };
    let mut lines = step.into_iter();
    let (mark, result) = lines.next().unwrap();
    screen.push((mark, finished_step(TITLE, None, &result, 0)));
    screen.extend(lines);
    // The display step: with no started GPU's screen, the GPU that draws
    // is copied to the screen the PC started on.
    let drawer = {
        let candidates: Vec<Candidate> = bound
            .iter()
            .map(|gpu| Candidate {
                draws: gpu.draws,
                integrated: gpu.integrated,
                drives_display: false,
            })
            .collect();
        metal_devices(&candidates)
            .serve
            .first()
            .map(|&first| bound[first].market)
    };
    if let Some(drawer) = drawer
        && !bound.iter().any(|gpu| gpu.shows_screen)
        && pc.iter().any(|gpu| gpu.screen)
    {
        screen.push((Mark::Info, shown_by_copy(drawer)));
    }
    let screen = screen
        .into_iter()
        .map(|(mark, text)| match mark {
            Mark::Ok => format!("[  ok  ]  {text}"),
            Mark::Warn => format!("[ warn ]  {text}"),
            Mark::Info => format!("          {text}"),
        })
        .collect();
    (screen, log)
}

/// The text of each PC, booted by each build.
fn text(pc: &[Gpu], build: Build) -> String {
    let (screen, log) = boot(pc, build);
    format!("{}\n--\n{}", screen.join("\n"), log.join("\n"))
}

/// The PC's whole text booted by `build`: the screen from the GPU step on,
/// then the log's `GPU SELECT` lines.
fn assert_boots(pc: &[Gpu], build: Build, expected: &str) {
    assert_eq!(text(pc, build), expected, "{build:?}");
}

/// An NVIDIA card alone draws in either build.
#[test]
fn a_desktop_with_an_nvidia_card_alone() {
    let pc: &[Gpu] = &[RTX_5080.screen()];
    let both = r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 5080)
--
GPU SELECT 0000:01:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 5080 at 0000:01:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#;
    assert_boots(pc, Build::Release, both);
    assert_boots(pc, Build::Full, both);
}

/// An AMD card alone: a release names it and asks for an NVIDIA GPU; the full build draws on it.
#[test]
fn a_desktop_with_an_amd_card_alone() {
    let pc: &[Gpu] = &[RX_7900_XTX.screen()];
    assert_boots(
        pc,
        Build::Release,
        r#"[ warn ]  Starting the GPU (this PC has no NVIDIA GPU, only the AMD Radeon RX 7900 XTX: Stela needs an NVIDIA GeForce GTX 16 or RTX 20, 30, 40 or 50 series GPU)
--
GPU SELECT 0000:03:00.0 1002:744c rev c8 AMD Radeon RX 7900 XTX: not used: AMDGPU is not ported to Stela
GPU SELECT started none"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (AMD Radeon RX 7900 XTX)
--
GPU SELECT 0000:03:00.0 1002:744c rev c8 AMD Radeon RX 7900 XTX: integrated or discrete, as its probe tells, draws; started first
GPU SELECT started in this order: AMD Radeon RX 7900 XTX at 0000:03:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// An Intel Arc card alone, as an AMD card.
#[test]
fn a_desktop_with_an_intel_arc_card_alone() {
    let pc: &[Gpu] = &[ARC_A770.screen()];
    assert_boots(
        pc,
        Build::Release,
        r#"[ warn ]  Starting the GPU (this PC has no NVIDIA GPU, only the Intel Arc A770: Stela needs an NVIDIA GeForce GTX 16 or RTX 20, 30, 40 or 50 series GPU)
--
GPU SELECT 0000:03:00.0 8086:56a0 rev 08 Intel Arc A770: not used: no driver here runs Stela's Metal device on this Intel GPU
GPU SELECT started none"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (Intel Arc A770)
--
GPU SELECT 0000:03:00.0 8086:56a0 rev 08 Intel Arc A770: discrete, draws; started first
GPU SELECT started in this order: Intel Arc A770 at 0000:03:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// A Ryzen's Radeon alone, as an AMD card.
#[test]
fn a_pc_with_an_amd_integrated_gpu_alone() {
    let pc: &[Gpu] = &[RADEON_780M.screen()];
    assert_boots(
        pc,
        Build::Release,
        r#"[ warn ]  Starting the GPU (this PC has no NVIDIA GPU, only the AMD Radeon 780M Graphics: Stela needs an NVIDIA GeForce GTX 16 or RTX 20, 30, 40 or 50 series GPU)
--
GPU SELECT 0000:65:00.0 1002:15bf rev c1 AMD Radeon 780M Graphics: not used: AMDGPU is not ported to Stela
GPU SELECT started none"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (AMD Radeon 780M Graphics)
--
GPU SELECT 0000:65:00.0 1002:15bf rev c1 AMD Radeon 780M Graphics: integrated or discrete, as its probe tells, draws; started first
GPU SELECT started in this order: AMD Radeon 780M Graphics at 0000:65:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// An Intel processor's GPU alone, as an AMD card.
#[test]
fn a_pc_with_an_intel_integrated_gpu_alone() {
    let pc: &[Gpu] = &[IRIS_XE.screen()];
    assert_boots(
        pc,
        Build::Release,
        r#"[ warn ]  Starting the GPU (this PC has no NVIDIA GPU, only the Intel Iris Xe Graphics: Stela needs an NVIDIA GeForce GTX 16 or RTX 20, 30, 40 or 50 series GPU)
--
GPU SELECT 0000:00:02.0 8086:9a49 rev 01 Intel Iris Xe Graphics: not used: no driver here runs Stela's Metal device on this Intel GPU
GPU SELECT started none"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (Intel Iris Xe Graphics)
--
GPU SELECT 0000:00:02.0 8086:9a49 rev 01 Intel Iris Xe Graphics: integrated, draws; started first
GPU SELECT started in this order: Intel Iris Xe Graphics at 0000:00:02.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// A laptop without a display mux: the NVIDIA GPU draws; the full build starts the Intel GPU to show the panel, a release copies each frame to the panel the firmware lit.
#[test]
fn a_laptop_with_nvidia_and_intel_gpus_and_the_panel_on_the_intel_one() {
    let pc: &[Gpu] = &[IRIS_XE_RPL.screen(), RTX_4060_LAPTOP];
    assert_boots(
        pc,
        Build::Release,
        r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 4060 Laptop GPU)
          No screen is connected to the NVIDIA GeForce RTX 4060 Laptop GPU, so Stela copies each frame of macOS to the screen the PC started on. The picture is slower than on a screen connected to the GPU.
--
GPU SELECT 0000:00:02.0 8086:a7a0 rev 04 Intel Iris Xe Graphics: not used: no driver here runs Stela's Metal device on this Intel GPU
GPU SELECT 0000:01:00.0 10de:28e0 rev a1 NVIDIA GeForce RTX 4060 Laptop GPU: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 4060 Laptop GPU at 0000:01:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 4060 Laptop GPU)
          the Intel Iris Xe Graphics shows the picture on the screens wired to it
--
GPU SELECT 0000:00:02.0 8086:a7a0 rev 04 Intel Iris Xe Graphics: integrated, draws; started after NVIDIA GeForce RTX 4060 Laptop GPU at 0000:01:00.0: it may only show the frames that GPU draws
GPU SELECT 0000:01:00.0 10de:28e0 rev a1 NVIDIA GeForce RTX 4060 Laptop GPU: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 4060 Laptop GPU at 0000:01:00.0, Intel Iris Xe Graphics at 0000:00:02.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// Its mux set to the NVIDIA GPU: the Intel GPU is left alone, and named in the log only.
#[test]
fn a_laptop_with_nvidia_and_intel_gpus_and_the_mux_on_the_nvidia_one() {
    let pc: &[Gpu] = &[IRIS_XE_RPL, RTX_4060_LAPTOP.screen()];
    assert_boots(
        pc,
        Build::Release,
        r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 4060 Laptop GPU)
--
GPU SELECT 0000:00:02.0 8086:a7a0 rev 04 Intel Iris Xe Graphics: not used: no driver here runs Stela's Metal device on this Intel GPU
GPU SELECT 0000:01:00.0 10de:28e0 rev a1 NVIDIA GeForce RTX 4060 Laptop GPU: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 4060 Laptop GPU at 0000:01:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 4060 Laptop GPU)
--
GPU SELECT 0000:00:02.0 8086:a7a0 rev 04 Intel Iris Xe Graphics: integrated, draws; left with the firmware: it would only show the frames NVIDIA GeForce RTX 4060 Laptop GPU at 0000:01:00.0 draws, and the firmware's picture is not on it
GPU SELECT 0000:01:00.0 10de:28e0 rev a1 NVIDIA GeForce RTX 4060 Laptop GPU: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 4060 Laptop GPU at 0000:01:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// An NVIDIA GPU beside a Ryzen's Radeon with the panel on it.
#[test]
fn a_laptop_with_nvidia_and_amd_gpus_and_the_panel_on_the_amd_one() {
    let pc: &[Gpu] = &[RTX_4060_LAPTOP, RADEON_780M.screen()];
    assert_boots(
        pc,
        Build::Release,
        r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 4060 Laptop GPU)
          No screen is connected to the NVIDIA GeForce RTX 4060 Laptop GPU, so Stela copies each frame of macOS to the screen the PC started on. The picture is slower than on a screen connected to the GPU.
--
GPU SELECT 0000:01:00.0 10de:28e0 rev a1 NVIDIA GeForce RTX 4060 Laptop GPU: discrete, draws; started first
GPU SELECT 0000:65:00.0 1002:15bf rev c1 AMD Radeon 780M Graphics: not used: AMDGPU is not ported to Stela
GPU SELECT started in this order: NVIDIA GeForce RTX 4060 Laptop GPU at 0000:01:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 4060 Laptop GPU)
          the AMD Radeon 780M Graphics shows the picture on the screens wired to it
--
GPU SELECT 0000:01:00.0 10de:28e0 rev a1 NVIDIA GeForce RTX 4060 Laptop GPU: discrete, draws; started first
GPU SELECT 0000:65:00.0 1002:15bf rev c1 AMD Radeon 780M Graphics: integrated or discrete, as its probe tells, draws; started after NVIDIA GeForce RTX 4060 Laptop GPU at 0000:01:00.0: it may only show the frames that GPU draws
GPU SELECT started in this order: NVIDIA GeForce RTX 4060 Laptop GPU at 0000:01:00.0, AMD Radeon 780M Graphics at 0000:65:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// The mux on the NVIDIA GPU: the Radeon, whose kind only its probe tells, starts after it and has no line on the screen, as no screen hangs on it.
#[test]
fn a_laptop_with_nvidia_and_amd_gpus_and_the_mux_on_the_nvidia_one() {
    let pc: &[Gpu] = &[RTX_4060_LAPTOP.screen(), RADEON_780M];
    assert_boots(
        pc,
        Build::Release,
        r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 4060 Laptop GPU)
--
GPU SELECT 0000:01:00.0 10de:28e0 rev a1 NVIDIA GeForce RTX 4060 Laptop GPU: discrete, draws; started first
GPU SELECT 0000:65:00.0 1002:15bf rev c1 AMD Radeon 780M Graphics: not used: AMDGPU is not ported to Stela
GPU SELECT started in this order: NVIDIA GeForce RTX 4060 Laptop GPU at 0000:01:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 4060 Laptop GPU)
--
GPU SELECT 0000:01:00.0 10de:28e0 rev a1 NVIDIA GeForce RTX 4060 Laptop GPU: discrete, draws; started first
GPU SELECT 0000:65:00.0 1002:15bf rev c1 AMD Radeon 780M Graphics: integrated or discrete, as its probe tells, draws; started after NVIDIA GeForce RTX 4060 Laptop GPU at 0000:01:00.0: it may only show the frames that GPU draws
GPU SELECT started in this order: NVIDIA GeForce RTX 4060 Laptop GPU at 0000:01:00.0, AMD Radeon 780M Graphics at 0000:65:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// Two AMD GPUs whose kinds their probes tell: both start, the discrete one draws.
#[test]
fn a_laptop_with_two_amd_gpus_and_the_panel_on_the_integrated_one() {
    let pc: &[Gpu] = &[RX_7600M_XT, RADEON_780M.screen()];
    assert_boots(
        pc,
        Build::Release,
        r#"[ warn ]  Starting the GPU (this PC has no NVIDIA GPU, only the AMD Radeon RX 7600M XT and the AMD Radeon 780M Graphics: Stela needs an NVIDIA GeForce GTX 16 or RTX 20, 30, 40 or 50 series GPU)
--
GPU SELECT 0000:03:00.0 1002:7480 rev c7 AMD Radeon RX 7600M XT: not used: AMDGPU is not ported to Stela
GPU SELECT 0000:65:00.0 1002:15bf rev c1 AMD Radeon 780M Graphics: not used: AMDGPU is not ported to Stela
GPU SELECT started none"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (AMD Radeon RX 7600M XT)
          the AMD Radeon 780M Graphics shows the picture on the screens wired to it
--
GPU SELECT 0000:03:00.0 1002:7480 rev c7 AMD Radeon RX 7600M XT: integrated or discrete, as its probe tells, draws; started first
GPU SELECT 0000:65:00.0 1002:15bf rev c1 AMD Radeon 780M Graphics: integrated or discrete, as its probe tells, draws; started first
GPU SELECT started in this order: AMD Radeon RX 7600M XT at 0000:03:00.0, AMD Radeon 780M Graphics at 0000:65:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// The same with the mux on the discrete GPU.
#[test]
fn a_laptop_with_two_amd_gpus_and_the_mux_on_the_discrete_one() {
    let pc: &[Gpu] = &[RX_7600M_XT.screen(), RADEON_780M];
    assert_boots(
        pc,
        Build::Release,
        r#"[ warn ]  Starting the GPU (this PC has no NVIDIA GPU, only the AMD Radeon RX 7600M XT and the AMD Radeon 780M Graphics: Stela needs an NVIDIA GeForce GTX 16 or RTX 20, 30, 40 or 50 series GPU)
--
GPU SELECT 0000:03:00.0 1002:7480 rev c7 AMD Radeon RX 7600M XT: not used: AMDGPU is not ported to Stela
GPU SELECT 0000:65:00.0 1002:15bf rev c1 AMD Radeon 780M Graphics: not used: AMDGPU is not ported to Stela
GPU SELECT started none"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (AMD Radeon RX 7600M XT)
--
GPU SELECT 0000:03:00.0 1002:7480 rev c7 AMD Radeon RX 7600M XT: integrated or discrete, as its probe tells, draws; started first
GPU SELECT 0000:65:00.0 1002:15bf rev c1 AMD Radeon 780M Graphics: integrated or discrete, as its probe tells, draws; started first
GPU SELECT started in this order: AMD Radeon RX 7600M XT at 0000:03:00.0, AMD Radeon 780M Graphics at 0000:65:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// A desktop's monitor on its card: the integrated GPU is left alone.
#[test]
fn a_desktop_with_a_card_and_an_integrated_gpu_and_the_monitor_on_the_card() {
    let pc: &[Gpu] = &[UHD_770, RTX_5080.screen()];
    assert_boots(
        pc,
        Build::Release,
        r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 5080)
--
GPU SELECT 0000:00:02.0 8086:a780 rev 04 Intel UHD Graphics 770: not used: no driver here runs Stela's Metal device on this Intel GPU
GPU SELECT 0000:01:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 5080 at 0000:01:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 5080)
--
GPU SELECT 0000:00:02.0 8086:a780 rev 04 Intel UHD Graphics 770: integrated, draws; left with the firmware: it would only show the frames NVIDIA GeForce RTX 5080 at 0000:01:00.0 draws, and the firmware's picture is not on it
GPU SELECT 0000:01:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 5080 at 0000:01:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// Its monitor on the motherboard: the card draws, the integrated GPU shows the picture, or in a release each frame is copied there.
#[test]
fn a_desktop_with_a_card_and_an_integrated_gpu_and_the_monitor_on_the_motherboard() {
    let pc: &[Gpu] = &[UHD_770.screen(), RTX_5080];
    assert_boots(
        pc,
        Build::Release,
        r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 5080)
          No screen is connected to the NVIDIA GeForce RTX 5080, so Stela copies each frame of macOS to the screen the PC started on. The picture is slower than on a screen connected to the GPU.
--
GPU SELECT 0000:00:02.0 8086:a780 rev 04 Intel UHD Graphics 770: not used: no driver here runs Stela's Metal device on this Intel GPU
GPU SELECT 0000:01:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 5080 at 0000:01:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 5080)
          the Intel UHD Graphics 770 shows the picture on the screens wired to it
--
GPU SELECT 0000:00:02.0 8086:a780 rev 04 Intel UHD Graphics 770: integrated, draws; started after NVIDIA GeForce RTX 5080 at 0000:01:00.0: it may only show the frames that GPU draws
GPU SELECT 0000:01:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 5080 at 0000:01:00.0, Intel UHD Graphics 770 at 0000:00:02.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// A card whose claim is refused: the integrated GPU takes its place and the screen says so.
#[test]
fn a_card_whose_claim_is_refused_leaves_the_integrated_gpu_to_draw() {
    let pc: &[Gpu] = &[
        UHD_770,
        RTX_5080
            .screen()
            .refused(Reason::Other, "the GPU's register BAR does not decode"),
    ];
    assert_boots(
        pc,
        Build::Release,
        r#"[ warn ]  Starting the GPU (the NVIDIA GeForce RTX 5080 at 0000:01:00.0 did not start: the GPU's register BAR does not decode. The boot log on this USB has the rest)
--
GPU SELECT 0000:00:02.0 8086:a780 rev 04 Intel UHD Graphics 770: not used: no driver here runs Stela's Metal device on this Intel GPU
GPU SELECT 0000:01:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; not started: its claim was refused: the GPU's register BAR does not decode
GPU SELECT started none"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (Intel UHD Graphics 770, as the NVIDIA GeForce RTX 5080 did not start)
[ warn ]  the NVIDIA GeForce RTX 5080 at 0000:01:00.0 did not start: the GPU's register BAR does not decode. The boot log on this USB has the rest
          No screen is connected to the Intel UHD Graphics 770, so Stela copies each frame of macOS to the screen the PC started on. The picture is slower than on a screen connected to the GPU.
--
GPU SELECT 0000:00:02.0 8086:a780 rev 04 Intel UHD Graphics 770: integrated, draws; started first
GPU SELECT 0000:01:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; not started: its claim was refused: the GPU's register BAR does not decode
GPU SELECT started in this order: Intel UHD Graphics 770 at 0000:00:02.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// A card whose driver fails on it beside the integrated GPU that shows the monitor: that one draws.
#[test]
fn a_card_whose_driver_fails_leaves_the_integrated_gpu_that_shows_the_monitor_to_draw() {
    let pc: &[Gpu] = &[UHD_770.screen(), RTX_5080.unbound()];
    assert_boots(
        pc,
        Build::Release,
        r#"[ warn ]  the NVIDIA GeForce RTX 5080 at 0000:01:00.0 did not start: its driver nouveau stopped with an error. The boot log on this USB says why
[ warn ]  Starting the GPU (no GPU started; the lines above say why)
--
GPU SELECT 0000:00:02.0 8086:a780 rev 04 Intel UHD Graphics 770: not used: no driver here runs Stela's Metal device on this Intel GPU
GPU SELECT 0000:01:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 5080 at 0000:01:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[ warn ]  the NVIDIA GeForce RTX 5080 at 0000:01:00.0 did not start: its driver nouveau stopped with an error. The boot log on this USB says why
[  ok  ]  Starting the GPU (Intel UHD Graphics 770, as the NVIDIA GeForce RTX 5080 did not start)
--
GPU SELECT 0000:00:02.0 8086:a780 rev 04 Intel UHD Graphics 770: integrated, draws; started after NVIDIA GeForce RTX 5080 at 0000:01:00.0: it may only show the frames that GPU draws
GPU SELECT 0000:01:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 5080 at 0000:01:00.0, Intel UHD Graphics 770 at 0000:00:02.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// A card whose driver fails with the monitor on it: the integrated GPU was left with the firmware before the probe, so no GPU starts.
#[test]
fn a_card_whose_driver_fails_with_the_monitor_on_it_leaves_no_gpu() {
    let pc: &[Gpu] = &[UHD_770, RTX_5080.screen().unbound()];
    assert_boots(
        pc,
        Build::Release,
        r#"[ warn ]  the NVIDIA GeForce RTX 5080 at 0000:01:00.0 did not start: its driver nouveau stopped with an error. The screen the PC started on is wired to it and stays dark: the GPU was reset for its driver. The boot log on this USB says why
[ warn ]  Starting the GPU (no GPU started; the lines above say why)
--
GPU SELECT 0000:00:02.0 8086:a780 rev 04 Intel UHD Graphics 770: not used: no driver here runs Stela's Metal device on this Intel GPU
GPU SELECT 0000:01:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 5080 at 0000:01:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[ warn ]  the NVIDIA GeForce RTX 5080 at 0000:01:00.0 did not start: its driver nouveau stopped with an error. The screen the PC started on is wired to it and stays dark: the GPU was reset for its driver. The boot log on this USB says why
[ warn ]  Starting the GPU (no GPU started; the lines above say why)
--
GPU SELECT 0000:00:02.0 8086:a780 rev 04 Intel UHD Graphics 770: integrated, draws; left with the firmware: it would only show the frames NVIDIA GeForce RTX 5080 at 0000:01:00.0 draws, and the firmware's picture is not on it
GPU SELECT 0000:01:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 5080 at 0000:01:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// A server whose firmware shows its picture on the BMC's display: the card draws and each frame is copied there; the BMC is named in the log only.
#[test]
fn a_server_with_a_bmc_display_beside_a_card() {
    let pc: &[Gpu] = &[BMC.screen(), RTX_5080_SERVER];
    let both = r#"[  ok  ]  Starting the GPU (NVIDIA GeForce RTX 5080)
          No screen is connected to the NVIDIA GeForce RTX 5080, so Stela copies each frame of macOS to the screen the PC started on. The picture is slower than on a screen connected to the GPU.
--
GPU SELECT 0000:02:00.0 1a03:2000 rev 52 display adapter: not used: no Stela GPU backend for this PCI vendor
GPU SELECT 0000:41:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; started first
GPU SELECT started in this order: NVIDIA GeForce RTX 5080 at 0000:41:00.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#;
    assert_boots(pc, Build::Release, both);
    assert_boots(pc, Build::Full, both);
}

/// A server with only a BMC display.
#[test]
fn a_server_with_only_a_bmc_display() {
    let pc: &[Gpu] = &[BMC.screen()];
    assert_boots(
        pc,
        Build::Release,
        r#"[ warn ]  Starting the GPU (this PC has no NVIDIA GPU: Stela needs an NVIDIA GeForce GTX 16 or RTX 20, 30, 40 or 50 series GPU)
--
GPU SELECT 0000:02:00.0 1a03:2000 rev 52 display adapter: not used: no Stela GPU backend for this PCI vendor
GPU SELECT started none"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[ warn ]  Starting the GPU (this PC has no GPU Stela drives: Stela needs an NVIDIA GeForce GTX 16 or RTX 20, 30, 40 or 50 series GPU, an AMD Radeon RX Vega or later GPU, or an Intel GPU from 6th generation Core processors on)
--
GPU SELECT 0000:02:00.0 1a03:2000 rev 52 display adapter: not used: no Stela GPU backend for this PCI vendor
GPU SELECT started none"#,
    );
}

/// A Radeon RX 580, which no build drives.
#[test]
fn a_desktop_with_only_a_gpu_no_build_drives() {
    let pc: &[Gpu] = &[RX_580.screen()];
    assert_boots(
        pc,
        Build::Release,
        r#"[ warn ]  Starting the GPU (this PC has no NVIDIA GPU, only the AMD Radeon RX 580 Series: Stela needs an NVIDIA GeForce GTX 16 or RTX 20, 30, 40 or 50 series GPU)
--
GPU SELECT 0000:03:00.0 1002:67df rev e7 AMD Radeon RX 580 Series: not used: AMDGPU is not ported to Stela
GPU SELECT started none"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[ warn ]  Starting the GPU (this PC has no GPU Stela drives, only the AMD Radeon RX 580 Series: Stela needs an NVIDIA GeForce GTX 16 or RTX 20, 30, 40 or 50 series GPU, an AMD Radeon RX Vega or later GPU, or an Intel GPU from 6th generation Core processors on)
--
GPU SELECT 0000:03:00.0 1002:67df rev e7 AMD Radeon RX 580 Series: not used: on this GPU's graphics generation (GFX8) Stela's Metal device is in no Apple GPU family: RADV has no raster order groups before GFX9, which the framebuffer reads of Apple3 need, and macOS draws its window server's framebuffers only on an Apple3 device
GPU SELECT started none"#,
    );
}

/// An NVIDIA card whose firmware the USB lacks.
#[test]
fn a_card_refused_for_missing_firmware() {
    let pc: &[Gpu] = &[RTX_5080
        .screen()
        .refused(Reason::MissingFirmware, NO_FIRMWARE)];
    let both = r#"[ warn ]  Starting the GPU (the USB has no firmware for this PC's NVIDIA GeForce RTX 5080. On another computer, plug in this USB and run `stela repair`)
--
GPU SELECT 0000:01:00.0 10de:2c02 rev a1 NVIDIA GeForce RTX 5080: discrete, draws; not started: its claim was refused: no pinned firmware for the observed GPU chipset was loaded
GPU SELECT started none"#;
    assert_boots(pc, Build::Release, both);
    assert_boots(pc, Build::Full, both);
}

/// The same in a laptop: the full build draws on the Intel GPU and says why.
#[test]
fn a_laptop_whose_nvidia_gpu_lacks_firmware_draws_on_the_integrated_one() {
    let pc: &[Gpu] = &[
        IRIS_XE_RPL.screen(),
        RTX_4060_LAPTOP.refused(Reason::MissingFirmware, NO_FIRMWARE),
    ];
    assert_boots(
        pc,
        Build::Release,
        r#"[ warn ]  Starting the GPU (the USB has no firmware for this PC's NVIDIA GeForce RTX 4060 Laptop GPU. On another computer, plug in this USB and run `stela repair`)
--
GPU SELECT 0000:00:02.0 8086:a7a0 rev 04 Intel Iris Xe Graphics: not used: no driver here runs Stela's Metal device on this Intel GPU
GPU SELECT 0000:01:00.0 10de:28e0 rev a1 NVIDIA GeForce RTX 4060 Laptop GPU: discrete, draws; not started: its claim was refused: no pinned firmware for the observed GPU chipset was loaded
GPU SELECT started none"#,
    );
    assert_boots(
        pc,
        Build::Full,
        r#"[  ok  ]  Starting the GPU (Intel Iris Xe Graphics, as the NVIDIA GeForce RTX 4060 Laptop GPU did not start)
[ warn ]  the USB has no firmware for this PC's NVIDIA GeForce RTX 4060 Laptop GPU. On another computer, plug in this USB and run `stela repair`
--
GPU SELECT 0000:00:02.0 8086:a7a0 rev 04 Intel Iris Xe Graphics: integrated, draws; started first
GPU SELECT 0000:01:00.0 10de:28e0 rev a1 NVIDIA GeForce RTX 4060 Laptop GPU: discrete, draws; not started: its claim was refused: no pinned firmware for the observed GPU chipset was loaded
GPU SELECT started in this order: Intel Iris Xe Graphics at 0000:00:02.0. A discrete GPU that draws gives the Metal device before an integrated one, and the next one does where it fails; the display is the first of them with a connected display, else the firmware's screen"#,
    );
}

/// A release drives NVIDIA GPUs alone: on any PC its screen names neither
/// the AMD and Intel ports nor what they drive, and a PC with only AMD or
/// Intel GPUs gets the words of a PC without an NVIDIA GPU.
#[test]
fn a_release_never_mentions_the_amd_and_intel_ports() {
    let others: &[&[Gpu]] = &[
        &[RX_7900_XTX.screen()],
        &[ARC_A770.screen()],
        &[RADEON_780M.screen()],
        &[IRIS_XE.screen()],
        &[RX_7600M_XT, RADEON_780M.screen()],
        &[UHD_770.screen(), ARC_A770],
        &[RX_580.screen()],
    ];
    for pc in others {
        let (screen, _) = boot(pc, Build::Release);
        assert_eq!(screen.len(), 1, "{screen:?}");
        assert!(
            screen[0]
                .starts_with("[ warn ]  Starting the GPU (this PC has no NVIDIA GPU, only the "),
            "{screen:?}"
        );
        assert!(
            screen[0].ends_with(
                ": Stela needs an NVIDIA GeForce GTX 16 or RTX 20, 30, 40 or 50 series GPU)"
            ),
            "{screen:?}"
        );
    }
    let mixed: &[&[Gpu]] = &[
        &[IRIS_XE_RPL.screen(), RTX_4060_LAPTOP],
        &[RTX_4060_LAPTOP, RADEON_780M.screen()],
        &[UHD_770.screen(), RTX_5080],
        &[BMC.screen()],
    ];
    for pc in others.iter().chain(mixed) {
        let (screen, _) = boot(pc, Build::Release);
        let screen = screen.join(
            "
",
        );
        for word in [
            "ported",
            "AMDGPU",
            "amdgpu",
            "i915",
            " xe ",
            "RADV",
            "ANV",
            "Vega or later",
            "Core processors",
            "drives",
        ] {
            assert!(!screen.contains(word), "{word}: {screen}");
        }
    }
}

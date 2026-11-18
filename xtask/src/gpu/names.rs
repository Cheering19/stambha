//! The market names of NVIDIA, AMD and Intel display controllers, by PCI
//! IDs, for the boot screen and log (`stela_drivers::gpu::names`). They are
//! generated from pinned sources, never typed by hand:
//!
//! - the PCI ID database's `pci.ids` ([`PCI_IDS_VERSION`], GPL-2.0-or-later
//!   or BSD-3-Clause), a release checked by its size and SHA-256: NVIDIA and
//!   AMD devices, and Intel's where it names them;
//! - libdrm's `data/amdgpu.ids` (MIT), AMD's own table by device and
//!   revision, at the `third_party/libdrm` pin;
//! - Mesa's `include/pci_ids/iris_pci_ids.h` and `crocus_pci_ids.h` (MIT) at
//!   the `third_party/mesa` pin: every Intel display controller Mesa drives,
//!   and its name where `pci.ids` has none.
//!
//! Only display controllers are kept: `pci.ids` entries whose own name marks
//! another function (audio, USB and the like) are left out, and Intel IDs
//! are Mesa's.
use super::fetch::pinned_file;
use crate::tools::root;
use anyhow::{Context, Result, ensure};
use std::{collections::BTreeMap, fmt::Write as _, fs, path::Path, process::Command};

/// The generated table, beside the lookup that reads it.
pub const GENERATED: &str = "crates/stela-driver-gpu/src/names/table.rs";

/// The `pci.ids` release the table is made from (its `Version:` line), as
/// the PCI ID Project publishes it (https://pci-ids.ucw.cz/, history at
/// https://github.com/pciutils/pciids) and hwdata installs it as
/// `/usr/share/hwdata/pci.ids`.
pub const PCI_IDS_VERSION: &str = "2026.02.12";
const PCI_IDS_BYTES: usize = 1_584_801;
const PCI_IDS_SHA256: &str = "d57b3524531fcc31701042f09f6dba44f85c951c7cbec5ba74a8724f85b59fd3";

const NVIDIA: u16 = 0x10de;
const AMD: u16 = 0x1002;
const INTEL: u16 = 0x8086;
/// A name that holds for every revision of its device.
const ANY: u16 = 0x100;

/// Words in a `pci.ids` device's own name that mark a function other than
/// a display controller.
const NOT_DISPLAY: &[&str] = &[
    "Audio",
    "USB",
    "UCSI",
    "SMBus",
    "Ethernet",
    "SATA",
    "IDE",
    "LPC",
    "Bridge",
    "Memory Controller",
];

/// Writes [`GENERATED`] from `pci_ids`, the pinned release, and the
/// submodules' pinned files.
pub fn generate(pci_ids: &Path) -> Result<()> {
    let data = fs::read(pci_ids).with_context(|| pci_ids.display().to_string())?;
    ensure!(
        data.len() == PCI_IDS_BYTES && stela_formats::sha256(&data) == PCI_IDS_SHA256,
        "{} is not pci.ids {PCI_IDS_VERSION} ({PCI_IDS_BYTES} bytes, SHA-256 {PCI_IDS_SHA256})",
        pci_ids.display()
    );
    let pci_ids = String::from_utf8_lossy(&data);
    let (libdrm, amdgpu_ids) = pinned_file("libdrm", "data/amdgpu.ids")?;
    let (mesa, iris) = pinned_file("mesa", "include/pci_ids/iris_pci_ids.h")?;
    let (_, crocus) = pinned_file("mesa", "include/pci_ids/crocus_pci_ids.h")?;
    let sources = format!(
        "pci.ids {PCI_IDS_VERSION}, libdrm {} data/amdgpu.ids, Mesa {} include/pci_ids",
        &libdrm[..12],
        &mesa[..12]
    );
    let names = table(&pci_ids, &amdgpu_ids, &[&iris, &crocus])?;
    let generated = root().join(GENERATED);
    fs::create_dir_all(generated.parent().context("no parent")?)?;
    fs::write(&generated, source(&sources, &names))?;
    let status = Command::new("rustfmt")
        .args(["--edition", "2024"])
        .arg(&generated)
        .status()?;
    ensure!(status.success(), "rustfmt failed on {GENERATED}");
    println!("wrote {GENERATED}: {} names from {sources}", names.len());
    Ok(())
}

/// Each name by (vendor, device, revision or [`ANY`]), without its maker.
fn table(
    pci_ids: &str,
    amdgpu_ids: &str,
    mesa: &[&str],
) -> Result<BTreeMap<(u16, u16, u16), String>> {
    let mut names = BTreeMap::new();
    let devices = pci_devices(pci_ids)?;
    let intel: BTreeMap<u16, String> = mesa
        .iter()
        .map(|text| mesa_names(text))
        .collect::<Result<Vec<_>>>()?
        .into_iter()
        .flatten()
        .collect();
    ensure!(!intel.is_empty(), "Mesa's Intel tables hold no device");
    for ((vendor, device), name) in &devices {
        let Some(name) = display_name(name) else {
            continue;
        };
        match *vendor {
            NVIDIA | AMD => {
                names.insert((*vendor, *device, ANY), name);
            }
            INTEL if intel.contains_key(device) => {
                names.insert((INTEL, *device, ANY), name);
            }
            _ => {}
        }
    }
    for (device, name) in &intel {
        names
            .entry((INTEL, *device, ANY))
            .or_insert_with(|| plain(name));
    }
    let amd = amdgpu_names(amdgpu_ids)?;
    ensure!(!amd.is_empty(), "amdgpu.ids holds no device");
    for ((device, revision), name) in amd {
        names.insert((AMD, device, u16::from(revision)), plain(&name));
    }
    Ok(names)
}

/// Every NVIDIA, AMD and Intel device line of `pci.ids`: its IDs and name.
fn pci_devices(text: &str) -> Result<BTreeMap<(u16, u16), String>> {
    let mut devices = BTreeMap::new();
    let mut vendor = None;
    for line in text.lines() {
        if line.starts_with('#') || line.trim().is_empty() {
            continue;
        }
        // The device classes follow the vendors.
        if line.starts_with("C ") {
            break;
        }
        if line.starts_with("\t\t") {
            continue;
        }
        let hex = |field: &str| u16::from_str_radix(field, 16);
        if let Some(entry) = line.strip_prefix('\t') {
            let (Some(vendor), Some((device, name))) = (vendor, entry.split_once("  ")) else {
                continue;
            };
            if [NVIDIA, AMD, INTEL].contains(&vendor) {
                let device = hex(device).with_context(|| format!("pci.ids line {line:?}"))?;
                devices.insert((vendor, device), name.to_owned());
            }
            continue;
        }
        vendor = line.split_once("  ").and_then(|(id, _)| hex(id).ok());
    }
    ensure!(
        devices.keys().any(|&(vendor, _)| vendor == NVIDIA),
        "pci.ids holds no NVIDIA device"
    );
    Ok(devices)
}

/// The market name in a `pci.ids` device name, the part in brackets after
/// the chip (`GB203 [GeForce RTX 5080]`), when the device is a display
/// controller.
fn display_name(name: &str) -> Option<String> {
    let (chip, rest) = name.split_once('[')?;
    let (market, _) = rest.rsplit_once(']')?;
    if NOT_DISPLAY.iter().any(|word| chip.contains(word)) {
        return None;
    }
    let market = plain(market);
    (!market.is_empty()).then_some(market)
}

/// `amdgpu.ids`' lines `device,\trevision,\tname`, both IDs in hex.
fn amdgpu_names(text: &str) -> Result<BTreeMap<(u16, u8), String>> {
    let mut names = BTreeMap::new();
    for line in text.lines() {
        let fields: Vec<&str> = line.splitn(3, ',').map(str::trim).collect();
        let [device, revision, name] = fields[..] else {
            // Comments and the format's version line.
            continue;
        };
        let (Ok(device), Ok(revision)) = (
            u16::from_str_radix(device, 16),
            u8::from_str_radix(revision, 16),
        ) else {
            continue;
        };
        names.insert((device, revision), name.to_owned());
    }
    Ok(names)
}

/// Mesa's `CHIPSET(0x56a0, dg2_g10, "DG2", "Intel(R) Arc(tm) A770 Graphics")`
/// lines: each device and its name.
fn mesa_names(text: &str) -> Result<Vec<(u16, String)>> {
    let mut names = Vec::new();
    for line in text.lines() {
        // A comment may follow the closing parenthesis.
        let Some((arguments, _)) = line
            .trim()
            .strip_prefix("CHIPSET(")
            .and_then(|line| line.rsplit_once(')'))
        else {
            continue;
        };
        let device = arguments
            .split(',')
            .next()
            .and_then(|id| id.trim().strip_prefix("0x"))
            .and_then(|id| u16::from_str_radix(id, 16).ok())
            .with_context(|| format!("Mesa line {line:?}"))?;
        // The last quoted argument; flags such as `FORCE_PROBE` may follow.
        let Some(name) = arguments.split('"').skip(1).step_by(2).last() else {
            anyhow::bail!("Mesa line {line:?} names no device");
        };
        // A placeholder Mesa replaces at run time names nothing.
        if !name.contains("XXX") {
            names.push((device, name.to_owned()));
        }
    }
    Ok(names)
}

/// `name` without trademark marks and its maker, which the lookup puts back
/// in one spelling.
fn plain(name: &str) -> String {
    let mut name = name.to_owned();
    for mark in ["(R)", "(r)", "(TM)", "(tm)", "®", "™"] {
        name = name.replace(mark, "");
    }
    let name = name.split_whitespace().collect::<Vec<_>>().join(" ");
    let mut name = name.as_str();
    for maker in ["NVIDIA ", "AMD ", "ATI ", "Intel "] {
        name = name.strip_prefix(maker).unwrap_or(name);
    }
    name.to_owned()
}

/// The generated Rust source of `names`.
fn source(sources: &str, names: &BTreeMap<(u16, u16, u16), String>) -> String {
    // Each distinct name once, in the order first used.
    let mut text = String::new();
    let mut offsets: BTreeMap<&str, usize> = BTreeMap::new();
    let mut lines = String::new();
    let mut entries = String::new();
    for ((vendor, device, revision), name) in names {
        let offset = *offsets.entry(name).or_insert_with(|| {
            let offset = text.len();
            text.push_str(name);
            text.push('\n');
            let _ = writeln!(lines, "    {:?},", format!("{name}\n"));
            offset
        });
        let _ = writeln!(
            entries,
            "    ({vendor:#06x}, {device:#06x}, {revision:#05x}, {offset}),"
        );
    }
    format!(
        "//! Generated by `cargo xtask gpu names` from {sources}; do not edit.\n\
         //! `xtask/src/gpu/names.rs` says what is kept.\n\n\
         /// Where the names come from.\n\
         pub(super) const SOURCES: &str = {sources:?};\n\n\
         /// Every name, without its maker, each ended by a newline.\n\
         pub(super) const TEXT: &str = concat!(\n{lines});\n\n\
         /// Each name's PCI vendor, device, revision or `0x100` for any, and \
         where it starts in [`TEXT`], sorted.\n\
         pub(super) static ENTRIES: [(u16, u16, u16, u32); {}] = [\n{entries}];\n",
        names.len()
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    const PCI_IDS: &str = "\
# comment
10de  NVIDIA Corporation
\t2c02  GB203 [GeForce RTX 5080]
\t\t1043 89f2  some board
\t2c22  GB203 High Definition Audio Controller
\t2c23  GB203 USB Type-C [UCSI]
1002  Advanced Micro Devices, Inc. [AMD/ATI]
\t13c0  Granite Ridge [Radeon Graphics]
\t744c  Navi 31 [Radeon RX 7900 XT/7900 XTX/7900 GRE/7900M]
\tab30  Navi 31 HDMI/DP Audio [Radeon RX 7900]
8086  Intel Corporation
\t56a0  DG2 [Arc A770]
\ta780  Raptor Lake-S GT1 [UHD Graphics 770]
\t7d55  Meteor Lake-P [Intel Arc Graphics]
\t1602  Broadwell-U Integrated Graphics
\t2723  Wi-Fi 6 AX200 [Wi-Fi 6 AX200 160MHz]
C 00  Unclassified device
\t00  Non-VGA unclassified device
";

    const AMDGPU_IDS: &str = "\
# List of AMDGPU IDs
#
# Syntax:
# device_id,\trevision_id,\tproduct_name        <-- single tab after comma

1.0.0
744C,\tC8,\tAMD Radeon RX 7900 XTX
744C,\tCC,\tAMD Radeon RX 7900 XT
";

    const IRIS: &str = "\
CHIPSET(0x1602, bdw_gt1, \"BDW GT1\", \"Intel(R) HD Graphics\")
CHIPSET(0x56a0, dg2_g10, \"DG2\", \"Intel(R) Arc(tm) A770 Graphics\")
CHIPSET(0xa780, rpl, \"RPL-S\", \"Intel(R) Graphics\")
CHIPSET(0xd750, nvl_p, \"NVL-P\", \"Intel(R) Graphics\", FORCE_PROBE)
CHIPSET(0x7d55, mtl_h, \"MTL\", \"Intel(R) Arc(tm) Graphics\")
CHIPSET(0x22b1, chv, \"BSW\", \"Intel(R) HD Graphics XXX\") /* Overridden at run time */
";

    #[test]
    fn names_are_market_names_of_display_controllers_only() {
        let names = table(PCI_IDS, AMDGPU_IDS, &[IRIS]).unwrap();
        let name = |vendor, device, revision| names.get(&(vendor, device, revision)).cloned();
        assert_eq!(
            name(NVIDIA, 0x2c02, ANY).as_deref(),
            Some("GeForce RTX 5080")
        );
        // Audio and USB functions, and the class section, are left out.
        assert_eq!(name(NVIDIA, 0x2c22, ANY), None);
        assert_eq!(name(NVIDIA, 0x2c23, ANY), None);
        assert_eq!(name(AMD, 0xab30, ANY), None);
        // AMD: its own table by revision, the database for the device.
        assert_eq!(
            name(AMD, 0x744c, 0xc8).as_deref(),
            Some("Radeon RX 7900 XTX")
        );
        assert_eq!(
            name(AMD, 0x744c, 0xcc).as_deref(),
            Some("Radeon RX 7900 XT")
        );
        assert_eq!(
            name(AMD, 0x744c, ANY).as_deref(),
            Some("Radeon RX 7900 XT/7900 XTX/7900 GRE/7900M")
        );
        assert_eq!(name(AMD, 0x13c0, ANY).as_deref(), Some("Radeon Graphics"));
        // Intel: Mesa's devices, the database's name first, the maker once.
        assert_eq!(name(INTEL, 0x56a0, ANY).as_deref(), Some("Arc A770"));
        assert_eq!(
            name(INTEL, 0xa780, ANY).as_deref(),
            Some("UHD Graphics 770")
        );
        assert_eq!(name(INTEL, 0x7d55, ANY).as_deref(), Some("Arc Graphics"));
        assert_eq!(name(INTEL, 0x1602, ANY).as_deref(), Some("HD Graphics"));
        assert_eq!(name(INTEL, 0xd750, ANY).as_deref(), Some("Graphics"));
        // An Intel device Mesa does not drive is no display controller here.
        assert_eq!(name(INTEL, 0x2723, ANY), None);
        assert_eq!(names.len(), 10);
    }

    #[test]
    fn the_source_holds_each_name_once_at_its_offset() {
        let names = table(PCI_IDS, AMDGPU_IDS, &[IRIS]).unwrap();
        let source = source("test", &names);
        assert!(source.contains("pub(super) static ENTRIES: [(u16, u16, u16, u32); 10]"));
        assert!(source.contains("    (0x10de, 0x2c02, 0x100, "));
        assert!(source.contains("    (0x1002, 0x744c, 0x0c8, "));
        assert_eq!(source.matches("\"Radeon Graphics\\n\"").count(), 1);
    }

    #[test]
    fn marks_and_makers_are_taken_out() {
        assert_eq!(
            plain("Intel(R) Arc(tm) A770  Graphics"),
            "Arc A770 Graphics"
        );
        assert_eq!(plain("AMD Radeon 860M Graphics"), "Radeon 860M Graphics");
        assert_eq!(plain("NVIDIA RTX A6000"), "RTX A6000");
        assert_eq!(
            display_name("GA102GL [RTX A6000]").as_deref(),
            Some("RTX A6000")
        );
        assert_eq!(display_name("TU104"), None);
    }
}

// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2015 Endless Mobile, Inc.
//! Exact USB/ROM profiles from pinned btusb.c and btrtl.c. No filename alone
//! identifies a controller, and no unknown running firmware is silently dropped.
use super::{Error, Project, Result};
use crate::{firmware::Target, usb::xhci::Device};
use alloc::{format, string::String};

/// All bytes of a standard Read Local Version return, including status.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LocalVersion {
    pub raw: [u8; 9],
}
impl LocalVersion {
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    pub(super) fn parse(p: &[u8]) -> Result<Self> {
        let raw = p
            .try_into()
            .map_err(|_| Error::protocol("Read Local Version length", p))?;
        if p[0] != 0 {
            return Err(Error::protocol("Read Local Version status", p));
        }
        Ok(Self { raw })
    }
    pub fn hci_version(&self) -> u8 {
        self.raw[1]
    }
    pub fn hci_revision(&self) -> u16 {
        u16::from_le_bytes([self.raw[2], self.raw[3]])
    }
    pub fn lmp_version(&self) -> u8 {
        self.raw[4]
    }
    pub fn manufacturer(&self) -> u16 {
        u16::from_le_bytes([self.raw[5], self.raw[6]])
    }
    pub fn lmp_subversion(&self) -> u16 {
        u16::from_le_bytes([self.raw[7], self.raw[8]])
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UsbIdentity {
    pub vendor: u16,
    pub product: u16,
    pub revision: u16,
}
impl UsbIdentity {
    pub(super) fn read(device: &Device) -> Self {
        let identity = super::super::identity::read(&device.descriptor);
        Self {
            vendor: identity.vendor,
            product: identity.product,
            revision: identity.revision,
        }
    }
    pub(crate) fn project(self) -> Option<Project> {
        match (self.vendor, self.product) {
            // Explicit btusb 8761BU/8761BUV entries; no vendor-wide expansion.
            (0x0bda, 0xa728 | 0x8771) | (0x0b05, 0x190e) => Some(Project::Rtl8761Bu),
            // Explicit btusb additional 8822BE entries.
            (0x13d3, 0x3526) | (0x0b05, 0x185c) => Some(Project::Rtl8822B),
            // Complete explicit btusb 8852BE group; independently require the
            // 8852B USB stock-ROM profile below, never infer it from the PID.
            (0x0cb8, 0xc559)
            | (0x0bda, 0x4853 | 0x887b | 0xb85b)
            | (0x13d3, 0x3570 | 0x3571 | 0x3572 | 0x3591)
            | (0x0489, 0xe123 | 0xe125) => Some(Project::Rtl8852Bu),
            // Complete explicit btusb 8852CE group (btusb.c:575-591); the
            // 8852C USB stock-ROM profile is required independently below.
            (0x04ca, 0x4007)
            | (0x04c5, 0x1675)
            | (0x0cb8, 0xc558)
            | (0x13d3, 0x3587 | 0x3586 | 0x3592 | 0x3612)
            | (0x0489, 0xe122) => Some(Project::Rtl8852Cu),
            _ => None,
        }
    }
    pub fn hardware_label(self) -> String {
        format!(
            "usb:{:04x}:{:04x}:{:04x}",
            self.vendor, self.product, self.revision
        )
    }
}
/// Complete independently observed selection, bound into every resource request.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Identity {
    pub usb: UsbIdentity,
    pub version: LocalVersion,
    pub register_subversion: u16,
    pub register_revision: Option<u16>,
    pub rom_version: u8,
    /// Both register response bytes kept; byte0 selects the security key.
    pub security: [u8; 2],
    pub project: Project,
}
impl Identity {
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    pub(super) fn select(
        usb: UsbIdentity,
        version: LocalVersion,
        register_subversion: u16,
        register_revision: Option<u16>,
        rom_version: u8,
        security: [u8; 2],
    ) -> Result<Self> {
        let project = usb.project().ok_or(Error::Unsupported("USB identity"))?;
        let (lmp, hci_rev, hci_ver) = match project {
            Project::Rtl8761Bu => (0x8761, 0xb, 0xa),
            Project::Rtl8822B => (0x8822, 0xb, 7),
            Project::Rtl8852Bu => (0x8852, 0xb, 0xb),
            // IC_INFO(RTL_ROM_LMP_8852A, 0xc, 0xc, HCI_USB) (btrtl.c:305-306).
            Project::Rtl8852Cu => (0x8852, 0xc, 0xc),
        };
        if version.manufacturer() != 0x005d
            || version.lmp_subversion() != lmp
            || version.hci_revision() != hci_rev
            || version.hci_version() != hci_ver
            || register_subversion != lmp
            || (project == Project::Rtl8822B
                && (register_revision.is_none() || register_revision == Some(0x000e)))
        {
            return Err(Error::Unsupported(
                "USB and observed stock ROM/HCI profile mismatch",
            ));
        }
        if security[0] != 0 {
            return Err(Error::Unsupported(
                "nonzero Realtek security key input/selection is unsupported",
            ));
        }
        Ok(Self {
            usb,
            version,
            register_subversion,
            register_revision,
            rom_version,
            security,
            project,
        })
    }
    /// The label bound into the first image's request
    /// ([`Self::firmware_images`]) and its configuration's.
    pub fn revision_label(&self) -> String {
        self.revision_label_for(self.firmware_images()[0].1)
    }
    /// The label bound into the request of an image in `container`.
    pub fn revision_label_for(&self, container: Container) -> String {
        let revision = self
            .register_revision
            .map_or_else(|| "none".into(), |v| format!("{v:04x}"));
        format!(
            "{}:hci-{:02x}-{:04x}:lmp-{:02x}-{:04x}:manufacturer-{:04x}:register-{:04x}-{revision}:rom-{:02x}:security-{:02x}-{:02x}",
            container.label(),
            self.version.hci_version(),
            self.version.hci_revision(),
            self.version.lmp_version(),
            self.version.lmp_subversion(),
            self.version.manufacturer(),
            self.register_subversion,
            self.rom_version,
            self.security[0],
            self.security[1]
        )
    }
    pub(super) fn target<'a>(
        &self,
        container: Container,
        hardware: &'a str,
        revision: &'a str,
    ) -> Target<'a> {
        Target {
            driver: container.driver(),
            hardware,
            revision,
            board: None,
        }
    }
    /// The images btrtl loads for this profile, first choice first
    /// (btrtl.c:1196-1209): an RTL8852C (lmp_subver 0x8852 with hci_rev
    /// 0x000c) tries `<fw_name>_v2.bin`, then `<fw_name>.bin`; every other
    /// profile its `<fw_name>.bin` alone. A later image is asked only when
    /// the one before it is absent.
    pub fn firmware_images(&self) -> &'static [(&'static str, Container)] {
        match self.project {
            Project::Rtl8761Bu => &[("rtl_bt/rtl8761bu_fw.bin", Container::V1)],
            Project::Rtl8822B => &[("rtl_bt/rtl8822b_fw.bin", Container::V1)],
            Project::Rtl8852Bu => &[("rtl_bt/rtl8852bu_fw.bin", Container::V2)],
            Project::Rtl8852Cu => &[
                ("rtl_bt/rtl8852cu_fw_v2.bin", Container::V2),
                ("rtl_bt/rtl8852cu_fw.bin", Container::V1),
            ],
        }
    }
    /// The first image's name.
    pub fn firmware_name(&self) -> &'static str {
        self.firmware_images()[0].0
    }
    pub fn configuration_name(&self) -> &'static str {
        match self.project {
            Project::Rtl8761Bu => "rtl_bt/rtl8761bu_config.bin",
            Project::Rtl8822B => "rtl_bt/rtl8822b_config.bin",
            Project::Rtl8852Bu => "rtl_bt/rtl8852bu_config.bin",
            Project::Rtl8852Cu => "rtl_bt/rtl8852cu_config.bin",
        }
    }
    pub fn configuration_required(&self) -> bool {
        match self.project {
            Project::Rtl8822B => true,
            // `.config_needed = false` (btrtl.c:307).
            Project::Rtl8761Bu | Project::Rtl8852Bu | Project::Rtl8852Cu => false,
        }
    }
}

/// The EPATCH container a profile's image is, by its name: RTBTCore
/// (`_v2.bin` and RTL8852BU's file) or Realtech. Its request's target names
/// it, and it chooses the parser; an image of the other signature is
/// refused rather than parsed as Linux would (btrtl.c:680-687).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Container {
    V1,
    V2,
}
impl Container {
    fn label(self) -> &'static str {
        match self {
            Self::V1 => "epatch-v1",
            Self::V2 => "epatch-v2",
        }
    }
    fn driver(self) -> &'static str {
        match self {
            Self::V1 => "realtek-bluetooth-usb-epatch-v1",
            Self::V2 => "realtek-bluetooth-usb-epatch-v2",
        }
    }
    /// The shortest image of this container the parser can take.
    pub(super) fn minimum_bytes(self) -> usize {
        match self {
            Self::V1 => 22,
            Self::V2 => 28,
        }
    }
}

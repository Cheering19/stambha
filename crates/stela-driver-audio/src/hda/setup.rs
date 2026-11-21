// SPDX-License-Identifier: GPL-2.0-only
//! The vendor codec setup a codec's identity selects when it binds, as
//! Linux's codec patch table selects its patch driver.
use super::{
    Alc269CommonReport, CirrusReport, CodecGraph, ConexantHeadsetInitialization, ConexantReport,
    Controller, RealtekAlc256Report, RealtekReport, Registers, SigmatelReport, cirrus, conexant,
    realtek, sigmatel,
};
use crate::{Error, Timer};

/// What [`Controller::initialize_codec`] applied.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CodecSetup {
    /// Managed ALC882/ALC662-family setup, with any PCI-subsystem OEM procedure.
    Realtek(RealtekReport),
    Alc269(Alc269CommonReport),
    /// Linux's ALC256 init, before any route.
    Alc256(RealtekAlc256Report),
    Cirrus(CirrusReport),
    Conexant(ConexantReport),
    ConexantHeadset(ConexantHeadsetInitialization),
    Sigmatel(SigmatelReport),
    /// No vendor table names the codec; its standard widgets need no setup,
    /// as Linux's generic parser drives them.
    Standard,
}

impl core::fmt::Display for CodecSetup {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Realtek(report) => write!(f, "Realtek {:?} ({:?})", report.family, report.oem),
            Self::Alc269(report) => write!(f, "Realtek ALC269 {:?}", report.variant),
            Self::Alc256(report) => {
                let jack = match report.headphone_sense {
                    Some(sense) if sense.present => "plugged in",
                    Some(_) => "empty",
                    None => "no jack sense",
                };
                write!(
                    f,
                    "Realtek ALC256, headphone pin {:#04x} ({jack}), ",
                    report.headphone_pin
                )?;
                match report.speaker_pin {
                    Some(pin) => write!(f, "speaker pin {pin:#04x}")?,
                    None => f.write_str("no speaker pin")?,
                }
                f.write_str(", EAPD on")?;
                if report.eapd_pins.is_empty() {
                    f.write_str(" no pin")?;
                }
                for pin in &report.eapd_pins {
                    write!(f, " {pin:#04x}")?;
                }
                Ok(())
            }
            Self::Cirrus(report) => write!(f, "Cirrus {:?}", report.profile),
            Self::Conexant(_) => f.write_str("Conexant"),
            Self::ConexantHeadset(_) => f.write_str("Conexant headset"),
            Self::Sigmatel(report) => write!(f, "Sigmatel {:?}", report.profile),
            Self::Standard => f.write_str("standard widgets"),
        }
    }
}

impl<R: Registers, T: Timer> Controller<R, T> {
    /// Apply the vendor setup `graph`'s identity selects. A codec outside
    /// every vendor table is left as discovered.
    pub fn initialize_codec(&mut self, graph: &CodecGraph) -> Result<CodecSetup, Error> {
        self.minimum_guard()?;
        let id = graph.vendor_id;
        if id == 0x10ec_0269 {
            self.initialize_realtek_alc269_common(graph)
                .map(CodecSetup::Alc269)
        } else if id == 0x10ec_0256 {
            self.initialize_realtek_alc256(graph)
                .map(CodecSetup::Alc256)
        } else if realtek::managed(id, graph.revision_id) {
            let report = if self.realtek_oem_selected(graph)? {
                self.initialize_realtek_oem_codec(graph)
            } else {
                self.initialize_realtek_codec(graph)
            };
            report.map(CodecSetup::Realtek)
        } else if cirrus::profile(id).is_ok() {
            self.initialize_cirrus_codec(graph).map(CodecSetup::Cirrus)
        } else if conexant::headset::is_profile(id) {
            self.initialize_conexant_headset_codec(graph)
                .map(CodecSetup::ConexantHeadset)
        } else if conexant::profile(id).is_ok() {
            self.initialize_conexant_codec(graph)
                .map(CodecSetup::Conexant)
        } else if sigmatel::profile(id).is_ok() {
            self.initialize_sigmatel_codec(graph)
                .map(CodecSetup::Sigmatel)
        } else {
            Ok(CodecSetup::Standard)
        }
    }
}

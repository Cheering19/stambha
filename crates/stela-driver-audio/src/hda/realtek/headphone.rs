// SPDX-License-Identifier: GPL-2.0-only
//! Timed Realtek headphone startup adapted from alc269.c::{alc_default_init,alc256_init} and
//! realtek.c::alc_fill_eapd_coef at
//! a804c5742bff224d4cb4d25dd383554d598be32b (GPL-2.0-or-later).
//! This is a selected standard headphone route; OEM GPIO/companion amplifiers,
//! presence inversion and headset CTIA/OMTP policies are not inferred.
//! An ALC256's bind runs the same init before any route, on the headphone
//! and speaker pins auto_parser.c::snd_hda_parse_pin_defcfg assigns, with
//! realtek.c::{alc_get_hp_pin,alc_auto_init_amp} and alc269.c::alc269_probe,
//! read at cc9de5d4b5ab6459f5774946aa3195ad51236fa8 (Linux 7.2.12).
use super::{CoefficientChange, RealtekCommonReport};
use crate::{
    Direction, Error, Timer,
    hda::{
        AnalogRoute, CodecGraph, Controller, PinConfiguration, PinConnection, PinDevice,
        PinOptions, PinSense, Registers, StreamId, Verb, WidgetKind,
    },
};
use alloc::vec::Vec;

/// Startup measurements and observed common controls after the explicit route
/// was configured and reserved. This is not an acoustic or OEM-readiness claim.
/// Common coefficient observations are ordered transactions; an index may
/// occur more than once. Fixed depop endpoints are verified during startup.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RealtekHeadphoneReport {
    pub common: RealtekCommonReport,
    pub headphone_pin: u8,
    pub initial_sense: PinSense,
}

/// What [`Controller::initialize_realtek_alc256`] applied as the codec
/// bound, before any route.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RealtekAlc256Report {
    pub common: RealtekCommonReport,
    /// The pin `alc256_init` sensed: `alc_get_hp_pin`'s, else 0x21.
    pub headphone_pin: u8,
    /// Its jack; `None` where the pin cannot sense one, which skips the
    /// jack's power-up as an empty jack does.
    pub headphone_sense: Option<PinSense>,
    /// The parser's first speaker, which a route drives.
    pub speaker_pin: Option<u8>,
    /// The listed pins whose EAPD `alc_auto_init_amp` turned on.
    pub eapd_pins: Vec<u8>,
}

/// `alc_auto_setup_eapd`'s pins: "We currently only handle front, HP".
const EAPD_PINS: [u8; 5] = [0x0f, 0x10, 0x14, 0x15, 0x17];
/// AC_PINCAP_EAPD and AC_PINCAP_OUT.
const PIN_EAPD: u32 = 1 << 16;
const PIN_OUT: u32 = 1 << 4;
/// AC_WCAP_POWER: the widget has its own power state.
const WIDGET_POWER: u32 = 1 << 10;
/// `alc269_ignore`: the PC-beep pin the parser skips.
const PC_BEEP: u8 = 0x1d;
/// `AUTO_CFG_MAX_OUTS`: pins of one kind past this many are ignored.
const MAX_OUTS: usize = 5;
/// `alc256_init`'s pin where the parser found no headphone.
const FALLBACK_HEADPHONE: u8 = 0x21;

/// The headphone and speaker pins among `pins` (node, configuration
/// default, pin capabilities, in node order), as `snd_hda_parse_pin_defcfg`
/// assigns its outputs for `alc269_parse_auto_config` (the PC-beep pin
/// ignored, no parser flags): `alc_get_hp_pin`'s headphone, the first of
/// `hp_pins`, else the first line out where headphones took that role, and
/// the first speaker likewise. Pins of equal sequence keep node order here,
/// where Linux's heap sort leaves them in no set order; its
/// `reorder_outputs` moves no first pin.
fn outputs(pins: impl IntoIterator<Item = (u8, u32, u32)>) -> (Option<u8>, Option<u8>) {
    let mut line = Vec::new();
    let mut speakers = Vec::new();
    let mut headphones = Vec::new();
    let mut line_association = 0;
    for (node, raw, caps) in pins {
        let configuration = PinConfiguration { raw };
        let connection = configuration.connection();
        if node == PC_BEEP || connection == PinConnection::None {
            continue;
        }
        let mut device = configuration.device();
        // "workaround for buggy BIOS setups"
        if device == PinDevice::LineOut
            && matches!(
                connection,
                PinConnection::Fixed | PinConnection::JackAndFixed
            )
        {
            device = PinDevice::Speaker;
        }
        // check_pincap_validity: an output needs AC_PINCAP_OUT, unless the
        // capabilities read zero.
        let output = matches!(
            device,
            PinDevice::LineOut | PinDevice::Speaker | PinDevice::Headphone
        );
        if !output || (caps != 0 && caps & PIN_OUT == 0) {
            continue;
        }
        let association = configuration.association();
        let sequence = configuration.sequence();
        let (list, order) = match device {
            PinDevice::LineOut => {
                if association == 0 {
                    continue;
                }
                if line_association == 0 {
                    line_association = association;
                } else if line_association != association {
                    continue;
                }
                (&mut line, sequence)
            }
            PinDevice::Speaker => (&mut speakers, association << 4 | sequence),
            _ => (&mut headphones, association << 4 | sequence),
        };
        if list.len() < MAX_OUTS {
            list.push((node, order));
        }
    }
    // "If no line-out is defined but multiple HPs are found, some of them
    // might be the real line-outs": those not of sequence 0x0f.
    let mut headphone_line = false;
    if line.is_empty() && headphones.len() > 1 {
        let (real, moved) = headphones
            .into_iter()
            .partition::<Vec<_>, _>(|&(_, order)| order & 0x0f == 0x0f);
        headphones = real;
        line = moved;
        headphone_line = headphones.is_empty();
    }
    for list in [&mut line, &mut speakers, &mut headphones] {
        list.sort_by_key(|&(_, order)| order);
    }
    // "if no line-outs are detected, try to use speaker or HP pin as a
    // primary output"
    let mut speaker_line = false;
    if line.is_empty() {
        if !speakers.is_empty() {
            line = core::mem::take(&mut speakers);
            speaker_line = true;
        } else if !headphones.is_empty() {
            line = core::mem::take(&mut headphones);
            headphone_line = true;
        }
    }
    let first = |list: &[(u8, u8)]| list.first().map(|&(node, _)| node);
    let line_as = |role: bool| role.then(|| first(&line)).flatten();
    (
        first(&headphones).or_else(|| line_as(headphone_line)),
        first(&speakers).or_else(|| line_as(speaker_line)),
    )
}
// These are fixed endpoints in the ALC256/257 donor procedure, not selectable
// register banks or arbitrary graph NIDs. The public caller chooses only hp_pin.
#[derive(Clone, Copy)]
pub(super) enum DepopControl {
    HeadphonePower,
    AnalogReset,
}
#[derive(Clone, Copy)]
enum DepopProfile {
    Alc255,
    Alc256And257,
}

impl<R: Registers, T: Timer> Controller<R, T> {
    pub(super) fn headphone_preflight(
        &mut self,
        graph: &CodecGraph,
        route: &AnalogRoute,
        id: StreamId,
        channel: u8,
        options: PinOptions,
    ) -> Result<bool, Error> {
        self.minimum_guard()?;
        self.realtek_common_guard(graph)?;
        self.preflight_analog_route(graph, route, id, channel, options)?;
        if route.direction() != Direction::Playback
            || route.group_node() != 1
            || channel != 0
            || self.stream_format(id)?.channels > 2
            || !matches!(
                options,
                PinOptions::Playback {
                    headphone: true,
                    ..
                }
            )
        {
            return Err(Error::UnsupportedTopology);
        }
        let pin = graph.widget(route.pin()).ok_or(Error::InvalidCodecGraph)?;
        let converter = graph
            .widget(route.converter())
            .ok_or(Error::InvalidCodecGraph)?;
        let configuration = pin.pin_configuration().ok_or(Error::InvalidCodecGraph)?;
        if pin.kind != WidgetKind::Pin
            || pin.channels() != 2
            || converter.channels() != 2
            || configuration.device() != PinDevice::Headphone
            || configuration.connection() == PinConnection::None
            || configuration.no_presence_detection()
            || pin
                .pin_capabilities
                .is_none_or(|value| value == u32::MAX || value & 0x1c != 0x1c)
            || pin
                .output_amplifier
                .is_none_or(|value| value == u32::MAX || value & (1 << 31) == 0)
        {
            return Err(Error::UnsupportedTopology);
        }
        // Graph fields are public metadata. Recheck the selected pin,
        // rather than authorizing pin/amp writes from edited cached fields.
        let nodes = self.realtek_parameter(graph.address, 1, 4)?;
        let first = (nodes >> 16) & 255;
        let count = nodes & 255;
        let end = first + count;
        if first < 2
            || count == 0
            || end > 128
            || !(first..end).contains(&u32::from(route.pin()))
            || !(first..end).contains(&u32::from(route.converter()))
        {
            return Err(Error::InvalidCodecGraph);
        }
        let caps = self.realtek_parameter(graph.address, route.pin(), 9)?;
        let pin_caps = self.realtek_parameter(graph.address, route.pin(), 0x0c)?;
        let defaults = self.verb(Verb::short(graph.address, route.pin(), 0xf1c, 0)?)?;
        let amp = self.realtek_parameter(
            graph.address,
            if caps & 8 != 0 { route.pin() } else { 1 },
            0x12,
        )?;
        if caps != pin.capabilities
            || pin.pin_capabilities != Some(pin_caps)
            || pin.pin_default != Some(defaults)
            || pin.output_amplifier != Some(amp)
        {
            return Err(Error::InvalidCodecGraph);
        }
        let pin_power = caps & (1 << 10) != 0;
        if pin_power {
            let raw = self.realtek_parameter(graph.address, route.pin(), 0x0f)?;
            let states = if raw == 0 {
                self.realtek_parameter(graph.address, 1, 0x0f)?
            } else {
                raw
            };
            if states == u32::MAX || states & 1 == 0 {
                return Err(Error::UnsupportedTopology);
            }
        }
        Ok(pin_power)
    }
    pub(super) fn headphone_unmute_zero(&mut self, codec: u8, pin: u8) -> Result<(), Error> {
        self.minimum_guard()?;
        // Exact AMP_OUT_UNMUTE: both declared stereo channels, gain code zero.
        // Generic route programming subsequently chooses its advertised 0dB.
        self.verb(Verb::long(codec, pin, 3, 0xb000)?)?;
        for get in [0xa000, 0x8000] {
            if self.verb(Verb::long(codec, pin, 11, get)?)? != 0 {
                return Err(Error::RouteRejected);
            }
        }
        Ok(())
    }

    /// Exact ALC298 standard-power startup followed by one explicit headphone
    /// route. A real firmware-described headphone pin and PCM path are required;
    /// the Linux fallback pin is not substituted for an unknown board route.
    /// Presence uses only standard non-inverted polling. Initial 30ms, present-
    /// jack 2ms, pin-enable 75ms and unmute 75ms intervals use the same Timer.
    /// Any failure after power/programming begins quarantines typed route/start
    /// until controller reinitialization; a successful route reserves its path.
    pub fn configure_realtek_alc298_headphone(
        &mut self,
        graph: &CodecGraph,
        route: &AnalogRoute,
        id: StreamId,
        channel: u8,
        options: PinOptions,
    ) -> Result<RealtekHeadphoneReport, Error> {
        self.minimum_guard()?;
        self.realtek_common_guard(graph)?;
        if graph.vendor_id != 0x10ec_0298 {
            return Err(Error::UnsupportedTopology);
        }
        let pin_power = self.headphone_preflight(graph, route, id, channel, options)?;
        let (codec_id, revision_id, subsystem_id, vendor_power) =
            self.realtek_common_identity(graph)?;
        // alc269_probe's ALC5505 hook replacement is outside this normal path.
        if self.realtek_parameter(graph.address, 0x51, 0)? != 0 {
            return Err(Error::UnsupportedTopology);
        }
        let result = (|| {
            self.power_d0(graph.address, 1)?;
            if vendor_power {
                self.power_d0(graph.address, 0x20)?;
            }
            if pin_power {
                self.power_d0(graph.address, route.pin())?;
            }
            let coefficients =
                alloc::vec![self.clear_coefficient_bits(graph.address, 0x10, 1 << 9)?];
            self.delay(30_000)?;
            let initial_sense = self.poll_pin_sense(graph, route.pin())?;
            if initial_sense.present {
                self.delay(2_000)?;
                self.route_control(graph.address, route.pin(), 0x707, 0xf07, 0x40)?;
                self.delay(75_000)?;
                self.headphone_unmute_zero(graph.address, route.pin())?;
                self.delay(75_000)?;
            }
            self.configure_analog_route(graph, route, id, channel, options)?;
            Ok(RealtekHeadphoneReport {
                common: RealtekCommonReport {
                    codec_id,
                    revision_id,
                    subsystem_id,
                    coefficients,
                },
                headphone_pin: route.pin(),
                initial_sense,
            })
        })();
        if result.is_err() {
            self.routing_failed = true;
        }
        result
    }
    pub(super) fn depop_control(
        &mut self,
        codec: u8,
        control: DepopControl,
        setting: u16,
    ) -> Result<(), Error> {
        self.minimum_guard()?;
        let (node, index, mask) = match control {
            DepopControl::HeadphonePower => (0x57, 4, 7),
            DepopControl::AnalogReset => (0x53, 2, 0x8000),
        };
        let fault = Error::CoefficientRejected { node, index };
        if setting & !mask != 0 {
            return Err(Error::InvalidConfiguration);
        }
        // Exact family identity is checked before this helper. These hidden
        // vendor endpoints are not invented standard widget-capability nodes;
        // positive index and data readback are required for every transaction.
        self.verb(Verb::long(codec, node, 5, index)?)?;
        if self.verb(Verb::long(codec, node, 13, 0)?)? != u32::from(index) {
            return Err(fault);
        }
        let previous =
            u16::try_from(self.verb(Verb::long(codec, node, 12, 0)?)?).map_err(|_| fault)?;
        let current = (previous & !mask) | setting;
        self.verb(Verb::long(codec, node, 5, index)?)?;
        if self.verb(Verb::long(codec, node, 13, 0)?)? != u32::from(index) {
            return Err(fault);
        }
        // Keep the donor's write even when the value matches, notably the
        // ordered high/low analog-reset transactions.
        self.verb(Verb::long(codec, node, 4, current)?)?;
        self.verb(Verb::long(codec, node, 5, index)?)?;
        if self.verb(Verb::long(codec, node, 13, 0)?)? != u32::from(index)
            || self.verb(Verb::long(codec, node, 12, 0)?)? != u32::from(current)
        {
            return Err(fault);
        }
        Ok(())
    }

    /// Exact ALC256/257 held-D0 headphone depop/startup, then explicit route
    /// configuration/reservation. It uses alc256_init's fixed node57 power,
    /// node53 analog-reset pulse and node20 controls, including the literal
    /// 0x5757 PC-beep/1Ah routing choice. The same live pin/codec/format and
    /// presence checks as ALC298 apply. ULP, OEM headset/GPIO/amp and shutup
    /// policies are not selected; those need their own board ownership.
    pub fn configure_realtek_alc256_headphone(
        &mut self,
        graph: &CodecGraph,
        route: &AnalogRoute,
        id: StreamId,
        channel: u8,
        options: PinOptions,
    ) -> Result<RealtekHeadphoneReport, Error> {
        self.minimum_guard()?;
        self.configure_realtek_depop_headphone(
            graph,
            route,
            id,
            channel,
            options,
            DepopProfile::Alc256And257,
        )
    }

    /// Exact ALC255 held-D0 headphone startup and explicit route reservation.
    /// Its fixed depop/reset sequence matches alc256_init, but its initial
    /// EAPD selection clears only node20/index10 bit9: index36 bit13 is left
    /// untouched until the donor's final unconditional 0x5757 write.
    /// The same live pin, presence, failure and OEM-policy bounds apply.
    pub fn configure_realtek_alc255_headphone(
        &mut self,
        graph: &CodecGraph,
        route: &AnalogRoute,
        id: StreamId,
        channel: u8,
        options: PinOptions,
    ) -> Result<RealtekHeadphoneReport, Error> {
        self.minimum_guard()?;
        self.configure_realtek_depop_headphone(
            graph,
            route,
            id,
            channel,
            options,
            DepopProfile::Alc255,
        )
    }

    fn configure_realtek_depop_headphone(
        &mut self,
        graph: &CodecGraph,
        route: &AnalogRoute,
        id: StreamId,
        channel: u8,
        options: PinOptions,
        profile: DepopProfile,
    ) -> Result<RealtekHeadphoneReport, Error> {
        self.minimum_guard()?;
        self.realtek_common_guard(graph)?;
        let accepts = match profile {
            DepopProfile::Alc255 => graph.vendor_id == 0x10ec_0255,
            DepopProfile::Alc256And257 => matches!(graph.vendor_id, 0x10ec_0256 | 0x10ec_0257),
        };
        if !accepts {
            return Err(Error::UnsupportedTopology);
        }
        let pin_power = self.headphone_preflight(graph, route, id, channel, options)?;
        let (codec_id, revision_id, subsystem_id, vendor_power) =
            self.realtek_common_identity(graph)?;
        if self.realtek_parameter(graph.address, 0x51, 0)? != 0 {
            return Err(Error::UnsupportedTopology);
        }
        let result = (|| {
            self.power_d0(graph.address, 1)?;
            if vendor_power {
                self.power_d0(graph.address, 0x20)?;
            }
            if pin_power {
                self.power_d0(graph.address, route.pin())?;
            }
            let (coefficients, initial_sense) =
                self.alc256_init(graph, route.pin(), profile, true)?;
            // A required sense was read or failed above.
            let initial_sense = initial_sense.ok_or(Error::InvalidCodecGraph)?;
            self.configure_analog_route(graph, route, id, channel, options)?;
            Ok(RealtekHeadphoneReport {
                common: RealtekCommonReport {
                    codec_id,
                    revision_id,
                    subsystem_id,
                    coefficients,
                },
                headphone_pin: route.pin(),
                initial_sense,
            })
        })();
        if result.is_err() {
            self.routing_failed = true;
        }
        result
    }

    /// `alc_fill_eapd_coef`, then `alc256_init` on headphone `pin`, the AFG,
    /// vendor node and pin already in D0: EAPD by verb, a 30 ms settle, the
    /// jack's sense, its depop power-up where a jack is in, the 3k pull-down
    /// release, the analog-reset pulse and the 1Ah routing word. A `required`
    /// sense fails as its read does; otherwise a pin that is absent or cannot
    /// sense presence reads as `None`, an empty jack.
    fn alc256_init(
        &mut self,
        graph: &CodecGraph,
        pin: u8,
        profile: DepopProfile,
        required: bool,
    ) -> Result<(Vec<CoefficientChange>, Option<PinSense>), Error> {
        self.minimum_guard()?;
        let mut coefficients = Vec::new();
        if matches!(profile, DepopProfile::Alc256And257) {
            coefficients.push(self.clear_coefficient_bits(graph.address, 0x36, 1 << 13)?);
        }
        coefficients.push(self.clear_coefficient_bits(graph.address, 0x10, 1 << 9)?);
        self.delay(30_000)?;
        let sense = if !required && graph.widget(pin).is_none() {
            None
        } else {
            match self.poll_pin_sense(graph, pin) {
                Ok(sense) => Some(sense),
                Err(Error::UnsupportedTopology) if !required => None,
                Err(error) => return Err(error),
            }
        };
        if sense.is_some_and(|sense| sense.present) {
            self.delay(2_000)?;
            self.depop_control(graph.address, DepopControl::HeadphonePower, 1)?;
            self.route_control(graph.address, pin, 0x707, 0xf07, 0x40)?;
            self.delay(75_000)?;
            self.headphone_unmute_zero(graph.address, pin)?;
            self.delay(75_000)?;
            self.depop_control(graph.address, DepopControl::HeadphonePower, 4)?;
        }
        coefficients.push(self.clear_coefficient_bits(graph.address, 0x46, 3 << 12)?);
        self.depop_control(graph.address, DepopControl::AnalogReset, 0x8000)?;
        self.depop_control(graph.address, DepopControl::AnalogReset, 0)?;
        let previous = self.read_coefficient(graph.address, 0x36)?;
        coefficients.push(self.write_coefficient_observed(
            graph.address,
            0x36,
            previous,
            0x5757,
        )?);
        Ok((coefficients, sense))
    }

    /// Linux's ALC256 probe and init as the codec binds, before any route:
    /// `alc269_probe`'s `alc_pre_init` and `alc_init`'s `alc256_init` on
    /// `alc_get_hp_pin`'s pin, else 0x21 ([`Self::alc256_init`]), then
    /// `alc_auto_init_amp`'s EAPD on each listed pin that has it. Every pin
    /// written is first put in D0 where it has its own power state, as
    /// Linux holds an ALC269-family codec's widgets (`power_save_node` 0).
    /// The generic parser's path setup (pin controls, amplifiers and the
    /// speaker's automute) is a route's; no PCI- or codec-subsystem fixup,
    /// GPIO, ultra-low-power or headset mode is applied. A failure after the
    /// first write refuses routes until the controller is reset.
    pub fn initialize_realtek_alc256(
        &mut self,
        graph: &CodecGraph,
    ) -> Result<RealtekAlc256Report, Error> {
        self.minimum_guard()?;
        self.realtek_common_guard(graph)?;
        if graph.vendor_id != 0x10ec_0256 {
            return Err(Error::UnsupportedTopology);
        }
        let (codec_id, revision_id, subsystem_id, vendor_power) =
            self.realtek_common_identity(graph)?;
        // alc269_probe's ALC5505 DSP hook replaces alc256_init.
        if self.realtek_parameter(graph.address, 0x51, 0)? != 0 {
            return Err(Error::UnsupportedTopology);
        }
        let group = graph
            .groups
            .iter()
            .find(|group| group.node == 1 && group.function_type & 255 == 1)
            .ok_or(Error::InvalidCodecGraph)?;
        let (headphone, speaker_pin) = outputs(
            group
                .widgets
                .iter()
                .filter(|widget| widget.kind == WidgetKind::Pin)
                .filter_map(|widget| {
                    Some((
                        widget.node,
                        widget.pin_default?,
                        widget.pin_capabilities.unwrap_or(0),
                    ))
                }),
        );
        let headphone_pin = headphone.unwrap_or(FALLBACK_HEADPHONE);
        let eapd_pins: Vec<u8> = EAPD_PINS
            .into_iter()
            .filter(|&node| {
                graph.widget(node).is_some_and(|widget| {
                    widget.kind == WidgetKind::Pin
                        && widget
                            .pin_capabilities
                            .is_some_and(|caps| caps != u32::MAX && caps & PIN_EAPD != 0)
                })
            })
            .collect();
        let result = (|| {
            self.power_d0(graph.address, 1)?;
            if vendor_power {
                self.power_d0(graph.address, 0x20)?;
            }
            for node in core::iter::once(headphone_pin).chain(eapd_pins.iter().copied()) {
                if graph
                    .widget(node)
                    .is_some_and(|widget| widget.capabilities & WIDGET_POWER != 0)
                {
                    self.power_d0(graph.address, node)?;
                }
            }
            let (coefficients, headphone_sense) =
                self.alc256_init(graph, headphone_pin, DepopProfile::Alc256And257, false)?;
            for &pin in &eapd_pins {
                self.route_control(graph.address, pin, 0x70c, 0xf0c, 2)?;
            }
            Ok(RealtekAlc256Report {
                common: RealtekCommonReport {
                    codec_id,
                    revision_id,
                    subsystem_id,
                    coefficients,
                },
                headphone_pin,
                headphone_sense,
                speaker_pin,
                eapd_pins: eapd_pins.clone(),
            })
        })();
        if result.is_err() {
            self.routing_failed = true;
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::outputs;

    const OUT: u32 = 1 << 4;
    const IN: u32 = 1 << 5;

    /// An ALC256 laptop's usual defaults: a DMIC at 12h, the internal
    /// speaker at 14h, a headset mic at 19h, the PC beep at 1Dh and the
    /// headphone jack at 21h; unused pins have no connection.
    #[test]
    fn an_alc256_laptops_headphone_and_speaker() {
        let pins = [
            (0x12, 0x90a6_0130, IN),
            (0x13, 0x4000_0000, IN),
            (0x14, 0x9017_0110, OUT),
            (0x18, 0x4111_11f0, IN | OUT),
            (0x19, 0x03a1_1020, IN),
            (0x1a, 0x4111_11f0, IN | OUT),
            (0x1b, 0x4111_11f0, IN | OUT),
            (0x1d, 0x4066_1b45, IN),
            (0x1e, 0x4111_11f0, OUT),
            (0x21, 0x0321_1030, OUT),
        ];
        assert_eq!(outputs(pins), (Some(0x21), Some(0x14)));
    }

    #[test]
    fn a_fixed_line_out_is_a_speaker_and_sequence_orders_pins() {
        // Line out device, fixed: the BIOS workaround's speaker.
        assert_eq!(outputs([(0x17, 0x9010_0112, OUT)]), (None, Some(0x17)));
        assert_eq!(
            outputs([
                (0x14, 0x9017_0112, OUT),
                (0x17, 0x9017_0110, OUT),
                (0x15, 0x0221_1022, OUT),
                (0x21, 0x0221_1021, OUT),
            ]),
            (Some(0x21), Some(0x17))
        );
    }

    #[test]
    fn several_headphones_without_a_line_out_keep_the_real_one() {
        // Sequence 0x0f marks the real headphone; the other is a line out.
        assert_eq!(
            outputs([(0x15, 0x0221_1020, OUT), (0x21, 0x0221_102f, OUT)]),
            (Some(0x21), None)
        );
        // None is: all serve as line outs, and the first is the headphone.
        assert_eq!(
            outputs([(0x15, 0x0221_1021, OUT), (0x21, 0x0221_1020, OUT)]),
            (Some(0x21), None)
        );
    }

    #[test]
    fn pins_the_parser_skips_are_never_chosen() {
        for pins in [
            // The PC-beep pin, no connection, no output capability.
            [(0x1d, 0x0221_1020, OUT)],
            [(0x21, 0x4221_1020, OUT)],
            [(0x21, 0x0221_1020, IN)],
        ] {
            assert_eq!(outputs(pins), (None, None), "{pins:x?}");
        }
        // Capabilities that read zero pass, as some old hardware's do.
        assert_eq!(outputs([(0x21, 0x0221_1020, 0)]), (Some(0x21), None));
        // A headphone is no speaker.
        assert_eq!(outputs([(0x21, 0x0221_1020, OUT)]), (Some(0x21), None));
    }
}

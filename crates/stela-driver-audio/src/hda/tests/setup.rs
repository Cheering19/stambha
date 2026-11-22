// SPDX-License-Identifier: GPL-2.0-only
//! Bind-time codec setup selection.
use super::*;

#[test]
fn cirrus_identity_selects_its_vendor_setup() {
    let mut fixture = Fixture::new();
    let model = cirrus::cs4208_device(false, false);
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    let graph = controller.enumerate_codec(0).unwrap();
    let Ok(CodecSetup::Cirrus(report)) = controller.initialize_codec(&graph) else {
        panic!("CS4208 did not select the Cirrus setup");
    };
    assert_eq!(report.profile, CirrusProfile::Cs4208);
}

/// An ALC256 model as on an ALC256 laptop: the headphone jack the model's
/// node 4 has, and the board's internal speaker, a fixed pin at 0x14 with
/// EAPD, and the coefficients Linux's init reads.
fn alc256_laptop(present: bool) -> Rc<RefCell<Model>> {
    let model = realtek_headphone_model(0x10ec0256, present);
    {
        let mut m = model.borrow_mut();
        m.parameters.insert((0x14, 9), (4 << 20) | 1);
        m.parameters.insert((0x14, 0x0c), 0x10 | (1 << 16));
        m.pin_defaults.insert(0x14, 0x9017_0110);
        m.route_controls.insert((0x14, 0xf0c), 0);
        m.coefficients.insert((0x57, 4), 0xab07);
        m.coefficients.insert((0x53, 2), 0x1234);
        m.coefficients.insert((0x20, 0x36), 0xaaff);
        m.coefficients.insert((0x20, 0x46), 0xb5a5);
    }
    model
}

/// Linux's ALC256 probe and init (`alc269_probe`'s `alc_pre_init`, then
/// `alc_init`): `alc_fill_eapd_coef`, `alc256_init` on the headphone pin
/// the pin defaults name, then `alc_auto_init_amp`'s EAPD on the listed
/// pins, with no route reserved.
#[test]
fn alc256_binds_with_linux_init_verbs() {
    for present in [false, true] {
        let mut fixture = Fixture::new();
        let model = alc256_laptop(present);
        let mut controller = create(&model, &mut fixture, config()).unwrap();
        let graph = controller.enumerate_codec(0).unwrap();
        let start = model.borrow().clock.now_us();
        let Ok(CodecSetup::Alc256(report)) = controller.initialize_codec(&graph) else {
            panic!("ALC256 did not select its setup");
        };
        assert_eq!(report.common.codec_id, 0x10ec0256);
        assert_eq!(report.headphone_pin, 4);
        assert_eq!(
            report.headphone_sense.map(|sense| sense.present),
            Some(present)
        );
        assert_eq!(report.speaker_pin, Some(0x14));
        assert_eq!(report.eapd_pins, [0x14]);
        // The boot's HOST HD AUDIO line.
        assert_eq!(
            alloc::format!("{}", CodecSetup::Alc256(report.clone())),
            alloc::format!(
                "Realtek ALC256, headphone pin 0x04 ({}), speaker pin 0x14, EAPD on 0x14",
                if present { "plugged in" } else { "empty" }
            )
        );
        let m = model.borrow();
        // alc_fill_eapd_coef: index 36h bit 13, then index 10h bit 9.
        let first: Vec<_> = m
            .coefficient_events
            .iter()
            .filter(|(node, _, _, _)| *node == 0x20)
            .map(|(_, index, value, _)| (*index, *value))
            .take(2)
            .collect();
        assert_eq!(first, [(0x36, 0x8aff), (0x10, 0xa5ff)]);
        assert_eq!(m.coefficients[&(0x20, 0x36)], 0x5757);
        assert_eq!(m.coefficients[&(0x20, 0x46)], 0x85a5);
        assert_eq!(m.coefficients[&(0x20, 0x10)], 0xa5ff);
        assert_eq!(m.coefficients[&(0x53, 2)], 0x1234);
        assert_eq!(
            m.coefficients[&(0x57, 4)],
            if present { 0xab04 } else { 0xab07 }
        );
        let sense = m
            .command_times
            .iter()
            .find(|(command, _)| (command >> 20) & 127 == 4 && (command >> 8) & 0xfff == 0xf09)
            .unwrap()
            .1;
        assert!(sense >= start + 30_000);
        // The headphone pin is driven only where a jack is in.
        assert_eq!(
            m.route_controls.get(&(4, 0xf07)).copied(),
            present.then_some(0x40)
        );
        // EAPD on the speaker after alc256_init's last write.
        assert_eq!(m.route_controls[&(0x14, 0xf0c)], 2);
        let eapd = m
            .command_times
            .iter()
            .find(|(command, _)| (command >> 20) & 127 == 0x14 && (command >> 8) & 0xfff == 0x70c)
            .unwrap()
            .1;
        let last = m
            .coefficient_events
            .iter()
            .rfind(|(node, index, _, _)| *node == 0x20 && *index == 0x36)
            .unwrap();
        assert_eq!(last.2, 0x5757);
        assert!(eapd >= last.3);
        // The speaker's pin and amplifiers wait for a route.
        assert_eq!(m.route_controls.get(&(0x14, 0xf07)), None);
        drop(m);
        assert!(controller.routes.is_empty());
        // A later headphone route still starts as before.
        let route = graph
            .analog_endpoints(Direction::Playback, stereo())
            .unwrap()
            .into_iter()
            .find(|endpoint| endpoint.route.pin() == 4)
            .unwrap()
            .route;
        let stream = controller
            .allocate_stream(
                Direction::Playback,
                stereo(),
                128,
                2,
                &mut fixture.allocator,
            )
            .unwrap();
        controller
            .configure_realtek_alc256_headphone(
                &graph,
                &route,
                stream,
                0,
                PinOptions::Playback {
                    headphone: true,
                    eapd: Some(true),
                },
            )
            .unwrap();
        assert_eq!(controller.routes.len(), 1);
    }
}

/// Without a headphone in the pin defaults, `alc256_init` senses 0x21, as
/// Linux does; a codec without that pin skips the jack's power-up.
#[test]
fn alc256_without_a_listed_headphone_senses_its_fallback_pin() {
    let mut fixture = Fixture::new();
    let model = alc256_laptop(true);
    model.borrow_mut().pin_defaults.insert(4, 0x0101_4010);
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    let graph = controller.enumerate_codec(0).unwrap();
    let Ok(CodecSetup::Alc256(report)) = controller.initialize_codec(&graph) else {
        panic!("ALC256 did not select its setup");
    };
    assert_eq!(report.headphone_pin, 0x21);
    assert_eq!(report.headphone_sense, None);
    let m = model.borrow();
    assert!(
        m.coefficient_events
            .iter()
            .all(|(node, _, _, _)| *node != 0x57)
    );
    assert_eq!(m.coefficients[&(0x20, 0x36)], 0x5757);
    assert_eq!(m.route_controls[&(0x14, 0xf0c)], 2);
}

/// A refused init write leaves the codec needing a reset, as every
/// Realtek setup here does, and the log names it.
#[test]
fn alc256_failed_init_blocks_routes() {
    let mut fixture = Fixture::new();
    let model = alc256_laptop(false);
    model.borrow_mut().coefficient_read_fault = Some(u32::MAX);
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    let graph = controller.enumerate_codec(0).unwrap();
    assert!(controller.initialize_codec(&graph).is_err());
    assert!(controller.routing_failed);
}

#[test]
fn unlisted_codec_keeps_its_standard_widgets() {
    let mut fixture = Fixture::new();
    let model = analog_model();
    let mut controller = create(&model, &mut fixture, config()).unwrap();
    let graph = controller.enumerate_codec(0).unwrap();
    let commands = model.borrow().commands.len();
    assert_eq!(
        controller.initialize_codec(&graph),
        Ok(CodecSetup::Standard)
    );
    assert_eq!(model.borrow().commands.len(), commands);
}

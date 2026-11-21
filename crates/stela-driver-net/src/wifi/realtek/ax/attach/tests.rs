extern crate std;
use super::*;
use crate::Link;
use crate::wifi::intel::policy::tests::{Rule, regdb};
use crate::wifi::network::Secret;
use crate::wifi::realtek::ax::bringup::tests::{
    BOTH, Chip, MAC, NO_LTR, assert_ends_with_power_off, chip, efuse_with_country, efuse_with_mac,
    never, tables_file,
};
use crate::wifi::realtek::ax::fwdl::tests::image_bytes;
use crate::wifi::realtek::ax::regd::{Regd, Regds};
use crate::wifi::realtek::ax::station::WIFI_CARRIER_WAIT_US;
use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
use crate::wifi::realtek::beacon::tests::{Domain, Source};
use crate::wifi::realtek::firmware::{Ce, Rtw89};
use crate::wifi::regulatory::database::Error as DatabaseError;
use alloc::{boxed::Box, string::ToString};
use stela_driver_base::entropy::EntropyError;
use zeroize::Zeroizing;

const CLOCK: Clock = Clock {
    ticks_per_microsecond: 1,
};
/// 2.4 GHz channels 1-13, the one band every rule set here allows.
const G2: [Rule; 1] = [(2_402_000, 2_482_000, 0, 2000)];

struct NoEntropy;

impl Entropy for NoEntropy {
    fn fill(&self, _: &mut [u8]) -> Result<(), EntropyError> {
        Err(EntropyError::Unavailable)
    }
}

/// The plan's test network, in `country`.
fn network(country: Option<[u8; 2]>) -> Network {
    Network {
        ssid: b"test".to_vec(),
        secret: Secret::Psk(Zeroizing::new([0; 32])),
        country,
    }
}

type Attached =
    Result<Wifi<Live<RegisterModel>>, (AttachError, Option<(RegisterModel, Option<Rings>)>)>;

/// [`attach`] on `c`'s chip, cut 2, without LTR, with the elements of
/// [`tables_file`], a database of `countries` and the network country
/// `country`.
#[expect(
    clippy::result_large_err,
    reason = "attach's own result, the window and the rings handed back by value"
)]
fn attach_on(c: Chip, countries: &[([u8; 2], &[Rule])], country: Option<[u8; 2]>) -> Attached {
    let Chip { io, m, .. } = c;
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    let file = tables_file();
    let firmware = FirmwareFile {
        image: &fw.image,
        file: &file,
    };
    let inputs = Inputs {
        clock: CLOCK,
        database: regdb(countries),
        network: network(country),
        entropy: Arc::new(NoEntropy),
    };
    // SAFETY: the model domain isolates only the modeled requester behind
    // this test's `RegisterModel`; its pages are plain kept test RAM that no
    // real device can reach, and this test is the only worker.
    unsafe {
        attach(
            io,
            &Source,
            Arc::new(Domain(m)),
            2,
            firmware,
            NO_LTR,
            inputs,
        )
    }
}

/// The failure of an attach that must fail.
fn refused(attached: Attached) -> (AttachError, Option<(RegisterModel, Option<Rings>)>) {
    match attached {
        Ok(_) => panic!("attached"),
        Err(refused) => refused,
    }
}

#[test]
fn attach_refuses_an_unknown_country() {
    // The network's country, else the world's `00`, must be in
    // regulatory.db, as rtw88's attach requires; nothing is touched first.
    for (country, want) in [(Some(*b"ZZ"), *b"ZZ"), (None, *b"00")] {
        let (error, back) = refused(attach_on(
            chip(efuse_with_mac(), 7, never, BOTH),
            &[(*b"US", &G2[..])],
            country,
        ));
        assert!(
            matches!(error, AttachError::Country(alpha2, DatabaseError::Country) if alpha2 == want),
            "{error:?}"
        );
        assert_eq!(
            error.to_string(),
            alloc::format!(
                "RTL8852CE: regulatory.db country {}: Country",
                core::str::from_utf8(&want).unwrap()
            )
        );
        let (io, rings) = back.expect("the window comes back");
        assert!(rings.is_none());
        assert!(
            io.trace.is_empty(),
            "{:?}",
            &io.trace[..4.min(io.trace.len())]
        );
    }
}

#[test]
fn attach_starts_and_keeps() {
    let attached = attach_on(
        chip(efuse_with_mac(), 7, never, BOTH),
        &[(*b"US", &G2[..])],
        Some(*b"US"),
    );
    let mut wifi = attached.unwrap_or_else(|(error, _)| panic!("{error}"));
    // The keeper's radio is the efuse's address, and its progress line
    // has reached nothing yet.
    assert_eq!(wifi.mac(), Mac(MAC));
    let progress = wifi.diagnostic().unwrap();
    assert!(
        progress.starts_with("scans 0 (0 networks, target -), auth -, assoc -, eapol -"),
        "{progress}"
    );
    assert_eq!(wifi.carrier_wait_us(), WIFI_CARRIER_WAIT_US);
    // The radio hands back the live chip it was given.
    let live = wifi.into_radio().into_chip();
    assert_eq!(live.mac(), MAC);
    assert!(!live.scanning() && live.bssid().is_none());
}

#[test]
fn attach_reporting_hands_back_what_the_bring_up_reported() {
    // The keeper keeps its radio to itself, so the bring-up's report comes
    // back beside it, as the live chip holds it.
    let Chip { io, m, .. } = chip(efuse_with_mac(), 7, never, BOTH);
    let bytes = image_bytes();
    let fw = Rtw89::parse(&bytes, 2, Ce::Never).unwrap();
    let file = tables_file();
    let inputs = Inputs {
        clock: CLOCK,
        database: regdb(&[(*b"US", &G2[..])]),
        network: network(Some(*b"US")),
        entropy: Arc::new(NoEntropy),
    };
    // SAFETY: as in `attach_on`.
    let reported = unsafe {
        attach_reporting(
            io,
            &Source,
            Arc::new(Domain(m)),
            2,
            FirmwareFile {
                image: &fw.image,
                file: &file,
            },
            NO_LTR,
            inputs,
        )
    };
    let (wifi, running) = reported.unwrap_or_else(|(error, _)| panic!("{error}"));
    let live = wifi.into_radio().into_chip();
    assert_eq!(running, *live.running());
    assert_eq!(running.mac, MAC);
    assert!(running.mac_init);
}

#[test]
fn a_missing_regulatory_database_is_named_as_rtw88_names_it() {
    // The probe's refusal, before anything is touched.
    assert_eq!(
        AttachError::NoRegulatoryDatabase.to_string(),
        "RTL8852CE: no regulatory database"
    );
}

#[test]
fn channel_rules_follow_the_network_and_tx_power_the_efuse() {
    // regulatory.db has only DE, so the channel rules can only be the
    // network's. The TX power is the efuse's US (FCC), which the firmware's
    // REGD map holds; with an efuse country the map lacks (JP), it falls to
    // the network's DE (ETSI).
    let fcc = Regds {
        g2: Regd::Fcc,
        g5: Regd::Fcc,
    };
    let etsi = Regds {
        g2: Regd::Etsi,
        g5: Regd::Etsi,
    };
    for (efuse, want) in [(*b"US", (fcc, *b"US")), (*b"JP", (etsi, *b"DE"))] {
        let attached = attach_on(
            chip(efuse_with_country(efuse), 7, never, BOTH),
            &[(*b"DE", &G2[..])],
            Some(*b"DE"),
        );
        let wifi = attached.unwrap_or_else(|(error, _)| panic!("{error}"));
        assert_eq!(wifi.into_radio().into_chip().regds(), want);
    }
}

#[test]
fn an_address_no_station_can_have_stops_the_chip() {
    // A group address in the efuse: the chip starts, then is stopped
    // before a radio that could not be made would take it.
    let mut phys = efuse_with_mac();
    phys[6] |= 0x01;
    let (error, back) = refused(attach_on(
        chip(phys, 7, never, BOTH),
        &[(*b"US", &G2[..])],
        Some(*b"US"),
    ));
    assert!(
        matches!(
            error,
            AttachError::Station(station::Error::Station(
                crate::wifi::station::Error::InvalidAddress
            ))
        ),
        "{error:?}"
    );
    assert!(
        error.to_string().starts_with("RTL8852CE station: "),
        "{error}"
    );
    let (io, rings) = back.expect("the window comes back");
    assert!(rings.is_some());
    assert_ends_with_power_off(&io);
}

#[test]
fn a_failed_start_hands_back_the_window_and_the_rings() {
    // SWSI stays busy from help_enter's TX scheduler read on, so the RF
    // part of set channel times out after the rings were installed.
    let mut c = chip(efuse_with_mac(), 7, never, BOTH);
    c.io.on_read(
        0xC398,
        Box::new(|m: &mut RegisterModel| {
            if m.trace.contains(&Op::W32(0x14978, 3)) {
                m.swsi_status = 0x0300_0000;
            }
            m.get32(0xC398)
        }),
    );
    let (error, back) = refused(attach_on(c, &[(*b"US", &G2[..])], Some(*b"US")));
    let AttachError::Start(failed) = &error else {
        panic!("{error:?}");
    };
    assert_eq!(failed.step, "set channel");
    assert!(
        error
            .to_string()
            .starts_with("RTL8852CE bring-up: set channel: "),
        "{error}"
    );
    let (io, rings) = back.expect("the window comes back");
    assert!(rings.is_some());
    assert_ends_with_power_off(&io);
}

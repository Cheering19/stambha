//! The AP MLD links an association requests, as `ieee80211_mgd_assoc` and
//! `mt76_select_links` pick them.
use super::*;
use crate::wifi::station::tests::{beacon, join};

const AP_MLD: Mac = Mac([2, 9, 9, 9, 9, 9]);
const CHANNEL_1: Channel = Channel {
    band: Band::Ghz2,
    number: 1,
    frequency_mhz: 2412,
};
const CHANNEL_44: Channel = Channel {
    band: Band::Ghz5,
    number: 44,
    frequency_mhz: 5220,
};

/// The AP of link `link_id` of `mld` on `channel`, its RNR naming `reported`
/// APs of its own AP MLD: link ID, last BSSID byte and channel number each.
fn ap(link_id: u8, last: u8, channel: Channel, mld: Mac, reported: &[(u8, u8, u8)]) -> Bss {
    let mut elements = Basic {
        address: mld,
        link_id: Some(link_id),
        ..Basic::default()
    }
    .encode();
    for &(link_id, last, number) in reported {
        // One ieee80211_tbtt_info_ge_11 with MLD parameters, AP MLD ID 0.
        let mut rnr = vec![
            0x00, 16, 0, number, 0, 2, 0, 0, 0, 0, last, 0, 0, 0, 0, 0x40, 0, 0,
        ];
        rnr.extend_from_slice(&u16::from(link_id).to_le_bytes());
        elements.extend_from_slice(&[201, rnr.len() as u8]);
        elements.extend_from_slice(&rnr);
    }
    Bss {
        bssid: Mac([2, 0, 0, 0, 0, last]),
        channel,
        elements,
        ..beacon(&join(PortPolicy::Controlled))
    }
}

#[test]
fn an_ap_mld_names_itself_and_the_link_its_bss_is() {
    let setup = ap(0, 1, CHANNEL_44, AP_MLD, &[]);
    assert_eq!(ap_mld(&setup), Some((AP_MLD, 0)));
    let mut single = setup.clone();
    single.elements.clear();
    assert_eq!(ap_mld(&single), None);
}

/// mt76_select_links: the setup link and the first other link by ID on
/// another band, which the scan observed as a link of the same AP MLD.
#[test]
fn the_other_link_is_the_first_observed_on_another_band() {
    let setup = ap(
        0,
        1,
        CHANNEL_44,
        AP_MLD,
        &[(3, 3, 1), (2, 2, 1), (1, 4, 48)],
    );
    // Link 1 shares the setup link's band; link 3 comes after link 2.
    let same_band = ap(
        1,
        4,
        Channel {
            number: 48,
            frequency_mhz: 5240,
            ..CHANNEL_44
        },
        AP_MLD,
        &[],
    );
    let two = ap(2, 2, CHANNEL_1, AP_MLD, &[]);
    let three = ap(3, 3, CHANNEL_1, AP_MLD, &[]);
    let observed = [(10, &same_band), (30, &three), (20, &two)];
    let any = |_: &Bss| true;
    assert_eq!(
        other_link(&setup, AP_MLD, &observed, any).map(|(id, handle, _)| (id, handle)),
        Some((2, 20))
    );
    // A link the station may not use passes to the next.
    let not_two = |other: &Bss| other.bssid != two.bssid;
    assert_eq!(
        other_link(&setup, AP_MLD, &observed, not_two).map(|(id, handle, _)| (id, handle)),
        Some((3, 30))
    );
    // An AP the RNR names as another AP MLD's, or unobserved, is no link.
    let foreign = ap(2, 2, CHANNEL_1, Mac([2, 8, 8, 8, 8, 8]), &[]);
    assert!(other_link(&setup, AP_MLD, &[(20, &foreign)], any).is_none());
    assert!(other_link(&setup, AP_MLD, &[(10, &same_band)], any).is_none());
}

/// wpa_supplicant's `sm->mlo` link: the AP's whole RSN element and RSNXE.
#[test]
fn a_keyed_link_carries_its_aps_whole_rsn_element_and_rsnxe() {
    let mut bss = ap(1, 2, CHANNEL_1, AP_MLD, &[]);
    bss.elements = vec![48, 2, 1, 0, 244, 1, 0x20];
    let link = keyed_link(1, Mac([2, 0xa, 0, 0, 0, 1]), &bss);
    assert_eq!(
        (link.ap, link.rsn, link.rsnxe, link.accepted),
        (
            bss.bssid,
            vec![48, 2, 1, 0],
            Some(vec![244, 1, 0x20]),
            false
        )
    );
}

/// Drive the generic keeper over a vendor's owned model while the last
/// handshake reply is in flight. The caller observes the real native key and
/// supplies its descriptor/TX-free/IRQ/work retirement, rather than a counter
/// that pretends to be a drain.
pub(crate) fn assert_secondary_key_waits_for_retirement<R: Radio>(
    mut keeper: Keeper<R>,
    ptk: KeyIdentity,
    management: R::Management,
    key: IntegrityKey,
    retire: impl FnOnce(&mut R),
    mut has_key: impl FnMut(&mut R) -> bool,
) -> Keeper<R> {
    let supplicant = Supplicant::new(
        supplicant::Suites {
            akm: Akm::Psk,
            pairwise: crate::wifi::security::Cipher::Ccmp128,
            group: crate::wifi::security::Cipher::Ccmp128,
            management: Some(crate::wifi::security::management::ManagementCipher::Cmac128),
        },
        Zeroizing::new([0; 32]),
        keeper.link.mac(),
        AP_MLD,
        vec![],
        vec![],
    )
    .unwrap();
    keeper.state = State::Handshake {
        supplicant: Box::new(supplicant),
        bssid: AP_MLD,
        deadline: u64::MAX,
        mic_failed: false,
        ptk: Some(ptk),
        integrity: None,
        link_integrity: vec![(1, key)],
        management: Some(management),
    };
    keeper.complete().unwrap();
    assert!(
        matches!(&keeper.state, State::Handshake { link_integrity, .. } if link_integrity.len()==1)
    );
    assert!(!has_key(&mut keeper.link));
    assert!(!keeper.link.carrier());
    retire(&mut keeper.link);
    keeper.complete().unwrap();
    assert!(matches!(keeper.state, State::Connected { .. }));
    assert!(has_key(&mut keeper.link));
    assert!(keeper.link.carrier());
    keeper
}

/// A radio that keeps the [`Link`] defaults, as the wired links do.
struct Wired {
    mac: Mac,
}

/// A radio whose link names its own carrier wait and state.
struct Tuned {
    mac: Mac,
}

impl Link for Wired {
    fn mac(&self) -> Mac {
        self.mac
    }
    fn transmit(&mut self, _frame: &[u8]) -> Result<(), LinkError> {
        Ok(())
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        Ok(None)
    }
}

impl Link for Tuned {
    fn mac(&self) -> Mac {
        self.mac
    }
    fn carrier_wait_us(&self) -> u64 {
        30_000_000
    }
    fn transmit(&mut self, _frame: &[u8]) -> Result<(), LinkError> {
        Ok(())
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        Ok(None)
    }
    fn diagnostic(&mut self) -> Option<String> {
        Some(String::from("x"))
    }
}

/// A [`Radio`] the keeper is only built over: none of its operations run.
macro_rules! inert_radio {
    ($radio:ty) => {
        impl Radio for $radio {
            type Handle = ();
            type Profile = ();
            type Management = ();
            type Error = &'static str;
            const NAME: &'static str = "inert";
            const LISTEN_INTERVAL: u16 = 1;
            fn deferred(_: &Self::Error) -> bool {
                false
            }
            fn service(&mut self) -> Result<(), Self::Error> {
                Ok(())
            }
            fn take_event(&mut self) -> Option<Event> {
                None
            }
            fn stage(&self) -> Stage {
                Stage::Idle
            }
            fn station(&self) -> &Station {
                unreachable!()
            }
            fn channels(&self, _: Band) -> Vec<(Channel, u32)> {
                Vec::new()
            }
            fn probes(&self, _: Channel) -> bool {
                false
            }
            fn supported_channels(&self, _: Band) -> &'static [u8] {
                &[]
            }
            fn regulated_power_dbm(&self, _: u16) -> i32 {
                0
            }
            fn advertisement(&self, _: Band) -> Result<Advertisement, Self::Error> {
                Err("inert")
            }
            fn profile(&self, _: Peer) -> Result<(), Self::Error> {
                Err("inert")
            }
            fn begin_scan(&mut self, _: scan::Request, _: u64) -> Result<(), Self::Error> {
                Err("inert")
            }
            unsafe fn retire_scan(&mut self) -> Result<(), Self::Error> {
                Err("inert")
            }
            fn six_ghz_scan(
                &self,
                _: &scan::Request,
                _: scan::six_ghz::Options,
            ) -> Result<Option<scan::Request>, Self::Error> {
                Err("inert")
            }
            fn bss(&self) -> Vec<((), &Bss)> {
                Vec::new()
            }
            fn observation(&self, _: ()) -> Result<Arc<NativeBssObservation>, Self::Error> {
                Err("inert")
            }
            fn join(&mut self, _: (), _: Join, _: ()) -> Result<(), Self::Error> {
                Err("inert")
            }
            fn begin_external_authentication(
                &mut self,
                _: (),
                _: (),
                _: Option<Affiliation>,
                _: u64,
            ) -> Result<Arc<external::Session>, Self::Error> {
                Err("inert")
            }
            fn begin_six_ghz_external_authentication(
                &mut self,
                _: (),
                _: Selection,
                _: (),
                _: Option<Affiliation>,
                _: u64,
            ) -> Result<Arc<external::Session>, Self::Error> {
                Err("inert")
            }
            fn transmit_external_authentication(
                &mut self,
                _: &external::Session,
                _: external::Frame<'_>,
            ) -> Result<(), Self::Error> {
                Err("inert")
            }
            fn associate_external(
                &mut self,
                _: &external::Session,
                _: (),
                _: Join,
            ) -> Result<(), Self::Error> {
                Err("inert")
            }
            fn configure(&mut self) -> Result<(), Self::Error> {
                Err("inert")
            }
            fn send_control(
                &mut self,
                _: &[u8],
                _: Option<KeyIdentity>,
            ) -> Result<(), Self::Error> {
                Err("inert")
            }
            fn install_key(
                &mut self,
                _: bool,
                _: u8,
                _: &[u8],
                _: ReplaySeed,
            ) -> Result<KeyIdentity, Self::Error> {
                Err("inert")
            }
            fn install_link_key(
                &mut self,
                _: u8,
                _: u8,
                _: &[u8],
                _: ReplaySeed,
            ) -> Result<(), Self::Error> {
                Err("inert")
            }
            unsafe fn install_management_key(
                &mut self,
                _: u64,
                _: u8,
                _: &[u8],
                _: PacketNumber,
            ) -> Result<(), Self::Error> {
                Err("inert")
            }
            unsafe fn rotate_management_key(
                &mut self,
                _: &(),
                _: u8,
                _: &[u8],
                _: PacketNumber,
            ) -> Result<(), Self::Error> {
                Err("inert")
            }
            fn authorize_port(&mut self, _: u64) -> Result<(), Self::Error> {
                Err("inert")
            }
            fn deauthenticate(&mut self, _: u16) -> Result<(), Self::Error> {
                Err("inert")
            }
            unsafe fn retire_peer(&mut self) -> Result<(), Self::Error> {
                Err("inert")
            }
        }
    };
}

inert_radio!(Wired);
inert_radio!(Tuned);

struct NoEntropy;

impl Entropy for NoEntropy {
    fn fill(&self, bytes: &mut [u8]) -> Result<(), stela_driver_base::entropy::EntropyError> {
        bytes.fill(0);
        Ok(())
    }
}

const LOCAL: Mac = Mac([2, 0x11, 0x22, 0x33, 0x44, 0x55]);

fn kept<R: Radio>(radio: R) -> Keeper<R> {
    let network = Network {
        ssid: b"test".to_vec(),
        secret: Secret::Psk(Zeroizing::new([0; 32])),
        country: None,
    };
    let clock = Clock {
        ticks_per_microsecond: 1,
    };
    Keeper::keep(radio, network, Arc::new(NoEntropy), clock)
}

#[test]
fn keeper_forwards_the_radios_carrier_wait() {
    assert_eq!(kept(Tuned { mac: LOCAL }).carrier_wait_us(), 30_000_000);
}

#[test]
fn keeper_forwards_the_radios_diagnostic() {
    assert_eq!(
        kept(Tuned { mac: LOCAL }).diagnostic().as_deref(),
        Some("x")
    );
}

#[test]
fn other_keepers_keep_the_wired_carrier_wait() {
    let mut keeper = kept(Wired { mac: LOCAL });
    assert_eq!(keeper.carrier_wait_us(), crate::WIRED_CARRIER_WAIT_US);
    assert_eq!(keeper.diagnostic(), None);
}

#[test]
fn into_radio_returns_the_radio() {
    let radio = kept(Tuned { mac: LOCAL }).into_radio();
    assert_eq!(radio.mac, LOCAL);
}

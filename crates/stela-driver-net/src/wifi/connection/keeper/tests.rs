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
        alternates: Default::default(),
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

/// A radio that names no wait of its own waits a scan, a join and the
/// handshake, not a wired PHY's 10 s.
#[test]
fn other_keepers_wait_for_a_scan_a_join_and_the_handshake() {
    let mut keeper = kept(Wired { mac: LOCAL });
    assert_eq!(keeper.carrier_wait_us(), super::CARRIER_WAIT_US);
    assert_ne!(keeper.carrier_wait_us(), crate::WIRED_CARRIER_WAIT_US);
    assert_eq!(keeper.diagnostic(), None);
}

#[test]
fn into_radio_returns_the_radio() {
    let radio = kept(Tuned { mac: LOCAL }).into_radio();
    assert_eq!(radio.mac, LOCAL);
}

/// The preferred network wins over any alternate, however weak; among
/// alternates, and among one network's BSSes, the stronger signal wins.
#[test]
fn the_preferred_network_outranks_alternates_and_signal_ranks_the_rest() {
    use super::outranks;
    assert!(outranks((0, -85), (2, -40)));
    assert!(!outranks((2, -40), (0, -85)));
    assert!(outranks((1, -50), (2, -60)));
    assert!(outranks((2, -50), (1, -60)));
    assert!(outranks((0, -50), (0, -60)));
    assert!(!outranks((1, -60), (1, -60)));
}

/// A keeper starts on the preferred network and keeps each alternate once,
/// one that repeats the preferred network's name left out.
#[test]
fn a_keeper_keeps_the_preferred_network_first_and_each_alternate_once() {
    let network = Network {
        ssid: b"home".to_vec(),
        secret: Secret::Psk(Zeroizing::new([1; 32])),
        country: Some(*b"IN"),
        alternates: vec![
            crate::wifi::network::Alternate {
                ssid: b"phone".to_vec(),
                secret: Secret::Psk(Zeroizing::new([2; 32])),
            },
            crate::wifi::network::Alternate {
                ssid: b"home".to_vec(),
                secret: Secret::Psk(Zeroizing::new([3; 32])),
            },
        ],
    };
    let clock = Clock {
        ticks_per_microsecond: 1,
    };
    let keeper = Keeper::keep(Tuned { mac: LOCAL }, network, Arc::new(NoEntropy), clock);
    let names: Vec<&[u8]> = keeper
        .candidates
        .iter()
        .map(|(network, _)| &network.ssid[..])
        .collect();
    assert_eq!(names, [&b"home"[..], &b"phone"[..]]);
    assert_eq!(*keeper.candidates[1].1, [2; 32]);
    assert_eq!(keeper.candidates[1].0.country, Some(*b"IN"));
    assert_eq!(keeper.network.ssid, b"home");
    assert!(keeper.network.alternates.is_empty());
}

/// A radio that is lost until its scripted recovery steps run out, each
/// step advancing the model clock by its own cost.
struct Recovering {
    stage: Stage,
    /// Each attempt's steps: the cost of each step, then how it ends.
    attempts: VecDeque<(Vec<u64>, Result<(), String>)>,
    steps: VecDeque<u64>,
    end: Option<Result<(), String>>,
    calls: Vec<u64>,
    recoverable: bool,
}

impl Recovering {
    fn new(recoverable: bool, attempts: Vec<(Vec<u64>, Result<(), String>)>) -> Self {
        Self {
            stage: Stage::Lost,
            attempts: attempts.into(),
            steps: VecDeque::new(),
            end: None,
            calls: Vec::new(),
            recoverable,
        }
    }
}

impl Link for Recovering {
    fn mac(&self) -> Mac {
        Mac([2, 0, 0, 0, 0, 1])
    }
    fn carrier(&mut self) -> bool {
        false
    }
    fn transmit(&mut self, _: &[u8]) -> Result<(), LinkError> {
        Err(LinkError::Adapter(String::from("lost")))
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        match self.stage {
            Stage::Lost => Err(LinkError::Adapter(String::from("lost"))),
            _ => Ok(None),
        }
    }
}

impl Radio for Recovering {
    type Handle = u8;
    type Profile = ();
    type Management = ();
    type Error = String;
    const NAME: &'static str = "model";
    const LISTEN_INTERVAL: u16 = 1;
    fn deferred(_: &String) -> bool {
        false
    }
    fn service(&mut self) -> Result<(), String> {
        Ok(())
    }
    fn recoverable(&self) -> bool {
        self.recoverable
    }
    fn recover(&mut self) -> Result<Recovery, String> {
        let start = crate::Clock::ticks();
        self.calls.push(start);
        if self.end.is_none() {
            let (steps, end) = self.attempts.pop_front().expect("no attempt left");
            self.steps = steps.into();
            self.end = Some(end);
        }
        if let Some(cost) = self.steps.pop_front() {
            crate::test_clock::advance(MODEL, cost);
        }
        let result = if self.steps.is_empty() {
            match self.end.take().unwrap() {
                Ok(()) => {
                    self.stage = Stage::Idle;
                    Ok(Recovery::Done)
                }
                Err(error) => Err(error),
            }
        } else {
            Ok(Recovery::Pending)
        };
        assert!(crate::Clock::ticks() - start <= RECOVERY_STEP_US);
        result
    }
    fn take_event(&mut self) -> Option<Event> {
        None
    }
    fn stage(&self) -> Stage {
        self.stage
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
    fn advertisement(&self, _: Band) -> Result<Advertisement, String> {
        unreachable!()
    }
    fn profile(&self, _: Peer) -> Result<(), String> {
        unreachable!()
    }
    fn begin_scan(&mut self, _: scan::Request, _: u64) -> Result<(), String> {
        unreachable!()
    }
    unsafe fn retire_scan(&mut self) -> Result<(), String> {
        unreachable!()
    }
    fn six_ghz_scan(
        &self,
        _: &scan::Request,
        _: scan::six_ghz::Options,
    ) -> Result<Option<scan::Request>, String> {
        unreachable!()
    }
    fn bss(&self) -> Vec<(u8, &Bss)> {
        Vec::new()
    }
    fn observation(&self, _: u8) -> Result<Arc<NativeBssObservation>, String> {
        unreachable!()
    }
    fn join(&mut self, _: u8, _: Join, _: ()) -> Result<(), String> {
        unreachable!()
    }
    fn begin_external_authentication(
        &mut self,
        _: u8,
        _: (),
        _: Option<Affiliation>,
        _: u64,
    ) -> Result<Arc<external::Session>, String> {
        unreachable!()
    }
    fn begin_six_ghz_external_authentication(
        &mut self,
        _: u8,
        _: Selection,
        _: (),
        _: Option<Affiliation>,
        _: u64,
    ) -> Result<Arc<external::Session>, String> {
        unreachable!()
    }
    fn transmit_external_authentication(
        &mut self,
        _: &external::Session,
        _: external::Frame<'_>,
    ) -> Result<(), String> {
        unreachable!()
    }
    fn associate_external(&mut self, _: &external::Session, _: u8, _: Join) -> Result<(), String> {
        unreachable!()
    }
    fn configure(&mut self) -> Result<(), String> {
        unreachable!()
    }
    fn send_control(&mut self, _: &[u8], _: Option<KeyIdentity>) -> Result<(), String> {
        unreachable!()
    }
    fn install_key(
        &mut self,
        _: bool,
        _: u8,
        _: &[u8],
        _: ReplaySeed,
    ) -> Result<KeyIdentity, String> {
        unreachable!()
    }
    fn install_link_key(&mut self, _: u8, _: u8, _: &[u8], _: ReplaySeed) -> Result<(), String> {
        unreachable!()
    }
    unsafe fn install_management_key(
        &mut self,
        _: u64,
        _: u8,
        _: &[u8],
        _: PacketNumber,
    ) -> Result<(), String> {
        unreachable!()
    }
    unsafe fn rotate_management_key(
        &mut self,
        _: &(),
        _: u8,
        _: &[u8],
        _: PacketNumber,
    ) -> Result<(), String> {
        unreachable!()
    }
    fn authorize_port(&mut self, _: u64) -> Result<(), String> {
        unreachable!()
    }
    fn deauthenticate(&mut self, _: u16) -> Result<(), String> {
        unreachable!()
    }
    unsafe fn retire_peer(&mut self) -> Result<(), String> {
        unreachable!()
    }
}

const MODEL: Clock = Clock {
    ticks_per_microsecond: 1,
};

fn keeper(radio: Recovering) -> Keeper<Recovering> {
    core::mem::forget(crate::test_clock::virtualize());
    struct Zero;
    impl Entropy for Zero {
        fn fill(&self, output: &mut [u8]) -> Result<(), stela_driver_base::entropy::EntropyError> {
            output.fill(0);
            Ok(())
        }
    }
    let network = Network {
        alternates: Default::default(),
        ssid: b"x".to_vec(),
        secret: Secret::parse(b"correct horse battery").unwrap(),
        country: None,
    };
    Keeper::keep(radio, network, Arc::new(Zero), MODEL)
}

fn notices(keeper: &mut Keeper<Recovering>) -> Vec<String> {
    core::iter::from_fn(|| keeper.take_diagnostic()).collect()
}

/// mt7925_mac_reset_work: the chip reset runs in bounded steps, one per
/// poll and with no pause between them, and the keeper joins again.
#[test]
fn a_lost_radio_recovers_in_bounded_steps_and_joins_again() {
    let steps = vec![
        RUNTIME_COMMAND_TIMEOUT_US,
        10_000,
        RUNTIME_COMMAND_TIMEOUT_US,
        1_000,
    ];
    let mut keeper = keeper(Recovering::new(true, vec![(steps, Ok(()))]));
    keeper.last_error = Some(String::from("model: command timed out"));
    // Each receive polls once: one step each, with no pause between them.
    for step in 1..=3 {
        assert_eq!(keeper.receive(), Ok(None));
        assert_eq!(keeper.radio().calls.len(), step);
        assert!(keeper.recovering());
        assert_eq!(keeper.transmit(&[0; 60]), Err(LinkError::NoCarrier));
        assert!(!keeper.signals());
    }
    assert!(!keeper.carrier());
    assert_eq!(keeper.radio().calls.len(), 4);
    assert!(!keeper.recovering());
    assert!(keeper.lost().is_none());
    assert!(matches!(keeper.state, State::Waiting { .. }));
    assert_eq!(
        notices(&mut keeper),
        [
            "model: command timed out",
            "model: recovered by a chip reset; joining again"
        ]
    );
}

/// A failed attempt waits a second before the next; ten in a row lose the
/// radio for good, reported once.
#[test]
fn ten_failed_recoveries_in_a_row_lose_the_radio_for_good() {
    let failed = |n: u32| (vec![1_000, 1_000], Err(format!("reset {n} failed")));
    let mut keeper = keeper(Recovering::new(true, (1..=10).map(failed).collect()));
    while keeper.lost().is_none() {
        keeper.poll();
        crate::test_clock::advance(MODEL, 100_000);
    }
    let calls = &keeper.radio().calls;
    assert_eq!(calls.len(), 20);
    for attempt in calls.chunks(2).collect::<Vec<_>>().windows(2) {
        // The second step follows on the next poll; the next attempt waits.
        assert!(attempt[0][1] - attempt[0][0] < RECOVERY_RETRY_US);
        assert!(attempt[1][0] - attempt[0][1] >= RECOVERY_RETRY_US);
    }
    let lost = String::from(keeper.lost().unwrap());
    assert_eq!(lost, "model: chip reset 10 of 10 failed: reset 10 failed");
    let reported = notices(&mut keeper);
    assert_eq!(reported.len(), 10);
    assert_eq!(reported[9], lost);
    assert_eq!(keeper.receive(), Err(LinkError::Adapter(lost.clone())));
    assert!(keeper.radio().attempts.is_empty());
    assert_eq!(notices(&mut keeper), Vec::<String>::new());
}

/// A failure that a later attempt cures resets the count: a radio that
/// cannot recover is lost at once.
#[test]
fn a_recovery_resets_the_failure_count_and_an_unrecoverable_radio_is_lost_at_once() {
    let attempts = vec![
        (vec![1_000], Err(String::from("first"))),
        (vec![1_000], Ok(())),
    ];
    let mut keeper = keeper(Recovering::new(true, attempts));
    keeper.poll();
    assert_eq!(keeper.recovery_failures, 1);
    crate::test_clock::advance(MODEL, RECOVERY_RETRY_US);
    keeper.poll();
    assert_eq!(keeper.recovery_failures, 0);
    assert!(keeper.lost().is_none());

    let mut keeper = keeper_lost();
    keeper.poll();
    assert_eq!(keeper.lost(), Some("model: the radio stopped for good"));
    assert!(keeper.radio().calls.is_empty());
}

fn keeper_lost() -> Keeper<Recovering> {
    keeper(Recovering::new(false, Vec::new()))
}

/// pmksa_cache_add and pmksa_cache_get: an SAE exchange's PMKSA per AP,
/// replaced by a newer one, gone after its 12 hours or a failed resumption.
#[test]
fn a_cached_pmksa_lasts_its_lifetime_and_goes_when_its_resumption_fails() {
    let mut radio = Recovering::new(false, Vec::new());
    radio.stage = Stage::Idle;
    let mut keeper = keeper(radio);
    let ap = Mac([2, 0, 0, 0, 0, 7]);
    let other = Mac([2, 0, 0, 0, 0, 8]);
    keeper.remember(ap, Zeroizing::new([1; 32]), [1; 16], false, 0);
    keeper.remember(other, Zeroizing::new([2; 32]), [2; 16], true, 0);
    keeper.remember(ap, Zeroizing::new([3; 32]), [3; 16], true, 100);
    let (pmk, pmkid, h2e) = keeper.cached(ap, 1_000).unwrap();
    assert_eq!((*pmk, pmkid, h2e), ([3; 32], [3; 16], true));
    assert_eq!(keeper.pmksa.len(), 2);
    // The first entry's 12 hours pass; the newer one's have not.
    assert!(keeper.cached(other, PMK_LIFETIME_US).is_none());
    assert!(keeper.cached(ap, PMK_LIFETIME_US).is_some());
    // A resumption that fails drops its entry, so the next runs SAE.
    keeper.resuming = Some(ap);
    keeper.failed(PMK_LIFETIME_US, String::from("model: association failed"));
    assert!(keeper.cached(ap, PMK_LIFETIME_US).is_none());
    // The cache keeps at most its 32 newest entries.
    for n in 0..=PMKSA_ENTRIES as u8 {
        keeper.remember(
            Mac([2, 1, 0, 0, 0, n]),
            Zeroizing::new([n; 32]),
            [n; 16],
            false,
            0,
        );
    }
    assert_eq!(keeper.pmksa.len(), PMKSA_ENTRIES);
    assert!(keeper.cached(Mac([2, 1, 0, 0, 0, 0]), 0).is_none());
    assert!(keeper.cached(Mac([2, 1, 0, 0, 0, 1]), 0).is_some());
}

/// key_mgmt=NONE matches a BSS without privacy, RSN or WPA; a WMM element
/// is no WPA element.
#[test]
fn an_open_network_takes_only_a_bss_without_privacy_rsn_or_wpa() {
    let mut bss = ap(1, 2, CHANNEL_1, AP_MLD, &[]);
    bss.elements.clear();
    bss.capabilities = 0x01;
    assert!(!protected(&bss));
    bss.capabilities |= 0x10;
    assert!(protected(&bss));
    bss.capabilities = 0x01;
    bss.elements = vec![48, 2, 1, 0];
    assert!(protected(&bss));
    bss.elements = vec![221, 4, 0, 0x50, 0xf2, 1];
    assert!(protected(&bss));
    bss.elements = vec![221, 4, 0, 0x50, 0xf2, 2];
    assert!(!protected(&bss));
}

/// What the join came to reaches the boot's record once per change: a
/// scan that finds no saved network, then a join, the same join again
/// giving nothing new.
#[test]
fn a_join_outcome_is_given_once_per_change() {
    let mut keeper = kept(Tuned { mac: LOCAL });
    let missing = crate::Join::NotInRange {
        ssid: b"test".to_vec(),
    };
    keeper.choose(0);
    assert_eq!(keeper.take_join(), Some(missing.clone()));
    assert_eq!(keeper.take_join(), None);
    keeper.choose(1);
    assert_eq!(keeper.take_join(), None);
    let joined = crate::Join::Joined {
        ssid: b"test".to_vec(),
    };
    keeper.note_join(joined.clone());
    keeper.note_join(joined.clone());
    assert_eq!(keeper.take_join(), Some(joined));
    assert_eq!(keeper.take_join(), None);
    keeper.note_join(missing.clone());
    assert_eq!(keeper.take_join(), Some(missing));
}

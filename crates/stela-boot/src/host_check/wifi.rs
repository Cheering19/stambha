//! `--wifi-check`: an RTL8852CE brought up to running firmware and powered
//! off again, as the mount's probe does (`rtw89_attach`), on the host check's
//! own tables and with no IOMMU.
//!
//! Before ExitBootServices ([`prepare`]): the firmware's PCI census names the
//! first `10ec:c852` function and its BAR2, which the tables map uncached;
//! the catalog of the USB stick's host firmware is verified as a storage
//! boot verifies it (`host_firmware::identity_catalog`); and pages below
//! 4 GiB are reserved for the rings. After it ([`check`]), in this order:
//! the function is parked as the mount parks it (`containment::Plan`: Bus
//! Master, MSI, MSI-X, No Snoop and ATS off, INTx disabled) and its
//! outstanding requests are waited for; it is brought to D0 and its BAR2 is
//! sized again and must be the range mapped; memory decode goes on; then
//! `rtw89_attach` stages `rtw89/rtw8852c_fw-2.bin` for the chip cut, stops a
//! MAC left on, rechecks the parked state, sets Bus Master and runs
//! `bring_up` on the passthrough domain over the reserved pages. Bus Master
//! is cleared again whatever the outcome, and memory decode is left as the
//! firmware left it.
//!
//! With `--wifi SSID` (M3 spec S17) the key file's secret is parsed before
//! ExitBootServices and its bytes wiped ([`Key`]), and a ChaCha20 stream is
//! seeded for the handshake's nonces. After it, regulatory.db is loaded
//! from the same catalog before the function is touched, and the same
//! take-over runs `ax::attach` in place of `bring_up`
//! (`rtw89_attach_with`): the chip stays up as an rtw89 station kept by the
//! connection keeper, polled through `carrier()` until its progress line
//! shows the 4-way handshake's frames both ways and the radio's refusal of
//! the first key, which M3a does not install, then [`FLUSH_US`] more, or
//! until [`STATION_BUDGET_US`] passes or `--deadline` comes within
//! [`DEADLINE_MARGIN_US`]. The keeper's last failure is taken, the station
//! deauthenticates and is polled [`FLUSH_US`] more, and the chip is stopped
//! (`Live::stop`); its rings and TX pools are kept. The network's name is
//! hidden wherever the line would print it (the keeper's failures name it),
//! then every address is masked; the secret is never printed: it lives in
//! the keeper's network, which clears it when the keeper goes.
//!
//! This touches no other device and writes no disk. It does not grant DMA
//! isolation: with no IOMMU the chip can reach any address, as under the
//! firmware. If the firmware left an IOMMU translating that does not map the
//! ring pages for the chip, its DMA faults or is dropped there and the check
//! fails at the first step the chip reads memory in, the firmware download
//! (`dma rings` only allocates and maps; CH12's descriptors or the
//! firmware's ready status then time out).
use super::{Machine, Timing, Verdict};
use alloc::{boxed::Box, format, string::String, string::ToString, sync::Arc, vec, vec::Vec};
use core::{any::Any, ops::Range, ptr::NonNull};
use stela_boot_core::{
    heap::Heap,
    host_wifi::{
        DEADLINE_MARGIN_US, End, FLUSH_US, Progress, STATION_BUDGET_US, Watch, hide_ssid,
        mask_addresses,
    },
    native_memory::NativeMemory,
    options::{DEFAULT_WIFI_KEY, HostCheck},
    passthrough::Passthrough,
};
use stela_drivers::{
    Clock,
    block::BlockError,
    bus::{
        dma::{mapping::DmaDomain, memory::PageSource},
        pci::{
            Address, Bar, ConfigSpace, EcamConfig, EcamWindow, Function, power_up_config, regs,
            requester::{
                ClaimError, containment::Plan as Park, validate_stopped,
                wait_for_pending_transactions,
            },
        },
    },
    entropy::Entropy,
    firmware::Catalog,
    net::{
        Link, LinkError,
        wifi::{
            connection::keeper::Radio,
            network::{Network, Secret},
            realtek::{
                ax::{self, AxIo, FirmwareFile, Host, NativeAxIo, attach::Inputs, pci::TxPools},
                identity::Observed,
            },
            regulatory::database::{self, Database},
        },
    },
    probe::{
        Error, ServiceError,
        native::{Rtw89Device, rtw89_attach, rtw89_attach_with},
    },
};
use uefi::boot::{self, AllocateType, MemoryType};

/// `rtw8852ce_id_table`'s one device (`rtw8852ce.c`).
const VENDOR: u16 = 0x10ec;
const DEVICE: u16 = 0xc852;
/// `rtw89_pci_setup_mapping` maps BAR2.
const BAR: u8 = 2;
/// The rings' pages: 13 TX BD rings, the RXQ and RPQ rings with 32
/// 11498-byte buffers of three pages each, and sixteen CH12 slots take 211
/// pages of payload, 278 with the heap's per-allocation page (a 16-byte
/// block header before each page-aligned block); the TX pools of the data
/// and management queues that every start now allocates (M3 spec S5.1)
/// take 43 more in one allocation, 44 with its header page: 322. The rest
/// is room.
const DMA_PAGES: usize = 384;
/// `WLAN_REASON_DEAUTH_LEAVING`, the station check's departure (spec S17.4).
const DEAUTH_LEAVING: u16 = 3;
const PAGE: usize = crate::paging::PAGE;
/// How long the function may keep requests outstanding after Bus Master is
/// cleared, about as long as Linux's `pci_wait_for_pending_transaction`.
const PENDING_SECONDS: u64 = 1;

/// The pages the rings come from, registered by [`pages`].
static HEAP: Heap = Heap::new();
static PAGES: NativeMemory =
    NativeMemory::new(&HEAP, crate::native_memory::cache::verify_writeback);

/// What [`prepare`] found before ExitBootServices.
pub(super) enum Plan {
    /// No `--wifi-check`.
    Off,
    /// Decided before ExitBootServices, with nothing touched.
    Decided(Verdict, String),
    Ready(Prepared),
}

pub(super) struct Prepared {
    address: Address,
    window: EcamWindow,
    /// BAR2, as the firmware's census reports it.
    registers: Range<u64>,
    /// [`DMA_PAGES`] reserved LOADER_DATA pages below 4 GiB.
    dma: Range<u64>,
    firmware: Arc<Catalog>,
    clock: Clock,
    /// With `--wifi`: the network to keep the chip connected to, boxed so
    /// that its secret (a passphrase's 63 bytes inline) is not copied each
    /// time the plan is handed on.
    station: Option<Box<Station>>,
}

/// The `--wifi` key file's bytes, wiped when dropped: [`prepare`] parses
/// them and drops them before anything else is prepared.
pub(crate) struct Key(Vec<u8>);

impl Key {
    pub(crate) fn new(bytes: Vec<u8>) -> Self {
        Self(bytes)
    }
}

impl Drop for Key {
    fn drop(&mut self) {
        for byte in self.0.iter_mut() {
            // SAFETY: a byte of the key, which nothing else reads, cleared
            // with a write the compiler keeps after its last use.
            unsafe { core::ptr::write_volatile(byte, 0) };
        }
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}

/// What `--wifi` keeps the chip connected to: the network, with the key
/// file's secret, and the handshake nonces' source.
struct Station {
    network: Network,
    entropy: Arc<dyn Entropy>,
}

/// The network `options` names, with `key`'s secret: a key file that is
/// missing, empty or unusable fails as `key: …`, without its contents; and
/// a stream seeded before ExitBootServices for the handshake's nonces.
fn station(options: &HostCheck, ssid: &str, key: Key) -> Result<Box<Station>, String> {
    let path = options.wifi_key.as_deref().unwrap_or(DEFAULT_WIFI_KEY);
    if key.0.is_empty() {
        return Err(format!("key: {path} is missing, empty or unreadable"));
    }
    let secret = Secret::parse(&key.0);
    drop(key);
    let secret = secret.map_err(|error| format!("key: {path} is unusable: {error}"))?;
    let country = options
        .wifi_country
        .as_deref()
        .and_then(|code| code.as_bytes().try_into().ok());
    let (source, _) = crate::entropy::source().map_err(|error| format!("entropy: {error}"))?;
    Ok(Box::new(Station {
        network: Network {
            ssid: ssid.as_bytes().to_vec(),
            secret,
            country,
        },
        entropy: Arc::new(source),
    }))
}

impl Plan {
    /// Map BAR2 uncached in the host check's tables.
    pub(super) fn map(self, tables: &mut crate::paging::Tables) -> Self {
        let Self::Ready(prepared) = self else {
            return self;
        };
        let Range { start, end } = prepared.registers;
        match tables.device(start as usize, end as usize) {
            Ok(()) => Self::Ready(prepared),
            Err(error) => Self::Decided(
                Verdict::Fail,
                format!("BAR2 {start:#x}..{end:#x} could not be mapped: {error}"),
            ),
        }
    }
}

/// Before ExitBootServices: with `--wifi`, the network and the nonces'
/// stream ([`station`]); then the first RTL8852CE in the firmware's PCI
/// census, its BAR2, the ECAM window that reaches it, the verified firmware
/// catalog and the ring pages. Nothing here touches the function.
pub(super) fn prepare(
    handoff: Option<&stela_drivers::Handoff>,
    resources: &crate::host_firmware::Resources,
    options: &HostCheck,
    key: Key,
) -> Plan {
    let fail = |detail: String| Plan::Decided(Verdict::Fail, detail);
    let station = match options.ssid.as_deref() {
        Some(ssid) => match station(options, ssid, key) {
            Ok(station) => Some(station),
            Err(why) => return fail(why),
        },
        None => {
            drop(key);
            None
        }
    };
    let census = match crate::pci_io::capture() {
        Ok(census) => census,
        Err(error) => return fail(format!("no firmware PCI census: {error}")),
    };
    let found = census
        .functions
        .iter()
        .filter(|function| {
            let identity = function.identity();
            (identity.vendor, identity.device) == (VENDOR, DEVICE)
        })
        .min_by_key(|function| function.address);
    let Some(function) = found else {
        return Plan::Decided(Verdict::Skip, String::from("no RTL8852CE"));
    };
    let address = function.address;
    let registers = match &function.bars[usize::from(BAR)] {
        Ok(regions) => stela_boot_core::firmware_pci::memory_bar(&function.header, BAR, regions)
            .map_err(String::from),
        Err(error) => Err(format!(
            "the firmware's BAR2 resources are unreadable: {error}"
        )),
    };
    let registers = match registers {
        Ok(registers) => registers,
        Err(why) => return fail(format!("pci={address} BAR2: {why}")),
    };
    let Some(handoff) = handoff else {
        return fail(String::from("no PCI access (see CHECK storage)"));
    };
    let window = handoff.ecam.iter().copied().find(|window| {
        window.segment == address.segment
            && (window.start_bus..=window.end_bus).contains(&address.bus)
    });
    let Some(window) = window else {
        return fail(format!("pci={address}: no ECAM window reaches it"));
    };
    // SAFETY: every immutable host-firmware page stays reserved
    // LOADER_DATA, and the host check's tables identity-map all such RAM
    // for the rest of the boot.
    let firmware = match unsafe { crate::host_firmware::identity_catalog(resources) } {
        Ok(Some(catalog)) => catalog,
        Ok(None) => return fail(String::from("firmware: the USB stick has no host firmware")),
        Err(error) => return fail(format!("firmware: {error}")),
    };
    let dma = match boot::allocate_pages(
        AllocateType::MaxAddress(0xffff_ffff),
        MemoryType::LOADER_DATA,
        DMA_PAGES,
    ) {
        Ok(pages) => pages.as_ptr() as u64,
        Err(_) => return fail(String::from("the firmware has no DMA pages below 4 GiB")),
    };
    crate::output::arguments(format_args!(
        "WIFI RTL8852CE pci={address} bar2={:#x}..{:#x} dma={dma:#x}..{:#x}\n",
        registers.start,
        registers.end,
        dma + (DMA_PAGES * PAGE) as u64
    ));
    Plan::Ready(Prepared {
        address,
        window,
        registers,
        dma: dma..dma + (DMA_PAGES * PAGE) as u64,
        firmware,
        clock: handoff.clock,
        station,
    })
}

/// After ExitBootServices: the bring-up, recorded as `CHECK wifi`, then
/// RF 0x18 per path and the networks the scan heard, one per line.
pub(super) fn check(machine: &mut Machine) {
    let mut lines = Vec::new();
    let (verdict, detail) = match core::mem::replace(&mut machine.wifi, Plan::Off) {
        Plan::Off => (Verdict::Skip, String::from("no --wifi-check")),
        Plan::Decided(verdict, detail) => (verdict, detail),
        Plan::Ready(prepared) => match run(prepared, &mut machine.kept, &mut machine.timing) {
            Ok((detail, found)) => {
                lines = found;
                (Verdict::Pass, detail)
            }
            Err(detail) => (Verdict::Fail, detail),
        },
    };
    machine.record("wifi", verdict, detail);
    for line in &lines {
        crate::output::arguments(format_args!("{line}\n"));
    }
    if !lines.is_empty() {
        crate::output::flush_log();
    }
}

/// One RTL8852CE through `rtw89_attach`: its parked function, the window
/// over its BAR2 and the passthrough domain over the ring pages.
struct Chip<'a> {
    function: &'a mut Function<EcamConfig>,
    io: NativeAxIo,
    domain: Arc<dyn DmaDomain>,
    /// DEVCTL2's LTR enable as the firmware left it ([`ltr_enabled`]).
    ltr: bool,
}

impl Rtw89Device for Chip<'_> {
    type Io = NativeAxIo;

    fn io(&mut self) -> &mut NativeAxIo {
        &mut self.io
    }

    fn host(&self) -> Host {
        Host { ltr: self.ltr }
    }

    /// The parked state checked again, after `quiesce`, then the
    /// passthrough domain: the host check has no IOMMU to bind.
    unsafe fn finalize(&mut self) -> Result<Arc<dyn DmaDomain>, Error> {
        validate_stopped(self.function)
            .map_err(|error| Error::Native(ServiceError::Claim(error)))?;
        Ok(self.domain.clone())
    }

    /// Bus Master on, with MSI and MSI-X off and INTx disabled.
    fn master(&mut self) -> Result<(), Error> {
        self.function.enable_polled().map_err(Error::Device)
    }
}

/// The bring-up of `prepared`'s function. Whatever happens past the park,
/// Bus Master ends clear and memory decode as the firmware left it; what
/// DMA could still reach (the rings) is kept in `kept`.
fn run(
    prepared: Prepared,
    kept: &mut Vec<Box<dyn Any>>,
    timing: &mut Timing,
) -> Result<(String, Vec<String>), String> {
    let address = prepared.address;
    // SAFETY: one CPU after ExitBootServices; the storage probe is done
    // with every function, and nothing else reaches this one's
    // configuration while the check holds it. The window is mapped
    // uncached (`storage::discover` maps every ECAM window).
    let config = unsafe {
        prepared
            .window
            .config(address.bus, address.device, address.function)
    };
    let mut function = Function::new(address, config)
        .map_err(|refusal| format!("pci={address} does not answer: {:?}", refusal.reason))?;
    if (function.vendor, function.device) != (VENDOR, DEVICE) {
        return Err(format!(
            "pci={address} is {:04x}:{:04x} now, not an RTL8852CE",
            function.vendor, function.device
        ));
    }
    let command = function
        .config()
        .read16(regs::COMMAND)
        .map_err(|error| format!("pci={address} COMMAND: {error}"))?;
    let result = attach(&mut function, prepared, kept, timing);
    // Bus Master off, and memory decode as found; the rest of the park
    // (MSI, MSI-X, No Snoop, ATS off, INTx disabled) stays.
    let restored = function.set_bus_master(false).and_then(|()| {
        if command & regs::COMMAND_MEMORY != 0 {
            return Ok(());
        }
        let config = function.config();
        let now = config.read16(regs::COMMAND)?;
        config.write16(regs::COMMAND, now & !regs::COMMAND_MEMORY)
    });
    kept.push(Box::new(function));
    match (result, restored) {
        (result, Ok(())) => result,
        (Ok((detail, _)) | Err(detail), Err(error)) => Err(format!(
            "{detail}; then clearing Bus Master or restoring memory decode failed: {error}"
        )),
    }
}

#[expect(
    clippy::result_large_err,
    reason = "rtw89_attach_with's step 5 returns the probe's own error, as bring_up's does."
)]
fn attach(
    function: &mut Function<EcamConfig>,
    prepared: Prepared,
    kept: &mut Vec<Box<dyn Any>>,
    timing: &mut Timing,
) -> Result<(String, Vec<String>), String> {
    let Prepared {
        address,
        registers,
        dma,
        firmware,
        clock,
        station,
        ..
    } = prepared;
    let at = |what: &str| format!("pci={address} {what}");
    // A station needs regulatory.db before anything is touched (spec S17.3).
    let station = match station {
        Some(station) => match database::load(provider(&firmware)) {
            Ok(database) => Some((station, database)),
            Err(error) => return Err(format!("regulatory.db: {error:?}")),
        },
        None => None,
    };
    park(function, timing.tsc_hz).map_err(|why| at(&why))?;
    // Linux `pci_power_up`, as the mount's `rtw89_wake`: D0, with the BARs
    // a D3hot soft reset cleared written back; then BAR2 sized again.
    power_up_config(function.config(), |microseconds| {
        clock
            .try_delay(microseconds)
            .map_err(|_| BlockError::Unsupported("the TSC cannot time the D0 delay"))
    })
    .map_err(|error| at(&format!("D0: {error}")))?;
    let bars = function
        .bars()
        .map_err(|error| at(&format!("BARs: {error}")))?;
    let sized = bars[usize::from(BAR)].as_ref().and_then(Bar::memory);
    if sized.as_ref() != Some(&registers) {
        return Err(at(&format!(
            "BAR2 sizes as {sized:x?}, not the {:#x}..{:#x} mapped",
            registers.start, registers.end
        )));
    }
    let config = function.config();
    let identity = config
        .read16(0x2c)
        .and_then(|vendor| Ok((vendor, config.read16(0x2e)?, config.read8(0x08)?)));
    let (subsystem_vendor, subsystem_device, pci_revision) =
        identity.map_err(|error| at(&format!("subsystem: {error}")))?;
    // Memory decode on, MSI and MSI-X off, INTx disabled; Bus Master stays
    // clear.
    let ltr = ltr_enabled(function);
    function
        .enable_polled_decode()
        .map_err(|error| at(&format!("memory decode: {error}")))?;
    let pages = pages(&dma)?;
    let bytes = (registers.end - registers.start) as usize;
    let window = NonNull::new(registers.start as *mut u8).ok_or_else(|| at("BAR2 is at 0"))?;
    // SAFETY: BAR2 is mapped uncached in the host check's tables for the
    // rest of the boot (`Plan::map`), `bytes` long, and sized just above to
    // that same range; this check alone drives the function, and the
    // handoff's clock is the TSC rate of this, the only, CPU. The DMA
    // addresses written through it are those `bring_up` writes for the
    // rings on the passthrough domain built for this requester below.
    let mut io = unsafe { NativeAxIo::new(window, bytes, Arc::new(()) as Arc<dyn Any>, clock) }
        .map_err(|error| format!("RTL8852CE bring-up: register window: {error}"))?;
    let sys_cfg1 = io
        .read32(0xf0)
        .map_err(|error| format!("RTL8852CE bring-up: identity: {error}"))?;
    let observed = Observed {
        vendor: function.vendor,
        device: function.device,
        subsystem_vendor,
        subsystem_device,
        pci_revision,
        sys_cfg1,
    };
    let mut chip = Chip {
        function,
        io,
        domain: Arc::new(Passthrough::new(vec![dma])),
        ltr,
    };
    let left_on = |was_on: bool| {
        if was_on {
            "; its MAC had been left powered on, and was powered off first"
        } else {
            ""
        }
    };
    // `rtw89_attach`'s contract, which is `rtw89_attach_with`'s too, as the
    // host check holds it:
    // - `chip.io` is the uncached BAR2 window of this function alone, which
    //   `park` stopped (Bus Master, MSI, MSI-X, No Snoop and ATS off, INTx
    //   disabled, as `containment::Plan` parks it for the mount) and whose
    //   outstanding requests it waited for; Bus Master has stayed clear since,
    //   and only `chip.master` sets it, after `quiesce` and `finalize`.
    // - Earlier users are retired: boot services have ended, so no firmware
    //   driver runs; the host check installs no handler for the function and
    //   runs nothing else on it.
    // - `finalize` rechecks the parked state (`validate_stopped`) and hands
    //   out a passthrough domain whose regions are only the ring pages: the
    //   host check has no IOMMU, so the domain confines nothing, and DMA
    //   isolation is not claimed.
    // - `pages` is coherent write-back RAM: reserved LOADER_DATA below
    //   4 GiB, identity-mapped write-back and checked so (`pages`), snooped
    //   by the chip with No Snoop off.
    // - `chip` keeps BAR2 and the function through every outcome, and the
    //   rings (and with `--wifi` the TX pools) are kept below.
    if let Some((station, database)) = station {
        // SAFETY: the contract above.
        let attached = unsafe {
            rtw89_attach_with(
                &mut chip,
                observed,
                provider(&firmware),
                pages,
                |io, pages, domain, cut, file, host| {
                    Ok(keep(
                        io, pages, domain, cut, file, host, station, database, clock, timing,
                    ))
                },
            )
        };
        let Chip { io, domain, .. } = chip;
        kept.push(Box::new((io, domain, firmware)));
        return match attached {
            Ok((ran, was_on)) => {
                let Kept {
                    result,
                    rings,
                    pools,
                } = ran;
                kept.push(Box::new((rings, pools)));
                match result {
                    Ok((detail, lines)) => Ok((format!("{detail}{}", left_on(was_on)), lines)),
                    Err(detail) => Err(format!("{detail}{}", left_on(was_on))),
                }
            }
            Err((error, was_on)) => Err(format!("{}{}", failed(&error), left_on(was_on))),
        };
    }
    // SAFETY: the contract above.
    let attached = unsafe { rtw89_attach(&mut chip, observed, provider(&firmware), pages) };
    let Chip { io, domain, .. } = chip;
    kept.push(Box::new((io, domain, firmware)));
    match attached {
        Ok((running, rings, was_on)) => {
            kept.push(Box::new(rings));
            // RF 0x18 per path (both windows), then the networks, strongest
            // first, at most 32 (`ScanReport::listed`), each on its own line.
            let mut lines: Vec<String> = Vec::new();
            if let Some(rf18) = &running.rf18 {
                lines.extend(rf18.lines().map(|line| line.to_string()));
            }
            if let Some(scan) = &running.scan {
                lines.extend(scan.listed().map(|listed| listed.to_string()));
            }
            Ok((format!("{running}{}", left_on(was_on)), lines))
        }
        Err((error, was_on)) => Err(format!("{}{}", failed(&error), left_on(was_on))),
    }
}

/// What the station check ran to: its line and the `WIFI RF18` lines, or
/// its failure; and what DMA could still reach, which is kept.
struct Kept {
    result: Result<(String, Vec<String>), String>,
    rings: Option<ax::Rings>,
    pools: Option<TxPools>,
}

/// The station check (M3 spec S17.3-S17.5) on a chip `rtw89_attach_with`
/// took over: `ax::attach` with `station`, `database` and `clock`, the
/// keeper polled through `carrier()` until [`Watch`] ends it, the keeper's
/// last failure taken with one `receive()` (`into_radio` drops one not yet
/// reported), then the deauthentication, [`FLUSH_US`] of service and the
/// stop. The line is the bring-up's report and the station's progress, with
/// the keeper's failure when it had one, the network's name hidden and then
/// every address masked; a PASS once the handshake ran, a FAIL when the
/// budget or the deadline's margin passed first or the attach failed.
/// `timing` beats through every wait but never stops here: the watch stops
/// [`DEADLINE_MARGIN_US`] short of `--deadline`, so the chip is stopped
/// (and then Bus Master cleared) before the deadline can end the check.
#[expect(
    clippy::too_many_arguments,
    reason = "rtw89_attach_with's step 5 inputs, then the station's own."
)]
#[expect(
    clippy::boxed_local,
    reason = "The box keeps the passphrase's inline bytes from being copied on every hand-on."
)]
fn keep(
    io: &mut NativeAxIo,
    pages: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    cut: u8,
    file: FirmwareFile<'_>,
    host: Host,
    station: Box<Station>,
    database: Database,
    clock: Clock,
    timing: &mut Timing,
) -> Kept {
    // The keeper owns the network once attached, and its failures name it.
    let ssid = station.network.ssid.clone();
    let inputs = Inputs {
        clock,
        database,
        network: station.network,
        entropy: station.entropy,
    };
    // SAFETY: `rtw89_attach_with` runs this with `bring_up`'s contract
    // holding for `io`, `pages` and `domain`, and `ax::attach`'s contract is
    // that one (`live::start`'s): the window is this requester's alone, the
    // domain isolates it, the pages are coherent write-back RAM, and this
    // check is its one worker. The station lives only inside this call,
    // while that holds, and is stopped before it returns.
    let attached = unsafe { ax::attach(io, pages, domain, cut, file, host, inputs) };
    let mut wifi = match attached {
        Ok(wifi) => wifi,
        Err((error, returned)) => {
            return Kept {
                result: Err(mask_addresses(&hide_ssid(&error.to_string(), &ssid))),
                rings: returned.and_then(|(_, rings)| rings),
                pools: None,
            };
        }
    };
    let now = clock.now_us();
    let deadline = timing
        .until_deadline_us()
        .map(|left| now.saturating_add(left));
    let mut watch = Watch::new(now, deadline);
    let end = loop {
        timing.beat();
        wifi.carrier();
        let line = wifi.diagnostic().unwrap_or_default();
        if let Some(end) = watch.after(clock.now_us(), &Progress::read(&line)) {
            break end;
        }
    };
    let failure = match wifi.receive() {
        Err(LinkError::Adapter(failure)) => Some(failure),
        Err(error) => Some(format!("{error:?}")),
        Ok(_) => None,
    }
    .map(|failure| hide_ssid(&failure, &ssid));
    let progress = wifi.diagnostic().unwrap_or_default();
    // Spec S17.4: leave, let the deauthentication go out, stop the chip.
    let mut radio = wifi.into_radio();
    let _ = radio.deauthenticate(DEAUTH_LEAVING);
    let left = clock.now_us();
    loop {
        timing.beat();
        let _ = radio.service();
        if clock.now_us().saturating_sub(left) >= FLUSH_US {
            break;
        }
    }
    let live = radio.into_chip();
    let running = *live.running();
    // SAFETY: as for `ax::attach` above, still holding.
    let (_, rings, pools) = unsafe { live.stop() };
    let mut detail = format!("{running}; station {progress}");
    if let Some(failure) = failure {
        detail.push_str(&format!("; keeper {failure}"));
    }
    match end {
        End::Handshaken => {}
        End::Budget => detail.push_str(&format!(
            "; no EAPOL handshake within {} s",
            STATION_BUDGET_US / 1_000_000
        )),
        End::Deadline => detail.push_str(&format!(
            "; stopped {} s before --deadline without an EAPOL handshake",
            DEADLINE_MARGIN_US / 1_000_000
        )),
    }
    let detail = mask_addresses(&hide_ssid(&detail, &ssid));
    let lines = running.rf18_lines().map(|line| line.to_string()).collect();
    Kept {
        result: if end == End::Handshaken {
            Ok((detail, lines))
        } else {
            Err(detail)
        },
        rings: Some(rings),
        pools: Some(pools),
    }
}

/// A failed attach in words: a staging failure as `firmware: …`, a
/// bring-up step as `BringUpError` writes it, anything else as the probe
/// writes it.
/// The verified catalog as a normal boot's drivers ask it
/// (`engine::firmware_requests`): a request with no exact binding gets the
/// USB bundle file its recipe pins, which the catalog holds by content
/// (`stela-bundle`, `sha256/…`), not by its linux-firmware name. Both live
/// for the rest of the boot, as the bring-up's rings borrow the image.
fn provider(catalog: &Arc<Catalog>) -> &'static dyn stela_drivers::firmware::Provider {
    let held: &'static Arc<Catalog> = Box::leak(Box::new(Arc::clone(catalog)));
    let catalog: &'static Catalog = held;
    Box::leak(Box::new(
        stela_loader::host_firmware::bundle::BundleProvider::new(catalog),
    ))
}

/// `rtw89_pci_dev_ltr_enabled` (`pci.c:3042`), as the probe reads it
/// (`probe::native`'s `rtw89_decode`): DEVCTL2's LTR enable (bit 10 at PCIe
/// capability + 0x28), false without a PCIe capability of version 2 or
/// above, or when config space cannot be read.
fn ltr_enabled(function: &mut Function<EcamConfig>) -> bool {
    const CAP_ID_EXP: u8 = 0x10;
    const EXP_FLAGS_VERS: u16 = 0x000f;
    const EXP_DEVCTL2: u16 = 0x28;
    const EXP_DEVCTL2_LTR_EN: u16 = 1 << 10;
    let Ok(capabilities) = function.capabilities() else {
        return false;
    };
    let Some(cap) = capabilities
        .into_iter()
        .find(|&(_, id)| id == CAP_ID_EXP)
        .map(|(offset, _)| u16::from(offset))
    else {
        return false;
    };
    let config = function.config();
    matches!(config.read16(cap + 2), Ok(flags) if flags & EXP_FLAGS_VERS > 1)
        && matches!(config.read16(cap + EXP_DEVCTL2), Ok(v) if v & EXP_DEVCTL2_LTR_EN != 0)
}

fn failed(error: &Error) -> String {
    match error {
        Error::Rtw89Wifi(error) if error.step == "firmware" => format!("firmware: {}", error.error),
        Error::Rtw89Wifi(error) => error.to_string(),
        error => format!("RTL8852CE: {error}"),
    }
}

/// Park the function as the mount's census does (`containment::Plan`),
/// then wait for its outstanding requests, as the mount's takeover does: a
/// bit still pending is reported and the check carries on, as Linux does.
fn park(function: &mut Function<EcamConfig>, tsc_hz: u64) -> Result<(), String> {
    let plan = {
        let config = function.config();
        Park::read(
            |offset, width| {
                match width {
                    1 => config.read8(offset).map(u32::from),
                    2 => config.read16(offset).map(u32::from),
                    _ => config.read32(offset),
                }
                .map_err(ClaimError::Config)
            },
            false,
            false,
        )
    }
    .map_err(|error| format!("park: {error:?}"))?;
    if let Some(error) = plan.refusal() {
        return Err(format!("park: a control cannot be stopped: {error:?}"));
    }
    plan.apply(function.config())
        .map_err(|error| format!("park: {error}"))?;
    let start = crate::apic::rdtsc();
    let pending = wait_for_pending_transactions(function.config(), || {
        crate::apic::rdtsc() - start > PENDING_SECONDS * tsc_hz
    })
    .map_err(|error| format!("park: {error:?}"))?;
    if let Some(pending) = pending {
        crate::output::arguments(format_args!(
            "WIFI RTL8852CE: Transactions Pending still set {PENDING_SECONDS} s after Bus Master \
             was cleared (status {:#06x} at {:#x}); carrying on\n",
            pending.status, pending.offset
        ));
    }
    Ok(())
}

/// The ring pages, registered once: write-back checked, then a heap.
fn pages(dma: &Range<u64>) -> Result<&'static NativeMemory, String> {
    let bytes = (dma.end - dma.start) as usize;
    // SAFETY: `prepare` reserved these LOADER_DATA pages for this alone;
    // the host check's active tables identity-map them, and nothing else
    // maps them.
    unsafe { crate::native_memory::qualify_reserved_writeback(dma.start, dma.start, bytes) }
        .map_err(|error| format!("the ring pages are not write-back RAM: {error}"))?;
    // SAFETY: reserved, exclusively owned, page-aligned RAM below 4 GiB,
    // identity-mapped write-back for the rest of the boot (offset 0), never
    // given to a guest or a free list; this runs once, before any
    // allocation from it, on the host check's one CPU.
    unsafe {
        HEAP.initialize(dma.start as *mut u8, bytes);
        PAGES.initialize(dma.start as usize, dma.start, bytes, 0);
    }
    Ok(&PAGES)
}

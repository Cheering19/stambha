//! `--host-check --bluetooth-check`'s lines: one `CHECK` line for each step
//! of a Realtek USB Bluetooth controller's bring-up as the host check's
//! probe ran it (`stela_drivers::usb::bluetooth::realtek`), one for the
//! passive LE scan after it, and a summary. The readings are named as
//! btrtl's kernel log names them (`examining hci_ver=… hci_rev=… lmp_ver=…
//! lmp_subver=…`, `rom_version`, `key id`, `fw version 0x…`).
//!
//! No address is printed whole: the controller's as [`masked::Mac`], its
//! vendor half and last byte, and a driver error without the bytes it
//! carries, which can hold one. Every line also passes
//! [`masked::mask_line`]. Nothing here touches hardware.
//!
//! [`BluetoothOnly`] is the firmware such a probe gives its drivers.
use alloc::{
    format,
    string::{String, ToString},
    vec::Vec,
};
use core::fmt;
use stela_contracts::masked;
use stela_drivers::{
    firmware::{self, Image, Provider, Requirement},
    usb::bluetooth::{
        InitializationError, InitializationReport, Setup,
        realtek::{
            self, Attempt, ConfigurationEvidence, ContainerVersion, FirmwareDrop, Identity,
            LocalVersion, OwnedReport, Project, ScanReport,
        },
    },
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    Pass,
    Fail,
    Skip,
}

/// One `CHECK name verdict detail` line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Line {
    pub name: &'static str,
    pub verdict: Verdict,
    pub detail: String,
}

/// A USB Bluetooth controller as the probe kept it.
pub struct Controller<'a> {
    pub location: &'a str,
    pub vendor: u16,
    pub product: u16,
    pub revision: u16,
    pub initialization: Option<Result<&'a InitializationReport, &'a InitializationError>>,
}

/// The step names, in order; the summary is `bluetooth`.
pub const STEPS: [&str; 6] = [
    "bluetooth-found",
    "bluetooth-rom",
    "bluetooth-firmware",
    "bluetooth-download",
    "bluetooth-version",
    "bluetooth-scan",
];

/// The controller the check takes among `controllers`: the first whose
/// IDs a Realtek setup takes.
pub fn chosen(controllers: &[Controller<'_>]) -> Option<usize> {
    controllers.iter().position(|controller| {
        Setup::of(controller.vendor, controller.product, false) == Some(Setup::Realtek)
    })
}

/// Whether the chosen controller came up, so the scan may run on it.
pub fn scannable(controller: &Controller<'_>) -> bool {
    matches!(
        controller.initialization,
        Some(Ok(InitializationReport::Realtek(_)))
    )
}

/// The check's lines for the controllers the probe kept, with the chosen
/// one's scan, `None` where none ran.
pub fn lines(
    controllers: &[Controller<'_>],
    scan: Option<&realtek::Result<ScanReport>>,
) -> Vec<Line> {
    let mut lines = Vec::with_capacity(STEPS.len() + 1);
    let mut add = |name, verdict, detail: String| {
        lines.push(Line {
            name,
            verdict,
            detail: masked::mask_line(&detail).into_owned(),
        })
    };
    let Some(index) = chosen(controllers) else {
        let found: Vec<String> = controllers
            .iter()
            .map(|c| format!("{:04x}:{:04x} at {}", c.vendor, c.product, c.location))
            .collect();
        add(
            STEPS[0],
            Verdict::Fail,
            format!(
                "no USB Bluetooth controller a Realtek setup takes; found {}",
                if found.is_empty() {
                    String::from("none")
                } else {
                    found.join(", ")
                }
            ),
        );
        for name in &STEPS[1..] {
            add(name, Verdict::Skip, String::from("no Realtek controller"));
        }
        add(
            "bluetooth",
            Verdict::Fail,
            String::from("failed at bluetooth-found"),
        );
        return lines;
    };
    let controller = &controllers[index];
    add(
        STEPS[0],
        Verdict::Pass,
        format!(
            "{:04x}:{:04x} revision {:04x} at {}: Realtek setup",
            controller.vendor, controller.product, controller.revision, controller.location
        ),
    );
    let steps = match controller.initialization {
        None => failed_before(String::from("the probe ran no initialization on it")),
        Some(Ok(InitializationReport::Realtek(report))) => booted(report),
        Some(Ok(_)) => failed_before(String::from("another vendor's setup took it")),
        Some(Err(InitializationError::Realtek(error))) => attempted(error),
        Some(Err(error)) => failed_before(transport_or_other(error)),
    };
    for (name, (verdict, detail)) in STEPS[1..5].iter().zip(steps) {
        add(name, verdict, detail);
    }
    let scan = match scan {
        None => (
            Verdict::Skip,
            String::from("the controller did not come up, so it was not scanned"),
        ),
        Some(Ok(report)) => scanned(report),
        Some(Err(error)) => (Verdict::Fail, format!("the scan failed: {}", cause(error))),
    };
    add(STEPS[5], scan.0, scan.1);
    let failed = lines
        .iter()
        .find(|line| line.verdict != Verdict::Pass)
        .map(|line| line.name);
    let summary = match failed {
        None => (
            Verdict::Pass,
            format!(
                "{:04x}:{:04x} runs its patch and its radio hears advertisers",
                controller.vendor, controller.product
            ),
        ),
        Some(name) => (Verdict::Fail, format!("failed at {name}")),
    };
    lines.push(Line {
        name: "bluetooth",
        verdict: summary.0,
        detail: summary.1,
    });
    lines
}

type Step = (Verdict, String);

/// The four bring-up steps of a controller whose initialization never
/// began Realtek's: the first fails for `why`, the rest are skipped.
fn failed_before(why: String) -> [Step; 4] {
    [
        (Verdict::Fail, why),
        (
            Verdict::Skip,
            String::from("the controller was not identified"),
        ),
        (Verdict::Skip, String::from("no image was staged")),
        (Verdict::Skip, String::from("no image was downloaded")),
    ]
}

fn booted(report: &OwnedReport) -> [Step; 4] {
    [
        (
            Verdict::Pass,
            identity(&report.before, report.dropped.as_deref()),
        ),
        (
            Verdict::Pass,
            image(
                &report.firmware,
                report.fallback.as_deref(),
                Some(&report.container_version),
                &match &report.configuration {
                    ConfigurationEvidence::Supplied(config) => receipt(config),
                    ConfigurationEvidence::OptionalMissing(_) => {
                        String::from("no config (optional, missing)")
                    }
                },
            ),
        ),
        (Verdict::Pass, download(&report.download)),
        (
            Verdict::Pass,
            format!(
                "{}; address {}",
                after(&report.after),
                masked::Mac(&display_order(&report.address))
            ),
        ),
    ]
}

fn attempted(error: &realtek::Error) -> [Step; 4] {
    let why = cause(error);
    let Some(attempt) = observation(error) else {
        return failed_before(why);
    };
    let identified = match (&attempt.identity, &attempt.version) {
        (Some(identity), _) => (
            Verdict::Pass,
            self::identity(identity, attempt.dropped.as_ref()),
        ),
        (None, Some(version)) => (
            Verdict::Fail,
            format!("{}: {why}", readings(attempt, version)),
        ),
        (None, None) => (Verdict::Fail, why.clone()),
    };
    if attempt.identity.is_none() {
        let mut steps = failed_before(String::new());
        steps[0] = identified;
        return steps;
    }
    let config = match &attempt.configuration {
        Some(Ok(config)) => receipt(config),
        Some(Err(error)) => format!("config {}", resource(error)),
        None => String::from("no config asked"),
    };
    let staged = attempt.container_version.is_some();
    let firmware = match (&attempt.firmware, staged) {
        (Some(firmware), true) => (
            Verdict::Pass,
            image(
                firmware,
                attempt.fallback.as_ref(),
                attempt.container_version.as_ref(),
                &config,
            ),
        ),
        (Some(firmware), false) => (
            Verdict::Fail,
            format!(
                "{}: {why}",
                image(firmware, attempt.fallback.as_ref(), None, &config)
            ),
        ),
        (None, _) => (
            Verdict::Fail,
            match &attempt.fallback {
                Some(first) => format!("{why} ({} first); {config}", resource(first)),
                None => format!("{why}; {config}"),
            },
        ),
    };
    let downloaded = attempt.after.is_some();
    let download_step = match (staged, downloaded) {
        (false, _) => (Verdict::Skip, String::from("no image was staged")),
        (true, true) => (Verdict::Pass, self::download(&attempt.download)),
        (true, false) => (
            Verdict::Fail,
            format!("{}: {why}", self::download(&attempt.download)),
        ),
    };
    let version = match &attempt.after {
        Some(version) => (Verdict::Fail, format!("{}: {why}", after(version))),
        None => (Verdict::Skip, String::from("no image was downloaded")),
    };
    [identified, firmware, download_step, version]
}

/// The partial observations a failed initialization kept.
fn observation(error: &realtek::Error) -> Option<&Attempt> {
    match error {
        realtek::Error::Attempt { observation, .. } => Some(observation),
        realtek::Error::Cleanup { cause, .. } => observation(cause),
        _ => None,
    }
}

fn project(project: Project) -> &'static str {
    match project {
        Project::Rtl8761Bu => "RTL8761BU",
        Project::Rtl8822B => "RTL8822B",
        Project::Rtl8852Bu => "RTL8852BU",
        Project::Rtl8852Cu => "RTL8852CU",
    }
}

/// btrtl's `examining` line's readings.
fn examining(version: &LocalVersion) -> String {
    format!(
        "hci_ver={:02x} hci_rev={:04x} lmp_ver={:02x} lmp_subver={:04x} manufacturer={:04x}",
        version.hci_version(),
        version.hci_revision(),
        version.lmp_version(),
        version.lmp_subversion(),
        version.manufacturer()
    )
}

fn register(subversion: u16, revision: Option<u16>) -> String {
    match revision {
        Some(revision) => format!("register={subversion:04x}/{revision:04x}"),
        None => format!("register={subversion:04x}"),
    }
}

fn dropped(drop: &FirmwareDrop) -> String {
    format!(
        "; a running patch was dropped first with 0xfc66 (it read {}, {}; {} events in the \
         200 ms after)",
        examining(&drop.version),
        register(drop.register_subversion, drop.register_revision),
        drop.event_count
    )
}

fn identity(identity: &Identity, drop: Option<&FirmwareDrop>) -> String {
    format!(
        "{} stock ROM: {} {} rom_version={:x} key_id={} (security {:02x}-{:02x}){}",
        project(identity.project),
        examining(&identity.version),
        register(identity.register_subversion, identity.register_revision),
        identity.rom_version,
        identity.security[0],
        identity.security[0],
        identity.security[1],
        drop.map(dropped).unwrap_or_default()
    )
}

/// The readings of a controller that was not identified.
fn readings(attempt: &Attempt, version: &LocalVersion) -> String {
    let mut text = examining(version);
    if let Some(subversion) = attempt.register_subversion {
        text.push(' ');
        text.push_str(&register(subversion, attempt.register_revision));
    }
    if let Some(drop) = &attempt.dropped {
        text.push_str(&dropped(drop));
    }
    text
}

fn short(sha256: &[u8; 32]) -> String {
    sha256[..8]
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn receipt(receipt: &stela_drivers::usb::bluetooth::FirmwareReceipt) -> String {
    let member = if receipt.member == receipt.name {
        String::new()
    } else {
        format!(" ({})", receipt.member)
    };
    format!("config {} {} bytes{member}", receipt.name, receipt.length)
}

fn image(
    firmware: &stela_drivers::usb::bluetooth::FirmwareReceipt,
    fallback: Option<&firmware::Error>,
    container: Option<&ContainerVersion>,
    config: &str,
) -> String {
    let container = match container {
        Some(ContainerVersion::V2(version)) => format!(
            " EPATCH v2 {}",
            version
                .iter()
                .map(|byte| format!("{byte:02x}"))
                .collect::<String>()
        ),
        Some(ContainerVersion::V1(version)) => format!(" EPATCH v1 {version:#010x}"),
        None => String::new(),
    };
    let fallback = fallback
        .map(|first| format!(" (after {})", resource(first)))
        .unwrap_or_default();
    format!(
        "{} {} bytes sha256 {}…{container}{fallback}; {config}",
        firmware.name,
        firmware.length,
        short(&firmware.sha256)
    )
}

fn download(download: &[realtek::DownloadResponse]) -> String {
    match download.last() {
        Some(last) => format!(
            "{} fragments acknowledged, the last sent as index {:#04x}",
            download.len(),
            last.sent_index
        ),
        None => String::from("no fragment acknowledged"),
    }
}

/// The post-patch Read Local Version, with btrtl's `fw version` line.
fn after(version: &LocalVersion) -> String {
    format!(
        "fw version 0x{:04x}{:04x}: {}",
        version.hci_revision(),
        version.lmp_subversion(),
        examining(version)
    )
}

/// Read BD_ADDR's bytes, least significant first, in the order an address
/// is written.
fn display_order(address: &[u8; 6]) -> [u8; 6] {
    let mut shown = *address;
    shown.reverse();
    shown
}

fn scanned(report: &ScanReport) -> Step {
    let seconds = report.window_us / 1_000_000;
    let mut extra = String::new();
    if report.saturated {
        extra.push_str(&format!(
            ", more than {} advertisers",
            realtek::MAX_ADVERTISERS
        ));
    }
    if report.malformed > 0 {
        extra.push_str(&format!(
            ", {} reports dropped as Linux drops them",
            report.malformed
        ));
    }
    if report.other_events > 0 {
        extra.push_str(&format!(", {} other events", report.other_events));
    }
    match (report.devices, report.best_rssi) {
        (0, _) => (
            Verdict::Fail,
            format!(
                "passive LE scan for {seconds} s heard no advertiser ({} reports{extra}): \
                 nothing advertised nearby, or the radio hears nothing (Wi-Fi may hold the \
                 shared antenna)",
                report.reports
            ),
        ),
        (devices, rssi) => (
            Verdict::Pass,
            format!(
                "passive LE scan for {seconds} s: {devices} advertisers in {} reports, best \
                 RSSI {}{extra}",
                report.reports,
                rssi.map_or_else(
                    || String::from("not reported"),
                    |rssi| format!("{rssi} dBm")
                )
            ),
        ),
    }
}

/// A driver error in words, without the bytes it carries: a response's
/// can be the controller's address.
fn cause(error: &realtek::Error) -> String {
    use realtek::Error;
    match error {
        Error::Attempt { cause: inner, .. } => cause(inner),
        Error::Cleanup {
            cause: inner,
            cleanup,
        } => format!(
            "{}; closing the transport failed too: {}",
            cause(inner),
            transport(cleanup)
        ),
        Error::Unsupported(why) => why.to_string(),
        Error::Protocol {
            requirement,
            observed,
        } => format!("{requirement} ({} bytes)", observed.len()),
        Error::Format(format) => format!("firmware format {format:?}"),
        Error::Resource(error) => resource(error),
        Error::Controller { opcode, status, .. } => {
            format!("command {opcode:#06x} answered status {status:#04x}")
        }
        Error::Timeout(what) => format!("timed out waiting for {what}"),
        Error::Transport(error) => transport(error),
    }
}

/// A firmware request's failure: its name, unquoted, and why.
fn resource(error: &firmware::Error) -> String {
    format!(
        "{} {}",
        error.name.as_str().unwrap_or("(unnamed)"),
        error.kind
    )
}

fn transport(error: &stela_drivers::usb::bluetooth::Error) -> String {
    use stela_drivers::usb::bluetooth::ErrorKind;
    let kind = match &error.kind {
        ErrorKind::Usb(error) => format!("USB {error}"),
        ErrorKind::InvalidPacket(why) => format!("invalid packet: {why}"),
        ErrorKind::TooLarge { maximum, actual } => {
            format!("packet of {actual} bytes, past {maximum}")
        }
        ErrorKind::PartialTimeout(pipe) => format!("partial {pipe:?} frame timed out"),
        ErrorKind::ControllerDump { driver, .. } => format!("controller dump ({driver})"),
        ErrorKind::Closed => String::from("transport closed"),
        ErrorKind::Failed => String::from("transport failed"),
    };
    match &error.cleanup {
        Some(cleanup) => format!("{kind}; cancellation failed: {cleanup}"),
        None => kind,
    }
}

fn transport_or_other(error: &InitializationError) -> String {
    match error {
        InitializationError::Transport(error) => transport(error),
        InitializationError::UnsupportedDevice { .. } => error.to_string(),
        _ => String::from("another vendor's setup failed on it"),
    }
}

impl fmt::Display for Verdict {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Pass => "PASS",
            Self::Fail => "FAIL",
            Self::Skip => "SKIP",
        })
    }
}

/// The firmware a `--bluetooth-check` probe gives its drivers: a USB
/// Bluetooth initializer's request (its target's driver names
/// `-bluetooth-usb-`) goes to `bluetooth`, the stick's verified bundle, any
/// other to `rest`, so every other device starts as a plain host check
/// starts it.
pub struct BluetoothOnly<'a> {
    pub bluetooth: &'a dyn Provider,
    pub rest: &'a dyn Provider,
}

impl Provider for BluetoothOnly<'_> {
    fn load(&self, requirement: Requirement<'_>) -> Result<Image<'_>, firmware::Error> {
        if requirement.target.driver.contains("-bluetooth-usb-") {
            self.bluetooth.load(requirement)
        } else {
            self.rest.load(requirement)
        }
    }
}

#[cfg(test)]
mod tests;

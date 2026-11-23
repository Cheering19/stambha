//! The PCI Express link above a GPU: its L1 PM Substates set at the claim
//! as Linux's ASPM core leaves a link the firmware set up, and, for the
//! boot log, both ends' ASPM, Clock Power Management, LTR and L1 PM
//! Substates as the firmware left them and after the claim's reset, and the
//! port's view when the GPU stops answering or answers again, which tells a
//! link that went down from a port that lost power with it, with the
//! recovery the port still allows. A port that forwards to the GPU, as
//! every GPU's does, keeps its own configuration page with the fabric's
//! owner of it, beside which no other mapping may open, so it is read
//! through that owner; another port through a read-only mapping kept for
//! the boot. Only the substates, or with `--gpu-aspm off` the link's power
//! management, are written, through the fabric that owns the port.
use crate::{native_mmio, output};
use alloc::{format, string::String};
use core::sync::atomic::{AtomicBool, Ordering};
use stela_boot_core::options::GpuAspm;
use stela_drivers::bus::pci::{
    Address, ConfigError, ConfigFailure, ConfigSpace, Function, link_report, owned::ObservedConfig,
    owned::OwnedConfig,
};

/// The PCI Express port directly above a GPU, read only, and the fabric
/// that owns it.
pub(in crate::engine) struct Port {
    address: Address,
    #[cfg_attr(not(feature = "nvk"), allow(dead_code, reason = "asked at a loss"))]
    gpu: Address,
    view: View,
    source: &'static native_mmio::Source,
}

/// How a port's configuration is read.
enum View {
    /// Through the fabric's owner of a port whose forwarding it restored.
    Held,
    /// Through a read-only mapping of its page kept for the boot.
    Observed(ObservedConfig),
}

/// A forwarding port's configuration, read through the fabric's owner of
/// it ([`native_mmio::Source::read_forwarding_bridge`]); never written.
struct Held<'a> {
    source: &'a native_mmio::Source,
    address: Address,
}

impl Held<'_> {
    fn read(&self, offset: u16, width: u16) -> Result<u32, ConfigError> {
        match self
            .source
            .read_forwarding_bridge(self.address, offset, width)
        {
            Some(Ok(value)) => Ok(value),
            Some(Err(error)) => Err(ConfigError::before_effect(ConfigFailure::Mapping(error))),
            // The fabric no longer holds it open.
            None => Err(ConfigError::before_effect(ConfigFailure::ForeignGrant)),
        }
    }
}

impl ConfigSpace for Held<'_> {
    fn read8(&self, offset: u16) -> Result<u8, ConfigError> {
        self.read(offset, 1).map(|value| value as u8)
    }
    fn read16(&self, offset: u16) -> Result<u16, ConfigError> {
        self.read(offset, 2).map(|value| value as u16)
    }
    fn read32(&self, offset: u16) -> Result<u32, ConfigError> {
        self.read(offset, 4)
    }
    fn write8(&mut self, _: u16, _: u8) -> Result<(), ConfigError> {
        Err(ConfigError::before_effect(ConfigFailure::Protection))
    }
    fn write16(&mut self, _: u16, _: u16) -> Result<(), ConfigError> {
        Err(ConfigError::before_effect(ConfigFailure::Protection))
    }
    fn write32(&mut self, _: u16, _: u32) -> Result<(), ConfigError> {
        Err(ConfigError::before_effect(ConfigFailure::Protection))
    }
}

impl Port {
    /// What the port reports now.
    fn read(&self) -> Result<link_report::LinkEnd, ConfigError> {
        match &self.view {
            View::Held => link_report::read(&Held {
                source: self.source,
                address: self.address,
            }),
            View::Observed(config) => link_report::read(config),
        }
    }

    /// `port <address> <its report>`, or why it could not be read.
    fn report(&self) -> String {
        match self.read() {
            Ok(end) => format!("port {} {end}", self.address),
            Err(error) => format!("port {} unreadable: {error:?}", self.address),
        }
    }
}

/// The port above the GPU at `address`, read through the fabric's owner of
/// it when it forwards, else mapped read-only for the boot; `None` for one
/// on a root bus, and, said in a line, for one that cannot be read.
pub(in crate::engine) fn port_above(
    mmio: &'static native_mmio::Source,
    address: Address,
) -> Option<Port> {
    let bridge = mmio.parent_bridge(address)?;
    if mmio.read_forwarding_bridge(bridge, 0, 4).is_some() {
        return Some(Port {
            address: bridge,
            gpu: address,
            view: View::Held,
            source: mmio,
        });
    }
    let refused = |why: &dyn core::fmt::Debug| {
        output::arguments(format_args!(
            "GPU {address}: its port {bridge} is not read for the link lines: {why:?}\n"
        ));
    };
    // SAFETY: the mount's fabric owns the bridge's configuration and keeps
    // other writers out; this mapping only reads it, for the boot.
    let (window, mapping) = match unsafe { mmio.observe_config_page(bridge) } {
        Ok(observed) => observed,
        Err(error) => {
            refused(&error);
            return None;
        }
    };
    // SAFETY: the source issued this exact UC read-only page of `bridge`.
    match unsafe { ObservedConfig::attach(bridge, window, mapping) } {
        Ok(config) => Some(Port {
            address: bridge,
            gpu: address,
            view: View::Observed(config),
            source: mmio,
        }),
        Err((error, mut mapping)) => {
            // SAFETY: the refused attachment published no view or reader.
            let _ = unsafe { mapping.retire() };
            refused(&error);
            None
        }
    }
}

/// `GPU <address> link <when>: GPU <its report>; port <address> <its
/// report>`, from the GPU's configuration `gpu` and the port above it.
pub(in crate::engine) fn log_link<C: ConfigSpace + ?Sized>(
    address: Address,
    when: &str,
    gpu: &C,
    port: Option<&Port>,
) {
    let own = match link_report::read(gpu) {
        Ok(end) => format!("{end}"),
        Err(error) => format!("unreadable: {error:?}"),
    };
    let above = port.map_or_else(|| String::from("no port above it is read"), Port::report);
    output::arguments(format_args!(
        "GPU {address} link {when}: GPU {own}; {above}\n"
    ));
}

/// `GPU <address> L1 PM Substates: <what was found and done>`: the link
/// above the GPU `function` at `address`, in D0, set as Linux leaves it
/// ([`stela_drivers::bus::pci::substates`]) before the claim's reset, which
/// then saves and restores the GPU's side as set here.
///
/// # Safety
/// `function` is the cold owner of `address` the port's fabric promoted,
/// in D0, with Bus Master sealed; no other configuration writer runs.
pub(in crate::engine) unsafe fn set_substates(
    address: Address,
    function: &mut Function<OwnedConfig>,
    port: Option<&Port>,
) {
    let done = match port {
        None => String::from("not set: no port above it is read"),
        // SAFETY: forwarded; the fabric owns the port.
        Some(port) => match unsafe { port.source.configure_link_substates(address, function) } {
            Ok(configured) => format!("{configured}"),
            Err(why) => format!("not set: {why}"),
        },
    };
    output::arguments(format_args!("GPU {address} L1 PM Substates: {done}\n"));
}

/// Whether the GPUs' links go without power management (`--gpu-aspm off`).
static ASPM_OFF: AtomicBool = AtomicBool::new(false);

/// What [`set_link_power`] does at each GPU's claim from here: the boot's
/// `--gpu-aspm`, chosen before the GPUs are claimed.
pub(in crate::engine) fn choose(aspm: GpuAspm) {
    ASPM_OFF.store(aspm == GpuAspm::Off, Ordering::Relaxed);
}

/// `GPU <address> link power: <what was found and done>`: with
/// `--gpu-aspm off`, the link above the GPU `function` at `address`, in D0,
/// with ASPM L0s and L1 and every L1 PM Substate off at both ends
/// ([`stela_drivers::bus::pci::disable_link_power`]), before the claim's
/// reset, which saves and restores the GPU's side as set here. Without a
/// port that can be written, only the GPU's end, which still keeps the
/// link out of L1. Otherwise the substates as [`set_substates`] sets them.
///
/// # Safety
/// As [`set_substates`].
pub(in crate::engine) unsafe fn set_link_power(
    address: Address,
    function: &mut Function<OwnedConfig>,
    port: Option<&Port>,
) {
    if !ASPM_OFF.load(Ordering::Relaxed) {
        // SAFETY: forwarded.
        unsafe { set_substates(address, function, port) };
        return;
    }
    let done = match port {
        // SAFETY: forwarded; the fabric owns the port.
        Some(port) => unsafe { port.source.disable_link_power(address, function) }
            .map(|found| (found, Some(port.address))),
        None => {
            stela_drivers::bus::pci::disable_link_power::<OwnedConfig, _>(function.config(), None)
                .map(|found| (found, None))
                .map_err(|error| format!("{error:?}"))
        }
    };
    let line = match done {
        Ok((found, Some(at))) => {
            let port = found
                .port
                .map_or_else(|| String::from("unread"), |end| format!("{end}"));
            format!(
                "off at both ends by --gpu-aspm off: the GPU was {}, its port {at} {port}; \
                 both read back off",
                found.device
            )
        }
        Ok((found, None)) => format!(
            "off at the GPU's end only by --gpu-aspm off (no port above it is read), which \
             keeps the link out of L1: the GPU was {}; read back off",
            found.device
        ),
        Err(why) => format!("--gpu-aspm off failed, the link may be left as it was: {why}"),
    };
    output::arguments(format_args!("GPU {address} link power: {line}\n"));
}

/// `GPU <name> <event>; port <address> <its report>`: what the port above
/// a GPU whose driver said `event` of it reads now, which tells whether the
/// port and its link are still up, and, when it stopped answering, which
/// recovery the port still allows.
#[cfg(feature = "nvk")]
pub(in crate::engine) fn log_port_at(name: &str, event: &str, port: Option<&Port>) {
    let above = port.map_or_else(|| String::from("no port above it is read"), Port::report);
    output::arguments(format_args!("GPU {name} {event}; {above}\n"));
    if event == stela_boot_core::gpu_loss::STOPPED_ANSWERING {
        let recovery = match port {
            None => stela_boot_core::gpu_loss::Recovery::Unknown,
            Some(port) => port.recovery(),
        };
        output::arguments(format_args!("GPU {name} recovery: {recovery}\n"));
    }
}

#[cfg(feature = "nvk")]
impl Port {
    /// Which recovery the port allows now: none when it does not answer
    /// either, else a secondary bus reset through it if the fabric would
    /// make one, else why not.
    fn recovery(&self) -> stela_boot_core::gpu_loss::Recovery {
        use stela_boot_core::gpu_loss::Recovery;
        let end = match self.read() {
            Ok(end) => end,
            Err(_) => return Recovery::Unknown,
        };
        if !end.answers() {
            return Recovery::PortGone;
        }
        let link_up = end
            .express
            .is_some_and(|express| express.link_status & (1 << 13) != 0);
        match self.source.parent_bus_reset_refusal(self.gpu) {
            None => Recovery::BusReset {
                port: format!("{}", self.address),
                link_up,
            },
            Some(why) => Recovery::NoBusReset { why, link_up },
        }
    }
}

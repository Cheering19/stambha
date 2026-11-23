//! `native::claim_quiesced` over the mount's real cold preparation, on an
//! owned-configuration model with a symbolic original root: the seal is
//! released, and the root drained and bound, only after a completed reset;
//! every refusal keeps the preparation and its seal.
#![allow(
    clippy::result_large_err,
    reason = "the claims under test return the probe's own Failure, which keeps its owners"
)]
extern crate std;
use super::super::{Claimed, Failure, claim_quiesced};
use super::Chip;
use crate::bus::{
    dma::{
        mapping::{DmaDomain, DmaError, DomainMapping},
        memory::PageSource,
        pages::DmaPages,
    },
    mmio::owned::{self, Mapping},
    pci::{
        Address, Bar, ConfigSpace, Function,
        owned::{OwnedConfig, test_support::fixture as config_fixture},
        regs,
        requester::{
            FunctionRole, RequesterBinding,
            preparation::{AttachmentBackend, PreparingRequester, Root},
        },
    },
};
use crate::probe::{Error, NativeServices, ServiceError, services::tests::Refusing};
use alloc::{boxed::Box, rc::Rc, sync::Arc, vec::Vec};
use core::{cell::RefCell, ops::Range};
use std::sync::Mutex;

const ADDRESS: Address = Address {
    segment: 0,
    bus: 3,
    device: 0,
    function: 0,
};

/// What the original root saw.
#[derive(Default)]
struct RootState {
    events: Mutex<Vec<&'static str>>,
}

struct OldRoot(Arc<RootState>);
// SAFETY: an isolated symbolic root: it reaches no device, and its binding
// stays kept by the attachment it returns.
unsafe impl Root for OldRoot {
    fn drain(&self) -> Result<(), DmaError> {
        self.0.events.lock().unwrap().push("drain");
        Ok(())
    }
    unsafe fn bind(
        &self,
        binding: RequesterBinding,
    ) -> Result<Box<dyn AttachmentBackend>, DmaError> {
        self.0.events.lock().unwrap().push("bind");
        Ok(Box::new(Bound(Some(binding))))
    }
}

struct Bound(Option<RequesterBinding>);
impl Drop for Bound {
    fn drop(&mut self) {
        core::mem::forget(self.0.take());
    }
}
// SAFETY: the symbolic attachment keeps its binding and never detaches.
unsafe impl AttachmentBackend for Bound {
    fn domain(&self) -> Arc<dyn DmaDomain> {
        Arc::new(NoDomain)
    }
    unsafe fn detach(
        &mut self,
    ) -> Result<(), crate::bus::pci::requester::preparation::DetachError> {
        Err(DmaError::Unmapped.into())
    }
}

/// A domain that maps nothing.
struct NoDomain;
// SAFETY: it refuses every mapping, so no device address exists.
unsafe impl DmaDomain for NoDomain {
    unsafe fn map(
        &self,
        _: &DmaPages,
        _: crate::bus::dma::mapping::Direction,
        _: u64,
    ) -> Result<DomainMapping, DmaError> {
        Err(DmaError::MappingFailed)
    }
    unsafe fn unmap(&self, _: DomainMapping) -> Result<(), DmaError> {
        Err(DmaError::Unmapped)
    }
    unsafe fn sync_for_device(
        &self,
        _: DomainMapping,
        _: Range<usize>,
        _: crate::bus::dma::mapping::Direction,
    ) -> Result<(), DmaError> {
        Err(DmaError::Unmapped)
    }
    unsafe fn sync_for_cpu(
        &self,
        _: DomainMapping,
        _: Range<usize>,
        _: crate::bus::dma::mapping::Direction,
    ) -> Result<(), DmaError> {
        Err(DmaError::Unmapped)
    }
}

/// A mount that hands out one cold requester over the model, and refuses
/// everything else as [`Refusing`] does.
struct Mount {
    requester: RefCell<Option<PreparingRequester>>,
    refusing: Refusing,
}

// SAFETY: the one preparation is the model's sole owner of its symbolic
// configuration, parked with Bus Master clear and sealed; every other
// service refuses.
unsafe impl NativeServices for Mount {
    fn minimum_delay(&self) -> Result<Rc<dyn crate::bus::pci::power::Timer>, ServiceError> {
        self.refusing.minimum_delay()
    }
    fn prepare_requester(&self, _: Address) -> Result<PreparingRequester, ServiceError> {
        self.requester
            .borrow_mut()
            .take()
            .ok_or(ServiceError::Unavailable)
    }
    fn bridge(&self, address: Address) -> Result<Function<OwnedConfig>, ServiceError> {
        self.refusing.bridge(address)
    }
    fn map_bar(&self, address: Address, bar: Bar) -> Result<Mapping, ServiceError> {
        self.refusing.map_bar(address, bar)
    }
    fn bar_resource(&self, address: Address, bar: Bar) -> Result<owned::Resource, ServiceError> {
        self.refusing.bar_resource(address, bar)
    }
    fn authority(&self) -> Result<owned::Authority, ServiceError> {
        self.refusing.authority()
    }
    fn domain(&self, binding: RequesterBinding) -> Result<Arc<dyn DmaDomain>, ServiceError> {
        self.refusing.domain(binding)
    }
    fn pages(&self) -> &'static dyn PageSource {
        self.refusing.pages()
    }
    fn interrupts(&self) -> Result<&'static dyn crate::bus::irq::IrqController, ServiceError> {
        self.refusing.interrupts()
    }
    #[cfg(feature = "native-acpi")]
    fn acpi_memory(
        &self,
        device: &crate::platform::acpi::resources::DeviceResources<'_>,
        ordinal: usize,
    ) -> Result<owned::Resource, ServiceError> {
        self.refusing.acpi_memory(device, ordinal)
    }
    #[cfg(feature = "native-acpi")]
    fn acpi(&self) -> Option<&'static crate::platform::acpi::Session> {
        None
    }
    #[cfg(feature = "native-acpi")]
    fn firmware_memory(
        &self,
        physical: u64,
        bytes: usize,
    ) -> Result<core::ptr::NonNull<u8>, ServiceError> {
        self.refusing.firmware_memory(physical, bytes)
    }
}

/// A parked Wi-Fi function `vendor:device` whose Device Capabilities hold
/// `devcap`, under the mount's cold preparation, with no bus reset offered.
fn mount(vendor: u16, device: u16, devcap: u32) -> (Mount, Arc<RootState>) {
    let (mut config, _) = config_fixture(ADDRESS);
    config
        .write32(0, u32::from(vendor) | u32::from(device) << 16)
        .unwrap();
    config.write32(8, 0x0280_0000).unwrap();
    config.write16(regs::COMMAND, 0x402).unwrap();
    config.write16(regs::STATUS, 0x10).unwrap();
    config.write8(0x34, 0x40).unwrap();
    config.write32(0x40, 0x0002_0010).unwrap();
    config.write32(0x44, devcap).unwrap();
    config.write32(0x10, 0x4000_0000).unwrap();
    let function = Function::new(ADDRESS, config).unwrap();
    let root = Arc::new(RootState::default());
    // SAFETY: an independent owned configuration model and a symbolic
    // original root; nothing reaches hardware.
    let requester = match unsafe {
        PreparingRequester::new(
            function,
            FunctionRole::Physical,
            Box::new(OldRoot(root.clone())),
        )
    } {
        Ok(requester) => requester,
        Err(_) => panic!("valid owned configuration fixture"),
    };
    (
        Mount {
            requester: RefCell::new(Some(requester)),
            refusing: Refusing::minimum_only(),
        },
        root,
    )
}

fn claimed(result: Result<Claimed, Failure>) -> Claimed {
    match result {
        Ok(claimed) => claimed,
        Err((error, _)) => panic!("{error}"),
    }
}

/// An AX210 with FLR is reset, its root drained and bound, and its seal
/// released: the Bus Master its driver sets is allowed, and was clear until
/// then.
#[test]
fn a_reset_radio_is_bound_and_may_master() {
    let (mount, root) = mount(0x8086, 0x2725, 1 << 28);
    let mut claimed = claimed(claim_quiesced(&mount, ADDRESS, |_| Ok(Chip::Intel)));
    assert_eq!(claimed.quiesced.reset, super::Reset::FunctionLevel);
    assert_eq!(*root.events.lock().unwrap(), ["drain", "bind"]);
    let command = claimed.function.config().read16(regs::COMMAND).unwrap();
    assert_eq!(command & regs::COMMAND_MASTER, 0);
    claimed.function.enable_polled().unwrap();
    let command = claimed.function.config().read16(regs::COMMAND).unwrap();
    assert_ne!(command & regs::COMMAND_MASTER, 0);
}

/// The kept preparation of a refused claim, its seal still set.
fn kept(failure: Failure) -> (Error, PreparingRequester) {
    let (error, kept) = failure;
    let kept = kept.expect("the preparation is kept");
    match kept.downcast::<PreparingRequester>() {
        Ok(requester) => (error, *requester),
        Err(_) => panic!("the kept owner is the preparation"),
    }
}

/// An MT7925 with no bus reset on offer stays sealed: its root is never
/// drained or bound, and Bus Master is still refused.
#[test]
fn an_unprovable_radio_keeps_its_seal() {
    let (mount, root) = mount(0x14c3, 0x7925, 1 << 28);
    let Err(failure) = claim_quiesced(&mount, ADDRESS, |_| Ok(Chip::Mt7925)) else {
        panic!("an MT7925 without a bus reset was claimed");
    };
    let (error, mut requester) = kept(failure);
    assert!(matches!(
        error,
        Error::WifiClaim(Chip::Mt7925, super::Error::NoReset(super::Flr::Broken))
    ));
    assert_eq!(
        alloc::format!("{error}"),
        "Wi-Fi claim: MT7925: kept sealed, Bus Master stays off: nothing proves its engines \
         stopped: its Function Level Reset is broken (Linux quirk_no_flr), and no Secondary Bus \
         Reset of its bus is offered (it sits on a root bus, or another device or a bridge shares \
         its bus)"
    );
    assert!(root.events.lock().unwrap().is_empty());
    assert!(requester.function_mut().enable_polled().is_err());
    let command = requester
        .function_mut()
        .config()
        .read16(regs::COMMAND)
        .unwrap();
    assert_eq!(command & regs::COMMAND_MASTER, 0);
}

/// A function its driver would refuse is refused before any reset, and
/// keeps its seal.
#[test]
fn a_refused_radio_is_not_reset() {
    let (mount, root) = mount(0x8086, 0x9df0, 1 << 28);
    let Err(failure) = claim_quiesced(&mount, ADDRESS, |_| {
        Err(Error::Native(ServiceError::Refused("not this radio")))
    }) else {
        panic!("a refused radio was claimed");
    };
    let (error, mut requester) = kept(failure);
    assert!(matches!(
        error,
        Error::Native(ServiceError::Refused("not this radio"))
    ));
    assert!(root.events.lock().unwrap().is_empty());
    // No FLR was initiated: Device Control was never written.
    let control = requester.function_mut().config().read16(0x48).unwrap();
    assert_eq!(control, 0);
    assert!(requester.function_mut().enable_polled().is_err());
}

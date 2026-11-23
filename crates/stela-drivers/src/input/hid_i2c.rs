// SPDX-License-Identifier: GPL-2.0-only
//! Standard HID-over-I2C transport, following Linux i2c-hid-core.c.
//!
//! Copyright (c) 2012 Benjamin Tissoires, Ecole Nationale de l'Aviation Civile
//! and Red Hat, Inc. Notices and upstream source hashes are in
//! NOTICE. This is a native Stela backend, not a Linux shim.
//!
//! The platform supplies an exclusively owned, powered I2C attachment, the ACPI
//! descriptor register and the interrupt pin. The CPU route stays masked
//! and drained; this polled backend observes the pin itself. Reset completes
//! by a new zero-length IRQ acknowledgement, except source-checked
//! no-reset-IRQ devices that explicitly expose their separate delay contract.
//! A device without a readable pin is polled on its input register instead,
//! with the same delay standing in for the reset interrupt.
//! Raw HID reports and the exact report descriptor pass to a shared HID consumer;
//! no keyboard, touchpad or touchscreen descriptor is substituted here.
use super::{Input, ptp};
use crate::bus::i2c::{self, Address, Bus, Operation, Timer};
use crate::bus::pci::power::{self, Timer as Minimum};
use crate::usb::hid::{ParseError, Translated, Translator};
use alloc::{rc::Rc, string::String, vec::Vec};
use core::mem::ManuallyDrop;
mod refusal;
pub use refusal::{Refusal, RefusalCause};

mod descriptor;
pub use descriptor::Descriptor;
use descriptor::Quirks;

const DESCRIPTOR_BYTES: usize = 30;
const MAX_DESCRIPTOR: usize = 4096;
const MAX_BUFFER: usize = 16384;
const RESET: u8 = 1;
const GET_REPORT: u8 = 2;
const SET_REPORT: u8 = 3;
const SET_POWER: u8 = 8;
const WAKE_DELAY_US: u64 = 60_000;
const ADDRESS_WAKE_DELAY_US: u64 = 400;
const GOODIX_RESUME_DELAY_US: u64 = 1_500_000;
const QUIRK_RESET_DELAY_US: u64 = 100_000;

/// Platform IRQ provider. Masking must acknowledge the CPU route's
/// mask/drain, not change a local boolean. `asserted` samples this
/// attachment's physical active level even while the CPU route is masked.
///
/// A device whose only interrupt the platform cannot sample is polled: its
/// provider says so through [`Interrupt::polled`], and `asserted` then says
/// when the input register is due for its next read.
pub trait Interrupt {
    fn mask_and_drain(&mut self) -> Result<(), InterruptError>;
    fn asserted(&mut self) -> Result<bool, InterruptError>;
    /// No pin backs `asserted`. An empty input register then reads as a
    /// zero length, which means no report rather than a reset.
    fn polled(&self) -> bool {
        false
    }
}
impl<I: Interrupt + ?Sized> Interrupt for alloc::boxed::Box<I> {
    fn mask_and_drain(&mut self) -> Result<(), InterruptError> {
        (**self).mask_and_drain()
    }
    fn asserted(&mut self) -> Result<bool, InterruptError> {
        (**self).asserted()
    }
    fn polled(&self) -> bool {
        (**self).polled()
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InterruptError {
    Unavailable,
    Hardware(u32),
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReportType {
    Input = 1,
    Output = 2,
    Feature = 3,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PowerState {
    Uninitialized,
    On,
    Sleep,
    SuspendedAwake,
    Stopped,
    Uncertain,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorKind {
    Clock(crate::ClockError),
    Minimum(power::Error),
    Interrupted,
    InvalidAddress,
    InvalidConfiguration,
    NotReady,
    AlreadyInitialized,
    NeedsReset,
    Bus(i2c::Error),
    Interrupt(InterruptError),
    Allocation,
    DescriptorLength(u16),
    Version(u16),
    ReportDescriptorLength(u16),
    BufferLength(u16),
    ReportLength { actual: usize, maximum: usize },
    ReportId { expected: u8, actual: u8 },
    UnsupportedReportType,
    NoOutputRegister,
    ResetTimeout,
    InputDrainTimeout,
    IdentityChanged,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Recovery {
    NotAttempted,
    IrqMasked,
    Interrupted,
    Failed(InterruptError),
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Error {
    pub kind: ErrorKind,
    pub recovery: Recovery,
}
impl From<ErrorKind> for Error {
    fn from(kind: ErrorKind) -> Self {
        Self {
            kind,
            recovery: Recovery::NotAttempted,
        }
    }
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "I2C HID {:?} (recovery {:?})", self.kind, self.recovery)
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Fresh,
    Ready,
    Sleeping,
    SuspendedAwake,
    Stopped,
    Faulted,
}
/// One observed input result. No-data and ELAN's documented bogus interrupt are
/// visible outcomes, not empty invented HID reports.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InputReport {
    Data(Vec<u8>),
    Empty,
    SpuriousInterrupt,
    /// A zero-length input on the asserted pin with no reset pending: a reset
    /// completed, the device's own or a late response to the host's. Linux
    /// `i2c_hid_get_input` drops it; the device stays ready, and a device
    /// that reset itself is back in its power-on state, as a touchpad in its
    /// firmware's mouse mode.
    Reset,
    /// Exact upstream BAD_INPUT_SIZE devices return the complete read buffer
    /// when their header is invalid. Preserve every byte and expose the
    /// original wire length; a consumer never mistakes this for a valid header.
    CorrectedLength {
        data: Vec<u8>,
        reported_length: u16,
        read_length: u16,
    },
}

/// Evidence for the most recent successful reset sequence. These outcomes are
/// distinct: a known no-IRQ device never claims an observed ACK.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResetCompletion {
    InterruptAcknowledgement,
    /// The exact pinned source profile omits a reset interrupt. The RESET write
    /// was accepted, the required 100ms elapsed, and post-reset power-on succeeded.
    DeviceQuirkDelay,
    /// The device is polled, so no interrupt can acknowledge the reset. The
    /// RESET write was accepted, Linux's 100ms no-reset-interrupt delay
    /// elapsed, and the input register was read until empty.
    PolledDelay,
}

/// Captured control-report bytes and the full HID length advertised on wire.
/// A caller may bound the read; truncation is never hidden.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ControlReport {
    pub data: Vec<u8>,
    pub reported_length: usize,
}
impl ControlReport {
    pub fn is_complete(&self) -> bool {
        self.data.len() == self.reported_length
    }
}

pub struct Device<B, I, T> {
    bus: B,
    interrupt: I,
    timer: T,
    minimum: Rc<dyn Minimum>,
    failure: Option<Error>,
    address: Address,
    descriptor_register: u16,
    reset_timeout_us: u64,
    state: State,
    descriptor: Option<Descriptor>,
    report_descriptor: Vec<u8>,
    input: Vec<u8>,
    quirks: Quirks,
    discarded_before_start: u64,
    reset_completion: Option<ResetCompletion>,
}
impl<B: Bus, I: Interrupt, T: Timer> Device<B, I, T> {
    /// No HID I/O. Every provider is parked before timing preflight callbacks;
    /// an owning refusal keeps that complete graph on error or abandonment.
    #[expect(
        clippy::result_large_err,
        reason = "the same original device accompanies refusal without error-path allocation"
    )]
    pub fn new(
        bus: B,
        interrupt: I,
        timer: T,
        minimum: Rc<dyn Minimum>,
        address: Address,
        descriptor_register: u16,
        reset_timeout_us: u64,
    ) -> Result<Self, Refusal<B, I, T>> {
        let device = ManuallyDrop::new(Self {
            bus,
            interrupt,
            timer,
            minimum,
            address,
            descriptor_register,
            reset_timeout_us,
            failure: None,
            state: State::Fresh,
            descriptor: None,
            report_descriptor: Vec::new(),
            input: Vec::new(),
            quirks: Quirks::default(),
            discarded_before_start: 0,
            reset_completion: None,
        });
        let checked = (|| {
            if !matches!(
                address,
                Address::SevenBit(0x08..=0x77) | Address::TenBit(0..=0x3ff)
            ) {
                return Err(ErrorKind::InvalidAddress);
            }
            if reset_timeout_us == 0 {
                return Err(ErrorKind::InvalidConfiguration);
            }
            device
                .timer
                .check_duration(reset_timeout_us.max(GOODIX_RESUME_DELAY_US))
                .map_err(ErrorKind::Clock)?;
            device.minimum.check().map_err(ErrorKind::Minimum)
        })();
        match checked {
            Ok(()) => Ok(ManuallyDrop::into_inner(device)),
            Err(kind) => Err(Refusal::transport(device, kind.into())),
        }
    }

    /// Initialize with the complete device parked across every callback.
    #[expect(
        clippy::result_large_err,
        reason = "refusal retains the original device without allocating during failure"
    )]
    pub fn initialize_owned(self) -> Result<Self, Refusal<B, I, T>> {
        let mut device = ManuallyDrop::new(self);
        match device.initialize() {
            Ok(()) => Ok(ManuallyDrop::into_inner(device)),
            Err(error) => Err(Refusal::transport(device, error)),
        }
    }

    pub fn first_error(&self) -> Option<Error> {
        self.failure
    }

    // Prearm denial before every foreign callback. Only returned success
    // restores admission; a callback unwind keeps the interruption cause.
    fn act<R>(
        &mut self,
        success: State,
        body: impl FnOnce(&mut Self) -> Result<R, ErrorKind>,
    ) -> Result<R, Error> {
        self.state = State::Faulted;
        self.failure = Some(ErrorKind::Interrupted.into());
        let result = self
            .minimum
            .check()
            .map_err(ErrorKind::Minimum)
            .and_then(|()| body(self));
        match result {
            Ok(value) => {
                self.state = success;
                self.failure = None;
                Ok(value)
            }
            Err(kind) => Err(self.fail(kind)),
        }
    }
    pub fn is_ready(&self) -> bool {
        self.state == State::Ready
    }
    pub fn needs_reset(&self) -> bool {
        self.state == State::Faulted
    }
    pub fn power_state(&self) -> PowerState {
        match self.state {
            State::Fresh => PowerState::Uninitialized,
            State::Ready => PowerState::On,
            State::Sleeping => PowerState::Sleep,
            State::SuspendedAwake => PowerState::SuspendedAwake,
            State::Stopped => PowerState::Stopped,
            State::Faulted => PowerState::Uncertain,
        }
    }
    pub fn descriptor(&self) -> Option<&Descriptor> {
        self.descriptor.as_ref()
    }
    /// The device's clock now, in microseconds.
    pub fn now_us(&self) -> u64 {
        self.timer.now_us()
    }
    pub fn report_descriptor(&self) -> &[u8] {
        &self.report_descriptor
    }
    pub fn discarded_before_start(&self) -> u64 {
        self.discarded_before_start
    }
    pub fn reset_completion(&self) -> Option<ResetCompletion> {
        self.reset_completion
    }
    fn ready(&self) -> Result<(), Error> {
        if self.state == State::Faulted {
            Err(ErrorKind::NeedsReset.into())
        } else if !self.is_ready() {
            Err(ErrorKind::NotReady.into())
        } else {
            Ok(())
        }
    }
    fn delay(&self, us: u64) -> Result<(), ErrorKind> {
        self.minimum.wait_us(us).map_err(ErrorKind::Minimum)
    }
    fn buffer(length: usize) -> Result<Vec<u8>, ErrorKind> {
        let mut bytes = Vec::new();
        bytes
            .try_reserve_exact(length)
            .map_err(|_| ErrorKind::Allocation)?;
        bytes.resize(length, 0);
        Ok(bytes)
    }
    fn transfer(&mut self, send: &[u8], receive: &mut [u8]) -> Result<(), ErrorKind> {
        let result = match (send.is_empty(), receive.is_empty()) {
            (false, false) => self.bus.transaction(
                self.address,
                &mut [Operation::Write(send), Operation::Read(receive)],
            ),
            (false, true) => self
                .bus
                .transaction(self.address, &mut [Operation::Write(send)]),
            (true, false) => self
                .bus
                .transaction(self.address, &mut [Operation::Read(receive)]),
            (true, true) => return Err(ErrorKind::InvalidConfiguration),
        };
        result.map_err(ErrorKind::Bus)
    }
    fn address_nack(error: ErrorKind) -> bool {
        matches!(
            error,
            ErrorKind::Bus(i2c::Error {
                kind: i2c::ErrorKind::Aborted {
                    kind: i2c::AbortKind::AddressNack,
                    ..
                },
                recovery: i2c::Recovery::Disabled
            })
        )
    }
    fn register(&mut self, register: u16, receive: &mut [u8]) -> Result<(), ErrorKind> {
        self.transfer(&register.to_le_bytes(), receive)
    }
    fn command(&self, opcode: u8, report_type: u8, id: u8) -> Result<Vec<u8>, ErrorKind> {
        let descriptor = self.descriptor.ok_or(ErrorKind::NotReady)?;
        let mut bytes = Self::buffer(if id < 15 { 4 } else { 5 })?;
        bytes[..2].copy_from_slice(&descriptor.command_register.to_le_bytes());
        bytes[2] = (report_type << 4) | id.min(15);
        bytes[3] = opcode;
        if id >= 15 {
            bytes[4] = id;
        }
        bytes
            .try_reserve_exact(2)
            .map_err(|_| ErrorKind::Allocation)?;
        Ok(bytes)
    }
    fn power(&mut self, on: bool) -> Result<(), ErrorKind> {
        let bytes = self.command(SET_POWER, 0, u8::from(!on))?;
        let result = self.transfer(&bytes, &mut []);
        if let Err(error) = result {
            if !on || !Self::address_nack(error) {
                return Err(error);
            }
            // The one-write command was address-NACKed, so no action reached
            // the device. No partially accepted command is replayed.
            self.delay(ADDRESS_WAKE_DELAY_US)?;
            self.transfer(&bytes, &mut [])?;
        }
        if on {
            self.delay(WAKE_DELAY_US)?;
        }
        Ok(())
    }
    fn read_input(&mut self) -> Result<u16, ErrorKind> {
        self.bus
            .transaction(self.address, &mut [Operation::Read(&mut self.input)])
            .map_err(ErrorKind::Bus)?;
        Ok(u16::from_le_bytes([self.input[0], self.input[1]]))
    }
    fn input_length(&self, length: u16) -> Result<usize, ErrorKind> {
        if length < 2 || usize::from(length) > self.input.len() {
            if self.quirks.bad_input_size {
                return Ok(self.input.len());
            }
            return Err(ErrorKind::ReportLength {
                actual: usize::from(length),
                maximum: self.input.len(),
            });
        }
        Ok(usize::from(length))
    }
    // An interrupt route drain does not consume the peripheral's report FIFO.
    // Establish a deasserted pin epoch before RESET so a stale zero-length
    // response cannot acknowledge the new command. Unknown pre-start reports
    // are counted and discarded, never delivered to a consumer.
    fn drain_before_reset(&mut self) -> Result<(), ErrorKind> {
        let start = self.timer.start();
        if self.interrupt.polled() {
            // Without a pin, the register itself says when it is empty.
            loop {
                let length = self.read_input()?;
                if length == 0 {
                    return Ok(());
                }
                if !(self.quirks.elan && length == u16::MAX) {
                    self.input_length(length)?;
                }
                self.discarded_before_start = self.discarded_before_start.saturating_add(1);
                if self.timer.elapsed_us(start) >= self.reset_timeout_us {
                    return Err(ErrorKind::InputDrainTimeout);
                }
                self.timer.relax();
            }
        }
        while self.interrupt.asserted().map_err(ErrorKind::Interrupt)? {
            let length = self.read_input()?;
            if length != 0 && !(self.quirks.elan && length == u16::MAX) {
                self.input_length(length)?;
            }
            self.discarded_before_start = self.discarded_before_start.saturating_add(1);
            if self.timer.elapsed_us(start) >= self.reset_timeout_us {
                return Err(ErrorKind::InputDrainTimeout);
            }
            self.timer.relax();
        }
        Ok(())
    }
    fn reset(&mut self) -> Result<(), ErrorKind> {
        self.reset_completion = None;
        self.power(true)?;
        self.drain_before_reset()?;
        let bytes = self.command(RESET, 0, 0)?;
        self.transfer(&bytes, &mut [])?;
        let completion = if self.interrupt.polled() {
            // As Linux's no-reset-interrupt quirk: wait, then take the
            // reset response and anything after it.
            self.delay(QUIRK_RESET_DELAY_US)?;
            self.drain_before_reset()?;
            ResetCompletion::PolledDelay
        } else if self.quirks.no_reset_irq {
            // Linux i2c_hid_finish_hwreset uses this exact 100ms contract only
            // for the listed device identities. Do not relabel it as an IRQ ACK.
            self.delay(QUIRK_RESET_DELAY_US)?;
            // Consume any response/input accumulated during the delay;
            // a stale delayed reset response cannot become guest input later.
            self.drain_before_reset()?;
            ResetCompletion::DeviceQuirkDelay
        } else {
            let start = self.timer.start();
            loop {
                if self.interrupt.asserted().map_err(ErrorKind::Interrupt)? {
                    let length = self.read_input()?;
                    if length == 0 {
                        break;
                    }
                    if !(self.quirks.elan && length == u16::MAX) {
                        self.input_length(length)?;
                    }
                    self.discarded_before_start = self.discarded_before_start.saturating_add(1);
                }
                if self.timer.elapsed_us(start) >= self.reset_timeout_us {
                    return Err(ErrorKind::ResetTimeout);
                }
                self.timer.relax();
            }
            ResetCompletion::InterruptAcknowledgement
        };
        if !self.quirks.elan {
            self.power(true)?;
        }
        self.reset_completion = Some(completion);
        Ok(())
    }
    fn fail(&mut self, kind: ErrorKind) -> Error {
        self.state = State::Faulted;
        let mut error: Error = kind.into();
        self.failure = Some(error);
        // Timing uncertainty authorizes no cleanup or retry callbacks.
        if !matches!(kind, ErrorKind::Minimum(_)) {
            error.recovery = Recovery::Interrupted;
            self.failure = Some(error);
            error.recovery = match self.interrupt.mask_and_drain() {
                Ok(()) => Recovery::IrqMasked,
                Err(cause) => Recovery::Failed(cause),
            };
            self.failure = Some(error);
        }
        error
    }
    pub fn initialize(&mut self) -> Result<(), Error> {
        if self.is_ready() {
            return Err(ErrorKind::AlreadyInitialized.into());
        }
        if !matches!(self.state, State::Fresh | State::Stopped) {
            return Err(ErrorKind::NeedsReset.into());
        }
        self.act(State::Ready, Self::start)
    }
    /// Read the descriptor, reset as `i2c_hid_hwreset` does (power on,
    /// RESET, then wait for the reset response on the pin) and read the
    /// report descriptor. A device started before keeps both descriptors.
    fn start(&mut self) -> Result<(), ErrorKind> {
        self.interrupt
            .mask_and_drain()
            .map_err(ErrorKind::Interrupt)?;
        let mut bytes = [0; DESCRIPTOR_BYTES];
        if let Err(error) = self.register(self.descriptor_register, &mut bytes) {
            if !Self::address_nack(error) {
                return Err(error);
            }
            self.delay(ADDRESS_WAKE_DELAY_US)?;
            self.register(self.descriptor_register, &mut bytes)?;
        }
        let descriptor = Descriptor::parse(&bytes)?;
        if self.descriptor.is_some_and(|old| old != descriptor) {
            return Err(ErrorKind::IdentityChanged);
        }
        self.quirks = Quirks::for_device(descriptor);
        self.input = Self::buffer(usize::from(descriptor.max_input_length))?;
        self.descriptor = Some(descriptor);
        self.reset()?;
        let mut report = Self::buffer(usize::from(descriptor.report_descriptor_length))?;
        self.register(descriptor.report_descriptor_register, &mut report)?;
        if !self.report_descriptor.is_empty() && self.report_descriptor != report {
            return Err(ErrorKind::IdentityChanged);
        }
        self.report_descriptor = report;
        if self.quirks.repower {
            self.power(true)?;
        }
        Ok(())
    }
    /// Start a started device again, ready or failed: power it on, reset it
    /// and wait for the reset response, and read both descriptors again,
    /// which must be unchanged. This puts a device that reset itself, or
    /// whose command failed, back in its power-on state. A failure of the
    /// timing source, or a callback that never returned, authorizes nothing.
    pub fn reinitialize(&mut self) -> Result<(), Error> {
        if self.descriptor.is_none() {
            return Err(ErrorKind::NotReady.into());
        }
        match self.state {
            State::Ready => {}
            State::Faulted
                if !matches!(
                    self.failure.map(|failure| failure.kind),
                    None | Some(ErrorKind::Minimum(_) | ErrorKind::Interrupted)
                ) => {}
            State::Faulted => return Err(ErrorKind::NeedsReset.into()),
            _ => return Err(ErrorKind::NotReady.into()),
        }
        self.act(State::Ready, Self::start)
    }
    /// Watch the pin for `us` after a command that can make the device
    /// reset, as a touchpad's mode switch: whether a reset response (a
    /// zero-length input) came, which ends the watch. Reports in the window
    /// are discarded and counted with those before start. A polled device
    /// cannot tell a reset from its empty register: it waits the window out
    /// and drains its register.
    pub fn settle(&mut self, us: u64) -> Result<bool, Error> {
        self.ready()?;
        self.act(State::Ready, |this| {
            if this.interrupt.polled() {
                this.delay(us)?;
                this.drain_before_reset()?;
                return Ok(false);
            }
            let start = this.timer.start();
            while this.timer.elapsed_us(start) < us {
                if this.interrupt.asserted().map_err(ErrorKind::Interrupt)? {
                    let length = this.read_input()?;
                    if length == 0 {
                        return Ok(true);
                    }
                    if !(this.quirks.elan && length == u16::MAX) {
                        this.input_length(length)?;
                    }
                    this.discarded_before_start = this.discarded_before_start.saturating_add(1);
                }
                this.timer.relax();
            }
            Ok(false)
        })
    }
    /// Read only when the device IRQ pin is asserted, or a polled device is
    /// due. Returned HID bytes
    /// include the report ID if the device's descriptor uses numbered reports.
    pub fn poll(&mut self) -> Result<Option<InputReport>, Error> {
        self.ready()?;
        self.act(State::Ready, |this| {
            if !this.interrupt.asserted().map_err(ErrorKind::Interrupt)? {
                return Ok(None);
            }
            let length = this.read_input()?;
            if length == 0 {
                // A polled device answers an empty register with no length;
                // only an asserted pin makes it a reset. Linux
                // `i2c_hid_get_input` takes it as a host or device initiated
                // reset completing and drops it: never a failure.
                if this.interrupt.polled() {
                    return Ok(None);
                }
                return Ok(Some(InputReport::Reset));
            }
            if this.quirks.elan && length == u16::MAX {
                return Ok(Some(InputReport::SpuriousInterrupt));
            }
            let effective = this.input_length(length)?;
            if length == 2 {
                return Ok(Some(InputReport::Empty));
            }
            let mut report = Self::buffer(effective - 2)?;
            report.copy_from_slice(&this.input[2..effective]);
            Ok(Some(if effective == usize::from(length) {
                InputReport::Data(report)
            } else {
                InputReport::CorrectedLength {
                    data: report,
                    reported_length: length,
                    read_length: effective as u16,
                }
            }))
        })
    }
    /// GET_REPORT returns raw HID data (including the report ID when nonzero).
    /// A bounded read can capture a prefix of a larger device report, as Linux
    /// permits. The returned full length makes that truncation explicit.
    pub fn get_report(
        &mut self,
        kind: ReportType,
        id: u8,
        maximum: usize,
    ) -> Result<ControlReport, Error> {
        self.ready()?;
        if maximum == 0 || maximum > MAX_BUFFER - 2 {
            return Err(ErrorKind::InvalidConfiguration.into());
        }
        self.act(State::Ready, |this| {
            let mut command = this.command(GET_REPORT, kind as u8, id)?;
            command.extend_from_slice(
                &this
                    .descriptor
                    .ok_or(ErrorKind::NotReady)?
                    .data_register
                    .to_le_bytes(),
            );
            let mut bytes = Self::buffer(maximum + 2)?;
            this.transfer(&command, &mut bytes)?;
            let length = usize::from(u16::from_le_bytes([bytes[0], bytes[1]]));
            if length < 2 {
                return Err(ErrorKind::ReportLength {
                    actual: length,
                    maximum: bytes.len(),
                });
            }
            if id != 0 && length > 2 && bytes[2] != id {
                return Err(ErrorKind::ReportId {
                    expected: id,
                    actual: bytes[2],
                });
            }
            bytes.truncate(length.min(bytes.len()));
            bytes.drain(..2);
            Ok(ControlReport {
                data: bytes,
                reported_length: length - 2,
            })
        })
    }
    fn formatted_report(id: u8, data: &[u8]) -> Result<Vec<u8>, ErrorKind> {
        let length = 2 + usize::from(id != 0) + data.len();
        if length > MAX_BUFFER {
            return Err(ErrorKind::ReportLength {
                actual: length,
                maximum: MAX_BUFFER,
            });
        }
        let mut bytes = Self::buffer(length)?;
        bytes[..2].copy_from_slice(&(length as u16).to_le_bytes());
        let offset = if id != 0 {
            bytes[2] = id;
            3
        } else {
            2
        };
        bytes[offset..].copy_from_slice(data);
        Ok(bytes)
    }
    /// SET_REPORT payload excludes its report ID; the encoder supplies it once.
    pub fn set_report(&mut self, kind: ReportType, id: u8, data: &[u8]) -> Result<(), Error> {
        self.ready()?;
        if kind == ReportType::Input {
            return Err(ErrorKind::UnsupportedReportType.into());
        }
        let report = Self::formatted_report(id, data).map_err(Error::from)?;
        self.act(State::Ready, |this| {
            let mut command = this.command(SET_REPORT, kind as u8, id)?;
            command.extend_from_slice(
                &this
                    .descriptor
                    .ok_or(ErrorKind::NotReady)?
                    .data_register
                    .to_le_bytes(),
            );
            command
                .try_reserve_exact(report.len())
                .map_err(|_| ErrorKind::Allocation)?;
            command.extend_from_slice(&report);
            this.transfer(&command, &mut [])
        })
    }
    /// Plain output-register transfer. A missing output register is explicit;
    /// callers may choose the separate SET_REPORT operation if appropriate.
    pub fn output(&mut self, id: u8, data: &[u8]) -> Result<(), Error> {
        self.ready()?;
        let descriptor = self.descriptor.ok_or(ErrorKind::NotReady)?;
        if descriptor.max_output_length == 0 {
            return Err(ErrorKind::NoOutputRegister.into());
        }
        let report = Self::formatted_report(id, data).map_err(Error::from)?;
        if report.len() > usize::from(descriptor.max_output_length) {
            return Err(ErrorKind::ReportLength {
                actual: report.len(),
                maximum: usize::from(descriptor.max_output_length),
            }
            .into());
        }
        let mut command = Self::buffer(2 + report.len()).map_err(Error::from)?;
        command[..2].copy_from_slice(&descriptor.output_register.to_le_bytes());
        command[2..].copy_from_slice(&report);
        self.act(State::Ready, |this| this.transfer(&command, &mut []))
    }
    pub fn sleep(&mut self) -> Result<(), Error> {
        self.ready()?;
        let state = if self.quirks.no_sleep {
            State::SuspendedAwake
        } else {
            State::Sleeping
        };
        self.act(state, |this| {
            this.interrupt
                .mask_and_drain()
                .map_err(ErrorKind::Interrupt)?;
            if this.quirks.no_sleep {
                Ok(())
            } else {
                this.power(false)
            }
        })
    }
    pub fn wake(&mut self) -> Result<(), Error> {
        if !matches!(self.state, State::Sleeping | State::SuspendedAwake) {
            return Err(if self.needs_reset() {
                ErrorKind::NeedsReset
            } else {
                ErrorKind::NotReady
            }
            .into());
        }
        self.act(State::Ready, |this| {
            if this.quirks.delayed_resume {
                this.delay(GOODIX_RESUME_DELAY_US)?;
            }
            if this.quirks.reset_resume {
                this.reset()
            } else {
                this.power(true)
            }
        })
    }
    pub fn stop(&mut self) -> Result<(), Error> {
        if self.is_ready() {
            self.sleep()?;
        }
        if !matches!(self.state, State::Sleeping | State::SuspendedAwake) {
            return Err(if self.needs_reset() {
                ErrorKind::NeedsReset
            } else {
                ErrorKind::NotReady
            }
            .into());
        }
        self.act(State::Stopped, |this| {
            this.interrupt
                .mask_and_drain()
                .map_err(ErrorKind::Interrupt)
        })
    }
}

/// Input reports one poll reads at most, as USB HID bounds its pass.
const POLL_REPORTS: usize = 16;
/// How long a touchpad's mode switch is watched for the reset it can cause:
/// Linux's no-reset-interrupt delay.
const MODE_SETTLE_US: u64 = QUIRK_RESET_DELAY_US;
/// Mode losses a touchpad is set back to touchpad mode for within
/// [`RESTORE_WINDOW_US`]; one more reads it as a mouse.
const MAX_RESTORES: u32 = 3;
const RESTORE_WINDOW_US: u64 = 10_000_000;

/// Why a precision touchpad is read in its firmware's mouse mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fallback {
    /// A mode report, or the watch after it, failed.
    Switch(Error),
    /// Starting the pad again after it lost the mode failed.
    Restart(Error),
    /// The pad reset and lost the mode again after it was switched back.
    ResetAfterSwitch,
    /// The pad lost the mode this many times within the restore window.
    Resets(u32),
}

impl core::fmt::Display for Fallback {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Switch(error) => write!(f, "the switch failed: {error}"),
            Self::Restart(error) => write!(f, "starting it again failed: {error}"),
            Self::ResetAfterSwitch => f.write_str("it reset and lost the mode again once switched"),
            Self::Resets(count) => write!(
                f,
                "it reset and lost the mode {count} times in {} s",
                RESTORE_WINDOW_US / 1_000_000
            ),
        }
    }
}

/// What an I²C-HID device did on its own, for the log.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Notice {
    /// A reset response no command waited for, dropped as Linux drops it;
    /// a precision touchpad still read back touchpad mode. Said once.
    ResetDropped,
    /// The touchpad reset itself and lost touchpad mode; it was started
    /// again and switched back, the `count`th time in the restore window.
    ModeRestored { count: u32 },
    /// Touchpad mode was not kept: the pad is read in its firmware's mouse
    /// mode from now on, so it still moves the pointer.
    MouseMode(Fallback),
}

impl core::fmt::Display for Notice {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::ResetDropped => f.write_str(
                "a reset response no command waited for (zero-length input), dropped as Linux \
                 i2c_hid_get_input drops it",
            ),
            Self::ModeRestored { count } => write!(
                f,
                "the touchpad reset itself and lost touchpad mode: started again and switched \
                 back (restore {count})"
            ),
            Self::MouseMode(cause) => write!(
                f,
                "precision touchpad mode not kept ({cause}): read as a mouse in its firmware's \
                 mouse mode"
            ),
        }
    }
}

/// An I²C-HID keyboard or pointer: the initialized device and the HID
/// report translator its input reports pass through.
/// Abandonment retains the device graph; transport sleep is not platform detach.
pub struct HidInput<B, I, T> {
    device: ManuallyDrop<Device<B, I, T>>,
    translator: Translator,
    /// A precision touchpad switched to report its fingers.
    ptp: Option<ptp::Ptp>,
    pub name: String,
    notices: Vec<Notice>,
    reset_dropped: bool,
    /// Mode restores in the window that started at `restored_at`.
    restores: u32,
    restored_at: u64,
}

impl<B: Bus, I: Interrupt, T: Timer> HidInput<B, I, T> {
    /// Translate the initialized device's reports. A parse refusal owns the
    /// same device and descriptor alongside the precise parser diagnostic.
    #[allow(
        clippy::result_large_err,
        reason = "a device whose descriptor does not parse goes back to its owner"
    )]
    pub fn new(device: Device<B, I, T>, name: String) -> Result<Self, Refusal<B, I, T>> {
        let mut device = ManuallyDrop::new(device);
        match Translator::describe(device.report_descriptor()) {
            Ok(translator) => {
                // A precision touchpad goes to touchpad mode, as Linux
                // hid-multitouch's `mt_set_modes` sets it. A switch that
                // fails, or that the pad answers by resetting out of the
                // mode, starts the pad again in its firmware's mouse mode
                // and reads it as a mouse: it always moves the pointer.
                let mut notices = Vec::new();
                let ptp = match translator.descriptor().and_then(ptp::Profile::select) {
                    Some(profile) => match Self::switch(&mut device, &profile) {
                        Ok(()) => Some(ptp::Ptp::new(profile)),
                        Err(cause) => {
                            notices.push(Self::mouse_mode(&mut device, cause));
                            None
                        }
                    },
                    None => None,
                };
                let mut name = name;
                if ptp.is_some() {
                    name.push_str(" precision touchpad");
                }
                Ok(Self {
                    device,
                    translator,
                    ptp,
                    name,
                    notices,
                    reset_dropped: false,
                    restores: 0,
                    restored_at: 0,
                })
            }
            Err(error) => Err(Refusal::descriptor(device, error)),
        }
    }

    /// Set touchpad mode, then watch for the reset the switch can cause, as
    /// `i2c_hid_hwreset` waits for a reset response. A pad that reset and
    /// no longer reads back touchpad mode is started again and switched once
    /// more.
    fn switch(device: &mut Device<B, I, T>, profile: &ptp::Profile) -> Result<(), Fallback> {
        for attempt in 0..2 {
            if attempt != 0 {
                device.reinitialize().map_err(Fallback::Restart)?;
            }
            for (id, payload) in profile.mode_reports() {
                device
                    .set_report(ReportType::Feature, id, &payload)
                    .map_err(Fallback::Switch)?;
            }
            match device.settle(MODE_SETTLE_US) {
                Ok(false) => return Ok(()),
                Ok(true) if Self::holds_mode(device, profile) => return Ok(()),
                Ok(true) => {}
                Err(error) => return Err(Fallback::Switch(error)),
            }
        }
        Err(Fallback::ResetAfterSwitch)
    }

    /// Whether the pad reads back touchpad mode. A failed read leaves the
    /// device to be started again, as a lost mode does.
    fn holds_mode(device: &mut Device<B, I, T>, profile: &ptp::Profile) -> bool {
        let Some((id, bytes)) = profile.input_mode_report() else {
            return false;
        };
        let skip = usize::from(id != 0);
        device
            .get_report(ReportType::Feature, id, bytes + skip)
            .is_ok_and(|report| profile.holds_touchpad_mode(report.data.get(skip..).unwrap_or(&[])))
    }

    /// Start the pad again, which leaves it in its firmware's mouse mode, to
    /// be read as a mouse. A device that does not start again stays failed
    /// and is removed with its own error.
    fn mouse_mode(device: &mut Device<B, I, T>, cause: Fallback) -> Notice {
        let _ = device.reinitialize();
        Notice::MouseMode(cause)
    }

    /// A reset response no command waited for. A precision touchpad that
    /// lost touchpad mode with it is started again and switched back, within
    /// [`MAX_RESTORES`] in [`RESTORE_WINDOW_US`], else read as a mouse;
    /// any other device is left in its power-on state, as Linux leaves it.
    fn reset(&mut self, inputs: &mut Vec<Input>) {
        let Self {
            device,
            ptp,
            notices,
            reset_dropped,
            restores,
            restored_at,
            ..
        } = self;
        let Some(pad) = ptp
            .as_mut()
            .filter(|pad| !Self::holds_mode(device, pad.profile()))
        else {
            if !core::mem::replace(reset_dropped, true) {
                notices.push(Notice::ResetDropped);
            }
            return;
        };
        pad.release(inputs);
        let now = device.now_us();
        if *restores == 0 || now.wrapping_sub(*restored_at) >= RESTORE_WINDOW_US {
            *restores = 0;
            *restored_at = now;
        }
        *restores += 1;
        let restored = if *restores > MAX_RESTORES {
            Err(Fallback::Resets(*restores))
        } else {
            device
                .reinitialize()
                .map_err(Fallback::Restart)
                .and_then(|()| Self::switch(device, pad.profile()))
        };
        match restored {
            Ok(()) => notices.push(Notice::ModeRestored { count: *restores }),
            Err(cause) => {
                notices.push(Self::mouse_mode(device, cause));
                *ptp = None;
            }
        }
    }

    /// What the device did on its own since the last call, for the log.
    pub fn take_notices(&mut self) -> Vec<Notice> {
        core::mem::take(&mut self.notices)
    }

    /// Whether the device is a precision touchpad reporting its fingers.
    pub fn is_precision_touchpad(&self) -> bool {
        self.ptp.is_some()
    }

    /// Set a precision touchpad's frames mode ([`ptp::Ptp::set_frames`]);
    /// whether the device is one.
    pub fn set_touch_frames(&mut self, on: bool) -> bool {
        self.ptp.as_mut().map(|ptp| ptp.set_frames(on)).is_some()
    }

    /// The surface a precision touchpad's frames span.
    pub fn touch_surface(&self) -> Option<super::multitouch::Surface> {
        self.ptp.as_ref().map(ptp::Ptp::surface)
    }

    pub fn device(&self) -> &Device<B, I, T> {
        &self.device
    }

    /// Append the input changes of the reports the device has ready to
    /// `inputs`, up to [`POLL_REPORTS`] of them.
    pub fn poll(&mut self, inputs: &mut Vec<Input>) -> Result<(), Error> {
        let mut events = Vec::new();
        for _ in 0..POLL_REPORTS {
            match self.device.poll()? {
                None => break,
                Some(
                    InputReport::Data(report) | InputReport::CorrectedLength { data: report, .. },
                ) => {
                    let at = events.len();
                    let translated = self.translator.translate(&report, &mut events, inputs);
                    if let (Some(ptp), Translated::Data(frame), Some(descriptor)) =
                        (&mut self.ptp, &translated, self.translator.descriptor())
                    {
                        ptp.report(
                            descriptor,
                            frame,
                            &events[at..],
                            self.device.now_us(),
                            inputs,
                        );
                    }
                }
                Some(InputReport::Empty | InputReport::SpuriousInterrupt) => {}
                Some(InputReport::Reset) => {
                    self.reset(inputs);
                    if self.device.needs_reset() {
                        return Err(self
                            .device
                            .first_error()
                            .unwrap_or_else(|| ErrorKind::NeedsReset.into()));
                    }
                    break;
                }
            }
        }
        Ok(())
    }

    /// Send `leds` (input LED codes as bits) to the device's LED output
    /// report, as Linux `hidinput_led_worker` does: through the output
    /// register, else SET_REPORT when the device has none.
    pub fn set_leds(&mut self, leds: u16) -> Result<(), Error> {
        let Some((id, report)) = self.translator.led_report(leds) else {
            return Ok(());
        };
        let data = &report[usize::from(id != 0)..];
        match self.device.output(id, data) {
            Err(error) if error.kind == ErrorKind::NoOutputRegister => {
                self.device.set_report(ReportType::Output, id, data)
            }
            result => result,
        }
    }

    /// The last report received, as it arrived.
    pub fn last_report(&self) -> &[u8] {
        self.translator.last_report()
    }
}

#[cfg(test)]
mod tests;

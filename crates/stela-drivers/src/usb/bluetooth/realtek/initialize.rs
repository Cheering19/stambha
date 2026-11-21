// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2015 Endless Mobile, Inc.
//! Pinned btrtl USB register/ROM/security, exact-image download and version flow.
//! HCI synchronization, failure quarantine and owned observations are Stela glue.
use super::{
    Attempt, Configuration, ConfigurationEvidence, Container, ContainerVersion, DownloadResponse,
    Error, FirmwareDrop, Identity, LocalVersion, OwnedReport, Patch, Result, UsbIdentity, V2Patch,
    identity::btrtl_ic,
};
use crate::firmware::{ErrorKind as ResourceErrorKind, Provider, Requirement};
use crate::usb::bluetooth::{
    FirmwareReceipt, Hci, Io, Packet, State,
    command::{self, Response},
};
use alloc::{boxed::Box, collections::VecDeque, vec::Vec};
const READ_VERSION: u16 = 0x1001;
const READ_ADDRESS: u16 = 0x1009;
const REGISTER: u16 = 0xfc61;
const ROM: u16 = 0xfc6d;
const DOWNLOAD: u16 = 0xfc20;
const SUBVERSION: [u8; 5] = [0x10, 0x38, 4, 0x28, 0x80];
const REVISION: [u8; 5] = [0x10, 0x3a, 4, 0x28, 0x80];
const SECURITY: [u8; 5] = [0x10, 0xa4, 0xad, 0, 0xb0];
/// The vendor command that drops a running patch (btrtl.c:1151).
const DROP_FIRMWARE: u16 = 0xfc66;
/// `msleep(200)` after it (btrtl.c:1166).
const DROP_US: u64 = 200_000;
/// The events of that wait kept whole; the rest are counted.
const DROP_EVENTS: usize = 4;
const COMMAND_US: u64 = 5_000_000;
const ATTEMPT_US: u64 = 120_000_000;

#[expect(
    clippy::result_large_err,
    reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
)]
pub(super) fn run<B: Io>(hci: &mut Hci<B>, provider: &dyn Provider) -> Result<OwnedReport> {
    let usb = UsbIdentity::read(hci.binding.device());
    if usb.project().is_none() {
        return Err(Error::Unsupported("USB identity"));
    }
    hci.open()?;
    let deadline = hci.io.now_us().saturating_add(ATTEMPT_US);
    let mut engine = Engine {
        hci,
        pending: VecDeque::new(),
        deadline,
        attempt: Attempt::default(),
        credits: 1,
        outside: None,
    };
    match engine.run(usb, provider) {
        Ok(r) => Ok(r),
        Err(cause) => {
            let cause = Error::Attempt {
                cause: Box::new(cause),
                observation: Box::new(engine.attempt),
            };
            if engine.hci.state() == State::Open
                && let Err(cleanup) = engine.hci.close()
            {
                return Err(Error::Cleanup {
                    cause: Box::new(cause),
                    cleanup,
                });
            }
            Err(cause)
        }
    }
}
struct Engine<'a, B: Io> {
    hci: &'a mut Hci<B>,
    pending: VecDeque<Vec<u8>>,
    deadline: u64,
    attempt: Attempt,
    /// HCI commands the controller will take now (one after reset).
    credits: u8,
    /// A command sent outside the command flow (0xfc66) whose completion
    /// or status has not come: one may still come, during any later wait.
    outside: Option<u16>,
}
impl<B: Io> Engine<'_, B> {
    /// Whether `e` is the completion or status of the command sent outside
    /// the flow. Its credits only raise the count, as Linux's core takes the
    /// count of an event no awaited command matches and drops the rest
    /// (`handle_cmd_cnt_and_timer`, `hci_req_cmd_complete`); none more of
    /// it is awaited.
    fn outside_reply(&mut self, e: &[u8]) -> bool {
        let Some(outside) = self.outside else {
            return false;
        };
        match command::response(e, outside) {
            Ok(Response::Complete { credits, .. } | Response::Status { credits, .. }) => {
                self.credits = self.credits.max(credits);
                self.outside = None;
                true
            }
            _ => false,
        }
    }
    /// Take what the transport received into `pending`. ACL fails: no
    /// connection exists while a controller is brought up or scanned.
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    fn receive(&mut self) -> Result<()> {
        for packet in self.hci.poll()? {
            match packet {
                Packet::Event(e) => self.pending.push_back(e),
                Packet::Acl(a) | Packet::Diagnostic(a) => {
                    return Err(Error::protocol("ACL while initializing firmware", &a));
                }
            }
        }
        if self.pending.len() > 16 {
            return Err(Error::protocol("excess initialization events", &[]));
        }
        Ok(())
    }
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    fn command(&mut self, opcode: u16, parameters: &[u8]) -> Result<Vec<u8>> {
        // No other event is expected. After 0xfc66 dropped a running patch,
        // one no command awaits (the late reply of 0xfc66, a vendor event)
        // is kept with the drop's and passed over, as Linux's core passes
        // over it.
        let dropped = self.attempt.dropped.is_some();
        let (mut kept, mut count) = (Vec::new(), 0);
        let result = self.exchange(opcode, parameters, &mut |e| {
            if !dropped {
                return Err(Error::protocol("unexpected initialization event", e));
            }
            count += 1;
            if kept.len() < DROP_EVENTS {
                kept.push(e.to_vec());
            }
            Ok(())
        });
        if let Some(drop) = self.attempt.dropped.as_mut() {
            drop.event_count += count;
            let room = DROP_EVENTS.saturating_sub(drop.events.len());
            drop.events.extend(kept.into_iter().take(room));
        }
        result
    }
    /// One command and its completion, within the controller's credits; an
    /// event that is no command event goes to `other`, and so does the
    /// reply of a command sent outside the flow ([`Self::outside_reply`]).
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    fn exchange(
        &mut self,
        opcode: u16,
        parameters: &[u8],
        other: &mut dyn FnMut(&[u8]) -> Result<()>,
    ) -> Result<Vec<u8>> {
        if !self.pending.is_empty() {
            return Err(Error::protocol("stale command response", &self.pending[0]));
        }
        if self.hci.io.now_us() >= self.deadline {
            return Err(Error::Timeout("complete initialization attempt"));
        }
        let packet =
            command::packet(opcode, parameters).map_err(|why| Error::protocol(why, parameters))?;
        let deadline = core::cmp::min(
            self.deadline,
            self.hci.io.now_us().saturating_add(COMMAND_US),
        );
        let mut sent = false;
        loop {
            if !sent && self.credits > 0 {
                self.hci.submit_command(&packet)?;
                self.credits -= 1;
                sent = true;
            }
            self.receive()?;
            while let Some(e) = self.pending.pop_front() {
                if self.outside_reply(&e) {
                    other(&e)?;
                    continue;
                }
                match command::response(&e, opcode).map_err(|why| Error::protocol(why, &e))? {
                    Response::Credits(credits) => self.credits = credits,
                    Response::Complete { .. } | Response::Status { .. } if !sent => {
                        return Err(Error::protocol("command event before its command", &e));
                    }
                    Response::Complete { credits, returned } => {
                        self.credits = credits;
                        if returned[0] != 0 {
                            return Err(Error::Controller {
                                opcode,
                                status: returned[0],
                                parameters: returned.to_vec(),
                            });
                        }
                        return Ok(returned.to_vec());
                    }
                    Response::Status { credits, status } => {
                        self.credits = credits;
                        if status != 0 {
                            return Err(Error::Controller {
                                opcode,
                                status,
                                parameters: e[2..].to_vec(),
                            });
                        }
                    }
                    Response::Other => other(&e)?,
                }
            }
            if self.hci.io.now_us() >= deadline {
                return Err(Error::Timeout(if sent {
                    "genuine command completion"
                } else {
                    "genuine HCI command credits"
                }));
            }
            crate::waiting();
            core::hint::spin_loop();
        }
    }
    /// Wait until `until`, at most the attempt's deadline, still draining
    /// the event pipe. A NOP completion sets the credits. The reply of the
    /// command sent outside the flow ([`Self::outside_reply`]) goes to
    /// `other` with every event that is no command event. Any other command
    /// event is refused: no command is in flight.
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    fn idle(&mut self, until: u64, other: &mut dyn FnMut(&[u8]) -> Result<()>) -> Result<()> {
        let until = until.min(self.deadline);
        loop {
            self.receive()?;
            while let Some(e) = self.pending.pop_front() {
                if self.outside_reply(&e)
                    || !matches!(
                        e.first(),
                        Some(&(command::COMMAND_COMPLETE | command::COMMAND_STATUS))
                    )
                {
                    other(&e)?;
                    continue;
                }
                // Opcode 0 is the NOP's, so any other opcode is refused.
                match command::response(&e, 0).map_err(|why| Error::protocol(why, &e))? {
                    Response::Credits(credits) => self.credits = credits,
                    _ => return Err(Error::protocol("command event with none in flight", &e)),
                }
            }
            if self.hci.io.now_us() >= until {
                return Ok(());
            }
            crate::waiting();
            core::hint::spin_loop();
        }
    }
    /// btrtl.c:1140-1169: drop the running patch of a controller whose
    /// readings matched no btrtl IC. 0xfc66 goes out as `hdev->send` sends
    /// it, outside the command flow: no credit is taken and no completion
    /// awaited. Then `msleep(200)`, here a wait that keeps what arrives; a
    /// reply of 0xfc66 that comes later is kept and passed over then.
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    fn drop_firmware(
        &mut self,
        register_subversion: u16,
        register_revision: Option<u16>,
        version: LocalVersion,
    ) -> Result<()> {
        self.attempt.dropped = Some(FirmwareDrop {
            register_subversion,
            register_revision,
            version,
            events: Vec::new(),
            event_count: 0,
        });
        let packet =
            command::packet(DROP_FIRMWARE, &[]).map_err(|why| Error::protocol(why, &[]))?;
        self.hci.submit_command(&packet)?;
        self.outside = Some(DROP_FIRMWARE);
        let until = self.hci.io.now_us().saturating_add(DROP_US);
        let (mut events, mut count) = (Vec::new(), 0);
        let waited = self.idle(until, &mut |e| {
            count += 1;
            if events.len() < DROP_EVENTS {
                events.push(e.to_vec());
            }
            Ok(())
        });
        if let Some(dropped) = self.attempt.dropped.as_mut() {
            dropped.events = events;
            dropped.event_count = count;
        }
        waited
    }
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    fn register(&mut self, which: &[u8; 5]) -> Result<[u8; 2]> {
        let p = self.command(REGISTER, which)?;
        if p.len() != 3 {
            return Err(Error::protocol("register response length", &p));
        }
        Ok([p[1], p[2]])
    }
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    fn run(&mut self, usb: UsbIdentity, provider: &dyn Provider) -> Result<OwnedReport> {
        // btrtl_initialize's `check_version` (btrtl.c:1088-1169): readings
        // that match no btrtl IC are those of a running patch, which is
        // dropped once before the controller is identified again.
        let (subversion, revision, version) = loop {
            let subversion = u16::from_le_bytes(self.register(&SUBVERSION)?);
            self.attempt.register_subversion = Some(subversion);
            let revision = if subversion == 0x8822 {
                Some(u16::from_le_bytes(self.register(&REVISION)?))
            } else {
                None
            };
            self.attempt.register_revision = revision;
            let version = LocalVersion::parse(&self.command(READ_VERSION, &[])?)?;
            self.attempt.version = Some(version.clone());
            if btrtl_ic(subversion, revision, &version) {
                break (subversion, revision, version);
            }
            if self.attempt.dropped.is_some() {
                // btrtl.c:1171-1175: Linux then uploads nothing and keeps the
                // controller as it runs; such a controller is refused here.
                return Err(Error::Unsupported(
                    "readings match no btrtl IC, also after its running patch was dropped",
                ));
            }
            self.drop_firmware(subversion, revision, version)?;
        };
        let rom = self.command(ROM, &[])?;
        if rom.len() != 2 {
            return Err(Error::protocol("ROM version response length", &rom));
        }
        self.attempt.rom_version = Some(rom[1]);
        let security = self.register(&SECURITY)?;
        self.attempt.security = Some(security);
        let identity = Identity::select(usb, version, subversion, revision, rom[1], security)?;
        self.attempt.identity = Some(identity.clone());
        let images = identity.firmware_images();
        let (mut name, mut container) = images[0];
        let hardware = identity.usb.hardware_label();
        let mut revision = identity.revision_label_for(container);
        // Both immutable requirements are observed before propagating any error.
        // A failed integrity/target/config parse is never optional absence.
        let mut firmware = provider.load(Requirement {
            name,
            target: identity.target(container, &hardware, &revision),
            minimum_bytes: container.minimum_bytes(),
        });
        // btrtl.c:1196-1209: the next image only when this one did not
        // load. Here only when it is absent: a present image whose integrity
        // or target is refused is never replaced by another.
        for &next in &images[1..] {
            match &firmware {
                Err(error) if error.kind == ResourceErrorKind::Missing => {
                    self.attempt.fallback = Some(error.clone());
                }
                _ => break,
            }
            (name, container) = next;
            revision = identity.revision_label_for(container);
            firmware = provider.load(Requirement {
                name,
                target: identity.target(container, &hardware, &revision),
                minimum_bytes: container.minimum_bytes(),
            });
        }
        // The configuration is asked under the target of the image taken.
        let target = identity.target(container, &hardware, &revision);
        let configuration = provider.load(Requirement {
            name: identity.configuration_name(),
            target,
            minimum_bytes: 6,
        });
        if let Ok(image) = firmware.as_ref() {
            self.attempt.firmware = Some(FirmwareReceipt::from(*image));
        }
        self.attempt.configuration = Some(
            configuration
                .as_ref()
                .map(|image| FirmwareReceipt::from(*image))
                .map_err(Clone::clone),
        );
        let firmware = firmware.map_err(Error::Resource)?;
        let (config, configuration) = match configuration {
            Ok(image) => (
                Some(Configuration::parse(image.bytes())?),
                ConfigurationEvidence::Supplied(image.into()),
            ),
            Err(error)
                if !identity.configuration_required()
                    && error.kind == ResourceErrorKind::Missing =>
            {
                (None, ConfigurationEvidence::OptionalMissing(error))
            }
            Err(error) => return Err(Error::Resource(error)),
        };
        let (container_version, stage) = match container {
            Container::V2 => {
                let patch = V2Patch::parse(
                    firmware.bytes(),
                    identity.project,
                    identity.rom_version,
                    identity.security[0],
                )?;
                let version = ContainerVersion::V2(patch.metadata.version);
                let stage = patch.stage(config.as_ref());
                self.attempt.v2 = Some(patch.metadata);
                (version, stage)
            }
            Container::V1 => {
                let patch = Patch::parse(firmware.bytes(), identity.project, identity.rom_version)?;
                (
                    ContainerVersion::V1(patch.firmware_version),
                    patch.stage(config.as_ref()),
                )
            }
        };
        self.attempt.container_version = Some(container_version);
        for fragment in stage.fragments() {
            let mut p = Vec::with_capacity(fragment.data.len() + 1);
            p.push(fragment.index);
            p.extend_from_slice(fragment.data);
            let response = self.command(DOWNLOAD, &p)?;
            if response.len() != 2 {
                return Err(Error::protocol("download response length", &response));
            }
            self.attempt.download.push(DownloadResponse {
                sent_index: fragment.index,
                returned_index: response[1],
            });
        }
        // The donor uses the final download flag to transition the controller,
        // then rereads Local Version. It issues no separate/synthetic reset CC.
        let after = LocalVersion::parse(&self.command(READ_VERSION, &[])?)?;
        self.attempt.after = Some(after.clone());
        if after.manufacturer() != 0x005d
            || !(6..=14).contains(&after.hci_version())
            || !(6..=14).contains(&after.lmp_version())
            || after.lmp_subversion() == identity.version.lmp_subversion()
        {
            return Err(Error::protocol(
                "controller did not leave selected stock ROM firmware",
                &after.raw,
            ));
        }
        let p = self.command(READ_ADDRESS, &[])?;
        if p.len() != 7 {
            return Err(Error::protocol("BD_ADDR response length", &p));
        }
        let address: [u8; 6] = p[1..].try_into().unwrap();
        if address == [0; 6] || address == [0xff; 6] {
            return Err(Error::protocol("invalid controller address", &p));
        }
        Ok(OwnedReport {
            dropped: self.attempt.dropped.take().map(Box::new),
            before: identity,
            after,
            address,
            fallback: self.attempt.fallback.take().map(Box::new),
            firmware: Box::new(firmware.into()),
            configuration,
            container_version,
            v2: self.attempt.v2.take(),
            download: core::mem::take(&mut self.attempt.download),
        })
    }
}

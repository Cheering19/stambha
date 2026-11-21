// SPDX-License-Identifier: GPL-2.0-or-later
//! A bounded passive LE scan through a controller [`super::initialize`]
//! brought up, for the host check's `--bluetooth-check`: the standard HCI
//! Linux's core sends once btrtl's setup has run, cut to what a passive scan
//! needs. HCI Reset (`hci_init1_sync`, hci_sync.c:3890-3901); Set Event Mask
//! with its unconditional events, Flow Specification Complete and the LE
//! Meta event (`hci_set_event_mask_sync`, hci_sync.c:4231, :4240, :4327-4328);
//! LE Set Event Mask with the advertising report alone
//! (`hci_le_set_event_mask_sync`, hci_sync.c:4502-4506); LE Set Scan
//! Parameters, passive, at the core's 60 ms interval and 30 ms window
//! (hci_core.c:2449-2450, hci_sync.c:3135-3155); LE Set Scan Enable without
//! duplicate filtering for the window, so each report's RSSI is seen; scan
//! off; then HCI Reset, as `btrtl_shutdown_realtek` resets a Realtek
//! controller before it closes (btrtl.c:1386-1404), and the transport is
//! closed. A failed command closes it at once.
//!
//! Linux takes the extended scan commands where a controller has them
//! (hci_sync.c:3141, :2305); the legacy ones here hear legacy advertising
//! only. Reports are read as `hci_le_adv_report_evt` reads them
//! (hci_event.c:6393-6433), one after another, and one whose data is longer
//! than legacy advertising's 31 bytes is dropped as Linux drops it. An
//! advertiser's address is kept only to count it once; none leaves here.
use super::{Error, Result, initialize::Engine};
use crate::usb::bluetooth::{Hci, InitializationReport, Io, State};
use alloc::vec::Vec;

const RESET: u16 = 0x0c03;
const SET_EVENT_MASK: u16 = 0x0c01;
const LE_SET_EVENT_MASK: u16 = 0x2001;
const LE_SET_SCAN_PARAMETERS: u16 = 0x200b;
const LE_SET_SCAN_ENABLE: u16 = 0x200c;
/// hci_sync.c:4231's base, Flow Specification Complete (:4240) and the LE
/// Meta event (:4328).
const EVENT_MASK: [u8; 8] = [0xff, 0xff, 0xfb, 0xff, 0x01, 0, 0, 0x20];
/// LE Advertising Report (hci_sync.c:4506).
const LE_EVENT_MASK: [u8; 8] = [0x02, 0, 0, 0, 0, 0, 0, 0];
/// Passive, interval 0x0060 and window 0x0030 (`DISCOV_LE_SCAN_INT_FAST`,
/// `DISCOV_LE_SCAN_WIN_FAST`), public own address, no filter list.
const PASSIVE: [u8; 7] = [0, 0x60, 0, 0x30, 0, 0, 0];
const LE_META: u8 = 0x3e;
const ADVERTISING_REPORT: u8 = 0x02;
/// `HCI_MAX_AD_LENGTH`.
const MAX_ADVERTISING_DATA: usize = 31;
/// `HCI_RSSI_INVALID`: the controller has no RSSI for the report.
const NO_RSSI: i8 = 127;
/// Advertisers counted one by one; more are only said to be there.
pub const MAX_ADVERTISERS: usize = 256;
/// What the scan's commands may take beyond its window.
const COMMANDS_US: u64 = 30_000_000;

/// What a passive LE scan heard. It holds no address.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ScanReport {
    /// Advertising reports taken, of every advertiser.
    pub reports: u32,
    /// Distinct advertisers (address type and address), at most
    /// [`MAX_ADVERTISERS`].
    pub devices: usize,
    /// More advertisers were heard than [`MAX_ADVERTISERS`].
    pub saturated: bool,
    /// The strongest RSSI a report gave, in dBm.
    pub best_rssi: Option<i8>,
    /// Reports cut short or with too much data, which Linux drops.
    pub malformed: u32,
    /// Other events heard meanwhile.
    pub other_events: u32,
    /// How long the scan was on, in microseconds.
    pub window_us: u64,
}

#[derive(Default)]
struct Heard {
    report: ScanReport,
    advertisers: Vec<(u8, [u8; 6])>,
}

impl Heard {
    fn event(&mut self, e: &[u8]) {
        match e {
            [LE_META, _, ADVERTISING_REPORT, count, reports @ ..] => self.reports(*count, reports),
            _ => self.report.other_events += 1,
        }
    }

    /// `hci_le_adv_report_evt`: each report is event type, address type,
    /// address, data length, data and RSSI, one after another.
    fn reports(&mut self, count: u8, mut rest: &[u8]) {
        for _ in 0..count {
            let Some(&length) = rest.get(8) else {
                self.report.malformed += 1;
                return;
            };
            let length = usize::from(length);
            let Some(&rssi) = rest.get(9 + length) else {
                self.report.malformed += 1;
                return;
            };
            let (kind, address) = (rest[1], rest[2..8].try_into().unwrap());
            rest = &rest[10 + length..];
            if length > MAX_ADVERTISING_DATA {
                self.report.malformed += 1;
                continue;
            }
            self.report.reports += 1;
            let rssi = rssi as i8;
            if rssi != NO_RSSI {
                self.report.best_rssi = Some(self.report.best_rssi.map_or(rssi, |b| b.max(rssi)));
            }
            if self.advertisers.contains(&(kind, address)) {
                continue;
            }
            if self.advertisers.len() == MAX_ADVERTISERS {
                self.report.saturated = true;
            } else {
                self.advertisers.push((kind, address));
            }
        }
    }
}

/// Scan for `window_us` through a transport whose Realtek initialization it
/// keeps, then reset the controller and close the transport. A transport
/// without that retained initialization is refused before any command.
#[expect(
    clippy::result_large_err,
    reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
)]
pub fn scan<B: Io>(hci: &mut Hci<B>, window_us: u64) -> Result<ScanReport> {
    let credits = match hci.initialization() {
        Some(Ok(InitializationReport::Realtek(report))) => report.credits,
        _ => {
            return Err(Error::Unsupported(
                "a scan needs the transport's retained Realtek initialization",
            ));
        }
    };
    let deadline = hci
        .io
        .now_us()
        .saturating_add(window_us)
        .saturating_add(COMMANDS_US);
    let mut engine = Engine::new(hci, deadline, credits);
    let mut heard = Heard::default();
    let result = run(&mut engine, window_us, &mut heard);
    let closed = if engine.hci.state() == State::Open {
        engine.hci.close()
    } else {
        Ok(())
    };
    match (result, closed) {
        (Ok(()), Ok(())) => Ok(ScanReport {
            devices: heard.advertisers.len(),
            window_us,
            ..heard.report
        }),
        (Ok(()), Err(cleanup)) => Err(Error::Transport(cleanup)),
        (Err(cause), Ok(())) => Err(cause),
        (Err(cause), Err(cleanup)) => Err(Error::Cleanup {
            cause: alloc::boxed::Box::new(cause),
            cleanup,
        }),
    }
}

#[expect(
    clippy::result_large_err,
    reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
)]
fn run<B: Io>(engine: &mut Engine<'_, B>, window_us: u64, heard: &mut Heard) -> Result<()> {
    let mut tally = |e: &[u8]| {
        heard.event(e);
        Ok(())
    };
    engine.exchange(RESET, &[], &mut tally)?;
    engine.exchange(SET_EVENT_MASK, &EVENT_MASK, &mut tally)?;
    engine.exchange(LE_SET_EVENT_MASK, &LE_EVENT_MASK, &mut tally)?;
    engine.exchange(LE_SET_SCAN_PARAMETERS, &PASSIVE, &mut tally)?;
    engine.exchange(LE_SET_SCAN_ENABLE, &[1, 0], &mut tally)?;
    let until = engine.hci.io.now_us().saturating_add(window_us);
    engine.idle(until, &mut tally)?;
    engine.exchange(LE_SET_SCAN_ENABLE, &[0, 0], &mut tally)?;
    // Reports that came in the stop's completion's transfer, before Reset.
    let now = engine.hci.io.now_us();
    engine.idle(now, &mut tally)?;
    engine.exchange(RESET, &[], &mut tally)?;
    Ok(())
}

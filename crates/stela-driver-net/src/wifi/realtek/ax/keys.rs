// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! The security CAM of an RTL8852C station (M3 spec S12), as
//! `rtw89_ops_set_key` (`mac80211.c:987-1024`) programs it for a CCMP-128
//! key of the station's one link:
//!
//! - [`install`] is `rtw89_cam_sec_key_add` (`cam.c:465-535`) for
//!   `WLAN_CIPHER_SUITE_CCMP` into `rtw89_cam_sec_key_install`
//!   (`cam.c:399-463`): the first free of the 128 sec CAM entries
//!   (`rtw89_cam_get_avail_sec_cam`, `cam.c:102-136`), the key in it by
//!   SEC_UPD (`rtw89_cam_send_sec_key_cmd`, `cam.c:52-100`), then
//!   `__rtw89_cam_attach_sec_cam` (`cam.c:248-290`): the address CAM slot of
//!   `rtw89_cam_get_addr_cam_key_idx` (`cam.c:138-213`, NORMAL mode: the
//!   first free of slots 0-1 for a pairwise key, of 2-4 for a group key),
//!   its key ID and entry, then the dctl (`rtw89_fw_h2c_dctl_sec_cam_v1`,
//!   `fw.c:2346-2382`, with `rtw89_cam_fill_dctl_sec_cam_info_v1`,
//!   `cam.c:930-1009`, layout `cam.h:119-186`) and the address CAM
//!   (`rtw89_fw_h2c_cam`, `fw.c:2290-2335`; the fields `cam.c:906-927`).
//! - [`delete`] is `rtw89_cam_sec_key_del(inform_fw = true)`
//!   (`cam.c:537-566`): `__rtw89_cam_detach_sec_cam` (`cam.c:215-246`)
//!   clears the slot's valid bit alone, leaving its key ID and entry as they
//!   were, sends the dctl and the address CAM, and the entry is freed; no
//!   SEC_UPD clears the entry itself, as the cleared valid bit disables it.
//!
//! Linux sends each of these H2Cs without a wait. Stela's own (spec S1.12,
//! S12.5): [`install`] waits for SEC_UPD's REC_ACK and for the address CAM's
//! own DONE_ACK, matched by seq, each within `ACK_WAIT_US`, and only then
//! is the key installed. A key that failed after its entry was taken is
//! undone in [`KeyCam`], the slot's valid bit and the entry, whichever step
//! failed. Linux's error path frees the entry on any failure
//! (`cam.c:455-462`) but clears the slot's valid bit only when the address
//! CAM's send fails (`cam.c:280-286`): a failed dctl send returns with the
//! bit still set (`cam.c:274-278`), naming the freed entry. Stela clears it
//! there too, its one departure from that path, so no later dctl or
//! address CAM names an entry the CAM gave up. The H2Cs already sent are
//! not taken back, as Linux sends none either; the next dctl and address
//! CAM carry the undone state. [`delete`] waits for nothing.
//!
//! This module does not grant: any cipher but CCMP-128 (WEP's ALL_UNI mode,
//! TKIP, GCMP, and the 256-bit keys whose `ext_key` takes two entries and a
//! second SEC_UPD), a BIP key (the IGTK stays on the host, spec S1.2), an
//! AP's or a TDLS peer's own address CAM, MLO, WoWLAN's IV words, the
//! DISABLE_KEY flushes of `rtw89_ops_set_key` (`rtw89_hci_flush_queues` and
//! `rtw89_mac_flush_txq`, `mac80211.c:1010-1015`, which the station runs
//! through `Live::flush` before each delete), its BTC
//! EAPOL_END notify and RX statistics reset, and any key on the air: a
//! frame goes under a key only as its TX descriptor names the entry.
use super::assoc::{AddrCam, CamKeys};
use super::command::{CommandError, Session, Wait};
use super::io::{AxError, AxIo};
use super::receive::Event;
use super::vif::{MAC_ID, PORT};
use alloc::vec::Vec;
use core::ops::RangeInclusive;

/// 8852C's `scam_num` (`rtw8852c.c:3220`).
pub const SEC_CAM_ENTRIES: u8 = 128;
/// `RTW89_SEC_CAM_IN_ADDR_CAM` (`core.h:4572`).
const SLOTS: usize = 7;
/// `RTW89_ADDR_CAM_SEC_NORMAL`'s slots (`cam.c:139-143`, `:166-188`):
/// unicast 0-1, group 2-4; BIP 5-6 stay unused.
const PAIRWISE_SLOTS: RangeInclusive<usize> = 0..=1;
const GROUP_SLOTS: RangeInclusive<usize> = 2..=4;
/// The key ID fields are two bits wide (`cam.h:89-95`, `:169-175`).
const MAX_KEY_ID: u8 = 3;
/// `SEC_ENT0_KEYID`'s low bit in address CAM w9 and dctl w4.
const KEYID_SHIFT: usize = 18;

/// What a delete of an entry holding no key gets (Linux's `-EINVAL`,
/// `cam.c:548-550`).
pub const NO_KEY: AxError = AxError::Mac("no key in that security CAM entry");
const NO_ENTRY: AxError = AxError::Mac("no free security CAM entry");
const NO_SLOT: AxError = AxError::Mac("no free address CAM security slot");

/// The sec CAM's entries in use (`cam_info->sec_cam_map`) and the station
/// vif's address CAM security fields (`addr_cam->sec_cam_map`,
/// `sec_ent_keyid`, `sec_ent`), as `rtw89_cam_init_addr_cam` leaves them
/// (`cam.c:646-684`) and the installs and deletes change them. A detached
/// slot keeps its key ID and entry, as Linux's does.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct KeyCam {
    used: u128,
    /// Bit `i`: slot `i` valid.
    valid: u8,
    keyids: [u8; SLOTS],
    ents: [u8; SLOTS],
}

impl KeyCam {
    /// `rtw89_cam_get_avail_sec_cam` for a 128-bit key: the first free
    /// entry, taken; `None` when all 128 are (`-EBUSY`).
    pub fn allocate(&mut self) -> Option<u8> {
        let idx = (!self.used).trailing_zeros();
        if idx >= u32::from(SEC_CAM_ENTRIES) {
            return None;
        }
        self.used |= 1 << idx;
        Some(idx as u8)
    }

    /// Entry `cam` free again; an entry past the CAM changes nothing.
    pub fn release(&mut self, cam: u8) {
        if cam < SEC_CAM_ENTRIES {
            self.used &= !(1 << cam);
        }
    }

    /// Whether entry `cam` is taken.
    pub fn installed(&self, cam: u8) -> bool {
        cam < SEC_CAM_ENTRIES && self.used & (1 << cam) != 0
    }

    /// The slot a pairwise or group key would take, NORMAL mode's first
    /// free one, or `None` (`-EBUSY`).
    pub fn free_slot(&self, pairwise: bool) -> Option<usize> {
        let mut slots = if pairwise {
            PAIRWISE_SLOTS
        } else {
            GROUP_SLOTS
        };
        slots.find(|&i| self.valid & (1 << i) == 0)
    }

    /// `__rtw89_cam_attach_sec_cam`'s slot for entry `cam`: the key ID
    /// `keyidx` and the entry in [`KeyCam::free_slot`], made valid; the
    /// fields then. `None`, changing nothing, without a free slot.
    pub fn attach(&mut self, cam: u8, pairwise: bool, keyidx: u8) -> Option<CamKeys> {
        let slot = self.free_slot(pairwise)?;
        self.keyids[slot] = keyidx;
        self.ents[slot] = cam;
        self.valid |= 1 << slot;
        Some(self.keys())
    }

    /// `__rtw89_cam_detach_sec_cam`'s software half: every valid slot
    /// naming entry `cam` made invalid, its key ID and entry left in place;
    /// the fields then. The entry stays taken ([`KeyCam::release`]).
    pub fn detach(&mut self, cam: u8) -> CamKeys {
        for slot in 0..SLOTS {
            if self.valid & (1 << slot) != 0 && self.ents[slot] == cam {
                self.valid &= !(1 << slot);
            }
        }
        self.keys()
    }

    /// The address CAM's and dctl's security fields
    /// (`rtw89_cam_fill_addr_cam_info`, `cam.c:906-927`): w9's key IDs, two
    /// bits each from bit 18; w10 the valid map [7:0] and entries 0-2; w11
    /// entries 3-6.
    pub fn keys(&self) -> CamKeys {
        let keyids = self.keyids.iter().enumerate().fold(0u32, |w, (slot, &k)| {
            w | u32::from(k & MAX_KEY_ID) << (KEYID_SHIFT + 2 * slot)
        });
        let e = self.ents.map(u32::from);
        CamKeys {
            keyids,
            valid_and_ents: u32::from(self.valid) | e[0] << 8 | e[1] << 16 | e[2] << 24,
            ents_hi: e[3] | e[4] << 8 | e[5] << 16 | e[6] << 24,
        }
    }

    /// The entries of the valid slots in Stela's order: the group keys by
    /// key ID, then the pairwise keys by key ID. It is the order
    /// `ieee80211_free_sta_keys` (`net/mac80211/key.c:1186-1215`) frees a
    /// station's own keys in, but in station mode the AP's station holds
    /// only the PTK there: mac80211 installs the GTK as a key of the vif,
    /// which cfg80211's disconnect deletes (`__cfg80211_disconnected`,
    /// `net/wireless/sme.c:1375-1391`). The radio deletes every key itself,
    /// the PTK first, before the chip leaves; this order is only for what
    /// is left then.
    pub fn deletion_order(&self) -> Vec<u8> {
        let valid = |slot: &usize| self.valid & (1 << slot) != 0;
        let mut order = Vec::new();
        for slots in [GROUP_SLOTS, PAIRWISE_SLOTS] {
            let mut found: Vec<usize> = slots.filter(valid).collect();
            found.sort_by_key(|&slot| self.keyids[slot]);
            order.extend(found.iter().map(|&slot| self.ents[slot]));
        }
        order
    }
}

/// What [`install`] refuses before any I/O: a key ID past 3, no free entry
/// of the 128, no free slot. Linux finds the slot only after SEC_UPD; here
/// nothing is sent for a key that has none.
pub fn check(cam: &KeyCam, pairwise: bool, keyidx: u8) -> Result<(), AxError> {
    if keyidx > MAX_KEY_ID {
        return Err(AxError::Mac("a key ID past 3"));
    }
    if cam.used == u128::MAX {
        return Err(NO_ENTRY);
    }
    cam.free_slot(pairwise).map(|_| ()).ok_or(NO_SLOT)
}

fn command(error: CommandError) -> AxError {
    match error {
        CommandError::Submit(e) | CommandError::Wait(e) => e,
    }
}

/// Install the CCMP-128 key `material`, pairwise or the group key
/// `keyidx`, for the station whose address CAM is `addr` (its own key
/// fields are replaced by `cam`'s), through `session` with what it drains
/// handed to `sink` (M3 spec S12.5; see the module documentation): SEC_UPD
/// into the first free entry and its REC_ACK, the slot attached, the dctl,
/// the address CAM and its own DONE_ACK with return 0. Returns the entry.
/// Refused before any I/O as [`check`] refuses; any later failure undoes the
/// slot and the entry in `cam` and is returned.
///
/// # Safety
/// The contract of `Ch12::submit` and `Rxq::poll` for `io` and the
/// session's rings: `io` is the exclusive register window of the same
/// RTL8852C requester that `pci::pre_init` programmed with them, `pre_init`
/// succeeded, and its contract still holds.
#[expect(
    clippy::too_many_arguments,
    reason = "rtw89_cam_sec_key_install's own inputs: the chip, the CAM state, the station and the key."
)]
pub unsafe fn install<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    cam: &mut KeyCam,
    addr: &AddrCam,
    pairwise: bool,
    keyidx: u8,
    material: &[u8; 16],
    sink: &mut dyn FnMut(Event<'_>),
) -> Result<u8, AxError> {
    check(cam, pairwise, keyidx)?;
    let idx = cam.allocate().ok_or(NO_ENTRY)?;
    let packet = session.h2c.sec_upd(idx, material);
    // SAFETY: the caller's contract is `send_and_wait_for`'s, word for word.
    let installed = unsafe { session.send_and_wait_for(io, packet, Wait::Received, sink) }
        .map_err(command)
        .and_then(|_| {
            let keys = cam.attach(idx, pairwise, keyidx).ok_or(NO_SLOT)?;
            let packet = session.h2c.dctl_sec(MAC_ID, &keys);
            // SAFETY: as above, `send`'s.
            unsafe { session.send(io, packet, sink) }?;
            let packet = session
                .h2c
                .addr_cam_with(&AddrCam { keys, ..*addr }, MAC_ID, PORT);
            let wait = Wait::Completed { mask: 0xFF };
            // SAFETY: as above.
            unsafe { session.send_and_wait_for(io, packet, wait, sink) }.map_err(command)
        });
    match installed {
        Ok(_) => Ok(idx),
        Err(error) => {
            cam.detach(idx);
            cam.release(idx);
            Err(error)
        }
    }
}

/// Delete the key in entry `idx` from the station whose address CAM is
/// `addr` (M3 spec S12.6): its slot detached, the dctl, the address CAM,
/// neither waited on; the entry is freed whatever they returned, as Linux
/// frees it, and the first failure is returned. An entry holding no key is
/// refused before any I/O ([`NO_KEY`]).
///
/// # Safety
/// As [`install`].
pub unsafe fn delete<I: AxIo + ?Sized>(
    io: &mut I,
    session: &mut Session<'_>,
    cam: &mut KeyCam,
    addr: &AddrCam,
    idx: u8,
    sink: &mut dyn FnMut(Event<'_>),
) -> Result<(), AxError> {
    if !cam.installed(idx) {
        return Err(NO_KEY);
    }
    let keys = cam.detach(idx);
    let packet = session.h2c.dctl_sec(MAC_ID, &keys);
    // SAFETY: the caller's contract is `send`'s, word for word.
    let dctl = unsafe { session.send(io, packet, sink) }.map(|_| ());
    // `__rtw89_cam_detach_sec_cam` sends the address CAM whatever the dctl
    // returned.
    let packet = session
        .h2c
        .addr_cam_with(&AddrCam { keys, ..*addr }, MAC_ID, PORT);
    // SAFETY: as above.
    let cammed = unsafe { session.send(io, packet, sink) }.map(|_| ());
    cam.release(idx);
    dctl.and(cammed)
}

#[cfg(test)]
mod tests;

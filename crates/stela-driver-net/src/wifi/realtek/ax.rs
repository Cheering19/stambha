// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE (rtw89, the "AX" generation) bring-up to running firmware and
//! an initialized MAC: its register access, with masked and polling helpers
//! that mirror rtw89's `rtw89_read*`/`rtw89_write*`,
//! `rtw89_write*_set`/`_clr`/`_mask` and `read_poll_timeout` uses from
//! pinned rtw89/core.h; MAC power on and off; the efuse; the DMAC and PCIe
//! pre-init with its rings; the firmware download; the MAC init of
//! `rtw89_mac_init` (BB/RF function enable, system, DMAC, CMAC band 0 and
//! IMR init, PCIe post-init); and one H2C, OFLD_CFG, whose acknowledgement
//! is read from RXQ by polling. After it, the radio steps of
//! `rtw89_core_start` through the Wi-Fi grant run ([`phy::init_radio`]):
//! the BB/RF reset ([`sys::reset_bb_rf`]), `rtw89_phy_init_bb_reg`
//! ([`bb::init_bb_reg`]) and `rtw89_phy_init_rf_reg` with its RF_REG H2C
//! pages ([`radio::init_rf_reg`]) from the firmware file's own PHY tables
//! (found by [`elements`], walked by RFE type and cut by [`phy_table`], BB
//! and RF registers reached by [`bb`] and [`rf`]), then the register end
//! state of the coex start with BT off ([`coex`]). Then `rtw89_phy_dm_init`
//! runs ([`phy::calibrate`]): its first part (thermals, BB GPIO set-up, env
//! monitor, PHY status bitmaps, DIG, crystal cap and EDCCA) and its tail
//! (TX power reference, efuse trims, RX path) from [`dm`], the NCTL table
//! ([`bb::init_rf_nctl`]), and `rtw8852c_rfk_init`'s RCK, DACK with its
//! DRCK, ADDCK and coex notify, and RX DCK from [`rfk`]. [`bring_up`] puts
//! these in Linux's order before powering the chip off again. [`chan`]
//! describes a 20 MHz 2.4 or 5 GHz channel as `rtw89_chan_create` does,
//! without programming it. [`ppdu`] reads a PPDU status's per-path RSSI
//! and channel index and matches it to the frames of its PPDU, without
//! queueing them. [`receive`] routes each RXQ buffer as `rtw89_core_rx`
//! does: C2Hs on, beacons and probe responses queued for their PPDU status,
//! tagged with the channel the firmware reports and stripped of their FCS.
//! [`channel`] is `__rtw89_set_channel` for a 20 MHz channel: the TX
//! scheduler and PPDU status stopped and resumed around the change, the MAC
//! rate checks, the baseband (gain tables, efuse RX gain offsets, CCK
//! filters, spur notches, TX path table), and RF 0x18's channel and
//! bandwidth on both paths. [`core_tail`] is the rest of
//! `rtw89_core_start` (PPDU status reports, the RTS threshold, and the
//! firmware log and BA CAM H2Cs), sent through a [`command::Session`],
//! which submits every H2C after `rtw89_phy_dm_init` on CH12 and drains RXQ
//! through one [`receive::Receiver`], waiting for a scan command's DONE_ACK
//! and recording any other refusal. [`vif`] is `rtw89_mac_vif_init` for one
//! unassociated station on port 0 and macid 0: the port registers, the DMAC
//! and CMAC tables, then the macid unpause, role, join, address CAM and CMAC
//! table H2Cs through the same session. [`live`] keeps the chip powered:
//! [`live::start`] runs the bring-up through `rtw89_phy_dm_init`, then
//! these in Linux's order (core start, channel 1 with its TX power, vif
//! init), then allocates the TX pools; [`live::Live::tune`] switches
//! channel and [`live::Live::stop`] tears the chip down. [`scan`] runs the
//! firmware's passive scan offload over 38 channels on it, and keeps the
//! beacons and probe responses it hears as networks by BSSID. [`bring_up`]
//! is [`live::start`], [`scan::run_once`] and [`live::Live::stop`], with the
//! BB gain table, efuse RX gain offsets, RPL bias and cut the steps before
//! the scan produced, and reports the networks with RF 0x18 read through
//! both of its windows ([`Rf18`]). The bring-up grants no channel but the
//! scan's boot channel, no IQK, DPK, TSSI or tracking, no association, no
//! transmission request and no interrupts; the chip is powered off at the
//! end. [`attach`] is the other way out of [`live::start`]: it keeps the
//! live chip as the [`station::Rtl8852c`] radio, connected through the
//! shared connection keeper to the configured network, under that network's
//! channel rules.
pub mod assoc;
pub mod attach;
pub mod bb;
pub mod bringup;
pub mod c2h;
pub mod chan;
pub mod channel;
pub mod cmac;
pub mod coex;
pub mod command;
pub mod core_tail;
pub mod dm;
pub mod dmac;
pub mod efuse;
pub mod elements;
pub mod fwdl;
pub mod h2c;
pub mod io;
pub mod live;
pub mod pci;
pub mod phy;
pub mod phy_table;
pub mod power;
pub mod ppdu;
pub mod radio;
pub mod receive;
pub mod regd;
pub mod rf;
pub mod rfk;
pub mod rx;
pub mod scan;
pub mod station;
pub mod sys;
#[cfg(test)]
pub(crate) mod tests;
pub mod trx;
pub mod txd;
pub mod txpwr;
pub mod txpwr_tables;
pub mod vif;

pub use attach::{AttachError, attach};
pub use bringup::{
    BringUpError, Earlier, FirmwareFile, Host, Rf18, Rf18Line, Running, bring_up, quiesce,
};
pub use command::H2cAck;
pub use fwdl::download;
pub use io::{
    AxError, AxIo, NativeAxIo, Polled, clear8, clear16, clear32, field8, field16, field32,
    poll_sample, poll_sample_atomic, poll32, poll32_atomic, poll32_value, poll32_value_atomic,
    set8, set16, set32,
};
pub use live::{Live, StartError};
pub use phy::{PhyReport, RfkReport};
// `pci::pre_init` and `dmac::pre_init` are reached by their module only.
pub use pci::{Ch12, Rings, encode_txbd};
pub use power::{power_on, xtal_si_read, xtal_si_write};
pub use scan::{Listed, Network, ScanEnd, ScanReport};
pub use station::{Rtl8852c, WIFI_CARRIER_WAIT_US, Wifi};

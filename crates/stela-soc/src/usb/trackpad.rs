// SPDX-License-Identifier: GPL-2.0-only
// The report layout follows VoodooInput d8978137 and Linux
// hid-magicmouse.c (Copyright (c) 2010 Michael Poole and Chase Douglas).
// See NOTICE.
//! An emulated USB Magic Trackpad 2 fed by the host's precision touchpad:
//! its multitouch input report ([`report`]).
pub mod report;

//! The PCIe link amdgpu takes for an AMD GPU's slot, given to it on Linux's
//! command line as `amdgpu.pcie_gen_cap` and `amdgpu.pcie_lane_cap`, which
//! `amdgpu_device_get_pcie_info` takes before it reads any link itself.
//!
//! On an Intel host amdgpu turns PCIe DPM off and has the SMU force every
//! link level to the slot's generation and width
//! (`smu_v13_0_0_update_pcie_parameters`), which it reads from the trained
//! links between the GPU and the root complex (`pcie_bandwidth_available`).
//! A width it cannot read sets no lane bit, so it asks for lane 0, which the
//! SMU's firmware refuses, and the probe fails. The link here is the one
//! amdgpu would read on Linux: the trained link of its `link_dev` and
//! `link_partner` (`amdgpu_device_init_pcie_links`, which passes over the
//! card's own switch), from the port snapshots the boot already holds.
use alloc::{format, string::String};

/// AMD's PCI vendor ID, which its GPUs and the switch on its cards carry
/// (Linux's `PCI_VENDOR_ID_ATI`).
const ATI: u16 = 0x1002;

/// A trained PCIe link, as a Link Status register (`PCI_EXP_LNKSTA`)
/// names it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Link {
    /// Its generation: 1 for 2.5 GT/s through 6 for 64 GT/s.
    pub generation: u8,
    /// Its lanes.
    pub width: u8,
}

impl Link {
    /// The link `status` names, or `None` where it names no known speed
    /// or no lanes, as a register that did not read does.
    pub fn from_status(status: u16) -> Option<Self> {
        let generation = (status & 0xf) as u8;
        let width = ((status >> 4) & 0x3f) as u8;
        ((1..=6).contains(&generation) && width != 0).then_some(Self { generation, width })
    }

    /// The narrower of two links: the slower generation and the fewer lanes.
    fn min(self, other: Self) -> Self {
        Self {
            generation: self.generation.min(other.generation),
            width: self.width.min(other.width),
        }
    }

    /// amdgpu's masks for this link (`amd_pcie.h`): `pcie_gen_cap` with the
    /// platform's and the ASIC's bit of each generation up to it, the
    /// highest amdgpu knows being 5; `pcie_lane_cap` with both bits of each
    /// width amdgpu names (x1, x2, x4, x8, x12, x16, x32) up to its lanes.
    /// A valid link always sets x1 and gen 1, so neither mask is 0.
    pub fn masks(self) -> (u32, u32) {
        const BOTH: u32 = 0x0001_0001;
        const WIDTHS: [u8; 7] = [1, 2, 4, 8, 12, 16, 32];
        let generations = self.generation.clamp(1, 5);
        let gen_cap = (0..generations).fold(0, |mask, k| mask | BOTH << k);
        let lane_cap = WIDTHS
            .iter()
            .enumerate()
            .filter(|&(_, &lanes)| lanes <= self.width)
            .fold(0, |mask, (k, _)| mask | BOTH << k);
        (gen_cap, lane_cap.max(BOTH))
    }

    /// The words of Linux's command line that give amdgpu this link.
    pub fn parameters(self) -> String {
        let (gen_cap, lane_cap) = self.masks();
        format!("amdgpu.pcie_gen_cap={gen_cap:#x} amdgpu.pcie_lane_cap={lane_cap:#x}")
    }
}

/// The Link Status register of the PCI Express capability in a function's
/// first 256 configuration bytes, `config`, or `None` where it has none.
pub fn status(config: &[u32; 64]) -> Option<u16> {
    const STATUS: usize = 0x06;
    const CAPABILITIES: u16 = 1 << 4;
    const POINTER: usize = 0x34;
    const EXPRESS: u8 = 0x10;
    const LINK_STATUS: usize = 0x12;
    let byte = |at: usize| (config[at / 4] >> (8 * (at % 4))) as u8;
    let word = |at: usize| u16::from(byte(at)) | u16::from(byte(at + 1)) << 8;
    if word(STATUS) & CAPABILITIES == 0 {
        return None;
    }
    let mut at = usize::from(byte(POINTER) & !3);
    // At most one capability for each dword past the header.
    for _ in 0..48 {
        if at < 0x40 || at + LINK_STATUS + 2 > 256 {
            return None;
        }
        if byte(at) == EXPRESS {
            return Some(word(at + LINK_STATUS));
        }
        at = usize::from(byte(at + 1) & !3);
    }
    None
}

/// The link amdgpu takes for the slot of a GPU whose own Link Status reads
/// `endpoint`, below `ports` (nearest first: each port's vendor and Link
/// Status). Its `link_dev` is the last of the card's own switch ports (AMD
/// ones) right above it, or the GPU where none is, and its `link_partner`
/// the first port above those; the slot's link is the narrower of the two
/// ends' trained links. Where either end's does not read, the GPU's own
/// trained link stands; `None` where that does not read either, so no link
/// is given and amdgpu reads its own.
pub fn slot(endpoint: Option<u16>, ports: &[(u16, Option<u16>)]) -> Option<Link> {
    let own = ports
        .iter()
        .take_while(|(vendor, _)| *vendor == ATI)
        .count();
    let link_dev = match own {
        0 => endpoint,
        _ => ports[own - 1].1,
    };
    let partner = ports.get(own).and_then(|(_, status)| *status);
    let ends = link_dev
        .and_then(Link::from_status)
        .zip(partner.and_then(Link::from_status));
    ends.map(|(near, far)| near.min(far))
        .or_else(|| endpoint.and_then(Link::from_status))
}

#[cfg(test)]
mod tests {
    use super::*;

    const INTEL: u16 = 0x8086;

    /// LNKSTA of a link at generation `generation` with `width` lanes.
    fn trained(generation: u16, width: u16) -> Option<u16> {
        Some(generation | width << 4)
    }

    /// A gen 4 x16 link gives every generation to 4 and every width to x16,
    /// a gen 3 x8 link to 3 and x8, as `amd_pcie.h` names their bits.
    #[test]
    fn a_link_gives_amdgpu_masks_of_every_step_up_to_it() {
        let link = |generation, width| Link { generation, width };
        assert_eq!(link(4, 16).masks(), (0x000f_000f, 0x003f_003f));
        assert_eq!(link(3, 8).masks(), (0x0007_0007, 0x000f_000f));
        assert_eq!(link(1, 1).masks(), (0x0001_0001, 0x0001_0001));
        // 64 GT/s is past amdgpu's masks: gen 5 is its highest.
        assert_eq!(link(6, 32).masks(), (0x001f_001f, 0x007f_007f));
        assert_eq!(
            link(4, 16).parameters(),
            "amdgpu.pcie_gen_cap=0xf000f amdgpu.pcie_lane_cap=0x3f003f"
        );
    }

    /// A status that names no lanes or no known speed is no link.
    #[test]
    fn an_unread_status_is_no_link() {
        assert_eq!(Link::from_status(0), None);
        assert_eq!(Link::from_status(4), None);
        assert_eq!(Link::from_status(16 << 4), None);
        assert_eq!(Link::from_status(7 | 16 << 4), None);
        assert_eq!(Link::from_status(0xffff), None);
        assert_eq!(
            Link::from_status(0x1104),
            Some(Link {
                generation: 4,
                width: 16
            })
        );
    }

    /// An RX 7900 XTX below its card's switch (1479, 1478, both gen 4 x16)
    /// below a gen 5 root port trained at gen 4 x16: the slot's link is
    /// gen 4 x16, from the switch's upstream port and the root port, as
    /// amdgpu reads them on Linux.
    #[test]
    fn the_slot_is_the_link_between_the_cards_switch_and_the_root_port() {
        let ports = [
            (ATI, trained(4, 16)),
            (ATI, trained(4, 16)),
            (INTEL, trained(4, 16)),
        ];
        let link = slot(trained(4, 16), &ports).unwrap();
        assert_eq!(link.masks(), (0x000f_000f, 0x003f_003f));
        // A slot trained at gen 3 x8 below the card's switch is that slot.
        let ports = [
            (ATI, trained(4, 16)),
            (ATI, trained(3, 8)),
            (INTEL, trained(3, 8)),
        ];
        assert_eq!(
            slot(trained(4, 16), &ports),
            Some(Link {
                generation: 3,
                width: 8
            })
        );
        // A card with no switch: the GPU and its root port.
        assert_eq!(
            slot(trained(3, 16), &[(0x1022, trained(4, 16))]),
            Some(Link {
                generation: 3,
                width: 16
            })
        );
    }

    /// A root port whose snapshot reads no link leaves the GPU's own link;
    /// with that unread too, no link is given, and never a width of 0.
    #[test]
    fn an_unread_end_falls_back_to_the_gpus_own_link_and_never_to_zero() {
        let ports = [
            (ATI, trained(4, 16)),
            (ATI, trained(4, 16)),
            (INTEL, Some(0)),
        ];
        assert_eq!(
            slot(trained(4, 16), &ports),
            Some(Link {
                generation: 4,
                width: 16
            })
        );
        let ports = [(ATI, trained(4, 16)), (ATI, None)];
        assert_eq!(
            slot(trained(4, 8), &ports).map(Link::masks),
            Some((0x000f_000f, 0x000f_000f))
        );
        assert_eq!(slot(Some(0), &ports), None);
        assert_eq!(slot(None, &[]), None);
        // On a root bus, with no port above, the GPU's own link.
        assert_eq!(
            slot(trained(4, 16), &[]),
            Some(Link {
                generation: 4,
                width: 16
            })
        );
    }

    /// The Link Status is read from the PCI Express capability wherever the
    /// capability list puts it.
    #[test]
    fn the_link_status_is_read_from_the_express_capability() {
        let mut config = [0u32; 64];
        assert_eq!(status(&config), None);
        // Status: a capability list from 0x50: power management, then
        // PCI Express at 0x64 with its link status 0x1104.
        config[1] = 0x0010 << 16;
        config[0x34 / 4] = 0x50;
        config[0x50 / 4] = 0x6401;
        config[0x64 / 4] = 0x0010;
        config[0x74 / 4] = 0x1104 << 16;
        assert_eq!(status(&config), Some(0x1104));
        // A list that loops without one ends.
        config[0x50 / 4] = 0x5001;
        assert_eq!(status(&config), None);
        // A snapshot that read all ones.
        assert_eq!(status(&[u32::MAX; 64]), None);
    }
}

//! The owner's side of a lent controller: the host's controller behind a
//! small trait, [`HostController`], which the engine's host drivers and the
//! tests' scripted controller implement alike, and a [`Link`] that carries
//! packets between it and the guest's function over service turns.
//!
//! The function's queues are reached under the devices' lock and the host's
//! controller outside it, so a turn takes what the function queued
//! ([`super::super::Hci::take`]) and how much room it has
//! ([`super::super::Hci::room_left`]), runs [`Link::send`] and
//! [`Link::receive`], then hands what came back to the function. The link
//! keeps what the controller has no room for yet, and what it gave beyond
//! the guest's room, for later turns: one poll can return more packets than
//! the guest's queues take.
use super::super::Outbound;
use alloc::{collections::VecDeque, string::String, vec::Vec};
pub use stela_drivers::usb::bluetooth::Packet;

/// The host's controller, as a service turn reaches it.
pub trait HostController {
    /// Offers the controller one packet from the guest's function:
    /// `Ok(true)` once it took it, `Ok(false)` while it has no room, which
    /// keeps the packet for a later turn. An error is the controller's
    /// failure, after which it takes nothing.
    fn send(&mut self, packet: &Outbound) -> Result<bool, String>;
    /// What the controller has for the guest now, never waiting for the
    /// radio.
    fn poll(&mut self) -> Result<Vec<Packet>, String>;
}

/// What waits between the function and the host's controller.
#[derive(Debug, Default)]
pub struct Link {
    /// The function's packets the controller had no room for.
    waiting: VecDeque<Outbound>,
    /// The controller's packets the guest had no room for.
    held: VecDeque<Packet>,
}

impl Link {
    /// Queues the function's packets behind those still waiting.
    pub fn queue(&mut self, packets: impl IntoIterator<Item = Outbound>) {
        self.waiting.extend(packets);
    }

    /// Offers `controller` what waits, in order, until it has no room.
    pub fn send(&mut self, controller: &mut (impl HostController + ?Sized)) -> Result<(), String> {
        while let Some(packet) = self.waiting.front() {
            if !controller.send(packet)? {
                return Ok(());
            }
            self.waiting.pop_front();
        }
        Ok(())
    }

    /// At most `room` packets for the guest: those held from an earlier
    /// turn first, then what `controller` has, which is polled only while
    /// fewer than `room` are held. The rest stays held.
    pub fn receive(
        &mut self,
        controller: &mut (impl HostController + ?Sized),
        room: usize,
    ) -> Result<Vec<Packet>, String> {
        if self.held.len() < room {
            self.held.extend(controller.poll()?);
        }
        let taken = room.min(self.held.len());
        Ok(self.held.drain(..taken).collect())
    }

    /// Drops what waits either way, once the controller has failed.
    pub fn clear(&mut self) {
        self.waiting.clear();
        self.held.clear();
    }

    /// Packets waiting for the controller, and held for the guest.
    pub fn pending(&self) -> (usize, usize) {
        (self.waiting.len(), self.held.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::{format, vec};

    /// A controller that takes `room` packets and gives `polls` in turn.
    #[derive(Default)]
    struct Scripted {
        room: usize,
        taken: Vec<Outbound>,
        polls: VecDeque<Vec<Packet>>,
        failed: bool,
    }

    impl HostController for Scripted {
        fn send(&mut self, packet: &Outbound) -> Result<bool, String> {
            if self.failed {
                return Err(String::from("gone"));
            }
            if self.room == 0 {
                return Ok(false);
            }
            self.room -= 1;
            self.taken.push(packet.clone());
            Ok(true)
        }

        fn poll(&mut self) -> Result<Vec<Packet>, String> {
            if self.failed {
                return Err(String::from("gone"));
            }
            Ok(self.polls.pop_front().unwrap_or_default())
        }
    }

    fn event(n: u8) -> Packet {
        Packet::Event(vec![0x13, 1, n])
    }

    #[test]
    fn what_the_controller_has_no_room_for_waits_in_order() {
        let mut link = Link::default();
        let mut controller = Scripted {
            room: 1,
            ..Scripted::default()
        };
        let command = |n: u8| Outbound::Command(vec![n, 0x0c, 0]);
        link.queue([command(1), command(2)]);
        link.send(&mut controller).unwrap();
        assert_eq!(controller.taken, [command(1)]);
        assert_eq!(link.pending(), (1, 0));
        link.queue([command(3)]);
        controller.room = 5;
        link.send(&mut controller).unwrap();
        assert_eq!(controller.taken, [command(1), command(2), command(3)]);
        assert_eq!(link.pending(), (0, 0));
    }

    /// Bounded delivery: one poll that gives more than the guest has room
    /// for leaves the rest held, in order, and nothing is lost.
    #[test]
    fn the_guest_gets_at_most_its_room_and_the_rest_is_held() {
        let mut link = Link::default();
        let mut controller = Scripted::default();
        controller.polls.push_back((0..5).map(event).collect());
        controller.polls.push_back(vec![event(5)]);
        assert_eq!(
            link.receive(&mut controller, 2).unwrap(),
            [event(0), event(1)]
        );
        assert_eq!(link.pending(), (0, 3));
        // Held packets go first; the controller is not polled while as many
        // are held as there is room.
        assert_eq!(
            link.receive(&mut controller, 3).unwrap(),
            [event(2), event(3), event(4)]
        );
        assert_eq!(controller.polls.len(), 1);
        assert!(link.receive(&mut controller, 0).unwrap().is_empty());
        assert_eq!(link.receive(&mut controller, 4).unwrap(), [event(5)]);
        assert!(link.receive(&mut controller, 4).unwrap().is_empty());
    }

    #[test]
    fn a_failure_is_the_controllers_and_clear_drops_what_waits() {
        let mut link = Link::default();
        let mut controller = Scripted {
            failed: true,
            ..Scripted::default()
        };
        link.queue([Outbound::Acl(vec![1, 0, 0, 0])]);
        assert_eq!(link.send(&mut controller), Err(String::from("gone")));
        assert_eq!(
            format!("{:?}", link.receive(&mut controller, 1)),
            "Err(\"gone\")"
        );
        link.clear();
        assert_eq!(link.pending(), (0, 0));
    }
}

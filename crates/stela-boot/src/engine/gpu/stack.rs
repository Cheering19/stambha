//! Stacks for the code that expects a process's: Mesa and LLVM run deeper than
//! a PE's or a Linux task's stack. Each lies in the host's page window between
//! absent pages, so running off either end faults.
use super::PAGE;
use crate::{native_memory::SOURCE, native_mmio::Window};
use core::mem::ManuallyDrop;
use stela_boot_core::page_window::Cache;
use stela_boot_core::stack_guards::{GuardHit, StackGuards};
use stela_drivers::bus::dma::memory::{CacheAttribute, PageSource, PinnedPages};

pub(super) use crate::stack_switch::{on_stack, pointer};

pub(super) struct Stack {
    window: &'static Window,
    memory: ManuallyDrop<PinnedPages>,
    /// Where the page beneath the stack would be.
    guard: u64,
}

/// The absent pages beneath and above a stack.
const GUARD_PAGES: usize = 2;

/// The live stacks, for the #PF handler: their guard pages lie in the window
/// among clients' mappings, and a fault there is this code's, not a client's.
/// The service's three stacks for each GPU and Mesa's threads fit.
static GUARDS: StackGuards<64> = StackGuards::new();

/// The stack `address` is a guard page of, if any.
pub(in crate::engine) fn guard_hit(address: u64) -> Option<GuardHit> {
    GUARDS.hit(address)
}

impl Stack {
    pub(super) fn map(window: &'static Window, bytes: usize) -> Option<Self> {
        let memory = SOURCE
            .allocate(bytes, PAGE as usize, CacheAttribute::WriteBack)
            .ok()?;
        let pages = memory.physical_pages().len();
        let guard = window.reserve(pages + GUARD_PAGES)?;
        let stack = Self {
            window,
            memory: ManuallyDrop::new(memory),
            guard,
        };
        for (&physical, page) in stack.memory.physical_pages().iter().zip(1..) {
            if !window.install(guard + page * PAGE, physical, Cache::WriteBack) {
                return None;
            }
        }
        GUARDS.add(stack.bottom()..stack.top());
        Some(stack)
    }

    fn pages(&self) -> u64 {
        self.memory.physical_pages().len() as u64
    }

    pub(super) fn bottom(&self) -> u64 {
        self.guard + PAGE
    }

    pub(super) fn top(&self) -> u64 {
        self.bottom() + self.pages() * PAGE
    }
}

impl Drop for Stack {
    fn drop(&mut self) {
        let pages = self.pages() as usize + GUARD_PAGES;
        GUARDS.remove(self.bottom());
        super::unmapped(super::Unmapped::Stacks);
        // A processor that kept the translation keeps the memory too.
        if self.window.remove(self.guard, pages) {
            self.window.release(self.guard, pages);
            // SAFETY: nothing maps the pages now, and this is their one drop.
            unsafe { ManuallyDrop::drop(&mut self.memory) };
        }
    }
}

/// The stack the boot's first stop runs on ([`crate::output::fatal`]), as
/// deep as an ordinary task's: a stop raised in a Linux task, as a driver's
/// failed probe raises one, stores the logs through the USB's drivers,
/// whose frames pass that task's 16 KiB.
const STOP_STACK_BYTES: usize = 1 << 20;

/// Maps the stop's stack once, before Linux runs; it is kept for the boot.
/// Without room for it, one line says so and a stop runs where it was
/// raised.
pub(super) fn keep_for_stop(window: &'static Window) {
    if crate::output::stop_stack_kept() {
        return;
    }
    match Stack::map(window, STOP_STACK_BYTES) {
        Some(stack) => {
            crate::output::keep_stop_stack(stack.bottom()..stack.top());
            // Kept for the boot: a stop may come at any time.
            core::mem::forget(stack);
        }
        None => crate::output::arguments(format_args!(
            "STOP stack of {} KiB not kept: the host's window had no room; a stop runs on the \
             stack it was raised on\n",
            STOP_STACK_BYTES >> 10
        )),
    }
}

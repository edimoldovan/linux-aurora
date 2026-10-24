// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Device-wide publication state that physical queues share without the device mutex.
//!
//! A firmware session owns its port, and the views it holds name that session's
//! firmware-shared memory. Code that does not hold the device mutex reaches the port only
//! through a preparation lease ([`super::Shared::port`]): shutdown and device loss join every
//! lease before they detach the session. Each piece below is atomic or has its own lock.
//!
//! Lock order: device mutex -> compute queue cell -> the locks of this port.

use super::{
    fw::{
        channels::{WorkSlot, WORK_RING_SLOTS},
        queue::{DataMaster, Policy},
    },
    qos,
};
use core::sync::atomic::{fence, AtomicBool, AtomicU32, AtomicU64, Ordering};
use kernel::{
    new_spinlock,
    prelude::*,
    sync::{Arc, SetOnce, SpinLock},
};

const PRIORITIES: usize = Policy::PRIORITIES as usize;

/// One priority class's compute work ring: its slot array and the cursors of its two firmware
/// consumers and of the host producer.
#[derive(Clone, Copy)]
pub(super) struct RingView {
    pub(super) slots: *mut u8,
    pub(super) consumers: [*const AtomicU32; 2],
    pub(super) producer: *const AtomicU32,
}

impl RingView {
    fn cursors(&self) -> Result<[u32; 3]> {
        // SAFETY: The views name 32-bit words of the session's retained firmware objects, which
        // the port's owner keeps alive. Other agents only use single 32-bit accesses.
        let values = unsafe {
            [
                (*self.consumers[0]).load(Ordering::Relaxed),
                (*self.consumers[1]).load(Ordering::Relaxed),
                (*self.producer).load(Ordering::Relaxed),
            ]
        };
        if values.iter().any(|&value| value >= WORK_RING_SLOTS) {
            return Err(EINVAL);
        }
        Ok(values)
    }

    /// Slots the host may still fill before reaching either consumer.
    fn free(&self) -> Result<u32> {
        let [first, second, producer] = self.cursors()?;
        let room = |consumer: u32| (consumer + WORK_RING_SLOTS - producer - 1) % WORK_RING_SLOTS;
        Ok(room(first).min(room(second)))
    }
}

/// Raw views of the session's firmware-shared memory, fixed once the firmware is built.
pub(super) struct Views {
    pub(super) compute: [RingView; PRIORITIES],
    pub(super) qos: *mut u8,
    pub(super) recovery: *const AtomicU32,
    pub(super) mailbox: *const super::Mailbox,
}

// SAFETY: The views only name memory and a mailbox owned by the firmware session, which outlives
// every user of the port. Accesses are atomic words or serialized by the port's locks.
unsafe impl Send for Views {}
// SAFETY: As above.
unsafe impl Sync for Views {}

#[pin_data]
pub(super) struct Port {
    views: SetOnce<Views>,
    /// Compute work-ring slots reserved by publishers that have not exposed them yet.
    #[pin]
    rings: SpinLock<[u32; PRIORITIES]>,
    #[pin]
    qos: SpinLock<qos::Accounting>,
    /// The accepted recovery generation, mirrored for publishers without the device mutex.
    generation: AtomicU64,
    /// Device publication order shared by compute and render commands.
    publications: AtomicU64,
    /// Full firmware effort is applied; cleared before an idle clear rechecks `submissions`.
    effort_full: AtomicBool,
    submissions: AtomicU64,
}

/// A compute work-ring slot reserved before any command becomes visible. Dropping an unused
/// claim returns the slot.
pub(super) struct OuterClaim<'a> {
    port: &'a Port,
    priority: usize,
    armed: bool,
}

impl Drop for OuterClaim<'_> {
    fn drop(&mut self) {
        if self.armed {
            let mut reserved = self.port.rings.lock();
            reserved[self.priority] = reserved[self.priority].saturating_sub(1);
        }
    }
}

impl Port {
    pub(super) fn new() -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
                views: SetOnce::new(),
                rings <- new_spinlock!([0; PRIORITIES], "g17::Port::rings"),
                qos <- new_spinlock!(qos::Accounting::new(), "g17::Port::qos"),
                generation: AtomicU64::new(0),
                publications: AtomicU64::new(0),
                effort_full: AtomicBool::new(false),
                submissions: AtomicU64::new(0),
            }),
            GFP_KERNEL,
        )
    }

    /// Called once while the session is built, before any publisher can see the port.
    pub(super) fn attach(&self, views: Views) -> Result {
        if self.views.populate(views) {
            Ok(())
        } else {
            Err(EBUSY)
        }
    }

    fn views(&self) -> Result<&Views> {
        self.views.as_ref().ok_or(ENODEV)
    }

    fn ring(&self, priority: u8) -> Result<(usize, RingView)> {
        let index = usize::from(priority);
        Ok((index, *self.views()?.compute.get(index).ok_or(EINVAL)?))
    }

    /// Reserves one slot of the compute work ring of `priority`. EAGAIN while firmware has not
    /// consumed enough slots; only consumption, never a waiting publisher, frees one.
    pub(super) fn reserve_compute(&self, priority: u8) -> Result<OuterClaim<'_>> {
        let (index, ring) = self.ring(priority)?;
        let mut reserved = self.rings.lock();
        if ring.free()? <= reserved[index] {
            return Err(EAGAIN);
        }
        reserved[index] += 1;
        Ok(OuterClaim {
            port: self,
            priority: index,
            armed: true,
        })
    }

    /// Whether a compute publication at `priority` can reserve a slot now.
    pub(super) fn compute_ready(&self, priority: u8) -> Result<bool> {
        let (index, ring) = self.ring(priority)?;
        let reserved = self.rings.lock();
        Ok(ring.free()? > reserved[index])
    }

    /// Exposes `record` in its reserved slot. Inner queue records must be visible first. Returns
    /// the new producer, the retained consumer witness for this slot.
    pub(super) fn publish_compute(&self, mut claim: OuterClaim<'_>, record: &WorkSlot) -> Result<u32> {
        if record.data_master != DataMaster::Compute as u32 || !core::ptr::eq(claim.port, self) {
            return Err(EINVAL);
        }
        let ring = self.views()?.compute[claim.priority];
        let mut reserved = self.rings.lock();
        let [first, second, producer] = ring.cursors()?;
        let next = (producer + 1) % WORK_RING_SLOTS;
        if next == first || next == second {
            return Err(EIO);
        }
        // SAFETY: The ring view spans WORK_RING_SLOTS records, checked when the session built
        // it; `producer` is in range. The reservation proves firmware does not own this slot and
        // the ring lock excludes every other host writer. The record is a padding-free ABI value.
        unsafe {
            core::ptr::copy_nonoverlapping(
                core::ptr::from_ref(record).cast::<u8>(),
                ring.slots.add(producer as usize * core::mem::size_of::<WorkSlot>()),
                core::mem::size_of::<WorkSlot>(),
            );
        }
        fence(Ordering::SeqCst);
        // SAFETY: As for `RingView::cursors`.
        unsafe { (*ring.producer).store(next, Ordering::Relaxed) };
        fence(Ordering::SeqCst);
        reserved[claim.priority] = reserved[claim.priority].saturating_sub(1);
        claim.armed = false;
        Ok(next)
    }

    fn qos_view(&self) -> Result<qos::View<'_>> {
        // SAFETY: The session retains the QoS table for at least as long as this port.
        unsafe { qos::View::from_raw(self.views()?.qos) }
    }

    pub(super) fn qos_publish(
        &self,
        owner: qos::Owner,
        scheduler: u64,
        policy: Policy,
    ) -> Result<qos::Publication> {
        let view = self.qos_view()?;
        self.qos.lock().publish(&view, owner, scheduler, policy, qos::clock)
    }

    pub(super) fn qos_cancel(&self, publication: qos::Publication) -> Result {
        let view = self.qos_view()?;
        self.qos.lock().cancel(&view, publication)
    }

    pub(super) fn qos_complete(&self, owners: &[qos::Owner]) -> Result {
        let view = self.qos_view()?;
        self.qos.lock().complete(&view, owners)
    }

    /// Rings the primary coprocessor's doorbell.
    pub(super) fn notify(&self, message: u64) -> Result {
        // SAFETY: The session's primary coprocessor owns the mailbox and outlives the port.
        let mailbox = unsafe { &*self.views()?.mailbox };
        mailbox.send(super::EP_DOORBELL, message)
    }

    pub(super) fn recovery_state(&self) -> Result<u32> {
        let word = self.views()?.recovery;
        fence(Ordering::Acquire);
        // SAFETY: As for `RingView::cursors`.
        Ok(unsafe { (*word).load(Ordering::Relaxed) })
    }

    pub(super) fn generation(&self) -> u64 {
        self.generation.load(Ordering::Acquire)
    }

    /// Called with the device mutex held after the accepted generation changed, before
    /// publication reopens.
    pub(super) fn set_generation(&self, generation: u64) {
        self.generation.store(generation, Ordering::Release);
    }

    pub(super) fn next_publication(&self) -> u64 {
        self.publications.fetch_add(1, Ordering::Relaxed) + 1
    }

    /// A publisher without the device mutex counts itself before it checks that full effort is
    /// applied. The idle clear withdraws the flag before it rechecks the count, so either the
    /// clear sees this submission or this submission sees the flag withdrawn.
    pub(super) fn note_submission(&self) -> bool {
        self.submissions.fetch_add(1, Ordering::SeqCst);
        self.effort_full.load(Ordering::SeqCst)
    }

    pub(super) fn submissions(&self) -> u64 {
        self.submissions.load(Ordering::SeqCst)
    }

    pub(super) fn set_effort_full(&self, full: bool) {
        self.effort_full.store(full, Ordering::SeqCst);
    }
}

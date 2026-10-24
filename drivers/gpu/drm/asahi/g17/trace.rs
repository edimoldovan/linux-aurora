// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Command lifecycle tracepoints (tracefs `events/asahi_g17`). Each call is a
//! static-key test in C while its event is disabled.

use kernel::{bindings, dma_fence::RawDmaFence, prelude::*, sync::Arc};

unsafe extern "C" {
    fn asahi_g17_trace_publish_enabled() -> bool;
    fn asahi_g17_trace_publish(engine: u8, qid: u8, ordinal: u32, ctx: u32, seq: u64,
        fence: *mut bindings::dma_fence);
    fn asahi_g17_trace_complete(engine: u8, qid: u8, ordinal: u32, ctx: u32, seq: u64,
        gpu_start: u64, gpu_end: u64);
    fn asahi_g17_trace_bind(engine: u8, qid: u8, ctx: u32, kind: u8);
    fn asahi_g17_trace_event(phase: u8);
}

pub(super) const RENDER: u8 = 0;
pub(super) const COMPUTE: u8 = 1;

/// Called once the command's doorbell has been rung.
pub(super) fn publish(engine: u8, qid: u8, ordinal: u32, packet: &Arc<super::job::Packet>) {
    // SAFETY: Pure static-key query.
    if !unsafe { asahi_g17_trace_publish_enabled() } {
        return;
    }
    let fence = packet.completion.fence();
    // SAFETY: `fence` holds a reference for the duration of the call.
    unsafe {
        asahi_g17_trace_publish(engine, qid, ordinal, packet.context.id(), packet.order.sequence,
            fence.raw())
    };
}

/// Called when exact completion with GPU timestamps is retired.
pub(super) fn complete(engine: u8, qid: u8, ordinal: u32, packet: &super::job::Packet,
    gpu: [u64; 2])
{
    // SAFETY: Plain values only.
    unsafe {
        asahi_g17_trace_complete(engine, qid, ordinal, packet.context.id(), packet.order.sequence,
            gpu[0], gpu[1])
    };
}

/// A physical queue was bound to a logical owner: 0 fresh, 1 rebound, 2 same VM.
pub(super) fn bind(engine: u8, qid: u8, ctx: u32, kind: u8) {
    // SAFETY: Plain values only.
    unsafe { asahi_g17_trace_bind(engine, qid, ctx, kind) };
}

/// Event worker pass boundaries: 0 begin, 1 fences signalled, 2 end.
pub(super) fn event(phase: u8) {
    // SAFETY: Plain values only.
    unsafe { asahi_g17_trace_event(phase) };
}

// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Compile-only M3 backend. No firmware or memory owner is constructed here.

// All imported modules are type-checked; live startup stays gated.
#![allow(dead_code)]

use kernel::{device::Core, platform, prelude::*};

#[derive(Clone, Copy)]
pub(crate) enum Chip {
    T8122,
    T6030,
    T6031,
    T6032,
}

/// Refuse M3 before DMA setup, MMIO mapping, firmware access or DRM allocation.
pub(crate) fn refuse_probe(pdev: &platform::Device<Core<'_>>, chip: Chip) -> Error {
    // pending provenance decision: T8122 register lists and InitData words.
    // [ryan: adapted to 7.2.10 — minimal, pending DJ review: M3 needs an
    // independent event/runtime owner and current GPUVM backing lifetimes.]
    match chip {
        Chip::T8122 => dev_err!(
            pdev.as_ref(),
            "M3/T8122 unavailable: pending provenance decision; backend is compile-only\n"
        ),
        _ => dev_err!(
            pdev.as_ref(),
            "M3 unavailable: event/runtime and GPUVM adaptation pending DJ review\n"
        ),
    }
    ENODEV
}

mod m3_compute_layout;

mod m3_compute_sequence;

mod m3_compute_storage;

mod m3_coverage;

mod m3_init_layout;

mod m3_init_storage;

mod m3_parameter_layout;

mod m3_pass_layout;

mod m3_pool_layout;

mod m3_queue_layout;

mod m3_render_sequence;

mod m3_scene_layout;

mod m3_shared_layout;

mod m3_state_layout;

mod m3_sync_layout;

mod m3_thermal_policy;

mod agx_host_progress;

mod agx_memory_stats;

mod agx_timing_stats;

mod m3_timeline;

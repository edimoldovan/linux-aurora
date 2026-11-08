// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Hardware configuration for t6050 platforms (M5 Max), GPU generation G17,
//! variant C.
//!
//! INCREMENT 1 — the interface-version boot gate. Real for this step: the chip
//! identity, the g17c interface version, and a device-tree-consistent 14-OPP
//! perf config. Still placeholders (the firmware consumes them only AFTER its
//! interface-version check): the register windows, the perf curves, and the
//! fixed buffers, all taken from t8140. The chip-variant ABI structs the full
//! port needs (PERF_STATES 14, Root 0xc8, HwData +0x128, RegionC +0xe38,
//! StatusBlock B +0x1e0) are NOT yet applied — see kernel/GPU.md.

use crate::g17::fw::initdata::PerfConfig;
use crate::g17::{Config, FixedBuffer, Identity};
use crate::hw::{t8140, GpuVariant};

/// Static configuration of the T6050 GPU.
pub(crate) const CONFIG: Config = Config {
    chip_id: CHIP_ID,
    identity: Identity {
        // ID register 0x0b041100: family 0x0b, variant 0x04 (probe).
        family: 0xb,
        variant: 0x4,
        num_dies: 1,
        gpu_variant: GpuVariant::C,
        // Placeholders (t8140); the g17c firmware's expected values are
        // unconfirmed and consumed after the version gate.
        usc_generation: 3,
        hal_generation: 200,
    },
    uat_ias: 42,
    uat_oas: 42,
    // Placeholder (t8140); consumed after the version gate.
    secondary_root_offset: 0x40000,
    // Crash-log buffers: the firmware requests these during RTKit init, before
    // the version gate. VAs observed from the firmware's RTKit buffer requests
    // (masked low 40 bits of iova 0x10002864000 / 0x10002920000); they match the
    // GFX / GFX1 __DATA_SHARED_RO segments from the proxy probe.
    crash_buffers: [
        FixedBuffer {
            va: 0x286_4000,
            size: 0x8000,
        },
        FixedBuffer {
            va: 0x292_0000,
            size: 0x8000,
        },
    ],
    // Placeholder (t8140); consumed after the version gate.
    pm_metrics: t8140::CONFIG.pm_metrics,
    // Placeholder register map (t8140); the firmware reads it only after the
    // version check. The real T6050 map is the sgx-control group + MCC, grown
    // on firmware faults — see kernel/GPU.md and GPU-t6050-dt-config.md.
    register_windows: &t8140::REGISTER_WINDOWS,
    reserved_slots: &t8140::RESERVED_REGISTER_SLOTS,
    perf: &PERF,
    opps: 14,
    // g17c firmware interface version (GFX 0x218a4, GFX1 0x26ce4). Accepted by
    // the firmware (gpu4: no mismatch; crash is downstream on G17P-layout structs).
    interface_version: [0x04c0, 0x8728, 0x4339, 0x0c8e],
};

/// Chip identifier written to the hardware data.
pub(crate) const CHIP_ID: u32 = 0x6050;

/// Placeholder perf config. The M5 Max has 14 OPPs / 13 active states, but the
/// shared `PERF_STATES` is still 11, so only the first 11 states are mapped here
/// (enough to build valid init data and reach the version gate). `relative_*`
/// are t8140's (macOS-measured, not statically recoverable for T6050); SRAM
/// floor is the T6050 value. DVFS tuning only, after the version gate.
pub(crate) const PERF: PerfConfig = PerfConfig {
    states_a: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
    states_b: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
    relative_a: [0, 15, 22, 29, 42, 56, 74, 86, 90, 92, 100],
    relative_b: [0, 0, 13, 24, 40, 52, 63, 73, 83, 88, 100],
    sram_min_mv: 775,
};

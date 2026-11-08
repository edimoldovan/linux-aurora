// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Hardware configuration for t6050 platforms (M5 Max), GPU generation G17,
//! variant C (g17c firmware, HAL300 command stream).
//!
//! The register map is the one macOS hands the firmware on this machine. Still
//! taken from t8140: the perf curves, the secondary root offset and the
//! parameter-buffer metrics buffer. See kernel/GPU.md.

use crate::g17::fw::initdata::{PerfConfig, RegisterWindow};
use crate::g17::{Abi, Config, FixedBuffer, Identity};
use crate::hw::{t8140, GpuVariant};

/// Static configuration of the T6050 GPU.
pub(crate) const CONFIG: Config = Config {
    abi: Abi::G17c,
    chip_id: CHIP_ID,
    identity: Identity {
        // ID register 0x0b041100: family 0x0b, variant 0x04 (probe).
        family: 0xb,
        variant: 0x4,
        num_dies: 1,
        gpu_variant: GpuVariant::C,
        // AGX3 shaders and the HAL300 command stream (Mesa's asahi_drm.h:
        // DRM_ASAHI_GPU_HAL_300 for G17G/G17S/G17C).
        usc_generation: 3,
        hal_generation: 300,
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
    // The requested iovas are the physical addresses of those segments, where the
    // firmware writes its crash logs.
    crash_log_phys: true,
    // Placeholder (t8140); consumed after the version gate.
    pm_metrics: t8140::CONFIG.pm_metrics,
    register_windows: &REGISTER_WINDOWS,
    reserved_slots: &RESERVED_REGISTER_SLOTS,
    perf: &PERF,
    opps: 14,
    // g17c firmware interface version (GFX 0x218a4, GFX1 0x26ce4). Accepted by
    // the firmware (gpu4: no mismatch; crash is downstream on G17P-layout structs).
    interface_version: [0x04c0, 0x8728, 0x4339, 0x0c8e],
};

/// Chip identifier written to the hardware data.
pub(crate) const CHIP_ID: u32 = 0x6050;

/// Base of the firmware MMIO aperture in the kernel window.
const MMIO_APERTURE: u64 = 0xffff_fc21_8000_0000;
/// GPU register block (sgx).
const SGX: u64 = 0x23_0000_0000;

const fn window(slot: usize, phys: u64, offset: u64, size: u32, unk_18: u64) -> RegisterWindow {
    RegisterWindow {
        slot,
        phys,
        va: MMIO_APERTURE + offset,
        size,
        elements: 1,
        phys_stride: 0,
        unk_18,
        unk_20: 2,
    }
}

/// A window of `elements` blocks of `element` bytes, `phys_stride` apart physically.
const fn array(slot: usize, phys: u64, offset: u64, element: u32, elements: u32, phys_stride: u64) -> RegisterWindow {
    RegisterWindow {
        elements,
        phys_stride,
        ..window(slot, phys, offset, element * elements, 0)
    }
}

/// A window macOS describes with an address but type 0 (`unk_20` 0).
const fn disabled(slot: usize, phys: u64, offset: u64, size: u32) -> RegisterWindow {
    RegisterWindow {
        unk_20: 0,
        ..window(slot, phys, offset, size, 0)
    }
}

/// Register windows the firmware maps, by register-map slot, as the macOS
/// AGXG17X kext fills them on this machine (workflow wf_a0538809: its static
/// sgx table, its per-block filler functions and the ADT). Slots macOS leaves
/// empty stay empty here.
pub(crate) const REGISTER_WINDOWS: [RegisterWindow; 30] = [
    // GPCAICSWInt: the AIC software-interrupt page.
    window(0, 0x2_8041_4000, 0xc_0000, 0x4000, 0),
    // MCache: one block per GPU cluster.
    array(3, 0x22_2010_4000, 0x10_0000, 0x2_4000, 4, 0x200_0000),
    disabled(4, 0x2_8046_8000, 0x1a_0000, 0x4000),
    // Push telemetry dashboard and its config space; the read space is disabled.
    window(9, 0x2_8425_0000, 0x1a_8000, 0x4000, 0),
    disabled(10, 0x2_8424_0000, 0x1b_0000, 0x4000),
    window(11, 0x2_8425_8000, 0x1b_8000, 0x4000, 0),
    window(12, 0x5_0965_c000, 0x1c_0000, 0x4000, 0),
    disabled(14, 0x23_8030_0000, 0x1c_8000, 0x8000),
    window(15, 0x2_8cdc_0000, 0x1d_0000, 0x2_4000, 0),
    // RGX: the sgx base.
    window(17, SGX, 0x8_0000, 0x2_1500, 0),
    window(18, SGX + 0xeb_4000, 0x20_0000, 0x4000, 0),
    // AFR: four blocks 0x20_0000 apart.
    array(20, SGX + 0x300_0000, 0x30_0000, 0x4_0000, 4, 0x20_0000),
    window(22, SGX + 0x100_0000, 0xb_0000, 0x8000, 0),
    window(23, SGX + 0xf0_0000, 0x20_8000, 0x1_0000, 0),
    // DVFM of the GPU: one block per cluster.
    array(24, SGX + 0xec_0000, 0x22_0000, 0x1_0000, 4, 0),
    window(31, SGX + 0xd0_4000, 0x0_0000, 0x8000, 0xd0_4000),
    window(33, SGX + 0xd5_0000, 0x1_0000, 0x1_0000, 0xd5_0000),
    window(34, SGX + 0xd1_0000, 0x2_0000, 0x4000, 0xd1_0000),
    window(36, SGX + 0xd4_0000, 0x2_8000, 0x4000, 0xd4_0000),
    window(37, SGX + 0xd6_0000, 0x3_0000, 0x2_0000, 0xd6_0000),
    window(38, SGX + 0xd4_4000, 0x5_4000, 0x4000, 0xd4_4000),
    window(39, SGX + 0xd4_c000, 0x5_c000, 0x200, 0xd4_c000),
    window(40, SGX + 0xe0_0000, 0x6_0000, 0x4000, 0xe0_0000),
    window(42, SGX + 0xe4_0000, 0x6_8000, 0x4000, 0xe4_0000),
    // Metrology sensors.
    array(44, SGX + 0xe0_8000, 0x26_0000, 0x8000, 2, 0),
    // Throttle state counters.
    window(46, SGX + 0xe3_3800, 0x27_3800, 0x4000, 0),
    window(48, SGX + 0xe6_0000, 0x7_0000, 0x58, 0xe6_0000),
    window(52, SGX + 0xe4_8000, 0x28_0000, 0x8000, 0),
    // AFRDCPMS ("TMB AFR DCPMS").
    window(53, SGX + 0x2_3d00, 0xc_bd00, 0x200, 0),
    window(58, SGX + 0xe7_3800, 0x29_3800, 0x4000, 0),
];

/// Register-map slots macOS names with type 2 but gives no window.
pub(crate) const RESERVED_REGISTER_SLOTS: [usize; 17] = [
    2, 5, 6, 7, 8, 21, 25, 26, 27, 32, 35, 41, 43, 45, 47, 49, 54,
];

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

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

use crate::g17::fw::initdata::{PerfConfig, RegisterWindow, SCRATCH_WINDOW_PHYS};
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
        unk_18,
        unk_20: 2,
    }
}

/// A slot whose register block is not located yet: host scratch memory at
/// `MMIO_APERTURE + 0x200_0000 + (slot << 21)`, so the firmware's accesses land
/// in RAM, and one past the window faults at an address that names the slot.
/// Four elements: gpu11 showed slot 24 (DVFMMgpu) written at +0x4000 in each
/// of four element-size strides, one per cluster; gpu12 read slot 20 (AFR)
/// at +0x40010.
const fn scratch(slot: usize) -> RegisterWindow {
    RegisterWindow {
        slot,
        phys: SCRATCH_WINDOW_PHYS,
        va: MMIO_APERTURE + 0x200_0000 + ((slot as u64) << 21),
        size: 0x10_0000,
        elements: 4,
        unk_18: 0,
        unk_20: 2,
    }
}

/// Register windows the firmware maps, by register-map slot (g17c numbering,
/// from the slot-name table in the GFX1 firmware data at 0xfffffc000007bf88).
/// The sgx-control group and slots 17/53 are the macOS AGX kext's T6050 table
/// (GPU-t6050-dt-config.md). Slots the kext marks "(unused)" stay empty: 1, 3,
/// 4, 14, 44, 52 (and the reserved ones).
pub(crate) const REGISTER_WINDOWS: [RegisterWindow; 44] = [
    // GPCAICSWInt: the AIC software-interrupt page (ADT sgx meta-sw-interrupt).
    window(0, 0x2_8041_4000, 0xc_0000, 0x4000, 0),
    // RGX: the sgx base. gpu6 faulted on slot 17 at +0x21038.
    window(17, SGX, 0x8_0000, 0x2_1500, 0),
    // GFXCAXI2AF, by analogy with t8140 (sgx + 0x100_0000).
    window(22, SGX + 0x100_0000, 0xb_0000, 0x8000, 0),
    window(31, SGX + 0xd0_4000, 0x0_0000, 0x8000, 0xd0_4000),
    window(33, SGX + 0xd5_0000, 0x1_0000, 0x1_0000, 0xd5_0000),
    window(34, SGX + 0xd1_0000, 0x2_0000, 0x4000, 0xd1_0000),
    window(36, SGX + 0xd4_0000, 0x2_8000, 0x4000, 0xd4_0000),
    window(37, SGX + 0xd6_0000, 0x3_0000, 0x2_0000, 0xd6_0000),
    window(38, SGX + 0xd4_4000, 0x5_4000, 0x4000, 0xd4_4000),
    window(39, SGX + 0xd4_c000, 0x5_c000, 0x200, 0xd4_c000),
    window(40, SGX + 0xe0_0000, 0x6_0000, 0x4000, 0xe0_0000),
    window(42, SGX + 0xe4_0000, 0x6_8000, 0x4000, 0xe4_0000),
    window(48, SGX + 0xe6_0000, 0x7_0000, 0x58, 0xe6_0000),
    // AFRDCPMS ("TMB AFR DCPMS").
    window(53, SGX + 0x2_3d00, 0xc_bd00, 0x200, 0),
    scratch(9),
    scratch(10),
    scratch(11),
    scratch(12),
    scratch(13),
    scratch(15),
    scratch(16),
    scratch(18),
    scratch(19),
    scratch(20),
    scratch(23),
    scratch(24),
    scratch(25),
    scratch(26),
    scratch(27),
    scratch(29),
    scratch(30),
    scratch(32),
    scratch(35),
    scratch(43),
    scratch(45),
    scratch(46),
    scratch(47),
    scratch(49),
    scratch(54),
    scratch(55),
    scratch(56),
    scratch(57),
    scratch(58),
    scratch(59),
];

/// Register-map slots that are reserved but have no window (t8140's low ones;
/// in g17c numbering they are blocks the kext marks "(unused)").
pub(crate) const RESERVED_REGISTER_SLOTS: [usize; 6] = [2, 5, 6, 7, 8, 21];

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

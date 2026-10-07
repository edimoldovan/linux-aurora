// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The per-SoC facts of the M3 runtime backend.
//!
//! Each table describes the configuration of one GPU. This compile-only subset holds only the
//! facts the M3 boundary carries; the register windows, memory layout, firmware images and
//! admission of the full table stay with the M3 runtime port. Nothing here is read by a probe.

use super::m3_compute_layout::RegisterSet;

/// The HwDataB words of the runtime backend's InitData that are neither its GPU VA layout nor
/// device-tree values: configuration words where the manager layout keeps 0 or its validated
/// values (including the A/B pair +0xa98/+0xb40 at 0), and the two unit masks. They are values
/// of one SoC's runtime InitData (`m3_soc::Soc::hwdata_b`), not derived from the device tree.
#[derive(Debug)]
pub(crate) struct G15RuntimeHwDataB {
    pub(crate) unk_454: u32,
    pub(crate) unk_464: u32,
    pub(crate) unk_a7c: u64,
    pub(crate) unk_a98: u32,
    pub(crate) unk_abc: u64,
    pub(crate) unk_ae4: u32,
    pub(crate) unk_b20: u32,
    pub(crate) unk_b24: u64,
    /// +0xb40, the "UAT enabled" word of the manager layout.
    pub(crate) unk_554: u32,
    pub(crate) unk_17b8: u32,
    /// Unit enable masks, +0x17c0/+0x17c8.
    pub(crate) unit_mask_a: u64,
    pub(crate) unit_mask_b: u32,
    pub(crate) unk_1808: u32,
    pub(crate) unk_1818: u32,
}

pub(crate) struct Soc {
    /// SoC name, for logs.
    pub(crate) name: &'static str,
    /// The runtime InitData's HwDataB configuration words and unit masks.
    pub(crate) hwdata_b: Option<&'static G15RuntimeHwDataB>,
    /// The register lists the GPU's firmware expects in compute and render commands.
    pub(crate) registers: RegisterSet,
    /// Words of the generated HwData object (offset into the object, value) that this SoC's
    /// firmware takes with other values than the shared builder writes.
    pub(crate) hwdata_words: &'static [(usize, u32)],
    /// The same for the Globals object.
    pub(crate) globals_words: &'static [(usize, u32)],
}

/// The HwDataB configuration words and unit masks of the T6030 runtime InitData, as checked on
/// J514S and J516S (the runtime's earlier fixed values).
pub(crate) static T6030_HWDATA_B: G15RuntimeHwDataB = G15RuntimeHwDataB {
    unk_454: 1,
    unk_464: 1,
    unk_a7c: 0x1_0000_0001,
    unk_a98: 0,
    unk_abc: 4,
    unk_ae4: 0x31,
    unk_b20: 0x14,
    unk_b24: 3,
    unk_554: 0,
    unk_17b8: 5,
    unit_mask_a: 0x7_0000_0003,
    unit_mask_b: 7,
    unk_1808: 1,
    unk_1818: 1,
};

/// The HwDataB configuration words and unit masks of the T8122 runtime InitData: T6030's, except
/// for four words the G15G firmware takes with other values (+0xa2c, the chip revision's major
/// number; +0xb20, the core slots; +0x17b8; +0x1818). The unit masks are T6030's.
pub(crate) static T8122_HWDATA_B: G15RuntimeHwDataB = G15RuntimeHwDataB {
    unk_454: 2,
    unk_b20: 0x0a,
    unk_17b8: 4,
    unk_1818: 0xffff_ffff,
    ..T6030_HWDATA_B
};

/// Every SoC the M3 runtime has a table for.
pub(crate) static SOCS: [&Soc; 2] = [&T6030, &T8122];

/// T6030 (M3 Pro, G15S): one die, two clusters of ten core slots.
pub(crate) static T6030: Soc = Soc {
    name: "T6030",
    hwdata_b: Some(&T6030_HWDATA_B),
    registers: RegisterSet::G15S,
    hwdata_words: &[],
    globals_words: &[],
};

/// T8122 (M3, G15G): one die, one cluster of ten core slots (eight or ten of them active).
pub(crate) static T8122: Soc = Soc {
    name: "T8122",
    hwdata_b: Some(&T8122_HWDATA_B),
    registers: RegisterSet::G15G,
    hwdata_words: &T8122_HWDATA_WORDS,
    globals_words: &T8122_GLOBALS_WORDS,
};

/// The T8122 HwData words that differ from the shared builder's (offsets into the object;
/// HwDataA starts at +0x4580):
/// - HwDataB +0xa30 and +0xa34: 0 and 4 (T6030: 1 and 0);
/// - HwDataB +0x17e0..+0x1860: the G15G firmware's flag words (T6030 has 1 at +0x17e8,
///   +0x1804, +0x1814 and +0x1860 and all-ones at +0x1848);
/// - HwDataA +0x11e0: 30 (T6030: 40), a word of the shader-engine controller block;
/// - HwDataA +0x1290: 125, and HwDataA +0x424c: 24000000, the 24 MHz reference clock.
static T8122_HWDATA_WORDS: [(usize, u32); 17] = [
    (0xa30, 0),
    (0xa34, 4),
    (0x17e0, 1),
    (0x17e8, 0),
    (0x17f4, 1),
    (0x17f8, 1),
    (0x1804, 0),
    (0x180c, 1),
    (0x1814, 0),
    (0x181c, 0xffff_ffff),
    (0x1848, 0),
    (0x184c, 0),
    (0x1858, 1),
    (0x1860, 0),
    (0x4580 + 0x11e0, 30),
    (0x4580 + 0x1290, 125),
    (0x4580 + 0x424c, 24_000_000),
];

/// The T8122 Globals words that differ from the shared builder's: +0x9bc, the CDM backoff
/// timeout (4) with the three bytes after it 0 (T6030: 1, 0, 0).
static T8122_GLOBALS_WORDS: [(usize, u32); 1] = [(0x9bc, 4)];

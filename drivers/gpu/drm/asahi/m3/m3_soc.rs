// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The per-SoC facts of the M3 runtime backend.
//!
//! Each table describes the configuration of one GPU. This compile-only subset holds only the
//! facts the M3 boundary carries; the register windows, memory layout, firmware images and
//! admission of the full table stay with the M3 runtime port. Nothing here is read by a probe.

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

/// Every SoC the M3 runtime has a table for.
pub(crate) static SOCS: [&Soc; 2] = [&T6030, &T8122];

/// T6030 (M3 Pro, G15S): one die, two clusters of ten core slots.
pub(crate) static T6030: Soc = Soc {
    name: "T6030",
    hwdata_b: Some(&T6030_HWDATA_B),
};

/// T8122 (M3, G15G): one die, one cluster of ten core slots (eight or ten of them active).
pub(crate) static T8122: Soc = Soc {
    name: "T8122",
    hwdata_b: None,
};

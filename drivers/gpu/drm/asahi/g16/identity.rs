// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Exact firmware and hardware identity.
//!
//! The firmware build is the SHA-256 of TEXT with the 8 entropy bytes after the single
//! `GKTS\x08\0\0\0` marker zeroed, plus the segment sizes, looked up in
//! [`super::hw::t8132::CONFIG`]`.firmware`; the hardware revision is checked against its
//! `hw_ids` at start ([`super::boot`]). An unknown build or revision is refused with its measured
//! identity. The canonical object VAs, the initdata graph, the board templates and the work
//! layouts all belong to one build, so nothing is written before the build is identified.

use kernel::{bindings, device, fmt, prelude::*};

use super::boot::glue;
use super::hw::FirmwareBuild;

/// Lowercase hex of a byte string, as `%*phN` prints it.
pub(crate) struct Hex<'a>(pub(crate) &'a [u8]);

impl fmt::Display for Hex<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for b in self.0 {
            write!(f, "{:02x}", b)?;
        }
        Ok(())
    }
}

/// Copies `size` bytes at physical `phys`, a range outside the kernel's System RAM, into a
/// private copy through a temporary write-combined mapping.
pub(crate) fn copy_physical(phys: u64, size: usize) -> Result<KVVec<u8>> {
    // SAFETY: memremap() of a boot-loader range creates a temporary mapping; it has no other
    // side effects.
    let map = unsafe { bindings::memremap(phys, size, bindings::MEMREMAP_WC as _) };
    if map.is_null() {
        return Err(ENOMEM);
    }
    let copy = KVVec::from_elem(0u8, size, GFP_KERNEL).map(|mut copy| {
        // SAFETY: `map` is a live mapping of `size` bytes and `copy` holds `size` bytes; the two do
        // not overlap.
        unsafe { core::ptr::copy_nonoverlapping(map as *const u8, copy.as_mut_ptr(), size) };
        copy
    });
    // SAFETY: `map` came from memremap() above and is not used afterwards.
    unsafe { bindings::memunmap(map) };
    Ok(copy?)
}

/// Both unaligned entropy records of the qualified J713/25G83 TEXT image.
const ENTROPY: [(usize, &[u8; 4]); 2] = [(0x5c8df, b"GKTS"), (0x5c9c2, b"ECAP")];

/// Identifies the loaded firmware build before any firmware-visible write.
pub(crate) fn firmware(
    dev: &device::Device,
    adt: &super::adt::GpuAdt,
    builds: &'static [FirmwareBuild],
) -> Result<&'static FirmwareBuild> {
    let size = usize::try_from(adt.text.end - adt.text.start)?;
    let data_size = adt.data.end - adt.data.start;
    let mut copy = copy_physical(adt.text.start, size)?;
    let again = copy_physical(adt.text.start, size)?;
    if copy.as_slice() != again.as_slice() {
        dev_err!(dev, "G16: firmware TEXT changed during stopped admission\n");
        return Err(EAGAIN);
    }
    for (at, tag) in ENTROPY {
        if copy.get(at..at + 4) != Some(tag.as_slice())
            || copy.get(at + 4..at + 8) != Some(8u32.to_le_bytes().as_slice())
            || at + 16 > size
        {
            dev_err!(dev, "G16: missing firmware entropy record at {:#x}\n", at);
            return Err(EINVAL);
        }
        copy[at + 8..at + 16].fill(0);
    }
    let at = ENTROPY[0].0;
    let digest = glue::sha256(&copy);
    for build in builds {
        if build.text_size == size as u64
            && build.data_size == data_size
            && build.marker as usize == at
            && build.text_sha256 == digest
        {
            dev_info!(dev, "GPU firmware build: {}\n", build.name);
            return Ok(build);
        }
    }
    dev_err!(
        dev,
        "unsupported GPU firmware build: TEXT {:#x} DATA {:#x} GKTS {:#x} sha256 {}\n",
        size,
        data_size,
        at,
        Hex(&digest)
    );
    for build in builds {
        dev_err!(dev, "supported: {}\n", build.name);
    }
    Err(EINVAL)
}

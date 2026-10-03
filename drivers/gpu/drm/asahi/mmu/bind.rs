// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Prepared userspace mapping transactions. Host callback resources are reserved for the
//! complete ioctl before its first visible mapping changes; leaf commits retain their
//! individual interruptible GPUVM reservation acquisition and quarantine admission check.

use super::*;

pub(crate) struct PreparedUserMap {
    gem: ARef<gem::Object>,
    vm_bo: ARef<gpuvm::GpuVmBo<VmInner>>,
    resources: VmOpResources,
    prot: Prot,
    addr: u64,
    size: u64,
    offset: u64,
    single_page: bool,
}

pub(crate) struct PreparedUserUnmap {
    pub(super) iova: u64,
    pub(super) size: u64,
    resources: VmOpResources,
}

pub(super) enum PreparedUserBindOp {
    Map(PreparedUserMap),
    Unmap(PreparedUserUnmap),
}

pub(crate) struct PreparedUserBindBatch {
    pub(super) ops: KVec<PreparedUserBindOp>,
}

impl PreparedUserBindBatch {
    pub(crate) fn new(capacity: usize) -> Result<Self> {
        Ok(Self {
            ops: KVec::with_capacity(capacity, GFP_KERNEL)?,
        })
    }
    pub(crate) fn push_map(&mut self, map: PreparedUserMap) -> Result {
        Ok(self.ops.push(PreparedUserBindOp::Map(map), GFP_KERNEL)?)
    }
    pub(crate) fn push_unmap(&mut self, unmap: PreparedUserUnmap) -> Result {
        Ok(self
            .ops
            .push(PreparedUserBindOp::Unmap(unmap), GFP_KERNEL)?)
    }
}

impl Vm {
    /// Prepares host state without taking DMA reservations or changing any leaf.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn prepare_bind_object(
        &self,
        gem: &gem::Object,
        addr: u64,
        size: u64,
        offset: u64,
        prot: Prot,
        single_page: bool,
    ) -> Result<PreparedUserMap> {
        let resources = VmOpResources {
            preallocated_gpuvas: [
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
            ],
        };
        // SAFETY: As in `Vm::bind_object()`, the device is bound while its file can issue binds.
        let dev = unsafe { self.dev.as_bound() };
        // Populate the scatter-gather table before any GPUVM lock is taken.
        gem.sg_table(dev)?;
        let vm_bo = self
            .gpuvm
            .obtain(gem, pin_init!(VmBoData { segments <- new_mutex!(None) }))?;
        {
            let mut segments = vm_bo.data().segments.lock();
            if segments.is_none() {
                segments.replace(prefetch_sgt_segments(dev, gem)?);
            }
        }
        if (addr | size | offset) & UAT_PGMSK as u64 != 0 {
            return Err(EINVAL);
        }
        Ok(PreparedUserMap {
            gem: gem.into(),
            vm_bo,
            resources,
            prot,
            addr,
            size,
            offset,
            single_page,
        })
    }

    pub(crate) fn prepare_user_unmap(&self, iova: u64, size: u64) -> Result<PreparedUserUnmap> {
        Ok(PreparedUserUnmap {
            iova,
            size,
            resources: VmOpResources {
                preallocated_gpuvas: [
                    Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                    Some(GpuVaAlloc::<VmInner>::new(GFP_KERNEL)?),
                    None,
                ],
            },
        })
    }

    /// Commits in ioctl order. Quarantine may defer an all-unmap batch, but can
    /// never allow a map to replace a mapping still owned by failed work.
    pub(crate) fn commit_prepared_user_bind_batch(
        &self,
        batch: &mut PreparedUserBindBatch,
    ) -> Result {
        if self.defer_bind_batch(batch)? {
            return Ok(());
        }
        let mut index = 0;
        while index < batch.ops.len() {
            let result = match &mut batch.ops[index] {
                PreparedUserBindOp::Map(map) => {
                    // SAFETY: See `lock_vm()`. As in `Vm::bind_object()`, the object itself is
                    // not locked: the map step may still populate its pages.
                    let mut inner = unsafe { self.gpuvm.exec_lock(None, true)? };
                    match self.mapping_commit() {
                        Ok(commit) => {
                            let (flags, repeat) = if map.single_page {
                                (GpuVaFlag::REPEAT.into(), UAT_PGSZ as u32)
                            } else {
                                (GpuVaFlags::empty(), 0)
                            };
                            // SAFETY: As in `Vm::bind_object()`.
                            let dev = unsafe { self.dev.as_bound() };
                            let mut ctx = StepContext::new(Some(dev), map.prot, &mut map.resources)?;
                            let result = inner.unique_gpuvm.sm_map(OpMapRequest {
                                addr: map.addr,
                                range: map.size,
                                gem_offset: map.offset,
                                gem_range: repeat,
                                vm_bo: &map.vm_bo,
                                context: &mut ctx,
                                flags,
                            });
                            if result.is_ok() {
                                self.track_shared_binding(
                                    &map.gem,
                                    map.addr,
                                    map.size,
                                    map.offset,
                                    map.single_page,
                                );
                            }
                            drop(inner);
                            drop(commit);
                            Some(result)
                        }
                        Err(error) if error == EIO => {
                            drop(inner);
                            None
                        }
                        Err(error) => return Err(error),
                    }
                }
                PreparedUserBindOp::Unmap(unmap) => {
                    // SAFETY: See `lock_vm()`.
                    let mut inner = unsafe { self.gpuvm.exec_lock(None, true)? };
                    match self.mapping_commit() {
                        Ok(commit) => {
                            let mut ctx =
                                StepContext::new(None, Prot::default(), &mut unmap.resources)?;
                            let result =
                                inner
                                    .unique_gpuvm
                                    .sm_unmap(unmap.iova, unmap.size, &mut ctx);
                            if result.is_ok() {
                                self.untrack_shared_range(unmap.iova, unmap.size);
                            }
                            drop(inner);
                            drop(commit);
                            Some(result)
                        }
                        Err(error) if error == EIO => {
                            drop(inner);
                            None
                        }
                        Err(error) => return Err(error),
                    }
                }
            };
            if let Some(result) = result {
                result?;
                index += 1;
            } else if self.defer_bind_batch(batch)? {
                return Ok(());
            }
        }
        Ok(())
    }
}

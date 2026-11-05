/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Low-level CPU offline paths for Apple silicon spin-table CPUs.
 *
 * The dying CPU calls apple_cpu_mmu_off_jump() through the idmap. It turns
 * the MMU and caches off and branches to the physical address of one of the
 * two exits below, which then run with the MMU off from their physical
 * address. Only the trampoline has to live in .idmap.text.
 *
 * Neither exit touches memory between turning the caches off and cleaning
 * them, so no stale dirty line can overwrite a non-cacheable store.
 */

#include <linux/linkage.h>
#include <linux/cfi_types.h>
#include <asm/assembler.h>
#include <asm/sysreg.h>

/* Apple CYC_OVRD (per core): S3_5_C15_C5_0. Field layout from m1n1 cpu_regs.h. */
#define APPLE_CYC_OVRD_MODES_MASK	(0x3f << 20)	/* FIQ[21:20] IRQ[23:22] WFI[25:24] */
#define APPLE_CYC_OVRD_FIQ_MODE_2	(2 << 20)
#define APPLE_CYC_OVRD_IRQ_MODE_2	(2 << 22)
#define APPLE_CYC_OVRD_WFI_MODE_3	(3 << 24)
#define APPLE_CYC_OVRD_DIS_WFI_RET	(1 << 0)
#define APPLE_CYC_OVRD_CORE_OFF		(APPLE_CYC_OVRD_FIQ_MODE_2 | APPLE_CYC_OVRD_IRQ_MODE_2 | \
					 APPLE_CYC_OVRD_WFI_MODE_3 | APPLE_CYC_OVRD_DIS_WFI_RET)

	.pushsection ".idmap.text", "a"
/*
 * apple_cpu_mmu_off_jump(entry_pa, arg0, arg1, arg2)
 *
 * Same shape as cpu_soft_restart() without the EL2 switch: with VHE the
 * write to SCTLR_EL1 is redirected to SCTLR_EL2. Must be called through the
 * idmap.
 */
SYM_TYPED_FUNC_START(apple_cpu_mmu_off_jump)
	mov_q	x12, INIT_SCTLR_EL1_MMU_OFF
	pre_disable_mmu_workaround
	msr	sctlr_el1, x12
	isb

	mov	x8, x0
	mov	x0, x1
	mov	x1, x2
	mov	x2, x3
	br	x8
SYM_FUNC_END(apple_cpu_mmu_off_jump)
	.popsection

	.text
/*
 * Clean and invalidate the data caches by set/way, from L1 up to (not
 * including) level x0 + 1. x0 = 1 is L1 only.
 *
 * This is the TF-A do_dcsw_op loop that m1n1 runs on M1 in mmu_disable()
 * (m1n1 src/memory_asm.S), restricted to DC CISW.
 * Uses only registers: clobbers x1-x11, x14, x16, x17. No stack.
 */
SYM_FUNC_START_LOCAL(apple_dcache_cisw_levels)
	lsl	x3, x0, #1			// x3 = (last level + 1) << 1
	cbz	x3, 4f
	mrs	x9, clidr_el1
	mov	x10, xzr			// x10 = current level << 1
	mov	w8, #1
1:	add	x2, x10, x10, lsr #1		// 3 * level
	lsr	x1, x9, x2
	and	x1, x1, #7			// cache type at this level
	cmp	x1, #2
	b.lo	3f				// no data cache here
	msr	csselr_el1, x10
	isb
	mrs	x1, ccsidr_el1
	and	x2, x1, #7
	add	x2, x2, #4			// log2(line size)
	ubfx	x4, x1, #3, #10			// max way number
	clz	w5, w4				// way field shift
	lsl	w14, w4, w5			// aligned max way
	lsl	w16, w8, w5			// way decrement
	orr	w14, w10, w14			// way | level
	ubfx	w6, w1, #13, #15		// max set number
	lsl	w17, w8, w2			// set decrement
	dsb	sy
2:	lsl	w7, w6, w2			// aligned max set
5:	orr	w11, w14, w7
	dc	cisw, x11
	subs	w7, w7, w17
	b.hs	5b
	subs	x14, x14, x16
	b.hs	2b
3:	add	x10, x10, #2
	cmp	x3, x10
	b.hi	1b
	msr	csselr_el1, xzr
	dsb	sy
	isb
4:	ret
SYM_FUNC_END(apple_dcache_cisw_levels)

/*
 * apple_cpu_park_mmuoff(pen_pa, levels) - MMU off, running at its PA.
 *
 * Milestone 1 ("park"): nothing is powered down. Empty the local caches so
 * that the CPU holds no stale line when it turns them on again, then enter
 * the kernel's own secondary_holding_pen, exactly where m1n1 sent it at
 * boot. cpu_boot releases it with secondary_holding_pen_release + SEV.
 */
SYM_FUNC_START(apple_cpu_park_mmuoff)
	mov	x19, x0
	mov	x0, x1
	bl	apple_dcache_cisw_levels
	ic	iallu
	tlbi	vmalle1
	dsb	sy
	isb
	br	x19
SYM_FUNC_END(apple_cpu_park_mmuoff)

/*
 * apple_cpu_off_mmuoff(stop_reg_pa, levels, stop_bit) - MMU off, at its PA.
 *
 * Milestone 2 ("powerdown"): the per-core stop that m1n1 smp_stop_cpu() does
 * (m1n1 src/smp.c:228-233 and cpu_sleep(false), src/utils.c:249-285):
 * request the stop in the PMGR CPU-start block, then CYC_OVRD
 * FIQ_MODE(2) | IRQ_MODE(2) | WFI_MODE(3) | DISABLE_WFI_RET and WFI.
 * ACC_OVRD is deliberately left alone: its deep-sleep bit is per cluster.
 *
 * The core comes back through PMGR CPU start, at its RVBAR (m1n1).
 */
SYM_FUNC_START(apple_cpu_off_mmuoff)
	mov	x19, x0
	mov	x20, x2
	mov	x0, x1
	bl	apple_dcache_cisw_levels
	ic	iallu
	dsb	sy
	isb

	str	w20, [x19]			// request this core's stop
	dsb	sy
	isb

	mrs	x0, s3_5_c15_c5_0
	bic	x0, x0, #APPLE_CYC_OVRD_MODES_MASK
	mov_q	x1, APPLE_CYC_OVRD_CORE_OFF
	orr	x0, x0, x1
	msr	s3_5_c15_c5_0, x0
	isb
1:	isb
	wfi
	b	1b
SYM_FUNC_END(apple_cpu_off_mmuoff)

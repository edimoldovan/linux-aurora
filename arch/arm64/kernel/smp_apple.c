// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPU offline/online for Apple silicon CPUs started through m1n1's
 * spin-table.
 *
 * m1n1 starts every secondary CPU at boot and leaves it in its spin-table
 * loop, which jumps to whatever address the kernel writes into the CPU's
 * release slot (m1n1 src/smp.c:118-143, :560-572). The kernel's spin-table
 * ops write secondary_holding_pen there and have no cpu_die, so a CPU can
 * never be taken offline, and the PM core aborts every suspend mode other
 * than s2idle.
 *
 * These ops wrap spin-table and add the hotplug half. They are selected at
 * run time from the kernel command line, because the device tree comes from
 * m1n1 and keeps enable-method = "spin-table":
 *
 *   apple_cpu_hotplug=park       offline: L1 cleaned, MMU and caches off,
 *                                back into the kernel's secondary_holding_pen
 *                                (no power change). online: pen release + SEV.
 *   apple_cpu_hotplug=powerdown  t8103 only. offline: per-core stop through
 *                                the PMGR CPU-start block and CYC_OVRD, as
 *                                m1n1 smp_stop_cpu() does. online: PMGR CPU
 *                                start; the core resets at its RVBAR into
 *                                m1n1, which re-applies the chicken bits and
 *                                jumps to the release slot, still holding
 *                                secondary_holding_pen.
 *   apple_cpu_hotplug=powerdown-legacy
 *                                the same, also with an m1n1 stage 2 older
 *                                than v1.9.9 (see apple_m1n1_reentry below).
 *
 * Rules that hold in every mode:
 * - The boot CPU never goes offline: its RVBAR is locked by iBoot to m1n1,
 *   which would boot again from scratch.
 * - The last online core of a cluster is never powered down, only parked.
 *   M1 cores run with HID4/EHID4 DISABLE_DC_SW_L2_OPS, so the kernel cannot
 *   clean the cluster L2 by set/way, and m1n1 warns that a powered-off
 *   cluster loses register access (src/smp.c:235-236). A parked core keeps
 *   the cluster up.
 * - A core that did not report itself off, or did not come back, is "stuck"
 *   and is never sent a PMGR start or a pen release again.
 *
 * A parked core costs power at run time: it waits clock-gated in WFE,
 * whereas an idle online core is power-gated by cpuidle-apple. Park mode is
 * a stepping stone towards suspend-to-RAM, not a power saving.
 *
 * Without the parameter nothing changes.
 */

#define pr_fmt(fmt) "apple-cpu-hotplug: " fmt

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cpumask.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/smp.h>
#include <linux/string.h>
#include <linux/types.h>

#include <asm/cacheflush.h>
#include <asm/cpu_ops.h>
#include <asm/cputype.h>
#include <asm/mmu_context.h>
#include <asm/smp.h>
#include <asm/smp_plat.h>
#include <asm/sysreg.h>

/*
 * t8103 (M1) addresses.
 * - PMGR CPU-start block: pmgr reg[0] 0x23b700000 + CPU_START_OFF_T8103
 *   0x54000 (m1n1 src/smp.c:17, :313, :424). +0x0 requests a core stop,
 *   +0x4 is the "system level startup/status" mask and +0x8 + 4 * cluster
 *   starts cores of that cluster (m1n1 src/smp.c:189-196, :228-229).
 *   Core bit = 1 << (4 * cluster + core); the ADT /cpus/cpuN
 *   function-enable_core masks agree (0x01..0x80).
 * - cpu-impl-reg: RVBAR at +0x0 (bit 0 lock, bits 47:12 address), power
 *   status at +0x100, low byte 0 when the core is off (m1n1 src/smp.c:28-29,
 *   :244-250). ADT: cpu0..3 0x210050000 + core * 0x100000,
 *   cpu4..7 0x211050000 + core * 0x100000.
 * - uart0 power state: PMGR PS register 0x23b700270 (t8103-pmgr.dtsi
 *   ps_uart0), actual state in bits 7:4, 0xf = on.
 */
#define T8103_CPU_START			0x23b754000ULL
#define CPU_START_STOP			0x0
#define CPU_START_ALIVE			0x4
#define CPU_START_CLUSTER(cl)		(0x8 + 4 * (cl))
#define T8103_CPU_IMPL(cl, co)		(0x210050000ULL + (cl) * 0x1000000ULL + \
					 (co) * 0x100000ULL)
#define CPU_IMPL_RVBAR			0x0
#define CPU_IMPL_RVBAR_ADDR		GENMASK_ULL(47, 12)
#define CPU_IMPL_PWRSTAT		0x100
#define CPU_IMPL_PWRSTAT_ON		GENMASK_ULL(7, 0)
#define T8103_PS_UART0			0x23b700270ULL
#define PS_ACTUAL			GENMASK(7, 4)
#define PS_ACTIVE			0xf

/*
 * Apple per-core registers that m1n1 sets (m1n1 src/cpu_regs.h). The
 * kernel itself only ever writes CYC_OVRD (cpuidle-apple).
 * - HID4 (Firestorm) / EHID4 (Icestorm): bit 44 DISABLE_DC_SW_L2_OPS,
 *   bit 11 DISABLE_DC_MVA, set by init_cpu() on every (re)entry.
 * - HID11 (Firestorm): bit 30 DISABLE_DMP, the GoFetch mitigation. m1n1
 *   applies it once, from mitigations_perform() after smp_start_secondaries()
 *   (src/payload.c:318-319, src/mitigations.c:11-15). An RVBAR re-entry runs
 *   only init_cpu(), so a restarted core loses it.
 * - ACC_OVRD: bit 34 DEEP_SLEEP; with it set a stop powers the cluster
 *   down (m1n1 cpu_sleep(true)). Never set by the kernel.
 * - ACTLR_EL1 bit 1: TSO, set once by m1n1 (do_enable_tso) and context
 *   switched by the kernel (CONFIG_ARM64_ACTLR_STATE); recorded only.
 */
#define SYS_APL_HID4			sys_reg(3, 0, 15, 4, 0)
#define SYS_APL_EHID4			sys_reg(3, 0, 15, 4, 1)
#define SYS_APL_HID11			sys_reg(3, 0, 15, 11, 0)
#define HID11_DISABLE_DMP		BIT_ULL(30)
#define SYS_APL_ACC_OVRD		sys_reg(3, 5, 15, 6, 0)
#define ACC_OVRD_DEEP_SLEEP		BIT_ULL(34)

/*
 * m1n1 re-entry. The first m1n1 release that finds a restarted core's own
 * stack and spin-table slot by MPIDR is v1.9.9 (commit 1b469fcb618f "smp:
 * Identify stack by MPIDR", in smp_secondary_entry()). Older stage 2s use
 * the stale target_cpu (the last core started at boot) for the slot and the
 * shared dummy_stack. That still reaches the kernel's pen if every release
 * slot holds it, which the kernel then makes sure of before each restart;
 * restarts are serialised by CPU hotplug, and a core stuck inside m1n1 with
 * the shared stack blocks all later restarts.
 */
enum apple_m1n1_reentry {
	APPLE_M1N1_UNKNOWN,	/* no or unparseable version: park only */
	APPLE_M1N1_LEGACY,	/* < v1.9.9: shared stack, stale slot */
	APPLE_M1N1_MPIDR,	/* >= v1.9.9: per-core stack and slot */
};

static const char *const apple_m1n1_reentry_name[] = {
	[APPLE_M1N1_UNKNOWN]	= "unknown",
	[APPLE_M1N1_LEGACY]	= "legacy",
	[APPLE_M1N1_MPIDR]	= "mpidr",
};

enum apple_hp_mode {
	APPLE_HP_NONE,
	APPLE_HP_PARK,
	APPLE_HP_POWERDOWN,
};

enum apple_cpu_state {
	APPLE_CPU_PARKED,	/* in secondary_holding_pen, MMU off (also at boot) */
	APPLE_CPU_RUNNING,
	APPLE_CPU_OFF,		/* PMGR stop requested, core powering down */
	APPLE_CPU_STARTING,	/* PMGR start sent, not yet in the kernel */
};

static const char *const apple_cpu_state_name[] = {
	[APPLE_CPU_PARKED]	= "parked",
	[APPLE_CPU_RUNNING]	= "running",
	[APPLE_CPU_OFF]		= "off",
	[APPLE_CPU_STARTING]	= "starting",
};

struct apple_cpu_regs {
	bool valid;
	bool pcore;
	u64 hid4;		/* HID4 on Firestorm, EHID4 on Icestorm */
	u64 hid11;		/* Firestorm only */
	u64 acc_ovrd;
	u64 actlr;
};

struct apple_cpu {
	void __iomem *impl;		/* cpu-impl-reg, powerdown mode only */
	u64 __iomem *release;		/* m1n1 spin-table release slot */
	phys_addr_t release_pa;
	u64 rvbar;
	bool can_powerdown;
	bool stuck;			/* never start or release it again */
	bool boot_pending;		/* cpu_boot() done, cpu_postboot() not yet */
	int state;			/* enum apple_cpu_state */
	int down_mode;			/* APPLE_CPU_PARKED or APPLE_CPU_OFF */
	const char *park_reason;	/* why the last offline parked */
	const char *stuck_reason;
	struct apple_cpu_regs snap;	/* first boot: what m1n1 left */
	struct apple_cpu_regs last;	/* most recent cpu_postboot() */
	unsigned long parks;
	unsigned long powerdowns;
	unsigned long restarts;
	unsigned long status_timeouts;
	unsigned long flush_timeouts;
	unsigned long dmp_restores;
	unsigned long reg_mismatches;
};

/*
 * Written with the MMU off by the dying CPU once its L1 is clean, read by
 * cpu_kill() after invalidating the line. One cache line per CPU.
 */
struct apple_cpu_flushed {
	unsigned long done;
} __aligned(SZ_128);

static struct apple_cpu_flushed apple_cpu_flushed[NR_CPUS]
	__section(".mmuoff.data.write");

extern const struct cpu_operations smp_spin_table_ops;
extern void secondary_holding_pen(void);
extern volatile unsigned long secondary_holding_pen_release;

/* smp_apple_asm.S */
void apple_cpu_mmu_off_jump(unsigned long entry, unsigned long arg0,
			    unsigned long arg1, unsigned long arg2);
void apple_cpu_park_mmuoff(unsigned long pen_pa, unsigned long flushed_pa);
void apple_cpu_off_mmuoff(unsigned long stop_pa, unsigned long flushed_pa,
			  unsigned long stop_bit);

static enum apple_hp_mode apple_hp_mode __ro_after_init;
static bool apple_hp_allow_legacy __ro_after_init;
static bool apple_hp_active __ro_after_init;
static enum apple_m1n1_reentry apple_m1n1_reentry __ro_after_init;
static const char *apple_m1n1_version __ro_after_init;
static struct apple_cpu apple_cpus[NR_CPUS];
static void __iomem *apple_cpu_start;
static void __iomem *apple_ps_uart0;
static bool apple_pmgr_unmapped;
/* Legacy m1n1 only: a core is stuck on m1n1's shared stack. */
static bool apple_restarts_blocked;

static int __init apple_cpu_hotplug_setup(char *arg)
{
	if (!arg)
		return -EINVAL;

	if (!strcmp(arg, "park") || !strcmp(arg, "1")) {
		apple_hp_mode = APPLE_HP_PARK;
	} else if (!strcmp(arg, "powerdown")) {
		apple_hp_mode = APPLE_HP_POWERDOWN;
	} else if (!strcmp(arg, "powerdown-legacy")) {
		apple_hp_mode = APPLE_HP_POWERDOWN;
		apple_hp_allow_legacy = true;
	} else if (!strcmp(arg, "0") || !strcmp(arg, "off")) {
		apple_hp_mode = APPLE_HP_NONE;
	} else {
		return -EINVAL;
	}

	return 0;
}
early_param("apple_cpu_hotplug", apple_cpu_hotplug_setup);

static unsigned int apple_cpu_cluster(unsigned int cpu)
{
	return MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 1);
}

static unsigned int apple_cpu_core(unsigned int cpu)
{
	return MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 0);
}

static u32 apple_cpu_bit(unsigned int cpu)
{
	return BIT(4 * apple_cpu_cluster(cpu) + apple_cpu_core(cpu));
}

/* Same as spin-table's write_pen_release(): visible to MMU-off readers. */
static void apple_write_pen_release(u64 val)
{
	unsigned long start = (unsigned long)&secondary_holding_pen_release;

	secondary_holding_pen_release = val;
	dcache_clean_inval_poc(start, start + sizeof(secondary_holding_pen_release));
}

static unsigned long apple_flushed_va(unsigned int cpu)
{
	return (unsigned long)&apple_cpu_flushed[cpu].done;
}

static void apple_flushed_clear(unsigned int cpu)
{
	unsigned long va = apple_flushed_va(cpu);

	WRITE_ONCE(apple_cpu_flushed[cpu].done, 0);
	dcache_clean_inval_poc(va, va + sizeof(apple_cpu_flushed[cpu].done));
}

static bool apple_flushed_read(unsigned int cpu)
{
	unsigned long va = apple_flushed_va(cpu);

	/* Clean+inval: never discards a line, drops our stale copy. */
	dcache_clean_inval_poc(va, va + sizeof(apple_cpu_flushed[cpu].done));
	return READ_ONCE(apple_cpu_flushed[cpu].done) != 0;
}

static bool apple_uart0_powered(void)
{
	/* No mapping: not t8103 powerdown, nothing to check. */
	if (!apple_ps_uart0)
		return true;

	return FIELD_GET(PS_ACTUAL, readl(apple_ps_uart0)) == PS_ACTIVE;
}

/* Is any page of [start, end] memory the kernel could hand out? */
static bool apple_range_has_kernel_ram(phys_addr_t start, phys_addr_t end)
{
	unsigned long pfn;

	for (pfn = PHYS_PFN(start); pfn <= PHYS_PFN(end); pfn++)
		if (pfn_valid(pfn) && !PageReserved(pfn_to_page(pfn)))
			return true;

	return false;
}

static bool apple_cpu_is_m1(void)
{
	u32 midr = read_cpuid_id();

	return MIDR_IMPLEMENTOR(midr) == ARM_CPU_IMP_APPLE &&
	       (MIDR_PARTNUM(midr) == APPLE_CPU_PART_M1_ICESTORM ||
		MIDR_PARTNUM(midr) == APPLE_CPU_PART_M1_FIRESTORM);
}

/* Runs on the CPU whose registers are read. t8103 cores only. */
static void apple_cpu_read_regs(struct apple_cpu_regs *r)
{
	r->pcore = MIDR_PARTNUM(read_cpuid_id()) == APPLE_CPU_PART_M1_FIRESTORM;
	if (r->pcore) {
		r->hid4 = read_sysreg_s(SYS_APL_HID4);
		r->hid11 = read_sysreg_s(SYS_APL_HID11);
	} else {
		r->hid4 = read_sysreg_s(SYS_APL_EHID4);
		r->hid11 = 0;
	}
	r->acc_ovrd = read_sysreg_s(SYS_APL_ACC_OVRD);
	r->actlr = read_sysreg(actlr_el1);
	r->valid = true;
}

/*
 * cpu_postboot(), powerdown mode on t8103: record what m1n1 set up the
 * first time, and put back what a restart through m1n1 loses.
 */
static void apple_cpu_check_regs(unsigned int cpu)
{
	struct apple_cpu *ac = &apple_cpus[cpu];

	if (apple_hp_mode != APPLE_HP_POWERDOWN || !apple_cpu_is_m1())
		return;

	if (!ac->snap.valid) {
		apple_cpu_read_regs(&ac->snap);
		ac->last = ac->snap;
		return;
	}

	apple_cpu_read_regs(&ac->last);

	if (ac->snap.pcore && (ac->snap.hid11 & HID11_DISABLE_DMP) &&
	    !(ac->last.hid11 & HID11_DISABLE_DMP)) {
		write_sysreg_s(ac->last.hid11 | HID11_DISABLE_DMP, SYS_APL_HID11);
		isb();
		ac->last.hid11 = read_sysreg_s(SYS_APL_HID11);
		ac->dmp_restores++;
		pr_info_once("CPU%u: HID11.DISABLE_DMP (GoFetch mitigation) lost on restart, restored\n",
			     cpu);
	}

	if (ac->last.hid4 != ac->snap.hid4 || ac->last.hid11 != ac->snap.hid11 ||
	    ac->last.acc_ovrd != ac->snap.acc_ovrd) {
		ac->reg_mismatches++;
		pr_warn_once("CPU%u: core registers differ after restart: HID4 %#llx -> %#llx, HID11 %#llx -> %#llx, ACC_OVRD %#llx -> %#llx\n",
			     cpu, ac->snap.hid4, ac->last.hid4, ac->snap.hid11,
			     ac->last.hid11, ac->snap.acc_ovrd, ac->last.acc_ovrd);
	}
}

static int apple_cpu_init(unsigned int cpu)
{
	struct device_node *dn;
	u64 release_pa = 0;
	int ret;

	ret = smp_spin_table_ops.cpu_init(cpu);
	if (ret)
		return ret;

	dn = of_get_cpu_node(cpu, NULL);
	if (dn) {
		of_property_read_u64(dn, "cpu-release-addr", &release_pa);
		of_node_put(dn);
	}
	apple_cpus[cpu].release_pa = release_pa;
	apple_cpus[cpu].state = APPLE_CPU_PARKED;

	return 0;
}

static void apple_cpu_unmap(struct apple_cpu *ac)
{
	if (ac->impl)
		iounmap(ac->impl);
	if (ac->release)
		iounmap((void __iomem *)ac->release);
	ac->impl = NULL;
	ac->release = NULL;
}

/*
 * Map what powerdown needs and check that the way back is safe. Any failure
 * leaves this CPU in park mode.
 */
static void apple_cpu_prepare_powerdown(unsigned int cpu)
{
	struct apple_cpu *ac = &apple_cpus[cpu];
	phys_addr_t rvbar_pa, lo, hi;

	if (apple_pmgr_unmapped)
		return;
	if (!apple_cpu_start) {
		void __iomem *start = ioremap_np(T8103_CPU_START, 0x10);
		void __iomem *uart0 = ioremap_np(T8103_PS_UART0, 4);

		if (!start || !uart0) {
			pr_err("cannot map the PMGR CPU-start block, powerdown disabled\n");
			if (start)
				iounmap(start);
			if (uart0)
				iounmap(uart0);
			apple_pmgr_unmapped = true;
			return;
		}
		apple_cpu_start = start;
		apple_ps_uart0 = uart0;
	}

	if (!ac->release_pa)
		return;

	ac->impl = ioremap_np(T8103_CPU_IMPL(apple_cpu_cluster(cpu),
					     apple_cpu_core(cpu)), 0x1000);
	ac->release = (u64 __iomem *)ioremap_cache(ac->release_pa,
						   sizeof(*ac->release));
	if (!ac->impl || !ac->release) {
		pr_err("CPU%u: cannot map cpu-impl-reg or release slot\n", cpu);
		apple_cpu_unmap(ac);
		return;
	}

	/*
	 * m1n1 writes each secondary's RVBAR itself before starting it, without
	 * the lock bit (m1n1 src/smp.c:184-187), so it points at m1n1's
	 * _vectors_start. A restarted core enters m1n1 there and runs start.S,
	 * _cpu_reset_c(), init_cpu(), printf and smp_secondary_entry() before it
	 * reads the release slot. All of that lives in m1n1's image, which m1n1
	 * reserves (/memreserve/ [_base, _end), m1n1 src/kboot.c). Require
	 * every page from the RVBAR target to the release slot to be memory the
	 * kernel cannot allocate; otherwise the reservation was lost on the way
	 * through U-Boot and the EFI stub, and powerdown is unsafe.
	 */
	ac->rvbar = readq(ac->impl + CPU_IMPL_RVBAR);
	rvbar_pa = ac->rvbar & CPU_IMPL_RVBAR_ADDR;
	if (!rvbar_pa) {
		pr_warn("CPU%u: RVBAR is 0, powerdown disabled\n", cpu);
		apple_cpu_unmap(ac);
		return;
	}
	lo = min(rvbar_pa, ac->release_pa);
	hi = max(rvbar_pa, ac->release_pa);
	if (hi - lo > SZ_64M) {
		pr_warn("CPU%u: RVBAR %pa and release slot %pa are %#llx apart, not one m1n1 image, powerdown disabled\n",
			cpu, &rvbar_pa, &ac->release_pa,
			(unsigned long long)(hi - lo));
		apple_cpu_unmap(ac);
		return;
	}
	if (apple_range_has_kernel_ram(lo, hi)) {
		pr_warn("CPU%u: m1n1 at %pa..%pa is not reserved, powerdown disabled\n",
			cpu, &lo, &hi);
		apple_cpu_unmap(ac);
		return;
	}

	ac->can_powerdown = true;
	pr_info("CPU%u: powerdown ready (RVBAR %#llx, release %pa, core bit %#x)\n",
		cpu, ac->rvbar, &ac->release_pa, apple_cpu_bit(cpu));
}

static int apple_cpu_prepare(unsigned int cpu)
{
	int ret;

	ret = smp_spin_table_ops.cpu_prepare(cpu);
	if (ret)
		return ret;

	if (apple_hp_mode == APPLE_HP_POWERDOWN)
		apple_cpu_prepare_powerdown(cpu);

	return 0;
}

/* Legacy m1n1 reads a stale slot: every secondary's slot must be mapped. */
static bool apple_all_release_mapped(void)
{
	unsigned int cpu;

	for_each_possible_cpu(cpu)
		if (cpu != 0 && !apple_cpus[cpu].release)
			return false;

	return true;
}

static void apple_write_release(struct apple_cpu *ac, unsigned long pen_pa)
{
	writeq_relaxed(pen_pa, ac->release);
	dcache_clean_inval_poc((__force unsigned long)ac->release,
			       (__force unsigned long)ac->release + sizeof(*ac->release));
}

static int apple_cpu_boot(unsigned int cpu)
{
	struct apple_cpu *ac = &apple_cpus[cpu];
	unsigned long pen_pa = __pa_symbol(secondary_holding_pen);
	int state = READ_ONCE(ac->state);
	unsigned int i;

	if (cpu == 0)
		return -EPERM;

	/*
	 * The last cpu_boot() never reached cpu_postboot(): __cpu_up() gave up
	 * after 5 s. The core may still be on its way; do not send it anything.
	 */
	if (ac->boot_pending) {
		apple_write_pen_release(INVALID_HWID);
		ac->boot_pending = false;
		ac->stuck = true;
		ac->stuck_reason = "did not come online";
		if (state == APPLE_CPU_STARTING &&
		    apple_m1n1_reentry != APPLE_M1N1_MPIDR) {
			/* It may sit on m1n1's shared dummy_stack. */
			apple_restarts_blocked = true;
		}
	}

	if (ac->stuck) {
		/* A late stop is fine: it is off now, so it can be started. */
		if (state == APPLE_CPU_OFF && ac->impl &&
		    !(readq(ac->impl + CPU_IMPL_PWRSTAT) & CPU_IMPL_PWRSTAT_ON)) {
			pr_info("CPU%u: now reports off, no longer stuck\n", cpu);
			ac->stuck = false;
			ac->stuck_reason = NULL;
		} else {
			pr_err("CPU%u: stuck (%s), not starting it\n", cpu,
			       ac->stuck_reason ?: "?");
			return -EBUSY;
		}
	}

	/*
	 * A CPU that dies during early boot reaches cpu_die() without passing
	 * RUNNING, so cpu_kill() cannot wait for the state change: make sure
	 * it never sees the previous cycle's report.
	 */
	apple_flushed_clear(cpu);

	if (state != APPLE_CPU_OFF) {
		/* Parked in the pen (or never started): spin-table release. */
		ac->boot_pending = true;
		apple_write_pen_release(cpu_logical_map(cpu));
		sev();
		return 0;
	}

	if (apple_restarts_blocked) {
		pr_err("CPU%u: a core is stuck in m1n1, not restarting\n", cpu);
		return -EBUSY;
	}

	/*
	 * m1n1 prints to uart0 on every RVBAR entry (m1n1 src/startup.c:221-229,
	 * src/smp.c:107). Do not send the core there if uart0 is powered off:
	 * leave it off rather than hang it in m1n1.
	 */
	if (!apple_uart0_powered()) {
		pr_err("CPU%u: uart0 is powered off, not restarting through m1n1\n",
		       cpu);
		return -EIO;
	}

	/*
	 * The slots still hold the pen; write them again in case m1n1 cleared
	 * one. A legacy m1n1 reads the slot of the last core it started at
	 * boot, not this core's, so write every slot.
	 */
	if (apple_m1n1_reentry == APPLE_M1N1_MPIDR) {
		apple_write_release(ac, pen_pa);
	} else {
		for_each_possible_cpu(i)
			if (i != 0 && apple_cpus[i].release)
				apple_write_release(&apple_cpus[i], pen_pa);
	}

	apple_write_pen_release(cpu_logical_map(cpu));

	smp_store_release(&ac->state, APPLE_CPU_STARTING);
	ac->boot_pending = true;

	/* m1n1 smp_start_cpu(), src/smp.c:189-196 */
	writel(apple_cpu_bit(cpu), apple_cpu_start + CPU_START_ALIVE);
	writel(BIT(apple_cpu_core(cpu)),
	       apple_cpu_start + CPU_START_CLUSTER(apple_cpu_cluster(cpu)));
	ac->restarts++;

	return 0;
}

static void apple_cpu_postboot(void)
{
	unsigned int cpu = smp_processor_id();
	struct apple_cpu *ac = &apple_cpus[cpu];

	/*
	 * Spin-table never resets the release word, so the last CPU booted
	 * would leave the pen at once the next time it is parked.
	 */
	apple_write_pen_release(INVALID_HWID);

	apple_cpu_check_regs(cpu);

	ac->boot_pending = false;
	smp_store_release(&ac->state, APPLE_CPU_RUNNING);
}

static bool apple_cpu_can_disable(unsigned int cpu)
{
	return cpu != 0;
}

static bool apple_cpu_last_in_cluster(unsigned int cpu)
{
	unsigned int i;

	for_each_online_cpu(i)
		if (i != cpu && apple_cpu_cluster(i) == apple_cpu_cluster(cpu))
			return false;

	return true;
}

/*
 * Runs on the dying CPU, before it is marked offline, with the online mask
 * stable (stop_machine). Chooses park or power-down.
 */
static int apple_cpu_disable(unsigned int cpu)
{
	struct apple_cpu *ac = &apple_cpus[cpu];
	u64 acc_ovrd;

	if (cpu == 0)
		return -EPERM;

	ac->down_mode = APPLE_CPU_PARKED;
	if (apple_hp_mode != APPLE_HP_POWERDOWN) {
		ac->park_reason = "mode";
		return 0;
	}
	if (!ac->can_powerdown) {
		ac->park_reason = "not ready";
		return 0;
	}
	if (apple_m1n1_reentry == APPLE_M1N1_LEGACY &&
	    !apple_all_release_mapped()) {
		ac->park_reason = "legacy m1n1, release slots unmapped";
		return 0;
	}
	if (apple_restarts_blocked) {
		ac->park_reason = "restarts blocked";
		return 0;
	}
	if (apple_cpu_last_in_cluster(cpu)) {
		ac->park_reason = "last online core of cluster";
		return 0;
	}
	if (!apple_uart0_powered()) {
		ac->park_reason = "uart0 off";
		return 0;
	}
	acc_ovrd = read_sysreg_s(SYS_APL_ACC_OVRD);
	if (acc_ovrd & ACC_OVRD_DEEP_SLEEP) {
		ac->park_reason = "ACC_OVRD deep sleep set";
		pr_warn_once("CPU%u: ACC_OVRD %#llx has DEEP_SLEEP set, parking instead of powering down\n",
			     cpu, acc_ovrd);
		return 0;
	}

	ac->park_reason = NULL;
	ac->down_mode = APPLE_CPU_OFF;
	return 0;
}

/*
 * Runs on the dying CPU with DAIF masked, after it reported itself dead.
 * No printk, no RCU from here on. Does not return.
 */
static void apple_cpu_die(unsigned int cpu)
{
	typeof(apple_cpu_mmu_off_jump) *jump;
	struct apple_cpu *ac = &apple_cpus[cpu];
	bool off = ac->down_mode == APPLE_CPU_OFF;
	unsigned long flushed_pa = __pa_symbol(&apple_cpu_flushed[cpu].done);

	/*
	 * Only reached for CPU0 through the crash-stop IPI: stay in the
	 * kernel, as plain spin-table does. m1n1 owns CPU0's reset vector.
	 */
	if (cpu == 0)
		cpu_park_loop();

	/* Never released by a stale release word. */
	apple_write_pen_release(INVALID_HWID);
	/* Cleared and cleaned before cpu_kill() can look at it. */
	apple_flushed_clear(cpu);

	if (off)
		ac->powerdowns++;
	else
		ac->parks++;
	/* cpu_disable() chooses again; a crash-stop cpu_die() without it parks. */
	ac->down_mode = APPLE_CPU_PARKED;
	smp_store_release(&ac->state, off ? APPLE_CPU_OFF : APPLE_CPU_PARKED);
	dsb(sy);

	jump = (void *)__pa_symbol(apple_cpu_mmu_off_jump);
	cpu_install_idmap();
	if (off)
		jump(__pa_symbol(apple_cpu_off_mmuoff),
		     T8103_CPU_START + CPU_START_STOP, flushed_pa,
		     apple_cpu_bit(cpu));
	else
		jump(__pa_symbol(apple_cpu_park_mmuoff),
		     __pa_symbol(secondary_holding_pen), flushed_pa, 0);

	unreachable();
}

/* Runs on another CPU once the dying CPU reported itself dead. */
static int apple_cpu_kill(unsigned int cpu)
{
	struct apple_cpu *ac = &apple_cpus[cpu];
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	int state;
	u64 val;
	int ret;

	while ((state = smp_load_acquire(&ac->state)) == APPLE_CPU_RUNNING) {
		if (time_after(jiffies, timeout)) {
			ac->stuck = true;
			ac->stuck_reason = "never reached cpu_die";
			return -ETIMEDOUT;
		}
		usleep_range(100, 200);
	}

	/*
	 * Wait until the dying CPU has cleaned its L1 with the MMU off. Only
	 * then is nothing of the kernel's left in its caches (suspend relies
	 * on that). A timeout here is reported, not fatal: the power status
	 * below is what decides whether a powered-down core may be started.
	 */
	timeout = jiffies + msecs_to_jiffies(100);
	while (!apple_flushed_read(cpu)) {
		if (time_after(jiffies, timeout)) {
			ac->flush_timeouts++;
			pr_warn("CPU%u: no L1-clean report 100 ms after cpu_die\n",
				cpu);
			if (state != APPLE_CPU_OFF)
				return -ETIMEDOUT;
			break;
		}
		usleep_range(50, 100);
	}

	if (state != APPLE_CPU_OFF)
		return 0;

	/*
	 * m1n1 smp_stop_cpu() polls the same status after a non-deep stop
	 * (src/smp.c:244-250). This core is never the last of its cluster, so
	 * the cluster, and with it this register, stays powered.
	 */
	ret = readq_poll_timeout(ac->impl + CPU_IMPL_PWRSTAT, val,
				 !(val & CPU_IMPL_PWRSTAT_ON),
				 100, 200 * USEC_PER_MSEC);
	if (ret) {
		ac->status_timeouts++;
		ac->stuck = true;
		ac->stuck_reason = "did not report off";
		pr_warn("CPU%u: power status %#llx 200 ms after the stop request, marked stuck\n",
			cpu, val);
	}

	return ret;
}

static const struct cpu_operations apple_spin_table_ops = {
	.name		= "apple,pmgr-spin-table",
	.cpu_init	= apple_cpu_init,
	.cpu_prepare	= apple_cpu_prepare,
	.cpu_boot	= apple_cpu_boot,
	.cpu_postboot	= apple_cpu_postboot,
	.cpu_can_disable = apple_cpu_can_disable,
	.cpu_disable	= apple_cpu_disable,
	.cpu_die	= apple_cpu_die,
	.cpu_kill	= apple_cpu_kill,
};

/*
 * m1n1 stage 2 writes its version, "git describe" style (v1.6.1,
 * v1.4.21-12-gabcdef, v2.0.0-dirty), to /chosen.
 */
static enum apple_m1n1_reentry __init apple_m1n1_probe(void)
{
	struct device_node *chosen;
	const char *ver = NULL;
	unsigned int maj, min, patch;

	chosen = of_find_node_by_path("/chosen");
	if (chosen) {
		of_property_read_string(chosen, "asahi,m1n1-stage2-version", &ver);
		of_node_put(chosen);
	}
	apple_m1n1_version = ver;
	if (!ver)
		return APPLE_M1N1_UNKNOWN;

	if (*ver == 'v')
		ver++;
	if (sscanf(ver, "%u.%u.%u", &maj, &min, &patch) != 3)
		return APPLE_M1N1_UNKNOWN;

	if (maj > 1 || (maj == 1 && (min > 9 || (min == 9 && patch >= 9))))
		return APPLE_M1N1_MPIDR;

	return APPLE_M1N1_LEGACY;
}

/*
 * Called by init_cpu_ops() for every CPU. Replaces spin-table on Apple
 * platforms when apple_cpu_hotplug= asks for it; m1n1 supplies the DT, so
 * the enable-method itself cannot be changed without new DTBs in m1n1.
 */
const struct cpu_operations * __init
apple_cpu_hotplug_ops(const struct cpu_operations *ops)
{
	if (apple_hp_mode == APPLE_HP_NONE || ops != &smp_spin_table_ops ||
	    !of_machine_is_compatible("apple,arm-platform"))
		return ops;

	if (apple_hp_active)
		return &apple_spin_table_ops;

	apple_m1n1_reentry = apple_m1n1_probe();
	pr_info("m1n1 stage 2 %s, re-entry %s\n",
		apple_m1n1_version ?: "(not reported)",
		apple_m1n1_reentry_name[apple_m1n1_reentry]);

	if (apple_hp_mode == APPLE_HP_POWERDOWN &&
	    !of_machine_is_compatible("apple,t8103")) {
		pr_warn("powerdown is t8103-only, using park\n");
		apple_hp_mode = APPLE_HP_PARK;
	}
	if (apple_hp_mode == APPLE_HP_POWERDOWN) {
		if (apple_m1n1_reentry == APPLE_M1N1_UNKNOWN) {
			pr_warn("m1n1 version unknown, using park\n");
			apple_hp_mode = APPLE_HP_PARK;
		} else if (apple_m1n1_reentry == APPLE_M1N1_LEGACY &&
			   !apple_hp_allow_legacy) {
			pr_warn("m1n1 older than v1.9.9 has no per-core re-entry; using park (apple_cpu_hotplug=powerdown-legacy overrides)\n");
			apple_hp_mode = APPLE_HP_PARK;
		} else if (apple_m1n1_reentry == APPLE_M1N1_LEGACY) {
			pr_warn("legacy m1n1 re-entry: every release slot is rewritten before a restart\n");
		}
	}

	pr_info("CPU offline enabled, mode %s\n",
		apple_hp_mode == APPLE_HP_POWERDOWN ? "powerdown" : "park");
	apple_hp_active = true;

	return &apple_spin_table_ops;
}

/*
 * Parked CPUs spin in this kernel's image, and powered-down ones come back
 * into it through m1n1: kexec and hibernation must keep refusing, as they do
 * for plain spin-table.
 */
bool apple_cpu_hotplug_active(void)
{
	return apple_hp_active;
}

static void apple_show_regs(struct seq_file *m, const char *tag,
			    const struct apple_cpu_regs *r)
{
	if (!r->valid)
		return;
	seq_printf(m, "  %s: %s hid4=%#llx hid11=%#llx acc_ovrd=%#llx actlr=%#llx\n",
		   tag, r->pcore ? "pcore" : "ecore", r->hid4, r->hid11,
		   r->acc_ovrd, r->actlr);
}

static int apple_cpu_hotplug_show(struct seq_file *m, void *v)
{
	unsigned int cpu;

	seq_printf(m, "mode: %s\n",
		   apple_hp_mode == APPLE_HP_POWERDOWN ? "powerdown" : "park");
	seq_printf(m, "m1n1: %s reentry=%s\n",
		   apple_m1n1_version ?: "(not reported)",
		   apple_m1n1_reentry_name[apple_m1n1_reentry]);
	seq_printf(m, "uart0_powered: %d\n", apple_uart0_powered());
	seq_printf(m, "restarts_blocked: %d\n", READ_ONCE(apple_restarts_blocked));

	for_each_possible_cpu(cpu) {
		struct apple_cpu *ac = &apple_cpus[cpu];
		unsigned int state = READ_ONCE(ac->state);

		if (cpu == 0)
			continue;

		seq_printf(m, "cpu%u mpidr=%#llx bit=%#x state=%s stuck=%d%s%s can_powerdown=%d rvbar=%#llx parks=%lu powerdowns=%lu restarts=%lu status_timeouts=%lu flush_timeouts=%lu dmp_restores=%lu reg_mismatches=%lu last_park_reason=%s\n",
			   cpu, cpu_logical_map(cpu), apple_cpu_bit(cpu),
			   state < ARRAY_SIZE(apple_cpu_state_name) ?
			   apple_cpu_state_name[state] : "?",
			   ac->stuck, ac->stuck ? ":" : "",
			   ac->stuck ? (ac->stuck_reason ?: "?") : "",
			   ac->can_powerdown, ac->rvbar, ac->parks,
			   ac->powerdowns, ac->restarts, ac->status_timeouts,
			   ac->flush_timeouts, ac->dmp_restores,
			   ac->reg_mismatches,
			   ac->park_reason ? ac->park_reason : "-");
		apple_show_regs(m, "boot", &ac->snap);
		apple_show_regs(m, "last", &ac->last);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(apple_cpu_hotplug);

static int __init apple_cpu_hotplug_debugfs(void)
{
	if (apple_hp_active)
		debugfs_create_file("apple_cpu_hotplug", 0444, NULL, NULL,
				    &apple_cpu_hotplug_fops);
	return 0;
}
late_initcall(apple_cpu_hotplug_debugfs);

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
 *   apple_cpu_hotplug=park       offline: MMU and caches off, back into the
 *                                kernel's secondary_holding_pen (no power
 *                                change). online: pen release + SEV.
 *   apple_cpu_hotplug=powerdown  t8103 only. offline: per-core stop through
 *                                the PMGR CPU-start block and CYC_OVRD, as
 *                                m1n1 smp_stop_cpu() does. online: PMGR CPU
 *                                start; the core resets at its RVBAR into
 *                                m1n1, which re-applies the chicken bits and
 *                                jumps to the release slot, still holding
 *                                secondary_holding_pen.
 *
 * Without the parameter nothing changes. The boot CPU never goes offline:
 * its RVBAR is locked by iBoot to m1n1, which would boot again from scratch.
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
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <linux/smp.h>
#include <linux/string.h>
#include <linux/types.h>

#include <asm/cacheflush.h>
#include <asm/cpu_ops.h>
#include <asm/cputype.h>
#include <asm/mmu_context.h>
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
#define T8103_PS_UART0			0x23b700270ULL
#define PS_ACTUAL			GENMASK(7, 4)
#define PS_ACTIVE			0xf

enum apple_hp_mode {
	APPLE_HP_NONE,
	APPLE_HP_PARK,
	APPLE_HP_POWERDOWN,
};

enum apple_cpu_state {
	APPLE_CPU_PARKED,	/* in secondary_holding_pen, MMU off (also at boot) */
	APPLE_CPU_RUNNING,
	APPLE_CPU_OFF,		/* PMGR stop requested, core powering down */
};

static const char *const apple_cpu_state_name[] = {
	[APPLE_CPU_PARKED]	= "parked",
	[APPLE_CPU_RUNNING]	= "running",
	[APPLE_CPU_OFF]		= "off",
};

struct apple_cpu {
	void __iomem *impl;		/* cpu-impl-reg, powerdown mode only */
	u64 __iomem *release;		/* m1n1 spin-table release slot */
	phys_addr_t release_pa;
	u64 rvbar;
	bool can_powerdown;
	int state;			/* enum apple_cpu_state */
	int down_mode;			/* APPLE_CPU_PARKED or APPLE_CPU_OFF */
	unsigned long parks;
	unsigned long powerdowns;
	unsigned long restarts;
	unsigned long status_timeouts;
};

extern const struct cpu_operations smp_spin_table_ops;
extern void secondary_holding_pen(void);
extern volatile unsigned long secondary_holding_pen_release;

/* smp_apple_asm.S */
void apple_cpu_mmu_off_jump(unsigned long entry, unsigned long arg0,
			    unsigned long arg1, unsigned long arg2);
void apple_cpu_park_mmuoff(unsigned long pen_pa, unsigned long levels);
void apple_cpu_off_mmuoff(unsigned long stop_pa, unsigned long levels,
			  unsigned long stop_bit);

static enum apple_hp_mode apple_hp_mode __ro_after_init;
static bool apple_hp_active __ro_after_init;
static struct apple_cpu apple_cpus[NR_CPUS];
static void __iomem *apple_cpu_start;
static void __iomem *apple_ps_uart0;
static bool apple_pmgr_unmapped;

static int __init apple_cpu_hotplug_setup(char *arg)
{
	if (!arg)
		return -EINVAL;

	if (!strcmp(arg, "park") || !strcmp(arg, "1"))
		apple_hp_mode = APPLE_HP_PARK;
	else if (!strcmp(arg, "powerdown"))
		apple_hp_mode = APPLE_HP_POWERDOWN;
	else if (!strcmp(arg, "0") || !strcmp(arg, "off"))
		apple_hp_mode = APPLE_HP_NONE;
	else
		return -EINVAL;

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

static bool apple_uart0_powered(void)
{
	/* No mapping: not t8103 powerdown, nothing to check. */
	if (!apple_ps_uart0)
		return true;

	return FIELD_GET(PS_ACTUAL, readl(apple_ps_uart0)) == PS_ACTIVE;
}

/* Is this page memory the kernel could hand out? */
static bool apple_pa_is_kernel_ram(phys_addr_t pa)
{
	unsigned long pfn = PHYS_PFN(pa);

	return pfn_valid(pfn) && !PageReserved(pfn_to_page(pfn));
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

/*
 * Map what powerdown needs and check that the way back is safe. Any failure
 * leaves this CPU in park mode.
 */
static void apple_cpu_prepare_powerdown(unsigned int cpu)
{
	struct apple_cpu *ac = &apple_cpus[cpu];
	phys_addr_t rvbar_pa;

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
	ac->release = ioremap_cache(ac->release_pa, sizeof(*ac->release));
	if (!ac->impl || !ac->release) {
		pr_err("CPU%u: cannot map cpu-impl-reg or release slot\n", cpu);
		return;
	}

	/*
	 * m1n1 writes each secondary's RVBAR itself before starting it, without
	 * the lock bit (m1n1 src/smp.c:184-187), so it points at m1n1's
	 * _vectors_start. A restarted core enters m1n1 there. That code and the
	 * release slot must still be intact: they live in m1n1's image, which
	 * m1n1 reserves (/memreserve/ [_base, _end), m1n1 src/kboot.c:3010).
	 * If the kernel can allocate those pages, the reservation was lost on
	 * the way through U-Boot and the EFI stub, and powerdown is unsafe.
	 */
	ac->rvbar = readq(ac->impl + CPU_IMPL_RVBAR);
	rvbar_pa = ac->rvbar & CPU_IMPL_RVBAR_ADDR;
	if (!rvbar_pa) {
		pr_warn("CPU%u: RVBAR is 0, powerdown disabled\n", cpu);
		return;
	}
	if (apple_pa_is_kernel_ram(rvbar_pa) ||
	    apple_pa_is_kernel_ram(ac->release_pa)) {
		pr_warn("CPU%u: m1n1 at RVBAR %pa / release %pa is not reserved, powerdown disabled\n",
			cpu, &rvbar_pa, &ac->release_pa);
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

static int apple_cpu_boot(unsigned int cpu)
{
	struct apple_cpu *ac = &apple_cpus[cpu];
	unsigned long pen_pa = __pa_symbol(secondary_holding_pen);

	if (READ_ONCE(ac->state) != APPLE_CPU_OFF) {
		/* Parked in the pen (or never started): spin-table release. */
		apple_write_pen_release(cpu_logical_map(cpu));
		sev();
		return 0;
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

	/* The slot still holds the pen; write it again in case m1n1 cleared it. */
	writeq_relaxed(pen_pa, ac->release);
	dcache_clean_inval_poc((__force unsigned long)ac->release,
			       (__force unsigned long)ac->release + sizeof(*ac->release));

	apple_write_pen_release(cpu_logical_map(cpu));

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

	/*
	 * Spin-table never resets the release word, so the last CPU booted
	 * would leave the pen at once the next time it is parked.
	 */
	apple_write_pen_release(INVALID_HWID);
	smp_store_release(&apple_cpus[cpu].state, APPLE_CPU_RUNNING);
}

static bool apple_cpu_can_disable(unsigned int cpu)
{
	return cpu != 0;
}

/* Runs on the dying CPU, before it is marked offline. */
static int apple_cpu_disable(unsigned int cpu)
{
	struct apple_cpu *ac = &apple_cpus[cpu];

	if (cpu == 0)
		return -EPERM;

	ac->down_mode = APPLE_CPU_PARKED;
	if (apple_hp_mode == APPLE_HP_POWERDOWN && ac->can_powerdown) {
		if (apple_uart0_powered())
			ac->down_mode = APPLE_CPU_OFF;
		else
			pr_warn_ratelimited("CPU%u: uart0 is off, parking instead of powering down\n",
					    cpu);
	}

	return 0;
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
 * Runs on the dying CPU with DAIF masked, after it reported itself dead.
 * No printk, no RCU from here on. Does not return.
 */
static void apple_cpu_die(unsigned int cpu)
{
	typeof(apple_cpu_mmu_off_jump) *jump;
	struct apple_cpu *ac = &apple_cpus[cpu];
	bool off = ac->down_mode == APPLE_CPU_OFF;
	unsigned long levels = 1;

	/* Never released by a stale release word. */
	apple_write_pen_release(INVALID_HWID);

	/*
	 * The last core of a cluster takes the cluster L2 with it if the
	 * cluster powers down; clean everything to the point of coherency.
	 */
	if (off && apple_cpu_last_in_cluster(cpu))
		levels = max_t(unsigned long,
			       FIELD_GET(GENMASK(26, 24), read_sysreg(clidr_el1)), 1);

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
		     T8103_CPU_START + CPU_START_STOP, levels, apple_cpu_bit(cpu));
	else
		jump(__pa_symbol(apple_cpu_park_mmuoff),
		     __pa_symbol(secondary_holding_pen), levels, 0);

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
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		usleep_range(100, 200);
	}

	if (state != APPLE_CPU_OFF)
		return 0;

	/*
	 * m1n1 smp_stop_cpu() polls the same status after a non-deep stop,
	 * including for the last core of a cluster (src/smp.c:244-250). The
	 * cluster L2 flush can take a few ms.
	 */
	ret = readq_poll_timeout(ac->impl + CPU_IMPL_PWRSTAT, val, !(val & 0xff),
				 100, 200 * USEC_PER_MSEC);
	if (ret) {
		ac->status_timeouts++;
		pr_warn("CPU%u: power status %#llx 200 ms after the stop request\n",
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

	if (apple_hp_mode == APPLE_HP_POWERDOWN &&
	    !of_machine_is_compatible("apple,t8103")) {
		pr_warn("powerdown is t8103-only, using park\n");
		apple_hp_mode = APPLE_HP_PARK;
	}

	if (!apple_hp_active)
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

static int apple_cpu_hotplug_show(struct seq_file *m, void *v)
{
	unsigned int cpu;

	seq_printf(m, "mode: %s\n",
		   apple_hp_mode == APPLE_HP_POWERDOWN ? "powerdown" : "park");
	seq_printf(m, "uart0_powered: %d\n", apple_uart0_powered());

	for_each_possible_cpu(cpu) {
		struct apple_cpu *ac = &apple_cpus[cpu];
		unsigned int state = READ_ONCE(ac->state);

		if (cpu == 0)
			continue;

		seq_printf(m, "cpu%u mpidr=%#llx bit=%#x state=%s can_powerdown=%d rvbar=%#llx parks=%lu powerdowns=%lu restarts=%lu status_timeouts=%lu\n",
			   cpu, cpu_logical_map(cpu), apple_cpu_bit(cpu),
			   state < ARRAY_SIZE(apple_cpu_state_name) ?
			   apple_cpu_state_name[state] : "?",
			   ac->can_powerdown, ac->rvbar, ac->parks,
			   ac->powerdowns, ac->restarts, ac->status_timeouts);
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

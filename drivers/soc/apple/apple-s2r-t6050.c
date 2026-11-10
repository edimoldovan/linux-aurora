// SPDX-License-Identifier: GPL-2.0-only
/*
 * Suspend-to-RAM ("mem", "deep") for the Apple M5 Max (t6050, J714c), first stage.
 *
 * Opt-in only: with apple_s2r=1 on the kernel command line (together with apple_cpu_hotplug=park,
 * which provides CPU offline) this registers platform_suspend_ops for PM_SUSPEND_MEM. Without it
 * nothing changes and the system keeps s2idle.
 *
 * Modelled on Naeem Malik's t8103 apple-s2r.c. The t6050 register operations come from the macOS 27
 * J714c kernelcache (AppleT6050PMGR and the kernel's deep-sleep site), with the physical addresses
 * from the J714c ADT (/arm-io/pmgr reg[]). The J714c is a single die. On the last online CPU, with
 * interrupts off:
 *
 *   1. watchdogs off (WD0, WD1): nothing pets them while the SoC sleeps,
 *   2. SIO power-state auto-enable cleared (PMGR 0x290, 0x2a0, bit 28), as quiesceClockConfig does,
 *   3. the four SoC clkgen muxes parked (0x90, 0x94, 0xa0, 0xa4: set bit 31, clear [27:24], wait for
 *      bit 30 to clear), as quiesceClockConfig does,
 *   4. every AIC interrupt masked,
 *   5. the S2R timer started (MasterMini PMGR + 0x4000), three times as on the t8103,
 *   6. into the t6050 deep WFI (apple_t6050_deep_wfi()).
 *
 * The secondaries go into the same deep WFI when the PM core takes them offline (apple_cpu_deep_park).
 * The system-level cache is left on: the J714c ADT has /arm-io/mcc skip-mcc-off-to-s2r = 1.
 *
 * This first stage has no resume: the wake (power button) comes back as a cold boot.
 */

#define pr_fmt(fmt) "apple-s2r-t6050: " fmt

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/suspend.h>

#include <asm/cpu_ops.h>

/* PMGR SoC east (pmgr reg[0]) */
#define T6050_PMGR			0x280600000ULL
#define PMGR_PS_SIO			0x290
#define PMGR_PS_SIO_CPU			0x2a0
#define PMGR_PS_AUTO_ENABLE		BIT(28)
#define PMGR_PS_TARGET			GENMASK(3, 0)
#define PMGR_PS_ACTUAL			GENMASK(7, 4)

/* SoC clkgen (pmgr reg[2]) */
#define T6050_CLKGEN			0x280000000ULL
static const u32 clkgen_muxes[] = { 0x90, 0x94, 0xa0, 0xa4 };
#define MUX_ENABLE			BIT(31)
#define MUX_BUSY			BIT(30)
#define MUX_SEL				GENMASK(27, 24)

/* MasterMini PMGR (pmgr reg[11]) */
#define T6050_S2R_TIMER			0x288f04000ULL

/* AON watchdog (/arm-io/wdt reg[0]) */
#define T6050_WDT			0x28836c000ULL
#define WDT_WD0_CTRL			0x0c
#define WDT_WD1_CTRL			0x1c

void apple_aic_mask_all(void);
void apple_t6050_deep_wfi(void);
extern bool apple_cpu_deep_park;

static bool apple_s2r_enabled __ro_after_init;

static unsigned int s2r_timer_val = 1;
module_param(s2r_timer_val, uint, 0644);
MODULE_PARM_DESC(s2r_timer_val, "value written to the S2R timer (MasterMini PMGR + 0x4000)");

static bool park_clocks = true;
module_param(park_clocks, bool, 0644);
MODULE_PARM_DESC(park_clocks, "park the SoC clkgen muxes and clear SIO auto-enable before the commit");

static void __iomem *pmgr, *clkgen, *s2r_timer, *wdt;

static int __init apple_s2r_setup(char *arg)
{
	apple_s2r_enabled = !arg || !strcmp(arg, "1") || !strcmp(arg, "on");
	return 0;
}
early_param("apple_s2r", apple_s2r_setup);

static void s2r_quiesce_clocks(void)
{
	static const u32 sio[] = { PMGR_PS_SIO, PMGR_PS_SIO_CPU };
	u32 v;
	int i;

	for (i = 0; i < ARRAY_SIZE(sio); i++) {
		v = readl_relaxed(pmgr + sio[i]);
		writel_relaxed(v & ~PMGR_PS_AUTO_ENABLE, pmgr + sio[i]);
		readl_poll_timeout_atomic(pmgr + sio[i], v,
					  FIELD_GET(PMGR_PS_ACTUAL, v) == FIELD_GET(PMGR_PS_TARGET, v),
					  1, 1000);
	}

	for (i = 0; i < ARRAY_SIZE(clkgen_muxes); i++) {
		v = readl_relaxed(clkgen + clkgen_muxes[i]);
		writel_relaxed((v | MUX_ENABLE) & ~MUX_SEL, clkgen + clkgen_muxes[i]);
		readl_poll_timeout_atomic(clkgen + clkgen_muxes[i], v, !(v & MUX_BUSY), 1, 1000);
	}
}

static int apple_s2r_enter(suspend_state_t state)
{
	int i;

	if (state != PM_SUSPEND_MEM)
		return -EINVAL;
	if (num_online_cpus() != 1)
		return -EBUSY;

	writel_relaxed(0, wdt + WDT_WD0_CTRL);
	writel_relaxed(0, wdt + WDT_WD1_CTRL);

	if (park_clocks)
		s2r_quiesce_clocks();

	apple_aic_mask_all();

	for (i = 0; i < 3; i++) {
		writel_relaxed(s2r_timer_val, s2r_timer);
		dsb(sy);
	}

	apple_t6050_deep_wfi();
	return 0;
}

static int apple_s2r_prepare_late(void)
{
	WRITE_ONCE(apple_cpu_deep_park, true);
	return 0;
}

static void apple_s2r_wake(void)
{
	WRITE_ONCE(apple_cpu_deep_park, false);
}

static const struct platform_suspend_ops apple_s2r_ops = {
	.valid		= suspend_valid_only_mem,
	.prepare_late	= apple_s2r_prepare_late,
	.enter		= apple_s2r_enter,
	.wake		= apple_s2r_wake,
};

static int __init apple_s2r_init(void)
{
	if (!apple_s2r_enabled)
		return 0;
	if (!of_machine_is_compatible("apple,t6050")) {
		pr_warn("apple_s2r=1 on a machine that is not a t6050, ignored\n");
		return 0;
	}
	if (!apple_cpu_hotplug_active()) {
		pr_warn("no CPU offline (apple_cpu_hotplug=park missing), staying with s2idle\n");
		return 0;
	}

	pmgr = ioremap_np(T6050_PMGR, SZ_4K);
	clkgen = ioremap_np(T6050_CLKGEN, SZ_4K);
	s2r_timer = ioremap_np(T6050_S2R_TIMER, SZ_4K);
	wdt = ioremap_np(T6050_WDT, SZ_4K);
	if (!pmgr || !clkgen || !s2r_timer || !wdt) {
		pr_err("cannot map the PMGR windows\n");
		return -ENOMEM;
	}

	suspend_set_ops(&apple_s2r_ops);
	pr_info("suspend-to-RAM registered (first stage: the wake is a cold boot)\n");
	return 0;
}
late_initcall(apple_s2r_init);

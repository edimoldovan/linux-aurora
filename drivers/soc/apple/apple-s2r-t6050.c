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
#include <linux/of_address.h>
#include <linux/suspend.h>

#include <asm/arch_timer.h>

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

/*
 * AP wake-source enables in die 0's AOP global block (RegMap id 0x42 = pmgr reg[12] 0x288000000,
 * + 0x400c, 11 words; a set bit enables that source, AppleT6050PMGR::setAPWakeupSources writes
 * valid & ~mask). Source id n is word n / 32, bit n % 32. The power button arrives as
 * SMC.CPUWakeupAP, id 122 (ADT ap-wake-sources).
 */
#define T6050_AOP_GLOBAL		0x288000000ULL
#define AOP_WAKE_EN(i)			(0x400c + 4 * (i))
#define AOP_WAKE_WORDS			11

/* S2R count (MiniPMGR, pmgr reg[9] 0x288300000 + 0x70a70; ApplePMGR::_getS2RLoCount) */
#define T6050_S2R_COUNT			0x288370a70ULL
#define WAKE_SRC_SMC			122

/*
 * SoC and DCS perf-state requests (RegMap id 0x5c = pmgr reg[4]); AppleT6050PMGR::quiescePerfState
 * drops them to SOC 4 -> DCS 8 -> SOC 3 -> DCS 6, each a read-modify-write of bits [3:0] that the
 * hardware acknowledges by clearing bit 31.
 */
#define T6050_PERF			0x280200000ULL
#define PERF_SOC			0x000
#define PERF_DCS			0x400
#define PERF_STATE			GENMASK(3, 0)
#define PERF_BUSY			BIT(31)

/* AON watchdog (/arm-io/wdt reg[0]) */
#define T6050_WDT			0x28836c000ULL
#define WDT_WD0_CTRL			0x0c
#define WDT_WD1_CTRL			0x1c

void apple_aic_mask_all(void);
void apple_t6050_deep_wfi(void);
void apple_t6050_deep_wfi_once(void);

/*
 * A marker byte in the primary PMU's legacy scratchpad (nub-spmi0 0x28a014000, USID 14, info-leg_scrpad
 * 0xf800 + 8), which a SoC reset leaves alone: 0x5a at the entry, 0xa1 before the boot CPU's deep WFI, 0xa2
 * each time that WFI returns. Read back from m1n1 after the reset: 0xa1 = the core gated.
 */
#define T6050_SPMI			0x28a014000ULL
#define SPMI_STATUS			0x200
#define SPMI_CMD			0x210
#define SPMI_REPLY			0x220
#define SPMI_RX_EMPTY			BIT(30)
#define SPMI_EXT_WRITEL			0x30
#define PMU_USID			0xe
#define PMU_MARKER			0xf808
#define PMU_PS_OFF			0xf809	/* power states switched off at the entry */
#define PMU_PS_STUCK			0xf80a	/* of those, how many did not reach off */
#define PMU_PS_LEFT			0xf80b	/* mapped power states still on after the pass */
#define PMU_PMP_OFF			0xf80c	/* 2 bytes: bit i = s2r_pmp_ps[i] reached off */
extern bool apple_cpu_deep_park;

static bool apple_s2r_enabled __ro_after_init;

static unsigned int s2r_timer_val = 1;
module_param(s2r_timer_val, uint, 0644);
MODULE_PARM_DESC(s2r_timer_val, "value written to the S2R timer (MasterMini PMGR + 0x4000)");

static bool s2r_timer_start = true;
module_param(s2r_timer_start, bool, 0644);
MODULE_PARM_DESC(s2r_timer_start, "start the S2R timer before the deep WFI (off: the SoC is left to commit on its own)");

static bool park_clocks = true;
module_param(park_clocks, bool, 0644);
MODULE_PARM_DESC(park_clocks, "park the SoC clkgen muxes and clear SIO auto-enable before the commit");

static unsigned int stop_after;
module_param(stop_after, uint, 0644);
MODULE_PARM_DESC(stop_after, "bisect the entry: spin with IRQs off after step N (1 watchdogs, 2 clocks, 3 AIC, 4 S2R timer; 0 = run it all)");

static bool wake_power_only = true;
module_param(wake_power_only, bool, 0644);
MODULE_PARM_DESC(wake_power_only, "enable only the SMC wake source (power button) before the commit");

static bool perf_quiesce = true;
module_param(perf_quiesce, bool, 0644);
MODULE_PARM_DESC(perf_quiesce, "drop the SoC and DCS perf states as macOS quiescePerfState does before the commit");

static void __iomem *pmgr, *clkgen, *s2r_timer, *wdt, *aop, *s2r_count, *perf, *spmi;

static void s2r_pmu_write(u16 addr, u8 val)
{
	u32 v;

	writel_relaxed(SPMI_EXT_WRITEL | PMU_USID << 8 | BIT(15) | (u32)addr << 16, spmi + SPMI_CMD);
	writel_relaxed(val, spmi + SPMI_CMD);
	if (!readl_poll_timeout_atomic(spmi + SPMI_STATUS, v, !(v & SPMI_RX_EMPTY), 1, 50000))
		readl_relaxed(spmi + SPMI_REPLY);
}

static void s2r_mark(u8 val)
{
	s2r_pmu_write(PMU_MARKER, val);
}

/*
 * The power states Linux leaves on that macOS has off before it commits: every PMGR power state that is
 * neither always-on nor firmware-owned, except the SMC's I2C masters (the SMC needs them until slpa;
 * the entry runs after it). genpd keeps the ones whose consumers never probed on (GENPD_FLAG_DEFER_OFF).
 * Mapped at init, switched off in reverse DT order (children follow their parents in the DT) at the
 * entry, after the last device callback.
 */
#define S2R_MAX_PS			320
static void __iomem *s2r_ps[S2R_MAX_PS];
static unsigned int s2r_nps;

static bool ps_off = true;
module_param(ps_off, bool, 0644);
MODULE_PARM_DESC(ps_off, "switch off the PMGR power states Linux left on at the entry");

/*
 * The PMP-tracked states macOS has off at S2R that Linux keeps on (ADT notify_pmp, not critical): ANS,
 * then the display chains, the DCP coprocessor before its front end and system state.
 */
static const char * const s2r_pmp_ps[] = {
	"ans", "disp_cpu", "disp_fe", "disp_sys",
	"dispext0_cpu", "dispext0_fe", "dispext0_sys", "dispext1_cpu", "dispext1_fe", "dispext1_sys",
	"dispext2_cpu", "dispext2_fe", "dispext2_sys", "dispext3_cpu", "dispext3_fe", "dispext3_sys",
};
static void __iomem *s2r_pmp_map[ARRAY_SIZE(s2r_pmp_ps)];

static bool pmp_ps_off = true;
module_param(pmp_ps_off, bool, 0644);
MODULE_PARM_DESC(pmp_ps_off, "switch off ANS and the display chains at the entry");

static bool pmp_ans = true;
module_param(pmp_ans, bool, 0644);
MODULE_PARM_DESC(pmp_ans, "include ANS in the PMP-tracked states switched off at the entry");

static bool pmp_disp = true;
module_param(pmp_disp, bool, 0644);
MODULE_PARM_DESC(pmp_disp, "include the main display chain (disp_cpu/fe/sys) in the PMP-tracked states");

static bool pmp_cpu;
module_param(pmp_cpu, bool, 0644);
MODULE_PARM_DESC(pmp_cpu, "include the DCP coprocessor states (*_cpu); switching a live one off resets the SoC");

/*
 * The main DCP's RTKit sleep, as m1n1's dcp_shutdown does it (rtkit_sleep, then a DISP_CPU reset):
 * management endpoint 0, SET_AP_PWR_STATE QUIESCED, then SET_IOP_PWR_STATE SLEEP, each acknowledged;
 * stop the ASC CPU; pulse the DISP_CPU reset. Marker 0x80-0x84 per step, 0x9x where one timed out.
 */
#define T6050_DCP_ASC			0x48ae00000ULL
#define ASC_CPU_CONTROL			0x44
#define ASC_CPU_CONTROL_RUN		BIT(4)
#define ASC_CPU_STATUS			0x48
#define ASC_CPU_STATUS_STOPPED		BIT(1)
#define ASC_MBOX			0x8000
#define MBOX_A2I_CONTROL		0x110
#define MBOX_A2I_FULL			BIT(16)
#define MBOX_I2A_CONTROL		0x114
#define MBOX_I2A_EMPTY			BIT(17)
#define MBOX_A2I_SEND0			0x800
#define MBOX_A2I_SEND1			0x808
#define MBOX_I2A_RECV0			0x830
#define MBOX_I2A_RECV1			0x838
#define RTKIT_MGMT_TYPE			GENMASK_ULL(59, 52)
#define RTKIT_SET_AP_PWR		0xb
#define RTKIT_SET_IOP_PWR		0x6
#define RTKIT_IOP_PWR_ACK		0x7
#define RTKIT_PWR_QUIESCED		0x10
#define RTKIT_PWR_SLEEP			0x1
#define PS_DEV_DISABLE			BIT(10)
#define PS_RESET			BIT(31)

static bool dcp_sleep = true;
module_param(dcp_sleep, bool, 0644);
MODULE_PARM_DESC(dcp_sleep, "put the main DCP to sleep over RTKit, then switch off its display chain");

static bool dcp_reset;
module_param(dcp_reset, bool, 0644);
MODULE_PARM_DESC(dcp_reset, "pulse the DISP_CPU reset after the DCP stops, as m1n1 does (resets the T6050 SoC)");

static bool dcp_chain;
module_param(dcp_chain, bool, 0644);
MODULE_PARM_DESC(dcp_chain, "switch the main display chain off after the DCP sleeps (disp_cpu off resets the T6050 SoC)");

static void __iomem *dcp_asc, *pmp_asc;

/* The PMP's ASC: m1n1's deep transition (QUIESCED, SLEEP, stop) instead of the AP IDLE 0x201 alone. */
#define T6050_PMP_ASC			0x284e00000ULL

static bool pmp_sleep = true;
module_param(pmp_sleep, bool, 0644);
MODULE_PARM_DESC(pmp_sleep, "put the PMP to sleep over RTKit (QUIESCED, SLEEP, ASC stop) at the entry");

static bool s2r_rtkit_xfer(void __iomem *asc, u64 msg, u8 type, u16 state)
{
	void __iomem *mb = asc + ASC_MBOX;
	u64 r0, r1;
	u32 v;
	int i;

	if (readl_poll_timeout_atomic(mb + MBOX_A2I_CONTROL, v, !(v & MBOX_A2I_FULL), 1, 1000000))
		return false;
	writeq_relaxed(msg, mb + MBOX_A2I_SEND0);
	writeq_relaxed(0, mb + MBOX_A2I_SEND1);
	for (i = 0; i < 64; i++) {
		if (readl_poll_timeout_atomic(mb + MBOX_I2A_CONTROL, v, !(v & MBOX_I2A_EMPTY), 1, 1000000))
			return false;
		r0 = readq_relaxed(mb + MBOX_I2A_RECV0);
		r1 = readq_relaxed(mb + MBOX_I2A_RECV1);
		if (!(r1 & 0xff) && FIELD_GET(RTKIT_MGMT_TYPE, r0) == type && (r0 & 0xffff) == state)
			return true;
	}
	return false;
}

/*
 * An RTKit coprocessor's sleep: AP QUIESCED, IOP SLEEP, each acknowledged, then its ASC CPU stopped.
 * Marker @mark at the start and @mark + n after step n; @mark + 0x10 + n where step n timed out.
 */
static bool s2r_rtkit_sleep(void __iomem *asc, u8 mark)
{
	u32 v;

	s2r_mark(mark);
	if (!s2r_rtkit_xfer(asc, FIELD_PREP(RTKIT_MGMT_TYPE, RTKIT_SET_AP_PWR) | RTKIT_PWR_QUIESCED,
			    RTKIT_SET_AP_PWR, RTKIT_PWR_QUIESCED)) {
		s2r_mark(mark + 0x11);
		return false;
	}
	s2r_mark(mark + 1);
	if (!s2r_rtkit_xfer(asc, FIELD_PREP(RTKIT_MGMT_TYPE, RTKIT_SET_IOP_PWR) | RTKIT_PWR_SLEEP,
			    RTKIT_IOP_PWR_ACK, RTKIT_PWR_SLEEP)) {
		s2r_mark(mark + 0x12);
		return false;
	}
	s2r_mark(mark + 2);
	writel_relaxed(readl_relaxed(asc + ASC_CPU_CONTROL) & ~ASC_CPU_CONTROL_RUN, asc + ASC_CPU_CONTROL);
	if (readl_poll_timeout_atomic(asc + ASC_CPU_STATUS, v, v & ASC_CPU_STATUS_STOPPED, 1, 1000000)) {
		s2r_mark(mark + 0x13);
		return false;
	}
	s2r_mark(mark + 3);
	return true;
}

static bool s2r_quiesce_dcp(void __iomem *ps)
{
	if (!s2r_rtkit_sleep(dcp_asc, 0x80))
		return false;
	if (!dcp_reset)
		return true;
	writel_relaxed(readl_relaxed(ps) | PS_DEV_DISABLE, ps);
	writel_relaxed(readl_relaxed(ps) | PS_RESET, ps);
	udelay(10);
	writel_relaxed(readl_relaxed(ps) & ~PS_RESET, ps);
	writel_relaxed(readl_relaxed(ps) & ~PS_DEV_DISABLE, ps);
	s2r_mark(0x84);
	return true;
}

static void __init s2r_map_ps(void)
{
	struct device_node *np, *parent;
	struct resource r;
	const char *label;
	unsigned int i, n;
	u32 off;

	for_each_compatible_node(np, NULL, "apple,t8103-pmgr-pwrstate") {
		if (!of_property_read_string(np, "label", &label)) {
			int j = match_string(s2r_pmp_ps, ARRAY_SIZE(s2r_pmp_ps), label);

			if (j >= 0) {
				parent = of_get_parent(np);
				if (!of_address_to_resource(parent, 0, &r) &&
				    !of_property_read_u32(np, "reg", &off))
					s2r_pmp_map[j] = ioremap_np(r.start + off, 4);
				of_node_put(parent);
				continue;
			}
		}
		if (s2r_nps == S2R_MAX_PS)
			continue;
		if (of_property_read_bool(np, "apple,always-on") ||
		    of_property_read_bool(np, "apple,firmware-owned"))
			continue;
		if (!of_property_read_string(np, "label", &label) && !strncmp(label, "smc_i2cm", 8))
			continue;
		parent = of_get_parent(np);
		if (of_address_to_resource(parent, 0, &r) || of_property_read_u32(np, "reg", &off)) {
			of_node_put(parent);
			continue;
		}
		of_node_put(parent);
		s2r_ps[s2r_nps] = ioremap_np(r.start + off, 4);
		if (s2r_ps[s2r_nps])
			s2r_nps++;
	}
	for (i = 0, n = 0; i < ARRAY_SIZE(s2r_pmp_map); i++)
		n += !!s2r_pmp_map[i];
	pr_info("%u power states to switch off at the entry, %u of %zu PMP-tracked ones mapped\n", s2r_nps,
		n, ARRAY_SIZE(s2r_pmp_ps));
}

/*
 * The main chain (disp_cpu/fe/sys) goes off once the DCP sleeps, or with pmp_disp when dcp_sleep is off;
 * a live DCP coprocessor state only with pmp_cpu: switching one off resets the SoC.
 */
static bool s2r_pmp_wanted(int i, bool dcp_ok)
{
	bool cpu = strstr(s2r_pmp_ps[i], "_cpu");

	if (i <= 3)
		return (dcp_ok && dcp_chain) || (!dcp_sleep && pmp_disp && (!cpu || pmp_cpu));
	return !cpu || pmp_cpu;
}

static bool s2r_pmp_ps_off(int i)
{
	u32 v;

	s2r_mark(0x70 + i);
	v = readl_relaxed(s2r_pmp_map[i]);
	writel_relaxed(v & ~(PMGR_PS_TARGET | PMGR_PS_AUTO_ENABLE), s2r_pmp_map[i]);
	return !readl_poll_timeout_atomic(s2r_pmp_map[i], v, !FIELD_GET(PMGR_PS_ACTUAL, v), 1, 1000);
}

/*
 * The display chains first (their states go before anything they sit on), then every other state in
 * reverse DT order, then ANS last. The marker byte tracks it: 0x70 + the PMP-tracked index, 0x60 during
 * the main pass, 0x61 after it.
 */
static void s2r_quiesce_ps(void)
{
	unsigned int n = 0, stuck = 0, pmp = 0;
	bool dcp_ok = false;
	u32 v;
	int i;

	if (dcp_sleep && dcp_asc && s2r_pmp_map[1])
		dcp_ok = s2r_quiesce_dcp(s2r_pmp_map[1]);

	if (pmp_ps_off)
		for (i = 1; i < ARRAY_SIZE(s2r_pmp_ps); i++)
			if (s2r_pmp_map[i] && s2r_pmp_wanted(i, dcp_ok) && s2r_pmp_ps_off(i))
				pmp |= BIT(i);

	s2r_mark(0x60);
	for (i = s2r_nps - 1; i >= 0; i--) {
		v = readl_relaxed(s2r_ps[i]);
		if (!FIELD_GET(PMGR_PS_ACTUAL, v))
			continue;
		writel_relaxed(v & ~(PMGR_PS_TARGET | PMGR_PS_AUTO_ENABLE), s2r_ps[i]);
		n++;
		if (readl_poll_timeout_atomic(s2r_ps[i], v, !FIELD_GET(PMGR_PS_ACTUAL, v), 1, 1000))
			stuck++;
	}
	s2r_pmu_write(PMU_PS_OFF, min(n, 255U));
	s2r_pmu_write(PMU_PS_STUCK, min(stuck, 255U));
	for (n = 0, i = 0; i < s2r_nps; i++)
		if (FIELD_GET(PMGR_PS_ACTUAL, readl_relaxed(s2r_ps[i])))
			n++;
	s2r_pmu_write(PMU_PS_LEFT, min(n, 255U));
	s2r_mark(0x61);

	if (pmp_ps_off && pmp_ans && s2r_pmp_map[0] && s2r_pmp_ps_off(0))
		pmp |= BIT(0);
	s2r_pmu_write(PMU_PMP_OFF, pmp & 0xff);
	s2r_pmu_write(PMU_PMP_OFF + 1, pmp >> 8);
}

static void s2r_stop(unsigned int step)
{
	if (stop_after != step)
		return;
	for (;;)
		cpu_relax();
}

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

static void s2r_quiesce_perf(void)
{
	static const struct { u32 reg, state; } steps[] = {
		{ PERF_SOC, 4 }, { PERF_DCS, 8 }, { PERF_SOC, 3 }, { PERF_DCS, 6 },
	};
	u32 v;
	int i;

	for (i = 0; i < ARRAY_SIZE(steps); i++) {
		v = readl_relaxed(perf + steps[i].reg);
		writel_relaxed((v & ~PERF_STATE) | steps[i].state, perf + steps[i].reg);
		readl_poll_timeout_atomic(perf + steps[i].reg, v, !(v & PERF_BUSY), 1, 10000);
	}
}

static int apple_s2r_enter(suspend_state_t state)
{
	int i;

	if (state != PM_SUSPEND_MEM)
		return -EINVAL;
	if (num_online_cpus() != 1)
		return -EBUSY;

	s2r_mark(0x5a);

	if (wake_power_only)
		for (i = 0; i < AOP_WAKE_WORDS; i++)
			writel_relaxed(i == WAKE_SRC_SMC / 32 ? BIT(WAKE_SRC_SMC % 32) : 0,
				       aop + AOP_WAKE_EN(i));

	writel_relaxed(0, wdt + WDT_WD0_CTRL);
	writel_relaxed(0, wdt + WDT_WD1_CTRL);
	s2r_stop(1);

	if (perf_quiesce)
		s2r_quiesce_perf();

	if (ps_off)
		s2r_quiesce_ps();

	if (pmp_sleep && pmp_asc)
		s2r_pmu_write(PMU_PS_OFF, s2r_rtkit_sleep(pmp_asc, 0xb0) ? 0xb3 : 0xbf);

	if (park_clocks)
		s2r_quiesce_clocks();
	s2r_stop(2);

	apple_aic_mask_all();
	s2r_stop(3);

	for (i = 0; i < 3 && s2r_timer_start; i++) {
		writel_relaxed(s2r_timer_val, s2r_timer);
		dsb(sy);
	}
	s2r_stop(4);

	/* A timer interrupt 1 s ahead brings an ungated core back out of the WFI (0xa2). */
	write_sysreg(read_sysreg(cntvct_el0) + arch_timer_get_cntfrq(), cntv_cval_el0);
	write_sysreg(ARCH_TIMER_CTRL_ENABLE, cntv_ctl_el0);
	isb();
	s2r_mark(0xa1);
	for (;;) {
		apple_t6050_deep_wfi_once();
		s2r_mark(0xa2);
	}
	return 0;
}

/*
 * The PMP (power-management coprocessor): macOS idles it (AP and IOP power IDLE) between MBSE slpw
 * and slpa. .prepare sits exactly there: after every device's .suspend (macsmc sends slpw), before
 * .suspend_late (macsmc sends slpa). The PMP driver registers the hook once it runs.
 */
static int (*s2r_pmp_power)(bool idle);
static bool s2r_pmp_idled;

static bool idle_pmp = true;
module_param(idle_pmp, bool, 0644);
MODULE_PARM_DESC(idle_pmp, "idle the PMP before the commit, as macOS does");

void apple_s2r_set_pmp_power(int (*fn)(bool idle))
{
	WRITE_ONCE(s2r_pmp_power, fn);
}
EXPORT_SYMBOL_GPL(apple_s2r_set_pmp_power);

static int apple_s2r_prepare(void)
{
	int (*fn)(bool) = READ_ONCE(s2r_pmp_power);
	int ret;

	s2r_pmp_idled = false;
	if (!idle_pmp)
		return 0;
	if (!fn) {
		pr_warn("no PMP hook registered, the PMP stays up\n");
		return 0;
	}
	ret = fn(true);
	pr_info("PMP idle: %d\n", ret);
	s2r_pmp_idled = !ret;
	return 0;
}

static void apple_s2r_finish(void)
{
	int (*fn)(bool) = READ_ONCE(s2r_pmp_power);

	if (s2r_pmp_idled && fn)
		pr_info("PMP wake: %d\n", fn(false));
	s2r_pmp_idled = false;
}

/*
 * macOS checks that the power domains are off before it commits (ApplePMGR::validateDevicePSLevel,
 * ADT check-ps-b4-sleep): log every PMGR power state whose actual state is not off.
 */
static void s2r_log_powered(void)
{
	struct device_node *np, *parent;
	struct resource r;
	void __iomem *ps;
	unsigned int on = 0, all = 0;
	const char *label;
	u32 off, v;

	for_each_compatible_node(np, NULL, "apple,t8103-pmgr-pwrstate") {
		parent = of_get_parent(np);
		if (of_address_to_resource(parent, 0, &r) || of_property_read_u32(np, "reg", &off)) {
			of_node_put(parent);
			continue;
		}
		of_node_put(parent);
		ps = ioremap_np(r.start + off, 4);
		if (!ps)
			continue;
		v = readl(ps);
		iounmap(ps);
		all++;
		if (!FIELD_GET(PMGR_PS_ACTUAL, v))
			continue;
		on++;
		if (of_property_read_string(np, "label", &label))
			label = np->name;
		pr_info("on before sleep: %s %llx %08x%s\n", label, (u64)r.start + off, v,
			of_property_read_bool(np, "apple,always-on") ? " always-on" : "");
	}
	pr_info("%u of %u power domains on before sleep\n", on, all);
}

static int apple_s2r_prepare_late(void)
{
	s2r_log_powered();
	WRITE_ONCE(apple_cpu_deep_park, true);
	return 0;
}

static void apple_s2r_wake(void)
{
	WRITE_ONCE(apple_cpu_deep_park, false);
}

static const struct platform_suspend_ops apple_s2r_ops = {
	.valid		= suspend_valid_only_mem,
	.prepare	= apple_s2r_prepare,
	.prepare_late	= apple_s2r_prepare_late,
	.enter		= apple_s2r_enter,
	.wake		= apple_s2r_wake,
	.finish		= apple_s2r_finish,
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
	aop = ioremap_np(T6050_AOP_GLOBAL, SZ_32K);
	s2r_count = ioremap_np(T6050_S2R_COUNT, 4);
	perf = ioremap_np(T6050_PERF, SZ_4K);
	spmi = ioremap_np(T6050_SPMI, SZ_4K);
	if (!pmgr || !clkgen || !s2r_timer || !wdt || !aop || !s2r_count || !perf || !spmi) {
		pr_err("cannot map the PMGR windows\n");
		return -ENOMEM;
	}

	pr_info("AP wake enables now: %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x %08x; S2R count %08x\n",
		readl(aop + AOP_WAKE_EN(0)), readl(aop + AOP_WAKE_EN(1)), readl(aop + AOP_WAKE_EN(2)),
		readl(aop + AOP_WAKE_EN(3)), readl(aop + AOP_WAKE_EN(4)), readl(aop + AOP_WAKE_EN(5)),
		readl(aop + AOP_WAKE_EN(6)), readl(aop + AOP_WAKE_EN(7)), readl(aop + AOP_WAKE_EN(8)),
		readl(aop + AOP_WAKE_EN(9)), readl(aop + AOP_WAKE_EN(10)), readl(s2r_count));

	/* Probe: does the wake-enable word keep a written value? Restored right after. */
	{
		u32 w3 = readl(aop + AOP_WAKE_EN(WAKE_SRC_SMC / 32));

		writel(w3 | BIT(WAKE_SRC_SMC % 32), aop + AOP_WAKE_EN(WAKE_SRC_SMC / 32));
		pr_info("wake word 3: %08x -> wrote bit %d -> reads %08x; S2R timer reads %08x\n", w3,
			WAKE_SRC_SMC % 32, readl(aop + AOP_WAKE_EN(WAKE_SRC_SMC / 32)), readl(s2r_timer));
		writel(w3, aop + AOP_WAKE_EN(WAKE_SRC_SMC / 32));
	}

	suspend_set_ops(&apple_s2r_ops);
	{
		u32 soc = readl(perf + PERF_SOC), dcs = readl(perf + PERF_DCS);

		pr_info("perf states: SOC %08x DCS %08x\n", soc, dcs);
		if (FIELD_GET(PERF_STATE, soc) < 3 || FIELD_GET(PERF_STATE, soc) > 5 ||
		    FIELD_GET(PERF_STATE, dcs) < 6 || FIELD_GET(PERF_STATE, dcs) > 13) {
			pr_info("perf states out of range, not dropping them\n");
			perf_quiesce = false;
		}
	}
	s2r_map_ps();
	dcp_asc = ioremap_np(T6050_DCP_ASC, SZ_64K);
	pmp_asc = ioremap_np(T6050_PMP_ASC, SZ_64K);
	pr_info("suspend-to-RAM registered (first stage: the wake is a cold boot)\n");
	return 0;
}
late_initcall(apple_s2r_init);

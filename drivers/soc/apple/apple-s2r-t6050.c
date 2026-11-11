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

#include "apple-s2r-t6050-macos-tail.h"
#include "apple-s2r-t6050-hpm-words.h"
#include "apple-s2r-t6050-macos-ps.h"

/*
 * replay_tail: replay macOS's own last 1.25 s before the S2R timer start, as traced under aura
 * (kernel/S2R-MACOS-TRACE.md): the 5 s sleep watchdog, AMCC, MCC and DCS, the power gates.
 */
static bool replay_tail = true;
module_param(replay_tail, bool, 0644);
MODULE_PARM_DESC(replay_tail, "replay macOS's traced boot-core writes of the last 1.25 s before the S2R timer");
static void __iomem *macos_tail_map[ARRAY_SIZE(macos_tail_pages)];

static void s2r_macos_tail(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(macos_tail); i++) {
		void __iomem *r = macos_tail_map[macos_tail[i].page] + macos_tail[i].off;

		if (macos_tail[i].w64)
			writeq_relaxed(macos_tail[i].val, r);
		else
			writel_relaxed(macos_tail[i].val, r);
		dsb(sy);
	}
}

/* PMGR SoC east (pmgr reg[1]; reg[0] 0x280600000 holds atc1_cio and dispext2_fe at these offsets) */
#define T6050_PMGR			0x280900000ULL
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

/* The other /arm-io/wdt windows (ADT reg[1..4]); which ones macOS zeroes for S2R is not known */
static const u64 wdt_extra[] = { 0x288335260ULL, 0x288330008ULL, 0x28833002cULL, 0x288330020ULL };

/*
 * AFR's power state in the GPU-side PS block (RegMap id 6, reg[51] 0x2300e80000 + 0x100; GFX at + 0x110).
 * AppleT6050PMGR::quiesceHW powers it off: [19:16] = 4, wait for [19:16] and actual 4, write 0, wait
 * for actual 0. (soc_cfg_pulse replays only the first half.)
 */
#define T6050_SOC_CFG			0x2300e80100ULL
#define SOC_CFG_FIELD			GENMASK(19, 16)

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

static bool wdt_extra_off = true;
module_param(wdt_extra_off, bool, 0644);
MODULE_PARM_DESC(wdt_extra_off, "zero the other /arm-io/wdt windows at the entry");

static bool soc_cfg_pulse;
module_param(soc_cfg_pulse, bool, 0644);
MODULE_PARM_DESC(soc_cfg_pulse, "pulse quiesceHW's SOC cfg field ([19:16] = 4, then 0) before the S2R timer");

static void s2r_mark(u8 val);
static void s2r_pmu_write(u16 addr, u8 val);

/*
 * ApplePMGR::enableCluster(c, false) from quiesceHW: RegGroup 2 = RegMap 0 + 0x20000 = 0x280620000, + 4 * the
 * physical cluster ID from the ADT soc-clusters records. The J714c has one record, DISP_INT (physical 3):
 * its low nibble goes to 0, then wait for bit 4 (transition busy) to clear.
 */
#define T6050_CLUSTER_PS		0x280620000ULL
#define CLUSTER_PS_TARGET		GENMASK(3, 0)
#define CLUSTER_PS_BUSY			BIT(4)
static const struct { u8 phys; bool wait; } s2r_clusters[] = {
	{ 0x03, true },
};
static void __iomem *cluster_ps;

static bool cluster_off = true;
module_param(cluster_off, bool, 0644);
MODULE_PARM_DESC(cluster_off, "disable the CPU and display clusters in the PMGR before the commit, as quiesceHW does");

static void s2r_cluster_off(void)
{
	void __iomem *r;
	unsigned int i, waited = 0;
	u32 v;

	for (i = 0; i < ARRAY_SIZE(s2r_clusters); i++) {
		r = cluster_ps + 4 * s2r_clusters[i].phys;
		s2r_mark(0xd0 + i);
		writel_relaxed(readl_relaxed(r) & ~CLUSTER_PS_TARGET, r);
		if (s2r_clusters[i].wait &&
		    !readl_poll_timeout_atomic(r, v, !(v & CLUSTER_PS_BUSY), 1, 192000))
			waited |= BIT(i);
	}
	s2r_pmu_write(PMU_PMP_OFF + 1, 0xe0 | waited >> 3);
}

/*
 * AppleT6050PMC::quiesce (quiesceHW's PMC branch): the AP agent (ADT ap-voter-agent-idx 3) drops its perf
 * floor for domains 1 and 0 to 0 and waits. PMC = pmgr reg[8] 0x286000000 + 0x700000; the AP floor
 * register at + 0xc000 holds a 4-bit field per domain, the pending flag is bit 3 of + 0xc004.
 */
#define T6050_PMC_AP_FLOOR		0x28670c000ULL
#define PMC_AP_PENDING			0x4
#define PMC_AP_PENDING_BUSY		BIT(3)
static void __iomem *pmc_ap;

static bool pmc_floor_off = true;
module_param(pmc_floor_off, bool, 0644);
MODULE_PARM_DESC(pmc_floor_off, "drop the AP's PMC perf floor for domains 0 and 1, as AppleT6050PMC::quiesce does");

static void s2r_pmc_quiesce(void)
{
	u32 v = readl_relaxed(pmc_ap);

	/* The floor as found at the entry goes to f809, the outcome to f80c. */
	s2r_pmu_write(PMU_PS_OFF, v & 0xff);
	writel_relaxed(v & ~GENMASK(7, 0), pmc_ap);
	s2r_pmu_write(PMU_PMP_OFF, readl_poll_timeout_atomic(pmc_ap + PMC_AP_PENDING, v,
					!(v & PMC_AP_PENDING_BUSY), 1, 192000) ? 0xfd : 0xf0);
}

/*
 * quiesceACC -> AppleT6050PMGR::enableCPMPowerDomainGating(true), first in quiesceHW: bit 31 of
 * pmgr reg[0] + 0x2c000 + 0x120 + 8 k for each of the three CPU clusters.
 */
#define T6050_CPM_PDG			0x28062c120ULL
#define CPM_PDG_ENABLE			BIT(31)
static void __iomem *cpm_pdg;

static bool cpm_gating = true;
module_param(cpm_gating, bool, 0644);
MODULE_PARM_DESC(cpm_gating, "enable CPM power-domain gating for the three CPU clusters, as quiesceACC does");

static void s2r_cpm_gating(void)
{
	u32 before = 0;
	int k;

	for (k = 0; k < 3; k++) {
		before |= !!(readl_relaxed(cpm_pdg + 8 * k) & CPM_PDG_ENABLE) << k;
		writel_relaxed(readl_relaxed(cpm_pdg + 8 * k) | CPM_PDG_ENABLE, cpm_pdg + 8 * k);
	}
	/* f809: 0xa0 | which clusters already had it at the entry */
	s2r_pmu_write(PMU_PS_OFF, 0xa0 | before);
}

/*
 * arm64_prepare_for_sleep: before its deep WFI every core unlocks its CoreSight external-debug block
 * (EDLAR = 0xc5acce55) and clears EDPRCR, so a debug power request does not hold the core domain up.
 * ADT coresight-reg: cpu 6c + n at 0x210010000 + 0x100000 (c * 0x1000000 + n) for c = 0..2, n = 0..5.
 */
#define T6050_CORESIGHT			0x210010000ULL
#define CORESIGHT_EDPRCR		0x310
#define CORESIGHT_EDLAR			0xfb0
#define CORESIGHT_KEY			0xc5acce55
#define S2R_NCPUS			18
static void __iomem *coresight[S2R_NCPUS];

static bool edprcr_clear;
module_param(edprcr_clear, bool, 0644);
MODULE_PARM_DESC(edprcr_clear, "clear every core's EDPRCR before the commit, as arm64_prepare_for_sleep does");

static void s2r_edprcr_clear(void)
{
	unsigned int i, held = 0;

	for (i = 0; i < S2R_NCPUS; i++) {
		if (!coresight[i])
			continue;
		writel_relaxed(CORESIGHT_KEY, coresight[i] + CORESIGHT_EDLAR);
		held += !!readl_relaxed(coresight[i] + CORESIGHT_EDPRCR);
		writel_relaxed(0, coresight[i] + CORESIGHT_EDPRCR);
	}
	/* f809: 0x40 + how many cores had a non-zero EDPRCR at the entry */
	s2r_pmu_write(PMU_PS_OFF, 0x40 + held);
}

/*
 * The TVM features (_enableFeature(f, 0) -> enableTVM, ADT soc-tvm, socgm-tvm, disp-tvm, dcs-tvm):
 * bit 0 of + 0x2a0 cleared, then wait for bit 1 to clear.
 */
static const u64 tvm_base[] = {
	0x280700000ULL, 0x280708000ULL, 0x2380700000ULL, 0x280710000ULL, 0x2380708000ULL, 0x2380710000ULL,
};
#define TVM_CTRL			0x2a0
#define TVM_ENABLE			BIT(0)
#define TVM_BUSY			BIT(1)
static void __iomem *tvm[ARRAY_SIZE(tvm_base)];

static bool tvm_off = true;
module_param(tvm_off, bool, 0644);
MODULE_PARM_DESC(tvm_off, "switch the TVM features off before the commit, as quiesceHW's _enableFeature does");

static void s2r_tvm_off(void)
{
	unsigned int i, on = 0, stuck = 0;
	u32 v;

	for (i = 0; i < ARRAY_SIZE(tvm); i++) {
		if (!tvm[i])
			continue;
		v = readl_relaxed(tvm[i] + TVM_CTRL);
		if (!(v & TVM_ENABLE))
			continue;
		on |= BIT(i);
		writel_relaxed(v & ~TVM_ENABLE, tvm[i] + TVM_CTRL);
		if (readl_poll_timeout_atomic(tvm[i] + TVM_CTRL, v, !(v & TVM_BUSY), 1, 192000))
			stuck |= BIT(i);
	}
	/* f80c: bits 0-5 = the features that were on, bit 7 = one did not settle */
	s2r_pmu_write(PMU_PMP_OFF, on | (stuck ? 0x80 : 0));
}

static bool afr_off = true;
module_param(afr_off, bool, 0644);
MODULE_PARM_DESC(afr_off, "power AFR off before the KeepAlive clear, as quiesceHW does");

static void __iomem *wdt_x[ARRAY_SIZE(wdt_extra)], *soc_cfg;

/* Each outcome goes to a PMU byte that survives the reset (AFR: f809, GFX: f80c). */
static void s2r_afr_off(void)
{
	u32 v = readl_relaxed(soc_cfg);
	u8 res = 0x90;

	if (FIELD_GET(PMGR_PS_TARGET, v)) {
		writel_relaxed((v & ~SOC_CFG_FIELD) | FIELD_PREP(SOC_CFG_FIELD, 4), soc_cfg);
		if (readl_poll_timeout_atomic(soc_cfg, v, FIELD_GET(SOC_CFG_FIELD, v) == 4 &&
					      FIELD_GET(PMGR_PS_ACTUAL, v) == 4, 1, 192000))
			res = 0x9d;
		else {
			writel_relaxed(0, soc_cfg);
			res = readl_poll_timeout_atomic(soc_cfg, v, !FIELD_GET(PMGR_PS_ACTUAL, v), 1, 192000) ?
			      0x9e : 0x9c;
		}
	}
	s2r_pmu_write(PMU_PS_OFF, res);
}

/* GFX (the GPU, PMP-tracked) at 0x2300e80110: off at the commit on macOS. */
static bool gfx_off = true;
module_param(gfx_off, bool, 0644);
MODULE_PARM_DESC(gfx_off, "power GFX off after AFR at the entry");

static void s2r_gfx_off(void)
{
	void __iomem *gfx = soc_cfg + 0x10;
	u32 v = readl_relaxed(gfx);
	u8 res = 0xc8;

	if (FIELD_GET(PMGR_PS_ACTUAL, v)) {
		s2r_mark(0x9f);
		writel_relaxed(v & ~(PMGR_PS_TARGET | PMGR_PS_AUTO_ENABLE), gfx);
		res = readl_poll_timeout_atomic(gfx, v, !FIELD_GET(PMGR_PS_ACTUAL, v), 1, 192000) ? 0xcd : 0xc0;
	}
	s2r_pmu_write(PMU_PMP_OFF, res);
}

/*
 * AppleT6050PMGR::quiesceHW's _enableKeepAlive(false): clear bit 0 of every KeepAlive register (27 at
 * 0x2807ec000 + 0x44 k, 25 at 0x280950000 + 0x24 k, 14 at 0x23807dc000 + 0x44 k) before the commit.
 */
static const struct { u64 base; u32 stride, n; } keepalive_blocks[] = {
	{ 0x2807ec000ULL, 0x44, 27 }, { 0x280950000ULL, 0x24, 25 }, { 0x23807dc000ULL, 0x44, 14 },
};
#define KEEPALIVE_ON			BIT(0)
static void __iomem *keepalive_map[ARRAY_SIZE(keepalive_blocks)];

static bool keepalive_off = true;
module_param(keepalive_off, bool, 0644);
MODULE_PARM_DESC(keepalive_off, "clear the PMGR KeepAlive bits before the commit, as quiesceHW does");

static unsigned int s2r_keepalive_count(void)
{
	unsigned int b, k, n = 0;

	for (b = 0; b < ARRAY_SIZE(keepalive_blocks); b++)
		for (k = 0; k < keepalive_blocks[b].n && keepalive_map[b]; k++)
			n += !!(readl_relaxed(keepalive_map[b] + keepalive_blocks[b].stride * k) & KEEPALIVE_ON);
	return n;
}

static void s2r_keepalive_clear(void)
{
	void __iomem *r;
	unsigned int b, k;

	for (b = 0; b < ARRAY_SIZE(keepalive_blocks); b++)
		for (k = 0; k < keepalive_blocks[b].n && keepalive_map[b]; k++) {
			r = keepalive_map[b] + keepalive_blocks[b].stride * k;
			writel_relaxed(readl_relaxed(r) & ~KEEPALIVE_ON, r);
		}
}

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

/*
 * idle_iops: the coprocessors macOS idles over RTKit before the cores go off, in its order (traced
 * under aura, kernel/S2R-MACOS-TRACE.md): AP power IDLE (0x201), then for the ones it stops, IOP power
 * IDLE (0x201) and the ASC CPU stopped. Skipped when the coprocessor's power state is off or its CPU
 * is not running. Marker 0xc0 + i on entry to coprocessor i, 0xc8 + i when one is skipped, 0xd0 + i
 * when a step timed out (that coprocessor is left as it is).
 */
static bool idle_iops = true;
module_param(idle_iops, bool, 0644);
MODULE_PARM_DESC(idle_iops, "idle MTP, GFX, GFX1, AOP2, ANS, SIO and the PMP over RTKit as macOS does, before the power states go off");

#define RTKIT_PWR_IDLE			0x201

static const struct {
	u64 asc;
	const char *ps;	/* power-state label, NULL for an always-on one */
	bool stop;	/* IOP power IDLE and the CPU stopped, as well as AP power IDLE */
} s2r_iops[] = {
	{ 0x294600000ULL, NULL, false },	/* mtp */
	{ 0x2302600000ULL, "gfx", true },	/* gfx-asc */
	{ 0x2302e00000ULL, "gfx", true },	/* gfx1-asc */
	{ 0x291600000ULL, NULL, false },	/* aop2 */
	{ 0x419600000ULL, "ans", true },	/* ans */
	{ 0x505e00000ULL, "sio_cpu", true },	/* sio */
	{ 0x284e00000ULL, "pmp", true },	/* pmp0 */
};
static void __iomem *s2r_iop_asc[ARRAY_SIZE(s2r_iops)], *s2r_iop_ps[ARRAY_SIZE(s2r_iops)];

static void __iomem *__init s2r_ps_by_label(const char *want)
{
	struct device_node *np, *parent;
	struct resource r;
	const char *label;
	void __iomem *m = NULL;
	u32 off;

	for_each_compatible_node(np, NULL, "apple,t8103-pmgr-pwrstate") {
		if (of_property_read_string(np, "label", &label) || strcmp(label, want))
			continue;
		parent = of_get_parent(np);
		if (!of_address_to_resource(parent, 0, &r) && !of_property_read_u32(np, "reg", &off))
			m = ioremap_np(r.start + off, 4);
		of_node_put(parent);
		of_node_put(np);
		break;
	}
	return m;
}

static void __init s2r_map_iops(void)
{
	int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(s2r_iops); i++) {
		s2r_iop_asc[i] = ioremap_np(s2r_iops[i].asc, SZ_64K);
		if (s2r_iops[i].ps)
			s2r_iop_ps[i] = s2r_ps_by_label(s2r_iops[i].ps);
		n += s2r_iop_asc[i] && (!s2r_iops[i].ps || s2r_iop_ps[i]);
	}
	pr_info("idle_iops: %d of %zu coprocessors mapped\n", n, ARRAY_SIZE(s2r_iops));
}

/* Per coprocessor, bit i: f804 idled (and stopped where macOS stops it), f805 skipped, f806 timed out. */
static void s2r_idle_iops(int from, int to)
{
	void __iomem *asc;
	u8 done = 0, skipped = 0, failed = 0;
	u32 v;
	int i;

	for (i = from; i < to; i++) {
		asc = s2r_iop_asc[i];
		s2r_mark(0xc0 + i);
		if (!asc || (s2r_iops[i].ps && (!s2r_iop_ps[i] ||
		    !FIELD_GET(PMGR_PS_ACTUAL, readl_relaxed(s2r_iop_ps[i])))) ||
		    !(readl_relaxed(asc + ASC_CPU_CONTROL) & ASC_CPU_CONTROL_RUN)) {
			s2r_mark(0xc8 + i);
			skipped |= BIT(i);
			continue;
		}
		if (!s2r_rtkit_xfer(asc, FIELD_PREP(RTKIT_MGMT_TYPE, RTKIT_SET_AP_PWR) | RTKIT_PWR_IDLE,
				    RTKIT_SET_AP_PWR, RTKIT_PWR_IDLE)) {
			s2r_mark(0xd0 + i);
			failed |= BIT(i);
			continue;
		}
		if (s2r_iops[i].stop) {
			if (!s2r_rtkit_xfer(asc, FIELD_PREP(RTKIT_MGMT_TYPE, RTKIT_SET_IOP_PWR) | RTKIT_PWR_IDLE,
					    RTKIT_IOP_PWR_ACK, RTKIT_PWR_IDLE)) {
				s2r_mark(0xd0 + i);
				failed |= BIT(i);
				continue;
			}
			writel_relaxed(readl_relaxed(asc + ASC_CPU_CONTROL) & ~ASC_CPU_CONTROL_RUN,
				       asc + ASC_CPU_CONTROL);
			if (readl_poll_timeout_atomic(asc + ASC_CPU_STATUS, v, v & ASC_CPU_STATUS_STOPPED,
						      1, 100000)) {
				s2r_mark(0xd0 + i);
				failed |= BIT(i);
				continue;
			}
		}
		done |= BIT(i);
	}
	if (from == 0) {
		s2r_pmu_write(0xf804, done);
		s2r_pmu_write(0xf805, skipped);
		s2r_pmu_write(0xf806, failed);
	} else {
		s2r_pmu_write(0xf80f, done | skipped << 4 | failed << 6);
	}
}

/*
 * sep_sleep: macOS's message to the SEP before the sleep (0x20c0800 on endpoint 0, answered within
 * about 30 ms). Linux never talks to the SEP otherwise. f807: 0x01 answered, 0x80 no answer within
 * 200 ms, 0x40 its mailbox full, 0x20 not mapped.
 * spmi_sleep: then 0x2000001 into +0x420 of the five SPMI controllers macOS writes it to.
 */
static bool sep_sleep;
module_param(sep_sleep, bool, 0644);
MODULE_PARM_DESC(sep_sleep, "send the SEP macOS's sleep message at the entry");
static bool spmi_sleep = true;
module_param(spmi_sleep, bool, 0644);
MODULE_PARM_DESC(spmi_sleep, "write 0x2000001 to +0x420 of the five SPMI controllers, as macOS does");

#define T6050_SEP_ASC			0x502600000ULL
static const u64 s2r_spmi_sleep_regs[] = {
	0x28a1a8420ULL, 0x28a094420ULL, 0x28a0d4420ULL, 0x290914420ULL, 0x28a188420ULL,
};
static void __iomem *sep_asc, *s2r_spmi_sleep_map[ARRAY_SIZE(s2r_spmi_sleep_regs)];

static void __init s2r_map_sep_spmi(void)
{
	int i;

	sep_asc = ioremap_np(T6050_SEP_ASC, SZ_64K);
	for (i = 0; i < ARRAY_SIZE(s2r_spmi_sleep_regs); i++)
		s2r_spmi_sleep_map[i] = ioremap_np(s2r_spmi_sleep_regs[i], 4);
}

static void s2r_sep_spmi(void)
{
	void __iomem *mb;
	u8 res = 0x20;
	u32 v;
	int i;

	if (sep_sleep && sep_asc) {
		mb = sep_asc + ASC_MBOX;
		res = 0x40;
		if (!readl_poll_timeout_atomic(mb + MBOX_A2I_CONTROL, v, !(v & MBOX_A2I_FULL), 1, 100000)) {
			writeq_relaxed(0x20c0800, mb + MBOX_A2I_SEND0);
			writeq_relaxed(0, mb + MBOX_A2I_SEND1);
			res = 0x80;
			if (!readl_poll_timeout_atomic(mb + MBOX_I2A_CONTROL, v, !(v & MBOX_I2A_EMPTY), 1, 200000)) {
				readq_relaxed(mb + MBOX_I2A_RECV0);
				readq_relaxed(mb + MBOX_I2A_RECV1);
				res = 0x01;
			}
		}
		s2r_pmu_write(0xf807, res);
	}
	for (i = 0; i < ARRAY_SIZE(s2r_spmi_sleep_map) && spmi_sleep; i++)
		if (s2r_spmi_sleep_map[i])
			writel_relaxed(0x2000001, s2r_spmi_sleep_map[i]);
}

/*
 * hpm_sleep: macOS's SPMI command words to the USB-C port controllers before the sleep, replayed in
 * order (SSPS 3, USBw 9 on slave 10, then SPMI SLEEP), each command's reply drained. f807: 0x01 every
 * command answered, else 0x80 | the number left unanswered (at most 0x7f).
 */
static bool hpm_sleep = true;
module_param(hpm_sleep, bool, 0644);
MODULE_PARM_DESC(hpm_sleep, "put the USB-C port controllers to sleep over SPMI as macOS does");

static const u64 s2r_hpm_ctl_base[] = { 0x28a188000ULL, 0x28a1a8000ULL };
static void __iomem *s2r_hpm_ctl[ARRAY_SIZE(s2r_hpm_ctl_base)];

static void s2r_hpm_sleep(void)
{
	void __iomem *c;
	unsigned int missed = 0;
	u32 v;
	int i, k;

	if (!s2r_hpm_ctl[0] || !s2r_hpm_ctl[1])
		return;
	for (i = 0; i < ARRAY_SIZE(s2r_hpm_words); i++) {
		if (s2r_hpm_words[i].ndata < 0)
			continue;
		c = s2r_hpm_ctl[s2r_hpm_words[i].ctl];
		readl_poll_timeout_atomic(c + SPMI_STATUS, v, !(v & BIT(15)), 1, 10000);
		writel_relaxed(s2r_hpm_words[i].word, c + SPMI_CMD);
		for (k = 1; k <= s2r_hpm_words[i].ndata && i + k < ARRAY_SIZE(s2r_hpm_words); k++)
			writel_relaxed(s2r_hpm_words[i + k].word, c + SPMI_CMD);
		if (readl_poll_timeout_atomic(c + SPMI_STATUS, v, !(v & SPMI_RX_EMPTY), 1, 20000)) {
			missed++;
		} else {
			udelay(20);
			for (k = 0; k < 16 && !(readl_relaxed(c + SPMI_STATUS) & SPMI_RX_EMPTY); k++)
				readl_relaxed(c + SPMI_REPLY);
		}
		udelay(500);
	}
	s2r_pmu_write(0xf807, missed ? 0x80 | min(missed, 0x7fU) : 0x01);
}

/*
 * go_hib: after the S2R timer start, the PMU's RTC alarm armed the way XNU does it (T8132, traced by
 * aura: read 0x2000, the compare at 0x2008, 0x200e <- 0, 0x2000 <- 0x41, "go hib"). The J714c PMU
 * (nub-spmi0 slave 0xe) has the same registers in its ADT node (info-rtc_alarm_ctrl 0x2000, -offset
 * 0x2008, info-rtc_irq_mask_offset 0x200e, en mask 0x40, monitor 0x1, event 0x2, info-rtc 0x2002).
 * The alarm is set wake_s seconds ahead. f807: 0x01 armed, 0x80 an SPMI read failed.
 */
static bool go_hib = true;
module_param(go_hib, bool, 0644);
MODULE_PARM_DESC(go_hib, "arm the PMU RTC alarm after the S2R timer start, as XNU's go hib");
static unsigned int wake_s = 30;
module_param(wake_s, uint, 0644);
MODULE_PARM_DESC(wake_s, "seconds until the PMU RTC alarm that go_hib arms");

#define SPMI_EXT_READL			0x38
#define PMU_RTC				0x2002
#define PMU_RTC_ALARM_CTRL		0x2000
#define PMU_RTC_ALARM			0x2008
#define PMU_RTC_IRQ_MASK		0x200e
#define RTC_ALARM_EN			0x40
#define RTC_ALARM_MONITOR		0x01
#define RTC_ALARM_EVENT			0x02
#define RTC_ALARM_IRQ			0x01

static bool s2r_pmu_read(u16 addr, u8 *buf, int len)
{
	u32 v, w;
	int i;

	writel_relaxed((SPMI_EXT_READL | (len - 1)) | PMU_USID << 8 | BIT(15) | (u32)addr << 16,
		       spmi + SPMI_CMD);
	if (readl_poll_timeout_atomic(spmi + SPMI_STATUS, v, !(v & SPMI_RX_EMPTY), 1, 50000))
		return false;
	readl_relaxed(spmi + SPMI_REPLY);	/* the reply frame */
	for (i = 0; i < len; i += 4) {
		if (readl_poll_timeout_atomic(spmi + SPMI_STATUS, v, !(v & SPMI_RX_EMPTY), 1, 50000))
			return false;
		w = readl_relaxed(spmi + SPMI_REPLY);
		memcpy(buf + i, &w, min(4, len - i));
	}
	return true;
}

static void s2r_pmu_write4(u16 addr, u32 val)
{
	u32 v;

	writel_relaxed((SPMI_EXT_WRITEL | 3) | PMU_USID << 8 | BIT(15) | (u32)addr << 16, spmi + SPMI_CMD);
	writel_relaxed(val, spmi + SPMI_CMD);
	if (!readl_poll_timeout_atomic(spmi + SPMI_STATUS, v, !(v & SPMI_RX_EMPTY), 1, 50000))
		readl_relaxed(spmi + SPMI_REPLY);
}

static void s2r_go_hib(void)
{
	u8 c[8] = {}, b;
	u64 cnt;
	u32 at;

	if (!s2r_pmu_read(PMU_RTC, c, 6) || !s2r_pmu_read(PMU_RTC_ALARM_CTRL, &b, 1)) {
		s2r_pmu_write(0xf807, 0x80);
		return;
	}
	memcpy(&cnt, c, sizeof(cnt));
	at = (cnt >> 16) + wake_s;
	s2r_pmu_write(PMU_RTC_ALARM_CTRL, b | RTC_ALARM_EVENT);
	s2r_pmu_write4(PMU_RTC_ALARM, at);
	if (!s2r_pmu_read(PMU_RTC_IRQ_MASK, &b, 1)) {
		s2r_pmu_write(0xf807, 0x80);
		return;
	}
	s2r_pmu_write(PMU_RTC_IRQ_MASK, b & ~RTC_ALARM_IRQ);
	if (!s2r_pmu_read(PMU_RTC_ALARM_CTRL, &b, 1)) {
		s2r_pmu_write(0xf807, 0x80);
		return;
	}
	s2r_pmu_write(PMU_RTC_ALARM_CTRL, (b & ~RTC_ALARM_EVENT) | RTC_ALARM_EN | RTC_ALARM_MONITOR);
	s2r_pmu_write(0xf807, 0x01);
}

/*
 * cpu_stop_bits: the PMGR CPU-start block's stop request (pmgr reg[0] + 0x88000, one bit per CPU
 * index, as m1n1 smp_stop_cpu() writes before a core's deep WFI). macOS writes every core's bit before
 * its sleep commit, the boot core's last. The parked secondaries' bits first, then the boot core's.
 */
static bool cpu_stop_bits = true;
module_param(cpu_stop_bits, bool, 0644);
MODULE_PARM_DESC(cpu_stop_bits, "write each core's PMGR stop request before the sleep commit, as macOS and m1n1 do");

#define T6050_CPU_STOP			0x280688000ULL
static void __iomem *cpu_stop;

static void s2r_cpu_stop_bits(void)
{
	int i;

	for (i = 1; i < 18; i++)
		writel_relaxed(BIT(i), cpu_stop);
	writel_relaxed(BIT(0), cpu_stop);
	dsb(sy);
}

/*
 * ps_like_macos: every power state to macOS's value at its S2R timer start (the domains it keeps on are
 * in AUTO_ENABLE, bit 28, for the hardware to take down; the display chain and the ATC CIOs off), in
 * place of ps_off and pmp_ps_off.
 */
static bool ps_like_macos = true;
static bool ps_disp;
module_param(ps_disp, bool, 0644);
MODULE_PARM_DESC(ps_disp, "ps_like_macos: also switch the main display chain off (stopped the CPU in s2r62c-f)");
module_param(ps_like_macos, bool, 0644);
MODULE_PARM_DESC(ps_like_macos, "set every power state to macOS's value at the sleep commit");
static void __iomem *macos_ps_map[ARRAY_SIZE(macos_ps)];

static void __init s2r_map_macos_ps(void)
{
	int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(macos_ps); i++)
		n += !!(macos_ps_map[i] = ioremap_np(macos_ps[i].pa, 4));
	pr_info("ps_like_macos: %d of %zu power states mapped\n", n, ARRAY_SIZE(macos_ps));
}

/*
 * dcps_like_macos: the five display coprocessors stopped as macOS does before their chains go off
 * (traced under aura): AP power IDLE over RTKit, the ASC CPU stopped, then six DART writes (0x4060 <-
 * ~0, 0x4000 <- 0, 0x4204 <- ~0, 0xc20 <- 0x800000 then 0x1000000, 0x80 <- 0xa000). The result goes
 * to f806: bit i set where DCP i was stopped, 0x80 when one timed out.
 */
static bool dcps_like_macos = true;
static bool disp_main_off;
module_param(disp_main_off, bool, 0644);
MODULE_PARM_DESC(disp_main_off, "dcps_like_macos: also take the main display chain down (stopped the CPU in s2r64c)");
module_param(dcps_like_macos, bool, 0644);
MODULE_PARM_DESC(dcps_like_macos, "stop the five DCPs and their DARTs as macOS does, then switch their chains off");

static const struct { u64 asc, dart, ps; } s2r_dcps[] = {
	{ 0x48ae00000ULL, 0x48a340000ULL, 0x2809001a8ULL },	/* dcp, disp_cpu */
	{ 0x2b2e00000ULL, 0x2b2340000ULL, 0x280600278ULL },	/* dcpext0 */
	{ 0x392e00000ULL, 0x392340000ULL, 0x2806002e0ULL },	/* dcpext1 */
	{ 0x302e00000ULL, 0x302340000ULL, 0x2806002b0ULL },	/* dcpext2 */
	{ 0x306e00000ULL, 0x306340000ULL, 0x2806002c8ULL },	/* dcpext3 */
};
static void __iomem *s2r_dcp_asc[5], *s2r_dcp_dart[5], *s2r_dcp_ps[5];

/* Each chain as macOS takes it down right after its DCP and DART: cpu twice, fe, sys (adma untouched). */
static const struct { u64 pa; u32 val; } s2r_disp_off[5][4] = {
	{ { 0x2809001a8ULL, 0x0f0000ff }, { 0x2809001a8ULL, 0x0f0003f0 },
	  { 0x280900198ULL, 0x000003f0 }, { 0x280900180ULL, 0x000003f0 } },
	{ { 0x280600278ULL, 0x0f0400ff }, { 0x280600278ULL, 0x0f0403f0 },
	  { 0x280600268ULL, 0x000003f0 }, { 0x280600200ULL, 0x000003f0 } },
	{ { 0x2806002e0ULL, 0x0f0400ff }, { 0x2806002e0ULL, 0x0f0403f0 },
	  { 0x2806002d0ULL, 0x000003f0 }, { 0x280600230ULL, 0x000003f0 } },
	{ { 0x2806002b0ULL, 0x0f0400ff }, { 0x2806002b0ULL, 0x0f0403f0 },
	  { 0x2806002a0ULL, 0x000003f0 }, { 0x280600220ULL, 0x000003f0 } },
	{ { 0x2806002c8ULL, 0x0f0400ff }, { 0x2806002c8ULL, 0x0f0403f0 },
	  { 0x2806002b8ULL, 0x000003f0 }, { 0x280600228ULL, 0x000003f0 } },
};
static void __iomem *s2r_disp_off_map[5][4];

static void __init s2r_map_dcps(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s2r_dcps); i++) {
		s2r_dcp_asc[i] = ioremap_np(s2r_dcps[i].asc, SZ_64K);
		s2r_dcp_dart[i] = ioremap_np(s2r_dcps[i].dart, SZ_32K);
		s2r_dcp_ps[i] = ioremap_np(s2r_dcps[i].ps, 4);
		for (int k = 0; k < 4; k++)
			s2r_disp_off_map[i][k] = ioremap_np(s2r_disp_off[i][k].pa, 4);
	}
}

/*
 * Returns true when every DCP that was on is stopped. f806: bit i DCP i stopped, f805: bit i the AP
 * power message to DCP i went unanswered, bit 4 + i its CPU did not stop (f805 holds idle_iops' skips
 * until here).
 */
static bool s2r_stop_dcps(void)
{
	void __iomem *asc, *d;
	u8 done = 0, bad = 0;
	u32 v;
	int i;

	for (i = 0; i < ARRAY_SIZE(s2r_dcps); i++) {
		asc = s2r_dcp_asc[i];
		d = s2r_dcp_dart[i];
		if (!asc || !d || !s2r_dcp_ps[i] ||
		    !FIELD_GET(PMGR_PS_ACTUAL, readl_relaxed(s2r_dcp_ps[i])))
			continue;
		if (i == 0 && (readl_relaxed(asc + ASC_CPU_CONTROL) & ASC_CPU_CONTROL_RUN)) {
			/* The main DCP, already put down by Linux's DCP driver, answers only the
			 * QUIESCED/SLEEP sequence (s2r_rtkit_sleep, which worked in s2r62e). */
			if (!s2r_rtkit_sleep(asc, 0x80)) {
				bad |= BIT(0) | BIT(4);
				continue;
			}
		} else if (readl_relaxed(asc + ASC_CPU_CONTROL) & ASC_CPU_CONTROL_RUN) {
			if (!s2r_rtkit_xfer(asc, FIELD_PREP(RTKIT_MGMT_TYPE, RTKIT_SET_AP_PWR) |
					    RTKIT_PWR_IDLE, RTKIT_SET_AP_PWR, RTKIT_PWR_IDLE))
				bad |= BIT(i);
			/* Then IOP power IDLE, acknowledged, as macOS sends it before the CPU stop. */
			if (!s2r_rtkit_xfer(asc, FIELD_PREP(RTKIT_MGMT_TYPE, RTKIT_SET_IOP_PWR) |
					    RTKIT_PWR_IDLE, RTKIT_IOP_PWR_ACK, RTKIT_PWR_IDLE))
				bad |= BIT(i);
			/* macOS writes CPU_CONTROL to 0, twice; clearing RUN alone did not stop them (s2r63b). */
			writel_relaxed(0, asc + ASC_CPU_CONTROL);
			writel_relaxed(0, asc + ASC_CPU_CONTROL);
			if (readl_poll_timeout_atomic(asc + ASC_CPU_STATUS, v, v & ASC_CPU_STATUS_STOPPED,
						      1, 100000))
				bad |= BIT(4 + min(i, 3));
		}
		s2r_mark(0x30 + 8 * i);		/* DCP i: 0x30 + 8i DART, +1..4 the chain writes, +7 done */
		writel_relaxed(0xffffffff, d + 0x4060);
		writel_relaxed(0, d + 0x4000);
		writel_relaxed(0xffffffff, d + 0x4204);
		writel_relaxed(0x800000, d + 0xc20);
		writel_relaxed(0x1000000, d + 0xc20);
		writel_relaxed(0xa000, d + 0x80);
		dsb(sy);
		for (int k = 0; k < 4 && (i || disp_main_off); k++)
			if (s2r_disp_off_map[i][k]) {
				s2r_mark(0x31 + 8 * i + k);
				writel_relaxed(s2r_disp_off[i][k].val, s2r_disp_off_map[i][k]);
				dsb(sy);
			}
		s2r_mark(0x37 + 8 * i);
		done |= BIT(i);
	}
	s2r_pmu_write(0xf806, done);
	s2r_pmu_write(0xf805, bad);
	return !(bad & 0xf0);
}

/* Every display chain (disp*, dispext*): switching one off under its running DCP stopped the CPU. */
static const u64 s2r_disp_ps[] = { 0x280600200ULL, 0x280600220ULL, 0x280600228ULL, 0x280600230ULL, 0x280600268ULL, 0x280600270ULL, 0x280600278ULL, 0x2806002a0ULL, 0x2806002a8ULL, 0x2806002b0ULL, 0x2806002b8ULL, 0x2806002c0ULL, 0x2806002c8ULL, 0x2806002d0ULL, 0x2806002d8ULL, 0x2806002e0ULL, 0x280900180ULL, 0x280900198ULL, 0x2809001a0ULL, 0x2809001a8ULL };

static bool s2r_disp_chain(u64 pa)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s2r_disp_ps); i++)
		if (s2r_disp_ps[i] == pa)
			return true;
	return false;
}

static void s2r_ps_like_macos(bool dcp_ok)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(macos_ps); i++)
		if (macos_ps_map[i] && (dcp_ok || !s2r_disp_chain(macos_ps[i].pa))) {
			if (i >= 248)
				s2r_mark(0x40 + i - 248);	/* entry i = m - 0x40 + 248 */
			else if (!(i % 8))
				s2r_mark(0x10 + i / 8);	/* 0x10.. 0x2f: entries 8 * (m - 0x10) .. +7 */
			u32 m = PMGR_PS_TARGET | PMGR_PS_AUTO_ENABLE;
			u32 cur = readl_relaxed(macos_ps_map[i]);
			u32 want = (cur & ~m) | (macos_ps[i].val & m);

			if (want == cur)
				continue;
			writel_relaxed(want, macos_ps_map[i]);
			if (macos_ps[i].val & 0xf)
				udelay(20);
		}
	dsb(sy);
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
	for (i = 0; i < ARRAY_SIZE(wdt_x) && wdt_extra_off; i++)
		if (wdt_x[i])
			writel_relaxed(0, wdt_x[i]);
	s2r_stop(1);

	/* All but the PMP (the last one): macOS stops the PMP after the display chains. */
	if (idle_iops)
		s2r_idle_iops(0, ARRAY_SIZE(s2r_iops) - 1);
	if (hpm_sleep)
		s2r_hpm_sleep();
	s2r_sep_spmi();

	if (perf_quiesce)
		s2r_quiesce_perf();

	if (ps_like_macos) {
		/* macOS stops the DCP before its display chain goes off; switching it off under a running
		 * DCP stopped the CPU (s2r62c). */
		if (dcps_like_macos)
			s2r_stop_dcps();
		s2r_ps_like_macos(false);	/* the display chains are done per DCP above */
		if (idle_iops)
			s2r_idle_iops(ARRAY_SIZE(s2r_iops) - 1, ARRAY_SIZE(s2r_iops));
		s2r_mark(0x99);
	} else if (ps_off) {
		s2r_quiesce_ps();
	}

	if (pmp_sleep && pmp_asc)
		s2r_pmu_write(PMU_PS_OFF, s2r_rtkit_sleep(pmp_asc, 0xb0) ? 0xb3 : 0xbf);

	if (park_clocks)
		s2r_quiesce_clocks();
	s2r_stop(2);

	apple_aic_mask_all();
	s2r_stop(3);

	if (afr_off && soc_cfg)
		s2r_afr_off();
	if (gfx_off && soc_cfg)
		s2r_gfx_off();
	if (pmc_floor_off && pmc_ap)
		s2r_pmc_quiesce();
	if (cpm_gating && cpm_pdg)
		s2r_cpm_gating();
	if (tvm_off) {
		s2r_mark(0xe1);
		s2r_tvm_off();
	}
	if (edprcr_clear) {
		s2r_mark(0xe2);
		s2r_edprcr_clear();
	}

	if (cluster_off && cluster_ps)
		s2r_cluster_off();

	if (keepalive_off) {
		s2r_keepalive_clear();
		s2r_mark(0x9b);
	}

	if (soc_cfg_pulse && soc_cfg) {
		u32 v = readl_relaxed(soc_cfg);

		writel_relaxed((v & ~SOC_CFG_FIELD) | FIELD_PREP(SOC_CFG_FIELD, 4), soc_cfg);
		writel_relaxed(v & ~SOC_CFG_FIELD, soc_cfg);
		s2r_mark(0x9a);
	}

	if (cpu_stop_bits && cpu_stop) {
		s2r_cpu_stop_bits();
		s2r_mark(0x9b);
	}

	if (replay_tail && macos_tail_map[0]) {
		s2r_mark(0x9c);
		s2r_macos_tail();
		s2r_mark(0x9d);
	}

	for (i = 0; i < 3 && s2r_timer_start; i++) {
		writel_relaxed(s2r_timer_val, s2r_timer);
		dsb(sy);
	}
	if (go_hib) {
		s2r_go_hib();
		s2r_mark(0x9e);
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
	int i;

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
	for (i = 0; i < ARRAY_SIZE(macos_tail_pages); i++) {
		macos_tail_map[i] = ioremap_np(macos_tail_pages[i], SZ_16K);
		if (!macos_tail_map[i]) {
			pr_err("cannot map 0x%llx for macos_tail\n", macos_tail_pages[i]);
			macos_tail_map[0] = NULL;
			break;
		}
	}
	pr_info("macos_tail: %zu writes on %zu pages%s\n", ARRAY_SIZE(macos_tail),
		ARRAY_SIZE(macos_tail_pages), macos_tail_map[0] ? "" : " (not mapped, off)");

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
	s2r_map_iops();
	s2r_map_sep_spmi();
	s2r_map_macos_ps();
	s2r_map_dcps();
	cpu_stop = ioremap_np(T6050_CPU_STOP, SZ_4K);
	s2r_hpm_ctl[0] = ioremap_np(s2r_hpm_ctl_base[0], SZ_4K);
	s2r_hpm_ctl[1] = ioremap_np(s2r_hpm_ctl_base[1], SZ_4K);
	soc_cfg = ioremap_np(T6050_SOC_CFG, 0x20);
	for (i = 0; i < ARRAY_SIZE(keepalive_blocks); i++)
		keepalive_map[i] = ioremap_np(keepalive_blocks[i].base, SZ_4K);
	cluster_ps = ioremap_np(T6050_CLUSTER_PS, 0x80);
	pmc_ap = ioremap_np(T6050_PMC_AP_FLOOR, 8);
	cpm_pdg = ioremap_np(T6050_CPM_PDG, 0x18);
	for (i = 0; i < ARRAY_SIZE(tvm_base); i++)
		tvm[i] = ioremap_np(tvm_base[i], SZ_4K);
	for (i = 0; i < S2R_NCPUS; i++)
		coresight[i] = ioremap_np(T6050_CORESIGHT + 0x1000000ULL * (i / 6) + 0x100000ULL * (i % 6), SZ_4K);
	if (cpm_pdg)
		pr_info("CPM power-domain gating %08x %08x %08x\n", readl(cpm_pdg), readl(cpm_pdg + 8),
			readl(cpm_pdg + 16));
	if (pmc_ap)
		pr_info("PMC AP floor %08x pending %08x\n", readl(pmc_ap), readl(pmc_ap + PMC_AP_PENDING));
	for (i = 0; cluster_ps && i < 0x20; i += 8)
		pr_info("cluster PS %02x: %08x %08x %08x %08x %08x %08x %08x %08x\n", i,
			readl(cluster_ps + 4 * i), readl(cluster_ps + 4 * i + 4), readl(cluster_ps + 4 * i + 8),
			readl(cluster_ps + 4 * i + 12), readl(cluster_ps + 4 * i + 16), readl(cluster_ps + 4 * i + 20),
			readl(cluster_ps + 4 * i + 24), readl(cluster_ps + 4 * i + 28));
	pr_info("KeepAlive bits set: %u of 66\n", s2r_keepalive_count());
	{
		void __iomem *gfx = ioremap_np(T6050_SOC_CFG + 0x10, 4);

		pr_info("AFR %08x GFX %08x\n", soc_cfg ? readl(soc_cfg) : ~0U, gfx ? readl(gfx) : ~0U);
		if (gfx)
			iounmap(gfx);
	}
	for (i = 0; i < ARRAY_SIZE(wdt_extra); i++)
		wdt_x[i] = ioremap_np(wdt_extra[i], 4);
	pr_info("wdt extra windows %08x %08x %08x %08x; SOC cfg %08x\n",
		wdt_x[0] ? readl(wdt_x[0]) : ~0U, wdt_x[1] ? readl(wdt_x[1]) : ~0U,
		wdt_x[2] ? readl(wdt_x[2]) : ~0U, wdt_x[3] ? readl(wdt_x[3]) : ~0U,
		soc_cfg ? readl(soc_cfg) : ~0U);
	pr_info("suspend-to-RAM registered (first stage: the wake is a cold boot)\n");
	return 0;
}
late_initcall(apple_s2r_init);

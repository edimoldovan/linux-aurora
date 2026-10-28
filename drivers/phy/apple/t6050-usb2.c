// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple T6050 (M5 Pro/Max) Type-C port, USB 2.0 host only.
 *
 * The T6050 Type-C PHY is a new generation; its SuperSpeed and CIO parts are
 * not driven here. USB 2.0 needs only the port's usb2phy block, the pipe
 * handler and the dwc3 global registers. phy_init() puts the port into
 * USB 2.0 host mode with the sequence the loader (m1n1 usb_t6050_host_prep)
 * uses on J714c, so the port works whatever state the loader left it in.
 * The xHCI registers are then driven by the generic xHCI platform driver,
 * which calls phy_init() before its host controller reset.
 *
 * A port whose node has the "atc-power" window also gets SuperSpeed: after the
 * USB 2.0 flip, phy_init() runs the port 1 SuperSpeed host sequence measured
 * over the m1n1 proxy (kernel/USB-BRINGUP.md), steps 1-16, including the boot
 * of the PHY CPU's RTKit firmware. The xHCI reset of the platform driver is
 * step 17 (phy_calibrate() restores the glue after it), and the hub's USB 2.0
 * port reset is step 18. Normal plug orientation only.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/soc/apple/tunable.h>
#include <linux/string.h>
#include <linux/usb/tipd.h>
#include <linux/workqueue.h>

#define USB2PHY_USBCTL			0x00
#define USB2PHY_USBCTL_RUN		2
#define USB2PHY_CTL			0x04
#define USB2PHY_CTL_RESET		BIT(0)
#define USB2PHY_CTL_PORT_RESET		BIT(1)
#define USB2PHY_SIG			0x08
#define USB2PHY_SIG_DEFAULT		0x01c1000f
#define USB2PHY_SIG_HOST		(7 << 12)
#define USB2PHY_MISCTUNE		0x1c
#define USB2PHY_MISCTUNE_DEFAULT	0x008c0813

#define PIPEHANDLER_OVERRIDE		0x00
#define PIPEHANDLER_OVERRIDE_VALUES	0x04
#define PIPEHANDLER_MUX_CTRL		0x0c
#define PIPEHANDLER_MUX_CTRL_DUMMY	0x22
#define PIPEHANDLER_AON_GEN		0x1c
#define PIPEHANDLER_AON_GEN_DWC3_RESET_N BIT(0)
#define PIPEHANDLER_NONSELECTED_OVERRIDE 0x20
#define PIPEHANDLER_NONSELECTED_DEFAULT	0x9332

#define DWC3_GCTL			0xc110
#define DWC3_GCTL_CORESOFTRESET		BIT(11)
#define DWC3_GCTL_PRTCAPDIR_MASK	(3 << 12)
#define DWC3_GCTL_PRTCAP_HOST		(1 << 12)
#define DWC3_GUSB2PHYCFG0		0xc200
#define DWC3_GUSB2PHYCFG_SUSPHY		BIT(6)
#define DWC3_GUSB3PIPECTL0		0xc2c0
#define DWC3_GUSB3PIPECTL_SUSPHY	BIT(17)
#define DWC3_DCTL			0xc704
#define DWC3_DCTL_RUN_STOP		BIT(31)
#define DWC3_DSTS			0xc70c
#define DWC3_DSTS_DEVCTRLHLT		BIT(22)
#define DWC3_GDBGLTSSM			0xc164

/* xHCI CAPLENGTH is 0x20 on these controllers; PORTSC of port 1 at op + 0x400. */
#define XHCI_USBCMD			0x20
#define XHCI_USBCMD_HCRST		BIT(1)
#define XHCI_USBSTS			0x24
#define XHCI_USBSTS_CNR			BIT(11)
#define XHCI_PORTSC1			0x420
#define XHCI_PORTSC2			0x430
#define XHCI_PORTSC_CCS			BIT(0)
#define XHCI_PORTSC_PR			BIT(4)

/* SuperSpeed: pipe handler, AUSBC power control, Type-C PHY ("atc") windows. */
#define PIPEHANDLER_LOCK_REQ		0x10
#define PIPEHANDLER_LOCK_ACK		0x14
#define PIPEHANDLER_LOCK		BIT(0)
#define PIPEHANDLER_MUX_CLK		GENMASK(6, 4)
#define PIPEHANDLER_MUX_DATA		GENMASK(1, 0)

#define PCTL_TXDETRX_REQ		0x18
#define PCTL_TXDETRX_ACK		0x1c

#define ATC_CFG0			0x0c
#define ATC_RX_PWR			0x10
#define ATC_LANE_MODE			0x6c
#define ATC_XBAR			0x78
#define ATC_TX_PWR			0x120
#define ATC_UC_CTRL			0x260
#define ATC_UC_TNBL_DONE		BIT(10)
#define ATC_BIST_CFG1			0x29004
#define ATC_BIST_CFG1_CLOCK_EN		BIT(0)
#define ATC_BIST_CFG1_USB_EN		BIT(3)
#define ATC_BIST_STATUS2		0x29104
#define ATC_BIST_STATUS2_PHY_STATUS	BIT(8)
#define ATC_BIST_OV1			0x2910c
#define ATC_BIST_OV1_PWR_DOWN		GENMASK(28, 25)
#define ATC_BIST_OV1_PIPE_RESET_N	BIT(20)
#define ATC_BIST_OV2			0x29110

#define ATC_PWR_CTRL			0x0
#define ATC_PWR_STAT			0x4
#define ATC_PWR_MISC			0x8
#define ATC_PWR_SLEEP_SMALL		BIT(0)
#define ATC_PWR_SLEEP_BIG		BIT(1)
#define ATC_PWR_CLAMP			BIT(2)
#define ATC_PWR_APB_RESET_N		BIT(3)
#define ATC_PWR_PHY_RESET_N		BIT(4)
#define ATC_PWR_MISC_USB_LN_SWAP	BIT(1)

/* MxWrap of the PHY CPU, with its RTKit mailbox at +0x8000. */
#define MXW_CPU_CTRL			0x28
#define MXW_CPU_CTRL_RUN		0x10
#define MXW_IDLE_CTRL			0x40
#define MXW_A2I_CTRL			0x8110
#define MXW_I2A_CTRL			0x8114
#define MXW_MBOX_FULL			BIT(16)
#define MXW_MBOX_EMPTY			BIT(17)
#define MXW_A2I_MSG0			0x8800
#define MXW_A2I_MSG1			0x8808
#define MXW_I2A_MSG0			0x8830
#define MXW_I2A_MSG1			0x8838

#define RTK_TYPE			GENMASK_ULL(59, 52)
#define RTK_HELLO			1
#define RTK_HELLO_ACK			2
#define RTK_START_EP			5
#define RTK_IOP_PWR			6
#define RTK_IOP_PWR_ACK			7
#define RTK_EPMAP			8
#define RTK_AP_PWR			0xb
#define RTK_HELLO_MIN			GENMASK_ULL(15, 0)
#define RTK_HELLO_MAX			GENMASK_ULL(31, 16)
#define RTK_EPMAP_LAST			BIT_ULL(51)
#define RTK_EPMAP_BASE			GENMASK_ULL(34, 32)
#define RTK_EPMAP_BITMAP		GENMASK_ULL(31, 0)
#define RTK_EPMAP_MORE			BIT_ULL(0)
#define RTK_EP				GENMASK_ULL(39, 32)
#define RTK_START_EP_FLAG		2
#define RTK_PWR_STATE			GENMASK_ULL(15, 0)
#define RTK_PWR_ON			0x20

/* PMGR power states: target [3:0], actual [7:4]. */
#define PS_TARGET			GENMASK(3, 0)
#define PS_ACTUAL			GENMASK(7, 4)
#define PS_ON				0xf

/* atc1_common, auss, atc1_cio, atc1_cio_usb, atc1_cio_auss, parent first ("pmgr"). */
static const u32 t6050_cio_domains[] = { 0x210, 0x248, 0x290, 0x358, 0x360 };
/* atc1_phymxwrap ("pmgr-mxwrap"). */
#define PS_PHYMXWRAP			0x170

/*
 * Apple glue registers of the dwc3 core (pipe and reference clock enables) and the
 * values iBoot leaves in them. The core soft reset of the USB 2.0 flip clears them,
 * they take writes only with CIO off, and the xHCI reset may change them.
 */
static const u32 t6050_glue[] = { 0xcd28, 0xcd2c, 0xcd40, 0xcd44 };
static const u32 t6050_glue_boot[] = { 0x886, 0x886, 0x3c20060c, 0x2000b };

struct t6050_tunable_desc {
	const char *reg;
	const char *prop;
};

/* In the order the proxy sequence applies them. */
static const struct t6050_tunable_desc t6050_host_tunables[] = {
	{ "core", "apple,tunable-gbl" },
	{ "drd-cfg", "apple,tunable-cfg" },
	{ "core", "apple,tunable-link" },
	{ "ausbc-debug", "apple,tunable-ausbc-debug" },
	{ "bulk-fabric", "apple,tunable-bulk-fabric" },
	{ "pipehandler", "apple,tunable-pipehandler" },
	{ "core", "apple,tunable-cio" },
};

static const struct t6050_tunable_desc t6050_phy_tunables[] = {
	{ "atc-common-cfg", "apple,tunable-atc-common-cfg" },
	{ "atc-fabric", "apple,tunable-atc-fabric" },
	{ "atc-axi2af", "apple,tunable-atc-axi2af" },
	{ "atc", "apple,tunable-atc-pll" },
	{ "atc-axi2af-lioa", "apple,tunable-atc-axi2af-lioa" },
	{ "atc", "apple,tunable-atc-lanes" },
};

struct t6050_tunable {
	void __iomem *regs;
	struct apple_tunable *tunable;
};

struct t6050_usb2 {
	struct device *dev;
	void __iomem *usb2phy;
	void __iomem *pipehandler;
	void __iomem *core;

	/* SuperSpeed only */
	bool ss;
	struct work_struct ltssm_work;
	struct device_node *hpm;
	struct delayed_work replug_work;
	unsigned int replug_tries;
	struct delayed_work u2reset_work;
	unsigned int u2resets;
	bool u2reset_queued;
	struct delayed_work standalone_work;
	void __iomem *pctl;
	void __iomem *atc;
	void __iomem *mxw;
	void __iomem *atc_pwr;
	void __iomem *pmgr;
	void __iomem *pmgr_mxw;
	struct t6050_tunable host_tun[ARRAY_SIZE(t6050_host_tunables)];
	struct t6050_tunable phy_tun[ARRAY_SIZE(t6050_phy_tunables)];
};

static void t6050_usb2_mask(void __iomem *reg, u32 clear, u32 set)
{
	writel((readl(reg) & ~clear) | set, reg);
}

static void t6050_usb2_host_flip(struct t6050_usb2 *p)
{
	u32 val;

	/* usb2phy out of reset and running, as for device mode. */
	writel(USB2PHY_SIG_DEFAULT, p->usb2phy + USB2PHY_SIG);
	writel(USB2PHY_CTL_RESET | USB2PHY_CTL_PORT_RESET, p->usb2phy + USB2PHY_CTL);
	t6050_usb2_mask(p->usb2phy + USB2PHY_CTL, USB2PHY_CTL_RESET, 0);
	t6050_usb2_mask(p->usb2phy + USB2PHY_CTL, USB2PHY_CTL_PORT_RESET, 0);
	writel(USB2PHY_MISCTUNE_DEFAULT, p->usb2phy + USB2PHY_MISCTUNE);
	writel(USB2PHY_USBCTL_RUN, p->usb2phy + USB2PHY_USBCTL);

	/* PIPE on the dummy backend, dwc3 out of reset. */
	writel(PIPEHANDLER_MUX_CTRL_DUMMY, p->pipehandler + PIPEHANDLER_MUX_CTRL);
	writel(PIPEHANDLER_AON_GEN_DWC3_RESET_N, p->pipehandler + PIPEHANDLER_AON_GEN);
	writel(PIPEHANDLER_NONSELECTED_DEFAULT, p->pipehandler + PIPEHANDLER_NONSELECTED_OVERRIDE);

	/* usb2phy host bits, gadget stopped, core soft reset, port capability host. */
	writel(USB2PHY_SIG_DEFAULT | USB2PHY_SIG_HOST, p->usb2phy + USB2PHY_SIG);
	t6050_usb2_mask(p->core + DWC3_DCTL, DWC3_DCTL_RUN_STOP, 0);
	if (readl_poll_timeout(p->core + DWC3_DSTS, val, val & DWC3_DSTS_DEVCTRLHLT, 100, 10000))
		dev_dbg(p->dev, "device controller did not halt (DSTS %#x)\n", val);
	t6050_usb2_mask(p->core + DWC3_GCTL, 0, DWC3_GCTL_CORESOFTRESET);
	msleep(10);
	t6050_usb2_mask(p->core + DWC3_GCTL, DWC3_GCTL_PRTCAPDIR_MASK | DWC3_GCTL_CORESOFTRESET,
			DWC3_GCTL_PRTCAP_HOST);
	msleep(10);

	/* PIPE parked: link detection forced off through the overrides. */
	t6050_usb2_mask(p->pipehandler + PIPEHANDLER_OVERRIDE_VALUES, BIT(1) | BIT(2), 0);
	t6050_usb2_mask(p->pipehandler + PIPEHANDLER_OVERRIDE, 0, BIT(0) | BIT(2));

	/* No PHY auto-suspend: the xHCI reset needs the PHY clocks. */
	t6050_usb2_mask(p->core + DWC3_GUSB2PHYCFG0, DWC3_GUSB2PHYCFG_SUSPHY, 0);
	t6050_usb2_mask(p->core + DWC3_GUSB3PIPECTL0, DWC3_GUSB3PIPECTL_SUSPHY, 0);
}

/* Polls (reg & mask) == want; a timeout is only logged, as in the proxy sequence. */
static bool t6050_wait(void __iomem *reg, u32 mask, u32 want, unsigned int ms)
{
	u32 val;

	return !readl_poll_timeout(reg, val, (val & mask) == want, 100, ms * 1000);
}

static bool t6050_ps_on(void __iomem *reg)
{
	u32 val = readl(reg);

	if (FIELD_GET(PS_ACTUAL, val) == PS_ON)
		return true;
	writel((val & ~(PS_TARGET | GENMASK(9, 8))) | PS_ON, reg);
	return t6050_wait(reg, PS_ACTUAL, PS_ACTUAL, 100);
}

/*
 * Timing of the SuperSpeed sequence, settable on the command line while the link
 * training is not reliable: the gap after each register write (the proxy that
 * measured the sequence took about 1 ms per write) and a pause after step 16.
 */
static unsigned int ss_pace_us;
module_param(ss_pace_us, uint, 0444);
MODULE_PARM_DESC(ss_pace_us, "SuperSpeed sequence: gap after each register write, us");

static unsigned int ss_settle_ms;
module_param(ss_settle_ms, uint, 0444);
MODULE_PARM_DESC(ss_settle_ms, "SuperSpeed sequence: pause after step 16, ms");

/*
 * GUSB3PIPECTL SUSPHY: set by iBoot and set throughout the proxy runs that trained
 * the link; the USB 2.0 flip clears it.
 */
static bool ss_susphy = true;
module_param(ss_susphy, bool, 0444);
MODULE_PARM_DESC(ss_susphy, "SuperSpeed sequence: set GUSB3PIPECTL SUSPHY again before step 1");

static bool ss_susphy_clear;
module_param(ss_susphy_clear, bool, 0444);
MODULE_PARM_DESC(ss_susphy_clear, "SuperSpeed sequence: clear GUSB3PIPECTL SUSPHY after the xHCI reset (step 17)");

/* Plug orientation: flipped uses lane 1 and the swapped crossbar (steps 7, 9, 10). */
static bool ss_flip;
module_param(ss_flip, bool, 0444);
MODULE_PARM_DESC(ss_flip, "SuperSpeed sequence: flipped plug orientation");

/*
 * A device plugged in at power-on no longer goes for SuperSpeed on a USB 2.0 port
 * reset, but a fresh connection trains at once: after step 16 the port's Type-C
 * controller simulates an unplug of this many seconds. Off by default: the device
 * does not come back as a fresh connection (kernel/USB-BRINGUP.md).
 */
static unsigned int ss_replug_s;
module_param(ss_replug_s, uint, 0444);
MODULE_PARM_DESC(ss_replug_s, "SuperSpeed: simulated disconnect after step 16, s (0: off)");

/*
 * Step 18 again: a device left at high speed by the hub's USB 2.0 resets often goes
 * for SuperSpeed on a later one (the proxy: U0 on the 5th). From 3 s after the xHCI
 * reset, while the SS port has no connection, reset the USB 2.0 port every 2 s. Off by
 * default: under Linux each reset costs the high-speed device and none has trained yet.
 */
static unsigned int ss_u2resets;
module_param(ss_u2resets, uint, 0444);
MODULE_PARM_DESC(ss_u2resets, "SuperSpeed: extra USB 2.0 port resets until the SS port connects (0: off)");

/*
 * Test without the xHCI driver (port 1's xHCI node disabled): the proxy run from
 * here, steps 1-17, run, port power, then USB 2.0 resets until the SS port is in U0.
 */
static bool ss_standalone;
module_param(ss_standalone, bool, 0444);
MODULE_PARM_DESC(ss_standalone, "SuperSpeed test: sequence and USB 2.0 resets without the xHCI driver");

static void t6050_ss_mask(void __iomem *reg, u32 clear, u32 set)
{
	t6050_usb2_mask(reg, clear, set);
	if (ss_pace_us)
		usleep_range(ss_pace_us, ss_pace_us + ss_pace_us / 5 + 1);
}

static void t6050_tunables_apply(struct t6050_tunable *t, unsigned int n)
{
	unsigned int i, j;

	for (i = 0; i < n; i++)
		for (j = 0; j < t[i].tunable->sz; j++)
			t6050_ss_mask(t[i].regs + t[i].tunable->values[j].offset,
				      t[i].tunable->values[j].mask, t[i].tunable->values[j].value);
}

static bool t6050_rtk_send(struct t6050_usb2 *p, u64 msg)
{
	writeq(msg, p->mxw + MXW_A2I_MSG0);
	writeq(0, p->mxw + MXW_A2I_MSG1);		/* endpoint 0, management */
	return t6050_wait(p->mxw + MXW_A2I_CTRL, MXW_MBOX_FULL, 0, 100);
}

/*
 * Boots the PHY CPU's RTKit firmware, which iBoot left loaded, the way m1n1's
 * StandardASC does: HELLO -> ack, EPMAP -> ack, start the system endpoints,
 * AP power on, then IOP power on. Endpoint 1 (crashlog) asks for a buffer at
 * a preallocated address, which needs no reply; other endpoints are drained.
 */
static void t6050_phy_cpu_boot(struct t6050_usb2 *p)
{
	unsigned long ap_end = jiffies + msecs_to_jiffies(3000);
	unsigned long end = ap_end + msecs_to_jiffies(3000), settle = 0;
	u32 eps = 0, ap = 0, iop = 0, ver = 0, other = 0, other_ep = 0;
	u64 msg, other_msg = 0;
	bool iop_sent = false;
	unsigned int ep;

	t6050_usb2_mask(p->mxw + MXW_IDLE_CTRL, 0, BIT(0));
	msleep(10);
	writel(MXW_CPU_CTRL_RUN, p->mxw + MXW_CPU_CTRL);

	while (settle ? time_before(jiffies, settle) : time_before(jiffies, end)) {
		/* The proxy sent IOP power after 3 s even without the AP power ack. */
		if (!iop_sent && time_after(jiffies, ap_end)) {
			t6050_rtk_send(p, FIELD_PREP(RTK_TYPE, RTK_IOP_PWR) |
				       FIELD_PREP(RTK_PWR_STATE, 0x220));
			iop_sent = true;
		}
		if (readl(p->mxw + MXW_I2A_CTRL) & MXW_MBOX_EMPTY) {
			usleep_range(1000, 2000);
			continue;
		}
		msg = readq(p->mxw + MXW_I2A_MSG0);
		ep = readq(p->mxw + MXW_I2A_MSG1) & 0xff;
		if (ep) {
			other++;
			other_ep = ep;
			other_msg = msg;
			continue;
		}

		switch (FIELD_GET(RTK_TYPE, msg)) {
		case RTK_HELLO:
			ver = FIELD_GET(RTK_HELLO_MAX, msg);
			t6050_rtk_send(p, FIELD_PREP(RTK_TYPE, RTK_HELLO_ACK) |
				       FIELD_PREP(RTK_HELLO_MIN, ver) | FIELD_PREP(RTK_HELLO_MAX, ver));
			break;
		case RTK_EPMAP:
			if (FIELD_GET(RTK_EPMAP_BASE, msg) == 0)
				eps |= FIELD_GET(RTK_EPMAP_BITMAP, msg);
			t6050_rtk_send(p, FIELD_PREP(RTK_TYPE, RTK_EPMAP) | (msg & RTK_EPMAP_LAST) |
				       (msg & RTK_EPMAP_BASE) |
				       ((msg & RTK_EPMAP_LAST) ? 0 : RTK_EPMAP_MORE));
			if (msg & RTK_EPMAP_LAST) {
				for (ep = 1; ep < 0x10; ep++)
					if (eps & BIT(ep))
						t6050_rtk_send(p, FIELD_PREP(RTK_TYPE, RTK_START_EP) |
							       FIELD_PREP(RTK_EP, ep) | RTK_START_EP_FLAG);
				t6050_rtk_send(p, FIELD_PREP(RTK_TYPE, RTK_AP_PWR) |
					       FIELD_PREP(RTK_PWR_STATE, RTK_PWR_ON));
			}
			break;
		case RTK_AP_PWR:
			ap = FIELD_GET(RTK_PWR_STATE, msg);
			if (ap == RTK_PWR_ON && !iop_sent) {
				t6050_rtk_send(p, FIELD_PREP(RTK_TYPE, RTK_IOP_PWR) |
					       FIELD_PREP(RTK_PWR_STATE, 0x220));
				iop_sent = true;
			}
			break;
		case RTK_IOP_PWR_ACK:
			iop = FIELD_GET(RTK_PWR_STATE, msg);
			break;
		}
		/* Booted: keep draining for one more second, as the proxy did. */
		if (ap == RTK_PWR_ON && iop == RTK_PWR_ON && !settle)
			settle = jiffies + msecs_to_jiffies(1000);
	}

	dev_info(p->dev, "SS: PHY CPU RTKit v%u, endpoints %#x, AP %#x, IOP %#x, %u other message(s), last %#x:%#llx\n",
		 ver, eps, ap, iop, other, other_ep, other_msg);
}

/* Steps 1-16 of the port 1 SuperSpeed host sequence (kernel/USB-BRINGUP.md). */
static void t6050_usb3_up(struct t6050_usb2 *p)
{
	void __iomem *atc = p->atc, *pipe = p->pipehandler;
	bool hcrst, cnr, cio = true, mxw, tnbl, s1, rise, fall, lock, ack, unlock, rel, wpr;
	unsigned long wpr_us;
	unsigned int l = ss_flip;	/* active lane */
	unsigned int i, n = 0;
	char trace[17 * 9 + 1];
	u32 val, tr[17];
	int len = 0;

	/* The glue back to the boot values while CIO is still off; tr[] = SS PORTSC after each step. */
	for (i = 0; i < ARRAY_SIZE(t6050_glue); i++)
		writel(t6050_glue_boot[i], p->core + t6050_glue[i]);
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	if (ss_susphy)
		t6050_usb2_mask(p->core + DWC3_GUSB3PIPECTL0, 0, DWC3_GUSB3PIPECTL_SUSPHY);

	/* 1. xHCI reset while CIO is still off. */
	t6050_ss_mask(p->core + XHCI_USBCMD, 0, XHCI_USBCMD_HCRST);
	hcrst = t6050_wait(p->core + XHCI_USBCMD, XHCI_USBCMD_HCRST, 0, 1000);
	cnr = t6050_wait(p->core + XHCI_USBSTS, XHCI_USBSTS_CNR, 0, 1000);
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 2. Controller tunables. 3. TXDETRX lock request. */
	t6050_tunables_apply(p->host_tun, ARRAY_SIZE(p->host_tun));
	t6050_ss_mask(p->pctl + PCTL_TXDETRX_REQ, 0, BIT(0));
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 4. CIO power domains, then the PHY power control; the PHY reset stays asserted. */
	for (i = 0; i < ARRAY_SIZE(t6050_cio_domains); i++)
		cio &= t6050_ps_on(p->pmgr + t6050_cio_domains[i]);
	mxw = t6050_ps_on(p->pmgr_mxw + PS_PHYMXWRAP);
	tr[n++] = readl(p->core + XHCI_PORTSC2);
	t6050_ss_mask(p->atc_pwr + ATC_PWR_CTRL, 0, ATC_PWR_SLEEP_SMALL);
	t6050_wait(p->atc_pwr + ATC_PWR_STAT, ATC_PWR_SLEEP_SMALL, ATC_PWR_SLEEP_SMALL, 100);
	t6050_ss_mask(p->atc_pwr + ATC_PWR_CTRL, 0, ATC_PWR_SLEEP_BIG);
	t6050_wait(p->atc_pwr + ATC_PWR_STAT, ATC_PWR_SLEEP_BIG, ATC_PWR_SLEEP_BIG, 100);
	t6050_ss_mask(p->atc_pwr + ATC_PWR_CTRL, ATC_PWR_CLAMP, 0);
	t6050_ss_mask(p->atc_pwr + ATC_PWR_CTRL, 0, ATC_PWR_APB_RESET_N);
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 5. PHY tunables. */
	t6050_tunables_apply(p->phy_tun, ARRAY_SIZE(p->phy_tun));
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 6. Port capability host, no core soft reset. */
	t6050_ss_mask(p->core + DWC3_GCTL, DWC3_GCTL_PRTCAPDIR_MASK, DWC3_GCTL_PRTCAP_HOST);
	t6050_ss_mask(atc + ATC_LANE_MODE, 0xfc, (2 << 2) | (2 << 5));
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 7. Sleep control, USB3 mode: common block and lane 0 overrides to the FSM. */
	t6050_ss_mask(atc + ATC_CFG0, BIT(1) | BIT(3) | BIT(5), 0);
	t6050_ss_mask(atc + ATC_RX_PWR, BIT(2 + l), 0);
	t6050_ss_mask(atc + ATC_RX_PWR, BIT(6 + l), 0);
	t6050_ss_mask(atc + ATC_CFG0, BIT(16 + l), 0);
	t6050_ss_mask(atc + ATC_CFG0, BIT(8 + l), 0);
	t6050_ss_mask(atc + ATC_CFG0, BIT(12 + l), 0);
	t6050_ss_mask(atc + ATC_TX_PWR, BIT(6 + l), 0);
	t6050_ss_mask(atc + ATC_TX_PWR, BIT(2 + l), 0);
	t6050_ss_mask(atc + ATC_TX_PWR, BIT(10 + l), 0);
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 8. MxWrap idle control. */
	t6050_ss_mask(p->mxw + MXW_IDLE_CTRL, 0, BIT(0));
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/*
	 * 9. Crossbar, USB3 + DP: normal lane 0 mode 1, lane 1 mode 2; flipped lane 0
	 * mode 2, lane 1 mode 1.
	 */
	if (l)
		t6050_ss_mask(atc + ATC_LANE_MODE, 0xfc | BIT(22) | BIT(24), 0x28);
	else
		t6050_ss_mask(atc + ATC_LANE_MODE, 0xfc | BIT(23) | BIT(25), 0x44);
	t6050_ss_mask(atc + ATC_XBAR, GENMASK(16, 0), 0x110 | l);
	t6050_ss_mask(atc + ATC_LANE_MODE, GENMASK(1, 0), 2);
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 10. BIST first half. */
	t6050_ss_mask(atc + ATC_BIST_CFG1, 0, ATC_BIST_CFG1_CLOCK_EN);
	t6050_ss_mask(atc + ATC_BIST_OV1, ATC_BIST_OV1_PWR_DOWN | ATC_BIST_OV1_PIPE_RESET_N,
			FIELD_PREP(ATC_BIST_OV1_PWR_DOWN, 2));
	t6050_ss_mask(atc + ATC_BIST_CFG1, 0, ATC_BIST_CFG1_USB_EN);
	t6050_ss_mask(atc + ATC_BIST_CFG1, ATC_BIST_CFG1_USB_EN, 0);
	t6050_ss_mask(atc + ATC_BIST_OV2, 0x660, 0x220);
	t6050_ss_mask(atc + ATC_BIST_CFG1, 0, ATC_BIST_CFG1_USB_EN);
	t6050_ss_mask(atc + ATC_BIST_CFG1, ATC_BIST_CFG1_USB_EN, 0);
	t6050_ss_mask(p->atc_pwr + ATC_PWR_MISC, ATC_PWR_MISC_USB_LN_SWAP,
		      l ? ATC_PWR_MISC_USB_LN_SWAP : 0);
	msleep(10);
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 11. PHY CPU firmware, then its tunables-done flag. */
	t6050_phy_cpu_boot(p);
	usleep_range(5000, 6000);
	tnbl = t6050_wait(atc + ATC_UC_CTRL, ATC_UC_TNBL_DONE, ATC_UC_TNBL_DONE, 1000);
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 12. PHY reset release. */
	t6050_ss_mask(p->atc_pwr + ATC_PWR_CTRL, 0, ATC_PWR_PHY_RESET_N);
	msleep(100);
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 13. BIST second half. */
	t6050_ss_mask(atc + ATC_BIST_OV1, 0, ATC_BIST_OV1_PIPE_RESET_N);
	t6050_ss_mask(atc + ATC_BIST_CFG1, 0, ATC_BIST_CFG1_USB_EN);
	t6050_ss_mask(atc + ATC_BIST_CFG1, ATC_BIST_CFG1_USB_EN, 0);
	s1 = t6050_wait(atc + ATC_BIST_STATUS2, ATC_BIST_STATUS2_PHY_STATUS, 0, 1000);
	t6050_ss_mask(atc + ATC_BIST_OV1, ATC_BIST_OV1_PWR_DOWN, FIELD_PREP(ATC_BIST_OV1_PWR_DOWN, 3));
	t6050_ss_mask(atc + ATC_BIST_CFG1, 0, ATC_BIST_CFG1_USB_EN);
	t6050_ss_mask(atc + ATC_BIST_CFG1, ATC_BIST_CFG1_USB_EN, 0);
	rise = t6050_wait(atc + ATC_BIST_STATUS2, ATC_BIST_STATUS2_PHY_STATUS,
			  ATC_BIST_STATUS2_PHY_STATUS, 1000);
	fall = t6050_wait(atc + ATC_BIST_STATUS2, ATC_BIST_STATUS2_PHY_STATUS, 0, 1000);
	/*
	 * The SS port has been in a warm reset since host mode (step 6); it completes
	 * with the PHY held here (power-down override 3). The proxy runs, which missed
	 * the PhyStatus pulse and waited out its 1 s timeout, always had it completed
	 * at this point and reached step 17 with a clean port; wait for it here.
	 */
	wpr_us = jiffies;
	wpr = t6050_wait(p->core + XHCI_PORTSC2, XHCI_PORTSC_PR, 0, 1000);
	wpr_us = jiffies_to_usecs(jiffies - wpr_us);
	t6050_ss_mask(atc + ATC_BIST_OV2, 0x660, 0);
	t6050_ss_mask(atc + ATC_BIST_CFG1, 0, ATC_BIST_CFG1_USB_EN);
	t6050_ss_mask(atc + ATC_BIST_CFG1, ATC_BIST_CFG1_USB_EN, 0);
	t6050_ss_mask(pipe + PIPEHANDLER_NONSELECTED_OVERRIDE, GENMASK(3, 0), 3);
	t6050_ss_mask(pipe + PIPEHANDLER_NONSELECTED_OVERRIDE, BIT(12), 0);
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 14. PIPE overrides and lock; the PHY answers the TXDETRX request. */
	t6050_ss_mask(pipe + PIPEHANDLER_OVERRIDE_VALUES, BIT(1) | BIT(2), 0);
	t6050_ss_mask(pipe + PIPEHANDLER_OVERRIDE, 0, BIT(0));
	t6050_ss_mask(pipe + PIPEHANDLER_OVERRIDE, 0, BIT(2));
	t6050_ss_mask(pipe + PIPEHANDLER_LOCK_REQ, 0, PIPEHANDLER_LOCK);
	lock = t6050_wait(pipe + PIPEHANDLER_LOCK_ACK, PIPEHANDLER_LOCK, PIPEHANDLER_LOCK, 100);
	ack = t6050_wait(p->pctl + PCTL_TXDETRX_ACK, BIT(0), BIT(0), 50);
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 15. PIPE to the USB3 PHY, overrides off, unlock. */
	t6050_ss_mask(pipe + PIPEHANDLER_MUX_CTRL, PIPEHANDLER_MUX_CLK, 0);
	t6050_ss_mask(pipe + PIPEHANDLER_MUX_CTRL, PIPEHANDLER_MUX_DATA, 0);
	t6050_ss_mask(pipe + PIPEHANDLER_MUX_CTRL, PIPEHANDLER_MUX_CLK,
			FIELD_PREP(PIPEHANDLER_MUX_CLK, 1));
	t6050_ss_mask(pipe + PIPEHANDLER_OVERRIDE, BIT(0), 0);
	t6050_ss_mask(pipe + PIPEHANDLER_OVERRIDE, BIT(2), 0);
	t6050_ss_mask(pipe + PIPEHANDLER_LOCK_REQ, PIPEHANDLER_LOCK, 0);
	unlock = t6050_wait(pipe + PIPEHANDLER_LOCK_ACK, PIPEHANDLER_LOCK, 0, 10);
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	/* 16. TXDETRX lock release. */
	t6050_ss_mask(p->pctl + PCTL_TXDETRX_REQ, BIT(0), 0);
	rel = t6050_wait(p->pctl + PCTL_TXDETRX_ACK, BIT(0), 0, 50);
	tr[n++] = readl(p->core + XHCI_PORTSC2);

	val = readl(atc + ATC_BIST_STATUS2);
	for (i = 0; i < n; i++)
		len += scnprintf(trace + len, sizeof(trace) - len, " %x", tr[i]);

	dev_info(p->dev, "SS: HCRST %d CNR %d, CIO %d, mxwrap %d, UC_TNBL_DONE %d\n",
		 hcrst, cnr, cio, mxw, tnbl);
	dev_info(p->dev, "SS: SS PORTSC from the write through step 16:%s\n", trace);
	dev_info(p->dev, "SS: PhyStatus s1 %d rise %d fall %d (STATUS2 %#x), lock %d ack %d unlock %d release %d\n",
		 s1, rise, fall, val, lock, ack, unlock, rel);
	dev_info(p->dev, "SS: warm reset in step 13 done %d after %lu us\n", wpr, wpr_us);
	dev_info(p->dev, "SS: pace %u us, settle %u ms, susphy %d, flip %d\n", ss_pace_us, ss_settle_ms,
		 ss_susphy, ss_flip);
	msleep(ss_settle_ms);
	dev_info(p->dev, "SS: glue %#x %#x %#x %#x, PORTSC %#x / %#x\n", readl(p->core + t6050_glue[0]),
		 readl(p->core + t6050_glue[1]), readl(p->core + t6050_glue[2]),
		 readl(p->core + t6050_glue[3]), readl(p->core + XHCI_PORTSC1),
		 readl(p->core + XHCI_PORTSC2));
	if (p->hpm && ss_replug_s)
		schedule_delayed_work(&p->replug_work, 0);
}

/* Diagnostic around the simulated unplug: the port 1 registers of both sides. */
static void t6050_port_state(struct t6050_usb2 *p, const char *when)
{
	dev_info(p->dev, "SS: %s: PORTSC %#x / %#x, usb2phy %#x %#x %#x, pipe mux %#x, GUSB2PHYCFG %#x, GUSB3PIPECTL %#x, power %#x / %#x, UC_CTRL %#x, STATUS2 %#x\n",
		 when, readl(p->core + XHCI_PORTSC1), readl(p->core + XHCI_PORTSC2),
		 readl(p->usb2phy + USB2PHY_USBCTL), readl(p->usb2phy + USB2PHY_CTL),
		 readl(p->usb2phy + USB2PHY_SIG), readl(p->pipehandler + PIPEHANDLER_MUX_CTRL),
		 readl(p->core + DWC3_GUSB2PHYCFG0), readl(p->core + DWC3_GUSB3PIPECTL0),
		 readl(p->atc_pwr + ATC_PWR_CTRL), readl(p->atc_pwr + ATC_PWR_STAT),
		 readl(p->atc + ATC_UC_CTRL), readl(p->atc + ATC_BIST_STATUS2));
}

/* Waits up to 10 s for the Type-C controller's driver, then simulates the unplug. */
static void t6050_replug(struct work_struct *work)
{
	struct t6050_usb2 *p = container_of(to_delayed_work(work), struct t6050_usb2, replug_work);
	int ret;

	if (!p->replug_tries)
		t6050_port_state(p, "before the disconnect");
	ret = sn201202x_simulate_disconnect(p->hpm, min(ss_replug_s, 255U));
	if (ret == -EPROBE_DEFER && ++p->replug_tries < 100) {
		schedule_delayed_work(&p->replug_work, msecs_to_jiffies(100));
		return;
	}
	dev_info(p->dev, "SS: simulated disconnect %u s: %d after %u tries\n", ss_replug_s, ret,
		 p->replug_tries + 1);
	t6050_port_state(p, "after the disconnect");
}

/*
 * Diagnostic while the link training is not reliable: for 3 s after the xHCI reset,
 * log every change of the two PORTSC registers and of the controller's LTSSM state.
 */
static void t6050_ltssm_watch(struct work_struct *work)
{
	struct t6050_usb2 *p = container_of(work, struct t6050_usb2, ltssm_work);
	u32 last[3] = { ~0U, ~0U, ~0U }, now[3];
	unsigned int i, changes = 0;

	for (i = 0; i < 6000 && changes < 60; i++) {
		now[0] = readl(p->core + XHCI_PORTSC1);
		now[1] = readl(p->core + XHCI_PORTSC2);
		now[2] = readl(p->core + DWC3_GDBGLTSSM);
		if (memcmp(now, last, sizeof(now))) {
			dev_info(p->dev, "SS watch: PORTSC %#x / %#x, GDBGLTSSM %#x\n",
				 now[0], now[1], now[2]);
			memcpy(last, now, sizeof(now));
			changes++;
		}
		usleep_range(500, 600);
	}
}

/* After the xHCI reset (step 17): the glue back to the iBoot values, host mode. */
static int t6050_usb2_calibrate(struct phy *phy)
{
	struct t6050_usb2 *p = phy_get_drvdata(phy);
	u32 rd[ARRAY_SIZE(t6050_glue)];
	unsigned int i, changed = 0;

	if (!p->ss)
		return 0;
	for (i = 0; i < ARRAY_SIZE(t6050_glue); i++) {
		rd[i] = readl(p->core + t6050_glue[i]);
		if (rd[i] == t6050_glue_boot[i])
			continue;
		writel(t6050_glue_boot[i], p->core + t6050_glue[i]);
		changed |= BIT(i);
	}
	t6050_usb2_mask(p->core + DWC3_GCTL, DWC3_GCTL_PRTCAPDIR_MASK, DWC3_GCTL_PRTCAP_HOST);
	if (ss_susphy_clear)
		t6050_usb2_mask(p->core + DWC3_GUSB3PIPECTL0, DWC3_GUSB3PIPECTL_SUSPHY, 0);
	dev_info(p->dev, "SS: after xHCI reset: glue read %#x %#x %#x %#x, rewritten %#x, now %#x %#x %#x %#x\n",
		 rd[0], rd[1], rd[2], rd[3], changed, readl(p->core + t6050_glue[0]),
		 readl(p->core + t6050_glue[1]), readl(p->core + t6050_glue[2]),
		 readl(p->core + t6050_glue[3]));
	dev_info(p->dev, "SS: after xHCI reset: USBSTS %#x, PORTSC %#x / %#x, GUSB2PHYCFG %#x, GUSB3PIPECTL %#x\n",
		 readl(p->core + XHCI_USBSTS), readl(p->core + XHCI_PORTSC1),
		 readl(p->core + XHCI_PORTSC2), readl(p->core + DWC3_GUSB2PHYCFG0),
		 readl(p->core + DWC3_GUSB3PIPECTL0));
	queue_work(system_long_wq, &p->ltssm_work);
	/* phy_calibrate() runs again on a root hub resume */
	if (ss_u2resets && !p->u2reset_queued) {
		p->u2reset_queued = true;
		schedule_delayed_work(&p->u2reset_work, msecs_to_jiffies(3000));
	}
	return 0;
}

static void t6050_usb3_inherited(struct t6050_usb2 *p);

static void t6050_standalone(struct work_struct *work)
{
	struct t6050_usb2 *p = container_of(to_delayed_work(work), struct t6050_usb2,
					    standalone_work);
	u32 last[2] = { ~0U, ~0U }, now[2];
	unsigned int i, k, changes = 0;
	bool hcrst, cnr;

	t6050_usb3_inherited(p);
	t6050_usb3_up(p);

	/* 17. xHCI reset, glue, host mode; then run and port power, as the proxy runs. */
	t6050_usb2_mask(p->core + XHCI_USBCMD, 0, XHCI_USBCMD_HCRST);
	hcrst = t6050_wait(p->core + XHCI_USBCMD, XHCI_USBCMD_HCRST, 0, 2000);
	cnr = t6050_wait(p->core + XHCI_USBSTS, XHCI_USBSTS_CNR, 0, 2000);
	for (i = 0; i < ARRAY_SIZE(t6050_glue); i++)
		writel(t6050_glue_boot[i], p->core + t6050_glue[i]);
	t6050_usb2_mask(p->core + DWC3_GCTL, DWC3_GCTL_PRTCAPDIR_MASK, DWC3_GCTL_PRTCAP_HOST);
	t6050_usb2_mask(p->core + XHCI_USBCMD, 0, BIT(0));
	for (i = 0; i < 2; i++) {
		void __iomem *reg = p->core + (i ? XHCI_PORTSC2 : XHCI_PORTSC1);

		writel((readl(reg) & 0x4e00ffe9) | BIT(9), reg);
	}
	dev_info(p->dev, "SS standalone: HCRST %d CNR %d, PORTSC %#x / %#x\n", hcrst, cnr,
		 readl(p->core + XHCI_PORTSC1), readl(p->core + XHCI_PORTSC2));
	msleep(100);

	/* 18. USB 2.0 port resets, 2 s apart, until the SS port is in U0 */
	for (k = 1; k <= 10; k++) {
		u32 hs = readl(p->core + XHCI_PORTSC1);

		writel((hs & 0x0e00c200) | XHCI_PORTSC_PR, p->core + XHCI_PORTSC1);
		dev_info(p->dev, "SS standalone: USB 2.0 reset %u, PORTSC %#x / %#x\n", k, hs,
			 readl(p->core + XHCI_PORTSC2));
		for (i = 0; i < 2000; i++) {
			now[0] = readl(p->core + XHCI_PORTSC1);
			now[1] = readl(p->core + XHCI_PORTSC2);
			if (memcmp(now, last, sizeof(now)) && changes++ < 100)
				dev_info(p->dev, "SS standalone: %u ms PORTSC %#x / %#x\n", i,
					 now[0], now[1]);
			memcpy(last, now, sizeof(now));
			if ((now[1] & 0x1e1) == XHCI_PORTSC_CCS) {
				dev_info(p->dev, "SS standalone: U0 after USB 2.0 reset %u\n", k);
				return;
			}
			usleep_range(1000, 1100);
		}
	}
	dev_info(p->dev, "SS standalone: no U0 after 10 USB 2.0 resets\n");
}

static void t6050_u2reset(struct work_struct *work)
{
	struct t6050_usb2 *p = container_of(to_delayed_work(work), struct t6050_usb2, u2reset_work);
	u32 hs = readl(p->core + XHCI_PORTSC1), ss = readl(p->core + XHCI_PORTSC2);

	if (ss & XHCI_PORTSC_CCS) {
		dev_info(p->dev, "SS: connected after %u extra USB 2.0 reset(s), PORTSC %#x / %#x\n",
			 p->u2resets, hs, ss);
		return;
	}
	if (p->u2resets == ss_u2resets) {
		dev_info(p->dev, "SS: no connection after %u extra USB 2.0 resets, PORTSC %#x / %#x\n",
			 p->u2resets, hs, ss);
		return;
	}
	if (hs & XHCI_PORTSC_CCS) {
		/* PP, port indicators and the wake enables kept; the change bits not cleared */
		writel((hs & 0x0e00c200) | XHCI_PORTSC_PR, p->core + XHCI_PORTSC1);
		p->u2resets++;
		dev_info(p->dev, "SS: extra USB 2.0 reset %u, PORTSC %#x / %#x\n", p->u2resets, hs, ss);
	}
	schedule_delayed_work(&p->u2reset_work, msecs_to_jiffies(2000));
}

/*
 * The port 1 state the kernel inherits, to compare with the state the proven proxy
 * runs started from. The core is read only with dwc3 out of reset.
 */
static void t6050_usb3_inherited(struct t6050_usb2 *p)
{
	void __iomem *pipe = p->pipehandler;
	u32 aon = readl(pipe + PIPEHANDLER_AON_GEN);

	dev_info(p->dev, "SS: inherited pipehandler +0x0 %#x +0x4 %#x +0xc %#x +0x1c %#x +0x20 %#x\n",
		 readl(pipe + PIPEHANDLER_OVERRIDE), readl(pipe + PIPEHANDLER_OVERRIDE_VALUES),
		 readl(pipe + PIPEHANDLER_MUX_CTRL), aon,
		 readl(pipe + PIPEHANDLER_NONSELECTED_OVERRIDE));
	if (!(aon & PIPEHANDLER_AON_GEN_DWC3_RESET_N)) {
		dev_info(p->dev, "SS: inherited dwc3 in reset, core not read; reset released\n");
		writel(PIPEHANDLER_AON_GEN_DWC3_RESET_N, pipe + PIPEHANDLER_AON_GEN);
		return;
	}
	dev_info(p->dev, "SS: inherited GCTL %#x, GUSB2PHYCFG %#x, GUSB3PIPECTL %#x, glue %#x / %#x, PORTSC %#x / %#x\n",
		 readl(p->core + DWC3_GCTL), readl(p->core + DWC3_GUSB2PHYCFG0),
		 readl(p->core + DWC3_GUSB3PIPECTL0), readl(p->core + t6050_glue[0]),
		 readl(p->core + t6050_glue[2]), readl(p->core + XHCI_PORTSC1),
		 readl(p->core + XHCI_PORTSC2));
}

static int t6050_usb2_init(struct phy *phy)
{
	struct t6050_usb2 *p = phy_get_drvdata(phy);
	static const unsigned int wait_ms[] = { 50, 100, 100 };
	unsigned int attempt;
	u32 portsc = 0;

	/*
	 * Port 1 starts the SuperSpeed sequence from the inherited state, as the
	 * proven proxy runs did: no USB 2.0 flip, step 6 sets host mode.
	 */
	if (p->ss) {
		t6050_usb3_inherited(p);
		t6050_usb3_up(p);
		return 0;
	}

	/*
	 * The loader saw on J714c that a device plugged in at power-on shows
	 * up only after a second bring-up and flip (it also redoes the dwc3
	 * device-mode bring-up in between; this driver repeats the flip
	 * only). A port without a device stays in host mode after the last
	 * attempt.
	 */
	for (attempt = 0; attempt < ARRAY_SIZE(wait_ms); attempt++) {
		t6050_usb2_host_flip(p);
		if (!readl_poll_timeout(p->core + XHCI_PORTSC1, portsc, portsc & XHCI_PORTSC_CCS,
					1000, wait_ms[attempt] * 1000))
			break;
	}

	dev_info(p->dev, "USB 2.0 host mode, PORTSC %#x after %u flip(s)\n", portsc,
		 min_t(unsigned int, attempt + 1, ARRAY_SIZE(wait_ms)));
	return 0;
}

static const struct phy_ops t6050_usb2_ops = {
	.init = t6050_usb2_init,
	.calibrate = t6050_usb2_calibrate,
	.owner = THIS_MODULE,
};

static void __iomem *t6050_usb2_map(struct platform_device *pdev, const char *name)
{
	struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);

	if (!res)
		return IOMEM_ERR_PTR(-EINVAL);
	/* Not requested: the core window overlaps the xHCI node's registers. */
	return devm_ioremap(&pdev->dev, res->start, resource_size(res)) ?: IOMEM_ERR_PTR(-ENOMEM);
}

static int t6050_tunables_parse(struct platform_device *pdev, const struct t6050_tunable_desc *desc,
				struct t6050_tunable *t, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, desc[i].reg);

		if (!res)
			return dev_err_probe(&pdev->dev, -EINVAL, "%s registers\n", desc[i].reg);
		t[i].regs = t6050_usb2_map(pdev, desc[i].reg);
		if (IS_ERR(t[i].regs))
			return PTR_ERR(t[i].regs);
		t[i].tunable = devm_apple_tunable_parse(&pdev->dev, pdev->dev.of_node, desc[i].prop, res);
		if (IS_ERR(t[i].tunable))
			return dev_err_probe(&pdev->dev, PTR_ERR(t[i].tunable), "%s\n", desc[i].prop);
	}
	return 0;
}

static int t6050_usb3_probe(struct platform_device *pdev, struct t6050_usb2 *p)
{
	struct {
		void __iomem **regs;
		const char *name;
	} maps[] = {
		{ &p->pctl, "pctl" }, { &p->atc, "atc" }, { &p->mxw, "atc-mxwrap" },
		{ &p->atc_pwr, "atc-power" }, { &p->pmgr, "pmgr" }, { &p->pmgr_mxw, "pmgr-mxwrap" },
	};
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(maps); i++) {
		*maps[i].regs = t6050_usb2_map(pdev, maps[i].name);
		if (IS_ERR(*maps[i].regs))
			return dev_err_probe(&pdev->dev, PTR_ERR(*maps[i].regs), "%s registers\n",
					     maps[i].name);
	}
	ret = t6050_tunables_parse(pdev, t6050_host_tunables, p->host_tun, ARRAY_SIZE(p->host_tun));
	if (ret)
		return ret;
	ret = t6050_tunables_parse(pdev, t6050_phy_tunables, p->phy_tun, ARRAY_SIZE(p->phy_tun));
	if (ret)
		return ret;
	INIT_WORK(&p->ltssm_work, t6050_ltssm_watch);
	INIT_DELAYED_WORK(&p->replug_work, t6050_replug);
	INIT_DELAYED_WORK(&p->u2reset_work, t6050_u2reset);
	INIT_DELAYED_WORK(&p->standalone_work, t6050_standalone);
	if (ss_standalone)
		schedule_delayed_work(&p->standalone_work, msecs_to_jiffies(1000));
	p->hpm = of_parse_phandle(pdev->dev.of_node, "apple,typec-controller", 0);
	p->ss = true;
	return 0;
}

static int t6050_usb2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct t6050_usb2 *p;
	struct phy *phy;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = dev;

	p->usb2phy = t6050_usb2_map(pdev, "usb2phy");
	if (IS_ERR(p->usb2phy))
		return dev_err_probe(dev, PTR_ERR(p->usb2phy), "usb2phy registers\n");
	p->pipehandler = t6050_usb2_map(pdev, "pipehandler");
	if (IS_ERR(p->pipehandler))
		return dev_err_probe(dev, PTR_ERR(p->pipehandler), "pipehandler registers\n");
	p->core = t6050_usb2_map(pdev, "core");
	if (IS_ERR(p->core))
		return dev_err_probe(dev, PTR_ERR(p->core), "core registers\n");

	if (platform_get_resource_byname(pdev, IORESOURCE_MEM, "atc-power")) {
		int ret = t6050_usb3_probe(pdev, p);

		if (ret)
			return ret;
	}

	phy = devm_phy_create(dev, NULL, &t6050_usb2_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);
	phy_set_drvdata(phy, p);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id t6050_usb2_of_match[] = {
	{ .compatible = "apple,t6050-usb2-phy" },
	{}
};
MODULE_DEVICE_TABLE(of, t6050_usb2_of_match);

static struct platform_driver t6050_usb2_driver = {
	.probe = t6050_usb2_probe,
	.driver = {
		.name = "phy-apple-t6050-usb2",
		.of_match_table = t6050_usb2_of_match,
	},
};
module_platform_driver(t6050_usb2_driver);

MODULE_DESCRIPTION("Apple T6050 Type-C port USB 2.0 host PHY");
MODULE_LICENSE("Dual MIT/GPL");

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

/* xHCI CAPLENGTH is 0x20 on these controllers; PORTSC of port 1 at op + 0x400. */
#define XHCI_USBCMD			0x20
#define XHCI_USBCMD_HCRST		BIT(1)
#define XHCI_USBSTS			0x24
#define XHCI_USBSTS_CNR			BIT(11)
#define XHCI_PORTSC1			0x420
#define XHCI_PORTSC2			0x430
#define XHCI_PORTSC_CCS			BIT(0)

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
	void __iomem *pctl;
	void __iomem *atc;
	void __iomem *mxw;
	void __iomem *atc_pwr;
	void __iomem *pmgr;
	void __iomem *pmgr_mxw;
	struct t6050_tunable host_tun[ARRAY_SIZE(t6050_host_tunables)];
	struct t6050_tunable phy_tun[ARRAY_SIZE(t6050_phy_tunables)];
	u32 glue[ARRAY_SIZE(t6050_glue)];
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

static void t6050_tunables_apply(struct t6050_tunable *t, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		apple_tunable_apply(t[i].regs, t[i].tunable);
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
	bool hcrst, cnr, cio = true, mxw, tnbl, s1, rise, fall, lock, ack, unlock, rel;
	unsigned int i;
	u32 val, glue;

	/* The glue back to the boot values while CIO is still off. */
	for (i = 0; i < ARRAY_SIZE(t6050_glue); i++)
		writel(t6050_glue_boot[i], p->core + t6050_glue[i]);
	glue = readl(p->core + t6050_glue[0]);

	/* 1. xHCI reset while CIO is still off. */
	t6050_usb2_mask(p->core + XHCI_USBCMD, 0, XHCI_USBCMD_HCRST);
	hcrst = t6050_wait(p->core + XHCI_USBCMD, XHCI_USBCMD_HCRST, 0, 1000);
	cnr = t6050_wait(p->core + XHCI_USBSTS, XHCI_USBSTS_CNR, 0, 1000);

	/* 2. Controller tunables. 3. TXDETRX lock request. */
	t6050_tunables_apply(p->host_tun, ARRAY_SIZE(p->host_tun));
	t6050_usb2_mask(p->pctl + PCTL_TXDETRX_REQ, 0, BIT(0));

	/* 4. CIO power domains, then the PHY power control; the PHY reset stays asserted. */
	for (i = 0; i < ARRAY_SIZE(t6050_cio_domains); i++)
		cio &= t6050_ps_on(p->pmgr + t6050_cio_domains[i]);
	mxw = t6050_ps_on(p->pmgr_mxw + PS_PHYMXWRAP);
	t6050_usb2_mask(p->atc_pwr + ATC_PWR_CTRL, 0, ATC_PWR_SLEEP_SMALL);
	t6050_wait(p->atc_pwr + ATC_PWR_STAT, ATC_PWR_SLEEP_SMALL, ATC_PWR_SLEEP_SMALL, 100);
	t6050_usb2_mask(p->atc_pwr + ATC_PWR_CTRL, 0, ATC_PWR_SLEEP_BIG);
	t6050_wait(p->atc_pwr + ATC_PWR_STAT, ATC_PWR_SLEEP_BIG, ATC_PWR_SLEEP_BIG, 100);
	t6050_usb2_mask(p->atc_pwr + ATC_PWR_CTRL, ATC_PWR_CLAMP, 0);
	t6050_usb2_mask(p->atc_pwr + ATC_PWR_CTRL, 0, ATC_PWR_APB_RESET_N);

	/* 5. PHY tunables. */
	t6050_tunables_apply(p->phy_tun, ARRAY_SIZE(p->phy_tun));

	/* 6. Port capability host, no core soft reset. */
	t6050_usb2_mask(p->core + DWC3_GCTL, DWC3_GCTL_PRTCAPDIR_MASK, DWC3_GCTL_PRTCAP_HOST);
	t6050_usb2_mask(atc + ATC_LANE_MODE, 0xfc, (2 << 2) | (2 << 5));

	/* 7. Sleep control, USB3 mode: common block and lane 0 overrides to the FSM. */
	t6050_usb2_mask(atc + ATC_CFG0, BIT(1) | BIT(3) | BIT(5), 0);
	t6050_usb2_mask(atc + ATC_RX_PWR, BIT(2), 0);
	t6050_usb2_mask(atc + ATC_RX_PWR, BIT(6), 0);
	t6050_usb2_mask(atc + ATC_CFG0, BIT(16), 0);
	t6050_usb2_mask(atc + ATC_CFG0, BIT(8), 0);
	t6050_usb2_mask(atc + ATC_CFG0, BIT(12), 0);
	t6050_usb2_mask(atc + ATC_TX_PWR, BIT(6), 0);
	t6050_usb2_mask(atc + ATC_TX_PWR, BIT(2), 0);
	t6050_usb2_mask(atc + ATC_TX_PWR, BIT(10), 0);

	/* 8. MxWrap idle control. */
	t6050_usb2_mask(p->mxw + MXW_IDLE_CTRL, 0, BIT(0));

	/* 9. Crossbar, normal orientation: lane 0 mode 1, lane 1 mode 2, USB3 + DP. */
	t6050_usb2_mask(atc + ATC_LANE_MODE, 0xfc | BIT(23) | BIT(25), 0x44);
	t6050_usb2_mask(atc + ATC_XBAR, GENMASK(16, 0), 0x110);
	t6050_usb2_mask(atc + ATC_LANE_MODE, GENMASK(1, 0), 2);

	/* 10. BIST first half. */
	t6050_usb2_mask(atc + ATC_BIST_CFG1, 0, ATC_BIST_CFG1_CLOCK_EN);
	t6050_usb2_mask(atc + ATC_BIST_OV1, ATC_BIST_OV1_PWR_DOWN | ATC_BIST_OV1_PIPE_RESET_N,
			FIELD_PREP(ATC_BIST_OV1_PWR_DOWN, 2));
	t6050_usb2_mask(atc + ATC_BIST_CFG1, 0, ATC_BIST_CFG1_USB_EN);
	t6050_usb2_mask(atc + ATC_BIST_CFG1, ATC_BIST_CFG1_USB_EN, 0);
	t6050_usb2_mask(atc + ATC_BIST_OV2, 0x660, 0x220);
	t6050_usb2_mask(atc + ATC_BIST_CFG1, 0, ATC_BIST_CFG1_USB_EN);
	t6050_usb2_mask(atc + ATC_BIST_CFG1, ATC_BIST_CFG1_USB_EN, 0);
	t6050_usb2_mask(p->atc_pwr + ATC_PWR_MISC, ATC_PWR_MISC_USB_LN_SWAP, 0);
	msleep(10);

	/* 11. PHY CPU firmware, then its tunables-done flag. */
	t6050_phy_cpu_boot(p);
	usleep_range(5000, 6000);
	tnbl = t6050_wait(atc + ATC_UC_CTRL, ATC_UC_TNBL_DONE, ATC_UC_TNBL_DONE, 1000);

	/* 12. PHY reset release. */
	t6050_usb2_mask(p->atc_pwr + ATC_PWR_CTRL, 0, ATC_PWR_PHY_RESET_N);
	msleep(100);

	/* 13. BIST second half. */
	t6050_usb2_mask(atc + ATC_BIST_OV1, 0, ATC_BIST_OV1_PIPE_RESET_N);
	t6050_usb2_mask(atc + ATC_BIST_CFG1, 0, ATC_BIST_CFG1_USB_EN);
	t6050_usb2_mask(atc + ATC_BIST_CFG1, ATC_BIST_CFG1_USB_EN, 0);
	s1 = t6050_wait(atc + ATC_BIST_STATUS2, ATC_BIST_STATUS2_PHY_STATUS, 0, 1000);
	t6050_usb2_mask(atc + ATC_BIST_OV1, ATC_BIST_OV1_PWR_DOWN, FIELD_PREP(ATC_BIST_OV1_PWR_DOWN, 3));
	t6050_usb2_mask(atc + ATC_BIST_CFG1, 0, ATC_BIST_CFG1_USB_EN);
	t6050_usb2_mask(atc + ATC_BIST_CFG1, ATC_BIST_CFG1_USB_EN, 0);
	rise = t6050_wait(atc + ATC_BIST_STATUS2, ATC_BIST_STATUS2_PHY_STATUS,
			  ATC_BIST_STATUS2_PHY_STATUS, 1000);
	fall = t6050_wait(atc + ATC_BIST_STATUS2, ATC_BIST_STATUS2_PHY_STATUS, 0, 1000);
	t6050_usb2_mask(atc + ATC_BIST_OV2, 0x660, 0);
	t6050_usb2_mask(atc + ATC_BIST_CFG1, 0, ATC_BIST_CFG1_USB_EN);
	t6050_usb2_mask(atc + ATC_BIST_CFG1, ATC_BIST_CFG1_USB_EN, 0);
	t6050_usb2_mask(pipe + PIPEHANDLER_NONSELECTED_OVERRIDE, GENMASK(3, 0), 3);
	t6050_usb2_mask(pipe + PIPEHANDLER_NONSELECTED_OVERRIDE, BIT(12), 0);

	/* 14. PIPE overrides and lock; the PHY answers the TXDETRX request. */
	t6050_usb2_mask(pipe + PIPEHANDLER_OVERRIDE_VALUES, BIT(1) | BIT(2), 0);
	t6050_usb2_mask(pipe + PIPEHANDLER_OVERRIDE, 0, BIT(0));
	t6050_usb2_mask(pipe + PIPEHANDLER_OVERRIDE, 0, BIT(2));
	t6050_usb2_mask(pipe + PIPEHANDLER_LOCK_REQ, 0, PIPEHANDLER_LOCK);
	lock = t6050_wait(pipe + PIPEHANDLER_LOCK_ACK, PIPEHANDLER_LOCK, PIPEHANDLER_LOCK, 10);
	ack = t6050_wait(p->pctl + PCTL_TXDETRX_ACK, BIT(0), BIT(0), 50);

	/* 15. PIPE to the USB3 PHY, overrides off, unlock. */
	t6050_usb2_mask(pipe + PIPEHANDLER_MUX_CTRL, PIPEHANDLER_MUX_CLK, 0);
	t6050_usb2_mask(pipe + PIPEHANDLER_MUX_CTRL, PIPEHANDLER_MUX_DATA, 0);
	t6050_usb2_mask(pipe + PIPEHANDLER_MUX_CTRL, PIPEHANDLER_MUX_CLK,
			FIELD_PREP(PIPEHANDLER_MUX_CLK, 1));
	t6050_usb2_mask(pipe + PIPEHANDLER_OVERRIDE, BIT(0), 0);
	t6050_usb2_mask(pipe + PIPEHANDLER_OVERRIDE, BIT(2), 0);
	t6050_usb2_mask(pipe + PIPEHANDLER_LOCK_REQ, PIPEHANDLER_LOCK, 0);
	unlock = t6050_wait(pipe + PIPEHANDLER_LOCK_ACK, PIPEHANDLER_LOCK, 0, 10);

	/* 16. TXDETRX lock release. */
	t6050_usb2_mask(p->pctl + PCTL_TXDETRX_REQ, BIT(0), 0);
	rel = t6050_wait(p->pctl + PCTL_TXDETRX_ACK, BIT(0), 0, 50);

	for (i = 0; i < ARRAY_SIZE(t6050_glue); i++)
		p->glue[i] = readl(p->core + t6050_glue[i]);
	val = readl(atc + ATC_BIST_STATUS2);

	dev_info(p->dev, "SS: glue written %#x, HCRST %d CNR %d, CIO %d, mxwrap %d, UC_TNBL_DONE %d\n",
		 glue, hcrst, cnr, cio, mxw, tnbl);
	dev_info(p->dev, "SS: PhyStatus s1 %d rise %d fall %d (STATUS2 %#x), lock %d ack %d unlock %d release %d\n",
		 s1, rise, fall, val, lock, ack, unlock, rel);
	dev_info(p->dev, "SS: glue %#x %#x %#x %#x, PORTSC %#x / %#x\n", p->glue[0], p->glue[1],
		 p->glue[2], p->glue[3], readl(p->core + XHCI_PORTSC1), readl(p->core + XHCI_PORTSC2));
}

/* After the xHCI reset (step 17): the glue back as the PHY sequence left it, host mode. */
static int t6050_usb2_calibrate(struct phy *phy)
{
	struct t6050_usb2 *p = phy_get_drvdata(phy);
	unsigned int i, changed = 0;

	if (!p->ss)
		return 0;
	for (i = 0; i < ARRAY_SIZE(t6050_glue); i++) {
		if (readl(p->core + t6050_glue[i]) == p->glue[i])
			continue;
		writel(p->glue[i], p->core + t6050_glue[i]);
		changed |= BIT(i);
	}
	t6050_usb2_mask(p->core + DWC3_GCTL, DWC3_GCTL_PRTCAPDIR_MASK, DWC3_GCTL_PRTCAP_HOST);
	dev_info(p->dev, "SS: after xHCI reset: glue restored %#x, USBSTS %#x, PORTSC %#x / %#x\n",
		 changed, readl(p->core + XHCI_USBSTS), readl(p->core + XHCI_PORTSC1),
		 readl(p->core + XHCI_PORTSC2));
	return 0;
}

static int t6050_usb2_init(struct phy *phy)
{
	struct t6050_usb2 *p = phy_get_drvdata(phy);
	static const unsigned int wait_ms[] = { 50, 100, 100 };
	unsigned int attempt;
	u32 portsc = 0;

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
	if (p->ss)
		t6050_usb3_up(p);
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

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
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

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
#define XHCI_PORTSC1			0x420
#define XHCI_PORTSC_CCS			BIT(0)

struct t6050_usb2 {
	struct device *dev;
	void __iomem *usb2phy;
	void __iomem *pipehandler;
	void __iomem *core;
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
	return 0;
}

static const struct phy_ops t6050_usb2_ops = {
	.init = t6050_usb2_init,
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

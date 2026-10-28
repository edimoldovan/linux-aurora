/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_USB_TIPD_H
#define __LINUX_USB_TIPD_H

#include <linux/errno.h>
#include <linux/types.h>

struct device_node;

#if IS_REACHABLE(CONFIG_TYPEC_SN201202X)
int sn201202x_simulate_disconnect(struct device_node *np, u8 seconds);
#else
static inline int sn201202x_simulate_disconnect(struct device_node *np, u8 seconds)
{
	return -ENODEV;
}
#endif

#endif /* __LINUX_USB_TIPD_H */

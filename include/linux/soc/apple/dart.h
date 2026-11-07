/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef __LINUX_SOC_APPLE_DART_H__
#define __LINUX_SOC_APPLE_DART_H__

#include <linux/kconfig.h>
#include <linux/errno.h>

struct device;
struct device_node;

/*
 * The tunneled PCIe IOMMU shares the port clock. Stop issuing commands
 * before that clock is gated, and allow them again once it is back.
 */
#if IS_ENABLED(CONFIG_APPLE_DART)
int apple_dart_quiesce_commands(struct device *dev);
int apple_dart_resume_commands(struct device *dev);
int apple_dart_save_tunnel_state(struct device *dev);
int apple_dart_restore_tunnel_state(struct device *dev);
#else
static inline int apple_dart_quiesce_commands(struct device *dev) { return 0; }
static inline int apple_dart_resume_commands(struct device *dev) { return 0; }
static inline int apple_dart_save_tunnel_state(struct device *dev) { return -EOPNOTSUPP; }
static inline int apple_dart_restore_tunnel_state(struct device *dev) { return -EOPNOTSUPP; }
#endif

/*
 * A DART marked apple,manual-availability answers only once its client (a
 * PCIe root complex) is up. The client makes it available, and it probes then.
 */
#if IS_ENABLED(CONFIG_APPLE_DART)
void apple_dart_make_available(struct device_node *np);
#else
static inline void apple_dart_make_available(struct device_node *np) { }
#endif

#endif

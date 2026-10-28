// SPDX-License-Identifier: GPL-2.0

#include <kunit/visibility.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/spmi.h>
#include <linux/usb/tipd.h>

#include "tps6598x.h"

#define tps_to_sn(tps) container_of_const((tps), struct sn201202x, cd.tps)

static int regmap_sn201202x_select_reg(struct spmi_device *sdev, u8 reg)
{
	int err;
	u8 val;
	bool warned = false;
	int attempts = 5;
	struct tps6598x *tps = spmi_device_get_drvdata(sdev);
	struct sn201202x *sn = tps_to_sn(tps);

	if (reg >= 0x80) {
		WARN_ON(1);
		return -EINVAL;
	}
	reinit_completion(&sn->select_completion);
	err = spmi_register_zero_write(sdev, reg);
	if (err)
		return err;

	if (sn->polled) {
		int poll_err;

		poll_err = read_poll_timeout(spmi_register_read, err,
					     err || val == reg, 100, 100000,
					     false, sdev, 0, &val);
		return err ?: poll_err;
	}

	if (!wait_for_completion_timeout(&sn->select_completion, msecs_to_jiffies(100)))
		return -ETIMEDOUT;

	while (--attempts) {
		err = spmi_register_read(sdev, 0, &val);
		if (err)
			return err;
		if (val == (reg | 0x80)) {
			if (!warned) {
				dev_warn(tps->dev,
					 "Got interrupt but selection not complete?\n");
				warned = true;
			}
			msleep(20);
			continue;
		}
		if (val == reg)
			return 0;
		return -EIO;
	}

	return -EIO;
}

static int regmap_sn201202x_read(void *context,
				 const void *reg, size_t reg_size,
				 void *val, size_t val_size)
{
	int err;
	unsigned int offset = 0x20;
	size_t len;
	u8 addr;

	if (reg_size != 1) {
		WARN_ON(1);
		return -EINVAL;
	}
	if (val_size > 0x40) {
		WARN_ON(1);
		return -EINVAL;
	}

	addr = *(u8 *)reg;

	err = regmap_sn201202x_select_reg(context, addr);
	if (err)
		return err;

	while (val_size) {
		len = min_t(size_t, val_size, 16);
		err = spmi_ext_register_read(context, offset, val, len);
		if (err)
			return err;
		offset += len;
		val += len;
		val_size -= len;
	}

	return 0;
}

static int regmap_sn201202x_write(void *context, const void *data,
				  size_t count)
{
	int err = 0;
	unsigned int offset = 0xa0;
	size_t len;
	u8 addr;

	if (count < 1) {
		WARN_ON(1);
		return -EINVAL;
	}

	addr = *(u8 *)data;
	data += 1;
	count -= 1;

	if (count > 0x40) {
		WARN_ON(1);
		return -EINVAL;
	}

	err = regmap_sn201202x_select_reg(context, addr);
	if (err)
		return err;

	while (count) {
		len = min_t(size_t, count, 16);
		err = spmi_ext_register_write(context, offset, data, len);
		if (err)
			return err;
		offset += len;
		data += len;
		count -= len;
	}

	return err;
}

static irqreturn_t sn201202x_irq(int irq, void *data)
{
	struct completion *c = data;

	complete(c);
	return IRQ_HANDLED;
}

static const struct regmap_bus regmap_sn201202x = {
	.read				= regmap_sn201202x_read,
	.write				= regmap_sn201202x_write,
	.reg_format_endian_default	= REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default	= REGMAP_ENDIAN_NATIVE,
};

static struct regmap *__devm_regmap_init_sn201202x(struct spmi_device *sdev,
						   const struct regmap_config *config,
						   struct lock_class_key *lock_key,
						   const char *lock_name)
{
	return __devm_regmap_init(&sdev->dev, &regmap_sn201202x, sdev, config,
				  lock_key, lock_name);
}

#define devm_regmap_init_sn201202x(dev, config)				\
	__regmap_lockdep_wrapper(__devm_regmap_init_sn201202x, #config,	\
				dev, config)

static const struct of_device_id sn201202x_of_match[] = {
	{ .compatible = "apple,sn201202x", (void *)TIPD_VARIANT_SN201202X},
	{}
};

static int sn201202x_wake(struct sn201202x *sn)
{
	unsigned long deadline;
	u8 size;
	int ret;

	reinit_completion(&sn->wake_completion);
	ret = spmi_command_wakeup(sn->sdev);
	if (ret)
		return ret;
	if (!sn->polled)
		return wait_for_completion_timeout(&sn->wake_completion,
						   msecs_to_jiffies(100)) ? 0 : -ETIMEDOUT;

	/* A sleeping ACE3 ACKs commands but returns zeros until it is awake. */
	deadline = jiffies + msecs_to_jiffies(500);
	do {
		ret = regmap_sn201202x_select_reg(sn->sdev, TPS_REG_MODE);
		if (ret == -ETIMEDOUT)
			continue;
		if (ret)
			return ret;
		ret = spmi_register_read(sn->sdev, 0x1f, &size);
		if (ret)
			return ret;
		if (size)
			return 0;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

static int sn201202x_probe(struct spmi_device *device)
{
	const struct of_device_id *match;
	const struct tipd_data *data;
	struct sn201202x *sn;
	struct tps6598x *tps;
	int irq_select, irq_sleep, irq_wake;
	int ret;

	match = of_match_device(sn201202x_of_match, &device->dev);
	if (!match)
		return -EINVAL;
	data = tipd_hw_variants[(enum tipd_hw_variant)match->data];

	sn = devm_kzalloc(&device->dev, data->tps_struct_size, GFP_KERNEL);
	if (!sn)
		return -ENOMEM;
	sn->sdev = device;
	tps = &sn->cd.tps;

	mutex_init(&tps->lock);
	tps->dev = &device->dev;
	tps->data = data;

	init_completion(&sn->select_completion);
	init_completion(&sn->sleep_completion);
	init_completion(&sn->wake_completion);

	sn->polled = !of_property_present(device->dev.of_node, "interrupts") &&
		     !of_property_present(device->dev.of_node, "interrupts-extended");
	if (!sn->polled) {
		tps->irq = of_irq_get_byname(device->dev.of_node, "irq");
		if (tps->irq < 0)
			return tps->irq;
		irq_select = of_irq_get_byname(device->dev.of_node, "select");
		if (irq_select < 0)
			return irq_select;
		irq_sleep = of_irq_get_byname(device->dev.of_node, "sleep");
		if (irq_sleep < 0)
			return irq_sleep;
		irq_wake = of_irq_get_byname(device->dev.of_node, "wake");
		if (irq_wake < 0)
			return irq_wake;

		ret = devm_request_irq(&device->dev, irq_select, sn201202x_irq,
				       0, NULL, &sn->select_completion);
		if (ret)
			return ret;
		ret = devm_request_irq(&device->dev, irq_sleep, sn201202x_irq,
				       0, NULL, &sn->sleep_completion);
		if (ret)
			return ret;
		ret = devm_request_irq(&device->dev, irq_wake, sn201202x_irq,
				       0, NULL, &sn->wake_completion);
		if (ret)
			return ret;
	}

	spmi_device_set_drvdata(device, tps);
	tps->regmap = devm_regmap_init_sn201202x(device, &tps6598x_regmap_config);
	if (IS_ERR(tps->regmap))
		return PTR_ERR(tps->regmap);

	ret = sn201202x_wake(sn);
	if (ret)
		return ret;

	ret = tipd_init(tps);
	if (ret)
		spmi_command_sleep(device);
	return ret;
}

/* MODE, STATUS, POWER_STATUS and DATA_STATUS (0x03, 0x1a, 0x3f, 0x5f), for the log. */
static u32 sn201202x_state(struct tps6598x *tps, const char *when)
{
	u32 mode = 0, status = 0, data = 0;
	u16 power = 0;

	tps6598x_block_read(tps, 0x03, &mode, sizeof(mode));
	tps6598x_block_read(tps, 0x1a, &status, sizeof(status));
	tps6598x_block_read(tps, 0x3f, &power, sizeof(power));
	tps6598x_block_read(tps, 0x5f, &data, sizeof(data));
	dev_info(tps->dev, "%s: mode %.4s, status %#x, power %#x, data %#x\n", when,
		 (char *)&mode, status, power, data);
	return status;
}

/*
 * "DISC" in the debug mode: unlocked with @key ("apple,unlock-key", the ADT
 * target-type's first four characters reversed, as m1n1 does), "DBMa" entered
 * for the command and left at once, while the port is still disconnected.
 * Called with tps->lock held.
 */
static int sn201202x_disc(struct tps6598x *tps, const char *key, u8 seconds)
{
	static const u8 lock_key[4];
	int lock, dbma = -ENODATA, disc = -ENODATA, leave = -ENODATA, relock;
	u32 status, last;
	unsigned int i;
	u8 en = 1;

	sn201202x_state(tps, "before DISC");
	lock = tps6598x_exec_cmd_tmo(tps, "LOCK", 4, (const u8 *)key, 0, NULL, 1000, 0);
	if (!lock)
		dbma = tps6598x_exec_cmd_tmo(tps, "DBMa", sizeof(en), &en, 0, NULL, 1000, 0);
	if (!lock && !dbma) {
		disc = tps6598x_exec_cmd_tmo(tps, "DISC", sizeof(seconds), &seconds, 0, NULL,
					     1000 * (seconds + 1), 0);
		sn201202x_state(tps, "DISC done");
		en = 0;
		leave = tps6598x_exec_cmd_tmo(tps, "DBMa", sizeof(en), &en, 0, NULL, 1000, 0);
	}
	relock = tps6598x_exec_cmd_tmo(tps, "LOCK", sizeof(lock_key), lock_key, 0, NULL, 1000, 0);
	dev_info(tps->dev, "simulated disconnect: LOCK %d, DBMa %d, DISC %d, DBMa 0 %d, LOCK 0 %d\n",
		 lock, dbma, disc, leave, relock);

	/* Diagnostic: every STATUS change for 25 s. */
	last = sn201202x_state(tps, "locked again");
	for (i = 0; i < 50; i++) {
		msleep(500);
		if (!tps6598x_block_read(tps, 0x1a, &status, sizeof(status)) && status != last)
			last = sn201202x_state(tps, "after");
	}
	return lock ?: dbma ?: disc;
}

/*
 * Simulated disconnect ("DISC"): the port drops its partner, VBUS included, for
 * @seconds and then sees it again as freshly plugged in. -EPROBE_DEFER until
 * the controller at @np is bound.
 */
int sn201202x_simulate_disconnect(struct device_node *np, u8 seconds)
{
	struct spmi_device *sdev = spmi_find_device_by_of_node(np);
	struct tps6598x *tps;
	const char *key;
	int ret = -EPROBE_DEFER;

	if (!sdev)
		return ret;
	if (device_is_bound(&sdev->dev)) {
		tps = spmi_device_get_drvdata(sdev);
		guard(mutex)(&tps->lock);
		if (!tps->dbma_at_probe && !of_property_read_string(np, "apple,unlock-key", &key) &&
		    strlen(key) == 4)
			ret = sn201202x_disc(tps, key, seconds);
		else
			ret = tps6598x_exec_cmd_tmo(tps, "DISC", sizeof(seconds), &seconds, 0,
						    NULL, 1000, 0);
	}
	put_device(&sdev->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(sn201202x_simulate_disconnect);

static void sn201202x_remove(struct spmi_device *device)
{
	struct tps6598x *tps = spmi_device_get_drvdata(device);
	struct sn201202x *sn = tps_to_sn(tps);

	tipd_remove(tps);
	spmi_command_sleep(sn->sdev);
}

VISIBLE_IF_KUNIT int __maybe_unused sn201202x_resume(struct device *dev)
{
	struct tps6598x *tps = dev_get_drvdata(dev);
	struct sn201202x *sn = tps_to_sn(tps);
	int err;

	err = sn201202x_wake(sn);
	if (!err)
		err = tipd_resume(tps);
	/* The resume revalidation acknowledges events latched while asleep. */
	if (tps->irq)
		enable_irq(tps->irq);
	return err;
}
EXPORT_SYMBOL_IF_KUNIT(sn201202x_resume);

VISIBLE_IF_KUNIT int __maybe_unused sn201202x_suspend(struct device *dev)
{
	struct tps6598x *tps = dev_get_drvdata(dev);
	struct sn201202x *sn = tps_to_sn(tps);
	int err, ret;

	/*
	 * A sleeping controller cannot be read, so its events must not be
	 * handled until it is awake again. An interrupt arriving meanwhile is
	 * replayed when it is enabled.
	 */
	if (tps->irq)
		disable_irq(tps->irq);
	err = tipd_suspend(tps);
	if (err)
		goto out_enable;
	reinit_completion(&sn->sleep_completion);
	err = spmi_command_sleep(sn->sdev);
	if (err)
		goto out_resume;
	if (!sn->polled &&
	    !wait_for_completion_timeout(&sn->sleep_completion, msecs_to_jiffies(100))) {
		err = -ETIMEDOUT;
		goto out_wake;
	}
	return 0;

out_wake:
	/* It accepted SLEEP, so it may be asleep without having confirmed it. */
	ret = sn201202x_wake(sn);
	if (ret)
		dev_err(dev, "failed to wake up after failed suspend: %d\n", ret);
out_resume:
	tipd_resume(tps);
out_enable:
	if (tps->irq)
		enable_irq(tps->irq);
	return err;
}
EXPORT_SYMBOL_IF_KUNIT(sn201202x_suspend);

MODULE_DEVICE_TABLE(of, sn201202x_of_match);

static const struct dev_pm_ops sn201202x_pm_ops = {
	.prepare = pm_sleep_ptr(tipd_prepare),
	.complete = pm_sleep_ptr(tipd_complete),
	SET_SYSTEM_SLEEP_PM_OPS(sn201202x_suspend, sn201202x_resume)
};

static struct spmi_driver sn201202x_driver = {
	.driver = {
		.name = "sn201202x",
		.pm = &sn201202x_pm_ops,
		.of_match_table = sn201202x_of_match,
	},
	.probe = sn201202x_probe,
	.remove = sn201202x_remove,
};
module_spmi_driver(sn201202x_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI SN201202x USB Power Delivery Controller Driver");

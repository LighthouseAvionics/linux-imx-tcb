// SPDX-License-Identifier: GPL-2.0-only
/*
 * Orin Control - Power and recovery control for Jetson Orin modules
 *
 * Copyright (C) 2025 Lighthouse Avionics
 *
 * Controls 8 Jetson Orin modules (7 real + 1 aux) via two PCA9555
 * GPIO expanders. Provides sysfs interface for power on/off and
 * recovery mode assertion.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>

#define NUM_ORINS	8
#define NUM_RECOVERY	6
#define DRIVER_NAME	"orin-control"

struct orin_control {
	struct device *dev;
	struct gpio_desc *power[NUM_ORINS];
	struct gpio_desc *recovery[NUM_RECOVERY];
};

static int orin_attr_index(const char *name)
{
	int idx = -1;

	if (sscanf(name, "orin%d_", &idx) != 1)
		return -EINVAL;
	return idx;
}

/* --- power (read/write) --- */

static ssize_t orin_power_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct orin_control *oc = dev_get_drvdata(dev);
	int idx = orin_attr_index(attr->attr.name);

	if (idx < 0 || idx >= NUM_ORINS)
		return -EINVAL;

	return sysfs_emit(buf, "%d\n", gpiod_get_value(oc->power[idx]));
}

static ssize_t orin_power_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct orin_control *oc = dev_get_drvdata(dev);
	int idx = orin_attr_index(attr->attr.name);
	unsigned int val;

	if (idx < 0 || idx >= NUM_ORINS)
		return -EINVAL;
	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	gpiod_set_value(oc->power[idx], !!val);
	return count;
}

/* --- recovery (write-only, orins 0-5 only) --- */

static ssize_t orin_recovery_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct orin_control *oc = dev_get_drvdata(dev);
	int idx = orin_attr_index(attr->attr.name);
	unsigned int val;

	if (idx < 0 || idx >= NUM_RECOVERY)
		return -EINVAL;
	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	gpiod_set_value(oc->recovery[idx], !!val);
	return count;
}

/* --- state (read-only) --- */

static ssize_t orin_state_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct orin_control *oc = dev_get_drvdata(dev);
	int idx = orin_attr_index(attr->attr.name);
	int pwr;

	if (idx < 0 || idx >= NUM_ORINS)
		return -EINVAL;

	pwr = gpiod_get_value(oc->power[idx]);

	if (idx < NUM_RECOVERY)
		return sysfs_emit(buf, "power=%d recovery=%d\n",
				  pwr, gpiod_get_value(oc->recovery[idx]));

	return sysfs_emit(buf, "power=%d recovery=n/a\n", pwr);
}

/* --- attribute declarations --- */

#define ORIN_POWER_ATTR_RW(_n) \
	static DEVICE_ATTR(orin##_n##_power, 0644, \
			   orin_power_show, orin_power_store)

#define ORIN_RECOVERY_ATTR_WO(_n) \
	static DEVICE_ATTR(orin##_n##_recovery, 0200, \
			   NULL, orin_recovery_store)

#define ORIN_STATE_ATTR_RO(_n) \
	static DEVICE_ATTR(orin##_n##_state, 0444, \
			   orin_state_show, NULL)

ORIN_POWER_ATTR_RW(0);
ORIN_POWER_ATTR_RW(1);
ORIN_POWER_ATTR_RW(2);
ORIN_POWER_ATTR_RW(3);
ORIN_POWER_ATTR_RW(4);
ORIN_POWER_ATTR_RW(5);
ORIN_POWER_ATTR_RW(6);
ORIN_POWER_ATTR_RW(7);

ORIN_RECOVERY_ATTR_WO(0);
ORIN_RECOVERY_ATTR_WO(1);
ORIN_RECOVERY_ATTR_WO(2);
ORIN_RECOVERY_ATTR_WO(3);
ORIN_RECOVERY_ATTR_WO(4);
ORIN_RECOVERY_ATTR_WO(5);

ORIN_STATE_ATTR_RO(0);
ORIN_STATE_ATTR_RO(1);
ORIN_STATE_ATTR_RO(2);
ORIN_STATE_ATTR_RO(3);
ORIN_STATE_ATTR_RO(4);
ORIN_STATE_ATTR_RO(5);
ORIN_STATE_ATTR_RO(6);
ORIN_STATE_ATTR_RO(7);

static struct attribute *orin_control_attrs[] = {
	&dev_attr_orin0_power.attr,
	&dev_attr_orin1_power.attr,
	&dev_attr_orin2_power.attr,
	&dev_attr_orin3_power.attr,
	&dev_attr_orin4_power.attr,
	&dev_attr_orin5_power.attr,
	&dev_attr_orin6_power.attr,
	&dev_attr_orin7_power.attr,
	&dev_attr_orin0_recovery.attr,
	&dev_attr_orin1_recovery.attr,
	&dev_attr_orin2_recovery.attr,
	&dev_attr_orin3_recovery.attr,
	&dev_attr_orin4_recovery.attr,
	&dev_attr_orin5_recovery.attr,
	&dev_attr_orin0_state.attr,
	&dev_attr_orin1_state.attr,
	&dev_attr_orin2_state.attr,
	&dev_attr_orin3_state.attr,
	&dev_attr_orin4_state.attr,
	&dev_attr_orin5_state.attr,
	&dev_attr_orin6_state.attr,
	&dev_attr_orin7_state.attr,
	NULL
};

ATTRIBUTE_GROUPS(orin_control);

/* --- probe --- */

static int orin_control_probe(struct platform_device *pdev)
{
	struct orin_control *oc;
	struct device *dev = &pdev->dev;
	int i, ret;

	oc = devm_kzalloc(dev, sizeof(*oc), GFP_KERNEL);
	if (!oc)
		return -ENOMEM;

	oc->dev = dev;

	for (i = 0; i < NUM_ORINS; i++) {
		oc->power[i] = devm_gpiod_get_index(dev, "power", i,
						     GPIOD_OUT_LOW);
		if (IS_ERR(oc->power[i])) {
			ret = PTR_ERR(oc->power[i]);
			dev_err(dev, "failed to get power gpio %d: %d\n",
				i, ret);
			return ret;
		}
	}

	for (i = 0; i < NUM_RECOVERY; i++) {
		oc->recovery[i] = devm_gpiod_get_index(dev, "recovery", i,
							GPIOD_OUT_LOW);
		if (IS_ERR(oc->recovery[i])) {
			ret = PTR_ERR(oc->recovery[i]);
			dev_err(dev, "failed to get recovery gpio %d: %d\n",
				i, ret);
			return ret;
		}
	}

	platform_set_drvdata(pdev, oc);

	dev_info(dev, "initialized (%d power, %d recovery)\n",
		 NUM_ORINS, NUM_RECOVERY);
	return 0;
}

static const struct of_device_id orin_control_of_match[] = {
	{ .compatible = "lha,orin-control" },
	{ }
};
MODULE_DEVICE_TABLE(of, orin_control_of_match);

static struct platform_driver orin_control_driver = {
	.probe = orin_control_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = orin_control_of_match,
		.dev_groups = orin_control_groups,
	},
};
module_platform_driver(orin_control_driver);

MODULE_AUTHOR("Lighthouse Avionics");
MODULE_DESCRIPTION("Power and recovery control for Jetson Orin modules");
MODULE_LICENSE("GPL");

// SPDX-License-Identifier: GPL-2.0-only
/*
 * TCB Relay Control - Power relay control for TCB peripherals
 *
 * Copyright (C) 2025 Lighthouse Avionics
 *
 * Controls peripheral power relays (PTZ camera, heater, 48V rail)
 * via GPIO expanders. Each relay is independently controllable and
 * readable. Default state is configured per relay in the device tree.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>

#define DRIVER_NAME "tcb-relays"

struct tcb_relay_info {
	const char *attr_name;     /* sysfs attribute name */
	const char *con_id;        /* GPIO consumer id ("ptz" → ptz-gpios) */
	const char *default_prop;  /* DT boolean: "<name>-default-on" */
};

static const struct tcb_relay_info tcb_relays[] = {
	{ "ptz_power",    "ptz",    "ptz-default-on"    },
	{ "heater_power", "heater", "heater-default-on" },
	{ "v48_power",    "v48",    "v48-default-on"    },
};

#define NUM_RELAYS ARRAY_SIZE(tcb_relays)

struct tcb_relay_ctx {
	struct gpio_desc *gpio[NUM_RELAYS];
};

static int relay_index_from_name(const char *name)
{
	int i;

	for (i = 0; i < NUM_RELAYS; i++)
		if (!strcmp(name, tcb_relays[i].attr_name))
			return i;
	return -EINVAL;
}

static ssize_t relay_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct tcb_relay_ctx *ctx = dev_get_drvdata(dev);
	int idx = relay_index_from_name(attr->attr.name);

	if (idx < 0)
		return idx;

	return sysfs_emit(buf, "%d\n", gpiod_get_value(ctx->gpio[idx]));
}

static ssize_t relay_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct tcb_relay_ctx *ctx = dev_get_drvdata(dev);
	int idx = relay_index_from_name(attr->attr.name);
	unsigned int val;

	if (idx < 0)
		return idx;
	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	gpiod_set_value(ctx->gpio[idx], !!val);
	return count;
}

static DEVICE_ATTR(ptz_power,    0644, relay_show, relay_store);
static DEVICE_ATTR(heater_power, 0644, relay_show, relay_store);
static DEVICE_ATTR(v48_power,    0644, relay_show, relay_store);

static struct attribute *tcb_relays_attrs[] = {
	&dev_attr_ptz_power.attr,
	&dev_attr_heater_power.attr,
	&dev_attr_v48_power.attr,
	NULL
};

ATTRIBUTE_GROUPS(tcb_relays);

static int tcb_relays_probe(struct platform_device *pdev)
{
	struct tcb_relay_ctx *ctx;
	struct device *dev = &pdev->dev;
	int i;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	for (i = 0; i < NUM_RELAYS; i++) {
		bool default_on = of_property_read_bool(dev->of_node,
						tcb_relays[i].default_prop);
		enum gpiod_flags flags = default_on ? GPIOD_OUT_HIGH
						    : GPIOD_OUT_LOW;

		ctx->gpio[i] = devm_gpiod_get(dev, tcb_relays[i].con_id, flags);
		if (IS_ERR(ctx->gpio[i]))
			return dev_err_probe(dev, PTR_ERR(ctx->gpio[i]),
					     "failed to get %s gpio\n",
					     tcb_relays[i].con_id);

		dev_info(dev, "%s initialized: %s\n",
			 tcb_relays[i].attr_name,
			 default_on ? "on" : "off");
	}

	platform_set_drvdata(pdev, ctx);
	return 0;
}

static const struct of_device_id tcb_relays_of_match[] = {
	{ .compatible = "lha,tcb-relays" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcb_relays_of_match);

static struct platform_driver tcb_relays_driver = {
	.probe = tcb_relays_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = tcb_relays_of_match,
		.dev_groups = tcb_relays_groups,
	},
};
module_platform_driver(tcb_relays_driver);

MODULE_AUTHOR("Lighthouse Avionics");
MODULE_DESCRIPTION("TCB peripheral relay control");
MODULE_LICENSE("GPL");

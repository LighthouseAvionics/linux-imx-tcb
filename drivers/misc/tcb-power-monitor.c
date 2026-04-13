// SPDX-License-Identifier: GPL-2.0-only
/*
 * TCB Power Monitor - System power monitoring for TCB board
 *
 * Copyright (C) 2025 Lighthouse Avionics
 *
 * Reads 4 channels from an ADS1015 ADC via the IIO subsystem and
 * exposes scaled voltage/current readings through hwmon.
 *
 * Hardware:
 *   AIN0 — Vin (input voltage)
 *           divider 15k/1k  → scale = ×16 to get mV
 *   AIN1 — 24V input current via INA186A3 (100V/V, 2mΩ shunt)
 *           divider 20k/12k → scale = ×(40/3) to get mA
 *   AIN2 — 3.8V rail
 *           divider 15k/10k → scale = ×(5/2) to get mV
 *   AIN3 — 3.3V rail
 *           divider 15k/12k → scale = ×(9/4) to get mV
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/iio/consumer.h>
#include <linux/of.h>

#define DRIVER_NAME "tcb-power-monitor"

struct tcb_power_monitor {
	struct device *dev;
	struct iio_channel *vin;
	struct iio_channel *current_24v;
	struct iio_channel *v3v8;
	struct iio_channel *v3v3;
};

static int tcb_read_voltage_scaled(struct iio_channel *chan,
				   int num, int den, long *val)
{
	int raw, ret;

	ret = iio_read_channel_raw(chan, &raw);
	if (ret < 0)
		return ret;

	/* Scale is 1 mV/LSB (PGA=2, 12-bit), so raw is in mV */
	*val = (long)raw * num / den;
	return 0;
}

static int tcb_power_read(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long *val)
{
	struct tcb_power_monitor *tp = dev_get_drvdata(dev);
	int raw, ret;

	if (attr != hwmon_in_input && attr != hwmon_curr_input)
		return -EOPNOTSUPP;

	switch (type) {
	case hwmon_in:
		switch (channel) {
		case 0: /* Vin: ×16 */
			ret = iio_read_channel_raw(tp->vin, &raw);
			dev_info(dev, "vin: raw=%d ret=%d\n", raw, ret);
			if (ret < 0)
				return ret;
			*val = (long)raw * 16;
			return 0;
		case 1: /* 3.8V: ×5/2 */
			ret = iio_read_channel_raw(tp->v3v8, &raw);
			dev_info(dev, "3v8: raw=%d ret=%d\n", raw, ret);
			if (ret < 0)
				return ret;
			*val = (long)raw * 5 / 2;
			return 0;
		case 2: /* 3.3V: ×9/4 */
			ret = iio_read_channel_raw(tp->v3v3, &raw);
			dev_info(dev, "3v3: raw=%d ret=%d\n", raw, ret);
			if (ret < 0)
				return ret;
			*val = (long)raw * 9 / 4;
			return 0;
		default:
			return -EOPNOTSUPP;
		}
	case hwmon_curr:
		if (channel == 0) {
			ret = iio_read_channel_raw(tp->current_24v, &raw);
			dev_info(dev, "24v_current: raw=%d ret=%d\n", raw, ret);
			if (ret < 0)
				return ret;
			*val = (long)raw * 40 / 3;
			return 0;
		}
		return -EOPNOTSUPP;
	default:
		return -EOPNOTSUPP;
	}
}

static int tcb_power_read_string(struct device *dev,
				 enum hwmon_sensor_types type,
				 u32 attr, int channel, const char **str)
{
	static const char * const voltage_labels[] = {
		"vin", "3v8", "3v3"
	};

	switch (type) {
	case hwmon_in:
		if (attr == hwmon_in_label && channel < 3) {
			*str = voltage_labels[channel];
			return 0;
		}
		break;
	case hwmon_curr:
		if (attr == hwmon_curr_label && channel == 0) {
			*str = "24v_current";
			return 0;
		}
		break;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static umode_t tcb_power_is_visible(const void *data,
				     enum hwmon_sensor_types type,
				     u32 attr, int channel)
{
	switch (type) {
	case hwmon_in:
		if (channel < 3 &&
		    (attr == hwmon_in_input || attr == hwmon_in_label))
			return 0444;
		break;
	case hwmon_curr:
		if (channel == 0 &&
		    (attr == hwmon_curr_input || attr == hwmon_curr_label))
			return 0444;
		break;
	default:
		break;
	}
	return 0;
}

static const struct hwmon_channel_info * const tcb_power_info[] = {
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LABEL),
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_INPUT | HWMON_C_LABEL),
	NULL
};

static const struct hwmon_ops tcb_power_ops = {
	.is_visible = tcb_power_is_visible,
	.read = tcb_power_read,
	.read_string = tcb_power_read_string,
};

static const struct hwmon_chip_info tcb_power_chip_info = {
	.ops = &tcb_power_ops,
	.info = tcb_power_info,
};

static int tcb_power_monitor_probe(struct platform_device *pdev)
{
	struct tcb_power_monitor *tp;
	struct device *dev = &pdev->dev;
	struct device *hwmon;

	tp = devm_kzalloc(dev, sizeof(*tp), GFP_KERNEL);
	if (!tp)
		return -ENOMEM;

	tp->dev = dev;

	tp->vin = devm_iio_channel_get(dev, "vin");
	if (IS_ERR(tp->vin))
		return dev_err_probe(dev, PTR_ERR(tp->vin),
				     "failed to get vin channel\n");

	tp->current_24v = devm_iio_channel_get(dev, "24v_current");
	if (IS_ERR(tp->current_24v))
		return dev_err_probe(dev, PTR_ERR(tp->current_24v),
				     "failed to get 24v_current channel\n");

	tp->v3v8 = devm_iio_channel_get(dev, "3v8");
	if (IS_ERR(tp->v3v8))
		return dev_err_probe(dev, PTR_ERR(tp->v3v8),
				     "failed to get 3v8 channel\n");

	tp->v3v3 = devm_iio_channel_get(dev, "3v3");
	if (IS_ERR(tp->v3v3))
		return dev_err_probe(dev, PTR_ERR(tp->v3v3),
				     "failed to get 3v3 channel\n");

	hwmon = devm_hwmon_device_register_with_info(dev, "tcb_power", tp,
						     &tcb_power_chip_info,
						     NULL);
	if (IS_ERR(hwmon))
		return dev_err_probe(dev, PTR_ERR(hwmon),
				     "failed to register hwmon\n");

	dev_info(dev, "TCB power monitor initialized\n");
	return 0;
}

static const struct of_device_id tcb_power_monitor_of_match[] = {
	{ .compatible = "lha,tcb-power-monitor" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcb_power_monitor_of_match);

static struct platform_driver tcb_power_monitor_driver = {
	.probe = tcb_power_monitor_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = tcb_power_monitor_of_match,
	},
};
module_platform_driver(tcb_power_monitor_driver);

MODULE_AUTHOR("Lighthouse Avionics");
MODULE_DESCRIPTION("TCB board system power monitor via ADS1015");
MODULE_LICENSE("GPL");

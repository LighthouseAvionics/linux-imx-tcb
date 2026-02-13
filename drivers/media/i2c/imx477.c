// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony imx477 Camera Sensor Driver
 *
 * Copyright (C) 2021 Intel Corporation (original imx412 base)
 * Modified for IMX477 2-lane operation
 */
#include <linux/unaligned.h>

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Streaming Mode */
#define IMX477_REG_MODE_SELECT	0x0100
#define IMX477_MODE_STANDBY	0x00
#define IMX477_MODE_STREAMING	0x01

/* Lines per frame */
#define IMX477_REG_LPFR		0x0340

/* Chip ID */
#define IMX477_REG_ID		0x0016
#define IMX477_ID		0x0477

/* Exposure control */
#define IMX477_REG_EXPOSURE_CIT	0x0202
#define IMX477_EXPOSURE_MIN	8
#define IMX477_EXPOSURE_OFFSET	22
#define IMX477_EXPOSURE_STEP	1
#define IMX477_EXPOSURE_DEFAULT	0x0648

/* Analog gain control */
#define IMX477_REG_AGAIN	0x0204
#define IMX477_AGAIN_MIN	0
#define IMX477_AGAIN_MAX	978
#define IMX477_AGAIN_STEP	1
#define IMX477_AGAIN_DEFAULT	0

/* Group hold register */
#define IMX477_REG_HOLD		0x0104

/* Input clock rate */
#define IMX477_INCLK_RATE	24000000

/* CSI2 HW configuration */
#define IMX477_LINK_FREQ	450000000
#define IMX477_NUM_DATA_LANES	2

#define IMX477_REG_MIN		0x00
#define IMX477_REG_MAX		0xffff

/**
 * struct imx477_reg - imx477 sensor register
 * @address: Register address
 * @val: Register value
 */
struct imx477_reg {
	u16 address;
	u8 val;
};

/**
 * struct imx477_reg_list - imx477 sensor register list
 * @num_of_regs: Number of registers in the list
 * @regs: Pointer to register list
 */
struct imx477_reg_list {
	u32 num_of_regs;
	const struct imx477_reg *regs;
};

/**
 * struct imx477_mode - imx477 sensor mode structure
 * @width: Frame width
 * @height: Frame height
 * @code: Format code
 * @hblank: Horizontal blanking in lines
 * @vblank: Vertical blanking in lines
 * @vblank_min: Minimum vertical blanking in lines
 * @vblank_max: Maximum vertical blanking in lines
 * @pclk: Sensor pixel clock
 * @link_freq_idx: Link frequency index
 * @reg_list: Register list for sensor mode
 */
struct imx477_mode {
	u32 width;
	u32 height;
	u32 code;
	u32 hblank;
	u32 vblank;
	u32 vblank_min;
	u32 vblank_max;
	u64 pclk;
	u32 link_freq_idx;
	struct imx477_reg_list reg_list;
};

static const char * const imx477_supply_names[] = {
	"dovdd",	/* Digital I/O power */
	"avdd",		/* Analog power */
	"dvdd",		/* Digital core power */
};

/**
 * struct imx477 - imx477 sensor device structure
 * @dev: Pointer to generic device
 * @client: Pointer to i2c client
 * @sd: V4L2 sub-device
 * @pad: Media pad. Only one pad supported
 * @reset_gpio: Sensor reset gpio
 * @inclk: Sensor input clock
 * @supplies: Regulator supplies
 * @ctrl_handler: V4L2 control handler
 * @link_freq_ctrl: Pointer to link frequency control
 * @pclk_ctrl: Pointer to pixel clock control
 * @hblank_ctrl: Pointer to horizontal blanking control
 * @vblank_ctrl: Pointer to vertical blanking control
 * @exp_ctrl: Pointer to exposure control
 * @again_ctrl: Pointer to analog gain control
 * @vblank: Vertical blanking in lines
 * @cur_mode: Pointer to current selected sensor mode
 * @mutex: Mutex for serializing sensor controls
 * @debugfs_dir: Debugfs directory for register access
 */
struct imx477 {
	struct device *dev;
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct gpio_desc *reset_gpio;
	struct clk *inclk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx477_supply_names)];
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq_ctrl;
	struct v4l2_ctrl *pclk_ctrl;
	struct v4l2_ctrl *hblank_ctrl;
	struct v4l2_ctrl *vblank_ctrl;
	struct {
		struct v4l2_ctrl *exp_ctrl;
		struct v4l2_ctrl *again_ctrl;
	};
	u32 vblank;
	const struct imx477_mode *cur_mode;
	struct mutex mutex;
	struct dentry *debugfs_dir;
};

static const s64 link_freq[] = {
	IMX477_LINK_FREQ,
};

/* Sensor mode registers for 2028x1520 @ 20fps, 2-lane MIPI */
static const struct imx477_reg mode_2028x1520_regs[] = {
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x3c7e, 0x08},
	{0x3c7f, 0x02},
	{0x38a8, 0x1f},
	{0x38a9, 0xff},
	{0x38aa, 0x1f},
	{0x38ab, 0xff},
	{0x55d4, 0x00},
	{0x55d5, 0x00},
	{0x55d6, 0x07},
	{0x55d7, 0xff},
	{0x55e8, 0x07},
	{0x55e9, 0xff},
	{0x55ea, 0x00},
	{0x55eb, 0x00},
	{0x575c, 0x07},
	{0x575d, 0xff},
	{0x575e, 0x00},
	{0x575f, 0x00},
	{0x5764, 0x00},
	{0x5765, 0x00},
	{0x5766, 0x07},
	{0x5767, 0xff},
	{0x5974, 0x04},
	{0x5975, 0x01},
	{0x5f10, 0x09},
	{0x5f11, 0x92},
	{0x5f12, 0x32},
	{0x5f13, 0x72},
	{0x5f14, 0x16},
	{0x5f15, 0xba},
	{0x5f17, 0x13},
	{0x5f18, 0x24},
	{0x5f19, 0x60},
	{0x5f1a, 0xe3},
	{0x5f1b, 0xad},
	{0x5f1c, 0x74},
	{0x5f2d, 0x25},
	{0x5f5c, 0xd0},
	{0x6a22, 0x00},
	{0x6a23, 0x1d},
	{0x7ba8, 0x00},
	{0x7ba9, 0x00},
	{0x886b, 0x00},
	{0x9002, 0x0a},
	{0x9004, 0x1a},
	{0x9214, 0x93},
	{0x9215, 0x69},
	{0x9216, 0x93},
	{0x9217, 0x6b},
	{0x9218, 0x93},
	{0x9219, 0x6d},
	{0x921a, 0x57},
	{0x921b, 0x58},
	{0x921c, 0x57},
	{0x921d, 0x59},
	{0x921e, 0x57},
	{0x921f, 0x5a},
	{0x9220, 0x57},
	{0x9221, 0x5b},
	{0x9222, 0x93},
	{0x9223, 0x02},
	{0x9224, 0x93},
	{0x9225, 0x03},
	{0x9226, 0x93},
	{0x9227, 0x04},
	{0x9228, 0x93},
	{0x9229, 0x05},
	{0x922a, 0x98},
	{0x922b, 0x21},
	{0x922c, 0xb2},
	{0x922d, 0xdb},
	{0x922e, 0xb2},
	{0x922f, 0xdc},
	{0x9230, 0xb2},
	{0x9231, 0xdd},
	{0x9232, 0xe2},
	{0x9233, 0xe1},
	{0x9234, 0xb2},
	{0x9235, 0xe2},
	{0x9236, 0xb2},
	{0x9237, 0xe3},
	{0x9238, 0xb7},
	{0x9239, 0xb9},
	{0x923a, 0xb7},
	{0x923b, 0xbb},
	{0x923c, 0xb7},
	{0x923d, 0xbc},
	{0x923e, 0xb7},
	{0x923f, 0xc5},
	{0x9240, 0xb7},
	{0x9241, 0xc7},
	{0x9242, 0xb7},
	{0x9243, 0xc9},
	{0x9244, 0x98},
	{0x9245, 0x56},
	{0x9246, 0x98},
	{0x9247, 0x55},
	{0x9380, 0x00},
	{0x9381, 0x62},
	{0x9382, 0x00},
	{0x9383, 0x56},
	{0x9384, 0x00},
	{0x9385, 0x52},
	{0x9388, 0x00},
	{0x9389, 0x55},
	{0x938a, 0x00},
	{0x938b, 0x55},
	{0x938c, 0x00},
	{0x938d, 0x41},
	{0x5078, 0x01},
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x01},  /* CSI_LANE_MODE: 2 lanes */
	{0x0342, 0x0b},
	{0x0343, 0xa0},
	{0x0340, 0x0d},  /* Frame length: 3528 lines for 20 fps */
	{0x0341, 0xc8},
	{0x0350, 0x00},  /* Frame length control: fixed (disable auto tracking) */
	{0x3210, 0x00},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0b},
	{0x034b, 0xdf},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00e5, 0x01},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0xe013, 0x00},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},  /* BINNING_MODE: enabled */
	{0x0901, 0x22},  /* BINNING_TYPE: 2x2 */
	{0x0902, 0x02},  /* BINNING_WEIGHTING: averaging */
	{0x3140, 0x02},
	{0x3241, 0x11},
	{0x3250, 0x03},
	{0x3e10, 0x00},
	{0x3e11, 0x00},
	{0x3f0d, 0x00},
	{0x3f42, 0x00},
	{0x3f43, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x07},  /* Digital crop width: 2028 */
	{0x040d, 0xec},
	{0x040e, 0x05},  /* Digital crop height: 1520 */
	{0x040f, 0xf0},
	{0x034c, 0x07},  /* X output size: 2028 */
	{0x034d, 0xec},
	{0x034e, 0x05},  /* Y output size: 1520 */
	{0x034f, 0xf0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0a},
	{0x030b, 0x01},
	{0x030d, 0x02},
	{0x030e, 0x00},  /* IOP PLL multiplier high: 75 for 450 MHz */
	{0x030f, 0x4b},  /* IOP PLL multiplier low: 75 (24/2*75=900MHz) */
	{0x0310, 0x00},
	{0x0820, 0x07},  /* Link bit rate: 1800 Mbps total (900 Mbps/lane) */
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x00},
	{0x3f57, 0x56},
	{0x3c0a, 0x73},  /* D-PHY timing (conservative for 400 Mbps) */
	{0x3c0b, 0x64},
	{0x3c0c, 0x5f},
	{0x3c0d, 0x00},
	{0x3c0e, 0x00},
	{0x3c0f, 0x00},
	{0x3c10, 0xa4},
	{0x3c11, 0x02},
	{0x3c12, 0x00},
	{0x3c13, 0x03},
	{0x3c14, 0x80},
	{0x3c15, 0x04},
	{0x3c16, 0x15},
	{0x3c17, 0x15},
	{0x3c18, 0x15},
	{0x3c19, 0x15},
	{0x3c1a, 0x15},
	{0x3c1b, 0x15},
	{0x3c1c, 0x06},
	{0x3c1d, 0x06},
	{0x3c1e, 0x06},
	{0x3c1f, 0x06},
	{0x3c20, 0x06},
	{0x3c21, 0x06},
	{0x3c22, 0x3f},
	{0x3c23, 0x0a},
	{0x3e35, 0x01},
	{0x3f4a, 0x01},
	{0x3f4b, 0x7f},
	{0x3f26, 0x00},
	{0x0202, 0x0d},  /* Coarse integration time: 3506 lines */
	{0x0203, 0xb2},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020e, 0x01},
	{0x020f, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0xbcf1, 0x00},
	{0xe000, 0x00},  /* FRAME_BLANKSTOP_CL: keep clock active (LP-11) */
};

/* Supported sensor mode configurations */
static const struct imx477_mode supported_mode = {
	.width = 2028,
	.height = 1520,
	.hblank = 948,
	.vblank = 2008,
	.vblank_min = 2008,
	.vblank_max = 32420,
	.pclk = 210000000,
	.link_freq_idx = 0,
	.code = MEDIA_BUS_FMT_SRGGB10_1X10,
	.reg_list = {
		.num_of_regs = ARRAY_SIZE(mode_2028x1520_regs),
		.regs = mode_2028x1520_regs,
	},
};

/**
 * to_imx477() - imx477 V4L2 sub-device to imx477 device.
 * @subdev: pointer to imx477 V4L2 sub-device
 *
 * Return: pointer to imx477 device
 */
static inline struct imx477 *to_imx477(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct imx477, sd);
}

/**
 * imx477_read_reg() - Read registers.
 * @imx477: pointer to imx477 device
 * @reg: register address
 * @len: length of bytes to read. Max supported bytes is 4
 * @val: pointer to register value to be filled.
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_read_reg(struct imx477 *imx477, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	struct i2c_msg msgs[2] = {0};
	u8 addr_buf[2] = {0};
	u8 data_buf[4] = {0};
	int ret;

	if (WARN_ON(len > 4))
		return -EINVAL;

	put_unaligned_be16(reg, addr_buf);

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/**
 * imx477_write_reg() - Write register
 * @imx477: pointer to imx477 device
 * @reg: register address
 * @len: length of bytes. Max supported bytes is 4
 * @val: register value
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_write_reg(struct imx477 *imx477, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	u8 buf[6] = {0};

	if (WARN_ON(len > 4))
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/**
 * imx477_write_regs() - Write a list of registers
 * @imx477: pointer to imx477 device
 * @regs: list of registers to be written
 * @len: length of registers array
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_write_regs(struct imx477 *imx477,
			     const struct imx477_reg *regs, u32 len)
{
	unsigned int i;
	int ret;

	dev_info(imx477->dev, "Writing %u mode registers\n", len);

	for (i = 0; i < len; i++) {
		ret = imx477_write_reg(imx477, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err(imx477->dev, "Failed to write reg 0x%04X = 0x%02X\n",
				regs[i].address, regs[i].val);
			return ret;
		}
		/* Log critical registers */
		if (regs[i].address == 0x0114 || regs[i].address == 0x0340 ||
		    regs[i].address == 0x030e || regs[i].address == 0x0900) {
			dev_info(imx477->dev, "Wrote reg 0x%04X = 0x%02X\n",
				 regs[i].address, regs[i].val);
		}
	}

	dev_info(imx477->dev, "Mode registers written successfully\n");
	return 0;
}

/**
 * imx477_update_controls() - Update control ranges based on streaming mode
 * @imx477: pointer to imx477 device
 * @mode: pointer to imx477_mode sensor mode
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_update_controls(struct imx477 *imx477,
				  const struct imx477_mode *mode)
{
	int ret;

	ret = __v4l2_ctrl_s_ctrl(imx477->link_freq_ctrl, mode->link_freq_idx);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(imx477->hblank_ctrl, mode->hblank);
	if (ret)
		return ret;

	return __v4l2_ctrl_modify_range(imx477->vblank_ctrl, mode->vblank_min,
					mode->vblank_max, 1, mode->vblank);
}

/**
 * imx477_update_exp_gain() - Set updated exposure and gain
 * @imx477: pointer to imx477 device
 * @exposure: updated exposure value
 * @gain: updated analog gain value
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_update_exp_gain(struct imx477 *imx477, u32 exposure, u32 gain)
{
	u32 lpfr;
	int ret;

	lpfr = imx477->vblank + imx477->cur_mode->height;

	dev_dbg(imx477->dev, "Set exp %u, analog gain %u, lpfr %u\n",
		exposure, gain, lpfr);

	ret = imx477_write_reg(imx477, IMX477_REG_HOLD, 1, 1);
	if (ret)
		return ret;

	ret = imx477_write_reg(imx477, IMX477_REG_LPFR, 2, lpfr);
	if (ret)
		goto error_release_group_hold;

	ret = imx477_write_reg(imx477, IMX477_REG_EXPOSURE_CIT, 2, exposure);
	if (ret)
		goto error_release_group_hold;

	ret = imx477_write_reg(imx477, IMX477_REG_AGAIN, 2, gain);

error_release_group_hold:
	imx477_write_reg(imx477, IMX477_REG_HOLD, 1, 0);

	return ret;
}

/**
 * imx477_set_ctrl() - Set subdevice control
 * @ctrl: pointer to v4l2_ctrl structure
 *
 * Supported controls:
 * - V4L2_CID_VBLANK
 * - cluster controls:
 *   - V4L2_CID_ANALOGUE_GAIN
 *   - V4L2_CID_EXPOSURE
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx477 *imx477 =
		container_of(ctrl->handler, struct imx477, ctrl_handler);
	u32 analog_gain;
	u32 exposure;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		imx477->vblank = imx477->vblank_ctrl->val;

		dev_dbg(imx477->dev, "Received vblank %u, new lpfr %u\n",
			imx477->vblank,
			imx477->vblank + imx477->cur_mode->height);

		ret = __v4l2_ctrl_modify_range(imx477->exp_ctrl,
					       IMX477_EXPOSURE_MIN,
					       imx477->vblank +
					       imx477->cur_mode->height -
					       IMX477_EXPOSURE_OFFSET,
					       1, IMX477_EXPOSURE_DEFAULT);
		break;
	case V4L2_CID_EXPOSURE:
		/* Set controls only if sensor is in power on state */
		if (!pm_runtime_get_if_in_use(imx477->dev))
			return 0;

		exposure = ctrl->val;
		analog_gain = imx477->again_ctrl->val;

		dev_dbg(imx477->dev, "Received exp %u, analog gain %u\n",
			exposure, analog_gain);

		ret = imx477_update_exp_gain(imx477, exposure, analog_gain);

		pm_runtime_put(imx477->dev);

		break;
	default:
		dev_err(imx477->dev, "Invalid control %d\n", ctrl->id);
		ret = -EINVAL;
	}

	return ret;
}

/* V4l2 subdevice control ops*/
static const struct v4l2_ctrl_ops imx477_ctrl_ops = {
	.s_ctrl = imx477_set_ctrl,
};

/**
 * imx477_enum_mbus_code() - Enumerate V4L2 sub-device mbus codes
 * @sd: pointer to imx477 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @code: V4L2 sub-device code enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = supported_mode.code;

	return 0;
}

/**
 * imx477_enum_frame_size() - Enumerate V4L2 sub-device frame sizes
 * @sd: pointer to imx477 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fsize: V4L2 sub-device size enumeration need to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fsize)
{
	if (fsize->index > 0)
		return -EINVAL;

	if (fsize->code != supported_mode.code)
		return -EINVAL;

	fsize->min_width = supported_mode.width;
	fsize->max_width = fsize->min_width;
	fsize->min_height = supported_mode.height;
	fsize->max_height = fsize->min_height;

	return 0;
}

/**
 * imx477_fill_pad_format() - Fill subdevice pad format
 *                            from selected sensor mode
 * @imx477: pointer to imx477 device
 * @mode: pointer to imx477_mode sensor mode
 * @fmt: V4L2 sub-device format need to be filled
 */
static void imx477_fill_pad_format(struct imx477 *imx477,
				   const struct imx477_mode *mode,
				   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = mode->code;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->format.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;
}

/**
 * imx477_get_pad_format() - Get subdevice pad format
 * @sd: pointer to imx477 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx477 *imx477 = to_imx477(sd);

	mutex_lock(&imx477->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		fmt->format = *framefmt;
	} else {
		imx477_fill_pad_format(imx477, imx477->cur_mode, fmt);
	}

	mutex_unlock(&imx477->mutex);

	return 0;
}

/**
 * imx477_set_pad_format() - Set subdevice pad format
 * @sd: pointer to imx477 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 * @fmt: V4L2 sub-device format need to be set
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx477 *imx477 = to_imx477(sd);
	const struct imx477_mode *mode;
	int ret = 0;

	mutex_lock(&imx477->mutex);

	mode = &supported_mode;
	imx477_fill_pad_format(imx477, mode, fmt);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *framefmt;

		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else {
		ret = imx477_update_controls(imx477, mode);
		if (!ret)
			imx477->cur_mode = mode;
	}

	mutex_unlock(&imx477->mutex);

	return ret;
}

/**
 * imx477_get_frame_desc() - Get CSI-2 frame descriptor
 * @sd: pointer to imx477 V4L2 sub-device structure
 * @pad: pad number
 * @fd: pointer to frame descriptor to be filled
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	struct imx477 *imx477 = to_imx477(sd);
	const struct imx477_mode *mode = imx477->cur_mode;

	if (pad != 0)
		return -EINVAL;

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	memset(fd->entry, 0, sizeof(fd->entry));

	fd->entry[0].flags = 0;
	fd->entry[0].pixelcode = mode->code;
	fd->entry[0].stream = 0;
	fd->entry[0].length = mode->width * mode->height * 10 / 8;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = 0x2b; /* RAW10 */

	return 0;
}

/**
 * imx477_init_state() - Initialize sub-device state
 * @sd: pointer to imx477 V4L2 sub-device structure
 * @sd_state: V4L2 sub-device configuration
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state)
{
	struct imx477 *imx477 = to_imx477(sd);
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = sd_state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	imx477_fill_pad_format(imx477, &supported_mode, &fmt);

	return imx477_set_pad_format(sd, sd_state, &fmt);
}

/**
 * imx477_start_streaming() - Start sensor stream
 * @imx477: pointer to imx477 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_start_streaming(struct imx477 *imx477)
{
	const struct imx477_reg_list *reg_list;
	int ret;

	dev_info(imx477->dev, "=== START STREAMING BEGIN ===\n");
	dev_info(imx477->dev, "Mode: %ux%u, code=0x%x\n",
		 imx477->cur_mode->width, imx477->cur_mode->height,
		 imx477->cur_mode->code);

	/* Write sensor mode registers */
	reg_list = &imx477->cur_mode->reg_list;
	dev_info(imx477->dev, "Writing %u mode registers...\n", reg_list->num_of_regs);
	ret = imx477_write_regs(imx477, reg_list->regs,
				reg_list->num_of_regs);
	if (ret) {
		dev_err(imx477->dev, "fail to write initial registers\n");
		return ret;
	}

	/* Setup handler will write actual exposure and gain */
	dev_info(imx477->dev, "Setting up control handler...\n");
	ret =  __v4l2_ctrl_handler_setup(imx477->sd.ctrl_handler);
	if (ret) {
		dev_err(imx477->dev, "fail to setup handler\n");
		return ret;
	}

	/* Delay is required before streaming*/
	dev_info(imx477->dev, "Waiting before stream start...\n");
	usleep_range(20000, 25000);  /* Increased delay */

	/* Start streaming */
	dev_info(imx477->dev, "Writing MODE_SELECT = STREAMING (0x01)\n");
	ret = imx477_write_reg(imx477, IMX477_REG_MODE_SELECT,
			       1, IMX477_MODE_STREAMING);
	if (ret) {
		dev_err(imx477->dev, "fail to start streaming\n");
		return ret;
	}

	/* Give sensor time to start D-PHY and reach HS mode */
	usleep_range(30000, 35000);

	dev_info(imx477->dev, "=== START STREAMING SUCCESS ===\n");
	return 0;
}

/**
 * imx477_stop_streaming() - Stop sensor stream
 * @imx477: pointer to imx477 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_stop_streaming(struct imx477 *imx477)
{
	dev_info(imx477->dev, "IMX477: Stopping sensor streaming\n");
	return imx477_write_reg(imx477, IMX477_REG_MODE_SELECT,
				1, IMX477_MODE_STANDBY);
}

/**
 * imx477_enable_streams() - Enable streams (new API)
 * @sd: pointer to imx477 V4L2 sub-device
 * @state: subdev state
 * @pad: pad number
 * @streams_mask: streams to enable
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct imx477 *imx477 = to_imx477(sd);
	int ret;

	dev_info(imx477->dev, "IMX477: enable_streams called, pad=%u, streams=0x%llx\n",
		 pad, streams_mask);

	if (pad != 0)
		return -EINVAL;

	mutex_lock(&imx477->mutex);

	ret = pm_runtime_resume_and_get(imx477->dev);
	if (ret) {
		dev_err(imx477->dev, "pm_runtime_resume_and_get failed: %d\n", ret);
		goto unlock;
	}

	ret = imx477_start_streaming(imx477);
	if (ret) {
		pm_runtime_put(imx477->dev);
		goto unlock;
	}

unlock:
	mutex_unlock(&imx477->mutex);
	return ret;
}

/**
 * imx477_disable_streams() - Disable streams (new API)
 * @sd: pointer to imx477 V4L2 sub-device
 * @state: subdev state
 * @pad: pad number
 * @streams_mask: streams to disable
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   u32 pad, u64 streams_mask)
{
	struct imx477 *imx477 = to_imx477(sd);

	dev_info(imx477->dev, "IMX477: disable_streams called, pad=%u, streams=0x%llx\n",
		 pad, streams_mask);

	if (pad != 0)
		return -EINVAL;

	mutex_lock(&imx477->mutex);
	imx477_stop_streaming(imx477);
	pm_runtime_put(imx477->dev);
	mutex_unlock(&imx477->mutex);

	return 0;
}

/**
 * imx477_set_stream() - Enable sensor streaming
 * @sd: pointer to imx477 subdevice
 * @enable: set to enable sensor streaming
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx477 *imx477 = to_imx477(sd);
	int ret;

	dev_info(imx477->dev, "IMX477: set_stream called, enable=%d\n", enable);

	mutex_lock(&imx477->mutex);

	if (enable) {
		dev_info(imx477->dev, "IMX477: Resuming device via pm_runtime...\n");
		ret = pm_runtime_resume_and_get(imx477->dev);
		if (ret) {
			dev_err(imx477->dev, "pm_runtime_resume_and_get failed: %d\n", ret);
			goto error_unlock;
		}
		dev_info(imx477->dev, "IMX477: Device resumed, calling start_streaming\n");

		ret = imx477_start_streaming(imx477);
		if (ret) {
			dev_err(imx477->dev, "start_streaming failed: %d\n", ret);
			goto error_power_off;
		}
	} else {
		dev_info(imx477->dev, "IMX477: Stopping stream\n");
		imx477_stop_streaming(imx477);
		pm_runtime_put(imx477->dev);
		dev_info(imx477->dev, "IMX477: Stream stopped\n");
	}

	mutex_unlock(&imx477->mutex);

	dev_info(imx477->dev, "IMX477: set_stream complete, enable=%d\n", enable);
	return 0;

error_power_off:
	pm_runtime_put(imx477->dev);
error_unlock:
	mutex_unlock(&imx477->mutex);

	dev_err(imx477->dev, "IMX477: set_stream FAILED, enable=%d, ret=%d\n", enable, ret);
	return ret;
}

/**
 * imx477_detect() - Detect imx477 sensor
 * @imx477: pointer to imx477 device
 *
 * Return: 0 if successful, -EIO if sensor id does not match
 */
static int imx477_detect(struct imx477 *imx477)
{
	int ret;
	u32 val;

	dev_info(imx477->dev, "IMX477: Detecting sensor...\n");

	ret = imx477_read_reg(imx477, IMX477_REG_ID, 2, &val);
	if (ret) {
		dev_err(imx477->dev, "Failed to read chip ID: %d\n", ret);
		return ret;
	}

	dev_info(imx477->dev, "Read chip ID: 0x%04X (expected 0x%04X)\n", val, IMX477_ID);

	if (val != IMX477_ID) {
		dev_err(imx477->dev, "chip id mismatch: %x!=%x\n",
			IMX477_ID, val);
		return -ENXIO;
	}

	dev_info(imx477->dev, "IMX477: Sensor detected successfully\n");
	return 0;
}

/**
 * imx477_parse_hw_config() - Parse HW configuration and check if supported
 * @imx477: pointer to imx477 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_parse_hw_config(struct imx477 *imx477)
{
	struct fwnode_handle *fwnode = dev_fwnode(imx477->dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *ep;
	unsigned long rate;
	unsigned int i;
	int ret;

	if (!fwnode)
		return -ENXIO;

	/* Request optional enable pin (active-high: 1=enabled, 0=disabled) */
	imx477->reset_gpio = devm_gpiod_get_optional(imx477->dev, "enable",
						     GPIOD_OUT_LOW);
	if (IS_ERR(imx477->reset_gpio)) {
		dev_err(imx477->dev, "failed to get enable gpio %ld\n",
			PTR_ERR(imx477->reset_gpio));
		return PTR_ERR(imx477->reset_gpio);
	}

	/* Get sensor input clock */
	imx477->inclk = devm_clk_get(imx477->dev, NULL);
	if (IS_ERR(imx477->inclk)) {
		dev_err(imx477->dev, "could not get inclk\n");
		return PTR_ERR(imx477->inclk);
	}

	rate = clk_get_rate(imx477->inclk);
	if (rate != IMX477_INCLK_RATE) {
		dev_err(imx477->dev, "inclk frequency mismatch\n");
		return -EINVAL;
	}

	/* Get optional DT defined regulators */
	for (i = 0; i < ARRAY_SIZE(imx477_supply_names); i++)
		imx477->supplies[i].supply = imx477_supply_names[i];

	ret = devm_regulator_bulk_get(imx477->dev,
				      ARRAY_SIZE(imx477_supply_names),
				      imx477->supplies);
	if (ret)
		return ret;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX477_NUM_DATA_LANES) {
		dev_err(imx477->dev,
			"number of CSI2 data lanes %d is not supported\n",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(imx477->dev, "no link frequencies defined\n");
		ret = -EINVAL;
		goto done_endpoint_free;
	}

	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++)
		if (bus_cfg.link_frequencies[i] == IMX477_LINK_FREQ)
			goto done_endpoint_free;
	dev_err(imx477->dev, "link frequency %lu is not supported\n",
		IMX477_LINK_FREQ);
	ret = -EINVAL;

done_endpoint_free:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

/* V4l2 subdevice ops */
static const struct v4l2_subdev_video_ops imx477_video_ops = {
	.s_stream = imx477_set_stream,
};

static const struct v4l2_subdev_pad_ops imx477_pad_ops = {
	.enum_mbus_code = imx477_enum_mbus_code,
	.enum_frame_size = imx477_enum_frame_size,
	.get_fmt = imx477_get_pad_format,
	.set_fmt = imx477_set_pad_format,
	.get_frame_desc = imx477_get_frame_desc,
};

static const struct v4l2_subdev_ops imx477_subdev_ops = {
	.video = &imx477_video_ops,
	.pad = &imx477_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx477_internal_ops = {
	.init_state = imx477_init_state,
};

/**
 * imx477_power_on() - Sensor power on sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx477 *imx477 = to_imx477(sd);
	int ret;

	dev_info(dev, "IMX477: Power ON begin\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(imx477_supply_names),
				    imx477->supplies);
	if (ret < 0) {
		dev_err(dev, "failed to enable regulators\n");
		return ret;
	}
	dev_info(dev, "IMX477: Regulators enabled\n");

	/* Enable sensor (1 = enabled, 0 = disabled) */
	gpiod_set_value_cansleep(imx477->reset_gpio, 1);
	dev_info(dev, "IMX477: Enable GPIO set HIGH (sensor enabled)\n");

	ret = clk_prepare_enable(imx477->inclk);
	if (ret) {
		dev_err(imx477->dev, "fail to enable inclk\n");
		goto error_reset;
	}
	dev_info(dev, "IMX477: Clock enabled (24 MHz)\n");

	usleep_range(1000, 1200);

	dev_info(dev, "IMX477: Power ON complete\n");
	return 0;

error_reset:
	/* Disable sensor */
	gpiod_set_value_cansleep(imx477->reset_gpio, 0);
	regulator_bulk_disable(ARRAY_SIZE(imx477_supply_names),
			       imx477->supplies);

	return ret;
}

/**
 * imx477_power_off() - Sensor power off sequence
 * @dev: pointer to i2c device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx477 *imx477 = to_imx477(sd);

	dev_info(dev, "IMX477: Power OFF begin\n");

	clk_disable_unprepare(imx477->inclk);

	/* Disable sensor (0 = disabled, 1 = enabled) */
	gpiod_set_value_cansleep(imx477->reset_gpio, 0);
	dev_info(dev, "IMX477: Enable GPIO set LOW (sensor disabled)\n");

	regulator_bulk_disable(ARRAY_SIZE(imx477_supply_names),
			       imx477->supplies);

	dev_info(dev, "IMX477: Power OFF complete\n");
	return 0;
}

/**
 * imx477_debugfs_regs_show() - Show sensor registers via debugfs
 * @s: seq_file pointer
 * @data: private data (imx477 device pointer)
 *
 * Return: 0 if successful
 */
static int imx477_debugfs_regs_show(struct seq_file *s, void *data)
{
	struct imx477 *imx477 = s->private;
	u32 val;
	int ret;
	static const struct {
		u16 reg;
		const char *name;
	} regs[] = {
		/* Identification */
		{0x0016, "Chip ID"},
		/* Control */
		{0x0100, "Mode Select"},
		{0x0101, "Image Orient"},
		{0x0103, "SW Reset"},
		{0x0104, "Group Hold"},
		/* CSI Interface */
		{0x0110, "CSI CH ID"},
		{0x0111, "CSI Sig Mode"},
		{0x0112, "CSI DT FMT H"},
		{0x0113, "CSI DT FMT L"},
		{0x0114, "CSI Lane Mode"},
		/* Clock */
		{0x0136, "EXCK Freq H"},
		{0x0137, "EXCK Freq L"},
		/* Exposure/Gain */
		{0x0202, "Exposure H"},
		{0x0203, "Exposure L"},
		{0x0204, "Analog Gain H"},
		{0x0205, "Analog Gain L"},
		{0x020E, "Dig Gain GR H"},
		{0x020F, "Dig Gain GR L"},
		/* PLL - IVT */
		{0x0301, "IVT PXCK DIV"},
		{0x0303, "IVT SYCK DIV"},
		{0x0305, "IVT PREPLL DIV"},
		{0x0306, "IVT PLL MPY H"},
		{0x0307, "IVT PLL MPY L"},
		/* PLL - IOP */
		{0x0309, "IOP PXCK DIV"},
		{0x030B, "IOP SYCK DIV"},
		{0x030D, "IOP PREPLL DIV"},
		{0x030E, "IOP PLL MPY H"},
		{0x030F, "IOP PLL MPY L"},
		{0x0310, "PLL MULT DRIV"},
		/* Frame Timing */
		{0x0340, "Frame Length H"},
		{0x0341, "Frame Length L"},
		{0x0342, "Line Length H"},
		{0x0343, "Line Length L"},
		{0x0350, "Frame Len Ctl"},
		/* Analog Crop */
		{0x0344, "X Add Sta H"},
		{0x0345, "X Add Sta L"},
		{0x0346, "Y Add Sta H"},
		{0x0347, "Y Add Sta L"},
		{0x0348, "X Add End H"},
		{0x0349, "X Add End L"},
		{0x034A, "Y Add End H"},
		{0x034B, "Y Add End L"},
		/* Output Size */
		{0x034C, "X Out Size H"},
		{0x034D, "X Out Size L"},
		{0x034E, "Y Out Size H"},
		{0x034F, "Y Out Size L"},
		/* Binning */
		{0x0900, "Binning Mode"},
		{0x0901, "Binning Type"},
		{0x0902, "Binning Weight"},
		/* Link Bit Rate */
		{0x0820, "Link Rate [3]"},
		{0x0821, "Link Rate [2]"},
		{0x0822, "Link Rate [1]"},
		{0x0823, "Link Rate [0]"},
		/* Output Control */
		{0x3E20, "Output Data Sel"},
		{0x3E37, "Out Data Ctrl"},
		/* Power Save */
		{0x3F50, "Power Save En"},
		{0x3F56, "Line Len INCK H"},
		{0x3F57, "Line Len INCK L"},
		/* CSI Blanking */
		{0xE000, "Frame BlankStop"},
		{0xE013, "DOL E013"},
		/* Embedded Data */
		{0xBCF1, "Embed Data Size"},
	};
	unsigned int i;

	pm_runtime_get_sync(imx477->dev);

	seq_puts(s, "IMX477 Register Dump\n");
	seq_puts(s, "====================\n\n");

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = imx477_read_reg(imx477, regs[i].reg, 1, &val);
		if (ret) {
			seq_printf(s, "0x%04X %-20s: ERROR\n",
				   regs[i].reg, regs[i].name);
		} else {
			seq_printf(s, "0x%04X %-20s: 0x%02X (%u)\n",
				   regs[i].reg, regs[i].name, val, val);
		}
	}

	pm_runtime_put(imx477->dev);

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(imx477_debugfs_regs);

/**
 * imx477_init_controls() - Initialize sensor subdevice controls
 * @imx477: pointer to imx477 device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_init_controls(struct imx477 *imx477)
{
	struct v4l2_ctrl_handler *ctrl_hdlr = &imx477->ctrl_handler;
	const struct imx477_mode *mode = imx477->cur_mode;
	u32 lpfr;
	int ret;

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 6);
	if (ret)
		return ret;

	/* Serialize controls with sensor device */
	ctrl_hdlr->lock = &imx477->mutex;

	/* Initialize exposure and gain */
	lpfr = mode->vblank + mode->height;
	imx477->exp_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
					     &imx477_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX477_EXPOSURE_MIN,
					     lpfr - IMX477_EXPOSURE_OFFSET,
					     IMX477_EXPOSURE_STEP,
					     IMX477_EXPOSURE_DEFAULT);

	imx477->again_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
					       &imx477_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN,
					       IMX477_AGAIN_MIN,
					       IMX477_AGAIN_MAX,
					       IMX477_AGAIN_STEP,
					       IMX477_AGAIN_DEFAULT);

	v4l2_ctrl_cluster(2, &imx477->exp_ctrl);

	imx477->vblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						&imx477_ctrl_ops,
						V4L2_CID_VBLANK,
						mode->vblank_min,
						mode->vblank_max,
						1, mode->vblank);

	/* Read only controls */
	imx477->pclk_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
					      &imx477_ctrl_ops,
					      V4L2_CID_PIXEL_RATE,
					      mode->pclk, mode->pclk,
					      1, mode->pclk);

	imx477->link_freq_ctrl = v4l2_ctrl_new_int_menu(ctrl_hdlr,
							&imx477_ctrl_ops,
							V4L2_CID_LINK_FREQ,
							ARRAY_SIZE(link_freq) -
							1,
							mode->link_freq_idx,
							link_freq);
	if (imx477->link_freq_ctrl)
		imx477->link_freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx477->hblank_ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						&imx477_ctrl_ops,
						V4L2_CID_HBLANK,
						IMX477_REG_MIN,
						IMX477_REG_MAX,
						1, mode->hblank);
	if (imx477->hblank_ctrl)
		imx477->hblank_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (ctrl_hdlr->error) {
		dev_err(imx477->dev, "control init failed: %d\n",
			ctrl_hdlr->error);
		v4l2_ctrl_handler_free(ctrl_hdlr);
		return ctrl_hdlr->error;
	}

	imx477->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

/**
 * imx477_probe() - I2C client device binding
 * @client: pointer to i2c client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static int imx477_probe(struct i2c_client *client)
{
	struct imx477 *imx477;
	const char *name;
	int ret;

	imx477 = devm_kzalloc(&client->dev, sizeof(*imx477), GFP_KERNEL);
	if (!imx477)
		return -ENOMEM;

	imx477->dev = &client->dev;
	name = device_get_match_data(&client->dev);
	if (!name)
		return -ENODEV;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&imx477->sd, client, &imx477_subdev_ops);
	imx477->sd.internal_ops = &imx477_internal_ops;

	ret = imx477_parse_hw_config(imx477);
	if (ret) {
		dev_err(imx477->dev, "HW configuration is not supported\n");
		return ret;
	}

	mutex_init(&imx477->mutex);

	ret = imx477_power_on(imx477->dev);
	if (ret) {
		dev_err(imx477->dev, "failed to power-on the sensor\n");
		goto error_mutex_destroy;
	}

	/* Check module identity */
	ret = imx477_detect(imx477);
	if (ret) {
		dev_err(imx477->dev, "failed to find sensor: %d\n", ret);
		goto error_power_off;
	}

	/* Set default mode to max resolution */
	imx477->cur_mode = &supported_mode;
	imx477->vblank = imx477->cur_mode->vblank;

	ret = imx477_init_controls(imx477);
	if (ret) {
		dev_err(imx477->dev, "failed to init controls: %d\n", ret);
		goto error_power_off;
	}

	/* Initialize subdev */
	imx477->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx477->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	v4l2_i2c_subdev_set_name(&imx477->sd, client, name, NULL);

	/* Initialize source pad */
	imx477->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&imx477->sd.entity, 1, &imx477->pad);
	if (ret) {
		dev_err(imx477->dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx477->sd);
	if (ret < 0) {
		dev_err(imx477->dev,
			"failed to register async subdev: %d\n", ret);
		goto error_media_entity;
	}

	/* Create debugfs directory for register access */
	imx477->debugfs_dir = debugfs_create_dir(dev_name(imx477->dev), NULL);
	if (!IS_ERR(imx477->debugfs_dir)) {
		debugfs_create_file("registers", 0444, imx477->debugfs_dir,
				    imx477, &imx477_debugfs_regs_fops);
	}

	pm_runtime_set_active(imx477->dev);
	pm_runtime_enable(imx477->dev);
	pm_runtime_idle(imx477->dev);

	dev_info(imx477->dev, "IMX477: Probe completed successfully!\n");
	dev_info(imx477->dev, "IMX477: Mode: %ux%u RAW10, link_freq=%lld Hz\n",
		 supported_mode.width, supported_mode.height, link_freq[0]);

	return 0;

error_media_entity:
	media_entity_cleanup(&imx477->sd.entity);
error_handler_free:
	v4l2_ctrl_handler_free(imx477->sd.ctrl_handler);
error_power_off:
	imx477_power_off(imx477->dev);
error_mutex_destroy:
	mutex_destroy(&imx477->mutex);

	return ret;
}

/**
 * imx477_remove() - I2C client device unbinding
 * @client: pointer to I2C client device
 *
 * Return: 0 if successful, error code otherwise.
 */
static void imx477_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);

	debugfs_remove_recursive(imx477->debugfs_dir);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx477_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&imx477->mutex);
}

static const struct dev_pm_ops imx477_pm_ops = {
	SET_RUNTIME_PM_OPS(imx477_power_off, imx477_power_on, NULL)
};

static const struct of_device_id imx477_of_match[] = {
	{ .compatible = "sony,imx477", .data = "imx477" },
	{ }
};

MODULE_DEVICE_TABLE(of, imx477_of_match);

static struct i2c_driver imx477_driver = {
	.probe = imx477_probe,
	.remove = imx477_remove,
	.driver = {
		.name = "imx477",
		.pm = &imx477_pm_ops,
		.of_match_table = imx477_of_match,
	},
};

module_i2c_driver(imx477_driver);

MODULE_DESCRIPTION("Sony imx477 sensor driver");
MODULE_LICENSE("GPL");

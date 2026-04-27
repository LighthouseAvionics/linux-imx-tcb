// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim GMSL2 Serializer Driver
 *
 * Copyright (C) 2024 Collabora Ltd.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fwnode.h>
#include <linux/gpio/driver.h>
#include <linux/i2c-mux.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define MAX96717_DEVICE_ID  0xbf
#define MAX96717F_DEVICE_ID 0xc8
#define MAX96717_PORTS      2
#define MAX96717_PAD_SINK   0
#define MAX96717_PAD_SOURCE 1
#define MAX96717_CSI_NLANES 4

#define MAX96717_DEFAULT_CLKOUT_RATE	24000000UL

/* DEV */
#define MAX96717_REG2         CCI_REG8(0x2)
#define MAX96717_VID_TX_EN_Z  BIT(6)
#define MAX96717_REG3    CCI_REG8(0x3)
#define MAX96717_RCLKSEL GENMASK(1, 0)
#define RCLKSEL_REF_PLL  CCI_REG8(0x3)
#define MAX96717_REG6    CCI_REG8(0x6)
#define RCLKEN           BIT(5)
#define MAX96717_DEV_ID  CCI_REG8(0xd)
#define MAX96717_DEV_REV CCI_REG8(0xe)
#define MAX96717_DEV_REV_MASK GENMASK(3, 0)

/* VID_TX Z */
#define MAX96717_VIDEO_TX0 CCI_REG8(0x110)
#define MAX96717_VIDEO_AUTO_BPP BIT(3)
#define MAX96717_VIDEO_TX2 CCI_REG8(0x112)
#define MAX96717_VIDEO_PCLKDET BIT(7)

/* VTX_Z */
#define MAX96717_VTX0                  CCI_REG8(0x24e)
#define MAX96717_VTX1                  CCI_REG8(0x24f)
#define MAX96717_PATTERN_CLK_FREQ      GENMASK(3, 1)
#define MAX96717_VTX_VS_DLY            CCI_REG24(0x250)
#define MAX96717_VTX_VS_HIGH           CCI_REG24(0x253)
#define MAX96717_VTX_VS_LOW            CCI_REG24(0x256)
#define MAX96717_VTX_V2H               CCI_REG24(0x259)
#define MAX96717_VTX_HS_HIGH           CCI_REG16(0x25c)
#define MAX96717_VTX_HS_LOW            CCI_REG16(0x25e)
#define MAX96717_VTX_HS_CNT            CCI_REG16(0x260)
#define MAX96717_VTX_V2D               CCI_REG24(0x262)
#define MAX96717_VTX_DE_HIGH           CCI_REG16(0x265)
#define MAX96717_VTX_DE_LOW            CCI_REG16(0x267)
#define MAX96717_VTX_DE_CNT            CCI_REG16(0x269)
#define MAX96717_VTX29                 CCI_REG8(0x26b)
#define MAX96717_VTX_MODE              GENMASK(1, 0)
#define MAX96717_VTX_GRAD_INC          CCI_REG8(0x26c)
#define MAX96717_VTX_CHKB_COLOR_A      CCI_REG24(0x26d)
#define MAX96717_VTX_CHKB_COLOR_B      CCI_REG24(0x270)
#define MAX96717_VTX_CHKB_RPT_CNT_A    CCI_REG8(0x273)
#define MAX96717_VTX_CHKB_RPT_CNT_B    CCI_REG8(0x274)
#define MAX96717_VTX_CHKB_ALT          CCI_REG8(0x275)

/* GPIO */
#define MAX96717_NUM_GPIO         11
#define MAX96717_GPIO_REG_A(gpio) CCI_REG8(0x2be + (gpio) * 3)
#define MAX96717_GPIO_OUT         BIT(4)
#define MAX96717_GPIO_IN          BIT(3)
#define MAX96717_GPIO_RX_EN       BIT(2)
#define MAX96717_GPIO_TX_EN       BIT(1)
#define MAX96717_GPIO_OUT_DIS     BIT(0)

/* FRONTTOP */
/* MAX96717 only have CSI port 'B' */
#define MAX96717_FRONTOP0     CCI_REG8(0x308)
#define MAX96717_START_PORT_B BIT(5)
#define MAX96717_FRONTTOP_9   CCI_REG8(0x311)
#define MAX96717_START_PORTBZ BIT(6)
/*
 * Pipe-Z software VC override. SOFT_VCZ_EN turns on VC override; the
 * 2-bit value at SOFT_VCZ_MASK is then stamped onto every outgoing MIPI
 * packet for pipe Z, replacing whatever VC the upstream sensor sent.
 * This is required for multi-camera GMSL setups in tunnel mode: the
 * MAX96724 deserializer's VC mapper is bypassed by tunnel mode, so each
 * MAX96717 must emit a unique VC on the wire so the iMX95 CSI receiver
 * can demultiplex per-camera streams to per-camera ISI pipes.
 */
#define MAX96717_FRONTTOP_22  CCI_REG8(0x31e)
#define MAX96717_SOFT_VCZ_EN  BIT(6)
#define MAX96717_FRONTTOP_24  CCI_REG8(0x320)
#define MAX96717_SOFT_VCZ_MASK  GENMASK(5, 4)
#define MAX96717_SOFT_VCZ_SHIFT 4

/* CMU — internal 1.1V regulator enable (required per chip spec) */
#define MAX96717_CMU_CMU2       CCI_REG8(0x302)
#define MAX96717_PFDDIV_VREG_1V1 0x10   /* (1 << PFDDIV_RSHORT_SHIFT=4) */

/* MIPI_RX */
#define MAX96717_MIPI_RX1       CCI_REG8(0x331)
#define MAX96717_MIPI_LANES_CNT GENMASK(5, 4)
#define MAX96717_MIPI_RX2       CCI_REG8(0x332) /* phy1 Lanes map */
#define MAX96717_PHY2_LANES_MAP GENMASK(7, 4)
#define MAX96717_MIPI_RX3       CCI_REG8(0x333) /* phy2 Lanes map */
#define MAX96717_PHY1_LANES_MAP GENMASK(3, 0)
#define MAX96717_MIPI_RX4       CCI_REG8(0x334) /* phy1 lane polarities */
#define MAX96717_PHY1_LANES_POL GENMASK(6, 4)
#define MAX96717_MIPI_RX5       CCI_REG8(0x335) /* phy2 lane polarities */
#define MAX96717_PHY2_LANES_POL GENMASK(2, 0)

/* MIPI_RX_EXT */
#define MAX96717_MIPI_RX_EXT8     CCI_REG8(0x380)
#define MAX96717_TUN_FIFO_OVF     BIT(0)
#define MAX96717_MIPI_RX_EXT11    CCI_REG8(0x383)
#define MAX96717_TUN_MODE         BIT(7)
#define MAX96717_PHY1_PKT_CNT     CCI_REG8(0x38d)
#define MAX96717_CSI1_PKT_CNT     CCI_REG8(0x38e)
#define MAX96717_TUN_PKT_CNT      CCI_REG8(0x38f)
#define MAX96717_PHY_CLK_CNT      CCI_REG8(0x390)

/* REF_VTG */
#define REF_VTG0                CCI_REG8(0x3f0)
#define REFGEN_PREDEF_EN        BIT(6)
#define REFGEN_PREDEF_FREQ_MASK GENMASK(5, 4)
#define REFGEN_PREDEF_FREQ_ALT  BIT(3)
#define REFGEN_RST              BIT(1)
#define REFGEN_EN               BIT(0)

/* MISC */
#define PIO_SLEW_1 CCI_REG8(0x570)

enum max96717_vpg_mode {
	MAX96717_VPG_DISABLED = 0,
	MAX96717_VPG_CHECKERBOARD = 1,
	MAX96717_VPG_GRADIENT = 2,
};

struct max96717_priv {
	struct i2c_client		  *client;
	struct regmap			  *regmap;
	struct i2c_mux_core		  *mux;
	struct v4l2_mbus_config_mipi_csi2 mipi_csi2;
	struct v4l2_subdev                sd;
	struct media_pad                  pads[MAX96717_PORTS];
	struct v4l2_ctrl_handler          ctrl_handler;
	struct v4l2_async_notifier        notifier;
	struct v4l2_subdev                *source_sd;
	u16                               source_sd_pad;
	u64			          enabled_source_streams;
	u8                                pll_predef_index;
	struct clk_hw                     clk_hw;
	struct gpio_chip                  gpio_chip;
	enum max96717_vpg_mode            pattern;
	struct dentry                     *debugfs_dir;
	u16                               debugfs_reg_addr;
	/*
	 * MIPI virtual channel (0-3) this serializer stamps onto its
	 * tunnel-mode output. Read from DT property `maxim,vc-id`;
	 * defaults to 0 when absent (single-camera-per-deser legacy
	 * behavior). Must be unique among serializers feeding the same
	 * MAX96724 to allow downstream VC-based demux.
	 */
	u8                                vc_id;
};

static inline struct max96717_priv *sd_to_max96717(struct v4l2_subdev *sd)
{
	return container_of(sd, struct max96717_priv, sd);
}

static inline struct max96717_priv *clk_hw_to_max96717(struct clk_hw *hw)
{
	return container_of(hw, struct max96717_priv, clk_hw);
}

static int max96717_i2c_mux_select(struct i2c_mux_core *mux, u32 chan)
{
	return 0;
}

static int max96717_i2c_mux_init(struct max96717_priv *priv)
{
	int ret;
	priv->mux = i2c_mux_alloc(priv->client->adapter, &priv->client->dev,
				  1, 0, I2C_MUX_LOCKED | I2C_MUX_GATE,
				  max96717_i2c_mux_select, NULL);
	if (!priv->mux) {
		return -ENOMEM;
	}

	ret = i2c_mux_add_adapter(priv->mux, 0, 0);
	return ret;
}

static inline int max96717_start_csi(struct max96717_priv *priv, bool start)
{
	int ret;
	ret = cci_update_bits(priv->regmap, MAX96717_FRONTOP0,
			       MAX96717_START_PORT_B,
			       start ? MAX96717_START_PORT_B : 0, NULL);
	/*
	 * FRONTTOP_9[6] START_PORTBZ starts the internal video pipe Z that
	 * processes data from CSI port B. NXP lib writes this alongside the
	 * FRONTOP0 START_PORT_B bit — Collabora driver was missing it.
	 */
	cci_write(priv->regmap, MAX96717_FRONTTOP_9,
		  start ? MAX96717_START_PORTBZ : 0, &ret);
	/*
	 * Program the software VC override on every start so the value
	 * survives a chip reset between streaming sessions. Stamps each
	 * outgoing tunnel-mode MIPI packet with priv->vc_id (set from
	 * DT). Without this, all serializers emit VC=0 and the iMX95
	 * CSI receiver can't demux multi-camera streams.
	 */
	if (start) {
		cci_update_bits(priv->regmap, MAX96717_FRONTTOP_22,
				MAX96717_SOFT_VCZ_EN,
				MAX96717_SOFT_VCZ_EN, &ret);
		cci_update_bits(priv->regmap, MAX96717_FRONTTOP_24,
				MAX96717_SOFT_VCZ_MASK,
				(priv->vc_id << MAX96717_SOFT_VCZ_SHIFT) &
				MAX96717_SOFT_VCZ_MASK, &ret);
	}
	/*
	 * REG2[6] VID_TX_EN_Z gates video transmission onto GMSL. Default
	 * is 0x03 (TX disabled) on boards where bootstrap straps don't set
	 * it. NXP lib notes the low two reserved bits must be kept set when
	 * enabling, hence the (VID_TX_EN_Z | 0x03) value.
	 */
	cci_write(priv->regmap, MAX96717_REG2,
		  start ? (MAX96717_VID_TX_EN_Z | 0x03) : 0x00, &ret);
	return ret;
}

static int max96717_apply_patgen_timing(struct max96717_priv *priv,
					struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, MAX96717_PAD_SOURCE);
	const u32 h_active = fmt->width;
	const u32 h_fp = 88;
	const u32 h_sw = 44;
	const u32 h_bp = 148;
	u32 h_tot;
	const u32 v_active = fmt->height;
	const u32 v_fp = 4;
	const u32 v_sw = 5;
	const u32 v_bp = 36;
	u32 v_tot;
	int ret = 0;


	h_tot = h_active + h_fp + h_sw + h_bp;
	v_tot = v_active + v_fp + v_sw + v_bp;

	/* 75 Mhz pixel clock */
	cci_update_bits(priv->regmap, MAX96717_VTX1,
			MAX96717_PATTERN_CLK_FREQ, 0xa, &ret);

	dev_info(&priv->client->dev, "height: %d width: %d\n", fmt->height,
		 fmt->width);

	cci_write(priv->regmap, MAX96717_VTX_VS_DLY, 0, &ret);
	cci_write(priv->regmap, MAX96717_VTX_VS_HIGH, v_sw * h_tot, &ret);
	cci_write(priv->regmap, MAX96717_VTX_VS_LOW,
		  (v_active + v_fp + v_bp) * h_tot, &ret);
	cci_write(priv->regmap, MAX96717_VTX_HS_HIGH, h_sw, &ret);
	cci_write(priv->regmap, MAX96717_VTX_HS_LOW, h_active + h_fp + h_bp,
		  &ret);
	cci_write(priv->regmap, MAX96717_VTX_V2D,
		  h_tot * (v_sw + v_bp) + (h_sw + h_bp), &ret);
	cci_write(priv->regmap, MAX96717_VTX_HS_CNT, v_tot, &ret);
	cci_write(priv->regmap, MAX96717_VTX_DE_HIGH, h_active, &ret);
	cci_write(priv->regmap, MAX96717_VTX_DE_LOW, h_fp + h_sw + h_bp,
		  &ret);
	cci_write(priv->regmap, MAX96717_VTX_DE_CNT, v_active, &ret);
	/* B G R */
	cci_write(priv->regmap, MAX96717_VTX_CHKB_COLOR_A, 0xfecc00, &ret);
	/* B G R */
	cci_write(priv->regmap, MAX96717_VTX_CHKB_COLOR_B, 0x006aa7, &ret);
	cci_write(priv->regmap, MAX96717_VTX_CHKB_RPT_CNT_A, 0x3c, &ret);
	cci_write(priv->regmap, MAX96717_VTX_CHKB_RPT_CNT_B, 0x3c, &ret);
	cci_write(priv->regmap, MAX96717_VTX_CHKB_ALT, 0x3c, &ret);
	cci_write(priv->regmap, MAX96717_VTX_GRAD_INC, 0x10, &ret);

	return ret;
}

static int max96717_apply_patgen(struct max96717_priv *priv,
				 struct v4l2_subdev_state *state)
{
	unsigned int val;
	int ret = 0;


	if (priv->pattern)
		ret = max96717_apply_patgen_timing(priv, state);

	cci_write(priv->regmap, MAX96717_VTX0, priv->pattern ? 0xfb : 0,
		  &ret);

	val = FIELD_PREP(MAX96717_VTX_MODE, priv->pattern);
	cci_update_bits(priv->regmap, MAX96717_VTX29, MAX96717_VTX_MODE,
			val, &ret);
	return ret;
}

static int max96717_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct max96717_priv *priv =
		container_of(ctrl->handler, struct max96717_priv, ctrl_handler);
	int ret;


	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		if (priv->enabled_source_streams) {
			return -EBUSY;
		}
		priv->pattern = ctrl->val;
		break;
	default:
		return -EINVAL;
	}

	/* Use bpp from bpp register */
	ret = cci_update_bits(priv->regmap, MAX96717_VIDEO_TX0,
			      MAX96717_VIDEO_AUTO_BPP,
			      priv->pattern ? 0 : MAX96717_VIDEO_AUTO_BPP,
			      NULL);

	/*
	 * Pattern generator doesn't work with tunnel mode.
	 * Needs RGB color format and deserializer tunnel mode must be disabled.
	 */
	ret = cci_update_bits(priv->regmap, MAX96717_MIPI_RX_EXT11,
			       MAX96717_TUN_MODE,
			       priv->pattern ? 0 : MAX96717_TUN_MODE, &ret);
	return ret;
}

static const char * const max96717_test_pattern[] = {
	"Disabled",
	"Checkerboard",
	"Gradient"
};

static const struct v4l2_ctrl_ops max96717_ctrl_ops = {
	.s_ctrl = max96717_s_ctrl,
};

static int max96717_gpiochip_get(struct gpio_chip *gpiochip,
				 unsigned int offset)
{
	struct max96717_priv *priv = gpiochip_get_data(gpiochip);
	u64 val;
	int ret;


	ret = cci_read(priv->regmap, MAX96717_GPIO_REG_A(offset),
		       &val, NULL);
	if (ret) {
		return ret;
	}

	if (val & MAX96717_GPIO_OUT_DIS) {
		ret = !!(val & MAX96717_GPIO_IN);
	} else {
		ret = !!(val & MAX96717_GPIO_OUT);
	}
	return ret;
}

static void max96717_gpiochip_set(struct gpio_chip *gpiochip,
				  unsigned int offset, int value)
{
	struct max96717_priv *priv = gpiochip_get_data(gpiochip);


	cci_update_bits(priv->regmap, MAX96717_GPIO_REG_A(offset),
			MAX96717_GPIO_OUT, MAX96717_GPIO_OUT, NULL);

}

static int max96717_gpio_get_direction(struct gpio_chip *gpiochip,
				       unsigned int offset)
{
	struct max96717_priv *priv = gpiochip_get_data(gpiochip);
	u64 val;
	int ret;


	ret = cci_read(priv->regmap, MAX96717_GPIO_REG_A(offset), &val, NULL);
	if (ret < 0) {
		return ret;
	}

	ret = !!(val & MAX96717_GPIO_OUT_DIS);
	return ret;
}

static int max96717_gpio_direction_out(struct gpio_chip *gpiochip,
				       unsigned int offset, int value)
{
	struct max96717_priv *priv = gpiochip_get_data(gpiochip);
	int ret;


	ret = cci_update_bits(priv->regmap, MAX96717_GPIO_REG_A(offset),
			       MAX96717_GPIO_OUT_DIS | MAX96717_GPIO_OUT,
			       value ? MAX96717_GPIO_OUT : 0, NULL);

	return ret;
}

static int max96717_gpio_direction_in(struct gpio_chip *gpiochip,
				      unsigned int offset)
{
	struct max96717_priv *priv = gpiochip_get_data(gpiochip);
	int ret;


	ret = cci_update_bits(priv->regmap, MAX96717_GPIO_REG_A(offset),
			       MAX96717_GPIO_OUT_DIS, MAX96717_GPIO_OUT_DIS,
			       NULL);

	return ret;
}

static int max96717_gpiochip_probe(struct max96717_priv *priv)
{
	struct device *dev = &priv->client->dev;
	struct gpio_chip *gc = &priv->gpio_chip;
	int i, ret = 0;


	gc->label = dev_name(dev);
	gc->parent = dev;
	gc->owner = THIS_MODULE;
	gc->ngpio = MAX96717_NUM_GPIO;
	gc->base = -1;
	gc->can_sleep = true;
	gc->get_direction = max96717_gpio_get_direction;
	gc->direction_input = max96717_gpio_direction_in;
	gc->direction_output = max96717_gpio_direction_out;
	gc->set = max96717_gpiochip_set;
	gc->get = max96717_gpiochip_get;
	gc->of_gpio_n_cells = 2;

	/* Disable GPIO forwarding */
	for (i = 0; i < gc->ngpio; i++)
		cci_update_bits(priv->regmap, MAX96717_GPIO_REG_A(i),
				MAX96717_GPIO_RX_EN | MAX96717_GPIO_TX_EN,
				0, &ret);

	if (ret) {
		return ret;
	}

	ret = devm_gpiochip_add_data(dev, gc, priv);
	if (ret) {
		dev_err(dev, "Unable to create gpio_chip\n");
		return ret;
	}

	return 0;
}

static int _max96717_set_routing(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_krouting *routing)
{
	static const struct v4l2_mbus_framefmt format = {
		.width = 1280,
		.height = 1080,
		.code = MEDIA_BUS_FMT_Y8_1X8,
		.field = V4L2_FIELD_NONE,
	};
	int ret;


	ret = v4l2_subdev_routing_validate(sd, routing,
					   V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
	if (ret) {
		return ret;
	}

	ret = v4l2_subdev_set_routing_with_fmt(sd, state, routing, &format);
	if (ret) {
		return ret;
	}

	return 0;
}

static int max96717_set_routing(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				enum v4l2_subdev_format_whence which,
				struct v4l2_subdev_krouting *routing)
{
	struct max96717_priv *priv = sd_to_max96717(sd);
	int ret;


	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->enabled_source_streams) {
		return -EBUSY;
	}

	ret = _max96717_set_routing(sd, state, routing);
	return ret;
}

static int max96717_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *format)
{
	struct max96717_priv *priv = sd_to_max96717(sd);
	struct v4l2_mbus_framefmt *fmt;
	u64 stream_source_mask;
	int ret;


	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    priv->enabled_source_streams) {
		return -EBUSY;
	}

	/* No transcoding, source and sink formats must match. */
	if (format->pad == MAX96717_PAD_SOURCE)
		return v4l2_subdev_get_fmt(sd, state, format);

	/* Set sink format */
	fmt = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (!fmt) {
		return -EINVAL;
	}

	*fmt = format->format;

	/* Propagate to source format */
	fmt = v4l2_subdev_state_get_opposite_stream_format(state, format->pad,
							   format->stream);
	if (!fmt) {
		return -EINVAL;
	}
	*fmt = format->format;

	stream_source_mask = BIT(format->stream);

	ret = v4l2_subdev_state_xlate_streams(state, MAX96717_PAD_SOURCE,
					       MAX96717_PAD_SINK,
					       &stream_source_mask);
	return ret;
}

static int max96717_init_state(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[] = {
		{
			.sink_pad = MAX96717_PAD_SINK,
			.sink_stream = 0,
			.source_pad = MAX96717_PAD_SOURCE,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};
	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	return _max96717_set_routing(sd, state, &routing);
}

static bool max96717_pipe_pclkdet(struct max96717_priv *priv)
{
	u64 val = 0;
	bool ret_val;


	cci_read(priv->regmap, MAX96717_VIDEO_TX2, &val, NULL);

	ret_val = !!(val & MAX96717_VIDEO_PCLKDET);
	return ret_val;
}

static int max96717_log_status(struct v4l2_subdev *sd)
{
	struct max96717_priv *priv = sd_to_max96717(sd);
	struct device *dev = &priv->client->dev;

	dev_info(dev, "Serializer: max96717\n");
	dev_info(dev, "Pipe: pclkdet:%d\n", max96717_pipe_pclkdet(priv));

	return 0;
}

static int max96717_enable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct max96717_priv *priv = sd_to_max96717(sd);
	u64 sink_streams;
	int ret;


	if (!priv->enabled_source_streams)
		max96717_start_csi(priv, true);

	ret = max96717_apply_patgen(priv, state);
	if (ret) {
		goto stop_csi;
	}

	if (!priv->pattern) {
		sink_streams =
			v4l2_subdev_state_xlate_streams(state,
							MAX96717_PAD_SOURCE,
							MAX96717_PAD_SINK,
							&streams_mask);

		ret = v4l2_subdev_enable_streams(priv->source_sd,
						 priv->source_sd_pad,
						 sink_streams);
		if (ret) {
			goto stop_csi;
		}
	}

	priv->enabled_source_streams |= streams_mask;

	return 0;

stop_csi:
	if (!priv->enabled_source_streams)
		max96717_start_csi(priv, false);

	return ret;
}

static int max96717_disable_streams(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state, u32 pad,
				    u64 streams_mask)
{
	struct max96717_priv *priv = sd_to_max96717(sd);
	u64 sink_streams;


	/*
	 * Stop the CSI receiver first then the source,
	 * otherwise the device may become unresponsive
	 * while holding the I2C bus low.
	 */
	priv->enabled_source_streams &= ~streams_mask;
	if (!priv->enabled_source_streams)
		max96717_start_csi(priv, false);

	if (!priv->pattern) {
		int ret;

		sink_streams =
			v4l2_subdev_state_xlate_streams(state,
							MAX96717_PAD_SOURCE,
							MAX96717_PAD_SINK,
							&streams_mask);

		ret = v4l2_subdev_disable_streams(priv->source_sd,
						  priv->source_sd_pad,
						  sink_streams);
		if (ret) {
			return ret;
		}
	}

	return 0;
}

/*
 * Forward CSI-2 frame descriptors from the upstream sensor. The serializer
 * is transparent to pixel-data formatting (it only repacks for the GMSL
 * link), so the descriptor — including pixelcode, length, virtual channel
 * and data type — is whatever the sensor reports for the single stream we
 * carry from sink (pad 0) to source (pad 1).
 *
 * Without this, the iMX95 ISI's mxc_isi_pipe_enable() walks upstream looking
 * for VC info, can't find a frame_desc op on max96717, and aborts the pipe
 * with -EPIPE — making multi-camera streaming impossible because the ISI
 * can't tell which MIPI VC to bind each pipe to.
 */
static int max96717_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				   struct v4l2_mbus_frame_desc *fd)
{
	struct max96717_priv *priv = sd_to_max96717(sd);

	if (pad != MAX96717_PAD_SOURCE)
		return -EINVAL;

	if (!priv->source_sd)
		return -EPIPE;

	return v4l2_subdev_call(priv->source_sd, pad, get_frame_desc,
				priv->source_sd_pad, fd);
}

static const struct v4l2_subdev_pad_ops max96717_pad_ops = {
	.enable_streams = max96717_enable_streams,
	.disable_streams = max96717_disable_streams,
	.set_routing = max96717_set_routing,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = max96717_set_fmt,
	.get_frame_desc = max96717_get_frame_desc,
};

static const struct v4l2_subdev_core_ops max96717_subdev_core_ops = {
	.log_status = max96717_log_status,
};

static const struct v4l2_subdev_internal_ops max96717_internal_ops = {
	.init_state = max96717_init_state,
};

static const struct v4l2_subdev_ops max96717_subdev_ops = {
	.core = &max96717_subdev_core_ops,
	.pad = &max96717_pad_ops,
};

static const struct media_entity_operations max96717_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int max96717_notify_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *source_subdev,
				 struct v4l2_async_connection *asd)
{
	struct max96717_priv *priv = sd_to_max96717(notifier->sd);
	struct device *dev = &priv->client->dev;
	int ret;


	ret = media_entity_get_fwnode_pad(&source_subdev->entity,
					  source_subdev->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(dev, "Failed to find pad for %s\n",
			source_subdev->name);
		return ret;
	}

	priv->source_sd = source_subdev;
	priv->source_sd_pad = ret;

	ret = media_create_pad_link(&source_subdev->entity, priv->source_sd_pad,
				    &priv->sd.entity, 0,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(dev, "Unable to link %s:%u -> %s:0\n",
			source_subdev->name, priv->source_sd_pad,
			priv->sd.name);
		return ret;
	}

	return 0;
}

static const struct v4l2_async_notifier_operations max96717_notify_ops = {
	.bound = max96717_notify_bound,
};

static int max96717_v4l2_notifier_register(struct max96717_priv *priv)
{
	struct device *dev = &priv->client->dev;
	struct v4l2_async_connection *asd;
	struct fwnode_handle *ep_fwnode;
	int ret;


	ep_fwnode = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev),
						    MAX96717_PAD_SINK, 0, 0);
	if (!ep_fwnode) {
		dev_err(dev, "No graph endpoint\n");
		return -ENODEV;
	}

	v4l2_async_subdev_nf_init(&priv->notifier, &priv->sd);

	asd = v4l2_async_nf_add_fwnode_remote(&priv->notifier, ep_fwnode,
					      struct v4l2_async_connection);

	fwnode_handle_put(ep_fwnode);

	if (IS_ERR(asd)) {
		dev_err(dev, "Failed to add subdev: %ld", PTR_ERR(asd));
		v4l2_async_nf_cleanup(&priv->notifier);
		ret = PTR_ERR(asd);
		return ret;
	}

	priv->notifier.ops = &max96717_notify_ops;

	ret = v4l2_async_nf_register(&priv->notifier);
	if (ret) {
		dev_err(dev, "Failed to register subdev_notifier");
		v4l2_async_nf_cleanup(&priv->notifier);
		return ret;
	}

	return 0;
}

static int max96717_subdev_init(struct max96717_priv *priv)
{
	struct device *dev = &priv->client->dev;
	int ret;


	v4l2_i2c_subdev_init(&priv->sd, priv->client, &max96717_subdev_ops);
	priv->sd.internal_ops = &max96717_internal_ops;

	v4l2_ctrl_handler_init(&priv->ctrl_handler, 1);
	priv->sd.ctrl_handler = &priv->ctrl_handler;

	v4l2_ctrl_new_std_menu_items(&priv->ctrl_handler,
				     &max96717_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(max96717_test_pattern) - 1,
				     0, 0, max96717_test_pattern);
	if (priv->ctrl_handler.error) {
		ret = priv->ctrl_handler.error;
		goto err_free_ctrl;
	}

	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;
	priv->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	priv->sd.entity.ops = &max96717_entity_ops;

	priv->pads[MAX96717_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	priv->pads[MAX96717_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&priv->sd.entity, 2, priv->pads);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to init pads\n");
		goto err_free_ctrl;
	}

	ret = v4l2_subdev_init_finalize(&priv->sd);
	if (ret) {
		dev_err_probe(dev, ret,
			      "v4l2 subdev init finalized failed\n");
		goto err_entity_cleanup;
	}
	ret = max96717_v4l2_notifier_register(priv);
	if (ret) {
		dev_err_probe(dev, ret,
			      "v4l2 subdev notifier register failed\n");
		goto err_free_state;
	}

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret) {
		dev_err_probe(dev, ret, "v4l2_async_register_subdev error\n");
		goto err_unreg_notif;
	}

	return 0;

err_unreg_notif:
	v4l2_async_nf_unregister(&priv->notifier);
	v4l2_async_nf_cleanup(&priv->notifier);
err_free_state:
	v4l2_subdev_cleanup(&priv->sd);
err_entity_cleanup:
	media_entity_cleanup(&priv->sd.entity);
err_free_ctrl:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return ret;
}

static void max96717_subdev_uninit(struct max96717_priv *priv)
{
	v4l2_async_unregister_subdev(&priv->sd);
	v4l2_async_nf_unregister(&priv->notifier);
	v4l2_async_nf_cleanup(&priv->notifier);
	v4l2_subdev_cleanup(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
}

struct max96717_pll_predef_freq {
	unsigned long freq;
	bool is_alt;
	u8 val;
};

static const struct max96717_pll_predef_freq max96717_predef_freqs[] = {
	{ 13500000, true,  0 }, { 19200000, false, 0 },
	{ 24000000, true,  1 }, { 27000000, false, 1 },
	{ 37125000, false, 2 }, { 74250000, false, 3 },
};

static unsigned long
max96717_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct max96717_priv *priv = clk_hw_to_max96717(hw);

	return max96717_predef_freqs[priv->pll_predef_index].freq;
}

static unsigned int max96717_clk_find_best_index(struct max96717_priv *priv,
						 unsigned long rate)
{
	unsigned int i, idx = 0;
	unsigned long diff_new, diff_old = U32_MAX;

	for (i = 0; i < ARRAY_SIZE(max96717_predef_freqs); i++) {
		diff_new = abs(rate - max96717_predef_freqs[i].freq);
		if (diff_new < diff_old) {
			diff_old = diff_new;
			idx = i;
		}
	}

	return idx;
}

static long max96717_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *parent_rate)
{
	struct max96717_priv *priv = clk_hw_to_max96717(hw);
	struct device *dev = &priv->client->dev;
	unsigned int idx;
	long ret_val;


	idx = max96717_clk_find_best_index(priv, rate);

	if (rate != max96717_predef_freqs[idx].freq) {
		dev_warn(dev, "Request CLK freq:%lu, found CLK freq:%lu\n",
			 rate, max96717_predef_freqs[idx].freq);
	}

	ret_val = max96717_predef_freqs[idx].freq;
	return ret_val;
}

static int max96717_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	struct max96717_priv *priv = clk_hw_to_max96717(hw);
	unsigned int val, idx;
	int ret = 0;


	idx = max96717_clk_find_best_index(priv, rate);

	val = FIELD_PREP(REFGEN_PREDEF_FREQ_MASK,
			 max96717_predef_freqs[idx].val);

	if (max96717_predef_freqs[idx].is_alt)
		val |= REFGEN_PREDEF_FREQ_ALT;

	val |= REFGEN_RST | REFGEN_PREDEF_EN;

	cci_write(priv->regmap, REF_VTG0, val, &ret);
	cci_update_bits(priv->regmap, REF_VTG0, REFGEN_RST | REFGEN_EN,
			REFGEN_EN, &ret);
	if (ret) {
		return ret;
	}

	priv->pll_predef_index = idx;

	return 0;
}

static int max96717_clk_prepare(struct clk_hw *hw)
{
	struct max96717_priv *priv = clk_hw_to_max96717(hw);
	int ret;

	ret = cci_update_bits(priv->regmap, MAX96717_REG6, RCLKEN,
			       RCLKEN, NULL);
	return ret;
}

static void max96717_clk_unprepare(struct clk_hw *hw)
{
	struct max96717_priv *priv = clk_hw_to_max96717(hw);

	cci_update_bits(priv->regmap, MAX96717_REG6, RCLKEN, 0, NULL);
}

static const struct clk_ops max96717_clk_ops = {
	.prepare     = max96717_clk_prepare,
	.unprepare   = max96717_clk_unprepare,
	.set_rate    = max96717_clk_set_rate,
	.recalc_rate = max96717_clk_recalc_rate,
	.round_rate  = max96717_clk_round_rate,
};

static int max96717_register_clkout(struct max96717_priv *priv)
{
	struct device *dev = &priv->client->dev;
	struct clk_init_data init = { .ops = &max96717_clk_ops };
	int ret;


	init.name = kasprintf(GFP_KERNEL, "max96717.%s.clk_out", dev_name(dev));
	if (!init.name) {
		return -ENOMEM;
	}

	/* RCLKSEL Reference PLL output */
	ret = cci_update_bits(priv->regmap, MAX96717_REG3, MAX96717_RCLKSEL,
			      MAX96717_RCLKSEL, NULL);
	/* MFP4 fastest slew rate */
	cci_update_bits(priv->regmap, PIO_SLEW_1, BIT(5) | BIT(4), 0, &ret);
	if (ret) {
		goto free_init_name;
	}

	priv->clk_hw.init = &init;

	/* Initialize to 24 MHz */
	ret = max96717_clk_set_rate(&priv->clk_hw,
				    MAX96717_DEFAULT_CLKOUT_RATE, 0);
	if (ret < 0) {
		goto free_init_name;
	}

	ret = devm_clk_hw_register(dev, &priv->clk_hw);
	kfree(init.name);
	if (ret) {
		return dev_err_probe(dev, ret, "Cannot register clock HW\n");
	}

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get,
					  &priv->clk_hw);
	if (ret) {
		return dev_err_probe(dev, ret,
				     "Cannot add OF clock provider\n");
	}

	return 0;

free_init_name:
	kfree(init.name);
	return ret;
}

static int max96717_init_csi_lanes(struct max96717_priv *priv)
{
	struct v4l2_mbus_config_mipi_csi2 *mipi = &priv->mipi_csi2;
	unsigned long lanes_used = 0;
	unsigned int nlanes, lane, val = 0;
	int ret;


	nlanes = mipi->num_data_lanes;

	ret = cci_update_bits(priv->regmap, MAX96717_MIPI_RX1,
			      MAX96717_MIPI_LANES_CNT,
			      FIELD_PREP(MAX96717_MIPI_LANES_CNT,
					 nlanes - 1), NULL);

	/* lanes polarity */
	for (lane = 0; lane < nlanes + 1; lane++) {
		if (!mipi->lane_polarities[lane])
			continue;
		/* Clock lane */
		if (lane == 0)
			val |= BIT(2);
		else if (lane < 3)
			val |= BIT(lane - 1);
		else
			val |= BIT(lane);
	}

	cci_update_bits(priv->regmap, MAX96717_MIPI_RX5,
			MAX96717_PHY2_LANES_POL,
			FIELD_PREP(MAX96717_PHY2_LANES_POL, val), &ret);

	cci_update_bits(priv->regmap, MAX96717_MIPI_RX4,
			MAX96717_PHY1_LANES_POL,
			FIELD_PREP(MAX96717_PHY1_LANES_POL,
				   val >> 3), &ret);
	/* lanes mapping */
	for (lane = 0, val = 0; lane < nlanes; lane++) {
		val |= (mipi->data_lanes[lane] - 1) << (lane * 2);
		lanes_used |= BIT(mipi->data_lanes[lane] - 1);
	}

	/*
	 * Unused lanes need to be mapped as well to not have
	 * the same lanes mapped twice.
	 */
	for (; lane < MAX96717_CSI_NLANES; lane++) {
		unsigned int idx = find_first_zero_bit(&lanes_used,
						       MAX96717_CSI_NLANES);

		val |= idx << (lane * 2);
		lanes_used |= BIT(idx);
	}

	cci_update_bits(priv->regmap, MAX96717_MIPI_RX3,
			MAX96717_PHY1_LANES_MAP,
			FIELD_PREP(MAX96717_PHY1_LANES_MAP, val), &ret);

	ret = cci_update_bits(priv->regmap, MAX96717_MIPI_RX2,
			       MAX96717_PHY2_LANES_MAP,
			       FIELD_PREP(MAX96717_PHY2_LANES_MAP, val >> 4),
			       &ret);
	return ret;
}

static int max96717_hw_init(struct max96717_priv *priv)
{
	struct device *dev = &priv->client->dev;
	u64 dev_id, val;
	int ret;


	ret = cci_read(priv->regmap, MAX96717_DEV_ID, &dev_id, NULL);
	if (ret) {
		return dev_err_probe(dev, ret,
				     "Fail to read the device id\n");
	}

	if (dev_id != MAX96717_DEVICE_ID && dev_id != MAX96717F_DEVICE_ID) {
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "Unsupported device id got %x\n", (u8)dev_id);
	}

	ret = cci_read(priv->regmap, MAX96717_DEV_REV, &val, NULL);
	if (ret) {
		return dev_err_probe(dev, ret,
				     "Fail to read device revision");
	}

	dev_dbg(dev, "Found %x (rev %lx)\n", (u8)dev_id,
		(u8)val & MAX96717_DEV_REV_MASK);

	ret = cci_read(priv->regmap, MAX96717_MIPI_RX_EXT11, &val, NULL);
	if (ret) {
		return dev_err_probe(dev, ret,
				     "Fail to read mipi rx extension");
	}

	if (!(val & MAX96717_TUN_MODE)) {
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "Only supporting tunnel mode");
	}

	/*
	 * Per chip spec, the internal 1.1V regulator must be enabled to
	 * guarantee proper device operation. NXP's max96717_lib does this
	 * in hw_init; the Collabora upstream driver skips it and only works
	 * on boards where bootstrap straps happen to leave this bit set.
	 */
	ret = cci_write(priv->regmap, MAX96717_CMU_CMU2,
			MAX96717_PFDDIV_VREG_1V1, NULL);
	if (ret) {
		return dev_err_probe(dev, ret,
				     "Fail to enable CMU 1.1V regulator\n");
	}

	ret = max96717_init_csi_lanes(priv);
	return ret;
}

static int max96717_parse_dt(struct max96717_priv *priv)
{
	struct device *dev = &priv->client->dev;
	struct v4l2_fwnode_endpoint vep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *ep_fwnode;
	unsigned char num_data_lanes;
	int ret;


	ep_fwnode = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev),
						    MAX96717_PAD_SINK, 0, 0);
	if (!ep_fwnode) {
		return dev_err_probe(dev, -ENOENT, "no endpoint found\n");
	}

	ret = v4l2_fwnode_endpoint_parse(ep_fwnode, &vep);

	fwnode_handle_put(ep_fwnode);

	if (ret < 0) {
		return dev_err_probe(dev, ret, "Failed to parse sink endpoint");
	}

	num_data_lanes = vep.bus.mipi_csi2.num_data_lanes;
	if (num_data_lanes < 1 || num_data_lanes > MAX96717_CSI_NLANES) {
		return dev_err_probe(dev, -EINVAL,
				     "Invalid data lanes must be 1 to 4\n");
	}

	priv->mipi_csi2 = vep.bus.mipi_csi2;

	/*
	 * Per-instance MIPI virtual channel for tunnel-mode output. Each
	 * serializer feeding the same MAX96724 must have a distinct value
	 * (0-3) so the downstream CSI receiver can demux. Absent property
	 * keeps VC=0 — matches the historical single-camera default.
	 */
	{
		u32 vc_id = 0;

		of_property_read_u32(dev_of_node(dev), "maxim,vc-id", &vc_id);
		if (vc_id > 3) {
			return dev_err_probe(dev, -EINVAL,
					     "maxim,vc-id %u out of range (0-3)\n",
					     vc_id);
		}
		priv->vc_id = (u8)vc_id;
	}

	return 0;
}

/*
 * max96717_debugfs_regs_show() - Dump known MAX96717 registers via debugfs
 */
static int max96717_debugfs_regs_show(struct seq_file *s, void *data)
{
	struct max96717_priv *priv = s->private;
	u64 val;
	int ret;
	static const struct {
		u32 reg;
		const char *name;
	} regs[] = {
		/* Identification */
		{MAX96717_DEV_ID,        "DEV_ID"},
		{MAX96717_DEV_REV,       "DEV_REV"},
		/* Device control */
		{MAX96717_REG2,          "REG2 (VID_TX_EN_Z bit6)"},
		/* Reference clock */
		{MAX96717_REG3,          "REG3 (RCLKSEL)"},
		{MAX96717_REG6,          "REG6 (RCLKEN bit5)"},
		/* Video TX */
		{MAX96717_VIDEO_TX0,     "VIDEO_TX0"},
		{MAX96717_VIDEO_TX2,     "VIDEO_TX2 (PCLKDET bit7)"},
		/* Front-top */
		{MAX96717_FRONTOP0,      "FRONTOP0 (START_B bit5)"},
		{MAX96717_FRONTTOP_9,    "FRONTTOP_9 (START_PORTBZ b6)"},
		/* MIPI RX */
		{MAX96717_MIPI_RX1,      "MIPI_RX1 (lane cnt)"},
		{MAX96717_MIPI_RX2,      "MIPI_RX2 (phy1 map)"},
		{MAX96717_MIPI_RX3,      "MIPI_RX3 (phy2 map)"},
		{MAX96717_MIPI_RX4,      "MIPI_RX4 (phy1 pol)"},
		{MAX96717_MIPI_RX5,      "MIPI_RX5 (phy2 pol)"},
		{MAX96717_MIPI_RX_EXT8,  "MIPI_RX_EXT8 (TUN_FIFO_OVF b0)"},
		{MAX96717_MIPI_RX_EXT11, "MIPI_RX_EXT11 (TUN bit7)"},
		{MAX96717_PHY1_PKT_CNT,  "PHY1_PKT_CNT (MIPI in)"},
		{MAX96717_CSI1_PKT_CNT,  "CSI1_PKT_CNT (sensor pkts)"},
		{MAX96717_TUN_PKT_CNT,   "TUN_PKT_CNT (tunnel out)"},
		{MAX96717_PHY_CLK_CNT,   "PHY_CLK_CNT (phy clk)"},
		/* Video TX pattern generator */
		{MAX96717_VTX0,          "VTX0"},
		{MAX96717_VTX1,          "VTX1"},
		{MAX96717_VTX29,         "VTX29 (VTX_MODE)"},
	};
	unsigned int i;

	seq_puts(s, "MAX96717 Register Dump\n");
	seq_puts(s, "======================\n\n");

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		val = 0;
		ret = cci_read(priv->regmap, regs[i].reg, &val, NULL);
		if (ret)
			seq_printf(s, "0x%04llX %-28s: ERROR (%d)\n",
				   (unsigned long long)CCI_REG_ADDR(regs[i].reg),
				   regs[i].name, ret);
		else
			seq_printf(s, "0x%04llX %-28s: 0x%02llX (%llu)\n",
				   (unsigned long long)CCI_REG_ADDR(regs[i].reg),
				   regs[i].name,
				   (unsigned long long)val,
				   (unsigned long long)val);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(max96717_debugfs_regs);

/*
 * debugfs: reg_addr - read/write the target register address (hex, 16-bit)
 */
static int max96717_debugfs_reg_addr_get(void *data, u64 *val)
{
	struct max96717_priv *priv = data;

	*val = priv->debugfs_reg_addr;
	return 0;
}

static int max96717_debugfs_reg_addr_set(void *data, u64 val)
{
	struct max96717_priv *priv = data;

	priv->debugfs_reg_addr = (u16)val;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(max96717_debugfs_reg_addr_fops,
			 max96717_debugfs_reg_addr_get,
			 max96717_debugfs_reg_addr_set,
			 "0x%04llx\n");

/*
 * debugfs: reg_val - read/write a single byte at the address set in reg_addr
 */
static int max96717_debugfs_reg_val_get(void *data, u64 *val)
{
	struct max96717_priv *priv = data;
	u64 regval = 0;
	int ret;

	ret = cci_read(priv->regmap, CCI_REG8(priv->debugfs_reg_addr),
		       &regval, NULL);
	if (ret)
		return ret;

	*val = regval;
	return 0;
}

static int max96717_debugfs_reg_val_set(void *data, u64 val)
{
	struct max96717_priv *priv = data;
	int ret;

	ret = cci_write(priv->regmap, CCI_REG8(priv->debugfs_reg_addr),
			(u8)val, NULL);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(max96717_debugfs_reg_val_fops,
			 max96717_debugfs_reg_val_get,
			 max96717_debugfs_reg_val_set,
			 "0x%02llx\n");

/*
 * debugfs: reg_rw - one-shot "addr [val]" interface
 * Write "0x1234" to read; write "0x1234 0xAB" to write.
 * Read returns "0xADDR = 0xVAL\n".
 */
static ssize_t max96717_debugfs_reg_rw_write(struct file *file,
					     const char __user *ubuf,
					     size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct max96717_priv *priv = s->private;
	char buf[64];
	unsigned int addr, val;
	int ret, matched;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	buf[count] = '\0';

	matched = sscanf(buf, "%x %x", &addr, &val);
	if (matched < 1)
		return -EINVAL;

	if (addr > 0xFFFF)
		return -EINVAL;

	priv->debugfs_reg_addr = (u16)addr;

	if (matched == 2) {
		if (val > 0xFF)
			return -EINVAL;
		ret = cci_write(priv->regmap, CCI_REG8(priv->debugfs_reg_addr),
				(u8)val, NULL);
		if (ret)
			return ret;
	}

	return count;
}

static int max96717_debugfs_reg_rw_show(struct seq_file *s, void *data)
{
	struct max96717_priv *priv = s->private;
	u64 val = 0;
	int ret;

	ret = cci_read(priv->regmap, CCI_REG8(priv->debugfs_reg_addr),
		       &val, NULL);
	if (ret)
		seq_printf(s, "0x%04X: ERROR (%d)\n",
			   priv->debugfs_reg_addr, ret);
	else
		seq_printf(s, "0x%04X = 0x%02llX\n",
			   priv->debugfs_reg_addr,
			   (unsigned long long)val);

	return 0;
}

static int max96717_debugfs_reg_rw_open(struct inode *inode, struct file *file)
{
	return single_open(file, max96717_debugfs_reg_rw_show, inode->i_private);
}

static const struct file_operations max96717_debugfs_reg_rw_fops = {
	.owner		= THIS_MODULE,
	.open		= max96717_debugfs_reg_rw_open,
	.read		= seq_read,
	.write		= max96717_debugfs_reg_rw_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void max96717_debugfs_init(struct max96717_priv *priv)
{
	struct device *dev = &priv->client->dev;

	priv->debugfs_dir = debugfs_create_dir(dev_name(dev), NULL);
	if (IS_ERR(priv->debugfs_dir))
		return;

	debugfs_create_file("registers", 0444, priv->debugfs_dir,
			    priv, &max96717_debugfs_regs_fops);
	debugfs_create_file_unsafe("reg_addr", 0666, priv->debugfs_dir,
				   priv, &max96717_debugfs_reg_addr_fops);
	debugfs_create_file_unsafe("reg_val", 0666, priv->debugfs_dir,
				   priv, &max96717_debugfs_reg_val_fops);
	debugfs_create_file("reg_rw", 0666, priv->debugfs_dir,
			    priv, &max96717_debugfs_reg_rw_fops);
}

static int max96717_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max96717_priv *priv;
	int ret;


	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}

	priv->client = client;
	priv->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		return dev_err_probe(dev, ret, "Failed to init regmap\n");
	}

	ret = max96717_parse_dt(priv);
	if (ret) {
		return dev_err_probe(dev, ret, "Failed to parse the dt\n");
	}

	ret = max96717_hw_init(priv);
	if (ret) {
		return dev_err_probe(dev, ret,
				     "Failed to initialize the hardware\n");
	}

	ret = max96717_gpiochip_probe(priv);
	if (ret) {
		return dev_err_probe(&client->dev, ret,
				     "Failed to init gpiochip\n");
	}

	ret = max96717_register_clkout(priv);
	if (ret) {
		return dev_err_probe(dev, ret, "Failed to register clkout\n");
	}

	ret = max96717_subdev_init(priv);
	if (ret) {
		return dev_err_probe(dev, ret,
				     "Failed to initialize v4l2 subdev\n");
	}

	ret = max96717_i2c_mux_init(priv);
	if (ret) {
		dev_err_probe(dev, ret, "failed to add remote i2c adapter\n");
		max96717_subdev_uninit(priv);
	} else {
		max96717_debugfs_init(priv);
	}

	return ret;
}

static void max96717_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct max96717_priv *priv = sd_to_max96717(sd);

	debugfs_remove_recursive(priv->debugfs_dir);
	max96717_subdev_uninit(priv);
	i2c_mux_del_adapters(priv->mux);
}

static const struct of_device_id max96717_of_ids[] = {
	{ .compatible = "maxim,max96717f" },
	{ }
};
MODULE_DEVICE_TABLE(of, max96717_of_ids);

static struct i2c_driver max96717_i2c_driver = {
	.driver	= {
		.name		= "max96717",
		.of_match_table	= max96717_of_ids,
	},
	.probe		= max96717_probe,
	.remove		= max96717_remove,
};

module_i2c_driver(max96717_i2c_driver);

MODULE_DESCRIPTION("Maxim GMSL2 MAX96717 Serializer Driver");
MODULE_AUTHOR("Julien Massot <julien.massot@collabora.com>");
MODULE_LICENSE("GPL");

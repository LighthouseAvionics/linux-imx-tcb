// SPDX-License-Identifier: GPL-2.0+
/*
 * Maxim MAX96724 GMSL2/1 Deserializer Driver
 *
 * Copyright 2024 NXP
 *
 */
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "max96724_regs.h"

#define MAX96724_DEV_ID				0xA2

/*
 * The sink and source pads are created to match the OF graph port numbers so
 * that their indexes can be used interchangeably.
 */
#define MAX96724_N_GMSL			4

#define MAX96724_N_SINKS		4
#define MAX96724_N_SOURCES		2
#define MAX96724_N_PADS			6
#define MAX96724_SRC_PAD		4

#define MAX96724_XTAL_CLOCK		25000000ULL

enum max96724_data_type {
	MAX96724_DT_EMBEDDED		= 0x12,
	MAX96724_DT_YUV422_8BIT		= 0x1E,
	MAX96724_DT_YUV422_10BIT	= 0x1F,
	MAX96724_DT_YUV422_12BIT	= 0x30,
	MAX96724_DT_RGB565		= 0x22,
	MAX96724_DT_RGB666		= 0x23,
	MAX96724_DT_RGB888		= 0x24,
	MAX96724_DT_RAW8		= 0x2A,
	MAX96724_DT_RAW10		= 0x2B,
	MAX96724_DT_RAW12		= 0x2C,
	MAX96724_DT_RAW14		= 0x2D,
	MAX96724_DT_RAW16		= 0x2E,
	MAX96724_DT_RAW20		= 0x2F,
	MAX96724_DT_YUV422_20BIT	= 0x30,
};

enum max96724_gmsl_speed {
	MAX96724_GMSL_3G		= 1,
	MAX96724_GMSL_6G		= 2,
};


enum max96724_i2c_speed {
	MAX96724_I2C_BPS_9920,
	MAX96724_I2C_BPS_33200,
	MAX96724_I2C_BPS_99200,
	MAX96724_I2C_BPS_123000,
	MAX96724_I2C_BPS_203000,
	MAX96724_I2C_BPS_397000,
	MAX96724_I2C_BPS_625000,
	MAX96724_I2C_BPS_980000,
};

struct max96724_source {
	struct v4l2_subdev *sd;
	struct fwnode_handle *fwnode;
};

struct max96724_asc {
	struct v4l2_async_connection base;
	struct max96724_source *source;
};

struct max96724_priv {
	struct i2c_client *client;
	struct regmap *rmap;

	unsigned int reset_gpio;

	struct i2c_mux_core *mux;
	int mux_chan;

	unsigned int gmsl_link_mask;
	unsigned int gmsl_links_used;
	enum max96724_gmsl_speed gmsl_speed;

	unsigned int source_mask;
	unsigned int nsources;

	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixrate_ctrl;
	struct v4l2_ctrl *link_freq_ctrl;

	struct media_pad pads[MAX96724_N_PADS];
	struct max96724_source sources[MAX96724_N_GMSL];
	struct v4l2_async_notifier notifier;

	struct v4l2_mbus_framefmt sink_fmt[MAX96724_N_SINKS];
	struct v4l2_fract interval;

	/* Protects controls and fmt structures */
	struct mutex lock;

	int csi2_data_lanes[MAX96724_N_SOURCES];
	unsigned int csi2_video_pipe_mask[MAX96724_N_SOURCES];

	/*
	 * Per-source precomputed MIPI PHY register bytes derived from the
	 * standard V4L2 endpoint properties (data-lanes, lane-polarities) on
	 * the deserializer's CSI output endpoint. See
	 * max96724_dt_parse_source_ep() for the translation from DT to these
	 * bytes.
	 *
	 *   mipi_phy_lane_map[i] -> MAX96724_MIPI_PHY_4+i (0x08A4, lane reorder)
	 *   mipi_phy_polarity[i] -> MAX96724_MIPI_PHY_5+i (0x08A5, P/N invert)
	 *
	 * 0x08A5 bit layout:
	 *   [5]PHY1_CLK [4]PHY1_D1 [3]PHY1_D0
	 *   [2]PHY0_CLK [1]PHY0_D1 [0]PHY0_D0
	 */
	u8 mipi_phy_lane_map[MAX96724_N_SOURCES];
	u8 mipi_phy_polarity[MAX96724_N_SOURCES];

	int enable_count;
};

#define to_index(priv, source) ((source) - &(priv)->sources[0])

static bool max96724_writable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX96724_DEV_CTRL12 ... MAX96724_DEV_REG13:
	case MAX96724_TOP_CTRL_PWR0:
	case MAX96724_TOP_CTRL_CTRL3:
	case MAX96724_TOP_CTRL_INTR3:
	case MAX96724_TOP_CTRL_INTR5:
	case MAX96724_TOP_CTRL_INTR7:
	case MAX96724_TOP_CTRL_INTR9:
	case MAX96724_TOP_CTRL_INTR11:
	case MAX96724_TOP_CTRL_VID_PXL_CRC_VIDEOMASK_INT_FLAG:
	case MAX96724_TOP_CTRL_PWR_STATUS_OV_FLAG:
	case MAX96724_TOP_CTRL_VDDCMP_STATUS_FLAG:
	case MAX96724_TOP_CTRL_DEV_REV:
	case MAX96724_TOP_CTRL_EFUSE_CRC_ERR:
	case MAX96724_LINE_FAULT_REG1 ... MAX96724_LINE_FAULT_REG5:
	case MAX96724_VID_RX_VIDEO_RX8(0):
	case MAX96724_VID_RX_VIDEO_RX8(1):
	case MAX96724_VID_RX_VIDEO_RX8(2):
	case MAX96724_VID_RX_VIDEO_RX8(3):
	case MAX96724_GPIO_AGGR0_READ_A_L ... MAX96724_GPIO_AGGR0_READ_CD_H:
	case MAX96724_BACKTOP0_1:
	case MAX96724_BACKTOP0_11:
	case MAX96724_BACKTOP1_12 ... MAX96724_BACKTOP1_15:
	case MAX96724_BACKTOP1_33:
	case MAX96724_FSYNC_ERR_CNT:
	case MAX96724_FSYNC_CALC_FRM_LEN_L ... MAX96724_FSYNC_22:
	case MAX96724_MIPI_PHY_11:
	case MAX96724_MIPI_PHY_17 ... MAX96724_MIPI_PHY_18:
	case MAX96724_MIPI_PHY_25 ... MAX96724_MIPI_PHY_28:
	case MAX96724_MIPI_PHY_ERR_PKT_8 ... MAX96724_MIPI_PHY_ERR_PKT_11:
	case MAX96724_MIPI_TX_STATUS(0):
	case MAX96724_MIPI_TX_STATUS(1):
	case MAX96724_MIPI_TX_STATUS(2):
	case MAX96724_MIPI_TX_STATUS(3):
	case MAX96724_MIPI_TX_52(0):
	case MAX96724_MIPI_TX_52(1):
	case MAX96724_MIPI_TX_52(2):
	case MAX96724_MIPI_TX_52(3):
	case MAX96724_GMSL1_DET_ERR(0) ... MAX96724_GMSL1_CRC_VALUE_3(0):
	case MAX96724_GMSL1_DET_ERR(1) ... MAX96724_GMSL1_CRC_VALUE_3(1):
	case MAX96724_GMSL1_DET_ERR(2) ... MAX96724_GMSL1_CRC_VALUE_3(2):
	case MAX96724_GMSL1_DET_ERR(3) ... MAX96724_GMSL1_CRC_VALUE_3(3):
	case MAX96724_GMSL1_CB(0):
	case MAX96724_GMSL1_CB(1):
	case MAX96724_GMSL1_CB(2):
	case MAX96724_GMSL1_CB(3):
	case MAX96724_VID_HVD_DET_DE_DET ... MAX96724_VID_HVD_DET_VS_POL:
	case MAX96724_VID_HVD_DET_VS_CNT_0 ... MAX96724_VID_HVD_DET_DE_CNT_0_LSB:
	case MAX96724_VID_HVD_DET_VRX_0_CMP_ERR_FLAG:
	case MAX96724_VID_HVD_DET_VS_CNT_1 ... MAX96724_VID_HVD_DET_DE_CNT_1_LSB:
	case MAX96724_VID_HVD_DET_VRX_1_CMP_ERR_FLAG:
	case MAX96724_VID_HVD_DET_VS_CNT_2 ... MAX96724_VID_HVD_DET_DE_CNT_2_LSB:
	case MAX96724_VID_HVD_DET_VRX_2_CMP_ERR_FLAG:
	case MAX96724_VID_HVD_DET_VS_CNT_3 ... MAX96724_VID_HVD_DET_DE_CNT_3_LSB:
	case MAX96724_VID_HVD_DET_VRX_3_CMP_ERR_FLAG:
		return false;

	default:
		return true;
	}
}

static bool max96724_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX96724_DEV_CTRL12 ... MAX96724_DEV_CTRL14:
	case MAX96724_TOP_CTRL_CTRL1 ... MAX96724_TOP_CTRL_CTRL3:
	case MAX96724_TOP_CTRL_INTR3:
	case MAX96724_TOP_CTRL_INTR5:
	case MAX96724_TOP_CTRL_INTR7:
	case MAX96724_TOP_CTRL_INTR9:
	case MAX96724_TOP_CTRL_INTR11:
	case MAX96724_TOP_CTRL_INTR13 ... MAX96724_TOP_CTRL_INTR16:
	case MAX96724_TOP_CTRL_DEC_ERR_A ... MAX96724_TOP_CTRL_IDLE_ERR_D:
	case MAX96724_TOP_CTRL_VID_PXL_CRC_VIDEOMASK_INT_FLAG:
	case MAX96724_TOP_CTRL_PWR_STATUS_OV_FLAG:
	case MAX96724_CFGL_GPIO_ARQ2(0):
	case MAX96724_CFGL_GPIO_ARQ2(1):
	case MAX96724_CFGL_GPIO_ARQ2(2):
	case MAX96724_CFGL_GPIO_ARQ2(3):
	case MAX96724_LINE_FAULT_REG1 ... MAX96724_LINE_FAULT_REG5:
	case MAX96724_GPIO_A_A(0):
	case MAX96724_GPIO_A_A(1):
	case MAX96724_GPIO_A_A(2):
	case MAX96724_GPIO_A_A(3):
	case MAX96724_GPIO_A_A(4):
	case MAX96724_GPIO_A_A(5):
	case MAX96724_GPIO_A_A(6):
	case MAX96724_GPIO_A_A(7):
	case MAX96724_GPIO_A_A(8):
	case MAX96724_GPIO_A_A(9):
	case MAX96724_GPIO_A_A(10):
	case MAX96724_BACKTOP0_1:
	case MAX96724_BACKTOP0_11:
	case MAX96724_BACKTOP1_33 ... MAX96724_BACKTOP1_HDR_ERR_4:
	case MAX96724_BACKTOP1_SRAM_LCRC_ERR:
	case MAX96724_FSYNC_ERR_CNT:
	case MAX96724_FSYNC_22:
	case MAX96724_MIPI_PHY_11:
	case MAX96724_MIPI_PHY_17:
	case MAX96724_MIPI_PHY_25 ... MAX96724_MIPI_PHY_28:
	case MAX96724_VID_PXL_CRC_ERR_AX ... MAX96724_VID_PXL_CRC_ERR_DU:
	case MAX96724_VID_HVD_DET_DE_DET ... MAX96724_VID_HVD_DET_VS_POL:
		return true;

	default:
		return false;
	}
}

static const struct regmap_config max96724_regmap_cfg = {
	.name = "max96724",
	.reg_bits = 16,
	.val_bits = 8,

	.max_register = MAX96724_TUN_DET_CPHY_DET,
	.writeable_reg = max96724_writable_register,
	.volatile_reg = max96724_volatile_reg,

	.cache_type = REGCACHE_RBTREE,
};

struct max96724_format_info {
	u32 code;
	u8 data_type;
};

static const struct max96724_format_info max96724_formats[] = {
	/* YUV formats */
	{
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.data_type = MAX96724_DT_YUV422_8BIT,
	}, {
		.code = MEDIA_BUS_FMT_VYUY8_1X16,
		.data_type = MAX96724_DT_YUV422_8BIT,
	}, {
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.data_type = MAX96724_DT_YUV422_8BIT,
	}, {
		.code = MEDIA_BUS_FMT_YVYU8_1X16,
		.data_type = MAX96724_DT_YUV422_8BIT,
	}, {
		.code = MEDIA_BUS_FMT_UYVY10_1X20,
		.data_type = MAX96724_DT_YUV422_10BIT,
	}, {
		.code = MEDIA_BUS_FMT_VYUY10_1X20,
		.data_type = MAX96724_DT_YUV422_10BIT,
	}, {
		.code = MEDIA_BUS_FMT_YUYV10_1X20,
		.data_type = MAX96724_DT_YUV422_10BIT,
	}, {
		.code = MEDIA_BUS_FMT_YVYU10_1X20,
		.data_type = MAX96724_DT_YUV422_10BIT,
	},
	/* RGB formats */
	{
		.code = MEDIA_BUS_FMT_RGB565_1X16,
		.data_type = MAX96724_DT_RGB565,
	}, {
		.code = MEDIA_BUS_FMT_BGR888_1X24,
		.data_type = MAX96724_DT_RGB888,
	},
	/* RAW formats */
	{
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.data_type = MAX96724_DT_RAW8,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.data_type = MAX96724_DT_RAW8,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.data_type = MAX96724_DT_RAW8,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.data_type = MAX96724_DT_RAW8,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.data_type = MAX96724_DT_RAW10,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.data_type = MAX96724_DT_RAW10,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.data_type = MAX96724_DT_RAW10,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.data_type = MAX96724_DT_RAW10,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.data_type = MAX96724_DT_RAW12,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.data_type = MAX96724_DT_RAW12,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.data_type = MAX96724_DT_RAW12,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.data_type = MAX96724_DT_RAW12,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR14_1X14,
		.data_type = MAX96724_DT_RAW14,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG14_1X14,
		.data_type = MAX96724_DT_RAW14,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG14_1X14,
		.data_type = MAX96724_DT_RAW14,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB14_1X14,
		.data_type = MAX96724_DT_RAW14,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR16_1X16,
		.data_type = MAX96724_DT_RAW16,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG16_1X16,
		.data_type = MAX96724_DT_RAW16,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG16_1X16,
		.data_type = MAX96724_DT_RAW16,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB16_1X16,
		.data_type = MAX96724_DT_RAW16,
	},
};

static const struct v4l2_mbus_framefmt max96724_default_format = {
	.width		= 1920,
	.height		= 1280,
	.code		= MEDIA_BUS_FMT_SBGGR16_1X16,
	.colorspace	= V4L2_COLORSPACE_RAW,
	.xfer_func	= V4L2_XFER_FUNC_DEFAULT,
};

static int max96724_i2c_mux_select(struct i2c_mux_core *muxc, u32 chan)
{
	struct max96724_priv *priv = i2c_mux_priv(muxc);
	u8 val = 0xff;


	if (priv->mux_chan == chan) {
		return 0;
	}

	val &= ~(0x3 << (chan * 2));
	val |= 0x2 << (chan * 2);
	regmap_write(priv->rmap, MAX96724_DEV_REG3, val);

	priv->mux_chan = chan;

	return 0;
}

static int max96724_i2c_init(struct max96724_priv *priv)
{
	int link;
	int ret;


	if (!i2c_check_functionality(priv->client->adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA)) {
		return -ENODEV;
	}

	priv->mux_chan = -1;

	priv->mux = i2c_mux_alloc(priv->client->adapter, &priv->client->dev,
				  priv->gmsl_links_used, 0, I2C_MUX_LOCKED,
				  max96724_i2c_mux_select, NULL);
	if (!priv->mux) {
		return -ENOMEM;
	}

	priv->mux->priv = priv;

	for (link = 0; link < MAX96724_N_GMSL; link++) {
		if (!(priv->gmsl_link_mask & BIT(link)))
			continue;

		ret = i2c_mux_add_adapter(priv->mux, 0, link);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	i2c_mux_del_adapters(priv->mux);
	return ret;
}

static int max96724_i2c_parse_dt(struct max96724_priv *priv)
{
	struct device *dev = &priv->client->dev;
	struct device_node *i2c_mux;
	struct device_node *node = NULL;


	i2c_mux = of_find_node_by_name(dev->of_node, "i2c-mux");
	if (!i2c_mux) {
		dev_err(dev, "Failed to find i2c-mux node\n");
		return -EINVAL;
	}

	/* Identify which i2c-mux channels are enabled */
	for_each_child_of_node(i2c_mux, node) {
		u32 id = 0;

		of_property_read_u32(node, "reg", &id);
		if (id >= MAX96724_N_GMSL)
			continue;

		if (!of_device_is_available(node)) {
			dev_dbg(dev, "Skipping disabled I2C bus port %u\n", id);
			continue;
		}

		priv->gmsl_link_mask |= BIT(id);
		priv->gmsl_links_used++;
	}
	of_node_put(node);
	of_node_put(i2c_mux);
	of_node_put(dev->of_node);

	return 0;
};

static int max96724_dt_parse_source_ep(struct max96724_priv *priv, struct device_node *node,
				       struct of_endpoint *ep)
{
	struct device *dev = &priv->client->dev;
	struct v4l2_fwnode_endpoint vep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int csi_port;
	int ret;


	if (ep->id < 4) {
		return 0;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node), &vep);
	if (ret) {
		of_node_put(node);
		return ret;
	}

	csi_port = ep->port - MAX96724_SRC_PAD;

	if (csi_port > 1) {
		dev_err(dev, "Wrong CSI port, deserializer has only 2 ports.\n");
		return -EINVAL;
	}

	priv->csi2_data_lanes[csi_port] = vep.bus.mipi_csi2.num_data_lanes;

	if (priv->csi2_data_lanes[csi_port] != 2 && priv->csi2_data_lanes[csi_port] != 4) {
		dev_err(dev, "Only 2 or 4 lanes are supported.");
		return -EINVAL;
	}

	/*
	 * Translate V4L2 endpoint data-lanes[] / lane-polarities[] into the
	 * MAX96724's per-PHY register bytes.
	 *
	 * data-lanes = <L0 L1 L2 L3>  (1-based logical lane numbers at each
	 *   physical position) -> MAX96724_MIPI_PHY_4 (0x08A4) with two-bit
	 *   fields per physical slot encoding (logical - 1).
	 *
	 * lane-polarities = <clk, d1, d2, d3, d4>  (0 = normal, 1 = inverted)
	 *   -> MAX96724_MIPI_PHY_5 (0x08A5). In 2x4 combined mode on a port,
	 *   PHY0 + PHY1 both appear; clock polarity applies to both; data
	 *   polarities map physical 0->PHY0_D0 (bit 0), physical 1->PHY0_D1
	 *   (bit 1), physical 2->PHY1_D0 (bit 3), physical 3->PHY1_D1 (bit 4).
	 */
	{
		const u8 *lanes = vep.bus.mipi_csi2.data_lanes;
		const bool *pol = vep.bus.mipi_csi2.lane_polarities;
		unsigned int n = vep.bus.mipi_csi2.num_data_lanes;
		u8 map = 0;
		u8 polarity = 0;
		unsigned int i;

		/*
		 * Lane reorder byte. Each physical slot's two-bit field encodes
		 * the logical lane number - 1. Slots beyond num_data_lanes are
		 * written as 0 (DT-authoritative, no vendor-default padding).
		 */
		for (i = 0; i < n; i++)
			map |= ((lanes[i] - 1) & 0x3) << (2 * i);
		priv->mipi_phy_lane_map[csi_port] = map;

		/*
		 * Polarity byte. pol[0] = clock, pol[1..n] = data lanes 1..n.
		 * Physical slot -> PHY register bit:
		 *   clock -> PHY0_CLK (bit 2) + PHY1_CLK (bit 5)
		 *   data 0 -> PHY0_D0 (bit 0)
		 *   data 1 -> PHY0_D1 (bit 1)
		 *   data 2 -> PHY1_D0 (bit 3)
		 *   data 3 -> PHY1_D1 (bit 4)
		 */
		if (pol[0]) polarity |= BIT(2) | BIT(5);
		if (n >= 1 && pol[1]) polarity |= BIT(0);
		if (n >= 2 && pol[2]) polarity |= BIT(1);
		if (n >= 3 && pol[3]) polarity |= BIT(3);
		if (n >= 4 && pol[4]) polarity |= BIT(4);
		priv->mipi_phy_polarity[csi_port] = polarity;

		dev_info(dev,
			 "CSI%u lane_map=0x%02x polarity=0x%02x (from DT bindings)\n",
			 csi_port, map, polarity);
	}

	return 0;
}

static int max96724_dt_parse_sink_ep(struct max96724_priv *priv, struct device_node *node,
				     struct of_endpoint *ep)
{
	struct device *dev = &priv->client->dev;
	struct max96724_source *source;
	struct device_node *csi_ep;
	struct of_endpoint csi_of_ep;
	unsigned int csi_port;


	/* Skip if the corresponding GMSL link is unavailable. */
	if (!(priv->gmsl_link_mask & BIT(ep->port))) {
		return 0;
	}

	if (ep->id == 1) {
		csi_ep = of_graph_get_remote_endpoint(node);
		of_graph_parse_endpoint(csi_ep, &csi_of_ep);

		csi_port = csi_of_ep.port - MAX96724_SRC_PAD;

		if (csi_port > 1) {
			dev_err(dev, "Wrong CSI port, deserializer has only 2 ports.\n");
			return -EINVAL;
		}

		priv->csi2_video_pipe_mask[csi_port] |= BIT(ep->port);
		of_node_put(csi_ep);
		return 0;
	}

	if (priv->sources[ep->port].fwnode) {
		dev_info(dev, "Multiple port endpoints are not supported: %d", ep->port);
		return 0;
	}

	source = &priv->sources[ep->port];
	source->fwnode = fwnode_graph_get_remote_endpoint(of_fwnode_handle(node));
	if (!source->fwnode) {
		dev_info(dev, "Endpoint %pOF has no remote endpoint connection\n", ep->local_node);
		return 0;
	}

	priv->source_mask |= BIT(ep->port);
	priv->nsources++;

	return 0;
}

static int max96724_parse_dt(struct max96724_priv *priv)
{
	struct device *dev = &priv->client->dev;
	struct device_node *node = NULL;
	int ret;


	for_each_endpoint_of_node(dev->of_node, node) {
		struct of_endpoint ep;

		of_graph_parse_endpoint(node, &ep);
		dev_dbg(dev, "Endpoint %pOF on port %d", ep.local_node, ep.port);

		if (ep.port >= MAX96724_N_PADS) {
			dev_err(dev, "Invalid endpoint %s on port %d",
				of_node_full_name(ep.local_node), ep.port);
			continue;
		}

		if (ep.port >= MAX96724_SRC_PAD) {
			ret = max96724_dt_parse_source_ep(priv, node, &ep);
			if (ret) {
				return ret;
			}

			continue;
		}

		ret = max96724_dt_parse_sink_ep(priv, node, &ep);
		if (ret) {
			return ret;
		}
	}
	of_node_put(node);

	return 0;
}

static int max96724_gmsl_speed_set(struct max96724_priv *priv,
				   enum max96724_gmsl_speed speed)
{
	int ret;


	ret = regmap_update_bits(priv->rmap, MAX96724_DEV_REG26,
				 RX_RATE_PHYA_MASK | RX_RATE_PHYB_MASK,
				 ((priv->gmsl_link_mask & 0x1) ? (speed << RX_RATE_PHYA_SHIFT) : 0) |
				 ((priv->gmsl_link_mask & 0x2) ? (speed << RX_RATE_PHYB_SHIFT) : 0));
	ret |= regmap_update_bits(priv->rmap, MAX96724_DEV_REG27,
				  RX_RATE_PHYC_MASK | RX_RATE_PHYD_MASK,
				 ((priv->gmsl_link_mask & 0x4) ? (speed << RX_RATE_PHYC_SHIFT) : 0) |
				 ((priv->gmsl_link_mask & 0x8) ? (speed << RX_RATE_PHYD_SHIFT) : 0));

	if (ret ? -EIO : 0) {
		return -EIO;
	}
	return 0;
}

static unsigned int max96724_check_gmsl_links(struct max96724_priv *priv)
{
	unsigned int locked_links_mask = 0;
	unsigned int links_mask = priv->gmsl_link_mask;
	u16 link_lock_addr[4] = {
		MAX96724_TOP_CTRL_CTRL3,
		MAX96724_DEV_CTRL12,
		MAX96724_DEV_CTRL13,
		MAX96724_DEV_CTRL14
	};
	unsigned long timeout;

	regmap_update_bits(priv->rmap, MAX96724_DEV_REG6,
			   LINK_EN_A | LINK_EN_B | LINK_EN_C | LINK_EN_D,
			   priv->gmsl_link_mask);

	timeout = jiffies + msecs_to_jiffies(100);

	while (!time_after(jiffies, timeout)) {
		int current_link = ffs(links_mask) - 1;

		if (current_link == -1)
			break;

		if (regmap_test_bits(priv->rmap, link_lock_addr[current_link], BIT(3)))
			locked_links_mask |= BIT(current_link);

		links_mask &= ~BIT(current_link);

		if (!links_mask && priv->gmsl_link_mask == locked_links_mask)
			break;
		else if (!links_mask)
			links_mask = priv->gmsl_link_mask & ~locked_links_mask;

		usleep_range(1000, 2000);
	}

	return locked_links_mask;
}

static int max96724_chip_init(struct max96724_priv *priv)
{
	unsigned int locked_links;
	struct device *dev = &priv->client->dev;
	int i, retries = 3;


	/*
	 * Per MAX96724FR User Guide: configure pixel/tunnel mode BEFORE link
	 * initialization. Enable tunnel mode manually on all 4 pipes and
	 * disable auto-detect; without DIS_AUTO_TUN_DET=1 the auto-detect
	 * can silently fail to arm when TUN_EN is manually set.
	 *
	 * 0x0936..0x09F6 TX_54 = 0x09 -> TUN_EN=1, default lane-count bit preserved
	 * 0x0939..0x09F9 TX_57 = 0x50 -> DIS_AUTO_TUN_DET=1, TUN_DEST=1 (PHY1)
	 */
	for (i = 0; i < 4; i++) {
		regmap_write(priv->rmap, MAX96724_MIPI_TX_54(i), 0x09);
		regmap_write(priv->rmap, MAX96724_MIPI_TX_57(i), 0x50);
	}
	/*
	 * User guide: "In tunneling mode, the mappings in the MIPI_CTRL_SEL_x
	 * bits must match the TUN_DEST selections." TUN_DEST=1 on all pipes,
	 * so point every pipe at Ctrl 1 (= PHY 1). 2 bits per pipe, value 1 for
	 * each pipe -> 0x55.
	 */
	regmap_write(priv->rmap, MAX96724_MIPI_PHY_MIPI_CTRL_SEL, 0x55);

	/*
	 * MIPI TX DPLL frequency: IMX477 emits 1200 Mbps/lane, tunnel mode
	 * preserves rate end-to-end. Default is 1500 Mbps/lane which is wrong.
	 * Value 0x0C (=12) * 100MHz = 1200 MHz DPLL / 1200 Mbps/lane.
	 * Bit 5 (PREDEF_EN) must also be set to select predefined frequency.
	 * Set on all 4 PHY DPLLs.
	 */
	regmap_write(priv->rmap, MAX96724_BACKTOP0_22, 0x20 | 0x0C); /* PHY 0 */
	regmap_write(priv->rmap, MAX96724_BACKTOP0_25, 0x20 | 0x0C); /* PHY 1 */
	regmap_write(priv->rmap, MAX96724_BACKTOP0_28, 0x20 | 0x0C); /* PHY 2 */
	regmap_write(priv->rmap, MAX96724_BACKTOP0_31, 0x20 | 0x0C); /* PHY 3 */

	/* Link reset so the new mode takes effect. */
	regmap_write(priv->rmap, MAX96724_TOP_CTRL_CTRL1,
		     RESET_ONESHOT_A | RESET_ONESHOT_B |
		     RESET_ONESHOT_C | RESET_ONESHOT_D);
	msleep(100);

	/* Set GMSL link speed from DT before checking for lock */
	if (priv->gmsl_speed) {
		dev_info(dev, "Setting GMSL link speed to %s\n",
			 priv->gmsl_speed == MAX96724_GMSL_3G ? "3Gbps" : "6Gbps");
		max96724_gmsl_speed_set(priv, priv->gmsl_speed);
		/* One-shot reset all links to renegotiate at new speed */
		regmap_write(priv->rmap, MAX96724_TOP_CTRL_CTRL1,
			     RESET_ONESHOT_A | RESET_ONESHOT_B |
			     RESET_ONESHOT_C | RESET_ONESHOT_D);
		msleep(100);
	}

	while (retries--) {
		locked_links = max96724_check_gmsl_links(priv);
		if (locked_links == priv->gmsl_link_mask)
			break;
		if (locked_links != 0)
			break;

		regmap_write(priv->rmap, MAX96724_TOP_CTRL_PWR1, RESET_ALL);
		usleep_range(2000, 2500);

		/* RESET_ALL reverts speed to default, re-apply */
		if (priv->gmsl_speed) {
			max96724_gmsl_speed_set(priv, priv->gmsl_speed);
			regmap_write(priv->rmap, MAX96724_TOP_CTRL_CTRL1,
				     RESET_ONESHOT_A | RESET_ONESHOT_B |
				     RESET_ONESHOT_C | RESET_ONESHOT_D);
			msleep(100);
		}
	}

	if (locked_links == 0) {
		dev_err(dev, "No GMSL link has locked after 3 retries. Abort!\n");
		return -ENODEV;
	}

	dev_info(dev, "GMSL link mask: configured = 0x%x, locked = 0x%x\n",
		 priv->gmsl_link_mask, locked_links);

	/* Disable links that didn't lock. Perhaps user didn't connect all sensors. */
	regmap_update_bits(priv->rmap, MAX96724_DEV_REG6,
			   LINK_EN_A | LINK_EN_B | LINK_EN_C | LINK_EN_D,
			   locked_links);

	/* Disable remote control channel on all links. */
	regmap_write(priv->rmap, MAX96724_DEV_REG3, 0xff);

	/* Disable all video pipes access. */
	regmap_write(priv->rmap, MAX96724_DEV_REG4, 0);

	/* Disable all MIPI PHYs. */
	regmap_update_bits(priv->rmap, MAX96724_MIPI_PHY_2, PHY_STDBY_N_MASK, 0);

	/* Disable CSI output. */
	regmap_update_bits(priv->rmap, MAX96724_BACKTOP0_12, CSI_OUT_EN, 0);

	/* Disable all video pipes */
	regmap_write(priv->rmap, MAX96724_VIDEO_PIPE_SEL_VIDEO_PIPE_EN, 0);

	/* Set I2C speed on al GMSL ports to 980kbps */
	for (i = 0; i < 4; i++)
		regmap_update_bits(priv->rmap, MAX96724_CC_G2P0_I2C_1(i), MST_BT_P0_A_MASK,
				   MAX96724_I2C_BPS_980000 << MST_BT_P0_A_SHIFT);

	priv->gmsl_link_mask = locked_links;
	priv->gmsl_links_used = hweight8(locked_links);

	return 0;
}

static int max96724_vc_mapping_set(struct max96724_priv *priv, int pipe, int map_no,
				   int src_vc, enum max96724_data_type src_dt,
				   int dst_vc, enum max96724_data_type dst_dt)
{
	int ret;


	ret = regmap_write(priv->rmap, MAX96724_MIPI_TX_MAP_SRC(pipe, map_no),
			   (src_vc << 6) | (src_dt & 0x3f));
	ret |= regmap_write(priv->rmap, MAX96724_MIPI_TX_MAP_DST(pipe, map_no),
			    (dst_vc << 6) | (dst_dt & 0x3f));

	if (ret) {
		return -EIO;
	}
	return 0;
}

/* Each set bit in the mapping_mask will enable a mapping. */
static int max96724_vc_mapping_en(struct max96724_priv *priv, int pipe, u16 mapping_mask)
{
	int ret;


	ret = regmap_write(priv->rmap, MAX96724_MIPI_TX_MAP_EN_H(pipe), mapping_mask >> 8);
	ret |= regmap_write(priv->rmap, MAX96724_MIPI_TX_MAP_EN_L(pipe), mapping_mask & 0xff);

	if (ret) {
		return -EIO;
	}
	return 0;
}

/* Each VC src/dst map with a set bit in mapping_mask will be routed to dphy_no. */
static int max96724_vc_dphy_dst_select(struct max96724_priv *priv, int pipe,
				       unsigned long *mapping_mask, int dphy_no)
{
	int i, bit;
	int base_dphy_map_addr = MAX96724_MIPI_TX_45(pipe);
	u8 offs;
	u8 dphy_map[4] = {0};
	int ret = 0;


	for_each_set_bit(bit, mapping_mask, 16) {
		offs = bit / 4;
		dphy_map[offs] |= dphy_no << ((bit & 0x3) * 2);
	}

	for (i = 0; i < 4; i++)
		ret |= regmap_write(priv->rmap, base_dphy_map_addr + i, dphy_map[i]);

	if (ret) {
		return -EIO;
	}
	return 0;
}

static int max96724_assign_pipe_to_mipi_ctrl(struct max96724_priv *priv, int pipe)
{
	int ctrl_sel = 0;
	int csi_out_port = 0;
	int i;
	int ret;


	for (i = 0; i < MAX96724_N_SOURCES; i++) {
		if (priv->csi2_video_pipe_mask[i] & BIT(pipe)) {
			csi_out_port = i;
			break;
		}
	}

	/*
	 * Tunnel mode: MIPI_CTRL_SEL must match TUN_DEST (set to 1 = PHY 1
	 * in chip_init for all pipes). In 2x4 mode, CSI-2 port A's master is
	 * PHY 1 / Ctrl 1 anyway, which matches.
	 */
	if (priv->csi2_data_lanes[csi_out_port] == 4)
		ctrl_sel = csi_out_port == 0 ? 1 : 2;
	else
		ctrl_sel = 1;  /* 4x2 mode tunnel: route to PHY 1 to match TUN_DEST */

	ret = regmap_update_bits(priv->rmap, MAX96724_MIPI_PHY_MIPI_CTRL_SEL, 0x3 << pipe * 2,
				  ctrl_sel << pipe * 2);
	return ret;
}

static int max96724_pipe_setup(struct max96724_priv *priv, int pipe,
			       struct v4l2_subdev_state *state)
{
	int ret, i;
	struct v4l2_mbus_framefmt *format;
	const struct max96724_format_info *info = NULL;
	unsigned long mapping_mask;
	u8 dt_vc0;
	int csi_port = (priv->csi2_video_pipe_mask[0] & BIT(pipe)) ? 0 : 1;
	u8 pos_shift;


	/* the pixel data is on VC0 */
	format = v4l2_subdev_state_get_format(state, pipe, 0);
	if (!format) {
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(max96724_formats); ++i) {
		if (max96724_formats[i].code == format->code) {
			info = &max96724_formats[i];
			break;
		}
	}

	if (!info) {
		dev_err(&priv->client->dev, "Format not supported.\n");
		return -ENOTSUPP;
	}

	/*
	 * VIDEO_PIPE_SEL nibble: bits[3:2]=link, bits[1:0]=stream ID.
	 * MAX96717 only has pipe Z which transmits on stream ID 2, so
	 * the MAX96724 deserializer pipe must listen for stream ID 2
	 * on the matching link. One pipe per link: link = pipe index.
	 */
	pos_shift = (pipe & 0x1) * 4;
	ret = regmap_update_bits(priv->rmap, MAX96724_VIDEO_PIPE_SEL_VIDEO_PIPE_SEL_0 + pipe / 2,
				 0xf << pos_shift,
				 ((pipe << 2) | 0x2) << pos_shift);
	ret |= regmap_update_bits(priv->rmap, MAX96724_DEV_REG4, BIT(pipe), BIT(pipe));
	ret |= regmap_update_bits(priv->rmap, MAX96724_VIDEO_PIPE_SEL_VIDEO_PIPE_EN,
				  BIT(pipe), BIT(pipe));

	/*
	 * In tunnel mode (MAX96717 is tunnel-only), the VC/DT mapper
	 * sits before the TUN_EN bypass and will filter out raw tunneled
	 * packets that can't be parsed as pixel-mode payload. Disable the
	 * mapper by clearing MAP_EN so packets flow through to tunnel.
	 */
	mapping_mask = 0;
	dt_vc0 = info->data_type;

	ret |= max96724_vc_mapping_en(priv, pipe, mapping_mask);
	ret |= max96724_vc_mapping_set(priv, pipe, 0, 0, dt_vc0, pipe, dt_vc0);
	ret |= max96724_vc_mapping_set(priv, pipe, 1, 0, 0x00, pipe, 0x00); /* frame-start */
	ret |= max96724_vc_mapping_set(priv, pipe, 2, 0, 0x01, pipe, 0x01); /* frame-end */
	ret |= max96724_vc_mapping_set(priv, pipe, 3, 0, MAX96724_DT_EMBEDDED,
				       pipe, MAX96724_DT_EMBEDDED);

	ret |= max96724_vc_dphy_dst_select(priv, pipe, &mapping_mask, csi_port == 0 ? 1 : 2);
	/*
	 * Write lane count AND explicitly clear CSI2_VCX_EN (bit 4) and
	 * CSI2_CPHY_EN (bit 5). On this chip rev the power-on default has
	 * CSI2_VCX_EN set, which makes the MIPI TX expect 4-bit VCs but the
	 * tunnel payload from MAX96717 uses standard 2-bit VCs - mismatch
	 * causes no packets to be emitted.
	 */
	ret |= regmap_update_bits(priv->rmap, MAX96724_MIPI_TX_10(csi_port == 0 ? 1 : 2),
			CSI2_LANE_CNT_MASK | CSI2_CPHY_EN | CSI2_VCX_EN,
			(priv->csi2_data_lanes[csi_port] - 1) << CSI2_LANE_CNT_SHIFT);

	/* Disable diskew */
	ret |= regmap_write(priv->rmap, MAX96724_MIPI_TX_DESKEW_INIT(csi_port == 0 ? 1 : 2), 0x07);
	ret |= regmap_write(priv->rmap, MAX96724_MIPI_TX_DESKEW_PER(csi_port == 0 ? 1 : 2), 0x01);

	/* Tunnel mode (TUN_EN, TUN_DEST, DIS_AUTO_TUN_DET) is set in chip_init
	 * before link initialization, per the MAX96724FR user guide. */

	ret |= max96724_assign_pipe_to_mipi_ctrl(priv, pipe);

	if (ret) {
		return -EIO;
	}
	return 0;
}

static int max96724_dphy_config(struct max96724_priv *priv)
{
	int i;
	struct dphy_setting {
		int port_a_lanes;
		int port_b_lanes;
		u8 setting;
	} dphy_settings[] = {
		{.port_a_lanes = 4, .port_b_lanes = 4, .setting = PHY_2X4},
		{.port_a_lanes = 4, .port_b_lanes = 2, .setting = PHY_1X4A_22},
		{.port_a_lanes = 4, .port_b_lanes = 0, .setting = PHY_2X4},
		{.port_a_lanes = 2, .port_b_lanes = 4, .setting = PHY_1X4B_22},
		{.port_a_lanes = 2, .port_b_lanes = 2, .setting = PHY_4X2},
		/*
		 * Port A only, 2 lanes: the board wires MAX96724 Port A (which
		 * is PHY0+PHY1 combined in 2x4 mode) to the host CSI. Even
		 * though we only use 2 data lanes, we must use 2x4 mode so the
		 * physical pads on Port A are driven. 4x2 would put the output
		 * on Port C/D pads which this board doesn't route.
		 */
		{.port_a_lanes = 2, .port_b_lanes = 0, .setting = PHY_2X4},
		{.port_a_lanes = 0, .port_b_lanes = 4, .setting = PHY_2X4},
		{.port_a_lanes = 0, .port_b_lanes = 2, .setting = PHY_4X2},
	};


	for (i = 0; i < ARRAY_SIZE(dphy_settings); i++) {
		if (priv->csi2_data_lanes[0] == dphy_settings[i].port_a_lanes &&
		    priv->csi2_data_lanes[1] == dphy_settings[i].port_b_lanes) {
			regmap_write(priv->rmap, MAX96724_MIPI_PHY_0, dphy_settings[i].setting);
			break;
		}
	}

	if (i == ARRAY_SIZE(dphy_settings)) {
		return -EINVAL;
	}

	for (i = 0; i < MAX96724_N_SOURCES; i++) {
		/*
		 * Lane mapping (physical->logical) and polarity come exclusively
		 * from the standard V4L2 endpoint bindings (data-lanes,
		 * lane-polarities) parsed in max96724_dt_parse_source_ep().
		 *
		 * Register layout:
		 *   0x08A3 (MIPI_PHY_3) = PHY0+PHY1 lane map  -> source 0 (Port A)
		 *   0x08A4 (MIPI_PHY_4) = PHY2+PHY3 lane map  -> source 1 (Port B)
		 *   0x08A5 (MIPI_PHY_5) = PHY0+PHY1 polarity  -> source 0
		 *   0x08A6 (MIPI_PHY_6) = PHY2+PHY3 polarity  -> source 1
		 *
		 * Note: the original driver used MIPI_PHY_4+i as the lane-map
		 * base, which was off-by-one — it wrote Port A's map into
		 * Port B's register. That was harmless when only one source was
		 * configured (Port A's reset default already matched intent).
		 * Using the correct base here so both sources can coexist.
		 */
		regmap_write(priv->rmap, MAX96724_MIPI_PHY_3 + i,
			     priv->mipi_phy_lane_map[i]);
		regmap_write(priv->rmap, MAX96724_MIPI_PHY_5 + i,
			     priv->mipi_phy_polarity[i]);
	}

	return 0;
}

static int max96724_phy_enable(struct max96724_priv *priv, bool en)
{
	int i;
	u8 phy_en = 0;
	int ret;


	/*
	 * In 2x4 mode, Port A combines PHY 0+1 (PHY 1 = master/clock). Even
	 * with 2 active data lanes on Port A, BOTH PHY 0 and PHY 1 must be
	 * taken out of standby because the physical pads of both PHYs make
	 * up Port A. Same story for Port B = PHY 2+3.
	 */
	for (i = 0; i < MAX96724_N_SOURCES; i++)
		phy_en |= (priv->csi2_data_lanes[i] >= 2 ? 0x3 : 0) << (2 * i);

	ret = regmap_update_bits(priv->rmap, MAX96724_MIPI_PHY_2,
				 PHY_STDBY_N_MASK, en ? phy_en << PHY_STDBY_N_SHIFT : 0);

	ret |= regmap_update_bits(priv->rmap, MAX96724_BACKTOP0_12, CSI_OUT_EN,
				  en ? CSI_OUT_EN : 0);

	if (ret) {
		return -EIO;
	}
	return 0;
}

static int max96724_pipes_disable_all(struct max96724_priv *priv)
{
	int ret;


	ret = regmap_update_bits(priv->rmap, MAX96724_VIDEO_PIPE_SEL_VIDEO_PIPE_EN,
				  VIDEO_PIPE_EN_MASK, 0);

	return ret;
}

static int max96724_fsync_set(struct max96724_priv *priv)
{
	int ret;


	/*
	 * According to Max96724 users guide, "sync signal frequency must be specified in terms of
	 * the onboard crystal clock (25MHz)"
	 */
	u32 fsync;

	if (!priv->interval.numerator || !priv->interval.denominator) {
		ret = regmap_update_bits(priv->rmap, MAX96724_FSYNC_0, FSYNC_METH_MASK, 0);
		return ret;
	}

	fsync = div_u64(MAX96724_XTAL_CLOCK * priv->interval.numerator, priv->interval.denominator);

	ret = regmap_write(priv->rmap, MAX96724_FSYNC_PERIOD_H, (fsync >> 16) & 0xff);
	ret |= regmap_write(priv->rmap, MAX96724_FSYNC_PERIOD_M, (fsync >> 8) & 0xff);
	ret |= regmap_write(priv->rmap, MAX96724_FSYNC_PERIOD_L, (fsync >> 0) & 0xff);

	ret |= regmap_update_bits(priv->rmap, MAX96724_FSYNC_0, FSYNC_METH_MASK | FSYNC_MODE_MASK,
				  (0x0 << FSYNC_METH_SHIFT) | (0x1 << FSYNC_MODE_SHIFT));

	ret |= regmap_write(priv->rmap, MAX96724_FSYNC_17, 0x00);
	ret |= regmap_write(priv->rmap, MAX96724_FSYNC_2, 0x00);

	if (ret) {
		return -EIO;
	}
	return 0;
}

/* V4L2 stuff */
static const struct media_entity_operations max96724_v4l2_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int max96724_s_ctrl(struct v4l2_ctrl *ctrl)
{

	switch (ctrl->id) {
	case V4L2_CID_PIXEL_RATE:
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops max96724_v4l2_ctrl_ops = {
	.s_ctrl = max96724_s_ctrl,
};

static int max96724_setup_all_pipes(struct max96724_priv *priv, struct v4l2_subdev_state *state)
{
	int i, ret;


	for (i = 0; i < MAX96724_N_SINKS; i++) {
		struct max96724_source *source = &priv->sources[i];

		if (!source->fwnode)
			continue;

		ret = max96724_pipe_setup(priv, i, state);
		if (ret == -EIO) {
			return ret;
		}
	}

	return 0;
}

static int max96724_get_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       struct v4l2_subdev_frame_interval *interval)
{
	struct max96724_priv *priv = container_of(sd, struct max96724_priv, sd);


	if (interval->pad < MAX96724_SRC_PAD) {
		return -EINVAL;
	}

	interval->interval = priv->interval;

	return 0;
}

static int max96724_set_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       struct v4l2_subdev_frame_interval *interval)
{
	struct max96724_priv *priv = container_of(sd, struct max96724_priv, sd);


	if (interval->pad < MAX96724_SRC_PAD) {
		return -EINVAL;
	}

	priv->interval = interval->interval;

	return 0;
}

static int max96724_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			    struct v4l2_subdev_format *format)
{
	unsigned int pad = format->pad;
	struct v4l2_mbus_framefmt *sink_fmt;
	struct v4l2_subdev_route *route;
	int i;


	if (pad >= MAX96724_SRC_PAD) {
		int ret = v4l2_subdev_get_fmt(sd, state, format);
		return ret;
	}

	/* Validate the format. */
	for (i = 0; i < ARRAY_SIZE(max96724_formats); ++i) {
		if (max96724_formats[i].code == format->format.code)
			break;
	}

	if (i == ARRAY_SIZE(max96724_formats))
		format->format.code = max96724_formats[12].code;

	sink_fmt = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (!sink_fmt) {
		return -EINVAL;
	}

	*sink_fmt = format->format;

	for_each_active_route(&state->routing, route) {
		struct v4l2_mbus_framefmt *source_fmt;

		if (route->sink_pad != format->pad || route->sink_stream != format->stream)
			continue;

		source_fmt = v4l2_subdev_state_get_format(state, route->source_pad,
								 route->source_stream);
		if (!source_fmt) {
			return -EINVAL;
		}

		*source_fmt = format->format;
	}

	return 0;
}

static int max96724_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state)
{
	struct max96724_priv *priv = container_of(sd, struct max96724_priv, sd);
	struct v4l2_subdev_krouting routing = {};
	struct v4l2_subdev_route *routes;
	int i;
	int src_pad[2] = { MAX96724_SRC_PAD, MAX96724_SRC_PAD + 1 };
	int ret;


	routes = kcalloc(MAX96724_N_SINKS, sizeof(*routes), GFP_KERNEL);
	if (!routes) {
		return -ENOMEM;
	}

	for (i = 0; i < MAX96724_N_SINKS; i++) {
		struct v4l2_subdev_route *route = &routes[i];

		route->source_pad = (BIT(i) & priv->csi2_video_pipe_mask[0]) ? src_pad[0] :
									       src_pad[1];
		route->source_stream = i;
		route->sink_pad = i;
		route->sink_stream = 0;
		route->flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;
	}

	routing.num_routes = MAX96724_N_SINKS;
	routing.routes = routes;

	ret = v4l2_subdev_set_routing_with_fmt(sd, sd_state, &routing, &max96724_default_format);
	return ret;
}

static int max96724_set_routing(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				enum v4l2_subdev_format_whence which,
				struct v4l2_subdev_krouting *routing)
{
	int ret;


	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && media_entity_is_streaming(&sd->entity)) {
		return -EBUSY;
	}

	ret = v4l2_subdev_routing_validate(sd, routing, V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
	if (ret) {
		return ret;
	}

	ret = v4l2_subdev_set_routing_with_fmt(sd, state, routing, &max96724_default_format);
	return ret;
}

static struct v4l2_subdev *max96724_xlate_streams(struct max96724_priv *priv,
						  struct v4l2_subdev_state *state, u32 src_pad,
						  u64 src_streams, u32 sink_pad, u64 *sink_streams,
						  u32 *remote_pad)
{
	struct device *dev = &priv->client->dev;
	u64 streams;
	struct v4l2_subdev *remote_sd;
	struct media_pad *pad;


	streams = v4l2_subdev_state_xlate_streams(state, src_pad, sink_pad, &src_streams);
	if (!streams)
		dev_dbg(dev, "no streams found on sink pad\n");

	pad = media_pad_remote_pad_first(&priv->pads[sink_pad]);
	if (!pad) {
		dev_dbg(dev, "no remote pad found for sink pad\n");
		return ERR_PTR(-EPIPE);
	}

	remote_sd = media_entity_to_v4l2_subdev(pad->entity);
	if (!remote_sd) {
		dev_dbg(dev, "no entity connected to CSI2 input\n");
		return ERR_PTR(-EPIPE);
	}

	*sink_streams = streams;
	*remote_pad = pad->index;

	return remote_sd;
}

static int max96724_enable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				   u32 src_pad, u64 streams_mask)
{
	struct max96724_priv *priv = container_of(sd, struct max96724_priv, sd);
	struct device *dev = &priv->client->dev;
	int ret = 0;
	unsigned int i;


	mutex_lock(&priv->lock);

	if (!priv->enable_count) {
		ret = max96724_setup_all_pipes(priv, state);
		if (ret)
			goto unlock;

		ret = max96724_phy_enable(priv, true);
		if (ret)
			goto unlock;

		ret = max96724_fsync_set(priv);
		if (ret)
			goto unlock;
	}

	/*
	 * Walk the active routing table. For each route where
	 * source_pad == src_pad and the source_stream is in streams_mask,
	 * enable the corresponding stream on the sink pad's remote subdev.
	 * Previously the loop treated ffs(streams_mask)-1 as both the stream
	 * position AND the sink pad index — that assumption only holds when
	 * the camera is on GMSL Link A (sink pad 0); for any other link the
	 * lookup returned EPIPE "no remote pad found for sink pad".
	 */
	for (i = 0; i < state->routing.num_routes; i++) {
		struct v4l2_subdev_route *route = &state->routing.routes[i];
		struct media_pad *pad;
		struct v4l2_subdev *remote_sd;

		if (!(route->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			continue;
		if (route->source_pad != src_pad)
			continue;
		if (!(streams_mask & BIT(route->source_stream)))
			continue;

		if (route->sink_pad >= MAX96724_N_SINKS) {
			dev_err(dev, "route sink_pad %u out of range\n", route->sink_pad);
			ret = -EINVAL;
			goto unlock;
		}

		pad = media_pad_remote_pad_first(&priv->pads[route->sink_pad]);
		if (!pad) {
			dev_err(dev, "no remote pad on sink pad %u\n", route->sink_pad);
			ret = -EPIPE;
			goto unlock;
		}
		remote_sd = media_entity_to_v4l2_subdev(pad->entity);
		if (!remote_sd) {
			dev_err(dev, "no v4l2 subdev on sink pad %u remote\n",
				route->sink_pad);
			ret = -EPIPE;
			goto unlock;
		}

		ret = v4l2_subdev_enable_streams(remote_sd, pad->index,
						 BIT(route->sink_stream));
		if (ret) {
			dev_err(dev, "failed to enable stream on '%s':%u: %d\n",
				remote_sd->name, pad->index, ret);
			goto unlock;
		}
	}

	priv->enable_count++;

unlock:
	mutex_unlock(&priv->lock);

	return ret;
}

static int max96724_disable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				    u32 src_pad, u64 streams_mask)
{
	struct max96724_priv *priv = container_of(sd, struct max96724_priv, sd);
	struct device *dev = &priv->client->dev;
	int ret = 0;
	unsigned int i;


	mutex_lock(&priv->lock);

	priv->enable_count--;

	/* Mirror of enable_streams: walk the routing table rather than
	 * assuming ffs(mask)-1 == sink_pad. */
	for (i = 0; i < state->routing.num_routes; i++) {
		struct v4l2_subdev_route *route = &state->routing.routes[i];
		struct media_pad *pad;
		struct v4l2_subdev *remote_sd;

		if (!(route->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			continue;
		if (route->source_pad != src_pad)
			continue;
		if (!(streams_mask & BIT(route->source_stream)))
			continue;

		if (route->sink_pad >= MAX96724_N_SINKS) {
			dev_err(dev, "route sink_pad %u out of range\n", route->sink_pad);
			ret = -EINVAL;
			goto unlock;
		}

		pad = media_pad_remote_pad_first(&priv->pads[route->sink_pad]);
		if (!pad) {
			dev_err(dev, "no remote pad on sink pad %u\n", route->sink_pad);
			ret = -EPIPE;
			goto unlock;
		}
		remote_sd = media_entity_to_v4l2_subdev(pad->entity);
		if (!remote_sd) {
			dev_err(dev, "no v4l2 subdev on sink pad %u remote\n",
				route->sink_pad);
			ret = -EPIPE;
			goto unlock;
		}

		ret = v4l2_subdev_disable_streams(remote_sd, pad->index,
						  BIT(route->sink_stream));
		if (ret) {
			dev_err(dev, "failed to disable stream on '%s':%u: %d\n",
				remote_sd->name, pad->index, ret);
			goto unlock;
		}
	}

	if (!priv->enable_count) {
		max96724_phy_enable(priv, false);
		max96724_pipes_disable_all(priv);
	}

unlock:
	mutex_unlock(&priv->lock);

	return ret;
}

static int max96724_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				   struct v4l2_mbus_frame_desc *fd)
{
	struct max96724_priv *priv = container_of(sd, struct max96724_priv, sd);
	struct device *dev = &priv->client->dev;
	struct v4l2_subdev_state *state;
	struct v4l2_subdev_route *route;
	struct media_pad *remote_pad;
	int ret = 0;
	int i;


	if (pad < MAX96724_SRC_PAD) {
		return -EINVAL;
	}

	memset(fd, 0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	for_each_active_route(&state->routing, route) {
		struct v4l2_mbus_frame_desc_entry *source_entry = NULL;
		struct v4l2_mbus_frame_desc source_fd = {0};

		if (route->source_pad != pad)
			continue;

		remote_pad = media_pad_remote_pad_first(&priv->pads[route->sink_pad]);
		if (!remote_pad) {
			dev_dbg(dev, "no remote pad found for sink pad\n");
			ret = -EPIPE;
			goto unlock_state;
		}

		ret = v4l2_subdev_call(priv->sources[route->sink_pad].sd, pad, get_frame_desc,
				       remote_pad->index, &source_fd);
		if (ret) {
			dev_err(dev, "Failed to get source frame desc for pad %u\n",
				route->sink_pad);

			goto unlock_state;
		}

		for (i = 0; i < source_fd.num_entries; i++) {
			if (source_fd.entry[i].stream == route->sink_stream) {
				source_entry = &source_fd.entry[i];
				break;
			}
		}

		if (!source_entry) {
			dev_err(dev, "Failed to find stream from source frame desc\n");

			ret = -EPIPE;
			goto unlock_state;
		}

		fd->entry[fd->num_entries].stream = route->source_stream;
		fd->entry[fd->num_entries].flags = source_entry->flags;
		fd->entry[fd->num_entries].length = source_entry->length;
		fd->entry[fd->num_entries].pixelcode = source_entry->pixelcode;

		/*
		 * TODO: Currently, the sink pad gives the VC number. But we need to fix this
		 * if we're using the 2nd CSI port as well as we will end up with VC0 and VC1 on
		 * CSI1 and VC2 and VC3 on CSI2. Should be VC0 and VC1 on CSI2.
		 */
		fd->entry[fd->num_entries].bus.csi2.vc = route->sink_pad;
		fd->entry[fd->num_entries].bus.csi2.dt = source_entry->bus.csi2.dt;

		fd->num_entries++;
	}

unlock_state:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int max96724_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_mbus_code_enum *code)
{

	if (code->pad >= MAX96724_SRC_PAD) {
		struct v4l2_mbus_framefmt *fmt;

		if (code->index > 0 || code->pad >= MAX96724_N_PADS) {
			return -EINVAL;
		}

		fmt = v4l2_subdev_state_get_opposite_stream_format(sd_state, code->pad,
								   code->stream);
		if (!fmt) {
			return -EINVAL;
		}

		code->code = fmt->code;

		return 0;
	}

	if (code->index >= ARRAY_SIZE(max96724_formats)) {
		return -EINVAL;
	}

	code->code = max96724_formats[code->index].code;

	return 0;
}

static const struct v4l2_subdev_pad_ops max96724_v4l2_pad_ops = {
	.enum_mbus_code		= max96724_enum_mbus_code,
	.get_fmt		= v4l2_subdev_get_fmt,
	.set_fmt		= max96724_set_fmt,
	.set_routing		= max96724_set_routing,
	.get_frame_desc		= max96724_get_frame_desc,
	.get_frame_interval	= max96724_get_frame_interval,
	.set_frame_interval	= max96724_set_frame_interval,
	.enable_streams		= max96724_enable_streams,
	.disable_streams	= max96724_disable_streams,
};

static int max96724_notify_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *subdev,
				 struct v4l2_async_connection *asc)
{
	struct max96724_priv *priv = container_of(notifier->sd, struct max96724_priv, sd);
	struct max96724_asc *async_conn = container_of(asc, struct max96724_asc, base);
	struct max96724_source *source = async_conn->source;
	unsigned int index = to_index(priv, source);
	unsigned int src_pad;
	int ret;
	struct device *dev = &priv->client->dev;


	ret = media_entity_get_fwnode_pad(&subdev->entity, source->fwnode, MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(dev, "Failed to find pad for %s\n", subdev->name);
		return ret;
	}

	source->sd = subdev;
	src_pad = ret;

	ret = media_create_pad_link(&source->sd->entity, src_pad, &priv->sd.entity, index,
				    MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(dev, "Unable to link %s:%u -> %s:%u\n",
			source->sd->name, src_pad, priv->sd.name, index);
		return ret;
	}

	dev_dbg(dev, "Bound %s pad: %u on index %u\n", subdev->name, src_pad, index);

	return 0;
}

static void max96724_notify_unbind(struct v4l2_async_notifier *notifier,
				   struct v4l2_subdev *subdev,
				   struct v4l2_async_connection *asc)
{
	struct max96724_asc *async_conn = container_of(asc, struct max96724_asc, base);
	struct max96724_source *source = async_conn->source;


	source->sd = NULL;

}

static const struct v4l2_subdev_ops max96724_v4l2_ops = {
	.pad = &max96724_v4l2_pad_ops,
};

static const struct v4l2_subdev_internal_ops max96724_internal_ops = {
	.init_state = max96724_init_state,
};

static const struct v4l2_async_notifier_operations max96724_notify_ops = {
	.bound = max96724_notify_bound,
	.unbind = max96724_notify_unbind,
};

static int max96724_v4l2_notifier_register(struct max96724_priv *priv)
{
	int i, ret;
	struct device *dev = &priv->client->dev;
	struct max96724_source *source = NULL;
	unsigned int added = 0;


	if (!priv->nsources) {
		return 0;
	}

	v4l2_async_subdev_nf_init(&priv->notifier, &priv->sd);

	/*
	 * Only add async subdev entries for GMSL links that actually locked at
	 * boot (priv->gmsl_link_mask was rewritten by max96724_check_gmsl_links()
	 * to contain the locked set). A missing / unlocked camera would
	 * otherwise leave the async notifier waiting forever for a subdev that
	 * can never appear, which would block the deserializer's own subdev
	 * completion and, transitively, the entire ISI media device. With this
	 * skip, a single missing camera degrades gracefully — its entry is
	 * simply absent from libcamera while the working cameras still
	 * enumerate.
	 */
	for (i = 0; i < MAX96724_N_SINKS; i++) {
		source = &priv->sources[i];
		struct max96724_asc *asc;

		if (!source->fwnode)
			continue;

		if (!(priv->gmsl_link_mask & BIT(i))) {
			dev_warn(dev,
				 "GMSL link %c: no lock at boot, skipping async subdev registration\n",
				 'A' + i);
			continue;
		}

		asc = v4l2_async_nf_add_fwnode(&priv->notifier, source->fwnode,
					       struct max96724_asc);
		if (IS_ERR(asc)) {
			dev_err(dev, "Failed to add subdev for source %u: %ld", i, PTR_ERR(asc));
			v4l2_async_nf_cleanup(&priv->notifier);
			return PTR_ERR(asc);
		}

		asc->source = source;
		added++;
	}

	/*
	 * If no links locked, nothing to wait for. Clean up the notifier and
	 * skip registration so the deserializer's subdev still completes and
	 * downstream media-device registration can proceed.
	 */
	if (!added) {
		dev_warn(dev, "No GMSL links locked; skipping async notifier registration\n");
		v4l2_async_nf_cleanup(&priv->notifier);
		return 0;
	}

	priv->notifier.ops = &max96724_notify_ops;

	ret = v4l2_async_nf_register(&priv->notifier);
	if (ret) {
		dev_err(dev, "Failed to register subdev_notifier");
		v4l2_async_nf_cleanup(&priv->notifier);
		return ret;
	}

	return 0;
}

static void max96724_v4l2_notifier_unregister(struct max96724_priv *priv)
{

	if (!priv->nsources) {
		return;
	}

	/*
	 * If notifier_register() skipped registration because no links were
	 * locked, unregister is a no-op (ops pointer is NULL in that case).
	 */
	if (!priv->notifier.ops)
		return;

	v4l2_async_nf_unregister(&priv->notifier);
	v4l2_async_nf_cleanup(&priv->notifier);

}

static int max96724_v4l2_init(struct max96724_priv *priv)
{
	int ret, i;
	struct device *dev = &priv->client->dev;
	struct v4l2_subdev *sd = &priv->sd;
	static const s64 link_freq[] = {
		750000000UL,
	};


	v4l2_i2c_subdev_init(sd, priv->client, &max96724_v4l2_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;

	ret = v4l2_ctrl_handler_init(&priv->ctrl_handler, 1);
	if (ret < 0) {
		dev_err(dev, "Cannot initialize V4L2 ctrl handler.\n");
		return ret;
	}
	sd->ctrl_handler = &priv->ctrl_handler;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &max96724_v4l2_media_ops;
	sd->internal_ops = &max96724_internal_ops;

	priv->pixrate_ctrl = v4l2_ctrl_new_std(&priv->ctrl_handler, &max96724_v4l2_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, 1, INT_MAX, 1, 50000000);

	priv->link_freq_ctrl = v4l2_ctrl_new_int_menu(&priv->ctrl_handler, NULL, V4L2_CID_LINK_FREQ,
						      ARRAY_SIZE(link_freq) - 1, 0, link_freq);
	if (priv->link_freq_ctrl)
		priv->link_freq_ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	for (i = 0; i < MAX96724_N_PADS; i++)
		priv->pads[i].flags = i < MAX96724_SRC_PAD ?
				      MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&priv->sd.entity, MAX96724_N_PADS, priv->pads);
	if (ret)
		goto error;

	ret = v4l2_subdev_init_finalize(&priv->sd);
	if (ret < 0)
		goto error_cleanup_entity;

	ret = max96724_v4l2_notifier_register(priv);
	if (ret) {
		dev_err(dev, "Unable to register v4l2 async notifiers\n");
		goto error_cleanup_entity;
	}

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret < 0) {
		dev_err(dev, "Unable to register subdevice\n");
		goto error_cleanup_entity;
	}

	return 0;

error_cleanup_entity:
	media_entity_cleanup(&priv->sd.entity);

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return ret;
}

static void max96724_v4l2_deinit(struct max96724_priv *priv)
{

	v4l2_async_unregister_subdev(&priv->sd);
	max96724_v4l2_notifier_unregister(priv);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

}

static int max96724_probe(struct i2c_client *client)
{
	struct max96724_priv *priv;
	struct device *dev = &client->dev;
	int ret;
	int chip_id;
	bool chip_present;


	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		return -ENOMEM;
	}

	priv->client = client;

	mutex_init(&priv->lock);

	priv->rmap = devm_regmap_init_i2c(client, &max96724_regmap_cfg);
	if (IS_ERR(priv->rmap)) {
		ret = PTR_ERR(priv->rmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(client, priv);

	ret = devm_regulator_get_enable_optional(dev, "v12p0");
	if (ret < 0 && ret != -ENODEV) {
		return ret;
	}

	priv->reset_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (gpio_is_valid(priv->reset_gpio)) {
		ret = devm_gpio_request_one(dev, priv->reset_gpio, GPIOF_OUT_INIT_HIGH,
					    "max96724_mipi_reset");
		if (ret < 0) {
			dev_err(dev, "Failed to set reset pin\n");
			return ret;
		}
	}

	/* power-up completes in approx 2ms, according to specifications */
	usleep_range(2000, 2500);

	/*
	 * Probe for chip presence. If the deserializer is not physically
	 * installed or is not responding on i2c, we still register a minimal
	 * v4l2_subdev stub so downstream consumers (ISI async notifier)
	 * complete cleanly and any other deserializer in the system keeps
	 * working. Only skip the i2c-touching initialization in that case.
	 */
	ret = regmap_read(priv->rmap, MAX96724_DEV_REG13, &chip_id);
	chip_present = (ret == 0) && (chip_id == MAX96724_DEV_ID);
	if (!chip_present) {
		if (ret)
			dev_warn(dev,
				 "MAX96724 not responding on i2c (%d); registering offline stub\n",
				 ret);
		else
			dev_warn(dev,
				 "Unexpected chip id 0x%02x; registering offline stub\n",
				 chip_id);
	}

	ret = max96724_i2c_parse_dt(priv);
	if (ret) {
		return -ENODEV;
	}

	/* Parse optional GMSL link speed: 3 = 3Gbps, 6 = 6Gbps */
	{
		u32 speed_gbps = 0;

		if (!of_property_read_u32(dev->of_node, "maxim,gmsl-speed", &speed_gbps)) {
			if (speed_gbps == 3)
				priv->gmsl_speed = MAX96724_GMSL_3G;
			else if (speed_gbps == 6)
				priv->gmsl_speed = MAX96724_GMSL_6G;
			else
				dev_warn(dev, "Invalid maxim,gmsl-speed %u, using default\n",
					 speed_gbps);
		}
	}

	if (chip_present) {
		ret = max96724_chip_init(priv);
		if (ret) {
			return -ENODEV;
		}
	} else {
		/* Chip absent: no GMSL links can lock. */
		priv->gmsl_link_mask = 0;
	}

	ret = max96724_parse_dt(priv);
	if (ret) {
		return ret;
	}

	if (chip_present) {
		ret = max96724_dphy_config(priv);
		if (ret) {
			return ret;
		}
	}

	ret = max96724_v4l2_init(priv);
	if (ret) {
		return ret;
	}

	/*
	 * The i2c-mux requires a live chip to forward transactions over GMSL.
	 * Skip it entirely in offline stub mode so we don't advertise non-
	 * functional child i2c buses. Serializer / sensor drivers won't
	 * even attempt to probe without those child buses.
	 */
	if (chip_present) {
		ret = max96724_i2c_init(priv);
		if (ret)
			return ret;
	}

	return 0;
}

static void max96724_remove(struct i2c_client *client)
{
	struct max96724_priv *priv = i2c_get_clientdata(client);


	/*
	 * i2c-mux is only created in max96724_i2c_init() which is skipped in
	 * offline-stub mode. priv->mux is NULL in that case and must not be
	 * freed.
	 */
	if (priv->mux)
		i2c_mux_del_adapters(priv->mux);

	max96724_v4l2_deinit(priv);
	mutex_destroy(&priv->lock);

}

static const struct of_device_id max96724_dt_ids[] = {
	{ .compatible = "maxim,max96724" },
	{},
};
MODULE_DEVICE_TABLE(of, max96724_dt_ids);

static struct i2c_driver max96724_i2c_driver = {
	.driver	= {
		.name		= "max96724",
		.of_match_table	= of_match_ptr(max96724_dt_ids),
	},
	.probe		= max96724_probe,
	.remove		= max96724_remove,
};

module_i2c_driver(max96724_i2c_driver);

MODULE_DESCRIPTION("Maxim MAX96724 GMSL2/1 Deserializer Driver");
MODULE_AUTHOR("Laurentiu Palcu");
MODULE_LICENSE("GPL");

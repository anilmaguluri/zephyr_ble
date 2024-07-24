/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_mipi_csi2rx

#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <soc.h>

#include <fsl_mipi_csi2rx.h>

LOG_MODULE_REGISTER(video_mipi_csi2rx, CONFIG_VIDEO_LOG_LEVEL);

#define MAX_SUPPORTED_PIXEL_RATE MHZ(96)

#define ABS(a, b) (a > b ? a - b : b - a)

struct mipi_csi2rx_config {
	const MIPI_CSI2RX_Type *base;
	const struct device *sensor_dev;
};

struct mipi_csi2rx_data {
	csi2rx_config_t csi2rxConfig;
};

struct mipi_csi2rx_tHsSettleEscClk_config {
	uint64_t pixel_rate;
	uint8_t tHsSettle_EscClk;
};

/* Must be in pixel rate ascending order */
const struct mipi_csi2rx_tHsSettleEscClk_config tHsSettleEscClk_configs[] = {
	{MHZ(24), 0x24},
	{MHZ(48), 0x12},
	{MHZ(96), 0x09},
};

static int mipi_csi2rx_update_settings(const struct device *dev, enum video_endpoint_id ep)
{
	const struct mipi_csi2rx_config *config = dev->config;
	struct mipi_csi2rx_data *drv_data = dev->data;
	uint8_t bpp;
	uint64_t sensor_pixel_rate, sensor_lane_rate, sensor_byte_clk;
	uint32_t best_match;
	int ret, ind = 0;
	struct video_format fmt;

	ret = video_get_format(config->sensor_dev, ep, &fmt);
	if (ret) {
		LOG_ERR("Cannot get sensor_dev pixel format");
		return ret;
	}

	ret = video_get_ctrl(config->sensor_dev, VIDEO_CID_PIXEL_RATE, &sensor_pixel_rate);
	if (ret) {
		LOG_ERR("Can not get sensor_dev pixel rate");
		return ret;
	}

	bpp = video_pix_fmt_bpp(fmt.pixelformat) * 8;
	sensor_lane_rate = sensor_pixel_rate * bpp / drv_data->csi2rxConfig.laneNum;

	if (sensor_pixel_rate > MAX_SUPPORTED_PIXEL_RATE) {
		LOG_ERR("Sensor pixel rate is not supported");
		return -ENOTSUP;
	}

	sensor_byte_clk = sensor_pixel_rate * bpp / drv_data->csi2rxConfig.laneNum / 8;
	if (sensor_byte_clk > CLOCK_GetRootClockFreq(kCLOCK_Root_Csi2)) {
		mipi_csi2rx_clock_set_freq(kCLOCK_Root_Csi2, sensor_byte_clk);
	}

	if (sensor_pixel_rate > CLOCK_GetRootClockFreq(kCLOCK_Root_Csi2_Ui)) {
		mipi_csi2rx_clock_set_freq(kCLOCK_Root_Csi2_Ui, sensor_pixel_rate);
	}

	/* Find the supported sensor_pixel_rate closest to the desired one */
	best_match = tHsSettleEscClk_configs[ind].pixel_rate;
	for (uint8_t i = 0; i < ARRAY_SIZE(tHsSettleEscClk_configs); i++) {
		if (ABS(tHsSettleEscClk_configs[i].pixel_rate, sensor_pixel_rate) <
		    ABS(tHsSettleEscClk_configs[i].pixel_rate, best_match)) {
			best_match = tHsSettleEscClk_configs[i].pixel_rate;
			ind = i;
		}
	}

	drv_data->csi2rxConfig.tHsSettle_EscClk = tHsSettleEscClk_configs[ind].tHsSettle_EscClk;

	return ret;
}

static int mipi_csi2rx_set_fmt(const struct device *dev, enum video_endpoint_id ep,
			       struct video_format *fmt)
{
	const struct mipi_csi2rx_config *config = dev->config;

	if (video_set_format(config->sensor_dev, ep, fmt)) {
		return -EIO;
	}

	return mipi_csi2rx_update_settings(dev, ep);
}

static int mipi_csi2rx_get_fmt(const struct device *dev, enum video_endpoint_id ep,
			       struct video_format *fmt)
{
	const struct mipi_csi2rx_config *config = dev->config;

	if (fmt == NULL || (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL)) {
		return -EINVAL;
	}

	if (video_get_format(config->sensor_dev, ep, fmt)) {
		return -EIO;
	}

	return 0;
}

static int mipi_csi2rx_stream_start(const struct device *dev)
{
	const struct mipi_csi2rx_config *config = dev->config;
	struct mipi_csi2rx_data *drv_data = dev->data;

	CSI2RX_Init((MIPI_CSI2RX_Type *)config->base, &drv_data->csi2rxConfig);

	if (video_stream_start(config->sensor_dev)) {
		return -EIO;
	}

	return 0;
}

static int mipi_csi2rx_stream_stop(const struct device *dev)
{
	const struct mipi_csi2rx_config *config = dev->config;

	if (video_stream_stop(config->sensor_dev)) {
		return -EIO;
	}

	CSI2RX_Deinit((MIPI_CSI2RX_Type *)config->base);

	return 0;
}

static int mipi_csi2rx_get_caps(const struct device *dev, enum video_endpoint_id ep,
				struct video_caps *caps)
{
	const struct mipi_csi2rx_config *config = dev->config;

	if (ep != VIDEO_EP_OUT && ep != VIDEO_EP_ALL) {
		return -EINVAL;
	}

	/* Just forward to sensor dev for now */
	return video_get_caps(config->sensor_dev, ep, caps);
}

static inline int mipi_csi2rx_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	const struct mipi_csi2rx_config *config = dev->config;

	if (config->sensor_dev) {
		return video_set_ctrl(config->sensor_dev, cid, value);
	}

	return -ENOTSUP;
}

static const struct video_driver_api mipi_csi2rx_driver_api = {
	.get_caps = mipi_csi2rx_get_caps,
	.get_format = mipi_csi2rx_get_fmt,
	.set_format = mipi_csi2rx_set_fmt,
	.stream_start = mipi_csi2rx_stream_start,
	.stream_stop = mipi_csi2rx_stream_stop,
	.set_ctrl = mipi_csi2rx_set_ctrl,
};

static int mipi_csi2rx_init(const struct device *dev)
{
	const struct mipi_csi2rx_config *config = dev->config;

	/* Check if there is any sensor device */
	if (!device_is_ready(config->sensor_dev)) {
		return -ENODEV;
	}

	/*
	 * CSI2 escape clock should be in the range [60, 80] Mhz. We set it
	 * to 60 Mhz.
	 */
	mipi_csi2rx_clock_set_freq(kCLOCK_Root_Csi2_Esc, MHZ(60));

	return mipi_csi2rx_update_settings(dev, VIDEO_EP_ALL);
}

#define MIPI_CSI2RX_INIT(n)                                                                        \
	static struct mipi_csi2rx_data mipi_csi2rx_data_##n = {                                    \
		.csi2rxConfig.laneNum =                                                            \
			DT_PROP_LEN(DT_CHILD(DT_CHILD(DT_INST_CHILD(n, ports), port_1), endpoint), \
				    data_lanes),                                                   \
	};                                                                                         \
                                                                                                   \
	static const struct mipi_csi2rx_config mipi_csi2rx_config_##n = {                          \
		.base = (MIPI_CSI2RX_Type *)DT_INST_REG_ADDR(n),                                   \
		.sensor_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, sensor)),                           \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &mipi_csi2rx_init, NULL, &mipi_csi2rx_data_##n,                   \
			      &mipi_csi2rx_config_##n, POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY,    \
			      &mipi_csi2rx_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MIPI_CSI2RX_INIT)

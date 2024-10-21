/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_s32_sent_controller

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nxp_s32_sent, CONFIG_SENT_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/drivers/sent/sent.h>

#include "Srx_Ip.h"

struct sent_nxp_s32_config {
	uint8_t ctrl_inst;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pin_cfg;
	void (*irq_config_func)(void);
};

struct sent_nxp_s32_rx_callback {
	sent_rx_callback_t callback;
	struct sent_frame frame;
	void *user_data;
};

struct sent_nxp_s32_channel_data {
	boolean started;
	struct sent_nxp_s32_rx_callback rx_callback;
};

struct sent_nxp_s32_data {
	struct sent_nxp_s32_channel_data channel_data[SRX_CNL_COUNT];
};

static int sent_nxp_s32_start(const struct device *dev, uint8_t channel_id)
{
	const struct sent_nxp_s32_config *config = dev->config;
	struct sent_nxp_s32_data *data = dev->data;
	struct sent_nxp_s32_channel_data *channel_data = &data->channel_data[channel_id];
	int err;

	if (channel_data->started) {
		return -EALREADY;
	}

	err = Srx_Ip_StartChannelReceiving(config->ctrl_inst, channel_id);

	if (err) {
		LOG_ERR("Failed to start SENT %d channel %d", config->ctrl_inst, channel_id);
		return -EIO;
	}

	channel_data->started = true;

	return 0;
}

static int sent_nxp_s32_stop(const struct device *dev, uint8_t channel_id)
{
	const struct sent_nxp_s32_config *config = dev->config;
	struct sent_nxp_s32_data *data = dev->data;
	struct sent_nxp_s32_channel_data *channel_data = &data->channel_data[channel_id];
	int err;

	if (!channel_data->started) {
		return -EALREADY;
	}

	err = Srx_Ip_StopChannelReceiving(config->ctrl_inst, channel_id);

	if (err) {
		LOG_ERR("Failed to stop SENT %d channel %d", config->ctrl_inst, channel_id);
		return -EIO;
	}

	channel_data->started = false;

	return 0;
}

static void sent_nxp_s32_add_rx_callback(const struct device *dev, uint8_t channel_id,
					 sent_rx_callback_t callback, void *user_data)
{
	struct sent_nxp_s32_data *data = dev->data;
	struct sent_nxp_s32_channel_data *channel_data = &data->channel_data[channel_id];

	channel_data->rx_callback.callback = callback;
	channel_data->rx_callback.user_data = user_data;
}

static const struct sent_driver_api sent_nxp_s32_driver_api = {
	.start = sent_nxp_s32_start,
	.stop = sent_nxp_s32_stop,
	.add_rx_callback = sent_nxp_s32_add_rx_callback,
};

static void sent_nxp_s32_isr_slow_msg(const struct device *dev)
{
	const struct sent_nxp_s32_config *config = dev->config;

	Srx_Ip_ProcessMsgCombinedInterrupt(config->ctrl_inst, SRX_IP_SERIAL_MSG_ONLY);
}

static void sent_nxp_s32_isr_fast_msg(const struct device *dev)
{
	const struct sent_nxp_s32_config *config = dev->config;

	Srx_Ip_ProcessMsgCombinedInterrupt(config->ctrl_inst, SRX_IP_FAST_MSG_ONLY);
}

static void sent_nxp_s32_isr_error(const struct device *dev)
{
	const struct sent_nxp_s32_config *config = dev->config;

	Srx_Ip_ProcessErrorCombinedInterrupt(config->ctrl_inst);
}

#define SENT_NXP_S32_HW_INSTANCE_CHECK(i, n) ((DT_INST_REG_ADDR(n) == IP_SRX_##i##_BASE) ? i : 0)

#define SENT_NXP_S32_HW_INSTANCE(n) \
	LISTIFY(SRX_INSTANCE_COUNT, SENT_NXP_S32_HW_INSTANCE_CHECK, (|), n)

#define _SENT_NXP_S32_IRQ_CONFIG(node_id, prop, idx)                                               \
	do {                                                                                       \
		IRQ_CONNECT(                                                                       \
			DT_IRQ_BY_IDX(node_id, idx, irq), DT_IRQ_BY_IDX(node_id, idx, priority),   \
			UTIL_CAT(sent_nxp_s32_isr_, DT_STRING_TOKEN_BY_IDX(node_id, prop, idx)),   \
			DEVICE_DT_GET(node_id), DT_IRQ_BY_IDX(node_id, idx, flags));               \
		irq_enable(DT_IRQ_BY_IDX(node_id, idx, irq));                                      \
	} while (false);

#define SENT_NXP_S32_IRQ_CONFIG(n)                                                                 \
	static void sent_irq_config_##n(void)                                                      \
	{                                                                                          \
		DT_INST_FOREACH_PROP_ELEM(n, interrupt_names, _SENT_NXP_S32_IRQ_CONFIG);           \
	}

#define DEV_SENT_NXP_S32_INIT(n)                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	SENT_NXP_S32_IRQ_CONFIG(n)                                                                 \
	static struct sent_nxp_s32_config sent_nxp_s32_config_##n = {                              \
		.ctrl_inst = SENT_NXP_S32_HW_INSTANCE(n),                                          \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),              \
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                      \
		.irq_config_func = sent_irq_config_##n,                                            \
	};                                                                                         \
	static struct sent_nxp_s32_data sent_nxp_s32_data_##n;                                     \
	static int sent_nxp_s32_init_##n(const struct device *dev)                                 \
	{                                                                                          \
		const struct sent_nxp_s32_config *config = dev->config;                            \
		int err = 0;                                                                       \
		uint32_t rate;                                                                     \
                                                                                                   \
		if (!device_is_ready(config->clock_dev)) {                                         \
			LOG_ERR("Clock control device not ready");                                 \
			return -ENODEV;                                                            \
		}                                                                                  \
                                                                                                   \
		err = clock_control_on(config->clock_dev, config->clock_subsys);                   \
		if (err) {                                                                         \
			LOG_ERR("Failed to enable clock");                                         \
			return err;                                                                \
		}                                                                                  \
                                                                                                   \
		err = clock_control_get_rate(config->clock_dev, config->clock_subsys, &rate);      \
		if (err) {                                                                         \
			LOG_ERR("Failed to get clock");                                            \
			return err;                                                                \
		}                                                                                  \
                                                                                                   \
		memcpy((uint32_t *)&sent_nxp_s32_controller_config[n].HighFreqRxClock, &rate,      \
		       sizeof(uint32_t));                                                          \
                                                                                                   \
		err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);                 \
		if (err < 0) {                                                                     \
			LOG_ERR("SENT pinctrl setup failed (%d)", err);                            \
			return err;                                                                \
		}                                                                                  \
                                                                                                   \
		config->irq_config_func();                                                         \
                                                                                                   \
		return 0;                                                                          \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(n, sent_nxp_s32_init_##n, NULL, &sent_nxp_s32_data_##n,              \
			      &sent_nxp_s32_config_##n, POST_KERNEL, CONFIG_SENT_INIT_PRIORITY,    \
			      &sent_nxp_s32_driver_api);

/*
 * The following definitions is defined for the initial configuration that used for
 * Srx_Ip_InitController()
 */

#define SENT_NXP_S32_BUS_TIMEOUT_CYCLES(node_id)                                                   \
	(DT_PROP(node_id, bus_timeout_cycles) == 256                                               \
		 ? SRX_IP_RECEIVER_CLOCK_TICK_COUNTS_256                                           \
		 : (DT_PROP(node_id, bus_timeout_cycles) == 512                                    \
			    ? SRX_IP_RECEIVER_CLOCK_TICK_COUNTS_512                                \
			    : (DT_PROP(node_id, bus_timeout_cycles) == 1024                        \
				       ? SRX_IP_RECEIVER_CLOCK_TICK_COUNTS_1024                    \
				       : (DT_PROP(node_id, bus_timeout_cycles) == 2048             \
						  ? SRX_IP_RECEIVER_CLOCK_TICK_COUNTS_2048         \
						  : SRX_IP_BUS_TIMEOUT_DISABLED))))

#define SENT_NXP_S32_CHANNEL_NODE(n, i) DT_INST_CHILD(n, DT_CAT(ch_, i))

#define SENT_NXP_S32_CHANNEL_ID_CNT(i, node_id, n)                                                 \
	(DT_NODE_HAS_STATUS(SENT_NXP_S32_CHANNEL_NODE(n, i), okay) &&                              \
			 (DT_REG_ADDR(SENT_NXP_S32_CHANNEL_NODE(n, i)) < DT_REG_ADDR(node_id))     \
		 ? 1                                                                               \
		 : 0)

#define SENT_NXP_S32_CHANNEL_ID(node_id, n)                                                        \
	LISTIFY(SRX_CNL_COUNT, SENT_NXP_S32_CHANNEL_ID_CNT, (+), node_id, n)

#define SENT_NXP_S32_CHANNEL_CONFIG(node_id, n)                                                    \
	Srx_Ip_ChannelUserConfigType _CONCAT(sent_nxp_s32_channel_config, node_id) = {             \
		.ControllerId = n,                                                                 \
		.ControllerHwOffset = SENT_NXP_S32_HW_INSTANCE(n),                                 \
		.ChannelId = SENT_NXP_S32_CHANNEL_ID(node_id, n),                                  \
		.ChannelHwOffset = DT_REG_ADDR(node_id),                                           \
		.ChannelDataLength = DT_PROP(node_id, num_data_nibbles),                           \
		.ChannelTickLengthUs = DT_PROP(node_id, tick_time_prescaler_us),                   \
		.ChannelConfigReg =                                                                \
			{                                                                          \
				.BusTimeout = SENT_NXP_S32_BUS_TIMEOUT_CYCLES(node_id),            \
				.FastCrcType = DT_PROP(node_id, slow_crc_no_data_nibble_xor),      \
				.SlowCrcType = DT_PROP(node_id, fast_crc_no_data_nibble_xor),      \
				.SuccessiveCalibCheck =                                            \
					!DT_PROP(node_id, calib_method_low_latency),               \
			},                                                                         \
	};

#define SENT_NXP_S32_CHANNEL_CONFIG_PTR(node_id) &_CONCAT(sent_nxp_s32_channel_config, node_id),

/* Define array channel configuration */
#define SENT_NXP_S32_ARRAY_CHANNEL_CONFIG(n)                                                       \
	DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(n, SENT_NXP_S32_CHANNEL_CONFIG, n)                 \
	static Srx_Ip_ChannelUserConfigType const                                                  \
		*const sent_nxp_s32_channel_array_config_##n[DT_INST_CHILD_NUM_STATUS_OKAY(n)] = { \
			DT_INST_FOREACH_CHILD_STATUS_OKAY(n, SENT_NXP_S32_CHANNEL_CONFIG_PTR)};

#define SENT_NXP_S32_CALLBACK(n)                                                                   \
	void sent_nxp_s32_cb_fast_msg_##n(uint8 ctrl_id, uint8 channel_id,                         \
					  Srx_Ip_FastMsgType *fmsg)                                \
	{                                                                                          \
		const struct device *dev = DEVICE_DT_INST_GET(n);                                  \
		struct sent_nxp_s32_data *data = dev->data;                                        \
		struct sent_nxp_s32_channel_data *channel_data = &data->channel_data[channel_id];  \
		struct sent_nxp_s32_rx_callback *rx_callback = &channel_data->rx_callback;         \
                                                                                                   \
		rx_callback->frame.fmsg.data = 0;                                                  \
		for (int i = 0; i < fmsg->Length; i++) {                                           \
			rx_callback->frame.fmsg.data =                                             \
				(rx_callback->frame.fmsg.data << 4) | (fmsg->DataNibble[i] & 0xf); \
		}                                                                                  \
		rx_callback->frame.fmsg.timestamp = fmsg->TimestampFast;                           \
		rx_callback->frame.fmsg.crc = fmsg->FastCrc;                                       \
                                                                                                   \
		if (rx_callback->callback) {                                                       \
			rx_callback->callback(dev, channel_id, &rx_callback->frame,                \
					      SENT_STATE_FMSG_RECEIVED, rx_callback->user_data);   \
		}                                                                                  \
	}                                                                                          \
	void sent_nxp_s32_cb_slow_msg_##n(uint8 ctrl_id, uint8 channel_id,                         \
					  Srx_Ip_SerialMsgType * smsg)                             \
	{                                                                                          \
		const struct device *dev = DEVICE_DT_INST_GET(n);                                  \
		struct sent_nxp_s32_data *data = dev->data;                                        \
		struct sent_nxp_s32_channel_data *channel_data = &data->channel_data[channel_id];  \
		struct sent_nxp_s32_rx_callback *rx_callback = &channel_data->rx_callback;         \
                                                                                                   \
		rx_callback->frame.smsg.id = smsg->MessageId;                                      \
		rx_callback->frame.smsg.data = smsg->MessageData;                                  \
		rx_callback->frame.smsg.timestamp = smsg->TimestampSerial;                         \
		rx_callback->frame.smsg.crc = smsg->SerialCrc;                                     \
                                                                                                   \
		if (rx_callback->callback) {                                                       \
			rx_callback->callback(dev, channel_id, &rx_callback->frame,                \
					      SENT_STATE_SMSG_RECEIVED, rx_callback->user_data);   \
		}                                                                                  \
	}                                                                                          \
	void sent_nxp_s32_error_cb_fast_msg_##n(uint8 ctrl_id, uint8 channel_id,                   \
						Srx_Ip_ChannelStatusType event)                    \
	{                                                                                          \
		const struct device *dev = DEVICE_DT_INST_GET(n);                                  \
		struct sent_nxp_s32_data *data = dev->data;                                        \
		struct sent_nxp_s32_channel_data *channel_data = &data->channel_data[channel_id];  \
		struct sent_nxp_s32_rx_callback *rx_callback = &channel_data->rx_callback;         \
                                                                                                   \
		if (rx_callback->callback) {                                                       \
			rx_callback->callback(dev, channel_id, NULL, SENT_STATE_FMSG_CRC_ERR,      \
					      rx_callback->user_data);                             \
		}                                                                                  \
	}                                                                                          \
	void sent_nxp_s32_error_cb_slow_msg_##n(uint8 ctrl_id, uint8 channel_id,                   \
						Srx_Ip_ChannelStatusType event)                    \
	{                                                                                          \
		const struct device *dev = DEVICE_DT_INST_GET(n);                                  \
		struct sent_nxp_s32_data *data = dev->data;                                        \
		struct sent_nxp_s32_channel_data *channel_data = &data->channel_data[channel_id];  \
		struct sent_nxp_s32_rx_callback *rx_callback = &channel_data->rx_callback;         \
                                                                                                   \
		if (rx_callback->callback) {                                                       \
			rx_callback->callback(dev, channel_id, NULL, SENT_STATE_SMSG_CRC_ERR,      \
					      rx_callback->user_data);                             \
		}                                                                                  \
	}

/* Define array instance configuration */
#define SENT_NXP_S32_INST_CONFIG(n)                                                                \
	{                                                                                          \
		.ControllerId = n,                                                                 \
		.ControllerHwOffset = SENT_NXP_S32_HW_INSTANCE(n),                                 \
		.ControllerMode = SRX_IP_INTERRUPT,                                                \
		.NumberChnlConfigured = DT_INST_CHILD_NUM_STATUS_OKAY(n),                          \
		.ChnlConfig = &sent_nxp_s32_channel_array_config_##n[0],                           \
		.FastErrorNotification = &sent_nxp_s32_error_cb_fast_msg_##n,                      \
		.SerialErrorNotification = &sent_nxp_s32_error_cb_slow_msg_##n,                    \
		.FastFrameNotification = &sent_nxp_s32_cb_fast_msg_##n,                            \
		.SerialFrameNotification = &sent_nxp_s32_cb_slow_msg_##n,                          \
	},

DT_INST_FOREACH_STATUS_OKAY(SENT_NXP_S32_ARRAY_CHANNEL_CONFIG)
DT_INST_FOREACH_STATUS_OKAY(SENT_NXP_S32_CALLBACK)

/* The structure configuration for all SENT controllers */
static Srx_Ip_ControllerConfigType sent_nxp_s32_controller_config[DT_NUM_INST_STATUS_OKAY(
	DT_DRV_COMPAT)] = {DT_INST_FOREACH_STATUS_OKAY(SENT_NXP_S32_INST_CONFIG)};

/* Initialize all SENT controllers */
static int sent_nxp_s32_ctrl_init(void)
{
	Srx_Ip_InitController(&sent_nxp_s32_controller_config[0]);

	return 0;
}

DT_INST_FOREACH_STATUS_OKAY(DEV_SENT_NXP_S32_INIT)

SYS_INIT(sent_nxp_s32_ctrl_init, POST_KERNEL, CONFIG_SENT_INIT_PRIORITY);

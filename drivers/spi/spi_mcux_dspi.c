/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <drivers/spi.h>
#include <drivers/clock_control.h>
#include <fsl_dspi.h>

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_mcux_dspi);

#include "spi_context.h"

struct spi_mcux_config {
	SPI_Type *base;
	char *clock_name;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(struct device *dev);
};

struct spi_mcux_data {
	dspi_master_handle_t handle;
	struct spi_context ctx;
	size_t transfer_len;
};

static int spi_mcux_transfer_next_packet(struct device *dev)
{
	const struct spi_mcux_config *config = dev->config->config_info;
	struct spi_mcux_data *data = dev->driver_data;
	SPI_Type *base = config->base;
	struct spi_context *ctx = &data->ctx;
	dspi_transfer_t transfer;
	status_t status;

	if ((ctx->tx_len == 0) && (ctx->rx_len == 0)) {
		/* nothing left to rx or tx, we're done! */
		spi_context_cs_control(&data->ctx, false);
		spi_context_complete(&data->ctx, 0);
		return 0;
	}

	transfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcsContinuous |
			       (ctx->config->slave << DSPI_MASTER_PCS_SHIFT);

	if (ctx->tx_len == 0) {
		/* rx only, nothing to tx */
		transfer.txData = NULL;
		transfer.rxData = ctx->rx_buf;
		transfer.dataSize = ctx->rx_len;
	} else if (ctx->rx_len == 0) {
		/* tx only, nothing to rx */
		transfer.txData = (u8_t *) ctx->tx_buf;
		transfer.rxData = NULL;
		transfer.dataSize = ctx->tx_len;
	} else if (ctx->tx_len == ctx->rx_len) {
		/* rx and tx are the same length */
		transfer.txData = (u8_t *) ctx->tx_buf;
		transfer.rxData = ctx->rx_buf;
		transfer.dataSize = ctx->tx_len;
	} else if (ctx->tx_len > ctx->rx_len) {
		/* Break up the tx into multiple transfers so we don't have to
		 * rx into a longer intermediate buffer. Leave chip select
		 * active between transfers.
		 */
		transfer.txData = (u8_t *) ctx->tx_buf;
		transfer.rxData = ctx->rx_buf;
		transfer.dataSize = ctx->rx_len;
		transfer.configFlags |= kDSPI_MasterActiveAfterTransfer;
	} else {
		/* Break up the rx into multiple transfers so we don't have to
		 * tx from a longer intermediate buffer. Leave chip select
		 * active between transfers.
		 */
		transfer.txData = (u8_t *) ctx->tx_buf;
		transfer.rxData = ctx->rx_buf;
		transfer.dataSize = ctx->tx_len;
		transfer.configFlags |= kDSPI_MasterActiveAfterTransfer;
	}

	if (!(ctx->tx_count <= 1 && ctx->rx_count <= 1)) {
		transfer.configFlags |= kDSPI_MasterActiveAfterTransfer;
	}

	data->transfer_len = transfer.dataSize;

	status = DSPI_MasterTransferNonBlocking(base, &data->handle, &transfer);
	if (status != kStatus_Success) {
		LOG_ERR("Transfer could not start");
	}

	return status == kStatus_Success ? 0 :
	       status == kDSPI_Busy ? -EBUSY : -EINVAL;
}

static void spi_mcux_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct spi_mcux_config *config = dev->config->config_info;
	struct spi_mcux_data *data = dev->driver_data;
	SPI_Type *base = config->base;

	DSPI_MasterTransferHandleIRQ(base, &data->handle);
}

static void spi_mcux_master_transfer_callback(SPI_Type *base,
		dspi_master_handle_t *handle, status_t status, void *userData)
{
	struct device *dev = userData;
	struct spi_mcux_data *data = dev->driver_data;

	spi_context_update_tx(&data->ctx, 1, data->transfer_len);
	spi_context_update_rx(&data->ctx, 1, data->transfer_len);

	spi_mcux_transfer_next_packet(dev);
}

static int spi_mcux_configure(struct device *dev,
			      const struct spi_config *spi_cfg)
{
	const struct spi_mcux_config *config = dev->config->config_info;
	struct spi_mcux_data *data = dev->driver_data;
	SPI_Type *base = config->base;
	dspi_master_config_t master_config;
	struct device *clock_dev;
	u32_t clock_freq;
	u32_t word_size;

	if (spi_context_configured(&data->ctx, spi_cfg)) {
		/* This configuration is already in use */
		return 0;
	}

	DSPI_MasterGetDefaultConfig(&master_config);

	if (spi_cfg->slave > FSL_FEATURE_DSPI_CHIP_SELECT_COUNT) {
		LOG_ERR("Slave %d is greater than %d",
			    spi_cfg->slave, FSL_FEATURE_DSPI_CHIP_SELECT_COUNT);
		return -EINVAL;
	}

	word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
	if (word_size > FSL_FEATURE_DSPI_MAX_DATA_WIDTH) {
		LOG_ERR("Word size %d is greater than %d",
			    word_size, FSL_FEATURE_DSPI_MAX_DATA_WIDTH);
		return -EINVAL;
	}

	master_config.ctarConfig.bitsPerFrame = word_size;

	master_config.ctarConfig.cpol =
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL)
		? kDSPI_ClockPolarityActiveLow
		: kDSPI_ClockPolarityActiveHigh;

	master_config.ctarConfig.cpha =
		(SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA)
		? kDSPI_ClockPhaseSecondEdge
		: kDSPI_ClockPhaseFirstEdge;

	master_config.ctarConfig.direction =
		(spi_cfg->operation & SPI_TRANSFER_LSB)
		? kDSPI_LsbFirst
		: kDSPI_MsbFirst;

	master_config.ctarConfig.baudRate = spi_cfg->frequency;

	clock_dev = device_get_binding(config->clock_name);
	if (clock_dev == NULL) {
		return -EINVAL;
	}

	if (clock_control_get_rate(clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}

	DSPI_MasterInit(base, &master_config, clock_freq);

	DSPI_MasterTransferCreateHandle(base, &data->handle,
					spi_mcux_master_transfer_callback, dev);

	data->ctx.config = spi_cfg;
	spi_context_cs_configure(&data->ctx);

	return 0;
}

static int transceive(struct device *dev,
		      const struct spi_config *spi_cfg,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      struct k_poll_signal *signal)
{
	struct spi_mcux_data *data = dev->driver_data;
	int ret;

	spi_context_lock(&data->ctx, asynchronous, signal);

	ret = spi_mcux_configure(dev, spi_cfg);
	if (ret) {
		goto out;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	spi_context_cs_control(&data->ctx, true);

	ret = spi_mcux_transfer_next_packet(dev);
	if (ret) {
		goto out;
	}

	ret = spi_context_wait_for_completion(&data->ctx);
out:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_mcux_transceive(struct device *dev,
			       const struct spi_config *spi_cfg,
			       const struct spi_buf_set *tx_bufs,
			       const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_mcux_transceive_async(struct device *dev,
				     const struct spi_config *spi_cfg,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs,
				     struct k_poll_signal *async)
{
	return transceive(dev, spi_cfg, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_mcux_release(struct device *dev,
		      const struct spi_config *spi_cfg)
{
	struct spi_mcux_data *data = dev->driver_data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int spi_mcux_init(struct device *dev)
{
	const struct spi_mcux_config *config = dev->config->config_info;
	struct spi_mcux_data *data = dev->driver_data;

	config->irq_config_func(dev);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct spi_driver_api spi_mcux_driver_api = {
	.transceive = spi_mcux_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_mcux_transceive_async,
#endif
	.release = spi_mcux_release,
};

#define SPI_MCUX_DSPI_DEVICE(id)					\
	static void spi_mcux_config_func_##id(struct device *dev);	\
	static const struct spi_mcux_config spi_mcux_config_##id = {	\
		.base =							\
		(SPI_Type *)DT_NXP_KINETIS_DSPI_SPI_##id##_BASE_ADDRESS,\
		.clock_name = DT_NXP_KINETIS_DSPI_SPI_##id##_CLOCKS_CONTROLLER,\
		.clock_subsys = 					\
		(clock_control_subsys_t)DT_NXP_KINETIS_DSPI_SPI_##id##_CLOCKS_NAME,\
		.irq_config_func = spi_mcux_config_func_##id,		\
	};								\
	static struct spi_mcux_data spi_mcux_data_##id = {		\
		SPI_CONTEXT_INIT_LOCK(spi_mcux_data_##id, ctx),		\
		SPI_CONTEXT_INIT_SYNC(spi_mcux_data_##id, ctx),		\
	};								\
	DEVICE_AND_API_INIT(spi_mcux_##id,				\
			    DT_NXP_KINETIS_DSPI_SPI_##id##_LABEL,	\
			    &spi_mcux_init,				\
			    &spi_mcux_data_##id,			\
			    &spi_mcux_config_##id,			\
			    POST_KERNEL,				\
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			    &spi_mcux_driver_api);			\
	static void spi_mcux_config_func_##id(struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_NXP_KINETIS_DSPI_SPI_##id##_IRQ_0,	\
			    DT_NXP_KINETIS_DSPI_SPI_##id##_IRQ_0_PRIORITY,\
			    spi_mcux_isr, DEVICE_GET(spi_mcux_##id),	\
			    0);						\
		irq_enable(DT_NXP_KINETIS_DSPI_SPI_##id##_IRQ_0);	\
	}

#ifdef CONFIG_SPI_0
SPI_MCUX_DSPI_DEVICE(0)
#endif /* CONFIG_SPI_0 */

#ifdef CONFIG_SPI_1
SPI_MCUX_DSPI_DEVICE(1)
#endif /* CONFIG_SPI_1 */

#ifdef CONFIG_SPI_2
SPI_MCUX_DSPI_DEVICE(2)
#endif /* CONFIG_SPI_2 */

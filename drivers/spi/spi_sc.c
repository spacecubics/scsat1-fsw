/*
 * Copyright (c) 2023 Space Cubics, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sc_qspi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_sc, CONFIG_SPI_LOG_LEVEL);

#include <zephyr/drivers/spi.h>
#include "spi_context.h"

/* Registers offset */
#define SC_QSPI_ACR_OFFSET    (0x0000) /* QSPI Access Control Register */
#define SC_QSPI_TDR_OFFSET    (0x0004) /* QSPI TX Data Register */
#define SC_QSPI_RDR_OFFSET    (0x0008) /* QSPI RX Data Register */
#define SC_QSPI_ASR_OFFSET    (0x000C) /* QSPI Access Status Register */
#define SC_QSPI_FIFOSR_OFFSET (0x0010) /* QSPI FIFO Status Register */
#define SC_QSPI_FIFORR_OFFSET (0x0014) /* QSPI FIFO Reset Register */
#define SC_QSPI_ISR_OFFSET    (0x0020) /* QSPI Interrupt Status Register */
#define SC_QSPI_IER_OFFSET    (0x0024) /* QSPI Interrupt Enable Register */
#define SC_QSPI_CCR_OFFSET    (0x0030) /* QSPI Clock Control Register */
#define SC_QSPI_DCMSR_OFFSET  (0x0034) /* QSPI Data Capture Mode Setting Register */
#define SC_QSPI_FTLSR_OFFSET  (0x0038) /* QSPI FIFO Threshold Level Setting Register */
#define SC_QSPI_VER_OFFSET    (0xF000) /* QSPI Controller IP Version Register */

/* QSPI Access Control Register */
#define SC_QSPI_ACR_SS_OFF      (0)
#define SC_QSPI_ACR_SS1         BIT(0)
#define SC_QSPI_ACR_SS2         BIT(1)
#define SC_QSPI_ACR_MODE_SINGLE (0)
#define SC_QSPI_ACR_MODE_DUAL   BIT(16)
#define SC_QSPI_ACR_MODE_QUAD   BIT(17)

/* QSPI Interrupt Status Register */
#define SC_QSPI_TXFIFOUTH BIT(26)
#define SC_QSPI_TXFIFOOVF BIT(25)
#define SC_QSPI_TXFIFOUDF BIT(24)
#define SC_QSPI_RXFIFOOTH BIT(18)
#define SC_QSPI_RXFIFOOVF BIT(17)
#define SC_QSPI_RXFIFOUDF BIT(16)
#define SC_QSPI_SPICTRLDN BIT(0)

/* QSPI Interrupt Enable Register */
#define SC_QSPI_TXFIFOUTHEMB BIT(26)
#define SC_QSPI_TXFIFOOVFEMB BIT(25)
#define SC_QSPI_TXFIFOUDFEMB BIT(24)
#define SC_QSPI_RXFIFOOTHEMB BIT(18)
#define SC_QSPI_RXFIFOOVFEMB BIT(17)
#define SC_QSPI_RXFIFOUDFEMB BIT(16)
#define SC_QSPI_SPIBUSYDNEMB BIT(0)
#define SC_QSPI_IER_ALL                                                                            \
	(SC_QSPI_TXFIFOUTHEMB | SC_QSPI_TXFIFOOVFEMB | SC_QSPI_TXFIFOUDFEMB |                      \
	 SC_QSPI_RXFIFOOTHEMB | SC_QSPI_RXFIFOOVFEMB | SC_QSPI_RXFIFOUDFEMB |                      \
	 SC_QSPI_SPIBUSYDNEMB)

/* QSPI Clock Control Register */
#define SC_QSPI_CCR_SCKPOL_SHIFT (20U)
#define SC_QSPI_CCR_SCKPHA_SHIFT (16U)
#define SC_QSPI_CCR_SCKDIV_SHIFT (0U)

/* QSPI Data Capture Mode Setting Register */
#define SC_QSPI_DCMSR_DTCAPT BIT(0)

/* TX/RX Buffer size */
#define SC_QSPI_TX_BUFFER_SIZE (16U)
#define SC_QSPI_RX_BUFFER_SIZE (16U)

typedef void (*irq_init_func_t)(const struct device *dev);

struct spi_sc_data {
	struct spi_context ctx;
	struct k_sem idle_sem;
};

struct spi_sc_cfg {
	mm_reg_t base;
	irq_init_func_t irq_init;
	bool data_capture_mode;
	uint8_t slave_num;
	uint8_t cpol;
	uint8_t cpha;
	uint16_t spiclk_div;
};

static int spi_sc_wait_for_idle(const struct device *dev)
{
	struct spi_sc_data *data = dev->data;
	int ret;

	/* Waiting for the 'SPICTRLDN' interrupt from the FPGA. */
	ret = k_sem_take(&data->idle_sem, K_MSEC(CONFIG_SPI_SC_IDLE_TIMEOUT_MS));
	if (ret < 0) {
		LOG_ERR("Waiting for the 'SPICTRLDN' interrupt, but it timed out. (%d)", ret);
	}

	return ret;
}

static void spi_sc_isr(const struct device *dev)
{
	uint32_t isr;
	const struct spi_sc_cfg *cfg = dev->config;
	struct spi_sc_data *data = dev->data;

	isr = sys_read32(cfg->base + SC_QSPI_ISR_OFFSET);
	LOG_DBG("IRQ Status 0x%08x", isr);

	sys_write32(isr, cfg->base + SC_QSPI_ISR_OFFSET);

	if (isr & SC_QSPI_SPICTRLDN) {
		k_sem_give(&data->idle_sem);
	}
}

static int spi_sc_send_tx_data(struct spi_context *ctx, const struct device *dev, uint8_t dfs,
			       uint32_t *tx_size)
{
	const struct spi_sc_cfg *cfg = dev->config;
	int ret;

	*tx_size = ctx->current_tx->len;

	if (ctx->current_tx->len > SC_QSPI_TX_BUFFER_SIZE) {
		*tx_size = SC_QSPI_TX_BUFFER_SIZE;
	}

	for (int i = 0; i < *tx_size; i++) {
		sys_write32(ctx->tx_buf[i], cfg->base + SC_QSPI_TDR_OFFSET);
		LOG_DBG("0x%02x sent", ctx->tx_buf[i]);

		ret = spi_sc_wait_for_idle(dev);
		if (ret < 0) {
			goto end;
		}
	}

	spi_context_update_tx(ctx, dfs, *tx_size);
end:
	return ret;
}

static int spi_sc_request_rx_data(const struct device *dev, size_t size)
{
	const struct spi_sc_cfg *cfg = dev->config;

	for (int i = 0; i < size; i++) {
		sys_write32(0, cfg->base + SC_QSPI_RDR_OFFSET);
	}

	return spi_sc_wait_for_idle(dev);
}

static int spi_sc_read_rx_data(struct spi_context *ctx, const struct device *dev, size_t size,
			       uint8_t dfs, bool discard)
{
	const struct spi_sc_cfg *cfg = dev->config;
	int ret = 0;
	uint8_t byte;

	if (size > SC_QSPI_RX_BUFFER_SIZE) {
		size = SC_QSPI_RX_BUFFER_SIZE;
	}

	if (!cfg->data_capture_mode) {
		ret = spi_sc_request_rx_data(dev, size);
		if (ret < 0) {
			LOG_ERR("Failed to request the RX data. (%d)", ret);
			goto end;
		}
	}

	for (int i = 0; i < size; i++) {
		byte = sys_read32(cfg->base + SC_QSPI_RDR_OFFSET);
		LOG_DBG("0x%02x recv", byte);
		if (!discard) {
			ctx->rx_buf[i] = byte;
		}
	}

	spi_context_update_rx(ctx, dfs, size);
end:
	return ret;
}

static int spi_sc_cs_control(struct spi_context *ctx, const struct device *dev, bool on)
{
	const struct spi_sc_cfg *cfg = dev->config;
	uint32_t line = ctx->config->operation & SPI_LINES_MASK;
	uint32_t acr = BIT(ctx->config->slave);

	if (on) {
		switch (line) {
		case SPI_LINES_DUAL:
			acr |= SC_QSPI_ACR_MODE_DUAL;
			break;
		case SPI_LINES_QUAD:
			acr |= SC_QSPI_ACR_MODE_QUAD;
			break;
		default:
			acr |= SC_QSPI_ACR_MODE_SINGLE;
			break;
		}
		sys_write32(acr, cfg->base + SC_QSPI_ACR_OFFSET);
	} else {
		sys_write32(SC_QSPI_ACR_SS_OFF, cfg->base + SC_QSPI_ACR_OFFSET);
	}

	return spi_sc_wait_for_idle(dev);
}

static int spi_sc_xfer(struct spi_context *ctx, const struct device *dev, uint8_t dfs)
{
	int ret;
	bool discard;
	size_t last_tx_size = 0;
	const struct spi_sc_cfg *cfg = dev->config;

	/*
	 * spi_context_cs_control() in spi_sc_transeive() is a fake.
	 * In the SPI IP core, the Chip Select (CS) is contolled through registers.
	 */
	ret = spi_sc_cs_control(ctx, dev, true);
	if (ret < 0) {
		LOG_ERR("Failed to assert the CS signal. (%d)", ret);
		goto end;
	}

	while (spi_context_tx_on(ctx) || spi_context_rx_on(ctx)) {

		LOG_DBG("tx buf count %d len %d", ctx->tx_count, ctx->tx_len);
		LOG_DBG("rx buf count %d len %d", ctx->rx_count, ctx->rx_len);

		if (spi_context_tx_buf_on(ctx)) {
			ret = spi_sc_send_tx_data(ctx, dev, dfs, &last_tx_size);
			if (ret < 0) {
				LOG_ERR("Failed to send the TX data. (%d)", ret);
				goto end;
			}
		}

		if (spi_context_rx_buf_on(ctx)) {
			discard = false;
			ret = spi_sc_read_rx_data(ctx, dev, ctx->current_rx->len, dfs, discard);
			if (ret < 0) {
				LOG_ERR("Failed to read the RX data. (%d)", ret);
				goto end;
			}
		} else if (cfg->data_capture_mode) {
			/* Read and discard capture data when write command */
			discard = true;
			ret = spi_sc_read_rx_data(ctx, dev, last_tx_size, dfs, discard);
			if (ret < 0) {
				LOG_ERR("Failed to read the RX data. (%d)", ret);
				goto end;
			}
		}
	}

end:
	ret = spi_sc_cs_control(ctx, dev, false);
	if (ret < 0) {
		LOG_ERR("Failed to negate the CS signal. (%d)", ret);
	}

	spi_context_complete(ctx, dev, ret);

	return ret;
}

static int spi_sc_configure(struct spi_context *ctx, const struct spi_config *config,
			    const struct spi_sc_cfg *cfg)
{
	int ret = 0;

	if ((config->operation & SPI_OP_MODE_SLAVE) == SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode is not supported");
		ret = -ENOTSUP;
		goto end;
	}

	if ((config->operation & SPI_MODE_LOOP) == SPI_MODE_LOOP) {
		LOG_ERR("Loop back mode is not supported");
		ret = -ENOTSUP;
		goto end;
	}

	if ((config->operation & SPI_LINES_OCTAL) == SPI_LINES_OCTAL) {
		LOG_ERR("Unsupported configuration. 0x%08x", config->operation);
		ret = -ENOTSUP;
		goto end;
	}

	/*
	 * QSPI IP core supports a Dual and Quad SPI mode, but it has not been
	 * implemented in this driver yet.
	 */
	if ((config->operation & SPI_LINES_DUAL) == SPI_LINES_DUAL) {
		LOG_ERR("Dual SPI mode is not supprted yet.");
		ret = -ENOTSUP;
		goto end;
	}

	if ((config->operation & SPI_LINES_QUAD) == SPI_LINES_QUAD) {
		LOG_ERR("Quad SPI mode is not supprted yet.");
		ret = -ENOTSUP;
		goto end;
	}

	ctx->config = config;

end:
	return ret;
}

static void spi_sc_clock_configure(const struct spi_sc_cfg *cfg)
{
	uint32_t clock;

	clock = cfg->cpol << SC_QSPI_CCR_SCKPOL_SHIFT | cfg->cpha << SC_QSPI_CCR_SCKPHA_SHIFT |
		cfg->spiclk_div << SC_QSPI_CCR_SCKDIV_SHIFT;

	sys_write32(clock, cfg->base + SC_QSPI_CCR_OFFSET);
	LOG_DBG("SPI Clock Control Register: 0x%08x", clock);
}

static int spi_sc_transceive(const struct device *dev, const struct spi_config *config,
			     const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs)
{
	const struct spi_sc_cfg *cfg = dev->config;
	struct spi_sc_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	int ret;
	/* SC QSPI Core only supports 1-byte words. */
	uint8_t dfs = 1;

	if (config->slave >= cfg->slave_num) {
		LOG_ERR("Invalid slave number: %d (range: 0 - %d)", config->slave,
			cfg->slave_num - 1);
		ret = -EINVAL;
		goto end;
	}

	ret = spi_sc_configure(ctx, config, cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure for SC QSPI. (%d)", ret);
		goto end;
	}

	spi_context_lock(ctx, false, NULL, NULL, config);
	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);
	spi_context_cs_control(ctx, true);

	ret = spi_sc_xfer(ctx, dev, dfs);
	if (ret < 0) {
		LOG_ERR("Failed to xfer the SPI data, but will continue the cleanup. (%d)", ret);
		goto cleanup;
	}

	ret = spi_context_wait_for_completion(ctx);

cleanup:
	spi_context_cs_control(ctx, false);
	spi_context_release(ctx, ret);

end:
	return ret;
}

static int spi_sc_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_sc_data *data = dev->data;

	if (!spi_context_configured(&data->ctx, config)) {
		return -EINVAL;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static void spi_sc_enable_irq(const struct device *dev)
{
	const struct spi_sc_cfg *cfg = dev->config;

	sys_set_bits(cfg->base + SC_QSPI_IER_OFFSET, SC_QSPI_IER_ALL);
	cfg->irq_init(dev);
}

static int spi_sc_init(const struct device *dev)
{
	const struct spi_sc_cfg *cfg = dev->config;
	struct spi_sc_data *data = dev->data;

	k_sem_init(&data->idle_sem, 0, 1);

	/*
	 * At present, as there are no different devices connected to the
	 * SPI bus, the clock frequency and mode of SPI is statically
	 * defined in the device tree (dts) to be common across all buses.
	 * Eventually, it should be configured based on the values set in
	 * spi_config.
	 */
	spi_sc_clock_configure(cfg);

	spi_context_unlock_unconditionally(&data->ctx);

	if (cfg->data_capture_mode) {
		sys_write32(SC_QSPI_DCMSR_DTCAPT, cfg->base + SC_QSPI_DCMSR_OFFSET);
	}

	spi_sc_enable_irq(dev);

	return 0;
}

static struct spi_driver_api spi_sc_api = {
	.transceive = spi_sc_transceive,
	.release = spi_sc_release,
};

#define SPI_SC_INIT(n)                                                                             \
                                                                                                   \
	static void spi_sc_##n##_irq_init(const struct device *dev);                               \
	static struct spi_sc_data spi_sc_data_##n = {                                              \
		SPI_CONTEXT_INIT_LOCK(spi_sc_data_##n, ctx),                                       \
		SPI_CONTEXT_INIT_SYNC(spi_sc_data_##n, ctx),                                       \
	};                                                                                         \
                                                                                                   \
	static const struct spi_sc_cfg spi_sc_cfg_##n = {                                          \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.irq_init = spi_sc_##n##_irq_init,                                                 \
		.data_capture_mode = DT_INST_PROP(n, data_capture_mode),                           \
		.slave_num = DT_INST_PROP(n, slave_num),                                           \
		.cpol = DT_INST_PROP(n, cpol),                                                     \
		.cpha = DT_INST_PROP(n, cpha),                                                     \
		.spiclk_div = DT_INST_PROP(n, spiclk_div),                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, spi_sc_init, NULL, &spi_sc_data_##n, &spi_sc_cfg_##n,             \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY, &spi_sc_api);                 \
	static void spi_sc_##n##_irq_init(const struct device *dev)                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), 0, spi_sc_isr, DEVICE_DT_INST_GET(n), 0);             \
                                                                                                   \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(SPI_SC_INIT)

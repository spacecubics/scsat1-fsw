/*
 * Copyright (c) 2023 Space Cubics,LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sc_i2c

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_sc, CONFIG_I2C_LOG_LEVEL);

/* Registers */
#define SC_I2C_ENR_OFFSET     (0x0000) /* I2C Master Enable Register */
#define SC_I2C_TXFIFOR_OFFSET (0x0004) /* I2C Master TX FIFO Register */
#define SC_I2C_RXFIFOR_OFFSET (0x0008) /* I2C Master RX FIFO Register */
#define SC_I2C_BSR_OFFSET     (0x000C) /* I2C Master Bus Status Register */
#define SC_I2C_ISR_OFFSET     (0x0010) /* I2C Master Interrupt Status Register */
#define SC_I2C_IER_OFFSET     (0x0014) /* I2C Master Interrupt Enable Register */
#define SC_I2C_FIFOSR_OFFSET  (0x0018) /* I2C Master FIFO Status Register */
#define SC_I2C_FIFORR_OFFSET  (0x001C) /* I2C Master FIFO Reset Register */
#define SC_I2C_FTLSR_OFFSET   (0x0020) /* I2C Master FIFO Threshold Level Setting Register */
#define SC_I2C_SCLTSR_OFFSET  (0x0024) /* I2C Master SCL Timeout Setting Register */
#define SC_I2C_THDSTAR_OFFSET (0x0030) /* I2C Master START Hold Timing Setting Register */
#define SC_I2C_TSUSTOR_OFFSET (0x0034) /* I2C Master STOP Setup Timing Setting Register */
#define SC_I2C_TSUSTAR_OFFSET (0x0038) /* I2C Master Repeated START Setup Timing Setting Register */
#define SC_I2C_THIGHR_OFFSET  (0x003C) /* I2C Master Clock High Timing Setting Register */
#define SC_I2C_THDDATR_OFFSET (0x0040) /* I2C Master Data Hold Timing Setting Register */
#define SC_I2C_TSUDATR_OFFSET (0x0044) /* I2C Master Data Setup Timing Setting Register */
#define SC_I2C_TBUFR_OFFSET   (0x0048) /* I2C Master Bus Free Timing Setting Register */
#define SC_I2C_TBSMPLR_OFFSET (0x004C) /* I2C Master Bus Sampling Timing Setting Register */
#define SC_I2C_VER_OFFSET     (0xF000) /* I2C Master Controller IP Version Register */

/* I2C Master Enable Register */
#define SC_I2C_ENABLE  BIT(0)

/* I2C Master TX FIFO Register */
#define SC_I2C_TXFIFO_RESTART BIT(9)
#define SC_I2C_TXFIFO_STOP    BIT(8)

/* I2C Master Interrupt Status Register */
#define SC_I2C_SCLTO     BIT(12)
#define SC_I2C_RXFIFOUDF BIT(11)
#define SC_I2C_TXFIFOOV  BIT(10)
#define SC_I2C_BITER     BIT(9)
#define SC_I2C_ACKER     BIT(8)
#define SC_I2C_RXFIFOOTH BIT(5)
#define SC_I2C_TXFIFOUTH BIT(4)
#define SC_I2C_ARBLST    BIT(1)
#define SC_I2C_COMP      BIT(0)

/* I2C Master Interrupt Enable Register */
#define SC_I2C_SCLTOENB     BIT(12)
#define SC_I2C_RXFIFOUDFENB BIT(11)
#define SC_I2C_TXFIFOOVFENB BIT(10)
#define SC_I2C_BITERENB     BIT(9)
#define SC_I2C_ACKERENB     BIT(8)
#define SC_I2C_RXFIFOOTHENB BIT(5)
#define SC_I2C_TXFIFOUTHENB BIT(4)
#define SC_I2C_ARBLSTENB    BIT(1)
#define SC_I2C_COMPENB      BIT(0)
#define SC_I2C_IER_ALL \
	(SC_I2C_SCLTOENB | SC_I2C_RXFIFOUDFENB | SC_I2C_TXFIFOOVFENB | SC_I2C_BITERENB | \
	 SC_I2C_ACKERENB | SC_I2C_RXFIFOOTHENB | SC_I2C_TXFIFOUTHENB | SC_I2C_ARBLSTENB | \
	 SC_I2C_COMPENB)

/* I2C Master FIFO Reset Register */
#define SC_I2C_RXFIFORST BIT(16)
#define SC_I2C_TXFIFORST BIT(0)
#define SC_I2C_RESET_FIFO_ALL (SC_I2C_RXFIFORST | SC_I2C_TXFIFORST)

/* I2C Master Controller IP Version Register */
#define SC_I2C_VER_MAJOR(x) (((x)&0xff000000) >> 24)
#define SC_I2C_VER_MINOR(x) (((x)&0x00ff0000) >> 16)
#define SC_I2C_VER_PATCH(x) (((x)&0x0000ffff) >> 0)

typedef void (*irq_init_func_t)(const struct device *dev);

struct i2c_sc_cfg {
	uint32_t reg_addr;
	irq_init_func_t irq_init;
	uint32_t bitrate;
};

struct i2c_sc_data {
	uint8_t status_mask;
	struct k_sem trans_sem;
};

static inline void i2c_sc_enable(const struct i2c_sc_cfg *config)
{
	sys_set_bits(config->reg_addr + SC_I2C_ENR_OFFSET, SC_I2C_ENABLE);
}

static inline void i2c_sc_disable(const struct i2c_sc_cfg *config)
{
	sys_clear_bits(config->reg_addr + SC_I2C_ENR_OFFSET, SC_I2C_ENABLE);
}

static inline void i2c_sc_reset_fifo(const struct i2c_sc_cfg *config)
{
	sys_set_bits(config->reg_addr + SC_I2C_FIFORR_OFFSET, SC_I2C_RESET_FIFO_ALL);
}

static void i2c_sc_enable_irq(const struct device * dev)
{
	const struct i2c_sc_cfg *config = dev->config;

	sys_set_bits(config->reg_addr + SC_I2C_IER_OFFSET, SC_I2C_IER_ALL);
	config->irq_init(dev);
}

static void i2c_sc_isr(const struct device *dev)
{
	const struct i2c_sc_cfg *config = dev->config;
	struct i2c_sc_data *data = dev->data;
	uint32_t isr;

	isr = sys_read32(config->reg_addr + SC_I2C_ISR_OFFSET);
	LOG_DBG("IRQ Status 0x%08x", isr);

	sys_write32(isr, config->reg_addr + SC_I2C_ISR_OFFSET);

	if (isr & SC_I2C_COMP) {
		k_sem_give(&data->trans_sem);
	}
}

static int i2c_sc_configure(const struct device *dev, uint32_t dev_config)
{
	/* Not implemented yet */
	return -ENOTSUP;
}

static int i2c_sc_init(const struct device *dev)
{
	const struct i2c_sc_cfg *config = dev->config;
	struct i2c_sc_data *data = dev->data;
	int v;

	i2c_sc_enable_irq(dev);

	k_sem_init(&data->trans_sem, 0, 1);

	/* Dump Version information */
	v = sys_read32(config->reg_addr + SC_I2C_VER_OFFSET);
	LOG_DBG("Space Cubics I2C Master controller v%d.%d.%d initialized",
			SC_I2C_VER_MAJOR(v),
			SC_I2C_VER_MINOR(v),
			SC_I2C_VER_PATCH(v));

	return 0;
}

static void i2c_sc_send_slave_addr(const struct i2c_sc_cfg *config, uint8_t addr, uint8_t flags)
{
	uint32_t reg = (addr << 1) | ((flags & I2C_MSG_RW_MASK) ? I2C_MSG_READ : I2C_MSG_WRITE);

	sys_write32(reg, config->reg_addr + SC_I2C_TXFIFOR_OFFSET);
	LOG_DBG("Send slave addr: 0x%08x", reg);
}

static int i2c_sc_read_data(const struct i2c_sc_cfg *config, struct i2c_sc_data *data, uint32_t len)
{
	int ret;

	/* Set the byte length for reading and initiate the data read from the I2C device. */
	sys_write32((len - 1) | SC_I2C_TXFIFO_STOP, config->reg_addr + SC_I2C_TXFIFOR_OFFSET);

	/* Waiting for the 'I2C_STOP' interrupt from the FPGA. */
	ret = k_sem_take(&data->trans_sem, K_MSEC(CONFIG_I2C_SC_TRANSFER_TIMEOUT_MS));
	if (ret < 0) {
		LOG_ERR("Waiting for the 'I2C_COM' interrupt, but it timed out. (%d)", ret);
	}

	return ret;
}

static void i2c_sc_read_from_fifo(const struct i2c_sc_cfg *config, struct i2c_msg *msgs)
{
	for (uint32_t i = 0; i < msgs->len; i++) {
		msgs->buf[i] = sys_read32(config->reg_addr + SC_I2C_RXFIFOR_OFFSET);
	}
}

static int i2c_sc_write_data(const struct i2c_sc_cfg *config, struct i2c_sc_data *data,
			struct i2c_msg *msgs, bool restart)
{
	int ret = 0;
	bool stop = false;
	uint32_t flag;

	if ((msgs->flags & I2C_MSG_STOP) != 0) {
		stop = true;
	}

	for (uint32_t i = 0; i < msgs->len; i++) {
		flag = 0;
		if (i == msgs->len - 1) {
			if (stop) {
				flag = SC_I2C_TXFIFO_STOP;
			} else if (restart) {
				flag = SC_I2C_TXFIFO_RESTART;
			}
		}
		sys_write32(msgs->buf[i] | flag, config->reg_addr + SC_I2C_TXFIFOR_OFFSET);
		LOG_DBG("Write data[%d]: 0x%08x", i, msgs->buf[i] | flag);
	}

	/* Waiting for the 'I2C_STOP' interrupt from the FPGA. */
	if (stop) {
		ret = k_sem_take(&data->trans_sem, K_MSEC(CONFIG_I2C_SC_TRANSFER_TIMEOUT_MS));
		if (ret < 0) {
			LOG_ERR("Waiting for the 'I2C_COM' interrupt, but it timed out. (%d)", ret);
		}
	}

	return ret;
}

static int i2c_sc_transfer_msg(const struct device *dev, struct i2c_msg *msgs, bool restart)
{
	const struct i2c_sc_cfg *config = dev->config;
	struct i2c_sc_data *data = dev->data;
	int ret;

	if ((msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
		ret = i2c_sc_read_data(config, data, msgs->len);
		if (ret < 0) {
			LOG_ERR("Failed to read data from I2C device. (%d)", ret);
			goto end;
		}
		i2c_sc_read_from_fifo(config, msgs);
	} else {
		ret = i2c_sc_write_data(config, data, msgs, restart);
		if (ret < 0) {
			LOG_ERR("Failed to write data to I2C device. (%d)", ret);
		}
	}

end:
	return ret;
}

static int i2c_sc_transfer(const struct device *dev,
			     struct i2c_msg *msgs, uint8_t num_msgs,
			     uint16_t addr)
{
	const struct i2c_sc_cfg *config = dev->config;
	int ret = 0;
	struct i2c_msg *next_msg;
	bool restart;

	if (num_msgs == 0) {
		ret = -EINVAL;
		goto end;
	}

	/*
	 * I2C IP core monitors the idle status on the I2C bus, so this
	 * driver does not verify the idle status.
	 */

	/*
	 * I2C IP core supports a 10-bit address, but it has not been
	 * implemented in this driver yet.
	 */
	if ((msgs->flags & I2C_MSG_ADDR_10_BITS) != 0) {
		ret = -ENOTSUP;
		goto end;
	}

	i2c_sc_reset_fifo(config);

	i2c_sc_enable(config);

	i2c_sc_send_slave_addr(config, addr, msgs->flags);

	while (num_msgs-- > 0) {

		LOG_DBG("Msg flag: 0x%x, len: %d", msgs->flags, msgs->len);

		/*
		 * RESTAT flag is used to send the 'I2C Repeated START Condition' to
		 * the I2C device, enabling the continuation of writing/reading data.
		 *
		 * i.g) Write Msg1 -> Repeated START Condition -> Read Msg2
		 *
		 * In Zephyr's I2C driver API, the RESTART flag is included in Msg2,
		 * but in the I2C IP core, it must be set in the register along with
		 * Msg1. Therefore, we check the REPEAT flag in the next Msg.
		 */
		restart = false;
		if (num_msgs > 0) {
			next_msg = msgs + 1;
			if ((next_msg->flags  & I2C_MSG_RESTART) != 0) {
				restart = true;
			}
		}

		/*
		 * If the 'I2C Repeated START Condition' was sent during the last
		 * operation, resend the slave address.
		 */
		if ((msgs->flags & I2C_MSG_RESTART) != 0) {
			i2c_sc_send_slave_addr(config, addr, msgs->flags);
		}

		if (msgs->len) {
			ret = i2c_sc_transfer_msg(dev, msgs, restart);
			if (ret < 0) {
				LOG_ERR("Failed to transfer the I2C data (%d)", ret);
				break;
			}
		}

		msgs++;
	}

	i2c_sc_disable(config);

end:
	return ret;
}

static const struct i2c_driver_api i2c_sc_driver_api = {
	.configure = i2c_sc_configure,
	.transfer = i2c_sc_transfer,
};

#define I2C_SC_INIT(n)							\
	static void i2c_sc_##n##_irq_init(const struct device *dev);	\
	static const struct i2c_sc_cfg i2c_sc_cfg_##n = {		\
		.reg_addr = DT_INST_REG_ADDR(n),			\
		.irq_init = i2c_sc_##n##_irq_init,			\
		.bitrate = DT_INST_PROP(n, clock_frequency),		\
	};								\
									\
	static struct i2c_sc_data i2c_sc_data_##n;			\
									\
	I2C_DEVICE_DT_INST_DEFINE(n,					\
			      i2c_sc_init,				\
			      NULL,					\
			      &i2c_sc_data_##n,				\
			      &i2c_sc_cfg_##n,				\
			      POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,	\
			      &i2c_sc_driver_api			\
			      );					\
	static void i2c_sc_##n##_irq_init(const struct device *dev)	\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    0,						\
			    i2c_sc_isr,					\
			    DEVICE_DT_INST_GET(n), 0);			\
									\
		irq_enable(DT_INST_IRQN(n));				\
	}

DT_INST_FOREACH_STATUS_OKAY(I2C_SC_INIT)

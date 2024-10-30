/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include "pwrctrl_adcs.h"
#include "imu.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu, CONFIG_SC_LIB_IMU_LOG_LEVEL);

#define IMU_STD_DATA_SIZE    (19U)
#define IMU_EXT_DATA_SIZE    (26U)
#define IMU_SPI_WRITE        (0U)
#define IMU_SPI_READ         BIT(7)
#define IMU_IMU_DATA_EXT_REG (0x60)
#define IMU_IMU_DATA_STD_REG (0x61)

#define IMU_MEMPHIS3_ID (0x20)

static struct spi_config config;
const struct device *imu = DEVICE_DT_GET(DT_NODELABEL(spi3));

static void imu_init_spi_config(void)
{
	config.slave = 0;
	config.frequency = 12000000;
	memset(&config.cs, 0, sizeof(config.cs));
	config.operation = SPI_OP_MODE_MASTER | SPI_LINES_SINGLE | SPI_WORD_SET(8);
}

static void imu_decode_imu_data_std(uint8_t *rxbuf, struct imu_data *data)
{
	data->id = rxbuf[0];
	data->timestamp = sys_get_be32(&rxbuf[1]);
	data->temp = sys_get_be16(&rxbuf[5]);
	data->gyro.x = sys_get_be16(&rxbuf[7]);
	data->gyro.y = sys_get_be16(&rxbuf[9]);
	data->gyro.z = sys_get_be16(&rxbuf[11]);
	data->acc.x = sys_get_be16(&rxbuf[13]);
	data->acc.y = sys_get_be16(&rxbuf[15]);
	data->acc.z = sys_get_be16(&rxbuf[17]);
}

static void imu_decode_imu_data_ext(uint8_t *rxbuf, struct imu_data *data)
{
	data->id = rxbuf[0];
	data->timestamp = sys_get_be32(&rxbuf[1]);
	data->temp = sys_get_be24(&rxbuf[5]);
	data->gyro.x = sys_get_be24(&rxbuf[8]);
	data->gyro.y = sys_get_be24(&rxbuf[11]);
	data->gyro.z = sys_get_be24(&rxbuf[14]);
	data->acc.x = sys_get_be24(&rxbuf[17]);
	data->acc.y = sys_get_be24(&rxbuf[20]);
	data->acc.z = sys_get_be24(&rxbuf[23]);
}

void imu_enable(void)
{
	sc_adcs_imu_reset_release();
	imu_init_spi_config();
}

void imu_disable(void)
{
	sc_adcs_imu_reset();
}

int get_imu_data_std(struct imu_data *data)
{
	int ret;
	uint8_t txbuf[1];
	uint8_t rxbuf[IMU_STD_DATA_SIZE];
	struct spi_buf tx_buf[1];
	struct spi_buf rx_buf[1];
	struct spi_buf_set tx_set = {.buffers = tx_buf, .count = 1};
	struct spi_buf_set rx_set = {.buffers = rx_buf, .count = 1};

	txbuf[0] = IMU_IMU_DATA_STD_REG | IMU_SPI_READ;
	tx_buf[0].buf = txbuf;
	tx_buf[0].len = 1;
	rx_buf[0].buf = rxbuf;
	rx_buf[0].len = IMU_STD_DATA_SIZE;

	ret = spi_transceive(imu, &config, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("Failed to SPI transfer for IMU Data STd (19byte). (%d)", ret);
		goto end;
	}

	imu_decode_imu_data_std(rxbuf, data);
	if (data->id != IMU_MEMPHIS3_ID) {
		LOG_ERR("Invalid ID: 0x%02x", data->id);
		ret = -1;
	}
end:
	return ret;
}

int get_imu_data_ext(struct imu_data *data)
{
	int ret;
	uint8_t txbuf[1];
	uint8_t rxbuf[IMU_EXT_DATA_SIZE];
	struct spi_buf tx_buf[1];
	struct spi_buf rx_buf[1];
	struct spi_buf_set tx_set = {.buffers = tx_buf, .count = 1};
	struct spi_buf_set rx_set = {.buffers = rx_buf, .count = 1};

	txbuf[0] = IMU_IMU_DATA_EXT_REG | IMU_SPI_READ;
	tx_buf[0].buf = txbuf;
	tx_buf[0].len = 1;
	rx_buf[0].buf = rxbuf;
	rx_buf[0].len = IMU_EXT_DATA_SIZE;

	ret = spi_transceive(imu, &config, &tx_set, &rx_set);
	if (ret < 0) {
		LOG_ERR("Failed to SPI transfer for IMU Data STd (19byte). (%d)", ret);
		goto end;
	}

	imu_decode_imu_data_ext(rxbuf, data);
	if (data->id != IMU_MEMPHIS3_ID) {
		LOG_ERR("Invalid ID: 0x%02x", data->id);
		ret = -1;
	}
end:
	return ret;
}

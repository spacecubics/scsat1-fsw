/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include "temp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(temp);

#define IO_TEMP_REG (0x00)

#define IO_TEMP_ONBOARD_1_ADDR (0x4F)
#define IO_TEMP_ONBOARD_2_ADDR (0x4F)
#define IO_TEMP_X_PLUS_ADDR    (0x48)
#define IO_TEMP_X_MINUS_ADDR   (0x48)
#define IO_TEMP_Y_PLUS_ADDR    (0x4A)
#define IO_TEMP_Y_MINUS_ADDR   (0x4A)

static const struct device *get_temp_device(enum io_temp_pos pos)
{
	const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));

	switch (pos) {
	case IO_TEMP_POS_ONBOARD_1:
	case IO_TEMP_POS_X_PLUS:
	case IO_TEMP_POS_Y_PLUS:
		if (device_is_ready(i2c0)) {
			return i2c0;
		}
		break;
	case IO_TEMP_POS_ONBOARD_2:
	case IO_TEMP_POS_X_MINUS:
	case IO_TEMP_POS_Y_MINUS:
		if (device_is_ready(i2c1)) {
			return i2c1;
		}
		break;
	default:
		break;
	}

	return NULL;
}

static int get_slave_addr(enum io_temp_pos pos, uint16_t *addr)
{
	int ret = 0;

	switch (pos) {
	case IO_TEMP_POS_ONBOARD_1:
		*addr = IO_TEMP_ONBOARD_1_ADDR;
		break;
	case IO_TEMP_POS_ONBOARD_2:
		*addr = IO_TEMP_ONBOARD_2_ADDR;
		break;
	case IO_TEMP_POS_X_PLUS:
		*addr = IO_TEMP_X_PLUS_ADDR;
		break;
	case IO_TEMP_POS_X_MINUS:
		*addr = IO_TEMP_X_MINUS_ADDR;
		break;
	case IO_TEMP_POS_Y_PLUS:
		*addr = IO_TEMP_Y_PLUS_ADDR;
		break;
	case IO_TEMP_POS_Y_MINUS:
		*addr = IO_TEMP_Y_MINUS_ADDR;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

int get_ioboard_temp(enum io_temp_pos pos, float *temp)
{
	int ret;
	uint8_t data[2];
	uint16_t addr;

	const struct device *dev = get_temp_device(pos);
	if (dev == NULL) {
		LOG_ERR("I2C device is not ready. (pos: %d)", pos);
		ret = -ENODEV;
		goto end;
	}

	ret = get_slave_addr(pos, &addr);
	if (ret < 0) {
		goto end;
	}

	ret = i2c_burst_read(dev, addr, IO_TEMP_REG, data, ARRAY_SIZE(data));
	if (ret < 0) {
		LOG_ERR("Failed to i2c_burst_read for Temperature Sensor (pos: %d) (%d)",
			    pos, ret);
		goto end;
	}

	data[1] = data[1] >> 4;
	*temp = (int8_t)data[0] + (float)data[1] * 0.0625f;
end:
	return ret;
}

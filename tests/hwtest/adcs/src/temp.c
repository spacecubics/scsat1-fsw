/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include "temp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(temp);

#define ADCS_TEMP_REG (0x00)

#define ADCS_TEMP_ONBOARD_1_ADDR (0x4B)
#define ADCS_TEMP_ONBOARD_2_ADDR (0x4F)
#define ADCS_TEMP_RW_ADDR        (0x4C)

static const struct device *get_temp_device(enum adcs_temp_pos pos)
{
	const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	switch (pos) {
	case ADCS_TEMP_POS_ONBOARD_1:
	case ADCS_TEMP_POS_ONBOARD_2:
	case ADCS_TEMP_POS_RW:
		if (device_is_ready(i2c0)) {
			return i2c0;
		}
		break;
	default:
		break;
	}

	return NULL;
}

static int get_slave_addr(enum adcs_temp_pos pos, uint16_t *addr)
{
	int ret = 0;

	switch (pos) {
	case ADCS_TEMP_POS_ONBOARD_1:
		*addr = ADCS_TEMP_ONBOARD_1_ADDR;
		break;
	case ADCS_TEMP_POS_ONBOARD_2:
		*addr = ADCS_TEMP_ONBOARD_2_ADDR;
		break;
	case ADCS_TEMP_POS_RW:
		*addr = ADCS_TEMP_RW_ADDR;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

int get_adcs_temp(enum adcs_temp_pos pos, float *temp)
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

	ret = i2c_burst_read(dev, addr, ADCS_TEMP_REG, data, ARRAY_SIZE(data));
	if (ret < 0) {
		LOG_ERR("Failed to i2c_burst_read for Temperature Sensor. (pos: %d) (%d)",
			    pos, ret);
		goto end;
	}

	data[1] = data[1] >> 4;
	*temp = (int8_t)data[0] + (float)data[1] * 0.0625;

end:
	return ret;
}

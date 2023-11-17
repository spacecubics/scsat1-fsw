/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "sunsens.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sunsens);

#define SUNSENS_SLAVE_ADDR     (0x05)
#define SUNSENS_SAMPLE_SUN_CMD (0x00)
#define SUNSENS_GET_SUN_CMD    (0x01)
#define SUNSENS_SAMPLE_TMP_CMD (0x04)
#define SUNSENS_GET_TMP_CMD    (0x05)

#define SUNSENS_RETRY_COUNT (100U)

static const struct device *get_sunsens_device(enum sunsens_pos pos)
{
	const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));

	switch (pos) {
	case SUNSENS_POS_Y_PLUS:
		if (device_is_ready(i2c0)) {
			return i2c0;
		}
		break;
	case SUNSENS_POS_Y_MINUS:
		if (device_is_ready(i2c1)) {
			return i2c1;
		}
		break;
	default:
		break;
	}

	return NULL;
}

int get_sunsens_temp(enum sunsens_pos pos, float *temp)
{
	int ret;
	uint8_t cmd = SUNSENS_SAMPLE_TMP_CMD;
	uint8_t data[2];

	const struct device *dev = get_sunsens_device(pos);
	if (dev == NULL) {
		LOG_ERR("I2C device is not ready. (pos: %d)", pos);
		ret = -ENODEV;
		goto end;
	}

	ret = i2c_write(dev, &cmd, 1, SUNSENS_SLAVE_ADDR);
	if (ret < 0) {
		LOG_ERR("Failed to i2c_write for sampling temperature. (%d)", ret);
	}

	k_sleep(K_MSEC(30));

	ret = i2c_burst_read(dev, SUNSENS_SLAVE_ADDR, SUNSENS_GET_TMP_CMD, data, ARRAY_SIZE(data));
	if (ret < 0) {
		LOG_ERR("Failed to i2c_burst_read for Sun Sensor Temperature. (%d)", ret);
		goto end;
	}

	*temp = (((int16_t)((data[0] << 8) + data[1])) >> 2) * 0.03125;
end:
	return ret;
}

int get_sunsens_data(enum sunsens_pos pos, struct sunsens_data *sun_data)
{
	int ret;
	uint8_t cmd = SUNSENS_SAMPLE_SUN_CMD;
	uint8_t data[8];

	const struct device *dev = get_sunsens_device(pos);
	if (dev == NULL) {
		LOG_ERR("I2C device is not ready. (pos: %d)", pos);
		ret = -ENODEV;
		goto end;
	}

	ret = i2c_write(dev, &cmd, 1, SUNSENS_SLAVE_ADDR);
	if (ret < 0) {
		LOG_ERR("Failed to i2c_write for sampling Sun Data. (%d)", ret);
		ret = -ENODEV;
		goto end;
	}

	k_sleep(K_MSEC(30));

	ret = i2c_burst_read(dev, SUNSENS_SLAVE_ADDR, SUNSENS_GET_SUN_CMD, data, ARRAY_SIZE(data));
	if (ret < 0) {
		LOG_ERR("Failed to i2c_burst_read for Sun Sensor Data. (%d)", ret);
		goto end;
	}

	sun_data->a = (data[0] << 8) | data[1];
	sun_data->b = (data[2] << 8) | data[3];
	sun_data->c = (data[4] << 8) | data[5];
	sun_data->d = (data[6] << 8) | data[7];
end:
	return ret;
}

/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "mgnm.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mgnm, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

#define MGNM_SLAVE_ADDR  (0x30)
#define MGNM_XOUTL_REG   (0x00)
#define MGNM_XOUTH_REG   (0x01)
#define MGNM_YOUTL_REG   (0x02)
#define MGNM_YOUTH_REG   (0x03)
#define MGNM_ZOUTL_REG   (0x04)
#define MGNM_ZOUTH_REG   (0x05)
#define MGNM_TOUT_REG    (0x06)
#define MGNM_STATUS_REG  (0x07)
#define MGNM_CTRL0_REG   (0x08)

#define MGNM_START_T BIT(1)
#define MGNM_START_M BIT(0)

#define MGNM_T_DONE (0x02)
#define MGNM_M_DONE (0x01)

#define MGNM_RETRY_COUNT (100U)

static const struct device *get_mgnm_device(enum mgnm_pos pos)
{
	const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));
	const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));

	switch (pos) {
	case MGNM_POS_X_PLUS:
		if (device_is_ready(i2c0)) {
			return i2c0;
		}
		break;
	case MGNM_POS_X_MINUS:
		if (device_is_ready(i2c1)) {
			return i2c1;
		}
		break;
	default:
		break;
	}

	return NULL;
}

int wait_for_measurement_complete(const struct device *dev, uint8_t mask_bit)
{
	int i;
	int ret;
	uint8_t data;

	/* Wait for the measurement to be complete */
	for (i = 0; i < MGNM_RETRY_COUNT; i++) {
		ret = i2c_burst_read(dev, MGNM_SLAVE_ADDR, MGNM_STATUS_REG, &data, 1);
		if (ret < 0) {
			LOG_ERR("Failed to i2c_burst_read for Status Register: %d", ret);
			goto end;
		}

		if ((data & mask_bit) == mask_bit) {
			break;
		}
	}

	if (i == MGNM_RETRY_COUNT) {
		ret = -ETIMEDOUT;
	}

end:
	return ret;
}

int start_mgnm_temp_measurement(enum mgnm_pos pos)
{
	int ret;
	uint8_t reg;

	const struct device *dev = get_mgnm_device(pos);
	if (dev == NULL) {
		LOG_ERR("I2C device is not ready. (pos: %d)", pos);
		ret = -ENODEV;
		goto end;
	}

	reg = MGNM_START_T;
	ret = i2c_burst_write(dev, MGNM_SLAVE_ADDR, MGNM_CTRL0_REG, &reg, 1);
	if (ret < 0) {
		LOG_ERR("Failed to i2c_burst_write for Magnet Meter Control: %d", ret);
	}

	ret = wait_for_measurement_complete(dev, MGNM_T_DONE);
end:
	return ret;
}

int start_mgnm_magnet_measurement(enum mgnm_pos pos)
{
	int ret;
	uint8_t reg;

	const struct device *dev = get_mgnm_device(pos);
	if (dev == NULL) {
		LOG_ERR("I2C device is not ready. (pos: %d)", pos);
		ret = -ENODEV;
		goto end;
	}

	reg = MGNM_START_M;
	ret = i2c_burst_write(dev, MGNM_SLAVE_ADDR, MGNM_CTRL0_REG, &reg, 1);
	if (ret < 0) {
		LOG_ERR("Failed to i2c_burst_write for Magnet Meter Control: %d", ret);
	}

	ret = wait_for_measurement_complete(dev, MGNM_M_DONE);
end:
	return ret;
}

int get_mgnm_temp(enum mgnm_pos pos, float *temp)
{
	int ret;
	uint8_t data;

	const struct device *dev = get_mgnm_device(pos);
	if (dev == NULL) {
		LOG_ERR("I2C device is not ready. (pos: %d)", pos);
		ret = -ENODEV;
		goto end;
	}

	ret = i2c_burst_read(dev, MGNM_SLAVE_ADDR, MGNM_TOUT_REG, &data, 1);
	if (ret < 0) {
		LOG_ERR("Failed to i2c_burst_read for Magnet Meter Temperature: %d", ret);
		goto end;
	}

	*temp = -75 + (data * 0.7);
end:
	return ret;
}

int get_mgnm_magnet(enum mgnm_pos pos, struct magnet_field *magnet)
{
	int ret = 0;
	uint8_t data;
	uint8_t out_low_regs[] = {MGNM_XOUTL_REG, MGNM_YOUTL_REG, MGNM_ZOUTL_REG};
	uint8_t out_hi_regs[] = {MGNM_XOUTH_REG, MGNM_YOUTH_REG, MGNM_ZOUTH_REG};
	uint32_t out_data[3] = {0};

	const struct device *dev = get_mgnm_device(pos);
	if (dev == NULL) {
		LOG_ERR("I2C device is not ready. (pos: %d)", pos);
		ret = -ENODEV;
		goto end;
	}

	for (int i = 0; i < 3; i++) {
		ret = i2c_burst_read(dev, MGNM_SLAVE_ADDR, out_low_regs[i], &data, 1);
		if (ret < 0) {
			LOG_ERR("Failed to i2c_burst_read for Magnet Filed 0(%d): %d", i, ret);
			out_data[i] = 0xFFFF;
			ret--;
			continue;
		}
		out_data[i] |= data;

		ret = i2c_burst_read(dev, MGNM_SLAVE_ADDR, out_hi_regs[i], &data, 1);
		if (ret < 0) {
			LOG_ERR("Failed to i2c_burst_read for Magnet Filed 1(%d): %d", i, ret);
			out_data[i] = 0xFFFF;
			ret--;
			continue;
		}
		out_data[i] |= data << 8;
	}

	/* Xout */
	magnet->x_out = out_data[0];

	/* Yout */
	magnet->y_out = out_data[1];

	/* Zout */
	magnet->z_out = out_data[2];

end:
	return ret;
}

int print_mgnm_field(void)
{
	int ret;
	const char pos_name[][3] = {"X+", "X-"};
	enum mgnm_pos pos_list[] = {MGNM_POS_X_PLUS, MGNM_POS_X_MINUS};

	struct magnet_field magnet;
	for (int i = 0; i < ARRAY_SIZE(pos_list); i++) {
		ret = start_mgnm_magnet_measurement(pos_list[i]);
		if (ret < 0) {
			LOG_ERR("Magnetometer %s X/Y/Z: Failed", pos_name[i]);
			break;
		}

		ret = get_mgnm_magnet(pos_list[i], &magnet);
		if (ret < 0) {
			LOG_ERR("Magnetometer %s X/Y/Z: Failed", pos_name[i]);
			break;
		}

		LOG_INF("Magnetometer %s X: 0x%08x, Y: 0x%08x, Z: 0x%08x", pos_name[i],
			magnet.x_out, magnet.y_out, magnet.z_out);
	}

	return ret;
}

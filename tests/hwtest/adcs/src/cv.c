/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include "cv.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cv);

#define CV_ADCS_ADDR  (0x42)
#define CV_RW_ADDR    (0x40)

#define CV_CH1_SHUNT_REG (0x01)
#define CV_CH1_BUS_REG   (0x02)
#define CV_CH2_SHUNT_REG (0x03)
#define CV_CH2_BUS_REG   (0x04)
#define CV_CH3_SHUNT_REG (0x05)
#define CV_CH3_BUS_REG   (0x06)
#define CV_RW_SHUNT_REG  (0x04)
#define CV_RW_BUS_REG    (0x05)

static const struct device *get_adcs_cv_device(enum adcs_cv_pos pos)
{
	const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	switch (pos) {
	case ADCS_VDD_3V3_IMU_SHUNT:
	case ADCS_VDD_3V3_IMU_BUS:
	case ADCS_VDD_3V3_GPS_SHUNT:
	case ADCS_VDD_3V3_GPS_BUS:
	case ADCS_VDD_3V3_DRV_SHUNT:
	case ADCS_VDD_3V3_DRV_BUS:
		if (device_is_ready(i2c0)) {
			return i2c0;
		}
		break;
	default:
		break;
	}

	return NULL;
}

static const struct device *get_rw_cv_device(enum rw_cv_pos pos)
{
	const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	const struct device *i2c2 = DEVICE_DT_GET(DT_NODELABEL(i2c2));
	const struct device *i2c3 = DEVICE_DT_GET(DT_NODELABEL(i2c3));

	switch (pos) {
	case ADCS_VDD_12V_DRVX_SHUNT:
	case ADCS_VDD_12V_DRVX_BUS:
		if (device_is_ready(i2c1)) {
			return i2c1;
		}
		break;
	case ADCS_VDD_12V_DRVY_SHUNT:
	case ADCS_VDD_12V_DRVY_BUS:
		if (device_is_ready(i2c2)) {
			return i2c2;
		}
		break;
	case ADCS_VDD_12V_DRVZ_SHUNT:
	case ADCS_VDD_12V_DRVZ_BUS:
		if (device_is_ready(i2c3)) {
			return i2c3;
		}
		break;
	default:
		break;
	}

	return NULL;
}

static int get_adcs_register_addr(enum adcs_cv_pos pos, uint16_t *addr)
{
	int ret = 0;

	switch (pos) {
	case ADCS_VDD_3V3_IMU_SHUNT:
		*addr = CV_CH1_SHUNT_REG;
		break;
	case ADCS_VDD_3V3_IMU_BUS:
		*addr = CV_CH1_BUS_REG;
		break;
	case ADCS_VDD_3V3_GPS_SHUNT:
		*addr = CV_CH2_SHUNT_REG;
		break;
	case ADCS_VDD_3V3_GPS_BUS:
		*addr = CV_CH2_BUS_REG;
		break;
	case ADCS_VDD_3V3_DRV_SHUNT:
		*addr = CV_CH3_SHUNT_REG;
		break;
	case ADCS_VDD_3V3_DRV_BUS:
		*addr = CV_CH3_BUS_REG;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static int get_rw_register_addr(enum adcs_cv_pos pos, uint16_t *addr)
{
	int ret = 0;

	switch (pos) {
	case ADCS_VDD_12V_DRVX_SHUNT:
	case ADCS_VDD_12V_DRVY_SHUNT:
	case ADCS_VDD_12V_DRVZ_SHUNT:
		*addr = CV_RW_SHUNT_REG;
		break;
	case ADCS_VDD_12V_DRVX_BUS:
	case ADCS_VDD_12V_DRVY_BUS:
	case ADCS_VDD_12V_DRVZ_BUS:
		*addr = CV_RW_BUS_REG;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static int32_t convert_cv_shunt(int16_t raw)
{
	return ((raw >> 3) * 40);
}

static int32_t convert_cv_bus(int16_t raw)
{
	return ((raw >> 3) * 8);
}

static float convert_rw_cv_shunt(int16_t raw)
{
	return (raw * 5) / 1000;
}

static float convert_rw_cv_bus(uint16_t raw)
{
	return (raw * 3.125) / 1000;
}

static int convert_cv(enum adcs_cv_pos pos, int16_t raw, int32_t *cv)
{
	int ret = 0;

	switch (pos) {
	case ADCS_VDD_3V3_IMU_SHUNT:
	case ADCS_VDD_3V3_GPS_SHUNT:
	case ADCS_VDD_3V3_DRV_SHUNT:
		*cv = convert_cv_shunt(raw);
		break;
	case ADCS_VDD_3V3_IMU_BUS:
	case ADCS_VDD_3V3_GPS_BUS:
	case ADCS_VDD_3V3_DRV_BUS:
		*cv = convert_cv_bus(raw);
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static int convert_rw_cv(enum rw_cv_pos pos, uint16_t raw, float *cv)
{
	int ret = 0;

	switch (pos) {
	case ADCS_VDD_12V_DRVX_SHUNT:
	case ADCS_VDD_12V_DRVY_SHUNT:
	case ADCS_VDD_12V_DRVZ_SHUNT:
		*cv = convert_rw_cv_shunt((int16_t)raw);
		break;
	case ADCS_VDD_12V_DRVX_BUS:
	case ADCS_VDD_12V_DRVY_BUS:
	case ADCS_VDD_12V_DRVZ_BUS:
		*cv = convert_rw_cv_bus(raw);
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

int get_adcs_cv(enum adcs_cv_pos pos, uint32_t *cv)
{
	int ret;
	uint8_t data[2];
	uint16_t reg;
	int16_t raw;

	const struct device *dev = get_adcs_cv_device(pos);
	if (dev == NULL) {
		LOG_ERR("I2C device is not ready. (pos: %d)", pos);
		ret = -ENODEV;
		goto end;
	}

	ret = get_adcs_register_addr(pos, &reg);
	if (ret < 0) {
		goto end;
	}

	ret = i2c_burst_read(dev, CV_ADCS_ADDR, reg, data, ARRAY_SIZE(data));
	if (ret < 0) {
		LOG_ERR("Failed to i2c_burst_read for Current/Voltage Monitor (pos: %d) (%d)",
				 pos, ret);
		goto end;
	}
	raw = data[0] << 8 | data[1];

	ret = convert_cv(pos, raw, cv);
end:
	return ret;
}

int get_rw_cv(enum rw_cv_pos pos, float *cv)
{
	int ret;
	uint8_t data[2];
	uint16_t reg;
	uint16_t raw;

	const struct device *dev = get_rw_cv_device(pos);
	if (dev == NULL) {
		LOG_ERR("I2C device is not ready. (pos: %d)", pos);
		ret = -ENODEV;
		goto end;
	}

	ret = get_rw_register_addr(pos, &reg);
	if (ret < 0) {
		goto end;
	}

	ret = i2c_burst_read(dev, CV_RW_ADDR, reg, data, ARRAY_SIZE(data));
	if (ret < 0) {
		LOG_ERR("Failed to i2c_burst_read for RW Current/Voltage Monitor (pos: %d) (%d)",
				 pos, ret);
		goto end;
	}
	raw = (data[0] << 8) | data[1];

	ret = convert_rw_cv(pos, raw, cv);
end:
	return ret;
}

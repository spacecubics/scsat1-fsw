/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "cv_main.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cv, CONFIG_SC_LIB_CORE_MAIN_LOG_LEVEL);

#define CV_ADDR (0x42)

#define CV_CH1_SHUNT_REG (0x01)
#define CV_CH1_BUS_REG   (0x02)
#define CV_CH2_SHUNT_REG (0x03)
#define CV_CH2_BUS_REG   (0x04)
#define CV_CH3_SHUNT_REG (0x05)
#define CV_CH3_BUS_REG   (0x06)

static const struct device *get_cv_device(enum io_cv_pos pos)
{
	const struct device *i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

	switch (pos) {
	case IO_PDU_O4_3V3_SHUNT:
	case IO_PDU_O4_3V3_BUS:
	case IO_VDD_3V3_SYS_SHUNT:
	case IO_VDD_3V3_SYS_BUS:
	case IO_VDD_3V3_SHUNT:
	case IO_VDD_3V3_BUS:
		if (device_is_ready(i2c0)) {
			return i2c0;
		}
		break;
	default:
		break;
	}

	return NULL;
}

static int get_register_addr(enum io_cv_pos pos, uint16_t *addr)
{
	int ret = 0;

	switch (pos) {
	case IO_PDU_O4_3V3_SHUNT:
		*addr = CV_CH1_SHUNT_REG;
		break;
	case IO_PDU_O4_3V3_BUS:
		*addr = CV_CH1_BUS_REG;
		break;
	case IO_VDD_3V3_SYS_SHUNT:
		*addr = CV_CH2_SHUNT_REG;
		break;
	case IO_VDD_3V3_SYS_BUS:
		*addr = CV_CH2_BUS_REG;
		break;
	case IO_VDD_3V3_SHUNT:
		*addr = CV_CH3_SHUNT_REG;
		break;
	case IO_VDD_3V3_BUS:
		*addr = CV_CH3_BUS_REG;
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

static int convert_cv(enum io_cv_pos pos, uint16_t raw, int32_t *cv)
{
	int ret = 0;

	switch (pos) {
	case IO_PDU_O4_3V3_SHUNT:
		*cv = convert_cv_shunt(raw);
		break;
	case IO_PDU_O4_3V3_BUS:
		*cv = convert_cv_bus(raw);
		break;
	case IO_VDD_3V3_SYS_SHUNT:
		*cv = convert_cv_shunt(raw);
		break;
	case IO_VDD_3V3_SYS_BUS:
		*cv = convert_cv_bus(raw);
		break;
	case IO_VDD_3V3_SHUNT:
		*cv = convert_cv_shunt(raw);
		break;
	case IO_VDD_3V3_BUS:
		*cv = convert_cv_bus(raw);
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

int get_ioboard_cv(enum io_cv_pos pos, uint32_t *cv)
{
	int ret;
	uint8_t data[2];
	uint16_t reg;
	uint16_t raw;

	const struct device *dev = get_cv_device(pos);
	if (dev == NULL) {
		LOG_ERR("I2C device is not ready. (pos: %d)", pos);
		ret = -ENODEV;
		goto end;
	}

	ret = get_register_addr(pos, &reg);
	if (ret < 0) {
		goto end;
	}

	ret = i2c_burst_read(dev, CV_ADDR, reg, data, ARRAY_SIZE(data));
	if (ret < 0) {
		LOG_ERR("Failed to i2c_burst_read for Current/Voltage Monitor (pos: %d) (%d)", pos,
			ret);
		goto end;
	}
	raw = data[0] << 8 | data[1];

	ret = convert_cv(pos, raw, cv);
end:
	return ret;
}

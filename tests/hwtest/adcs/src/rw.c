/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include "rw.h"
#include "cv_adcs.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rw, CONFIG_SCSAT1_ADCS_LOG_LEVEL);

#define SC_RWM_BASE_ADDR    (0x50030000)
#define SC_RWM_MENR_OFFSET  (0x0000)
#define SC_RWM_MPSCR_OFFSET (0x0004)
#define SC_RWM_MCRX_OFFSET  (0x0010)
#define SC_RWM_MCRY_OFFSET  (0x0014)
#define SC_RWM_MCRZ_OFFSET  (0x0018)
#define SC_RWM_VER_OFFSET   (0x0F00)

#define SC_RWM_MENR_ZMEN BIT(2)
#define SC_RWM_MENR_YMEN BIT(1)
#define SC_RWM_MENR_XMEN BIT(0)

#define RW_PTM_SLAVE_ADDR (0x2E)
#define RW_ROTENTION_REG  (0x00)

static const uint8_t rw_pwr_en[] = {
	DRVX,
	DRVY,
	DRVZ,
};

static const uint8_t rw_monitor_en[] = {
	SC_RWM_MENR_XMEN,
	SC_RWM_MENR_YMEN,
	SC_RWM_MENR_ZMEN,
};

static const uint16_t rw_monitor_reg[] = {
	SC_RWM_MCRX_OFFSET,
	SC_RWM_MCRY_OFFSET,
	SC_RWM_MCRZ_OFFSET,
};

static const struct device *get_rw_device(enum rw_pos pos)
{
	const struct device *i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	const struct device *i2c2 = DEVICE_DT_GET(DT_NODELABEL(i2c2));
	const struct device *i2c3 = DEVICE_DT_GET(DT_NODELABEL(i2c3));

	switch (pos) {
	case RW_POS_X:
		if (device_is_ready(i2c1)) {
			return i2c1;
		}
		break;
	case RW_POS_Y:
		if (device_is_ready(i2c2)) {
			return i2c2;
		}
		break;
	case RW_POS_Z:
		if (device_is_ready(i2c3)) {
			return i2c3;
		}
		break;
	default:
		break;
	}

	return NULL;
}

static int set_potention_meter(const struct device *dev, uint8_t pot)
{
	int ret;

	ret = i2c_burst_write(dev, RW_PTM_SLAVE_ADDR, RW_ROTENTION_REG, &pot, 1);
	if (ret < 0) {
		LOG_ERR("Failed to i2c_burst_write for Potention Meter. (%d)", ret);
	}

	return ret;
}

int rw_start(enum rw_pos pos, uint8_t pot)
{
	int ret;
	const struct device *i2c = get_rw_device(pos);

	LOG_INF("Set Potention: 0x%02x on %s", pot, rw_pos_name[pos]);
	ret = set_potention_meter(i2c, pot);
	if (ret < 0) {
		goto end;
	}

	LOG_INF("Start the Reaction Wheel measurement (%s)", rw_pos_name[pos]);
	rw_start_measurment(pos);

	LOG_INF("Power on the Motor Driver on %s", rw_pos_name[pos]);
	sc_adcs_motor_enable(rw_pwr_en[pos]);
end:
	return ret;
}

void rw_stop(enum rw_pos pos)
{
	LOG_INF("Stop the Reaction Wheel measurement (%s)", rw_pos_name[pos]);
	rw_stop_measurment(pos);

	LOG_INF("Power off the Motor Driver on %s", rw_pos_name[pos]);
	sc_adcs_motor_disable(rw_pwr_en[pos]);
}

int rw_change_speed(enum rw_pos pos, uint16_t pot)
{
	const struct device *i2c = get_rw_device(pos);

	return set_potention_meter(i2c, pot);
}

void rw_start_measurment(enum rw_pos pos)
{
	sys_set_bits(SC_RWM_BASE_ADDR + SC_RWM_MENR_OFFSET, rw_monitor_en[pos]);
}

void rw_stop_measurment(enum rw_pos pos)
{
	sys_clear_bits(SC_RWM_BASE_ADDR + SC_RWM_MENR_OFFSET, rw_monitor_en[pos]);
}

int32_t rw_get_count(enum rw_pos pos)
{
	return (int32_t)sys_read32(SC_RWM_BASE_ADDR + rw_monitor_reg[pos]);
}

void rw_print_cv(enum rw_pos pos)
{
	float cv;

	switch (pos) {
	case RW_POS_X:
		get_rw_cv(ADCS_VDD_12V_DRVX_SHUNT, &cv);
		LOG_INF("%s Shunt: %.4f [mv]", rw_pos_name[pos], (double)cv);
		get_rw_cv(ADCS_VDD_12V_DRVX_BUS, &cv);
		LOG_INF("%s X Bus %.4f [v]", rw_pos_name[pos], (double)cv);
		break;
	case RW_POS_Y:
		get_rw_cv(ADCS_VDD_12V_DRVY_SHUNT, &cv);
		LOG_INF("%s Shunt: %.4f [mv]", rw_pos_name[pos], (double)cv);
		get_rw_cv(ADCS_VDD_12V_DRVY_BUS, &cv);
		LOG_INF("%s Bus %.4f [v]", rw_pos_name[pos], (double)cv);
		break;
	case RW_POS_Z:
		get_rw_cv(ADCS_VDD_12V_DRVZ_SHUNT, &cv);
		LOG_INF("%s Shunt: %.4f [mv]", rw_pos_name[pos], (double)cv);
		get_rw_cv(ADCS_VDD_12V_DRVZ_BUS, &cv);
		LOG_INF("%s Bus %.4f [v]", rw_pos_name[pos], (double)cv);
		break;
	default:
		break;
	}
}

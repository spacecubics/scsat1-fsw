/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "imu.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu_test);

static void imu_print_imu_data(struct imu_data *data)
{
	LOG_INF("ID  : 0x%02x", data->id);
	LOG_INF("TIME: 0x%08x", data->timestamp);
	LOG_INF("TEMP: 0x%08x", data->temp);
	LOG_INF("GX  : 0x%08x", data->gyro.x);
	LOG_INF("GY  : 0x%08x", data->gyro.y);
	LOG_INF("GZ  : 0x%08x", data->gyro.z);
	LOG_INF("AX  : 0x%08x", data->acc.x);
	LOG_INF("AY  : 0x%08x", data->acc.y);
	LOG_INF("AZ  : 0x%08x", data->acc.z);
}

int imu_test(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	struct imu_data data;

	ret = get_imu_data_ext(&data);
	if (ret < 0) {
		all_ret = -1;
		(*err_cnt)++;
	}
	imu_print_imu_data(&data);

	return all_ret;
}

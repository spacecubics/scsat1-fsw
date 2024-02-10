/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include "gnss_test.h"

int gnss_test(struct gnss_test_result *gnss_ret, uint32_t *err_cnt, bool log)
{
	int ret;
	struct gnss_hwmon_data data;

	ret = get_gnss_hwmon_data(&data, log);
	if (ret < 0) {
		memset(gnss_ret, 0, sizeof(*gnss_ret));
		(*err_cnt)++;
	} else {
		gnss_ret->sequence = data.sequence;
		gnss_ret->idle_time = data.idle_time;
		memcpy(gnss_ret->time_status, data.time_status, sizeof(gnss_ret->time_status));
		gnss_ret->week = data.week;
		gnss_ret->seconds = data.seconds;
		gnss_ret->receiver_status = data.receiver_status;
		gnss_ret->reserved = data.reserved;
		gnss_ret->receiver_version = data.receiver_version;
		gnss_ret->temp = data.temp;
		gnss_ret->voltage_3v3 = data.voltage_3v3;
		gnss_ret->voltage_antenna = data.voltage_antenna;
		gnss_ret->core_voltage_1v2 = data.core_voltage_1v2;
		gnss_ret->supply_voltage = data.supply_voltage;
		gnss_ret->voltage_1v8 = data.voltage_1v8;
	}
	gnss_ret->status = ret;

	return ret;
}

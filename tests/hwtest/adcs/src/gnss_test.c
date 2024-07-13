/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include "gnss_test.h"

int gnss_test(struct gnss_hwmon_result *hwmon_ret, struct gnss_bestpos_result *pos_ret,
	      uint32_t *err_cnt, bool log)
{
	int ret;
	struct gnss_hwmon_data hwmon_data;
	struct gnss_bestpos_data pos_data;

	ret = get_gnss_hwmon_data(&hwmon_data, log);
	if (ret < 0) {
		memset(hwmon_ret, 0, sizeof(*hwmon_ret));
		(*err_cnt)++;
	} else {
		hwmon_ret->sequence = hwmon_data.sequence;
		hwmon_ret->idle_time = hwmon_data.idle_time;
		memcpy(hwmon_ret->time_status, hwmon_data.time_status,
		       sizeof(hwmon_ret->time_status));
		hwmon_ret->week = hwmon_data.week;
		hwmon_ret->seconds = hwmon_data.seconds;
		hwmon_ret->receiver_status = hwmon_data.receiver_status;
		hwmon_ret->reserved = hwmon_data.reserved;
		hwmon_ret->receiver_version = hwmon_data.receiver_version;
		hwmon_ret->temp = hwmon_data.temp;
		hwmon_ret->voltage_3v3 = hwmon_data.voltage_3v3;
		hwmon_ret->voltage_antenna = hwmon_data.voltage_antenna;
		hwmon_ret->core_voltage_1v2 = hwmon_data.core_voltage_1v2;
		hwmon_ret->supply_voltage = hwmon_data.supply_voltage;
		hwmon_ret->voltage_1v8 = hwmon_data.voltage_1v8;
	}
	hwmon_ret->status = ret;

	ret = get_gnss_bestpos_data(&pos_data, log);
	if (ret < 0) {
		memset(pos_ret, 0, sizeof(*pos_ret));
		(*err_cnt)++;
	} else {
		pos_ret->sequence = pos_data.sequence;
		pos_ret->idle_time = pos_data.idle_time;
		memcpy(pos_ret->time_status, pos_data.time_status, sizeof(pos_ret->time_status));
		pos_ret->week = pos_data.week;
		pos_ret->seconds = pos_data.seconds;
		pos_ret->receiver_status = pos_data.receiver_status;
	}

	return ret;
}

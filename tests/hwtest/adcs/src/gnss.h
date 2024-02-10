/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>

struct gnss_hwmon_data {
	int32_t sequence;
	float idle_time;
	char time_status[10];
	uint32_t week;
	float seconds;
	uint32_t receiver_status;
	uint32_t reserved;
	uint32_t receiver_version;
	float temp;
	float voltage_3v3;
	float voltage_antenna;
	float core_voltage_1v2;
	float supply_voltage;
	float voltage_1v8;
};

int gnss_enable(void);
void gnss_disable(void);
int get_gnss_hwmon_data(struct gnss_hwmon_data *data, bool log);

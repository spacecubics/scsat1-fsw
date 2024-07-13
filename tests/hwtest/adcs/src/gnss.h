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

struct gnss_bestpos_data {
	int32_t sequence;
	float idle_time;
	char time_status[10];
	uint32_t week;
	float seconds;
	uint32_t receiver_status;
	uint32_t reserved;
	uint32_t receiver_version;
	char sol_stat[24];
	char pos_type[24];
	float lat_deg;
	float lon_deg;
	float hgt_deg;
	float undulation;
	char datum_id[16];
	float lat_m;
	float lon_m;
	float hgt_m;
	char stn_id[12];
	float diff_age;
	float sol_age;
	uint8_t svs;
	uint8_t soln_svs;
	uint8_t soln1_svs;
	uint8_t solnmulti_svs;
	uint8_t pos_reserved;
	uint8_t ext_sol_stat;
	uint8_t galileo_mask;
	uint8_t gps_mask;
};

int gnss_enable(void);
void gnss_disable(void);
int get_gnss_hwmon_data(struct gnss_hwmon_data *data, bool log);
int get_gnss_bestpos_data(struct gnss_bestpos_data *data, bool log);

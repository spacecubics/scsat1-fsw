/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "sc_dstrx3.h"

struct dstrx3_test_ret {
	int8_t status;
	uint8_t free_count;
	uint8_t wdt_count;
	uint8_t rssi;
	int16_t rcv_freq;
	int8_t temperature;
	uint8_t voltage;
	uint8_t tx_power;
	uint8_t carrier_lock: 1;
	uint8_t sub_carrier_lock: 1;
	uint8_t tx_power_set: 4;
	uint8_t bit_rate_set: 2;
	uint8_t program_no;
	uint8_t checksum;
} __attribute__((__packed__));

int dstrx3_test(struct dstrx3_test_ret *dstrx3_ret, uint32_t *err_cnt, bool log);

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

enum main_obc_tlm_type {
	SYSHK = 0,
	TEMP,
	CURRENT_VOLTAGE,
	CSP,
	SUN_SENSOR,
	MAGNET_METER,
	DSTRX3,
	ALL_TEST_RESULT,
	TLM_TYPE_NUM,
};

struct syshk_u8_data {
	int8_t status;
	uint8_t data;
} __attribute__((__packed__));

struct syshk_u16_data {
	int8_t status;
	uint16_t data;
} __attribute__((__packed__));

struct syshk_u32_data {
	int8_t status;
	uint32_t data;
} __attribute__((__packed__));

struct syshk_i8_data {
	int8_t status;
	int8_t data;
} __attribute__((__packed__));

struct syshk_i16_data {
	int8_t status;
	int16_t data;
} __attribute__((__packed__));

struct syshk_i32_data {
	int8_t status;
	int32_t data;
} __attribute__((__packed__));

struct syshk_float_data {
	int8_t status;
	float data;
} __attribute__((__packed__));

struct syshk_double_data {
	int8_t status;
	float data;
} __attribute__((__packed__));

int send_syshk(enum main_obc_tlm_type type, void *data, uint16_t size);

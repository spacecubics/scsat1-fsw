/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include "imu.h"

struct imu_test_result {
	int8_t status;
	struct imu_data data;
} __attribute__((__packed__));

int imu_test(struct imu_test_result *imu_ret, uint32_t *err_cnt, bool log);

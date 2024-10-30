/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

struct imu_gyro {
	uint32_t x;
	uint32_t y;
	uint32_t z;
};

struct imu_acc {
	uint32_t x;
	uint32_t y;
	uint32_t z;
};

struct imu_data {
	uint8_t id;
	uint32_t timestamp;
	uint32_t temp;
	struct imu_gyro gyro;
	struct imu_acc acc;
};

void imu_enable(void);
void imu_disable(void);
int get_imu_data_std(struct imu_data *data);
int get_imu_data_ext(struct imu_data *data);

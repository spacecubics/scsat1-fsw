/*
 * Copyright (c) 2023 Space Cubics, LLC.↲
 *↲
 * SPDX-License-Identifier: Apache-2.0↲
 */

#pragma once

#include <stdint.h>

#define DRV_PWR BIT(2)
#define GPS_PWR BIT(1)
#define IMU_PWR BIT(0)

void sc_adcs_power_enable(uint8_t target_bit);
void sc_adcs_power_disable(uint8_t target_bit);
void sc_adcs_imu_reset(void);
void sc_adcs_imu_reset_release(void);

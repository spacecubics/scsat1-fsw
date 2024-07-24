/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#define DRV_PWR BIT(2)
#define GPS_PWR BIT(1)
#define IMU_PWR BIT(0)

#define DRVZ_DIR_B BIT(10)
#define DRVY_DIR_B BIT(9)
#define DRVX_DIR_B BIT(8)
#define DRVZ       BIT(2)
#define DRVY       BIT(1)
#define DRVX       BIT(0)

void sc_adcs_power_enable(uint8_t target_bit);
void sc_adcs_power_disable(uint8_t target_bit);
void sc_adcs_imu_reset(void);
void sc_adcs_imu_reset_release(void);
void sc_adcs_motor_enable(uint8_t target_bit);
void sc_adcs_motor_disable(uint8_t target_bit);
void sc_adcs_power_cycle_req(void);

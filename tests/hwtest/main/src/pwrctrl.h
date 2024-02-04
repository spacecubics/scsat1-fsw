/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#define DRV2_PWR     BIT(5)
#define DRV1_PWR     BIT(4)
#define DSTRX_IO_PWR BIT(3)
#define PDU_O3_PWR   BIT(2)
#define PDU_O2_PWR   BIT(1)
#define PDU_O1_PWR   BIT(0)

void sc_main_power_enable(uint8_t target_bit);
void sc_main_power_disable(uint8_t target_bit);

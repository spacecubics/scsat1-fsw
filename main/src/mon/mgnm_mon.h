/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "mgnm.h"

struct mgnm_data_entry {
	int8_t status;
	struct magnet_field data;
};

struct mgnm_temp_entry {
	int8_t status;
	float data;
};

struct mgnm_msg {
	struct mgnm_data_entry magnet[MGNM_POS_NUM];
	struct mgnm_temp_entry temp[MGNM_POS_NUM];
};

void start_mgnm_monitor(void);

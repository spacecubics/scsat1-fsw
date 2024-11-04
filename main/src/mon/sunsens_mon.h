/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "sunsens.h"

struct sunsens_data_entry {
	int8_t status;
	struct sunsens_data data;
};

struct sunsens_temp_entry {
	int8_t status;
	float data;
};

struct sunsens_msg {
	struct sunsens_data_entry sun[SUNSENS_POS_NUM];
	struct sunsens_temp_entry temp[SUNSENS_POS_NUM];
};

void start_sunsens_monitor(void);

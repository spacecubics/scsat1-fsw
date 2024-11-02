/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "temp_adcs.h"
#include "sc_fpgamon.h"

struct temp_entry {
	int8_t status;
	float temp;
};

struct temp_msg {
	struct temp_entry obc[OBC_TEMP_POS_NUM];
	struct temp_entry xadc;
	struct temp_entry adcs[ADCS_TEMP_POS_NUM];
};

void start_temp_monitor(void);

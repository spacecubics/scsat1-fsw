/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include "pwrctrl.h"

#define RW_MAX_POTENTION  (0x7F)
#define RW_HALF_POTENTION (0x40)

enum rw_pos {
	RW_POS_X = 0,
	RW_POS_Y,
	RW_POS_Z,
};

static const char rw_pos_name[][5] = {
	"RW X",
	"RW Y",
	"RW Z",
};

int rw_start(enum rw_pos pos);
void rw_stop(enum rw_pos pos);
int rw_change_speed(enum rw_pos pos, uint16_t pot);
void rw_start_measurment(enum rw_pos pos);
void rw_stop_measurment(enum rw_pos pos);
int32_t rw_get_count(enum rw_pos pos);
void rw_print_cv(enum rw_pos pos);

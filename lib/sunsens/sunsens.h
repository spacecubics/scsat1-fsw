/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

enum sunsens_pos {
	SUNSENS_POS_Y_PLUS = 0,
	SUNSENS_POS_Y_MINUS,
	SUNSENS_POS_NUM,
};

struct sunsens_data {
	uint16_t a;
	uint16_t b;
	uint16_t c;
	uint16_t d;
};

int get_sunsens_temp(enum sunsens_pos pos, float *temp);
int get_sunsens_data(enum sunsens_pos pos, struct sunsens_data *sun_data);
int set_sunsens_addr(enum sunsens_pos pos, uint16_t old_addr, uint16_t new_addr);
int save_sunsens_addr(enum sunsens_pos pos);

/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

enum io_temp_pos {
	IO_TEMP_POS_ONBOARD_1 = 0,
	IO_TEMP_POS_ONBOARD_2,
	IO_TEMP_POS_X_PLUS,
	IO_TEMP_POS_X_MINUS,
	IO_TEMP_POS_Y_PLUS,
	IO_TEMP_POS_Y_MINUS,
	IO_TEMP_POS_NUM,
};

int get_ioboard_temp(enum io_temp_pos pos, float *temp);

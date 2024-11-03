/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

enum mgnm_pos {
	MGNM_POS_X_PLUS = 0,
	MGNM_POS_X_MINUS,
	MGNM_POS_NUM,
};

struct magnet_field {
	uint16_t x_out;
	uint16_t y_out;
	uint16_t z_out;
};

int start_mgnm_temp_measurement(enum mgnm_pos pos);
int start_mgnm_magnet_measurement(enum mgnm_pos pos);
int get_mgnm_temp(enum mgnm_pos pos, float *temp);
int get_mgnm_magnet(enum mgnm_pos pos, struct magnet_field *magnet);
int print_mgnm_field(void);

/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

enum adcs_temp_pos {
	ADCS_TEMP_POS_ONBOARD_1 = 0,
	ADCS_TEMP_POS_ONBOARD_2,
	ADCS_TEMP_POS_RW,
	ADCS_TEMP_POS_NUM,
};

int get_adcs_temp(enum adcs_temp_pos pos, float *temp);

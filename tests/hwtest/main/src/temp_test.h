/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "syshk.h"

#define TEMP_OBC_POS_NUM     (3U)
#define TEMP_IOBOARD_POS_NUM (6U)

struct main_temp_test_result {
	struct syshk_float_data obc_temp[TEMP_OBC_POS_NUM];
	struct syshk_float_data xadc_temp;
	struct syshk_float_data ioboard_temp[TEMP_IOBOARD_POS_NUM];
};

int temp_test(struct main_temp_test_result *temp_ret, uint32_t *err_cnt, bool log);

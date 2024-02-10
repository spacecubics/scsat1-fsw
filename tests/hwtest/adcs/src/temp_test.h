/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "syshk.h"

#define TEMP_OBC_POS_NUM  (3U)
#define TEMP_ADCS_POS_NUM (3U)

struct adcs_temp_test_result {
	struct syshk_float_data obc_temp[TEMP_OBC_POS_NUM];
	struct syshk_float_data xadc_temp;
	struct syshk_float_data adcs_temp[TEMP_ADCS_POS_NUM];
};

int temp_test(struct adcs_temp_test_result *temp_ret, uint32_t *err_cnt, bool log);

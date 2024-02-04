/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "syshk.h"

#define SUN_POS_NUM  (2U)
#define SUN_DATA_NUM (4U)

struct sunsens_test_ret {
	struct syshk_float_data temp[SUN_POS_NUM];
	struct syshk_u16_data sun[SUN_POS_NUM][SUN_DATA_NUM];
};

int sunsens_test(struct sunsens_test_ret *sunsens_ret, uint32_t *err_cnt, bool log);

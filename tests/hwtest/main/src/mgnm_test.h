/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "syshk.h"

#define MGNM_POS_NUM  (2U)
#define MGNM_DATA_NUM (3U)

struct mgnm_test_ret {
	struct syshk_float_data temp[MGNM_POS_NUM];
	struct syshk_u32_data out[MGNM_POS_NUM][MGNM_DATA_NUM];
};

int mgnm_test(struct mgnm_test_ret *mgnm_ret, uint32_t *err_cnt, bool log);

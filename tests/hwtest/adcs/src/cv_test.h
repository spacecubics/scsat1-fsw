/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "syshk.h"

#define CV_OBC_POS_NUM  (12U)
#define CV_XADC_POS_NUM (3U)
#define CV_ADCS_POS_NUM (6U)
#define CV_RW_POS_NUM   (6U)

struct adcs_cv_test_result {
	struct syshk_u32_data obc_cv[CV_OBC_POS_NUM];
	struct syshk_float_data xadc_cv[CV_XADC_POS_NUM];
	struct syshk_u32_data adcs_cv[CV_ADCS_POS_NUM];
	struct syshk_u32_data rw_cv[CV_RW_POS_NUM];
};

int cv_test(struct adcs_cv_test_result *cv_ret, uint32_t *err_cnt, bool log);

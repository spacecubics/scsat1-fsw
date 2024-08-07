/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "syshk.h"

#define CSP_TARGET_NUM (5U)

struct csp_test_result {
	struct syshk_i32_data ping[CSP_TARGET_NUM];
	struct syshk_u32_data uptime[CSP_TARGET_NUM];
	struct syshk_float_data temp_zero;
	struct syshk_u16_data fcount_zero;
};

int csp_test_init(void);
int csp_test(struct csp_test_result *test_ret, uint32_t *err_cnt, bool log);

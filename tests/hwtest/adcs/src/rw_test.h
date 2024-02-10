/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "rw.h"

struct rw_count_data {
	int32_t x_count;
	int32_t y_count;
	int32_t z_count;
};

int rw_test(uint32_t *err_cnt);
void rw_get_counts(struct rw_count_data *data);

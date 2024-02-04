/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#define LOOP_STOP_EVENT (1U)

int loop_test(int32_t loop_count, uint32_t *err_cnt);

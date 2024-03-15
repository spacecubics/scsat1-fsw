/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr/logging/log.h>

#define LOOP_STOP_EVENT (1U)
#define LOG_ENABLE      (true)
#define LOG_DISABLE     (false)

enum hwtest_mode {
	MAIN_ONLY_WITHOUT_DSTRX = 0,
	MAIN_ONLY,
	MAIN_ADCS_ONLY,
	FULL,
	TEST_MODE_NUM,
};

#define HWTEST_LOG_INF(log, format, ...)                                                           \
	if (log) {                                                                                 \
		LOG_INF(format, ##__VA_ARGS__);                                                    \
	}
#define HWTEST_LOG_DBG(log, format, ...)                                                           \
	if (log) {                                                                                 \
		LOG_DBG(format, ##__VA_ARGS__);                                                    \
	}
#define HWTEST_LOG_ERR(log, format, ...)                                                           \
	if (log) {                                                                                 \
		LOG_ERR(format, ##__VA_ARGS__);                                                    \
	}

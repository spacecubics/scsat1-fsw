/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "cv_main.h"
#include "sc_fpgamon.h"

struct cv_int32_entry {
	int8_t status;
	int32_t cv;
};

struct cv_float_entry {
	int8_t status;
	float cv;
};

struct cv_msg {
	struct cv_int32_entry obc[OBC_CV_POS_NUM];
	struct cv_float_entry xadc[OBC_XADC_CV_POS_NUM];
	struct cv_int32_entry io[IO_CV_POS_NUM];
};

void start_cv_monitor(void);

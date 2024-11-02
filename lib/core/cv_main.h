/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

enum io_cv_pos {
	IO_PDU_O4_3V3_SHUNT = 0,
	IO_PDU_O4_3V3_BUS,
	IO_VDD_3V3_SYS_SHUNT,
	IO_VDD_3V3_SYS_BUS,
	IO_VDD_3V3_SHUNT,
	IO_VDD_3V3_BUS,
	IO_CV_POS_NUM,
};

int get_ioboard_cv(enum io_cv_pos pos, int32_t *cv);

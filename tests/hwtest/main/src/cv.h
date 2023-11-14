/*
 * Copyright (c) 2023 Space Cubics, LLC.↲
 *↲
 * SPDX-License-Identifier: Apache-2.0↲
 */

#pragma once

#include <stdint.h>

enum io_cv_pos {
	IO_PDU_O4_3V3_SHUNT,
	IO_PDU_O4_3V3_BUS,
	IO_VDD_3V3_SYS_SHUNT,
	IO_VDD_3V3_SYS_BUS,
	IO_VDD_3V3_SHUNT,
	IO_VDD_3V3_BUS,
};

int get_ioboard_cv(enum io_cv_pos pos, uint32_t *cv);

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

enum obc_temp_pos {
	OBC_TEMP_1,
	OBC_TEMP_2,
	OBC_TEMP_3,
};

enum obc_cv_pos {
	OBC_1V0_SHUNT,
	OBC_1V0_BUS,
	OBC_1V8_SHUNT,
	OBC_1V8_BUS,
	OBC_3V3_SHUNT,
	OBC_3V3_BUS,
	OBC_3V3_SYSA_SHUNT,
	OBC_3V3_SYSA_BUS,
	OBC_3V3_SYSB_SHUNT,
	OBC_3V3_SYSB_BUS,
	OBC_3V3_IO_SHUNT,
	OBC_3V3_IO_BUS,
};

enum xadc_cv_pos {
	OBC_XADC_VCCINT,
	OBC_XADC_VCCAUX,
	OBC_XADC_VCCBRAM,
};

void sc_kick_wdt_timer(void);
int sc_bhm_enable(void);
int sc_bhm_disable(void);
int sc_bhm_get_obc_temp(enum obc_temp_pos pos, float *temp);
int sc_bhm_get_xadc_temp(float *temp);
int sc_bhm_get_obc_cv(enum obc_cv_pos pos, int32_t *cv);
int sc_bhm_get_xadc_cv(enum xadc_cv_pos pos, float *cv);

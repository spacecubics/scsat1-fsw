/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

enum adcs_cv_pos {
	ADCS_VDD_3V3_IMU_SHUNT = 0,
	ADCS_VDD_3V3_IMU_BUS,
	ADCS_VDD_3V3_GPS_SHUNT,
	ADCS_VDD_3V3_GPS_BUS,
	ADCS_VDD_3V3_DRV_SHUNT,
	ADCS_VDD_3V3_DRV_BUS,
	ADCS_CV_POS_NUM,
};

enum rw_cv_pos {
	ADCS_VDD_12V_DRVX_SHUNT = 0,
	ADCS_VDD_12V_DRVX_BUS,
	ADCS_VDD_12V_DRVY_SHUNT,
	ADCS_VDD_12V_DRVY_BUS,
	ADCS_VDD_12V_DRVZ_SHUNT,
	ADCS_VDD_12V_DRVZ_BUS,
	ADCS_RW_CV_POS_NUM,
};

int get_adcs_cv(enum adcs_cv_pos pos, int32_t *cv);
int get_rw_cv(enum rw_cv_pos pos, float *cv);

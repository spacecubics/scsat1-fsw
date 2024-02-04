/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "cv.h"
#include "sysmon.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cv_test);

static int cv_obc_test(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	uint32_t cv;
	enum obc_cv_pos obc_pos_list[] = {
		OBC_1V0_SHUNT,      OBC_1V0_BUS,      OBC_1V8_SHUNT,      OBC_1V8_BUS,
		OBC_3V3_SHUNT,      OBC_3V3_BUS,      OBC_3V3_SYSA_SHUNT, OBC_3V3_SYSA_BUS,
		OBC_3V3_SYSB_SHUNT, OBC_3V3_SYSB_BUS, OBC_3V3_IO_SHUNT,   OBC_3V3_IO_BUS,
	};
	const char obc_pos_name[][32] = {
		"OBC_1V0 Shunt",      "OBC_1V0 Bus",      "OBC_1V8 Shunt",      "OBC_1V8 Bus",
		"OBC_3V3 Shunt",      "OBC_3V3 Bus",      "OBC_3V3_SYSA Shunt", "OBC_3V3_SYSA Bus",
		"OBC_3V3_SYSB Shunt", "OBC_3V3_SYSB Bus", "OBC_3V3_IO Shunt",   "OBC_3V3_IO Bus",
	};
	const char obc_unit_name[][5] = {
		"[uv]", "[mv]", "[uv]", "[mv]", "[uv]", "[mv]",
		"[uv]", "[mv]", "[uv]", "[mv]", "[uv]", "[mv]",
	};

	for (int i = 0; i < ARRAY_SIZE(obc_pos_list); i++) {
		ret = sc_adcs_bhm_get_obc_cv(obc_pos_list[i], &cv);
		if (ret < 0) {
			LOG_ERR("%s: Failed", obc_pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
			continue;
		}
		LOG_INF("%s: %d %s", obc_pos_name[i], cv, obc_unit_name[i]);
	}

	return all_ret;
}

static int cv_xadc_test(uint32_t *err_cnt)
{
	int ret = 0;
	int all_ret = 0;
	float cv;
	enum xadc_cv_pos xadc_pos_list[] = {
		OBC_XADC_VCCINT,
		OBC_XADC_VCCAUX,
		OBC_XADC_VCCBRAM,
	};
	const char xadc_pos_name[][32] = {
		"OBC_XADC VCCINT",
		"OBC_XADC VCCAUX",
		"OBC_XADC VCCBRAM",
	};

	for (int i = 0; i < ARRAY_SIZE(xadc_pos_list); i++) {
		ret = sc_adcs_bhm_get_xadc_cv(xadc_pos_list[i], &cv);
		if (ret < 0) {
			LOG_ERR("%s: Failed", xadc_pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
			continue;
		}
		LOG_INF("%s: %.4f [v]", xadc_pos_name[i], (double)cv);
	}

	return all_ret;
}

static int cv_adcs_test(uint32_t *err_cnt)
{
	int ret = 0;
	int all_ret = 0;
	uint32_t cv;
	enum adcs_cv_pos adcs_pos_list[] = {
		ADCS_VDD_3V3_IMU_SHUNT, ADCS_VDD_3V3_IMU_BUS,   ADCS_VDD_3V3_GPS_SHUNT,
		ADCS_VDD_3V3_GPS_BUS,   ADCS_VDD_3V3_DRV_SHUNT, ADCS_VDD_3V3_DRV_BUS,
	};
	const char adcs_pos_name[][32] = {
		"ADCS_VDD_3V3_IMU Shunt", "ADCS_VDD_3V3_IMU Bus",   "ADCS_VDD_3V3_GPS Shunt",
		"ADCS_VDD_3V3_GPS Bus",   "ADCS_VDD_3V3_DRV Shunt", "ADCS_VDD_3V3_DRV Bus",
	};
	const char adcs_unit_name[][5] = {
		"[uv]", "[mv]", "[uv]", "[mv]", "[uv]", "[mv]",
	};

	/* ADCS Board */
	for (int i = 0; i < ARRAY_SIZE(adcs_pos_list); i++) {
		ret = get_adcs_cv(adcs_pos_list[i], &cv);
		if (ret < 0) {
			LOG_ERR("%s: Failed", adcs_pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
			continue;
		}
		LOG_INF("%s: %d %s", adcs_pos_name[i], cv, adcs_unit_name[i]);
	}

	return all_ret;
}

static int cv_rw_test(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	float cv;
	enum rw_cv_pos rw_pos_list[] = {
		ADCS_VDD_12V_DRVX_SHUNT, ADCS_VDD_12V_DRVX_BUS,   ADCS_VDD_12V_DRVY_SHUNT,
		ADCS_VDD_12V_DRVY_BUS,   ADCS_VDD_12V_DRVZ_SHUNT, ADCS_VDD_12V_DRVZ_BUS,
	};
	const char rw_pos_name[][32] = {
		"ADCS_VDD_12V_DRVX Shunt", "ADCS_VDD_12V_DRVX Bus",   "ADCS_VDD_12V_DRVY Shunt",
		"ADCS_VDD_12V_DRVY Bus",   "ADCS_VDD_12V_DRVZ Shunt", "ADCS_VDD_12V_DRVZ Bus",
	};
	const char rw_unit_name[][5] = {
		"[mv]", "[v]", "[mv]", "[v]", "[mv]", "[v]",
	};

	for (int i = 0; i < ARRAY_SIZE(rw_pos_list); i++) {
		ret = get_rw_cv(rw_pos_list[i], &cv);
		if (ret < 0) {
			LOG_ERR("%s: Failed", rw_pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
			continue;
		}
		LOG_INF("%s: %.4f %s", rw_pos_name[i], (double)cv, rw_unit_name[i]);
	}

	return ret;
}

int cv_test(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	ret = cv_obc_test(err_cnt);
	if (ret < 0) {
		all_ret--;
	}

	ret = cv_xadc_test(err_cnt);
	if (ret < 0) {
		all_ret--;
	}

	ret = cv_adcs_test(err_cnt);
	if (ret < 0) {
		all_ret--;
	}

	ret = cv_rw_test(err_cnt);
	if (ret < 0) {
		all_ret--;
	}

	return all_ret;
}

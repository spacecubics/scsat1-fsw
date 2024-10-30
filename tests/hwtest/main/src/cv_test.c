/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "common.h"
#include "cv_main.h"
#include "sc_fpgamon.h"
#include "cv_test.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cv_test, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

#define CV_INVALID_FLOAT (0.0f)
#define CV_INVALID_UINT  (0U)

static int cv_obc_test(struct main_cv_test_result *cv_ret, uint32_t *err_cnt, bool log)
{
	int ret;
	int all_ret = 0;
	uint32_t cv;
	enum obc_cv_pos obc_pos_list[] = {
		OBC_1V0_SHUNT,      OBC_1V0_BUS,      OBC_1V8_SHUNT,      OBC_1V8_BUS,
		OBC_3V3_SHUNT,      OBC_3V3_BUS,      OBC_3V3_SYSA_SHUNT, OBC_3V3_SYSA_BUS,
		OBC_3V3_SYSB_SHUNT, OBC_3V3_SYSB_BUS, OBC_3V3_IO_SHUNT,   OBC_3V3_IO_BUS,
	};
	const char obc_pos_name[][20] = {
		"OBC_1V0 Shunt",      "OBC_1V0 Bus",      "OBC_1V8 Shunt",      "OBC_1V8 Bus",
		"OBC_3V3 Shunt",      "OBC_3V3 Bus",      "OBC_3V3_SYSA Shunt", "OBC_3V3_SYSA Bus",
		"OBC_3V3_SYSB Shunt", "OBC_3V3_SYSB Bus", "OBC_3V3_IO Shunt",   "OBC_3V3_IO Bus",
	};
	const char obc_unit_name[][5] = {
		"[uv]", "[mv]", "[uv]", "[mv]", "[uv]", "[mv]",
		"[uv]", "[mv]", "[uv]", "[mv]", "[uv]", "[mv]",
	};

	for (int i = 0; i < ARRAY_SIZE(obc_pos_list); i++) {
		ret = sc_bhm_get_obc_cv(obc_pos_list[i], &cv);
		if (ret < 0) {
			cv_ret->obc_cv[i].data = CV_INVALID_UINT;
			HWTEST_LOG_ERR(log, "%s: Failed", obc_pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
		} else {
			cv_ret->obc_cv[i].data = cv;
			HWTEST_LOG_INF(log, "%s: %d %s", obc_pos_name[i], cv, obc_unit_name[i]);
		}
		cv_ret->obc_cv[i].status = ret;
	}

	return all_ret;
}

static int cv_xadc_test(struct main_cv_test_result *cv_ret, uint32_t *err_cnt, bool log)
{
	int ret;
	int all_ret = 0;
	float cv;
	enum xadc_cv_pos xadc_pos_list[] = {
		OBC_XADC_VCCINT,
		OBC_XADC_VCCAUX,
		OBC_XADC_VCCBRAM,
	};
	const char xadc_pos_name[][18] = {
		"OBC_XADC VCCINT",
		"OBC_XADC VCCAUX",
		"OBC_XADC VCCBRAM",
	};

	for (int i = 0; i < ARRAY_SIZE(xadc_pos_list); i++) {
		ret = sc_bhm_get_xadc_cv(xadc_pos_list[i], &cv);
		if (ret < 0) {
			cv_ret->xadc_cv[i].data = CV_INVALID_FLOAT;
			HWTEST_LOG_ERR(log, "%s: Failed", xadc_pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
		} else {
			cv_ret->xadc_cv[i].data = cv;
			HWTEST_LOG_INF(log, "%s: %.4f [v]", xadc_pos_name[i], (double)cv);
		}
		cv_ret->xadc_cv[i].status = ret;
	}

	return all_ret;
}

static int cv_ioboard_test(struct main_cv_test_result *cv_ret, uint32_t *err_cnt, bool log)
{
	int ret;
	int all_ret = 0;
	uint32_t cv;
	enum io_cv_pos io_pos_list[] = {
		IO_PDU_O4_3V3_SHUNT, IO_PDU_O4_3V3_BUS, IO_VDD_3V3_SYS_SHUNT,
		IO_VDD_3V3_SYS_BUS,  IO_VDD_3V3_SHUNT,  IO_VDD_3V3_BUS,
	};
	const char io_pos_name[][22] = {
		"IO_PDU_04_3V3 Shunt", "IO_PDU_04_3V3 Bus", "IO_VDD_3V3_SYS Shunt",
		"IO_VDD_3V3_SYS Bus",  "IO_VDD_3V3 Shunt",  "IO_VDD_3V3 Bus",
	};
	const char io_unit_name[][5] = {
		"[uv]", "[mv]", "[uv]", "[mv]", "[uv]", "[mv]",
	};

	for (int i = 0; i < ARRAY_SIZE(io_pos_list); i++) {
		ret = get_ioboard_cv(io_pos_list[i], &cv);
		if (ret < 0) {
			cv_ret->ioboard_cv[i].data = CV_INVALID_UINT;
			HWTEST_LOG_ERR(log, "%s: Failed", io_pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
		} else {
			cv_ret->ioboard_cv[i].data = cv;
			HWTEST_LOG_INF(log, "%s: %d %s", io_pos_name[i], cv, io_unit_name[i]);
		}
		cv_ret->ioboard_cv[i].status = ret;
	}

	return all_ret;
}

int cv_test(struct main_cv_test_result *cv_ret, uint32_t *err_cnt, bool log)
{
	int ret;
	int all_ret = 0;

	ret = cv_obc_test(cv_ret, err_cnt, log);
	if (ret < 0) {
		all_ret = -1;
	}

	ret = cv_xadc_test(cv_ret, err_cnt, log);
	if (ret < 0) {
		all_ret = -1;
	}

	ret = cv_ioboard_test(cv_ret, err_cnt, log);
	if (ret < 0) {
		all_ret = -1;
	}

	return all_ret;
}

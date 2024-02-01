/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include "temp.h"
#include "sysmon.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(temp_test);

static int temp_obc_test(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	float temp;
	enum obc_temp_pos obc_pos_list[] = {
		OBC_TEMP_1,
		OBC_TEMP_2,
		OBC_TEMP_3,
	};
	const char obc_pos_name[][6] = {
		"OBC 1",
		"OBC 2",
		"OBC 3",
	};

	for (int i=0; i<ARRAY_SIZE(obc_pos_list); i++) {
		ret = sc_adcs_bhm_get_obc_temp(obc_pos_list[i], &temp);
		if (ret < 0) {
			LOG_ERR("%s Temperature: Failed", obc_pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
			continue;
		}
		LOG_INF("%s Temperature: %.4f [deg]", obc_pos_name[i], (double)temp);
	}

	return all_ret;
}

static int temp_xadc_test(uint32_t *err_cnt)
{
	int ret;
	float temp;

	ret = sc_adcs_bhm_get_xadc_temp(&temp);
	if (ret < 0) {
		LOG_ERR("OBC XADC Temperature: Failed");
		(*err_cnt)++;
	} else {
		LOG_INF("OBC XADC Temperature: %.4f [deg]", (double)temp);
	}

	return ret;
}

static int temp_adcs_test(uint32_t *err_cnt)
{
	int ret = 0;
	int all_ret = 0;
	float temp;
	const char adcs_pos_name[][14] = {
		"ADCS Board 1",
		"ADCS Board 2",
		"ADCS RW",
	};
	enum adcs_temp_pos adcs_pos_list[] = {
		ADCS_TEMP_POS_ONBOARD_1,
		ADCS_TEMP_POS_ONBOARD_2,
		ADCS_TEMP_POS_RW,
	};

	for (int i=0; i<ARRAY_SIZE(adcs_pos_list); i++) {
		ret = get_adcs_temp(adcs_pos_list[i], &temp);
		if (ret < 0) {
			LOG_ERR("%s Temperature: Failed", adcs_pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
			continue;
		}
		LOG_INF("%s Temperature: %.4f [deg]", adcs_pos_name[i], (double)temp);
	}

	return all_ret;
}

int temp_test(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	ret = temp_obc_test(err_cnt);
	if (ret < 0) {
		all_ret = -1;
	}

	ret = temp_xadc_test(err_cnt);
	if (ret < 0) {
		all_ret = -1;
	}

	ret = temp_adcs_test(err_cnt);
	if (ret < 0) {
		all_ret = -1;
	}

	return all_ret;
}

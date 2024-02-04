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
	int i;
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

	for (i = 0; i < ARRAY_SIZE(obc_pos_list); i++) {
		ret = sc_main_bhm_get_obc_temp(obc_pos_list[i], &temp);
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

	ret = sc_main_bhm_get_xadc_temp(&temp);
	if (ret < 0) {
		LOG_ERR("OBC XADC Temperature: Failed");
		(*err_cnt)++;
	} else {
		LOG_INF("OBC XADC Temperature: %.4f [deg]", (double)temp);
	}

	return ret;
}

static int temp_ioboard_test(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	float temp;
	enum io_temp_pos io_pos_list[] = {
		IO_TEMP_POS_ONBOARD_1, IO_TEMP_POS_ONBOARD_2, IO_TEMP_POS_X_PLUS,
		IO_TEMP_POS_X_MINUS,   IO_TEMP_POS_Y_PLUS,    IO_TEMP_POS_Y_MINUS,
	};
	const char io_pos_name[][12] = {
		"I/O Board 1", "I/O Board 2", "X+", "X-", "Y+", "Y-",
	};

	for (int i = 0; i < ARRAY_SIZE(io_pos_list); i++) {
		ret = get_ioboard_temp(io_pos_list[i], &temp);
		if (ret < 0) {
			LOG_ERR("%s Temperature: Failed", io_pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
			continue;
		}
		LOG_INF("%s Temperature: %.4f [deg]", io_pos_name[i], (double)temp);
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

	ret = temp_ioboard_test(err_cnt);
	if (ret < 0) {
		all_ret = -1;
	}

	return all_ret;
}

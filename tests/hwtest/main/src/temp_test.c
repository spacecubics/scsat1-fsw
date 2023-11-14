/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include "temp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(temp_test);

int temp_test(uint32_t *err_cnt)
{
	int ret = 0;
	int all_ret = 0;
	float temp;
	enum io_temp_pos io_pos_list[] = {
		IO_TEMP_POS_ONBOARD_1,
		IO_TEMP_POS_ONBOARD_2,
		IO_TEMP_POS_X_PLUS,
		IO_TEMP_POS_X_MINUS,
		IO_TEMP_POS_Y_PLUS,
		IO_TEMP_POS_Y_MINUS,
	};
	const char io_pos_name[][12] = {
		"I/O Board 1",
		"I/O Board 2",
		"X+",
		"X-",
		"Y+",
		"Y-",
	};

	for (int i=0; i<ARRAY_SIZE(io_pos_list); i++) {
		ret = get_ioboard_temp(io_pos_list[i], &temp);
		if (ret < 0) {
			LOG_ERR("%s Temperature: Failed", io_pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
			continue;
		}
		LOG_INF("%s Temperature: %.4f [deg]", io_pos_name[i], temp);
	}

	return all_ret;
}

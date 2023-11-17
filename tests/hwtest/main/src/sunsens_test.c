/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "sunsens.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sunsens_test);

int sunsens_test(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	float temp;
	struct sunsens_data sun_data;
	enum sunsens_pos pos_list[] = {
		SUNSENS_POS_Y_PLUS,
		SUNSENS_POS_Y_MINUS,
	};
	const char pos_name[][15] = {
		"Sun Sensor Y+",
		"Sun Sensor Y-",
	};

	for (int i=0; i<ARRAY_SIZE(pos_list); i++) {
		ret = get_sunsens_temp(SUNSENS_POS_Y_MINUS, &temp);
		if (ret < 0) {
			LOG_ERR("%s Temtemperature: Failed", pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
			continue;
		}
		LOG_INF("%s Temtemperature: %f [deg]", pos_name[i], temp);

		ret = get_sunsens_data(SUNSENS_POS_Y_MINUS, &sun_data);
		if (ret < 0) {
			LOG_ERR("%s Temtemperature: Failed", pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
			continue;
		}
		LOG_INF("%s Sun Sensor: [A: 0x%04x, B: 0x%04x, C: 0x%04x, D: 0x%04x]",
				 pos_name[i],
				 sun_data.a,
				 sun_data.b,
				 sun_data.c,
				 sun_data.d);
	}

	return all_ret;
}

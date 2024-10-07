/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "common.h"
#include "sunsens.h"
#include "sunsens_test.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sunsens_test, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

#define SUNSENS_INVALID_TMEP (0.0f)
#define SUNSENS_INVALID_DATA (0U)

static void sunsens_fill_ret(struct sunsens_test_ret *sunsens_ret, uint8_t index, int ret,
			     struct sunsens_data *data)
{
	for (int i = 0; i < SUN_DATA_NUM; i++) {
		sunsens_ret->sun[index][i].status = ret;
		if (ret < 0) {
			sunsens_ret->sun[index][i].data = SUNSENS_INVALID_DATA;
		}
	}

	if (ret >= 0) {
		sunsens_ret->sun[index][0].data = data->a;
		sunsens_ret->sun[index][1].data = data->b;
		sunsens_ret->sun[index][2].data = data->c;
		sunsens_ret->sun[index][3].data = data->d;
	}
}

int sunsens_test(struct sunsens_test_ret *sunsens_ret, uint32_t *err_cnt, bool log)
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

	for (int i = 0; i < ARRAY_SIZE(pos_list); i++) {
		ret = get_sunsens_temp(pos_list[i], &temp);
		if (ret < 0) {
			sunsens_ret->temp[i].data = SUNSENS_INVALID_TMEP;
			HWTEST_LOG_ERR(log, "%s Temperature: Failed", pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
		} else {
			sunsens_ret->temp[i].data = temp;
			HWTEST_LOG_INF(log, "%s Temperature: %f [deg]", pos_name[i], (double)temp);
		}
		sunsens_ret->temp[i].status = ret;

		ret = get_sunsens_data(pos_list[i], &sun_data);
		sunsens_fill_ret(sunsens_ret, i, ret, &sun_data);
		if (ret < 0) {
			HWTEST_LOG_ERR(log, "%s Sun Sensor: Failed", pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
		} else {
			HWTEST_LOG_INF(
				log, "%s Sun Sensor: [A: 0x%04x, B: 0x%04x, C: 0x%04x, D: 0x%04x]",
				pos_name[i], sun_data.a, sun_data.b, sun_data.c, sun_data.d);
		}
	}

	return all_ret;
}

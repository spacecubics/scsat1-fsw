/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include "common.h"
#include "mgnm.h"
#include "mgnm_test.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mgnm_test, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

#define MGNM_INVALID_TEMP (0.0f)
#define MGNM_INVALID_DATA (0U)

static void mgnm_fill_ret(struct mgnm_test_ret *mgnm_ret, uint8_t index, int ret,
			  struct magnet_field *data)
{
	for (int i = 0; i < MGNM_DATA_NUM; i++) {
		mgnm_ret->out[index][i].status = ret;
		if (ret < 0) {
			mgnm_ret->out[index][i].data = MGNM_INVALID_DATA;
		}
	}

	if (ret >= 0) {
		mgnm_ret->out[index][0].data = data->x_out;
		mgnm_ret->out[index][1].data = data->y_out;
		mgnm_ret->out[index][2].data = data->z_out;
	}
}

int mgnm_test(struct mgnm_test_ret *mgnm_ret, uint32_t *err_cnt, bool log)
{
	int ret = 0;
	float temp;
	int i;
	const char pos_name[][3] = {"X+", "X-"};
	enum mgnm_pos pos_list[] = {MGNM_POS_X_PLUS, MGNM_POS_X_MINUS};
	struct magnet_field magnet;

	/* Temperature */
	for (i = 0; i < ARRAY_SIZE(pos_list); i++) {
		ret = start_mgnm_temp_measurement(pos_list[i]);
		if (ret < 0) {
			mgnm_ret->temp[i].status = ret;
			mgnm_ret->temp[i].data = MGNM_INVALID_TEMP;
			HWTEST_LOG_ERR(log, "Magnetometer %s Temperature: Failed", pos_name[i]);
			(*err_cnt)++;
			ret--;
			continue;
		}

		ret = get_mgnm_temp(pos_list[i], &temp);
		if (ret < 0) {
			mgnm_ret->temp[i].status = ret;
			mgnm_ret->temp[i].data = MGNM_INVALID_TEMP;
			HWTEST_LOG_ERR(log, "Magnetometer %s Temperature: Failed", pos_name[i]);
			(*err_cnt)++;
			ret--;
			continue;
		}

		mgnm_ret->temp[i].status = ret;
		mgnm_ret->temp[i].data = temp;
		HWTEST_LOG_INF(log, "Magnetometer %s Temperature: %.1f [deg]", pos_name[i],
			       (double)temp);
	}

	/* Magnet Field */
	for (i = 0; i < ARRAY_SIZE(pos_list); i++) {
		ret = start_mgnm_magnet_measurement(pos_list[i]);
		if (ret < 0) {
			mgnm_fill_ret(mgnm_ret, i, ret, &magnet);
			HWTEST_LOG_ERR(log, "Magnetometer %s X/Y/Z: Failed", pos_name[i]);
			(*err_cnt)++;
			ret--;
			continue;
		}

		ret = get_mgnm_magnet(pos_list[i], &magnet);
		mgnm_fill_ret(mgnm_ret, i, ret, &magnet);
		if (ret < 0) {
			HWTEST_LOG_ERR(log, "Magnetometer %s X/Y/Z: Failed", pos_name[i]);
			(*err_cnt)++;
			ret--;
			continue;
		}

		HWTEST_LOG_INF(log, "Magnetometer %s X: 0x%08x, Y: 0x%08x, Z: 0x%08x", pos_name[i],
			       magnet.x_out, magnet.y_out, magnet.z_out);
	}

	return ret;
}

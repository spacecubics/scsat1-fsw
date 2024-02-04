/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include "mgnm.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mgnm_test);

int mgnm_test(uint32_t *err_cnt)
{
	int ret = 0;
	float temp;
	int i;
	const char pos_name[][3] = {"X+", "X-"};
	enum mgnm_pos pos_list[] = {MGNM_POS_X_PLUS, MGNM_POS_X_MINUS};

	/* Temperature */
	for (i = 0; i < ARRAY_SIZE(pos_list); i++) {
		ret = start_mgnm_temp_measurement(pos_list[i]);
		if (ret < 0) {
			LOG_ERR("Magnetometer %s Temperature: Failed", pos_name[i]);
			(*err_cnt)++;
			ret--;
			continue;
		}

		ret = get_mgnm_temp(pos_list[i], &temp);
		if (ret < 0) {
			LOG_ERR("Magnetometer %s Temperature: Failed", pos_name[i]);
			(*err_cnt)++;
			ret--;
			continue;
		}

		LOG_INF("Magnetometer %s Temperature: %.1f [deg]", pos_name[i], (double)temp);
	}

	/* Magnet Field */
	struct magnet_field magnet;
	for (i = 0; i < ARRAY_SIZE(pos_list); i++) {
		ret = start_mgnm_magnet_measurement(pos_list[i]);
		if (ret < 0) {
			LOG_ERR("Magnetometer %s X/Y/Z: Failed", pos_name[i]);
			(*err_cnt)++;
			ret--;
			continue;
		}

		ret = get_mgnm_magnet(pos_list[i], &magnet);
		if (ret < 0) {
			LOG_ERR("Magnetometer %s X/Y/Z: Failed", pos_name[i]);
			(*err_cnt)++;
			ret--;
			continue;
		}

		LOG_INF("Magnetometer %s X: 0x%08x, Y: 0x%08x, Z: 0x%08x", pos_name[i],
			magnet.x_out, magnet.y_out, magnet.z_out);
	}

	return ret;
}

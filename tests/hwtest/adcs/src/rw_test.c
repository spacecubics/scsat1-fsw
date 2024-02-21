/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "rw_test.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rw_test);

static void print_rw_status(enum rw_pos pos, uint32_t sec)
{
	for (int j = 0; j < 5; j++) {
		k_sleep(K_SECONDS(1));
		rw_print_cv(pos);
		LOG_INF("%s count: %d", rw_pos_name[pos], rw_get_count(pos));
	}
}

int rw_test(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	enum rw_pos pos_list[] = {
		RW_POS_X,
		RW_POS_Y,
		RW_POS_Z,
	};

	for (int i = 0; i < ARRAY_SIZE(pos_list); i++) {
		ret = rw_start(pos_list[i], RW_HALF_POTENTION);
		if (ret < 0) {
			(*err_cnt)++;
			all_ret = -1;
			goto end;
		}

		print_rw_status(pos_list[i], 5);

		LOG_INF("Change Potention: 0x%02x on %s", 0x20,
			rw_pos_name[pos_list[i]]);
		ret = rw_change_speed(pos_list[i], 0x20);
		if (ret < 0) {
			(*err_cnt)++;
			all_ret = -1;
			goto stop;
		}

		print_rw_status(pos_list[i], 5);
	stop:
		rw_stop(pos_list[i]);
	end:
	}

	return all_ret;
}

void rw_get_counts(struct rw_count_data *data)
{
	data->x_count = rw_get_count(RW_POS_X);
	data->y_count = rw_get_count(RW_POS_Y);
	data->z_count = rw_get_count(RW_POS_Z);
}

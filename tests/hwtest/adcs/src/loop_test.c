/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include "loop_test.h"
#include "pwrctrl.h"
#include "temp_test.h"
#include "cv_test.h"
#include "imu_test.h"
#include "gnss_test.h"
#include "rw.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(loop_test);

struct k_event loop_event;

static void update_rw_idx(uint8_t *rw_idx)
{
	(*rw_idx)++;
	if (*rw_idx >= 3) {
		*rw_idx = 0;
	}
}

static int one_loop(enum rw_pos pos, uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	LOG_INF("===[RW Count Test Start (total err: %d)]===", *err_cnt);
	LOG_INF("%s count: %d", rw_pos_name[pos], rw_get_count(pos));

	LOG_INF("===[Temp Test Start (total err: %d)]===", *err_cnt);
	ret = temp_test(err_cnt);
	if (ret < 0) {
		all_ret = -1;
	}

	LOG_INF("===[CV Test Start (total err: %d)]===", *err_cnt);
	ret = cv_test(err_cnt);
	if (ret < 0) {
		all_ret = -1;
	}

	LOG_INF("===[IMU Test Start (total err: %d)]===", *err_cnt);
	ret = imu_test(err_cnt);
	if (ret < 0) {
		all_ret = -1;
	}

	LOG_INF("===[GNSS Test Start (total err: %d)]===", *err_cnt);
	ret = gnss_test(err_cnt);
	if (ret < 0) {
		all_ret = -1;
	}

	return all_ret;
}

static int verify_status(enum rw_pos pos, uint32_t sec, uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	for (int j=0; j<10; j++) {
		ret = one_loop(pos, err_cnt);
		if (ret < 0) {
			all_ret = -1;
		}
	}

	return all_ret;
}

static bool is_loop_stop(void)
{
	if (k_event_wait(&loop_event, LOOP_STOP_EVENT, false, K_NO_WAIT) != 0) {
		return true;
	}

	return false;
}

int loop_test(int32_t loop_count, uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	enum rw_pos pos_list[] = {
		RW_POS_X,
		RW_POS_Y,
		RW_POS_Z,
	};
	uint8_t rw_idx = 0;

	if (loop_count < 0) {
		loop_count = INT32_MAX;
	}

	for (int i=1; i<=loop_count; i++) {
		if (is_loop_stop()) {
			break;
		}

		LOG_INF("===[Loop Test %d Start (total err: %d)]===", i, *err_cnt);

		LOG_INF("===[CV Test Start (total err: %d)]===", *err_cnt);
		ret = cv_test(err_cnt);
		k_sleep(K_SECONDS(1));
		continue;

		LOG_INF("===[RW Start (total err: %d)]===", *err_cnt);
		ret = rw_start(pos_list[rw_idx]);
		if (ret < 0) {
			(*err_cnt)++;
			all_ret = -1;
		}

		ret = verify_status(pos_list[rw_idx], 5, err_cnt);
		if (ret < 0) {
			all_ret = -1;
		}

		LOG_INF("===[Change potention (total err: %d)]===", *err_cnt);
		ret = rw_change_speed(pos_list[rw_idx], RW_HALF_POTENTION);
		if (ret < 0) {
			(*err_cnt)++;
			all_ret = -1;
		}

		ret = verify_status(pos_list[rw_idx], 5, err_cnt);
		if (ret < 0) {
			all_ret = -1;
		}

		LOG_INF("===[RW Stop (total err: %d)]===", *err_cnt);
		rw_stop(pos_list[rw_idx]);

		update_rw_idx(&rw_idx);

		LOG_INF("===[Loop Test %d Finish (err_cnt: %d)]===",
				 i, *err_cnt);
	}

	return all_ret;
}

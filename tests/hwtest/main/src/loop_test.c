/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "common.h"
#include "loop_test.h"
#include "pwrctrl.h"
#include "temp_test.h"
#include "cv_test.h"
#include "csp_test.h"
#include "sunsens_test.h"
#include "mgnm_test.h"
#include "dstrx3_test.h"
#include "mtq.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(loop_test, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

struct k_event loop_event;
extern enum hwtest_mode test_mode;

static void update_mtq_idx(uint8_t *axes_idx, uint8_t *pol_idx)
{
	if (*pol_idx < 2) {
		(*pol_idx)++;
	} else {
		*pol_idx = 0;
		(*axes_idx)++;
	}

	if (*axes_idx >= 3) {
		*axes_idx = 0;
	}
}

static int one_loop(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	struct main_temp_test_result temp_ret;
	struct main_cv_test_result cv_ret;
	struct csp_test_result csp_ret;
	struct sunsens_test_ret sunsens_ret;
	struct mgnm_test_ret mgnm_ret;
	struct dstrx3_test_ret dstrx3_ret;

	LOG_INF("===[Temp Test Start (total err: %d)]===", *err_cnt);
	ret = temp_test(&temp_ret, err_cnt, LOG_ENABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	k_sleep(K_MSEC(100));

	LOG_INF("===[CV Test Start (total err: %d)]===", *err_cnt);
	ret = cv_test(&cv_ret, err_cnt, LOG_ENABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	k_sleep(K_MSEC(100));

	LOG_INF("===[CSP Test Start (total err: %d)]===", *err_cnt);
	ret = csp_test(&csp_ret, err_cnt, LOG_ENABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	k_sleep(K_MSEC(100));

	LOG_INF("===[Sun Sensor Test Start (total err: %d)]===", *err_cnt);
	ret = sunsens_test(&sunsens_ret, err_cnt, LOG_ENABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	k_sleep(K_MSEC(100));

	LOG_INF("===[Magnetometer Test Start (total err: %d)]===", *err_cnt);
	ret = mgnm_test(&mgnm_ret, err_cnt, LOG_ENABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	k_sleep(K_MSEC(100));

	if (test_mode < MAIN_ONLY) {
		goto end;
	}

	LOG_INF("===[DSTRX-3 Test Start (total err: %d)]===", *err_cnt);
	ret = dstrx3_test(&dstrx3_ret, err_cnt, LOG_ENABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	k_sleep(K_MSEC(100));

end:
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
	float duty = 1.0;
	enum mtq_axes axes_list[] = {
		MTQ_AXES_MAIN_X,
		MTQ_AXES_MAIN_Y,
		MTQ_AXES_MAIN_Z,
	};
	enum mtq_polarity pol_list[] = {
		MTQ_POL_PLUS,
		MTQ_POL_MINUS,
		MTQ_POL_NON,
	};
	uint8_t axes_idx = 0;
	uint8_t pol_idx = 0;

	if (loop_count < 0) {
		loop_count = INT32_MAX;
	}

	if (test_mode == FULL) {
		LOG_INF("===[Init photo directory on Zero)]===");
		csp_test_init();
		k_sleep(K_SECONDS(10));
	}

	for (int i = 1; i <= loop_count; i++) {
		if (is_loop_stop()) {
			break;
		}

		LOG_INF("===[Loop Test %d Start (total err: %d)]===", i, *err_cnt);

		LOG_INF("===[MTQ Start (total err: %d)]===", *err_cnt);
		ret = mtq_start(axes_list[axes_idx], pol_list[pol_idx], duty);
		if (ret < 0) {
			(*err_cnt)++;
			all_ret = -1;
		}

		for (int j = 0; j < 5; j++) {
			ret = one_loop(err_cnt);
			if (ret < 0) {
				all_ret = -1;
			}
		}

		LOG_INF("===[MTQ Stop (total err: %d)]===", *err_cnt);
		ret = mtq_stop(axes_list[axes_idx]);
		if (ret < 0) {
			(*err_cnt)++;
			all_ret = -1;
		}

		update_mtq_idx(&axes_idx, &pol_idx);

		LOG_INF("===[Loop Test %d Finish (total err: %d))]===", i, *err_cnt);
	}

	return all_ret;
}

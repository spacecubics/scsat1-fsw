/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "mtq.h"
#include "mgnm.h"

static int print_mgnm(uint32_t sec)
{
	for (int i = 0; i < sec; i++) {
		k_sleep(K_SECONDS(1));
		print_mgnm_field();
	}

	return 0;
}

int mtq_test(uint32_t *err_cnt, uint32_t wait_sec)
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

	if (wait_sec == 0) {
		wait_sec = 5;
	}

	for (int i = 0; i < ARRAY_SIZE(axes_list); i++) {
		for (int j = 0; j < ARRAY_SIZE(pol_list); j++) {
			ret = mtq_start(axes_list[i], pol_list[j], duty);
			if (ret < 0) {
				(*err_cnt)++;
				all_ret = -1;
				continue;
			}

			ret = print_mgnm(wait_sec);
			if (ret < 0) {
				(*err_cnt)++;
				all_ret = -1;
				continue;
			}

			ret = mtq_stop(axes_list[i]);
			if (ret < 0) {
				(*err_cnt)++;
				all_ret = -1;
				continue;
			}
		}
	}

	return all_ret;
}

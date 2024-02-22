/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "rw.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rw_test);

static void print_rw_status(enum rw_pos pos, uint32_t sec)
{
	for (int j=0; j<sec; j++) {
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
		//RW_POS_Y,
		//RW_POS_Z,
	};
	uint8_t pot_list[] = {

		0x40,
/*
		0x6F,
		0x55,
		0x34,
		0x22,
		0x14,
		0x00,
*/
	};
	uint8_t sec = 20;

	//LOG_INF("Change Potention: 0x7F on X");
	//ret = rw_change_speed(RW_POS_X, 0x7F);
	LOG_INF("Change Potention: 0x00 on X");
	ret = rw_change_speed(RW_POS_X, 0x00);
	if (ret < 0) {
		(*err_cnt)++;
		all_ret = -1;
		goto end;
	}

	ret = rw_start(RW_POS_X);
	if (ret < 0) {
		(*err_cnt)++;
		all_ret = -1;
		goto end;
	}

	//print_rw_status(RW_POS_X, sec);
	print_rw_status(RW_POS_X, 5);
	
	for (int i=0; i<ARRAY_SIZE(pot_list); i++) {

		LOG_INF("Change Potention: 0x%02x on X",
			    pot_list[i]);
		ret = rw_change_speed(RW_POS_X, pot_list[i]);
		if (ret < 0) {
			(*err_cnt)++;
			all_ret = -1;
			break;
		}

		print_rw_status(RW_POS_X, sec);
	}

	rw_stop(RW_POS_X);

end:
	return all_ret;
}

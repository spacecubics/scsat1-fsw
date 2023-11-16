/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include "gnss.h"

int gnss_test(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	ret = get_gnss_hwmon_data();
	if (ret < 0) {
		(*err_cnt)++;
		all_ret = -1;
	}

	return all_ret;
}

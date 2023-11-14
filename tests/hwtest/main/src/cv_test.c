/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "cv.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cv_test);

int cv_test(uint32_t *err_cnt)
{
	int ret = 0;
	int all_ret = 0;
	uint32_t cv;
	enum io_cv_pos io_pos_list[] = {
		IO_PDU_O4_3V3_SHUNT,
		IO_PDU_O4_3V3_BUS,
		IO_VDD_3V3_SYS_SHUNT,
		IO_VDD_3V3_SYS_BUS,
		IO_VDD_3V3_SHUNT,
		IO_VDD_3V3_BUS,
	};
	const char io_pos_name[][22] = {
		"IO_PDU_04_3V3 Shunt",
		"IO_PDU_04_3V3 Bus",
		"IO_VDD_3V3_SYS Shunt",
		"IO_VDD_3V3_SYS Bus",
		"IO_VDD_3VS Shunt",
		"IO_VDD_3VS Bus",
	};
	const char io_unit_name[][5] = {
		"[uv]",
		"[mv]",
		"[uv]",
		"[mv]",
		"[uv]",
		"[mv]",
	};

	for (int i=0; i<ARRAY_SIZE(io_pos_list); i++) {
		ret = get_ioboard_cv(io_pos_list[i], &cv);
		if (ret < 0) {
			LOG_ERR("%s: Failed", io_pos_name[i]);
			(*err_cnt)++;
			all_ret = -1;
			continue;
		}
		LOG_INF("%s: %d %s", io_pos_name[i], cv, io_unit_name[i]);
	}

	return all_ret;
}

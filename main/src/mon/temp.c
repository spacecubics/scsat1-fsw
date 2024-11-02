/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zbus/zbus.h>
#include "temp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(temp_mon, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

ZBUS_CHAN_DECLARE(temp_chan);
extern struct k_work_q monitor_workq;

#define TEMP_INVALID_VALUE (0.0f)

static void temp_monitor(struct k_work *work)
{
	int pos;
	struct temp_msg msg;
	enum obc_temp_pos obc_pos_list[] = {
		OBC_TEMP_1,
		OBC_TEMP_2,
		OBC_TEMP_3,
	};
	enum io_temp_pos io_pos_list[] = {
		IO_TEMP_POS_ONBOARD_1, IO_TEMP_POS_ONBOARD_2, IO_TEMP_POS_X_PLUS,
		IO_TEMP_POS_X_MINUS,   IO_TEMP_POS_Y_PLUS,    IO_TEMP_POS_Y_MINUS,
	};

	for (pos = 0; pos < ARRAY_SIZE(obc_pos_list); pos++) {
		msg.obc[pos].status = sc_bhm_get_obc_temp(obc_pos_list[pos], &msg.obc[pos].temp);
		if (msg.obc[pos].status < 0) {
			msg.obc[pos].temp = TEMP_INVALID_VALUE;
		}
	}

	msg.xadc.status = sc_bhm_get_xadc_temp(&msg.xadc.temp);
	if (msg.xadc.status < 0) {
		msg.xadc.temp = TEMP_INVALID_VALUE;
	}

	for (pos = 0; pos < ARRAY_SIZE(io_pos_list); pos++) {
		msg.io[pos].status = get_ioboard_temp(io_pos_list[pos], &msg.io[pos].temp);
		if (msg.io[pos].status < 0) {
			msg.io[pos].temp = TEMP_INVALID_VALUE;
		}
	}

	zbus_chan_pub(&temp_chan, &msg, K_NO_WAIT);
}

static K_WORK_DEFINE(temp_monitor_work, temp_monitor);

static void temp_monitor_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&monitor_workq, &temp_monitor_work);
}

static K_TIMER_DEFINE(temp_monitor_timer, temp_monitor_handler, NULL);

void start_temp_monitor(void)
{
	k_timer_start(&temp_monitor_timer, K_MSEC(CONFIG_SCSAT1_MAIN_MON_TEMP_DELAY_MSEC),
		      K_MSEC(CONFIG_SCSAT1_MAIN_MON_TEMP_INTERVAL_MSEC));
}

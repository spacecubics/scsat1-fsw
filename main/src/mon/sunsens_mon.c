/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zbus/zbus.h>
#include "sunsens.h"
#include "sunsens_mon.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sunsens_mon, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

ZBUS_CHAN_DECLARE(sunsens_chan);
extern struct k_work_q monitor_workq;

#define SUNSENS_INVALID_TEMP   (0.0F)
#define SUNSENS_INVALID_SUNDATA (0U)

static void sunsens_monitor(struct k_work *work)
{
	int pos;
	struct sunsens_msg msg;
	enum sunsens_pos pos_list[] = {SUNSENS_POS_Y_PLUS, SUNSENS_POS_Y_MINUS};

	for (pos = 0; pos < ARRAY_SIZE(pos_list); pos++) {
		msg.sun[pos].status = get_sunsens_data(pos_list[pos], &msg.sun[pos].data);
		if (msg.sun[pos].status < 0) {
			LOG_ERR("Failed to get the Sun data");
			msg.sun[pos].data.a = SUNSENS_INVALID_SUNDATA;
			msg.sun[pos].data.b = SUNSENS_INVALID_SUNDATA;
			msg.sun[pos].data.c = SUNSENS_INVALID_SUNDATA;
			msg.sun[pos].data.d = SUNSENS_INVALID_SUNDATA;
		}
	}

	for (pos = 0; pos < ARRAY_SIZE(pos_list); pos++) {
		msg.temp[pos].status = get_sunsens_temp(pos_list[pos], &msg.temp[pos].data);
		if (msg.temp[pos].status < 0) {
			LOG_ERR("Failed to get the temperature");
			msg.temp[pos].data = SUNSENS_INVALID_TEMP;
		}
	}

	zbus_chan_pub(&sunsens_chan, &msg, K_NO_WAIT);
}

static K_WORK_DEFINE(sunsens_monitor_work, sunsens_monitor);

static void sunsens_monitor_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&monitor_workq, &sunsens_monitor_work);
}

static K_TIMER_DEFINE(sunsens_monitor_timer, sunsens_monitor_handler, NULL);

void start_sunsens_monitor(void)
{
	k_timer_start(&sunsens_monitor_timer, K_MSEC(CONFIG_SCSAT1_MAIN_MON_TEMP_DELAY_MSEC),
		      K_MSEC(CONFIG_SCSAT1_MAIN_MON_TEMP_INTERVAL_MSEC));
}

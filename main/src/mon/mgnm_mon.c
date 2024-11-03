/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zbus/zbus.h>
#include "mgnm.h"
#include "mgnm_mon.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mgnm_mon, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

ZBUS_CHAN_DECLARE(mgnm_chan);
extern struct k_work_q monitor_workq;

#define MGNM_INVALID_TEMP   (0.0F)
#define MGNM_INVALID_MAGNET (0U)

static void mgnm_monitor(struct k_work *work)
{
	int pos;
	struct mgnm_msg msg;
	enum mgnm_pos pos_list[] = {MGNM_POS_X_PLUS, MGNM_POS_X_MINUS};

	for (pos = 0; pos < ARRAY_SIZE(pos_list); pos++) {
		msg.magnet[pos].status = start_mgnm_magnet_measurement(pos_list[pos]);
		if (msg.magnet[pos].status < 0) {
			LOG_ERR("Failed to start the measurement magnet field");
			msg.magnet[pos].data.x_out = MGNM_INVALID_MAGNET;
			msg.magnet[pos].data.y_out = MGNM_INVALID_MAGNET;
			msg.magnet[pos].data.z_out = MGNM_INVALID_MAGNET;
			continue;
		}

		msg.magnet[pos].status = get_mgnm_magnet(pos_list[pos], &msg.magnet[pos].data);
		if (msg.magnet[pos].status < 0) {
			LOG_ERR("Failed to get the magnet field");
			msg.magnet[pos].data.x_out = MGNM_INVALID_MAGNET;
			msg.magnet[pos].data.y_out = MGNM_INVALID_MAGNET;
			msg.magnet[pos].data.z_out = MGNM_INVALID_MAGNET;
		}
	}

	for (pos = 0; pos < ARRAY_SIZE(pos_list); pos++) {
		msg.temp[pos].status = start_mgnm_temp_measurement(pos_list[pos]);
		if (msg.temp[pos].status < 0) {
			LOG_ERR("Failed to start the measurement temperature");
			msg.temp[pos].data = MGNM_INVALID_TEMP;
			continue;
		}

		msg.temp[pos].status = get_mgnm_temp(pos_list[pos], &msg.temp[pos].data);
		if (msg.temp[pos].status < 0) {
			LOG_ERR("Failed to get the temperature");
			msg.temp[pos].data = MGNM_INVALID_TEMP;
		}
	}

	zbus_chan_pub(&mgnm_chan, &msg, K_NO_WAIT);
}

static K_WORK_DEFINE(mgnm_monitor_work, mgnm_monitor);

static void mgnm_monitor_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&monitor_workq, &mgnm_monitor_work);
}

static K_TIMER_DEFINE(mgnm_monitor_timer, mgnm_monitor_handler, NULL);

void start_mgnm_monitor(void)
{
	k_timer_start(&mgnm_monitor_timer, K_MSEC(CONFIG_SCSAT1_MAIN_MON_TEMP_DELAY_MSEC),
		      K_MSEC(CONFIG_SCSAT1_MAIN_MON_TEMP_INTERVAL_MSEC));
}

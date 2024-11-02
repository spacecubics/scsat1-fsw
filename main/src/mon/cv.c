/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zbus/zbus.h>
#include "cv_main.h"
#include "cv.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cv_mon, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

ZBUS_CHAN_DECLARE(cv_chan);
extern struct k_work_q monitor_workq;

#define CV_INVALID_FLOAT (0.0F)
#define CV_INVALID_INT   (0U)

static void cv_monitor(struct k_work *work)
{
	struct cv_msg msg;
	int pos;

	enum obc_cv_pos obc_pos_list[] = {
		OBC_1V0_SHUNT,      OBC_1V0_BUS,      OBC_1V8_SHUNT,      OBC_1V8_BUS,
		OBC_3V3_SHUNT,      OBC_3V3_BUS,      OBC_3V3_SYSA_SHUNT, OBC_3V3_SYSA_BUS,
		OBC_3V3_SYSB_SHUNT, OBC_3V3_SYSB_BUS, OBC_3V3_IO_SHUNT,   OBC_3V3_IO_BUS,
	};
	enum xadc_cv_pos xadc_pos_list[] = {
		OBC_XADC_VCCINT,
		OBC_XADC_VCCAUX,
		OBC_XADC_VCCBRAM,
	};
	enum io_cv_pos io_pos_list[] = {
		IO_PDU_O4_3V3_SHUNT, IO_PDU_O4_3V3_BUS, IO_VDD_3V3_SYS_SHUNT,
		IO_VDD_3V3_SYS_BUS,  IO_VDD_3V3_SHUNT,  IO_VDD_3V3_BUS,
	};

	for (pos = 0; pos < ARRAY_SIZE(obc_pos_list); pos++) {
		msg.obc[pos].status = sc_bhm_get_obc_cv(obc_pos_list[pos], &msg.obc[pos].cv);
		if (msg.obc[pos].status < 0) {
			msg.obc[pos].cv = CV_INVALID_INT;
		}
	}

	for (pos = 0; pos < ARRAY_SIZE(xadc_pos_list); pos++) {
		msg.xadc[pos].status = sc_bhm_get_xadc_cv(xadc_pos_list[pos], &msg.xadc[pos].cv);
		if (msg.xadc[pos].status < 0) {
			msg.xadc[pos].cv = CV_INVALID_INT;
		}
	}

	for (pos = 0; pos < ARRAY_SIZE(io_pos_list); pos++) {
		msg.io[pos].status = get_ioboard_cv(io_pos_list[pos], &msg.io[pos].cv);
		if (msg.io[pos].status < 0) {
			msg.io[pos].cv = CV_INVALID_INT;
		}
	}

	zbus_chan_pub(&cv_chan, &msg, K_NO_WAIT);
}

static K_WORK_DEFINE(cv_monitor_work, cv_monitor);

static void cv_monitor_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&monitor_workq, &cv_monitor_work);
}

static K_TIMER_DEFINE(cv_monitor_timer, cv_monitor_handler, NULL);

void start_cv_monitor(void)
{
	k_timer_start(&cv_monitor_timer, K_MSEC(CONFIG_SCSAT1_MAIN_MON_CV_DELAY_MSEC),
		      K_MSEC(CONFIG_SCSAT1_MAIN_MON_CV_INTERVAL_MSEC));
}

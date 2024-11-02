/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zbus/zbus.h>
#include "cv_main.h"
#include "cv.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cv_mon, CONFIG_SCSAT1_ADCS_LOG_LEVEL);

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
	enum adcs_cv_pos adcs_pos_list[] = {
		ADCS_VDD_3V3_IMU_SHUNT, ADCS_VDD_3V3_IMU_BUS,   ADCS_VDD_3V3_GPS_SHUNT,
		ADCS_VDD_3V3_GPS_BUS,   ADCS_VDD_3V3_DRV_SHUNT, ADCS_VDD_3V3_DRV_BUS,
	};
	enum rw_cv_pos rw_pos_list[] = {
		ADCS_VDD_12V_DRVX_SHUNT, ADCS_VDD_12V_DRVX_BUS,   ADCS_VDD_12V_DRVY_SHUNT,
		ADCS_VDD_12V_DRVY_BUS,   ADCS_VDD_12V_DRVZ_SHUNT, ADCS_VDD_12V_DRVZ_BUS,
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
			msg.xadc[pos].cv = CV_INVALID_FLOAT;
		}
	}

	for (pos = 0; pos < ARRAY_SIZE(adcs_pos_list); pos++) {
		msg.adcs[pos].status = get_adcs_cv(adcs_pos_list[pos], &msg.adcs[pos].cv);
		if (msg.adcs[pos].status < 0) {
			msg.adcs[pos].cv = CV_INVALID_INT;
		}
	}

	for (pos = 0; pos < ARRAY_SIZE(rw_pos_list); pos++) {
		msg.rw[pos].status = get_rw_cv(rw_pos_list[pos], &msg.rw[pos].cv);
		if (msg.rw[pos].status < 0) {
			msg.rw[pos].cv = CV_INVALID_INT;
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
	k_timer_start(&cv_monitor_timer, K_MSEC(CONFIG_SCSAT1_ADCS_MON_CV_DELAY_MSEC),
		      K_MSEC(CONFIG_SCSAT1_ADCS_MON_CV_INTERVAL_MSEC));
}

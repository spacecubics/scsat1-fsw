/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/posix/time.h>
#include <zephyr/zbus/zbus.h>
#include <stdlib.h>
#include "system.h"
#include "handler.h"
#include "fram.h"
#include "pwrctrl_adcs.h"
#include "sc_fpgaconf.h"
#include "sc_fpgahrmem.h"
#include "sc_fpgamon.h"
#include "sc_fpgasys.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(system_mon, CONFIG_SCSAT1_ADCS_LOG_LEVEL);

ZBUS_CHAN_DECLARE(system_chan);
extern struct k_work_q monitor_workq;

extern struct csp_stat csp_stat;

static void system_monitor(struct k_work *work)
{
	int ret;
	struct system_msg msg;
	struct timespec ts;

	ret = clock_gettime(CLOCK_REALTIME, &ts);
	if (ret < 0) {
		LOG_WRN("Faild to get clock time, set with 0s");
		msg.wall_clock = 0;
	} else {
		msg.wall_clock = ts.tv_sec;
	}

	msg.sysup_time = k_uptime_get_32() / MSEC_PER_SEC;
	msg.sw_version = (uint32_t)strtoul(SCSAT1_FSW_GIT_HASH, NULL, 16);
	sc_fram_get_boot_count(&msg.boot_count);
	msg.power_status = sc_adcs_get_power_status();
	msg.fpga_version = sc_get_fpga_build_hash();
	msg.fpga_config_bank = sc_get_boot_cfgmem();
	msg.fpga_fallback_state = sc_fpgaconf_get_bootsts();
	msg.received_command_count = csp_stat.received_command_count;
	msg.last_csp_port = csp_stat.last_csp_port;
	msg.last_command_id = csp_stat.last_command_id;
	sc_hrmem_get_ecc_error_count(&msg.ecc_error_count_by_auto, &msg.ecc_error_count_by_bus);
	sc_sem_get_error_count(&msg.sem_error_count);

	zbus_chan_pub(&system_chan, &msg, K_NO_WAIT);
}

static K_WORK_DEFINE(system_monitor_work, system_monitor);

static void system_monitor_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&monitor_workq, &system_monitor_work);
}

static K_TIMER_DEFINE(system_monitor_timer, system_monitor_handler, NULL);

void start_system_monitor(void)
{
	k_timer_start(&system_monitor_timer, K_MSEC(CONFIG_SCSAT1_ADCS_MON_SYSTEM_DELAY_MSEC),
		      K_MSEC(CONFIG_SCSAT1_ADCS_MON_SYSTEM_INTERVAL_MSEC));
}

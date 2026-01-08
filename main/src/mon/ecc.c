/*
 * Copyright (c) 2025 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/posix/time.h>
#include <zephyr/zbus/zbus.h>
#include <stdlib.h>
#include "ecc.h"
#include "handler.h"
#include "sc_fpgahrmem.h"
#include "sc_fpgamon.h"
#include "sc_fpgasys.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ecc_mon, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

ZBUS_CHAN_DECLARE(ecc_chan);
extern struct k_work_q monitor_workq;

static void ecc_monitor(struct k_work *work)
{
	struct ecc_msg msg;

	msg.ecc_enable_ecc_collect = sc_hrmem_is_enable_ecc_collect();
	msg.ecc_enable_mem_scrub = sc_hrmem_is_enable_memory_scrub();
	msg.ecc_enable_mem_scrub_arbit = sc_hrmem_is_memscrub_arbitration();
	msg.ecc_mem_scrub_cycle = sc_hrmem_get_memscrub_cycle();
	msg.ecc_error_address = sc_hrmem_get_ecc_error_address();
	msg.ecc_error_discard_count = sc_hrmem_get_ecc_error_discard();
	sc_hrmem_get_ecc_error_count(&msg.ecc_error_count_by_auto, &msg.ecc_error_count_by_bus);
	msg.sem_contoller_state = sc_sem_get_controller_state();
	sc_sem_get_error_count(&msg.sem_error_count);

	zbus_chan_pub(&ecc_chan, &msg, K_NO_WAIT);
}

static K_WORK_DEFINE(ecc_monitor_work, ecc_monitor);

static void ecc_monitor_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&monitor_workq, &ecc_monitor_work);
}

static K_TIMER_DEFINE(ecc_monitor_timer, ecc_monitor_handler, NULL);

void start_ecc_monitor(void)
{
	/* Change HRMEM Scrubbing Cycle to 1 seconds */
	sc_hrmem_set_memscrub_cycle(0x001F);

	k_timer_start(&ecc_monitor_timer, K_MSEC(CONFIG_SCSAT1_MAIN_MON_ECC_DELAY_MSEC),
		      K_MSEC(CONFIG_SCSAT1_MAIN_MON_ECC_INTERVAL_MSEC));
}

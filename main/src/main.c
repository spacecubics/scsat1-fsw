/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "csp.h"
#include "wdog.h"
#include "config_nor.h"
#include "data_nor.h"
#include "fram.h"
#include "pwrctrl_main.h"
#include "syshk.h"
#include "monitor.h"
#include "eps.h"

int32_t last_time_sync_ret;
uint16_t time_sync_count;

int main(void)
{
	uint32_t interval = 0;

	start_kick_wdt_thread();

	datafs_init();
	sc_config_nor_set_addr_mode();

	sc_fram_update_boot_count();

	sc_csp_init();
	csp_time_sync_from_eps();

	sc_main_power_enable(PDU_O1_PWR);

	start_monitor();

	start_send_syshk();

	while (true) {
		interval++;
		if (interval % CONFIG_SCSAT1_MAIN_MON_TIMESYNC_INTERVAL_SEC == 0) {
			last_time_sync_ret = csp_time_sync_from_eps();
			if (last_time_sync_ret == 0) {
				time_sync_count++;
			}
		}
		k_sleep(K_SECONDS(1));
	}

	return 0;
}

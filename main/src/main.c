/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "csp.h"
#include "wdog.h"
#include "data_nor.h"
#include "fram.h"
#include "pwrctrl_main.h"
#include "syshk.h"

int main(void)
{
	start_kick_wdt_thread();

	datafs_init();

	sc_fram_update_boot_count();

	sc_csp_init();

	sc_main_power_enable(PDU_O1_PWR);

	start_send_syshk();

	while (true) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}

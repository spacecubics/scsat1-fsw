/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "sc_fpgamon.h"

static void *kick_watchdog_timer(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		sc_kick_wdt_timer();
		k_sleep(K_SECONDS(CONFIG_SC_LIB_CORE_WDOG_KICK_INTERVAL_SEC));
	}

	return NULL;
}

K_THREAD_DEFINE(kick_wdt_id, CONFIG_SC_LIB_CORE_WDOG_THREAD_STACK_SIZE, kick_watchdog_timer, NULL,
		NULL, NULL, CONFIG_SC_LIB_CORE_WDOG_THREAD_PRIORITY, 0, K_TICKS_FOREVER);

void start_kick_wdt_thread(void)
{
	k_thread_start(kick_wdt_id);
}

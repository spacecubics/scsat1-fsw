/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "sysmon.h"

#define WDT_THREAD_STACK_SIZE (256U)
#define WDT_THREAD_PRIORITY   (0U)

static void *kick_watchdog_timer(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		sc_main_kick_wdt_timer();
		k_sleep(K_SECONDS(1));
	}

	return NULL;
}

K_THREAD_DEFINE(kick_wdt_id, WDT_THREAD_STACK_SIZE, kick_watchdog_timer, NULL, NULL, NULL,
		WDT_THREAD_PRIORITY, 0, K_TICKS_FOREVER);

void start_kick_wdt_thread(void)
{
	k_thread_start(kick_wdt_id);
}

/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "csp.h"
#include "wdog.h"

int main(void)
{
	start_kick_wdt_thread();

	sc_csp_init();

	while (true) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}

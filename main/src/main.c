/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "csp.h"

int main(void)
{
	sc_csp_init();

	while (true) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}

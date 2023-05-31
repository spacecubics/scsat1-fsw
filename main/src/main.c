/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <csp/csp.h>

int main(void)
{
	csp_init();

	printk("Hello World! %s\n", CONFIG_BOARD);
	return 0;
}

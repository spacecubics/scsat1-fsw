/*
 * Copyright (c) 2023 Space Cubics,LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

/* Registers */
#define SCADCS_SYSREG_BASE_ADDR (0x50000000)
#define SCADCS_PCR_OFFSET (0x0000)

/* Power Control Register */
#define SCADCS_PCR_KEYCODE (0x5A5A << 16)

void sc_adcs_power_enable(uint8_t target_bit)
{
	sys_set_bits(SCADCS_SYSREG_BASE_ADDR + SCADCS_PCR_OFFSET,
				 SCADCS_PCR_KEYCODE | target_bit);
}

void sc_adcs_power_disable(uint8_t target_bit)
{
	sys_clear_bits(SCADCS_SYSREG_BASE_ADDR + SCADCS_PCR_OFFSET,
				 SCADCS_PCR_KEYCODE | target_bit);
}

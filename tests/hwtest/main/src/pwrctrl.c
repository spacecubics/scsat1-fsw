/*
 * Copyright (c) 2023 Space Cubics,LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

/* Registers */
#define SCMAIN_SYSREG_BASE_ADDR (0x50000000)
#define SCMAIN_PCR_OFFSET       (0x0000)

/* Power Control Register */
#define SCMAIN_PCR_KEYCODE (0x5A5A << 16)

void sc_main_power_enable(uint8_t target_bit)
{
	sys_set_bits(SCMAIN_SYSREG_BASE_ADDR + SCMAIN_PCR_OFFSET, SCMAIN_PCR_KEYCODE | target_bit);
}

void sc_main_power_disable(uint8_t target_bit)
{
	uint32_t val = sys_read32(SCMAIN_SYSREG_BASE_ADDR + SCMAIN_PCR_OFFSET);

	val = SCMAIN_PCR_KEYCODE | (val & ~target_bit);
	sys_write32(val, SCMAIN_SYSREG_BASE_ADDR + SCMAIN_PCR_OFFSET);
}

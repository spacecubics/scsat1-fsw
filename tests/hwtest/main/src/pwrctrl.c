/*
 * Copyright (c) 2023 Space Cubics,LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

/* Registers */
#define SC_MAIN_SYSREG_BASE_ADDR (0x4F000000)
#define SC_MAIN_PWRCYCLE         (0x0020)

#define SC_MAIN_MAIN_BASE_ADDR (0x50000000)
#define SC_MAIN_PCR_OFFSET     (0x0000)

/* Power Control Register */
#define SC_MAIN_PCR_KEYCODE (0x5A5A << 16)

/* Power Cycle Request to TRCH */
#define SC_MAIN_PWRCYCLEPKC (0x5A5A << 16)
#define SC_MAIN_PWRCYCLEREQ (1U)

void sc_main_power_enable(uint8_t target_bit)
{
	sys_set_bits(SC_MAIN_MAIN_BASE_ADDR + SC_MAIN_PCR_OFFSET, SC_MAIN_PCR_KEYCODE | target_bit);
}

void sc_main_power_disable(uint8_t target_bit)
{
	uint32_t val = sys_read32(SC_MAIN_MAIN_BASE_ADDR + SC_MAIN_PCR_OFFSET);

	val = SC_MAIN_PCR_KEYCODE | (val & ~target_bit);
	sys_write32(val, SC_MAIN_MAIN_BASE_ADDR + SC_MAIN_PCR_OFFSET);
}

void sc_main_power_cycle_req(void)
{
	sys_write32(SC_MAIN_PWRCYCLEPKC | SC_MAIN_PWRCYCLEREQ,
		    SC_MAIN_SYSREG_BASE_ADDR + SC_MAIN_PWRCYCLE);
}

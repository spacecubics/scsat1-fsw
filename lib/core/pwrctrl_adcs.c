/*
 * Copyright (c) 2023 Space Cubics,LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

/* Registers */
#define SC_ADCS_SYSREG_BASE_ADDR (0x4F000000)
#define SC_ADCS_PWRCYCLE         (0x0020)

#define SCADCS_ADCS_BASE_ADDR   (0x50000000)
#define SCADCS_PCR_OFFSET       (0x0000)
#define SCADCS_IRCR_OFFSET      (0x0004)
#define SCADCS_MDCR_OFFSET      (0x0008)
#define SCADCS_DRVISR_OFFSET    (0x0020)
#define SCADCS_DRVIER_OFFSET    (0x0024)

/* Power Control Register */
#define SCADCS_PCR_KEYCODE (0x5A5A << 16)

/* IMU Reset Control Register */
#define SCADCS_IMU_RESET_BIT BIT(0)

/* Power Cycle Request to TRCH */
#define SC_ADCS_PWRCYCLEPKC (0x5A5A << 16)
#define SC_ADCS_PWRCYCLEREQ (1U)

void sc_adcs_power_enable(uint8_t target_bit)
{
	sys_set_bits(SCADCS_ADCS_BASE_ADDR + SCADCS_PCR_OFFSET, SCADCS_PCR_KEYCODE | target_bit);
}

void sc_adcs_power_disable(uint8_t target_bit)
{
	sys_clear_bits(SCADCS_ADCS_BASE_ADDR + SCADCS_PCR_OFFSET,
		       SCADCS_PCR_KEYCODE | target_bit);
}

void sc_adcs_imu_reset(void)
{
	sys_clear_bits(SCADCS_ADCS_BASE_ADDR + SCADCS_IRCR_OFFSET, SCADCS_IMU_RESET_BIT);
	k_sleep(K_USEC(1));
}

void sc_adcs_imu_reset_release(void)
{
	sys_set_bits(SCADCS_ADCS_BASE_ADDR + SCADCS_IRCR_OFFSET, SCADCS_IMU_RESET_BIT);
	k_sleep(K_MSEC(300));
}

void sc_adcs_motor_enable(uint8_t target_bit)
{
	sys_set_bits(SCADCS_ADCS_BASE_ADDR + SCADCS_MDCR_OFFSET, target_bit);
}

void sc_adcs_motor_disable(uint8_t target_bit)
{
	sys_clear_bits(SCADCS_ADCS_BASE_ADDR + SCADCS_MDCR_OFFSET, target_bit);
}

void sc_adcs_power_cycle_req(void)
{
	sys_write32(SC_ADCS_PWRCYCLEPKC | SC_ADCS_PWRCYCLEREQ,
		    SC_ADCS_SYSREG_BASE_ADDR + SC_ADCS_PWRCYCLE);
}

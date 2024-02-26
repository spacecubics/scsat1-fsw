/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "qspi_norflash.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bootloader);

#define SCOBCA1_FPGA_SYSREG_CODEMSEL (0x4F000000)
#define SCOBCA1_FPGA_HRMEM_ADDR      (0x60000000)
#define SYSREG_CODEMSEL_ENABLE_HRMEM (0x5a5a0000)

int main(void)
{
	int ret;

	ret = qspi_norflash_copy_to_hrmem(QSPI_CFG_MEM0, CONFIG_SCOBC_A1_BOOT_CFG_MEM_ADDR,
					  KB(CONFIG_SCOBC_A1_BOOT_CFG_COPY_SIZE_KB),
					  SCOBCA1_FPGA_HRMEM_ADDR);
	if (ret < 0) {
		LOG_ERR("Failed to copy the Zephyr application from Config memory to HRMEM.");
		goto end;
	}

	sys_write32(SYSREG_CODEMSEL_ENABLE_HRMEM, SCOBCA1_FPGA_SYSREG_CODEMSEL);

end:
	return ret;
}

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>

#define SCOBCA1_FPGA_SYSREG_CODEMSEL (0x4F000000)
#define SCOBCA1_FPGA_HRMEM_ADDR      (0x60000000)
#define SYSREG_CODEMSEL_ENABLE_HRMEM (0x5a5a0000)
#define QSPI_RX_FIFO_MAX_BYTE        (16U)

static int copy_norflash_to_hrmem(uint8_t partition_id, mm_reg_t sram_mem_addr, uint32_t size)
{
	int ret;
	const struct flash_area *flash = NULL;
	uint8_t read_vals[QSPI_RX_FIFO_MAX_BYTE];
	uint16_t loop_count;
	off_t offset = 0;

	ret = flash_area_open(partition_id, &flash);
	if (ret < 0) {
		goto end;
	}

	loop_count = size / QSPI_RX_FIFO_MAX_BYTE;
	for (uint16_t i = 0; i < loop_count; i++) {

		ret = flash_area_read(flash, offset, (void *)read_vals, QSPI_RX_FIFO_MAX_BYTE);
		if (ret < 0) {
			goto cleanup;
		}

		memcpy((void *)sram_mem_addr, read_vals, QSPI_RX_FIFO_MAX_BYTE);

		offset += QSPI_RX_FIFO_MAX_BYTE;
		sram_mem_addr += QSPI_RX_FIFO_MAX_BYTE;
	}

cleanup:
	flash_area_close(flash);

end:
	return ret;
}

int main(void)
{
	int ret;

	ret = copy_norflash_to_hrmem(FIXED_PARTITION_ID(fsw_partition), SCOBCA1_FPGA_HRMEM_ADDR,
				     KB(CONFIG_SCOBC_A1_BOOT_CFG_COPY_SIZE_KB));
	if (ret < 0) {
		goto end;
	}

	sys_write32(SYSREG_CODEMSEL_ENABLE_HRMEM, SCOBCA1_FPGA_SYSREG_CODEMSEL);

end:
	return ret;
}

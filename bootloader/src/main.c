/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include "version.h"
#include "sc_fpgasys.h"
#include "sc_fpgaconf.h"

#define SCOBCA1_FPGA_SYSREG_CFGMEMCTL (0x4F000010)
#define SCOBCA1_FPGA_HRMEM_ADDR       (0x60000000)
#define SYSREG_CODEMSEL_ENABLE_HRMEM  (0x5a5a0000)
#define QSPI_RX_FIFO_MAX_BYTE         (16U)

static int select_cfg_memory(void)
{
	int ret;
	enum sc_cfgmem boot_bank;

	/*
	 * FPGA configures using the Config memory bank specified by TRCH,
	 * but after control is transferred to the FPGA, the Config Memory
	 * selects the default 0 bank. Therefore, the Bootloader needs to
	 * check the FPGA registers to determine which bank was used for
	 * configuration, and then load the FSW from the same bank and copy
	 * it to HRMEM.
	 */
	boot_bank = sc_get_boot_cfgmem();

	ret = sc_select_cfgmem(boot_bank);
	if (ret == 0) {
		printk("Boot from CFG bank %d\n", boot_bank);
	} else {
		printk("Faild to boot from CFG bank %d\n", boot_bank);
	}

	return ret;
}

static int copy_norflash_to_hrmem(uint8_t partition_id, mm_reg_t sram_mem_addr, uint32_t size)
{
	int ret;
	const struct flash_area *flash = NULL;
	uint8_t read_vals[QSPI_RX_FIFO_MAX_BYTE];
	uint16_t loop_count;
	off_t offset = 0;

	ret = select_cfg_memory();
	if (ret < 0) {
		goto end;
	}

	printk("Load from partition id: %d\n", partition_id);

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
	uint8_t partition_id = FIXED_PARTITION_ID(update_fsw);

	printk("Start SC-Sat1 Boot loader (v%s)\n", BOOT_LOADER_VERSION);
	printk("FPGA Boot Status : 0x%08x\n", sc_get_bootsts());

	if (sc_fpgaconf_is_fallback()) {
		partition_id = FIXED_PARTITION_ID(golden_fsw);
	}

	ret = copy_norflash_to_hrmem(partition_id, SCOBCA1_FPGA_HRMEM_ADDR,
				     KB(CONFIG_SCOBC_A1_BOOT_CFG_COPY_SIZE_KB));
	if (ret < 0) {
		goto end;
	}

	sc_select_codemem(SC_HRMEM);

end:
	return ret;
}

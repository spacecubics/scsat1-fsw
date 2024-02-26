/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "qspi_norflash.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(qspi_norflash);

#define SCOBCA1_FPGA_SYSREG_CFGMEMCTL (0x4F000010)

#define QSPI_NOR_FLASH_MEM_ADDR_SIZE     (3u)
#define QSPI_DATA_MEM0_SS                (0x01)
#define QSPI_DATA_MEM1_SS                (0x02)
#define QSPI_ASR_IDLE                    (0x00)
#define QSPI_ASR_BUSY                    (0x01)
#define QSPI_RX_FIFO_MAX_BYTE            (16u)
#define QSPI_NOR_FLASH_DUMMY_CYCLE_COUNT (4u)
#define QSPI_SPI_MODE_QUAD               (0x00020000)

#define TRCH_CFG_MEM_MONI_BIT  (2u) /* TRCH RB2 */
#define TRCH_CFG_MEM_MONI_MASK (0x04)

#define REG_READ_RETRY(count) (count)

static inline void write32(uint32_t addr, uint32_t val)
{
	sys_write32(val, addr);
}

static bool verify(uint32_t addr, uint32_t exp, uint32_t retry)
{
	uint32_t regval;

	regval = sys_read32(addr);
	if (regval == exp) {
		LOG_DBG("  read32  [0x%08X] 0x%08x (exp:0x%08x)", addr, regval, exp);
		return true;
	} else if (retry == 0) {
		LOG_ERR("  read32  [0x%08X] 0x%08x (exp:0x%08x)", addr, regval, exp);
	}

	for (uint32_t i = 0; i < retry; i++) {
		regval = sys_read32(addr);
		if (regval == exp) {
			LOG_DBG("  read32  [0x%08X] 0x%08x (exp:0x%08x) (retry:%d)", addr, regval,
				exp, i + 1);
			return true;
		} else if (i + 1 == retry) {
			LOG_ERR("  read32  [0x%08X] 0x%08x (exp:0x%08x) (retry:%d)", addr, regval,
				exp, i + 1);
		} else {
			LOG_DBG("  read32  [0x%08X] 0x%08x (exp:0x%08x) (retry:%d)", addr, regval,
				exp, i + 1);
		}
		k_usleep(1);
	}

	LOG_ERR("  !!! Assertion failed: retry count: %d", retry);
	return false;
}

static bool qspi_select_mem(uint32_t base, uint8_t mem_no, uint32_t *spi_ss)
{
	if (base == SCOBCA1_FPGA_CFG_BASE_ADDR) {
		/* Config Memory is switched by CFGMEMSEL */
		if (mem_no == QSPI_DATA_MEM0) {
			LOG_DBG("* [#0] Select Config Memory 0");
			write32(SCOBCA1_FPGA_SYSREG_CFGMEMCTL, 0x00);
			if (!verify(SCOBCA1_FPGA_SYSREG_CFGMEMCTL, 0x00, REG_READ_RETRY(100000))) {
				LOG_ERR("  !!! Can not select Config Memory %d", mem_no);
				return false;
			}
		} else {
			LOG_DBG("* [#0] Select Config Memory 1");
			write32(SCOBCA1_FPGA_SYSREG_CFGMEMCTL, 0x10);
			if (!verify(SCOBCA1_FPGA_SYSREG_CFGMEMCTL, 0x30, REG_READ_RETRY(100000))) {
				LOG_ERR("  !!! Can not select Config Memory %d", mem_no);
				return false;
			}
		}
		*spi_ss = 0x01;
	} else {
		if (mem_no == QSPI_DATA_MEM0) {
			*spi_ss = QSPI_DATA_MEM0_SS;
		} else {
			*spi_ss = QSPI_DATA_MEM1_SS;
		}
	}

	return true;
}

static bool is_qspi_idle(uint32_t base)
{
	LOG_DBG("* Confirm QSPI Access Status is `Idle`");
	if (!verify(SCOBCA1_FPGA_NORFLASH_QSPI_ASR(base), QSPI_ASR_IDLE, REG_READ_RETRY(10))) {
		LOG_ERR("QSPI (Data Memory) is busy, so exit test");
		return false;
	}

	return true;
}

static bool activate_spi_ss(uint32_t base, uint32_t spi_mode)
{
	LOG_DBG("* Activate SPI SS with %08x", spi_mode);
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_ACR(base), spi_mode);
	if (!is_qspi_idle(base)) {
		return false;
	}

	return true;
}

static bool inactivate_spi_ss(uint32_t base)
{
	LOG_DBG("* Inactivate SPI SS");
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_ACR(base), 0x00000000);
	if (!is_qspi_idle(base)) {
		return false;
	}

	return true;
}

static void write_mem_addr_to_flash(uint32_t base, mm_reg_t mem_addr)
{
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), (mem_addr & 0x00FF0000) >> 16);
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), (mem_addr & 0x0000FF00) >> 8);
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), (mem_addr & 0x000000FF));
}

static bool send_dummy_cycle(uint32_t base, uint8_t dummy_count)
{
	LOG_DBG("* Send dummy cycle %d byte", dummy_count);
	for (uint8_t i = 0; i < dummy_count; i++) {
		write32(SCOBCA1_FPGA_NORFLASH_QSPI_RDR(base), 0x00);
	}

	if (!is_qspi_idle(base)) {
		return false;
	}

	LOG_DBG("* Discard dummy data");
	for (uint8_t i = 0; i < dummy_count; i++) {
		sys_read32(SCOBCA1_FPGA_NORFLASH_QSPI_RDR(base));
	}

	return true;
}

static bool read_rx_data(uint32_t base, size_t read_size, uint8_t *read_vals)
{
	bool ret = true;

	LOG_DBG("* Reqest RX FIFO %d byte", read_size);
	for (uint8_t i = 0; i < read_size; i++) {
		write32(SCOBCA1_FPGA_NORFLASH_QSPI_RDR(base), 0x00);
	}

	if (!is_qspi_idle(base)) {
		return false;
	}

	LOG_DBG("* Read RX FIFO %d byte", read_size);
	for (uint8_t i = 0; i < read_size; i++) {
		read_vals[i] = sys_read8(SCOBCA1_FPGA_NORFLASH_QSPI_RDR(base));
		LOG_DBG("0x%2x", read_vals[i]);
	}

	return ret;
}

static bool read_and_verify_rx_data(uint32_t base, size_t exp_size, uint32_t *exp_val)
{
	bool ret = true;

	LOG_DBG("* Reqest RX FIFO %d byte", exp_size);
	for (uint8_t i = 0; i < exp_size; i++) {
		write32(SCOBCA1_FPGA_NORFLASH_QSPI_RDR(base), 0x00);
	}

	if (!is_qspi_idle(base)) {
		return false;
	}

	LOG_DBG("* Read RX FIFO %d byte and verify the value", exp_size);
	for (uint8_t i = 0; i < exp_size; i++) {
		if (!verify(SCOBCA1_FPGA_NORFLASH_QSPI_RDR(base), exp_val[i], REG_READ_RETRY(0))) {
			ret = false;
		}
	}

	return ret;
}

static bool is_qspi_control_done(uint32_t base)
{
	LOG_DBG("* Confirm QSPI IntLOG_ERRupt Stauts is `SPI Control Done`");
	if (!verify(SCOBCA1_FPGA_NORFLASH_QSPI_ISR(base), 0x01, REG_READ_RETRY(10))) {
		return false;
	}

	LOG_DBG("* Clear QSPI IntLOG_ERRupt Stauts");
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_ISR(base), 0x01);
	if (!verify(SCOBCA1_FPGA_NORFLASH_QSPI_ISR(base), 0x00, REG_READ_RETRY(10))) {
		return false;
	}

	return true;
}

static bool verify_status_resisger1(uint32_t base, uint32_t spi_ss, size_t exp_size,
				    uint32_t *exp_val)
{
	bool ret;

	/* Activate SPI SS with SINGLE-IO */
	if (!activate_spi_ss(base, spi_ss)) {
		return false;
	}

	LOG_DBG("* Request Status Register 1");
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), 0x05);
	if (!is_qspi_idle(base)) {
		return false;
	}

	/* Read Memory data (2byte) adn Verify */
	ret = read_and_verify_rx_data(base, exp_size, exp_val);
	if (!ret) {
	}

	/* Inactive SPI SS */
	if (!inactivate_spi_ss(base)) {
		return false;
	}

	/* Confirm SPI Control is Done */
	if (!is_qspi_control_done(base)) {
		return false;
	}

	return ret;
}

static bool clear_status_register(uint32_t base, uint32_t spi_ss)
{
	/* Activate SPI SS with SINGLE-IO */
	if (!activate_spi_ss(base, spi_ss)) {
		return false;
	}

	/* Clear All ISR */
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_ISR(base), 0xFFFFFFFF);

	LOG_DBG("* Clear Status Register (Instructure:0x30) ");
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), 0x30);

	/* Inactive SPI SS */
	if (!inactivate_spi_ss(base)) {
		return false;
	}

	/* Confirm SPI Control is Done */
	if (!is_qspi_control_done(base)) {
		return false;
	}

	return true;
}

static bool set_write_enable(uint32_t base, uint32_t spi_ss)
{
	uint32_t exp_write_enable[] = {0x02, 0x02};

	/* Active SPI SS with SINGLE-IO */
	if (!activate_spi_ss(base, spi_ss)) {
		return false;
	}

	LOG_DBG("* Set `Write Enable` (Instructure:0x06) ");
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), 0x06);

	/* Inactive SPI SS */
	if (!inactivate_spi_ss(base)) {
		return false;
	}

	/* Confirm SPI Control is Done */
	if (!is_qspi_control_done(base)) {
		return false;
	}

	if (!verify_status_resisger1(base, spi_ss, ARRAY_SIZE(exp_write_enable),
				     exp_write_enable)) {
		return false;
	}

	return true;
}

static bool set_quad_io_mode(uint32_t base, uint32_t spi_ss)
{
	/* Activate SPI SS with SINGLE-IO */
	if (!activate_spi_ss(base, spi_ss)) {
		return false;
	}

	LOG_DBG("* Set QUAD I/O mode to configuration register");
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), 0x01);
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), 0x00);
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), 0x02);
	if (!is_qspi_idle(base)) {
		return false;
	}

	/* Inactivate SPI SS */
	if (!inactivate_spi_ss(base)) {
		return false;
	}

	/* Confirm SPI Control is Done */
	if (!is_qspi_control_done(base)) {
		return false;
	}

	return true;
}

static bool verify_config_register(uint32_t base, uint32_t spi_ss, size_t exp_size,
				   uint32_t *exp_val)
{
	bool ret;

	/* Activate SPI SS with SINGLE-IO */
	if (!activate_spi_ss(base, spi_ss)) {
		return false;
	}

	LOG_DBG("* Request Configuration Register");
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), 0x35);
	if (!is_qspi_idle(base)) {
		return false;
	}

	/* Read Memory data (2byte) adn Verify */
	ret = read_and_verify_rx_data(base, exp_size, exp_val);

	/* Inactive SPI SS */
	if (!inactivate_spi_ss(base)) {
		return false;
	}

	/* Confirm SPI Control is Done */
	if (!is_qspi_control_done(base)) {
		return false;
	}

	return ret;
}

static bool verify_quad_io_mode(uint32_t base, uint32_t spi_ss)
{
	uint32_t exp_quad_mode[2] = {0x02, 0x02};

	if (!verify_config_register(base, spi_ss, ARRAY_SIZE(exp_quad_mode), exp_quad_mode)) {
		return false;
	}

	return true;
}

bool qspi_norflash_init(uint32_t base, uint8_t mem_no)
{
	uint32_t spi_ss;

	if (mem_no > 1) {
		LOG_ERR("   !!! Invalid Mem number %d (expected 0 or 1)", mem_no);
		return false;
	}

	if (!qspi_select_mem(base, mem_no, &spi_ss)) {
		return false;
	}

	LOG_DBG("* [#1] Clear Status Register");
	if (!clear_status_register(base, spi_ss)) {
		return false;
	}

	LOG_DBG("* [#2] Set to `Write Enable'");
	if (!set_write_enable(base, spi_ss)) {
		return false;
	}

	LOG_DBG("* [#3] Set to `QUAD I/O modee'");
	if (!set_quad_io_mode(base, spi_ss)) {
		return false;
	}

	/* Wait 1 sec */
	k_sleep(K_MSEC(1000));

	LOG_DBG("* [#4] Verify Configuration Register is QUAD I/O mode (0x02)");
	if (!verify_quad_io_mode(base, spi_ss)) {
		return false;
	}

	return true;
}

static bool qspi_norflash_set_quad_read_mode(uint32_t base, uint32_t spi_ss)
{
	/* Active SPI SS with SINGLE-IO */
	if (!activate_spi_ss(base, spi_ss)) {
		return false;
	}

	LOG_DBG("* Set QUAD-IO read mode");
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), 0xEB);
	if (!is_qspi_idle(base)) {
		return false;
	}

	/* Keep SPI SS for QUAD Read */
	return true;
}

static bool qspi_norflash_quad_read_data(uint32_t base, uint32_t spi_ss, mm_reg_t mem_addr,
					 uint8_t read_size, uint8_t *read_vals)
{
	bool ret;

	LOG_DBG("* Activate SPI SS with Quad-IO SPI Mode");
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_ACR(base), QSPI_SPI_MODE_QUAD + spi_ss);

	LOG_DBG("* Send Memory Address (3byte)");
	write_mem_addr_to_flash(base, mem_addr);

	LOG_DBG("* Send Mode (0x00)");
	write32(SCOBCA1_FPGA_NORFLASH_QSPI_TDR(base), 0x00);

	/* Send Dummy Cycle */
	send_dummy_cycle(base, QSPI_NOR_FLASH_DUMMY_CYCLE_COUNT);
	if (!is_qspi_idle(base)) {
		return false;
	}

	/* Read Initial RX data and verify */
	ret = read_rx_data(base, read_size, read_vals);
	if (!ret) {
	}

	/* Inactive the SPI SS */
	if (!inactivate_spi_ss(base)) {
		return false;
	}

	return ret;
}

int qspi_norflash_copy_to_hrmem(uint8_t mem_no, mm_reg_t cfg_mem_addr, uint32_t size,
				mm_reg_t sram_mem_addr)
{
	int ret = 0;
	uint32_t base = SCOBCA1_FPGA_CFG_BASE_ADDR;
	uint32_t spi_ss;
	uint8_t read_vals[QSPI_RX_FIFO_MAX_BYTE];
	uint16_t loop_count;

	if (mem_no > 1) {
		LOG_ERR("Invalid Mem number %d (expected 0 or 1)", mem_no);
		ret = -1;
		goto end;
	}

	if (!qspi_select_mem(base, mem_no, &spi_ss)) {
		ret = -1;
		goto end;
	}

	LOG_DBG("* [#1] Clear Status Register");
	if (!clear_status_register(base, spi_ss)) {
		ret = -1;
		goto end;
	}

	loop_count = size / QSPI_RX_FIFO_MAX_BYTE;
	for (uint16_t i = 0; i < loop_count; i++) {

		LOG_DBG("* [#2] Set QUAD-IO Read Mode");
		if (!qspi_norflash_set_quad_read_mode(base, spi_ss)) {
			ret = -1;
			goto end;
		}

		LOG_DBG("* [#3] Read Data (QUAD-IO Mode) ");
		if (!qspi_norflash_quad_read_data(base, spi_ss, cfg_mem_addr, QSPI_RX_FIFO_MAX_BYTE,
						  read_vals)) {
			ret = -1;
			goto end;
		}

		memcpy((void *)sram_mem_addr, read_vals, QSPI_RX_FIFO_MAX_BYTE);

		cfg_mem_addr += QSPI_RX_FIFO_MAX_BYTE;
		sram_mem_addr += QSPI_RX_FIFO_MAX_BYTE;
	}

end:
	return ret;
}

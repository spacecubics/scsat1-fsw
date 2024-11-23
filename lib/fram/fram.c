/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "fram.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(fram, CONFIG_SC_LIB_FRAM_LOG_LEVEL);

/* Base address */
#define SCOBCA1_FPGA_FRAM_BASE_ADDR (0x40200000)

/* SC Control Register for FeRAM */
#define SCOBCA1_FPGA_FRAM_QSPI_ACR    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_ACR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_TDR    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_TDR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_RDR    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_RDR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_ASR    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_ASR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_FIFOSR (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_FIFOSR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_FIFORR (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_FIFORR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_ISR    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_ISR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_IER    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_IER_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_CCR    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_CCR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_DCMSR  (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_DCMSR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_FTLSR  (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_FTLSR_OFFSET)
#define SCOBCA1_FPGA_FRAM_QSPI_VER    (SCOBCA1_FPGA_FRAM_BASE_ADDR + QSPI_VER_OFFSET)

/* Offset */
#define QSPI_ACR_OFFSET    (0x0000) /* QSPI Access Control Register */
#define QSPI_TDR_OFFSET    (0x0004) /* QSPI TX Data Register */
#define QSPI_RDR_OFFSET    (0x0008) /* QSPI RX Data Register */
#define QSPI_ASR_OFFSET    (0x000C) /* QSPI Access Status Register */
#define QSPI_FIFOSR_OFFSET (0x0010) /* QSPI FIFO Status Register */
#define QSPI_FIFORR_OFFSET (0x0014) /* QSPI FIFO Reset Register */
#define QSPI_ISR_OFFSET    (0x0020) /* QSPI Interrupt Status Register */
#define QSPI_IER_OFFSET    (0x0024) /* QSPI Interrupt Enable Register */
#define QSPI_CCR_OFFSET    (0x0030) /* QSPI Clock Control Register */
#define QSPI_DCMSR_OFFSET  (0x0034) /* QSPI Data Capture Mode Setting Register */
#define QSPI_FTLSR_OFFSET  (0x0038) /* QSPI FIFO Threshold Level Setting Register */
#define QSPI_VER_OFFSET    (0xF000) /* QSPI Controller IP Version Register */

#define QSPI_FRAM_MEM_ADDR_SIZE          (3U)
#define QSPI_FRAM_MEM0_SS                (0x01)
#define QSPI_FRAM_MEM1_SS                (0x02)
#define QSPI_ASR_IDLE                    (0x00)
#define QSPI_ASR_BUSY                    (0x01)
#define QSPI_FIFO_MAX_BYTE               (16U)
#define QSPI_NOR_FLASH_DUMMY_CYCLE_COUNT (2U)
#define QSPI_SPI_MODE_QUAD               (0x00020000)

#define REG_READ_RETRY(count) (count)
#define FRAM_CRC_FNAME_MAX    (64U)
#define FRAM_BOOT_COUNT_ADDR  (0x000000)
#define FRAM_CRC_FILE_ADDR    (0x000100)

static bool assert32(uint32_t addr, uint32_t exp, uint32_t retry)
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

static bool is_qspi_idle(void)
{
	LOG_DBG("* Confirm QSPI Access Status is `Idle`");
	if (!assert32(SCOBCA1_FPGA_FRAM_QSPI_ASR, QSPI_ASR_IDLE, REG_READ_RETRY(100))) {
		LOG_ERR("QSPI (FRAM) is busy");
		return false;
	}

	return true;
}

static bool activate_spi_ss(uint32_t spi_mode)
{
	LOG_DBG("* Activate SPI SS with %08x", spi_mode);
	sys_write32(spi_mode, SCOBCA1_FPGA_FRAM_QSPI_ACR);
	if (!is_qspi_idle()) {
		return false;
	}

	return true;
}

static bool inactivate_spi_ss(void)
{
	LOG_DBG("* Inactivate SPI SS");
	sys_write32(0x00000000, SCOBCA1_FPGA_FRAM_QSPI_ACR);
	if (!is_qspi_idle()) {
		return false;
	}

	return true;
}

static void write_data_to_flash(uint8_t *write_data, size_t size)
{
	LOG_DBG("* Write TX FIFO %d byte", size);
	for (uint8_t i = 0; i < size; i++) {
		sys_write32(write_data[i], SCOBCA1_FPGA_FRAM_QSPI_TDR);
	}
}

static void write_mem_addr_to_flash(uint32_t mem_addr)
{
	sys_write32((mem_addr & 0x00FF0000) >> 16, SCOBCA1_FPGA_FRAM_QSPI_TDR);
	sys_write32((mem_addr & 0x0000FF00) >> 8, SCOBCA1_FPGA_FRAM_QSPI_TDR);
	sys_write32((mem_addr & 0x000000FF), SCOBCA1_FPGA_FRAM_QSPI_TDR);
}

static bool send_dummy_cycle(uint8_t dummy_count)
{
	LOG_DBG("* Send dummy cycle %d byte", dummy_count);
	for (uint8_t i = 0; i < dummy_count; i++) {
		sys_write32(0x00, SCOBCA1_FPGA_FRAM_QSPI_RDR);
	}

	if (!is_qspi_idle()) {
		return false;
	}

	LOG_DBG("* Discard dummy data");
	for (uint8_t i = 0; i < dummy_count; i++) {
		sys_read32(SCOBCA1_FPGA_FRAM_QSPI_RDR);
	}

	return true;
}

static bool read_and_verify_rx_data(size_t exp_size, uint32_t *exp_val)
{
	bool ret = true;

	LOG_DBG("* Reqest RX FIFO %d byte", exp_size);
	for (uint8_t i = 0; i < exp_size; i++) {
		sys_write32(0x00, SCOBCA1_FPGA_FRAM_QSPI_RDR);
	}

	if (!is_qspi_idle()) {
		return false;
	}

	LOG_DBG("* Read RX FIFO %d byte and verify the value", exp_size);
	for (uint8_t i = 0; i < exp_size; i++) {
		if (!assert32(SCOBCA1_FPGA_FRAM_QSPI_RDR, exp_val[i], REG_READ_RETRY(0))) {
			ret = false;
		}
	}

	return ret;
}

static bool is_qspi_control_done(void)
{
	LOG_DBG("* Confirm Interrupt Stauts is `SPI Control Done`");
	if (!assert32(SCOBCA1_FPGA_FRAM_QSPI_ISR, 0x01, REG_READ_RETRY(100))) {
		return false;
	}

	LOG_DBG("* Clear Interrupt Stauts");
	sys_write32(0x01, SCOBCA1_FPGA_FRAM_QSPI_ISR);
	if (!assert32(SCOBCA1_FPGA_FRAM_QSPI_ISR, 0x00, REG_READ_RETRY(100))) {
		return false;
	}

	return true;
}

static bool verify_status_resisger1(uint32_t spi_ss, size_t exp_size, uint32_t *exp_val)
{
	bool ret;

	/* Activate SPI SS with SINGLE-IO */
	if (!activate_spi_ss(spi_ss)) {
		return false;
	}

	LOG_DBG("* Request Status Register 1");
	sys_write32(0x05, SCOBCA1_FPGA_FRAM_QSPI_TDR);
	if (!is_qspi_idle()) {
		return false;
	}

	/* Read Memory data (1byte) adn Verify */
	ret = read_and_verify_rx_data(exp_size, exp_val);
	if (!ret) {
		return false;
	}

	/* Inactive SPI SS */
	if (!inactivate_spi_ss()) {
		return false;
	}

	/* Confirm SPI Control is Done */
	if (!is_qspi_control_done()) {
		return false;
	}

	return true;
}

static bool set_write_enable(uint32_t spi_ss, bool enable)
{
	uint32_t exp_write_disable[] = {0x00};
	uint32_t exp_write_enable[] = {0x02};

	/* Active SPI SS with SINGLE-IO */
	if (!activate_spi_ss(spi_ss)) {
		return false;
	}

	if (enable) {
		LOG_DBG("* Set `Write Enable` (Instructure:0x06)");
		sys_write32(0x06, SCOBCA1_FPGA_FRAM_QSPI_TDR);
	} else {
		LOG_DBG("* Set `Write Disable` (Instructure:0x04)");
		sys_write32(0x04, SCOBCA1_FPGA_FRAM_QSPI_TDR);
	}

	if (!is_qspi_idle()) {
		return false;
	}

	/* Inactive SPI SS */
	if (!inactivate_spi_ss()) {
		return false;
	}

	/* Confirm SPI Control is Done */
	if (!is_qspi_control_done()) {
		return false;
	}

	if (enable) {
		if (!verify_status_resisger1(spi_ss, ARRAY_SIZE(exp_write_enable),
					     exp_write_enable)) {
			return false;
		}
	} else {
		if (!verify_status_resisger1(spi_ss, ARRAY_SIZE(exp_write_disable),
					     exp_write_disable)) {
			return false;
		}
	}

	return true;
}

static int sc_fram_set_quad_read_mode(uint32_t spi_ss)
{
	int ret;

	/* Active SPI SS with SINGLE-IO */
	if (!activate_spi_ss(spi_ss)) {
		ret = -EIO;
		goto end;
	}

	LOG_DBG("* Set QUAD-IO read mode");
	sys_write32(0xEB, SCOBCA1_FPGA_FRAM_QSPI_TDR);
	if (!is_qspi_idle()) {
		ret = -ETIMEDOUT;
		goto end;
	}

	/* Keep SPI SS for Quad Read */

end:
	return true;
}

static int sc_fram_quad_read_data(uint32_t spi_ss, uint32_t mem_addr, uint8_t size, uint8_t *val)
{
	int ret = 0;
	uint8_t i;

	LOG_DBG("* Activate SPI SS with Quad-IO SPI Mode");
	sys_write32(QSPI_SPI_MODE_QUAD + spi_ss, SCOBCA1_FPGA_FRAM_QSPI_ACR);

	LOG_DBG("* Send Memory Address (3byte)");
	write_mem_addr_to_flash(mem_addr);

	LOG_DBG("* Send Mode (0x00)");
	sys_write32(0x00, SCOBCA1_FPGA_FRAM_QSPI_TDR);

	/* Send Dummy Cycle */
	send_dummy_cycle(QSPI_NOR_FLASH_DUMMY_CYCLE_COUNT);
	if (!is_qspi_idle()) {
		ret = -ETIMEDOUT;
		goto end;
	}

	/* Read RX data */
	LOG_DBG("* Reqest RX FIFO %d byte", size);
	for (i = 0; i < size; i++) {
		sys_write32(0x00, SCOBCA1_FPGA_FRAM_QSPI_RDR);
	}

	if (!is_qspi_idle()) {
		ret = -EBUSY;
		goto end;
	}

	LOG_DBG("* Read RX FIFO %d byte", size);
	for (i = 0; i < size; i++) {
		val[i] = sys_read32(SCOBCA1_FPGA_FRAM_QSPI_RDR);
		LOG_DBG("Read Data: 0x%02x", val[i]);
	}

	/* Inactive SPI SS */
	if (!inactivate_spi_ss()) {
		ret = -EIO;
		goto end;
	}

end:
	return ret;
}

static int sc_fram_quad_write_data(uint32_t spi_ss, uint32_t mem_addr, uint8_t size, uint8_t *val)
{
	int ret = 0;

	if (!activate_spi_ss(spi_ss)) {
		ret = -EIO;
		goto end;
	}

	LOG_DBG("* Snd QUAD I/O Write instruction");
	sys_write32(0xD2, SCOBCA1_FPGA_FRAM_QSPI_TDR);
	if (!is_qspi_idle()) {
		ret = -EBUSY;
		goto end;
	}

	LOG_DBG("* Activate SPI SS with Quad-IO SPI Mode");
	sys_write32(QSPI_SPI_MODE_QUAD + spi_ss, SCOBCA1_FPGA_FRAM_QSPI_ACR);

	LOG_DBG("* Send Memory Address (3byte)");
	write_mem_addr_to_flash(mem_addr);

	LOG_DBG("* Send Mode (0x00)");
	sys_write32(0x00, SCOBCA1_FPGA_FRAM_QSPI_TDR);

	if (!is_qspi_idle()) {
		ret = -EBUSY;
		goto end;
	}

	/* Write data */
	write_data_to_flash(val, size);
	if (!is_qspi_idle()) {
		ret = -EBUSY;
		goto end;
	}

	if (!inactivate_spi_ss()) {
		ret = -EBUSY;
		goto end;
	}

end:
	return ret;
}

int sc_fram_write(uint8_t mem_no, uint32_t mem_addr, uint32_t size, uint8_t *val)
{
	int ret;
	uint32_t spi_ss;

	if (mem_no > 1) {
		LOG_ERR("Invalid Mem number %d (expected 0 or 1)", mem_no);
		ret = -EINVAL;
		goto end;
	}

	if (size > QSPI_FIFO_MAX_BYTE) {
		LOG_ERR("This API only supports up to %d bytes.", QSPI_FIFO_MAX_BYTE);
		ret = -EINVAL;
		goto end;
	}

	if (mem_no == SC_FRAM_MEM0) {
		spi_ss = QSPI_FRAM_MEM0_SS;
	} else {
		spi_ss = QSPI_FRAM_MEM1_SS;
	}

	LOG_DBG("* [#1] Set to `Write Enable'");
	if (!set_write_enable(spi_ss, true)) {
		ret = -EIO;
		goto end;
	}

	LOG_DBG("* [#2] Write Data (QUAD Mode)");
	ret = sc_fram_quad_write_data(spi_ss, mem_addr, size, val);

	LOG_DBG("* [#3] Set to `Write Disable'");
	if (!set_write_enable(spi_ss, false)) {
		LOG_ERR("Failed to disable `Write enable`");
		ret = -EIO;
	}

end:
	return ret;
}

int sc_fram_read(uint8_t mem_no, uint32_t mem_addr, uint32_t size, uint8_t *val)
{
	int ret;
	uint32_t spi_ss;

	if (mem_no > 1) {
		LOG_ERR("Invalid Mem number %d (expected 0 or 1)", mem_no);
		ret = -EINVAL;
		goto end;
	}

	if (size > QSPI_FIFO_MAX_BYTE) {
		LOG_ERR("This API only supports up to %d bytes.", QSPI_FIFO_MAX_BYTE);
		ret = -EINVAL;
		goto end;
	}

	if (mem_no == SC_FRAM_MEM0) {
		spi_ss = QSPI_FRAM_MEM0_SS;
	} else {
		spi_ss = QSPI_FRAM_MEM1_SS;
	}

	LOG_DBG("* [#1] Set QUAD-IO Read Mode");
	ret = sc_fram_set_quad_read_mode(spi_ss);
	if (ret < 0) {
		LOG_ERR("Failed to set QUAD-IO Read Mode (%d)", ret);
		goto end;
	}

	LOG_DBG("* [#2] Read Data (QUAD-IO Mode)");
	ret = sc_fram_quad_read_data(spi_ss, mem_addr, size, val);
	if (ret < 0) {
		LOG_ERR("Failed to read data (%d)", ret);
	}

end:
	return ret;
}

int sc_fram_clear_boot_count(void)
{
	int ret;
	uint8_t boot_count = 0;

	ret = sc_fram_write(SC_FRAM_MEM0, FRAM_BOOT_COUNT_ADDR, sizeof(boot_count), &boot_count);
	if (ret < 0) {
		LOG_ERR("Faild to clear the boot count (%d)", ret);
	}

	return ret;
}

int sc_fram_update_boot_count(void)
{
	int ret;
	uint8_t boot_count;

	ret = sc_fram_read(SC_FRAM_MEM0, FRAM_BOOT_COUNT_ADDR, sizeof(boot_count), &boot_count);
	if (ret < 0) {
		LOG_ERR("Faild to read the boot count (%d)", ret);
		goto end;
	}
	LOG_INF("FRAM read boot count:%u", boot_count);

	boot_count++;

	ret = sc_fram_write(SC_FRAM_MEM0, FRAM_BOOT_COUNT_ADDR, sizeof(boot_count), &boot_count);
	if (ret < 0) {
		LOG_ERR("Faild to write the boot count (%d)", ret);
		goto end;
	}

	LOG_INF("FRAM write new boot count %u", boot_count);

end:
	return ret;
}

int sc_fram_get_boot_count(uint8_t *boot_count)
{
	int ret;

	ret = sc_fram_read(SC_FRAM_MEM0, FRAM_BOOT_COUNT_ADDR, sizeof(*boot_count), boot_count);
	if (ret < 0) {
		LOG_ERR("Faild to get the boot count (%d)", ret);
	}

	return ret;
}

int sc_fram_update_crc_for_file(const char *fname, uint32_t crc32)
{
	int ret;
	uint32_t fram_addr;
	off_t offset = 0;

	fram_addr = FRAM_CRC_FILE_ADDR;

	for (int i = 0; i < FRAM_CRC_FNAME_MAX / QSPI_FIFO_MAX_BYTE; i++) {
		ret = sc_fram_write(SC_FRAM_MEM0, fram_addr, QSPI_FIFO_MAX_BYTE,
				    (uint8_t *)&fname[offset]);
		if (ret < 0) {
			LOG_ERR("Faild to write the CRC file name (fname: %s) (offset: %ld) (%d)",
				fname, offset, ret);
			goto end;
		}
		fram_addr += QSPI_FIFO_MAX_BYTE;
		offset += QSPI_FIFO_MAX_BYTE;
	}

	ret = sc_fram_write(SC_FRAM_MEM0, fram_addr, sizeof(crc32), (uint8_t *)&crc32);
	if (ret < 0) {
		LOG_ERR("Faild to write the CRC32 (fname: %s) (crc32: 0x%08x) (%d)", fname, crc32,
			ret);
		goto end;
	}

	LOG_INF("FRAM write CRC result (fname: %s, crc32 0x%08x)", fname, crc32);

end:
	return ret;
}

int sc_fram_get_crc_for_file(char *fname, uint32_t *crc32)
{
	int ret;
	uint32_t fram_addr;
	off_t offset = 0;

	fram_addr = FRAM_CRC_FILE_ADDR;

	for (int i = 0; i < FRAM_CRC_FNAME_MAX / QSPI_FIFO_MAX_BYTE; i++) {
		ret = sc_fram_read(SC_FRAM_MEM0, fram_addr, QSPI_FIFO_MAX_BYTE,
				   (uint8_t *)&fname[offset]);
		if (ret < 0) {
			LOG_ERR("Faild to read the CRC file name (offset: %ld) (%d)", offset, ret);
			goto end;
		}
		fram_addr += QSPI_FIFO_MAX_BYTE;
		offset += QSPI_FIFO_MAX_BYTE;
	}

	ret = sc_fram_read(SC_FRAM_MEM0, fram_addr, sizeof(*crc32), (uint8_t *)crc32);
	if (ret < 0) {
		LOG_ERR("Faild to write the CRC32 (fname: %s) (crc32: 0x%08x) (%d)", fname, *crc32,
			ret);
		goto end;
	}

	LOG_INF("FRAM read CRC result (fname: %s, crc32 0x%08x)", fname, *crc32);

end:
	return ret;
}

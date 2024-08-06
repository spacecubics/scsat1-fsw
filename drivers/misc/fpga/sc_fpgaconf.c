/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sc_fpgconf

#include "sc_fpgaconf.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sc_fpgaconf, CONFIG_SC_FPGACONF_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>

#define FPGACONF_BASE DT_REG_ADDR_BY_IDX(DT_INST(0, sc_fpgaconf), 0)

#define FPGACONF_VERSION (FPGACONF_BASE + 0x00)
#define FPGACONF_ADDR    (FPGACONF_BASE + 0x04)
#define FPGACONF_DATA    (FPGACONF_BASE + 0x08)
#define FPGACONF_CLK     (FPGACONF_BASE + 0x0c)

#define FPGACONF_MAGIC (0x5a5a0000)

#define FPGACONF_VERSION_MAJOR(x) ((x) >> 24)
#define FPGACONF_VERSION_MINOR(x) (((x) << 8) >> 24)
#define FPGACONF_VERSION_PATCH(x) ((x) & 16)

#define FPGACONF_CRC      (0x00)
#define FPGACONF_FAR      (0x01)
#define FPGACONF_FDRI     (0x02)
#define FPGACONF_FDRO     (0x03)
#define FPGACONF_CMD      (0x04)
#define FPGACONF_CTL0     (0x05)
#define FPGACONF_MASK     (0x06)
#define FPGACONF_STAT     (0x07)
#define FPGACONF_LOUT     (0x08)
#define FPGACONF_COR0     (0x09)
#define FPGACONF_MFWR     (0x0a)
#define FPGACONF_CBC      (0x0b)
#define FPGACONF_IDCODE   (0x0c)
#define FPGACONF_AXSS     (0x0d)
#define FPGACONF_COR1     (0x0e)
#define FPGACONF_WBSTR    (0x10)
#define FPGACONF_TIMER    (0x11)
#define FPGACONF_RBCRC_SW (0x13)
#define FPGACONF_BOOTSTS  (0x16)

#define BOOTSTS_VALID      BIT(0)
#define BOOTSTS_FALLBACK   BIT(1)
#define BOOTSTS_IPROG      BIT(2)
#define BOOTSTS_WTO_ERROR  BIT(3)
#define BOOTSTS_ID_ERROR   BIT(4)
#define BOOTSTS_CRC_ERROR  BIT(5)
#define BOOTSTS_WRAP_ERROR BIT(6)

static void sc_fpgaconf_enable_clock(void)
{
	sys_write32(FPGACONF_MAGIC, FPGACONF_CLK);
}

static uint32_t sc_fpgaconf_get_version(void)
{
	return sys_read32(FPGACONF_VERSION);
}

static inline uint32_t sc_fpgaconf_get(uint32_t reg)
{
	sys_write32(reg, FPGACONF_ADDR);
	return sys_read32(FPGACONF_DATA);
}

uint32_t sc_fpgaconf_get_crc(void)
{
	return sc_fpgaconf_get(FPGACONF_CRC);
}
uint32_t sc_fpgaconf_get_far(void)
{
	return sc_fpgaconf_get(FPGACONF_FAR);
}
uint32_t sc_fpgaconf_get_fdro(void)
{
	return sc_fpgaconf_get(FPGACONF_FDRO);
}
uint32_t sc_fpgaconf_get_cmd(void)
{
	return sc_fpgaconf_get(FPGACONF_CMD);
}
uint32_t sc_fpgaconf_get_ctl0(void)
{
	return sc_fpgaconf_get(FPGACONF_CTL0);
}
uint32_t sc_fpgaconf_get_mask(void)
{
	return sc_fpgaconf_get(FPGACONF_MASK);
}
uint32_t sc_fpgaconf_get_stat(void)
{
	return sc_fpgaconf_get(FPGACONF_STAT);
}
uint32_t sc_fpgaconf_get_cor0(void)
{
	return sc_fpgaconf_get(FPGACONF_COR0);
}
uint32_t sc_fpgaconf_get_idcode(void)
{
	return sc_fpgaconf_get(FPGACONF_IDCODE);
}
uint32_t sc_fpgaconf_get_axss(void)
{
	return sc_fpgaconf_get(FPGACONF_AXSS);
}
uint32_t sc_fpgaconf_get_cor1(void)
{
	return sc_fpgaconf_get(FPGACONF_COR1);
}
uint32_t sc_fpgaconf_get_wbstr(void)
{
	return sc_fpgaconf_get(FPGACONF_WBSTR);
}
uint32_t sc_fpgaconf_get_timer(void)
{
	return sc_fpgaconf_get(FPGACONF_TIMER);
}
uint32_t sc_fpgaconf_get_rbcrc_sw(void)
{
	return sc_fpgaconf_get(FPGACONF_RBCRC_SW);
}
uint32_t sc_fpgaconf_get_bootsts(void)
{
	return sc_fpgaconf_get(FPGACONF_BOOTSTS);
}

static size_t sc_fpgaconf_get_boot_status(uint8_t reg, int nr, char *buf, size_t buflen)
{
	size_t len = 0;

	if (reg & BOOTSTS_VALID) {
		if (len < buflen && (reg & BOOTSTS_FALLBACK)) {
			len += snprintf(buf + len, buflen - len, " fallback%d", nr);
		}
		if (len < buflen && (reg & BOOTSTS_IPROG)) {
			len += snprintf(buf + len, buflen - len, " iprog%d", nr);
		}
		if (len < buflen && (reg & BOOTSTS_WTO_ERROR)) {
			len += snprintf(buf + len, buflen - len, " wto%d", nr);
		}
		if (len < buflen && (reg & BOOTSTS_ID_ERROR)) {
			len += snprintf(buf + len, buflen - len, " id%d", nr);
		}
		if (len < buflen && (reg & BOOTSTS_CRC_ERROR)) {
			len += snprintf(buf + len, buflen - len, " crc%d", nr);
		}
		if (len < buflen && (reg & BOOTSTS_WRAP_ERROR)) {
			len += snprintf(buf + len, buflen - len, " wrap%d", nr);
		}
	}

	return len;
}

/*
 * Checks if the boot status is the fallback mode.
 *
 * Returns:
 *   - true if the BOOTSTS_FALLBACK bit is set.
 *   - false otherwise.
 */
bool sc_fpgaconf_is_fallback(void)
{
	uint32_t val = sc_fpgaconf_get(FPGACONF_BOOTSTS);
	return (val & BOOTSTS_FALLBACK) != 0;
}

static int sc_fpgaconf_init(void)
{
	sc_fpgaconf_enable_clock();

	/* The first read is invalid. Throw it away */
	(void)sc_fpgaconf_get_bootsts();

	if (IS_ENABLED(CONFIG_SC_FPGACONF_PRINT_VERSION)) {
		uint32_t reg;

		reg = sc_fpgaconf_get_version();

		LOG_INF("v%d.%d.%d", FPGACONF_VERSION_MAJOR(reg), FPGACONF_VERSION_MINOR(reg),
			FPGACONF_VERSION_PATCH(reg));
	}

	if (IS_ENABLED(CONFIG_SC_FPGACONF_PRINT_BOOT_STATUS)) {
		size_t len;
		uint32_t status;
		char buf[128] = {0};

		status = sc_fpgaconf_get_bootsts();
		len = sc_fpgaconf_get_boot_status(status & 0xff, 0, buf, sizeof(buf));
		sc_fpgaconf_get_boot_status((status >> 8) & 0xff, 1, buf + len, sizeof(buf) - len);

		LOG_INF("Boot Status: 0x%x%s", status, buf);
	}

	return 0;
}

SYS_INIT(sc_fpgaconf_init, POST_KERNEL, CONFIG_SC_FPGACONF_INIT_PRIORITY);

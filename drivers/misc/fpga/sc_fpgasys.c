/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sc_fpgconf

#include "sc_fpgasys.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sc_fpgasys, CONFIG_SC_FPGASYS_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>

#define SCOBCA1_SYSREG_BASE DT_REG_ADDR_BY_IDX(DT_INST(0, sc_fpgasys), 0)

#define SCOBCA1_SYSREG_CODEMSEL  (SCOBCA1_SYSREG_BASE + 0x0000)
#define SCOBCA1_SYSREG_SYSCLKCT  (SCOBCA1_SYSREG_BASE + 0x0004)
#define SCOBCA1_SYSREG_PLLINFO   (SCOBCA1_SYSREG_BASE + 0x0008)
#define SCOBCA1_SYSREG_CFGMEMCTL (SCOBCA1_SYSREG_BASE + 0x0010)
#define SCOBCA1_SYSREG_PWRCYCLE  (SCOBCA1_SYSREG_BASE + 0x0020)
#define SCOBCA1_SYSREG_PWRMANAGE (SCOBCA1_SYSREG_BASE + 0x0030)
#define SCOBCA1_SYSREG_SPAD1     (SCOBCA1_SYSREG_BASE + 0x00F0)
#define SCOBCA1_SYSREG_SPAD2     (SCOBCA1_SYSREG_BASE + 0x00F4)
#define SCOBCA1_SYSREG_SPAD3     (SCOBCA1_SYSREG_BASE + 0x00F8)
#define SCOBCA1_SYSREG_SPAD4     (SCOBCA1_SYSREG_BASE + 0x00FC)
#define SCOBCA1_SYSREG_VER       (SCOBCA1_SYSREG_BASE + 0xF000)
#define SCOBCA1_SYSREG_BUILDINFO (SCOBCA1_SYSREG_BASE + 0xFF00)
#define SCOBCA1_SYSREG_DNA1      (SCOBCA1_SYSREG_BASE + 0xFF10)
#define SCOBCA1_SYSREG_DNA2      (SCOBCA1_SYSREG_BASE + 0xFF14)

#define SCOBCA1_SYSREG_CFGBOOTMEM(x) ((((x) & BIT(12)) >> 12))
#define SCOBCA1_SYSREG_CFGMEMSEL(x)  ((((x) & BIT(0)) << 4))
#define SCOBCA1_SYSREG_CFGMEMMON(x)  ((((x) & BIT(5)) >> 5))

#define SCOBCA1_SYSREG_MAGIC (0x5a5a0000)

#define CFG_MEMSEL_RETRY_COUNT    (100U)
#define CFG_MEMSEL_RETRY_INTERVAL K_MSEC(1)

enum sc_cfgmem sc_get_boot_cfgmem(void)
{
	return SCOBCA1_SYSREG_CFGBOOTMEM(sys_read32(SCOBCA1_SYSREG_CFGMEMCTL));
}

enum sc_cfgmem sc_get_cfgmem(void)
{
	return SCOBCA1_SYSREG_CFGMEMMON(sys_read32(SCOBCA1_SYSREG_CFGMEMCTL));
}

int sc_select_cfgmem(enum sc_cfgmem mem)
{
	int ret = -1;
	uint16_t retry = CFG_MEMSEL_RETRY_COUNT;
	enum sc_cfgmem sel_mem;

	sys_write32(SCOBCA1_SYSREG_CFGMEMSEL(mem), SCOBCA1_SYSREG_CFGMEMCTL);

	while (retry--) {
		sel_mem = sc_get_cfgmem();
		if (sel_mem == mem) {
			ret = 0;
			break;
		}

		k_sleep(CFG_MEMSEL_RETRY_INTERVAL);
	}

	return ret;
}

void sc_select_codemem(enum sc_codemem mem)
{
	sys_write32(SCOBCA1_SYSREG_MAGIC | mem, SCOBCA1_SYSREG_CODEMSEL);
}

uint32_t sc_get_fpga_ip_ver(void)
{
	return sys_read32(SCOBCA1_SYSREG_VER);
}

uint32_t sc_get_fpga_build_hash(void)
{
	return sys_read32(SCOBCA1_SYSREG_BUILDINFO);
}

uint32_t sc_get_fpga_dna_1(void)
{
	return sys_read32(SCOBCA1_SYSREG_DNA1);
}

uint32_t sc_get_fpga_dna_2(void)
{
	return sys_read32(SCOBCA1_SYSREG_DNA1);
}

static int sc_fpgasys_init(void)
{
	return 0;
}

SYS_INIT(sc_fpgasys_init, POST_KERNEL, CONFIG_SC_FPGASYS_INIT_PRIORITY);

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sc_fpgahrmem

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sc_fpgahrmem, CONFIG_SC_FPGAHRMEM_LOG_LEVEL);

#define SCOBCA1_HRMEMREG_BASE DT_REG_ADDR_BY_IDX(DT_INST(0, sc_fpgahrmem), 0)

#define SCOBCA1_HRMEM_ECCCOLENR     (SCOBCA1_HRMEMREG_BASE + 0x0000)
#define SCOBCA1_HRMEM_MEMSCRCTRLR   (SCOBCA1_HRMEMREG_BASE + 0x0008)
#define SCOBCA1_HRMEM_INTSTR        (SCOBCA1_HRMEMREG_BASE + 0x0010)
#define SCOBCA1_HRMEM_INTENR        (SCOBCA1_HRMEMREG_BASE + 0x0014)
#define SCOBCA1_HRMEM_ECC1ERRCNTR   (SCOBCA1_HRMEMREG_BASE + 0x0020)
#define SCOBCA1_HRMEM_ECDISCNTR     (SCOBCA1_HRMEMREG_BASE + 0x0028)
#define SCOBCA1_HRMEM_ERRCNTCLRR    (SCOBCA1_HRMEMREG_BASE + 0x002C)
#define SCOBCA1_HRMEM_ECCERRADMR    (SCOBCA1_HRMEMREG_BASE + 0x0030)
#define SCOBCA1_HRMEM_PFEMDCTLR     (SCOBCA1_HRMEMREG_BASE + 0x0070)
#define SCOBCA1_HRMEM_SPEPFENR      (SCOBCA1_HRMEMREG_BASE + 0x0074)
#define SCOBCA1_HRMEM_SPEPFADRSETR1 (SCOBCA1_HRMEMREG_BASE + 0x0080)
#define SCOBCA1_HRMEM_SPEPFADRSETR2 (SCOBCA1_HRMEMREG_BASE + 0x0084)
#define SCOBCA1_HRMEM_VER           (SCOBCA1_HRMEMREG_BASE + 0xF000)

#define SCOBCA1_HRMEM_ECCCOLEN      BIT(0)
#define SCOBCA1_HRMEM_MEMSCRBEN     BIT(0)
#define SCOBCA1_HRMEM_COLFSRDSTPBEN BIT(8)
#define SCOBCA1_HRMEM_MEMSCRCYC(x)     (((x) & GENMASK(31, 16)) >> 16)
#define SCOBCA1_HRMEM_ATRDE1ERRCNT(x)  (((x) & GENMASK(31, 16)) >> 16)
#define SCOBCA1_HRMEM_BUSRDE1ERRCNT(x) (((x) & GENMASK(15, 0)))
#define SCOBCA1_HRMEM_ECDISCNT(x)      (((x) & GENMASK(15, 0)))
#define SCOBCA1_HRMEM_ECCERRADDR(x)    (((x) & GENMASK(21, 0)))

void sc_hrmem_enable_ecc_collect(void)
{
	sys_set_bits(SCOBCA1_HRMEM_ECCCOLENR, SCOBCA1_HRMEM_ECCCOLEN);
}

void sc_hrmem_disable_ecc_collect(void)
{
	sys_clear_bits(SCOBCA1_HRMEM_ECCCOLENR, SCOBCA1_HRMEM_ECCCOLEN);
}

bool sc_hrmem_is_enable_ecc_collect(void)
{
	uint32_t val;

	val = sys_read32(SCOBCA1_HRMEM_ECCCOLENR);
	return (val & SCOBCA1_HRMEM_ECCCOLEN) != 0;
}

void sc_hrmem_enable_memory_scrub(void)
{
	sys_set_bits(SCOBCA1_HRMEM_MEMSCRCTRLR, SCOBCA1_HRMEM_MEMSCRBEN);
}

void sc_hrmem_disable_memory_scrub(void)
{
	sys_clear_bits(SCOBCA1_HRMEM_MEMSCRCTRLR, SCOBCA1_HRMEM_MEMSCRBEN);
}

bool sc_hrmem_is_enable_memory_scrub(void)
{
	uint32_t val;

	val = sys_read32(SCOBCA1_HRMEM_MEMSCRCTRLR);
	return (val & SCOBCA1_HRMEM_MEMSCRBEN) != 0;
}

uint16_t sc_hrmem_get_memscrub_cycle(void)
{
	uint32_t val;

	val = sys_read32(SCOBCA1_HRMEM_MEMSCRCTRLR);
	return SCOBCA1_HRMEM_MEMSCRCYC(val);
}

void sc_hrmem_set_memscrub_cycle(uint16_t cycle)
{
	uint32_t val;

	val = sys_read32(SCOBCA1_HRMEM_MEMSCRCTRLR);
	sys_write32((val & 0x0000FFFF) | ((uint32_t)cycle << 16), SCOBCA1_HRMEM_MEMSCRCTRLR);
}

bool sc_hrmem_is_memscrub_arbitration(void)
{
	uint32_t val;

	val = sys_read32(SCOBCA1_HRMEM_MEMSCRCTRLR);
	return (val & SCOBCA1_HRMEM_COLFSRDSTPBEN) != 0;
}

uint16_t sc_hrmem_get_ecc_error_discard(void)
{
	uint32_t val;

	val = sys_read32(SCOBCA1_HRMEM_ECDISCNTR);
	return SCOBCA1_HRMEM_ECDISCNT(val);
}

uint32_t sc_hrmem_get_ecc_error_address(void)
{
	uint32_t val;

	val = sys_read32(SCOBCA1_HRMEM_ECCERRADMR);
	return SCOBCA1_HRMEM_ECCERRADDR(val);
}

int sc_hrmem_get_ecc_error_count(uint16_t *by_auto, uint16_t *by_bus)
{
	uint32_t val;

	val = sys_read32(SCOBCA1_HRMEM_ECC1ERRCNTR);
	*by_auto = SCOBCA1_HRMEM_ATRDE1ERRCNT(val);
	*by_bus = SCOBCA1_HRMEM_BUSRDE1ERRCNT(val);

	return 0;
}

static int sc_fpgahrmem_init(void)
{
	return 0;
}

SYS_INIT(sc_fpgahrmem_init, POST_KERNEL, CONFIG_SC_FPGAHRMEM_INIT_PRIORITY);

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sc_fpgmon

#include "sc_fpgamon.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sc_fpgamon, CONFIG_SC_FPGAMON_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>

#define SCOBCA1_SYSMON_BASE DT_REG_ADDR_BY_IDX(DT_INST(0, sc_fpgamon), 0)
#define SCOBCA1_GPTMR_BASE  (0x4F050000)

#define SCOBCA1_SYSMON_WDOG_CTRL     (SCOBCA1_SYSMON_BASE + 0x0000)
#define SCOBCA1_SYSMON_CLOCK_STATE   (SCOBCA1_SYSMON_BASE + 0x0020)
#define SCOBCA1_SYSMON_ISR           (SCOBCA1_SYSMON_BASE + 0x0030)
#define SCOBCA1_SYSMON_SEM_STATE     (SCOBCA1_SYSMON_BASE + 0x0040)
#define SCOBCA1_SYSMON_SEM_ECCOUNT   (SCOBCA1_SYSMON_BASE + 0x0044)
#define SCOBCA1_SYSMON_SEM_HTIMEOUT  (SCOBCA1_SYSMON_BASE + 0x0048)
#define SCOBCA1_SYSMON_XADC_TEMP     (SCOBCA1_SYSMON_BASE + 0x1000)
#define SCOBCA1_SYSMON_XADC_VCCINT   (SCOBCA1_SYSMON_BASE + 0x1010)
#define SCOBCA1_SYSMON_XADC_VCCAUX   (SCOBCA1_SYSMON_BASE + 0x1020)
#define SCOBCA1_SYSMON_XADC_VCCBRAM  (SCOBCA1_SYSMON_BASE + 0x1060)
#define SCOBCA1_SYSMON_INICTLR       (SCOBCA1_SYSMON_BASE + 0x2000)
#define SCOBCA1_SYSMON_ACCCTLR       (SCOBCA1_SYSMON_BASE + 0x2004)
#define SCOBCA1_SYSMON_BSM_ISR       (SCOBCA1_SYSMON_BASE + 0x2010)
#define SCOBCA1_SYSMON_1V0_SNTVR     (SCOBCA1_SYSMON_BASE + 0x2020)
#define SCOBCA1_SYSMON_1V0_BUSVR     (SCOBCA1_SYSMON_BASE + 0x2024)
#define SCOBCA1_SYSMON_1V8_SNTVR     (SCOBCA1_SYSMON_BASE + 0x2028)
#define SCOBCA1_SYSMON_1V8_BUSVR     (SCOBCA1_SYSMON_BASE + 0x202C)
#define SCOBCA1_SYSMON_3V3_SNTVR     (SCOBCA1_SYSMON_BASE + 0x2030)
#define SCOBCA1_SYSMON_3V3_BUSVR     (SCOBCA1_SYSMON_BASE + 0x2034)
#define SCOBCA1_SYSMON_3V3SYSA_SNTVR (SCOBCA1_SYSMON_BASE + 0x2038)
#define SCOBCA1_SYSMON_3V3SYSA_BUSVR (SCOBCA1_SYSMON_BASE + 0x203C)
#define SCOBCA1_SYSMON_3V3SYSB_SNTVR (SCOBCA1_SYSMON_BASE + 0x2040)
#define SCOBCA1_SYSMON_3V3SYSB_BUSVR (SCOBCA1_SYSMON_BASE + 0x2044)
#define SCOBCA1_SYSMON_3V3IO_SNTVR   (SCOBCA1_SYSMON_BASE + 0x2048)
#define SCOBCA1_SYSMON_3V3IO_BUSVR   (SCOBCA1_SYSMON_BASE + 0x204C)
#define SCOBCA1_SYSMON_TEMP1R        (SCOBCA1_SYSMON_BASE + 0x2050)
#define SCOBCA1_SYSMON_TEMP2R        (SCOBCA1_SYSMON_BASE + 0x2054)
#define SCOBCA1_SYSMON_TEMP3R        (SCOBCA1_SYSMON_BASE + 0x2058)

#define SCOBCA1_SYSMON_CFGMEMCTL_OFFSET (0x0010)
#define SCOBCA1_SYSMON_VER_OFFSET       (0xF000)
#define SCOBCA1_SYSMON_BUILDINFO_OFFSET (0xFF00)
#define SCOBCA1_SYSMON_DNA1_OFFSET      (0xFF10)
#define SCOBCA1_SYSMON_DNA2_OFFSET      (0xFF14)

#define SCOBCA1_GPTMR_TECR    (SCOBCA1_GPTMR_BASE + 0x0004)
#define SCOBCA1_GPTMR_HITCR   (SCOBCA1_GPTMR_BASE + 0x0200)
#define SCOBCA1_GPTMR_HITPR   (SCOBCA1_GPTMR_BASE + 0x0204)
#define SCOBCA1_GPTMR_HITOCR1 (SCOBCA1_GPTMR_BASE + 0x0210)
#define SCOBCA1_GPTMR_HITOCR2 (SCOBCA1_GPTMR_BASE + 0x0214)
#define SCOBCA1_GPTMR_HITOCR3 (SCOBCA1_GPTMR_BASE + 0x0218)

/* SEM Error Correction Count Register */
#define SCOBCA1_SYSMON_SEMCCOUNT(x) (((x) & GENMASK(15, 0)))

/* BHM Initialization Access Control Register */
#define SCOBCA1_SYSMON_INIT_REQ     BIT(16)
#define SCOBCA1_SYSMON_TEMP3_INITEN BIT(4)
#define SCOBCA1_SYSMON_TEMP2_INITEN BIT(3)
#define SCOBCA1_SYSMON_TEMP1_INITEN BIT(2)
#define SCOBCA1_SYSMON_CVM2_INITEN  BIT(1)
#define SCOBCA1_SYSMON_CVM1_INITEN  BIT(0)
#define SCOBCA1_SYSMON_INIT_ALL                                                                    \
	(SCOBCA1_SYSMON_INIT_REQ | SCOBCA1_SYSMON_TEMP3_INITEN | SCOBCA1_SYSMON_TEMP2_INITEN |     \
	 SCOBCA1_SYSMON_TEMP1_INITEN | SCOBCA1_SYSMON_CVM2_INITEN | SCOBCA1_SYSMON_CVM1_INITEN)

/* BHM Access Control Register */
#define SCOBCA1_SYSMON_TEMP3_MONIEN BIT(4)
#define SCOBCA1_SYSMON_TEMP2_MONIEN BIT(3)
#define SCOBCA1_SYSMON_TEMP1_MONIEN BIT(2)
#define SCOBCA1_SYSMON_CVM2_MONIEN  BIT(1)
#define SCOBCA1_SYSMON_CVM1_MONIEN  BIT(0)
#define SCOBCA1_SYSMON_MONIEN_ALL                                                                  \
	(SCOBCA1_SYSMON_TEMP3_MONIEN | SCOBCA1_SYSMON_TEMP2_MONIEN | SCOBCA1_SYSMON_TEMP1_MONIEN | \
	 SCOBCA1_SYSMON_CVM2_MONIEN | SCOBCA1_SYSMON_CVM1_MONIEN)

/* BHM Interrupt Status Register */
#define SCOBCA1_SYSMON_INIT_ACCEND BIT(0)

/* Timer Enable Control Register */
#define SCOBCA1_SYSMON_GPTMR_HITEN BIT(1)

#define SYSMON_RETRY_COUNT (100U)

static float convert_obc_temp(uint32_t raw)
{
	int8_t int_tmp = (raw & 0x0000FF00) >> 8;
	float point = ((raw & 0x000000F0) >> 4) * 0.0625;

	return int_tmp + point;
}

static float convert_xadc_temp(uint32_t raw)
{
	float conv_tmp;

	conv_tmp = ((((raw & 0x0000FFF0) >> 4) * 503.975) / 4096) - 273.15;

	return conv_tmp;
}

static uint32_t convert_cv_shunt(int16_t raw)
{
	return ((raw >> 3) * 40);
}

static uint32_t convert_cv_bus(int16_t raw)
{
	return ((raw >> 3) * 8);
}

static float convert_cv_xadc(uint32_t raw)
{
	float conv_cv;

	conv_cv = (raw & 0x0000FFF0) >> 4;

	return conv_cv / 4096 * 3;
}

static int convert_obc_cv(enum obc_cv_pos pos, uint32_t raw, int32_t *cv)
{
	int ret = 0;
	int16_t rawval = raw & 0x0000FFFF;

	switch (pos) {
	case OBC_1V0_SHUNT:
	case OBC_1V8_SHUNT:
	case OBC_3V3_SHUNT:
	case OBC_3V3_SYSA_SHUNT:
	case OBC_3V3_SYSB_SHUNT:
	case OBC_3V3_IO_SHUNT:
		*cv = convert_cv_shunt(rawval);
		break;
	case OBC_1V0_BUS:
	case OBC_1V8_BUS:
	case OBC_3V3_BUS:
	case OBC_3V3_SYSA_BUS:
	case OBC_3V3_SYSB_BUS:
	case OBC_3V3_IO_BUS:
		*cv = convert_cv_bus(rawval);
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static int get_obc_temp_register_addr(enum obc_temp_pos pos, uint32_t *addr)
{
	int ret = 0;

	switch (pos) {
	case OBC_TEMP_1:
		*addr = SCOBCA1_SYSMON_TEMP1R;
		break;
	case OBC_TEMP_2:
		*addr = SCOBCA1_SYSMON_TEMP2R;
		break;
	case OBC_TEMP_3:
		*addr = SCOBCA1_SYSMON_TEMP3R;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static int get_obc_cv_register_addr(enum obc_cv_pos pos, uint32_t *addr)
{
	int ret = 0;

	switch (pos) {
	case OBC_1V0_SHUNT:
		*addr = SCOBCA1_SYSMON_1V0_SNTVR;
		break;
	case OBC_1V0_BUS:
		*addr = SCOBCA1_SYSMON_1V0_BUSVR;
		break;
	case OBC_1V8_SHUNT:
		*addr = SCOBCA1_SYSMON_1V8_SNTVR;
		break;
	case OBC_1V8_BUS:
		*addr = SCOBCA1_SYSMON_1V8_BUSVR;
		break;
	case OBC_3V3_SHUNT:
		*addr = SCOBCA1_SYSMON_3V3_SNTVR;
		break;
	case OBC_3V3_BUS:
		*addr = SCOBCA1_SYSMON_3V3_BUSVR;
		break;
	case OBC_3V3_SYSA_SHUNT:
		*addr = SCOBCA1_SYSMON_3V3SYSA_SNTVR;
		break;
	case OBC_3V3_SYSA_BUS:
		*addr = SCOBCA1_SYSMON_3V3SYSA_BUSVR;
		break;
	case OBC_3V3_SYSB_SHUNT:
		*addr = SCOBCA1_SYSMON_3V3SYSB_SNTVR;
		break;
	case OBC_3V3_SYSB_BUS:
		*addr = SCOBCA1_SYSMON_3V3SYSB_BUSVR;
		break;
	case OBC_3V3_IO_SHUNT:
		*addr = SCOBCA1_SYSMON_3V3IO_SNTVR;
		break;
	case OBC_3V3_IO_BUS:
		*addr = SCOBCA1_SYSMON_3V3IO_BUSVR;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static int get_xadc_register_addr(enum xadc_cv_pos pos, uint32_t *addr)
{
	int ret = 0;

	switch (pos) {
	case OBC_XADC_VCCINT:
		*addr = SCOBCA1_SYSMON_XADC_VCCINT;
		break;
	case OBC_XADC_VCCAUX:
		*addr = SCOBCA1_SYSMON_XADC_VCCAUX;
		break;
	case OBC_XADC_VCCBRAM:
		*addr = SCOBCA1_SYSMON_XADC_VCCBRAM;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static int wait_for_bhm_complete(uint32_t target_bit)
{
	int ret = 0;
	int i;
	uint32_t reg;

	/* Wait for any processing to be complete */
	for (i = 0; i < SYSMON_RETRY_COUNT; i++) {
		reg = sys_read32(SCOBCA1_SYSMON_BSM_ISR);
		if ((reg & target_bit) == target_bit) {
			sys_write32(target_bit, SCOBCA1_SYSMON_BSM_ISR);
			break;
		}
		k_sleep(K_MSEC(1));
	}

	if (i == SYSMON_RETRY_COUNT) {
		ret = -ETIMEDOUT;
	}

	return ret;
}

static int sc_bhm_init(void)
{
	sys_set_bits(SCOBCA1_SYSMON_INICTLR, SCOBCA1_SYSMON_INIT_ALL);

	return wait_for_bhm_complete(SCOBCA1_SYSMON_INIT_ACCEND);
}

static void sc_bhm_set_timer(void)
{
	/*
	 * This value is using the recommended value from the FPGA technical
	 * reference manual.
	 */
	sys_write32(0x00280000, SCOBCA1_GPTMR_HITCR);
	sys_write32(0x095F, SCOBCA1_GPTMR_HITPR);
	sys_write32(0x0050, SCOBCA1_GPTMR_HITOCR1);
	sys_write32(0x0001, SCOBCA1_GPTMR_HITOCR2);
	sys_write32(0x0040, SCOBCA1_GPTMR_HITOCR3);
}

static inline void sc_bhm_monitor_enable(void)
{
	sys_set_bits(SCOBCA1_SYSMON_ACCCTLR, SCOBCA1_SYSMON_MONIEN_ALL);
}

static inline void sc_bhm_monitor_disable(void)
{
	sys_clear_bits(SCOBCA1_SYSMON_ACCCTLR, SCOBCA1_SYSMON_MONIEN_ALL);
}

static inline void sc_bhm_timer_enable(void)
{
	sys_set_bits(SCOBCA1_GPTMR_TECR, SCOBCA1_SYSMON_GPTMR_HITEN);
}

static inline void sc_bhm_timer_disable(void)
{
	sys_clear_bits(SCOBCA1_GPTMR_TECR, SCOBCA1_SYSMON_GPTMR_HITEN);
}

void sc_kick_wdt_timer(void)
{
	uint32_t reg;

	reg = sys_read32(SCOBCA1_SYSMON_WDOG_CTRL);
	sys_write32(reg, SCOBCA1_SYSMON_WDOG_CTRL);
}

int sc_bhm_enable(void)
{
	int ret;

	ret = sc_bhm_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize the BHM. (%d)", ret);
		goto end;
	}

	sc_bhm_set_timer();

	sc_bhm_monitor_enable();

	sc_bhm_timer_enable();
end:
	return ret;
}

int sc_bhm_get_obc_temp(enum obc_temp_pos pos, float *temp)
{
	int ret;
	uint32_t addr;
	uint32_t raw;

	ret = get_obc_temp_register_addr(pos, &addr);
	if (ret < 0) {
		goto end;
	}

	raw = sys_read32(addr);
	*temp = convert_obc_temp(raw);
end:
	return ret;
}

int sc_bhm_get_obc_cv(enum obc_cv_pos pos, int32_t *cv)
{
	int ret;
	uint32_t addr;
	uint32_t raw;

	ret = get_obc_cv_register_addr(pos, &addr);
	if (ret < 0) {
		goto end;
	}

	raw = sys_read32(addr);
	ret = convert_obc_cv(pos, raw, cv);
end:
	return ret;
}

int sc_bhm_get_xadc_temp(float *temp)
{
	uint32_t raw;

	raw = sys_read32(SCOBCA1_SYSMON_XADC_TEMP);
	*temp = convert_xadc_temp(raw);

	return 0;
}

int sc_bhm_get_xadc_cv(enum xadc_cv_pos pos, float *cv)
{
	int ret;
	uint32_t addr;
	uint32_t raw;

	ret = get_xadc_register_addr(pos, &addr);
	if (ret < 0) {
		goto end;
	}

	raw = sys_read32(addr);
	*cv = convert_cv_xadc(raw);
end:
	return ret;
}

int sc_bhm_disable(void)
{
	return 0;
}

int sc_sem_get_error_count(uint16_t *count)
{
	uint32_t val;

	val = sys_read32(SCOBCA1_SYSMON_SEM_ECCOUNT);
	*count = SCOBCA1_SYSMON_SEMCCOUNT(val);

	return 0;
}

uint32_t sc_sem_get_controller_state(void)
{
	return sys_read32(SCOBCA1_SYSMON_SEM_STATE);
}

uint8_t sc_sem_get_heartbeat_timeout(void)
{
	return sys_read32(SCOBCA1_SYSMON_SEM_HTIMEOUT) & 0xFF;
}

uint32_t sc_clock_get_status(void)
{
	return sys_read32(SCOBCA1_SYSMON_CLOCK_STATE);
}

uint32_t sc_sysmon_get_isr(void)
{
	return sys_read32(SCOBCA1_SYSMON_ISR);
}

static int sc_fpgamon_init(void)
{
	return 0;
}

SYS_INIT(sc_fpgamon_init, POST_KERNEL, CONFIG_SC_FPGAMON_INIT_PRIORITY);

/*
 * Copyright (c) 2023 Space Cubics,LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "sysmon.h"
#include "version.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sysmon);

/* Registers */
#define SC_ADCS_SYSREG_BASE_ADDR (0x4F000000) /* System Register */
#define SC_ADCS_SYSMON_BASE_ADDR (0x4F040000) /* System Monitor */
#define SC_ADCS_GPT_BASE_ADDR    (0x4F050000) /* General Purpose Timer */

#define SC_ADCS_SYSREG_VER_OFFSET       (0xF000)
#define SC_ADCS_SYSREG_BUILDINFO_OFFSET (0xFF00)
#define SC_ADCS_SYSREG_DNA1_OFFSET      (0xFF10)
#define SC_ADCS_SYSREG_DNA2_OFFSET      (0xFF14)

#define SC_ADCS_SYSMON_WDOG_CTRL_OFFSET    (0x0000) /* Watchdog Control Register */
#define SC_ADCS_SYSMON_XADC_TEMP_OFFSET    (0x1000) /* XADC Temperature Status */
#define SC_ADCS_SYSMON_XADC_VCCINT_OFFSET  (0x1010) /* XADC VCCINT Status */
#define SC_ADCS_SYSMON_XADC_VCCAUX_OFFSET  (0x1020) /* XADC VCCAUX Status */
#define SC_ADCS_SYSMON_XADC_VCCBRAM_OFFSET (0x1060) /* XADC VCCBRAM Status */
#define SC_ADCS_SYSMON_BHM_INICTLR_OFFSET                                                          \
	(0x2000)                                   /* BHM Initialization Access Control Register   \
						    */
#define SC_ADCS_SYSMON_BHM_ACCCTLR_OFFSET (0x2004) /* BHM Access Control Register */
#define SC_ADCS_SYSMON_BHM_ISR_OFFSET     (0x2010) /* BHM Interrupt Status Register */
#define SC_ADCS_SYSMON_BHM_1V0_SNTVR_OFFSET                                                        \
	(0x2020) /* BHM VDD_1V0 Shunt Voltage Monitor Register */
#define SC_ADCS_SYSMON_BHM_1V0_BUSVR_OFFSET                                                        \
	(0x2024) /* BHM VDD_1V0 Bus Voltage Monitor Register                                       \
		  */
#define SC_ADCS_SYSMON_BHM_1V8_SNTVR_OFFSET                                                        \
	(0x2028) /* BHM VDD_1V8 Shunt Voltage Monitor Register */
#define SC_ADCS_SYSMON_BHM_1V8_BUSVR_OFFSET                                                        \
	(0x202C) /* BHM VDD_1V8 Bus Voltage Monitor Register                                       \
		  */
#define SC_ADCS_SYSMON_BHM_3V3_SNTVR_OFFSET                                                        \
	(0x2030) /* BHM VDD_3V3 Shunt Voltage Monitor Register */
#define SC_ADCS_SYSMON_BHM_3V3_BUSVR_OFFSET                                                        \
	(0x2034) /* BHM VDD_3V3 Bus Voltage Monitor Register                                       \
		  */
#define SC_ADCS_SYSMON_BHM_3V3SYSA_SNTVR_OFFSET                                                    \
	(0x2038) /* BHM VDD_3V3_SYS_A Shunt Voltage Monitor Register */
#define SC_ADCS_SYSMON_BHM_3V3SYSA_BUSVR_OFFSET                                                    \
	(0x203C) /* BHM VDD_3V3_SYS_A Bus Voltage Monitor Register */
#define SC_ADCS_SYSMON_BHM_3V3SYSB_SNTVR_OFFSET                                                    \
	(0x2040) /* BHM VDD_3V3_SYS_B Shunt Voltage Monitor Register */
#define SC_ADCS_SYSMON_BHM_3V3SYSB_BUSVR_OFFSET                                                    \
	(0x2044) /* BHM VDD_3V3_SYS_B Bus Voltage Monitor Register */
#define SC_ADCS_SYSMON_BHM_3V3IO_SNTVR_OFFSET                                                      \
	(0x2048) /* BHM VDD_3V3_IO Shunt Voltage Monitor Register */
#define SC_ADCS_SYSMON_BHM_3V3IO_BUSVR_OFFSET                                                      \
	(0x204C)                                  /* BHM VDD_3V3_IO Bus Voltage Monitor Register */
#define SC_ADCS_SYSMON_BHM_TEMP1R_OFFSET (0x2050) /* BHM Temperature1 Monitor Register */
#define SC_ADCS_SYSMON_BHM_TEMP2R_OFFSET (0x2054) /* BHM Temperature2 Monitor Register */
#define SC_ADCS_SYSMON_BHM_TEMP3R_OFFSET (0x2058) /* BHM Temperature3 Monitor Register */

#define SC_ADCS_GPTMR_TECR_OFFSET  (0x0004) /* Timer Enable Control Register */
#define SC_ADCS_GPTMR_HITCR_OFFSET (0x0200) /* Hardware Interrupt Timer Control Register */
#define SC_ADCS_GPTMR_HITPR_OFFSET (0x0204) /* Hardware Interrupt Timer Prescaler Register */
#define SC_ADCS_GPTMR_HITOCR1_OFFSET                                                               \
	(0x0210) /* Hardware Interrupt Timer Output Compare Register 1 */
#define SC_ADCS_GPTMR_HITOCR2_OFFSET                                                               \
	(0x0214) /* Hardware Interrupt Timer Output Compare Register 2 */
#define SC_ADCS_GPTMR_HITOCR3_OFFSET                                                               \
	(0x0218) /* Hardware Interrupt Timer Output Compare Register 3 */

/* BHM Initialization Access Control Register */
#define SC_ADCS_SYSMON_BHM_INIT_REQ     BIT(16)
#define SC_ADCS_SYSMON_BHM_TEMP3_INITEN BIT(4)
#define SC_ADCS_SYSMON_BHM_TEMP2_INITEN BIT(3)
#define SC_ADCS_SYSMON_BHM_TEMP1_INITEN BIT(2)
#define SC_ADCS_SYSMON_BHM_CVM2_INITEN  BIT(1)
#define SC_ADCS_SYSMON_BHM_CVM1_INITEN  BIT(0)
#define SC_ADCS_SYSMON_BHM_INIT_ALL                                                                \
	(SC_ADCS_SYSMON_BHM_INIT_REQ | SC_ADCS_SYSMON_BHM_TEMP3_INITEN |                           \
	 SC_ADCS_SYSMON_BHM_TEMP2_INITEN | SC_ADCS_SYSMON_BHM_TEMP1_INITEN |                       \
	 SC_ADCS_SYSMON_BHM_CVM2_INITEN | SC_ADCS_SYSMON_BHM_CVM1_INITEN)

/* BHM Access Control Register */
#define SC_ADCS_SYSMON_BHM_TEMP3_MONIEN BIT(4)
#define SC_ADCS_SYSMON_BHM_TEMP2_MONIEN BIT(3)
#define SC_ADCS_SYSMON_BHM_TEMP1_MONIEN BIT(2)
#define SC_ADCS_SYSMON_BHM_CVM2_MONIEN  BIT(1)
#define SC_ADCS_SYSMON_BHM_CVM1_MONIEN  BIT(0)
#define SC_ADCS_SYSMON_BHM_MONIEN_ALL                                                              \
	(SC_ADCS_SYSMON_BHM_TEMP3_MONIEN | SC_ADCS_SYSMON_BHM_TEMP2_MONIEN |                       \
	 SC_ADCS_SYSMON_BHM_TEMP1_MONIEN | SC_ADCS_SYSMON_BHM_CVM2_MONIEN |                        \
	 SC_ADCS_SYSMON_BHM_CVM1_MONIEN)

/* BHM Interrupt Status Register */
#define SC_ADCS_SYSMON_BHM_INIT_ACCEND BIT(0)

/* Timer Enable Control Register */
#define SC_ADCS_SYSMON_GPTMR_HITEN BIT(1)

#define SYSMON_NUM_OF_RETRY (100U)

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

	*addr = SC_ADCS_SYSMON_BASE_ADDR;

	switch (pos) {
	case OBC_TEMP_1:
		*addr += SC_ADCS_SYSMON_BHM_TEMP1R_OFFSET;
		break;
	case OBC_TEMP_2:
		*addr += SC_ADCS_SYSMON_BHM_TEMP2R_OFFSET;
		break;
	case OBC_TEMP_3:
		*addr += SC_ADCS_SYSMON_BHM_TEMP3R_OFFSET;
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

	*addr = SC_ADCS_SYSMON_BASE_ADDR;

	switch (pos) {
	case OBC_1V0_SHUNT:
		*addr += SC_ADCS_SYSMON_BHM_1V0_SNTVR_OFFSET;
		break;
	case OBC_1V0_BUS:
		*addr += SC_ADCS_SYSMON_BHM_1V0_BUSVR_OFFSET;
		break;
	case OBC_1V8_SHUNT:
		*addr += SC_ADCS_SYSMON_BHM_1V8_SNTVR_OFFSET;
		break;
	case OBC_1V8_BUS:
		*addr += SC_ADCS_SYSMON_BHM_1V8_BUSVR_OFFSET;
		break;
	case OBC_3V3_SHUNT:
		*addr += SC_ADCS_SYSMON_BHM_3V3_SNTVR_OFFSET;
		break;
	case OBC_3V3_BUS:
		*addr += SC_ADCS_SYSMON_BHM_3V3_BUSVR_OFFSET;
		break;
	case OBC_3V3_SYSA_SHUNT:
		*addr += SC_ADCS_SYSMON_BHM_3V3SYSA_SNTVR_OFFSET;
		break;
	case OBC_3V3_SYSA_BUS:
		*addr += SC_ADCS_SYSMON_BHM_3V3SYSA_BUSVR_OFFSET;
		break;
	case OBC_3V3_SYSB_SHUNT:
		*addr += SC_ADCS_SYSMON_BHM_3V3SYSB_SNTVR_OFFSET;
		break;
	case OBC_3V3_SYSB_BUS:
		*addr += SC_ADCS_SYSMON_BHM_3V3SYSB_BUSVR_OFFSET;
		break;
	case OBC_3V3_IO_SHUNT:
		*addr += SC_ADCS_SYSMON_BHM_3V3IO_SNTVR_OFFSET;
		break;
	case OBC_3V3_IO_BUS:
		*addr += SC_ADCS_SYSMON_BHM_3V3IO_BUSVR_OFFSET;
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

	*addr = SC_ADCS_SYSMON_BASE_ADDR;

	switch (pos) {
	case OBC_XADC_VCCINT:
		*addr += SC_ADCS_SYSMON_XADC_VCCINT_OFFSET;
		break;
	case OBC_XADC_VCCAUX:
		*addr += SC_ADCS_SYSMON_XADC_VCCAUX_OFFSET;
		break;
	case OBC_XADC_VCCBRAM:
		*addr += SC_ADCS_SYSMON_XADC_VCCBRAM_OFFSET;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	return ret;
}

static int wait_for_bhm_complete(uint32_t target_bit)
{
	int i;
	int ret = 0;
	uint32_t reg;

	/* Wait for any processing to be complete */
	for (i = 0; i < SYSMON_NUM_OF_RETRY; i++) {
		reg = sys_read32(SC_ADCS_SYSMON_BASE_ADDR + SC_ADCS_SYSMON_BHM_ISR_OFFSET);
		if ((reg & target_bit) == target_bit) {
			sys_write32(target_bit,
				    SC_ADCS_SYSMON_BASE_ADDR + SC_ADCS_SYSMON_BHM_ISR_OFFSET);
			break;
		}
		k_sleep(K_MSEC(1));
	}

	if (i == SYSMON_NUM_OF_RETRY) {
		ret = -ETIMEDOUT;
	}

	return ret;
}

static int sc_adcs_bhm_init(void)
{
	sys_set_bits(SC_ADCS_SYSMON_BASE_ADDR + SC_ADCS_SYSMON_BHM_INICTLR_OFFSET,
		     SC_ADCS_SYSMON_BHM_INIT_ALL);

	return wait_for_bhm_complete(SC_ADCS_SYSMON_BHM_INIT_ACCEND);
}

static void sc_adcs_bhm_set_timer(void)
{
	/*
	 * This value is using the recommended value from the FPGA technical
	 * reference manual.
	 */
	sys_write32(0x00280000, SC_ADCS_GPT_BASE_ADDR + SC_ADCS_GPTMR_HITCR_OFFSET);
	sys_write32(0x095F, SC_ADCS_GPT_BASE_ADDR + SC_ADCS_GPTMR_HITPR_OFFSET);
	sys_write32(0x0050, SC_ADCS_GPT_BASE_ADDR + SC_ADCS_GPTMR_HITOCR1_OFFSET);
	sys_write32(0x0001, SC_ADCS_GPT_BASE_ADDR + SC_ADCS_GPTMR_HITOCR2_OFFSET);
	sys_write32(0x0040, SC_ADCS_GPT_BASE_ADDR + SC_ADCS_GPTMR_HITOCR3_OFFSET);
}

static inline void sc_adcs_bhm_monitor_enable(void)
{
	sys_set_bits(SC_ADCS_SYSMON_BASE_ADDR + SC_ADCS_SYSMON_BHM_ACCCTLR_OFFSET,
		     SC_ADCS_SYSMON_BHM_MONIEN_ALL);
}

static inline void sc_adcs_bhm_monitor_disable(void)
{
	sys_clear_bits(SC_ADCS_SYSMON_BASE_ADDR + SC_ADCS_SYSMON_BHM_ACCCTLR_OFFSET,
		       SC_ADCS_SYSMON_BHM_MONIEN_ALL);
}

static inline void sc_adcs_bhm_timer_enable(void)
{
	sys_set_bits(SC_ADCS_GPT_BASE_ADDR + SC_ADCS_GPTMR_TECR_OFFSET, SC_ADCS_SYSMON_GPTMR_HITEN);
}

static inline void sc_adcs_bhm_timer_disable(void)
{
	sys_clear_bits(SC_ADCS_GPT_BASE_ADDR + SC_ADCS_GPTMR_TECR_OFFSET,
		       SC_ADCS_SYSMON_GPTMR_HITEN);
}

void sc_adcs_kick_wdt_timer(void)
{
	uint32_t reg;

	reg = sys_read32(SC_ADCS_SYSMON_BASE_ADDR + SC_ADCS_SYSMON_WDOG_CTRL_OFFSET);
	sys_write32(reg, SC_ADCS_SYSMON_BASE_ADDR + SC_ADCS_SYSMON_WDOG_CTRL_OFFSET);
}

void sc_adcs_print_fpga_ids(void)
{
	LOG_INF("* FSW Version       : %s", ADCS_HWTEST_VERSION);
	LOG_INF("* IP Version        : %08x",
		sys_read32(SC_ADCS_SYSREG_BASE_ADDR + SC_ADCS_SYSREG_VER_OFFSET));
	LOG_INF("* Build Information : %08x",
		sys_read32(SC_ADCS_SYSREG_BASE_ADDR + SC_ADCS_SYSREG_BUILDINFO_OFFSET));
	LOG_INF("* Device DNA 1      : %08x",
		sys_read32(SC_ADCS_SYSREG_BASE_ADDR + SC_ADCS_SYSREG_DNA1_OFFSET));
	LOG_INF("* Device DNA 2      : %08x",
		sys_read32(SC_ADCS_SYSREG_BASE_ADDR + SC_ADCS_SYSREG_DNA2_OFFSET));
}

int sc_adcs_bhm_enable(void)
{
	int ret;

	ret = sc_adcs_bhm_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize the BHM. (%d)", ret);
		goto end;
	}

	sc_adcs_bhm_set_timer();

	sc_adcs_bhm_monitor_enable();

	sc_adcs_bhm_timer_enable();
end:
	return ret;
}

int sc_adcs_bhm_get_obc_temp(enum obc_temp_pos pos, float *temp)
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

int sc_adcs_bhm_get_obc_cv(enum obc_cv_pos pos, int32_t *cv)
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

int sc_adcs_bhm_get_xadc_temp(float *temp)
{
	uint32_t raw;

	raw = sys_read32(SC_ADCS_SYSMON_BASE_ADDR + SC_ADCS_SYSMON_XADC_TEMP_OFFSET);
	*temp = convert_xadc_temp(raw);

	return 0;
}

int sc_adcs_bhm_get_xadc_cv(enum xadc_cv_pos pos, float *cv)
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

int sc_adcs_bhm_disable(void)
{
	return 0;
}

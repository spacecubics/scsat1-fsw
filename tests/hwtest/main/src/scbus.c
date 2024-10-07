/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(scbus, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

/* Registers */
#define SC_MAIN_SCBUS_BASE_ADDR (0x40300000) /* SC Bus Control */

#define SC_MAIN_SCBUS_PHY_OFFSET (0x0004) /* SC Bus Phy Control Register */
#define SC_MAIN_SCBUS_CFG_OFFSET (0x0008) /* SC Bus Configuration Register */
#define SC_MAIN_SCBUS_PST_OFFSET (0x0020) /* SC Bus Port Status Register */
#define SC_MAIN_SCBUS_FRM_OFFSET (0x0034) /* SC Bus Frame Remaining Register */

#define SC_MAIN_SCBUS_PHY_CLK_EN    BIT(16)
#define SC_MAIN_SCBUS_PHY_RESET     BIT(4)
#define SC_MAIN_SCBUS_PHY_PWR_DOWN  BIT(0)
#define SC_MAIN_SCBUS_CFG_HOST_MODE BIT(0)
#define SC_MAIN_SCBUS_FRM_FRAME_EN  BIT(16)
#define SC_MAIN_SCBUS_PST_STATE_CHG BIT(16)
#define SC_MAIN_SCBUS_PST_CONNECT   BIT(0)
#define SC_MAIN_SCBUS_PST_RESET     BIT(9)

#define SC_MAIN_SCBUS_DETECT_CONNECT_RETRY (10U)
#define SC_MAIN_SCBUS_DETECT_CONNECT_WAIT  K_MSEC(10)

int scbus_sof_start(void)
{
	int ret = 0;
	int i;
	uint32_t reg;

	/* Power Down / ULPI Reset */
	sys_set_bits(SC_MAIN_SCBUS_BASE_ADDR + SC_MAIN_SCBUS_PHY_OFFSET,
		       SC_MAIN_SCBUS_PHY_PWR_DOWN | SC_MAIN_SCBUS_PHY_RESET);

	/* Negate Power Down / ULPI Reset */
	sys_clear_bits(SC_MAIN_SCBUS_BASE_ADDR + SC_MAIN_SCBUS_PHY_OFFSET,
		       SC_MAIN_SCBUS_PHY_PWR_DOWN | SC_MAIN_SCBUS_PHY_RESET);

	/* Wait for clock enable */
	for (i = 0; i < SC_MAIN_SCBUS_DETECT_CONNECT_RETRY; i++) {
		reg = sys_read32(SC_MAIN_SCBUS_BASE_ADDR + SC_MAIN_SCBUS_PHY_OFFSET);
		if (reg & SC_MAIN_SCBUS_PHY_CLK_EN) {
			break;
		}
		k_sleep(SC_MAIN_SCBUS_DETECT_CONNECT_WAIT);
	}

	if (i == SC_MAIN_SCBUS_DETECT_CONNECT_RETRY) {
		LOG_ERR("Clock not enabled");
		ret = -ETIMEDOUT;
		goto end;
	}

	/* Set Host Mode */
	sys_set_bits(SC_MAIN_SCBUS_BASE_ADDR + SC_MAIN_SCBUS_CFG_OFFSET,
		     SC_MAIN_SCBUS_CFG_HOST_MODE);

	/* Set Frame Enable */
	sys_set_bits(SC_MAIN_SCBUS_BASE_ADDR + SC_MAIN_SCBUS_FRM_OFFSET,
		     SC_MAIN_SCBUS_FRM_FRAME_EN);

	/* Wait for device connect */
	for (i = 0; i < SC_MAIN_SCBUS_DETECT_CONNECT_RETRY; i++) {
		reg = sys_read32(SC_MAIN_SCBUS_BASE_ADDR + SC_MAIN_SCBUS_PST_OFFSET);
		if ((reg & SC_MAIN_SCBUS_PST_STATE_CHG) && (reg & SC_MAIN_SCBUS_PST_CONNECT)) {
			sys_set_bits(SC_MAIN_SCBUS_BASE_ADDR + SC_MAIN_SCBUS_PST_OFFSET,
				     SC_MAIN_SCBUS_PST_STATE_CHG);
			LOG_INF("USB Device connect");
			break;
		}
		k_sleep(SC_MAIN_SCBUS_DETECT_CONNECT_WAIT);
	}

	if (i == SC_MAIN_SCBUS_DETECT_CONNECT_RETRY) {
		LOG_ERR("USB Device not connected");
		ret = -ETIMEDOUT;
		goto end;
	}

	/* Set Port Rest */
	sys_set_bits(SC_MAIN_SCBUS_BASE_ADDR + SC_MAIN_SCBUS_PST_OFFSET, SC_MAIN_SCBUS_PST_RESET);

end:
	return ret;
}

void scbus_sof_stop(void)
{
	/* Set Frame Disable */
	sys_clear_bits(SC_MAIN_SCBUS_BASE_ADDR + SC_MAIN_SCBUS_FRM_OFFSET,
		       SC_MAIN_SCBUS_FRM_FRAME_EN);
}

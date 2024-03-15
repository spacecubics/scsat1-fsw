/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "pwrctrl.h"
#include "sysmon.h"
#include "csp.h"
#include "scbus.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main_init);

int main_init(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	ret = sc_main_bhm_enable();
	if (ret < 0) {
		LOG_ERR("Failed to enable the BHM. (%d)", ret);
		(*err_cnt)++;
		all_ret = -1;
	} else {
		LOG_INF("Enable the BHM");
	}

	sc_main_power_enable(DRV2_PWR | DRV1_PWR);
	LOG_INF("Power on the DRV1/DRV2");

	k_sleep(K_SECONDS(1));

	sc_main_power_enable(DSTRX_IO_PWR);
	LOG_INF("Power on the DSTRX-3 IO");

	k_sleep(K_SECONDS(1));

	sc_main_power_enable(PDU_O1_PWR);
	LOG_INF("Power on the ADCS Board");

	k_sleep(K_SECONDS(1));

	sc_main_power_enable(PDU_O2_PWR);
	LOG_INF("Power on the Payload Board");

	ret = csp_enable();
	if (ret < 0) {
		LOG_ERR("Failed to enable the CSP. (%d)", ret);
		(*err_cnt)++;
		all_ret = -1;
	} else {
		LOG_INF("Enable the CSP");
	}

	ret = scbus_sof_start();
	if (ret < 0) {
		LOG_ERR("Failed to start the sending SOF. (%d)", ret);
		(*err_cnt)++;
		all_ret = -1;
	} else {
		LOG_INF("Start the sending SOF Packet");
	}

	return all_ret;
}

int main_off(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	ret = sc_main_bhm_disable();
	if (ret < 0) {
		LOG_ERR("Failed to disable the BHM. (%d)", ret);
		(*err_cnt)++;
		all_ret = -1;
	} else {
		LOG_INF("Disable the BHM");
	}

	sc_main_power_disable(DRV2_PWR | DRV1_PWR);
	LOG_INF("Power off the DRV1/DRV2");

	k_sleep(K_SECONDS(1));

	sc_main_power_disable(DSTRX_IO_PWR);
	LOG_INF("Power off the DSTRX-3 IO");

	k_sleep(K_SECONDS(1));

	sc_main_power_disable(PDU_O1_PWR);
	LOG_INF("Power off the ADCS Board");

	k_sleep(K_SECONDS(1));

	sc_main_power_disable(PDU_O2_PWR);
	LOG_INF("Power off the Payload Board");

	csp_disable();
	LOG_INF("Disable the CSP");

	scbus_sof_stop();
	LOG_INF("Stop the sending SOF Packet");

	return all_ret;
}

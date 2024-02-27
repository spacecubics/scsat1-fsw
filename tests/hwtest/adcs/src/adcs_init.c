/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "pwrctrl.h"
#include "sysmon.h"
#include "gnss.h"
#include "imu.h"
#include "csp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adcs_init);

int adcs_init(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	ret = sc_adcs_bhm_enable();
	if (ret < 0) {
		LOG_ERR("Failed to enable the BHM. (%d)", ret);
		(*err_cnt)++;
		all_ret = -1;
	} else {
		LOG_INF("Enable the BHM");
	}

	sc_adcs_power_enable(DRV_PWR);
	LOG_INF("Power on the DRV");

	k_sleep(K_SECONDS(1));

	sc_adcs_power_enable(IMU_PWR);
	imu_enable();
	LOG_INF("Power on the IMU");

	k_sleep(K_SECONDS(1));

	sc_adcs_power_enable(GPS_PWR);
	ret = gnss_enable();
	if (ret < 0) {
		LOG_ERR("Failed to enable the GNSS. (%d)", ret);
		(*err_cnt)++;
		all_ret = -1;
	} else {
		LOG_INF("Power on the GNSS");
	}

	k_sleep(K_SECONDS(1));

	ret = csp_enable();
	if (ret < 0) {
		LOG_ERR("Failed to enable the CSP. (%d)", ret);
		(*err_cnt)++;
		all_ret = -1;
	} else {
		LOG_INF("Enable the CSP");
	}

	return all_ret;
}

int adcs_off(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	ret = sc_adcs_bhm_disable();
	if (ret < 0) {
		LOG_ERR("Failed to disble the BHM. (%d)", ret);
		(*err_cnt)++;
		all_ret = -1;
	} else {
		LOG_INF("Disable the BHM");
	}

	sc_adcs_power_disable(DRV_PWR);
	LOG_INF("Power off the DRV");

	k_sleep(K_SECONDS(1));

	sc_adcs_power_disable(IMU_PWR);
	imu_disable();
	LOG_INF("Power off the IMU");

	k_sleep(K_SECONDS(1));

	sc_adcs_power_disable(GPS_PWR);
	gnss_disable();
	LOG_INF("Power off the GNSS");

	k_sleep(K_SECONDS(1));

	csp_disable();
	LOG_INF("Disable the CSP");

	return all_ret;
}

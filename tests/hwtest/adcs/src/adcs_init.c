/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "pwrctrl_adcs.h"
#include "sc_fpgamon.h"
#include "gnss.h"
#include "imu.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adcs_init, CONFIG_SCSAT1_ADCS_LOG_LEVEL);

int adcs_init(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	ret = sc_bhm_enable();
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

	return all_ret;
}

int adcs_off(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	ret = sc_bhm_disable();
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

	return all_ret;
}

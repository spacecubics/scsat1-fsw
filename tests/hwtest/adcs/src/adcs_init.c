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
	k_sleep(K_SECONDS(1));

/*
	for (int i=0; i<100; i++){
		sc_adcs_power_enable(IMU_PWR);
		k_sleep(K_USEC(1));
		sc_adcs_power_disable(IMU_PWR);
		k_sleep(K_USEC(1));
	}	

	k_sleep(K_SECONDS(5));
	sc_adcs_power_disable(DRV_PWR);
	k_sleep(K_SECONDS(5));
	sc_adcs_power_disable(IMU_PWR);
	k_sleep(K_SECONDS(5));
	sc_adcs_power_disable(GPS_PWR);
	k_sleep(K_SECONDS(5));
*/
/*
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
*/

	return all_ret;
}

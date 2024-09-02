/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "common.h"
#include "pwrctrl.h"
#include "sysmon.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main_init);

enum hwtest_mode test_mode;

int main_init(enum hwtest_mode mode, uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;

	if (mode >= TEST_MODE_NUM) {
		LOG_ERR("Invalid test mode. (%d)", mode);
		(*err_cnt)++;
		all_ret = -1;
		goto end;
	}

	test_mode = mode;

	ret = sc_main_bhm_enable();
	if (ret < 0) {
		LOG_ERR("Failed to enable the BHM. (%d)", ret);
		(*err_cnt)++;
		all_ret = -1;
	} else {
		LOG_INF("Enable the BHM");
	}

	sc_main_power_enable(DRV1_PWR);
	LOG_INF("Power on the DRV1");

	k_sleep(K_SECONDS(1));

	if (mode > MAIN_ONLY_WITHOUT_DSTRX) {
		sc_main_dstrx3_io_power_enable();
		LOG_INF("Power on the DSTRX-3 IO");

		k_sleep(K_SECONDS(1));
	}

	if (mode > MAIN_ONLY) {
		sc_main_power_enable(PDU_O1_PWR);
		LOG_INF("Power on the ADCS Board");

		k_sleep(K_SECONDS(1));
	}

	if (mode > MAIN_ADCS_ONLY) {
		sc_main_power_enable(PDU_O2_PWR);
		LOG_INF("Power on the Payload Board");
	}

end:
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

	sc_main_power_disable(DRV1_PWR);
	LOG_INF("Power off the DRV1");

	k_sleep(K_SECONDS(1));

	sc_main_dstrx3_io_power_disable();
	LOG_INF("Power off the DSTRX-3 IO");

	k_sleep(K_SECONDS(1));

	sc_main_power_disable(PDU_O1_PWR);
	LOG_INF("Power off the ADCS Board");

	k_sleep(K_SECONDS(1));

	sc_main_power_disable(PDU_O2_PWR);
	LOG_INF("Power off the Payload Board");

	return all_ret;
}

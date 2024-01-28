/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "sc_dstrx3.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dstrx3_test);

int dstrx3_test(uint32_t *err_cnt)
{
	int ret;
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(dstrx));
	struct sc_dstrx3_hk hk;

	ret = sc_dstrx3_get_hk_telemetry(dev, &hk);
	if (ret < 0) {
		LOG_ERR("DSTRX-3 HK: Failed");
		(*err_cnt)++;
		goto end;
	}

	LOG_INF("DSTRX-3 HK FREE_COUNTER     : %d", hk.free_count);
	LOG_INF("DSTRX-3 HK WDT_COUNTER      : %d", hk.wdt_count);
	LOG_INF("DSTRX-3 HK RSSI             : %d", -(hk.rssi + 16));
	LOG_INF("DSTRX-3 HK RCV_FREQ         : %.4f [Hz] (raw: %d)",
		(2105.350 + (hk.rcv_freq * 100)), hk.rcv_freq);
	LOG_INF("DSTRX-3 HK TEMPERATURE      : %d [deg]", hk.temperature);
	LOG_INF("DSTRX-3 HK VOLTAGE          : %.04f [v] (raw: %d)",
		(float)((hk.voltage / 256) * 2.5), hk.voltage);
	LOG_INF("DSTRX-3 HK TX_PWR           : %.04f [dBm] (raw: %d)",
		((0.2473 * hk.tx_power) + 0.99), hk.tx_power);
	LOG_INF("DSTRX-3 HK CARRIER_LOCK     : %d", hk.carrier_lock);
	LOG_INF("DSTRX-3 HK SUB_CARRIER_LOCK : %d", hk.sub_carrier_lock);
	LOG_INF("DSTRX-3 HK TX_PWR_SET       : %d", hk.tx_power_set);
	LOG_INF("DSTRX-3 HK BIT_RATE_SET     : %d", hk.bit_rate_set);
	LOG_INF("DSTRX-3 HK PROG_NO          : %d", hk.program_no);
	LOG_INF("DSTRX-3 HK CHK_SUM          : %d", hk.checksum);

end:
	return ret;
}

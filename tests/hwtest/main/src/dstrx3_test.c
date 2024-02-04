/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "common.h"
#include "dstrx3_test.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dstrx3_test);

static void dstrx3_print_hk(struct sc_dstrx3_hk *hk)
{
	LOG_INF("DSTRX-3 HK FREE_COUNTER     : %d", hk->free_count);
	LOG_INF("DSTRX-3 HK WDT_COUNTER      : %d", hk->wdt_count);
	LOG_INF("DSTRX-3 HK RSSI             : %d [dBm] (raw: %d)", -(hk->rssi + 16), hk->rssi);
	LOG_INF("DSTRX-3 HK RCV_FREQ         : %.4f [Hz] (raw: %d)",
		(2105.350 + (hk->rcv_freq * 100)), hk->rcv_freq);
	LOG_INF("DSTRX-3 HK TEMPERATURE      : %d [deg]", hk->temperature);
	LOG_INF("DSTRX-3 HK VOLTAGE          : %.04f [v] (raw: %d)",
		(double)((hk->voltage / 256.0f) * 2.5f), hk->voltage);
	LOG_INF("DSTRX-3 HK TX_PWR           : %.04f [dBm] (raw: %d)",
		((0.2473 * hk->tx_power) + 0.99), hk->tx_power);
	LOG_INF("DSTRX-3 HK CARRIER_LOCK     : %d", hk->carrier_lock);
	LOG_INF("DSTRX-3 HK SUB_CARRIER_LOCK : %d", hk->sub_carrier_lock);
	LOG_INF("DSTRX-3 HK TX_PWR_SET       : %d", hk->tx_power_set);
	LOG_INF("DSTRX-3 HK BIT_RATE_SET     : %d", hk->bit_rate_set);
	LOG_INF("DSTRX-3 HK PROG_NO          : %d", hk->program_no);
	LOG_INF("DSTRX-3 HK CHK_SUM          : %d", hk->checksum);
}

int dstrx3_test(struct dstrx3_test_ret *dstrx3_ret, uint32_t *err_cnt, bool log)
{
	int ret;
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(dstrx));
	struct sc_dstrx3_hk hk;

	ret = sc_dstrx3_get_hk_telemetry(dev, &hk);
	if (ret < 0) {
		memset(dstrx3_ret, 0, sizeof(*dstrx3_ret));
		HWTEST_LOG_ERR(log, "DSTRX-3 HK: Failed");
		(*err_cnt)++;
	} else {
		dstrx3_ret->free_count = hk.free_count;
		dstrx3_ret->wdt_count = hk.wdt_count;
		dstrx3_ret->rssi = hk.rssi;
		dstrx3_ret->rcv_freq = hk.rcv_freq;
		dstrx3_ret->temperature = hk.temperature;
		dstrx3_ret->voltage = hk.voltage;
		dstrx3_ret->tx_power = hk.tx_power;
		dstrx3_ret->carrier_lock = hk.carrier_lock;
		dstrx3_ret->sub_carrier_lock = hk.sub_carrier_lock;
		dstrx3_ret->tx_power_set = hk.tx_power_set;
		dstrx3_ret->bit_rate_set = hk.bit_rate_set;
		dstrx3_ret->program_no = hk.program_no;
		dstrx3_ret->checksum = hk.checksum;
		if (log) {
			dstrx3_print_hk(&hk);
		}
	}
	dstrx3_ret->status = ret;

	return ret;
}

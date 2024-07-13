/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr/device.h>

struct sc_dstrx3_hk {
	uint8_t free_count;
	uint8_t wdt_count;
	uint8_t rssi;
	int16_t rcv_freq;
	int8_t temperature;
	uint8_t voltage;
	uint8_t tx_power;
	uint8_t carrier_lock: 1;
	uint8_t sub_carrier_lock: 1;
	uint8_t tx_power_set: 4;
	uint8_t bit_rate_set: 2;
	uint8_t program_no;
	uint8_t checksum;
};

/* TX parameters */
enum sc_dstrx3_tx_power {
	SC_DSTRX3_TX_PWR_12_0 = 0,
	SC_DSTRX3_TX_PWR_12_2,
	SC_DSTRX3_TX_PWR_12_4,
	SC_DSTRX3_TX_PWR_13_4,
	SC_DSTRX3_TX_PWR_14_4,
	SC_DSTRX3_TX_PWR_15_4,
	SC_DSTRX3_TX_PWR_16_4,
	SC_DSTRX3_TX_PWR_17_4,
	SC_DSTRX3_TX_PWR_17_8,
	SC_DSTRX3_TX_PWR_18_0,
	SC_DSTRX3_TX_PWR_18_2,
	SC_DSTRX3_TX_PWR_18_4,
	SC_DSTRX3_TX_PWR_18_8,
	SC_DSTRX3_TX_PWR_19_2,
	SC_DSTRX3_TX_PWR_19_6,
	SC_DSTRX3_TX_PWR_20_0,
};

enum sc_dstrx3_bit_rate {
	SC_DSTRX3_BIT_RATE_4K = 0,
	SC_DSTRX3_BIT_RATE_16K,
	SC_DSTRX3_BIT_RATE_32K,
	SC_DSTRX3_BIT_RATE_64K,
};

void sc_dstrx3_enable_hk(const struct device *dev);
void sc_dstrx3_disable_hk(const struct device *dev);
void sc_dstrx3_enable_cmdif(const struct device *dev);
void sc_dstrx3_disable_cmdif(const struct device *dev);
void sc_dstrx3_enable_downlink(const struct device *dev);
void sc_dstrx3_disable_downlink(const struct device *dev);
int sc_dstrx3_get_hk_telemetry(const struct device *dev, struct sc_dstrx3_hk *hk);
void sc_dstrx3_set_tx_param(const struct device *dev, enum sc_dstrx3_tx_power tx_power,
			    enum sc_dstrx3_bit_rate bit_rate);
void sc_dstrx3_set_default_tx_param(const struct device *dev);

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

/* Downlink Data Control */
#define DLD_NRZI_EN     BIT(5)
#define DLD_CC_EN       BIT(4)
#define DLD_PR_EN       BIT(3)
#define DLD_RS_EN       BIT(2)
#define DLD_CRC32C_EN   BIT(1)
#define DLD_ZPADDING_EN BIT(0)

/* Uplink Data Control */
#define ULD_NRZI_EN    BIT(5)
#define ULD_CC_EN      BIT(4)
#define ULD_PR_EN      BIT(3)
#define ULD_RS_EN      BIT(2)
#define ULD_CRC32C_EN  BIT(1)
#define ULD_FIXDLEN_EN BIT(0)

void sc_dstrx3_enable_hk(const struct device *dev);
void sc_dstrx3_disable_hk(const struct device *dev);
void sc_dstrx3_enable_cmdif(const struct device *dev);
void sc_dstrx3_disable_cmdif(const struct device *dev);
void sc_dstrx3_enable_downlink(const struct device *dev);
void sc_dstrx3_disable_downlink(const struct device *dev);
void sc_dstrx3_enable_uplink(const struct device *dev);
void sc_dstrx3_disable_uplink(const struct device *dev);
int sc_dstrx3_get_hk_telemetry(const struct device *dev, struct sc_dstrx3_hk *hk);
void sc_dstrx3_set_tx_param(const struct device *dev, enum sc_dstrx3_tx_power tx_power,
			    enum sc_dstrx3_bit_rate bit_rate);
void sc_dstrx3_set_default_tx_param(const struct device *dev);
void sc_dstrx3_set_downlink_control(const struct device *dev, uint32_t control);
int sc_dstrx3_downlink_data(const struct device *dev, const uint8_t *data, uint16_t size);
void sc_dstrx3_set_uplink_control(const struct device *dev, uint32_t control);
void sc_dstrx3_get_uplink_status(const struct device *dev, uint8_t *count, uint8_t *status);
bool sc_dstrx3_is_uplink_crc_error(const struct device *dev, uint8_t status);
bool sc_dstrx3_is_uplink_rs_error(const struct device *dev, uint8_t status);
bool sc_dstrx3_is_uplink_dlen_error(const struct device *dev, uint8_t status);
int sc_dstrx3_get_uplink_data(const struct device *dev, uint8_t *data, uint16_t *size);

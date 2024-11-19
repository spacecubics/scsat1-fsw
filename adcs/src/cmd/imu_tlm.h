/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <csp/csp.h>
#include "imu.h"

struct imu_telemetry {
	uint8_t telemetry_id;
	uint32_t error_code;
	uint8_t imu_data[IMU_EXT_DATA_SIZE];
} __attribute__((__packed__));

void send_imu_tlm(csp_packet_t *packet, uint8_t command_id);

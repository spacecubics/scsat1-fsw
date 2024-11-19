/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "imu_tlm.h"

#include <csp/csp.h>
#include "sc_csp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu_tlm, CONFIG_SCSAT1_ADCS_LOG_LEVEL);

#define CSP_CONNECTION_TIMEOUT_MS (1000U)

void send_imu_tlm(csp_packet_t *packet, uint8_t command_id)
{
	int ret;
	struct imu_data data;
	struct imu_telemetry tlm;

	ret = get_imu_data_ext(&data);
	if (ret < 0) {
		LOG_ERR("Failed to get the IMU EXT Data. (%d)", ret);
		memset(&data, 0, sizeof(data));
	}

	tlm.telemetry_id = command_id;
	tlm.error_code = ret;
	memcpy(tlm.imu_data, data.raw, sizeof(data.raw));
	memcpy(packet->data, &tlm, sizeof(tlm));
	packet->length = sizeof(tlm);

	csp_sendto_reply(packet, packet, CSP_O_SAME);
}

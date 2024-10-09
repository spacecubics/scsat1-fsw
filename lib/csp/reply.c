/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <csp/csp.h>
#include "sc_csp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(reply, CONFIG_SC_LIB_CSP_LOG_LEVEL);

#define CSP_CONNECTION_TIMEOUT_MS (1000U)

void csp_send_std_reply(csp_packet_t *packet, uint8_t command_id, int err_code)
{
	packet->data[CSP_TELEMETRY_ID_OFFSET] = command_id;
	sys_put_le32(err_code, &packet->data[CSP_ERROR_CODE_OFFSET]);
	packet->length = sizeof(command_id) + sizeof(err_code);

	csp_sendto_reply(packet, packet, CSP_O_SAME);
}

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <csp/csp.h>

struct csp_stat {
	uint16_t received_command_count;
	uint8_t last_csp_port;
	uint8_t last_command_id;
};

int csp_system_handler(csp_packet_t *packet);
void csp_system_update_stat(csp_packet_t *packet, struct csp_stat *stat);

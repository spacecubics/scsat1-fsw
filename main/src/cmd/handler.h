/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

struct csp_stat {
	uint16_t received_command_count;
	uint8_t last_csp_port;
	uint8_t last_command_id;
};

void csp_cmd_handler(void);

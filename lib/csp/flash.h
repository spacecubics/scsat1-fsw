/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <csp/csp.h>

struct cfg_crc_telemetry {
	uint8_t telemetry_id;
	uint32_t error_code;
	uint8_t bank;
	uint8_t partition_id;
	uint32_t crc32;
} __attribute__((__packed__));

void csp_flash_handler_init(void);
int csp_flash_handler(csp_packet_t *packet);

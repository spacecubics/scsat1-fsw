/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr/fs/fs.h>
#include <csp/csp.h>

struct file_info_telemetry {
	uint8_t telemetry_id;
	uint32_t error_code;
	uint8_t entry_type;
	uint32_t file_size;
	char file_name[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN];
} __attribute__((__packed__));

void csp_file_handler_init(void);
int csp_file_handler(csp_packet_t *packet);

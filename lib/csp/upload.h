/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <csp/csp.h>

struct upload_open_reply_telemetry {
	uint8_t telemetry_id;
	uint32_t error_code;
	uint16_t session_id;
	char file_name[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN];
} __attribute__((__packed__));

struct upload_data_reply_entry {
	uint32_t error_code;
	uint32_t offset;
	uint32_t size;
} __attribute__((__packed__));

struct upload_data_reply_telemetry {
	uint8_t telemetry_id;
	uint16_t session_id;
	struct upload_data_reply_entry entry[CONFIG_CS_LIB_CSP_UPLOAD_DATA_REPLY_ENTRY];
} __attribute__((__packed__));

void csp_upload_handler_init(void);
int csp_file_upload_open_cmd(uint8_t command_id, csp_packet_t *packet);
int csp_file_upload_data_cmd(uint8_t command_id, csp_packet_t *packet);

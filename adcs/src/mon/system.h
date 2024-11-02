/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

struct system_msg {
	uint32_t wall_clock;
	uint32_t sysup_time;
	uint32_t sw_version;
	uint8_t boot_count;
	uint8_t power_status;
	uint32_t fpga_version;
	uint8_t fpga_config_bank;
	uint32_t fpga_fallback_state;
	uint16_t received_command_count;
	uint8_t last_csp_port;
	uint8_t last_command_id;
	uint16_t ecc_error_count_by_auto;
	uint16_t ecc_error_count_by_bus;
	uint16_t sem_error_count;
};

void start_system_monitor(void);

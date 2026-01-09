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
	uint32_t fpga_clock_state;
	uint32_t fpga_sysmon_isr;
	uint16_t received_command_count;
	uint8_t last_csp_port;
	uint8_t last_command_id;
	int32_t last_time_sync_ret;
	uint16_t time_sync_count;
};

void start_system_monitor(void);

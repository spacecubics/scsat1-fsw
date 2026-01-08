/*
 * Copyright (c) 2025 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

struct ecc_msg {
	uint8_t ecc_enable_ecc_collect;
	uint8_t ecc_enable_mem_scrub;
	uint8_t ecc_enable_mem_scrub_arbit;
	uint16_t ecc_mem_scrub_cycle;
	uint16_t ecc_error_count_by_auto;
	uint16_t ecc_error_count_by_bus;
	uint32_t ecc_error_discard_count;
	uint32_t ecc_error_address;
	uint32_t sem_contoller_state;
	uint16_t sem_error_count;
};

void start_ecc_monitor(void);

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

void sc_hrmem_enable_ecc_collect(void);
void sc_hrmem_disable_ecc_collect(void);
bool sc_hrmem_is_enable_ecc_collect(void);
void sc_hrmem_enable_memory_scrub(void);
void sc_hrmem_disable_memory_scrub(void);
bool sc_hrmem_is_enable_memory_scrub(void);
uint16_t sc_hrmem_get_memscrub_cycle(void);
void sc_hrmem_set_memscrub_cycle(uint16_t cycle);
bool sc_hrmem_is_memscrub_arbitration(void);
int sc_hrmem_get_ecc_error_count(uint16_t *by_auto, uint16_t *by_bus);
uint16_t sc_hrmem_get_ecc_error_discard(void);
uint32_t sc_hrmem_get_ecc_error_address(void);

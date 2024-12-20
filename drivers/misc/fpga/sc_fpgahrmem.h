/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

void sc_hrmem_enable_ecc_collect(void);
void sc_hrmem_disable_ecc_collect(void);
void sc_hrmem_enable_memory_scrub(void);
void sc_hrmem_disable_memory_scrub(void);
int sc_hrmem_get_ecc_error_count(uint16_t *by_auto, uint16_t *by_bus);

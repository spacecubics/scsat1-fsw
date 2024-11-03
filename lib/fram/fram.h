/*
 * Copyright (c) 2022 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#define SC_FRAM_MEM0 (0U)
#define SC_FRAM_MEM1 (1U)

int sc_fram_write(uint8_t mem_no, uint32_t mem_addr, uint32_t size, uint8_t *val);
int sc_fram_read(uint8_t mem_no, uint32_t mem_addr, uint32_t size, uint8_t *val);
int sc_fram_clear_boot_count(void);
int sc_fram_update_boot_count(void);
int sc_fram_get_boot_count(uint8_t *boot_count);

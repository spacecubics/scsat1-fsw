/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

int sc_config_nor_erase(uint8_t bank, uint8_t id, off_t offset, size_t size);
int sc_config_nor_calc_crc(uint8_t bank, uint8_t id, off_t offset, size_t size, uint32_t *crc32);
void sc_config_nor_set_addr_mode(void);

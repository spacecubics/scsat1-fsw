/*
 * Space Cubics OBC TRCH Software
 *  Definitions for spi
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#pragma once

#include <stdint.h>

void spi_init (void);
void spi_get (void);
void spi_release (void);
uint8_t spi_trans (uint8_t buf);
uint8_t spi_read8(uint16_t addr);
uint16_t spi_read16(uint16_t addr);
uint32_t spi_read32(uint16_t addr);

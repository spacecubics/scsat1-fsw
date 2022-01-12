/*
 * Space Cubics OBC TRCH Software
 *  Definitions for spi
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#pragma once

void spi_init (void);
void spi_get (void);
void spi_release (void);
uint8_t spi_trans (uint8_t buf);

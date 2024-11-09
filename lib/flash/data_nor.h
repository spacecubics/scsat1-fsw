/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

int datafs_init(void);
int update_boot_count(const char *fname);
int data_nor_erase(uint8_t id);

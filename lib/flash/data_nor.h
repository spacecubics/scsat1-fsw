/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr/kernel.h>

struct syshk_stored_info {
	uint32_t tlm_block_size;
	uint32_t flash_size;
	uint32_t flash_start_addr;
	uint32_t erase_secotor_size;
	uint32_t max_tlm_num;
	uint32_t available_history_count;
};

int datafs_init(void);
int update_boot_count(const char *fname);
int data_nor_erase(uint8_t id);
int init_stored_tlm_info(struct syshk_stored_info *info);
bool is_flash_erased(const struct device *flash_dev, uint32_t addr, size_t len);
const struct device *stored_tlm_flash_dev();

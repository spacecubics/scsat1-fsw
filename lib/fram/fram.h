/*
 * Copyright (c) 2022 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#define SC_FRAM_MEM0 (0U)
#define SC_FRAM_MEM1 (1U)

struct fram_cfgmem_crc {
	uint8_t bank;
	uint8_t partition_id;
	uint32_t offset;
	uint32_t size;
	uint32_t crc32;
};

int sc_fram_write(uint8_t mem_no, uint32_t mem_addr, uint32_t size, uint8_t *val);
int sc_fram_read(uint8_t mem_no, uint32_t mem_addr, uint32_t size, uint8_t *val);
int sc_fram_clear_boot_count(void);
int sc_fram_update_boot_count(void);
int sc_fram_get_boot_count(uint8_t *boot_count);
int sc_fram_update_crc_for_file(const char* fname, uint32_t crc32);
int sc_fram_get_crc_for_file(char* fname, uint32_t *crc32);
int sc_fram_update_crc_for_cfgmem(struct fram_cfgmem_crc crc_info);
int sc_fram_get_crc_for_cfgmem(struct fram_cfgmem_crc *crc_info);
int sc_fram_clear_tlm_seq_num(void);
int sc_fram_update_tlm_seq_num(void);
int sc_fram_get_tlm_seq_num(uint32_t *tlm_seq_num);

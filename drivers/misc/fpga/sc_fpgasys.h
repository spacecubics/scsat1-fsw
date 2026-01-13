/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

enum sc_codemem {
	SC_HRMEM = 0,
	SC_ITCMEM = 1,
};

enum sc_cfgmem {
	SC_CFG_MEM_0 = 0,
	SC_CFG_MEM_1 = 1,
};

uint32_t sc_get_fpga_ip_ver(void);
uint32_t sc_get_fpga_build_hash(void);
uint32_t sc_get_fpga_dna_1(void);
uint32_t sc_get_fpga_dna_2(void);
enum sc_cfgmem sc_get_boot_cfgmem(void);
enum sc_cfgmem sc_get_cfgmem(void);
int sc_select_cfgmem(enum sc_cfgmem);
void sc_select_codemem(enum sc_codemem mem);
uint32_t sc_get_bootsts(void);
bool sc_is_fallback(void);

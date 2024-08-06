/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

uint32_t sc_fpgaconf_get_crc(void);
uint32_t sc_fpgaconf_get_far(void);
uint32_t sc_fpgaconf_get_fdro(void);
uint32_t sc_fpgaconf_get_cmd(void);
uint32_t sc_fpgaconf_get_ctl0(void);
uint32_t sc_fpgaconf_get_mask(void);
uint32_t sc_fpgaconf_get_stat(void);
uint32_t sc_fpgaconf_get_cor0(void);
uint32_t sc_fpgaconf_get_idcode(void);
uint32_t sc_fpgaconf_get_axss(void);
uint32_t sc_fpgaconf_get_cor1(void);
uint32_t sc_fpgaconf_get_wbstr(void);
uint32_t sc_fpgaconf_get_timer(void);
uint32_t sc_fpgaconf_get_rbcrc_sw(void);
uint32_t sc_fpgaconf_get_bootsts(void);
bool sc_fpgaconf_is_fallback(void);

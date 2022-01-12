/*
 * Space Cubics OBC TRCH Software
 *  Definitions for TMP175
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#pragma once

#include <stdint.h>

#define REG_TEMP   0x00
#define REG_CONFIG 0x01
#define REG_TLOW   0x02
#define REG_THIGH  0x03

struct tmp175_data {
        int  master;
        uint8_t addr;
        uint8_t data[2];
        int  error;
};

extern int tmp175_data_read (struct tmp175_data *td, int fpga_state);

/*
 * Space Cubics OBC TRCH Software
 *  Definitions for TMP175
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#pragma once

#include <stdint.h>
#include "fpga.h"

#define REG_TEMP   0x00
#define REG_CONFIG 0x01
#define REG_TLOW   0x02
#define REG_THIGH  0x03

#define TMP175_ERROR_I2C_UNACCESSIBLE		(1)
#define TMP175_ERROR_I2C_NAK			(2)

struct tmp175_data {
        uint8_t master;
        uint8_t addr;
        uint8_t data[2];
        uint8_t error;
};

extern int8_t tmp175_data_read (struct tmp175_data *td, enum FpgaState fpga_state);

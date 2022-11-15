/*
 * Space Cubics OBC TRCH Software
 *  Definitions for FPGA Control Utility
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define FPGA_BOOT_24MHZ 0
#define FPGA_BOOT_48MHZ 1
#define FPGA_BOOT_96MHZ 2

enum FpgaState{
        FPGA_STATE_POWER_DOWN,
        FPGA_STATE_POWER_OFF,
        FPGA_STATE_POWER_UP,
        FPGA_STATE_READY,
        FPGA_STATE_CONFIG,
        FPGA_STATE_ACTIVE,
        FPGA_STATE_LAST,
};

extern enum FpgaState fpga_init(void);
extern bool fpga_is_i2c_accessible (enum FpgaState state);
extern enum FpgaState fpga_state_control(bool activate_fpga);

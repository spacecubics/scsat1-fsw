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
        FPGA_STATE_POWER_OFF,
        FPGA_STATE_READY,
        FPGA_STATE_CONFIG,
        FPGA_STATE_ACTIVE,
        FPGA_STATE_LAST,
};

struct fpga_management_data {
        enum FpgaState state;
        int mem_select;
        unsigned boot_mode: 2;
        int time;
#ifdef CONFIG_ENABLE_WDT_RESET
        bool wdt_value;
        uint32_t wdt_last_tick;
#endif
};

extern enum FpgaState fpga_init(struct fpga_management_data *fmd);
extern bool fpga_is_i2c_accessible (enum FpgaState state);
extern void fpga_state_control(struct fpga_management_data *fmd, bool activate_fpga);

/*
 * Space Cubics OBC TRCH Software
 *  FPGA Control Utility
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#include <pic.h>
#include <trch.h>
#include <fpga.h>

fpga_st fpga_state = ST_POWER_OFF;

void fpga_init (void) {
        if (VDD_3V3) {
                fpga_config_wait(1);
                fpga_state = ST_FPGA_READY;
                FPGA_PROGRAM_B_DIR = 0;
                FPGA_PROGRAM_B = 1;
        } else {
                fpga_config_wait(0);
                fpga_state = ST_POWER_OFF;
                FPGA_PROGRAM_B_DIR = 1;
                FPGA_PROGRAM_B = 1;
        }
}

void fpga_config_wait (int sw) {
        if (sw) {
                FPGA_INIT_B = 1;
                FPGA_INIT_B_DIR = 0;
        } else {
                FPGA_INIT_B = 0;
                FPGA_INIT_B_DIR = 0;
        }
}

void fpga_reconfig (void) {
        FPGA_PROGRAM_B = 0;
        __delay_us(200);
        FPGA_PROGRAM_B = 1;
}

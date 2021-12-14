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

void fpga_init (void) {
        fmd.state = ST_POWER_OFF;
        fmd.count = 0;
        fmd.time = 0;
}

void check_fpga_power (void) {
        if (VDD_3V3)
                switch_fpga_state (ST_FPGA_READY);
}

int switch_fpga_state (fpga_st next_st) {
        if (fmd.state == ST_POWER_OFF) {
                if (next_st == ST_FPGA_READY) {
                        fmd.state = ST_FPGA_READY;
                        FPGA_INIT_B_DIR = 0;
                        FPGA_PROGRAM_B = 1;
                        FPGA_PROGRAM_B_DIR = 0;
                }
        } else if (fmd.state == ST_FPGA_READY) {
                if (next_st == ST_FPGA_CONFIG) {
                        if (fmd.count == 0)
                                FPGA_INIT_B_DIR = 1;
                        else
                                FPGA_PROGRAM_B = 1;
                        fmd.state = ST_FPGA_CONFIG;
                        fmd.count++;
                        return 0;
                }
        } else if (fmd.state == ST_FPGA_CONFIG) {
                if (next_st == ST_FPGA_READY) {
                        FPGA_PROGRAM_B = 0;
                        fmd.state = ST_FPGA_READY;
                        return 0;
                }
        }
        return 1;
}

/*
 * Space Cubics OBC TRCH Software
 *  FPGA Control Utility
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#include <pic.h>
#include "trch.h"
#include "fpga.h"

void fpga_init (fpga_management_data *fmd) {
        (*fmd).state = ST_POWER_OFF;
        (*fmd).config_ok = 0;
        (*fmd).count = 0;
        (*fmd).time = 0;
}

void f_power_off (fpga_management_data *fmd) {
        if (VDD_3V3) {
                (*fmd).state = ST_FPGA_READY;
                FPGA_INIT_B_DIR = 0;
                FPGA_PROGRAM_B = 1;
                FPGA_PROGRAM_B_DIR = 0;
        }
}

void f_fpga_ready (fpga_management_data *fmd) {
        if ((*fmd).config_ok) {
                if ((*fmd).count == 0)
                        FPGA_INIT_B_DIR = 1;
                else
                        FPGA_PROGRAM_B = 1;
                (*fmd).state = ST_FPGA_CONFIG;
                (*fmd).count++;
        }
}

void f_fpga_config (fpga_management_data *fmd) {
        if (!(*fmd).config_ok) {
                FPGA_PROGRAM_B = 0;
                (*fmd).state = ST_FPGA_READY;
        } else if (CFG_DONE)
                (*fmd).state = ST_FPGA_ACTIVE;
}

void f_fpga_active (fpga_management_data *fmd) {
        if (!(*fmd).config_ok) {
                FPGA_PROGRAM_B = 0;
                (*fmd).state = ST_FPGA_READY;
        } else if (CFG_DONE)
                TRCH_CFG_MEM_SEL = FPGA_CFG_MEM_SEL;
        else
                (*fmd).state = ST_FPGA_CONFIG;
}

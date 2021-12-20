/*
 * Space Cubics OBC TRCH Software
 *  Definitions for FPGA Control Utility
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

typedef enum { ST_POWER_OFF, ST_FPGA_READY, ST_FPGA_CONFIG, ST_FPGA_ACTIVE } fpga_st;

typedef struct s_fpga_management_data {
        fpga_st state;
        unsigned config_ok: 1;
        int count;
        int time;
} fpga_management_data;

typedef void (*STATEFUNC)(fpga_management_data *fmd);

extern void fpga_init (fpga_management_data *fmd);
extern void f_power_off (fpga_management_data *fmd);
extern void f_fpga_ready (fpga_management_data *fmd);
extern void f_fpga_config (fpga_management_data *fmd);
extern void f_fpga_active (fpga_management_data *fmd);

STATEFUNC fpgafunc[] = { f_power_off,
                         f_fpga_ready,
                         f_fpga_config,
                         f_fpga_active };

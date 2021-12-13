/*
 * Space Cubics OBC TRCH Software
 *  Definitions for FPGA Control Utility
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

typedef enum { ST_POWER_OFF, ST_FPGA_READY, ST_FPGA_CONFIG, ST_FPGA_ACTIVE } fpga_st;

struct fpga_management_data {
        fpga_st state;
        int count;
        int time;
};

struct fpga_management_data fmd;

extern void fpga_init (void);
extern void check_fpga_power (void);
extern int switch_fpga_state (fpga_st next_st);

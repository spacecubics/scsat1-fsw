/*
 * Space Cubics OBC TRCH Software
 *  Definitions for FPGA Control Utility
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

typedef enum { ST_POWER_OFF, ST_FPGA_READY, ST_FPGA_CONFIG, ST_FPGA_ACTIVE } fpga_st;

extern void fpga_init (void);
extern void fpga_config_wait (int sw);
extern void fpga_reconfig (void);

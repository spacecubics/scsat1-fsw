/*
 * Space Cubics OBC TRCH Software
 *  Definitions for FPGA Control Utility
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

typedef enum {
	ST_POWER_OFF,
	ST_FPGA_READY,
	ST_FPGA_CONFIG,
	ST_FPGA_ACTIVE,
	ST_LAST,
} fpga_st;

typedef struct s_fpga_management_data {
        fpga_st state;
        unsigned config_ok: 1;
        int count;
        int time;
} fpga_management_data;

typedef void (*STATEFUNC)(fpga_management_data *fmd);

extern void fpga_init (fpga_management_data *fmd);

extern STATEFUNC fpgafunc[ST_LAST];

/*
 * Space Cubics OBC TRCH Software
 *  Definitions for FPGA Control Utility
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

enum FpgaState{
	FPGA_STATE_POWER_OFF,
	FPGA_STATE_READY,
	FPGA_STATE_CONFIG,
	FPGA_STATE_ACTIVE,
	FPGA_STATE_LAST,
};

typedef struct s_fpga_management_data {
        enum FpgaState state;
        unsigned config_ok: 1;
        int count;
        int time;
} fpga_management_data;

typedef void (*STATEFUNC)(fpga_management_data *fmd);

extern void fpga_init (fpga_management_data *fmd);

extern STATEFUNC fpgafunc[FPGA_STATE_LAST];

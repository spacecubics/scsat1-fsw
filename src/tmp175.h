/*
 * Space Cubics OBC TRCH Software
 *  Definitions for TMP175
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#define REG_TEMP   0x00
#define REG_CONFIG 0x01
#define REG_TLOW   0x02
#define REG_THIGH  0x03

typedef struct s_tmp175_data {
        int  master;
        char addr;
        char data[2];
        int  error;
} tmp175_data;

extern int tmp175_data_read (tmp175_data *td, int fpga_state);

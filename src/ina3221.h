/*
 * Space Cubics OBC TRCH Software
 *  Definitions for INA3221
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#define REG_VOLTAGE_BASE 0x01

struct ina3221_data {
        int  master;
        char addr;
        int  channel;
        char shunt[2];
        char bus[2];
        int  error;
};

extern int ina3221_data_read (struct ina3221_data *id, int fpga_state, int type);

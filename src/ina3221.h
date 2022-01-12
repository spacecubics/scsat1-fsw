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
        uint8_t shunt[2];
        uint8_t bus[2];
        int  error;
};

extern int ina3221_data_read (struct ina3221_data *id, int fpga_state, int type);

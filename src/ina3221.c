/*
 * Space Cubics OBC TRCH Software
 *  INA3221 Driver
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#include <pic.h>
#include "interrupt.h"
#include "i2c-gpio.h"
#include "ina3221.h"

int ina3221_data_read (ina3221_data *id, int fpga_state, int type) {
        char addr = (char)((*id).addr << 1);
        char reg_addr = (char)(((*id).channel -1) * 2 + type + REG_VOLTAGE_BASE);
        int err = 0;
        if (i2c_get((*id).master, fpga_state))
                return 1;

        interrupt_lock(1);
        i2c_send_start((*id).master);
        err |= i2c_send_data((*id).master, addr);
        err |= i2c_send_data((*id).master, reg_addr);
        i2c_send_stop((*id).master);
        interrupt_lock(0);

        interrupt_lock(1);
        i2c_send_start((*id).master);
        err |= i2c_send_data((*id).master, addr | 0x01);
        if (type) {
                (*id).bus[0] = i2c_receive_data((*id).master);
                (*id).bus[1] = i2c_receive_data((*id).master);
        } else {
                (*id).shunt[0] = i2c_receive_data((*id).master);
                (*id).shunt[1] = i2c_receive_data((*id).master);
        }
        i2c_send_stop((*id).master);
        interrupt_lock(0);

        (*id).error = err;
        return 0;
}

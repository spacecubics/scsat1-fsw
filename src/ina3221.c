/*
 * Space Cubics OBC TRCH Software
 *  INA3221 Driver
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#include "ina3221.h"

#include <pic.h>
#include <stdint.h>
#include "interrupt.h"
#include "i2c.h"
#include "fpga.h"

int ina3221_data_read (struct ina3221_data *id, enum FpgaState fpga_state, int type) {
        uint8_t addr = (uint8_t)(id->addr << 1);
        uint8_t reg_addr = (uint8_t)((id->channel -1) * 2 + type + REG_VOLTAGE_BASE);
        int err = 0;

	if (!fpga_is_i2c_accessible(fpga_state))
		return 1;

        i2c_get(id->master);

        interrupt_lock();
        i2c_send_start(id->master);
        err |= i2c_send_data(id->master, addr);
        err |= i2c_send_data(id->master, reg_addr);
        i2c_send_stop(id->master);
        interrupt_unlock();

        interrupt_lock();
        i2c_send_start(id->master);
        err |= i2c_send_data(id->master, addr | 0x01);
        if (type) {
                id->bus[0] = i2c_receive_data(id->master);
                id->bus[1] = i2c_receive_data(id->master);
        } else {
                id->shunt[0] = i2c_receive_data(id->master);
                id->shunt[1] = i2c_receive_data(id->master);
        }
        i2c_send_stop(id->master);
        interrupt_unlock();

        id->error = err;
        return 0;
}

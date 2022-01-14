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

#define REG_VOLTAGE_BASE 0x01u

static uint8_t to_reg (uint8_t channel, enum Ina3221VoltageType type) {

        /*
         * reg 0: configuration
         * reg 1: channel 1 shunt
         * reg 2: channel 1 bus
         * reg 3: channel 2 shunt
         * reg 4: channel 2 bus
         * reg 5: channel 3 shunt
         * reg 6: channel 3 bus
         */
        return ((channel - 1) * 2) + REG_VOLTAGE_BASE + type;
}

int8_t ina3221_data_read (struct ina3221_data *id, enum FpgaState fpga_state, enum Ina3221VoltageType type) {
        uint8_t addr = (uint8_t)(id->addr << 1);
        uint8_t reg_addr;
        uint8_t nak = 0;
        int8_t ret = -1;

        if (!(type == INA3221_VOLTAGE_SHUNT || type == INA3221_VOLTAGE_BUS)) {
                id->error = INA3221_ERROR_INVALID_VOLTAGE_TYPE;
                return ret;
        }
        if (!fpga_is_i2c_accessible(fpga_state)) {
                id->error = INA3221_ERROR_I2C_UNACCESSIBLE;
                return ret;
        }

        reg_addr = to_reg(id->channel, type);
        i2c_get(id->master);

        interrupt_disable();
        i2c_send_start(id->master);
        nak |= i2c_send_data(id->master, addr);
        nak |= i2c_send_data(id->master, reg_addr);
        i2c_send_stop(id->master);
        interrupt_enable();

        interrupt_disable();
        i2c_send_start(id->master);
        nak |= i2c_send_data(id->master, addr | 0x01);
        if (type == INA3221_VOLTAGE_BUS) {
                id->bus[0] = i2c_receive_data(id->master);
                id->bus[1] = i2c_receive_data(id->master);
        } else {
                id->shunt[0] = i2c_receive_data(id->master);
                id->shunt[1] = i2c_receive_data(id->master);
        }
        i2c_send_stop(id->master);
        interrupt_enable();

        if (nak != 0) {
                id->error = INA3221_ERROR_I2C_NAK;
                return ret;
        }

        return 0;
}

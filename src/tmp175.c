/*
 * Space Cubics OBC TRCH Software
 *  TMP175 Driver
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#include "tmp175.h"

#include <pic.h>
#include <stdint.h>
#include "interrupt.h"
#include "i2c.h"
#include "fpga.h"

int8_t tmp175_data_read (struct tmp175_data *td, enum FpgaState fpga_state) {
        uint8_t addr = (uint8_t)(td->addr << 1);
        uint8_t nak = 0;
        int8_t ret = -1;

        if (fpga_is_i2c_accessible(fpga_state)) {
                td->error = TMP175_ERROR_I2C_UNACCESSIBLE;
                return ret;
        }

        i2c_get(td->master);

        interrupt_disable();
        i2c_send_start(td->master);
        nak |= i2c_send_data(td->master, addr);
        nak |= i2c_send_data(td->master, REG_TEMP);
        i2c_send_start(td->master);
        nak |= i2c_send_data(td->master, addr | 0x01);
        td->data[0] = i2c_receive_data(td->master);
        td->data[1] = i2c_receive_data(td->master);
        i2c_send_stop(td->master);
        interrupt_enable();

        if (nak != 0) {
                td->error = TMP175_ERROR_I2C_NAK;
                return ret;
        }

        return 0;
}

/*
 * Space Cubics OBC TRCH Software
 *  TMP175 Driver
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#include <pic.h>
#include <stdint.h>
#include "interrupt.h"
#include "i2c.h"
#include "tmp175.h"

int tmp175_data_read (struct tmp175_data *td, int fpga_state) {
        uint8_t addr = (uint8_t)(td->addr << 1);
        int err = 0;
        if (i2c_get(td->master, fpga_state))
                return 1;

        interrupt_lock();
        i2c_send_start(td->master);
        err |= i2c_send_data(td->master, addr);
        err |= i2c_send_data(td->master, REG_TEMP);
        i2c_send_start(td->master);
        err |= i2c_send_data(td->master, addr | 0x01);
        td->data[0] = i2c_receive_data(td->master);
        td->data[1] = i2c_receive_data(td->master);
        i2c_send_stop(td->master);
        interrupt_unlock();

        td->error = err;
        return 0;
}

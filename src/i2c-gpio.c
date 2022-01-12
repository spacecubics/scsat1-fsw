/*
 * Space Cubics OBC TRCH Software
 *  I2C-GPIO Driver
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#include <pic.h>
#include "trch.h"
#include "i2c-gpio.h"
#include "fpga.h"

int i2c_get (int m, int fpga_state) {
        if (fpga_state == FPGA_STATE_CONFIG |
            fpga_state == FPGA_STATE_ACTIVE)
                return 1;
        if (!m) {
                INT_SCL_DIR = 1;
                INT_SDA_DIR = 1;
        } else {
                EXT_SCL_DIR = 1;
                EXT_SDA_DIR = 1;
        }
        return 0;
}

void i2c_send_start (int m) {
        if (!m) {
                INT_SCL_DIR = 1;
                INT_SDA_DIR = 0;
                INT_SCL_DIR = 0;
        } else {
                EXT_SCL_DIR = 1;
                EXT_SDA_DIR = 0;
                EXT_SCL_DIR = 0;
        }
}

void i2c_send_stop (int m) {
        if (!m) {
                INT_SDA_DIR = 0;
                INT_SCL_DIR = 1;
                INT_SDA_DIR = 1;
        } else {
                EXT_SDA_DIR = 0;
                EXT_SCL_DIR = 1;
                EXT_SDA_DIR = 1;
        }
}

static void send_bit (int m, char l) {
        if (!m) {
                INT_SDA_DIR = l;
                INT_SCL_DIR = 1;
                INT_SCL_DIR = 0;
        } else {
                EXT_SDA_DIR = l;
                EXT_SCL_DIR = 1;
                EXT_SCL_DIR = 0;
        }
}

static void send_bit_befor_resp (int m, char l) {
        if (!m) {
                INT_SDA_DIR = l;
                INT_SCL_DIR = 1;
                INT_SCL_DIR = 0;
                INT_SDA_DIR = 1;
        } else {
                EXT_SDA_DIR = l;
                EXT_SCL_DIR = 1;
                EXT_SCL_DIR = 0;
                EXT_SDA_DIR = 1;
        }
}

static char receive_bit (int m) {
        char r = 0x00;
        if (!m) {
                INT_SDA_DIR = 1;
                INT_SCL_DIR = 1;
                r = INT_SDA_DAT;
                INT_SCL_DIR = 0;
        } else {
                EXT_SDA_DIR = 1;
                EXT_SCL_DIR = 1;
                r = INT_SDA_DAT;
                EXT_SCL_DIR = 0;
        }
        return r;
}

int i2c_send_data (int master, char data) {
        send_bit(master, (data >> 7));
        send_bit(master, (data >> 6));
        send_bit(master, (data >> 5));
        send_bit(master, (data >> 4));
        send_bit(master, (data >> 3));
        send_bit(master, (data >> 2));
        send_bit(master, (data >> 1));
        send_bit_befor_resp(master, data & 0x01);
        return receive_bit(master);
}

char i2c_receive_data (int master) {
        char data = 0x00;
        data = ((char)(data << 1) | receive_bit(master));
        data = ((char)(data << 1) | receive_bit(master));
        data = ((char)(data << 1) | receive_bit(master));
        data = ((char)(data << 1) | receive_bit(master));
        data = ((char)(data << 1) | receive_bit(master));
        data = ((char)(data << 1) | receive_bit(master));
        data = ((char)(data << 1) | receive_bit(master));
        data = ((char)(data << 1) | receive_bit(master));
        send_bit(master, 0);
        return data;
}

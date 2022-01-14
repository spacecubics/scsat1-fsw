/*
 * Space Cubics OBC TRCH Software
 *  I2C-GPIO Driver
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#include "i2c.h"

#include <pic.h>
#include <stdint.h>
#include "trch.h"

void i2c_get (int m) {
        if (!m) {
                I2C_PORT = I2C_PORT & ~INT_I2C_MASK;
                INT_SCL_DIR = 1;
                INT_SDA_DIR = 1;
        } else {
                I2C_PORT = I2C_PORT & ~EXT_I2C_MASK;
                EXT_SCL_DIR = 1;
                EXT_SDA_DIR = 1;
        }
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

static void send_bit (int m, uint8_t l) {
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

static void send_bit_befor_resp (int m, uint8_t l) {
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

static uint8_t receive_bit (int m) {
        uint8_t r = 0x00;
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

uint8_t i2c_send_data (int master, uint8_t data) {
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

uint8_t i2c_receive_data (int master) {
        uint8_t data = 0x00;
        data = ((uint8_t)(data << 1) | receive_bit(master));
        data = ((uint8_t)(data << 1) | receive_bit(master));
        data = ((uint8_t)(data << 1) | receive_bit(master));
        data = ((uint8_t)(data << 1) | receive_bit(master));
        data = ((uint8_t)(data << 1) | receive_bit(master));
        data = ((uint8_t)(data << 1) | receive_bit(master));
        data = ((uint8_t)(data << 1) | receive_bit(master));
        data = ((uint8_t)(data << 1) | receive_bit(master));
        send_bit(master, 0);
        return data;
}

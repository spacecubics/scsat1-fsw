/*
 * Space Cubics OBC TRCH Software
 *  Definitions for i2c
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#pragma once

#include <stdint.h>

void i2c_get (int m);
void i2c_send_start (int m);
void i2c_send_stop (int m);
int i2c_send_data (int master, uint8_t data);
uint8_t i2c_receive_data (int master);

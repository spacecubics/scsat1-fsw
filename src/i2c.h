/*
 * Space Cubics OBC TRCH Software
 *  Definitions for i2c
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

int i2c_get (int m, int fpga_state);
void i2c_send_start (int m);
void i2c_send_stop (int m);
int i2c_send_data (int master, char data);
char i2c_receive_data (int master);

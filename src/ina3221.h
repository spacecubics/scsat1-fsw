/*
 * Space Cubics OBC TRCH Software
 *  Definitions for INA3221
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#pragma once

#include <stdint.h>

#include "fpga.h"

enum Ina3221VoltageType {
	INA3221_VOLTAGE_SHUNT,
	INA3221_VOLTAGE_BUS,
};

struct ina3221_data {
        int  master;
        uint8_t addr;
        uint8_t channel;
        uint8_t shunt[2];
        uint8_t bus[2];
        int  error;
};

extern int ina3221_data_read (struct ina3221_data *id, enum FpgaState fpga_state, enum Ina3221VoltageType type);

/*
 * Copyright (c) 2022 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "common.h"

uint8_t read8(uint32_t addr)
{
	uint8_t val = sys_read8(addr);
	debug("     read8 [addr:0x%08X] 0x%02x\n", addr, val);
	return val;
}

void write8(uint32_t addr, uint8_t val)
{
	sys_write8(val, addr);
	debug("     write8 [addr:0x%08X] 0x%02x\n", addr, val);
}

uint16_t read16(uint32_t addr)
{
	uint16_t val = sys_read16(addr);
	debug("     read16 [addr:0x%08X] 0x%04x\n", addr, val);
	return val;
}

void write16(uint32_t addr, uint16_t val)
{
	sys_write16(val, addr);
	debug("     write16 [addr:0x%08X] 0x%04x\n", addr, val);
}

uint32_t read32(uint32_t addr)
{
	uint32_t val = sys_read32(addr);
	debug("     read32 [addr:0x%08X] 0x%08x\n", addr, val);
	return val;
}

void write32(uint32_t addr, uint32_t val)
{
	sys_write32(val, addr);
	debug("     write32 [addr:0x%08X] 0x%08x\n", addr, val);
}

bool assert32(uint32_t addr, uint32_t exp, uint32_t retry)
{
	uint32_t regval;

	regval = sys_read32(addr);
	if (regval == exp) {
		debug("  read32  [0x%08X] 0x%08x (exp:0x%08x)\n", addr, regval, exp);
		return true;
	} else if (retry == 0) {
		err("  read32  [0x%08X] 0x%08x (exp:0x%08x)\n", addr, regval, exp);
	}

	for (uint32_t i = 0; i < retry; i++) {
		regval = sys_read32(addr);
		if (regval == exp) {
			debug("  read32  [0x%08X] 0x%08x (exp:0x%08x) (retry:%d)\n", addr, regval,
			      exp, i + 1);
			return true;
		} else if (i + 1 == retry) {
			err("  read32  [0x%08X] 0x%08x (exp:0x%08x) (retry:%d)\n", addr, regval,
			    exp, i + 1);
		} else {
			debug("  read32  [0x%08X] 0x%08x (exp:0x%08x) (retry:%d)\n", addr, regval,
			      exp, i + 1);
		}
		k_usleep(1);
	}

	err("  !!! Assertion failed: retry count: %d\n", retry);
	return false;
}

void print_result(uint32_t test_no, uint32_t err_cnt)
{
	if (err_cnt == 0) {
		info("* [%d] Test Result: Passed\n", test_no);
	} else {
		info("* [%d] Test Result: Failed (Assertion count: %d)\n", test_no, err_cnt);
	}
}

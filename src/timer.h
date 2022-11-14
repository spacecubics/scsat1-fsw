/*
 * Space Cubics OBC TRCH Software
 *  Definitions for Timer
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#pragma once

#include <stdint.h>

#define HZ (1)
#define MSEC_PER_SEC (1000)
#define MSEC_TO_TICKS(ms) (((ms) + ((MSEC_PER_SEC / HZ) - 1)) / (MSEC_PER_SEC / HZ))

extern void timer2_init (void);
extern void timer2_ctrl (uint8_t control);
extern void timer2_isr (void);

extern uint32_t timer_get_ticks(void);

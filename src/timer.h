/*
 * Space Cubics OBC TRCH Software
 *  Definitions for Timer
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#pragma once

extern void timer2_init (void);
extern void timer2_ctrl (uint8_t control);
extern void timer2_isr (void);

struct interval_timer {
        int ms4;
        int etiming;
        unsigned event: 1;
};
extern struct interval_timer tmr2;

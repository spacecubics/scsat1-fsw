/*
 * Space Cubics OBC TRCH Software
 *  Definitions for Timer
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

extern void timer2_init (void);
extern void timer2_ctrl (char control);
extern void timer2_isr (void);

struct interval_timer {
        int ms4;
        int etiming;
        unsigned event: 1;
};
struct interval_timer tmr2;

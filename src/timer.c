/*
 * Space Cubics OBC TRCH Software
 *  Timer Driver
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#include <pic.h>
#include "timer.h"

struct interval_timer tmr2;

/*
 * Initialize Timer 2
 *  Timer Interval 4 ms
 */
void timer2_init (void) {
        T2CONbits.T2CKPS = 0b10;
        PR2 = 0xFA;
}

void timer2_ctrl (char control) {
        T2CONbits.TMR2ON = control;
        PIE1bits.TMR2IE = 1;
}

void timer2_isr (void) {
        PIR1bits.TMR2IF = 0;
        if ((tmr2.ms4 +1) % tmr2.etiming == 0)
                tmr2.event = 1;
        if (tmr2.ms4 == 249)
                tmr2.ms4 = 0;
        else
                tmr2.ms4++;
}

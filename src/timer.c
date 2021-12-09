/*
 * Space Cubics OBC TRCH Software
 *  Timer Driver
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#include <pic.h>
#include <timer.h>

/*
 * Initialize Timer 2
 *  Timer Interval 100 us
 */
void timer2_init (void) {
        T2CONbits.T2CKPS = 0b00;
        PR2 = 0x63;
}

void timer2_ctrl (char control) {
        T2CONbits.TMR2ON = control;
        PIE1bits.TMR2IE = 1;
}

void timer2_int (void) {
        PIR1bits.TMR2IF = 0;
        if (tmr2.us == 9) {
                tmr2.us = 0;
                if (tmr2.ms == 999) {
                        tmr2.ms = 0;
                        tmr2.event = 1;
                } else
                        tmr2.ms++;
        } else
                tmr2.us++;
}

/*
 * Space Cubics OBC TRCH Software
 *  Timer Driver
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#include "timer.h"

#include <pic.h>
#include <stdint.h>

#include "interrupt.h"

#define MSEC_PER_SEC (1000)
#define MSEC(x) (x)
#define TIMER_INTERVAL (MSEC(4))

static uint32_t current_ticks;

uint32_t timer_get_ticks(void)
{
        interrupt_disable();
        uint32_t ret = current_ticks;
        interrupt_enable();

        return ret;
}

/*
 * Initialize Timer 2
 *  Timer Interval 4 ms
 */
void timer2_init (void) {
        T2CONbits.T2CKPS = 0b10;
        PR2 = 0xFA;

        current_ticks = 0;
}

void timer2_ctrl (uint8_t control) {
        T2CONbits.TMR2ON = control;
        PIE1bits.TMR2IE = 1;
}

void timer2_isr (void) {
        static uint8_t count = 0;

        PIR1bits.TMR2IF = 0;

        count++;
        count %= MSEC_PER_SEC / TIMER_INTERVAL;

        if (count == 0) {
                current_ticks++;
        }
}

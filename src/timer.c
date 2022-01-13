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

#define SEC_IN_MSEC(x) (1000 * (x))
#define MSEC(x) (x)
#define TIMER_INTERVAL (MSEC(4))

uint32_t gtimer;

uint32_t timer_get_gtimer(void)
{
	interrupt_disable();
	uint32_t ret = gtimer;
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

	gtimer = 0;
}

void timer2_ctrl (uint8_t control) {
        T2CONbits.TMR2ON = control;
        PIE1bits.TMR2IE = 1;
}

void timer2_isr (void) {
	static uint8_t count = 0;

	PIR1bits.TMR2IF = 0;

	count++;
	count %= SEC_IN_MSEC(1) / TIMER_INTERVAL;

	if (count == 0) {
		gtimer++;
	}
}

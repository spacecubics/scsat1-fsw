/*
 * Space Cubics OBC TRCH Software
 *  Interrupt
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#include <pic.h>
#include "interrupt.h"

void interrupt_init (void) {
        INTCONbits.PEIE = 1;
        INTCONbits.GIE = 1;
}

void interrupt_lock () {
	INTCONbits.PEIE = 0;
	INTCONbits.GIE = 0;
}

void interrupt_unlock () {
	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;
}

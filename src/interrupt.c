/*
 * Space Cubics OBC TRCH Software
 *  Interrupt
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#include "interrupt.h"

#include <pic.h>

void interrupt_disable () {
	INTCONbits.PEIE = 0;
	INTCONbits.GIE = 0;
}

void interrupt_enable () {
	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;
}

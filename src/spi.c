/*
 * Space Cubics OBC TRCH Software
 *  SPI Driver
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#include <pic.h>
#include "trch.h"
#include "spi.h"

/*
 * SSPSTAT.SMP      : Sample bit        : 0b0
 * SSPSTAT.CKE      : Clock Edge        : 0b1
 * SSPCON.WCOL      : Write collision   : 0b0
 * SSPCON.SSPOV     : SSP Overflow      : 0b1
 * SSPCON.SSPEN     : Serial Port Enable: 0b1
 * SSPCON.CKP       : Clock Polarity    : 0b0
 * SSPCON.SSPM[3:0] : Serial Clock      : 0b0000
 */
void spi_init (void) {
        SPICAN_CS_B_DIR = 0;
        SPICAN_CS_B = 1;
        SPICAN_MOSI_DIR = 0;
        SPICAN_SCK_DIR = 0;
        SSPSTAT = 0x40;
        SSPCON = 0x60;
}

void spi_get (void) {
        SPICAN_CS_B = 0;
}

void spi_release (void) {
        SPICAN_CS_B = 1;
}

char spi_trans (char buf) {
        char rdata;
        SSPBUF = buf;
        while(!SSPSTATbits.BF);
        rdata = SSPBUF;
        return rdata;
}

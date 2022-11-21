/*
 * Space Cubics OBC TRCH Software
 *  SPI Driver
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#include "spi.h"

#include <pic.h>
#include <stdint.h>
#include "trch.h"

/*
 * SSPSTAT.SMP      : Sample bit        : 0b0
 * SSPSTAT.CKE      : Clock Edge        : 0b1
 *
 * SSPCON.WCOL      : Write collision   : 0b0
 * SSPCON.SSPOV     : SSP Overflow      : 0b1
 * SSPCON.SSPEN     : Serial Port Enable: 0b1
 * SSPCON.CKP       : Clock Polarity    : 0b0
 * SSPCON.SSPM[3:0] : Serial Clock      : 0b0000
 */
void spi_init (void) {
        SPICAN_CS_B_DIR = 0;
        SPICAN_CS_B = 1;
        SPICAN_SCK_DIR = 0;
        SPICAN_MISO_DIR = 1;
        SPICAN_MOSI_DIR = 0;

        SSPSTAT = 0x40;
        SSPCON = 0x60;
}

void spi_get (void) {
        SPICAN_CS_B = 0;
}

void spi_release (void) {
        SPICAN_CS_B = 1;
}

uint8_t spi_trans (uint8_t buf) {
        uint8_t rdata;
        SSPBUF = buf;
        while(!SSPSTATbits.BF);
        rdata = SSPBUF;
        return rdata;
}

uint8_t spi_read8(uint16_t addr)
{
        uint8_t buf0;
        uint8_t buf1;
        uint8_t ret;

        buf0 = ((addr >> 8) & 0xf) | 0x30;
        buf1 = addr & 0xff;

        spi_get();
        spi_trans(buf0);
        spi_trans(buf1);
        ret = spi_trans(0);
        spi_release();

        return ret;
}

uint16_t spi_read16(uint16_t addr)
{
        uint8_t buf0;
        uint8_t buf1;
        uint16_t ret;

        buf0 = ((addr >> 8) & 0xf) | 0x30;
        buf1 = addr & 0xff;

        spi_get();
        spi_trans(buf0);
        spi_trans(buf1);
        ret = (((uint16_t)spi_trans(0) & 0xff) << 0);
        ret = (((uint16_t)spi_trans(0) & 0xff) << 8) | ret;
        spi_release();

        return ret;
}

uint32_t spi_read32(uint16_t addr)
{
        uint8_t buf0;
        uint8_t buf1;
        uint32_t ret;

        buf0 = ((addr >> 8) & 0xf) | 0x30;
        buf1 = addr & 0xff;

        spi_get();
        spi_trans(buf0);
        spi_trans(buf1);
        ret = (((uint32_t)spi_trans(0) & 0xff) <<  0);
        ret = (((uint32_t)spi_trans(0) & 0xff) <<  8) | ret;
        ret = (((uint32_t)spi_trans(0) & 0xff) << 16) | ret;
        ret = (((uint32_t)spi_trans(0) & 0xff) << 24) | ret;
        spi_release();

        return ret;
}

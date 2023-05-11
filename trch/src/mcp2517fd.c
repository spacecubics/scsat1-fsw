/*
 * Copyright (c) 2023 Space Cubics, LLC.
 */

#include "can.h"

#include <xc.h>
#include <pic.h>
#include <stdio.h>
#include <stdbool.h>

#include "spi.h"
#include "utils.h"

#define C1CON      (0x000)
#define C1NBTCFG   (0x004)
#define C1DBTCFG   (0x008)
#define C1TDC      (0x00c)
#define C1TBC      (0x010)
#define C1INT      (0x01c)
#define C1RXIF     (0x024)
#define C1BDIAG0   (0x038)
#define C1BDIAG1   (0x03c)

#define C1FIFOCON1 (0x05c)
#define C1FIFOSTA1 (0x060)
#define C1FIFOUA1  (0x064)

#define C1FIFOCON2 (0x068)
#define C1FIFOSTA2 (0x06c)
#define C1FIFOUA2  (0x070)

#define C1FLTCON0  (0x1D0)

#define C1FLTOBJ0  (0x1F0)
#define C1MASK0    (0x1F4)

#define C1RAM      (0x400)

/* CiCON bit 31 - 24 */
#define C1CON_TXBWS_DELAY(x)          (((x) & 0xf) << 4)
#define C1CON_ABAT                    BIT(3)
#define C1CON_REQOP_CANFD             (0)
#define C1CON_REQOP_SLEEP             (1)
#define C1CON_REQOP_INTERNAL_LOOPBACK (2)
#define C1CON_REQOP_LISTEN            (3)
#define C1CON_REQOP_CONFIGURATION     (4)
#define C1CON_REQOP_EXTERNAL_LOOPBACK (5)
#define C1CON_REQOP_CAN20             (6)
#define C1CON_REQOP_RESTRICTED        (7)

/* CiCON bit 23 - 16 */
#define C1CON_TXQEN                   BIT(4)
#define C1CON_STEF                    BIT(3)
#define C1CON_SERR2LOM                BIT(2)
#define C1CON_ESIGM                   BIT(1)
#define C1CON_RTXAT                   BIT(0)

/* CiINT */
#define C1INT_RXIE                    BIT(17)

/* CiFIFOCONm */
#define C1FIFOCONm_TXEN               BIT(7)
#define C1FIFOCONm_RTREN              BIT(6)
#define C1FIFOCONm_RXTSEN             BIT(5)
#define C1FIFOCONm_TXATIE             BIT(4)
#define C1FIFOCONm_RXOVIE             BIT(3)
#define C1FIFOCONm_TFERFFIE           BIT(2)
#define C1FIFOCONm_TFHRFHIE           BIT(1)
#define C1FIFOCONm_TFNRFNIE           BIT(0)

/* CiFIFOCONm */
#define C1FIFOCONm_TXREQ              BIT(1)
#define C1FIFOCONm_UINC               BIT(0)

/* CiFLTCONm */
#define C1FLTCONm_FLTEN0              BIT(7)
#define C1FLTCONm_F0BP(x)             (x)

/* CiFIFOSTAm */
#define C1FIFOSTAm_TFNRFNIF           BIT(0)

/* CiFLTOBJm */
#define C1FLTOBJm_SID(x)              CAN_ID(x)

/* CiMASKm */
#define C1MASKm_MIDE                  BIT(30)
#define C1MASKm_MSID(x)               CAN_ID(x)

#define CAN_STANDARD_11IBITS_ID_MASK  (0x7ff)
#define CAN_ID(x)                     ((x) & CAN_STANDARD_11IBITS_ID_MASK)
#define CAN_DLC_MASK                  (0xf)
#define CAN_DLC(x)                    ((x) & CAN_DLC_MASK)


static void mpc2717fd_reset(void)
{
        uint8_t buf[2];

        spi_get();
        (void)spi_trans(0);
        (void)spi_trans(0);
        spi_release();
}

static void mpc2717fd_set_bit_timing(void)
{
        /* Setup bit rate and baud rate */
        /* we want 1 MHz baud based on 20 MHz SYSCLK.  That means the
           Time Quanta (Tq) is 50 ns and 20 Tq in one Nominal Bit
           time.  We want 80 % Sampling point.

           - Sync-Seg:   1 Tq (req.)
           - Phase Seg2: 4 Tq (20 * 0.2)

           At this point we have 15 Tq for Prop Seg and Phase Seg1.

           For now, let's go with 10:5.

           To do that, we need to set

           - BRP:    1 (to get 20 MHz; Baud Rate Prescaler)
           - TSEG1: 15 (Propagation Segment + Phase Segment 1)
           - TSEG2:  4 (Phase Segment 2)
           - SJW:    4 (Synchronization Jump Width)
         */
        spi_write8(3, C1NBTCFG + 0); /* SJW */
        spi_write8(3, C1NBTCFG + 1); /* TSEG2 */
        spi_write8(14, C1NBTCFG + 2); /* TSEG1 */
        spi_write8(0, C1NBTCFG + 3); /* BRP */
}

static void mpc2717fd_enable_tx_fifo(void)
{
        /* Have one and only one TX FIFO for now */

        /* Use FIFO 1 as Transimitting FIFO */
        spi_write8(C1FIFOCONm_TXEN, C1FIFOCON1);
}

static void mpc2717fd_enable_rx_fifo(void)
{
        /* Have one and only one RX FIFO for now */

        /* Use FIFO 2 as Receiving FIFO and enable Receive FIFO Not Empty Interrupt */
        spi_write8(C1FIFOCONm_TFNRFNIE, C1FIFOCON2);

        /* Enable Recive FIFO Interrupt */
        spi_write32(C1INT_RXIE, C1INT);
}

static void mpc2717fd_setup_filter0(uint16_t sid, uint16_t sid_mask)
{
        /* Disable the filter */
        spi_write8(0, C1FLTCON0);

        /* Setup filter object and mask */
        /* only for standard 11 bits ID for now */
        spi_write32(C1FLTOBJm_SID(sid), C1FLTOBJ0);
        spi_write32(C1MASKm_MIDE | C1MASKm_MSID(sid_mask), C1MASK0);

        /* Enable the filter on FIFO 2 */
        spi_write8(C1FLTCONm_F0BP(2) | C1FLTCONm_FLTEN0, C1FLTCON0);
}


static bool mpc2717fd_recv_fifo_is_empty(void)
{
        return !(spi_read8(C1FIFOSTA2) & C1FIFOSTAm_TFNRFNIF);
}

static void mpc2717fd_recv_fifo_increment(void)
{
        /* After the Receive Message Object is read from RAM, the RX
         * FIFO needs to be incremented by setting
         * CiFIFOCONm.UINC. This will cause the CAN FD Controller
         * module to increment the tail of the FIFO and update
         * CiFIFOUAm. */
        spi_write8(C1FIFOCONm_UINC, C1FIFOCON2 + 1);
}

static uint8_t mpc2717fd_recv_fifo_get_dlc(uint16_t addr)
{
        return spi_read32(C1RAM + 4 + addr) & CAN_DLC_MASK;
}

void mpc2717fd_spi_read32(canbuf_t *buf, uint8_t len, uint16_t addr)
{
        uint8_t buf0;
        uint8_t buf1;

        buf0 = ((addr >> 8) & 0xf) | 0x30;
        buf1 = addr & 0xff;

        spi_get();
        spi_trans(buf0);
        spi_trans(buf1);

        if (len) {
                buf->data[0] = spi_trans(0);
                buf->data[1] = spi_trans(0);
                buf->data[2] = spi_trans(0);
                buf->data[3] = spi_trans(0);
        }
        if (len > 4) {
                buf->data[4] = spi_trans(0);
                buf->data[5] = spi_trans(0);
                buf->data[6] = spi_trans(0);
                buf->data[7] = spi_trans(0);
        }

        spi_release();

        return;
}

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

static uint8_t mpc2717fd_recv_fifo_copy_to_buffer(canbuf_t *buf)
{
        uint16_t addr;
        uint8_t dlc;

        addr = spi_read16(C1FIFOUA2);

        dlc = mpc2717fd_recv_fifo_get_dlc(addr);
        dlc = MIN(dlc, sizeof(canbuf_t));

        mpc2717fd_spi_read32(buf, dlc, addr);

        return dlc;
}

int can_recv(canbuf_t *buf)
{
        int ret = -1;

        if (!mpc2717fd_recv_fifo_is_empty()) {
                ret = mpc2717fd_recv_fifo_copy_to_buffer(buf);
                mpc2717fd_recv_fifo_increment();
                /* Unconditionaly incremented the fifo. This means
                 * that unread data is lost. */
        }

        return ret;
}


static void mpc2717fd_spi_write(canbuf_t *buf, uint8_t len, uint16_t addr)
{
        uint8_t buf0;
        uint8_t buf1;

        buf0 = ((addr >> 8) & 0xf) | 0x20;
        buf1 = addr & 0xff;

        spi_get();
        spi_trans(buf0);
        spi_trans(buf1);

        /* Write commands must always write a multiple of 4 data
         * bytes. After every fourth data byte, with the falling edge
         * on SCK, the RAM Word gets written. In case nCS goes high
         * before a multiple of 4 data bytes is received on SDI, the
         * data of the incomplete Word will not be written to RAM. */

        if (len) {
                (void)spi_trans(buf->data[0]);
                (void)spi_trans(buf->data[1]);
                (void)spi_trans(buf->data[2]);
                (void)spi_trans(buf->data[3]);
        }
        if (len > 4) {
                (void)spi_trans(buf->data[4]);
                (void)spi_trans(buf->data[5]);
                (void)spi_trans(buf->data[6]);
                (void)spi_trans(buf->data[7]);
        }

        spi_release();

        return;
}

static void mpc2717fd_send(uint32_t sid, canbuf_t *buf, uint8_t len)
{
        uint16_t addr;

        addr = spi_read16(C1FIFOUA1);

        spi_write32(CAN_ID(sid), (uint16_t)C1RAM + addr);
        spi_write32(CAN_DLC(len), (uint16_t)C1RAM + 4 + addr);
        mpc2717fd_spi_write(buf, len, (uint16_t)C1RAM + 8 + addr);

        /* send it */
        /* UINC and TXREQ of the CiFIFOCONm register must be set at
         * the same time after appending a message. -- DS20005678E */
        spi_write8(C1FIFOCONm_TXREQ | C1FIFOCONm_UINC, C1FIFOCON1 + 1);
}

void can_set_filter(uint8_t id, uint8_t mask)
{
        mpc2717fd_setup_filter0(id, mask);
}

void can_send(uint32_t id, canbuf_t *buf, uint8_t len)
{
        mpc2717fd_send(id, buf, len);
}

void can_init(void)
{
        spi_init();

        mpc2717fd_reset();
        mpc2717fd_set_bit_timing();
        mpc2717fd_enable_tx_fifo();
        mpc2717fd_enable_rx_fifo();

        /* with RTXAT = 1: Restricted retransmission attempts,
         * CiFIFOCONm.TXAT is used. */
        spi_write8(C1CON_TXQEN | C1CON_STEF | C1CON_RTXAT, C1CON + 2);

        /* move to CAN 2.0 mode */
        spi_write8(C1CON_TXBWS_DELAY(0) | C1CON_REQOP_CAN20, C1CON + 3);
}

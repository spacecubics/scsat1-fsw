/*
 * Space Cubics OBC TRCH Software
 *  USART Driver
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#include "usart.h"

#include <xc.h>
#include <pic.h>
#include <string.h>

#include "interrupt.h"

/* Variables shared with main */
struct usart_tx_msg tx_msg;
static struct usart_rx_msg rx_msg;

/*
 * USART Initialize
 *  - Asynchronouse mode
 *  - 8 bit transmission
 *  - No parity bit
 *  - boardrate 9600 [bps]
 */
void usart_init (void) {
        TXSTAbits.TX9 = 0;
        TXSTAbits.SYNC = 0;
        TXSTAbits.BRGH = 1;
        TXSTAbits.TX9D = 0;
        TXSTAbits.TXEN = 1;
        RCSTAbits.SPEN = 1;
        RCSTAbits.RX9 = 0;
        RCSTAbits.ADDEN = 0;
        RCSTAbits.RX9D = 0;
        SPBRG = 0x19;
        tx_msg.active = 0;
        rx_msg.active = 0;
        rx_msg.addr = 0;
}

void putch(char ch)
{
        if (ch == '\n') {
                while (!PIR1bits.TXIF);
                TXREG = '\r';
                /* AN774: There is a delay of one instruction cycle
                 * after writing to TXREG, before TXIF gets
                 * cleared. */
                NOP();
        }
        while (!PIR1bits.TXIF);
        TXREG = ch;
}

void usart_start_receive (void) {
        char buf;
        RCSTAbits.CREN = 1;
        if (RCSTAbits.FERR || PIR1bits.RCIF)
                buf = RCREG;
        rx_msg.active = 0;
        rx_msg.addr = 0;
        rx_msg.err = 0;
        PIE1bits.RCIE = 1;
}

#define RX_MSG_DELIMITER (0x0d)

void usart_receive_msg_isr (void) {
        char buf;

        if (RCSTAbits.OERR || RCSTAbits.FERR) {
                rx_msg.active = 1;
                rx_msg.err = 1;
                if (RCSTAbits.OERR)
                        RCSTAbits.CREN = 0;
                else
                        buf = RCREG;
        } else {
                buf = RCREG;
                putch(buf);
                if (buf == RX_MSG_DELIMITER)
                        rx_msg.active = 1;
                else {
                        rx_msg.msg[rx_msg.addr] = buf;
                        rx_msg.addr = (rx_msg.addr+1) % MSG_LEN;
                }
        }
}

void usart_receive_msg_clear (void) {
        interrupt_disable();
        memset(rx_msg.msg, 0, MSG_LEN);
        rx_msg.active = 0;
        rx_msg.err = 0;
        rx_msg.addr = 0;
        interrupt_enable();
}

void usart_copy_received_msg(char *msg) {
        interrupt_disable();
        memcpy(msg, rx_msg.msg, MSG_LEN);
        interrupt_enable();
}

bool usart_is_received_msg_active (void) {
        return rx_msg.active;
}

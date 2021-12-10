/*
 * Space Cubics OBC TRCH Software
 *  USART Driver
 *
 * (C) Copyright 2021
 *         Space Cubics, LLC
 *
 */

#include <pic.h>
#include <usart.h>

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

void send_char (char msg) {
        while (!PIR1bits.TXIF);
        TXREG = msg;
}

void send_msg (char *msg) {
        int i;
        tx_msg.msg = msg;
        tx_msg.active = 1;

        // Transfer start
        while (tx_msg.active) {
                while (!PIR1bits.TXIF);
                if (*tx_msg.msg == 0x00)
                        tx_msg.active = 0;
                else {
                        TXREG = *tx_msg.msg;
                        tx_msg.msg++;
                }
        }
        for (i=0; i<2; i++) {
                while (!PIR1bits.TXIF);
                TXREG = newline[i];
        }
}

void start_usart_receive (void) {
        char buf;
        RCSTAbits.CREN = 1;
        if (RCSTAbits.FERR | PIR1bits.RCIF)
                buf = RCREG;
        rx_msg.active = 0;
        rx_msg.addr = 0;
        rx_msg.err = 0;
        PIE1bits.RCIE = 1;
}

void receive_msg_int (void) {
        char buf;
        if (RCSTAbits.OERR | RCSTAbits.FERR) {
                rx_msg.active = 1;
                rx_msg.err = 1;
                if (RCSTAbits.OERR)
                        RCSTAbits.CREN = 0;
                else
                        buf = RCREG;
        } else {
                buf = RCREG;
                send_char(buf);
                if (buf == rx_msg_delimiter)
                        rx_msg.active = 1;
                else {
                        rx_msg.msg[rx_msg.addr] = buf;
                        rx_msg.addr = (rx_msg.addr+1) % MSG_LEN;
                }
        }
}

void receive_msg_clear (void) {
        rx_msg.active = 0;
        rx_msg.err = 0;
        rx_msg.addr = 0;
}

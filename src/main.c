/*
 * Space Cubics OBC TRCH Software
 *
 * (C) Copyright Space Cubics, LLC
 *
 */

#include <xc.h>
#include <pic.h>
#include <trch.h>
#include <fpga_ctrl.h>
#include <usart.h>
#include <timer.h>

// PIC16LF877A Configuration Bit Settings
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define _XTAL_FREQ 4000000

void __interrupt() isr(void) {
        if (PIE1bits.TMR2IE && PIR1bits.TMR2IF) {
                timer2_int();
        }
        if (PIR1bits.RCIF) {
                receive_msg_int();
        }
}

void trch_init (void) {
        ADCON1 = 0x07;
        TRISA = TRISA_INIT;
        TRISB = TRISB_INIT;
        TRISC = TRISC_INIT;
        TRISD = TRISD_INIT;
        TRISE = TRISE_INIT;
        INTCONbits.PEIE = 1;
        INTCONbits.GIE = 1;
}

void main (void) {
        // Initialize trch-firmware
        unsigned long gtimer = 0;
        trch_init();
        usart_init();
        timer2_init();
        TRISA = 0x3F;
        TRISE = 0x01;
        int first = 1;
        timer2_ctrl(1);

        /*
         * Space Cubics OBC TRCH-Firmware Main
         */
        send_msg("SC OBC Firmware v1.0 for board evaluation\n");
        start_usart_receive();
        while (1) {
                if (tmr2.event) {
                        if (gtimer == 1 && first) {
                                TRISAbits.TRISA1 = 0;
                                first = 0;
                        }
                        gtimer++;
                        tmr2.event = 0;
                }
        }
        return;
}

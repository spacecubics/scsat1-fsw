/*
 * Space Cubics OBC TRCH Software
 *
 * (C) Copyright Space Cubics, LLC
 *
 */

#include <xc.h>
#include <pic.h>
#include <trch.h>
#include <fpga.h>
#include <i2c-gpio.h>
#include <usart.h>
#include <timer.h>
#include <interrupt.h>
#include <string.h>

// PIC16LF877A Configuration Bit Settings
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define BUF_LEN 20
extern void cmd_parser (void);
extern void conv_message (char *data, int count);

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
}

void main (void) {
        // Initialize trch-firmware
        unsigned long gtimer = 0;
        trch_init();
        fpga_init();
        usart_init();
        timer2_init();
        TRISE = 0x01;
        timer2_ctrl(1);
        interrupt_init();

        /*
         * Space Cubics OBC TRCH-Firmware Main
         */
        send_msg("SC OBC Firmware v1.0 for board evaluation");
        start_usart_receive();
        while (1) {
                if (fmd.state == ST_FPGA_ACTIVE)
                        TRCH_CFG_MEM_SEL = FPGA_CFG_MEM_SEL;

                if (rx_msg.active)
                        cmd_parser();

                if (tmr2.event) {
                        gtimer++;
                        tmr2.event = 0;
                }
        }
        return;
}

void cmd_parser (void) {
        char buf[BUF_LEN] = { };

        send_msg(rx_msg.msg);
        if(!strcmp(rx_msg.msg,"ld01")) {
                if ((PORTE & (_PORTE_RE1_MASK | _PORTE_RE2_MASK)) == 0x02)
                        PORTE = 0x04;
                else
                        PORTE = 0x02;

        // FPGA Command
        } else if (!strcmp(rx_msg.msg,"fc")) {
                send_msg("fpga configuration");
                if (switch_fpga_state(ST_FPGA_CONFIG))
                        send_msg(" Configuration Error");
        } else if (!strcmp(rx_msg.msg,"fu")) {
                send_msg("fpga unconfiguration");
                if (switch_fpga_state(ST_FPGA_READY))
                        send_msg(" Unconfiguration Error");

        // Configuration Memory Select
        } else if (!strncmp(rx_msg.msg,"bc",2)) {
                if (fmd.state != ST_FPGA_ACTIVE) {
                        send_msg("buffer select control");
                        buf[0] = *(rx_msg.msg+2) - 0x30;
                        if (buf[0] == 0x00) {
                                TRCH_CFG_MEM_SEL = 0;
                                send_msg("  level 0");
                        } else if (buf[0] == 0x01) {
                                TRCH_CFG_MEM_SEL = 1;
                                send_msg("  level 1");
                        }
                } else
                        send_msg("fpga state error");

        // I2C-GPIO Command
        } else if (!strcmp(rx_msg.msg,"i2cr")) {
                if (!get_i2c()) {
                        interrupt_lock(1);
                        PORTE = 0x04;
                        send_start(0);
                        i2c_send_data(0, 0x90);
                        i2c_send_data(0, 0x03);
                        send_start(0);
                        i2c_send_data(0, 0x91);
                        buf[0] = i2c_receive_data(0);
                        buf[1] = i2c_receive_data(0);
                        send_stop(0);
                        interrupt_lock(0);
                        conv_message(buf, 2);
                } else
                        send_msg("i2c bus error");

        } else
                send_msg("cmd error");
        receive_msg_clear();
}

void conv_message (char *data, int count) {
        int i, j=0;
        char text[] = "0123456789ABCDEF";
        char message[BUF_LEN] = { };
        for (i=0; i<count; i++) {
                message[j]   = text[(*data & 0xF0) >> 4];
                message[j+1] = text[*data & 0x0F];
                j=j+2;
                data++;
        }
        send_msg(message);
}

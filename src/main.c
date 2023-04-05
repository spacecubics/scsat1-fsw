/*
 * Space Cubics OBC TRCH Software
 *
 * (C) Copyright 2021-2022 Space Cubics, LLC
 *
 */

#include <xc.h>
#include <pic.h>
#include <stdio.h>

#include "trch.h"
#include "utils.h"
#include "fpga.h"
#include "spi.h"
#include "i2c.h"
#include "tmp175.h"
#include "ina3221.h"
#include "usart.h"
#include "timer.h"
#include "interrupt.h"
#include "ioboard.h"
#include "version.h"

// PIC16LF877A Configuration Bit Settings
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

static void __interrupt() isr(void) {
        if (PIR1bits.TMR2IF) {
                timer2_isr();
        }
        if (PIR1bits.RCIF) {
                usart_receive_msg_isr();
        }
}

static void trch_init (void) {
        ADCON1 = 0x07;
        PORTA = PORTA_INIT;
        TRISA = TRISA_INIT;
        PORTB = PORTB_INIT;
        TRISB = TRISB_INIT;
        PORTC = PORTC_INIT;
        TRISC = TRISC_INIT;
        PORTD = PORTD_INIT;
        TRISD = TRISD_INIT;
        PORTE = PORTE_INIT;
        TRISE = TRISE_INIT;
}

void main (void)
{
        enum FpgaState fpga_state;
        enum FpgaGoal activate_fpga = FPGA_SHUTDOWN;
        int config_memory = 0;
        int boot_mode = FPGA_BOOT_48MHZ;

        // Initialize trch-firmware
        trch_init();
        fpga_program_maybe();
        fpga_state = fpga_init();

        ioboard_init();
        spi_init();
        usart_init();
        timer2_init();
        timer2_ctrl(1);
        interrupt_enable();

        /*
         * Space Cubics OBC TRCH-Firmware Main
         */
        puts("SC OBC TRCH-FW " VERSION);

        while (1) {
                fpga_state = fpga_state_control(activate_fpga, config_memory, boot_mode);
                switch(fpga_state) {
                case FPGA_STATE_POWER_DOWN:
                        break;

                case FPGA_STATE_POWER_OFF:
                        activate_fpga = FPGA_ACTIVATE;
                        break;

                case FPGA_STATE_POWER_UP:
                        break;

                case FPGA_STATE_READY:
                        break;

                case FPGA_STATE_CONFIG:
                        break;

                case FPGA_STATE_ACTIVE:
                        if (FPGA_PWR_CYCLE_REQ) {
                                activate_fpga = FPGA_SHUTDOWN;
                        }

                        break;

                case FPGA_STATE_ERROR:
                        activate_fpga = FPGA_SHUTDOWN;
                        /* activate_fpga = FPGA_RECONFIGURE; */
                        config_memory = !config_memory;
                        break;

                default:
                        break;
                }
        }
        return;
}

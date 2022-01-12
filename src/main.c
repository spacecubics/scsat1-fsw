/*
 * Space Cubics OBC TRCH Software
 *
 * (C) Copyright 2021-2022 Space Cubics, LLC
 *
 */

#define AUTO_CONFIG 1

#include <xc.h>
#include <pic.h>

#include "trch.h"
#include "fpga.h"
#include "spi.h"
#include "i2c.h"
#include "tmp175.h"
#include "ina3221.h"
#include "usart.h"
#include "timer.h"
#include "interrupt.h"
#include "cmd_parser.h"

// PIC16LF877A Configuration Bit Settings
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

struct trch_state {
        unsigned long gtimer;
        struct fpga_management_data fmd;
};

struct trch_bstatus {
        struct tmp175_data ts1;
        struct tmp175_data ts2;
        struct tmp175_data ts3;
        struct ina3221_data vm1v0;
        struct ina3221_data vm1v8;
        struct ina3221_data vm3v3;
        struct ina3221_data vm3v3a;
        struct ina3221_data vm3v3b;
        struct ina3221_data vm3v3i;
};

extern void get_vm (struct ina3221_data *id, int fpga_state, int type);
extern void get_vm_all (struct trch_state *tst, struct trch_bstatus *tbs);
extern void get_tmp (struct tmp175_data *td, int fpga_state);
extern void get_tmp_all (struct trch_state *tst, struct trch_bstatus *tbs);

void __interrupt() isr(void) {
        if (PIE1bits.TMR2IE && PIR1bits.TMR2IF) {
                timer2_isr();
        }
        if (PIR1bits.RCIF) {
                usart_receive_msg_isr();
        }
}

void trch_init (void) {
        ADCON1 = 0x07;
        TRISA = TRISA_INIT;
        PORTA = PORTA_INIT;
        TRISB = TRISB_INIT;
        PORTB = PORTB_INIT;
        TRISC = TRISC_INIT;
        PORTC = PORTC_INIT;
        TRISD = TRISD_INIT;
        PORTD = PORTD_INIT;
        TRISE = TRISE_INIT;
        PORTE = PORTE_INIT;
}

void main (void) {
        struct trch_state tst;
        struct trch_bstatus tbs;
        // Initialize trch-firmware
        trch_init();
        fpga_init(&(tst.fmd));
#if AUTO_CONFIG
        tst.fmd.config_ok = 1;
#endif
        spi_init();
        usart_init();
        tst.gtimer = 0;
        timer2_init();
        tmr2.etiming = 250;
        timer2_ctrl(1);
        interrupt_unlock();

        /*
         * Space Cubics OBC TRCH-Firmware Main
         */
        usart_send_msg("SC OBC TRCH-FW v0.3");

        /*
         *  Get Board Status
         *  ---------------------------------------------------
         *  Temperature Sensor 1 (IC16 beside TRCH)
         *   I2C Master   : 0
         *   Slave Address: 0x4C
         *  Temperature Sensor 2 (IC17 beside FPGA)
         *   I2C Master   : 0
         *   Slave Address: 0x4D
         *  Temperature Sensor 3 (IC20 beside FPGA and Power)
         *   I2C Master   : 0
         *   Slave Address: 0x4E
         *  Current/Voltage Monitor 1 (IC22)
         *   I2C Master   : 0
         *   Slave Address: 0x40
         *   - Channel 1 (VDD 1V0)
         *   - Channel 2 (VDD 1V8)
         *   - Channel 3 (VDD 3V3)
         *  Current/Voltage Monitor 2 (IC21)
         *   I2C Master   : 0
         *   Slave Address: 0x41
         *   - Channel 1 (VDD 3V3A)
         *   - Channel 2 (VDD 3V3B)
         *   - Channel 3 (VDD 3V3IO)
         */
        tbs.ts1.master     = 0;
        tbs.ts1.addr       = 0x4C;
        tbs.ts2.master     = 0;
        tbs.ts2.addr       = 0x4D;
        tbs.ts3.master     = 0;
        tbs.ts3.addr       = 0x4E;

        tbs.vm1v0.master   = 0;
        tbs.vm1v0.addr     = 0x40;
        tbs.vm1v0.channel  = 1;
        tbs.vm1v8.master   = 0;
        tbs.vm1v8.addr     = 0x40;
        tbs.vm1v8.channel  = 2;
        tbs.vm3v3.master   = 0;
        tbs.vm3v3.addr     = 0x40;
        tbs.vm3v3.channel  = 3;
        tbs.vm3v3a.master  = 0;
        tbs.vm3v3a.addr    = 0x41;
        tbs.vm3v3a.channel = 1;
        tbs.vm3v3b.master  = 0;
        tbs.vm3v3b.addr    = 0x41;
        tbs.vm3v3b.channel = 2;
        tbs.vm3v3i.master  = 0;
        tbs.vm3v3i.addr    = 0x41;
        tbs.vm3v3i.channel = 3;

        get_vm_all(&tst,  &tbs);
        get_tmp_all(&tst, &tbs);

        usart_start_receive();
        while (1) {
                // FPGA State Control
                fpgafunc[tst.fmd.state](&tst.fmd);

                if (rx_msg.active)
                        cmd_parser(&tst.fmd);

                if (tmr2.event) {
                        tst.gtimer++;
                        tmr2.event = 0;
                }
        }
        return;
}

void get_vm (struct ina3221_data *id, int fpga_state, int type) {
        int retry = 3;
        while (retry) {
                ina3221_data_read(id, fpga_state, type);
                if (id->error)
                        retry--;
                else
                        return;
        }
}

void get_tmp (struct tmp175_data *td, int fpga_state) {
        int retry = 3;
        while (retry) {
                tmp175_data_read(td, fpga_state);
                if (td->error)
                        retry--;
                else
                        return;
        }
}

void get_vm_all (struct trch_state *tst, struct trch_bstatus *tbs) {
        get_vm(&tbs->vm3v3a, tst->fmd.state, 1);
        get_vm(&tbs->vm3v3b, tst->fmd.state, 1);
        get_vm(&tbs->vm1v0, tst->fmd.state, 1);
        get_vm(&tbs->vm1v8, tst->fmd.state, 1);
        get_vm(&tbs->vm3v3, tst->fmd.state, 1);
        get_vm(&tbs->vm3v3i, tst->fmd.state, 1);
        get_vm(&tbs->vm3v3a, tst->fmd.state, 0);
        get_vm(&tbs->vm3v3b, tst->fmd.state, 0);
        get_vm(&tbs->vm1v0, tst->fmd.state, 0);
        get_vm(&tbs->vm1v8, tst->fmd.state, 0);
        get_vm(&tbs->vm3v3, tst->fmd.state, 0);
        get_vm(&tbs->vm3v3i, tst->fmd.state, 1);
}

void get_tmp_all (struct trch_state *tst, struct trch_bstatus *tbs) {
        get_tmp(&tbs->ts1, tst->fmd.state);
        get_tmp(&tbs->ts2, tst->fmd.state);
        get_tmp(&tbs->ts3, tst->fmd.state);
}

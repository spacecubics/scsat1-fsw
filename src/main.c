/*
 * Space Cubics OBC TRCH Software
 *
 * (C) Copyright Space Cubics, LLC
 *
 */

#define AUTO_CONFIG 1

#include <xc.h>
#include <pic.h>
#include <trch.h>
#include <fpga.h>
#include <spi.h>
#include <i2c-gpio.h>
#include <tmp175.h>
#include <ina3221.h>
#include <usart.h>
#include <timer.h>
#include <interrupt.h>
#include <string.h>
#include <stdio.h>

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
extern void conv_message (char *data, int count);
extern char conv_asc2hex (char data);

typedef struct s_trch_state {
        unsigned long gtimer;
        fpga_management_data fmd;
} trch_state;

typedef struct s_trch_bstatus {
        tmp175_data ts1;
        tmp175_data ts2;
        tmp175_data ts3;
        ina3221_data vm1v0;
        ina3221_data vm1v8;
        ina3221_data vm3v3;
        ina3221_data vm3v3a;
        ina3221_data vm3v3b;
        ina3221_data vm3v3i;
} trch_bstatus;

extern void get_vm (ina3221_data *id, int type);
extern void get_vm_all (trch_bstatus *tbs);
extern void get_tmp (tmp175_data *td, int fpga_state);
extern void get_tmp_all (trch_state *tst, trch_bstatus *tbs);
extern void cmd_parser (trch_state *tst);

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
        trch_state tst;
        trch_bstatus tbs;
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
        interrupt_init();

        /*
         * Space Cubics OBC TRCH-Firmware Main
         */
        send_msg("SC OBC TRCH-FW v0.2");

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

        get_vm_all(&tbs);
        get_tmp_all(&tst, &tbs);

        start_usart_receive();
        while (1) {
                // FPGA State Control
                fpgafunc[tst.fmd.state](&tst.fmd);

                if (rx_msg.active)
                        cmd_parser(&tst);

                if (tmr2.event) {
                        tst.gtimer++;
                        tmr2.event = 0;
                }
        }
        return;
}

void get_vm (ina3221_data *id, int type) {
        int retry = 3;
        while (retry) {
                ina3221_data_read(id, type);
                if ((*id).error)
                        retry--;
                else
                        return;
        }
}

void get_tmp (tmp175_data *td, int fpga_state) {
        int retry = 3;
        while (retry) {
                tmp175_data_read(td, fpga_state);
                if ((*td).error)
                        retry--;
                else
                        return;
        }
}

void get_vm_all (trch_bstatus *tbs) {
        get_vm(&tbs->vm3v3a, 1);
        get_vm(&tbs->vm3v3b, 1);
        get_vm(&tbs->vm1v0, 1);
        get_vm(&tbs->vm1v8, 1);
        get_vm(&tbs->vm3v3, 1);
        get_vm(&tbs->vm3v3i, 1);
        get_vm(&tbs->vm3v3a, 0);
        get_vm(&tbs->vm3v3b, 0);
        get_vm(&tbs->vm1v0, 0);
        get_vm(&tbs->vm1v8, 0);
        get_vm(&tbs->vm3v3, 0);
        get_vm(&tbs->vm3v3i, 1);
}

void get_tmp_all (trch_state *tst, trch_bstatus *tbs) {
        get_tmp(&tbs->ts1, (*tst).fmd.state);
        get_tmp(&tbs->ts2, (*tst).fmd.state);
        get_tmp(&tbs->ts3, (*tst).fmd.state);
}

void cmd_parser (trch_state *tst) {
        char buf[BUF_LEN] = { };
        char data;
        tmp175_data temp;
        ina3221_data voltage;

        send_msg(rx_msg.msg);
        // FPGA Command
        if (!strcmp(rx_msg.msg,"fc")) {
                send_msg("fpga configuration");
                if ((*tst).fmd.state != ST_FPGA_READY)
                        send_msg(" Configuration Error");
                else
                        (*tst).fmd.config_ok = 1;
        } else if (!strcmp(rx_msg.msg,"fu")) {
                send_msg("fpga unconfiguration");
                if ((*tst).fmd.state != ST_FPGA_CONFIG &
                    (*tst).fmd.state != ST_FPGA_ACTIVE)
                        send_msg(" Unconfiguration Error");
                else
                        (*tst).fmd.config_ok = 0;

        // Configuration Memory Select
        } else if (!strncmp(rx_msg.msg,"ms",2)) {
                if ((*tst).fmd.state != ST_FPGA_ACTIVE) {
                        send_msg("Memory select control");
                        buf[0] = *(rx_msg.msg+2) - 0x30;
                        if (buf[0] == 0x00) {
                                TRCH_CFG_MEM_SEL = 0;
                                send_msg("  memory 0");
                        } else if (buf[0] == 0x01) {
                                TRCH_CFG_MEM_SEL = 1;
                                send_msg("  memory 1");
                        }
                } else
                        send_msg("fpga state error");

        // User IO
        } else if (!strncmp(rx_msg.msg,"uio0",4)) {
                send_msg("UIO3 00");
                buf[0] = *(rx_msg.msg+4) - 0x30;
                if (buf[0] == 0x00) {
                        UIO3_00 = 0;
                        send_msg("  level 0");
                } else if (buf[0] == 0x01) {
                        UIO3_00 = 1;
                        send_msg("  level 1");
                }
        } else if (!strncmp(rx_msg.msg,"uio1",4)) {
                send_msg("UIO3 01");
                buf[0] = *(rx_msg.msg+4) - 0x30;
                if (buf[0] == 0x00) {
                        UIO3_01 = 0;
                        send_msg("  level 0");
                } else if (buf[0] == 0x01) {
                        UIO3_01 = 1;
                        send_msg("  level 1");
                }
        } else if (!strncmp(rx_msg.msg,"uio2",4)) {
                send_msg("UIO3 01");
                buf[0] = *(rx_msg.msg+4) - 0x30;
                if (buf[0] == 0x00) {
                        UIO3_02 = 0;
                        send_msg("  level 0");
                } else if (buf[0] == 0x01) {
                        UIO3_02 = 1;
                        send_msg("  level 1");
                }

        // SPI Command
        } else if (!strcmp(rx_msg.msg,"spistart")) {
                send_msg("spi start (cs=0)");
                get_spi();

        } else if (!strcmp(rx_msg.msg,"spistop")) {
                send_msg("spi stop (cs=1)");
                release_spi();

        } else if (!strncmp(rx_msg.msg,"spi",3)) {
                data = conv_asc2hex(*(rx_msg.msg+3));
                data = (char)(data << 4) | conv_asc2hex(*(rx_msg.msg+4));
                buf[0] = spi_trans(data);
                conv_message(buf, 1);

        // Temperature sensor read Command
        } else if (!strncmp(rx_msg.msg,"ts",2)) {
                send_msg("Read Temperature sensor");
                buf[0] = *(rx_msg.msg+2) - 0x30;
                temp.master = 0;
                if (buf[0] == 0x00)
                        temp.addr = 0x4c;
                else if (buf[0] == 0x01)
                        temp.addr = 0x4d;
                else if (buf[0] == 0x02)
                        temp.addr = 0x4e;
                else
                        send_msg("Sensor number error");

                if (temp.addr != 0) {
                        if (tmp175_data_read(&temp, (*tst).fmd.state) | temp.error)
                                send_msg("i2c bus error");
                        conv_message(temp.data,2);
                }

        // Voltage sensor read Command
        } else if  (!strncmp(rx_msg.msg,"vs",2)) {
                int type = 0;
                send_msg("Read Voltage sensor");
                buf[0] = *(rx_msg.msg+2) - 0x30;
                buf[1] = *(rx_msg.msg+3) - 0x30;
                buf[2] = *(rx_msg.msg+4) - 0x30;
                voltage.master = 0;
                if (buf[0] == 0x00)
                        voltage.addr = 0x40;
                else if (buf[0] == 0x01)
                        voltage.addr = 0x41;
                else
                        send_msg("Sensor number error");
                voltage.channel = buf[1];
                if (buf[2] == 0x00 | buf[2] == 0x01) {
                        type = (int)buf[2];
                        if (ina3221_data_read(&voltage, type))
                                send_msg("i2c bus error");
                        if (type)
                                conv_message(voltage.bus,2);
                        else
                                conv_message(voltage.shunt,2);
                } else
                        send_msg("type error");

        // Register Check
        } else if (!strcmp(rx_msg.msg,"chkreg")) {
                send_msg("TRISA");
                buf[0] = TRISA;
                conv_message(buf, 1);
                send_msg("PORTA");
                buf[0] = PORTA;
                conv_message(buf, 1);

                send_msg("TRISB");
                buf[0] = TRISB;
                conv_message(buf, 1);
                send_msg("PORTB");
                buf[0] = PORTB;
                conv_message(buf, 1);

                send_msg("TRISC");
                buf[0] = TRISC;
                conv_message(buf, 1);
                send_msg("PORTC");
                buf[0] = PORTC;
                conv_message(buf, 1);

                send_msg("TRISD");
                buf[0] = TRISD;
                conv_message(buf, 1);
                send_msg("PORTD");
                buf[0] = PORTD;
                conv_message(buf, 1);

                send_msg("TRISE");
                buf[0] = TRISE;
                conv_message(buf, 1);
                send_msg("PORTE");
                buf[0] = PORTE;
                conv_message(buf, 1);

                send_msg("FPGA State");
                buf[0] = (*tst).fmd.state;
                conv_message(buf, 1);
                buf[0] = (char)((*tst).fmd.count);
                conv_message(buf, 1);

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

char conv_asc2hex (char data) {
        if (data >= 0x30 && data <= 39)
                return (data - 0x30);
        else if (data >= 0x41 && data <= 0x46)
                return (data - 0x37);
        else if (data >= 0x61 && data <= 0x66)
                return (data - 0x57);
        else
                return 0x00;
}

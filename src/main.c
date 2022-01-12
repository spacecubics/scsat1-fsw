/*
 * Space Cubics OBC TRCH Software
 *
 * (C) Copyright Space Cubics, LLC
 *
 */

#define AUTO_CONFIG 1

#include <xc.h>
#include <pic.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "trch.h"
#include "fpga.h"
#include "spi.h"
#include "i2c.h"
#include "tmp175.h"
#include "ina3221.h"
#include "usart.h"
#include "timer.h"
#include "interrupt.h"

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
extern void conv_message (uint8_t *data, int count);
extern char conv_asc2hex (char data);

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
extern void cmd_parser (struct trch_state *tst);

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
                        cmd_parser(&tst);

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

void cmd_parser (struct trch_state *tst) {
        uint8_t buf[BUF_LEN] = { };
        uint8_t data;
        struct tmp175_data temp;
        struct ina3221_data voltage;

        usart_send_msg(rx_msg.msg);
        // FPGA Command
        if (!strcmp(rx_msg.msg,"fc")) {
                usart_send_msg("fpga configuration");
                if (tst->fmd.state != FPGA_STATE_READY)
                        usart_send_msg(" Configuration Error");
                else
                        tst->fmd.config_ok = 1;
        } else if (!strcmp(rx_msg.msg,"fu")) {
                usart_send_msg("fpga unconfiguration");
                if (tst->fmd.state != FPGA_STATE_CONFIG &
                    tst->fmd.state != FPGA_STATE_ACTIVE)
                        usart_send_msg(" Unconfiguration Error");
                else
                        tst->fmd.config_ok = 0;

        // Configuration Memory Select
        } else if (!strncmp(rx_msg.msg,"ms",2)) {
                if (tst->fmd.state != FPGA_STATE_ACTIVE) {
                        usart_send_msg("Memory select control");
                        buf[0] = *(rx_msg.msg+2) - 0x30;
                        if (buf[0] == 0x00) {
                                TRCH_CFG_MEM_SEL = 0;
                                usart_send_msg("  memory 0");
                        } else if (buf[0] == 0x01) {
                                TRCH_CFG_MEM_SEL = 1;
                                usart_send_msg("  memory 1");
                        }
                } else
                        usart_send_msg("fpga state error");

        // User IO
        } else if (!strncmp(rx_msg.msg,"uio0",4)) {
                usart_send_msg("UIO3 00");
                buf[0] = *(rx_msg.msg+4) - 0x30;
                if (buf[0] == 0x00) {
                        UIO3_00 = 0;
                        usart_send_msg("  level 0");
                } else if (buf[0] == 0x01) {
                        UIO3_00 = 1;
                        usart_send_msg("  level 1");
                }
        } else if (!strncmp(rx_msg.msg,"uio1",4)) {
                usart_send_msg("UIO3 01");
                buf[0] = *(rx_msg.msg+4) - 0x30;
                if (buf[0] == 0x00) {
                        UIO3_01 = 0;
                        usart_send_msg("  level 0");
                } else if (buf[0] == 0x01) {
                        UIO3_01 = 1;
                        usart_send_msg("  level 1");
                }
        } else if (!strncmp(rx_msg.msg,"uio2",4)) {
                usart_send_msg("UIO3 01");
                buf[0] = *(rx_msg.msg+4) - 0x30;
                if (buf[0] == 0x00) {
                        UIO3_02 = 0;
                        usart_send_msg("  level 0");
                } else if (buf[0] == 0x01) {
                        UIO3_02 = 1;
                        usart_send_msg("  level 1");
                }

        // SPI Command
        } else if (!strcmp(rx_msg.msg,"spistart")) {
                usart_send_msg("spi start (cs=0)");
                spi_get();

        } else if (!strcmp(rx_msg.msg,"spistop")) {
                usart_send_msg("spi stop (cs=1)");
                spi_release();

        } else if (!strncmp(rx_msg.msg,"spi",3)) {
                data = conv_asc2hex(*(rx_msg.msg+3));
                data = (uint8_t)(data << 4) | conv_asc2hex(*(rx_msg.msg+4));
                buf[0] = spi_trans(data);
                conv_message(buf, 1);

        // Temperature sensor read Command
        } else if (!strncmp(rx_msg.msg,"ts",2)) {
                usart_send_msg("Read Temperature sensor");
                buf[0] = *(rx_msg.msg+2) - 0x30;
                temp.master = 0;
                if (buf[0] == 0x00)
                        temp.addr = 0x4c;
                else if (buf[0] == 0x01)
                        temp.addr = 0x4d;
                else if (buf[0] == 0x02)
                        temp.addr = 0x4e;
                else
                        usart_send_msg("Sensor number error");

                if (temp.addr != 0) {
                        if (tmp175_data_read(&temp, tst->fmd.state) | temp.error)
                                usart_send_msg("i2c bus error");
                        conv_message(temp.data,2);
                }

        // Voltage sensor read Command
        } else if  (!strncmp(rx_msg.msg,"vs",2)) {
                int type = 0;
                usart_send_msg("Read Voltage sensor");
                buf[0] = *(rx_msg.msg+2) - 0x30;
                buf[1] = *(rx_msg.msg+3) - 0x30;
                buf[2] = *(rx_msg.msg+4) - 0x30;
                voltage.master = 0;
                if (buf[0] == 0x00)
                        voltage.addr = 0x40;
                else if (buf[0] == 0x01)
                        voltage.addr = 0x41;
                else
                        usart_send_msg("Sensor number error");
                voltage.channel = buf[1];
                if (buf[2] == 0x00 | buf[2] == 0x01) {
                        type = (int)buf[2];
                        if (ina3221_data_read(&voltage, tst->fmd.state, type))
                                usart_send_msg("i2c bus error");
                        if (type)
                                conv_message(voltage.bus,2);
                        else
                                conv_message(voltage.shunt,2);
                } else
                        usart_send_msg("type error");

        // Port Control
        } else if (!strncmp(rx_msg.msg,"pt",2)) {
                char port;
                port = conv_asc2hex(*(rx_msg.msg+2));
                buf[0] = (char)(conv_asc2hex(*(rx_msg.msg+3)) << 4) | conv_asc2hex(*(rx_msg.msg+4));
                if (port == 0x0A) {
                        usart_send_msg("Set TRISA");
                        TRISA = buf[0];
                } else if (port == 0x0B) {
                        usart_send_msg("Set TRISB");
                        TRISB = buf[0];
                } else if (port == 0x0C) {
                        usart_send_msg("Set TRISC");
                        TRISC = buf[0];
                } else if (port == 0x0D) {
                        usart_send_msg("Set TRISD");
                        TRISD = buf[0];
                } else if (port == 0x0E) {
                        usart_send_msg("Set TRISE");
                        TRISD = buf[0];
                } else
                        usart_send_msg("TRIS Port Select Error");

        } else if (!strncmp(rx_msg.msg,"po",2)) {
                char port;
                port = conv_asc2hex(*(rx_msg.msg+2));
                buf[0] = (char)(conv_asc2hex(*(rx_msg.msg+3)) << 4) | conv_asc2hex(*(rx_msg.msg+4));
                if (port == 0x0A) {
                        usart_send_msg("Set PORTA");
                        PORTA = buf[0];
                } else if (port == 0x0B) {
                        usart_send_msg("Set PORTB");
                        PORTB = buf[0];
                } else if (port == 0x0C) {
                        usart_send_msg("Set PORTC");
                        PORTC = buf[0];
                } else if (port == 0x0D) {
                        usart_send_msg("Set PORTD");
                        PORTD = buf[0];
                } else if (port == 0x0E) {
                        usart_send_msg("Set PORTE");
                        PORTD = buf[0];
                } else
                        usart_send_msg("Port Select Error");

        // Register Check
        } else if (!strcmp(rx_msg.msg,"chkreg")) {
                usart_send_msg("TRISA");
                buf[0] = TRISA;
                conv_message(buf, 1);
                usart_send_msg("PORTA");
                buf[0] = PORTA;
                conv_message(buf, 1);

                usart_send_msg("TRISB");
                buf[0] = TRISB;
                conv_message(buf, 1);
                usart_send_msg("PORTB");
                buf[0] = PORTB;
                conv_message(buf, 1);

                usart_send_msg("TRISC");
                buf[0] = TRISC;
                conv_message(buf, 1);
                usart_send_msg("PORTC");
                buf[0] = PORTC;
                conv_message(buf, 1);

                usart_send_msg("TRISD");
                buf[0] = TRISD;
                conv_message(buf, 1);
                usart_send_msg("PORTD");
                buf[0] = PORTD;
                conv_message(buf, 1);

                usart_send_msg("TRISE");
                buf[0] = TRISE;
                conv_message(buf, 1);
                usart_send_msg("PORTE");
                buf[0] = PORTE;
                conv_message(buf, 1);

                usart_send_msg("FPGA State");
                buf[0] = tst->fmd.state;
                conv_message(buf, 1);
                buf[0] = (char)(tst->fmd.count);
                conv_message(buf, 1);

        } else
                usart_send_msg("cmd error");
        usart_receive_msg_clear();
}

void conv_message (uint8_t *data, int count) {
        int i, j=0;
        char text[] = "0123456789ABCDEF";
        char message[BUF_LEN] = { };
        for (i=0; i<count; i++) {
                message[j]   = text[(*data & 0xF0) >> 4];
                message[j+1] = text[*data & 0x0F];
                j=j+2;
                data++;
        }
        usart_send_msg(message);
}

char conv_asc2hex (char data) {
        if (data >= 0x30 && data <= 0x39)
                return (data - 0x30);
        else if (data >= 0x41 && data <= 0x46)
                return (data - 0x37);
        else if (data >= 0x61 && data <= 0x66)
                return (data - 0x57);
        else
                return 0x00;
}

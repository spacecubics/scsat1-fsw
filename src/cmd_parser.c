/*
 * Space Cubics OBC TRCH Software
 *  FPGA Control Utility
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#include "cmd_parser.h"

#include <stdint.h>
#include <string.h>
#include "trch.h"
#include "fpga.h"
#include "usart.h"
#include "tmp175.h"
#include "ina3221.h"
#include "spi.h"
#include "timer.h"

#define BUF_LEN 20

static void conv_message (uint8_t *data, int count) {
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

static char conv_asc2hex (char data) {
        if (data >= 0x30 && data <= 0x39)
                return (data - 0x30);
        else if (data >= 0x41 && data <= 0x46)
                return (data - 0x37);
        else if (data >= 0x61 && data <= 0x66)
                return (data - 0x57);
        else
                return 0x00;
}

#define bswap32(x) ((uint32_t) ( (((x) & 0x000000ff) << 24) |	\
				 (((x) & 0x0000ff00) <<  8) |	\
				 (((x) & 0x00ff0000) >>  8) |	\
				 (((x) & 0xff000000) >> 24)) )

void cmd_parser (struct fpga_management_data *fmd) {
        uint8_t buf[BUF_LEN] = { };
        uint8_t data;
        struct tmp175_data temp;
        struct ina3221_data voltage;

        usart_send_msg(rx_msg.msg);
        // FPGA Command
        if (!strcmp(rx_msg.msg,"fc")) {
                usart_send_msg("fpga configuration");
                if (fmd->state != FPGA_STATE_READY)
                        usart_send_msg(" Configuration Error");
                else
                        fmd->config_ok = 1;
        } else if (!strcmp(rx_msg.msg,"fu")) {
                usart_send_msg("fpga unconfiguration");
                if (fmd->state != FPGA_STATE_CONFIG &
                    fmd->state != FPGA_STATE_ACTIVE)
                        usart_send_msg(" Unconfiguration Error");
                else
                        fmd->config_ok = 0;

        // Timer
	} else if (!strncmp(rx_msg.msg, "gtimer", 6)) {
		uint32_t ticks = timer_get_ticks();
		ticks = bswap32(ticks);
		usart_send_msg("Global Timer");
		conv_message((uint8_t *)&ticks, sizeof(ticks));

        // Configuration Memory Select
        } else if (!strncmp(rx_msg.msg,"ms",2)) {
                if (fmd->state != FPGA_STATE_ACTIVE) {
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
                        if (tmp175_data_read(&temp, fmd->state) || temp.error)
                                usart_send_msg("i2c bus error");
                        conv_message(temp.data,2);
                }

        // Voltage sensor read Command
        } else if  (!strncmp(rx_msg.msg,"vs",2)) {
                enum Ina3221VoltageType type;
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
                if (buf[2] == 0x00 || buf[2] == 0x01) {
                        type = buf[2] == 0x0 ? INA3221_VOLTAGE_SHUNT : INA3221_VOLTAGE_BUS;
                        if (ina3221_data_read(&voltage, fmd->state, type))
                                usart_send_msg("i2c bus error");
                        if (type == INA3221_VOLTAGE_BUS)
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
                buf[0] = fmd->state;
                conv_message(buf, 1);
                buf[0] = (char)(fmd->count);
                conv_message(buf, 1);

        } else
                usart_send_msg("cmd error");
        usart_receive_msg_clear();
}

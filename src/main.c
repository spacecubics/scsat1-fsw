/*
 * Space Cubics OBC TRCH Software
 *
 * (C) Copyright 2021-2022 Space Cubics, LLC
 *
 */

#include <xc.h>
#include <pic.h>

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

// PIC16LF877A Configuration Bit Settings
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

struct temp_sensors {
        struct tmp175_data ts1;
        struct tmp175_data ts2;
        struct tmp175_data ts3;
};

struct voltage_sensors {
        struct ina3221_data vm1v0;
        struct ina3221_data vm1v8;
        struct ina3221_data vm3v3;
        struct ina3221_data vm3v3a;
        struct ina3221_data vm3v3b;
        struct ina3221_data vm3v3i;
};

static void get_voltage_monitor (struct ina3221_data *id, enum FpgaState fpga_state, enum Ina3221VoltageType type) {
        int retry = 3;
        int8_t ret;

        while (retry) {
                ret = ina3221_data_read(id, fpga_state, type);
                if (ret < 0 && id->error == INA3221_ERROR_I2C_NAK)
                        retry--;
                else
                        return;
        }
}

static void get_tmp (struct tmp175_data *td, enum FpgaState fpga_state) {
        int retry = 3;
        int8_t ret;

        while (retry) {
                ret = tmp175_data_read(td, fpga_state);
                if (ret < 0 && td->error == TMP175_ERROR_I2C_NAK)
                        retry--;
                else
                        return;
        }
}

static void get_voltage_monitor_all(enum FpgaState fpga_state, struct voltage_sensors *volts)
{
        get_voltage_monitor(&volts->vm3v3a, fpga_state, INA3221_VOLTAGE_BUS);
        get_voltage_monitor(&volts->vm3v3b, fpga_state, INA3221_VOLTAGE_BUS);
        get_voltage_monitor(&volts->vm1v0, fpga_state, INA3221_VOLTAGE_BUS);
        get_voltage_monitor(&volts->vm1v8, fpga_state, INA3221_VOLTAGE_BUS);
        get_voltage_monitor(&volts->vm3v3, fpga_state, INA3221_VOLTAGE_BUS);
        get_voltage_monitor(&volts->vm3v3i, fpga_state, INA3221_VOLTAGE_BUS);
        get_voltage_monitor(&volts->vm3v3a, fpga_state, INA3221_VOLTAGE_SHUNT);
        get_voltage_monitor(&volts->vm3v3b, fpga_state, INA3221_VOLTAGE_SHUNT);
        get_voltage_monitor(&volts->vm1v0, fpga_state, INA3221_VOLTAGE_SHUNT);
        get_voltage_monitor(&volts->vm1v8, fpga_state, INA3221_VOLTAGE_SHUNT);
        get_voltage_monitor(&volts->vm3v3, fpga_state, INA3221_VOLTAGE_SHUNT);
        get_voltage_monitor(&volts->vm3v3i, fpga_state, INA3221_VOLTAGE_SHUNT);
}

static void get_tmp_all(enum FpgaState fpga_state, struct temp_sensors *temps)
{
        get_tmp(&temps->ts1, fpga_state);
        get_tmp(&temps->ts2, fpga_state);
        get_tmp(&temps->ts3, fpga_state);
}

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

static void fpga_program_maybe(void)
{
        if (IS_ENABLED(CONFIG_FPGA_PROGRAM_MODE)) {
                if (!FPGAPROG_MODE_B) {
                        FPGAPWR_EN = 1;
                        while (1) {
                                if (FPGA_CFG_MEM_SEL) {
                                        TRCH_CFG_MEM_SEL = 1;
                                }
                        }
                }
        }
}

void main (void)
{
        struct voltage_sensors volts;
        struct temp_sensors temps;
        enum FpgaState fpga_state;
        enum FpgaGoal activate_fpga = FPGA_SHUTDOWN;
        int config_memory = 0;
        int boot_mode = FPGA_BOOT_48MHZ;

        // Initialize trch-firmware
        trch_init();
        fpga_program_maybe();
        fpga_state = fpga_init();

        spi_init();
        usart_init();
        timer2_init();
        timer2_ctrl(1);
        interrupt_enable();

        /*
         * Space Cubics OBC TRCH-Firmware Main
         */
        usart_send_msg("SC OBC TRCH-FW v0.8");

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
        temps.ts1.master     = 0;
        temps.ts1.addr       = 0x4C;
        temps.ts2.master     = 0;
        temps.ts2.addr       = 0x4D;
        temps.ts3.master     = 0;
        temps.ts3.addr       = 0x4E;

        volts.vm1v0.master   = 0;
        volts.vm1v0.addr     = 0x40;
        volts.vm1v0.channel  = 1;
        volts.vm1v8.master   = 0;
        volts.vm1v8.addr     = 0x40;
        volts.vm1v8.channel  = 2;
        volts.vm3v3.master   = 0;
        volts.vm3v3.addr     = 0x40;
        volts.vm3v3.channel  = 3;
        volts.vm3v3a.master  = 0;
        volts.vm3v3a.addr    = 0x41;
        volts.vm3v3a.channel = 1;
        volts.vm3v3b.master  = 0;
        volts.vm3v3b.addr    = 0x41;
        volts.vm3v3b.channel = 2;
        volts.vm3v3i.master  = 0;
        volts.vm3v3i.addr    = 0x41;
        volts.vm3v3i.channel = 3;

        get_voltage_monitor_all(fpga_state,  &volts);
        get_tmp_all(fpga_state, &temps);

        while (1) {
                if (FPGA_PWR_CYCLE_REQ) {
                        activate_fpga = FPGA_SHUTDOWN;
                }

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
                        break;

                case FPGA_STATE_ERROR:
                        activate_fpga = FPGA_SHUTDOWN;
                        config_memory = !config_memory;
                        break;

                default:
                        break;
                }
        }
        return;
}

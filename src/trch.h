/*
 * Definitions for TRCH Hardware
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#pragma once

#include <pic.h>

/*
 * Hardware Configuration
 */
#define _XTAL_FREQ 4000000

#define PORT_DIR_OUT (0)
#define PORT_DIR_IN (1)
#define PORT_DATA_LOW (0)
#define PORT_DATA_HIGH (1)

#define OUT(n) (0)
#define IN(n) BIT(n)
#define LOW(n) (0)
#define HIGH(n) BIT(n)

/*
 * PORTA
 */
#define TRCH_CFG_MEM_SEL_BIT (_PORTA_RA0_POSITION)
#define FPGA_BOOT0_BIT       (_PORTA_RA1_POSITION)
#define FPGA_BOO11_BIT       (_PORTA_RA2_POSITION)
#define FPGA_PROGRAM_B_BIT   (_PORTA_RA3_POSITION)
#define FPGA_INIT_B_BIT      (_PORTA_RA4_POSITION)
#define FPGAPWR_EN_BIT       (_PORTA_RA5_POSITION)

#define TRISA_INIT (                            \
        OUT(TRCH_CFG_MEM_SEL_BIT) |             \
        OUT(FPGA_BOOT0_BIT) |                   \
        OUT(FPGA_BOOT1_BIT) |                   \
        IN(FPGA_PROGRAM_B_BIT) |                \
        IN(FPGA_INIT_B_BIT) |                   \
        OUT(FPGAPWR_EN_BIT))

#define PORTA_INIT (                            \
        LOW(TRCH_CFG_MEM_SEL_BIT) |             \
        LOW(FPGA_BOOT0_BIT) |                   \
        LOW(FPGA_BOOT1_BIT) |                   \
        LOW(FPGA_PROGRAM_B_BIT) |               \
        LOW(FPGA_INIT_B_BIT) |                  \
        LOW(FPGAPWR_EN))

#define TRISA_FPGA_READY   0x00
#define TRCH_CFG_MEM_SEL   PORTAbits.RA0
#define FPGA_BOOT0         PORTAbits.RA1
#define FPGA_BOOT1         PORTAbits.RA2
#define FPGA_PROGRAM_B     PORTAbits.RA3
#define FPGA_PROGRAM_B_DIR TRISAbits.TRISA3
#define FPGA_INIT_B        PORTAbits.RA4
#define FPGA_INIT_B_DIR    TRISAbits.TRISA4
#define FPGAPWR_EN         PORTAbits.RA5
#define FPGAPWR_EN_DIR     TRISAbits.TRISA5

/*
 * PORTB
 */
#define SPICAN_INT_B_BIT       (_PORTB_RB0_POSITION)
#define FPGA_PWR_CYCLE_REQ_BIT (_PORTB_RB1_POSITION)
#define TRCH_CFG_MEM_MONI_BIT  (_PORTB_RB2_POSITION)
#define ICSP_PGM_BIT           (_PORTB_RB3_POSITION)
#define FPGA_WATCHDOG_BIT      (_PORTB_RB4_POSITION)
#define FPGA_CFG_MEM_SEL_BIT   (_PORTB_RB5_POSITION)
#define ICSP_PGC_BIT           (_PORTB_RB6_POSITION)
#define ICSP_PGD_BIT           (_PORTB_RB7_POSITION)

#define TRISB_INIT (                            \
        IN(SPICAN_INT_B_BIT) |                  \
        IN(FPGA_PWR_CYCLE_REQ_BIT) |            \
        IN(TRCH_CFG_MEM_MONI_BIT) |             \
        IN(ICSP_PGM_BIT) |                      \
        IN(FPGA_WATCHDOG_BIT) |                 \
        IN(FPGA_CFG_MEM_SEL_BIT) |              \
        OUT(ICSP_PGC_BIT) |                     \
        OUT(ICSP_PGD_BIT))

#define PORTB_INIT (                            \
        LOW(SPICAN_INT_B_BIT) |                 \
        LOW(FPGA_PWR_CYCLE_REQ_BIT) |           \
        LOW(TRCH_CFG_MEM_MONI_BIT) |            \
        LOW(ICSP_PGM_BIT) |                     \
        LOW(FPGA_WATCHDOG_BIT) |                \
        LOW(FPGA_CFG_MEM_SEL_BIT) |             \
        LOW(ICSP_PGC_BIT) |                     \
        LOW(ICSP_PGD_BIT))

#define FPGA_PWR_CYCLE_REQ PORTBbits.RB1
#define FPGA_WATCHDOG      PORTBbits.RB4
#define FPGA_CFG_MEM_SEL   PORTBbits.RB5

/*
 * PORTC
 */
#define TRCH_CAN_SLEEP_EN_BIT (_PORTC_RC0_POSITION)
#define SPICAN_CS_B_BIT       (_PORTC_RC1_POSITION)
#define FPGAPROG_MODE_B_BIT   (_PORTC_RC2_POSITION)
#define SPICAN_SCK_BIT        (_PORTC_RC3_POSITION)
#define SPICAN_MISO_BIT       (_PORTC_RC4_POSITION)
#define SPICAN_MOSI_BIT       (_PORTC_RC5_POSITION)
#define TRCH_UART_TX_BIT      (_PORTC_RC6_POSITION)
#define TRCH_UART_RX_BIT      (_PORTC_RC7_POSITION)

#define TRISC_INIT (                            \
        OUT(TRCH_CAN_SLEEP_EN_BIT) |            \
        OUT(SPICAN_CS_B_BIT) |                  \
        IN(FPGAPROG_MODE_B_BIT) |               \
        OUT(SPICAN_SCK_BIT) |                   \
        IN(SPICAN_MISO_BIT) |                   \
        OUT(SPICAN_MOSI_BIT) |                  \
        OUT(TRCH_UART_TX_BIT) |                 \
        IN(TRCH_UART_RX_BIT))

#define PORTC_INIT (                            \
        LOW(TRCH_CAN_SLEEP_EN_BIT) |            \
        HIGH(SPICAN_CS_B_BIT) |                 \
        LOW(FPGAPROG_MODE_B_BIT) |              \
        LOW(SPICAN_SCK_BIT) |                   \
        LOW(SPICAN_MISO_BIT) |                  \
        LOW(SPICAN_MOSI_BIT) |                  \
        LOW(TRCH_UART_TX_BIT) |                 \
        LOW(TRCH_UART_RX_BIT))

#define SPICAN_CS_B_DIR    TRISCbits.TRISC1
#define SPICAN_CS_B        PORTCbits.RC1
#define FPGAPROG_MODE_B    PORTCbits.RC2
#define SPICAN_SCK_DIR     TRISCbits.TRISC3
#define SPICAN_MISO_DIR    TRISCbits.TRISC4
#define SPICAN_MOSI_DIR    TRISCbits.TRISC5

/*
 * PORTD
 */
#define I2C_INT_SCL_BIT  (_PORTD_RD0_POSITION)
#define I2C_INT_SDA_BIT  (_PORTD_RD1_POSITION)
#define I2C_EXT_SCL_BIT  (_PORTD_RD2_POSITION)
#define I2C_EXT_SDA_BIT  (_PORTD_RD3_POSITION)
#define UIO3_00_BIT      (_PORTD_RD4_POSITION)
#define UIO3_01_BIT      (_PORTD_RD5_POSITION)
#define UIO3_02_BIT      (_PORTD_RD6_POSITION)
#define FPGA_RESERVE_BIT (_PORTD_RD7_POSITION)

#define TRISD_INIT ( \
        IN(I2C_INT_SCL_BIT) |                   \
        IN(I2C_INT_SDA_BIT) |                   \
        IN(I2C_EXT_SCL_BIT) |                   \
        IN(I2C_EXT_SDA_BIT) |                   \
        OUT(UIO3_00_BIT) |                      \
        OUT(UIO3_01_BIT) |                      \
        OUT(UIO3_02_BIT) |                      \
        OUT(FPGA_RESERVE_BIT))

#define PORTD_INIT (                            \
        LOW(I2C_INT_SCL_BIT) |                  \
        LOW(I2C_INT_SDA_BIT) |                  \
        LOW(I2C_EXT_SCL_BIT) |                  \
        LOW(I2C_EXT_SDA_BIT) |                  \
        LOW(UIO3_00_BIT) |                      \
        LOW(UIO3_01_BIT) |                      \
        LOW(UIO3_02_BIT) |                      \
        LOW(FPGA_RESERVE_BIT))

#define I2C_PORT           PORTD
#define I2C_INT_SCL_DIR    TRISDbits.TRISD0
#define I2C_INT_SCL        PORTDbits.RD0
#define I2C_INT_SDA_DIR    TRISDbits.TRISD1
#define I2C_INT_SDA        PORTDbits.RD1
#define I2C_INT_MASK       (0x03)
#define I2C_EXT_SCL_DIR    TRISDbits.TRISD2
#define I2C_EXT_SCL        PORTDbits.RD2
#define I2C_EXT_SDA_DIR    TRISDbits.TRISD3
#define I2C_EXT_SDA        PORTDbits.RD3
#define I2C_EXT_I2C_MASK   (0x0C)
#define UIO3_00            PORTDbits.RD4
#define UIO3_01            PORTDbits.RD5
#define UIO3_02            PORTDbits.RD6

/*
 * PORTE
 */

#define WDOG_OUT_BIT          (_PORTE_RE0_POSITION)
#define FPGA_PROGRAM_B_IN_BIT (_PORTE_RE1_POSITION)
#define FPGA_INIT_B_IN_BIT    (_PORTE_RE2_POSITION)

#define TRISE_INIT (                            \
        OUT(WDOG_OUT_BIT) |                     \
        IN(FPGA_PROGRAM_B_IN_BIT) |             \
        IN(FPGA_INIT_B_IN_BIT) )

#define PORTE_INIT (                            \
        LOW(I2C_INT_SCL_BIT) |                  \
        LOW(FPGA_PROGRAM_B_IN_BIT) |            \
        LOW(FPGA_INIT_B_IN_BIT) )

#define VDD_3V3            PORTEbits.RE1

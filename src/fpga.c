/*
 * Space Cubics OBC TRCH Software
 *  FPGA Control Utility
 *
 * (C) Copyright 2021-2022
 *         Space Cubics, LLC
 *
 */

#include "fpga.h"

#include <pic.h>
#include <stdbool.h>
#include "trch.h"
#include "timer.h"

struct fpga_management_data {
        enum FpgaState state;
        int mem_select;
        unsigned boot_mode: 2;
        int time;
#ifdef CONFIG_ENABLE_WDT_RESET
        bool wdt_value;
        uint32_t wdt_last_tick;
#endif
};

#define FPGA_WATCHDOG_TIMEOUT 3

static struct fpga_management_data the_fmd;

static void fpga_wdt_init(struct fpga_management_data *fmd) {
#ifdef CONFIG_ENABLE_WDT_RESET
        fmd->wdt_value = 0;
        fmd->wdt_last_tick = timer_get_ticks();
#endif
}

/*
 * Check the FPGA Watchdog kick
 *
 * Return true if detected. false otherwise.
 */
static bool fpga_wdt(struct fpga_management_data *fmd, bool wdt_value, uint32_t tick)
{
        bool ret = true;

#ifdef CONFIG_ENABLE_WDT_RESET
        if (fmd->wdt_value != wdt_value) {
                fmd->wdt_value = !fmd->wdt_value;
                fmd->wdt_last_tick = tick;
        }

        if (fmd->wdt_last_tick + FPGA_WATCHDOG_TIMEOUT < tick)
                ret = false;
#endif
        return ret;
}

enum FpgaState fpga_init()
{
        the_fmd.state = FPGA_STATE_POWER_OFF;
        the_fmd.mem_select = 0;
        the_fmd.boot_mode = FPGA_BOOT_48MHZ;
        the_fmd.time = 0;

        return the_fmd.state;
}

bool fpga_is_i2c_accessible (enum FpgaState state) {
        return state == FPGA_STATE_POWER_OFF || state == FPGA_STATE_READY;
}

/*
 * POWER_OFF state
 *
 * pre-condition
 *  - state: POWER_OFF
 *  - FPGAPWR_EN: LOW
 * post-condition
 *  - state: READY
 *    - activate_fpga: 1
 *    - FPGAPWR_EN: HIGH
 *  - state: POWER_OFF
 *    - activate_fpga: 0
 *    - FPGAPWR_EN: LOW
 *
 * When the user ask to actiavte the FPGA by setting activate_fpga to 1,
 * set FPGAPWR_EN HIGH to start the power sequence, and transition to
 * READY state.
 *
 * Keep the FPGA power off, otherwise.
 */
static enum FpgaState f_power_off(struct fpga_management_data *fmd, bool activate_fpga)
{
        /* check user request */
        if (activate_fpga) {
                /* Transition to READY */
                /* TRCH_CFG_MEM_SEL keep */
                /* FPGA_BOOT0 keep */
                /* FPGA_BOOT1 keep */
                /* FPGA_PROGRAM_B_DIR keep */
                FPGA_INIT_B = PORT_DATA_LOW;
                FPGA_INIT_B_DIR = PORT_DIR_OUT;
                FPGAPWR_EN = PORT_DATA_HIGH;
                FPGAPWR_EN_DIR = PORT_DIR_OUT;
                /* I2C_INT_SCL_DIR keep */
                /* I2C_INT_SDA_DIR keep */
                /* I2C_EXT_SCL_DIR keep */
                /* I2C_EXT_SDA_DIR keep */

                fmd->state = FPGA_STATE_READY;
        }

        return fmd->state;
}

/*
 * READY state
 *
 * pre-condition
 *  - state: READY
 *  - FPGAPWR_EN: HIGH
 * post-condtion
 *  - state: POWER_OFF
 *    - activate_fpga: 0
 *    - FPGAPWR_EN: LOW
 *  - state: CONFIG
 *    - activate_fpga: 1
 *    - FPGAPWR_EN: HIGH
 *    - VDD_3V3: 1
 *  - state: READY
 *    - activate_fpga: 1
 *    - FPGAPWR_EN: HIGH
 *    - VDD_3V3: HIGH
 *
 * Wait for FPGA power to be stable by monitoring VDD_3V3 become high.
 *
 * Go back to POWER_OFF when activate_fpga become 0.
 *
 * Stay in READY otherwise.
 */
static enum FpgaState f_fpga_ready(struct fpga_management_data *fmd, bool activate_fpga)
{
        /* check user request */
        if (!activate_fpga) {
                /* Transition to POWER_OFF */
                /* TRCH_CFG_MEM_SEL keep */
                /* FPGA_BOOT0 keep */
                /* FPGA_BOOT1 keep */
                /* FPGA_PROGRAM_B_DIR keep */
                /* FPGA_INIT_B don't care */
                FPGA_INIT_B_DIR = PORT_DIR_IN;
                FPGAPWR_EN = PORT_DATA_LOW;
                FPGAPWR_EN_DIR = PORT_DIR_OUT;
                /* I2C_INT_SCL_DIR keep */
                /* I2C_INT_SDA_DIR keep */
                /* I2C_EXT_SCL_DIR keep */
                /* I2C_EXT_SDA_DIR keep */

                fmd->state = FPGA_STATE_POWER_OFF;

                return fmd->state;
        }

        /* wait for VDD_3V3 */
        if (VDD_3V3) {
                /* Transition to CONFIG */
                TRCH_CFG_MEM_SEL = (char)fmd->mem_select;
                FPGA_BOOT0 = 0b01 & fmd->boot_mode;
                FPGA_BOOT1 = 0b01 & (fmd->boot_mode >> 1);
                /* FPGA_PROGRAM_B_DIR keep */
                FPGA_INIT_B = PORT_DATA_HIGH;
                FPGA_INIT_B_DIR = PORT_DIR_OUT;
                /* FPGAPWR_EN keep */
                /* FPGAPWR_EN_DIR keep */
                I2C_INT_SCL_DIR = PORT_DIR_IN;
                I2C_INT_SDA_DIR = PORT_DIR_IN;
                I2C_EXT_SCL_DIR = PORT_DIR_IN;
                I2C_EXT_SDA_DIR = PORT_DIR_IN;

                fpga_wdt_init(fmd);

                fmd->state = FPGA_STATE_CONFIG;
        }

        return fmd->state;
}

/*
 * CONFIG state
 *
 * pre-condition
 *  - state: CONFIG
 *  - FPGAPWR_EN: HIGH
 *  - VDD_3V3: HIGH
 * post-condtion
 *  - state: POWER_OFF
 *    - activate_fpga: 0
 *    - FPGAPWR_EN: LOW
 *  - state: CONFIG
 *    - activate_fpga: 1
 *    - FPGAPWR_EN: HIGH
 *    - VDD_3V3: HIGH
 *    - FPGA_INIT_B: NOT LOW
 *
 * Wait for the completion of the FPGA configuration sequence.  The
 * FPGA must drive FPGA_WATCHDOG HIGH once the configuration is done.
 *
 * Go back to POWER_OFF when activate_fpga become 0.
 *
 * Stay in CONFIG state, otherwise.
 *
 * Note that fpga_wdt() counts ticks from the last wdt kick and sets
 * activate_fpga to 0 if the count exceeds FPGA_WATCHDOG_TIMEOUT.
 */
static enum FpgaState f_fpga_config(struct fpga_management_data *fmd, bool activate_fpga)
{
        bool kicked;

        kicked = fpga_wdt(fmd, FPGA_WATCHDOG, timer_get_ticks());
        if (!kicked) {
                activate_fpga = false;
        }

        /* check user request */
        if (!activate_fpga) {
                /* Transition to POWER_OFF */
                TRCH_CFG_MEM_SEL = PORT_DATA_LOW;
                FPGA_BOOT0 = PORT_DATA_LOW;
                FPGA_BOOT1 = PORT_DATA_LOW;
                FPGA_PROGRAM_B_DIR = PORT_DIR_IN;
                /* FPGA_INIT_B don't care */
                FPGA_INIT_B_DIR = PORT_DIR_IN;
                FPGAPWR_EN = PORT_DATA_LOW;
                FPGAPWR_EN_DIR = PORT_DIR_OUT;
                /* I2C_INT_SCL_DIR keep */
                /* I2C_INT_SDA_DIR keep */
                /* I2C_EXT_SCL_DIR keep */
                /* I2C_EXT_SDA_DIR keep */

                fmd->mem_select = !fmd->mem_select;
                fmd->state = FPGA_STATE_POWER_OFF;

                return fmd->state;
        }

        /* wait for watchdog pulse from the fpga */
        if (FPGA_WATCHDOG) {
                /* Transition to ACTIVE */
                /* TRCH_CFG_MEM_SEL keep */
                /* FPGA_BOOT0 keep */
                /* FPGA_BOOT1 keep */
                /* FPGA_PROGRAM_B_DIR keep */
                /* FPGA_INIT_B don't care */
                FPGA_INIT_B_DIR = PORT_DIR_IN;
                /* FPGAPWR_EN keep */
                /* FPGAPWR_EN_DIR keep */
                /* I2C_INT_SCL_DIR keep */
                /* I2C_INT_SDA_DIR keep */
                /* I2C_EXT_SCL_DIR keep */
                /* I2C_EXT_SDA_DIR keep */

                fmd->state = FPGA_STATE_ACTIVE;
        }

        return fmd->state;
}

/*
 * ACTIVE state
 *
 * pre-condition
 *  - state: ACTIVE
 *  - FPGAPWR_EN: HIGH
 *  - VDD_3V3: 1
 *  - FPGA_INIT_B: NOT LOW
 * post-condtion
 *  - state: POWER_OFF
 *    - activate_fpga: 0
 *    - FPGAPWR_EN: LOW
 *  - state: ACTIVE
 *    - activate_fpga: 1
 *    - FPGAPWR_EN: HIGH
 *    - VDD_3V3: HIGH
 *    - FPGA_INIT_B: NOT LOW
 *
 * Monitor watchdog kicks from the FPGA.  Shutdown the FPGA if no kick
 * is observed in FPGA_WATCHDOG_TIMEOUT ticks.
 *
 * Stay in ACTIVE state, otherwise.
 *
 * Note that fpga_wdt() counts ticks from the last wdt kick and sets
 * activate_fpga to 0 if the count exceeds FPGA_WATCHDOG_TIMEOUT.
 */
static enum FpgaState f_fpga_active(struct fpga_management_data *fmd, bool activate_fpga)
{
        bool kicked;

        kicked = fpga_wdt(fmd, FPGA_WATCHDOG, timer_get_ticks());
        if (!kicked) {
                activate_fpga = false;
        }

        /* check user request */
        if (!activate_fpga) {
                /* Transition to POWER_OFF */
                TRCH_CFG_MEM_SEL = PORT_DATA_LOW;
                FPGA_BOOT0 = PORT_DATA_LOW;
                FPGA_BOOT1 = PORT_DATA_LOW;
                FPGA_PROGRAM_B_DIR = PORT_DIR_IN;
                /* FPGA_INIT_B don't care */
                FPGA_INIT_B_DIR = PORT_DIR_IN;
                FPGAPWR_EN = PORT_DATA_LOW;
                FPGAPWR_EN_DIR = PORT_DIR_OUT;
                /* I2C_INT_SCL_DIR keep */
                /* I2C_INT_SDA_DIR keep */
                /* I2C_EXT_SCL_DIR keep */
                /* I2C_EXT_SDA_DIR keep */

                fmd->mem_select = !fmd->mem_select;
                fmd->state = FPGA_STATE_POWER_OFF;

                return fmd->state;
        }

        TRCH_CFG_MEM_SEL = FPGA_CFG_MEM_SEL;

        return fmd->state;
}

typedef enum FpgaState (*STATEFUNC)(struct fpga_management_data *fmd, bool activate_fpga);

static STATEFUNC fpgafunc[] = {
        f_power_off,
        f_fpga_ready,
        f_fpga_config,
        f_fpga_active };

enum FpgaState fpga_state_control(bool activate_fpga)
{
        return fpgafunc[the_fmd.state](&the_fmd, activate_fpga);
}

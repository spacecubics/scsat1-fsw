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
#include "utils.h"

struct fpga_management_data {
        enum FpgaState state;
        uint8_t mem_select;
        uint8_t boot_mode;
#ifdef CONFIG_ENABLE_WDT_RESET
        bool wdt_value;
        uint32_t wdt_last_tick;
#endif
};

static struct fpga_management_data the_fmd;

static void fpga_wdt_init(struct fpga_management_data *fmd)
{
        if (IS_ENABLED(CONFIG_ENABLE_WDT_RESET)) {
                fmd->wdt_value = 0;
                fmd->wdt_last_tick = timer_get_ticks();
        }
}

/*
 * Check the FPGA Watchdog kick
 *
 * Return true if detected. false otherwise.
 */
static bool fpga_wdt(struct fpga_management_data *fmd, bool wdt_value, uint32_t tick)
{
        bool ret = true;

        if (IS_ENABLED(CONFIG_ENABLE_WDT_RESET)) {
                if (fmd->wdt_value != wdt_value) {
                        fmd->wdt_value = !fmd->wdt_value;
                        fmd->wdt_last_tick = tick;
                }

                if (fmd->wdt_last_tick + MSEC_TO_TICKS(CONFIG_FPGA_WATCHDOG_TIMEOUT * 1000) < tick)
                        ret = false;
        }
        return ret;
}

enum FpgaState fpga_init()
{
        the_fmd.state = FPGA_STATE_POWER_DOWN;
        the_fmd.mem_select = 0;
        the_fmd.boot_mode = FPGA_BOOT_48MHZ;

        return the_fmd.state;
}

bool fpga_is_i2c_accessible (enum FpgaState state) {
        return state == FPGA_STATE_POWER_OFF || state == FPGA_STATE_READY;
}

/*
 * Transition to the POWER_DOWN state
 *
 * In order to shut the FPGA down, we must set a few signal as
 * appropriate.  Do not drive any pins HIGH, which are connected to the
 * FPGA, directly.  We can drive LOW or make it HiZ.
 *
 * TRCH_CFG_MEM_SEL: It is connected from TRCH to the FPGA.  Must be LOW.
 *
 * FPGA_BOOT0 and FPGA_BOOT1: These are also directly connected to the
 * FPGA. Make sure they are LOW.
 *
 * FPGA_PROGRAM_B: We don't use it but just in case users set it, make
 * it HiZ by setting the dir IN so that it pulled-up externally.
 *
 * FPGA_INIT_B: Actually we don't care because the FPGA is off. But we
 * make sure that the pin is HiZ.
 *
 * FPGAPWR_EN: Drive LOW to shut the power down.
 */
static enum FpgaState trans_to_power_down(void)
{
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

        return FPGA_STATE_POWER_DOWN;
}

static enum FpgaState trans_to_power_off(void)
{
        return FPGA_STATE_POWER_OFF;
}

/*
 * Transition to the POWER_UP state
 *
 * Provide power to the FPGA.  We are not sure the power to the FPGA
 * is stable yet.  Do not drive any pins HIGH, which are connected to
 * the FPGA, directly.  We can drive pins LOW or make them HiZ.  You
 * can drive the pins in the VDD_3V3_SYS domain HIGH.
 *
 * FPGA_INIT_B: Drive LOW to make the FPGA wait after power up.
 *
 * FPGAPWR_EN: Drive HIGH to power up the FPGA.
 */
static enum FpgaState trans_to_power_up(void)
{
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

        return FPGA_STATE_POWER_UP;
}

static enum FpgaState trans_to_ready(void)
{
        return FPGA_STATE_READY;
}

/*
 * Transition to the CONFIG state
 *
 * Start the configuration of the FPGA.  The power to the FPGA is
 * stable and VDD_3V3 is HIGH.  The FPGA is already powered up but
 * waiting on INIT_B.  So we can drive the pins to the FPGA.
 *
 * TRCH_CFG_MEM_SEL: Select the configuration memory.
 *
 * FPGA_BOOT0 and FPGA_BOOT1: Tell the FPGA what clock speed.
 *
 * FPGA_INIT_B: Make it pull-up HIGH by seting the port HiZ to
 * initiate the FPGA configuraiton.
 *
 * I2C_*: Make sure the pins are HiZ while FPGA is active.  We don't
 * want to disturb both the internal and the external I2C lines.  Let
 * the FPGA control them exclusively.
 */
static enum FpgaState trans_to_config(uint8_t mem_select, uint8_t boot_mode)
{
        TRCH_CFG_MEM_SEL = mem_select & 0x01;
        FPGA_BOOT0 = 0b01 & boot_mode;
        FPGA_BOOT1 = 0b01 & (boot_mode >> 1);
        /* FPGA_PROGRAM_B_DIR keep */
        /* FPGA_INIT_B don't care */
        FPGA_INIT_B_DIR = PORT_DIR_IN;
        /* FPGAPWR_EN keep */
        /* FPGAPWR_EN_DIR keep */
        I2C_INT_SCL_DIR = PORT_DIR_IN;
        I2C_INT_SDA_DIR = PORT_DIR_IN;
        I2C_EXT_SCL_DIR = PORT_DIR_IN;
        I2C_EXT_SDA_DIR = PORT_DIR_IN;

        return FPGA_STATE_CONFIG;
}

/*
 * Transition to the ACTIVE state
 *
 * In the ACTIVE state, FPGA is fully functioning.
 *
 */
static enum FpgaState trans_to_active(void)
{
        /* TRCH_CFG_MEM_SEL keep */
        /* FPGA_BOOT0 keep */
        /* FPGA_BOOT1 keep */
        /* FPGA_PROGRAM_B_DIR keep */
        /* FPGA_INIT_B don't care */
        /* FPGA_INIT_B_DIR keep */
        /* FPGAPWR_EN keep */
        /* FPGAPWR_EN_DIR keep */
        /* I2C_INT_SCL_DIR keep */
        /* I2C_INT_SDA_DIR keep */
        /* I2C_EXT_SCL_DIR keep */
        /* I2C_EXT_SDA_DIR keep */

        return FPGA_STATE_ACTIVE;
}

/*
 * POWER_OFF state
 *
 * When the user ask to actiavte the FPGA by setting activate_fpga to
 * 1, transition to READY state.
 *
 * Stay in POWER_OFF, otherwise.
 */
static enum FpgaState f_power_off(struct fpga_management_data *fmd, bool activate_fpga)
{
        /* check user request */
        if (activate_fpga) {
                fmd->state = trans_to_power_up();
        }

        return fmd->state;
}

#define WAIT_STABLE_VDD_3V3_LOW (MSEC_TO_TICKS(150u))
#define WAIT_STABLE_VDD_3V3_HIGH (MSEC_TO_TICKS(3u))

/*
 * POWER_DOWN
 *
 * Wait for FPGA power to be LOW and stable.  This is a transient
 * state waiting for VDD_3V3.  User is not allowed to change any pin
 * during this state.  The state will be POWER_OFF once VDD_3V3 is
 * stable.
 */
static enum FpgaState f_fpga_power_down(struct fpga_management_data *fmd, bool activate_fpga)
{
        static bool vdd3v3_change_detected = false;
        static uint16_t detected_tick;
        uint16_t current_tick;

        current_tick = (uint16_t)timer_get_ticks();

        if (!VDD_3V3) {
                if (!vdd3v3_change_detected) {
                        vdd3v3_change_detected = true;
                        detected_tick = current_tick;
                }
                else {
                        if ((current_tick - detected_tick) >= WAIT_STABLE_VDD_3V3_LOW) {
                                vdd3v3_change_detected = false;
                                fmd->state = trans_to_power_off();
                        }
                }
        }
        return fmd->state;
}

/*
 * POWER_UP
 *
 * Wait for FPGA power to be HIGH and stable.  This is a transient
 * state waiting for VDD_3V3.  User is not allowed to change any pin
 * during this state.  The state will be READY once VDD_3V3 is stable.
 */
static enum FpgaState f_fpga_power_up(struct fpga_management_data *fmd, bool activate_fpga)
{
        static bool vdd3v3_change_detected = false;
        static uint16_t detected_tick;
        uint16_t current_tick;

        current_tick = (uint16_t)timer_get_ticks();

        if (VDD_3V3) {
                if (!vdd3v3_change_detected) {
                        vdd3v3_change_detected = true;
                        detected_tick = current_tick;
                }
                else {
                        if ((current_tick - detected_tick) >= WAIT_STABLE_VDD_3V3_HIGH) {
                                vdd3v3_change_detected = false;
                                fmd->state = trans_to_ready();
                        }
                }

        }
        return fmd->state;
}

/*
 * READY state
 *
 * FPGA power VDD_3V3 is HIGH and stable.
 *
 * Go back to POWER_OFF when activate_fpga become 0.
 *
 * Move on to CONFIG otherwise.
 */
static enum FpgaState f_fpga_ready(struct fpga_management_data *fmd, bool activate_fpga)
{
        /* check user request */
        if (!activate_fpga) {
                fmd->state = trans_to_power_down();

                return fmd->state;
        }

        fmd->state = trans_to_config(fmd->mem_select, fmd->boot_mode);
        fpga_wdt_init(fmd);

        return fmd->state;
}

/*
 * CONFIG state
 *
 * Wait for the completion of the FPGA configuration sequence.  The
 * FPGA must drive FPGA_WATCHDOG HIGH once the configuration is done.
 *
 * Go back to POWER_OFF when activate_fpga become 0.
 *
 * Stay in CONFIG state, otherwise.
 *
 * Note that fpga_wdt() counts ticks from the last wdt kick and sets
 * activate_fpga to 0 if the count exceeds CONFIG_FPGA_WATCHDOG_TIMEOUT.
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
                fmd->mem_select = !fmd->mem_select;
                fmd->state = trans_to_power_down();

                return fmd->state;
        }

        /* wait for watchdog pulse from the fpga */
        if (FPGA_WATCHDOG) {
                fmd->state = trans_to_active();

                return fmd->state;
        }

        return fmd->state;
}

/*
 * ACTIVE state
 *
 * Monitor watchdog kicks from the FPGA.  Shutdown the FPGA if no kick
 * is observed in CONFIG_FPGA_WATCHDOG_TIMEOUT ticks.
 *
 * Stay in ACTIVE state, otherwise.
 *
 * Note that fpga_wdt() counts ticks from the last wdt kick and sets
 * activate_fpga to 0 if the count exceeds CONFIG_FPGA_WATCHDOG_TIMEOUT.
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
                fmd->mem_select = !fmd->mem_select;
                fmd->state = trans_to_power_down();

                return fmd->state;
        }

        TRCH_CFG_MEM_SEL = FPGA_CFG_MEM_SEL;

        return fmd->state;
}

typedef enum FpgaState (*STATEFUNC)(struct fpga_management_data *fmd, bool activate_fpga);

static STATEFUNC fpgafunc[] = {
        f_fpga_power_down,
        f_power_off,
        f_fpga_power_up,
        f_fpga_ready,
        f_fpga_config,
        f_fpga_active };

enum FpgaState fpga_state_control(bool activate_fpga)
{
        return fpgafunc[the_fmd.state](&the_fmd, activate_fpga);
}

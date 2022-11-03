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

#define FPGA_WATCHDOG_TIMEOUT 3

static void fpga_wdt_init(struct fpga_management_data *fmd) {
#ifdef CONFIG_ENABLE_WDT_RESET
        fmd->wdt_value = 0;
        fmd->wdt_last_tick = timer_get_ticks();
#endif
}

static void fpga_wdt(struct fpga_management_data *fmd, bool wdt_value, uint32_t tick) {
#ifdef CONFIG_ENABLE_WDT_RESET
        if (fmd->wdt_value != wdt_value) {
                fmd->wdt_value = !fmd->wdt_value;
                fmd->wdt_last_tick = tick;
        }

        if (fmd->wdt_last_tick + FPGA_WATCHDOG_TIMEOUT < tick)
                fmd->config_ok = 0;
#endif
}

void fpga_init (struct fpga_management_data *fmd) {
        fmd->state = FPGA_STATE_POWER_OFF;
        fmd->config_ok = 0;
        fmd->mem_select = 0;
        fmd->boot_mode = FPGA_BOOT_48MHZ;
        fmd->time = 0;
}

bool fpga_is_i2c_accessible (enum FpgaState state) {
        return state == FPGA_STATE_POWER_OFF || state == FPGA_STATE_READY;
}

static void f_power_off (struct fpga_management_data *fmd)
{
        /* check user request */
        if (fmd->config_ok) {
                FPGA_PWR_EN = 1;
                fmd->state = FPGA_STATE_READY;
        }
}

static void f_fpga_ready (struct fpga_management_data *fmd)
{
        /* check user request */
        if (!fmd->config_ok) {
                FPGA_PWR_EN = 0;
                fmd->state = FPGA_STATE_POWER_OFF;
                return;
        }

        /* wait for VDD_3V3 */
        if (VDD_3V3) {
                TRCH_CFG_MEM_SEL = (char)fmd->mem_select;
                FPGA_BOOT0 = 0b01 & fmd->boot_mode;
                FPGA_BOOT1 = 0b01 & (fmd->boot_mode >> 1);
                fpga_wdt_init(fmd);
                fmd->state = FPGA_STATE_CONFIG;
        }
}

static void f_fpga_config (struct fpga_management_data *fmd)
{
        fpga_wdt(fmd, FPGA_WATCHDOG, timer_get_ticks());

        /* check user request */
        if (!fmd->config_ok) {
                FPGA_PWR_EN = 0;
                fmd->mem_select = !fmd->mem_select;
                fmd->state = FPGA_STATE_POWER_OFF;
                return;
        }

        /* wait for watchdog pulse from the fpga */
        if (FPGA_WATCHDOG) {
                fmd->state = FPGA_STATE_ACTIVE;
        }
}

static void f_fpga_active (struct fpga_management_data *fmd)
{
        fpga_wdt(fmd, FPGA_WATCHDOG, timer_get_ticks());

        /* check user request */
        if (!fmd->config_ok) {
                FPGA_PWR_EN = 0;
                fmd->mem_select = !fmd->mem_select;
                fmd->state = FPGA_STATE_POWER_OFF;
                return;
        }

        TRCH_CFG_MEM_SEL = FPGA_CFG_MEM_SEL;
}

typedef void (*STATEFUNC)(struct fpga_management_data *fmd);

static STATEFUNC fpgafunc[] = {
        f_power_off,
        f_fpga_ready,
        f_fpga_config,
        f_fpga_active };

void fpga_state_control(struct fpga_management_data *fmd)
{
       fpgafunc[fmd->state](fmd);
}

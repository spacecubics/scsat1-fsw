/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "sc_fpgamon.h"
#include "temp.h"
#include "cv.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(monitor, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

K_THREAD_STACK_DEFINE(monitor_workq_stack, CONFIG_SCSAT1_MAIN_MONITOR_THREAD_STACK_SIZE);
struct k_work_q monitor_workq;

void start_monitor(void)
{
	k_work_queue_start(&monitor_workq, monitor_workq_stack,
			   K_THREAD_STACK_SIZEOF(monitor_workq_stack),
			   CONFIG_SCSAT1_MAIN_MONITOR_THREAD_PRIORITY, NULL);
	k_thread_name_set(&monitor_workq.thread, "monitor_workq");

	sc_bhm_enable();
	start_temp_monitor();
	start_cv_monitor();
}

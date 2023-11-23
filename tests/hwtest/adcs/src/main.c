/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include "wdog.h"
#include "sysmon.h"
#include "temp_test.h"
#include "cv_test.h"
#include "imu_test.h"
#include "gnss_test.h"
#include "rw_test.h"
#include "adcs_init.h"
#include "loop_test.h"

#define CMD_HANDLER_PRIO (0U)
#define CMD_EXEC_EVENT   (1U)

K_THREAD_STACK_DEFINE(cmd_thread_stack, 2048);
static struct k_thread cmd_thread;
static struct k_event exec_event;

LOG_MODULE_REGISTER(adcs_main);

static void cmd_handler(void * p1, void * p2, void * p3)
{
	int ret = 0;
	uint32_t err_cnt = 0;
	char *cmd = (char*)p1;
	char *arg = (char*)p2;

	k_event_set(&exec_event, CMD_EXEC_EVENT);

	ARG_UNUSED(p3);

	if (strcmp(cmd, "info") == 0) {
		sc_adcs_print_fpga_ids();
	} else if (strcmp(cmd, "init") == 0) {
		ret = adcs_init(&err_cnt);
	} else if (strcmp(cmd, "temp") == 0) {
		ret = temp_test(&err_cnt);
	} else if (strcmp(cmd, "cv") == 0) {
		ret = cv_test(&err_cnt);
	} else if (strcmp(cmd, "imu") == 0) {
		ret = imu_test(&err_cnt);
	} else if (strcmp(cmd, "gnss") == 0) {
		ret = gnss_test(&err_cnt);
	} else if (strcmp(cmd, "rw") == 0) {
		ret = rw_test(&err_cnt);
	} else if (strcmp(cmd, "loop") == 0) {
		ret = loop_test(atoi(arg), &err_cnt);
	} else {
		goto end;
	}

	if (ret < 0) {
		LOG_ERR("%s test Failed. eror count: %d", cmd, err_cnt);
	} else {
		LOG_INF("%s test successed.", cmd);
	}
end:
	k_event_clear(&exec_event, CMD_EXEC_EVENT);

	return;
}

static int start_cmd_thread(const struct shell *sh, size_t argc, char **argv)
{
	int ret = 0;
	k_tid_t cmd_tid;

	if (argc < 1) {
		shell_error(sh, "Invalid argument");
		ret = -EINVAL;
		goto end;
	}

	if (k_event_wait(&exec_event, CMD_EXEC_EVENT, false, K_NO_WAIT) != 0) {
		shell_error(sh, "Now another command is executing");
		ret = -EBUSY;
		goto end;
	}

	cmd_tid = k_thread_create(&cmd_thread, cmd_thread_stack,
					 K_THREAD_STACK_SIZEOF(cmd_thread_stack),
					 (k_thread_entry_t)cmd_handler,
					 argv[0], argv[1], NULL,
					 CMD_HANDLER_PRIO, 0, K_NO_WAIT);
	if (!cmd_tid) {
		shell_error(sh, "Failed to create command thread");
		ret = ENOSPC;
	}

end:
	return ret;
}

int main(void)
{
	printk("This is for HW test program for %s\n", CONFIG_BOARD);

	start_kick_wdt_thread();

	sc_adcs_print_fpga_ids();

	k_event_init(&exec_event);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_hwtest,
	SHELL_CMD(info, NULL, "ADCS Board Information", start_cmd_thread),
	SHELL_CMD(init, NULL, "ADCS Board Initialization", start_cmd_thread),
	SHELL_CMD(temp, NULL, "Temperature test command", start_cmd_thread),
	SHELL_CMD(cv, NULL, "Current/Voltage test command", start_cmd_thread),
	SHELL_CMD(imu, NULL, "IMU test command", start_cmd_thread),
	SHELL_CMD(gnss, NULL, "GNSS test command", start_cmd_thread),
	SHELL_CMD(rw, NULL, "Reaction Wheel test command", start_cmd_thread),
	SHELL_CMD(loop, NULL, "Loop test command", start_cmd_thread),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(hwtest, &sub_hwtest, "SC-Sat1 HW test commands", NULL);

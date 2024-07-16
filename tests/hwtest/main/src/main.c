/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell_uart.h>
#include "common.h"
#include "csp.h"
#include "wdog.h"
#include "sysmon.h"
#include "temp_test.h"
#include "cv_test.h"
#include "mgnm_test.h"
#include "csp_test.h"
#include "sunsens_test.h"
#include "mtq_test.h"
#include "dstrx3_test.h"
#include "scbus.h"
#include "main_init.h"
#include "loop_test.h"
#include "syshk_test.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#define CMD_HANDLER_PRIO (0U)
#define CMD_EXEC_EVENT   (1U)

char last_cmd[32];

K_THREAD_STACK_DEFINE(cmd_thread_stack, 2048);
static struct k_thread cmd_thread;
static struct k_event exec_event;
extern struct k_event loop_event;

static void cmd_handler(void *p1, void *p2, void *p3)
{
	int ret = 0;
	uint32_t err_cnt = 0;
	char *cmd = (char *)p1;
	char *arg = (char *)p2;
	struct main_temp_test_result temp_ret;
	struct main_cv_test_result cv_ret;
	struct csp_test_result csp_ret;
	struct sunsens_test_ret sunsens_ret;
	struct mgnm_test_ret mgnm_ret;
	struct dstrx3_test_ret dstrx3_ret;

	k_event_set(&exec_event, CMD_EXEC_EVENT);

	ARG_UNUSED(p3);

	if (strcmp(cmd, "info") == 0) {
		sc_main_print_fpga_ids();
	} else if (strcmp(cmd, "init") == 0) {
		ret = main_init((enum hwtest_mode)atoi(arg), &err_cnt);
	} else if (strcmp(cmd, "off") == 0) {
		ret = main_off(&err_cnt);
	} else if (strcmp(cmd, "temp") == 0) {
		ret = temp_test(&temp_ret, &err_cnt, LOG_ENABLE);
	} else if (strcmp(cmd, "cv") == 0) {
		ret = cv_test(&cv_ret, &err_cnt, LOG_ENABLE);
	} else if (strcmp(cmd, "mgnm") == 0) {
		ret = mgnm_test(&mgnm_ret, &err_cnt, LOG_ENABLE);
	} else if (strcmp(cmd, "csp") == 0) {
		ret = csp_test(&csp_ret, &err_cnt, LOG_ENABLE);
	} else if (strcmp(cmd, "sun") == 0) {
		ret = sunsens_test(&sunsens_ret, &err_cnt, LOG_ENABLE);
	} else if (strcmp(cmd, "mtq") == 0) {
		ret = mtq_test(&err_cnt);
	} else if (strcmp(cmd, "dstrx3") == 0) {
		ret = dstrx3_test(&dstrx3_ret, &err_cnt, LOG_ENABLE);
	} else if (strcmp(cmd, "loop") == 0) {
		ret = loop_test(atoi(arg), &err_cnt);
		k_event_clear(&loop_event, LOOP_STOP_EVENT);
	} else if (strcmp(cmd, "syshk") == 0) {
		ret = syshk_test(atoi(arg), &err_cnt);
		k_event_clear(&loop_event, LOOP_STOP_EVENT);
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

	strncpy(last_cmd, sh->ctx->temp_buff, MIN(sh->ctx->cmd_tmp_buff_len, sizeof(last_cmd) - 1));

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
				  (k_thread_entry_t)cmd_handler, argv[0], argv[1], NULL,
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
	int ret;

	printk("This is for HW test program for %s\n", CONFIG_BOARD);

	start_kick_wdt_thread();

	sc_main_print_fpga_ids();

	k_event_init(&exec_event);

	ret = csp_enable();
	if (ret < 0) {
		LOG_ERR("Failed to enable the CSP. (%d)", ret);
	} else {
		LOG_INF("Enable the CSP");
	}

	if (IS_ENABLED(CONFIG_SCSAT1_MAIN_AUTO_RUN_HWTEST)) {
		shell_execute_cmd(shell_backend_uart_get_ptr(), "hwtest init 3");

		k_sleep(K_SECONDS(60));

		shell_execute_cmd(shell_backend_uart_get_ptr(), "hwtest syshk -1");
	}

	return 0;
}

static int stop_cmd(const struct shell *sh, size_t argc, char **argv)
{
	strncpy(last_cmd, sh->ctx->temp_buff, MIN(sh->ctx->cmd_tmp_buff_len, sizeof(last_cmd) - 1));
	k_event_set(&loop_event, LOOP_STOP_EVENT);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_hwtest, SHELL_CMD(info, NULL, "MAIN Board Information", start_cmd_thread),
	SHELL_CMD(init, NULL, "MAIN Board Initialization", start_cmd_thread),
	SHELL_CMD(off, NULL, "Turn off the each power and CAN", start_cmd_thread),
	SHELL_CMD(temp, NULL, "Temperature test command", start_cmd_thread),
	SHELL_CMD(cv, NULL, "Current/Voltage test command", start_cmd_thread),
	SHELL_CMD(mgnm, NULL, "Magnet Meteor test command", start_cmd_thread),
	SHELL_CMD(mtq, NULL, "Magnetorquer test command", start_cmd_thread),
	SHELL_CMD(csp, NULL, "CSP test command", start_cmd_thread),
	SHELL_CMD(sun, NULL, "Sun Sensor test command", start_cmd_thread),
	SHELL_CMD(dstrx3, NULL, "DSTRX-3 test command", start_cmd_thread),
	SHELL_CMD(loop, NULL, "Loop test command", start_cmd_thread),
	SHELL_CMD(syshk, NULL, "System HK test command", start_cmd_thread), SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(hwtest, &sub_hwtest, "SC-Sat1 HW test commands", NULL);
SHELL_CMD_REGISTER(stop, NULL, "SC-Sat1 HW test stop", stop_cmd);

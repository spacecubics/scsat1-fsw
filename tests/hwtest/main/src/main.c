/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#define CMD_HANDLER_PRIO (0U)
#define CMD_EXEC_EVENT   (1U)

K_THREAD_STACK_DEFINE(cmd_thread_stack, 2048);
static struct k_thread cmd_thread;
static struct k_event exec_event;

static void cmd_handler(void * p1, void * p2, void * p3)
{
	char *cmd = (char*)p1;

	k_event_set(&exec_event, CMD_EXEC_EVENT);

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	printk("Sub Cmd is %s\n", (char*)cmd);

	k_event_clear(&exec_event, CMD_EXEC_EVENT);
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
					 argv[0], NULL, NULL,
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

	k_event_init(&exec_event);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_hwtest,
	SHELL_CMD(demo, NULL, "Demo command", start_cmd_thread),
	SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(hwtest, &sub_hwtest, "SC-Sat1 HW test commands", NULL);
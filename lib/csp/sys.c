/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <csp/csp.h>
#include "sc_csp.h"
#include "reply.h"
#include "fram.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sys_handler, CONFIG_SC_LIB_CSP_LOG_LEVEL);

/* Command size */
#define SYSTEM_CMD_MIN_SIZE (1U)

/* Command ID */
#define SYSTEM_CLEAR_BOOT_COUNT_CMD (0U)

static int csp_system_clear_boot_count_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret;

	LOG_INF("Clear boot count command");

	ret = sc_fram_clear_boot_count();

	csp_send_std_reply(packet, command_id, ret);

	return ret;
}

int csp_system_handler(csp_packet_t *packet)
{
	int ret = 0;
	uint8_t command_id;

	if (packet == NULL) {
		ret = -EINVAL;
		command_id = CSP_UNKNOWN_CMD_CODE;
		goto end;
	}

	if (packet->length < SYSTEM_CMD_MIN_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		command_id = CSP_UNKNOWN_CMD_CODE;
		goto free;
	}

	command_id = packet->data[CSP_COMMAND_ID_OFFSET];

	switch (command_id) {
	case SYSTEM_CLEAR_BOOT_COUNT_CMD:
		csp_system_clear_boot_count_cmd(command_id, packet);
		break;
	default:
		LOG_ERR("Unkown command code: %d", command_id);
		ret = -EINVAL;
		command_id = CSP_UNKNOWN_CMD_CODE;
		break;
	}

free:
	if (ret < 0) {
		csp_send_std_reply(packet, command_id, ret);
		csp_buffer_free(packet);
	}

end:
	return ret;
}

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <csp/csp.h>
#include "sc_csp.h"
#include "reply.h"
#include "syshk.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tlm, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

/* Command size */
#define TLM_CMD_MIN_SIZE   (1U)

/* Command ID */
#define TLM_SYSHK_CMD (0U)

#define UNKOWN_COMMAND_ID (0xFF)

int csp_tlm_handler(csp_packet_t *packet)
{
	int ret = 0;
	uint8_t command_id;

	if (packet == NULL) {
		ret = -EINVAL;
		goto end;
	}

	if (packet->length < TLM_CMD_MIN_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		goto free;
	}

	command_id = packet->data[CSP_COMMAND_ID_OFFSET];

	switch (command_id) {
	case TLM_SYSHK_CMD:
		send_syshk_to_ground();
		csp_buffer_free(packet);
		break;
	default:
		LOG_ERR("Unkown command code: %d", command_id);
		ret = -EINVAL;
		break;
	}

free:
	if (ret < 0) {
		csp_send_std_reply(packet, UNKOWN_COMMAND_ID, ret);
		csp_buffer_free(packet);
	}

end:
	return ret;
}

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <csp/csp.h>
#include "sc_csp.h"
#include "pwrctrl_main.h"
#include "reply.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwrctrl, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

/* Command size */
#define PWRCTRL_CMD_MIN_SIZE   (1U)
#define PWRCTRL_ONOFF_CMD_SIZE (3U)

/* Command ID */
#define PWRCTRL_ONOFF_CMD (0U)

/* Command argument offset */
#define PWRCTRL_TARGET_OFFSET (1U)
#define PWRCTRL_ONOFF_OFFSET  (2U)

static int csp_power_control_onoff_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret = 0;
	uint8_t target;
	uint8_t onoff;

	if (packet->length != PWRCTRL_ONOFF_CMD_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		goto end;
	}

	target = packet->data[PWRCTRL_TARGET_OFFSET];
	onoff = packet->data[PWRCTRL_ONOFF_OFFSET];

	if (onoff) {
		sc_main_power_enable(target);
	} else {
		sc_main_power_disable(target);
	}

	LOG_INF("Power control command (target: %d) (onoff: %d)", target, onoff);

end:
	csp_send_std_reply(packet, command_id, ret);
	return ret;
}

int csp_pwrctrl_handler(csp_packet_t *packet)
{
	int ret;
	uint8_t command_id;

	if (packet == NULL) {
		ret = -EINVAL;
		goto end;
	}

	if (packet->length < PWRCTRL_CMD_MIN_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		goto free;
	}

	command_id = packet->data[CSP_COMMAND_ID_OFFSET];

	switch (command_id) {
	case PWRCTRL_ONOFF_CMD:
		csp_power_control_onoff_cmd(command_id, packet);
		break;
	default:
		LOG_ERR("Unkown command code: %d", command_id);
		ret = -EINVAL;
		break;
	}

free:
	csp_buffer_free(packet);

end:
	return ret;
}

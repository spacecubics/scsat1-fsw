/*
 * Copyright (c) 2025 Space Cubics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <csp/csp.h>
#include "sc_csp.h"
#include "reply.h"
#include "mtq.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mtqctrl, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

/* Command size */
#define MTQCTRL_START_CMD_SIZE (3U)
#define MTQCTRL_STOP_CMD_SIZE  (1U)

/* Command ID */
#define MTQCTRL_START_CMD (0U)
#define MTQCTRL_STOP_CMD  (1U)

/* Command argument offset */
#define MTQCTRL_AXES_OFFSET (1U)
#define MTQCTRL_POL_OFFSET  (2U)
#define MTQCTRL_DUTY_OFFSET (3U)

static int csp_mtq_control_start_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret = 0;
	uint8_t axes;
	uint8_t pol;
	uint8_t duty; /* 1 - 100 */
	float duty_ratio;

	if (packet->length < MTQCTRL_START_CMD_SIZE) {
		LOG_ERR("Invalid command size: %d", packet->length);
		ret = -EMSGSIZE;
		goto end;
	}

	axes = packet->data[MTQCTRL_AXES_OFFSET];
	if (axes > MTQ_AXES_BKUP_Z) {
		LOG_ERR("Invalid axes: %d", axes);
		ret = -EINVAL;
		goto end;
	}

	pol = packet->data[MTQCTRL_POL_OFFSET];
	if (pol > MTQ_POL_NON) {
		LOG_ERR("Invalid polarity: %d", pol);
		ret = -EINVAL;
		goto end;
	}

	duty = packet->data[MTQCTRL_DUTY_OFFSET];
	if (duty > 100) {
		LOG_ERR("Invalid duty: %d", duty);
		ret = -EINVAL;
		goto end;
	}
	duty_ratio = (float)duty / 100.0f;

	ret = mtq_start(axes, pol, duty_ratio);

	LOG_INF("MTQ control START command (axes: %d, pol: %d, duty: %f)", axes, pol,
		(double)duty_ratio);

end:
	return ret;
}

static int csp_mtq_control_stop_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret = 0;
	uint8_t axes;

	if (packet->length < MTQCTRL_STOP_CMD_SIZE) {
		LOG_ERR("Invalid command size: %d", packet->length);
		ret = -EMSGSIZE;
		goto end;
	}

	axes = packet->data[MTQCTRL_AXES_OFFSET];
	if (axes > MTQ_AXES_BKUP_Z) {
		LOG_ERR("Invalid axes: %d", axes);
		ret = -EINVAL;
		goto end;
	}

	ret = mtq_stop(axes);

	LOG_INF("MTQ control STOP command (axes: %d)", axes);

end:
	return ret;
}

int csp_mtqctrl_handler(csp_packet_t *packet)
{
	int ret = 0;
	uint8_t command_id;

	if (packet == NULL) {
		ret = -EINVAL;
		goto end;
	}

	command_id = packet->data[CSP_COMMAND_ID_OFFSET];
	switch (command_id) {
	case MTQCTRL_START_CMD:
		ret = csp_mtq_control_start_cmd(command_id, packet);
		break;
	case MTQCTRL_STOP_CMD:
		ret = csp_mtq_control_stop_cmd(command_id, packet);
		break;
	default:
		LOG_ERR("Unkown command code: %d", command_id);
		ret = -EINVAL;
		break;
	}

	csp_send_std_reply(packet, command_id, ret);

end:
	return ret;
}

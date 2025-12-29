/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <csp/csp.h>
#include "sc_csp.h"
#include "reply.h"
#include "syshk.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tlm, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

/* Command size */
#define TLM_CMD_MIN_SIZE     (1U)
#define TLM_HISTORY_CMD_SIZE (13U)

/* Command ID */
#define TLM_SYSHK_CMD          (0U)
#define TLM_HISTORY_CMD        (1U)
#define TLM_CANCEL_HISTORY_CMD (2U)

#define UNKOWN_COMMAND_ID (0xFF)

/* Command argument offset */
#define TLM_HISTORY_START_SEQ_OFFSET (1U)
#define TLM_HISTORY_END_SEQ_OFFSET   (5U)
#define TLM_HISTORY_SKIP_OFFSET      (9U)
#define TLM_HISTORY_INTVL_OFFSET     (11U)

/* thread shared parameter */
static struct k_work work;
static volatile uint32_t start_seq_num;
static volatile uint32_t end_seq_num;
static volatile uint16_t skip_seq_count;
static volatile uint16_t send_intvl_ms;

static void csp_tlm_history_work(struct k_work *item)
{
	send_syshk_history_to_ground(start_seq_num, end_seq_num, skip_seq_count, send_intvl_ms);
}

static int csp_tlm_history_handler(const csp_packet_t *packet)
{
	int ret = 0;
	static bool is_init = false;

	if (!is_init) {
		k_work_init(&work, csp_tlm_history_work);
		is_init = true;
	}

	if (k_work_is_pending(&work)) {
		LOG_ERR("history teleme command is currently executing, so rejected.");
		ret = -EBUSY;
		goto end;
	}

	if (packet->length != TLM_HISTORY_CMD_SIZE) {
		LOG_ERR("teleme history, invalid command size: %d, exepcted: %d", packet->length,
			TLM_HISTORY_CMD_SIZE);
		ret = -EINVAL;
		goto end;
	}

	start_seq_num = sys_le32_to_cpu(*(uint32_t *)&packet->data[TLM_HISTORY_START_SEQ_OFFSET]);
	end_seq_num = sys_le32_to_cpu(*(uint32_t *)&packet->data[TLM_HISTORY_END_SEQ_OFFSET]);
	skip_seq_count = sys_le16_to_cpu(*(uint16_t *)&packet->data[TLM_HISTORY_SKIP_OFFSET]);
	send_intvl_ms = sys_le16_to_cpu(*(uint16_t *)&packet->data[TLM_HISTORY_INTVL_OFFSET]);

	ret = check_syshk_history_req_param(start_seq_num, end_seq_num, send_intvl_ms);
	if (ret != 0) {
		goto end;
	}

	LOG_INF("syshk history request received, start_seq_num: %u, end_seq_num: %u, skip: %u, "
		"intvl: %u",
		start_seq_num, end_seq_num, skip_seq_count, send_intvl_ms);

	k_work_submit(&work);

end:
	return ret;
}

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
	case TLM_HISTORY_CMD:
		ret = csp_tlm_history_handler(packet);
		csp_send_std_reply(packet, TLM_HISTORY_CMD, ret);
		break;
	case TLM_CANCEL_HISTORY_CMD:
		cancel_syshk_history();
		csp_send_std_reply(packet, TLM_CANCEL_HISTORY_CMD, 0);
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

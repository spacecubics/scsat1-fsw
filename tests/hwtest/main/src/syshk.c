/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <csp/csp.h>
#include <csp/drivers/can_zephyr.h>
#include <zephyr/device.h>
#include "csp.h"
#include "csp_test.h"
#include "syshk.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(syshk, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

static uint32_t seq_counters[TLM_TYPE_NUM] = {0};

int send_syshk(enum main_obc_tlm_type type, void *data, uint16_t size)
{
	int ret = 0;
	size_t type_size = sizeof(type);
	size_t seq_size = sizeof(seq_counters[type]);
	size_t pkt_size = size + type_size + seq_size;

	LOG_DBG("Send System HK %d byte (type:%d)", size, type);

	csp_conn_t *conn = csp_connect(CSP_PRIO_NORM, CSP_ID_GND, SYSHK_PORT,
				       CSP_SYSHK_TIMEOUT_MSEC, CSP_O_NONE);
	if (conn == NULL) {
		LOG_ERR("CSP Connection failed");
		ret = -ETIMEDOUT;
		goto end;
	}

	csp_packet_t *packet = csp_buffer_get(pkt_size);
	if (packet == NULL) {
		LOG_ERR("Failed to get CSP buffer");
		ret = -ENOBUFS;
		goto end;
	}
	memcpy(packet->data, &type, type_size);
	memcpy(packet->data + type_size, &seq_counters[type], seq_size);
	memcpy(packet->data + type_size + seq_size, data, size);
	packet->length = pkt_size;

	seq_counters[type]++;

	csp_send(conn, packet);

	csp_close(conn);

end:
	return ret;
}

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/posix/time.h>
#include <zephyr/sys/byteorder.h>
#include <csp/csp.h>
#include "sc_csp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eps, CONFIG_SC_LIB_CSP_LOG_LEVEL);

#define CSP_PORT_EPS_GET_TIME (8U)
#define CSP_EPS_GET_TIME_SIZE (6U)
#define CSP_EPS_TIME_OFFSET   (2U)

int csp_time_sync_from_eps(void)
{
	int ret = 0;
	csp_conn_t *conn;
	csp_packet_t *packet;
	uint8_t command_id = 0;
	struct timespec ts;

	conn = csp_connect(CSP_PRIO_NORM, CSP_ID_EPS, CSP_PORT_EPS_GET_TIME,
			   CONFIG_SC_LIB_CSP_CONN_TIMEOUT_MSEC, CSP_O_NONE);
	if (conn == NULL) {
		LOG_ERR("Failed to connect to EPS");
		ret = -ETIME;
		goto end;
	}

	packet = csp_buffer_get(0);
	if (packet == NULL) {
		LOG_ERR("Failed to get CSP buffer");
		ret = -ENOMEM;
		goto close;
	}

	memcpy(&packet->data, &command_id, sizeof(command_id));
	packet->length = sizeof(command_id);

	csp_send(conn, packet);
	LOG_DBG("Send GET TIME packet to EPS");

	packet = csp_read(conn, CONFIG_SC_LIB_CSP_READ_TIMEOUT_MSEC);
	if (packet == NULL) {
		LOG_ERR("Failed to read EPS GET TIME reply");
		ret = -ETIME;
		goto close;
	}

	if (packet->length != CSP_EPS_GET_TIME_SIZE) {
		LOG_ERR("Invalid data size of EPS GET TIME reply (%d)", packet->length);
		ret = -EINVAL;
		goto close;
	}

	ts.tv_sec = sys_le32_to_cpu(*(uint32_t *)&packet->data[CSP_EPS_TIME_OFFSET]);
	ts.tv_nsec = 0;

	ret = clock_settime(CLOCK_REALTIME, &ts);
	if (ret < 0) {
		LOG_ERR("Failed to set time (%d)", ret);
		goto close;
	}

	LOG_INF("Time sync from EPS (%lld)", ts.tv_sec);

close:
	csp_close(conn);

end:
	return ret;
}

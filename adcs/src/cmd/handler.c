/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <csp/csp.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(handler, CONFIG_SCSAT1_ADCS_LOG_LEVEL);

void csp_cmd_handler(void)
{
	LOG_INF("CSP command handler task started");

	csp_socket_t sock = {0};

	csp_bind(&sock, CSP_ANY);

	csp_listen(&sock, 10);

	while (true) {

		csp_conn_t *conn;
		if ((conn = csp_accept(&sock, 10000)) == NULL) {
			continue;
		}

		csp_packet_t *packet;
		while ((packet = csp_read(conn, 50)) != NULL) {
			switch (csp_conn_dport(conn)) {
			default:
				csp_service_handler(packet);
				break;
			}
		}

		csp_close(conn);
	}

	return;
}

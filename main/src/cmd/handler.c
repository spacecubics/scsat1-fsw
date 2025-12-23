/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <csp/csp.h>
#include "sc_csp.h"
#include "handler.h"
#include "pwrctrl.h"
#include "mtqctrl.h"
#include "file.h"
#include "flash.h"
#include "tlm.h"
#include "sys.h"

#define CSP_PORT_MAIN_MTQCTRL (11U)
#define CSP_PORT_MAIN_PWRCTRL (12U)
#define CSP_PORT_MAIN_FILE    (13U)
#define CSP_PORT_MAIN_FLASH   (14U)
#define CSP_PORT_MAIN_TLM     (15U)
#define CSP_PORT_MAIN_SYSTEM  (16U)

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(handler, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

struct csp_stat csp_stat = {0};

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
			csp_system_update_stat(packet, &csp_stat);
			switch (csp_conn_dport(conn)) {
			case CSP_PORT_MAIN_PWRCTRL:
				csp_pwrctrl_handler(packet);
				break;
			case CSP_PORT_MAIN_FILE:
				csp_file_handler(packet);
				break;
			case CSP_PORT_MAIN_FLASH:
				csp_flash_handler(packet);
				break;
			case CSP_PORT_MAIN_TLM:
				csp_tlm_handler(packet);
				break;
			case CSP_PORT_MAIN_SYSTEM:
				csp_system_handler(packet);
				break;
			case CSP_PORT_MAIN_MTQCTRL:
				csp_mtqctrl_handler(packet);
				break;
			default:
				csp_service_handler(packet);
				break;
			}
		}

		csp_close(conn);
	}

	return;
}

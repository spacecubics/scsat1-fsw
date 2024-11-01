/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/sys/byteorder.h>
#include <zephyr/zbus/zbus.h>
#include <csp/drivers/can_zephyr.h>
#include <csp/csp.h>
#include "sc_csp.h"
#include "syshk.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(syshk, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

K_THREAD_STACK_DEFINE(syshk_workq_stack, CONFIG_SCSAT1_MAIN_SEND_SYSHK_THREAD_STACK_SIZE);
struct k_work_q syshk_workq;

struct syshk_tlm {
	uint8_t telemetry_id;
	uint32_t seq_num;
} __attribute__((__packed__));

static struct syshk_tlm syshk = {.telemetry_id = CSP_TLM_ID_SYSHK, .seq_num = 0};

static void send_syshk(struct k_work *work)
{
	csp_conn_t *conn;
	csp_packet_t *packet;

	conn = csp_connect(CSP_PRIO_NORM, CSP_ID_GND, CSP_PORT_TLM,
			   CONFIG_SCSAT1_MAIN_CSP_CONN_TIMEOUT_MSEC, CSP_O_NONE);
	if (conn == NULL) {
		LOG_ERR("Failed to connect to GND");
		goto end;
	}

	packet = csp_buffer_get(0);
	if (packet == NULL) {
		LOG_ERR("Failed to get CSP buffer");
		goto close;
	}

	memcpy(&packet->data, &syshk, sizeof(syshk));
	packet->length = sizeof(syshk);

	csp_send(conn, packet);
	LOG_DBG("Send HK to GND %d byte", packet->length);
	syshk.seq_num++;

close:
	csp_close(conn);

end:
}

static K_WORK_DEFINE(syshk_work, send_syshk);

static void syshk_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&syshk_workq, &syshk_work);
}

static K_TIMER_DEFINE(syshk_timer, syshk_handler, NULL);

void start_send_syshk(void)
{
	k_work_queue_start(&syshk_workq, syshk_workq_stack,
			   K_THREAD_STACK_SIZEOF(syshk_workq_stack),
			   CONFIG_SCSAT1_MAIN_SEND_SYSHK_THREAD_PRIORITY, NULL);
	k_thread_name_set(&syshk_workq.thread, "syshk_workq");

	k_timer_start(&syshk_timer, K_SECONDS(CONFIG_SCSAT1_MAIN_SYSHK_INHIBIT_PERIOD_SEC),
		      K_SECONDS(CONFIG_SCSAT1_MAIN_SYSHK_INTERVAL_SEC));
}

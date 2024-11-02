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
#include "temp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(syshk, CONFIG_SCSAT1_ADCS_LOG_LEVEL);

K_THREAD_STACK_DEFINE(syshk_workq_stack, CONFIG_SCSAT1_ADCS_SEND_SYSHK_THREAD_STACK_SIZE);
struct k_work_q syshk_workq;

ZBUS_CHAN_DEFINE(temp_chan, struct temp_msg, NULL, NULL, ZBUS_OBSERVERS(syshk_sub),
		 ZBUS_MSG_INIT(0));
ZBUS_SUBSCRIBER_DEFINE(syshk_sub, CONFIG_SCSAT1_ADCS_SYSHK_SUB_QUEUE_SIZE);

#define SYSHK_TEMP_BLOCK_SIZE (35U)

struct syshk_tlm {
	uint8_t telemetry_id;
	uint32_t seq_num;
	uint8_t temp_block[SYSHK_TEMP_BLOCK_SIZE];
} __attribute__((__packed__));

static struct syshk_tlm syshk = {.telemetry_id = CSP_TLM_ID_SYSHK, .seq_num = 0};

static void copy_temp_to_syshk(struct temp_msg *msg)
{
	int pos;
	uint8_t *temp_block = syshk.temp_block;

	for (pos = 0; pos < OBC_TEMP_POS_NUM; pos++) {
		memcpy(temp_block, &msg->obc[pos].status, sizeof(msg->obc[pos].status));
		temp_block += sizeof(msg->obc[pos].status);
		memcpy(temp_block, &msg->obc[pos].temp, sizeof(msg->obc[pos].temp));
		temp_block += sizeof(msg->obc[pos].temp);
	}

	memcpy(temp_block, &msg->xadc.status, sizeof(msg->xadc.status));
	temp_block += sizeof(msg->xadc.status);
	memcpy(temp_block, &msg->xadc.temp, sizeof(msg->xadc.temp));
	temp_block += sizeof(msg->xadc.temp);

	for (pos = 0; pos < ADCS_TEMP_POS_NUM; pos++) {
		memcpy(temp_block, &msg->adcs[pos].status, sizeof(msg->adcs[pos].status));
		temp_block += sizeof(msg->adcs[pos].status);
		memcpy(temp_block, &msg->adcs[pos].temp, sizeof(msg->adcs[pos].temp));
		temp_block += sizeof(msg->adcs[pos].temp);
	}
}

static void syshk_sub_task(void *sub)
{
	const struct zbus_channel *chan;
	const struct zbus_observer *subscriber = sub;
	struct temp_msg temp_msg;

	LOG_INF("Start the system HK Subscribing thread");

	while (!zbus_sub_wait(subscriber, &chan, K_FOREVER)) {
		if (&temp_chan == chan) {
			zbus_chan_read(chan, &temp_msg,
				       K_MSEC(CONFIG_SCSAT1_ADCS_ZBUS_READ_TIMEOUT_MSEC));
			LOG_DBG("Subscribe TEMP msg %d byte", sizeof(temp_msg));
			copy_temp_to_syshk(&temp_msg);
		} else {
			LOG_ERR("Wrong channel %p!", chan);
			continue;
		}
	}
}

K_THREAD_DEFINE(syshk_sub_task_id, CONFIG_SCSAT1_ADCS_SUB_SYSHK_THREAD_STACK_SIZE, syshk_sub_task,
		&syshk_sub, NULL, NULL, CONFIG_SCSAT1_ADCS_SUB_SYSHK_THREAD_PRIORITY, 0, 0);

static void send_syshk(struct k_work *work)
{
	csp_conn_t *conn;
	csp_packet_t *packet;

	if (!IS_ENABLED(CONFIG_SCSAT1_ADCS_AUTO_SYSHK_DOWNLINK)) {
		goto end;
	}

	conn = csp_connect(CSP_PRIO_NORM, CSP_ID_GND, CSP_PORT_TLM,
			   CONFIG_SCSAT1_ADCS_CSP_CONN_TIMEOUT_MSEC, CSP_O_NONE);
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
			   CONFIG_SCSAT1_ADCS_SEND_SYSHK_THREAD_PRIORITY, NULL);
	k_thread_name_set(&syshk_workq.thread, "syshk_workq");

	k_timer_start(&syshk_timer, K_SECONDS(CONFIG_SCSAT1_ADCS_SYSHK_INHIBIT_PERIOD_SEC),
		      K_SECONDS(CONFIG_SCSAT1_ADCS_SYSHK_INTERVAL_SEC));
}

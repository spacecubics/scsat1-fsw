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
#include "system.h"
#include "temp.h"
#include "cv.h"
#include "mgnm_mon.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(syshk, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

K_THREAD_STACK_DEFINE(syshk_workq_stack, CONFIG_SCSAT1_MAIN_SEND_SYSHK_THREAD_STACK_SIZE);
struct k_work_q syshk_workq;

ZBUS_CHAN_DEFINE(system_chan, struct system_msg, NULL, NULL, ZBUS_OBSERVERS(syshk_sub),
		 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(temp_chan, struct temp_msg, NULL, NULL, ZBUS_OBSERVERS(syshk_sub),
		 ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(cv_chan, struct cv_msg, NULL, NULL, ZBUS_OBSERVERS(syshk_sub), ZBUS_MSG_INIT(0));
ZBUS_CHAN_DEFINE(mgnm_chan, struct mgnm_msg, NULL, NULL, ZBUS_OBSERVERS(syshk_sub),
		 ZBUS_MSG_INIT(0));
ZBUS_SUBSCRIBER_DEFINE(syshk_sub, CONFIG_SCSAT1_MAIN_SYSHK_SUB_QUEUE_SIZE);

#define SYSHK_SYSTEM_BLOCK_SIZE (33U)
#define SYSHK_TEMP_BLOCK_SIZE   (50U)
#define SYSHK_CV_BLOCK_SIZE     (105U)
#define SYSHK_MGNM_BLOCK_SIZE   (24U)

struct syshk_tlm {
	uint8_t telemetry_id;
	uint32_t seq_num;
	uint8_t system_block[SYSHK_SYSTEM_BLOCK_SIZE];
	uint8_t temp_block[SYSHK_TEMP_BLOCK_SIZE];
	uint8_t cv_block[SYSHK_CV_BLOCK_SIZE];
	uint8_t mgnm_block[SYSHK_MGNM_BLOCK_SIZE];
} __attribute__((__packed__));

static struct syshk_tlm syshk = {.telemetry_id = CSP_TLM_ID_SYSHK, .seq_num = 0};

static void copy_system_to_syshk(struct system_msg *msg)
{
	uint8_t *system_block = syshk.system_block;

	memcpy(system_block, &msg->wall_clock, sizeof(msg->wall_clock));
	system_block += sizeof(msg->wall_clock);
	memcpy(system_block, &msg->sysup_time, sizeof(msg->sysup_time));
	system_block += sizeof(msg->sysup_time);
	memcpy(system_block, &msg->sw_version, sizeof(msg->sw_version));
	system_block += sizeof(msg->sw_version);
	memcpy(system_block, &msg->boot_count, sizeof(msg->boot_count));
	system_block += sizeof(msg->boot_count);
	memcpy(system_block, &msg->power_status, sizeof(msg->power_status));
	system_block += sizeof(msg->power_status);
	memcpy(system_block, &msg->fpga_version, sizeof(msg->fpga_version));
	system_block += sizeof(msg->fpga_version);
	memcpy(system_block, &msg->fpga_config_bank, sizeof(msg->fpga_config_bank));
	system_block += sizeof(msg->fpga_config_bank);
	memcpy(system_block, &msg->fpga_fallback_state, sizeof(msg->fpga_fallback_state));
	system_block += sizeof(msg->fpga_fallback_state);
	memcpy(system_block, &msg->received_command_count, sizeof(msg->received_command_count));
	system_block += sizeof(msg->received_command_count);
	memcpy(system_block, &msg->last_csp_port, sizeof(msg->last_csp_port));
	system_block += sizeof(msg->last_csp_port);
	memcpy(system_block, &msg->last_command_id, sizeof(msg->last_command_id));
	system_block += sizeof(msg->last_command_id);
	memcpy(system_block, &msg->ecc_error_count_by_auto, sizeof(msg->ecc_error_count_by_auto));
	system_block += sizeof(msg->ecc_error_count_by_auto);
	memcpy(system_block, &msg->ecc_error_count_by_bus, sizeof(msg->ecc_error_count_by_bus));
	system_block += sizeof(msg->ecc_error_count_by_bus);
	memcpy(system_block, &msg->sem_error_count, sizeof(msg->sem_error_count));
	system_block += sizeof(msg->sem_error_count);
}

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

	for (pos = 0; pos < IO_TEMP_POS_NUM; pos++) {
		memcpy(temp_block, &msg->io[pos].status, sizeof(msg->io[pos].status));
		temp_block += sizeof(msg->io[pos].status);
		memcpy(temp_block, &msg->io[pos].temp, sizeof(msg->io[pos].temp));
		temp_block += sizeof(msg->io[pos].temp);
	}
}

static void copy_cv_to_syshk(struct cv_msg *msg)
{
	int pos;
	uint8_t *cv_block = syshk.cv_block;

	for (pos = 0; pos < OBC_CV_POS_NUM; pos++) {
		memcpy(cv_block, &msg->obc[pos].status, sizeof(msg->obc[pos].status));
		cv_block += sizeof(msg->obc[pos].status);
		memcpy(cv_block, &msg->obc[pos].cv, sizeof(msg->obc[pos].cv));
		cv_block += sizeof(msg->obc[pos].cv);
	}

	for (pos = 0; pos < OBC_XADC_CV_POS_NUM; pos++) {
		memcpy(cv_block, &msg->xadc[pos].status, sizeof(msg->xadc[pos].status));
		cv_block += sizeof(msg->xadc[pos].status);
		memcpy(cv_block, &msg->xadc[pos].cv, sizeof(msg->xadc[pos].cv));
		cv_block += sizeof(msg->xadc[pos].cv);
	}

	for (pos = 0; pos < IO_CV_POS_NUM; pos++) {
		memcpy(cv_block, &msg->io[pos].status, sizeof(msg->io[pos].status));
		cv_block += sizeof(msg->io[pos].status);
		memcpy(cv_block, &msg->io[pos].cv, sizeof(msg->io[pos].cv));
		cv_block += sizeof(msg->io[pos].cv);
	}
}

static void copy_mgnm_to_syshk(struct mgnm_msg *msg)
{
	int pos;
	uint8_t *mgnm_block = syshk.mgnm_block;

	for (pos = 0; pos < MGNM_POS_NUM; pos++) {
		memcpy(mgnm_block, &msg->magnet[pos].status, sizeof(msg->magnet[pos].status));
		mgnm_block += sizeof(msg->magnet[pos].status);
		memcpy(mgnm_block, &msg->magnet[pos].data, sizeof(msg->magnet[pos].data));
		mgnm_block += sizeof(msg->magnet[pos].data);
	}

	for (pos = 0; pos < MGNM_POS_NUM; pos++) {
		memcpy(mgnm_block, &msg->temp[pos].status, sizeof(msg->temp[pos].status));
		mgnm_block += sizeof(msg->temp[pos].status);
		memcpy(mgnm_block, &msg->temp[pos].data, sizeof(msg->temp[pos].data));
		mgnm_block += sizeof(msg->temp[pos].data);
	}
}

static void syshk_sub_task(void *sub)
{
	const struct zbus_channel *chan;

	const struct zbus_observer *subscriber = sub;

	struct system_msg system_msg;
	struct temp_msg temp_msg;
	struct cv_msg cv_msg;
	struct mgnm_msg mgnm_msg;

	LOG_INF("Start the system HK Subscribing thread");

	while (!zbus_sub_wait(subscriber, &chan, K_FOREVER)) {
		if (&system_chan == chan) {
			zbus_chan_read(chan, &system_msg,
				       K_MSEC(CONFIG_SCSAT1_MAIN_ZBUS_READ_TIMEOUT_MSEC));
			LOG_DBG("Subscribe SYSTEM msg %d byte", sizeof(system_msg));
			copy_system_to_syshk(&system_msg);
		} else if (&temp_chan == chan) {
			zbus_chan_read(chan, &temp_msg,
				       K_MSEC(CONFIG_SCSAT1_MAIN_ZBUS_READ_TIMEOUT_MSEC));
			LOG_DBG("Subscribe TEMP msg %d byte", sizeof(temp_msg));
			copy_temp_to_syshk(&temp_msg);
		} else if (&cv_chan == chan) {
			zbus_chan_read(chan, &cv_msg,
				       K_MSEC(CONFIG_SCSAT1_MAIN_ZBUS_READ_TIMEOUT_MSEC));
			LOG_DBG("Subscribe CV msg %d byte", sizeof(cv_msg));
			copy_cv_to_syshk(&cv_msg);
		} else if (&mgnm_chan == chan) {
			zbus_chan_read(chan, &mgnm_msg,
				       K_MSEC(CONFIG_SCSAT1_MAIN_ZBUS_READ_TIMEOUT_MSEC));
			LOG_DBG("Subscribe MGNM msg %d byte", sizeof(mgnm_msg));
			copy_mgnm_to_syshk(&mgnm_msg);
		} else {
			LOG_ERR("Wrong channel %p!", chan);
			continue;
		}
	}
}

K_THREAD_DEFINE(syshk_sub_task_id, CONFIG_SCSAT1_MAIN_SUB_SYSHK_THREAD_STACK_SIZE, syshk_sub_task,
		&syshk_sub, NULL, NULL, CONFIG_SCSAT1_MAIN_SUB_SYSHK_THREAD_PRIORITY, 0, 0);

static void send_syshk(struct k_work *work)
{
	csp_conn_t *conn;
	csp_packet_t *packet;

	if (!IS_ENABLED(CONFIG_SCSAT1_MAIN_AUTO_SYSHK_DOWNLINK)) {
		goto end;
	}

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

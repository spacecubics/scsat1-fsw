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
#include "common.h"
#include "temp_test.h"
#include "cv_test.h"
#include "imu_test.h"
#include "gnss_test.h"
#include "rw_test.h"
#include "syshk.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(csp);

#define ROUTER_STACK_SIZE (256U)
#define SERVER_STACK_SIZE (1024U)
#define ROUTER_PRIO       (0U)
#define SERVER_PRIO       (0U)

#define CSP_GET_SYSHK_PORT (10U)

extern uint8_t syshk_tail;
extern struct rw_count_data rw_data_fifo[SYSHK_FIFO_NUM];
extern struct adcs_temp_test_result temp_ret_fifo[SYSHK_FIFO_NUM];
extern struct adcs_cv_test_result cv_ret_fifo[SYSHK_FIFO_NUM];
extern struct imu_test_result imu_ret_fifo[SYSHK_FIFO_NUM];
extern struct gnss_hwmon_result gnss_hwmon_ret_fifo[SYSHK_FIFO_NUM];
extern struct gnss_bestpos_result gnss_bestpos_ret_fifo[SYSHK_FIFO_NUM];
extern struct all_test_result test_ret_fifo[SYSHK_FIFO_NUM];

static csp_iface_t *can_iface = NULL;

void server();

static void *router_task(void *param)
{

	while (true) {
		csp_route_work();
	}

	return NULL;
}

static void *server_task(void *p1, void *p2, void *p3)
{

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	server();

	return NULL;
}

K_THREAD_DEFINE(router_id, ROUTER_STACK_SIZE, router_task, NULL, NULL, NULL, ROUTER_PRIO, 0,
		K_TICKS_FOREVER);
K_THREAD_DEFINE(server_id, SERVER_STACK_SIZE, server_task, NULL, NULL, NULL, SERVER_PRIO, 0,
		K_TICKS_FOREVER);

static void router_start(void)
{
	k_thread_start(router_id);
}

static void server_start(void)
{
	k_thread_start(server_id);
}

extern csp_conf_t csp_conf;

static void csp_get_syshk(csp_conn_t *conn, csp_packet_t *packet)
{
	enum adcs_obc_tlm_type type = packet->data[0];

	switch (type) {
	case TEMP:
		send_syshk(type, &temp_ret_fifo[syshk_tail], sizeof(struct adcs_temp_test_result),
			   conn);
		break;
	case CURRENT_VOLTAGE:
		send_syshk(type, &cv_ret_fifo[syshk_tail], sizeof(struct adcs_cv_test_result),
			   conn);
		break;
	case IMU:
		send_syshk(type, &imu_ret_fifo[syshk_tail], sizeof(struct imu_test_result), conn);
		break;
	case GNSS_HWMON:
		send_syshk(type, &gnss_hwmon_ret_fifo[syshk_tail], sizeof(struct gnss_hwmon_result),
			   conn);
		break;
	case GNSS_BESTPOS:
		send_syshk(type, &gnss_bestpos_ret_fifo[syshk_tail],
			   sizeof(struct gnss_bestpos_result), conn);
		break;
	case RW:
		send_syshk(type, &rw_data_fifo[syshk_tail], sizeof(struct rw_count_data), conn);
		break;
	case ALL_TEST_RESULT:
		send_syshk(type, &test_ret_fifo[syshk_tail], sizeof(struct all_test_result), conn);
		break;
	default:
		goto end;
	}

end:
	return;
}

void server(void)
{

	LOG_INF("Server task started");

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
			case CSP_GET_SYSHK_PORT:
				csp_get_syshk(conn, packet);
				csp_buffer_free(packet);
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

int csp_enable(void)
{
	int ret;
	const char *ifname = "CAN2";
	const struct device *can2 = DEVICE_DT_GET(DT_NODELABEL(can0));
	uint32_t bitrate = 1000000;
	uint16_t filter_addr = CSP_ID_ADCS;
	uint16_t filter_mask = 0x1F;

	LOG_INF("Initialising CSP");

	csp_conf.version = 1;

	csp_init();

	ret = csp_can_open_and_add_interface(can2, ifname, CSP_ID_ADCS, bitrate, filter_addr,
					     filter_mask, &can_iface);
	if (ret != CSP_ERR_NONE) {
		LOG_ERR("failed to add CAN interface [%s], error: %d\n", ifname, ret);
		goto end;
	}
	can_iface->is_default = 1;

	LOG_INF("Connection table");
	csp_conn_print_table();

	LOG_INF("Interfaces");
	csp_iflist_print();

	router_start();

	server_start();
end:
	return ret;
}

void csp_disable(void)
{
	csp_can_stop(can_iface);
}

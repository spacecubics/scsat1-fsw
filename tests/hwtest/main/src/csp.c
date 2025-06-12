/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <csp/csp.h>
#include <csp/csp_id.h>
#include <csp/drivers/can_zephyr.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell_uart.h>
#include "csp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(csp, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

#define ROUTER_STACK_SIZE 256
#define SERVER_STACK_SIZE 1024
#define ROUTER_PRIO       0
#define SERVER_PRIO       0

static csp_iface_t *can_iface1 = NULL;
static csp_iface_t *can_iface2 = NULL;

void server();

static void *router_task(void *param)
{

	/* Here there be routing */
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
			case CSP_PORT_MAIN_SHELL_CMD:
				shell_execute_cmd(shell_backend_uart_get_ptr(), packet->data);
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
	const char *ifname1 = "CAN1";
	const char *ifname2 = "CAN2";
	const struct device *can1 = DEVICE_DT_GET(DT_NODELABEL(can0));
	const struct device *can2 = DEVICE_DT_GET(DT_NODELABEL(can1));
	uint32_t bitrate = 1000000;
	uint16_t filter_addr = CSP_ID_MAIN;
	uint16_t filter_mask = 0x1F;

	LOG_INF("Initialising CSP");

	csp_conf.version = 1;

	csp_init();

	ret = csp_can_open_and_add_interface(can1, ifname1, CSP_ID_MAIN, bitrate, filter_addr,
					     filter_mask, &can_iface1);
	if (ret != CSP_ERR_NONE) {
		LOG_ERR("failed to add CAN interface [%s], error: %d\n", ifname1, ret);
		goto end;
	}
	can_iface1->is_default = 1;

	ret = csp_can_open_and_add_interface(can2, ifname2, CSP_ID_MAIN, bitrate, filter_addr,
					     filter_mask, &can_iface2);
	if (ret != CSP_ERR_NONE) {
		LOG_ERR("failed to add CAN interface [%s], error: %d\n", ifname2, ret);
		goto end;
	}

	csp_rtable_set(CSP_ID_ADCS, csp_id_get_host_bits(), can_iface2, CSP_NO_VIA_ADDRESS);
	csp_rtable_set(CSP_ID_ZERO, csp_id_get_host_bits(), can_iface2, CSP_NO_VIA_ADDRESS);
	csp_rtable_set(CSP_ID_PICO, csp_id_get_host_bits(), can_iface2, CSP_NO_VIA_ADDRESS);

	LOG_INF("Connection table");
	csp_conn_print_table();

	LOG_INF("Interfaces");
	csp_iflist_print();

	csp_print("Route table\n");
	csp_rtable_print();

	router_start();

	server_start();
end:
	return ret;
}

void csp_disable(void)
{
	csp_can_stop(can_iface1);
	csp_can_stop(can_iface2);
}

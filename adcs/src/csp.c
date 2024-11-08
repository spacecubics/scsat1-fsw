/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <csp/csp.h>
#include <csp/drivers/can_zephyr.h>
#include <zephyr/device.h>
#include "sc_csp.h"
#include "flash.h"
#include "file.h"
#include "cmd/handler.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(csp, CONFIG_SCSAT1_ADCS_LOG_LEVEL);

static csp_iface_t *can_iface = NULL;

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

	csp_cmd_handler();

	return NULL; }

K_THREAD_DEFINE(router_id, CONFIG_SCSAT1_ADCS_CSP_ROUTER_THREAD_STACK_SIZE, router_task, NULL, NULL,
		NULL, CONFIG_SCSAT1_ADCS_CSP_ROUTER_THREAD_PRIORITY, 0, K_TICKS_FOREVER);
K_THREAD_DEFINE(server_id, CONFIG_SCSAT1_ADCS_CSP_SERVER_THREAD_STACK_SIZE, server_task, NULL, NULL,
		NULL, CONFIG_SCSAT1_ADCS_CSP_SERVER_THREAD_PRIORITY, 0, K_TICKS_FOREVER);

static void router_start(void)
{
	k_thread_start(router_id);
}

static void server_start(void)
{
	k_thread_start(server_id);
}

extern csp_conf_t csp_conf;

int sc_csp_init(void)
{
	int ret;
	const char *ifname = "CAN2";
	const struct device *can2 = DEVICE_DT_GET(DT_NODELABEL(can0));
	uint16_t filter_addr = CSP_ID_ADCS;
	uint16_t filter_mask = 0x1F;

	LOG_INF("Initialising CSP");

	csp_conf.version = 1;

	csp_init();

	ret = csp_can_open_and_add_interface(can2, ifname, CSP_ID_ADCS, CSP_CAN_BITRATE,
					     filter_addr, filter_mask, &can_iface);
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

	csp_flash_handler_init();
	csp_file_handler_init();
end:
	return ret;
}

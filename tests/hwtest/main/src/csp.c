#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <csp/csp.h>
#include <csp/drivers/can_zephyr.h>
#include <zephyr/device.h>
#include "csp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(csp);

#define ROUTER_STACK_SIZE 256
#define SERVER_STACK_SIZE 1024
#define ROUTER_PRIO 0
#define SERVER_PRIO 0

static csp_iface_t *can_iface1 = NULL;
static csp_iface_t *can_iface2 = NULL;

static void *router_task(void *param) {

	/* Here there be routing */
	while (true) {
		csp_route_work();
	}

	return NULL;
}

K_THREAD_DEFINE(router_id, ROUTER_STACK_SIZE,
				router_task, NULL, NULL, NULL,
				ROUTER_PRIO, 0, K_TICKS_FOREVER);

static void router_start(void) {
	k_thread_start(router_id);
}

extern csp_conf_t csp_conf;

int csp_enable(void)
{
	int ret;
	const char *rtable = "2 CAN2,3 CAN2";
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

	ret = csp_can_open_and_add_interface(can1, ifname1, CSP_ID_MAIN, bitrate,
						   filter_addr, filter_mask, &can_iface1);
	if (ret != CSP_ERR_NONE) {
		LOG_ERR("failed to add CAN interface [%s], error: %d\n", ifname1, ret);
		goto end;
	}
	can_iface1->is_default = 1;

	ret = csp_can_open_and_add_interface(can2, ifname2, CSP_ID_MAIN, bitrate,
						   filter_addr, filter_mask, &can_iface2);
	if (ret != CSP_ERR_NONE) {
		LOG_ERR("failed to add CAN interface [%s], error: %d\n", ifname1, ret);
		goto end;
	}

	ret = csp_rtable_load(rtable);
	if (ret < 1) {
		csp_print("csp_rtable_load(%s) failed, error: %d\n", rtable, ret);
		goto end;
	}

	LOG_INF("Connection table");
	csp_conn_print_table();

	LOG_INF("Interfaces");
	csp_iflist_print();

	csp_print("Route table\n");
	csp_rtable_print();

	router_start();
end:
	return ret;
}

void csp_disable(void)
{
	csp_can_stop(can_iface1);
	csp_can_stop(can_iface2);
}

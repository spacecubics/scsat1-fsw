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
#include "csp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(csp_test);

#define CSP_TIMEOUT_MSEC (100U)
#define MY_SERVER_PORT   (10U)

static int csp_get_pyld_status_cmd(float *temp, uint16_t *count)
{
	int ret = 0;

	csp_conn_t *conn = csp_connect(CSP_PRIO_NORM, CSP_ID_PYLD, MY_SERVER_PORT, CSP_TIMEOUT_MSEC,
				       CSP_O_NONE);
	if (conn == NULL) {
		LOG_ERR("CSP Connection failed");
		ret = -ETIMEDOUT;
		goto end;
	}

	csp_packet_t *packet = csp_buffer_get(0);
	if (packet == NULL) {
		LOG_ERR("Failed to get CSP buffer");
		ret = -ENOBUFS;
		goto end;
	}
	packet->length = 0;

	csp_send(conn, packet);

	packet = csp_read(conn, CSP_TIMEOUT_MSEC);
	if (packet == NULL) {
		LOG_ERR("Failed to CSP read");
		ret = -ETIMEDOUT;
		goto cleanup;
	}

	*temp = (int8_t)packet->data[1] + (float)packet->data[0] * 0.0625f;
	*count = (packet->data[3] << 8) + packet->data[2];
cleanup:
	csp_buffer_free(packet);
	csp_close(conn);

end:
	return ret;
}

int csp_test(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	uint32_t uptime;
	float temp = 0;
	uint16_t count = 0;
	uint8_t csp_id_list[] = {
		CSP_ID_EPS,
		CSP_ID_SRS3,
		CSP_ID_ADCS,
		CSP_ID_PYLD,
	};
	const char csp_name_list[][20] = {
		"EPS",
		"SRS-3",
		"ADCS Board",
		"Payload Board",
	};

	for (int i = 0; i < ARRAY_SIZE(csp_id_list); i++) {
		ret = csp_ping(csp_id_list[i], CSP_TIMEOUT_MSEC, 1, CSP_O_NONE);
		if (ret < 0) {
			LOG_ERR("Ping to %s: Failed", csp_name_list[i]);
			(*err_cnt)++;
			all_ret = -1;
		} else {
			LOG_INF("Ping to %s: %d [ms]", csp_name_list[i], ret);
		}

		ret = csp_get_uptime(csp_id_list[i], CSP_TIMEOUT_MSEC, &uptime);
		if (ret < 0) {
			LOG_ERR("Uptime of %s: Failed", csp_name_list[i]);
			(*err_cnt)++;
			all_ret = -1;
		} else {
			LOG_INF("Uptime of %s: %d [s]", csp_name_list[i], uptime);
		}
	}

	ret = csp_get_pyld_status_cmd(&temp, &count);
	if (ret < 0) {
		LOG_ERR("Payload Board Temperature: Failed");
		LOG_ERR("Payload Board JPEG Count: Failed");
		(*err_cnt)++;
		all_ret = -1;
	} else {
		LOG_INF("Payload Board Temperature: %.1f [deg]", (double)temp);
		LOG_INF("Payload Board JPEG Count: %d", count);
	}

	return all_ret;
}

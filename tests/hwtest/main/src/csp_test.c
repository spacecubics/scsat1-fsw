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
#include "csp.h"
#include "csp_test.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(csp_test);

#define CSP_TIMEOUT_MSEC   (100U)
#define MY_SERVER_PORT     (10U)
#define CSP_INVALID_PING   (0U)
#define CSP_INVALID_UPTIME (0U)
#define CSP_INVALID_TEMP   (0U)
#define CSP_INVALID_COUNT  (0U)

extern enum hwtest_mode test_mode;

struct pyld_status_data {
	float temp;
	uint16_t jpeg_count;
};

static int csp_get_pyld_status_cmd(struct pyld_status_data *status, bool log)
{
	int ret = 0;

	csp_conn_t *conn = csp_connect(CSP_PRIO_NORM, CSP_ID_ZERO, MY_SERVER_PORT, CSP_TIMEOUT_MSEC,
				       CSP_O_NONE);
	if (conn == NULL) {
		HWTEST_LOG_ERR(log, "CSP Connection failed");
		ret = -ETIMEDOUT;
		goto end;
	}

	csp_packet_t *packet = csp_buffer_get(0);
	if (packet == NULL) {
		HWTEST_LOG_ERR(log, "Failed to get CSP buffer");
		ret = -ENOBUFS;
		goto end;
	}
	packet->length = 0;

	csp_send(conn, packet);

	packet = csp_read(conn, CSP_TIMEOUT_MSEC);
	if (packet == NULL) {
		HWTEST_LOG_ERR(log, "Failed to CSP read");
		ret = -ETIMEDOUT;
		goto cleanup;
	}

	status->temp = (int8_t)packet->data[1] + (float)packet->data[0] * 0.0625f;
	status->jpeg_count = (packet->data[3] << 8) + packet->data[2];
cleanup:
	csp_buffer_free(packet);
	csp_close(conn);

end:
	return ret;
}

int csp_test(struct csp_test_result *csp_ret, uint32_t *err_cnt, bool log)
{
	int ret;
	int all_ret = 0;
	uint32_t uptime;
	struct pyld_status_data pyld_status = {0};
	uint8_t csp_id_list[] = {
		CSP_ID_EPS,
		CSP_ID_SRS3,
		CSP_ID_ADCS,
		CSP_ID_ZERO,
		CSP_ID_PICO,
	};
	const char csp_name_list[][12] = {
		"EPS",
		"SRS-3",
		"ADCS Board",
		"Zero",
		"Pico",
	};

	for (int i = 0; i < ARRAY_SIZE(csp_id_list); i++) {

		if ((csp_id_list[i] == CSP_ID_ADCS) && test_mode < MAIN_ADCS_ONLY) {
			continue;
		}
		if ((csp_id_list[i] == CSP_ID_ZERO) && test_mode < FULL) {
			continue;
		}

		ret = csp_ping(csp_id_list[i], CSP_TIMEOUT_MSEC, 1, CSP_O_NONE);
		if (ret < 0) {
			csp_ret->ping[i].status = ret;
			csp_ret->ping[i].data = CSP_INVALID_PING;
			HWTEST_LOG_ERR(log, "Ping to %s: Failed", csp_name_list[i]);
			(*err_cnt)++;
			all_ret = -1;
		} else {
			csp_ret->ping[i].status = 0;
			csp_ret->ping[i].data = ret;
			HWTEST_LOG_INF(log, "Ping to %s: %d [ms]", csp_name_list[i], ret);
		}

		ret = csp_get_uptime(csp_id_list[i], CSP_TIMEOUT_MSEC, &uptime);
		if (ret < 0) {
			csp_ret->uptime[i].data = CSP_INVALID_UPTIME;
			HWTEST_LOG_ERR(log, "Uptime of %s: Failed", csp_name_list[i]);
			(*err_cnt)++;
			all_ret = -1;
		} else {
			csp_ret->uptime[i].data = uptime;
			HWTEST_LOG_INF(log, "Uptime of %s: %d [s]", csp_name_list[i], uptime);
		}
		csp_ret->uptime[i].status = ret;
	}

	if (test_mode < FULL) {
		goto end;
	}

	ret = csp_get_pyld_status_cmd(&pyld_status, log);
	if (ret < 0) {
		csp_ret->temp_pyld.data = CSP_INVALID_TEMP;
		csp_ret->jpeg_count_pyld.data = CSP_INVALID_COUNT;
		HWTEST_LOG_ERR(log, "Zero Temperature: Failed");
		HWTEST_LOG_ERR(log, "Zero JPEG Count: Failed");
		(*err_cnt)++;
		all_ret = -1;
	} else {
		csp_ret->temp_pyld.data = pyld_status.temp;
		csp_ret->jpeg_count_pyld.data = pyld_status.jpeg_count;
		HWTEST_LOG_INF(log, "Zero Temperature: %.1f [deg]",
			       (double)pyld_status.temp);
		HWTEST_LOG_INF(log, "Zero JPEG Count: %d", pyld_status.jpeg_count);
	}
	csp_ret->temp_pyld.status = ret;
	csp_ret->jpeg_count_pyld.status = ret;

end:
	return all_ret;
}

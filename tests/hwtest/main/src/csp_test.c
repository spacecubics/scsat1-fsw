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
#define CSP_GET_TEMP_ZERO  (11U)
#define CSP_INIT_DIR_ZERO  (12U)
#define CSP_GET_COUNT_ZERO (14U)
#define CSP_INVALID_PING   (0U)
#define CSP_INVALID_UPTIME (0U)
#define CSP_INVALID_TEMP   (0U)
#define CSP_INVALID_COUNT  (0U)

extern enum hwtest_mode test_mode;

static csp_packet_t *csp_send_cmd_to_zero(uint8_t port, bool reply, bool log)
{
	csp_packet_t *packet = NULL;

	csp_conn_t *conn =
		csp_connect(CSP_PRIO_NORM, CSP_ID_ZERO, port, CSP_TIMEOUT_MSEC, CSP_O_NONE);
	if (conn == NULL) {
		HWTEST_LOG_ERR(log, "CSP Connection failed");
		goto end;
	}

	packet = csp_buffer_get(0);
	if (packet == NULL) {
		HWTEST_LOG_ERR(log, "Failed to get CSP buffer");
		goto end;
	}
	packet->length = 0;

	csp_send(conn, packet);

	if (reply) {
		packet = csp_read(conn, CSP_TIMEOUT_MSEC);
		if (packet == NULL) {
			HWTEST_LOG_ERR(log, "Failed to CSP read");
			goto cleanup;
		}
	}

cleanup:
	csp_close(conn);

end:
	return packet;
}

static int csp_get_zero_temp(float *temp, bool log)
{
	csp_packet_t *packet;
	int ret = 0;
	bool reply = true;

	packet = csp_send_cmd_to_zero(CSP_GET_TEMP_ZERO, reply, log);
	if (packet == NULL || packet->length != 2) {
		ret = -1;
		goto end;
	}

	*temp = (int8_t)packet->data[1] + (float)packet->data[0] * 0.0625f;
	csp_buffer_free(packet);
end:
	return ret;
}

static int csp_get_zero_frame_count(uint16_t *count, bool log)
{
	csp_packet_t *packet;
	int ret = 0;
	bool reply = true;

	packet = csp_send_cmd_to_zero(CSP_GET_COUNT_ZERO, reply, log);
	if (packet == NULL || packet->length != 2) {
		ret = -1;
		goto end;
	}

	*count = (packet->data[1] << 8) + packet->data[0];
	csp_buffer_free(packet);
end:
	return ret;
}

int csp_test_init(void)
{
	int ret = 0;
	csp_packet_t *packet;
	bool reply = false;
	bool log = true;

	packet = csp_send_cmd_to_zero(CSP_INIT_DIR_ZERO, reply, log);
	if (packet == NULL) {
		ret = -1;
	}

	return ret;
}

int csp_test(struct csp_test_result *csp_ret, uint32_t *err_cnt, bool log)
{
	int ret;
	int all_ret = 0;
	uint32_t uptime;
	float zero_temp;
	uint16_t zero_fcount;
	uint8_t csp_id_list[] = {
		CSP_ID_EPS, CSP_ID_SRS3, CSP_ID_ADCS, CSP_ID_ZERO, CSP_ID_PICO,
	};
	const char csp_name_list[][12] = {
		"EPS", "SRS-3", "ADCS Board", "Zero", "Pico",
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

	ret = csp_get_zero_temp(&zero_temp, log);
	if (ret < 0) {
		csp_ret->temp_zero.data = CSP_INVALID_TEMP;
		HWTEST_LOG_ERR(log, "Zero Temperature: Failed");
		(*err_cnt)++;
		all_ret = -1;
	} else {
		csp_ret->temp_zero.data = zero_temp;
		HWTEST_LOG_INF(log, "Zero Temperature: %.1f [deg]", (double)zero_temp);
	}
	csp_ret->temp_zero.status = ret;

	ret = csp_get_zero_frame_count(&zero_fcount, log);
	if (ret < 0) {
		csp_ret->fcount_zero.data = CSP_INVALID_COUNT;
		HWTEST_LOG_ERR(log, "Zero Frame Count: Failed");
		(*err_cnt)++;
		all_ret = -1;
	} else {
		csp_ret->fcount_zero.data = zero_fcount;
		HWTEST_LOG_INF(log, "Zero Frame Count: %d", zero_fcount);
	}
	csp_ret->fcount_zero.status = ret;

end:
	return all_ret;
}

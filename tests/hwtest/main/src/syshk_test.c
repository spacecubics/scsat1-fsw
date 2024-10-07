/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <csp/csp.h>
#include "common.h"
#include "pwrctrl.h"
#include "temp_test.h"
#include "cv_test.h"
#include "csp_test.h"
#include "sunsens_test.h"
#include "mgnm_test.h"
#include "dstrx3_test.h"
#include "mtq.h"
#include "csp.h"
#include "syshk.h"
#include "version.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(syshk_test, CONFIG_SCSAT1_MAIN_LOG_LEVEL);

extern struct k_event loop_event;
extern enum hwtest_mode test_mode;
extern char last_cmd[];

struct all_test_result {
	uint32_t loop_count;
	uint32_t err_cnt;
	char version[32];
	char last_cmd[32];
};

#define CSP_GET_SYSHK_PORT       (10U)
#define CSP_GET_TEMP_CMD         (1U)
#define CSP_GET_CV_CMD           (2U)
#define CSP_GET_IMU_CMD          (3U)
#define CSP_GET_GNSS_HWMON_CMD   (4U)
#define CSP_GET_GNSS_BESTPOS_CMD (5U)
#define CSP_GET_RW_CMD           (6U)
#define CSP_GET_RET_CMD          (7U)

/* TODO: This is workaround for send ADCS telemetry to GND */
void csp_send_direct(csp_id_t* idout, csp_packet_t * packet, csp_iface_t * routed_from);

static int send_adcs_syshk(void)
{
	int ret = 0;
	csp_conn_t *adcs_conn;
	csp_conn_t *gnd_conn;
	csp_packet_t *packet;
	uint8_t csp_cmd_list[] = {
		CSP_GET_TEMP_CMD,         CSP_GET_CV_CMD, CSP_GET_IMU_CMD, CSP_GET_GNSS_HWMON_CMD,
		CSP_GET_GNSS_BESTPOS_CMD, CSP_GET_RW_CMD, CSP_GET_RET_CMD,
	};

	gnd_conn = csp_connect(CSP_PRIO_NORM, CSP_ID_GND, SYSHK_PORT, CSP_SYSHK_TIMEOUT_MSEC,
			       CSP_O_NONE);
	if (gnd_conn == NULL) {
		LOG_ERR("CSP Connection failed for GND");
		ret = -ETIMEDOUT;
		goto end;
	}

	adcs_conn = csp_connect(CSP_PRIO_NORM, CSP_ID_ADCS, CSP_GET_SYSHK_PORT,
				CSP_SYSHK_TIMEOUT_MSEC, CSP_O_NONE);
	if (adcs_conn == NULL) {
		LOG_ERR("CSP Connection failed for ADCS");
		ret = -ETIMEDOUT;
		goto cleanup;
	}

	for (int i = 0; i < ARRAY_SIZE(csp_cmd_list); i++) {

		packet = csp_buffer_get(0);
		if (packet == NULL) {
			LOG_ERR("Failed to get CSP buffer");
			ret = -ENOBUFS;
			continue;
		}
		memcpy(&packet->data, &csp_cmd_list[i], sizeof(uint8_t));
		packet->length = sizeof(uint8_t);

		csp_send(adcs_conn, packet);

		packet = csp_read(adcs_conn, CSP_SYSHK_TIMEOUT_MSEC);
		if (packet == NULL) {
			LOG_ERR("Failed to CSP read for ADCS HK (cmd:%d)", csp_cmd_list[i]);
			ret = -ETIMEDOUT;
			continue;
		}

		/* Send ADCS HK to Ground */
		/* TODO: This is workaround for send ADCS telemetry to GND */
		packet->id.dst = CSP_ID_GND;
		packet->id.dport = SYSHK_PORT;
		LOG_INF("Send ADCS HK (src:%d dst:%d)", packet->id.src, packet->id.dst);
		csp_send_direct(&packet->id, packet, NULL);

		csp_buffer_free(packet);
	}

	csp_close(adcs_conn);
cleanup:
	csp_close(gnd_conn);

end:
	return ret;
}

static void update_mtq_idx(uint8_t *axes_idx, uint8_t *pol_idx)
{
	if (*pol_idx < 2) {
		(*pol_idx)++;
	} else {
		*pol_idx = 0;
		(*axes_idx)++;
	}

	if (*axes_idx >= 3) {
		*axes_idx = 0;
	}
}

static int one_loop(uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	struct main_temp_test_result temp_ret;
	struct main_cv_test_result cv_ret;
	struct csp_test_result csp_ret;
	struct sunsens_test_ret sunsens_ret;
	struct mgnm_test_ret mgnm_ret;
	struct dstrx3_test_ret dstrx3_ret;
	struct all_test_result test_ret;
	static uint32_t loop_count = 0;

	/*
	 * The tests in CSP have some that are not conducted depending on
	 * the test mode, so all result fields are initialized to 0.
	 */
	memset(&csp_ret, 0, sizeof(csp_ret));

	LOG_INF("===[Temp Test Start (total err: %d)]===", *err_cnt);
	ret = temp_test(&temp_ret, err_cnt, LOG_DISABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	send_syshk(TEMP, &temp_ret, sizeof(temp_ret));

	k_sleep(K_MSEC(100));

	LOG_INF("===[CV Test Start (total err: %d)]===", *err_cnt);
	ret = cv_test(&cv_ret, err_cnt, LOG_DISABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	send_syshk(CURRENT_VOLTAGE, &cv_ret, sizeof(cv_ret));

	k_sleep(K_MSEC(100));

	LOG_INF("===[CSP Test Start (total err: %d)]===", *err_cnt);
	ret = csp_test(&csp_ret, err_cnt, LOG_DISABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	send_syshk(CSP, &csp_ret, sizeof(csp_ret));

	k_sleep(K_MSEC(100));

	LOG_INF("===[Sun Sensor Test Start (total err: %d)]===", *err_cnt);
	ret = sunsens_test(&sunsens_ret, err_cnt, LOG_DISABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	send_syshk(SUN_SENSOR, &sunsens_ret, sizeof(sunsens_ret));

	k_sleep(K_MSEC(100));

	LOG_INF("===[Magnetometer Test Start (total err: %d)]===", *err_cnt);
	ret = mgnm_test(&mgnm_ret, err_cnt, LOG_DISABLE);
	if (ret < 0) {
		all_ret = -1;
	}

	send_syshk(MAGNET_METER, &mgnm_ret, sizeof(mgnm_ret));

	k_sleep(K_MSEC(100));

	if (test_mode >= MAIN_ONLY) {

		LOG_INF("===[DSTRX-3 Test Start (total err: %d)]===", *err_cnt);
		ret = dstrx3_test(&dstrx3_ret, err_cnt, LOG_DISABLE);
		if (ret < 0) {
			all_ret = -1;
		}

		send_syshk(DSTRX3, &dstrx3_ret, sizeof(dstrx3_ret));

		k_sleep(K_MSEC(100));
	}

	memset(&test_ret, 0, sizeof(test_ret));
	test_ret.loop_count = loop_count;
	test_ret.err_cnt = *err_cnt;
	strcpy(test_ret.version, MAIN_HWTEST_VERSION);
	strcpy(test_ret.last_cmd, last_cmd);

	send_syshk(ALL_TEST_RESULT, &test_ret, sizeof(test_ret));
	loop_count++;

	if (test_mode >= MAIN_ADCS_ONLY) {

		LOG_INF("===[ADCS HK Send Start (total err: %d)]===", *err_cnt);
		send_adcs_syshk();
	}

	return all_ret;
}

static bool is_loop_stop(void)
{
	if (k_event_wait(&loop_event, LOOP_STOP_EVENT, false, K_NO_WAIT) != 0) {
		return true;
	}

	return false;
}

int syshk_test(int32_t loop_count, uint32_t *err_cnt)
{
	int ret;
	int all_ret = 0;
	float duty = 1.0;
	enum mtq_axes axes_list[] = {
		MTQ_AXES_MAIN_X,
		MTQ_AXES_MAIN_Y,
		MTQ_AXES_MAIN_Z,
	};
	enum mtq_polarity pol_list[] = {
		MTQ_POL_PLUS,
		MTQ_POL_MINUS,
		MTQ_POL_NON,
	};
	uint8_t axes_idx = 0;
	uint8_t pol_idx = 0;

	if (loop_count < 0) {
		loop_count = INT32_MAX;
	}

	if (test_mode >= MAIN_ADCS_ONLY) {
		csp_test_init();
		k_sleep(K_SECONDS(10));
	}

	for (int i = 1; i <= loop_count; i++) {
		if (is_loop_stop()) {
			break;
		}

		ret = mtq_start(axes_list[axes_idx], pol_list[pol_idx], duty);
		if (ret < 0) {
			(*err_cnt)++;
			all_ret = -1;
		}

		for (int j = 0; j < 5; j++) {
			ret = one_loop(err_cnt);
			if (ret < 0) {
				all_ret = -1;
			}
		}

		ret = mtq_stop(axes_list[axes_idx]);
		if (ret < 0) {
			(*err_cnt)++;
			all_ret = -1;
		}

		update_mtq_idx(&axes_idx, &pol_idx);
	}

	return all_ret;
}

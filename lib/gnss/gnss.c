/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include "gnss.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gnss, CONFIG_SC_LIB_GNSS_LOG_LEVEL);

#define GNSS_WAKE_RETRY_COUNT  (200U)
#define GNSS_HWMON_RETRY_COUNT (2000U)
#define GNSS_PROMPT            "[COM1]"
#define GNSS_HWMON_ONCE_CMD    "log hwmonitora once\n"
#define GNSS_BESTPOS_ONCE_CMD  "log bestposa once\n"
#define GNSS_RESP_MAX_SIZE     (300U)
#define GNSS_HWMON_DATA_NUM    (24U)
#define GNSS_BESTPOS_DATA_NUM  (33U)

const struct device *gnss = DEVICE_DT_GET(DT_NODELABEL(gnss_uart));

static void gnss_fifo_clear(void)
{
	int ret;
	uint8_t c;

	while (true) {
		ret = uart_fifo_read(gnss, &c, 1);
		if (ret == 1) {
			continue;
		} else {
			break;
		}
	}
}

static int gnss_parse_hwmon_data(const char *gnss_str, struct gnss_hwmon_data *data)
{
	int ret;
	char ok[5];
	char com[20];
	char port[10];
	uint8_t item_num;
	uint16_t type[5];
	char last[20];

	ret = sscanf(
		gnss_str,
		"%5s %20[^,],%10[^,],%d,%f,%10[^,],%d,%f,%x,%x,%d;%hhu,%f,%hx,%f,%hx,%f,%hx,%f,%"
		"hx,%f,%hx,%f,%s",
		ok, com, port, &data->sequence, &data->idle_time, data->time_status, &data->week,
		&data->seconds, &data->receiver_status, &data->reserved, &data->receiver_version,
		&item_num, &data->temp, &type[0], &data->voltage_3v3, &type[1],
		&data->voltage_antenna, &type[2], &data->core_voltage_1v2, &type[3],
		&data->supply_voltage, &type[4], &data->voltage_1v8, last);

	LOG_DBG("OK: %s", ok);
	LOG_DBG("COM: %s", com);
	LOG_DBG("PORT: %s", port);
	LOG_DBG("SEQUENCE: %d", data->sequence);
	LOG_DBG("IDLE_TIME: %f", (double)data->idle_time);
	LOG_DBG("TIMESTATUS: %s", data->time_status);
	LOG_DBG("WEEK: %d", data->week);
	LOG_DBG("SECONDS: %f", (double)data->seconds);
	LOG_DBG("RECEIVER_STATUS: %d", data->receiver_status);
	LOG_DBG("RESERVED: %x", data->reserved);
	LOG_DBG("RECEIVER_VERSION: %d", data->receiver_version);
	LOG_DBG("TEMP: %f", (double)data->temp);
	LOG_DBG("3V3_VOLTAGE: %f", (double)data->voltage_3v3);
	LOG_DBG("ANTENNA_VOLTAGE: %f", (double)data->voltage_antenna);
	LOG_DBG("1V2_CORE_VOLTAGE: %f", (double)data->core_voltage_1v2);
	LOG_DBG("SUPPLY_VOLTAGE: %f", (double)data->supply_voltage);
	LOG_DBG("1V8_VOLTAGE: %f", (double)data->voltage_1v8);

	if (ret == GNSS_HWMON_DATA_NUM) {
		return 0;
	} else {
		return -1;
	}
}

static int gnss_parse_bestpos_data(const char *gnss_str, struct gnss_bestpos_data *data)
{
	int ret;
	char ok[5];
	char com[20];
	char port[10];
	char last[20];

	ret = sscanf(gnss_str,
		     "%5s %20[^,],%10[^,],%d,%f,%10[^,],%d,%f,%x,%x,%d;%24[^,],%24[^,],%f,%f,%f,%f,"
		     "%16[^,],%f,%f,%f,%12[^,],%f,%f,%hhu,%hhu,%hhu,%hhu,%hhx,%hhx,%hhx,%hhx*%s",
		     ok, com, port, &data->sequence, &data->idle_time, data->time_status,
		     &data->week, &data->seconds, &data->receiver_status, &data->reserved,
		     &data->receiver_version, data->sol_stat, data->pos_type, &data->lat_deg,
		     &data->lon_deg, &data->hgt_deg, &data->undulation, data->datum_id,
		     &data->lat_m, &data->lon_m, &data->hgt_m, data->stn_id, &data->diff_age,
		     &data->sol_age, &data->svs, &data->soln_svs, &data->soln1_svs,
		     &data->solnmulti_svs, &data->pos_reserved, &data->ext_sol_stat,
		     &data->galileo_mask, &data->gps_mask, last);

	LOG_DBG("OK: %s", ok);
	LOG_DBG("COM: %s", com);
	LOG_DBG("PORT: %s", port);
	LOG_DBG("SEQUENCE: %d", data->sequence);
	LOG_DBG("IDLE_TIME: %f", (double)data->idle_time);
	LOG_DBG("TIMESTATUS: %s", data->time_status);
	LOG_DBG("WEEK: %d", data->week);
	LOG_DBG("SECONDS: %f", (double)data->seconds);
	LOG_DBG("RECEIVER_STATUS: %d", data->receiver_status);
	LOG_DBG("RESERVED: %x", data->reserved);
	LOG_DBG("RECEIVER_VERSION: %d", data->receiver_version);
	LOG_DBG("SOL_STAT: %s", data->sol_stat);
	LOG_DBG("POS_TYPE: %s", data->pos_type);
	LOG_DBG("LAT_DEG: %f", (double)data->lat_deg);
	LOG_DBG("LON_DEG: %f", (double)data->lon_deg);
	LOG_DBG("HGT_DEG: %f", (double)data->hgt_deg);
	LOG_DBG("UNDULATION: %f", (double)data->undulation);
	LOG_DBG("DATUM_ID: %s", data->datum_id);
	LOG_DBG("LAT_M: %f", (double)data->lat_m);
	LOG_DBG("LON_M: %f", (double)data->lon_m);
	LOG_DBG("HGT_M: %f", (double)data->hgt_m);
	LOG_DBG("STN_ID: %s", data->stn_id);
	LOG_DBG("DIFF_AGE: %f", (double)data->diff_age);
	LOG_DBG("SOL_AGE: %f", (double)data->sol_age);
	LOG_DBG("SVS: %d", data->svs);
	LOG_DBG("SOLN_SVS: %d", data->soln_svs);
	LOG_DBG("SOLN1_SVS: %d", data->soln1_svs);
	LOG_DBG("SOLNMULTI_SVS: %d", data->solnmulti_svs);
	LOG_DBG("RESERVED: %d", data->pos_reserved);
	LOG_DBG("EXT_SOL_STAT: %02x", data->ext_sol_stat);
	LOG_DBG("GALILEO_MASK: %02x", data->galileo_mask);
	LOG_DBG("GPS_MASK: %02x", data->gps_mask);
	LOG_DBG("LAST: %s", last);

	if (ret == GNSS_BESTPOS_DATA_NUM) {
		return 0;
	} else {
		return -1;
	}

	return 0;
};

static int gnss_wait_uart_data(char *uart_data, bool log)
{
	int ret;
	int i;
	uint8_t c;
	uint16_t read_size = 0;
	uint8_t lf_count = 0;
	uint16_t usec_count = 0;
	uint16_t msec_count = 0;

	for (i = 0; i < GNSS_HWMON_RETRY_COUNT; i++) {
		if (read_size >= GNSS_RESP_MAX_SIZE) {
			break;
		} else if (lf_count == 3) {
			break;
		}

		ret = uart_fifo_read(gnss, &c, 1);
		if (ret < 0) {
			LOG_ERR("Failed to read the UART FIFO. (%d)", ret);
			break;
		} else if (ret == 1) {
			uart_data[read_size] = c;
			if (c == '\n') {
				lf_count++;
			}
			read_size++;
			usec_count++;
			k_sleep(K_USEC(20));
		} else {
			msec_count++;
			k_sleep(K_MSEC(1));
		}
	}

	if (i == GNSS_HWMON_RETRY_COUNT) {
		uart_data[read_size] = '\0';
		LOG_INF("%s", uart_data);
		LOG_ERR("GNSS data not received. read_size: %d", read_size);
		ret = -ETIMEDOUT;
	} else {
		uart_data[read_size] = '\0';
		if (log) {
			LOG_INF("%s", uart_data);
			LOG_INF("All GNSS data received: usec: %d, msec: %d", usec_count,
				msec_count);
		}
	}

	return ret;
}

int gnss_enable(void)
{
	int ret;
	int i;
	uint8_t prompt[strlen(GNSS_PROMPT) + 1];

	LOG_INF("Wait for wake up the GNSS about 10 seconds");
	for (i = 0; i < GNSS_WAKE_RETRY_COUNT; i++) {
		ret = uart_fifo_read(gnss, prompt, sizeof(prompt));
		if (ret == 0) {
			k_sleep(K_MSEC(100));
			continue;
		} else if (ret < 0) {
			LOG_ERR("Failed to read the UART FIFO. (%d)", ret);
			break;
		}
		prompt[strlen(GNSS_PROMPT)] = '\0';
		LOG_INF("GNSS has just awakened. %s", prompt);
		break;
	}

	if (i == GNSS_WAKE_RETRY_COUNT) {
		ret = -ETIMEDOUT;
	}

	return ret;
}

void gnss_disable(void)
{
}

int get_gnss_hwmon_data(struct gnss_hwmon_data *data, bool log)
{
	int ret;
	char uart_data[GNSS_RESP_MAX_SIZE];

	ret = uart_fifo_fill(gnss, GNSS_HWMON_ONCE_CMD, strlen(GNSS_HWMON_ONCE_CMD));
	if (ret < 0) {
		printk("Failed to fill the command strings to UART FIFO. %d\n", ret);
		goto end;
	}

	gnss_fifo_clear();

	ret = gnss_wait_uart_data(uart_data, log);
	if (ret < 0) {
		LOG_ERR("Failed to get the GNSS data. (%d)", ret);
		goto end;
	}

	ret = gnss_parse_hwmon_data(uart_data, data);
	if (ret < 0) {
		LOG_ERR("Failed to parse the GNSS data. (%d)", ret);
	}

end:
	return ret;
}

int get_gnss_bestpos_data(struct gnss_bestpos_data *data, bool log)
{
	int ret;
	char uart_data[GNSS_RESP_MAX_SIZE];

	ret = uart_fifo_fill(gnss, GNSS_BESTPOS_ONCE_CMD, strlen(GNSS_BESTPOS_ONCE_CMD));
	if (ret < 0) {
		printk("Failed to fill the command strings to UART FIFO. %d\n", ret);
		goto end;
	}

	gnss_fifo_clear();

	ret = gnss_wait_uart_data(uart_data, log);
	if (ret < 0) {
		LOG_ERR("Failed to get the GNSS data. (%d)", ret);
		goto end;
	}

	ret = gnss_parse_bestpos_data(uart_data, data);
	if (ret < 0) {
		LOG_ERR("Failed to parse the GNSS data. (%d)", ret);
	}

end:
	return ret;
}

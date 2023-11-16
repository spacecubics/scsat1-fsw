/*
 * Copyright (c) 2023 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gnss);

#define GNSS_WAKE_RETRY_COUNT (200U)
#define GNSS_HWMON_RETRY_COUNT (2000U)
#define GNSS_PROMPT "[COM1]"
#define GNSS_ONCE_CMD "log hwmonitora once\n"
#define GNSS_RESP_MAX_SIZE (300U)

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

static int gnss_wait_data(void)
{
	int ret;
	int i;
	uint8_t c;
	char gnss_data[GNSS_RESP_MAX_SIZE];
	uint16_t read_size = 0;
	uint8_t lf_count = 0;
	uint16_t usec_count = 0;
	uint16_t msec_count = 0;

	for (i=0; i<GNSS_HWMON_RETRY_COUNT; i++) {
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
			gnss_data[read_size] = c;
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
		gnss_data[read_size] = '\0';
		LOG_INF("%s", gnss_data);
		LOG_ERR("GNSS data not received. read_size: %d", read_size);
		ret = -ETIMEDOUT;
	} else {
		gnss_data[read_size] = '\0';
		LOG_INF("%s", gnss_data);
		LOG_INF("All GNSS data received: usec: %d, msec: %d",
				 usec_count, msec_count);
	}

	return ret;
}

int gnss_enable(void)
{
	int ret;
	int i;
	uint8_t prompt[strlen(GNSS_PROMPT)+1];

	LOG_INF("Wait for wake up the GNSS about 10 seconds");
	for (i=0; i<GNSS_WAKE_RETRY_COUNT; i++) {
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

int get_gnss_hwmon_data(void)
{
	int ret;

	ret = uart_fifo_fill(gnss, GNSS_ONCE_CMD, strlen(GNSS_ONCE_CMD));
	if (ret < 0) {
		printk("Failed to fill the command strings to UART FIFO. %d\n", ret);
		goto end;
	}

	gnss_fifo_clear();

	ret = gnss_wait_data();
	if (ret < 0) {
		LOG_ERR("Failed to get the GNSS data. (%d)", ret);
	}
end:
	return ret;
}

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(config_nor, CONFIG_SC_LIB_FLASH_LOG_LEVEL);

#include "sc_fpgasys.h"

K_MUTEX_DEFINE(sc_config_nor_mutex);

int sc_config_nor_erase(uint8_t bank, uint8_t id, off_t offset, size_t size)
{
	int ret;
	const struct flash_area *flash = NULL;

	/*
	 * Since erasing the Configuration Flash memory is not a frequently performed operation,
	 * it will adopt a specification that uniformly disallows concurrent execution,
	 * regardless of differences in banks or partitions.
	 */
	ret = k_mutex_lock(&sc_config_nor_mutex, K_NO_WAIT);
	if (ret != 0) {
		goto end;
	}

	ret = sc_select_cfgmem(bank);
	if (ret < 0) {
		LOG_ERR("Failed to select the config memory (bank:%d)", bank);
		goto unlock;
	}

	ret = flash_area_open(id, &flash);
	if (ret < 0) {
		LOG_ERR("Failed to open the partition (bank:%d, id:%d)", bank, id);
		goto unlock;
	}

	/* If both offset and size are 0, the entire area will be erased. */
	if (offset == 0 && size == 0) {
		size = flash->fa_size;
	}

	ret = flash_area_erase(flash, offset, size);
	if (ret < 0) {
		LOG_ERR("Failed to erase the partition (bank:%d, id:%d)", bank, id);
	} else {
		LOG_INF("Finish to erase the partition (bank:%d, id:%d, offset:%ld, size:%d)", bank,
			id, offset, size);
	}

	flash_area_close(flash);

unlock:
	k_mutex_unlock(&sc_config_nor_mutex);

end:
	return ret;
}

int sc_config_nor_calc_crc(uint8_t bank, uint8_t id, off_t offset, size_t size, uint32_t *crc32)
{
	int ret;
	const struct flash_area *flash = NULL;
	size_t remainig_size = size;
	size_t read_size;
	uint8_t chunk[CONFIG_SC_LIB_FLASH_CRC_CHUNK_SIZE];

	ret = k_mutex_lock(&sc_config_nor_mutex, K_NO_WAIT);
	if (ret != 0) {
		goto end;
	}

	ret = sc_select_cfgmem(bank);
	if (ret < 0) {
		LOG_ERR("Failed to select the config memory (bank:%d)", bank);
		goto unlock;
	}

	ret = flash_area_open(id, &flash);
	if (ret < 0) {
		LOG_ERR("Failed to open the partition (bank:%d, id:%d)", bank, id);
		goto unlock;
	}

	/* If both offset and size are 0, the entire area will be calculated. */
	if (offset == 0 && size == 0) {
		remainig_size = flash->fa_size;
	}

	*crc32 = 0;

	while (remainig_size) {

		if (remainig_size < sizeof(chunk)) {
			read_size = remainig_size;
		} else {
			read_size = sizeof(chunk);
		}

		ret = flash_area_read(flash, offset, chunk, sizeof(chunk));
		if (ret < 0) {
			LOG_ERR("Failed to read the NOR Flash (bank: %d, id:%d) (offset: %ld)",
				bank, id, offset);
			break;
		}

		*crc32 = crc32_ieee_update(*crc32, chunk, read_size);

		remainig_size -= read_size;
		offset += read_size;

		k_sleep(K_MSEC(CONFIG_SC_LIB_CSP_CRC_CALC_SLEEP_MSEC));
	}

	flash_area_close(flash);

	LOG_INF("Finish to calculate CRC32 the partition (bank:%d, id:%d, size:%d, crc32: 0x%08x)",
		bank, id, size, *crc32);

unlock:
	k_mutex_unlock(&sc_config_nor_mutex);

end:
	return ret;
}

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <csp/csp.h>
#include "flash.h"
#include "sc_csp.h"
#include "reply.h"
#include "config_nor.h"
#include "data_nor.h"
#include "fram.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sc_flash, CONFIG_SC_LIB_CSP_LOG_LEVEL);

/* Command size */
#define FLASH_CMD_MIN_SIZE        (1U)
#define FLASH_ERASE_CFG_CMD_SIZE  (11U)
#define FLASH_ERASE_DATA_CMD_SIZE (2U)
#define FLASH_CALC_CRC_CMD_SIZE   (12U)

/* Command ID */
#define FLASH_CFG_ERASE_CMD  (0U)
#define FLASH_DATA_ERASE_CMD (1U)
#define FLASH_CALC_CRC_CMD   (2U)

/* Command argument offset */
#define FLASH_CFG_BANK_OFFSET (1U)
#define FLASH_CFG_PID_OFFET   (2U)
#define FLASH_CFG_FRAM_OFFET  (3U)
#define FLASH_CFG_OFST_OFFSET (4U)
#define FLASH_CFG_SIZE_OFFSET (8U)
#define FLASH_DATA_PID_OFFET  (1U)

#define UNKOWN_COMMAND_ID (0xFF)

struct flash_work_msg {
	struct k_work work;
	uint8_t command_id;
	csp_packet_t *packet;
};

static struct flash_work_msg flash_work_msg;

static void csp_send_crc_reply(csp_packet_t *packet, uint8_t command_id, int err_code, uint8_t bank,
			       uint8_t id, uint32_t crc32)
{
	struct cfg_crc_telemetry tlm;

	tlm.telemetry_id = command_id;
	tlm.error_code = sys_cpu_to_le32(err_code);
	tlm.bank = bank;
	tlm.partition_id = id;
	tlm.crc32 = sys_cpu_to_le32(crc32);

	memcpy(packet->data, &tlm, sizeof(tlm));
	packet->length = sizeof(tlm);

	csp_sendto_reply(packet, packet, CSP_O_SAME);
}

static int csp_flash_cfg_erase_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret;
	uint8_t bank;
	uint8_t partition_id;
	off_t offset;
	size_t size;

	if (packet->length != FLASH_ERASE_CFG_CMD_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EMSGSIZE;
		goto end;
	}

	bank = packet->data[FLASH_CFG_BANK_OFFSET];
	partition_id = packet->data[FLASH_CFG_PID_OFFET];
	offset = sys_le32_to_cpu(*(uint32_t *)&packet->data[FLASH_CFG_OFST_OFFSET]);
	size = sys_le32_to_cpu(*(uint32_t *)&packet->data[FLASH_CFG_SIZE_OFFSET]);

	LOG_INF("Config NOR erase command (bank: %d) (partition_id: %d) (offset: %ld) (size: %d)",
		bank, partition_id, offset, size);

	ret = sc_config_nor_erase(bank, partition_id, offset, size);

end:
	csp_send_std_reply(packet, command_id, ret);
	return ret;
}

static int csp_flash_data_erase_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret;
	uint8_t partition_id;

	if (packet->length != FLASH_ERASE_DATA_CMD_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EMSGSIZE;
		goto end;
	}

	partition_id = packet->data[FLASH_DATA_PID_OFFET];

	LOG_INF("Data NOR erase command (partition_id: %d)", partition_id);

	ret = data_nor_erase(partition_id);

end:
	csp_send_std_reply(packet, command_id, ret);
	return ret;
}

static int csp_flash_calc_crc_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret;
	uint8_t bank = 0;
	uint8_t partition_id = 0;
	uint8_t fram_opt;
	off_t offset = 0;
	size_t size = 0;
	uint32_t crc32 = 0;
	struct fram_cfgmem_crc crc_info;

	if (packet->length != FLASH_CALC_CRC_CMD_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EMSGSIZE;
		goto end;
	}

	bank = packet->data[FLASH_CFG_BANK_OFFSET];
	partition_id = packet->data[FLASH_CFG_PID_OFFET];
	fram_opt = packet->data[FLASH_CFG_FRAM_OFFET];
	offset = sys_le32_to_cpu(*(uint32_t *)&packet->data[FLASH_CFG_OFST_OFFSET]);
	size = sys_le32_to_cpu(*(uint32_t *)&packet->data[FLASH_CFG_SIZE_OFFSET]);

	LOG_INF("Calculate CRC32 command (bank: %d) (partition_id: %d) (offset: %ld) (size: %d)",
		bank, partition_id, offset, size);

	ret = sc_config_nor_calc_crc(bank, partition_id, offset, size, &crc32);
	if (ret == 0 && fram_opt) {
		crc_info.bank = bank;
		crc_info.partition_id = partition_id;
		crc_info.offset = offset;
		crc_info.size = size;
		crc_info.crc32 = crc32;
		(void)sc_fram_update_crc_for_cfgmem(crc_info);
		(void)sc_fram_get_crc_for_cfgmem(&crc_info);
	}

end:
	csp_send_crc_reply(packet, command_id, ret, bank, partition_id, crc32);

	return ret;
}

static void csp_flash_work_handler(struct k_work *item)
{
	uint8_t command_id;

	struct flash_work_msg *msg = CONTAINER_OF(item, struct flash_work_msg, work);
	command_id = msg->command_id;
	csp_packet_t *packet = msg->packet;

	switch (command_id) {
	case FLASH_CFG_ERASE_CMD:
		csp_flash_cfg_erase_cmd(command_id, packet);
		break;
	case FLASH_DATA_ERASE_CMD:
		csp_flash_data_erase_cmd(command_id, packet);
		break;
	case FLASH_CALC_CRC_CMD:
		csp_flash_calc_crc_cmd(command_id, packet);
		break;
	default:
		LOG_ERR("Unkown command code: %d", command_id);
		csp_send_std_reply(packet, UNKOWN_COMMAND_ID, -EINVAL);
		break;
	}

	csp_buffer_free(packet);
}

int csp_flash_handler(csp_packet_t *packet)
{
	int ret = 0;
	uint8_t command_id;

	if (packet == NULL) {
		ret = -EINVAL;
		goto end;
	}

	if (packet->length < FLASH_CMD_MIN_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		command_id = UNKOWN_COMMAND_ID;
		ret = -EMSGSIZE;
		goto reply;
	}

	command_id = packet->data[CSP_COMMAND_ID_OFFSET];

	if (k_work_is_pending(&flash_work_msg.work)) {
		LOG_ERR("Flash command is currently executing, so the request is rejected. "
			"(command id: %d)",
			command_id);
		ret = -EBUSY;
		goto reply;
	}

	flash_work_msg.command_id = command_id;
	flash_work_msg.packet = packet;
	k_work_submit(&flash_work_msg.work);

reply:
	if (ret < 0) {
		csp_send_std_reply(packet, command_id, ret);
		csp_buffer_free(packet);
	}

end:
	return ret;
}

void csp_flash_handler_init(void)
{
	k_work_init(&flash_work_msg.work, csp_flash_work_handler);
}

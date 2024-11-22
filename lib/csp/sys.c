/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <csp/csp.h>
#include "sc_csp.h"
#include "reply.h"
#include "fram.h"
#include "sys.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sys_handler, CONFIG_SC_LIB_CSP_LOG_LEVEL);

/* Command size */
#define SYSTEM_CMD_MIN_SIZE      (1U)
#define SYSTEM_READ_REG_CMD_SIZE (2U)

/* Command ID */
#define SYSTEM_CLEAR_BOOT_COUNT_CMD (0U)
#define SYSTEM_READ_REG_CMD         (1U)

/* Command argument offset */
#define SYSTEM_REG_ADDR_OFFSET (1U)

static int csp_system_clear_boot_count_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret;

	LOG_INF("Clear boot count command");

	ret = sc_fram_clear_boot_count();

	csp_send_std_reply(packet, command_id, ret);

	return ret;
}

static int csp_system_read_reg_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret = 0;
	struct system_reg_telemetry tlm;
	uint32_t addr;
	uint32_t value = 0;

	if (packet->length < SYSTEM_READ_REG_CMD_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		goto reply;
	}

	addr = sys_be32_to_cpu(*(uint32_t *)&packet->data[SYSTEM_REG_ADDR_OFFSET]);
	value = sys_read32(addr);

	LOG_INF("Read FPGA register command: (addr: 0x%08x) (value: 0x%08x) ", addr, value);

reply:
	tlm.telemetry_id = command_id;
	tlm.error_code = sys_cpu_to_le32(ret);
	tlm.reg_value = sys_cpu_to_be32(value);

	memcpy(packet->data, &tlm, sizeof(tlm));
	packet->length = sizeof(tlm);

	csp_sendto_reply(packet, packet, CSP_O_SAME);

	return ret;
}

int csp_system_handler(csp_packet_t *packet)
{
	int ret = 0;
	uint8_t command_id;

	if (packet == NULL) {
		ret = -EINVAL;
		command_id = CSP_UNKNOWN_CMD_CODE;
		goto end;
	}

	if (packet->length < SYSTEM_CMD_MIN_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		command_id = CSP_UNKNOWN_CMD_CODE;
		goto free;
	}

	command_id = packet->data[CSP_COMMAND_ID_OFFSET];

	switch (command_id) {
	case SYSTEM_CLEAR_BOOT_COUNT_CMD:
		csp_system_clear_boot_count_cmd(command_id, packet);
		break;
	case SYSTEM_READ_REG_CMD:
		csp_system_read_reg_cmd(command_id, packet);
		break;
	default:
		LOG_ERR("Unkown command code: %d", command_id);
		ret = -EINVAL;
		command_id = CSP_UNKNOWN_CMD_CODE;
		break;
	}

free:
	if (ret < 0) {
		csp_send_std_reply(packet, command_id, ret);
		csp_buffer_free(packet);
	}

end:
	return ret;
}

void csp_system_update_stat(csp_packet_t *packet, struct csp_stat *stat)
{
	/* Pings from the EPS are not counted */
	if (packet->id.src != CSP_ID_GND) {
		goto end;
	}

	stat->received_command_count++;
	stat->last_csp_port = packet->id.dport;

	if (packet->length > 0) {
		stat->last_command_id = packet->data[CSP_COMMAND_ID_OFFSET];
	} else {
		stat->last_command_id = CSP_UNKNOWN_CMD_CODE;
	}

end:
	return;
}

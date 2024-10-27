/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <csp/csp.h>
#include "file.h"
#include "sc_csp.h"
#include "reply.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(file, CONFIG_SC_LIB_CSP_LOG_LEVEL);

/* Command size */
#define FILE_CMD_MIN_SIZE    (1U)
#define FILE_INFO_CMD_SIZE   (1U) /* without file name length */
#define FILE_REMOVE_CMD_SIZE (1U) /* without file name length */

/* Command ID */
#define FILE_INFO_CMD   (0U)
#define FILE_REMOVE_CMD (1U)

/* Command argument offset */
#define FILE_FNAME_OFFSET (1U)

static int csp_get_file_info(const char *fname, struct fs_dirent *entry)
{
	int ret;

	ret = fs_stat(fname, entry);
	if (ret < 0) {
		LOG_ERR("Faild to get the file info %s (%d)", fname, ret);
	}

	LOG_INF("File Info: (entry %d) (size: %u) (name: %s)", entry->type, entry->size,
		entry->name);

	return ret;
}

static void csp_send_file_info_reply(struct fs_dirent *entry, csp_packet_t *packet,
				     uint8_t command_id, int err_code)
{
	struct file_info_telemetry tlm;

	if (err_code) {
		memset(&tlm, 0, sizeof(tlm));
	} else {
		tlm.entry_type = entry->type;
		tlm.file_size = sys_cpu_to_le32(entry->size);
		strncpy(tlm.file_name, entry->name, CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN);
		tlm.file_name[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN - 1] = '\0';
	}

	tlm.telemetry_id = command_id;
	tlm.error_code = sys_cpu_to_le32(err_code);

	memcpy(packet->data, &tlm, sizeof(tlm));
	packet->length = sizeof(tlm);

	csp_sendto_reply(packet, packet, CSP_O_SAME);
}

static int csp_file_info_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret = 0;
	char fname[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN];
	struct fs_dirent entry;

	if (packet->length != FILE_INFO_CMD_SIZE + CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		goto end;
	}

	strncpy(fname, &packet->data[FILE_FNAME_OFFSET], CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN);
	fname[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN - 1] = '\0';

	LOG_INF("File info command (fname: %s)", fname);

	ret = csp_get_file_info(fname, &entry);

end:
	csp_send_file_info_reply(&entry, packet, command_id, ret);
	return ret;
}

static int csp_file_remove_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret = 0;
	char fname[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN];

	if (packet->length != FILE_REMOVE_CMD_SIZE + CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		goto end;
	}

	strncpy(fname, &packet->data[FILE_FNAME_OFFSET], CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN);
	fname[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN - 1] = '\0';

	LOG_INF("File remove command (fname: %s)", fname);

	ret = fs_unlink(fname);

end:
	csp_send_std_reply(packet, command_id, ret);
	return ret;
}

int csp_file_handler(csp_packet_t *packet)
{
	int ret;
	uint8_t command_id;

	if (packet == NULL) {
		ret = -EINVAL;
		goto end;
	}

	if (packet->length < FILE_CMD_MIN_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		goto free;
	}

	command_id = packet->data[CSP_COMMAND_ID_OFFSET];

	switch (command_id) {
	case FILE_INFO_CMD:
		csp_file_info_cmd(command_id, packet);
		break;
	case FILE_REMOVE_CMD:
		csp_file_remove_cmd(command_id, packet);
		break;
	default:
		LOG_ERR("Unkown command code: %d", command_id);
		ret = -EINVAL;
		break;
	}

free:
	csp_buffer_free(packet);

end:
	return ret;
}

/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <csp/csp.h>
#include "file.h"
#include "sc_csp.h"
#include "reply.h"
#include "upload.h"
#include "sc_fpgaconf.h"
#include "sc_fpgasys.h"
#include "fram.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(file, CONFIG_SC_LIB_CSP_LOG_LEVEL);

K_THREAD_STACK_DEFINE(file_workq_stack, CONFIG_SC_LIB_CSP_FILE_THREAD_STACK_SIZE);

struct file_work {
	struct k_work work;
	csp_packet_t *packet;
};

struct k_work_q file_workq;
struct file_work file_works[CONFIG_SC_LIB_CSP_MAX_FILE_WORK];

/* Command size */
#define FILE_CMD_MIN_SIZE         (1U)
#define FILE_INFO_CMD_SIZE        (3U)  /* without file name length */
#define FILE_REMOVE_CMD_SIZE      (1U)  /* without file name length */
#define FILE_COPY_TO_CFG_CMD_SIZE (15U) /* without file name length */

/* Command ID */
#define FILE_INFO_CMD         (0U)
#define FILE_REMOVE_CMD       (1U)
#define FILE_UPLOAD_OPEN_CMD  (2U)
#define FILE_UPLOAD_DATA_CMD  (3U)
#define FILE_UPLOAD_CLOSE_CMD (4U)
#define FILE_COPY_TO_CFG_CMD  (5U)

/* Command argument offset */
#define FILE_CRC_OFFSET        (1U)
#define FILE_FRAM_OFFSET       (2U)
#define FILE_FNAME_OFFSET      (3U)
#define FILE_RM_FNAME_OFFSET   (1U)
#define FILE_COPY_FNAME_OFFSET (1U)
#define FILE_SRC_OFT_OFFSET    FILE_COPY_FNAME_OFFSET + CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN
#define FILE_COPY_SIZE_OFFSET  FILE_SRC_OFT_OFFSET + (4U)
#define FILE_COPY_BANK_OFFSET  FILE_COPY_SIZE_OFFSET + (4U)
#define FILE_DST_PRT_OFFSET    FILE_COPY_BANK_OFFSET + (1U)
#define FILE_DST_OFT_OFFSET    FILE_DST_PRT_OFFSET + (1U)

static struct file_work *csp_get_file_work(void)
{
	struct file_work *file_work;

	for (int i = 0; i < CONFIG_SC_LIB_CSP_MAX_FILE_WORK; i++) {
		file_work = &file_works[i];
		if (!k_work_is_pending(&file_work->work)) {
			return file_work;
		}
	}

	return NULL;
}

static int csp_get_file_info(const char *fname, struct fs_dirent *entry)
{
	int ret;

	ret = fs_stat(fname, entry);
	if (ret < 0) {
		LOG_ERR("Faild to get the file info %s (%d)", fname, ret);
		goto end;
	}

	LOG_INF("File Info: (entry %d) (size: %u) (name: %s)", entry->type, entry->size,
		entry->name);

end:
	return ret;
}

static int csp_calc_crc32(const char *fname, size_t size, uint32_t *crc32)
{
	int ret;
	struct fs_file_t file;
	uint8_t chunk[CONFIG_SC_LIB_CSP_CRC_CHUNK_SIZE];
	ssize_t remainig_size = size;
	ssize_t read_size;

	fs_file_t_init(&file);

	ret = fs_open(&file, fname, FS_O_READ);
	if (ret < 0) {
		LOG_ERR("Faild to open the file %s (%d)", fname, ret);
		goto end;
	}

	*crc32 = 0;

	while (remainig_size) {
		read_size = fs_read(&file, chunk, sizeof(chunk));
		if (read_size < 0) {
			LOG_ERR("Failed to read file %s", fname);
			break;
		}

		*crc32 = crc32_ieee_update(*crc32, chunk, read_size);
		remainig_size -= read_size;

		k_sleep(K_MSEC(CONFIG_SC_LIB_CSP_CRC_CALC_SLEEP_MSEC));
	}

	fs_close(&file);

	LOG_INF("CRC32: 0x%08x", *crc32);

end:
	return ret;
}

static int copy_file_to_cfg(const char *src_file, off_t src_offset, size_t size, uint8_t bank,
			    uint8_t partition_id, off_t dst_offset)
{
	int ret;
	const struct flash_area *flash = NULL;
	struct fs_file_t file;
	struct fs_dirent entry;
	uint8_t buffer[CONFIG_SC_LIB_CSP_COPY_CHUNK_SIZE];
	size_t remainig_size;
	size_t read_size;
	off_t offset = dst_offset;

	ret = fs_stat(src_file, &entry);
	if (ret < 0) {
		LOG_ERR("Faild to get the file info %s (%d)", src_file, ret);
		goto end;
	}

	fs_file_t_init(&file);

	if (entry.size < size) {
		LOG_ERR("Invalide copy size: %d", size);
		ret = -EINVAL;
		goto close;
	}

	ret = fs_open(&file, src_file, FS_O_READ);
	if (ret < 0) {
		LOG_ERR("Faild to open the src file %s (%d)", src_file, ret);
		goto end;
	}

	ret = fs_seek(&file, src_offset, FS_SEEK_SET);
	if (ret < 0) {
		LOG_ERR("Faild to seek the upload file %s (%ld) (%d)", src_file, src_offset, ret);
		goto close;
	}

	ret = sc_select_cfgmem(bank);
	if (ret < 0) {
		LOG_ERR("Faild to select the config mem (%d) (%d)", bank, ret);
		goto close;
	}

	ret = flash_area_open(partition_id, &flash);
	if (ret < 0) {
		LOG_ERR("Failed to open flash area (%d)", partition_id);
		goto close;
	}

	if (size > 0) {
		remainig_size = size;
	} else {
		remainig_size = entry.size;
	}

	while (remainig_size) {

		if (remainig_size < sizeof(buffer)) {
			read_size = remainig_size;
		} else {
			read_size = sizeof(buffer);
		}

		read_size = fs_read(&file, buffer, read_size);
		if (read_size < 0) {
			LOG_ERR("Failed to read from src file %s (%d)", src_file, read_size);
			break;
		}

		ret = flash_area_write(flash, offset, buffer, sizeof(buffer));
		if (ret < 0) {
			LOG_ERR("Failed to write to NOR flash (%ld) (%d)", offset, ret);
			break;
		}

		offset += read_size;
		remainig_size -= read_size;

		k_sleep(K_MSEC(CONFIG_SC_LIB_CSP_COPY_SLEEP_MSEC));
	}

	flash_area_close(flash);

	LOG_INF("Finish to copy from %s to MEM%d:%d", src_file, bank, partition_id);

close:
	fs_close(&file);

end:
	return ret;
}

static void csp_send_file_info_reply(struct fs_dirent *entry, uint32_t crc32, csp_packet_t *packet,
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
	tlm.crc32 = sys_cpu_to_le32(crc32);

	memcpy(packet->data, &tlm, sizeof(tlm));
	packet->length = sizeof(tlm);

	csp_sendto_reply(packet, packet, CSP_O_SAME);
}

static int csp_file_info_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret = 0;
	uint8_t crc_opt;
	uint8_t fram_opt;
	char fname[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN];
	struct fs_dirent entry;
	uint32_t crc32 = 0;

	if (packet->length != FILE_INFO_CMD_SIZE + CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		goto end;
	}

	crc_opt = packet->data[FILE_CRC_OFFSET];
	fram_opt = packet->data[FILE_FRAM_OFFSET];
	strncpy(fname, &packet->data[FILE_FNAME_OFFSET], CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN);
	fname[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN - 1] = '\0';

	LOG_INF("File info command (fname: %s)", fname);

	ret = csp_get_file_info(fname, &entry);
	if (ret == 0 && crc_opt) {
		(void)csp_calc_crc32(fname, entry.size, &crc32);
		if (fram_opt) {
			(void)sc_fram_update_crc_for_file(fname, crc32);
		}
	}

end:
	csp_send_file_info_reply(&entry, crc32, packet, command_id, ret);
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

	strncpy(fname, &packet->data[FILE_RM_FNAME_OFFSET], CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN);
	fname[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN - 1] = '\0';

	LOG_INF("File remove command (fname: %s)", fname);

	ret = fs_unlink(fname);

end:
	csp_send_std_reply(packet, command_id, ret);
	return ret;
}
static int csp_file_copy_to_cfg_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret = 0;
	char src_file[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN];
	off_t src_offset;
	size_t size;
	uint8_t bank;
	uint8_t partition_id;
	off_t dst_offset;

	if (packet->length != FILE_COPY_TO_CFG_CMD_SIZE + CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		goto end;
	}

	strncpy(src_file, &packet->data[FILE_COPY_FNAME_OFFSET],
		CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN);
	src_file[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN - 1] = '\0';
	src_offset = sys_le32_to_cpu(*(uint32_t *)&packet->data[FILE_SRC_OFT_OFFSET]);
	size = sys_le32_to_cpu(*(uint32_t *)&packet->data[FILE_COPY_SIZE_OFFSET]);
	bank = packet->data[FILE_COPY_BANK_OFFSET];
	partition_id = packet->data[FILE_DST_PRT_OFFSET];
	dst_offset = sys_le32_to_cpu(*(uint32_t *)&packet->data[FILE_DST_OFT_OFFSET]);

	LOG_INF("File copy to config mem command (src: %s) (src offset: %ld) (size: %d) (dst bank: "
		"%d) (dst partition: %d) (dst offste: %ld)",
		src_file, src_offset, size, bank, partition_id, dst_offset);

	ret = copy_file_to_cfg(src_file, src_offset, size, bank, partition_id, dst_offset);

end:
	csp_send_std_reply(packet, command_id, ret);
	return ret;
}

static void csp_file_work(struct k_work *work)
{
	int ret = 0;
	uint8_t command_id;
	struct file_work *file_work;
	csp_packet_t *packet;

	file_work = CONTAINER_OF(work, struct file_work, work);
	packet = file_work->packet;

	if (packet == NULL) {
		ret = -EINVAL;
		command_id = CSP_UNKNOWN_CMD_CODE;
		goto end;
	}

	if (packet->length < FILE_CMD_MIN_SIZE) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		command_id = CSP_UNKNOWN_CMD_CODE;
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
	case FILE_UPLOAD_OPEN_CMD:
		csp_file_upload_open_cmd(command_id, packet);
		break;
	case FILE_UPLOAD_DATA_CMD:
		csp_file_upload_data_cmd(command_id, packet);
		break;
	case FILE_UPLOAD_CLOSE_CMD:
		csp_file_upload_close_cmd(command_id, packet);
		break;
	case FILE_COPY_TO_CFG_CMD:
		csp_file_copy_to_cfg_cmd(command_id, packet);
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
	}

end:
	return;
}

int csp_file_handler(csp_packet_t *packet)
{
	int ret = 0;
	struct file_work *file_work;

	file_work = csp_get_file_work();
	if (file_work == NULL) {
		LOG_ERR("File operation work queue is busy");
		ret = -EBUSY;
		csp_buffer_free(packet);
		goto end;
	}

	file_work->packet = packet;
	k_work_submit_to_queue(&file_workq, &file_work->work);

end:
	return ret;
}

void csp_file_handler_init(void)
{
	k_work_queue_start(&file_workq, file_workq_stack, K_THREAD_STACK_SIZEOF(file_workq_stack),
			   CONFIG_SC_LIB_CSP_FILE_THREAD_PRIORITY, NULL);
	k_thread_name_set(&file_workq.thread, "file_workq");

	for (int i = 0; i < CONFIG_SC_LIB_CSP_MAX_FILE_WORK; i++) {
		k_work_init(&file_works[i].work, csp_file_work);
	}
}

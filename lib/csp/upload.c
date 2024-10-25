/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/fs/fs.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/slist.h>
#include <csp/csp.h>
#include "sc_csp.h"
#include "reply.h"
#include "upload.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(upload, CONFIG_SC_LIB_CSP_LOG_LEVEL);

/* Command size */
#define UPLOAD_OPEN_CMD_SIZE (3U)  /* without file name length */
#define UPLOAD_DATA_CMD_SIZE (11U) /* without data length */

/* Command argument offset */
#define UPLOAD_SID_OFFSET   (1U)
#define UPLOAD_FNAME_OFFSET (3U)
#define UPLOAD_OFST_OFFSET  (3U)
#define UPLOAD_SIZE_OFFSET  (7U)
#define UPLOAD_DATA_OFFSET  (11U)

BUILD_ASSERT(CONFIG_SC_LIB_CSP_UPLOAD_MAX_SESSION <= CONFIG_ZVFS_OPEN_MAX,
	     "CONFIG_SC_LIB_CSP_UPLOAD_MAX_SESSION >= CONFIG_ZVFS_OPEN_MAX");

struct session_entry {
	sys_snode_t node;
	uint16_t id;
	struct fs_file_t file;
	char fname[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN];
	struct upload_data_reply_telemetry data_reply;
	uint8_t reply_count;
};

static struct session_entry sessions[CONFIG_SC_LIB_CSP_UPLOAD_MAX_SESSION];
static sys_slist_t session_used = SYS_SLIST_STATIC_INIT(&session_used);
static sys_slist_t session_unused = SYS_SLIST_STATIC_INIT(&session_unused);

struct session_entry *search_used_session(uint16_t session_id)
{
	struct session_entry *session;

	SYS_SLIST_FOR_EACH_CONTAINER(&session_used, session, node) {
		if (session->id == session_id) {
			return session;
		}
	}

	return NULL;
}

struct session_entry *get_unused_session(void)
{
	sys_snode_t *node;

	node = sys_slist_peek_head(&session_unused);
	if (node != NULL) {
		sys_slist_remove(&session_unused, NULL, node);
		sys_slist_prepend(&session_used, node);
		return CONTAINER_OF(node, struct session_entry, node);
	}

	return NULL;
}

static void release_session(struct session_entry *session)
{
	sys_slist_find_and_remove(&session_used, &session->node);
	sys_slist_prepend(&session_unused, &session->node);
}

static void csp_send_upload_open_reply(csp_packet_t *packet, uint8_t command_id, int err_code,
				       uint16_t session_id, const char *fname)
{
	struct upload_open_reply_telemetry tlm;

	tlm.telemetry_id = command_id;
	tlm.error_code = sys_cpu_to_le32(err_code);
	tlm.session_id = sys_cpu_to_le16(session_id);
	strncpy(tlm.file_name, fname, CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN);
	tlm.file_name[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN - 1] = '\0';

	memcpy(packet->data, &tlm, sizeof(tlm));
	packet->length = sizeof(tlm);

	csp_sendto_reply(packet, packet, CSP_O_SAME);
}

static void csp_send_upload_data_err_reply(csp_packet_t *packet, uint8_t command_id, int err_code,
					   uint16_t session_id, uint32_t offset, uint32_t size)
{
	struct upload_data_reply_telemetry tlm;

	tlm.telemetry_id = command_id;
	tlm.session_id = sys_cpu_to_le16(session_id);
	tlm.entry[0].error_code = sys_cpu_to_le32(err_code);
	tlm.entry[0].offset = sys_cpu_to_le32(offset);
	tlm.entry[0].size = sys_cpu_to_le32(size);

	memcpy(packet->data, &tlm, sizeof(tlm));
	packet->length = sizeof(tlm);

	csp_sendto_reply(packet, packet, CSP_O_SAME);
}

static void csp_send_upload_data_reply(csp_packet_t *packet, struct session_entry *session)
{
	memcpy(packet->data, &session->data_reply, sizeof(session->data_reply));
	packet->length = sizeof(session->data_reply);

	csp_sendto_reply(packet, packet, CSP_O_SAME);

	session->reply_count = 0;
	memset(&session->data_reply.entry, 0, sizeof(session->data_reply.entry));

	return;
}

static void csp_stack_upload_data_reply(struct session_entry *session, uint8_t command_id,
					int err_code, uint32_t offset, uint32_t size)
{
	struct upload_data_reply_telemetry *tlm;
	struct upload_data_reply_entry *entry;

	tlm = &session->data_reply;
	entry = &tlm->entry[session->reply_count];

	tlm->telemetry_id = command_id;
	tlm->session_id = session->id;
	entry->error_code = sys_cpu_to_le32(err_code);
	entry->offset = sys_cpu_to_le32(offset);
	entry->size = sys_cpu_to_le32(size);

	session->reply_count++;
}

static int csp_open_upload_file(uint16_t session_id, const char *fname)
{
	struct session_entry *session;
	int ret;

	session = search_used_session(session_id);
	if (session != NULL) {
		LOG_ERR("This session ID is already used in (%s)", session->fname);
		ret = -EEXIST;
		goto end;
	}

	session = get_unused_session();
	if (session == NULL) {
		LOG_ERR("No more sessions are available.");
		ret = -EMFILE;
		goto end;
	}

	fs_file_t_init(&session->file);

	ret = fs_open(&session->file, fname, FS_O_CREATE | FS_O_RDWR);
	if (ret < 0) {
		LOG_ERR("Faild to open the upload file %s (%d)", fname, ret);
		release_session(session);
	}

	session->id = session_id;
	strncpy(session->fname, fname, strlen(fname));

end:
	return ret;
}

static int csp_write_upload_file(struct session_entry *session, uint32_t offset, uint8_t *data,
				 uint32_t size)
{
	int ret;

	ret = fs_seek(&session->file, offset, FS_SEEK_SET);
	if (ret < 0) {
		LOG_ERR("Faild to seek the upload file %s (%d)", session->fname, ret);
		goto end;
	}

	ret = fs_write(&session->file, data, size);
	if (ret < 0) {
		LOG_ERR("Faild to write the upload file %s (%d)", session->fname, ret);
	}

end:
	return ret;
}

void csp_upload_handler_init(void)
{
	for (int i = 0; i < CONFIG_SC_LIB_CSP_UPLOAD_MAX_SESSION; i++) {
		sessions[i].reply_count = 0;
		sys_slist_prepend(&session_unused, &sessions[i].node);
	}
}

int csp_file_upload_open_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret = 0;
	uint16_t session_id = 0;
	char fname[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN] = {0};

	if (packet->length != UPLOAD_OPEN_CMD_SIZE + CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		goto end;
	}

	session_id = sys_le16_to_cpu(*(uint16_t *)&packet->data[UPLOAD_SID_OFFSET]);
	strncpy(fname, &packet->data[UPLOAD_FNAME_OFFSET], CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN);
	fname[CONFIG_SC_LIB_CSP_FILE_NAME_MAX_LEN - 1] = '\0';

	LOG_INF("Upload (OPEN) command (session_id: %d) (fname: %s)", session_id, fname);

	ret = csp_open_upload_file(session_id, fname);

end:
	csp_send_upload_open_reply(packet, command_id, ret, session_id, fname);
	return ret;
}

int csp_file_upload_data_cmd(uint8_t command_id, csp_packet_t *packet)
{
	int ret = 0;
	uint16_t session_id = 0;
	uint32_t offset = 0;
	uint32_t size = 0;
	struct session_entry *session;
	uint8_t data[CONFIG_SC_LIB_CSP_UPLOAD_DATA_LEN] = {0};

	if (packet->length != UPLOAD_DATA_CMD_SIZE + CONFIG_SC_LIB_CSP_UPLOAD_DATA_LEN) {
		LOG_ERR("Invalide command size: %d", packet->length);
		ret = -EINVAL;
		goto end;
	}

	session_id = sys_le16_to_cpu(*(uint16_t *)&packet->data[UPLOAD_SID_OFFSET]);
	offset = sys_le32_to_cpu(*(uint32_t *)&packet->data[UPLOAD_OFST_OFFSET]);
	size = sys_le32_to_cpu(*(uint32_t *)&packet->data[UPLOAD_SIZE_OFFSET]);
	memcpy(data, &packet->data[UPLOAD_DATA_OFFSET], size);

	LOG_DBG("Upload (DATA) command (session_id: %d) (offset: %d) (size: %d)", session_id,
		offset, size);

	session = search_used_session(session_id);
	if (session == NULL) {
		LOG_ERR("This session ID is not used (%d)", session_id);
		ret = -ENOENT;
		goto end;
	}

	ret = csp_write_upload_file(session, offset, data, size);

	/*
	 * The reply telemetry for the DATA command will increase in volume, so to use
	 * the downlink bandwidth efficiently, it will stacked a certain number of them
	 * before sending.
	 */
	csp_stack_upload_data_reply(session, command_id, ret, offset, size);
	if (session->reply_count >= CONFIG_CS_LIB_CSP_UPLOAD_DATA_REPLY_ENTRY ||
	    size != CONFIG_SC_LIB_CSP_UPLOAD_DATA_LEN) {
		csp_send_upload_data_reply(packet, session);
	} else {
		csp_buffer_free(packet);
	}

end:
	if (ret < 0) {
		csp_send_upload_data_err_reply(packet, command_id, ret, session_id, offset, size);
	}

	return ret;
}

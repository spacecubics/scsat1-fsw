/*
 * Copyright (c) 2024 Space Cubics, LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(data_nor, CONFIG_SC_LIB_FLASH_LOG_LEVEL);

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
static struct fs_mount_t lfs_storage_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &storage,
	.storage_dev = (void *)FIXED_PARTITION_ID(storage_partition),
	.mnt_point = CONFIG_SC_LIB_FLASH_DATA_STORE_MNT_POINT,
};

struct fs_mount_t *mountpoint = &lfs_storage_mnt;

int datafs_init(void)
{
	struct fs_statvfs sbuf;
	int ret;

	ret = fs_mount(mountpoint);
	if (ret < 0) {
		LOG_ERR("Faild to mount %s (%d)", mountpoint->mnt_point, ret);
		goto end;
	}

	ret = fs_statvfs(mountpoint->mnt_point, &sbuf);
	if (ret < 0) {
		LOG_ERR("Faild to get the fs status %s (%d)", mountpoint->mnt_point, ret);
		goto end;
	}

	LOG_INF("%s: bsize = %lu ; frsize = %lu ;"
		" blocks = %lu ; bfree = %lu",
		mountpoint->mnt_point, sbuf.f_bsize, sbuf.f_frsize, sbuf.f_blocks, sbuf.f_bfree);

end:
	return ret;
}

int update_boot_count(const char *fname)
{
	uint32_t boot_count = 0;
	struct fs_file_t file;
	int ret;

	fs_file_t_init(&file);

	ret = fs_open(&file, fname, FS_O_CREATE | FS_O_RDWR);
	if (ret < 0) {
		LOG_ERR("Faild to open the boot count file %s (%d)", fname, ret);
		goto end;
	}

	ret = fs_read(&file, &boot_count, sizeof(boot_count));
	if (ret < 0) {
		LOG_ERR("Faild to read the boot count file %s (%d)", fname, ret);
		goto close;
	}
	LOG_INF("%s read count:%u (bytes: %d)", fname, boot_count, ret);

	ret = fs_seek(&file, 0, FS_SEEK_SET);
	if (ret < 0) {
		LOG_ERR("Faild to seek the boot count file %s (%d)", fname, ret);
		goto close;
	}

	boot_count++;
	ret = fs_write(&file, &boot_count, sizeof(boot_count));
	if (ret < 0) {
		LOG_ERR("Faild to write the boot count file %s (%d)", fname, ret);
		goto close;
	}

	LOG_INF("%s write new boot count %u: (bytes: %d]", fname, boot_count, ret);

close:
	ret = fs_close(&file);
	if (ret < 0) {
		LOG_ERR("Faild to close the boot count file %s (%d)", fname, ret);
		goto close;
	}

end:
	return ret;
}

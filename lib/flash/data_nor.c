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
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

LOG_MODULE_REGISTER(lfs);

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

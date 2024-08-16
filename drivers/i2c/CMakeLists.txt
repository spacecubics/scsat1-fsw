# Copyright (c) 2023 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_I2C_SC i2c_sc.c)

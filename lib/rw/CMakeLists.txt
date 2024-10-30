# Copyright (c) 2024 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SC_LIB_RW rw.c)
zephyr_include_directories(.)

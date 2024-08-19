# Copyright (c) 2024 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SC_DSTRX3 sc_dstrx3.c)

# Export sc_dstrx3.h temporarily
zephyr_include_directories(.)

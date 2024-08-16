# Copyright (c) 2023 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_CAN_SCCAN can_sccan.c)

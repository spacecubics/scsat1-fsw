# Copyright (c) 2024 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SC_FPGASYS sc_fpgasys.c)
zephyr_library_sources_ifdef(CONFIG_SC_FPGAMON sc_fpgamon.c)
zephyr_library_sources_ifdef(CONFIG_SC_FPGAHRMEM sc_fpgahrmem.c)

zephyr_include_directories(.)

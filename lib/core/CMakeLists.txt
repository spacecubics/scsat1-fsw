# Copyright (c) 2024 Space Cubics, LLC.
# SPDX-License-Identifier: Apache-2.0

if (CONFIG_BOARD_SCSAT1_MAIN OR CONFIG_BOARD_SCSAT1_ADCS)
	zephyr_library()
	zephyr_library_sources_ifdef(CONFIG_SC_LIB_CORE_MAIN pwrctrl_main.c)
	zephyr_library_sources_ifdef(CONFIG_SC_LIB_CORE_ADCS pwrctrl_adcs.c)
	zephyr_include_directories(.)
endif()
